// ============================================================
// Distributed Luminaire Controller — Phase 2
// RPI Pico W (Arduino framework, dual-core)
// ============================================================
//
// All nodes run identical code. Hub function activates when
// USB serial is connected. Algorithms switchable at runtime.
//
// Core 0: Serial handling, CAN messages, hub, distributed alg
// Core 1: 100 Hz local PI control loop
// ============================================================

#include <cstdio>
#include <cmath>
#include <cstring>
#include <SPI.h>
#include <mcp2515.h>
#include <pico/unique_id.h>
#include <pico/mutex.h>

#include "pi_controller.h"
#include "can_comms.h"
#include "network_manager.h"
#include "calibration.h"
#include "distributed_controller.h"
#include "hub.h"
#include "metrics.h"
#include "ring_buffer.h"

// ======================== Configuration =======================

// --- Pin definitions ---
static const int LDR_PIN = A0;
static const int LED_PIN = 15;

// --- Circuit constants ---
static const float VCC     = 3.3f;
static const float R_FIXED = 10000.0f;
static const int   ADC_MAX = 4095;
static const int   PWM_MAX = 4095;
static const float P_MAX   = 1.0f;

// --- Sampling ---
static const float T_SAMPLE = 0.01f;  // 100 Hz

// --- EMA filter ---
static const float EMA_ALPHA = 0.9f;

// --- LDR calibration ---
static float m_ldr = -0.7f;
static float b_ldr = 5.847f;

// ======================== Global Objects =======================

PIController pid;
CANComms can;
NetworkManager network;
Calibration calib;
DistributedController dist_ctrl;
Hub hub;
PerformanceMetrics metrics;
RingBuffer ring_buf;

// Mutex for shared state between cores
auto_init_mutex(state_mutex);

// ======================== Shared State =========================

volatile float current_lux  = 0.0f;
volatile float current_duty = 0.0f;
volatile float current_ref  = 0.0f;
volatile float serial_duty  = 0.0f;
volatile float ema_lux      = 0.0f;

// Occupancy and references
volatile int  occupancy_state = 1;  // 1=unoccupied(low)
volatile float ref_low        = 10.0f;
volatile float ref_high       = 40.0f;

// Timing
volatile unsigned long last_ctrl_us = 0;
volatile unsigned long start_time_us = 0;

// Streaming flags
volatile bool stream_lux  = false;
volatile bool stream_duty = false;

// Distributed algorithm timing
static const unsigned long DIST_PERIOD_MS = 100;  // 10 Hz
unsigned long last_dist_ms = 0;
int dist_iter_count = 0;
static const int DIST_MAX_ITER_PER_ROUND = 50;

// Boot state
bool system_ready = false;

// ======================== LUX Sensing =========================

float adcToLux(float adc_val) {
  float v_out = adc_val * (VCC / ADC_MAX);
  if (v_out <= 0.001f || v_out >= VCC - 0.001f) return 0.0f;
  float r_ldr   = R_FIXED * (VCC / v_out - 1.0f);
  float log_lux = (log10(r_ldr) - b_ldr) / m_ldr;
  return pow(10.0f, log_lux);
}

float measureLux() {
  long sum = 0;
  for (int i = 0; i < 100; i++)
    sum += analogRead(LDR_PIN);
  float adc_avg = (float)sum / 100.0f;
  float raw = adcToLux(adc_avg);
  ema_lux = EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * ema_lux;
  return ema_lux;
}

float measureLuxDirect() {
  long sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += analogRead(LDR_PIN);
    delay(10);
  }
  return adcToLux((float)sum / 100.0f);
}

// ======================== LED Driver ==========================

void setLEDRaw(float duty) {
  duty = constrain(duty, 0.0f, 1.0f);
  current_duty = duty;
  analogWrite(LED_PIN, (int)(duty * PWM_MAX));
}

// Wrapper for calibration module
void setLEDCallback(float duty) { setLEDRaw(duty); }
float measureLuxCallback() { return measureLuxDirect(); }

// ======================== Serial Command Parser ===============

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char buf[128];
  line.toCharArray(buf, sizeof(buf));

  char cmd = buf[0];
  int id;
  double val;
  char subcmd, x;

  switch (cmd) {

    // ===== Phase 1 commands (now routed via hub) =====

    case 'd': {  // set duty directly
      std::sscanf(buf, "%c %d %lf", &cmd, &id, &val);
      id--;  // convert to 0-based
      if (id == network.my_logical_id - 1) {
        serial_duty = (float)val;
        pid.feedback_on = false;
        setLEDRaw(serial_duty);
        Serial.println("ack");
      } else if (hub.active && id >= 0 && id < network.num_nodes) {
        hub.forwardCommand(id + 1, buf, strlen(buf));
      } else {
        Serial.println("err");
      }
      break;
    }

    case 'r': {  // set illuminance reference
      std::sscanf(buf, "%c %d %lf", &cmd, &id, &val);
      id--;
      if (id == network.my_logical_id - 1) {
        mutex_enter_blocking(&state_mutex);
        current_ref = (float)val;
        mutex_exit(&state_mutex);
        Serial.println("ack");
      } else if (hub.active) {
        hub.forwardCommand(id + 1, buf, strlen(buf));
      } else {
        Serial.println("err");
      }
      break;
    }

    case 'o': {  // set occupancy
      std::sscanf(buf, "%c %d %lf", &cmd, &id, &val);
      id--;
      if (id == network.my_logical_id - 1) {
        mutex_enter_blocking(&state_mutex);
        occupancy_state = (int)val;
        if (occupancy_state == 0)
          current_ref = 0.0f;
        else if (occupancy_state == 1)
          current_ref = ref_low;
        else
          current_ref = ref_high;
        if (!pid.feedback_on) {
          pid.feedback_on = true;
          pid.reset();
        }
        mutex_exit(&state_mutex);

        // Notify distributed controller
        dist_ctrl.setOccupancy(id, occupancy_state);

        // Broadcast occupancy change
        can.broadcastFloat(MSG_OCCUPY_UPDATE, (float)((id << 8) | occupancy_state));

        Serial.println("ack");
      } else if (hub.active) {
        hub.forwardCommand(id + 1, buf, strlen(buf));
      } else {
        Serial.println("err");
      }
      break;
    }

    case 'a': {  // toggle anti-windup
      std::sscanf(buf, "%c %d %lf", &cmd, &id, &val);
      id--;
      if (id == network.my_logical_id - 1) {
        pid.antiwindup_on = ((int)val == 1);
        Serial.println("ack");
      } else if (hub.active) {
        hub.forwardCommand(id + 1, buf, strlen(buf));
      } else {
        Serial.println("err");
      }
      break;
    }

    case 'k': {  // toggle feedback
      std::sscanf(buf, "%c %d %lf", &cmd, &id, &val);
      id--;
      if (id == network.my_logical_id - 1) {
        serial_duty = current_duty;
        pid.feedback_on = ((int)val == 1);
        if (pid.feedback_on) pid.reset();
        Serial.println("ack");
      } else if (hub.active) {
        hub.forwardCommand(id + 1, buf, strlen(buf));
      } else {
        Serial.println("err");
      }
      break;
    }

    // ===== Getter commands =====
    case 'g': {
      char peek;
      std::sscanf(buf, "%c %c", &cmd, &peek);

      if (peek == 'b') {  // buffer dump
        std::sscanf(buf, "%c %c %c %d", &cmd, &subcmd, &x, &id);
        id--;
        if (id == network.my_logical_id - 1) {
          ring_buf.startDump(x);
        }
        break;
      }

      std::sscanf(buf, "%c %c %d", &cmd, &subcmd, &id);
      id--;
      if (id == network.my_logical_id - 1) {
        int lid = network.my_logical_id;
        switch (subcmd) {
          case 'd': Serial.printf("d %d %.4f\n", lid, (float)current_duty); break;
          case 'r': Serial.printf("r %d %.2f\n", lid, (float)current_ref); break;
          case 'l': Serial.printf("l %d %.2f\n", lid, (float)current_lux); break;
          case 'o': Serial.printf("o %d %d\n", lid, (int)occupancy_state); break;
          case 'a': Serial.printf("a %d %d\n", lid, pid.antiwindup_on ? 1 : 0); break;
          case 'k': Serial.printf("k %d %d\n", lid, pid.feedback_on ? 1 : 0); break;
          case 'x': Serial.printf("x %d %.2f\n", lid,
                      fmaxf(0.0f, current_lux - calib.K[id][id] * current_duty)); break;
          case 'p': Serial.printf("p %d %.4f\n", lid, P_MAX * (float)current_duty); break;
          case 't': Serial.printf("t %d %.2f\n", lid, micros() / 1e6f); break;
          case 'e': Serial.printf("e %d %.4f\n", lid, metrics.energy_acc); break;
          case 'v': Serial.printf("v %d %.4f\n", lid, metrics.vis_error_acc); break;
          case 'f': Serial.printf("f %d %.6f\n", lid, metrics.flicker_acc); break;
          default:  Serial.println("err"); break;
        }
      } else if (hub.active) {
        hub.forwardCommand(id + 1, buf, strlen(buf));
      } else {
        Serial.println("err");
      }
      break;
    }

    // ===== Streaming =====
    case 's': {
      std::sscanf(buf, "%c %c %d", &cmd, &x, &id);
      id--;
      if (id == network.my_logical_id - 1) {
        if (x == 'l') stream_lux  = true;
        if (x == 'd') stream_duty = true;
        Serial.println("ack");
      }
      break;
    }
    case 'S': {
      std::sscanf(buf, "%c %c %d", &cmd, &x, &id);
      id--;
      if (id == network.my_logical_id - 1) {
        if (x == 'l') stream_lux  = false;
        if (x == 'd') stream_duty = false;
        Serial.println("ack");
      }
      break;
    }

    // ===== Phase 2 commands =====

    case 'A': {  // Set algorithm: A <alg_id>
      int alg;
      std::sscanf(buf, "%c %d", &cmd, &alg);
      dist_ctrl.setAlgorithm((Algorithm)alg);
      dist_ctrl.initAlgorithm();
      dist_iter_count = 0;
      // Broadcast algorithm change to all nodes
      can.broadcastFloat(MSG_SYNC, (float)alg);
      Serial.printf("ack alg=%d\n", alg);
      break;
    }

    case 'C': {  // Set cost coefficient: C <node_id> <val>
      std::sscanf(buf, "%c %d %lf", &cmd, &id, &val);
      id--;
      dist_ctrl.setCost(id, (float)val);
      // Broadcast cost update
      can.broadcastFloat2(MSG_COST_UPDATE, (float)id, (float)val);
      Serial.println("ack");
      break;
    }

    case 'R': {  // Trigger recalibration
      Serial.println("Recalibrating...");
      if (network.my_logical_id == 1) {
        calib.runCoordinator(can);
      } else {
        calib.runFollower(can);
      }
      dist_ctrl.init(network.num_nodes, network.my_logical_id - 1,
                     calib.K, calib.o);
      Serial.println("ack");
      break;
    }

    case 'W': {  // Network wake-up / reboot
      Serial.println("Rebooting network...");
      network.boot();
      Serial.printf("Nodes: %d, My ID: %d\n",
                    network.num_nodes, network.my_logical_id);
      Serial.println("ack");
      break;
    }

    case 'K': {  // Get/print calibration matrix
      Serial.println("Gain matrix K:");
      for (int i = 0; i < network.num_nodes; i++) {
        for (int j = 0; j < network.num_nodes; j++) {
          Serial.printf("  K[%d][%d]=%.2f", i + 1, j + 1, calib.K[i][j]);
        }
        Serial.println();
      }
      Serial.printf("Background: ");
      for (int i = 0; i < network.num_nodes; i++)
        Serial.printf("o[%d]=%.2f ", i + 1, calib.o[i]);
      Serial.println();
      break;
    }

    case 'D': {  // Get distributed controller state
      Serial.printf("Alg: %d, Iter: %d, Conv: %d\n",
                    dist_ctrl.current_alg,
                    dist_ctrl.getIteration(),
                    dist_ctrl.isConverged() ? 1 : 0);
      for (int i = 0; i < network.num_nodes; i++)
        Serial.printf("  d[%d]=%.4f L_ref[%d]=%.2f c[%d]=%.2f\n",
                      i + 1, dist_ctrl.d[i],
                      i + 1, dist_ctrl.L_ref[i],
                      i + 1, dist_ctrl.c[i]);
      break;
    }

    case 'c': {  // PI controller parameter changes
      std::sscanf(buf, "%c %c %d %lf", &cmd, &subcmd, &id, &val);
      id--;
      if (id == network.my_logical_id - 1) {
        switch (subcmd) {
          case 'k': pid.Kp_old = pid.Kp; pid.Kp = (float)val; break;
          case 'i': pid.Ki = (float)val; break;
          case 'b': pid.b_sp_old = pid.b_sp; pid.b_sp = (float)val; break;
          case 't': if (val > 0.001) pid.Tt = (float)val; break;
          default:  Serial.println("err"); return;
        }
        Serial.println("ack");
      } else if (hub.active) {
        hub.forwardCommand(id + 1, buf, strlen(buf));
      }
      break;
    }

    default:
      Serial.println("err");
      break;
  }
}

// ======================== CAN Message Handler ==================

void handleCANMessages() {
  CANMessage msg;
  while (can.hasMessage() && can.receive(msg)) {
    // Skip our own messages
    if (msg.sender == network.my_logical_id) continue;

    switch (msg.type) {
      case MSG_DUTY_UPDATE:
      case MSG_CONSENSUS_D: {
        float val = CANComms::extractFloat(msg.data);
        float data[DC_MAX_NODES] = {0};
        data[0] = val;
        dist_ctrl.receiveUpdate(msg.sender - 1, data);
        break;
      }

      case MSG_COST_UPDATE: {
        int node = (int)CANComms::extractFloat(msg.data);
        float cost = CANComms::extractFloat(msg.data, 4);
        dist_ctrl.setCost(node, cost);
        break;
      }

      case MSG_OCCUPY_UPDATE: {
        int packed = (int)CANComms::extractFloat(msg.data);
        int node = (packed >> 8) & 0xFF;
        int state = packed & 0xFF;
        dist_ctrl.setOccupancy(node, state);
        break;
      }

      case MSG_SYNC: {
        int alg = (int)CANComms::extractFloat(msg.data);
        dist_ctrl.setAlgorithm((Algorithm)alg);
        dist_ctrl.initAlgorithm();
        dist_iter_count = 0;
        break;
      }

      case MSG_HUB_CMD: {
        // Remote command from hub
        char cmd_buf[64];
        if (hub.handleIncoming(msg, cmd_buf, sizeof(cmd_buf))) {
          // TODO: parse and execute locally, send response
        }
        break;
      }

      case MSG_CALIB_CMD:
      case MSG_CALIB_DATA:
        // Handled during calibration sequence
        break;

      default:
        break;
    }
  }
}

// ======================== Distributed Algorithm Loop ===========

void runDistributedIteration() {
  if (dist_ctrl.current_alg == ALG_NONE) return;
  if (dist_ctrl.isConverged() && !dist_ctrl.needs_recompute) return;

  if (dist_ctrl.needs_recompute)
    dist_ctrl.initAlgorithm();

  // Run one iteration
  dist_ctrl.iterate();

  // Broadcast our state
  dist_ctrl.broadcastState(can);

  // Finalize after receiving updates (best-effort: use latest)
  dist_ctrl.finalizeIteration();

  dist_iter_count++;

  // Apply distributed solution to local reference
  float optimal_duty = dist_ctrl.getMyDuty();
  float estimated_lux = calib.estimateLux(network.my_logical_id - 1,
                                          dist_ctrl.d, network.num_nodes);

  // Feed the estimated target lux to the local PI controller
  mutex_enter_blocking(&state_mutex);
  current_ref = estimated_lux;
  mutex_exit(&state_mutex);
}

// ======================== Setup (Core 0) =======================

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogWriteResolution(12);
  analogWriteFreq(30000);
  pinMode(LED_PIN, OUTPUT);
  setLEDRaw(0.0f);

  delay(1000);  // Wait for serial and other nodes

  // Initialize CAN-BUS with temporary ID 0
  can.init(0);

  // Network boot: discover nodes and assign IDs
  Serial.println("Booting network...");
  network.init(&can);
  int n = network.boot();

  // Re-init CAN with proper ID
  can.my_id = network.my_logical_id;

  Serial.printf("Network: %d nodes, I am node %d%s\n",
                n, network.my_logical_id,
                network.is_hub ? " (hub)" : "");

  // Initialize hub
  hub.init(&can, network.my_logical_id - 1, n, network.is_hub);

  // Run calibration
  Serial.println("Calibrating...");
  calib.init(n, network.my_logical_id, setLEDCallback, measureLuxCallback);
  if (network.my_logical_id == 1) {
    calib.runCoordinator(can);
  } else {
    calib.runFollower(can);
  }

  // Print calibration results
  Serial.println("Gain matrix K:");
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++)
      Serial.printf("  K[%d][%d]=%.2f", i + 1, j + 1, calib.K[i][j]);
    Serial.println();
  }

  // Initialize distributed controller
  dist_ctrl.init(n, network.my_logical_id - 1, calib.K, calib.o);
  dist_ctrl.ref_low  = ref_low;
  dist_ctrl.ref_high = ref_high;

  // Default: all desks unoccupied
  for (int i = 0; i < n; i++)
    dist_ctrl.setOccupancy(i, 1);

  current_ref = ref_low;
  start_time_us = micros();
  last_ctrl_us  = micros();

  system_ready = true;
  Serial.println("Ready.");
}

// ======================== Main Loop (Core 0) ===================

void loop() {
  // Handle serial commands
  handleSerial();

  // Handle incoming CAN messages
  handleCANMessages();

  // Check hub responses
  if (hub.waiting_response)
    hub.checkResponse();

  // Non-blocking buffer dump
  ring_buf.dumpBatch();

  // Distributed algorithm iteration (10 Hz)
  if (millis() - last_dist_ms >= DIST_PERIOD_MS) {
    last_dist_ms = millis();
    runDistributedIteration();
  }
}

// ======================== Control Loop (Core 1) ================

void setup1() {
  // Core 1 setup: wait for system ready
  while (!system_ready) delay(10);
}

void loop1() {
  // 100 Hz control cycle
  unsigned long now = micros();
  if (now - last_ctrl_us < (unsigned long)(T_SAMPLE * 1e6f)) return;
  float dt = (now - last_ctrl_us) / 1e6f;
  last_ctrl_us = now;

  // Measure illuminance
  float lux = measureLux();

  mutex_enter_blocking(&state_mutex);
  current_lux = lux;
  float ref = current_ref;
  mutex_exit(&state_mutex);

  // Run local PI controller
  if (pid.feedback_on && ref > 0.0f) {
    float u = pid.compute(ref, lux, dt);
    setLEDRaw(u);
  } else if (!pid.feedback_on) {
    setLEDRaw(serial_duty);
  }

  // Update metrics
  metrics.update(ref, lux, current_duty, dt, P_MAX);

  // Record into buffer
  ring_buf.push(lux, current_duty);

  // Streaming output
  if (stream_lux || stream_duty) {
    unsigned long t_ms = millis();
    int lid = network.my_logical_id;
    if (stream_lux)
      Serial.printf("s l %d %.2f %lu\n", lid, lux, t_ms);
    if (stream_duty)
      Serial.printf("s d %d %.4f %lu\n", lid, (float)current_duty, t_ms);
  }
}
