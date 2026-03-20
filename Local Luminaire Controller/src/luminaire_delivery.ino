// ============================================================
// Luminaire PID Controller — Delivery Version (Stage 1)
// RPI Pico W (Arduino framework)
// ============================================================
//
// PI controller with back-calculation anti-windup.
// Tuned parameters: Kp=0.01, Ki=0.11, b=1.0, Tt=0.15
// EMA sensor filter (alpha=0.9), 100 Hz control loop.
//
// Command set follows the SCDTR project specification.
// ============================================================

#include <cstdio>
#include <cmath>

// ======================== Configuration =======================

// Luminaire ID — change this per desk
const int LUMINAIRE = 1;

// --- Pin definitions ---
const int LDR_PIN = A0;
const int LED_PIN = 15;

// --- Circuit constants ---
const float VCC       = 3.3f;
const float R_FIXED   = 10000.0f;
const int   ADC_MAX   = 4095;
const int   PWM_MAX   = 4095;
const float P_MAX     = 1.0f;   // maximum LED power (W)

// --- Sampling period ---
const float T_SAMPLE  = 0.01f;  // 10 ms (100 Hz)

// --- EMA filter ---
const float EMA_ALPHA = 0.9f;

// --- LDR calibration defaults ---
float m_ldr           = -0.7f;
float b_ldr           = 5.847f;

// --- System model ---
float lux_background  = 0.13f;
float static_gain     = 77.0f;

// ======================== PI Controller =======================

class PIController {
public:
  // Controller parameters (exposed via 'c' commands)
  float Kp;       // proportional gain
  float Ki;       // integral gain
  float b_sp;     // set-point weight for proportional term
  float Tt;       // anti-windup tracking time constant (s)

  // Operating limits
  float u_min, u_max;

  // Modes
  bool feedback_on;
  bool antiwindup_on;

  // Internal state
  float integral;
  float prev_ref;
  float prev_meas;

  // Bumpless transfer bookkeeping
  float Kp_old, b_sp_old;

  PIController()
    : Kp(0.01f), Ki(0.11f), b_sp(1.0f), Tt(0.15f),
      u_min(0.0f), u_max(1.0f),
      feedback_on(true), antiwindup_on(true),
      integral(0.0f), prev_ref(0.0f), prev_meas(0.0f),
      Kp_old(0.01f), b_sp_old(1.0f)
  {}

  float compute(float ref, float meas, float dt) {
    if (!feedback_on) return 0.0f;

    // Bumpless transfer on online gain changes
    if (Kp != Kp_old || b_sp != b_sp_old) {
      integral += Kp_old * (b_sp_old * ref - meas)
                - Kp     * (b_sp     * ref - meas);
      Kp_old   = Kp;
      b_sp_old = b_sp;
    }

    float e_p = b_sp * ref - meas;   // proportional error
    float e   = ref - meas;          // integral error
    float P   = Kp * e_p;

    integral += Ki * dt * e;

    float u     = P + integral;
    float u_sat = constrain(u, u_min, u_max);

    // Anti-windup via back-calculation
    if (antiwindup_on)
      integral += (dt / Tt) * (u_sat - u);

    prev_ref  = ref;
    prev_meas = meas;
    return u_sat;
  }

  void reset() {
    integral  = 0.0f;
    prev_ref  = 0.0f;
    prev_meas = 0.0f;
  }
};

// ======================== Global State ========================

PIController pid;

// Occupancy: 0=unoccupied, 1=occupied
int occupancy_state    = 0;
float ref_unoccupied   = 10.0f;   // lower desk reference (lux)
float ref_occupied     = 40.0f;   // upper desk reference (lux)
float current_ref      = 0.0f;    // active illuminance reference
float current_duty     = 0.0f;    // current LED duty cycle [0,1]
float current_lux      = 0.0f;    // latest measured illuminance
float serial_duty      = 0.0f;    // duty set via 'd' command (open-loop)

// EMA filter state
float ema_lux          = 0.0f;

// Timing
unsigned long last_ctrl_us = 0;
unsigned long start_time_us = 0;

// ======================== Performance Metrics =================

float energy_acc       = 0.0f;    // accumulated energy (J)
float vis_error_acc    = 0.0f;    // accumulated visibility error
float flicker_acc      = 0.0f;    // accumulated flicker metric
long  n_samples        = 0;
float prev_duty_m1     = 0.0f;    // duty[k-1] for flicker
float prev_duty_m2     = 0.0f;    // duty[k-2] for flicker

// ======================== Streaming ===========================

bool stream_lux        = false;
bool stream_duty       = false;

// ======================== Last-Minute Buffer ==================
// 6000 samples = 60 s at 100 Hz (~48 KB, well within Pico RAM)

const int BUF_SIZE = 6000;
float buf_lux[BUF_SIZE];
float buf_duty[BUF_SIZE];
int   buf_idx          = 0;
bool  buf_full         = false;

// Non-blocking buffer dump state
bool  buf_dumping      = false;
char  buf_dump_var     = 'l';   // 'l' or 'd'
int   buf_dump_count   = 0;     // total samples to dump
int   buf_dump_sent    = 0;     // samples sent so far
int   buf_dump_start   = 0;     // start index in circular buffer

// ======================== LUX Sensing =========================

// Convert ADC reading to LUX using the LDR model
float adcToLux(float adc_val) {
  float v_out = adc_val * (VCC / ADC_MAX);
  if (v_out <= 0.001f || v_out >= VCC - 0.001f) return 0.0f;
  float r_ldr   = R_FIXED * (VCC / v_out - 1.0f);
  float log_lux = (log10(r_ldr) - b_ldr) / m_ldr;
  return pow(10.0f, log_lux);
}

// Measure LUX: average 100 fast ADC samples (covers ~6 PWM cycles
// at 30 kHz), then apply EMA filter for noise smoothing.
float measureLux() {
  long sum = 0;
  for (int i = 0; i < 100; i++)
    sum += analogRead(LDR_PIN);
  float adc_avg = (float)sum / 100.0f;
  float raw     = adcToLux(adc_avg);
  ema_lux       = EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * ema_lux;
  return ema_lux;
}

// ======================== LED Driver ==========================

void setLED(float duty) {
  duty = constrain(duty, 0.0f, 1.0f);
  current_duty = duty;
  analogWrite(LED_PIN, (int)(duty * PWM_MAX));
}

// ======================== Calibration =========================

// Direct LUX reading for calibration (bypasses EMA, long settle)
float measureLuxDirect() {
  long sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += analogRead(LDR_PIN);
    delay(10);
  }
  return adcToLux((float)sum / 100.0f);
}

// Calibrate LDR b-parameter with a known illuminance value
void calibrateLDR(float known_lux) {
  float adc_val = 0;
  for (int i = 0; i < 5; i++) {
    adc_val += analogRead(LDR_PIN);
    delayMicroseconds(200);
  }
  adc_val /= 5.0f;
  float v_out = adc_val * (VCC / ADC_MAX);
  float r_ldr = R_FIXED * (VCC / v_out - 1.0f);
  b_ldr = log10(r_ldr) - m_ldr * log10(known_lux);
  Serial.print("Calibrated b = ");
  Serial.println(b_ldr, 4);
}

// Calibrate static gain: LED off -> LED full, measure delta lux
void calibrateGain() {
  Serial.println("Calibrating gain...");
  setLED(0.0f);
  delay(2000);
  lux_background = measureLuxDirect();
  setLED(1.0f);
  delay(3000);
  float lux_full = measureLuxDirect();
  setLED(0.0f);
  static_gain = lux_full - lux_background;
  ema_lux = 0.0f;
  Serial.printf("Background = %.2f, Full = %.2f, Gain = %.2f\n",
                lux_background, lux_full, static_gain);
  pid.reset();
  Serial.println("ack");
}

// ======================== Performance Update ==================

void updateMetrics(float ref, float lux, float duty, float dt) {
  n_samples++;
  // Energy: integral of power over time
  energy_acc += P_MAX * duty * dt;
  // Visibility error: average of max(0, ref - lux)
  float v = max(0.0f, ref - lux);
  vis_error_acc += (v - vis_error_acc) / n_samples;
  // Flicker: detect sign changes in duty derivative
  float fk = 0.0f;
  if ((duty - prev_duty_m1) * (prev_duty_m1 - prev_duty_m2) < 0)
    fk = fabsf(duty - prev_duty_m1) + fabsf(prev_duty_m1 - prev_duty_m2);
  flicker_acc  += (fk - flicker_acc) / n_samples;
  prev_duty_m2  = prev_duty_m1;
  prev_duty_m1  = duty;
}

// ======================== Serial Command Parser ===============

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  // Parse into a C string for sscanf
  char buf[128];
  line.toCharArray(buf, sizeof(buf));

  char cmd = buf[0];
  int id;
  double val;
  char subcmd, x;

  switch (cmd) {

    // ----- d <i> <val> : set duty cycle directly -----
    case 'd': {
      std::sscanf(buf, "%c %d %lf", &cmd, &id, &val);
      if (id == LUMINAIRE) {
        serial_duty = (float)val;
        pid.feedback_on = false;
        setLED(serial_duty);
        Serial.println("ack");
      } else {
        Serial.println("err");
      }
      break;
    }

    // ----- r <i> <val> : set illuminance reference -----
    case 'r': {
      std::sscanf(buf, "%c %d %lf", &cmd, &id, &val);
      if (id == LUMINAIRE) {
        current_ref = (float)val;
        Serial.println("ack");
      } else {
        Serial.println("err");
      }
      break;
    }

    // ----- o <i> <state> : set occupancy (0 or 1) -----
    case 'o': {
      std::sscanf(buf, "%c %d %lf", &cmd, &id, &val);
      if (id == LUMINAIRE) {
        occupancy_state = (int)val;
        if (occupancy_state == 0) {
          current_ref = ref_unoccupied;
        } else {
          current_ref = ref_occupied;
        }
        if (!pid.feedback_on) {
          pid.feedback_on = true;
          pid.reset();
        }
        Serial.println("ack");
      } else {
        Serial.println("err");
      }
      break;
    }

    // ----- a <i> <0|1> : toggle anti-windup -----
    case 'a': {
      std::sscanf(buf, "%c %d %lf", &cmd, &id, &val);
      if (id == LUMINAIRE) {
        pid.antiwindup_on = ((int)val == 1);
        Serial.println("ack");
      } else {
        Serial.println("err");
      }
      break;
    }

    // ----- k <i> <0|1> : toggle feedback on/off -----
    case 'k': {
      std::sscanf(buf, "%c %d %lf", &cmd, &id, &val);
      if (id == LUMINAIRE) {
        serial_duty = current_duty;  // capture current duty for open-loop
        pid.feedback_on = ((int)val == 1);
        if (pid.feedback_on) pid.reset();
        Serial.println("ack");
      } else {
        Serial.println("err");
      }
      break;
    }

    // ----- g <subcmd> ... : get commands -----
    case 'g': {
      // Check for 'g b <x> <i>' (buffer) first — needs 4-char parse
      char peek;
      std::sscanf(buf, "%c %c", &cmd, &peek);
      if (peek == 'b') {
        std::sscanf(buf, "%c %c %c %d", &cmd, &subcmd, &x, &id);
        if (id == LUMINAIRE) {
          // Start non-blocking buffer dump
          buf_dump_var   = x;
          buf_dump_count = buf_full ? BUF_SIZE : buf_idx;
          buf_dump_start = buf_full ? buf_idx : 0;
          buf_dump_sent  = 0;
          buf_dumping    = true;
        }
        break;
      }

      // Standard 'g <subcmd> <i>'
      std::sscanf(buf, "%c %c %d", &cmd, &subcmd, &id);
      if (id != LUMINAIRE) { Serial.println("err"); break; }

      switch (subcmd) {
        case 'd':  // duty cycle
          Serial.printf("d %d %.4f\n", LUMINAIRE,
                        pid.feedback_on ? current_duty : serial_duty);
          break;
        case 'r':  // reference
          Serial.printf("r %d %.2f\n", LUMINAIRE, current_ref);
          break;
        case 'l':  // measured illuminance
          Serial.printf("l %d %.2f\n", LUMINAIRE, current_lux);
          break;
        case 'o':  // occupancy
          Serial.printf("o %d %d\n", LUMINAIRE, occupancy_state);
          break;
        case 'a':  // anti-windup state
          Serial.printf("a %d %d\n", LUMINAIRE, pid.antiwindup_on ? 1 : 0);
          break;
        case 'k':  // feedback state
          Serial.printf("k %d %d\n", LUMINAIRE, pid.feedback_on ? 1 : 0);
          break;
        case 'x':  // external illuminance
          Serial.printf("x %d %.2f\n", LUMINAIRE,
                        max(0.0f, current_lux - static_gain * current_duty));
          break;
        case 'p':  // instantaneous power
          Serial.printf("p %d %.4f\n", LUMINAIRE, P_MAX * current_duty);
          break;
        case 't':  // elapsed time (s)
          Serial.printf("t %d %.2f\n", LUMINAIRE, micros() / 1e6f);
          break;
        case 'e':  // accumulated energy
          Serial.printf("e %d %.4f\n", LUMINAIRE, energy_acc);
          break;
        case 'v':  // average visibility error
          Serial.printf("v %d %.4f\n", LUMINAIRE, vis_error_acc);
          break;
        case 'f':  // average flicker error
          Serial.printf("f %d %.6f\n", LUMINAIRE, flicker_acc);
          break;
        default:
          Serial.println("err");
          break;
      }
      break;
    }

    // ----- s <x> <i> : start streaming (x = 'l' or 'd') -----
    case 's': {
      std::sscanf(buf, "%c %c %d", &cmd, &x, &id);
      if (id == LUMINAIRE) {
        if (x == 'l') stream_lux  = true;
        if (x == 'd') stream_duty = true;
        Serial.println("ack");
      }
      break;
    }

    // ----- S <x> <i> : stop streaming -----
    case 'S': {
      std::sscanf(buf, "%c %c %d", &cmd, &x, &id);
      if (id == LUMINAIRE) {
        if (x == 'l') stream_lux  = false;
        if (x == 'd') stream_duty = false;
        Serial.println("ack");
      }
      break;
    }

    // ----- c <subcmd> <i> <val> : change controller params -----
    case 'c': {
      std::sscanf(buf, "%c %c %d %lf", &cmd, &subcmd, &id, &val);
      if (id == LUMINAIRE) {
        switch (subcmd) {
          case 'k':  // Kp
            pid.Kp_old = pid.Kp;
            pid.Kp = (float)val;
            Serial.println("ack");
            break;
          case 'i':  // Ki
            pid.Ki = (float)val;
            Serial.println("ack");
            break;
          case 'b':  // set-point weight b
            pid.b_sp_old = pid.b_sp;
            pid.b_sp = (float)val;
            Serial.println("ack");
            break;
          case 't':  // anti-windup Tt
            if (val > 0.001)
              pid.Tt = (float)val;
            Serial.println("ack");
            break;
          case 'r':  // re-calibrate gain
            calibrateGain();
            break;
          default:
            Serial.println("err");
            break;
        }
      } else {
        Serial.println("err");
      }
      break;
    }

    default:
      Serial.println("err");
      break;
  }
}

// ======================== Setup ===============================

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogWriteResolution(12);
  analogWriteFreq(30000);  // 30 kHz PWM
  pinMode(LED_PIN, OUTPUT);
  setLED(0.0f);

  start_time_us = micros();
  last_ctrl_us  = micros();

  Serial.println("Ready.");
}

// ======================== Main Loop ===========================

void loop() {
  handleSerial();

  // 100 Hz control cycle
  unsigned long now = micros();
  if (now - last_ctrl_us < (unsigned long)(T_SAMPLE * 1e6f)) return;
  float dt = (now - last_ctrl_us) / 1e6f;
  last_ctrl_us = now;

  // Measure illuminance
  current_lux = measureLux();

  // Run controller (only when feedback is active and reference > 0)
  if (pid.feedback_on && current_ref > 0.0f) {
    float u = pid.compute(current_ref, current_lux, dt);
    setLED(u);
  } else if (!pid.feedback_on) {
    // Open-loop: hold the last set duty
    setLED(serial_duty);
  }

  // Update performance metrics
  updateMetrics(current_ref, current_lux, current_duty, dt);

  // Record into last-minute circular buffer
  buf_lux[buf_idx]  = current_lux;
  buf_duty[buf_idx] = current_duty;
  buf_idx++;
  if (buf_idx >= BUF_SIZE) { buf_idx = 0; buf_full = true; }

  // Non-blocking buffer dump (20 samples per loop iteration)
  if (buf_dumping) {
    float *data = (buf_dump_var == 'd') ? buf_duty : buf_lux;
    int batch = min(20, buf_dump_count - buf_dump_sent);
    for (int k = 0; k < batch; k++) {
      int idx = (buf_dump_start + buf_dump_sent) % BUF_SIZE;
      if (buf_dump_sent > 0) Serial.print(",");
      Serial.print(data[idx], (buf_dump_var == 'd') ? 4 : 2);
      buf_dump_sent++;
    }
    if (buf_dump_sent >= buf_dump_count) {
      Serial.println();
      buf_dumping = false;
    }
  }

  // Streaming output
  unsigned long t_ms = millis();
  if (stream_lux)
    Serial.printf("s l %d %.2f %lu\n", LUMINAIRE, current_lux, t_ms);
  if (stream_duty)
    Serial.printf("s d %d %.4f %lu\n", LUMINAIRE, current_duty, t_ms);
}
