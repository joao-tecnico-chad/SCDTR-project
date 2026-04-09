#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "can_comms.h"

// ============================================================
// System Calibration
// Measures the gain matrix K and background illuminance o.
// Model: L = K * d + o
//   K[i][j] = lux at node i when node j has duty=1.0 (minus background)
//   o[i]    = background (external) illuminance at node i
// ============================================================

static const int CALIB_MAX_NODES  = 3;
static const int CALIB_SETTLE_MS  = 2000;  // LED settle time
static const int CALIB_SAMPLE_MS  = 1000;  // measurement duration

// Calibration commands sent via CAN
enum CalibPhase : uint8_t {
  CALIB_START       = 0x01,  // begin calibration
  CALIB_ALL_OFF     = 0x02,  // all LEDs off, measure background
  CALIB_LED_ON      = 0x03,  // data[1] = which node turns on
  CALIB_REPORT      = 0x04,  // report measurement
  CALIB_DONE        = 0x05   // calibration complete
};

class Calibration {
public:
  // The gain matrix and background vector
  float K[CALIB_MAX_NODES][CALIB_MAX_NODES];
  float o[CALIB_MAX_NODES];

  int   num_nodes;
  int   my_id;       // 1-based
  bool  completed;

  // Function pointers for hardware access (set by main firmware)
  void  (*setLED)(float duty);
  float (*measureLux)();

  Calibration()
    : num_nodes(0), my_id(0), completed(false),
      setLED(nullptr), measureLux(nullptr)
  {
    memset(K, 0, sizeof(K));
    memset(o, 0, sizeof(o));
  }

  void init(int n_nodes, int node_id,
            void (*led_fn)(float), float (*lux_fn)()) {
    num_nodes  = n_nodes;
    my_id      = node_id;
    setLED     = led_fn;
    measureLux = lux_fn;
    completed  = false;
  }

  // Run calibration as the coordinating node (typically node 1).
  // All nodes must be listening for CAN calibration messages.
  void runCoordinator(CANComms &can) {
    // Phase 1: All LEDs off → measure background
    uint8_t cmd[2];
    cmd[0] = CALIB_ALL_OFF;
    cmd[1] = 0;
    can.broadcast(MSG_CALIB_CMD, cmd, 2);

    setLED(0.0f);
    delay(CALIB_SETTLE_MS);
    o[my_id - 1] = measureLux();

    // Send my background measurement
    broadcastMeasurement(can, 0, my_id, o[my_id - 1]);

    // Collect background from others
    collectMeasurements(can, 0, CALIB_SETTLE_MS);

    // Phase 2: Turn on each node one at a time
    for (int j = 0; j < num_nodes; j++) {
      int active_node = j + 1;  // 1-based

      cmd[0] = CALIB_LED_ON;
      cmd[1] = (uint8_t)active_node;
      can.broadcast(MSG_CALIB_CMD, cmd, 2);

      // Set my LED
      if (active_node == my_id)
        setLED(1.0f);
      else
        setLED(0.0f);

      delay(CALIB_SETTLE_MS);
      float lux = measureLux();

      // K[my_id-1][j] = measured lux minus background
      K[my_id - 1][j] = lux - o[my_id - 1];
      if (K[my_id - 1][j] < 0.0f) K[my_id - 1][j] = 0.0f;

      broadcastMeasurement(can, active_node, my_id, K[my_id - 1][j]);
      collectMeasurements(can, active_node, CALIB_SETTLE_MS);
    }

    // All done
    setLED(0.0f);
    cmd[0] = CALIB_DONE;
    can.broadcast(MSG_CALIB_CMD, cmd, 2);
    completed = true;
  }

  // Run calibration as a follower node (not coordinator).
  void runFollower(CANComms &can) {
    bool done = false;
    while (!done) {
      CANMessage msg;
      if (!can.receive(msg)) continue;

      if (msg.type == MSG_CALIB_CMD && msg.len >= 1) {
        uint8_t phase = msg.data[0];

        switch (phase) {
          case CALIB_ALL_OFF: {
            setLED(0.0f);
            delay(CALIB_SETTLE_MS);
            o[my_id - 1] = measureLux();
            broadcastMeasurement(can, 0, my_id, o[my_id - 1]);
            break;
          }
          case CALIB_LED_ON: {
            int active_node = msg.data[1];
            if (active_node == my_id)
              setLED(1.0f);
            else
              setLED(0.0f);
            delay(CALIB_SETTLE_MS);
            float lux = measureLux();
            K[my_id - 1][active_node - 1] = lux - o[my_id - 1];
            if (K[my_id - 1][active_node - 1] < 0.0f)
              K[my_id - 1][active_node - 1] = 0.0f;
            broadcastMeasurement(can, active_node, my_id,
                                 K[my_id - 1][active_node - 1]);
            break;
          }
          case CALIB_DONE:
            setLED(0.0f);
            done = true;
            completed = true;
            break;
        }
      }
      // Also collect measurements from other nodes
      if (msg.type == MSG_CALIB_DATA) {
        parseCalibData(msg);
      }
    }

    // Drain remaining calibration data messages
    unsigned long t = millis();
    while (millis() - t < 500) {
      CANMessage msg;
      if (can.receive(msg) && msg.type == MSG_CALIB_DATA)
        parseCalibData(msg);
    }
  }

  // Get estimated illuminance for a given duty vector
  float estimateLux(int node_idx, const float d[], int n) const {
    float lux = o[node_idx];
    for (int j = 0; j < n; j++)
      lux += K[node_idx][j] * d[j];
    return lux;
  }

private:
  void broadcastMeasurement(CANComms &can, int phase_node,
                            int reporter, float value) {
    uint8_t data[8];
    data[0] = (uint8_t)phase_node;  // which LED was on (0 = background)
    data[1] = (uint8_t)reporter;    // who measured
    memcpy(data + 2, &value, 4);
    can.broadcast(MSG_CALIB_DATA, data, 6);
  }

  void collectMeasurements(CANComms &can, int phase_node,
                           unsigned long timeout_ms) {
    unsigned long t = millis();
    int received = 0;
    int expected = num_nodes - 1;  // from other nodes

    while (received < expected && millis() - t < timeout_ms) {
      CANMessage msg;
      if (can.receive(msg) && msg.type == MSG_CALIB_DATA) {
        if (parseCalibData(msg))
          received++;
      }
    }
  }

  bool parseCalibData(const CANMessage &msg) {
    if (msg.len < 6) return false;
    int phase_node = msg.data[0];
    int reporter   = msg.data[1];
    float value;
    memcpy(&value, msg.data + 2, 4);

    if (reporter < 1 || reporter > num_nodes) return false;

    if (phase_node == 0) {
      // Background measurement
      o[reporter - 1] = value;
    } else {
      // Gain measurement
      K[reporter - 1][phase_node - 1] = value;
    }
    return true;
  }
};

#endif // CALIBRATION_H
