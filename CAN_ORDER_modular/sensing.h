// ============================================================================
// sensing.h
// Local sensing, actuation, peer management, and illuminance bounds
//
// Provides the ADC-to-Lux conversion with oversampling and first-order
// low-pass filtering, LED PWM output, rolling history buffers, and the
// peer table that tracks other CAN nodes discovered via hello frames.
//
// Authors: Joao Rocha, Ricardo Gaspar, Diogo Costa — SCDTR 2025/2026
// ============================================================================
#pragma once

#include "protocol_defs.h"

// ----- LDR reading with oversampling and low-pass filter -----

// Reads the LDR sensor and converts to lux using a 3-step pipeline:
//   1. Oversample: average 32 ADC readings to reduce noise (12-bit ADC)
//   2. Convert to resistance: voltage divider formula with R_FIXED (10k ohm)
//   3. Convert to lux: power-law model lux = 10^((log10(R_ldr) - B) / M)
//   4. Low-pass filter: smooths output with alpha=0.15 (15% new, 85% old)
inline float readLuxFiltered() {
  // Step 1: Oversample -- take 32 readings and average to reduce ADC noise
  uint32_t sum = 0;
  for (int i = 0; i < 32; ++i) {
    sum += analogRead(LDR_PIN);
  }

  // Step 2: Convert ADC value to voltage (0 to ADC_REF volts)
  lastLdrVoltage = (sum / 32.0f) * (ADC_REF / ADC_MAX);
  if (lastLdrVoltage < 0.001f) {
    lastLdrVoltage = 0.001f;  // Clamp to avoid divide-by-zero below
  }

  // Step 3a: Voltage divider formula -> LDR resistance
  // Circuit: VREF -- [R_FIXED] -- ADC_PIN -- [R_LDR] -- GND
  // V_adc = VREF * R_LDR / (R_FIXED + R_LDR)  =>  R_LDR = R_FIXED * V / (VREF - V)
  float rLdr = R_FIXED * (ADC_REF - lastLdrVoltage) / lastLdrVoltage;

  // Step 3b: LDR resistance to lux using power-law calibration
  // log10(R) = M * log10(lux) + B  =>  lux = 10^((log10(R) - B) / M)
  float rawLux = powf(10.0f, (log10f(rLdr) - B_PARAM) / M_PARAM);

  // Step 4: First-order low-pass filter (exponential moving average)
  // alpha=0.15 means 15% weight on new reading, 85% on previous filtered value
  // This removes high-frequency sensor noise while preserving control response
  const float alpha = 0.15f;
  filteredLux = (alpha * rawLux) + ((1.0f - alpha) * filteredLux);
  return filteredLux;
}

// ----- Power estimation -----

// Estimates instantaneous power consumption from duty cycle.
// Assumes linear relationship: power = max_power * (pwm / 4095).
// Used for energy metric accumulation in the control loop.
inline float getInstantPowerW() {
  return MAX_POWER_W * ((float)localPwm / 4095.0f);
}

// ----- History buffers -----

// Stores lux and duty readings in circular buffers for the 'b' (buffer) command.
// Lux is stored as fixed-point (lux * 100) in uint16_t to save memory.
inline void pushHistory(float lux, uint16_t pwm) {
  yHistory[historyIndex] = (uint16_t)constrain((int)(lux * 100.0f), 0, 65535);
  uHistory[historyIndex] = pwm;
  historyIndex = (historyIndex + 1) % HISTORY_LEN;
  if (historyIndex == 0) {
    historyWrapped = true;
  }
}

// ----- LED output -----

// Sets the LED brightness via 12-bit PWM (0=off, 4095=max).
// Clamps to valid range and updates the global localPwm tracker.
inline void setLedPwm(uint16_t pwm) {
  localPwm = constrain(pwm, 0, 4095);
  analogWrite(LED_PIN, localPwm);
}

// ----- Occupancy lower-bound management -----

// Updates the active illuminance lower bound based on occupancy state.
// 'h' (HIGH) = someone is at the desk -> use highLuxBound (e.g. 30 lux)
// 'l' (LOW) or 'o' (OFF) -> use lowLuxBound (e.g. 10 lux)
inline void updateCurrentLowerBound() {
  currentLuxLowerBound = (occupancyState == 'h') ? highLuxBound : lowLuxBound;
}

// ----- Peer table -----

// Updates the peer table when a Hello frame is received from another node.
// If the sender is already known, updates its values; otherwise adds a new entry.
// Peers are tracked for network awareness (who's alive, their current state).
inline void updatePeer(uint8_t senderId, uint16_t pwm, float lux, float peerRef) {
  if (senderId == nodeId) {
    return;  // Ignore our own Hello frames
  }

  // First pass: look for existing entry to update
  for (int i = 0; i < MAX_NODES; ++i) {
    if (peers[i].active && peers[i].nodeId == senderId) {
      peers[i].pwm = pwm;
      peers[i].lux = lux;
      peers[i].refLux = peerRef;
      peers[i].lastSeenMs = millis();
      return;
    }
  }

  for (int i = 0; i < MAX_NODES; ++i) {
    if (!peers[i].active) {
      peers[i].active = true;
      peers[i].nodeId = senderId;
      peers[i].pwm = pwm;
      peers[i].lux = lux;
      peers[i].refLux = peerRef;
      peers[i].lastSeenMs = millis();
      return;
    }
  }
}

inline PeerStatus *findPeer(uint8_t id) {
  for (int i = 0; i < MAX_NODES; ++i) {
    if (peers[i].active && peers[i].nodeId == id) {
      return &peers[i];
    }
  }
  return nullptr;
}

inline void removeStalePeers() {
  const uint32_t now = millis();
  for (int i = 0; i < MAX_NODES; ++i) {
    if (peers[i].active && (now - peers[i].lastSeenMs > PEER_TIMEOUT_MS)) {
      peers[i].active = false;
    }
  }
}
