// ============================================================================
// sensing.h
// Local sensing, actuation, peer management, and illuminance bounds
//
// Provides the ADC-to-Lux conversion with oversampling and first-order
// low-pass filtering, LED PWM output, rolling history buffers, and the
// peer table that tracks other CAN nodes discovered via hello frames.
//
// Authors: Joao Rocha, SCDTR course project
// ============================================================================
#pragma once

#include "protocol_defs.h"

// ----- LDR reading with oversampling and low-pass filter -----

inline float readLuxFiltered() {
  uint32_t sum = 0;
  for (int i = 0; i < 32; ++i) {
    sum += analogRead(LDR_PIN);  // Oversample to reduce ADC noise before converting to Lux.
  }

  lastLdrVoltage = (sum / 32.0f) * (ADC_REF / ADC_MAX);
  if (lastLdrVoltage < 0.001f) {
    lastLdrVoltage = 0.001f;  // Avoid a divide-by-zero when deriving the LDR resistance.
  }

  float rLdr = R_FIXED * (ADC_REF - lastLdrVoltage) / lastLdrVoltage;
  float rawLux = powf(10.0f, (log10f(rLdr) - B_PARAM) / M_PARAM);

  const float alpha = 0.15f;
  filteredLux = (alpha * rawLux) + ((1.0f - alpha) * filteredLux);  // First-order low-pass filter.
  return filteredLux;
}

// ----- Power estimation -----

inline float getInstantPowerW() {
  return MAX_POWER_W * ((float)localPwm / 4095.0f);
}

// ----- History buffers -----

inline void pushHistory(float lux, uint16_t pwm) {
  yHistory[historyIndex] = (uint16_t)constrain((int)(lux * 100.0f), 0, 65535);
  uHistory[historyIndex] = pwm;
  historyIndex = (historyIndex + 1) % HISTORY_LEN;
  if (historyIndex == 0) {
    historyWrapped = true;
  }
}

// ----- LED output -----

inline void setLedPwm(uint16_t pwm) {
  localPwm = constrain(pwm, 0, 4095);
  analogWrite(LED_PIN, localPwm);
}

// ----- Occupancy lower-bound management -----

inline void updateCurrentLowerBound() {
  currentLuxLowerBound = (occupancyState == 'h') ? highLuxBound : lowLuxBound;
}

// ----- Peer table -----

inline void updatePeer(uint8_t senderId, uint16_t pwm, float lux, float peerRef) {
  if (senderId == nodeId) {
    return;
  }

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
