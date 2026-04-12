// ============================================================================
// consensus.h
// Distributed consensus optimisation algorithm
//
// Implements a primal-averaging consensus for the multi-luminaire system.
// Each node solves a local QP that minimises energy cost subject to the
// illuminance constraints of ALL nodes, then broadcasts its duty vector.
// After collecting proposals from every peer the node averages them and
// repeats until convergence (or max iterations).
//
// The gain exchange phase is also managed here because ADMM and Dual
// Decomposition reuse the same K matrix, baseline vector, and references
// stored in ConsensusState (the `cons` global).
//
// Authors: Joao Rocha, Ricardo Gaspar, Diogo Costa — SCDTR 2025/2026
// ============================================================================
#pragma once

#include "protocol_defs.h"

// ----- Consensus core -----

inline void initConsensus() {
  cons.numNodes = totalNodes;
  cons.myIndex = nodeId - 1;
  cons.iteration = 0;
  cons.converged = false;
  cons.active = true;
  cons.iterByte = 0;

  memset(cons.d, 0, sizeof(cons.d));
  memset(cons.d_avg, 0, sizeof(cons.d_avg));
  memset(cons.d_others, 0, sizeof(cons.d_others));
  memset(cons.received, 0, sizeof(cons.received));

  // Build K matrix from calibration (convert gain-per-PWM to gain-per-duty)
  for (int j = 0; j < cons.numNodes; j++) {
    cons.K[cons.myIndex][j] = calib.gainRow[j + 1] * 4095.0f;
  }
  cons.o[cons.myIndex] = calib.baselineLux;

  // Set own reference and cost (others come from gain exchange)
  cons.L_ref[cons.myIndex] = refLux;
  cons.c[cons.myIndex] = energyCost;

  Serial.print("Consensus init: myIdx="); Serial.print(cons.myIndex);
  Serial.print(" L_ref=[");
  for (int i = 0; i < cons.numNodes; i++) { Serial.print(cons.L_ref[i]); Serial.print(" "); }
  Serial.print("] K_diag=[");
  for (int i = 0; i < cons.numNodes; i++) { Serial.print(cons.K[i][i]); Serial.print(" "); }
  Serial.println("]");

  // Initial guess: feedforward
  for (int i = 0; i < cons.numNodes; i++) {
    float needed = cons.L_ref[i] - cons.o[i];
    if (needed < 0) needed = 0;
    cons.d[i] = (cons.K[i][i] > 0) ? needed / cons.K[i][i] : 0.0f;
    cons.d[i] = constrain(cons.d[i], 0.0f, 1.0f);
  }
}

// Solve the local optimisation step: find d[myIndex] that minimises cost
// while satisfying all nodes' illuminance lower bounds.
inline void solveLocalConsensus() {
  int i = cons.myIndex;

  // Copy d_avg for all other nodes (we only optimize our own d[i])
  for (int j = 0; j < cons.numNodes; j++) {
    if (j != i) cons.d[j] = cons.d_avg[j];
  }

  // Check ALL nodes' illuminance constraints, not just our own.
  // For each node k, compute the lux it would receive from every OTHER
  // node's average duty, then determine the minimum d[i] (our duty)
  // required so that node k still meets its lower bound L_ref[k].
  // The tightest (largest) requirement across all k becomes d_min.
  float d_min = 0.0f;
  for (int k = 0; k < cons.numNodes; k++) {
    float lux_k = cons.o[k];
    for (int j = 0; j < cons.numNodes; j++) {
      if (j != i) lux_k += cons.K[k][j] * cons.d_avg[j];
    }
    if (cons.K[k][i] > 0.0f) {
      float d_needed = (cons.L_ref[k] - lux_k) / cons.K[k][i];
      if (d_needed > d_min) d_min = d_needed;
    }
  }
  if (d_min < 0.0f) d_min = 0.0f;

  // Cost-optimal unconstrained solution: d_unc = d_avg - c/rho (CONS_RHO=2.0).
  // With c=1 and rho=2.0, this shifts duty down by 0.5 at most.
  // Higher rho -> smaller cost effect, faster consensus convergence.
  float d_unc = cons.d_avg[i] - cons.c[i] / CONS_RHO;

  // Take the higher of cost-optimal and constraint-required duty.
  // Feasibility always wins over cost savings: we never violate any
  // node's illuminance lower bound.
  float d_new = (d_unc > d_min) ? d_unc : d_min;
  cons.d[i] = constrain(d_new, 0.0f, 1.0f);
}

// Average own and peers' proposals; check for convergence.
inline void updateConsensusAverage() {
  float d_old[CONS_MAX_NODES];
  memcpy(d_old, cons.d_avg, sizeof(float) * cons.numNodes);

  for (int j = 0; j < cons.numNodes; j++) {
    float sum = cons.d[j];
    int count = 1;
    for (int k = 0; k < cons.numNodes; k++) {
      if (k == cons.myIndex) continue;
      sum += cons.d_others[k][j];
      count++;
    }
    cons.d_avg[j] = sum / count;
  }

  float max_diff = 0.0f;
  for (int j = 0; j < cons.numNodes; j++) {
    float diff = fabsf(cons.d_avg[j] - d_old[j]);
    if (diff > max_diff) max_diff = diff;
  }
  cons.converged = (max_diff < CONS_TOL);
  cons.iteration++;
}

// Encode 3 duty values as uint16 (d * 10000) in 8 bytes and send.
inline void sendConsensusProposal() {
  uint8_t payload[8] = {0};
  payload[0] = cons.iterByte;
  payload[1] = (uint8_t)cons.myIndex;
  for (int j = 0; j < cons.numNodes && j < 3; j++) {
    uint16_t enc = (uint16_t)(cons.d[j] * 10000.0f);
    payload[2 + j * 2] = (enc >> 8) & 0xFF;
    payload[3 + j * 2] = enc & 0xFF;
  }
  enqueueTxFrame(CAN_ID_CONSENSUS_BASE + nodeId, payload, 8);
}

inline void handleConsensusFrame(uint8_t len, const uint8_t *data) {
  if (len < 8 || cons.mode != ALG_CONSENSUS) return;
  uint8_t senderIdx = data[1];
  if (senderIdx >= CONS_MAX_NODES || senderIdx == cons.myIndex) return;

  for (int j = 0; j < cons.numNodes && j < 3; j++) {
    uint16_t enc = ((uint16_t)data[2 + j * 2] << 8) | data[3 + j * 2];
    cons.d_others[senderIdx][j] = enc / 10000.0f;
  }
  cons.received[senderIdx] = true;
}

// ----- Gain exchange (shared by all algorithms) -----

// Send own gain row: 3 frames (one per gain value) + baseline + L_ref + cost.
inline void sendGainRow() {
  for (int j = 0; j < totalNodes && j < 3; j++) {
    uint8_t payload[8] = {0};
    payload[0] = (uint8_t)(nodeId - 1);  // sender row index
    payload[1] = (uint8_t)j;              // column index
    union { float f; uint8_t b[4]; } u;
    u.f = calib.gainRow[j + 1] * 4095.0f;  // convert to duty-space
    memcpy(&payload[2], u.b, 4);
    payload[6] = 0xAA;  // marker: gain value
    enqueueTxFrame(CAN_ID_GAINEXCH_BASE + nodeId, payload, 7);
  }
  // Send baseline + L_ref + cost in one frame
  uint8_t payload[8] = {0};
  payload[0] = (uint8_t)(nodeId - 1);
  payload[1] = 0xFF;  // marker for baseline
  union { float f; uint8_t b[4]; } u;
  u.f = calib.baselineLux;
  memcpy(&payload[2], u.b, 4);
  payload[6] = 0xBB;
  enqueueTxFrame(CAN_ID_GAINEXCH_BASE + nodeId, payload, 7);

  // Send L_ref
  memset(payload, 0, sizeof(payload));
  payload[0] = (uint8_t)(nodeId - 1);
  payload[1] = 0xFE;  // marker for L_ref
  u.f = currentLuxLowerBound;
  memcpy(&payload[2], u.b, 4);
  enqueueTxFrame(CAN_ID_GAINEXCH_BASE + nodeId, payload, 6);

  // Send cost
  memset(payload, 0, sizeof(payload));
  payload[0] = (uint8_t)(nodeId - 1);
  payload[1] = 0xFD;  // marker for cost
  u.f = energyCost;
  memcpy(&payload[2], u.b, 4);
  enqueueTxFrame(CAN_ID_GAINEXCH_BASE + nodeId, payload, 6);
}

inline void handleGainExchangeFrame(uint8_t len, const uint8_t *data) {
  if (len < 7) return;
  uint8_t rowIdx = data[0];
  uint8_t colIdx = data[1];
  if (rowIdx >= CONS_MAX_NODES) return;

  union { float f; uint8_t b[4]; } u;
  memcpy(u.b, &data[2], 4);

  if (colIdx == 0xFF) {
    cons.o[rowIdx] = u.f;
  } else if (colIdx == 0xFE) {
    cons.L_ref[rowIdx] = u.f;
  } else if (colIdx == 0xFD) {
    cons.c[rowIdx] = u.f;
  } else if (colIdx < CONS_MAX_NODES) {
    cons.K[rowIdx][colIdx] = u.f;
  }
  cons.gainRowReceived[rowIdx] = true;
}

inline bool allGainsReceived() {
  for (int i = 0; i < totalNodes; i++) {
    if (!cons.gainRowReceived[i]) return false;
  }
  return true;
}

// Main service routine: handles the gain-exchange phase then iterates
// the consensus algorithm at CONSENSUS_PERIOD_MS intervals.
inline void serviceConsensus() {
  if (cons.mode != ALG_CONSENSUS || startupState != STARTUP_READY) return;

  uint32_t now = millis();

  // Gain exchange phase: broadcast gains after calibration
  if (!cons.gainsExchanged && calib.gainsReady) {
    if (cons.gainExchStartMs == 0) {
      cons.gainExchStartMs = now;
      cons.gainRowReceived[cons.myIndex] = true;
      // Fill own row
      for (int j = 0; j < totalNodes; j++) {
        cons.K[cons.myIndex][j] = calib.gainRow[j + 1] * 4095.0f;
      }
      cons.o[cons.myIndex] = calib.baselineLux;
    }
    // Broadcast gains periodically until all received
    if (now - cons.gainExchStartMs < 5000) {
      static uint32_t lastGainSend = 0;
      if (now - lastGainSend > 500) {
        sendGainRow();
        lastGainSend = now;
      }
      if (allGainsReceived()) {
        cons.gainsExchanged = true;
        if (cons.mode == ALG_CONSENSUS) initConsensus();
        if (cons.mode == ALG_ADMM) initADMM();
        if (cons.mode == ALG_DUAL_DECOMP) initDualDecomp();
      }
      return;
    }
    // Timeout: proceed with what we have
    cons.gainsExchanged = true;
    if (cons.mode == ALG_CONSENSUS) initConsensus();
    if (cons.mode == ALG_ADMM) initADMM();
    if (cons.mode == ALG_DUAL_DECOMP) initDualDecomp();
  }

  if (!cons.active) return;
  if (cons.converged && cons.iteration >= CONS_MAX_ITER) return;

  // Run at CONSENSUS_PERIOD_MS rate
  if (now - cons.lastIterMs < CONSENSUS_PERIOD_MS) return;
  cons.lastIterMs = now;

  // Solve local
  solveLocalConsensus();

  // Broadcast proposal
  sendConsensusProposal();

  // Check if all received
  bool allRx = true;
  for (int k = 0; k < cons.numNodes; k++) {
    if (k == cons.myIndex) continue;
    if (!cons.received[k]) { allRx = false; break; }
  }

  if (allRx || cons.iteration > 0) {
    updateConsensusAverage();
    memset(cons.received, 0, sizeof(cons.received));
    cons.iterByte++;

    // Apply consensus duty to PI reference
    if (feedbackEnabled) {
      float estLux = cons.o[cons.myIndex];
      for (int j = 0; j < cons.numNodes; j++) {
        estLux += cons.K[cons.myIndex][j] * cons.d_avg[j];
      }
      refLux = estLux;
    }
  }
}
