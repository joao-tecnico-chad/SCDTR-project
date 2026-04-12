// ============================================================================
// dual_decomp.h
// Dual Decomposition algorithm for distributed luminaire control
//
// Each node performs a primal gradient descent on its own duty variable,
// then all nodes collectively update the dual variables (Lagrange
// multipliers) for the shared illuminance constraints. The step size
// decays geometrically to ensure convergence. Like ADMM, this module
// reuses the gain matrix K, offsets o, references L_ref, and costs c
// from the shared ConsensusState (`cons`).
//
// Authors: Joao Rocha, Ricardo Gaspar, Diogo Costa — SCDTR 2025/2026
// ============================================================================
#pragma once

#include "protocol_defs.h"

// ----- Dual Decomposition core routines -----

inline void initDualDecomp() {
  dd.iteration = 0;
  dd.converged = false;
  dd.active = true;
  dd.alpha = DD_ALPHA_INIT;
  dd.lastIterMs = 0;
  memset(dd.received, 0, sizeof(dd.received));
  memset(dd.lambda, 0, sizeof(dd.lambda));

  for (int i = 0; i < cons.numNodes; i++) {
    float needed = cons.L_ref[i] - cons.o[i];
    if (needed < 0) needed = 0;
    dd.d[i] = (cons.K[i][i] > 0) ? needed / cons.K[i][i] : 0.0f;
    dd.d[i] = constrain(dd.d[i], 0.0f, 1.0f);
  }
}

// Primal update: gradient descent on the Lagrangian w.r.t. own duty.
// The gradient of L w.r.t. d[i] is:
//   dL/dd[i] = c[i] - sum_k( lambda[k] * K[k][i] )
// The first term pushes duty down (minimise energy cost); the second
// pushes it up when shadow prices lambda are high (constraint violated).
// A negative step along this gradient reduces duty when cost exceeds
// the shadow price benefit, and increases it otherwise.
inline void ddUpdatePrimal() {
  int i = cons.myIndex;
  float grad = cons.c[i];
  for (int k = 0; k < cons.numNodes; k++)
    grad -= dd.lambda[k] * cons.K[k][i];

  dd.d[i] -= dd.alpha * grad;
  dd.d[i] = constrain(dd.d[i], 0.0f, 1.0f);
}

// Dual update: subgradient ascent on Lagrange multipliers.
// For each illuminance constraint (K*d + o >= L_ref), compute the
// violation = L_ref - (K*d + o).  If positive the constraint is
// violated, so lambda increases (raising the shadow price of light).
// Projection onto lambda >= 0 keeps multipliers non-negative
// (inequality constraints).
inline void ddUpdateDual() {
  float max_violation = 0.0f;
  for (int k = 0; k < cons.numNodes; k++) {
    float lux = cons.o[k];
    for (int j = 0; j < cons.numNodes; j++)
      lux += cons.K[k][j] * dd.d[j];

    float violation = cons.L_ref[k] - lux;
    dd.lambda[k] += dd.alpha * violation;
    if (dd.lambda[k] < 0.0f) dd.lambda[k] = 0.0f;

    if (fabsf(violation) > max_violation)
      max_violation = fabsf(violation);
  }
  // Step size decreases geometrically to ensure convergence:
  // large early steps explore quickly, small late steps refine.
  dd.alpha *= DD_ALPHA_DECAY;
  dd.converged = (max_violation < CONS_TOL);
  dd.iteration++;
}

inline void sendDualDecompProposal() {
  uint8_t payload[8] = {0};
  payload[0] = (uint8_t)cons.myIndex;
  union { float f; uint8_t b[4]; } u;
  u.f = dd.d[cons.myIndex];
  memcpy(&payload[1], u.b, 4);
  enqueueTxFrame(CAN_ID_DUAL_BASE + nodeId, payload, 5);
}

inline void handleDualDecompFrame(uint8_t len, const uint8_t *data) {
  if (len < 5 || cons.mode != ALG_DUAL_DECOMP) return;
  uint8_t senderIdx = data[0];
  if (senderIdx >= CONS_MAX_NODES || senderIdx == cons.myIndex) return;

  union { float f; uint8_t b[4]; } u;
  memcpy(u.b, &data[1], 4);
  dd.d[senderIdx] = u.f;
  dd.received[senderIdx] = true;
}

// Run one dual decomposition iteration per CONSENSUS_PERIOD_MS.
inline void serviceDualDecomp() {
  if (cons.mode != ALG_DUAL_DECOMP || startupState != STARTUP_READY) return;
  if (!cons.gainsExchanged) return;
  if (!dd.active) return;
  if (dd.converged && dd.iteration >= 100) return;

  uint32_t now = millis();
  if (now - dd.lastIterMs < CONSENSUS_PERIOD_MS) return;
  dd.lastIterMs = now;

  ddUpdatePrimal();
  sendDualDecompProposal();

  bool allRx = true;
  for (int k = 0; k < cons.numNodes; k++) {
    if (k == cons.myIndex) continue;
    if (!dd.received[k]) { allRx = false; break; }
  }

  if (allRx || dd.iteration > 0) {
    ddUpdateDual();
    memset(dd.received, 0, sizeof(dd.received));

    if (feedbackEnabled) {
      float estLux = cons.o[cons.myIndex];
      for (int j = 0; j < cons.numNodes; j++)
        estLux += cons.K[cons.myIndex][j] * dd.d[j];
      refLux = estLux;
    }
  }
}
