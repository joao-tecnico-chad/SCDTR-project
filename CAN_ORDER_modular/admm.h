// ============================================================================
// admm.h
// Alternating Direction Method of Multipliers (ADMM) for distributed control
//
// ADMM splits the optimisation into a local d-update (each node independently),
// a global z-update (projected onto the feasible set defined by K*z + o >= L_ref),
// and a dual u-update. The algorithm reuses the gain matrix K, baseline o,
// references L_ref, and costs c stored in the shared ConsensusState (`cons`).
//
// Authors: Joao Rocha, SCDTR course project
// ============================================================================
#pragma once

#include "protocol_defs.h"

// ----- ADMM core routines -----

inline void initADMM() {
  admm.iteration = 0;
  admm.converged = false;
  admm.active = true;
  admm.lastIterMs = 0;
  memset(admm.received, 0, sizeof(admm.received));

  // Initialize from feedforward using consensus K matrix
  for (int i = 0; i < cons.numNodes; i++) {
    float needed = cons.L_ref[i] - cons.o[i];
    if (needed < 0) needed = 0;
    admm.d[i] = (cons.K[i][i] > 0) ? needed / cons.K[i][i] : 0.0f;
    admm.d[i] = constrain(admm.d[i], 0.0f, 1.0f);
    admm.z[i] = admm.d[i];
    admm.u[i] = 0.0f;
  }
}

// d-update: closed-form primal update.
// Minimise the local cost  c[i]*d[i] + (rho/2)*||d[i] - z[i] + u[i]||^2
// Taking the derivative and setting to zero gives:
//   d[i] = z[i] - u[i] - c[i]/rho
// Then clamp to [0,1] to keep the duty physically valid.
inline void admmUpdateD() {
  int i = cons.myIndex;
  admm.d[i] = admm.z[i] - admm.u[i] - cons.c[i] / ADMM_RHO;
  admm.d[i] = constrain(admm.d[i], 0.0f, 1.0f);
}

// z-update: project onto the feasible set defined by K*z + o >= L_ref.
// Start from the unconstrained optimum z = d + u, then iteratively clip
// any violated illuminance constraint by pushing z along the constraint
// gradient (the corresponding row of K).  This is Dykstra-style
// alternating projection, repeated up to 20 times or until all
// constraints are satisfied.
inline void admmUpdateZ() {
  float z_target[CONS_MAX_NODES];
  for (int i = 0; i < cons.numNodes; i++)
    z_target[i] = admm.d[i] + admm.u[i];

  memcpy(admm.z, z_target, sizeof(float) * cons.numNodes);

  for (int iter = 0; iter < 20; iter++) {
    bool feasible = true;
    for (int k = 0; k < cons.numNodes; k++) {
      float lux = cons.o[k];
      for (int j = 0; j < cons.numNodes; j++)
        lux += cons.K[k][j] * admm.z[j];

      // If node k's illuminance falls below its reference, push z
      // towards feasibility along the K[k] direction.
      if (lux < cons.L_ref[k]) {
        feasible = false;
        float deficit = cons.L_ref[k] - lux;
        float norm_sq = 0.0f;
        for (int j = 0; j < cons.numNodes; j++)
          norm_sq += cons.K[k][j] * cons.K[k][j];
        if (norm_sq > 1e-8f) {
          float step = deficit / norm_sq;
          for (int j = 0; j < cons.numNodes; j++)
            admm.z[j] += step * cons.K[k][j];
        }
      }
    }
    // Clamp to physical duty range [0,1] after each projection pass
    for (int j = 0; j < cons.numNodes; j++)
      admm.z[j] = constrain(admm.z[j], 0.0f, 1.0f);
    if (feasible) break;
  }
}

// u-update: dual ascent step.
// Accumulate the constraint violation (d - z) into the scaled dual
// variable u.  When d != z the nodes disagree on the global variable;
// u drives them towards consensus over successive iterations.
// Convergence is declared when the primal residual ||d - z|| is small.
inline void admmUpdateU() {
  float primal_res = 0.0f;
  for (int i = 0; i < cons.numNodes; i++) {
    admm.u[i] += admm.d[i] - admm.z[i];
    primal_res += (admm.d[i] - admm.z[i]) * (admm.d[i] - admm.z[i]);
  }
  admm.converged = (sqrtf(primal_res) < CONS_TOL);
  admm.iteration++;
}

// Send d[myIndex] and u[myIndex] over CAN.
inline void sendADMMProposal() {
  uint8_t payload[8] = {0};
  payload[0] = (uint8_t)cons.myIndex;
  union { float f; uint8_t b[4]; } ud, uu;
  ud.f = admm.d[cons.myIndex];
  memcpy(&payload[1], ud.b, 4);
  // Pack u as int16 (u * 1000)
  int16_t u_enc = (int16_t)(admm.u[cons.myIndex] * 1000.0f);
  payload[5] = (u_enc >> 8) & 0xFF;
  payload[6] = u_enc & 0xFF;
  enqueueTxFrame(CAN_ID_ADMM_BASE + nodeId, payload, 7);
}

inline void handleADMMFrame(uint8_t len, const uint8_t *data) {
  if (len < 7 || cons.mode != ALG_ADMM) return;
  uint8_t senderIdx = data[0];
  if (senderIdx >= CONS_MAX_NODES || senderIdx == cons.myIndex) return;

  union { float f; uint8_t b[4]; } ud;
  memcpy(ud.b, &data[1], 4);
  admm.d[senderIdx] = ud.f;

  int16_t u_enc = ((int16_t)data[5] << 8) | data[6];
  admm.u[senderIdx] = u_enc / 1000.0f;

  admm.received[senderIdx] = true;
}

// Run one ADMM iteration per CONSENSUS_PERIOD_MS.
inline void serviceADMM() {
  if (cons.mode != ALG_ADMM || startupState != STARTUP_READY) return;
  if (!cons.gainsExchanged) return;  // need gains first
  if (!admm.active) return;
  if (admm.converged && admm.iteration >= CONS_MAX_ITER) return;

  uint32_t now = millis();
  if (now - admm.lastIterMs < CONSENSUS_PERIOD_MS) return;
  admm.lastIterMs = now;

  // d-update (local)
  admmUpdateD();

  // Broadcast d and u
  sendADMMProposal();

  // Check if all received
  bool allRx = true;
  for (int k = 0; k < cons.numNodes; k++) {
    if (k == cons.myIndex) continue;
    if (!admm.received[k]) { allRx = false; break; }
  }

  if (allRx || admm.iteration > 0) {
    // z-update (all nodes compute same z from shared d,u)
    admmUpdateZ();
    // u-update
    admmUpdateU();
    memset(admm.received, 0, sizeof(admm.received));

    // Apply to PI reference
    if (feedbackEnabled) {
      float estLux = cons.o[cons.myIndex];
      for (int j = 0; j < cons.numNodes; j++)
        estLux += cons.K[cons.myIndex][j] * admm.z[j];
      refLux = estLux;
    }
  }
}
