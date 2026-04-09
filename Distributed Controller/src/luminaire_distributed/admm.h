#ifndef ADMM_H
#define ADMM_H

#include <cmath>

// ============================================================
// ADMM (Alternating Direction Method of Multipliers)
//
// Problem: minimize sum(c_i * d_i)
//          subject to: K * d + o >= L_ref
//                      0 <= d_i <= 1
//
// Reformulation with consensus variable z:
//   minimize sum(c_i*d_i) + indicator(K*z+o >= L_ref)
//   subject to: d = z
//
// ADMM updates:
//   d-update: d_i = argmin { c_i*d_i + (rho/2)||d_i - z_i + u_i||^2 }
//   z-update: z = argmin { indicator(K*z+o>=L_ref) + (rho/2)||d-z+u||^2 }
//   u-update: u = u + d - z
// ============================================================

static const int ADMM_MAX_NODES = 3;
static const int ADMM_MAX_ITER  = 50;
static const float ADMM_TOL     = 1e-3f;

class ADMMController {
public:
  int   num_nodes;
  int   my_id;  // 0-based

  float K[ADMM_MAX_NODES][ADMM_MAX_NODES];
  float o[ADMM_MAX_NODES];
  float L_ref[ADMM_MAX_NODES];
  float c[ADMM_MAX_NODES];

  float d[ADMM_MAX_NODES];   // primal variable (local copies)
  float z[ADMM_MAX_NODES];   // consensus variable
  float u[ADMM_MAX_NODES];   // scaled dual variable

  float rho;        // penalty parameter
  int   iteration;
  bool  converged;

  ADMMController()
    : num_nodes(0), my_id(0), rho(1.0f), iteration(0), converged(false)
  {
    memset(K, 0, sizeof(K));
    memset(o, 0, sizeof(o));
    memset(L_ref, 0, sizeof(L_ref));
    memset(d, 0, sizeof(d));
    memset(z, 0, sizeof(z));
    memset(u, 0, sizeof(u));
    for (int i = 0; i < ADMM_MAX_NODES; i++) c[i] = 1.0f;
  }

  void init(int n, int id, const float gain[ADMM_MAX_NODES][ADMM_MAX_NODES],
            const float bg[], const float ref[], const float cost[]) {
    num_nodes = n;
    my_id = id;
    memcpy(K, gain, sizeof(K));
    memcpy(o, bg, sizeof(float) * n);
    memcpy(L_ref, ref, sizeof(float) * n);
    memcpy(c, cost, sizeof(float) * n);

    // Initialize from feedforward
    for (int i = 0; i < n; i++) {
      float needed = L_ref[i] - o[i];
      if (needed < 0) needed = 0;
      d[i] = (K[i][i] > 0) ? needed / K[i][i] : 0.0f;
      if (d[i] > 1.0f) d[i] = 1.0f;
      if (d[i] < 0.0f) d[i] = 0.0f;
      z[i] = d[i];
      u[i] = 0.0f;
    }
    iteration = 0;
    converged = false;
  }

  // d-update: minimize c_i*d_i + (rho/2)*(d_i - z_i + u_i)^2
  // subject to 0 <= d_i <= 1
  // Solution: d_i = z_i - u_i - c_i/rho, clamped to [0,1]
  void updateD() {
    int i = my_id;
    d[i] = z[i] - u[i] - c[i] / rho;
    if (d[i] < 0.0f) d[i] = 0.0f;
    if (d[i] > 1.0f) d[i] = 1.0f;
  }

  // z-update: project onto the constraint set K*z+o >= L_ref
  // while staying close to (d + u).
  // This is a QP: minimize (rho/2)||z - (d+u)||^2
  //               subject to K*z + o >= L_ref
  // Solved via active-set / iterative projection.
  void updateZ() {
    // Target: z_target = d + u
    float z_target[ADMM_MAX_NODES];
    for (int i = 0; i < num_nodes; i++)
      z_target[i] = d[i] + u[i];

    // Start from target, iteratively project
    memcpy(z, z_target, sizeof(float) * num_nodes);

    for (int iter = 0; iter < 20; iter++) {
      bool feasible = true;
      for (int k = 0; k < num_nodes; k++) {
        float lux = o[k];
        for (int j = 0; j < num_nodes; j++)
          lux += K[k][j] * z[j];

        if (lux < L_ref[k]) {
          feasible = false;
          // Project: increase z proportionally to K[k,:]
          float deficit = L_ref[k] - lux;
          float norm_sq = 0.0f;
          for (int j = 0; j < num_nodes; j++)
            norm_sq += K[k][j] * K[k][j];
          if (norm_sq > 1e-8f) {
            float step = deficit / norm_sq;
            for (int j = 0; j < num_nodes; j++) {
              z[j] += step * K[k][j];
            }
          }
        }
      }
      // Box constraint
      for (int j = 0; j < num_nodes; j++) {
        if (z[j] < 0.0f) z[j] = 0.0f;
        if (z[j] > 1.0f) z[j] = 1.0f;
      }
      if (feasible) break;
    }
  }

  // u-update: dual variable
  void updateU() {
    float primal_res = 0.0f;
    float dual_res   = 0.0f;
    float z_old[ADMM_MAX_NODES];
    memcpy(z_old, z, sizeof(float) * num_nodes);

    for (int i = 0; i < num_nodes; i++) {
      u[i] += d[i] - z[i];
      primal_res += (d[i] - z[i]) * (d[i] - z[i]);
    }

    converged = (sqrtf(primal_res) < ADMM_TOL);
    iteration++;
  }

  float getMyDuty() const { return d[my_id]; }

  float totalCost() const {
    float cost = 0.0f;
    for (int i = 0; i < num_nodes; i++)
      cost += c[i] * d[i];
    return cost;
  }
};

#endif // ADMM_H
