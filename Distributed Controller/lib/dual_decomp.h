#ifndef DUAL_DECOMP_H
#define DUAL_DECOMP_H

#include <cmath>

// ============================================================
// Dual Decomposition (Subgradient Method)
//
// Problem: minimize sum(c_i * d_i)
//          subject to: K * d + o >= L_ref
//                      0 <= d_i <= 1
//
// Lagrangian: L = sum(c_i*d_i) + sum(lambda_i*(L_ref_i - K_i*d - o_i))
// Each node updates d_i to minimize its contribution to L.
// Lambda updated via subgradient ascent.
// ============================================================

static const int DD_MAX_NODES = 3;
static const int DD_MAX_ITER  = 100;
static const float DD_TOL     = 1e-3f;

class DualDecompController {
public:
  int   num_nodes;
  int   my_id;  // 0-based

  float K[DD_MAX_NODES][DD_MAX_NODES];
  float o[DD_MAX_NODES];
  float L_ref[DD_MAX_NODES];
  float c[DD_MAX_NODES];

  float d[DD_MAX_NODES];          // primal: duty cycles
  float lambda[DD_MAX_NODES];     // dual: Lagrange multipliers

  float alpha;        // step size
  float alpha_decay;  // step size decay factor per iteration
  int   iteration;
  bool  converged;

  DualDecompController()
    : num_nodes(0), my_id(0), alpha(0.01f), alpha_decay(0.995f),
      iteration(0), converged(false)
  {
    memset(K, 0, sizeof(K));
    memset(o, 0, sizeof(o));
    memset(L_ref, 0, sizeof(L_ref));
    memset(d, 0, sizeof(d));
    memset(lambda, 0, sizeof(lambda));
    for (int i = 0; i < DD_MAX_NODES; i++) c[i] = 1.0f;
  }

  void init(int n, int id, const float gain[DD_MAX_NODES][DD_MAX_NODES],
            const float bg[], const float ref[], const float cost[]) {
    num_nodes = n;
    my_id = id;
    memcpy(K, gain, sizeof(K));
    memcpy(o, bg, sizeof(float) * n);
    memcpy(L_ref, ref, sizeof(float) * n);
    memcpy(c, cost, sizeof(float) * n);

    // Initial lambda = 0
    for (int i = 0; i < n; i++) lambda[i] = 0.0f;

    // Initial d from feedforward
    for (int i = 0; i < n; i++) {
      float needed = L_ref[i] - o[i];
      if (needed < 0) needed = 0;
      d[i] = (K[i][i] > 0) ? needed / K[i][i] : 0.0f;
      if (d[i] > 1.0f) d[i] = 1.0f;
      if (d[i] < 0.0f) d[i] = 0.0f;
    }

    iteration = 0;
    converged = false;
    alpha = 0.01f;
  }

  // Primal update: each node minimizes its Lagrangian contribution
  // d_i = argmin { c_i*d_i - sum_k(lambda_k * K[k][i] * d_i) }
  //      subject to 0 <= d_i <= 1
  void updatePrimal() {
    int i = my_id;

    // Gradient of Lagrangian w.r.t. d_i:
    // dL/d(d_i) = c_i - sum_k(lambda_k * K[k][i])
    float grad = c[i];
    for (int k = 0; k < num_nodes; k++)
      grad -= lambda[k] * K[k][i];

    // Linear cost → solution is at boundary
    if (grad > 0.0f)
      d[i] = 0.0f;
    else
      d[i] = 1.0f;

    // Smooth version: gradient descent from current d
    // d[i] = d[i] - alpha * grad;
    // Clamp:
    // if (d[i] < 0.0f) d[i] = 0.0f;
    // if (d[i] > 1.0f) d[i] = 1.0f;
  }

  // Dual update: subgradient ascent on lambda
  // After all nodes have broadcast their d values
  void updateDual() {
    float d_old[DD_MAX_NODES];
    memcpy(d_old, d, sizeof(float) * num_nodes);

    float max_violation = 0.0f;
    for (int k = 0; k < num_nodes; k++) {
      // Constraint violation: L_ref[k] - (K[k,:]*d + o[k])
      float lux = o[k];
      for (int j = 0; j < num_nodes; j++)
        lux += K[k][j] * d[j];

      float violation = L_ref[k] - lux;
      lambda[k] += alpha * violation;

      // Lambda must be non-negative (inequality constraint)
      if (lambda[k] < 0.0f) lambda[k] = 0.0f;

      if (fabsf(violation) > max_violation)
        max_violation = fabsf(violation);
    }

    // Decay step size
    alpha *= alpha_decay;

    converged = (max_violation < DD_TOL);
    iteration++;
  }

  // Smooth primal update (alternative to bang-bang)
  void updatePrimalSmooth() {
    int i = my_id;
    float grad = c[i];
    for (int k = 0; k < num_nodes; k++)
      grad -= lambda[k] * K[k][i];

    d[i] -= alpha * grad;
    if (d[i] < 0.0f) d[i] = 0.0f;
    if (d[i] > 1.0f) d[i] = 1.0f;
  }

  float getMyDuty() const { return d[my_id]; }

  float totalCost() const {
    float cost = 0.0f;
    for (int i = 0; i < num_nodes; i++)
      cost += c[i] * d[i];
    return cost;
  }
};

#endif // DUAL_DECOMP_H
