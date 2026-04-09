#ifndef CONSENSUS_H
#define CONSENSUS_H

#include <cmath>

// ============================================================
// Consensus-based Distributed Optimization
//
// Problem: minimize sum(c_i * d_i)
//          subject to: K * d + o >= L_ref  (per node)
//                      0 <= d_i <= 1       (per node)
//
// Each node solves a local QP, then averages its solution
// with proposals received from neighbors.
// ============================================================

static const int CONS_MAX_NODES = 3;
static const int CONS_MAX_ITER  = 50;
static const float CONS_TOL     = 1e-3f;

class ConsensusController {
public:
  int   num_nodes;
  int   my_id;         // 0-based index

  float K[CONS_MAX_NODES][CONS_MAX_NODES];  // gain matrix
  float o[CONS_MAX_NODES];                  // background
  float L_ref[CONS_MAX_NODES];              // illuminance references
  float c[CONS_MAX_NODES];                  // cost coefficients

  float d[CONS_MAX_NODES];                  // current solution (duty)
  float d_avg[CONS_MAX_NODES];              // averaged solution
  float d_others[CONS_MAX_NODES][CONS_MAX_NODES]; // proposals from others

  float rho;           // penalty parameter for augmented cost
  int   iteration;
  bool  converged;

  ConsensusController()
    : num_nodes(0), my_id(0), rho(0.01f), iteration(0), converged(false)
  {
    memset(K, 0, sizeof(K));
    memset(o, 0, sizeof(o));
    memset(L_ref, 0, sizeof(L_ref));
    memset(c, 0, sizeof(c));
    memset(d, 0, sizeof(d));
    memset(d_avg, 0, sizeof(d_avg));
    memset(d_others, 0, sizeof(d_others));
    for (int i = 0; i < CONS_MAX_NODES; i++) c[i] = 1.0f;
  }

  void init(int n, int id, const float gain[CONS_MAX_NODES][CONS_MAX_NODES],
            const float bg[], const float ref[], const float cost[]) {
    num_nodes = n;
    my_id = id;
    memcpy(K, gain, sizeof(K));
    memcpy(o, bg, sizeof(float) * n);
    memcpy(L_ref, ref, sizeof(float) * n);
    memcpy(c, cost, sizeof(float) * n);

    // Initial guess: feedforward from local gain
    for (int i = 0; i < n; i++) {
      float needed = L_ref[i] - o[i];
      if (needed < 0) needed = 0;
      d[i] = (K[i][i] > 0) ? needed / K[i][i] : 0.0f;
      if (d[i] > 1.0f) d[i] = 1.0f;
      if (d[i] < 0.0f) d[i] = 0.0f;
    }
    iteration = 0;
    converged = false;
  }

  // Solve the local optimization subproblem.
  // Node i minimizes: c_i*d_i + (rho/2)*sum_j(d_j - d_avg_j)^2
  // subject to: K[i,:]*d + o[i] >= L_ref[i], 0 <= d_j <= 1
  // Only optimizes d[my_id]; keeps others at d_avg.
  void solveLocal() {
    int i = my_id;

    // Unconstrained optimum for d[i]:
    // derivative of cost w.r.t. d[i] = c[i] + rho*(d[i] - d_avg[i]) = 0
    // But must also satisfy illuminance constraint.

    // Try unconstrained minimum
    float d_unc = d_avg[i] - c[i] / rho;

    // Check illuminance constraint for this node
    // L_i = K[i][i]*d[i] + sum_{j!=i} K[i][j]*d_avg[j] + o[i] >= L_ref[i]
    float lux_others = o[i];
    for (int j = 0; j < num_nodes; j++) {
      if (j != i) lux_others += K[i][j] * d_avg[j];
    }

    float d_min_constraint = 0.0f;
    if (K[i][i] > 0.0f) {
      d_min_constraint = (L_ref[i] - lux_others) / K[i][i];
    }

    // Also check constraints for other nodes (cross-coupling)
    for (int k = 0; k < num_nodes; k++) {
      if (k == i) continue;
      float lux_k = o[k];
      for (int j = 0; j < num_nodes; j++) {
        if (j != i) lux_k += K[k][j] * d_avg[j];
      }
      if (K[k][i] > 0.0f) {
        float d_needed = (L_ref[k] - lux_k) / K[k][i];
        if (d_needed > d_min_constraint)
          d_min_constraint = d_needed;
      }
    }

    // Pick the larger of unconstrained and constraint-satisfying
    float d_new = d_unc;
    if (d_new < d_min_constraint) d_new = d_min_constraint;

    // Box constraint
    if (d_new < 0.0f) d_new = 0.0f;
    if (d_new > 1.0f) d_new = 1.0f;

    d[i] = d_new;
  }

  // Update the average after receiving proposals from all nodes
  void updateAverage() {
    float d_old[CONS_MAX_NODES];
    memcpy(d_old, d_avg, sizeof(float) * num_nodes);

    for (int j = 0; j < num_nodes; j++) {
      float sum = d[j];  // my proposal
      int count = 1;
      for (int k = 0; k < num_nodes; k++) {
        if (k == my_id) continue;
        sum += d_others[k][j];
        count++;
      }
      d_avg[j] = sum / count;
    }

    // Check convergence
    float max_diff = 0.0f;
    for (int j = 0; j < num_nodes; j++) {
      float diff = fabsf(d_avg[j] - d_old[j]);
      if (diff > max_diff) max_diff = diff;
    }
    converged = (max_diff < CONS_TOL);
    iteration++;
  }

  // Get this node's optimal duty cycle
  float getMyDuty() const {
    return d[my_id];
  }

  // Get total cost
  float totalCost() const {
    float cost = 0.0f;
    for (int i = 0; i < num_nodes; i++)
      cost += c[i] * d_avg[i];
    return cost;
  }
};

#endif // CONSENSUS_H
