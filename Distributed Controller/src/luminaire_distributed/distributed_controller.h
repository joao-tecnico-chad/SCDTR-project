#ifndef DISTRIBUTED_CONTROLLER_H
#define DISTRIBUTED_CONTROLLER_H

#include "consensus.h"
#include "dual_decomp.h"
#include "admm.h"
#include "can_comms.h"

// ============================================================
// Distributed Controller — Algorithm Selector & Orchestrator
// Wraps consensus, dual decomposition, and ADMM.
// Switchable at runtime via serial command.
// ============================================================

enum Algorithm : uint8_t {
  ALG_NONE       = 0,  // local PI only, no distributed optimization
  ALG_CONSENSUS  = 1,
  ALG_DUAL_DECOMP = 2,
  ALG_ADMM       = 3
};

static const int DC_MAX_NODES = 3;

class DistributedController {
public:
  Algorithm current_alg;
  int num_nodes;
  int my_id;  // 0-based

  // Algorithm instances
  ConsensusController consensus;
  DualDecompController dual_decomp;
  ADMMController admm;

  // Shared state
  float K[DC_MAX_NODES][DC_MAX_NODES];
  float o[DC_MAX_NODES];
  float L_ref[DC_MAX_NODES];
  float c[DC_MAX_NODES];   // cost coefficients
  float d[DC_MAX_NODES];   // current duty solution

  // Occupancy per node: 0=off, 1=unoccupied(low), 2=occupied(high)
  int occupancy[DC_MAX_NODES];
  float ref_low;    // unoccupied reference
  float ref_high;   // occupied reference

  bool  needs_recompute;  // flag to trigger re-optimization

  DistributedController()
    : current_alg(ALG_NONE), num_nodes(0), my_id(0),
      ref_low(10.0f), ref_high(40.0f), needs_recompute(false)
  {
    memset(K, 0, sizeof(K));
    memset(o, 0, sizeof(o));
    memset(L_ref, 0, sizeof(L_ref));
    memset(d, 0, sizeof(d));
    memset(occupancy, 0, sizeof(occupancy));
    for (int i = 0; i < DC_MAX_NODES; i++) c[i] = 1.0f;
  }

  void init(int n, int id,
            const float gain[DC_MAX_NODES][DC_MAX_NODES],
            const float bg[]) {
    num_nodes = n;
    my_id = id;
    memcpy(K, gain, sizeof(K));
    memcpy(o, bg, sizeof(float) * n);
    updateReferences();
  }

  void setAlgorithm(Algorithm alg) {
    current_alg = alg;
    needs_recompute = true;
  }

  void setCost(int node, float cost_val) {
    if (node >= 0 && node < num_nodes) {
      c[node] = cost_val;
      needs_recompute = true;
    }
  }

  void setOccupancy(int node, int state) {
    if (node >= 0 && node < num_nodes) {
      occupancy[node] = state;
      updateReferences();
      needs_recompute = true;
    }
  }

  void updateReferences() {
    for (int i = 0; i < num_nodes; i++) {
      switch (occupancy[i]) {
        case 0:  L_ref[i] = 0.0f;      break;  // off
        case 1:  L_ref[i] = ref_low;    break;  // unoccupied
        case 2:  L_ref[i] = ref_high;   break;  // occupied
        default: L_ref[i] = ref_low;    break;
      }
    }
  }

  // Initialize the current algorithm with latest state
  void initAlgorithm() {
    switch (current_alg) {
      case ALG_CONSENSUS:
        consensus.init(num_nodes, my_id, K, o, L_ref, c);
        break;
      case ALG_DUAL_DECOMP:
        dual_decomp.init(num_nodes, my_id, K, o, L_ref, c);
        break;
      case ALG_ADMM:
        admm.init(num_nodes, my_id, K, o, L_ref, c);
        break;
      default:
        break;
    }
    needs_recompute = false;
  }

  // Run one iteration of the distributed algorithm.
  // Returns true if converged.
  bool iterate() {
    switch (current_alg) {
      case ALG_CONSENSUS:
        consensus.solveLocal();
        return consensus.converged;
      case ALG_DUAL_DECOMP:
        dual_decomp.updatePrimalSmooth();
        return dual_decomp.converged;
      case ALG_ADMM:
        admm.updateD();
        return admm.converged;
      default:
        return true;
    }
  }

  // Process received duty/variable updates from other nodes
  void receiveUpdate(int sender, const float *data) {
    switch (current_alg) {
      case ALG_CONSENSUS:
        // Received d[] proposal from sender
        for (int j = 0; j < num_nodes; j++)
          consensus.d_others[sender][j] = data[j];
        break;
      case ALG_DUAL_DECOMP:
        // Received d[sender] value
        dual_decomp.d[sender] = data[0];
        break;
      case ALG_ADMM:
        // Received d[sender]
        admm.d[sender] = data[0];
        break;
      default:
        break;
    }
  }

  // Finalize iteration after all updates received
  void finalizeIteration() {
    switch (current_alg) {
      case ALG_CONSENSUS:
        consensus.updateAverage();
        memcpy(d, consensus.d_avg, sizeof(float) * num_nodes);
        break;
      case ALG_DUAL_DECOMP:
        dual_decomp.updateDual();
        memcpy(d, dual_decomp.d, sizeof(float) * num_nodes);
        break;
      case ALG_ADMM:
        admm.updateZ();
        admm.updateU();
        memcpy(d, admm.d, sizeof(float) * num_nodes);
        break;
      default:
        break;
    }
  }

  // Broadcast this node's state for the current algorithm
  void broadcastState(CANComms &can) {
    switch (current_alg) {
      case ALG_CONSENSUS: {
        // Send full d[] vector (up to 3 floats = 12 bytes → 2 messages)
        uint8_t data[8];
        memcpy(data, consensus.d, sizeof(float) * 2);
        can.broadcast(MSG_CONSENSUS_D, data, 8);
        if (num_nodes > 2) {
          memcpy(data, &consensus.d[2], sizeof(float));
          can.broadcast(MSG_CONSENSUS_D, data, 4);
        }
        break;
      }
      case ALG_DUAL_DECOMP:
        can.broadcastFloat(MSG_DUTY_UPDATE, dual_decomp.d[my_id]);
        break;
      case ALG_ADMM:
        can.broadcastFloat(MSG_DUTY_UPDATE, admm.d[my_id]);
        break;
      default:
        break;
    }
  }

  float getMyDuty() const {
    switch (current_alg) {
      case ALG_CONSENSUS:  return consensus.getMyDuty();
      case ALG_DUAL_DECOMP: return dual_decomp.getMyDuty();
      case ALG_ADMM:       return admm.getMyDuty();
      default:             return d[my_id];
    }
  }

  bool isConverged() const {
    switch (current_alg) {
      case ALG_CONSENSUS:  return consensus.converged;
      case ALG_DUAL_DECOMP: return dual_decomp.converged;
      case ALG_ADMM:       return admm.converged;
      default:             return true;
    }
  }

  int getIteration() const {
    switch (current_alg) {
      case ALG_CONSENSUS:  return consensus.iteration;
      case ALG_DUAL_DECOMP: return dual_decomp.iteration;
      case ALG_ADMM:       return admm.iteration;
      default:             return 0;
    }
  }
};

#endif // DISTRIBUTED_CONTROLLER_H
