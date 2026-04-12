// ============================================================================
// can_protocol.h
// CAN protocol decoding on Core 0
//
// After a CAN frame is reassembled from the inter-core FIFO it arrives here
// for dispatch. The packet's CAN ID determines which handler runs: hello,
// command, calibration plan, query, reply, stream, gain exchange, consensus,
// ADMM, or dual decomposition. Each handler validates length and fields
// before acting on the payload.
//
// Authors: Joao Rocha, Ricardo Gaspar, Diogo Costa — SCDTR 2025/2026
// ============================================================================
#pragma once

#include "protocol_defs.h"

// ----- Command frame handler -----
// Processes CMD frames (base 0x300). These set LED duty, reference, occupancy,
// cost, bounds, anti-windup, feedback, and streaming on the target node.
// Payload: [command_type, sender_id, value_hi, value_lo]

inline void handleCommandFrame(uint32_t packetId, uint8_t packetSize, const uint8_t *data) {
  if (packetSize < 2) {
    reportProtocolError("cmd_short", packetId, packetSize);
    return;
  }

  uint16_t targetNode = packetId - CAN_ID_COMMAND_BASE;
  if (targetNode != nodeId && targetNode != BROADCAST_NODE) {
    return;
  }

  switch (data[0]) {
    case CMD_LED_SET_PWM:
      if (packetSize < 4) {
        reportProtocolError("cmd_pwm_short", packetId, packetSize);
        return;
      }
      setLedPwm(((uint16_t)data[2] << 8) | data[3]);
      break;

    case CMD_LED_OFF:
      setLedPwm(0);
      break;

    case CMD_ANNOUNCE:
      sendHelloFrame();
      break;

    case CMD_SET_REF:
      if (packetSize < 4) {
        reportProtocolError("cmd_ref_short", packetId, packetSize);
        return;
      }
      refLux = (((uint16_t)data[2] << 8) | data[3]) / 100.0f;
      break;

    case CMD_RESTART:
      resetRuntimeState();
      break;

    case CMD_SET_HIGH_BOUND:
      if (packetSize < 4) {
        reportProtocolError("cmd_high_short", packetId, packetSize);
        return;
      }
      highLuxBound = (((uint16_t)data[2] << 8) | data[3]) / 100.0f;
      updateCurrentLowerBound();
      break;

    case CMD_SET_LOW_BOUND:
      if (packetSize < 4) {
        reportProtocolError("cmd_low_short", packetId, packetSize);
        return;
      }
      lowLuxBound = (((uint16_t)data[2] << 8) | data[3]) / 100.0f;
      updateCurrentLowerBound();
      break;

    case CMD_SET_COST:
      if (packetSize < 4) {
        reportProtocolError("cmd_cost_short", packetId, packetSize);
        return;
      }
      energyCost = (((uint16_t)data[2] << 8) | data[3]) / 100.0f;
      break;

    case CMD_SET_OCCUPANCY: {
      if (packetSize < 4) {
        reportProtocolError("cmd_occ_short", packetId, packetSize);
        return;
      }
      char state = (char)data[3];
      if (state != 'o' && state != 'l' && state != 'h') {
        reportProtocolError("cmd_occ_value", packetId, packetSize);
        return;
      }
      occupancyState = state;
      updateCurrentLowerBound();
      refLux = currentLuxLowerBound;
      break;
    }

    case CMD_SET_ANTI_WINDUP:
      if (packetSize < 4) {
        reportProtocolError("cmd_aw_short", packetId, packetSize);
        return;
      }
      antiWindupEnabled = (data[3] != 0);
      break;

    case CMD_SET_FEEDBACK:
      if (packetSize < 4) {
        reportProtocolError("cmd_fb_short", packetId, packetSize);
        return;
      }
      feedbackEnabled = (data[3] != 0);
      if (feedbackEnabled) { piIntegral = 0.0f; piPrevError = 0.0f; }
      break;

    case CMD_STREAM_START:
      if (packetSize < 4) {
        reportProtocolError("cmd_stream_start_short", packetId, packetSize);
        return;
      }
      if (data[3] != 'y' && data[3] != 'u') {
        reportProtocolError("cmd_stream_start_value", packetId, packetSize);
        return;
      }
      streamState.active = true;
      streamState.variable = (char)data[3];
      break;

    case CMD_STREAM_STOP:
      if (packetSize < 4) {
        reportProtocolError("cmd_stream_stop_short", packetId, packetSize);
        return;
      }
      if (streamState.variable == (char)data[3] || data[3] == 0) {
        streamState.active = false;
        streamState.variable = 0;
      }
      break;

    default:
      reportProtocolError("cmd_unknown", packetId, packetSize);
      break;
  }
}

// ----- Calibration plan frame handler -----
// Calibration plan is split into two frames (PLAN_A + PLAN_B) because 8 bytes
// isn't enough for all parameters. When both halves are received with matching
// session IDs, the calibration session starts with synchronized timing.

inline void processCalibrationPlanFrame(uint8_t packetSize, const uint8_t *data) {
  if (packetSize < 8) {
    reportProtocolError("cal_short", CAN_ID_CALIB_BASE, packetSize);
    return;
  }

  const uint16_t sessionId = ((uint16_t)data[2] << 8) | data[3];
  if (data[0] == CALIB_PLAN_A) {
    pendingPlan.hasA = true;
    pendingPlan.sessionId = sessionId;
    pendingPlan.coordinatorNode = data[1];
    pendingPlan.calPwm = ((uint16_t)data[4] << 8) | data[5];
    pendingPlan.totalNodes = constrain((int)data[6], 1, MAX_NODES);
    pendingPlan.settleMs = (uint16_t)data[7] * CAL_TIMEBASE_MS;
  } else if (data[0] == CALIB_PLAN_B) {
    pendingPlan.hasB = true;
    pendingPlan.sessionId = sessionId;
    pendingPlan.coordinatorNode = data[1];
    pendingPlan.measureMs = (uint16_t)data[4] * CAL_TIMEBASE_MS;
    pendingPlan.gapMs = (uint16_t)data[5] * CAL_TIMEBASE_MS;
    pendingPlan.startDelayTick10 = ((uint16_t)data[6] << 8) | data[7];
  } else {
    reportProtocolError("cal_unknown", CAN_ID_CALIB_BASE, packetSize);
    return;
  }

  if (pendingPlan.hasA && pendingPlan.hasB && pendingPlan.sessionId == sessionId) {
    totalNodes = pendingPlan.totalNodes;
    const uint16_t scheduledStartTick10 = (uint16_t)(nowTick10() + pendingPlan.startDelayTick10);
    startCalibrationSession(pendingPlan.sessionId,
                            pendingPlan.calPwm,
                            pendingPlan.settleMs,
                            pendingPlan.measureMs,
                            pendingPlan.gapMs,
                            scheduledStartTick10,
                            pendingPlan.totalNodes);
    pendingPlan = CalibrationPlanBuffer();
  }
}

// ----- Query frame handler -----

inline void handleQueryFrame(uint32_t packetId, uint8_t packetSize, const uint8_t *data) {
  if (packetSize < 2) {
    reportProtocolError("query_short", packetId, packetSize);
    return;
  }

  uint16_t targetNode = packetId - CAN_ID_QUERY_BASE;
  if (targetNode != nodeId) {
    return;
  }

  bool ok = false;
  float value = readQueryValue((QueryCode)data[0], ok);
  if (!ok) {
    reportProtocolError("query_unknown", packetId, packetSize);
    return;
  }

  sendReply(data[1], (QueryCode)data[0], value);
}

// ----- Reply frame handler -----

inline void handleReplyFrame(uint8_t packetSize, const uint8_t *data) {
  if (packetSize < 6) {
    reportProtocolError("reply_short", CAN_ID_REPLY_BASE, packetSize);
    return;
  }

  union {
    float f;
    uint8_t b[4];
  } payload;

  payload.b[0] = data[2];
  payload.b[1] = data[3];
  payload.b[2] = data[4];
  payload.b[3] = data[5];

  char code = (char)data[0];
  uint8_t sourceNode = data[1];
  if (pendingRemoteQuery.active && pendingRemoteQuery.code == (QueryCode)code && pendingRemoteQuery.targetNode == sourceNode) {
    pendingRemoteQuery.active = false;
  }

  Serial.print(code);
  Serial.print(" ");
  Serial.print(sourceNode);
  Serial.print(" ");
  if (code == 'o') {
    Serial.print((char)((int)payload.f));
  } else if (code == 'a' || code == 'f') {
    Serial.print((int)payload.f);
  } else if (code == 'u') {
    Serial.print(payload.f, 0);
  } else {
    Serial.print(payload.f, 3);
  }
  printQueryLabel(code);
  Serial.println();
}

// ----- Stream frame handler -----

inline void handleStreamFrame(uint8_t packetSize, const uint8_t *data) {
  if (packetSize < 8) {
    reportProtocolError("stream_short", CAN_ID_STREAM_BASE, packetSize);
    return;
  }

  union {
    float f;
    uint8_t b[4];
  } payload;

  payload.b[0] = data[2];
  payload.b[1] = data[3];
  payload.b[2] = data[4];
  payload.b[3] = data[5];
  uint16_t timeMs = ((uint16_t)data[6] << 8) | data[7];

  Serial.print("s ");
  Serial.print((char)data[0]);
  Serial.print(" ");
  Serial.print(data[1]);
  Serial.print(" ");
  Serial.print(payload.f, ((char)data[0] == 'u') ? 0 : 3);
  Serial.print(" ");
  Serial.println(timeMs);
}

// ----- Top-level CAN frame dispatcher -----
// Routes every received CAN frame to the appropriate handler based on
// the CAN ID range. The ID space is partitioned:
//   0x100-0x17F: Hello (heartbeat/discovery)
//   0x300-0x37F: Command (set duty, ref, occupancy, etc.)
//   0x400-0x47F: Calibration plan
//   0x500-0x57F: Query (request value from peer)
//   0x580-0x5FF: Reply (response to query)
//   0x600-0x67F: Stream (real-time telemetry)
//   0x680-0x6FF: Gain Exchange (K matrix rows after calibration)
//   0x700-0x77F: Consensus (duty proposals)
//   0x780-0x7BF: ADMM (primal/dual variables)
//   0x7C0-0x7FF: Dual Decomposition (duty/lambda)

inline void handleReceivedCanFrame(const CanFrame &frame) {
  if (startupState != STARTUP_READY) {
    return;
  }

  const uint32_t packetId = frame.id;
  const uint8_t packetSize = frame.len;
  const uint8_t *data = frame.data;

  if (packetId >= CAN_ID_HELLO_BASE && packetId < (CAN_ID_HELLO_BASE + 0x80)) {
    if (packetSize < 7) {
      reportProtocolError("hello_short", packetId, packetSize);
      return;
    }
    uint8_t senderId = data[0];
    uint16_t pwm = ((uint16_t)data[1] << 8) | data[2];
    float lux = (((uint16_t)data[3] << 8) | data[4]) / 100.0f;
    float peerRef = (((uint16_t)data[5] << 8) | data[6]) / 100.0f;
    updatePeer(senderId, pwm, lux, peerRef);
    return;
  }

  if (packetId >= CAN_ID_COMMAND_BASE && packetId < (CAN_ID_COMMAND_BASE + 0x80)) {
    handleCommandFrame(packetId, packetSize, data);
    return;
  }

  if (packetId >= CAN_ID_CALIB_BASE && packetId < (CAN_ID_CALIB_BASE + 0x80)) {
    processCalibrationPlanFrame(packetSize, data);
    return;
  }

  if (packetId >= CAN_ID_QUERY_BASE && packetId < (CAN_ID_QUERY_BASE + 0x80)) {
    handleQueryFrame(packetId, packetSize, data);
    return;
  }

  if (packetId >= CAN_ID_REPLY_BASE && packetId < (CAN_ID_REPLY_BASE + 0x80)) {
    handleReplyFrame(packetSize, data);
    return;
  }

  if (packetId >= CAN_ID_STREAM_BASE && packetId < (CAN_ID_STREAM_BASE + 0x80)) {
    handleStreamFrame(packetSize, data);
    return;
  }

  if (packetId >= CAN_ID_GAINEXCH_BASE && packetId < (CAN_ID_GAINEXCH_BASE + 0x80)) {
    handleGainExchangeFrame(packetSize, data);
    return;
  }

  if (packetId >= CAN_ID_CONSENSUS_BASE && packetId < (CAN_ID_CONSENSUS_BASE + 0x80)) {
    handleConsensusFrame(packetSize, data);
    return;
  }

  if (packetId >= CAN_ID_ADMM_BASE && packetId < (CAN_ID_ADMM_BASE + 0x80)) {
    handleADMMFrame(packetSize, data);
    return;
  }

  if (packetId >= CAN_ID_DUAL_BASE && packetId < (CAN_ID_DUAL_BASE + 0x40)) {
    handleDualDecompFrame(packetSize, data);
    return;
  }

  reportProtocolError("id_unknown", packetId, packetSize);
}

// ----- FIFO reassembly from Core 1 -----

// Reassembles FIFO words sent by Core 1 into CAN frames or diagnostics, then
// dispatches them into the application-level protocol handlers above.
inline void serviceFifoFromCore1() {
  static uint8_t stage = 0;
  static uint32_t words[3] = {0};
  static FifoMessageKind kind = FIFO_RX_FRAME;

  uint32_t word = 0;
  while (rp2040.fifo.pop_nb(&word)) {
    if (stage == 0) {
      kind = (FifoMessageKind)((word >> 24) & 0xFF);
      words[0] = word;
      if (kind == FIFO_DIAG) {
        reportCore1Diag((Core1DiagStage)((word >> 16) & 0xFF),
                        (uint8_t)((word >> 8) & 0xFF),
                        (uint8_t)(word & 0xFF));
      } else {
        stage = 1;
      }
      continue;
    }

    words[stage] = word;
    stage++;
    if (stage == 3) {
      stage = 0;
      if (kind != FIFO_RX_FRAME) {
        continue;
      }

      CanFrame frame;
      frame.id = (uint16_t)((words[0] >> 8) & 0xFFFF);
      frame.len = (uint8_t)(words[0] & 0xFF);
      unpackBytes32(words[1], frame.data);
      unpackBytes32(words[2], frame.data + 4);
      if (frame.len > 8) {
        reportProtocolError("fifo_len", frame.id, frame.len);
        continue;
      }
      handleReceivedCanFrame(frame);
    }
  }
}
