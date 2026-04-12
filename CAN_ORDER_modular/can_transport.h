// ============================================================================
// can_transport.h
// Inter-core FIFO helpers, TX/RX queues, and high-level CAN message builders
//
// The RP2040 dual-core architecture requires that only Core 1 touches the
// MCP2515 SPI driver. Core 0 enqueues outbound frames into a ring buffer,
// which is then forwarded word-by-word through the hardware FIFO. Received
// frames travel in the opposite direction. This module implements all of
// that plumbing plus the typed CAN frame builders (hello, command, query,
// reply, stream, calibration plan).
//
// Authors: Joao Rocha, SCDTR course project
// ============================================================================
#pragma once

#include "protocol_defs.h"

// ----- Timer ISR and CAN interrupt -----

// The repeating timer only signals that one control period elapsed.
inline bool controlTimerCallback(repeating_timer_t *timer) {
  (void)timer;
  if (controlDueCount < 255) {
    controlDueCount++;  // Saturating counter: Core 0 consumes one pending control step at a time.
  }
  return true;
}

// The CAN interrupt handler is intentionally minimal and defers all MCP2515
// access to Core 1's main loop.
inline void canIrqHandler() {
  canIrqPending = true;
}

// ----- FIFO packing/unpacking -----
// CAN frames are sent between cores as 3 x uint32_t words through the RP2040 FIFO:
//   Word 0: [kind:8][canId:16][length:8]  (header)
//   Word 1: data[0..3] packed little-endian  (payload low)
//   Word 2: data[4..7] packed little-endian  (payload high)
// Diagnostics use only 1 word: [FIFO_DIAG:8][stage:8][code:8][detail:8]

inline uint32_t packFrameHeader(FifoMessageKind kind, uint16_t canId, uint8_t length) {
  // Layout: [kind:8][canId:16][len:8]
  return ((uint32_t)kind << 24) | ((uint32_t)canId << 8) | length;
}

inline uint32_t packBytes32(const uint8_t *bytes) {
  return ((uint32_t)bytes[0]) |
         ((uint32_t)bytes[1] << 8) |
         ((uint32_t)bytes[2] << 16) |
         ((uint32_t)bytes[3] << 24);
}

inline void unpackBytes32(uint32_t word, uint8_t *bytes) {
  bytes[0] = (uint8_t)(word & 0xFF);
  bytes[1] = (uint8_t)((word >> 8) & 0xFF);
  bytes[2] = (uint8_t)((word >> 16) & 0xFF);
  bytes[3] = (uint8_t)((word >> 24) & 0xFF);
}

inline uint32_t packDiagWord(Core1DiagStage stage, uint8_t code, uint8_t detail) {
  return ((uint32_t)FIFO_DIAG << 24) | ((uint32_t)stage << 16) | ((uint32_t)code << 8) | detail;
}

// ----- Calibration tick helpers -----

inline uint16_t nowTick10() {
  // Calibration timing is exchanged in 10 ms ticks so one byte can represent up to 2.55 s.
  return (uint16_t)((millis() / CAL_TIMEBASE_MS) & 0xFFFF);
}

inline bool tick10Reached(uint16_t targetTick) {
  // Signed subtraction preserves correct wrap-around behavior on the 16-bit tick counter.
  return (int16_t)(nowTick10() - targetTick) >= 0;
}

// ----- Utility printers -----

inline void printAck() {
  Serial.println("ack");
}

inline void printErr() {
  Serial.println("err");
}

inline void reportProtocolError(const char *stage, uint32_t canId, uint8_t length) {
  canProtocolErrorCount++;
  Serial.print("can_err ");
  Serial.print(stage);
  Serial.print(" id=0x");
  Serial.print(canId, HEX);
  Serial.print(" len=");
  Serial.println(length);
}

inline void reportCore1Diag(Core1DiagStage stage, uint8_t code, uint8_t detail) {
  if (stage == DIAG_CAN_SEND) {
    canTxErrorCount++;
  }
  if (stage == DIAG_CAN_CHECK || stage == DIAG_CAN_READ || stage == DIAG_CAN_HEALTH || stage == DIAG_CAN_FIFO_DROP) {
    canRxErrorCount++;
  }

  // Suppress health check spam (only count, don't print)
  if (stage == DIAG_CAN_HEALTH) return;

  Serial.print("can_err ");
  switch (stage) {
    case DIAG_CAN_INIT: Serial.print("init"); break;
    case DIAG_CAN_MODE: Serial.print("mode"); break;
    case DIAG_CAN_SEND: Serial.print("tx"); break;
    case DIAG_CAN_CHECK: Serial.print("rx_check"); break;
    case DIAG_CAN_READ: Serial.print("rx_read"); break;
    case DIAG_CAN_HEALTH: Serial.print("health"); break;
    case DIAG_CAN_FIFO_DROP: Serial.print("fifo_drop"); break;
    default: Serial.print("unknown"); break;
  }
  Serial.print(" code=");
  Serial.print(code);
  Serial.print(" detail=");
  Serial.println(detail);
}

inline uint8_t encodeCalTick(uint16_t msValue) {
  // Compress millisecond durations into 10 ms units to fit the calibration plan inside 8-byte frames.
  return (uint8_t)constrain((int)((msValue + (CAL_TIMEBASE_MS / 2)) / CAL_TIMEBASE_MS), 1, 255);
}

// ----- Core 0 -> Core 1 TX queue -----

// Core 0 never touches the MCP2515 directly; it only enqueues high-level CAN
// frames to be forwarded to Core 1 through the RP2040 FIFO.
inline bool enqueueTxFrame(uint16_t canId, const uint8_t *payload, uint8_t length) {
  if (length > 8 || txQueueCount >= TX_QUEUE_LEN) {
    canTxErrorCount++;
    Serial.println("can_err tx_queue_full");
    return false;
  }

  CanFrame &frame = txQueue[txQueueTail];
  frame.id = canId;
  frame.len = length;
  memset(frame.data, 0, sizeof(frame.data));  // Zero the unused bytes so packed FIFO words stay deterministic.
  memcpy(frame.data, payload, length);

  txQueueTail = (txQueueTail + 1) % TX_QUEUE_LEN;
  txQueueCount++;
  return true;
}

inline bool dequeueTxFrame(CanFrame &frame) {
  if (txQueueCount == 0) {
    return false;
  }

  frame = txQueue[txQueueHead];
  txQueueHead = (txQueueHead + 1) % TX_QUEUE_LEN;
  txQueueCount--;
  return true;
}

inline void prepareFifoFrame(FifoTxStaging &staging, FifoMessageKind kind, const CanFrame &frame) {
  staging.active = true;
  staging.index = 0;
  staging.wordCount = 3;
  staging.words[0] = packFrameHeader(kind, frame.id, frame.len);
  staging.words[1] = packBytes32(frame.data);
  staging.words[2] = packBytes32(frame.data + 4);
}

inline void prepareFifoDiag(FifoTxStaging &staging, Core1DiagStage stage, uint8_t code, uint8_t detail) {
  staging.active = true;
  staging.index = 0;
  staging.wordCount = 1;
  staging.words[0] = packDiagWord(stage, code, detail);
}

// Pushes queued transmit requests to Core 1 using nonblocking FIFO writes.
inline void serviceCore0ToCore1Fifo() {
  if (!core0ToCore1Staging.active) {
    CanFrame frame;
    if (dequeueTxFrame(frame)) {
      prepareFifoFrame(core0ToCore1Staging, FIFO_TX_FRAME, frame);
    }
  }

  while (core0ToCore1Staging.active) {
    if (!rp2040.fifo.push_nb(core0ToCore1Staging.words[core0ToCore1Staging.index])) {
      return;  // Nonblocking by design: leave the partially-sent frame staged for the next loop iteration.
    }
    core0ToCore1Staging.index++;
    if (core0ToCore1Staging.index >= core0ToCore1Staging.wordCount) {
      core0ToCore1Staging.active = false;
    }
  }
}

// ----- Core 1 -> Core 0 RX queue -----

inline bool enqueueCore1EventFrame(const CanFrame &frame) {
  if (core1EventCount >= CORE1_EVENT_QUEUE_LEN) {
    return false;
  }

  core1EventQueue[core1EventTail] = frame;
  core1EventTail = (core1EventTail + 1) % CORE1_EVENT_QUEUE_LEN;
  core1EventCount++;
  return true;
}

inline bool dequeueCore1EventFrame(CanFrame &frame) {
  if (core1EventCount == 0) {
    return false;
  }

  frame = core1EventQueue[core1EventHead];
  core1EventHead = (core1EventHead + 1) % CORE1_EVENT_QUEUE_LEN;
  core1EventCount--;
  return true;
}

inline void queueCore1Diag(Core1DiagStage stage, uint8_t code, uint8_t detail) {
  if (core1ToCore0Staging.active) {
    return;
  }
  prepareFifoDiag(core1ToCore0Staging, stage, code, detail);
}

// Returns received CAN frames and diagnostics back to Core 0 without blocking
// Core 1's CAN service loop.
inline void serviceCore1ToCore0Fifo() {
  if (!core1ToCore0Staging.active) {
    CanFrame frame;
    if (dequeueCore1EventFrame(frame)) {
      prepareFifoFrame(core1ToCore0Staging, FIFO_RX_FRAME, frame);
    }
  }

  while (core1ToCore0Staging.active) {
    if (!rp2040.fifo.push_nb(core1ToCore0Staging.words[core1ToCore0Staging.index])) {
      return;  // Keep Core 1 responsive to CAN even if FIFO is temporarily full.
    }
    core1ToCore0Staging.index++;
    if (core1ToCore0Staging.index >= core1ToCore0Staging.wordCount) {
      core1ToCore0Staging.active = false;
    }
  }
}

// ----- High-level CAN message builders -----
// Each builder constructs a payload byte array and calls enqueueTxFrame().
// The CAN ID = BASE + nodeId (or targetNode for unicast messages).

// Hello frame: broadcast this node's current state to all peers.
// Payload (7 bytes): [nodeId, pwm_hi, pwm_lo, lux_hi, lux_lo, ref_hi, ref_lo]
// Lux and ref are fixed-point: value * 100, stored as uint16.
inline void sendHelloFrame() {
  uint16_t luxEnc = (uint16_t)constrain((int)(filteredLux * 100.0f), 0, 65535);
  uint16_t refEnc = (uint16_t)constrain((int)(refLux * 100.0f), 0, 65535);
  uint8_t payload[7] = {
    nodeId,
    (uint8_t)((localPwm >> 8) & 0xFF),
    (uint8_t)(localPwm & 0xFF),
    (uint8_t)((luxEnc >> 8) & 0xFF),
    (uint8_t)(luxEnc & 0xFF),
    (uint8_t)((refEnc >> 8) & 0xFF),
    (uint8_t)(refEnc & 0xFF)
  };
  enqueueTxFrame(CAN_ID_HELLO_BASE + nodeId, payload, sizeof(payload));
}

// Command frame: send a set/control command to a specific node (or broadcast).
// Payload (4 bytes): [command_type, sender_id, value_hi, value_lo]
inline void sendSimpleCommand(uint8_t targetNode, CommandType command, uint16_t value) {
  uint8_t payload[4] = {
    (uint8_t)command,
    nodeId,
    (uint8_t)((value >> 8) & 0xFF),
    (uint8_t)(value & 0xFF)
  };
  enqueueTxFrame(CAN_ID_COMMAND_BASE + targetNode, payload, sizeof(payload));
}

// Query frame: ask a remote node for a specific value (lux, duty, energy, etc.).
// Payload (2 bytes): [query_code, requester_id]
// The reply comes back as a Reply frame with the float value.
inline void sendQuery(uint8_t targetNode, QueryCode queryCode) {
  uint8_t payload[2] = {(uint8_t)queryCode, nodeId};
  if (enqueueTxFrame(CAN_ID_QUERY_BASE + targetNode, payload, sizeof(payload))) {
    pendingRemoteQuery.active = true;
    pendingRemoteQuery.code = queryCode;
    pendingRemoteQuery.targetNode = targetNode;
    pendingRemoteQuery.requestedAtMs = millis();
  }
}

// Reply frame: respond to a query with a float value.
// Payload (6 bytes): [query_code, sender_id, float_b0, float_b1, float_b2, float_b3]
// Float is sent as raw IEEE 754 bytes (little-endian).
inline void sendReply(uint8_t requesterNode, QueryCode queryCode, float value) {
  union {
    float f;
    uint8_t b[4];
  } payload;
  payload.f = value;

  uint8_t frame[6] = {
    (uint8_t)queryCode,
    nodeId,
    payload.b[0],
    payload.b[1],
    payload.b[2],
    payload.b[3]
  };
  enqueueTxFrame(CAN_ID_REPLY_BASE + requesterNode, frame, sizeof(frame));
}

inline void sendCharCommand(uint8_t targetNode, CommandType command, char value) {
  sendSimpleCommand(targetNode, command, (uint16_t)(uint8_t)value);
}

// Stream frame: broadcast real-time telemetry (lux or duty) with timestamp.
// Payload (8 bytes): [variable, nodeId, float_b0..b3, time_hi, time_lo]
inline void sendStreamFrame(char variable) {
  float value = (variable == 'y') ? filteredLux : (float)localPwm;
  union {
    float f;
    uint8_t b[4];
  } payload;
  payload.f = value;
  uint32_t t = millis();

  uint8_t frame[8] = {
    (uint8_t)variable,
    nodeId,
    payload.b[0],
    payload.b[1],
    payload.b[2],
    payload.b[3],
    (uint8_t)((t >> 8) & 0xFF),
    (uint8_t)(t & 0xFF)
  };
  enqueueTxFrame(CAN_ID_STREAM_BASE + nodeId, frame, sizeof(frame));
}
