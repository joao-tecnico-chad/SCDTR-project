#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <pico/time.h>

// ============================================================================
// Hardware Mapping And Timing Configuration
// ============================================================================

const uint8_t LED_PIN = 15;
const uint8_t LDR_PIN = 26;
const uint8_t CAN_CS_PIN = 17;
const uint8_t CAN_INT_PIN = 20;

const float ADC_REF = 3.3f;
const float ADC_MAX = 4095.0f;
const float R_FIXED = 3000.0f;
const float M_PARAM = -1.0f;
const float B_PARAM = 5.8f;
const float MAX_POWER_W = 1.0f;

const uint32_t CONTROL_PERIOD_MS = 10;
const uint32_t HELLO_PERIOD_MS = 1000;
const uint32_t STREAM_PERIOD_MS = 100;
const uint32_t WAKEUP_WINDOW_MS = 5000;
const uint32_t WAKEUP_RETRY_MS = 500;
const uint32_t PEER_TIMEOUT_MS = 8000;
const uint32_t CORE1_HEALTH_PERIOD_MS = 250;
const uint32_t DIAG_PERIOD_MS = 1000;

const uint16_t CAN_ID_HELLO_BASE = 0x100;
const uint16_t CAN_ID_COMMAND_BASE = 0x300;
const uint16_t CAN_ID_CALIB_BASE = 0x400;
const uint16_t CAN_ID_QUERY_BASE = 0x500;
const uint16_t CAN_ID_REPLY_BASE = 0x580;
const uint16_t CAN_ID_STREAM_BASE = 0x600;

const uint8_t BROADCAST_NODE = 0x7F;
const int MAX_NODES = 8;
const int HISTORY_LEN = 600;
const int SERIAL_BUFFER_LEN = 96;
const int TX_QUEUE_LEN = 24;
const int CORE1_EVENT_QUEUE_LEN = 24;

const uint16_t DEFAULT_CAL_PWM = 2800;
const uint16_t DEFAULT_SETTLE_MS = 250;
const uint16_t DEFAULT_MEASURE_MS = 600;
const uint16_t DEFAULT_GAP_MS = 250;
const uint16_t DEFAULT_START_DELAY_MS = 1500;
const uint8_t CAL_PLAN_REPEAT_COUNT = 5;
const uint32_t CAL_PLAN_REPEAT_INTERVAL_MS = 60;

const uint8_t CAL_TIMEBASE_MS = 10;

MCP_CAN canBus(CAN_CS_PIN);
repeating_timer_t controlTimer;

// ============================================================================
// Protocol And Runtime State Definitions
// ============================================================================

enum CommandType : uint8_t {
  CMD_LED_SET_PWM = 0x01,
  CMD_LED_OFF = 0x02,
  CMD_ANNOUNCE = 0x03,
  CMD_SET_REF = 0x04,
  CMD_RESTART = 0x05,
  CMD_SET_HIGH_BOUND = 0x06,
  CMD_SET_LOW_BOUND = 0x07,
  CMD_SET_COST = 0x08,
  CMD_SET_OCCUPANCY = 0x09,
  CMD_SET_ANTI_WINDUP = 0x0A,
  CMD_SET_FEEDBACK = 0x0B,
  CMD_STREAM_START = 0x0C,
  CMD_STREAM_STOP = 0x0D
};

enum CalibType : uint8_t {
  CALIB_PLAN_A = 0x10,
  CALIB_PLAN_B = 0x11
};

enum QueryCode : uint8_t {
  Q_U = 'u',
  Q_R = 'r',
  Q_Y = 'y',
  Q_V = 'v',
  Q_OCC = 'o',
  Q_AW = 'a',
  Q_FB = 'f',
  Q_D = 'd',
  Q_P = 'p',
  Q_T = 't',
  Q_E = 'E',
  Q_VIS = 'V',
  Q_FLICK = 'F',
  Q_REF_HIGH = 'O',
  Q_REF_LOW = 'U',
  Q_REF_CURRENT = 'L',
  Q_COST = 'C'
};

enum StartupState : uint8_t {
  STARTUP_WAIT_NODE_ID = 0,
  STARTUP_WAIT_TOTAL_NODES = 1,
  STARTUP_READY = 2
};

enum WakeupState : uint8_t {
  WAKEUP_BOOT = 0,
  WAKEUP_DISCOVERY_OPEN = 1,
  WAKEUP_DISCOVERY_STABLE = 2,
  WAKEUP_RUN = 3
};

enum CalibrationState : uint8_t {
  CAL_IDLE = 0,
  CAL_WAIT_START_TIME = 1,
  CAL_BASELINE_SETTLE = 2,
  CAL_BASELINE_MEASURE = 3,
  CAL_SLOT_SETTLE = 4,
  CAL_SLOT_MEASURE = 5,
  CAL_FINISHED = 6
};

enum FifoMessageKind : uint8_t {
  FIFO_TX_FRAME = 0x01,
  FIFO_RX_FRAME = 0x02,
  FIFO_DIAG = 0x03
};

enum Core1DiagStage : uint8_t {
  DIAG_CAN_INIT = 1,
  DIAG_CAN_MODE = 2,
  DIAG_CAN_SEND = 3,
  DIAG_CAN_CHECK = 4,
  DIAG_CAN_READ = 5,
  DIAG_CAN_HEALTH = 6,
  DIAG_CAN_FIFO_DROP = 7
};

struct PeerStatus {
  bool active = false;
  uint8_t nodeId = 0;
  uint16_t pwm = 0;
  float lux = 0.0f;
  float refLux = 0.0f;
  uint32_t lastSeenMs = 0;
};

struct StreamState {
  bool active = false;
  char variable = 0;
};

struct CanFrame {
  uint16_t id = 0;
  uint8_t len = 0;
  uint8_t data[8] = {0};
};

struct FifoTxStaging {
  bool active = false;
  uint32_t words[3] = {0};
  uint8_t wordCount = 0;
  uint8_t index = 0;
};

struct CalibrationPlanBuffer {
  bool hasA = false;
  bool hasB = false;
  uint16_t sessionId = 0;
  uint16_t calPwm = 0;
  uint8_t totalNodes = 0;
  uint16_t settleMs = 0;
  uint16_t measureMs = 0;
  uint16_t gapMs = 0;
  uint16_t startDelayTick10 = 0;
  uint8_t coordinatorNode = 0;
};

struct CalibrationContext {
  CalibrationState state = CAL_IDLE;
  bool active = false;
  uint16_t sessionId = 0;
  uint16_t calPwm = DEFAULT_CAL_PWM;
  uint8_t totalNodes = 0;
  uint16_t settleMs = DEFAULT_SETTLE_MS;
  uint16_t measureMs = DEFAULT_MEASURE_MS;
  uint16_t gapMs = DEFAULT_GAP_MS;
  uint16_t scheduledStartTick10 = 0;
  uint32_t phaseDeadlineMs = 0;
  int currentSlot = -1;
  float baselineLux = -1.0f;
  float slotLux[MAX_NODES + 1] = {0};
  float gainRow[MAX_NODES + 1] = {0};
  bool gainsReady = false;
  float measureAccumulator = 0.0f;
  uint16_t measureCount = 0;
};

struct PendingRemoteQuery {
  bool active = false;
  QueryCode code = Q_U;
  uint8_t targetNode = 0;
  uint32_t requestedAtMs = 0;
};

struct CalibrationBroadcastState {
  bool active = false;
  uint16_t sessionId = 0;
  uint16_t calPwm = DEFAULT_CAL_PWM;
  uint8_t totalNodes = 1;
  uint16_t settleMs = DEFAULT_SETTLE_MS;
  uint16_t measureMs = DEFAULT_MEASURE_MS;
  uint16_t gapMs = DEFAULT_GAP_MS;
  uint16_t startDelayTick10 = 0;
  uint8_t repeatsRemaining = 0;
  uint32_t nextSendMs = 0;
};

// ============================================================================
// Shared Runtime State Owned By Core 0
// ============================================================================

PeerStatus peers[MAX_NODES];
StreamState streamState;
CalibrationPlanBuffer pendingPlan;
CalibrationContext calib;
PendingRemoteQuery pendingRemoteQuery;
CalibrationBroadcastState calibBroadcast;

CanFrame txQueue[TX_QUEUE_LEN];
uint8_t txQueueHead = 0;
uint8_t txQueueTail = 0;
uint8_t txQueueCount = 0;
FifoTxStaging core0ToCore1Staging;

uint8_t nodeId = 1;
uint8_t totalNodes = 1;
bool isCoordinator = true;
StartupState startupState = STARTUP_WAIT_NODE_ID;
WakeupState wakeupState = WAKEUP_BOOT;
bool autoCalibrationTriggered = false;

float filteredLux = 0.0f;
float lastLdrVoltage = 0.0f;
uint16_t localPwm = 0;
float refLux = 20.0f;
char occupancyState = 'o';
bool antiWindupEnabled = true;
bool feedbackEnabled = false;
float highLuxBound = 30.0f;
float lowLuxBound = 10.0f;
float currentLuxLowerBound = 10.0f;
float energyCost = 1.0f;

float energyJ = 0.0f;
float visibilityErrorIntegral = 0.0f;
float flickerIntegral = 0.0f;
float restartSeconds = 0.0f;

uint16_t yHistory[HISTORY_LEN];
uint16_t uHistory[HISTORY_LEN];
int historyIndex = 0;
bool historyWrapped = false;

char serialBuffer[SERIAL_BUFFER_LEN];
size_t serialLength = 0;

uint32_t bootMs = 0;
uint32_t lastHelloMs = 0;
uint32_t lastWakeupMs = 0;
uint32_t lastStreamMs = 0;
uint32_t lastDiagMs = 0;

uint32_t canTxErrorCount = 0;
uint32_t canRxErrorCount = 0;
uint32_t canProtocolErrorCount = 0;
uint32_t controlOverrunCount = 0;
uint32_t controlMaxExecUs = 0;
uint32_t controlLastExecUs = 0;

volatile uint8_t controlDueCount = 0;

volatile bool canIrqPending = false;
CanFrame core1EventQueue[CORE1_EVENT_QUEUE_LEN];
uint8_t core1EventHead = 0;
uint8_t core1EventTail = 0;
uint8_t core1EventCount = 0;
FifoTxStaging core1ToCore0Staging;
uint32_t core1LastHealthMs = 0;

// ============================================================================
// Interrupt And FIFO Packing Helpers
// ============================================================================

// The repeating timer only signals that one control period elapsed.
bool controlTimerCallback(repeating_timer_t *timer) {
  (void)timer;
  if (controlDueCount < 255) {
    controlDueCount++;  // Saturating counter: Core 0 consumes one pending control step at a time.
  }
  return true;
}

// The CAN interrupt handler is intentionally minimal and defers all MCP2515
// access to Core 1's main loop.
void canIrqHandler() {
  canIrqPending = true;
}

// FIFO traffic is encoded as a small fixed number of 32-bit words so the two
// cores can exchange CAN frames without sharing driver state directly.
uint32_t packFrameHeader(FifoMessageKind kind, uint16_t canId, uint8_t length) {
  // Layout: [kind:8][canId:16][len:8]
  return ((uint32_t)kind << 24) | ((uint32_t)canId << 8) | length;
}

uint32_t packBytes32(const uint8_t *bytes) {
  // Packs four payload bytes into one FIFO word to keep the inter-core protocol compact.
  return ((uint32_t)bytes[0]) |
         ((uint32_t)bytes[1] << 8) |
         ((uint32_t)bytes[2] << 16) |
         ((uint32_t)bytes[3] << 24);
}

void unpackBytes32(uint32_t word, uint8_t *bytes) {
  // Inverse of packBytes32(): reconstruct the original CAN payload bytes.
  bytes[0] = (uint8_t)(word & 0xFF);
  bytes[1] = (uint8_t)((word >> 8) & 0xFF);
  bytes[2] = (uint8_t)((word >> 16) & 0xFF);
  bytes[3] = (uint8_t)((word >> 24) & 0xFF);
}

uint32_t packDiagWord(Core1DiagStage stage, uint8_t code, uint8_t detail) {
  return ((uint32_t)FIFO_DIAG << 24) | ((uint32_t)stage << 16) | ((uint32_t)code << 8) | detail;
}

uint16_t nowTick10() {
  // Calibration timing is exchanged in 10 ms ticks so one byte can represent up to 2.55 s.
  return (uint16_t)((millis() / CAL_TIMEBASE_MS) & 0xFFFF);
}

bool tick10Reached(uint16_t targetTick) {
  // Signed subtraction preserves correct wrap-around behavior on the 16-bit tick counter.
  return (int16_t)(nowTick10() - targetTick) >= 0;
}

void printAck() {
  Serial.println("ack");
}

void printErr() {
  Serial.println("err");
}

void reportProtocolError(const char *stage, uint32_t canId, uint8_t length) {
  canProtocolErrorCount++;
  Serial.print("can_err ");
  Serial.print(stage);
  Serial.print(" id=0x");
  Serial.print(canId, HEX);
  Serial.print(" len=");
  Serial.println(length);
}

void reportCore1Diag(Core1DiagStage stage, uint8_t code, uint8_t detail) {
  if (stage == DIAG_CAN_SEND) {
    canTxErrorCount++;
  }
  if (stage == DIAG_CAN_CHECK || stage == DIAG_CAN_READ || stage == DIAG_CAN_HEALTH || stage == DIAG_CAN_FIFO_DROP) {
    canRxErrorCount++;
  }

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

// ============================================================================
// Local Sensing And Actuation Helpers
// ============================================================================

// Reads the LDR, converts it to Lux, and applies a simple low-pass filter so
// the control task works with a less noisy illuminance estimate.
float readLuxFiltered() {
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

float getInstantPowerW() {
  return MAX_POWER_W * ((float)localPwm / 4095.0f);
}

void pushHistory(float lux, uint16_t pwm) {
  yHistory[historyIndex] = (uint16_t)constrain((int)(lux * 100.0f), 0, 65535);
  uHistory[historyIndex] = pwm;
  historyIndex = (historyIndex + 1) % HISTORY_LEN;
  if (historyIndex == 0) {
    historyWrapped = true;
  }
}

void setLedPwm(uint16_t pwm) {
  localPwm = constrain(pwm, 0, 4095);
  analogWrite(LED_PIN, localPwm);
}

void updateCurrentLowerBound() {
  currentLuxLowerBound = (occupancyState == 'h') ? highLuxBound : lowLuxBound;
}

void updatePeer(uint8_t senderId, uint16_t pwm, float lux, float peerRef) {
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

PeerStatus *findPeer(uint8_t id) {
  for (int i = 0; i < MAX_NODES; ++i) {
    if (peers[i].active && peers[i].nodeId == id) {
      return &peers[i];
    }
  }
  return nullptr;
}

void removeStalePeers() {
  const uint32_t now = millis();
  for (int i = 0; i < MAX_NODES; ++i) {
    if (peers[i].active && (now - peers[i].lastSeenMs > PEER_TIMEOUT_MS)) {
      peers[i].active = false;
    }
  }
}

// ============================================================================
// Core 0 -> Core 1 Transmit Queue And FIFO Transport
// ============================================================================

// Core 0 never touches the MCP2515 directly; it only enqueues high-level CAN
// frames to be forwarded to Core 1 through the RP2040 FIFO.
bool enqueueTxFrame(uint16_t canId, const uint8_t *payload, uint8_t length) {
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

bool dequeueTxFrame(CanFrame &frame) {
  if (txQueueCount == 0) {
    return false;
  }

  frame = txQueue[txQueueHead];
  txQueueHead = (txQueueHead + 1) % TX_QUEUE_LEN;
  txQueueCount--;
  return true;
}

void prepareFifoFrame(FifoTxStaging &staging, FifoMessageKind kind, const CanFrame &frame) {
  staging.active = true;
  staging.index = 0;
  staging.wordCount = 3;
  staging.words[0] = packFrameHeader(kind, frame.id, frame.len);
  staging.words[1] = packBytes32(frame.data);
  staging.words[2] = packBytes32(frame.data + 4);
}

void prepareFifoDiag(FifoTxStaging &staging, Core1DiagStage stage, uint8_t code, uint8_t detail) {
  staging.active = true;
  staging.index = 0;
  staging.wordCount = 1;
  staging.words[0] = packDiagWord(stage, code, detail);
}

// Pushes queued transmit requests to Core 1 using nonblocking FIFO writes.
void serviceCore0ToCore1Fifo() {
  if (!core0ToCore1Staging.active) {
    CanFrame frame;
    if (dequeueTxFrame(frame)) {
      prepareFifoFrame(core0ToCore1Staging, FIFO_TX_FRAME, frame);  // Stage one full CAN frame for FIFO transfer.
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

// ============================================================================
// Core 1 -> Core 0 Receive Queue And FIFO Transport
// ============================================================================

bool enqueueCore1EventFrame(const CanFrame &frame) {
  if (core1EventCount >= CORE1_EVENT_QUEUE_LEN) {
    return false;
  }

  core1EventQueue[core1EventTail] = frame;
  core1EventTail = (core1EventTail + 1) % CORE1_EVENT_QUEUE_LEN;
  core1EventCount++;
  return true;
}

bool dequeueCore1EventFrame(CanFrame &frame) {
  if (core1EventCount == 0) {
    return false;
  }

  frame = core1EventQueue[core1EventHead];
  core1EventHead = (core1EventHead + 1) % CORE1_EVENT_QUEUE_LEN;
  core1EventCount--;
  return true;
}

void queueCore1Diag(Core1DiagStage stage, uint8_t code, uint8_t detail) {
  if (core1ToCore0Staging.active) {
    return;
  }
  prepareFifoDiag(core1ToCore0Staging, stage, code, detail);
}

// Returns received CAN frames and diagnostics back to Core 0 without blocking
// Core 1's CAN service loop.
void serviceCore1ToCore0Fifo() {
  if (!core1ToCore0Staging.active) {
    CanFrame frame;
    if (dequeueCore1EventFrame(frame)) {
      prepareFifoFrame(core1ToCore0Staging, FIFO_RX_FRAME, frame);  // Ship a received CAN frame back to Core 0.
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

// ============================================================================
// High-Level CAN Message Builders
// ============================================================================

void sendHelloFrame() {
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

void sendSimpleCommand(uint8_t targetNode, CommandType command, uint16_t value = 0) {
  uint8_t payload[4] = {
    (uint8_t)command,
    nodeId,
    (uint8_t)((value >> 8) & 0xFF),
    (uint8_t)(value & 0xFF)
  };
  enqueueTxFrame(CAN_ID_COMMAND_BASE + targetNode, payload, sizeof(payload));
}

void sendQuery(uint8_t targetNode, QueryCode queryCode) {
  uint8_t payload[2] = {(uint8_t)queryCode, nodeId};
  if (enqueueTxFrame(CAN_ID_QUERY_BASE + targetNode, payload, sizeof(payload))) {
    pendingRemoteQuery.active = true;  // Track one outstanding query so timeouts can be reported later.
    pendingRemoteQuery.code = queryCode;
    pendingRemoteQuery.targetNode = targetNode;
    pendingRemoteQuery.requestedAtMs = millis();
  }
}

void sendReply(uint8_t requesterNode, QueryCode queryCode, float value) {
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

void sendCharCommand(uint8_t targetNode, CommandType command, char value) {
  sendSimpleCommand(targetNode, command, (uint16_t)(uint8_t)value);
}

void sendStreamFrame(char variable) {
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

uint8_t encodeCalTick(uint16_t msValue) {
  // Compress millisecond durations into 10 ms units to fit the calibration plan inside 8-byte frames.
  return (uint8_t)constrain((int)((msValue + (CAL_TIMEBASE_MS / 2)) / CAL_TIMEBASE_MS), 1, 255);
}

// ============================================================================
// Calibration Session Management
// ============================================================================

void resetCalibrationArrays() {
  calib.baselineLux = -1.0f;
  calib.gainsReady = false;
  calib.measureAccumulator = 0.0f;
  calib.measureCount = 0;
  for (int i = 0; i <= MAX_NODES; ++i) {
    calib.slotLux[i] = -1.0f;
    calib.gainRow[i] = 0.0f;
  }
}

void enterCalibrationState(CalibrationState state, uint32_t deadlineMs) {
  calib.state = state;
  calib.phaseDeadlineMs = deadlineMs;   // Absolute deadline keeps the state machine nonblocking.
  calib.measureAccumulator = 0.0f;      // Start a fresh average for the new phase.
  calib.measureCount = 0;
}

void finishCalibration() {
  calib.active = false;
  calib.state = CAL_FINISHED;
  setLedPwm(0);

  if (calib.baselineLux < 0.0f) {
    return;
  }

  for (int sourceId = 1; sourceId <= calib.totalNodes; ++sourceId) {
    if (calib.slotLux[sourceId] >= 0.0f) {
      calib.gainRow[sourceId] = (calib.slotLux[sourceId] - calib.baselineLux) / (float)calib.calPwm;
    }
  }

  calib.gainsReady = true;
}

void startCalibrationSession(uint16_t sessionId,
                             uint16_t calPwm,
                             uint16_t settleMs,
                             uint16_t measureMs,
                             uint16_t gapMs,
                             uint16_t scheduledStartTick10,
                             uint8_t planNodes) {
  calib.active = true;
  calib.sessionId = sessionId;
  calib.calPwm = calPwm;
  calib.settleMs = settleMs;
  calib.measureMs = measureMs;
  calib.gapMs = gapMs;
  calib.scheduledStartTick10 = scheduledStartTick10;
  calib.totalNodes = constrain(planNodes, 1, MAX_NODES);
  calib.currentSlot = -1;
  resetCalibrationArrays();
  setLedPwm(0);
  calib.state = CAL_WAIT_START_TIME;
  calib.phaseDeadlineMs = 0;
}

// Calibration is announced asynchronously with two bounded frames. The CAN plan
// carries only a relative start delay, and each node converts it to a local
// scheduled start tick on reception.
void broadcastCalibrationPlan(uint16_t sessionId,
                              uint16_t calPwm,
                              uint8_t planNodes,
                              uint16_t settleMs,
                              uint16_t measureMs,
                              uint16_t gapMs,
                              uint16_t startDelayTick10) {
  uint8_t planA[8] = {
    CALIB_PLAN_A,
    nodeId,
    (uint8_t)((sessionId >> 8) & 0xFF),
    (uint8_t)(sessionId & 0xFF),
    (uint8_t)((calPwm >> 8) & 0xFF),
    (uint8_t)(calPwm & 0xFF),
    planNodes,
    encodeCalTick(settleMs)  // Frame A carries node count, calibration PWM, and settle time.
  };

  uint8_t planB[8] = {
    CALIB_PLAN_B,
    nodeId,
    (uint8_t)((sessionId >> 8) & 0xFF),
    (uint8_t)(sessionId & 0xFF),
    encodeCalTick(measureMs),
    encodeCalTick(gapMs),
    (uint8_t)((startDelayTick10 >> 8) & 0xFF),
    (uint8_t)(startDelayTick10 & 0xFF)  // Frame B carries measurement/gap timing and the relative start delay.
  };

  enqueueTxFrame(CAN_ID_CALIB_BASE + BROADCAST_NODE, planA, sizeof(planA));
  enqueueTxFrame(CAN_ID_CALIB_BASE + BROADCAST_NODE, planB, sizeof(planB));
}

void scheduleCalibrationPlanBroadcast(uint16_t sessionId,
                                      uint16_t calPwm,
                                      uint8_t planNodes,
                                      uint16_t settleMs,
                                      uint16_t measureMs,
                                      uint16_t gapMs,
                                      uint16_t startDelayTick10) {
  calibBroadcast.active = true;
  calibBroadcast.sessionId = sessionId;
  calibBroadcast.calPwm = calPwm;
  calibBroadcast.totalNodes = planNodes;
  calibBroadcast.settleMs = settleMs;
  calibBroadcast.measureMs = measureMs;
  calibBroadcast.gapMs = gapMs;
  calibBroadcast.startDelayTick10 = startDelayTick10;
  calibBroadcast.repeatsRemaining = CAL_PLAN_REPEAT_COUNT;
  calibBroadcast.nextSendMs = millis();
}

// Resets the local node state while preserving the multicore architecture and
// immediately restarts the distributed wake-up/discovery sequence.
void resetRuntimeState() {
  energyJ = 0.0f;
  visibilityErrorIntegral = 0.0f;
  flickerIntegral = 0.0f;
  restartSeconds = 0.0f;
  historyIndex = 0;
  historyWrapped = false;
  streamState.active = false;
  streamState.variable = 0;
  pendingRemoteQuery.active = false;
  calibBroadcast = CalibrationBroadcastState();
  autoCalibrationTriggered = false;
  memset(peers, 0, sizeof(peers));
  pendingPlan = CalibrationPlanBuffer();
  calib = CalibrationContext();
  setLedPwm(0);
  updateCurrentLowerBound();
  bootMs = millis();
  filteredLux = readLuxFiltered();
  wakeupState = WAKEUP_DISCOVERY_OPEN;
  lastHelloMs = 0;
  lastWakeupMs = 0;
  sendHelloFrame();
  sendSimpleCommand(BROADCAST_NODE, CMD_ANNOUNCE);
}

void printCalibrationReport() {
  if (!calib.gainsReady) {
    Serial.println("Calibracao ainda nao concluida.");
    return;
  }

  Serial.print("No ");
  Serial.print(nodeId);
  Serial.println(" | ganhos estimados:");
  Serial.print("  luz_base=");
  Serial.println(calib.baselineLux, 3);

  for (int sourceId = 1; sourceId <= calib.totalNodes; ++sourceId) {
    Serial.print("  G[");
    Serial.print(nodeId);
    Serial.print("][");
    Serial.print(sourceId);
    Serial.print("] = ");
    Serial.println(calib.gainRow[sourceId], 6);
  }
}

void printBuffer(char variable) {
  uint16_t *buffer = (variable == 'y') ? yHistory : uHistory;
  int count = historyWrapped ? HISTORY_LEN : historyIndex;
  int start = historyWrapped ? historyIndex : 0;

  Serial.print("b ");
  Serial.print(variable);
  Serial.print(" ");
  Serial.print(nodeId);
  Serial.print(" ");
  for (int i = 0; i < count; ++i) {
    int idx = (start + i) % HISTORY_LEN;
    if (variable == 'y') {
      Serial.print(buffer[idx] / 100.0f, 2);
    } else {
      Serial.print(buffer[idx]);
    }
    if (i < count - 1) {
      Serial.print(", ");
    }
  }
  Serial.println();
}

float readQueryValue(QueryCode queryCode, bool &ok) {
  ok = true;
  switch (queryCode) {
    case Q_U: return (float)localPwm;
    case Q_R: return refLux;
    case Q_Y: return filteredLux;
    case Q_V: return lastLdrVoltage;
    case Q_OCC: return (float)occupancyState;
    case Q_AW: return antiWindupEnabled ? 1.0f : 0.0f;
    case Q_FB: return feedbackEnabled ? 1.0f : 0.0f;
    case Q_D: return (calib.baselineLux >= 0.0f) ? calib.baselineLux : 0.0f;
    case Q_P: return getInstantPowerW();
    case Q_T: return restartSeconds;
    case Q_E: return energyJ;
    case Q_VIS: return (restartSeconds > 0.0f) ? (visibilityErrorIntegral / restartSeconds) : 0.0f;
    case Q_FLICK: return (restartSeconds > 0.0f) ? (flickerIntegral / restartSeconds) : 0.0f;
    case Q_REF_HIGH: return highLuxBound;
    case Q_REF_LOW: return lowLuxBound;
    case Q_REF_CURRENT: return currentLuxLowerBound;
    case Q_COST: return energyCost;
    default:
      ok = false;
      return 0.0f;
  }
}

// ============================================================================
// CAN Protocol Decoding On Core 0
// ============================================================================

void handleCommandFrame(uint32_t packetId, uint8_t packetSize, const uint8_t *data) {
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

void processCalibrationPlanFrame(uint8_t packetSize, const uint8_t *data) {
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
    totalNodes = pendingPlan.totalNodes;  // Once both halves arrived, the plan is complete and can be executed locally.
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

void handleQueryFrame(uint32_t packetId, uint8_t packetSize, const uint8_t *data) {
  if (packetSize < 2) {
    reportProtocolError("query_short", packetId, packetSize);
    return;
  }

  uint16_t targetNode = packetId - CAN_ID_QUERY_BASE;  // Node addressing is encoded directly in the CAN identifier.
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

void handleReplyFrame(uint8_t packetSize, const uint8_t *data) {
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
    Serial.println((char)((int)payload.f));
  } else if (code == 'a' || code == 'f') {
    Serial.println((int)payload.f);
  } else if (code == 'u') {
    Serial.println(payload.f, 0);
  } else {
    Serial.println(payload.f, 3);
  }
}

void handleStreamFrame(uint8_t packetSize, const uint8_t *data) {
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

void handleReceivedCanFrame(const CanFrame &frame) {
  if (startupState != STARTUP_READY) {
    return;  // Ignore network traffic until the local node identity is configured from the PC.
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

  reportProtocolError("id_unknown", packetId, packetSize);
}

// Reassembles FIFO words sent by Core 1 into CAN frames or diagnostics, then
// dispatches them into the application-level protocol handlers above.
void serviceFifoFromCore1() {
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
        stage = 1;  // Expect the next two FIFO words to contain the remaining CAN payload bytes.
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

// ============================================================================
// Nonblocking Wake-Up And Calibration State Machines
// ============================================================================

void captureCalibrationSample() {
  if (!calib.active) {
    return;
  }

  if (calib.state == CAL_BASELINE_MEASURE || calib.state == CAL_SLOT_MEASURE) {
    calib.measureAccumulator += filteredLux;  // The control task provides the measurement cadence for calibration averaging.
    calib.measureCount++;
  }
}

void advanceCalibrationAfterMeasure(uint32_t now) {
  float averageLux = (calib.measureCount > 0) ? (calib.measureAccumulator / calib.measureCount) : filteredLux;
  calib.measureAccumulator = 0.0f;
  calib.measureCount = 0;

  if (calib.state == CAL_BASELINE_MEASURE) {
    calib.baselineLux = averageLux;
    calib.currentSlot = 1;
    if (calib.currentSlot > calib.totalNodes) {
      finishCalibration();
      return;
    }
    setLedPwm((nodeId == calib.currentSlot) ? calib.calPwm : 0);  // Only the node assigned to this slot turns its luminaire on.
    enterCalibrationState(CAL_SLOT_SETTLE, now + calib.gapMs + calib.settleMs);
    return;
  }

  if (calib.state == CAL_SLOT_MEASURE) {
    if (calib.currentSlot >= 1 && calib.currentSlot <= MAX_NODES) {
      calib.slotLux[calib.currentSlot] = averageLux;
    }
    setLedPwm(0);
    calib.currentSlot++;
    if (calib.currentSlot > calib.totalNodes) {
      finishCalibration();
      return;
    }
    setLedPwm((nodeId == calib.currentSlot) ? calib.calPwm : 0);
    enterCalibrationState(CAL_SLOT_SETTLE, now + calib.gapMs + calib.settleMs);
  }
}

void serviceCalibrationStateMachine() {
  if (!calib.active) {
    return;
  }

  const uint32_t now = millis();

  switch (calib.state) {
    case CAL_WAIT_START_TIME:
      if (!tick10Reached(calib.scheduledStartTick10)) {
        return;  // Stay responsive to CAN/UI while waiting for this node's locally scheduled start instant.
      }
      setLedPwm(0);
      enterCalibrationState(CAL_BASELINE_SETTLE, now + calib.settleMs);
      break;

    case CAL_BASELINE_SETTLE:
      if (now >= calib.phaseDeadlineMs) {
        enterCalibrationState(CAL_BASELINE_MEASURE, now + calib.measureMs);
      }
      break;

    case CAL_BASELINE_MEASURE:
      if (now >= calib.phaseDeadlineMs) {
        advanceCalibrationAfterMeasure(now);
      }
      break;

    case CAL_SLOT_SETTLE:
      if (now >= calib.phaseDeadlineMs) {
        enterCalibrationState(CAL_SLOT_MEASURE, now + calib.measureMs);
      }
      break;

    case CAL_SLOT_MEASURE:
      if (now >= calib.phaseDeadlineMs) {
        advanceCalibrationAfterMeasure(now);
      }
      break;

    case CAL_FINISHED:
    case CAL_IDLE:
    default:
      break;
  }
}

// The wake-up flow is time-driven rather than blocking: discovery stays open
// for a fixed window, then the coordinator may publish a future calibration
// plan that every node executes independently.
void serviceWakeupStateMachine() {
  if (startupState != STARTUP_READY) {
    return;
  }

  const uint32_t now = millis();
  switch (wakeupState) {
    case WAKEUP_BOOT:
      resetRuntimeState();
      break;

    case WAKEUP_DISCOVERY_OPEN:
      if (now - lastWakeupMs >= WAKEUP_RETRY_MS) {
        lastWakeupMs = now;
        sendHelloFrame();
        sendSimpleCommand(BROADCAST_NODE, CMD_ANNOUNCE);  // Periodic announce helps late peers discover this node.
      }
      if (now - bootMs >= WAKEUP_WINDOW_MS) {
        wakeupState = WAKEUP_DISCOVERY_STABLE;
      }
      break;

    case WAKEUP_DISCOVERY_STABLE:
      if (isCoordinator && !autoCalibrationTriggered && (now - bootMs >= WAKEUP_WINDOW_MS + 500)) {
        autoCalibrationTriggered = true;
        uint16_t sessionId = (uint16_t)(millis() & 0xFFFF);
        uint16_t startDelayTick10 = encodeCalTick(DEFAULT_START_DELAY_MS);  // Relative start delay applied locally on each node.
        scheduleCalibrationPlanBroadcast(sessionId,
                                         DEFAULT_CAL_PWM,
                                         totalNodes,
                                         DEFAULT_SETTLE_MS,
                                         DEFAULT_MEASURE_MS,
                                         DEFAULT_GAP_MS,
                                         startDelayTick10);
        startCalibrationSession(sessionId,
                                DEFAULT_CAL_PWM,
                                DEFAULT_SETTLE_MS,
                                DEFAULT_MEASURE_MS,
                                DEFAULT_GAP_MS,
                                (uint16_t)(nowTick10() + startDelayTick10),
                                totalNodes);
      }
      wakeupState = WAKEUP_RUN;
      break;

    case WAKEUP_RUN:
    default:
      break;
  }
}

// ============================================================================
// Periodic Control Task And Diagnostics
// ============================================================================

void servicePeriodicHello() {
  if (startupState != STARTUP_READY) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastHelloMs >= HELLO_PERIOD_MS) {
    lastHelloMs = now;
    sendHelloFrame();
    removeStalePeers();
  }
}

void serviceCalibrationPlanBroadcast() {
  if (!calibBroadcast.active || startupState != STARTUP_READY) {
    return;
  }

  const uint32_t now = millis();
  if (now < calibBroadcast.nextSendMs) {
    return;
  }

  broadcastCalibrationPlan(calibBroadcast.sessionId,
                           calibBroadcast.calPwm,
                           calibBroadcast.totalNodes,
                           calibBroadcast.settleMs,
                           calibBroadcast.measureMs,
                           calibBroadcast.gapMs,
                           calibBroadcast.startDelayTick10);

  if (calibBroadcast.repeatsRemaining > 0) {
    calibBroadcast.repeatsRemaining--;
  }

  if (calibBroadcast.repeatsRemaining == 0) {
    calibBroadcast.active = false;
  } else {
    calibBroadcast.nextSendMs = now + CAL_PLAN_REPEAT_INTERVAL_MS;
  }
}

void serviceStreamingOutput() {
  if (!streamState.active || startupState != STARTUP_READY) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastStreamMs < STREAM_PERIOD_MS) {
    return;
  }

  lastStreamMs = now;
  Serial.print("s ");
  Serial.print(streamState.variable);
  Serial.print(" ");
  Serial.print(nodeId);
  Serial.print(" ");
  if (streamState.variable == 'y') {
    Serial.print(filteredLux, 3);
  } else {
    Serial.print(localPwm);
  }
  Serial.print(" ");
  Serial.println(now);
}

// Executes exactly one sampled control update whenever the timer callback has
// marked a period as due, and records execution time for overrun detection.
void runControlStep() {
  if (startupState != STARTUP_READY) {
    return;
  }

  const uint32_t startUs = micros();  // Measure execution time to detect missed deadlines.
  const float prevLux = filteredLux;

  readLuxFiltered();
  restartSeconds = (millis() - bootMs) / 1000.0f;
  energyJ += getInstantPowerW() * (CONTROL_PERIOD_MS / 1000.0f);
  visibilityErrorIntegral += fabsf(refLux - filteredLux) * (CONTROL_PERIOD_MS / 1000.0f);
  flickerIntegral += fabsf(filteredLux - prevLux);
  pushHistory(filteredLux, localPwm);
  captureCalibrationSample();  // Reuse the periodic control cadence for calibration sampling.

  controlLastExecUs = micros() - startUs;
  if (controlLastExecUs > controlMaxExecUs) {
    controlMaxExecUs = controlLastExecUs;
  }

  if (controlLastExecUs > (CONTROL_PERIOD_MS * 1000UL)) {
    controlOverrunCount++;
    Serial.print("ctrl_overrun_us ");
    Serial.println(controlLastExecUs);
  }
}

void serviceControlTask() {
  if (controlDueCount == 0) {
    return;
  }

  noInterrupts();
  uint8_t pending = controlDueCount;
  if (pending > 0) {
    controlDueCount--;  // Consume one scheduled control slot atomically with respect to the timer callback.
  }
  interrupts();

  if (pending > 1) {
    controlOverrunCount++;
  }

  runControlStep();
}

void serviceDiagnostics() {
  const uint32_t now = millis();
  if (now - lastDiagMs < DIAG_PERIOD_MS) {
    return;
  }

  lastDiagMs = now;
  if (pendingRemoteQuery.active && (now - pendingRemoteQuery.requestedAtMs > 1000)) {
    Serial.print("can_err query_timeout node=");
    Serial.print(pendingRemoteQuery.targetNode);
    Serial.print(" code=");
    Serial.println((char)pendingRemoteQuery.code);
    pendingRemoteQuery.active = false;
  }
}

// ============================================================================
// User Interface Parsing On Core 0
// ============================================================================

bool parseGetCommand(const char *line) {
  char a = 0;
  char b = 0;
  int i = 0;

  if (sscanf(line, "g %c %d", &a, &i) == 2) {
    if (i < 0 || i > MAX_NODES) {
      printErr();
      return true;
    }

    if (i == nodeId) {
      if (a == 'o') {
        Serial.print(a);
        Serial.print(" ");
        Serial.print(i);
        Serial.print(" ");
        Serial.println(occupancyState);
        return true;
      }

      bool ok = false;
      float value = readQueryValue((QueryCode)a, ok);
      if (!ok) {
        printErr();
        return true;
      }
      Serial.print(a);
      Serial.print(" ");
      Serial.print(i);
      Serial.print(" ");
      if (a == 'u') {
        Serial.println(value, 0);
      } else if (a == 'a' || a == 'f') {
        Serial.println((int)value);
      } else {
        Serial.println(value, 3);
      }
      return true;
    }

    if (a == 'u' || a == 'r' || a == 'y') {
      PeerStatus *peer = findPeer(i);
      if (!peer) {
        printErr();
        return true;
      }
      Serial.print(a);
      Serial.print(" ");
      Serial.print(i);
      Serial.print(" ");
      if (a == 'u') {
        Serial.println(peer->pwm);
      } else if (a == 'r') {
        Serial.println(peer->refLux, 3);
      } else {
        Serial.println(peer->lux, 3);
      }
      return true;
    }

    sendQuery((uint8_t)i, (QueryCode)a);
    return true;
  }

  if (sscanf(line, "g %c %c %d", &a, &b, &i) == 3) {
    if (a == 'b' && (b == 'y' || b == 'u') && i == nodeId) {
      printBuffer(b);
      return true;
    }
    printErr();
    return true;
  }

  return false;
}

void handleStartupLine(const char *line) {
  int value = atoi(line);  // Startup input is intentionally simple: one numeric line at a time.
  if (startupState == STARTUP_WAIT_NODE_ID) {
    if (value < 1 || value > MAX_NODES) {
      Serial.println("Valor invalido. Tenta outra vez.");
      return;
    }
    nodeId = (uint8_t)value;
    startupState = STARTUP_WAIT_TOTAL_NODES;
    Serial.print("ID configurado: ");
    Serial.println(nodeId);
    Serial.println("Introduz o numero total de nos (1-8):");
    return;
  }

  if (startupState == STARTUP_WAIT_TOTAL_NODES) {
    if (value < 1 || value > MAX_NODES) {
      Serial.println("Valor invalido. Tenta outra vez.");
      return;
    }
    totalNodes = (uint8_t)value;
    isCoordinator = (nodeId == 1);
    startupState = STARTUP_READY;
    updateCurrentLowerBound();
    resetRuntimeState();
    Serial.print("Nos totais configurados: ");
    Serial.println(totalNodes);
  }
}

void handleLineCommand(const char *line) {
  if (line[0] == '\0') {
    return;
  }

  if (startupState != STARTUP_READY) {
    handleStartupLine(line);
    return;
  }

  if (strcmp(line, "R") == 0) {
    sendSimpleCommand(BROADCAST_NODE, CMD_RESTART);
    resetRuntimeState();
    printAck();
    return;
  }

  if (parseGetCommand(line)) {
    return;
  }

  char c1 = 0;
  int i = 0;
  float val = 0.0f;
  int node = 0;
  int intVal = 0;

  if (sscanf(line, "u %d %f", &i, &val) == 2) {
    if (i == 0) {
      sendSimpleCommand(BROADCAST_NODE, CMD_LED_SET_PWM, (uint16_t)val);
      setLedPwm((uint16_t)val);
      printAck();
      return;
    }
    if (i == nodeId) {
      setLedPwm((uint16_t)val);
      printAck();
      return;
    }
    sendSimpleCommand((uint8_t)i, CMD_LED_SET_PWM, (uint16_t)val);
    printAck();
    return;
  }

  if (sscanf(line, "r %d %f", &i, &val) == 2) {
    if (i == 0) {
      uint16_t refEnc = (uint16_t)(val * 100.0f);
      sendSimpleCommand(BROADCAST_NODE, CMD_SET_REF, refEnc);
      refLux = val;
      printAck();
      return;
    }
    if (i == nodeId) {
      refLux = val;
      printAck();
      return;
    }
    sendSimpleCommand((uint8_t)i, CMD_SET_REF, (uint16_t)(val * 100.0f));
    printAck();
    return;
  }

  if (sscanf(line, "o %d %c", &i, &c1) == 2) {
    if (c1 != 'o' && c1 != 'l' && c1 != 'h') {
      printErr();
      return;
    }
    if (i == 0) {
      sendCharCommand(BROADCAST_NODE, CMD_SET_OCCUPANCY, c1);
      occupancyState = c1;
      updateCurrentLowerBound();
      printAck();
      return;
    }
    if (i == nodeId) {
      occupancyState = c1;
      updateCurrentLowerBound();
      printAck();
      return;
    }
    sendCharCommand((uint8_t)i, CMD_SET_OCCUPANCY, c1);
    printAck();
    return;
  }

  if (sscanf(line, "a %d %d", &node, &intVal) == 2) {
    if (node == 0) {
      sendSimpleCommand(BROADCAST_NODE, CMD_SET_ANTI_WINDUP, (uint16_t)(intVal != 0));
      antiWindupEnabled = (intVal != 0);
      printAck();
      return;
    }
    if (node == nodeId) {
      antiWindupEnabled = (intVal != 0);
      printAck();
      return;
    }
    sendSimpleCommand((uint8_t)node, CMD_SET_ANTI_WINDUP, (uint16_t)(intVal != 0));
    printAck();
    return;
  }

  if (sscanf(line, "f %d %d", &node, &intVal) == 2) {
    if (node == 0) {
      sendSimpleCommand(BROADCAST_NODE, CMD_SET_FEEDBACK, (uint16_t)(intVal != 0));
      feedbackEnabled = (intVal != 0);
      printAck();
      return;
    }
    if (node == nodeId) {
      feedbackEnabled = (intVal != 0);
      printAck();
      return;
    }
    sendSimpleCommand((uint8_t)node, CMD_SET_FEEDBACK, (uint16_t)(intVal != 0));
    printAck();
    return;
  }

  if (sscanf(line, "O %d %f", &node, &val) == 2) {
    if (node == 0) {
      uint16_t encoded = (uint16_t)(val * 100.0f);
      sendSimpleCommand(BROADCAST_NODE, CMD_SET_HIGH_BOUND, encoded);
      highLuxBound = val;
      updateCurrentLowerBound();
      printAck();
      return;
    }
    if (node == nodeId) {
      highLuxBound = val;
      updateCurrentLowerBound();
      printAck();
      return;
    }
    sendSimpleCommand((uint8_t)node, CMD_SET_HIGH_BOUND, (uint16_t)(val * 100.0f));
    printAck();
    return;
  }

  if (sscanf(line, "U %d %f", &node, &val) == 2) {
    if (node == 0) {
      uint16_t encoded = (uint16_t)(val * 100.0f);
      sendSimpleCommand(BROADCAST_NODE, CMD_SET_LOW_BOUND, encoded);
      lowLuxBound = val;
      updateCurrentLowerBound();
      printAck();
      return;
    }
    if (node == nodeId) {
      lowLuxBound = val;
      updateCurrentLowerBound();
      printAck();
      return;
    }
    sendSimpleCommand((uint8_t)node, CMD_SET_LOW_BOUND, (uint16_t)(val * 100.0f));
    printAck();
    return;
  }

  if (sscanf(line, "C %d %f", &node, &val) == 2) {
    if (node == 0) {
      uint16_t encoded = (uint16_t)(val * 100.0f);
      sendSimpleCommand(BROADCAST_NODE, CMD_SET_COST, encoded);
      energyCost = val;
      printAck();
      return;
    }
    if (node == nodeId) {
      energyCost = val;
      printAck();
      return;
    }
    sendSimpleCommand((uint8_t)node, CMD_SET_COST, (uint16_t)(val * 100.0f));
    printAck();
    return;
  }

  if (sscanf(line, "s %c %d", &c1, &node) == 2) {
    if (c1 != 'y' && c1 != 'u') {
      printErr();
      return;
    }
    if (node == 0) {
      sendCharCommand(BROADCAST_NODE, CMD_STREAM_START, c1);
      streamState.active = true;
      streamState.variable = c1;
      printAck();
      return;
    }
    if (node == nodeId) {
      streamState.active = true;
      streamState.variable = c1;
      printAck();
      return;
    }
    sendCharCommand((uint8_t)node, CMD_STREAM_START, c1);
    printAck();
    return;
  }

  if (sscanf(line, "S %c %d", &c1, &node) == 2) {
    if (c1 != 'y' && c1 != 'u') {
      printErr();
      return;
    }
    if (node == 0) {
      sendCharCommand(BROADCAST_NODE, CMD_STREAM_STOP, c1);
      streamState.active = false;
      streamState.variable = 0;
      printAck();
      return;
    }
    if (node == nodeId) {
      streamState.active = false;
      streamState.variable = 0;
      printAck();
      return;
    }
    sendCharCommand((uint8_t)node, CMD_STREAM_STOP, c1);
    printAck();
    return;
  }

  if (sscanf(line, "c %d", &intVal) == 1) {
    if (!isCoordinator) {
      printErr();
      return;
    }
    uint16_t pwm = (intVal > 0) ? (uint16_t)intVal : DEFAULT_CAL_PWM;
    uint16_t sessionId = (uint16_t)(millis() & 0xFFFF);
    uint16_t startDelayTick10 = encodeCalTick(DEFAULT_START_DELAY_MS);
    scheduleCalibrationPlanBroadcast(sessionId,
                                     pwm,
                                     totalNodes,
                                     DEFAULT_SETTLE_MS,
                                     DEFAULT_MEASURE_MS,
                                     DEFAULT_GAP_MS,
                                     startDelayTick10);
    startCalibrationSession(sessionId,
                            pwm,
                            DEFAULT_SETTLE_MS,
                            DEFAULT_MEASURE_MS,
                            DEFAULT_GAP_MS,
                            (uint16_t)(nowTick10() + startDelayTick10),
                            totalNodes);
    printAck();
    return;
  }

  if (strcmp(line, "rpt") == 0) {
    printCalibrationReport();
    return;
  }

  if (strcmp(line, "g diag") == 0) {
    Serial.print("diag txq=");
    Serial.print(txQueueCount);
    Serial.print(" tx_err=");
    Serial.print(canTxErrorCount);
    Serial.print(" rx_err=");
    Serial.print(canRxErrorCount);
    Serial.print(" proto_err=");
    Serial.print(canProtocolErrorCount);
    Serial.print(" ctrl_overrun=");
    Serial.print(controlOverrunCount);
    Serial.print(" ctrl_last_us=");
    Serial.print(controlLastExecUs);
    Serial.print(" ctrl_max_us=");
    Serial.println(controlMaxExecUs);
    return;
  }

  printErr();
}

// Serial parsing is nonblocking: characters are accumulated until newline and
// then parsed once, allowing CAN, control, and calibration to keep running.
void serviceSerialNonblocking() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      serialBuffer[serialLength] = '\0';
      handleLineCommand(serialBuffer);  // Parse only when a full line is available.
      serialLength = 0;
      continue;
    }

    if (serialLength < (SERIAL_BUFFER_LEN - 1)) {
      serialBuffer[serialLength++] = c;
    } else {
      serialLength = 0;
      Serial.println("err cmd too long");
    }
  }
}

// ============================================================================
// Core 0 Entry Points
// ============================================================================

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogWriteResolution(12);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LDR_PIN, INPUT);
  setLedPwm(0);
  filteredLux = readLuxFiltered();
  updateCurrentLowerBound();

  Serial.println("Introduz o ID do no (1-8):");
  add_repeating_timer_ms(-(int32_t)CONTROL_PERIOD_MS, controlTimerCallback, nullptr, &controlTimer);
}

void loop() {
  serviceSerialNonblocking();
  serviceFifoFromCore1();
  serviceCore0ToCore1Fifo();
  serviceWakeupStateMachine();
  serviceCalibrationStateMachine();
  serviceCalibrationPlanBroadcast();
  serviceControlTask();
  servicePeriodicHello();
  serviceStreamingOutput();
  serviceDiagnostics();
}

// ============================================================================
// Core 1 CAN Service
// ============================================================================

// Drains the MCP2515 receive buffers as soon as possible after an interrupt so
// RX overflows are less likely on a busy CAN bus.
void drainCanRxCore1() {
  while (true) {
    const uint8_t rxStatus = canBus.checkReceive();
    if (rxStatus == CAN_NOMSG) {
      break;
    }
    if (rxStatus != CAN_MSGAVAIL) {
      queueCore1Diag(DIAG_CAN_CHECK, rxStatus, 0);
      break;
    }

    unsigned long rawId = 0;
    uint8_t length = 0;
    uint8_t payload[8] = {0};
    const uint8_t readStatus = canBus.readMsgBuf(&rawId, &length, payload);  // Core 1 is the sole owner of the MCP2515 driver.
    if (readStatus != CAN_OK) {
      queueCore1Diag(DIAG_CAN_READ, readStatus, 0);
      break;
    }

    CanFrame frame;
    frame.id = (uint16_t)rawId;
    frame.len = length;
    memcpy(frame.data, payload, sizeof(frame.data));
    if (!enqueueCore1EventFrame(frame)) {
      queueCore1Diag(DIAG_CAN_FIFO_DROP, length, 0);
    }
  }
}

// Periodically polls controller health so CAN errors are surfaced even when no
// new frame traffic is arriving.
void serviceCore1Health() {
  const uint32_t now = millis();
  if (now - core1LastHealthMs < CORE1_HEALTH_PERIOD_MS) {
    return;
  }

  core1LastHealthMs = now;
  const uint8_t status = canBus.checkError();
  if (status != CAN_OK) {
    queueCore1Diag(DIAG_CAN_HEALTH, status, 0);
  }
}

// Reassembles transmit requests sent by Core 0 and performs the actual
// `sendMsgBuf` call on the core that owns the CAN controller.
void serviceCore1IncomingFifo() {
  static uint8_t stage = 0;
  static uint32_t words[3] = {0};
  static FifoMessageKind kind = FIFO_TX_FRAME;

  uint32_t word = 0;
  while (rp2040.fifo.pop_nb(&word)) {
    if (stage == 0) {
      kind = (FifoMessageKind)((word >> 24) & 0xFF);
      words[0] = word;
      if (kind == FIFO_DIAG) {
        continue;
      }
      stage = 1;
      continue;
    }

    words[stage] = word;
    stage++;
    if (stage == 3) {
      stage = 0;
      if (kind != FIFO_TX_FRAME) {
        continue;
      }

      CanFrame frame;
      frame.id = (uint16_t)((words[0] >> 8) & 0xFFFF);
      frame.len = (uint8_t)(words[0] & 0xFF);
      unpackBytes32(words[1], frame.data);
      unpackBytes32(words[2], frame.data + 4);
      if (frame.len > 8) {
        queueCore1Diag(DIAG_CAN_FIFO_DROP, frame.len, 1);
        continue;
      }

      const uint8_t sendStatus = canBus.sendMsgBuf(frame.id, 0, frame.len, frame.data);
      if (sendStatus != CAN_OK) {
        queueCore1Diag(DIAG_CAN_SEND, sendStatus, frame.len);  // Report the failure back to Core 0 for diagnostics.
      }
    }
  }
}

// Core 1 initializes the MCP2515 and becomes the sole owner of the CAN
// peripheral for the lifetime of the program.
void setup1() {
  SPI.setRX(16);
  SPI.setCS(CAN_CS_PIN);
  SPI.setSCK(18);
  SPI.setTX(19);
  SPI.begin();
  pinMode(CAN_INT_PIN, INPUT_PULLUP);

  const uint8_t beginStatus = canBus.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
  if (beginStatus != CAN_OK) {
    queueCore1Diag(DIAG_CAN_INIT, beginStatus, 0);
    return;
  }

  const uint8_t modeStatus = canBus.setMode(MCP_NORMAL);
  if (modeStatus != CAN_OK) {
    queueCore1Diag(DIAG_CAN_MODE, modeStatus, 0);
    return;
  }

  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), canIrqHandler, FALLING);
}

// Core 1 continuously services outbound FIFO traffic, RX interrupts, and CAN
// health checks without calling back into Core 0 state directly.
void loop1() {
  serviceCore1IncomingFifo();

  if (canIrqPending || digitalRead(CAN_INT_PIN) == LOW) {
    canIrqPending = false;
    drainCanRxCore1();
  }

  serviceCore1Health();
  serviceCore1ToCore0Fifo();
}
