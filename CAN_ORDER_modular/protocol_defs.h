// ============================================================================
// protocol_defs.h
// All enumerations, data structures, and extern declarations of global state
//
// This header is the "data dictionary" for the entire firmware. Every other
// module includes it to access shared types and extern-declared runtime
// variables. The actual variable definitions live in CAN_ORDER_modular.ino.
//
// Authors: Joao Rocha, SCDTR course project
// ============================================================================
#pragma once

#include "hardware_config.h"
#include <mcp_can.h>
#include <pico/time.h>

// ----- CAN command / calibration / query enumerations -----

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

enum AlgorithmMode : uint8_t {
  ALG_NONE = 0,
  ALG_CONSENSUS = 1,
  ALG_ADMM = 2,
  ALG_DUAL_DECOMP = 3
};

// ----- Data structures -----

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
  uint32_t startMs = 0;
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

// ----- Consensus state (shared by ADMM and Dual Decomp via cons.K, cons.o, etc.) -----

struct ConsensusState {
  AlgorithmMode mode = ALG_NONE;
  bool active = false;
  bool converged = false;
  bool gainsExchanged = false;
  int iteration = 0;
  int numNodes = 0;
  int myIndex = 0;  // 0-based

  float K[CONS_MAX_NODES][CONS_MAX_NODES];
  float o[CONS_MAX_NODES];
  float L_ref[CONS_MAX_NODES];
  float c[CONS_MAX_NODES];

  float d[CONS_MAX_NODES];
  float d_avg[CONS_MAX_NODES];
  float d_others[CONS_MAX_NODES][CONS_MAX_NODES];
  bool received[CONS_MAX_NODES];

  uint32_t lastIterMs = 0;
  uint8_t iterByte = 0;

  // Gain exchange tracking
  bool gainRowReceived[CONS_MAX_NODES];
  uint32_t gainExchStartMs = 0;
};

// ----- ADMM state -----

struct ADMMState {
  float d[CONS_MAX_NODES];
  float z[CONS_MAX_NODES];
  float u[CONS_MAX_NODES];
  float d_others[CONS_MAX_NODES];  // d values from other nodes
  bool received[CONS_MAX_NODES];
  int iteration;
  bool converged;
  bool active;
  uint32_t lastIterMs;
};

// ----- Dual decomposition state -----

struct DualDecompState {
  float d[CONS_MAX_NODES];
  float lambda[CONS_MAX_NODES];
  float d_others[CONS_MAX_NODES];
  bool received[CONS_MAX_NODES];
  float alpha;
  int iteration;
  bool converged;
  bool active;
  uint32_t lastIterMs;
};

// ============================================================================
// Extern declarations of global variables (defined in CAN_ORDER_modular.ino)
// ============================================================================

extern MCP_CAN canBus;
extern repeating_timer_t controlTimer;

extern PeerStatus peers[MAX_NODES];
extern StreamState streamState;
extern CalibrationPlanBuffer pendingPlan;
extern CalibrationContext calib;
extern PendingRemoteQuery pendingRemoteQuery;
extern CalibrationBroadcastState calibBroadcast;

extern CanFrame txQueue[TX_QUEUE_LEN];
extern uint8_t txQueueHead;
extern uint8_t txQueueTail;
extern uint8_t txQueueCount;
extern FifoTxStaging core0ToCore1Staging;

extern uint8_t nodeId;
extern uint8_t totalNodes;
extern bool isCoordinator;
extern StartupState startupState;
extern WakeupState wakeupState;
extern bool autoCalibrationTriggered;

extern float B_PARAM;
extern float filteredLux;
extern float lastLdrVoltage;
extern uint16_t localPwm;
extern float refLux;
extern char occupancyState;
extern bool antiWindupEnabled;
extern bool feedbackEnabled;
extern float highLuxBound;
extern float lowLuxBound;
extern float currentLuxLowerBound;
extern float energyCost;

extern float energyJ;
extern float visibilityErrorIntegral;
extern float flickerIntegral;
extern float restartSeconds;
extern uint32_t metricSampleCount;

extern float piIntegral;
extern float piPrevError;
extern float piPrevDuty;
extern float piPrevPrevDuty;

extern ConsensusState cons;
extern ADMMState admm;
extern DualDecompState dd;

extern uint16_t yHistory[HISTORY_LEN];
extern uint16_t uHistory[HISTORY_LEN];
extern int historyIndex;
extern bool historyWrapped;

extern char serialBuffer[SERIAL_BUFFER_LEN];
extern size_t serialLength;

extern uint32_t bootMs;
extern uint32_t lastHelloMs;
extern uint32_t lastWakeupMs;
extern uint32_t lastStreamMs;
extern uint32_t lastDiagMs;

extern uint32_t canTxErrorCount;
extern uint32_t canRxErrorCount;
extern uint32_t canProtocolErrorCount;
extern uint32_t controlOverrunCount;
extern uint32_t controlMaxExecUs;
extern uint32_t controlLastExecUs;

extern volatile uint8_t controlDueCount;
extern volatile bool canIrqPending;

extern CanFrame core1EventQueue[CORE1_EVENT_QUEUE_LEN];
extern uint8_t core1EventHead;
extern uint8_t core1EventTail;
extern uint8_t core1EventCount;
extern FifoTxStaging core1ToCore0Staging;
extern uint32_t core1LastHealthMs;

// ============================================================================
// Forward declarations of functions used across modules
// ============================================================================

// can_transport.h
bool enqueueTxFrame(uint16_t canId, const uint8_t *payload, uint8_t length);

// sensing.h
float readLuxFiltered();
float getInstantPowerW();
void setLedPwm(uint16_t pwm);
void pushHistory(float lux, uint16_t pwm);
void updateCurrentLowerBound();
void updatePeer(uint8_t senderId, uint16_t pwm, float lux, float peerRef);
PeerStatus *findPeer(uint8_t id);
void removeStalePeers();

// can_transport.h
void sendHelloFrame();
void sendSimpleCommand(uint8_t targetNode, CommandType command, uint16_t value = 0);
void sendQuery(uint8_t targetNode, QueryCode queryCode);
void sendReply(uint8_t requesterNode, QueryCode queryCode, float value);
void sendCharCommand(uint8_t targetNode, CommandType command, char value);
void sendStreamFrame(char variable);
uint8_t encodeCalTick(uint16_t msValue);
uint16_t nowTick10();
bool tick10Reached(uint16_t targetTick);

// calibration.h
void broadcastCalibrationPlan(uint16_t sessionId, uint16_t calPwm, uint8_t planNodes,
                              uint16_t settleMs, uint16_t measureMs, uint16_t gapMs,
                              uint16_t startDelayTick10);
void scheduleCalibrationPlanBroadcast(uint16_t sessionId, uint16_t calPwm, uint8_t planNodes,
                                      uint16_t settleMs, uint16_t measureMs, uint16_t gapMs,
                                      uint16_t startDelayTick10);
void startCalibrationSession(uint16_t sessionId, uint16_t calPwm, uint16_t settleMs,
                             uint16_t measureMs, uint16_t gapMs,
                             uint16_t scheduledStartTick10, uint8_t planNodes);
void captureCalibrationSample();
void serviceCalibrationStateMachine();
void serviceWakeupStateMachine();
void serviceCalibrationPlanBroadcast();
void finishCalibration();

// consensus.h
void initConsensus();
void sendGainRow();
void serviceConsensus();
void handleConsensusFrame(uint8_t len, const uint8_t *data);
void handleGainExchangeFrame(uint8_t len, const uint8_t *data);
bool allGainsReceived();

// admm.h
void initADMM();
void serviceADMM();
void handleADMMFrame(uint8_t len, const uint8_t *data);

// dual_decomp.h
void initDualDecomp();
void serviceDualDecomp();
void handleDualDecompFrame(uint8_t len, const uint8_t *data);

// serial_ui.h
void resetRuntimeState();
void printCalibrationReport();
float readQueryValue(QueryCode queryCode, bool &ok);
void handleLineCommand(const char *line);
void serviceSerialNonblocking();

// control.h
void runControlStep();
void serviceControlTask();
void servicePeriodicHello();
void serviceStreamingOutput();
void serviceDiagnostics();

// can_transport.h (FIFO helpers)
uint32_t packFrameHeader(FifoMessageKind kind, uint16_t canId, uint8_t length);
uint32_t packBytes32(const uint8_t *bytes);
void unpackBytes32(uint32_t word, uint8_t *bytes);
uint32_t packDiagWord(Core1DiagStage stage, uint8_t code, uint8_t detail);
void serviceCore0ToCore1Fifo();
bool dequeueTxFrame(CanFrame &frame);
void prepareFifoFrame(FifoTxStaging &staging, FifoMessageKind kind, const CanFrame &frame);
void prepareFifoDiag(FifoTxStaging &staging, Core1DiagStage stage, uint8_t code, uint8_t detail);
bool enqueueCore1EventFrame(const CanFrame &frame);
bool dequeueCore1EventFrame(CanFrame &frame);
void queueCore1Diag(Core1DiagStage stage, uint8_t code, uint8_t detail);
void serviceCore1ToCore0Fifo();

// can_protocol.h
void handleReceivedCanFrame(const CanFrame &frame);
void serviceFifoFromCore1();

// Utility helpers
void printAck();
void printErr();
void reportProtocolError(const char *stage, uint32_t canId, uint8_t length);
void reportCore1Diag(Core1DiagStage stage, uint8_t code, uint8_t detail);
