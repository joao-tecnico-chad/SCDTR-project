// ============================================================================
// CAN_ORDER_modular.ino
// Distributed Illuminance Controller for RP2040 + MCP2515
//
// Multi-node luminaire control system using CAN bus communication.
// Supports three distributed optimisation algorithms (Consensus, ADMM,
// Dual Decomposition) and automatic multi-node calibration.
//
// Architecture:
//   Core 0 - control loop, serial UI, protocol logic, calibration FSM
//   Core 1 - sole owner of MCP2515 SPI driver, CAN TX/RX
//   Inter-core communication via RP2040 hardware FIFO
//
// Module layout (all in this folder):
//   hardware_config.h  - Pin/ADC/timing/CAN constants
//   protocol_defs.h    - Enums, structs, extern declarations, forward decls
//   sensing.h          - LDR reading, LED output, peer management
//   can_transport.h    - FIFO helpers, TX/RX queues, CAN message builders
//   calibration.h      - Calibration session FSM, wakeup FSM, plan broadcast
//   consensus.h        - Consensus algorithm + gain exchange
//   admm.h             - ADMM algorithm
//   dual_decomp.h      - Dual decomposition algorithm
//   can_protocol.h     - CAN frame dispatch and decoding
//   control.h          - PI controller, hello, streaming, diagnostics
//   serial_ui.h        - Serial command parser, startup, state reset
//
// Authors: Joao Rocha, Ricardo Gaspar, Diogo Costa — SCDTR 2025/2026
//
// Serial Command Quick Reference (type "help" at runtime for full list):
//   u <n> <pwm>     Set LED PWM            r <n> <lux>   Set lux reference
//   o <n> <l/h>     Set occupancy           C <n> <cost>  Set energy cost
//   O <n> <lux>     HIGH bound              U <n> <lux>   LOW bound
//   a <n> <0/1>     Anti-windup             f <n> <0/1>   Feedback on/off
//   g <var> <n>     Get value (y/u/r/E/V/F/o/d/p/t/O/U/L/C)
//   s <y/u> <n>     Start streaming         S <y/u> <n>   Stop streaming
//   A <0-3>         Algorithm (PI/Cons/ADMM/DD)
//   R               Restart all             RM            Reset metrics
//   c               Calibrate               rpt           Calibration report
//   help / h / ?    Show help
// ============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <pico/time.h>

// Include all project headers (order matters for dependency resolution)
#include "hardware_config.h"
#include "protocol_defs.h"
#include "sensing.h"
#include "can_transport.h"
#include "calibration.h"
#include "consensus.h"
#include "admm.h"
#include "dual_decomp.h"
#include "can_protocol.h"
#include "control.h"
#include "serial_ui.h"

// ============================================================================
// Global Variable Definitions (declared extern in protocol_defs.h)
// ============================================================================

MCP_CAN canBus(CAN_CS_PIN);
repeating_timer_t controlTimer;

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

float B_PARAM = B_PARAM_DEFAULT;
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
uint32_t metricSampleCount = 0;

// PI Controller state
float piIntegral = 0.0f;
float piPrevError = 0.0f;
float piPrevDuty = 0.0f;
float piPrevPrevDuty = 0.0f;

ConsensusState cons;
ADMMState admm;
DualDecompState dd;

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
// Core 0 Entry Points
// ============================================================================

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogWriteResolution(12);
  analogWriteFreq(45000);  // 45 kHz PWM -- must be >10x the LDR RC filter cutoff (~160 Hz)
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
  serviceConsensus();
  serviceADMM();
  serviceDualDecomp();
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
    const uint8_t readStatus = canBus.readMsgBuf(&rawId, &length, payload);
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
        queueCore1Diag(DIAG_CAN_SEND, sendStatus, frame.len);
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

  const uint8_t beginStatus = canBus.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ);
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
  yield();  // Allow USB stack to process reset requests
}
