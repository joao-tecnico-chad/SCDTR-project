// ****************************************************************************
// *                                                                          *
// *   Distributed Cooperative Illumination Control System                    *
// *                                                                          *
// *   Authors:  Joao Rocha   (ist1106509)                                   *
// *             Ricardo Gaspar (ist1104180)                                  *
// *             Diogo Costa                                                  *
// *                                                                          *
// *   Course:   SCDTR - Distributed Real-Time Control Systems               *
// *             Instituto Superior Tecnico, 2025/2026                        *
// *                                                                          *
// ****************************************************************************
//
// DESCRIPTION
// -----------
// Firmware for an RP2040 Pico-based luminaire node that participates in a
// distributed cooperative lighting network.  Each node:
//   - Measures ambient illuminance via an LDR sensor
//   - Drives an LED through 12-bit PWM
//   - Communicates with peers over a CAN bus (MCP2515 controller)
//   - Runs a closed-loop PI controller with feedforward and anti-windup
//   - Performs cooperative distributed optimisation (Consensus ADMM, ADMM,
//     or Dual Decomposition) to minimise energy while meeting illuminance
//     constraints across all nodes
//   - Executes a coordinated multi-node calibration procedure to estimate
//     the static gain matrix K
//
// The RP2040 dual-core architecture is exploited:
//   Core 0  -  Application logic: serial UI, control loop, calibration
//              state machine, distributed algorithms, CAN protocol decode
//   Core 1  -  CAN driver: sole owner of the MCP2515 SPI peripheral,
//              handles TX/RX and health monitoring
//
// Inter-core communication uses the RP2040 hardware FIFO (32-bit words).
//
// ============================================================================
//  TABLE OF CONTENTS  (approximate line numbers after commenting)
// ============================================================================
//
//   1.  HARDWARE CONFIGURATION & CONSTANTS .................. ~line  113
//   2.  PROTOCOL ENUMS, STRUCTS & TYPE DEFINITIONS ......... ~line  180
//   3.  GLOBAL RUNTIME STATE (Core 0 owned) ................ ~line  368
//   4.  DISTRIBUTED CONSENSUS ALGORITHM .................... ~line  432
//   5.  ADMM ALGORITHM ..................................... ~line  769
//   6.  DUAL DECOMPOSITION ALGORITHM ....................... ~line  946
//   7.  HISTORY BUFFERS & TIMING STATE ..................... ~line 1083
//   8.  INTERRUPT & FIFO PACKING HELPERS ................... ~line 1126
//   9.  LOCAL SENSING & ACTUATION HELPERS .................. ~line 1231
//  10.  CORE 0 -> CORE 1 TX QUEUE & FIFO TRANSPORT ........ ~line 1334
//  11.  CORE 1 -> CORE 0 RX QUEUE & FIFO TRANSPORT ........ ~line 1408
//  12.  HIGH-LEVEL CAN MESSAGE BUILDERS .................... ~line 1464
//  13.  CALIBRATION SESSION MANAGEMENT ..................... ~line 1562
//  14.  STATE RESET & REPORTING UTILITIES .................. ~line 1699
//  15.  CAN PROTOCOL DECODING (Core 0) ..................... ~line 1812
//  16.  FIFO REASSEMBLY FROM CORE 1 ....................... ~line 2153
//  17.  WAKEUP & CALIBRATION STATE MACHINES ................ ~line 2203
//  18.  PERIODIC CONTROL TASK & DIAGNOSTICS ................ ~line 2361
//  19.  SERIAL COMMAND PARSER (User Interface) ............. ~line 2578
//  20.  CORE 0 ENTRY POINTS (setup / loop) ................. ~line 3044
//  21.  CORE 1 CAN SERVICE (setup1 / loop1) ................ ~line 3091
//
// ============================================================================
//  SERIAL COMMAND QUICK REFERENCE
// ============================================================================
//
//  STARTUP (before node is ready):
//    <nodeId>              - Set this node's ID (1-8)
//    <totalNodes>          - Set total number of nodes in the network
//
//  SET COMMANDS (node 0 = broadcast to all):
//    u <node> <pwm>        - Set LED duty cycle directly (0-4095)
//    r <node> <lux>        - Set illuminance reference (lux)
//    o <node> <state>      - Set occupancy state: 'o'=off, 'l'=low, 'h'=high
//    a <node> <0|1>        - Enable/disable anti-windup
//    f <node> <0|1>        - Enable/disable feedback (PI) control
//    O <node> <lux>        - Set high (occupied) illuminance bound
//    U <node> <lux>        - Set low (unoccupied) illuminance bound
//    C <node> <cost>       - Set energy cost coefficient
//
//  STREAM COMMANDS:
//    s <y|u> <node>        - Start streaming variable ('y'=lux, 'u'=PWM)
//    S <y|u> <node>        - Stop streaming variable
//
//  GET / QUERY COMMANDS:
//    g <var> <node>         - Get a variable from local or remote node
//      Variables: u=PWM, r=ref, y=lux, v=voltage, o=occupancy,
//                 a=anti-windup, f=feedback, d=baseline, p=power,
//                 t=uptime, E=energy, V=visibility, F=flicker,
//                 O=high_ref, U=low_ref, L=current_ref, C=cost
//    g b <y|u> <node>      - Get full history buffer
//    g diag                - Get CAN/control diagnostics
//
//  CALIBRATION:
//    c <pwm>               - Trigger calibration (coordinator only)
//    rpt                   - Print calibration gain report
//
//  ALGORITHM SELECTION:
//    A <mode>              - 0=none, 1=consensus, 2=ADMM, 3=dual decomp
//
//  SYSTEM:
//    R                     - Restart (broadcast reset to all nodes)
//    RM                    - Reset accumulated metrics only
//    BOOTSEL               - Reboot Pico into USB bootloader mode
//
// ============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <pico/time.h>

// ============================================================================
//  SECTION 1: HARDWARE CONFIGURATION & CONSTANTS
//  Pin assignments, ADC/LDR parameters, timing periods, CAN ID ranges,
//  and default calibration parameters.
// ============================================================================

// --- GPIO Pin Assignments ---
const uint8_t LED_PIN = 15;       // PWM output to LED driver
const uint8_t LDR_PIN = 26;       // ADC input from light-dependent resistor
const uint8_t CAN_CS_PIN = 17;    // SPI chip-select for MCP2515
const uint8_t CAN_INT_PIN = 20;   // Interrupt line from MCP2515 (active LOW)

// --- ADC / LDR Conversion Parameters ---
// The LDR is in a voltage divider with R_FIXED.  Raw ADC -> voltage -> resistance
// -> lux via the Steinhart-style log-linear model:  log10(lux) = (log10(R) - B) / M
const float ADC_REF = 3.3f;       // Pico ADC reference voltage
const float ADC_MAX = 4095.0f;    // 12-bit ADC full scale
const float R_FIXED = 10000.0f;   // Fixed resistor in LDR voltage divider (ohms)
const float M_PARAM = -0.7f;      // LDR model slope (log-log)
const float B_PARAM_DEFAULT = 5.928f;  // LDR model intercept (default)
// Per-node B parameters (index 0 unused; nodes 1-8 may have individual LDR curves)
const float B_PARAM_PER_NODE[] = {0.0f, 6.293f, 5.928f, 5.364f, 5.928f, 5.928f, 5.928f, 5.928f, 5.928f};
float B_PARAM = B_PARAM_DEFAULT;
const float MAX_POWER_W = 1.0f;   // Maximum LED power consumption (watts)

// --- Timing Constants ---
const uint32_t CONTROL_PERIOD_MS = 10;        // 100 Hz control loop (timer-interrupt driven)
const uint32_t HELLO_PERIOD_MS = 1000;        // Heartbeat / peer-status broadcast interval
const uint32_t STREAM_PERIOD_MS = 100;        // Real-time data streaming rate to serial
const uint32_t WAKEUP_WINDOW_MS = 5000;       // Discovery window after boot (ms)
const uint32_t WAKEUP_RETRY_MS = 500;         // Announce retry interval during discovery
const uint32_t PEER_TIMEOUT_MS = 8000;        // Peer considered dead after this silence
const uint32_t CORE1_HEALTH_PERIOD_MS = 250;  // MCP2515 health-check polling interval
const uint32_t DIAG_PERIOD_MS = 1000;         // Diagnostics / timeout check interval

// --- CAN Identifier Ranges ---
// Each message class occupies a 0x80-wide ID range; the low bits encode the
// source or destination node ID so hardware filtering can be applied later.
const uint16_t CAN_ID_HELLO_BASE = 0x100;     // Heartbeat / peer status
const uint16_t CAN_ID_COMMAND_BASE = 0x300;    // Remote parameter commands
const uint16_t CAN_ID_CALIB_BASE = 0x400;      // Calibration plan frames
const uint16_t CAN_ID_QUERY_BASE = 0x500;      // Remote query request
const uint16_t CAN_ID_REPLY_BASE = 0x580;      // Remote query reply
const uint16_t CAN_ID_STREAM_BASE = 0x600;     // Real-time data stream

// --- Network & Buffer Sizing ---
const uint8_t BROADCAST_NODE = 0x7F;           // Pseudo-node ID for broadcast
const int MAX_NODES = 8;                       // Maximum nodes supported
const int HISTORY_LEN = 600;                   // Circular buffer depth for y/u history
const int SERIAL_BUFFER_LEN = 96;              // Serial line buffer size
const int TX_QUEUE_LEN = 24;                   // Core 0 -> Core 1 CAN TX queue depth
const int CORE1_EVENT_QUEUE_LEN = 24;          // Core 1 -> Core 0 CAN RX queue depth

// --- Calibration Default Parameters ---
const uint16_t DEFAULT_CAL_PWM = 2800;         // PWM level driven during calibration slots
const uint16_t DEFAULT_SETTLE_MS = 250;        // Time to let light settle before measuring
const uint16_t DEFAULT_MEASURE_MS = 600;       // Measurement averaging window
const uint16_t DEFAULT_GAP_MS = 250;           // Gap between consecutive calibration slots
const uint16_t DEFAULT_START_DELAY_MS = 1500;  // Delay before calibration begins (synchronisation)
const uint8_t CAL_PLAN_REPEAT_COUNT = 5;       // How many times to re-broadcast the plan for reliability
const uint32_t CAL_PLAN_REPEAT_INTERVAL_MS = 60; // Interval between plan re-broadcasts

const uint8_t CAL_TIMEBASE_MS = 10;            // Tick unit for calibration timing (10 ms per tick)

MCP_CAN canBus(CAN_CS_PIN);
repeating_timer_t controlTimer;

// ============================================================================
//  SECTION 2: PROTOCOL ENUMS, STRUCTS & TYPE DEFINITIONS
//  CAN command codes, calibration types, query codes, state-machine enums,
//  and data structures for peers, calibration, FIFO transport, etc.
// ============================================================================

// Commands sent via CAN_ID_COMMAND_BASE frames to change remote node parameters
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

// Calibration plan is split into two CAN frames (A and B) due to 8-byte limit
enum CalibType : uint8_t {
  CALIB_PLAN_A = 0x10,   // First half: session ID, PWM, node count, settle time
  CALIB_PLAN_B = 0x11    // Second half: measure time, gap time, start delay
};

// Query codes used in CAN_ID_QUERY / CAN_ID_REPLY exchanges (ASCII-based for readability)
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

// Startup sequence: user must enter node ID and total nodes via serial before operation
enum StartupState : uint8_t {
  STARTUP_WAIT_NODE_ID = 0,      // Waiting for user to type this node's ID
  STARTUP_WAIT_TOTAL_NODES = 1,  // Waiting for user to type total node count
  STARTUP_READY = 2              // Fully configured, normal operation
};

// Wakeup / discovery state machine: nodes find each other, then coordinator triggers calibration
enum WakeupState : uint8_t {
  WAKEUP_BOOT = 0,              // Initial state, triggers resetRuntimeState()
  WAKEUP_DISCOVERY_OPEN = 1,    // Broadcasting HELLO/ANNOUNCE, waiting for peers
  WAKEUP_DISCOVERY_STABLE = 2,  // Discovery window closed, coordinator starts calibration
  WAKEUP_RUN = 3                // Normal operation after discovery + calibration
};

// Calibration state machine: each node cycles through baseline + per-node slot measurements
// Sequence: IDLE -> WAIT_START -> BASELINE_SETTLE -> BASELINE_MEASURE ->
//           [SLOT_SETTLE -> SLOT_MEASURE] x N -> FINISHED
enum CalibrationState : uint8_t {
  CAL_IDLE = 0,               // No calibration in progress
  CAL_WAIT_START_TIME = 1,    // Waiting for the synchronized start tick
  CAL_BASELINE_SETTLE = 2,    // All LEDs off, letting light settle
  CAL_BASELINE_MEASURE = 3,   // Measuring ambient baseline illuminance
  CAL_SLOT_SETTLE = 4,        // One node's LED on, waiting for settle
  CAL_SLOT_MEASURE = 5,       // Measuring illuminance with one node's LED on
  CAL_FINISHED = 6            // Calibration complete, gains computed
};

// Inter-core FIFO message types (encoded in the top byte of each 32-bit FIFO word)
enum FifoMessageKind : uint8_t {
  FIFO_TX_FRAME = 0x01,   // Core 0 -> Core 1: "please transmit this CAN frame"
  FIFO_RX_FRAME = 0x02,   // Core 1 -> Core 0: "I received this CAN frame"
  FIFO_DIAG = 0x03        // Core 1 -> Core 0: diagnostic / error report (1 word)
};

// Diagnostic error codes reported by Core 1 back to Core 0 via FIFO
enum Core1DiagStage : uint8_t {
  DIAG_CAN_INIT = 1,       // MCP2515 initialisation failed
  DIAG_CAN_MODE = 2,       // setMode() failed
  DIAG_CAN_SEND = 3,       // sendMsgBuf() failed
  DIAG_CAN_CHECK = 4,      // checkReceive() returned unexpected status
  DIAG_CAN_READ = 5,       // readMsgBuf() failed
  DIAG_CAN_HEALTH = 6,     // Periodic health check detected error
  DIAG_CAN_FIFO_DROP = 7   // Core 1 event queue full, frame dropped
};

// Status of a remote peer node, updated via HELLO frames on the CAN bus
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

// Generic CAN frame used for both TX queue and RX event queue
struct CanFrame {
  uint16_t id = 0;
  uint8_t len = 0;
  uint8_t data[8] = {0};
};

// Staging buffer for multi-word FIFO transfers between cores.
// A CAN frame requires 3 words: header + 4 payload bytes + 4 payload bytes.
struct FifoTxStaging {
  bool active = false;
  uint32_t words[3] = {0};
  uint8_t wordCount = 0;
  uint8_t index = 0;
};

// Receives the two halves of a calibration plan (PLAN_A + PLAN_B) from CAN
// and stores them until both arrive, at which point a session is started.
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

// Full calibration session state: tracks state machine phase, timing,
// accumulated measurements, and the resulting gain row K[myNode][*].
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

// Tracks one outstanding remote query so we can detect timeouts
struct PendingRemoteQuery {
  bool active = false;
  QueryCode code = Q_U;
  uint8_t targetNode = 0;
  uint32_t requestedAtMs = 0;
};

// Coordinator-side state for repeatedly broadcasting the calibration plan
// (reliability: the plan is sent multiple times since CAN has no ACK at app level)
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
//  SECTION 3: GLOBAL RUNTIME STATE (Core 0 owned)
//  All mutable state below is read/written exclusively by Core 0.
//  Core 1 only accesses the FIFO queues and its own local variables.
// ============================================================================

// --- Peer tracking & subsystem contexts ---
PeerStatus peers[MAX_NODES];               // Table of known remote peers
StreamState streamState;                   // Active real-time stream (if any)
CalibrationPlanBuffer pendingPlan;         // Partially received calibration plan
CalibrationContext calib;                  // Active calibration session state
PendingRemoteQuery pendingRemoteQuery;     // Outstanding remote query tracker
CalibrationBroadcastState calibBroadcast;  // Coordinator plan-broadcast state

// --- Core 0 -> Core 1 CAN transmit queue (circular buffer) ---
CanFrame txQueue[TX_QUEUE_LEN];
uint8_t txQueueHead = 0;
uint8_t txQueueTail = 0;
uint8_t txQueueCount = 0;
FifoTxStaging core0ToCore1Staging;  // Staging area for multi-word FIFO push

// --- Node identity & startup ---
uint8_t nodeId = 1;                                // This node's ID (set by user at startup)
uint8_t totalNodes = 1;                            // Total nodes in the network
bool isCoordinator = true;                         // Node 1 is the coordinator by default
StartupState startupState = STARTUP_WAIT_NODE_ID;  // Sequential startup flow
WakeupState wakeupState = WAKEUP_BOOT;             // Discovery / wakeup state
bool autoCalibrationTriggered = false;              // Prevents double auto-calibration

// --- Sensor readings & control setpoints ---
float filteredLux = 0.0f;           // Low-pass filtered illuminance (lux)
float lastLdrVoltage = 0.0f;       // Most recent LDR voltage (for diagnostics)
uint16_t localPwm = 0;             // Current LED PWM output (0-4095)
float refLux = 20.0f;              // Illuminance reference / setpoint (lux)
char occupancyState = 'o';         // 'o'=off, 'l'=low, 'h'=high occupancy
bool antiWindupEnabled = true;     // PI anti-windup via back-calculation
bool feedbackEnabled = false;      // Master switch for closed-loop PI control
float highLuxBound = 30.0f;        // Illuminance bound when occupied ('h')
float lowLuxBound = 10.0f;         // Illuminance bound when unoccupied ('l'/'o')
float currentLuxLowerBound = 10.0f; // Active lower bound (depends on occupancy)
float energyCost = 1.0f;           // Cost coefficient c_i for optimisation

// --- Accumulated performance metrics ---
float energyJ = 0.0f;                   // Total energy consumed (joules)
float visibilityErrorIntegral = 0.0f;   // Sum of max(0, ref - lux) over samples
float flickerIntegral = 0.0f;           // Accumulated flicker metric
float restartSeconds = 0.0f;            // Time since last restart (seconds)
uint32_t metricSampleCount = 0;         // Number of control samples taken

// --- PI Controller state ---
// The controller combines feedforward (from calibrated gains) with a PI
// corrector.  Anti-windup uses back-calculation: when the raw output saturates,
// the integrator is adjusted by (clamped - raw) / TT to prevent windup.
float piIntegral = 0.0f;       // Integrator accumulator
float piPrevError = 0.0f;      // Previous error (unused with current structure, reserved for PID)
float piPrevDuty = 0.0f;       // Previous duty (for flicker detection)
float piPrevPrevDuty = 0.0f;   // Two-steps-ago duty (for flicker detection)
const float PI_KP = 0.01f;     // Proportional gain
const float PI_KI = 0.11f;     // Integral gain
const float PI_KD = 0.0f;      // Derivative gain (currently unused)
const float PI_B_SP = 1.0f;    // Set-point weighting for proportional term (1 = standard PI)
const float PI_TT = 0.15f;     // Anti-windup tracking time constant (smaller = faster recovery)
const float PI_DT = 0.01f;     // Sample period in seconds (100 Hz = 10 ms)

// ============================================================================
//  SECTION 4: DISTRIBUTED CONSENSUS ALGORITHM
//  Implements a consensus-based distributed optimisation where each node
//  solves a local cost minimisation subject to illuminance constraints,
//  then averages its duty-cycle proposal with all peers.  Convergence is
//  detected when the maximum change in the averaged duty vector falls
//  below CONS_TOL.
//
//  Problem:  min  sum_i c_i * d_i
//            s.t. K * d + o >= L_ref   (illuminance constraints for all nodes)
//                 0 <= d_i <= 1        (duty bounds)
//
//  Each iteration: (1) solve local, (2) broadcast proposal, (3) average.
// ============================================================================

// Algorithm selector shared across all three distributed optimisation methods
enum AlgorithmMode : uint8_t { ALG_NONE = 0, ALG_CONSENSUS = 1, ALG_ADMM = 2, ALG_DUAL_DECOMP = 3 };

const int CONS_MAX_NODES = 3;                  // Max nodes for the optimisation (K matrix dimension)
const int CONS_MAX_ITER = 50;                  // Maximum consensus iterations before stopping
const float CONS_TOL = 1e-3f;                  // Convergence tolerance on d_avg change
const float CONS_RHO = 2.0f;                   // Augmented Lagrangian penalty parameter
const uint32_t CONSENSUS_PERIOD_MS = 100;      // Period between consensus iterations
const uint16_t CAN_ID_CONSENSUS_BASE = 0x700;  // CAN ID range for consensus proposals
const uint16_t CAN_ID_GAINEXCH_BASE = 0x680;   // CAN ID range for gain matrix exchange

// Full state for the consensus algorithm, including the shared gain matrix K,
// baseline offsets o, reference illuminances L_ref, and cost coefficients c.
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

ConsensusState cons;

// Initialises the consensus algorithm after gain exchange is complete.
// Fills the K matrix from calibration data and sets an initial feedforward guess.
void initConsensus() {
  cons.numNodes = totalNodes;
  cons.myIndex = nodeId - 1;
  cons.iteration = 0;
  cons.converged = false;
  cons.active = true;
  cons.iterByte = 0;

  memset(cons.d, 0, sizeof(cons.d));
  memset(cons.d_avg, 0, sizeof(cons.d_avg));
  memset(cons.d_others, 0, sizeof(cons.d_others));
  memset(cons.received, 0, sizeof(cons.received));

  // Build K matrix from calibration (convert gain-per-PWM to gain-per-duty)
  for (int j = 0; j < cons.numNodes; j++) {
    cons.K[cons.myIndex][j] = calib.gainRow[j + 1] * 4095.0f;
  }
  cons.o[cons.myIndex] = calib.baselineLux;

  // Set own reference and cost (others come from gain exchange)
  cons.L_ref[cons.myIndex] = refLux;
  cons.c[cons.myIndex] = energyCost;

  Serial.print("Consensus init: myIdx="); Serial.print(cons.myIndex);
  Serial.print(" L_ref=[");
  for (int i = 0; i < cons.numNodes; i++) { Serial.print(cons.L_ref[i]); Serial.print(" "); }
  Serial.print("] K_diag=[");
  for (int i = 0; i < cons.numNodes; i++) { Serial.print(cons.K[i][i]); Serial.print(" "); }
  Serial.println("]");

  // Initial guess: feedforward
  for (int i = 0; i < cons.numNodes; i++) {
    float needed = cons.L_ref[i] - cons.o[i];
    if (needed < 0) needed = 0;
    cons.d[i] = (cons.K[i][i] > 0) ? needed / cons.K[i][i] : 0.0f;
    cons.d[i] = constrain(cons.d[i], 0.0f, 1.0f);
  }
}

// Solves the local optimisation for this node's duty cycle d[i].
// Finds the minimum d[i] that satisfies ALL nodes' illuminance constraints,
// then picks the cost-optimal value above that minimum.
// This is the key per-node step in the consensus ADMM iteration.
void solveLocalConsensus() {
  int i = cons.myIndex;

  // Copy d_avg for all other nodes (we only optimize our own d[i])
  for (int j = 0; j < cons.numNodes; j++) {
    if (j != i) cons.d[j] = cons.d_avg[j];
  }

  // Check ALL nodes' constraints to find minimum d[i]
  float d_min = 0.0f;
  for (int k = 0; k < cons.numNodes; k++) {
    float lux_k = cons.o[k];
    for (int j = 0; j < cons.numNodes; j++) {
      if (j != i) lux_k += cons.K[k][j] * cons.d_avg[j];
    }
    if (cons.K[k][i] > 0.0f) {
      float d_needed = (cons.L_ref[k] - lux_k) / cons.K[k][i];
      if (d_needed > d_min) d_min = d_needed;
    }
  }
  if (d_min < 0.0f) d_min = 0.0f;

  // Cost-optimal duty (unconstrained): minimize c[i]*d[i] + rho/2*(d[i]-d_avg[i])^2
  float d_unc = cons.d_avg[i] - cons.c[i] / CONS_RHO;

  // Take the maximum of constraint and cost-optimal (constraint always wins)
  float d_new = (d_unc > d_min) ? d_unc : d_min;
  cons.d[i] = constrain(d_new, 0.0f, 1.0f);
}

// Computes the new average duty vector from this node's proposal and all
// received peer proposals.  Checks convergence by comparing d_avg change.
void updateConsensusAverage() {
  float d_old[CONS_MAX_NODES];
  memcpy(d_old, cons.d_avg, sizeof(float) * cons.numNodes);

  for (int j = 0; j < cons.numNodes; j++) {
    float sum = cons.d[j];
    int count = 1;
    for (int k = 0; k < cons.numNodes; k++) {
      if (k == cons.myIndex) continue;
      sum += cons.d_others[k][j];
      count++;
    }
    cons.d_avg[j] = sum / count;
  }

  float max_diff = 0.0f;
  for (int j = 0; j < cons.numNodes; j++) {
    float diff = fabsf(cons.d_avg[j] - d_old[j]);
    if (diff > max_diff) max_diff = diff;
  }
  cons.converged = (max_diff < CONS_TOL);
  cons.iteration++;
}

// Broadcasts this node's current duty proposal over CAN.
// Encodes up to 3 duty values as uint16 (d * 10000) in one 8-byte frame.
void sendConsensusProposal() {
  // Encode 3 duty values as uint16 (d * 10000) in 8 bytes
  uint8_t payload[8] = {0};
  payload[0] = cons.iterByte;
  payload[1] = (uint8_t)cons.myIndex;
  for (int j = 0; j < cons.numNodes && j < 3; j++) {
    uint16_t enc = (uint16_t)(cons.d[j] * 10000.0f);
    payload[2 + j * 2] = (enc >> 8) & 0xFF;
    payload[3 + j * 2] = enc & 0xFF;
  }
  enqueueTxFrame(CAN_ID_CONSENSUS_BASE + nodeId, payload, 8);
}

// Processes a received consensus proposal from a peer node.
// Decodes the duty values and stores them in d_others[senderIdx][].
void handleConsensusFrame(uint8_t len, const uint8_t *data) {
  if (len < 8 || cons.mode != ALG_CONSENSUS) return;
  uint8_t senderIdx = data[1];
  if (senderIdx >= CONS_MAX_NODES || senderIdx == cons.myIndex) return;

  for (int j = 0; j < cons.numNodes && j < 3; j++) {
    uint16_t enc = ((uint16_t)data[2 + j * 2] << 8) | data[3 + j * 2];
    cons.d_others[senderIdx][j] = enc / 10000.0f;
  }
  cons.received[senderIdx] = true;
}

// Broadcasts this node's calibration gain row (K[myIdx][*]), baseline, L_ref,
// and cost coefficient over CAN so all peers can build the full K matrix.
// Called repeatedly during the gain-exchange phase until all nodes have responded.
void sendGainRow() {
  // Send own gain row: 3 frames (one per gain value) + 1 for baseline
  for (int j = 0; j < totalNodes && j < 3; j++) {
    uint8_t payload[8] = {0};
    payload[0] = (uint8_t)(nodeId - 1);  // sender row index
    payload[1] = (uint8_t)j;              // column index
    union { float f; uint8_t b[4]; } u;
    u.f = calib.gainRow[j + 1] * 4095.0f;  // convert to duty-space
    memcpy(&payload[2], u.b, 4);
    payload[6] = 0xAA;  // marker: gain value
    enqueueTxFrame(CAN_ID_GAINEXCH_BASE + nodeId, payload, 7);
  }
  // Send baseline + L_ref + cost in one frame
  uint8_t payload[8] = {0};
  payload[0] = (uint8_t)(nodeId - 1);
  payload[1] = 0xFF;  // marker for baseline
  union { float f; uint8_t b[4]; } u;
  u.f = calib.baselineLux;
  memcpy(&payload[2], u.b, 4);
  payload[6] = 0xBB;
  enqueueTxFrame(CAN_ID_GAINEXCH_BASE + nodeId, payload, 7);

  // Send L_ref
  memset(payload, 0, sizeof(payload));
  payload[0] = (uint8_t)(nodeId - 1);
  payload[1] = 0xFE;  // marker for L_ref
  u.f = currentLuxLowerBound;
  memcpy(&payload[2], u.b, 4);
  enqueueTxFrame(CAN_ID_GAINEXCH_BASE + nodeId, payload, 6);

  // Send cost
  memset(payload, 0, sizeof(payload));
  payload[0] = (uint8_t)(nodeId - 1);
  payload[1] = 0xFD;  // marker for cost
  u.f = energyCost;
  memcpy(&payload[2], u.b, 4);
  enqueueTxFrame(CAN_ID_GAINEXCH_BASE + nodeId, payload, 6);
}

// Processes a received gain-exchange frame from a peer.
// Discriminates between gain values (colIdx < N), baseline (0xFF), L_ref (0xFE), and cost (0xFD).
void handleGainExchangeFrame(uint8_t len, const uint8_t *data) {
  if (len < 7) return;
  uint8_t rowIdx = data[0];
  uint8_t colIdx = data[1];
  if (rowIdx >= CONS_MAX_NODES) return;

  union { float f; uint8_t b[4]; } u;
  memcpy(u.b, &data[2], 4);

  if (colIdx == 0xFF) {
    cons.o[rowIdx] = u.f;
  } else if (colIdx == 0xFE) {
    cons.L_ref[rowIdx] = u.f;
  } else if (colIdx == 0xFD) {
    cons.c[rowIdx] = u.f;
  } else if (colIdx < CONS_MAX_NODES) {
    cons.K[rowIdx][colIdx] = u.f;
  }
  cons.gainRowReceived[rowIdx] = true;
}

// Returns true when gain-exchange frames have been received from all nodes.
bool allGainsReceived() {
  for (int i = 0; i < totalNodes; i++) {
    if (!cons.gainRowReceived[i]) return false;
  }
  return true;
}

// Top-level consensus service function, called from loop().
// Manages the gain-exchange phase first, then runs iterative consensus.
// When converged, applies the optimal duty to the PI controller reference.
void serviceConsensus() {
  if (cons.mode != ALG_CONSENSUS || startupState != STARTUP_READY) return;

  uint32_t now = millis();

  // Gain exchange phase: broadcast gains after calibration
  if (!cons.gainsExchanged && calib.gainsReady) {
    if (cons.gainExchStartMs == 0) {
      cons.gainExchStartMs = now;
      cons.gainRowReceived[cons.myIndex] = true;
      // Fill own row
      for (int j = 0; j < totalNodes; j++) {
        cons.K[cons.myIndex][j] = calib.gainRow[j + 1] * 4095.0f;
      }
      cons.o[cons.myIndex] = calib.baselineLux;
    }
    // Broadcast gains periodically until all received
    if (now - cons.gainExchStartMs < 5000) {
      static uint32_t lastGainSend = 0;
      if (now - lastGainSend > 500) {
        sendGainRow();
        lastGainSend = now;
      }
      if (allGainsReceived()) {
        cons.gainsExchanged = true;
        if (cons.mode == ALG_CONSENSUS) initConsensus();
        if (cons.mode == ALG_ADMM) initADMM();
        if (cons.mode == ALG_DUAL_DECOMP) initDualDecomp();
      }
      return;
    }
    // Timeout: proceed with what we have
    cons.gainsExchanged = true;
    if (cons.mode == ALG_CONSENSUS) initConsensus();
    if (cons.mode == ALG_ADMM) initADMM();
    if (cons.mode == ALG_DUAL_DECOMP) initDualDecomp();
  }

  if (!cons.active) return;
  if (cons.converged && cons.iteration >= CONS_MAX_ITER) return;

  // Run at CONSENSUS_PERIOD_MS rate
  if (now - cons.lastIterMs < CONSENSUS_PERIOD_MS) return;
  cons.lastIterMs = now;

  // Solve local
  solveLocalConsensus();

  // Broadcast proposal
  sendConsensusProposal();

  // Check if all received
  bool allRx = true;
  for (int k = 0; k < cons.numNodes; k++) {
    if (k == cons.myIndex) continue;
    if (!cons.received[k]) { allRx = false; break; }
  }

  if (allRx || cons.iteration > 0) {
    updateConsensusAverage();
    memset(cons.received, 0, sizeof(cons.received));
    cons.iterByte++;

    // Apply consensus duty to PI reference
    if (feedbackEnabled) {
      float estLux = cons.o[cons.myIndex];
      for (int j = 0; j < cons.numNodes; j++) {
        estLux += cons.K[cons.myIndex][j] * cons.d_avg[j];
      }
      refLux = estLux;
    }
  }
}

// ============================================================================
//  SECTION 5: ADMM ALGORITHM (Alternating Direction Method of Multipliers)
//  Three-step iteration: d-update (local cost), z-update (constraint projection),
//  u-update (dual variable / scaled residual).  Convergence when the primal
//  residual ||d - z|| drops below CONS_TOL.
//
//  ADMM splits the problem into:
//    d-step: each node minimises  c_i*d_i + (rho/2)*||d_i - z_i + u_i||^2
//    z-step: project z onto the feasible set  K*z + o >= L_ref, 0 <= z <= 1
//    u-step: u_i += d_i - z_i  (dual ascent)
// ============================================================================

const float ADMM_RHO = 1.0f;
const uint16_t CAN_ID_ADMM_BASE = 0x780;

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

ADMMState admm;

// Initialises ADMM state with a feedforward initial guess from the gain matrix.
void initADMM() {
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

// ADMM d-update: minimise cost + penalty for deviation from z, for this node only.
// Closed-form solution: d_i = z_i - u_i - c_i / rho, clamped to [0,1].
void admmUpdateD() {
  int i = cons.myIndex;
  admm.d[i] = admm.z[i] - admm.u[i] - cons.c[i] / ADMM_RHO;
  admm.d[i] = constrain(admm.d[i], 0.0f, 1.0f);
}

// ADMM z-update: project (d + u) onto the feasible set defined by K*z + o >= L_ref.
// Uses iterative constraint projection (up to 20 inner iterations).
// Each violated constraint pushes z along the constraint normal by the deficit.
void admmUpdateZ() {
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
    for (int j = 0; j < cons.numNodes; j++)
      admm.z[j] = constrain(admm.z[j], 0.0f, 1.0f);
    if (feasible) break;
  }
}

// ADMM u-update: dual variable ascent.  Also computes primal residual for convergence check.
void admmUpdateU() {
  float primal_res = 0.0f;
  for (int i = 0; i < cons.numNodes; i++) {
    admm.u[i] += admm.d[i] - admm.z[i];
    primal_res += (admm.d[i] - admm.z[i]) * (admm.d[i] - admm.z[i]);
  }
  admm.converged = (sqrtf(primal_res) < CONS_TOL);
  admm.iteration++;
}

// Broadcasts this node's d and u values to peers via CAN.
// d is sent as a float (4 bytes), u as a scaled int16 (u * 1000).
void sendADMMProposal() {
  // Send d[myIndex] and u[myIndex] as floats
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

// Processes a received ADMM proposal frame from a peer node.
void handleADMMFrame(uint8_t len, const uint8_t *data) {
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

// Top-level ADMM service function, called from loop().
// Runs one d-z-u iteration per CONSENSUS_PERIOD_MS and applies the result to refLux.
void serviceADMM() {
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

// ============================================================================
//  SECTION 6: DUAL DECOMPOSITION ALGORITHM
//  A subgradient-based method that decomposes the problem using Lagrange
//  multipliers (lambda) on the illuminance constraints.
//
//  Each iteration:
//    Primal: d_i -= alpha * (c_i - sum_k lambda_k * K[k][i])   (gradient descent)
//    Dual:   lambda_k += alpha * (L_ref_k - lux_k)             (subgradient ascent)
//
//  The step size alpha decays geometrically (alpha *= DD_ALPHA_DECAY).
// ============================================================================

const uint16_t CAN_ID_DUAL_BASE = 0x7C0;   // CAN ID range for dual decomposition proposals
const float DD_ALPHA_INIT = 0.05f;         // Initial step size for subgradient method
const float DD_ALPHA_DECAY = 0.995f;       // Geometric decay factor for step size

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

DualDecompState dd;

// Initialises the dual decomposition state with feedforward initial guess.
void initDualDecomp() {
  dd.iteration = 0;
  dd.converged = false;
  dd.active = true;
  dd.alpha = DD_ALPHA_INIT;
  dd.lastIterMs = 0;
  memset(dd.received, 0, sizeof(dd.received));
  memset(dd.lambda, 0, sizeof(dd.lambda));

  for (int i = 0; i < cons.numNodes; i++) {
    float needed = cons.L_ref[i] - cons.o[i];
    if (needed < 0) needed = 0;
    dd.d[i] = (cons.K[i][i] > 0) ? needed / cons.K[i][i] : 0.0f;
    dd.d[i] = constrain(dd.d[i], 0.0f, 1.0f);
  }
}

// Primal update: gradient descent on the Lagrangian for this node's duty d[i].
// grad = c_i - sum_k( lambda_k * K[k][i] ),  then d[i] -= alpha * grad
void ddUpdatePrimal() {
  int i = cons.myIndex;
  float grad = cons.c[i];
  for (int k = 0; k < cons.numNodes; k++)
    grad -= dd.lambda[k] * cons.K[k][i];

  dd.d[i] -= dd.alpha * grad;
  dd.d[i] = constrain(dd.d[i], 0.0f, 1.0f);
}

// Dual update: subgradient ascent on lambda.
// violation_k = L_ref_k - (K*d + o)_k;  lambda_k += alpha * violation_k
// Lambda is projected to non-negative (KKT complementarity).
void ddUpdateDual() {
  float max_violation = 0.0f;
  for (int k = 0; k < cons.numNodes; k++) {
    float lux = cons.o[k];
    for (int j = 0; j < cons.numNodes; j++)
      lux += cons.K[k][j] * dd.d[j];

    float violation = cons.L_ref[k] - lux;
    dd.lambda[k] += dd.alpha * violation;
    if (dd.lambda[k] < 0.0f) dd.lambda[k] = 0.0f;

    if (fabsf(violation) > max_violation)
      max_violation = fabsf(violation);
  }
  dd.alpha *= DD_ALPHA_DECAY;
  dd.converged = (max_violation < CONS_TOL);
  dd.iteration++;
}

// Broadcasts this node's duty value d[myIndex] as a float over CAN.
void sendDualDecompProposal() {
  uint8_t payload[8] = {0};
  payload[0] = (uint8_t)cons.myIndex;
  union { float f; uint8_t b[4]; } u;
  u.f = dd.d[cons.myIndex];
  memcpy(&payload[1], u.b, 4);
  enqueueTxFrame(CAN_ID_DUAL_BASE + nodeId, payload, 5);
}

// Processes a received dual decomposition duty value from a peer.
void handleDualDecompFrame(uint8_t len, const uint8_t *data) {
  if (len < 5 || cons.mode != ALG_DUAL_DECOMP) return;
  uint8_t senderIdx = data[0];
  if (senderIdx >= CONS_MAX_NODES || senderIdx == cons.myIndex) return;

  union { float f; uint8_t b[4]; } u;
  memcpy(u.b, &data[1], 4);
  dd.d[senderIdx] = u.f;
  dd.received[senderIdx] = true;
}

// Top-level dual decomposition service, called from loop().
void serviceDualDecomp() {
  if (cons.mode != ALG_DUAL_DECOMP || startupState != STARTUP_READY) return;
  if (!cons.gainsExchanged) return;
  if (!dd.active) return;
  if (dd.converged && dd.iteration >= 100) return;

  uint32_t now = millis();
  if (now - dd.lastIterMs < CONSENSUS_PERIOD_MS) return;
  dd.lastIterMs = now;

  ddUpdatePrimal();
  sendDualDecompProposal();

  bool allRx = true;
  for (int k = 0; k < cons.numNodes; k++) {
    if (k == cons.myIndex) continue;
    if (!dd.received[k]) { allRx = false; break; }
  }

  if (allRx || dd.iteration > 0) {
    ddUpdateDual();
    memset(dd.received, 0, sizeof(dd.received));

    if (feedbackEnabled) {
      float estLux = cons.o[cons.myIndex];
      for (int j = 0; j < cons.numNodes; j++)
        estLux += cons.K[cons.myIndex][j] * dd.d[j];
      refLux = estLux;
    }
  }
}

// ============================================================================
//  SECTION 7: HISTORY BUFFERS, TIMING, & DIAGNOSTIC COUNTERS
// ============================================================================

// --- Circular history buffers for 'g b y/u' command ---
uint16_t yHistory[HISTORY_LEN];    // Illuminance history (lux * 100 as uint16)
uint16_t uHistory[HISTORY_LEN];    // PWM history (raw 0-4095)
int historyIndex = 0;
bool historyWrapped = false;

// --- Serial input accumulator (nonblocking line reader) ---
char serialBuffer[SERIAL_BUFFER_LEN];
size_t serialLength = 0;

// --- Timing bookkeeping ---
uint32_t bootMs = 0;              // millis() at last reset
uint32_t lastHelloMs = 0;         // Last heartbeat broadcast time
uint32_t lastWakeupMs = 0;        // Last discovery announce time
uint32_t lastStreamMs = 0;        // Last stream output time
uint32_t lastDiagMs = 0;          // Last diagnostics check time

// --- CAN & control error counters (reported via 'g diag') ---
uint32_t canTxErrorCount = 0;
uint32_t canRxErrorCount = 0;
uint32_t canProtocolErrorCount = 0;
uint32_t controlOverrunCount = 0;  // Control steps that exceeded CONTROL_PERIOD_MS
uint32_t controlMaxExecUs = 0;     // Worst-case control step execution time
uint32_t controlLastExecUs = 0;    // Most recent control step execution time

// --- Timer-driven control scheduling ---
// The repeating timer ISR increments controlDueCount; the main loop decrements it.
// If controlDueCount > 1, we missed a deadline (overrun).
volatile uint8_t controlDueCount = 0;

// --- Core 1 state (CAN RX event queue and FIFO staging) ---
volatile bool canIrqPending = false;               // Set by ISR, cleared by Core 1 loop
CanFrame core1EventQueue[CORE1_EVENT_QUEUE_LEN];   // Received frames waiting for FIFO transfer
uint8_t core1EventHead = 0;
uint8_t core1EventTail = 0;
uint8_t core1EventCount = 0;
FifoTxStaging core1ToCore0Staging;   // Staging area for Core 1 -> Core 0 FIFO push
uint32_t core1LastHealthMs = 0;      // Last MCP2515 health check timestamp

// ============================================================================
//  SECTION 8: INTERRUPT HANDLERS & FIFO PACKING HELPERS
//  The RP2040 hardware FIFO transfers 32-bit words between cores.
//  A CAN frame is encoded as 3 words: [header][payload_lo][payload_hi].
//  Diagnostics are a single word with stage/code/detail packed in.
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

// ============================================================================
//  SECTION 9: LOCAL SENSING & ACTUATION HELPERS
//  LDR reading with oversampling and low-pass filter, LED PWM control,
//  peer management, and history recording.
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

// Returns the instantaneous LED power consumption based on current PWM duty.
float getInstantPowerW() {
  return MAX_POWER_W * ((float)localPwm / 4095.0f);
}

// Records a (lux, pwm) sample into the circular history buffers.
void pushHistory(float lux, uint16_t pwm) {
  yHistory[historyIndex] = (uint16_t)constrain((int)(lux * 100.0f), 0, 65535);
  uHistory[historyIndex] = pwm;
  historyIndex = (historyIndex + 1) % HISTORY_LEN;
  if (historyIndex == 0) {
    historyWrapped = true;
  }
}

// Sets the LED PWM output, clamped to the 12-bit range [0, 4095].
void setLedPwm(uint16_t pwm) {
  localPwm = constrain(pwm, 0, 4095);
  analogWrite(LED_PIN, localPwm);
}

// Selects the active illuminance lower bound based on occupancy state.
void updateCurrentLowerBound() {
  currentLuxLowerBound = (occupancyState == 'h') ? highLuxBound : lowLuxBound;
}

// Updates or creates a peer entry from a received HELLO frame.
// First tries to find an existing entry; if not found, allocates a free slot.
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

// Looks up a peer by node ID.  Returns nullptr if not found.
PeerStatus *findPeer(uint8_t id) {
  for (int i = 0; i < MAX_NODES; ++i) {
    if (peers[i].active && peers[i].nodeId == id) {
      return &peers[i];
    }
  }
  return nullptr;
}

// Marks peers as inactive if no HELLO has been received within PEER_TIMEOUT_MS.
void removeStalePeers() {
  const uint32_t now = millis();
  for (int i = 0; i < MAX_NODES; ++i) {
    if (peers[i].active && (now - peers[i].lastSeenMs > PEER_TIMEOUT_MS)) {
      peers[i].active = false;
    }
  }
}

// ============================================================================
//  SECTION 10: CORE 0 -> CORE 1 TRANSMIT QUEUE & FIFO TRANSPORT
//  Core 0 never touches the MCP2515 SPI bus directly.  Instead, it enqueues
//  CAN frames into a circular buffer.  The service function then serialises
//  them into 3 x 32-bit FIFO words and pushes them to Core 1 nonblockingly.
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
//  SECTION 11: CORE 1 -> CORE 0 RECEIVE QUEUE & FIFO TRANSPORT
//  Core 1 enqueues received CAN frames and diagnostics into its local queue,
//  then serialises them into FIFO words headed for Core 0.
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
//  SECTION 12: HIGH-LEVEL CAN MESSAGE BUILDERS
//  Convenience functions that construct CAN frame payloads for each message
//  type (hello, command, query, reply, stream, calibration plan) and enqueue
//  them for transmission via the Core 0 -> Core 1 FIFO path.
// ============================================================================

// Sends a periodic heartbeat frame containing this node's ID, PWM, lux, and reference.
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

// Sends a command frame to a specific node (or broadcast).
// Payload: [command_type, sender_id, value_hi, value_lo]
void sendSimpleCommand(uint8_t targetNode, CommandType command, uint16_t value = 0) {
  uint8_t payload[4] = {
    (uint8_t)command,
    nodeId,
    (uint8_t)((value >> 8) & 0xFF),
    (uint8_t)(value & 0xFF)
  };
  enqueueTxFrame(CAN_ID_COMMAND_BASE + targetNode, payload, sizeof(payload));
}

// Sends a query request to a remote node and registers it as pending.
void sendQuery(uint8_t targetNode, QueryCode queryCode) {
  uint8_t payload[2] = {(uint8_t)queryCode, nodeId};
  if (enqueueTxFrame(CAN_ID_QUERY_BASE + targetNode, payload, sizeof(payload))) {
    pendingRemoteQuery.active = true;  // Track one outstanding query so timeouts can be reported later.
    pendingRemoteQuery.code = queryCode;
    pendingRemoteQuery.targetNode = targetNode;
    pendingRemoteQuery.requestedAtMs = millis();
  }
}

// Sends a query reply containing a float value back to the requesting node.
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

// Sends a command with a single character value (e.g., occupancy state 'h'/'l'/'o').
void sendCharCommand(uint8_t targetNode, CommandType command, char value) {
  sendSimpleCommand(targetNode, command, (uint16_t)(uint8_t)value);
}

// Sends a real-time stream frame containing a timestamped variable value.
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
//  SECTION 13: CALIBRATION SESSION MANAGEMENT
//  The calibration procedure estimates the static gain matrix K[i][j], which
//  represents how much illuminance node i sees when node j drives its LED.
//
//  Procedure (all nodes synchronised via a CAN plan):
//    1. All LEDs off -> measure baseline illuminance
//    2. For each node j = 1..N:
//       a. Node j turns on at calPwm, all others off
//       b. Wait settle time, then measure illuminance
//       c. Gain K[i][j] = (measured_lux - baseline) / calPwm
//    3. Gains stored locally; exchanged via CAN for distributed algorithms
// ============================================================================

// Clears all calibration measurement data and gain values.
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

// Completes the calibration session: computes gains from slot measurements.
// Gain K[myNode][sourceId] = (slotLux[sourceId] - baseline) / calPwm
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
  Serial.print("Calibration complete in ");
  Serial.print(millis() - calib.startMs);
  Serial.println(" ms");
}

// Starts a new calibration session with the given parameters.
// All nodes start at the same scheduled tick (synchronised via CAN plan).
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

// Schedules repeated broadcasts of the calibration plan for reliability.
// Called by the coordinator to ensure all nodes receive the plan.
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

// ============================================================================
//  SECTION 14: STATE RESET & REPORTING UTILITIES
// ============================================================================

// Resets the local node state while preserving the multicore architecture and
// immediately restarts the distributed wake-up/discovery sequence.
void resetRuntimeState() {
  energyJ = 0.0f;
  visibilityErrorIntegral = 0.0f;
  flickerIntegral = 0.0f;
  restartSeconds = 0.0f;
  metricSampleCount = 0;
  piPrevDuty = 0.0f;
  piPrevPrevDuty = 0.0f;
  cons = ConsensusState();
  cons.mode = ALG_NONE;
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

// Prints the calibration gain matrix row for this node to the serial console.
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

// Dumps the circular history buffer for the given variable ('y' or 'u') to serial.
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

// Returns the current value for a given query code, used both locally and for CAN replies.
// Sets ok=false if the query code is not recognised.
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
    case Q_VIS: return (metricSampleCount > 0) ? (visibilityErrorIntegral / (float)metricSampleCount) : 0.0f;
    case Q_FLICK: return (metricSampleCount > 0) ? (flickerIntegral / (float)metricSampleCount) : 0.0f;
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
//  SECTION 15: CAN PROTOCOL DECODING (Core 0)
//  Received CAN frames are dispatched by ID range to specialised handlers.
//  Each handler validates the payload length, extracts fields, and updates
//  local state or sends a reply as appropriate.
// ============================================================================

// Processes a received CAN command frame (CMD_LED_SET_PWM, CMD_SET_REF, etc.).
// Commands addressed to another node or to broadcast are accepted; others are ignored.
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

// Processes a received calibration plan frame (PLAN_A or PLAN_B).
// Once both halves with matching session IDs arrive, starts a calibration session.
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

// Handles a query request: reads the requested value and sends a reply frame.
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

// Handles a query reply from a remote node: prints the value to serial.
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

// Handles a received real-time stream frame from a remote node: prints to serial.
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

// Master CAN frame dispatcher: routes each frame to the appropriate handler
// based on the CAN identifier range.  Frames received before startup is complete
// are silently discarded.
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

// ============================================================================
//  SECTION 16: FIFO REASSEMBLY FROM CORE 1
//  Reads 32-bit words from the RP2040 FIFO, reassembles them into CAN frames
//  (3 words) or diagnostics (1 word), then dispatches to protocol handlers.
// ============================================================================

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
//  SECTION 17: NONBLOCKING WAKEUP & CALIBRATION STATE MACHINES
//  Both state machines are entirely nonblocking: they check deadlines on each
//  call and only advance when the current phase's time has elapsed.
// ============================================================================

// Accumulates a lux sample during calibration measurement phases.
// Called from the control task at 100 Hz, providing the measurement cadence.
void captureCalibrationSample() {
  if (!calib.active) {
    return;
  }

  if (calib.state == CAL_BASELINE_MEASURE || calib.state == CAL_SLOT_MEASURE) {
    calib.measureAccumulator += filteredLux;  // The control task provides the measurement cadence for calibration averaging.
    calib.measureCount++;
  }
}

// Called when a measurement phase deadline expires.
// Computes the average lux, stores it (baseline or slot), and advances to
// the next calibration phase (next slot or finish).
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

// Advances the calibration state machine by one step if the current phase
// deadline has been reached.  Called from the main loop (nonblocking).
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
      calib.startMs = now;
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
//  SECTION 18: PERIODIC CONTROL TASK & DIAGNOSTICS
//  The 100 Hz control task reads the sensor, runs the PI controller (when
//  feedback is enabled), accumulates performance metrics, and records history.
//  Supporting functions handle hello broadcasts, stream output, and timeouts.
// ============================================================================

// Sends a heartbeat HELLO frame every HELLO_PERIOD_MS and prunes stale peers.
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

// Repeatedly broadcasts the calibration plan at timed intervals for reliability.
// Coordinator-only; decrements a repeat counter until all copies are sent.
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

// Outputs real-time variable data to the serial port at STREAM_PERIOD_MS rate.
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

  // Visibility error: V = (1/N) * sum(max(0, ref - lux))
  float visErr = (refLux > filteredLux) ? (refLux - filteredLux) : 0.0f;
  visibilityErrorIntegral += visErr;

  // Flicker: f_k = |d_k - d_{k-1}| + |d_{k-1} - d_{k-2}| if direction change
  float currentDuty = (float)localPwm / 4095.0f;
  float diff1 = currentDuty - piPrevDuty;
  float diff2 = piPrevDuty - piPrevPrevDuty;
  if (diff1 * diff2 < 0.0f) {
    flickerIntegral += fabsf(diff1) + fabsf(diff2);
  }
  metricSampleCount++;

  pushHistory(filteredLux, localPwm);
  captureCalibrationSample();

  // --- PI Controller with feedforward and anti-windup ---
  //
  // Architecture:  duty = ff_duty + P + I
  //
  //   ff_duty: static feedforward from calibrated gain
  //            ff = (refLux - baseline) / (K[self][self] * 4095)
  //            Provides an immediate open-loop estimate, so the PI only
  //            needs to correct the residual error.
  //
  //   P:       proportional term with set-point weighting
  //            P = Kp * (b * ref - measured)
  //            b=1 gives standard PI; b<1 reduces overshoot on setpoint changes.
  //
  //   I:       integral term with back-calculation anti-windup
  //            I += Ki * dt * error
  //            When the raw output saturates at [0,1], the integrator is
  //            corrected:  I += (dt/Tt) * (clamped - raw)
  //            This prevents the integrator from winding up during saturation
  //            and gives faster recovery when the error changes sign.
  //
  if (feedbackEnabled && refLux > 0.0f && !calib.active) {
    // Feedforward: instant open-loop estimate of duty needed
    float ff_duty = 0.0f;
    float staticGain = calib.gainRow[nodeId] * 4095.0f;  // Convert gain-per-PWM to gain-per-duty
    if (staticGain > 0.01f) {
      ff_duty = (refLux - calib.baselineLux) / staticGain;
    }
    ff_duty = constrain(ff_duty, 0.0f, 1.0f);

    // PI corrects the residual error between reference and measured lux
    float error = refLux - filteredLux;

    // Proportional term with set-point weighting (PI_B_SP = 1.0 -> standard)
    float P = PI_KP * (PI_B_SP * refLux - filteredLux);

    // Integral term: trapezoidal integration at 100 Hz
    piIntegral += PI_KI * PI_DT * error;

    // Anti-windup via back-calculation:
    // If raw output exceeds [0,1], push the integrator back proportionally
    // to the saturation amount, scaled by dt/Tt (tracking time constant).
    if (antiWindupEnabled) {
      float rawOutput = ff_duty + P + piIntegral;
      float clampedOutput = constrain(rawOutput, 0.0f, 1.0f);
      piIntegral += (PI_DT / PI_TT) * (clampedOutput - rawOutput);
    }

    // Compute final output duty = feedforward + PI correction, clamped to [0,1]
    float duty = ff_duty + P + piIntegral;
    duty = constrain(duty, 0.0f, 1.0f);

    // Convert normalised duty [0,1] to 12-bit PWM [0,4095] and apply
    setLedPwm((uint16_t)(duty * 4095.0f));
  }

  // Track duty history for flicker (always, regardless of control mode)
  piPrevPrevDuty = piPrevDuty;
  piPrevDuty = (float)localPwm / 4095.0f;

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

// Checks if the timer ISR has signalled a pending control step, then runs it.
// Uses atomic decrement to avoid race with the ISR.
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

// Periodic diagnostics: checks for timed-out remote queries.
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
//  SECTION 19: SERIAL COMMAND PARSER (User Interface)
//  Parses newline-terminated commands from the USB serial port.
//  See the SERIAL COMMAND QUICK REFERENCE at the top of this file.
//  All parsing is nonblocking: characters are accumulated until '\n'.
// ============================================================================

// Parses "g <var> <node>" and "g b <var> <node>" get/query commands.
// Returns true if the line was recognised as a get command (even if invalid).
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

// Handles serial input during the startup phase (before STARTUP_READY).
// First line sets node ID, second line sets total node count.
void handleStartupLine(const char *line) {
  int value = atoi(line);  // Startup input is intentionally simple: one numeric line at a time.
  if (startupState == STARTUP_WAIT_NODE_ID) {
    if (value < 1 || value > MAX_NODES) {
      Serial.println("Valor invalido. Tenta outra vez.");
      return;
    }
    nodeId = (uint8_t)value;
    B_PARAM = B_PARAM_PER_NODE[nodeId];
    startupState = STARTUP_WAIT_TOTAL_NODES;
    Serial.print("ID configurado: ");
    Serial.print(nodeId);
    Serial.print(" B_PARAM=");
    Serial.println(B_PARAM, 3);
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

// Main command dispatcher for serial input.  Routes to startup handler if not
// ready, otherwise matches the command prefix and executes the corresponding action.
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

  if (strcmp(line, "RM") == 0) {
    energyJ = 0.0f;
    visibilityErrorIntegral = 0.0f;
    flickerIntegral = 0.0f;
    metricSampleCount = 0;
    Serial.println("Metrics reset.");
    return;
  }

  if (strcmp(line, "BOOTSEL") == 0) {
    Serial.println("Rebooting to bootloader...");
    Serial.flush();
    delay(100);
    rp2040.rebootToBootloader();
    return;
  }

  {
    int algVal = 0;
    if (sscanf(line, "A %d", &algVal) == 1) {
    cons.mode = (AlgorithmMode)algVal;
    if ((algVal == ALG_CONSENSUS || algVal == ALG_ADMM || algVal == ALG_DUAL_DECOMP) && calib.gainsReady) {
      cons.gainsExchanged = false;
      cons.gainExchStartMs = 0;
      memset(cons.gainRowReceived, 0, sizeof(cons.gainRowReceived));
      if (algVal == ALG_ADMM) admm = ADMMState();
      if (algVal == ALG_DUAL_DECOMP) dd = DualDecompState();
    } else if (algVal == ALG_NONE) {
      cons.active = false;
      admm.active = false;
      dd.active = false;
    }
    Serial.print("alg=");
    Serial.println(algVal);
    printAck();
    return;
  }}


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
      refLux = currentLuxLowerBound;
      printAck();
      return;
    }
    if (i == nodeId) {
      occupancyState = c1;
      updateCurrentLowerBound();
      refLux = currentLuxLowerBound;
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
      if (feedbackEnabled) { piIntegral = 0.0f; piPrevError = 0.0f; }
      printAck();
      return;
    }
    if (node == nodeId) {
      feedbackEnabled = (intVal != 0);
      if (feedbackEnabled) { piIntegral = 0.0f; piPrevError = 0.0f; }
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
//  SECTION 20: CORE 0 ENTRY POINTS (setup / loop)
//  Core 0 owns all application logic: serial parsing, control loop,
//  calibration, distributed algorithms, and CAN protocol decoding.
//  The main loop is cooperative (nonblocking): each service function checks
//  whether it has work to do and returns immediately if not.
// ============================================================================

// Core 0 setup: initialise serial, ADC, PWM, and start the 100 Hz control timer.
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

// Allow rebooting to bootloader via serial command "BOOTSEL"
void checkSerialReset() {
  // Handled inline — nothing needed here
}

// Core 0 main loop: cooperative round-robin of all service functions.
// Each function returns immediately if it has no pending work.
void loop() {
  checkSerialReset();
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
//  SECTION 21: CORE 1 CAN SERVICE (setup1 / loop1)
//  Core 1 is the sole owner of the MCP2515 SPI peripheral.  It handles:
//    - Initialisation (begin + setMode + interrupt attach)
//    - Transmitting frames received from Core 0 via FIFO
//    - Draining MCP2515 RX buffers on interrupt and forwarding to Core 0
//    - Periodic health checks on the CAN controller
//  Core 1 never accesses any Core 0 application state directly.
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

// Core 1 setup: configure SPI pins, initialise MCP2515 at 500 kbps with 16 MHz
// crystal, set normal mode, and attach the CAN interrupt handler.
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

// Core 1 main loop: service outbound FIFO traffic, drain CAN RX on interrupt,
// check controller health, and forward received frames to Core 0.
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
