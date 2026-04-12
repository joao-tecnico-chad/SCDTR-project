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

// CommandType: the first byte of every CAN command frame, identifying the action.
// Carried in payload[0] of frames sent to CAN_ID_COMMAND_BASE + targetNode.
enum CommandType : uint8_t {
  CMD_LED_SET_PWM = 0x01,      // Set LED to a specific 12-bit PWM value (payload carries uint16)
  CMD_LED_OFF = 0x02,          // Turn LED off immediately (no extra payload needed)
  CMD_ANNOUNCE = 0x03,         // Request that the receiver broadcasts a hello frame
  CMD_SET_REF = 0x04,          // Set the illuminance reference in lux (encoded as lux*100)
  CMD_RESTART = 0x05,          // Full runtime state reset on the target node
  CMD_SET_HIGH_BOUND = 0x06,   // Set the HIGH-occupancy lux lower bound (encoded as lux*100)
  CMD_SET_LOW_BOUND = 0x07,    // Set the LOW-occupancy lux lower bound (encoded as lux*100)
  CMD_SET_COST = 0x08,         // Set the energy cost coefficient (encoded as cost*100)
  CMD_SET_OCCUPANCY = 0x09,    // Set occupancy state: 'l' (low), 'h' (high), or 'o' (off)
  CMD_SET_ANTI_WINDUP = 0x0A,  // Enable (1) or disable (0) PI anti-windup
  CMD_SET_FEEDBACK = 0x0B,     // Enable (1) or disable (0) PI feedback control
  CMD_STREAM_START = 0x0C,     // Start streaming a variable ('y' for lux, 'u' for duty)
  CMD_STREAM_STOP = 0x0D       // Stop streaming the specified variable
};

// CalibType: marker byte in calibration plan frames.
// The calibration plan is split into two 8-byte CAN frames (A and B) because
// the full plan exceeds one CAN frame's 8-byte maximum payload.
enum CalibType : uint8_t {
  CALIB_PLAN_A = 0x10,  // First half: session ID, cal PWM, total nodes, settle time
  CALIB_PLAN_B = 0x11   // Second half: measure time, gap time, scheduled start tick
};

// QueryCode: ASCII character identifying which value to read from a remote node.
// Used in both serial "g <code> <node>" commands and CAN query/reply frames.
enum QueryCode : uint8_t {
  Q_U = 'u',            // Current LED PWM value (0-4095)
  Q_R = 'r',            // Current illuminance reference (lux)
  Q_Y = 'y',            // Current filtered illuminance reading (lux)
  Q_V = 'v',            // Last raw LDR voltage (V)
  Q_OCC = 'o',          // Occupancy state character ('l', 'h', or 'o')
  Q_AW = 'a',           // Anti-windup enabled flag (1 or 0)
  Q_FB = 'f',           // Feedback enabled flag (1 or 0)
  Q_D = 'd',            // Background/baseline lux (o_i from calibration)
  Q_P = 'p',            // Instantaneous power consumption (W)
  Q_T = 't',            // Uptime since last restart (seconds)
  Q_E = 'E',            // Accumulated energy consumption (J)
  Q_VIS = 'V',          // Average visibility error: mean(max(0, ref - lux))
  Q_FLICK = 'F',        // Average flicker metric (duty direction changes)
  Q_REF_HIGH = 'O',     // HIGH-occupancy lux lower bound
  Q_REF_LOW = 'U',      // LOW-occupancy lux lower bound
  Q_REF_CURRENT = 'L',  // Currently active lux lower bound (depends on occupancy)
  Q_COST = 'C'          // Energy cost coefficient for distributed optimisation
};

// StartupState: serial startup sequence before the node can operate.
// The user must type the node ID and total node count over serial.
enum StartupState : uint8_t {
  STARTUP_WAIT_NODE_ID = 0,      // Waiting for the user to enter this node's ID (1-8)
  STARTUP_WAIT_TOTAL_NODES = 1,  // Waiting for the user to enter the total number of nodes in the network
  STARTUP_READY = 2              // Configuration complete, all subsystems are active
};

// WakeupState: post-startup discovery and auto-calibration state machine.
// Runs after STARTUP_READY; discovers peers, then the coordinator triggers calibration.
enum WakeupState : uint8_t {
  WAKEUP_BOOT = 0,              // Initial state: triggers resetRuntimeState() and enters discovery
  WAKEUP_DISCOVERY_OPEN = 1,    // Broadcasting hellos and announcements to find other nodes
  WAKEUP_DISCOVERY_STABLE = 2,  // Discovery window closed; coordinator may auto-start calibration
  WAKEUP_RUN = 3                // Normal operation: hello heartbeats, control loop, algorithms
};

// CalibrationState: FSM states for the sequential calibration protocol.
// Flow: IDLE -> WAIT_START_TIME -> BASELINE_SETTLE -> BASELINE_MEASURE
//       -> (for each node) SLOT_SETTLE -> SLOT_MEASURE -> FINISHED
enum CalibrationState : uint8_t {
  CAL_IDLE = 0,               // No calibration in progress
  CAL_WAIT_START_TIME = 1,    // Waiting for the scheduled 10 ms tick to arrive (synchronises all nodes)
  CAL_BASELINE_SETTLE = 2,    // All LEDs off; waiting for LDR to settle before measuring ambient
  CAL_BASELINE_MEASURE = 3,   // Averaging LDR readings to determine baseline (ambient) illuminance
  CAL_SLOT_SETTLE = 4,        // One source LED turned on; waiting for LDR to settle
  CAL_SLOT_MEASURE = 5,       // Averaging LDR readings with one source LED on
  CAL_FINISHED = 6            // Calibration done; gains have been computed
};

// FifoMessageKind: type tag in the first FIFO word, identifying what follows.
// The RP2040 hardware FIFO carries 32-bit words between cores; the first byte
// of the first word tells the receiver how to interpret the remaining words.
enum FifoMessageKind : uint8_t {
  FIFO_TX_FRAME = 0x01,  // Core 0 -> Core 1: a CAN frame to transmit (3 words total)
  FIFO_RX_FRAME = 0x02,  // Core 1 -> Core 0: a received CAN frame (3 words total)
  FIFO_DIAG = 0x03        // Core 1 -> Core 0: a diagnostic event (1 word only)
};

// Core1DiagStage: identifies where in Core 1's CAN pipeline an error occurred.
// Packed into a single FIFO_DIAG word so Core 0 can log it to serial.
enum Core1DiagStage : uint8_t {
  DIAG_CAN_INIT = 1,       // MCP2515 begin() failed during Core 1 setup
  DIAG_CAN_MODE = 2,       // setMode(MCP_NORMAL) failed during Core 1 setup
  DIAG_CAN_SEND = 3,       // sendMsgBuf() returned an error when transmitting
  DIAG_CAN_CHECK = 4,      // checkReceive() returned unexpected status (not MSGAVAIL/NOMSG)
  DIAG_CAN_READ = 5,       // readMsgBuf() failed to read a pending message
  DIAG_CAN_HEALTH = 6,     // Periodic checkError() detected a CAN controller fault
  DIAG_CAN_FIFO_DROP = 7   // Core 1 RX queue full: an inbound frame was dropped
};

// AlgorithmMode: selects which distributed optimisation algorithm to run.
// Set via the serial "A <n>" command. ALG_NONE means only local PI control.
enum AlgorithmMode : uint8_t {
  ALG_NONE = 0,         // No distributed algorithm; local PI + feedforward only
  ALG_CONSENSUS = 1,    // Primal-averaging consensus (solves local QP, averages proposals)
  ALG_ADMM = 2,         // Alternating Direction Method of Multipliers (d-z-u updates)
  ALG_DUAL_DECOMP = 3   // Dual decomposition (primal gradient + subgradient dual ascent)
};

// ----- Data structures -----

// PeerStatus: tracks the last known state of a remote node, updated by hello frames.
// The peers[] array holds one entry per discovered peer (up to MAX_NODES).
struct PeerStatus {
  bool active = false;          // true if this slot holds a valid peer
  uint8_t nodeId = 0;           // CAN network node ID (1-8) of the remote peer
  uint16_t pwm = 0;             // Last reported LED PWM value from this peer
  float lux = 0.0f;             // Last reported filtered lux reading from this peer
  float refLux = 0.0f;          // Last reported illuminance reference of this peer
  uint32_t lastSeenMs = 0;      // millis() timestamp of the last hello frame received from this peer
};

// StreamState: tracks whether this node is periodically printing a variable to serial.
struct StreamState {
  bool active = false;     // true if streaming is currently enabled
  char variable = 0;       // 'y' for lux or 'u' for duty; 0 if inactive
};

// CanFrame: represents a single CAN 2.0A frame (standard 11-bit ID, up to 8 data bytes).
// Used in both TX and RX queues and as the unit of inter-core FIFO transfer.
struct CanFrame {
  uint16_t id = 0;         // 11-bit standard CAN identifier
  uint8_t len = 0;         // Data length code (0-8 bytes)
  uint8_t data[8] = {0};   // Payload bytes (unused bytes are zero-filled)
};

// FifoTxStaging: holds a partially-sent multi-word FIFO message.
// A CAN frame requires 3 words (header + 4 data bytes + 4 data bytes).
// A diagnostic requires only 1 word. The staging buffer lets non-blocking
// FIFO writes resume where they left off if the FIFO is momentarily full.
struct FifoTxStaging {
  bool active = false;         // true if a message is currently being pushed word-by-word
  uint32_t words[3] = {0};    // Up to 3 packed 32-bit words to send through the hardware FIFO
  uint8_t wordCount = 0;      // Total number of words in this message (1 for diag, 3 for frame)
  uint8_t index = 0;          // Index of the next word to push (resumes after a failed push)
};

// CalibrationPlanBuffer: accumulates the two-part calibration plan (Plan A + Plan B)
// received over CAN. A session only starts once both halves with matching sessionId arrive.
struct CalibrationPlanBuffer {
  bool hasA = false;               // true once Plan A frame has been received
  bool hasB = false;               // true once Plan B frame has been received
  uint16_t sessionId = 0;          // Unique session identifier; must match between A and B
  uint16_t calPwm = 0;             // PWM value each source node will drive during its slot
  uint8_t totalNodes = 0;          // Number of nodes participating in the calibration
  uint16_t settleMs = 0;           // Settle time in ms (decoded from 10 ms ticks)
  uint16_t measureMs = 0;          // Measurement averaging time in ms
  uint16_t gapMs = 0;              // Dead time between slots in ms
  uint16_t startDelayTick10 = 0;   // Delay from now until calibration starts (in 10 ms ticks)
  uint8_t coordinatorNode = 0;     // Node ID of the node that initiated the calibration
};

// CalibrationContext: full runtime state for an active calibration session on this node.
// Tracks the FSM state, timing, accumulated measurements, and final computed gains.
struct CalibrationContext {
  CalibrationState state = CAL_IDLE;  // Current state in the calibration FSM
  bool active = false;                // true while a calibration session is running
  uint16_t sessionId = 0;            // Identifies the current session (to ignore stale plans)
  uint16_t calPwm = DEFAULT_CAL_PWM; // PWM value driven by the source node during its slot
  uint8_t totalNodes = 0;            // Number of nodes in this calibration session
  uint16_t settleMs = DEFAULT_SETTLE_MS;    // Settle time per phase (ms)
  uint16_t measureMs = DEFAULT_MEASURE_MS;  // Measurement averaging time per phase (ms)
  uint16_t gapMs = DEFAULT_GAP_MS;          // Gap between slots (ms)
  uint16_t scheduledStartTick10 = 0; // Absolute 10 ms tick at which calibration should begin
  uint32_t phaseDeadlineMs = 0;      // millis() deadline for the current FSM phase
  int currentSlot = -1;              // Which source node (1-based) is currently being measured; -1 = baseline
  float baselineLux = -1.0f;         // Measured ambient illuminance with all LEDs off (-1 = not yet measured)
  float slotLux[MAX_NODES + 1] = {0};  // Measured lux for each source node slot (index = source nodeId)
  float gainRow[MAX_NODES + 1] = {0};  // Computed gain row: gainRow[j] = (slotLux[j] - baseline) / calPwm
  bool gainsReady = false;            // true once gains have been computed after calibration finishes
  float measureAccumulator = 0.0f;    // Running sum of lux samples during a measurement phase
  uint16_t measureCount = 0;          // Number of samples accumulated so far in the current phase
  uint32_t startMs = 0;              // millis() when the calibration session actually started
};

// PendingRemoteQuery: tracks an outstanding query sent to another node over CAN.
// Only one query can be pending at a time. A timeout fires if no reply arrives.
struct PendingRemoteQuery {
  bool active = false;             // true while waiting for a reply
  QueryCode code = Q_U;            // Which value was requested
  uint8_t targetNode = 0;          // Node ID of the remote node being queried
  uint32_t requestedAtMs = 0;      // millis() when the query was sent (for timeout detection)
};

// CalibrationBroadcastState: coordinator-side state for repeatedly broadcasting
// the calibration plan. CAN is unreliable, so the plan is sent multiple times.
struct CalibrationBroadcastState {
  bool active = false;                      // true while repeats are still pending
  uint16_t sessionId = 0;                   // Session ID to include in every broadcast
  uint16_t calPwm = DEFAULT_CAL_PWM;        // PWM value for the calibration plan
  uint8_t totalNodes = 1;                   // Number of nodes in the plan
  uint16_t settleMs = DEFAULT_SETTLE_MS;    // Settle time to encode in the plan
  uint16_t measureMs = DEFAULT_MEASURE_MS;  // Measure time to encode in the plan
  uint16_t gapMs = DEFAULT_GAP_MS;          // Gap time to encode in the plan
  uint16_t startDelayTick10 = 0;            // Scheduled start delay in 10 ms ticks
  uint8_t repeatsRemaining = 0;             // How many more times to send the plan
  uint32_t nextSendMs = 0;                  // millis() at which to send the next repeat
};

// ----- Consensus state (shared by ADMM and Dual Decomp via cons.K, cons.o, etc.) -----
//
// This struct holds the system model (K, o, L_ref, c) that ALL three distributed
// algorithms need, plus the consensus-specific iteration variables (d, d_avg, d_others).
// ADMM and Dual Decomp read cons.K, cons.o, cons.L_ref, cons.c but maintain their
// own primal/dual variables in ADMMState / DualDecompState.
struct ConsensusState {
  AlgorithmMode mode = ALG_NONE;  // Currently selected distributed algorithm
  bool active = false;            // true while the consensus algorithm is iterating
  bool converged = false;         // true when max change in d_avg < CONS_TOL
  bool gainsExchanged = false;    // true once all nodes have shared their K rows, baselines, refs, and costs
  int iteration = 0;              // Current iteration counter
  int numNodes = 0;               // Number of nodes participating (= totalNodes)
  int myIndex = 0;                // This node's 0-based index in the K/d/o arrays (= nodeId - 1)

  // --- System model (filled during gain exchange after calibration) ---
  float K[CONS_MAX_NODES][CONS_MAX_NODES]; // Coupling matrix: K[i][j] = lux at node i per unit duty at node j
  float o[CONS_MAX_NODES];                  // Baseline (ambient) lux at each node with all LEDs off
  float L_ref[CONS_MAX_NODES];              // Illuminance lower bound for each node (from occupancy settings)
  float c[CONS_MAX_NODES];                  // Energy cost coefficient for each node

  // --- Consensus-specific iteration variables ---
  float d[CONS_MAX_NODES];                                // This node's current duty proposal vector
  float d_avg[CONS_MAX_NODES];                            // Running average of all nodes' proposals
  float d_others[CONS_MAX_NODES][CONS_MAX_NODES];         // d vectors received from other nodes: d_others[senderIdx][j]
  bool received[CONS_MAX_NODES];                           // Tracks which peers have sent their proposal this iteration

  uint32_t lastIterMs = 0;   // millis() of the last iteration (rate-limits to CONSENSUS_PERIOD_MS)
  uint8_t iterByte = 0;      // Iteration counter byte sent in CAN frames (wraps at 255)

  // --- Gain exchange tracking ---
  bool gainRowReceived[CONS_MAX_NODES]; // true for each node whose gain row, baseline, ref, and cost have arrived
  uint32_t gainExchStartMs = 0;         // millis() when gain exchange started (times out after 5 s)
};

// ----- ADMM state -----
// ADMM decomposes the optimisation into three steps per iteration:
//   d-update (local primal), z-update (global projection), u-update (dual ascent).
struct ADMMState {
  float d[CONS_MAX_NODES];         // Primal variable: each node's own duty proposal
  float z[CONS_MAX_NODES];         // Global consensus variable: agreed-upon duty vector (feasible)
  float u[CONS_MAX_NODES];         // Scaled dual variable: accumulates (d - z) residuals
  float d_others[CONS_MAX_NODES];  // d[i] values received from other nodes this iteration
  bool received[CONS_MAX_NODES];   // Tracks which peers have sent their d/u this iteration
  int iteration;                   // Current ADMM iteration counter
  bool converged;                  // true when primal residual ||d - z|| < CONS_TOL
  bool active;                     // true while the ADMM algorithm is running
  uint32_t lastIterMs;             // millis() of the last iteration for rate-limiting
};

// ----- Dual decomposition state -----
// Dual decomposition uses Lagrange multipliers (lambda) to enforce the shared
// illuminance constraints. Each iteration: primal gradient descent, then dual
// subgradient ascent with a decaying step size.
struct DualDecompState {
  float d[CONS_MAX_NODES];         // Primal variable: each node's duty (d[myIndex] is local, others from CAN)
  float lambda[CONS_MAX_NODES];    // Lagrange multipliers for each illuminance constraint (one per node)
  float d_others[CONS_MAX_NODES];  // d values received from other nodes this iteration
  bool received[CONS_MAX_NODES];   // Tracks which peers have sent their d this iteration
  float alpha;                     // Current step size (decays by DD_ALPHA_DECAY each iteration)
  int iteration;                   // Current dual decomposition iteration counter
  bool converged;                  // true when max constraint violation < CONS_TOL
  bool active;                     // true while the dual decomposition algorithm is running
  uint32_t lastIterMs;             // millis() of the last iteration for rate-limiting
};

// ============================================================================
// Extern declarations of global variables (defined in CAN_ORDER_modular.ino)
// ============================================================================

extern MCP_CAN canBus;                   // MCP2515 CAN controller driver instance (used only by Core 1)
extern repeating_timer_t controlTimer;   // RP2040 hardware timer triggering the 100 Hz control loop

// --- Peer table and high-level protocol state ---
extern PeerStatus peers[MAX_NODES];               // Table of discovered remote nodes
extern StreamState streamState;                    // Whether this node is streaming data to serial
extern CalibrationPlanBuffer pendingPlan;          // Accumulates Plan A + Plan B before starting calibration
extern CalibrationContext calib;                   // Full calibration session state and results
extern PendingRemoteQuery pendingRemoteQuery;      // Outstanding remote query waiting for a CAN reply
extern CalibrationBroadcastState calibBroadcast;   // Coordinator's repeated plan broadcast state

// --- Inter-core TX queue (Core 0 -> Core 1) ---
extern CanFrame txQueue[TX_QUEUE_LEN];   // Circular buffer of outbound CAN frames
extern uint8_t txQueueHead;              // Index of the next frame to dequeue (consumer pointer)
extern uint8_t txQueueTail;              // Index where the next frame will be enqueued (producer pointer)
extern uint8_t txQueueCount;             // Number of frames currently in the queue
extern FifoTxStaging core0ToCore1Staging; // Staging buffer for partially-pushed FIFO words (Core 0 side)

// --- Node identity and startup ---
extern uint8_t nodeId;                    // This node's ID (1-8), set during serial startup
extern uint8_t totalNodes;                // Total number of nodes in the network
extern bool isCoordinator;                // true if this node is node 1 (responsible for auto-calibration)
extern StartupState startupState;         // Current phase of the serial startup sequence
extern WakeupState wakeupState;           // Current phase of the discovery/wakeup FSM
extern bool autoCalibrationTriggered;     // Prevents the coordinator from triggering calibration more than once

// --- Sensor, actuator, and control parameters ---
extern float B_PARAM;                    // Active LDR log-log model intercept (selected per nodeId at startup)
extern float filteredLux;                // Latest low-pass-filtered illuminance reading (lux)
extern float lastLdrVoltage;             // Latest raw LDR voltage (V) before lux conversion
extern uint16_t localPwm;               // Current LED PWM output (0-4095)
extern float refLux;                     // Target illuminance reference (lux), set by user or algorithm
extern char occupancyState;              // 'o' (off/default), 'l' (low), or 'h' (high occupancy)
extern bool antiWindupEnabled;           // Whether PI anti-windup back-calculation is active
extern bool feedbackEnabled;             // Whether the PI feedback controller is active
extern float highLuxBound;              // Lux lower bound when occupancy is 'h' (high)
extern float lowLuxBound;               // Lux lower bound when occupancy is 'l' (low)
extern float currentLuxLowerBound;      // Currently active lower bound (depends on occupancyState)
extern float energyCost;                 // Cost coefficient c[i] for distributed optimisation

// --- Quality-of-service metrics (accumulated since last reset) ---
extern float energyJ;                    // Cumulative energy consumed (Joules)
extern float visibilityErrorIntegral;    // Sum of max(0, ref - lux) at each sample
extern float flickerIntegral;            // Sum of duty direction-change magnitudes
extern float restartSeconds;             // Seconds elapsed since last restart
extern uint32_t metricSampleCount;       // Number of control samples since last metrics reset

// --- PI controller internal state ---
extern float piIntegral;                 // Accumulated integral term of the PI controller
extern float piPrevError;                // Previous error (unused in current PI, reserved for D term)
extern float piPrevDuty;                 // Previous duty cycle (for flicker detection)
extern float piPrevPrevDuty;             // Duty two samples ago (for flicker detection: direction change)

// --- Distributed algorithm state ---
extern ConsensusState cons;              // Consensus algorithm state (also holds shared K, o, L_ref, c)
extern ADMMState admm;                   // ADMM algorithm state
extern DualDecompState dd;               // Dual decomposition algorithm state

// --- History ring buffers for "g b y/u" command ---
extern uint16_t yHistory[HISTORY_LEN];   // Lux history (stored as lux * 100, uint16)
extern uint16_t uHistory[HISTORY_LEN];   // PWM history (raw 0-4095 values)
extern int historyIndex;                 // Next write position in the ring buffer
extern bool historyWrapped;              // true once the buffer has filled and wrapped around

// --- Serial input buffer ---
extern char serialBuffer[SERIAL_BUFFER_LEN]; // Accumulates characters until newline
extern size_t serialLength;                   // Current number of characters in the buffer

// --- Timing bookkeeping ---
extern uint32_t bootMs;         // millis() at last restart (used to compute uptime)
extern uint32_t lastHelloMs;    // millis() of the last hello broadcast
extern uint32_t lastWakeupMs;   // millis() of the last wakeup retry
extern uint32_t lastStreamMs;   // millis() of the last streaming output
extern uint32_t lastDiagMs;     // millis() of the last diagnostics check

// --- Error counters (for "g diag" command) ---
extern uint32_t canTxErrorCount;       // Number of CAN transmit failures
extern uint32_t canRxErrorCount;       // Number of CAN receive errors
extern uint32_t canProtocolErrorCount; // Number of malformed or unrecognised CAN frames
extern uint32_t controlOverrunCount;   // Number of times the control step exceeded its period
extern uint32_t controlMaxExecUs;      // Worst-case control step execution time (microseconds)
extern uint32_t controlLastExecUs;     // Most recent control step execution time (microseconds)

// --- ISR-shared flags (volatile because set in ISR / timer callback) ---
extern volatile uint8_t controlDueCount; // Saturating counter: incremented by timer ISR, consumed by Core 0 loop
extern volatile bool canIrqPending;      // Set by CAN interrupt handler; cleared by Core 1 after draining RX

// --- Inter-core RX queue (Core 1 -> Core 0) ---
extern CanFrame core1EventQueue[CORE1_EVENT_QUEUE_LEN]; // Circular buffer of received CAN frames
extern uint8_t core1EventHead;           // Consumer pointer (Core 0 reads from here)
extern uint8_t core1EventTail;           // Producer pointer (Core 1 writes here)
extern uint8_t core1EventCount;          // Number of frames currently in the queue
extern FifoTxStaging core1ToCore0Staging; // Staging buffer for partially-pushed FIFO words (Core 1 side)
extern uint32_t core1LastHealthMs;       // millis() of Core 1's last MCP2515 health check

// ============================================================================
// Forward declarations of functions used across modules
//
// These are needed because the .h files are included in a fixed order but
// reference each other's functions. The actual implementations are in the
// corresponding .h files as inline functions.
// ============================================================================

// --- can_transport.h: TX queue ---
bool enqueueTxFrame(uint16_t canId, const uint8_t *payload, uint8_t length); // Add a CAN frame to the Core 0 -> Core 1 TX queue

// --- sensing.h: sensor and actuator ---
float readLuxFiltered();          // Read ADC, oversample 32x, convert to lux, apply low-pass filter
float getInstantPowerW();         // Estimate current power from PWM duty cycle
void setLedPwm(uint16_t pwm);    // Clamp and write PWM to the LED pin
void pushHistory(float lux, uint16_t pwm);   // Record a lux/duty sample into the ring buffer
void updateCurrentLowerBound();   // Recompute currentLuxLowerBound from occupancy state
void updatePeer(uint8_t senderId, uint16_t pwm, float lux, float peerRef); // Update or create a peer table entry
PeerStatus *findPeer(uint8_t id); // Look up a peer by nodeId; returns nullptr if not found
void removeStalePeers();          // Mark peers as inactive if no hello received within PEER_TIMEOUT_MS

// --- can_transport.h: message builders ---
void sendHelloFrame();            // Broadcast a hello heartbeat with this node's current state
void sendSimpleCommand(uint8_t targetNode, CommandType command, uint16_t value = 0); // Send a 4-byte command frame
void sendQuery(uint8_t targetNode, QueryCode queryCode);   // Send a remote query and arm the pending-reply tracker
void sendReply(uint8_t requesterNode, QueryCode queryCode, float value); // Respond to a remote query with a float value
void sendCharCommand(uint8_t targetNode, CommandType command, char value); // Convenience: send a command with a char value
void sendStreamFrame(char variable);       // Send a streaming data frame over CAN
uint8_t encodeCalTick(uint16_t msValue);   // Convert milliseconds to 10 ms tick units (fits in 1 byte)
uint16_t nowTick10();                      // Current time as a 16-bit value in 10 ms ticks
bool tick10Reached(uint16_t targetTick);   // Check if the current tick has reached (or passed) the target

// --- calibration.h ---
void broadcastCalibrationPlan(uint16_t sessionId, uint16_t calPwm, uint8_t planNodes,
                              uint16_t settleMs, uint16_t measureMs, uint16_t gapMs,
                              uint16_t startDelayTick10);   // Send Plan A + Plan B frames to all nodes
void scheduleCalibrationPlanBroadcast(uint16_t sessionId, uint16_t calPwm, uint8_t planNodes,
                                      uint16_t settleMs, uint16_t measureMs, uint16_t gapMs,
                                      uint16_t startDelayTick10); // Schedule repeated plan broadcasts
void startCalibrationSession(uint16_t sessionId, uint16_t calPwm, uint16_t settleMs,
                             uint16_t measureMs, uint16_t gapMs,
                             uint16_t scheduledStartTick10, uint8_t planNodes); // Initialise and start a calibration session
void captureCalibrationSample();           // Called every control step to accumulate lux during measurement phases
void serviceCalibrationStateMachine();     // Advance the calibration FSM based on time deadlines
void serviceWakeupStateMachine();          // Advance the discovery/auto-calibration FSM
void serviceCalibrationPlanBroadcast();    // Send the next plan repeat if due
void finishCalibration();                  // Compute gains from measured data and mark calibration complete

// --- consensus.h ---
void initConsensus();              // Initialise consensus iteration from calibration data
void sendGainRow();                // Broadcast this node's K row, baseline, L_ref, and cost over CAN
void serviceConsensus();           // Run gain exchange and consensus iterations at the configured rate
void handleConsensusFrame(uint8_t len, const uint8_t *data);     // Process a received consensus proposal
void handleGainExchangeFrame(uint8_t len, const uint8_t *data);  // Process a received gain exchange frame
bool allGainsReceived();           // Check if all nodes' gain rows have arrived

// --- admm.h ---
void initADMM();                   // Initialise ADMM from feedforward using the shared K matrix
void serviceADMM();                // Run d-z-u updates at the configured rate
void handleADMMFrame(uint8_t len, const uint8_t *data); // Process a received ADMM d/u frame

// --- dual_decomp.h ---
void initDualDecomp();             // Initialise dual decomposition from feedforward
void serviceDualDecomp();          // Run primal/dual updates at the configured rate
void handleDualDecompFrame(uint8_t len, const uint8_t *data); // Process a received dual decomp d frame

// --- serial_ui.h ---
void resetRuntimeState();          // Reset all runtime state and restart discovery
void printCalibrationReport();     // Print the gain matrix row and baseline to serial
float readQueryValue(QueryCode queryCode, bool &ok); // Read a local value by QueryCode
void handleLineCommand(const char *line);    // Parse and execute a complete serial command line
void serviceSerialNonblocking();   // Accumulate serial characters and dispatch complete lines

// --- control.h ---
void runControlStep();             // Execute one PI control update, read sensor, update metrics
void serviceControlTask();         // Check if a control step is due and run it
void servicePeriodicHello();       // Send periodic hello heartbeats and prune stale peers
void serviceStreamingOutput();     // Print streamed variable to serial at STREAM_PERIOD_MS
void serviceDiagnostics();         // Check for query timeouts and other diagnostic conditions

// --- can_transport.h: FIFO helpers ---
uint32_t packFrameHeader(FifoMessageKind kind, uint16_t canId, uint8_t length); // Pack FIFO word 0: [kind:8][canId:16][len:8]
uint32_t packBytes32(const uint8_t *bytes);              // Pack 4 bytes into a uint32 (little-endian)
void unpackBytes32(uint32_t word, uint8_t *bytes);       // Unpack a uint32 into 4 bytes
uint32_t packDiagWord(Core1DiagStage stage, uint8_t code, uint8_t detail); // Pack a diagnostic into one FIFO word
void serviceCore0ToCore1Fifo();    // Push queued TX frames from Core 0 to Core 1 via hardware FIFO
bool dequeueTxFrame(CanFrame &frame);  // Pop the next frame from the TX queue
void prepareFifoFrame(FifoTxStaging &staging, FifoMessageKind kind, const CanFrame &frame); // Stage a 3-word FIFO message
void prepareFifoDiag(FifoTxStaging &staging, Core1DiagStage stage, uint8_t code, uint8_t detail); // Stage a 1-word diag
bool enqueueCore1EventFrame(const CanFrame &frame);  // Add a received frame to the Core 1 -> Core 0 queue
bool dequeueCore1EventFrame(CanFrame &frame);         // Pop the next frame from the RX queue
void queueCore1Diag(Core1DiagStage stage, uint8_t code, uint8_t detail); // Queue a diagnostic from Core 1
void serviceCore1ToCore0Fifo();    // Push queued RX frames and diagnostics from Core 1 to Core 0

// --- can_protocol.h ---
void handleReceivedCanFrame(const CanFrame &frame); // Top-level CAN ID dispatcher: routes to the correct handler
void serviceFifoFromCore1();       // Reassemble FIFO words from Core 1 into frames/diags and dispatch

// --- Utility helpers ---
void printAck();                   // Print "ack" to serial (command acknowledgement)
void printErr();                   // Print "err" to serial (command error)
void reportProtocolError(const char *stage, uint32_t canId, uint8_t length); // Log a CAN protocol error
void reportCore1Diag(Core1DiagStage stage, uint8_t code, uint8_t detail);    // Log a Core 1 diagnostic event
