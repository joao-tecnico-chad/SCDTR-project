// ============================================================================
// hardware_config.h
// Pin definitions, ADC calibration constants, timing constants, CAN ID bases
//
// This module centralises every hardware-dependent constant so board changes
// only require editing one file. It also defines timing intervals used by
// the control loop, CAN heartbeat, calibration protocol, and diagnostics.
//
// Authors: Joao Rocha, SCDTR course project
// ============================================================================
#pragma once

#include <Arduino.h>

// ----- GPIO pin assignments -----
const uint8_t LED_PIN = 15;       // PWM output driving the LED luminaire (GP15 on Pico)
const uint8_t LDR_PIN = 26;       // ADC input from the LDR voltage divider (GP26 = ADC0)
const uint8_t CAN_CS_PIN = 17;    // SPI chip-select for the MCP2515 CAN controller (active-low)
const uint8_t CAN_INT_PIN = 20;   // MCP2515 interrupt output, active-low when a frame arrives

// ----- ADC / LDR calibration -----
// The LDR sits in a voltage divider with R_FIXED. The ADC reads voltage across
// the LDR, which is converted to resistance, then to lux via the Steinhart-like
// log-linear model: log10(lux) = (log10(R_ldr) - B) / M.
const float ADC_REF = 3.3f;        // Pico ADC reference voltage (3.3 V rail)
const float ADC_MAX = 4095.0f;     // 12-bit ADC maximum raw count
const float R_FIXED = 10000.0f;    // Fixed resistor in the LDR voltage divider (10 kohm)
const float M_PARAM = -0.7f;       // Slope of log10(R_ldr) vs log10(lux) curve (negative: resistance drops as lux rises)
const float B_PARAM_DEFAULT = 5.928f; // Default y-intercept of the log-log LDR model (used if per-node not set)
// Per-node B parameter: each LDR has slightly different characteristics.
// Index 0 is unused; indices 1-8 map to nodeId 1-8.
const float B_PARAM_PER_NODE[] = {0.0f, 6.293f, 5.928f, 5.364f, 5.928f, 5.928f, 5.928f, 5.928f, 5.928f};
const float MAX_POWER_W = 1.0f;    // LED power at full duty cycle (1 W), used for energy metric

// ----- Timing intervals (ms) -----
const uint32_t CONTROL_PERIOD_MS = 10;        // PI control loop period (10 ms = 100 Hz sample rate)
const uint32_t HELLO_PERIOD_MS = 1000;        // How often to broadcast a hello heartbeat frame (1 s)
const uint32_t STREAM_PERIOD_MS = 100;        // Interval between streamed lux/duty prints (100 ms = 10 Hz)
const uint32_t WAKEUP_WINDOW_MS = 5000;       // Discovery window: how long to listen for peers at boot (5 s)
const uint32_t WAKEUP_RETRY_MS = 500;         // Retry interval for hello broadcasts during discovery (500 ms)
const uint32_t PEER_TIMEOUT_MS = 8000;        // Mark a peer as stale if no hello received within 8 s
const uint32_t CORE1_HEALTH_PERIOD_MS = 250;  // Core 1 polls MCP2515 error registers every 250 ms
const uint32_t DIAG_PERIOD_MS = 1000;         // General diagnostics check interval (query timeouts, etc.)

// ----- CAN identifier bases -----
// Each message type occupies a range of CAN IDs: base + nodeId.
// Non-overlapping ranges let the receiver dispatch by ID alone.
const uint16_t CAN_ID_HELLO_BASE = 0x100;    // Heartbeat / peer discovery frames
const uint16_t CAN_ID_COMMAND_BASE = 0x300;   // Set commands (PWM, reference, occupancy, etc.)
const uint16_t CAN_ID_CALIB_BASE = 0x400;     // Calibration plan frames (Plan A + Plan B)
const uint16_t CAN_ID_QUERY_BASE = 0x500;     // Remote query request (ask a node for a value)
const uint16_t CAN_ID_REPLY_BASE = 0x580;     // Remote query reply (node responds with a float)
const uint16_t CAN_ID_STREAM_BASE = 0x600;    // Real-time streaming of lux or duty values

// ----- Network limits -----
const uint8_t BROADCAST_NODE = 0x7F;      // Special target address meaning "all nodes" (CAN_ID_xxx_BASE + 0x7F)
const int MAX_NODES = 8;                  // Maximum number of luminaires in the network
const int HISTORY_LEN = 6000;             // Circular buffer: 6000 samples at 100 Hz = 60 s (last minute)
const int SERIAL_BUFFER_LEN = 96;         // Maximum serial command line length in bytes
const int TX_QUEUE_LEN = 24;              // Outbound CAN frame ring buffer depth (Core 0 side)
const int CORE1_EVENT_QUEUE_LEN = 24;     // Inbound CAN frame ring buffer depth (Core 1 -> Core 0)

// ----- Calibration defaults -----
// During calibration each node turns its LED on at a known PWM one at a time.
// All nodes measure the resulting illuminance to build the coupling matrix K.
const uint16_t DEFAULT_CAL_PWM = 2800;           // PWM value each node drives during its calibration slot (~68% duty)
const uint16_t DEFAULT_SETTLE_MS = 250;           // Time (ms) to wait for the LDR to stabilise after an LED change
const uint16_t DEFAULT_MEASURE_MS = 600;          // Time (ms) to average LDR readings during each measurement phase
const uint16_t DEFAULT_GAP_MS = 250;              // Dead time (ms) between turning off one LED and settling the next
const uint16_t DEFAULT_START_DELAY_MS = 1500;     // Delay (ms) between broadcasting the plan and starting calibration
const uint8_t CAL_PLAN_REPEAT_COUNT = 5;          // Number of times the coordinator re-broadcasts the plan (for CAN reliability)
const uint32_t CAL_PLAN_REPEAT_INTERVAL_MS = 60;  // Interval (ms) between repeated plan broadcasts
const uint8_t CAL_TIMEBASE_MS = 10;               // Calibration tick unit: all timing in the plan is in multiples of 10 ms

// ----- PI controller tuning -----
// The controller is a discrete PI with feedforward, set-point weighting,
// and back-calculation anti-windup.
const float PI_KP = 0.01f;    // Proportional gain: how aggressively to react to current error
const float PI_KI = 0.11f;    // Integral gain: how fast to eliminate steady-state error
const float PI_KD = 0.0f;     // Derivative gain: currently unused (pure PI, no D term)
const float PI_B_SP = 1.0f;   // Set-point weighting on P term: b*ref - y (1.0 = full SP weight)
const float PI_TT = 0.15f;    // Anti-windup tracking time constant (smaller = faster unwind of saturated integral)
const float PI_DT = 0.01f;    // Sampling period in seconds (100 Hz = 10 ms)

// ----- Consensus / distributed algorithm constants -----
const int CONS_MAX_NODES = 3;              // Maximum nodes the distributed algorithm supports (array dimension)
const int CONS_MAX_ITER = 50;              // Stop iterating after this many rounds even if not converged
const float CONS_TOL = 1e-3f;             // Convergence tolerance: max change in d_avg between iterations
const float CONS_RHO = 2.0f;              // Consensus penalty weight: higher = faster consensus but more oscillation
const uint32_t CONSENSUS_PERIOD_MS = 100;  // Interval between consensus iterations (100 ms)
const uint16_t CAN_ID_CONSENSUS_BASE = 0x700;  // CAN ID range for consensus duty-vector proposals
const uint16_t CAN_ID_GAINEXCH_BASE = 0x680;   // CAN ID range for gain-exchange frames (K matrix, baseline, L_ref, cost)

// ----- ADMM constants -----
const float ADMM_RHO = 1.0f;              // ADMM augmented Lagrangian penalty parameter
const uint16_t CAN_ID_ADMM_BASE = 0x780;  // CAN ID range for ADMM d/u exchange frames

// ----- Dual decomposition constants -----
const uint16_t CAN_ID_DUAL_BASE = 0x7C0;  // CAN ID range for dual decomposition duty exchange
const float DD_ALPHA_INIT = 0.05f;         // Initial step size for subgradient updates
const float DD_ALPHA_DECAY = 0.995f;       // Geometric decay factor applied to step size each iteration
