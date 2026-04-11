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
const uint8_t LED_PIN = 15;
const uint8_t LDR_PIN = 26;
const uint8_t CAN_CS_PIN = 17;
const uint8_t CAN_INT_PIN = 20;

// ----- ADC / LDR calibration -----
const float ADC_REF = 3.3f;
const float ADC_MAX = 4095.0f;
const float R_FIXED = 10000.0f;
const float M_PARAM = -0.7f;
const float B_PARAM_DEFAULT = 5.928f;
const float B_PARAM_PER_NODE[] = {0.0f, 6.293f, 5.928f, 5.364f, 5.928f, 5.928f, 5.928f, 5.928f, 5.928f}; // index 0 unused, nodes 1-8
const float MAX_POWER_W = 1.0f;

// ----- Timing intervals (ms) -----
const uint32_t CONTROL_PERIOD_MS = 10;
const uint32_t HELLO_PERIOD_MS = 1000;
const uint32_t STREAM_PERIOD_MS = 100;
const uint32_t WAKEUP_WINDOW_MS = 5000;
const uint32_t WAKEUP_RETRY_MS = 500;
const uint32_t PEER_TIMEOUT_MS = 8000;
const uint32_t CORE1_HEALTH_PERIOD_MS = 250;
const uint32_t DIAG_PERIOD_MS = 1000;

// ----- CAN identifier bases -----
const uint16_t CAN_ID_HELLO_BASE = 0x100;
const uint16_t CAN_ID_COMMAND_BASE = 0x300;
const uint16_t CAN_ID_CALIB_BASE = 0x400;
const uint16_t CAN_ID_QUERY_BASE = 0x500;
const uint16_t CAN_ID_REPLY_BASE = 0x580;
const uint16_t CAN_ID_STREAM_BASE = 0x600;

// ----- Network limits -----
const uint8_t BROADCAST_NODE = 0x7F;
const int MAX_NODES = 8;
const int HISTORY_LEN = 600;
const int SERIAL_BUFFER_LEN = 96;
const int TX_QUEUE_LEN = 24;
const int CORE1_EVENT_QUEUE_LEN = 24;

// ----- Calibration defaults -----
const uint16_t DEFAULT_CAL_PWM = 2800;
const uint16_t DEFAULT_SETTLE_MS = 250;
const uint16_t DEFAULT_MEASURE_MS = 600;
const uint16_t DEFAULT_GAP_MS = 250;
const uint16_t DEFAULT_START_DELAY_MS = 1500;
const uint8_t CAL_PLAN_REPEAT_COUNT = 5;
const uint32_t CAL_PLAN_REPEAT_INTERVAL_MS = 60;
const uint8_t CAL_TIMEBASE_MS = 10;

// ----- PI controller tuning -----
const float PI_KP = 0.01f;
const float PI_KI = 0.11f;
const float PI_KD = 0.0f;
const float PI_B_SP = 1.0f;   // set-point weighting
const float PI_TT = 0.15f;    // anti-windup tracking time constant
const float PI_DT = 0.01f;    // 100 Hz = 10 ms

// ----- Consensus / distributed algorithm constants -----
const int CONS_MAX_NODES = 3;
const int CONS_MAX_ITER = 50;
const float CONS_TOL = 1e-3f;
const float CONS_RHO = 2.0f;
const uint32_t CONSENSUS_PERIOD_MS = 100;
const uint16_t CAN_ID_CONSENSUS_BASE = 0x700;
const uint16_t CAN_ID_GAINEXCH_BASE = 0x680;

// ----- ADMM constants -----
const float ADMM_RHO = 1.0f;
const uint16_t CAN_ID_ADMM_BASE = 0x780;

// ----- Dual decomposition constants -----
const uint16_t CAN_ID_DUAL_BASE = 0x7C0;
const float DD_ALPHA_INIT = 0.05f;
const float DD_ALPHA_DECAY = 0.995f;
