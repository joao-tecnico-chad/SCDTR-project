// ============================================================================
// calibration.h
// Calibration session management and state machines
//
// During calibration each node sequentially turns its LED on at a known PWM
// while all nodes measure the resulting illuminance. From the difference
// between the lit and baseline readings each node computes a gain row
// (one entry per source node) that forms the system's coupling matrix K.
//
// The calibration plan is broadcast as two 8-byte CAN frames (Plan A / B)
// and repeated several times for reliability. The wake-up state machine
// handles discovery and auto-calibration on coordinator nodes.
//
// Authors: Joao Rocha, SCDTR course project
// ============================================================================
#pragma once

#include "protocol_defs.h"

// ----- Calibration array helpers -----

inline void resetCalibrationArrays() {
  calib.baselineLux = -1.0f;
  calib.gainsReady = false;
  calib.measureAccumulator = 0.0f;
  calib.measureCount = 0;
  for (int i = 0; i <= MAX_NODES; ++i) {
    calib.slotLux[i] = -1.0f;
    calib.gainRow[i] = 0.0f;
  }
}

inline void enterCalibrationState(CalibrationState state, uint32_t deadlineMs) {
  calib.state = state;
  calib.phaseDeadlineMs = deadlineMs;
  calib.measureAccumulator = 0.0f;
  calib.measureCount = 0;
}

inline void finishCalibration() {
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

// ----- Session start -----

inline void startCalibrationSession(uint16_t sessionId,
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

// ----- Plan broadcast -----

inline void broadcastCalibrationPlan(uint16_t sessionId,
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
    encodeCalTick(settleMs)
  };

  uint8_t planB[8] = {
    CALIB_PLAN_B,
    nodeId,
    (uint8_t)((sessionId >> 8) & 0xFF),
    (uint8_t)(sessionId & 0xFF),
    encodeCalTick(measureMs),
    encodeCalTick(gapMs),
    (uint8_t)((startDelayTick10 >> 8) & 0xFF),
    (uint8_t)(startDelayTick10 & 0xFF)
  };

  enqueueTxFrame(CAN_ID_CALIB_BASE + BROADCAST_NODE, planA, sizeof(planA));
  enqueueTxFrame(CAN_ID_CALIB_BASE + BROADCAST_NODE, planB, sizeof(planB));
}

inline void scheduleCalibrationPlanBroadcast(uint16_t sessionId,
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

// ----- Calibration measurement sampling (called from control step) -----

inline void captureCalibrationSample() {
  if (!calib.active) {
    return;
  }

  if (calib.state == CAL_BASELINE_MEASURE || calib.state == CAL_SLOT_MEASURE) {
    calib.measureAccumulator += filteredLux;
    calib.measureCount++;
  }
}

// ----- Calibration state advancement after measurement phase -----

inline void advanceCalibrationAfterMeasure(uint32_t now) {
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
    setLedPwm((nodeId == calib.currentSlot) ? calib.calPwm : 0);
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

// ----- Nonblocking calibration state machine -----

inline void serviceCalibrationStateMachine() {
  if (!calib.active) {
    return;
  }

  const uint32_t now = millis();

  // State transitions:
  //   WAIT_START_TIME -> BASELINE_SETTLE -> BASELINE_MEASURE
  //     -> (for each source node) SLOT_SETTLE -> SLOT_MEASURE
  //     -> FINISHED
  switch (calib.state) {
    case CAL_WAIT_START_TIME:
      // All nodes wait for the same scheduled tick so they start in sync.
      // This compensates for CAN propagation delay of the plan broadcast.
      if (!tick10Reached(calib.scheduledStartTick10)) {
        return;
      }
      setLedPwm(0);
      calib.startMs = now;
      // Transition: begin settling with all LEDs off to measure ambient
      enterCalibrationState(CAL_BASELINE_SETTLE, now + calib.settleMs);
      break;

    case CAL_BASELINE_SETTLE:
      // Wait for the LDR to settle after turning all LEDs off.
      // Transition: once settled, start averaging ambient readings.
      if (now >= calib.phaseDeadlineMs) {
        enterCalibrationState(CAL_BASELINE_MEASURE, now + calib.measureMs);
      }
      break;

    case CAL_BASELINE_MEASURE:
      // Accumulate LDR samples to compute baseline (ambient) illuminance.
      // Transition: store baseline, then begin first source-node slot.
      if (now >= calib.phaseDeadlineMs) {
        advanceCalibrationAfterMeasure(now);
      }
      break;

    case CAL_SLOT_SETTLE:
      // The current source node has its LED on at calPwm; wait for the
      // LDR reading to stabilise before measuring.
      // Transition: once settled, start averaging the lit readings.
      if (now >= calib.phaseDeadlineMs) {
        enterCalibrationState(CAL_SLOT_MEASURE, now + calib.measureMs);
      }
      break;

    case CAL_SLOT_MEASURE:
      // Accumulate samples with one source LED on; the difference from
      // baseline gives the coupling gain for that source.
      // Transition: advance to next slot, or finish if all slots done.
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

// ----- Wake-up / discovery state machine -----

// The wake-up flow is time-driven rather than blocking: discovery stays open
// for a fixed window, then the coordinator may publish a future calibration
// plan that every node executes independently.
inline void serviceWakeupStateMachine() {
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
        sendSimpleCommand(BROADCAST_NODE, CMD_ANNOUNCE);
      }
      if (now - bootMs >= WAKEUP_WINDOW_MS) {
        wakeupState = WAKEUP_DISCOVERY_STABLE;
      }
      break;

    case WAKEUP_DISCOVERY_STABLE:
      if (isCoordinator && !autoCalibrationTriggered && (now - bootMs >= WAKEUP_WINDOW_MS + 500)) {
        autoCalibrationTriggered = true;
        uint16_t sessionId = (uint16_t)(millis() & 0xFFFF);
        uint16_t startDelayTick10 = encodeCalTick(DEFAULT_START_DELAY_MS);
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

// ----- Calibration plan broadcast service -----

inline void serviceCalibrationPlanBroadcast() {
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
