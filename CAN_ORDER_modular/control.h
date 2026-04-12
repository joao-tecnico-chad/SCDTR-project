// ============================================================================
// control.h
// PI controller with feedforward, periodic hello, streaming, diagnostics
//
// The control task runs at 100 Hz (CONTROL_PERIOD_MS = 10 ms), triggered by
// a hardware repeating timer. Each step reads the LDR, computes energy and
// quality-of-service metrics, runs the PI + feedforward controller when
// feedback is enabled, and records history samples for later retrieval.
//
// Authors: Joao Rocha, SCDTR course project
// ============================================================================
#pragma once

#include "protocol_defs.h"

// ----- Core control step -----

// Executes exactly one sampled control update whenever the timer callback has
// marked a period as due, and records execution time for overrun detection.
inline void runControlStep() {
  if (startupState != STARTUP_READY) {
    return;
  }

  const uint32_t startUs = micros();
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
  // The controller combines two parts:
  //   1. Feedforward: uses the calibrated static gain to instantly estimate
  //      the duty cycle needed for the target lux, avoiding slow convergence
  //      from a zero starting point.
  //   2. PI feedback: corrects any residual error that the feedforward
  //      misses (e.g. due to external light changes or model mismatch).
  //
  if (feedbackEnabled && refLux > 0.0f && !calib.active) {

    // -- Feedforward --
    // Compute an open-loop duty estimate from the static gain model:
    //   lux = baselineLux + staticGain * duty
    // Solving for duty gives the instant best guess without waiting for
    // the feedback loop to integrate up.  This dramatically reduces the
    // settling time when the reference changes.
    float ff_duty = 0.0f;
    float staticGain = calib.gainRow[nodeId] * 4095.0f;
    if (staticGain > 0.01f) {
      ff_duty = (refLux - calib.baselineLux) / staticGain;
    }
    ff_duty = constrain(ff_duty, 0.0f, 1.0f);

    // -- PI feedback correction --
    // The proportional term reacts to the current error, providing fast
    // transient response.  The integral term accumulates residual error
    // over time to eliminate steady-state offset (which pure P cannot do).
    float error = refLux - filteredLux;

    // Proportional term with set-point weighting (b * ref - y) reduces
    // overshoot on reference steps while still rejecting disturbances.
    float P = PI_KP * (PI_B_SP * refLux - filteredLux);

    // Integral term: accumulates error * dt so steady-state error -> 0
    piIntegral += PI_KI * PI_DT * error;

    // -- Anti-windup (back-calculation) --
    // When the actuator saturates (duty clipped to [0,1]) the integral
    // keeps growing ("winds up"), causing large overshoot when the error
    // reverses.  Back-calculation fixes this by feeding the difference
    // between the raw and clamped output back into the integrator,
    // effectively "unwinding" the excess integral at rate 1/TT (PI_TT=0.15s).
    if (antiWindupEnabled) {
      float rawOutput = ff_duty + P + piIntegral;
      float clampedOutput = constrain(rawOutput, 0.0f, 1.0f);
      piIntegral += (PI_DT / PI_TT) * (clampedOutput - rawOutput);
    }

    // -- Final duty output --
    // Sum the feedforward estimate and the PI correction, then clamp to
    // the valid PWM range [0, 1] before converting to a 12-bit integer.
    float duty = ff_duty + P + piIntegral;
    duty = constrain(duty, 0.0f, 1.0f);

    // Apply the computed duty cycle to the LED hardware
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

inline void serviceControlTask() {
  if (controlDueCount == 0) {
    return;
  }

  noInterrupts();
  uint8_t pending = controlDueCount;
  if (pending > 0) {
    controlDueCount--;
  }
  interrupts();

  if (pending > 1) {
    controlOverrunCount++;
  }

  runControlStep();
}

// ----- Periodic services -----

inline void servicePeriodicHello() {
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

inline void serviceStreamingOutput() {
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

inline void serviceDiagnostics() {
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
