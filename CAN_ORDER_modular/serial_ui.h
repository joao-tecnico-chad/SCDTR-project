// ============================================================================
// serial_ui.h
// Serial command parsing, startup sequence, and runtime state reset
//
// All user interaction happens over USB-serial. During startup the node
// collects its ID and total node count. Once STARTUP_READY, the parser
// accepts get/set commands for PWM, reference, occupancy, bounds, cost,
// anti-windup, feedback, streaming, calibration, algorithm selection, and
// diagnostics. Parsing is nonblocking: characters accumulate until newline.
//
// Authors: Joao Rocha, Ricardo Gaspar, Diogo Costa — SCDTR 2025/2026
// ============================================================================
#pragma once

#include "protocol_defs.h"

// ----- Query value reader -----

inline float readQueryValue(QueryCode queryCode, bool &ok) {
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

// ----- Runtime state reset -----

// Resets the local node state while preserving the multicore architecture and
// immediately restarts the distributed wake-up/discovery sequence.
inline void resetRuntimeState() {
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

// ----- Calibration report -----

inline void printCalibrationReport() {
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

// ----- Buffer printing -----

inline void printBuffer(char variable) {
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

// ----- Get command parser -----

inline bool parseGetCommand(const char *line) {
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
        Serial.print(occupancyState);
        printQueryLabel('o');
        Serial.println();
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
        Serial.print(value, 0);
      } else if (a == 'a' || a == 'f') {
        Serial.print((int)value);
      } else if (a == 'E' || a == 'V' || a == 'F') {
        Serial.print(value, 6);  // 6 decimal places for metrics
      } else {
        Serial.print(value, 3);
      }
      printQueryLabel(a);
      Serial.println();
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

// ----- Startup line handler -----

inline void handleStartupLine(const char *line) {
  int value = atoi(line);
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

// ----- Main command handler -----

inline void handleLineCommand(const char *line) {
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

  if (strcmp(line, "help") == 0 || strcmp(line, "h") == 0 || strcmp(line, "?") == 0) {
    Serial.println("=== SERIAL COMMAND REFERENCE ===");
    Serial.println("--- Setting values ---");
    Serial.println("  u <node> <pwm>     Set LED PWM (0-4095)");
    Serial.println("  r <node> <lux>     Set illuminance reference");
    Serial.println("  o <node> <l/h>     Set occupancy (l=LOW, h=HIGH)");
    Serial.println("  C <node> <cost>    Set energy cost coefficient");
    Serial.println("  O <lux>            Set HIGH occupancy bound (all)");
    Serial.println("  U <lux>            Set LOW occupancy bound (all)");
    Serial.println("  a <node> <0/1>     Enable/disable anti-windup");
    Serial.println("  f <node> <0/1>     Enable/disable feedback");
    Serial.println("--- Getting values ---");
    Serial.println("  g y <node>         Get current lux");
    Serial.println("  g u <node>         Get current PWM");
    Serial.println("  g r <node>         Get current reference");
    Serial.println("  g E <node>         Get accumulated energy (J)");
    Serial.println("  g V <node>         Get visibility error (lux)");
    Serial.println("  g F <node>         Get flicker metric");
    Serial.println("  g o <node>         Get occupancy state");
    Serial.println("  g d <node>         Get background lux (o_i)");
    Serial.println("  g p <node>         Get instant power (W)");
    Serial.println("  g t <node>         Get uptime (s)");
    Serial.println("  g O <node>         Get HIGH bound");
    Serial.println("  g U <node>         Get LOW bound");
    Serial.println("  g L <node>         Get current lower bound");
    Serial.println("  g C <node>         Get cost coefficient");
    Serial.println("--- Buffer ---");
    Serial.println("  g b <y/u> <node>   Get last-minute buffer (lux or duty)");
    Serial.println("--- Streaming ---");
    Serial.println("  s <y/u> <node>     Start streaming lux or duty");
    Serial.println("  S <y/u> <node>     Stop streaming");
    Serial.println("--- System ---");
    Serial.println("  A <0-3>            Set algorithm (0=PI,1=Cons,2=ADMM,3=DD)");
    Serial.println("  R                  Restart all nodes");
    Serial.println("  RM                 Reset metrics (energy/vis/flicker)");
    Serial.println("  c                  Trigger calibration");
    Serial.println("  rpt                Print calibration report");
    Serial.println("  help / h / ?       Show this help");
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

// ----- Nonblocking serial service -----

// Serial parsing is nonblocking: characters are accumulated until newline and
// then parsed once, allowing CAN, control, and calibration to keep running.
inline void serviceSerialNonblocking() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      serialBuffer[serialLength] = '\0';
      handleLineCommand(serialBuffer);
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
