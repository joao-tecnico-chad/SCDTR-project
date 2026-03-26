#include <Arduino.h>
#include <SPI.h>
#include <CAN.h>

const uint8_t LED_PIN = 15;
const uint8_t LDR_PIN = 26;
const uint8_t CAN_CS_PIN = 17;
const uint8_t CAN_INT_PIN = 20;

const float ADC_REF = 3.3f;
const float ADC_MAX = 4095.0f;
const float R_FIXED = 3000.0f;
const float m_param = -1.0f;
const float b_param = 5.8f;
const float MAX_POWER_W = 1.0f;

const uint32_t SAMPLE_PERIOD_MS = 100;
const uint32_t HELLO_PERIOD_MS = 1000;
const uint32_t WAKEUP_WINDOW_MS = 5000;
const uint32_t WAKEUP_RETRY_MS = 500;
const uint32_t PEER_TIMEOUT_MS = 8000;

const uint16_t CAN_ID_HELLO_BASE = 0x100;
const uint16_t CAN_ID_COMMAND_BASE = 0x300;
const uint16_t CAN_ID_CALIB_BASE = 0x400;
const uint16_t CAN_ID_QUERY_BASE = 0x500;
const uint16_t CAN_ID_REPLY_BASE = 0x580;
const uint16_t CAN_ID_STREAM_BASE = 0x600;

const uint8_t BROADCAST_NODE = 0x7F;
const int MAX_NODES = 8;
const int HISTORY_LEN = 600;

const uint16_t DEFAULT_CAL_PWM = 2800;
const uint16_t DEFAULT_SETTLE_MS = 250;
const uint16_t DEFAULT_MEASURE_MS = 600;
const uint16_t DEFAULT_GAP_MS = 250;
const uint16_t DEFAULT_START_DELAY_MS = 1500;

enum CommandType : uint8_t {
  CMD_LED_SET_PWM = 0x01,
  CMD_LED_OFF = 0x02,
  CMD_ANNOUNCE = 0x03,
  CMD_SET_REF = 0x04,
  CMD_RESTART = 0x05
};

enum CalibType : uint8_t {
  CALIB_START = 0x10
};

enum QueryCode : uint8_t {
  Q_U = 'u',
  Q_R = 'r',
  Q_Y = 'y',
  Q_V = 'v',
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

struct PeerStatus {
  bool active;
  uint8_t nodeId;
  uint16_t pwm;
  float lux;
  float refLux;
  uint32_t lastSeenMs;
};

struct CalibrationConfig {
  bool active;
  uint16_t sessionId;
  uint16_t calPwm;
  uint16_t settleMs;
  uint16_t measureMs;
  uint16_t gapMs;
  uint32_t startAtMs;
};

struct StreamState {
  bool active;
  char variable;
};

PeerStatus peers[MAX_NODES];
CalibrationConfig calib = {false, 0, DEFAULT_CAL_PWM, DEFAULT_SETTLE_MS, DEFAULT_MEASURE_MS, DEFAULT_GAP_MS, 0};
StreamState streamState = {false, 0};

uint8_t nodeId = 0;
uint8_t totalNodes = 0;
bool isCoordinator = false;
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

uint32_t bootMs = 0;
uint32_t lastHelloMs = 0;
uint32_t lastWakeupMs = 0;
uint32_t lastSampleMs = 0;
uint32_t lastStreamMs = 0;

int currentSlot = -2;
bool slotLedApplied = false;
bool slotMeasured = false;
float baselineLux = -1.0f;
float slotLux[MAX_NODES + 1];
float gainRow[MAX_NODES + 1];
float measureAccumulator = 0.0f;
uint16_t measureCount = 0;
bool gainsReady = false;

uint8_t readNumberFromSerial(const char *prompt, uint8_t minValue, uint8_t maxValue) {
  int value = 0;

  Serial.println(prompt);
  while (value < minValue || value > maxValue) {
    if (Serial.available()) {
      value = Serial.parseInt();
      while (Serial.available()) {
        Serial.read();
      }
      if (value < minValue || value > maxValue) {
        Serial.println("Valor invalido. Tenta outra vez.");
      }
    }
    delay(10);
  }

  Serial.println(value);
  return (uint8_t)value;
}

void printAck() {
  Serial.println("ack");
}

void printErr() {
  Serial.println("err");
}

float readLuxFiltered() {
  uint32_t sum = 0;
  for (int i = 0; i < 32; ++i) {
    sum += analogRead(LDR_PIN);
  }

  lastLdrVoltage = (sum / 32.0f) * (ADC_REF / ADC_MAX);
  if (lastLdrVoltage < 0.001f) {
    lastLdrVoltage = 0.001f;
  }

  float rLdr = R_FIXED * (ADC_REF - lastLdrVoltage) / lastLdrVoltage;
  float rawLux = powf(10.0f, (log10f(rLdr) - b_param) / m_param);

  const float alpha = 0.15f;
  filteredLux = (alpha * rawLux) + ((1.0f - alpha) * filteredLux);
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

void sendHello() {
  uint16_t luxEnc = (uint16_t)constrain((int)(filteredLux * 100.0f), 0, 65535);
  uint16_t refEnc = (uint16_t)constrain((int)(refLux * 100.0f), 0, 65535);

  CAN.beginPacket(CAN_ID_HELLO_BASE + nodeId);
  CAN.write(nodeId);
  CAN.write((localPwm >> 8) & 0xFF);
  CAN.write(localPwm & 0xFF);
  CAN.write((luxEnc >> 8) & 0xFF);
  CAN.write(luxEnc & 0xFF);
  CAN.write((refEnc >> 8) & 0xFF);
  CAN.write(refEnc & 0xFF);
  CAN.endPacket();
}

void sendSimpleCommand(uint8_t targetNode, CommandType command, uint16_t value = 0) {
  CAN.beginPacket(CAN_ID_COMMAND_BASE + targetNode);
  CAN.write(command);
  CAN.write(nodeId);
  CAN.write((value >> 8) & 0xFF);
  CAN.write(value & 0xFF);
  CAN.endPacket();
}

void sendQuery(uint8_t targetNode, QueryCode queryCode) {
  CAN.beginPacket(CAN_ID_QUERY_BASE + targetNode);
  CAN.write((uint8_t)queryCode);
  CAN.write(nodeId);
  CAN.endPacket();
}

void sendReply(uint8_t requesterNode, QueryCode queryCode, float value) {
  union {
    float f;
    uint8_t b[4];
  } payload;
  payload.f = value;

  CAN.beginPacket(CAN_ID_REPLY_BASE + requesterNode);
  CAN.write((uint8_t)queryCode);
  CAN.write(nodeId);
  CAN.write(payload.b[0]);
  CAN.write(payload.b[1]);
  CAN.write(payload.b[2]);
  CAN.write(payload.b[3]);
  CAN.endPacket();
}

void sendStreamFrame(char variable) {
  float value = (variable == 'y') ? filteredLux : (float)localPwm;
  union {
    float f;
    uint8_t b[4];
  } payload;
  payload.f = value;
  uint32_t t = millis();

  CAN.beginPacket(CAN_ID_STREAM_BASE + nodeId);
  CAN.write((uint8_t)variable);
  CAN.write(nodeId);
  CAN.write(payload.b[0]);
  CAN.write(payload.b[1]);
  CAN.write(payload.b[2]);
  CAN.write(payload.b[3]);
  CAN.write((t >> 8) & 0xFF);
  CAN.write(t & 0xFF);
  CAN.endPacket();
}

void broadcastCalibrationStart(uint16_t sessionId, uint16_t calPwm, uint16_t settleMs, uint16_t measureMs, uint16_t gapMs, uint16_t startDelayMs) {
  CAN.beginPacket(CAN_ID_CALIB_BASE + BROADCAST_NODE);
  CAN.write(CALIB_START);
  CAN.write(nodeId);
  CAN.write((sessionId >> 8) & 0xFF);
  CAN.write(sessionId & 0xFF);
  CAN.write((calPwm >> 8) & 0xFF);
  CAN.write(calPwm & 0xFF);
  CAN.write((settleMs >> 8) & 0xFF);
  CAN.write(settleMs & 0xFF);
  CAN.endPacket();

  delay(5);

  CAN.beginPacket(CAN_ID_CALIB_BASE + nodeId);
  CAN.write(CALIB_START);
  CAN.write(totalNodes);
  CAN.write((measureMs >> 8) & 0xFF);
  CAN.write(measureMs & 0xFF);
  CAN.write((gapMs >> 8) & 0xFF);
  CAN.write(gapMs & 0xFF);
  CAN.write((startDelayMs >> 8) & 0xFF);
  CAN.write(startDelayMs & 0xFF);
  CAN.endPacket();
}

void resetCalibrationArrays() {
  baselineLux = -1.0f;
  for (int i = 0; i <= MAX_NODES; ++i) {
    slotLux[i] = -1.0f;
    gainRow[i] = 0.0f;
  }
  gainsReady = false;
}

void startCalibrationSession(uint16_t sessionId, uint16_t calPwm, uint16_t settleMs, uint16_t measureMs, uint16_t gapMs, uint16_t startDelayMs) {
  calib.active = true;
  calib.sessionId = sessionId;
  calib.calPwm = calPwm;
  calib.settleMs = settleMs;
  calib.measureMs = measureMs;
  calib.gapMs = gapMs;
  calib.startAtMs = millis() + startDelayMs;

  currentSlot = -1;
  slotLedApplied = false;
  slotMeasured = false;
  measureAccumulator = 0.0f;
  measureCount = 0;
  resetCalibrationArrays();
  setLedPwm(0);
}

void finishCalibration() {
  calib.active = false;
  setLedPwm(0);

  if (baselineLux < 0.0f) {
    return;
  }

  for (int sourceId = 1; sourceId <= totalNodes; ++sourceId) {
    if (slotLux[sourceId] >= 0.0f) {
      gainRow[sourceId] = (slotLux[sourceId] - baselineLux) / (float)calib.calPwm;
    }
  }

  gainsReady = true;
}

void restartSystemState() {
  energyJ = 0.0f;
  visibilityErrorIntegral = 0.0f;
  flickerIntegral = 0.0f;
  restartSeconds = 0.0f;
  historyIndex = 0;
  historyWrapped = false;
  streamState.active = false;
  setLedPwm(0);
  resetCalibrationArrays();
  autoCalibrationTriggered = false;
  bootMs = millis();
  filteredLux = readLuxFiltered();
  sendHello();
  sendSimpleCommand(BROADCAST_NODE, CMD_ANNOUNCE);
}

void printCalibrationReport() {
  if (!gainsReady) {
    Serial.println("Calibracao ainda nao concluida.");
    return;
  }

  Serial.print("No ");
  Serial.print(nodeId);
  Serial.println(" | ganhos estimados:");
  Serial.print("  luz_base=");
  Serial.println(baselineLux, 3);

  for (int sourceId = 1; sourceId <= totalNodes; ++sourceId) {
    Serial.print("  G[");
    Serial.print(nodeId);
    Serial.print("][");
    Serial.print(sourceId);
    Serial.print("] = ");
    Serial.println(gainRow[sourceId], 6);
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

void processCalibrationTimeline() {
  if (!calib.active) {
    return;
  }

  const uint32_t now = millis();
  if (now < calib.startAtMs) {
    return;
  }

  const uint32_t slotWindow = calib.settleMs + calib.measureMs + calib.gapMs;
  const int slotIndex = (int)((now - calib.startAtMs) / slotWindow) - 1;

  if (slotIndex != currentSlot) {
    currentSlot = slotIndex;
    slotLedApplied = false;
    slotMeasured = false;
    measureAccumulator = 0.0f;
    measureCount = 0;
  }

  if (currentSlot < -1) {
    return;
  }

  if (currentSlot > totalNodes) {
    finishCalibration();
    return;
  }

  const uint32_t slotStart = calib.startAtMs + (uint32_t)(currentSlot + 1) * slotWindow;
  const uint32_t settleEnd = slotStart + calib.settleMs;
  const uint32_t measureEnd = settleEnd + calib.measureMs;

  if (!slotLedApplied) {
    if (currentSlot == -1) {
      setLedPwm(0);
    } else {
      setLedPwm((nodeId == currentSlot + 1) ? calib.calPwm : 0);
    }
    slotLedApplied = true;
  }

  if (now >= settleEnd && now < measureEnd) {
    measureAccumulator += readLuxFiltered();
    measureCount++;
  }

  if (!slotMeasured && now >= measureEnd) {
    float averageLux = (measureCount > 0) ? (measureAccumulator / measureCount) : readLuxFiltered();
    if (currentSlot == -1) {
      baselineLux = averageLux;
    } else {
      slotLux[currentSlot + 1] = averageLux;
    }

    setLedPwm(0);
    slotMeasured = true;
  }
}

float readQueryValue(QueryCode queryCode, bool &ok) {
  ok = true;
  switch (queryCode) {
    case Q_U: return (float)localPwm;
    case Q_R: return refLux;
    case Q_Y: return filteredLux;
    case Q_V: return lastLdrVoltage;
    case Q_D: return (baselineLux >= 0.0f) ? baselineLux : 0.0f;
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

void handleCommand(long packetId, int packetSize, uint8_t *data) {
  if (packetSize < 2) {
    return;
  }

  uint16_t targetNode = packetId - CAN_ID_COMMAND_BASE;
  if (targetNode != nodeId && targetNode != BROADCAST_NODE) {
    return;
  }

  switch (data[0]) {
    case CMD_LED_SET_PWM:
      if (packetSize >= 4) {
        uint16_t pwm = ((uint16_t)data[2] << 8) | data[3];
        setLedPwm(pwm);
      }
      break;
    case CMD_LED_OFF:
      setLedPwm(0);
      break;
    case CMD_ANNOUNCE:
      sendHello();
      break;
    case CMD_SET_REF:
      if (packetSize >= 4) {
        refLux = (((uint16_t)data[2] << 8) | data[3]) / 100.0f;
      }
      break;
    case CMD_RESTART:
      restartSystemState();
      break;
  }
}

void handleCalibration(long packetId, int packetSize, uint8_t *data) {
  if (packetSize < 8) {
    return;
  }

  if (packetId == (CAN_ID_CALIB_BASE + BROADCAST_NODE) && data[0] == CALIB_START) {
    uint16_t sessionId = ((uint16_t)data[2] << 8) | data[3];
    uint16_t calPwm = ((uint16_t)data[4] << 8) | data[5];
    uint16_t settleMs = ((uint16_t)data[6] << 8) | data[7];

    const uint32_t deadline = millis() + 300;
    while (millis() < deadline) {
      int packetSize2 = CAN.parsePacket();
      if (!packetSize2) {
        delay(1);
        continue;
      }

      long packetId2 = CAN.packetId();
      if (packetId2 != (CAN_ID_CALIB_BASE + data[1]) || packetSize2 < 8) {
        while (CAN.available()) {
          CAN.read();
        }
        continue;
      }

      uint8_t data2[8] = {0};
      int i2 = 0;
      while (CAN.available() && i2 < 8) {
        data2[i2++] = (uint8_t)CAN.read();
      }

      if (data2[0] != CALIB_START) {
        continue;
      }

      totalNodes = data2[1];
      uint16_t measureMs = ((uint16_t)data2[2] << 8) | data2[3];
      uint16_t gapMs = ((uint16_t)data2[4] << 8) | data2[5];
      uint16_t startDelayMs = ((uint16_t)data2[6] << 8) | data2[7];

      startCalibrationSession(sessionId, calPwm, settleMs, measureMs, gapMs, startDelayMs);
      return;
    }
  }
}

void handleQuery(long packetId, int packetSize, uint8_t *data) {
  if (packetSize < 2) {
    return;
  }

  uint16_t targetNode = packetId - CAN_ID_QUERY_BASE;
  if (targetNode != nodeId) {
    return;
  }

  bool ok = false;
  float value = readQueryValue((QueryCode)data[0], ok);
  if (ok) {
    sendReply(data[1], (QueryCode)data[0], value);
  }
}

void handleReply(int packetSize, uint8_t *data) {
  if (packetSize < 6) {
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

  Serial.print(code);
  Serial.print(" ");
  Serial.print(sourceNode);
  Serial.print(" ");
  Serial.println(payload.f, (code == 'u' || code == 't' || code == 'E') ? 3 : 3);
}

void handleStreamPacket(int packetSize, uint8_t *data) {
  if (packetSize < 8) {
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

void handleCanMessage() {
  int packetSize = CAN.parsePacket();
  if (!packetSize) {
    return;
  }

  long packetId = CAN.packetId();
  uint8_t data[8] = {0};
  int i = 0;

  while (CAN.available() && i < 8) {
    data[i++] = (uint8_t)CAN.read();
  }

  if (packetId >= CAN_ID_HELLO_BASE && packetId < (CAN_ID_HELLO_BASE + 0x80) && packetSize >= 7) {
    uint8_t senderId = data[0];
    uint16_t pwm = ((uint16_t)data[1] << 8) | data[2];
    float lux = (((uint16_t)data[3] << 8) | data[4]) / 100.0f;
    float peerRef = (((uint16_t)data[5] << 8) | data[6]) / 100.0f;
    updatePeer(senderId, pwm, lux, peerRef);
    return;
  }

  if (packetId >= CAN_ID_COMMAND_BASE && packetId < (CAN_ID_COMMAND_BASE + 0x80)) {
    handleCommand(packetId, packetSize, data);
    return;
  }

  if (packetId >= CAN_ID_CALIB_BASE && packetId < (CAN_ID_CALIB_BASE + 0x80)) {
    handleCalibration(packetId, packetSize, data);
    return;
  }

  if (packetId >= CAN_ID_QUERY_BASE && packetId < (CAN_ID_QUERY_BASE + 0x80)) {
    handleQuery(packetId, packetSize, data);
    return;
  }

  if (packetId >= CAN_ID_REPLY_BASE && packetId < (CAN_ID_REPLY_BASE + 0x80)) {
    handleReply(packetSize, data);
    return;
  }

  if (packetId >= CAN_ID_STREAM_BASE && packetId < (CAN_ID_STREAM_BASE + 0x80)) {
    handleStreamPacket(packetSize, data);
  }
}

bool parseGetCommand(const String &line) {
  char a = 0;
  char b = 0;
  int i = 0;

  if (sscanf(line.c_str(), "g %c %d", &a, &i) == 2 && (a == 'o' || a == 'a' || a == 'f')) {
    if (i != nodeId) {
      printErr();
      return true;
    }
    Serial.print(a);
    Serial.print(" ");
    Serial.print(i);
    Serial.print(" ");
    if (a == 'o') {
      Serial.println(occupancyState);
    } else if (a == 'a') {
      Serial.println(antiWindupEnabled ? 1 : 0);
    } else {
      Serial.println(feedbackEnabled ? 1 : 0);
    }
    return true;
  }

  if (sscanf(line.c_str(), "g %c %d", &a, &i) == 2) {
    if (i < 0 || i > MAX_NODES) {
      printErr();
      return true;
    }

    if (i == nodeId) {
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
      Serial.println(value, (a == 'u') ? 0 : 3);
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

  if (sscanf(line.c_str(), "g %c %c %d", &a, &b, &i) == 3) {
    if (a == 'b' && (b == 'y' || b == 'u') && i == nodeId) {
      printBuffer(b);
      return true;
    }
    printErr();
    return true;
  }

  return false;
}

void handleLineCommand(String line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  if (line == "R") {
    sendSimpleCommand(BROADCAST_NODE, CMD_RESTART);
    restartSystemState();
    printAck();
    return;
  }

  if (parseGetCommand(line)) {
    return;
  }

  char c1 = 0;
  char c2 = 0;
  int i = 0;
  float val = 0.0f;

  if (sscanf(line.c_str(), "u %d %f", &i, &val) == 2) {
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

  if (sscanf(line.c_str(), "r %d %f", &i, &val) == 2) {
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

  if (sscanf(line.c_str(), "o %d %c", &i, &c1) == 2) {
    if (i != nodeId || (c1 != 'o' && c1 != 'l' && c1 != 'h')) {
      printErr();
      return;
    }
    occupancyState = c1;
    updateCurrentLowerBound();
    printAck();
    return;
  }

  int node = 0;
  int intVal = 0;
  if (sscanf(line.c_str(), "a %d %d", &node, &intVal) == 2) {
    if (node != nodeId) {
      printErr();
      return;
    }
    antiWindupEnabled = (intVal != 0);
    printAck();
    return;
  }

  if (sscanf(line.c_str(), "f %d %d", &node, &intVal) == 2) {
    if (node != nodeId) {
      printErr();
      return;
    }
    feedbackEnabled = (intVal != 0);
    printAck();
    return;
  }

  if (sscanf(line.c_str(), "O %d %f", &node, &val) == 2) {
    if (node != nodeId) {
      printErr();
      return;
    }
    highLuxBound = val;
    updateCurrentLowerBound();
    printAck();
    return;
  }

  if (sscanf(line.c_str(), "U %d %f", &node, &val) == 2) {
    if (node != nodeId) {
      printErr();
      return;
    }
    lowLuxBound = val;
    updateCurrentLowerBound();
    printAck();
    return;
  }

  if (sscanf(line.c_str(), "C %d %f", &node, &val) == 2) {
    if (node != nodeId) {
      printErr();
      return;
    }
    energyCost = val;
    printAck();
    return;
  }

  if (sscanf(line.c_str(), "s %c %d", &c1, &node) == 2) {
    if ((c1 != 'y' && c1 != 'u') || node != nodeId) {
      printErr();
      return;
    }
    streamState.active = true;
    streamState.variable = c1;
    printAck();
    return;
  }

  if (sscanf(line.c_str(), "S %c %d", &c1, &node) == 2) {
    if ((c1 != 'y' && c1 != 'u') || node != nodeId) {
      printErr();
      return;
    }
    streamState.active = false;
    streamState.variable = 0;
    printAck();
    return;
  }

  if (sscanf(line.c_str(), "c %d", &intVal) == 1) {
    if (!isCoordinator) {
      printErr();
      return;
    }
    uint16_t pwm = (intVal > 0) ? (uint16_t)intVal : DEFAULT_CAL_PWM;
    uint16_t sessionId = (uint16_t)(millis() & 0xFFFF);
    broadcastCalibrationStart(sessionId, pwm, DEFAULT_SETTLE_MS, DEFAULT_MEASURE_MS, DEFAULT_GAP_MS, DEFAULT_START_DELAY_MS);
    startCalibrationSession(sessionId, pwm, DEFAULT_SETTLE_MS, DEFAULT_MEASURE_MS, DEFAULT_GAP_MS, DEFAULT_START_DELAY_MS);
    printAck();
    return;
  }

  if (line == "rpt") {
    printCalibrationReport();
    return;
  }

  printErr();
}

void handleSerial() {
  if (!Serial.available()) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  handleLineCommand(line);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  analogReadResolution(12);
  analogWriteResolution(12);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LDR_PIN, INPUT);
  setLedPwm(0);

  while (!Serial) {
    delay(10);
  }

  nodeId = readNumberFromSerial("Introduz o ID do no (1-8):", 1, MAX_NODES);
  totalNodes = readNumberFromSerial("Introduz o numero total de nos (1-8):", 1, MAX_NODES);
  isCoordinator = (nodeId == 1);
  updateCurrentLowerBound();

  SPI.setRX(16);
  SPI.setCS(CAN_CS_PIN);
  SPI.setSCK(18);
  SPI.setTX(19);
  SPI.begin();

  CAN.setPins(CAN_CS_PIN, CAN_INT_PIN);
  CAN.setSPIFrequency(1E6);
  CAN.setClockFrequency(8E6);

  if (!CAN.begin(500E3)) {
    while (1) {
      delay(1000);
    }
  }

  bootMs = millis();
  filteredLux = readLuxFiltered();
  sendHello();
  sendSimpleCommand(BROADCAST_NODE, CMD_ANNOUNCE);
}

void loop() {
  handleSerial();
  handleCanMessage();
  processCalibrationTimeline();

  const uint32_t now = millis();

  if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
    float prevLux = filteredLux;
    lastSampleMs = now;
    readLuxFiltered();
    restartSeconds = (now - bootMs) / 1000.0f;
    energyJ += getInstantPowerW() * (SAMPLE_PERIOD_MS / 1000.0f);
    visibilityErrorIntegral += fabsf(refLux - filteredLux) * (SAMPLE_PERIOD_MS / 1000.0f);
    flickerIntegral += fabsf(filteredLux - prevLux);
    pushHistory(filteredLux, localPwm);
  }

  if (now - lastHelloMs >= HELLO_PERIOD_MS) {
    lastHelloMs = now;
    sendHello();
    removeStalePeers();
  }

  if ((now - bootMs) <= WAKEUP_WINDOW_MS && (now - lastWakeupMs >= WAKEUP_RETRY_MS)) {
    lastWakeupMs = now;
    sendHello();
    sendSimpleCommand(BROADCAST_NODE, CMD_ANNOUNCE);
  }

  if (streamState.active && now - lastStreamMs >= SAMPLE_PERIOD_MS) {
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

  if (isCoordinator && !autoCalibrationTriggered && (now - bootMs > WAKEUP_WINDOW_MS + 500)) {
    autoCalibrationTriggered = true;
    uint16_t sessionId = (uint16_t)(millis() & 0xFFFF);
    broadcastCalibrationStart(sessionId, DEFAULT_CAL_PWM, DEFAULT_SETTLE_MS, DEFAULT_MEASURE_MS, DEFAULT_GAP_MS, DEFAULT_START_DELAY_MS);
    startCalibrationSession(sessionId, DEFAULT_CAL_PWM, DEFAULT_SETTLE_MS, DEFAULT_MEASURE_MS, DEFAULT_GAP_MS, DEFAULT_START_DELAY_MS);
  }
}
