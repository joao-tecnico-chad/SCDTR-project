#ifndef CAN_COMMS_H
#define CAN_COMMS_H

#include <SPI.h>
#include <mcp2515.h>

// ============================================================
// CAN-BUS Communication Layer
// Uses MCP2515 via SPI on RPI Pico.
// Joy-It CAN-BUS module with termination jumpers.
// ============================================================

// --- SPI pin assignments for Pico → MCP2515 ---
static const int CAN_SCK_PIN  = 18;
static const int CAN_MOSI_PIN = 19;
static const int CAN_MISO_PIN = 16;
static const int CAN_CS_PIN   = 17;
static const int CAN_INT_PIN  = 20;

// --- CAN message types (lower 4 bits of arbitration ID) ---
enum CANMsgType : uint8_t {
  MSG_BOOT_ANNOUNCE    = 0x01,
  MSG_BOOT_ACK         = 0x02,
  MSG_CALIB_CMD        = 0x03,  // calibration phase command
  MSG_CALIB_DATA       = 0x04,  // calibration measurement result
  MSG_DUTY_UPDATE      = 0x05,  // share current duty cycle
  MSG_REF_UPDATE       = 0x06,  // share illuminance reference
  MSG_COST_UPDATE      = 0x07,  // share cost coefficient
  MSG_LAMBDA_UPDATE    = 0x08,  // Lagrange multipliers (dual decomp)
  MSG_ADMM_Z_UPDATE    = 0x09,  // ADMM z variable
  MSG_ADMM_U_UPDATE    = 0x0A,  // ADMM dual variable
  MSG_HUB_CMD          = 0x0B,  // command forwarded from PC via hub
  MSG_HUB_RESPONSE     = 0x0C,  // response to hub for forwarding to PC
  MSG_CONSENSUS_D      = 0x0D,  // consensus duty proposal
  MSG_OCCUPY_UPDATE    = 0x0E,  // occupancy state change
  MSG_SYNC             = 0x0F   // synchronization pulse
};

// --- CAN message structure ---
struct CANMessage {
  uint8_t sender;       // logical node ID of sender
  uint8_t type;         // CANMsgType
  uint8_t data[8];      // payload
  uint8_t len;          // payload length (0-8)
};

// --- Encode arbitration ID: (sender << 4) | msg_type ---
static inline uint32_t canArbId(uint8_t sender, uint8_t msg_type) {
  return ((uint32_t)sender << 4) | (msg_type & 0x0F);
}

static inline uint8_t canArbSender(uint32_t arb_id) {
  return (uint8_t)(arb_id >> 4);
}

static inline uint8_t canArbType(uint32_t arb_id) {
  return (uint8_t)(arb_id & 0x0F);
}

// --- CAN-BUS driver class ---
class CANComms {
public:
  MCP2515 mcp;
  uint8_t my_id;
  volatile bool msg_available;

  CANComms()
    : mcp(CAN_CS_PIN), my_id(0), msg_available(false)
  {}

  bool init(uint8_t node_id) {
    my_id = node_id;

    // Configure SPI1 pins for Pico
    SPI1.setSCK(CAN_SCK_PIN);
    SPI1.setTX(CAN_MOSI_PIN);
    SPI1.setRX(CAN_MISO_PIN);
    SPI1.setCS(CAN_CS_PIN);

    mcp.setSPI(&SPI1);
    mcp.reset();
    mcp.setBitrate(CAN_500KBPS, MCP_16MHZ);

    // Accept all messages (no filter)
    mcp.setFilterMask(MCP2515::MASK0, false, 0x000);
    mcp.setFilterMask(MCP2515::MASK1, false, 0x000);

    mcp.setNormalMode();

    // Interrupt pin for incoming messages
    pinMode(CAN_INT_PIN, INPUT_PULLUP);

    return true;
  }

  // Send a message to all nodes (broadcast)
  bool broadcast(uint8_t msg_type, const uint8_t *data, uint8_t len) {
    struct can_frame frame;
    frame.can_id  = canArbId(my_id, msg_type);
    frame.can_dlc = len;
    if (len > 8) len = 8;
    memcpy(frame.data, data, len);
    return mcp.sendMessage(&frame) == MCP2515::ERROR_OK;
  }

  // Send a float value as a 4-byte payload
  bool broadcastFloat(uint8_t msg_type, float value) {
    uint8_t data[4];
    memcpy(data, &value, 4);
    return broadcast(msg_type, data, 4);
  }

  // Send two floats (8 bytes)
  bool broadcastFloat2(uint8_t msg_type, float v1, float v2) {
    uint8_t data[8];
    memcpy(data, &v1, 4);
    memcpy(data + 4, &v2, 4);
    return broadcast(msg_type, data, 8);
  }

  // Send a targeted message (target node ID encoded in data[0])
  bool sendTo(uint8_t target, uint8_t msg_type,
              const uint8_t *data, uint8_t len) {
    struct can_frame frame;
    frame.can_id  = canArbId(my_id, msg_type);
    frame.can_dlc = len + 1;
    if (frame.can_dlc > 8) frame.can_dlc = 8;
    frame.data[0] = target;
    memcpy(frame.data + 1, data, (len > 7) ? 7 : len);
    return mcp.sendMessage(&frame) == MCP2515::ERROR_OK;
  }

  // Check for incoming message (non-blocking)
  bool receive(CANMessage &msg) {
    struct can_frame frame;
    if (mcp.readMessage(&frame) != MCP2515::ERROR_OK)
      return false;

    msg.sender = canArbSender(frame.can_id);
    msg.type   = canArbType(frame.can_id);
    msg.len    = frame.can_dlc;
    memcpy(msg.data, frame.data, frame.can_dlc);
    return true;
  }

  // Extract a float from message data at given offset
  static float extractFloat(const uint8_t *data, int offset = 0) {
    float val;
    memcpy(&val, data + offset, 4);
    return val;
  }

  // Check interrupt pin (active low)
  bool hasMessage() {
    return digitalRead(CAN_INT_PIN) == LOW;
  }
};

#endif // CAN_COMMS_H
