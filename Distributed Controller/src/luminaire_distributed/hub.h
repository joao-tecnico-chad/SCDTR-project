#ifndef HUB_H
#define HUB_H

#include "can_comms.h"

// ============================================================
// Hub Function
// Bridges serial (PC) ↔ CAN-BUS (network).
// Any node with USB serial becomes the hub.
// Commands targeting remote nodes are forwarded via CAN.
// ============================================================

static const unsigned long HUB_CAN_TIMEOUT_MS = 500;

class Hub {
public:
  CANComms *can;
  int my_id;      // 0-based
  int num_nodes;
  bool active;    // true if this node is the hub

  // Pending response state
  bool waiting_response;
  int  response_from;
  unsigned long response_timeout;

  Hub()
    : can(nullptr), my_id(0), num_nodes(0), active(false),
      waiting_response(false), response_from(0), response_timeout(0)
  {}

  void init(CANComms *can_ptr, int id, int n_nodes, bool is_hub) {
    can = can_ptr;
    my_id = id;
    num_nodes = n_nodes;
    active = is_hub;
  }

  // Forward a serial command to a remote node via CAN.
  // The command string is packed into CAN message(s).
  bool forwardCommand(int target_node, const char *cmd, int cmd_len) {
    if (!active || !can) return false;

    // Pack command into CAN payload (max 7 bytes per msg, byte 0 = target)
    uint8_t data[8];
    data[0] = (uint8_t)target_node;

    int sent = 0;
    while (sent < cmd_len) {
      int chunk = cmd_len - sent;
      if (chunk > 7) chunk = 7;
      memcpy(data + 1, cmd + sent, chunk);
      can->broadcast(MSG_HUB_CMD, data, chunk + 1);
      sent += chunk;
    }

    waiting_response = true;
    response_from = target_node;
    response_timeout = millis() + HUB_CAN_TIMEOUT_MS;
    return true;
  }

  // Check for and forward CAN responses to serial
  bool checkResponse() {
    if (!waiting_response) return false;

    if (millis() > response_timeout) {
      Serial.println("err timeout");
      waiting_response = false;
      return false;
    }

    CANMessage msg;
    if (can->receive(msg)) {
      if (msg.type == MSG_HUB_RESPONSE && msg.sender == response_from) {
        // Forward response data to serial
        for (int i = 0; i < msg.len; i++)
          Serial.write(msg.data[i]);
        Serial.println();
        waiting_response = false;
        return true;
      }
    }
    return false;
  }

  // Handle an incoming HUB_CMD message (called on remote nodes)
  // Returns the command string for local processing.
  bool handleIncoming(const CANMessage &msg, char *cmd_buf, int buf_size) {
    if (msg.type != MSG_HUB_CMD) return false;
    int target = msg.data[0];
    if (target != my_id + 1) return false;  // not for us (1-based target)

    int len = msg.len - 1;
    if (len > buf_size - 1) len = buf_size - 1;
    memcpy(cmd_buf, msg.data + 1, len);
    cmd_buf[len] = '\0';
    return true;
  }

  // Send a response back to the hub via CAN
  void sendResponse(const char *resp, int len) {
    if (!can) return;
    uint8_t data[8];
    int sent = 0;
    while (sent < len) {
      int chunk = len - sent;
      if (chunk > 8) chunk = 8;
      memcpy(data, resp + sent, chunk);
      can->broadcast(MSG_HUB_RESPONSE, data, chunk);
      sent += chunk;
    }
  }
};

#endif // HUB_H
