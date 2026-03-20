#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include "can_comms.h"
#include <pico/unique_id.h>
#include <algorithm>

// ============================================================
// Network Boot & Node Discovery
// Uses pico_unique_id for hardware identification.
// Deterministic ID assignment: sort unique IDs → rank = logical ID.
// ============================================================

static const int MAX_NODES = 3;
static const unsigned long BOOT_TIMEOUT_MS   = 3000;  // discovery window
static const unsigned long BOOT_ANNOUNCE_MS  = 200;   // announce interval

class NetworkManager {
public:
  CANComms *can;

  // Network state
  int   num_nodes;
  int   my_logical_id;   // 1-based logical ID
  bool  is_hub;          // true if USB serial is connected

  // Unique IDs of all discovered nodes (8 bytes each from Pico flash)
  uint64_t unique_ids[MAX_NODES];
  int      logical_ids[MAX_NODES];  // sorted order → logical ID mapping

  // My hardware unique ID
  uint64_t my_unique_id;

  NetworkManager()
    : can(nullptr), num_nodes(0), my_logical_id(0), is_hub(false),
      my_unique_id(0)
  {
    memset(unique_ids, 0, sizeof(unique_ids));
    memset(logical_ids, 0, sizeof(logical_ids));
  }

  void init(CANComms *can_ptr) {
    can = can_ptr;

    // Read this Pico's unique ID (8 bytes)
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);
    memcpy(&my_unique_id, board_id.id, sizeof(uint64_t));

    // Check if USB serial is connected (hub detection)
    is_hub = Serial;  // truthy if USB CDC is active
  }

  // Run the boot/discovery sequence. Returns number of nodes found.
  int boot() {
    num_nodes = 0;

    // Register ourselves first
    unique_ids[0] = my_unique_id;
    num_nodes = 1;

    unsigned long start = millis();
    unsigned long last_announce = 0;

    // Discovery phase: announce and listen
    while (millis() - start < BOOT_TIMEOUT_MS) {
      // Periodically broadcast our unique ID
      if (millis() - last_announce >= BOOT_ANNOUNCE_MS) {
        uint8_t data[8];
        memcpy(data, &my_unique_id, 8);
        can->broadcast(MSG_BOOT_ANNOUNCE, data, 8);
        last_announce = millis();
      }

      // Listen for announcements from others
      CANMessage msg;
      while (can->receive(msg)) {
        if (msg.type == MSG_BOOT_ANNOUNCE && msg.len >= 8) {
          uint64_t sender_uid;
          memcpy(&sender_uid, msg.data, 8);
          addNode(sender_uid);
        }
      }
    }

    // Assign logical IDs: sort unique IDs, rank = ID (1-based)
    assignLogicalIds();

    // Broadcast ACK with our assigned ID
    uint8_t ack_data[2];
    ack_data[0] = (uint8_t)my_logical_id;
    ack_data[1] = (uint8_t)num_nodes;
    can->broadcast(MSG_BOOT_ACK, ack_data, 2);

    // Brief listen for ACKs from others
    delay(500);
    CANMessage msg;
    while (can->receive(msg)) {
      // consume ACKs (could verify consistency here)
    }

    return num_nodes;
  }

private:
  void addNode(uint64_t uid) {
    // Check if already known
    for (int i = 0; i < num_nodes; i++) {
      if (unique_ids[i] == uid) return;
    }
    if (num_nodes < MAX_NODES) {
      unique_ids[num_nodes] = uid;
      num_nodes++;
    }
  }

  void assignLogicalIds() {
    // Sort unique IDs to get deterministic ordering
    // Simple bubble sort for 3 elements
    uint64_t sorted[MAX_NODES];
    memcpy(sorted, unique_ids, sizeof(uint64_t) * num_nodes);

    for (int i = 0; i < num_nodes - 1; i++) {
      for (int j = 0; j < num_nodes - i - 1; j++) {
        if (sorted[j] > sorted[j + 1]) {
          uint64_t tmp = sorted[j];
          sorted[j] = sorted[j + 1];
          sorted[j + 1] = tmp;
        }
      }
    }

    // My logical ID = position in sorted array + 1
    for (int i = 0; i < num_nodes; i++) {
      if (sorted[i] == my_unique_id) {
        my_logical_id = i + 1;
      }
    }
  }
};

#endif // NETWORK_MANAGER_H
