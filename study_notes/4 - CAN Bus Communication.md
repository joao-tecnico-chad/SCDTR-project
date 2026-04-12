# 4 - CAN Bus Communication

> [!abstract] Summary
> CAN (Controller Area Network) is the communication backbone. This page explains CAN from scratch — what it is, how arbitration works, why it's reliable, then describes our specific protocol: message types, ID structure, payload formats, and the hub concept.

Back to [[SCDEEC Home]]

---

## CAN Bus from Scratch

### What problem does CAN solve?

Imagine you want to connect 3 microcontrollers together so they can send messages to each other. Options:
1. **Point-to-point wires:** Need N×(N-1)/2 = 3 wires for 3 nodes. Scales badly.
2. **I²C or SPI:** Designed for short distances on a PCB, not meters of cable.
3. **UART:** Only point-to-point, not multi-node.
4. **CAN bus:** One pair of wires, any node can talk to any other, handles collisions automatically, designed to work in electrically noisy environments. **This is what we use.**

### The physical layer — differential signaling

CAN uses **two wires**: CAN_H (high) and CAN_L (low). They always carry opposite signals.

When transmitting a "dominant" bit (logical 0):
- CAN_H ≈ 3.5V
- CAN_L ≈ 1.5V
- Difference: CAN_H − CAN_L ≈ +2V

When transmitting a "recessive" bit (logical 1):
- CAN_H ≈ 2.5V
- CAN_L ≈ 2.5V
- Difference: CAN_H − CAN_L ≈ 0V

The receiver measures the **difference** between the two wires, not the absolute voltage. Why does this matter?

> [!example] Analogy: Noise cancellation headphones
> If someone shouts near your cable, the electrical noise adds equally to both CAN_H and CAN_L. Since the receiver measures only the DIFFERENCE, the common-mode noise cancels out. A regular single-wire serial cable would be completely corrupted by the same noise.

This is why CAN was originally designed for cars — car engines produce enormous electromagnetic noise (ignition coils, alternators, motors), yet CAN works reliably just centimeters from the engine.

### Multi-master and arbitration — the clever part

In CAN, any node can transmit at any time. If two nodes transmit simultaneously... what happens?

**The answer: the one with the lower message ID wins, without either node knowing there was a collision.**

Here's how:
1. When a node wants to transmit, it first listens. If the bus is idle, it starts transmitting its ID bits (11 bits for standard CAN).
2. The node continuously monitors what's on the bus while transmitting.
3. If a node transmits a recessive (1) bit but sees a dominant (0) bit on the bus, it means another node transmitted a 0 — and 0 wins (dominant beats recessive).
4. The node that "loses" immediately stops transmitting and waits.
5. The node with all zeros (or the lowest ID) "wins" every bit comparison and successfully transmits its frame.

> [!example] Analogy: Voting in a room full of people
> Imagine 100 people each holding a sign showing a binary number. They all reveal their first digit simultaneously. If anyone has a 0, the answer for that position is "0" — 0 (dominant) beats 1 (recessive). The person with all zeros wins every position and gets to speak.

This is called **non-destructive arbitration**: the losing node's frame is not corrupted — it simply tries again after the winner finishes. No data is lost.

### Why lower ID = higher priority?

In our CAN ID assignments:
- Hello/heartbeat messages use base ID **0x100** (binary: 000 0001 0000 0000)
- Algorithm data uses base ID **0x780** (binary: 011 1100 0000)

ID 0x100 starts with many 0s. ID 0x780 starts with more 1s. During arbitration, the lower ID "wins" every bit where it has a 0 and the other has a 1. So:
- **Low ID = high priority** = gets through even when the bus is busy
- **High ID = low priority** = yields to management traffic

In our system, Hello frames (network management) have the highest priority, ensuring peer discovery and heartbeats always get through.

### Error detection — why CAN is reliable

Every CAN frame includes:
1. **CRC (Cyclic Redundancy Check):** 15-bit checksum over the data. Detects all single-bit errors and most multi-bit errors.
2. **Bit stuffing:** After 5 identical bits in a row, a complementary bit is inserted. This prevents long runs of the same bit (which could confuse synchronization).
3. **Acknowledgement:** Every receiver that successfully gets a frame pulls a specific bit dominant. If the transmitter sees its ACK slot remain recessive, it knows nobody received the frame and will retransmit.
4. **Error counters:** If a node makes too many errors (bus-off condition), it disconnects itself from the bus to avoid disrupting others.

---

## Our CAN Configuration

- **Standard 11-bit identifiers** (not extended 29-bit)
- **500 kbps** data rate
- **MCP_16MHZ** clock setting (16 MHz crystal on MCP2515)
- **CAN_500KBPS** baud rate setting in the mcp_can library

At 500 kbps, one bit takes 2 microseconds. A full 8-byte CAN frame (with overhead) takes about 130 bits = **260 microseconds**. At our algorithm rate of 100 ms per iteration, each iteration could send/receive up to 384 frames before timing out — more than enough.

---

## Message Protocol — Our Custom Design

### ID structure

We use 11-bit CAN IDs structured as:

```
CAN_ID = BASE_ADDRESS + node_id
```

- `node_id` is 1–3 for our nodes
- `BROADCAST_NODE = 0x7F` (127) for "send to everyone"

### Message type table

| Base ID | Type | Priority | What it carries |
|---------|------|----------|----------------|
| **0x100** | Hello / Heartbeat | Highest | Periodic lux, duty, reference from each node |
| **0x300** | Command | High | Remote control: set duty, reference, occupancy, etc. |
| **0x400** | Calibration Plan | High | Plan A + Plan B frames from coordinator |
| **0x500** | Query | Medium | Ask a remote node for a specific value |
| **0x580** | Reply | Medium | Response to a query (float value) |
| **0x600** | Stream | Medium | Real-time lux or duty data for PC plotting |
| **0x680** | Gain Exchange | Medium | K matrix rows + baseline + L_ref + cost |
| **0x700** | Consensus | Low | Duty vector proposals (all 3 duties per frame) |
| **0x780** | ADMM | Low | d[i] and u[i] for ADMM updates |
| **0x7C0** | Dual Decomp | Lowest | d[i] for dual decomposition |

### Why these priority levels?

The system must never lose network management frames (Hello), even if there's heavy algorithm traffic. By assigning Hello the lowest numerical ID (0x100), it always wins arbitration against algorithm data (0x700+). This is intentional CAN protocol design.

---

## Detailed Payload Formats

### Hello Frame (CAN ID: 0x100 + nodeId)
Sent every 1 second by each node to announce it's alive.

```
Byte 0: nodeId
Byte 1-2: PWM value as uint16 (big-endian)
Byte 3-4: lux × 100 as uint16 (e.g., 20.15 lux → 2015)
Byte 5-6: refLux × 100 as uint16
Total: 7 bytes
```

> [!info] Why lux × 100 instead of float?
> A float is 4 bytes. Storing lux as an integer (lux × 100) uses only 2 bytes and gives 0.01 lux resolution — more than sufficient. Smaller frames = faster CAN transmission.

### Command Frame (CAN ID: 0x300 + targetNodeId)
Used by the hub to send commands to individual nodes (or broadcast 0x37F).

```
Byte 0: CommandType (enum: 0x01=SET_PWM, 0x04=SET_REF, 0x09=SET_OCCUPANCY, ...)
Byte 1+: payload, depends on command

Examples:
CMD_LED_SET_PWM: bytes 1-2 = PWM uint16
CMD_SET_REF:     bytes 1-2 = (lux × 100) as uint16
CMD_SET_COST:    bytes 1-2 = (cost × 100) as uint16
CMD_SET_OCCUPANCY: byte 1 = 'l', 'h', or 'o'
```

### Calibration Plan (CAN ID: 0x47F, broadcast)
Split into two frames (Plan A and Plan B) because the full plan is > 8 bytes.

**Plan A:**
```
Byte 0: 0x10 (CALIB_PLAN_A marker)
Byte 1: sender nodeId (coordinator)
Byte 2-3: sessionId (uint16)
Byte 4-5: calPwm (uint16) = 2800
Byte 6: planNodes = 3
Byte 7: encoded settleMs (in 10 ms ticks)
```

**Plan B:**
```
Byte 0: 0x11 (CALIB_PLAN_B marker)
Byte 1: sender nodeId
Byte 2-3: sessionId (must match Plan A)
Byte 4: encoded measureMs
Byte 5: encoded gapMs
Byte 6-7: startDelayTick10 (scheduled start time)
```

The plan is sent 5 times (CAL_PLAN_REPEAT_COUNT = 5), 60 ms apart. If a node misses the first transmission, it will catch one of the repeats.

### Gain Exchange Frame (CAN ID: 0x680 + nodeId)
Sent after calibration to share K matrix.

```
Byte 0: row index (0-based node index)
Byte 1: column index (0-based)
       Special: 0xFF = baseline lux, 0xFE = L_ref, 0xFD = cost
Bytes 2-5: float32 value
Byte 6: marker (0xAA for gain value, 0xBB for baseline)
Total: 7 bytes
```

Per node, this requires: 3 gain frames + 1 baseline frame + 1 L_ref frame + 1 cost frame = **6 frames per node**.

### Consensus Frame (CAN ID: 0x700 + nodeId)
Each node broadcasts its full proposed duty vector.

```
Byte 0: iterByte (iteration counter, 0-255, wraps)
Byte 1: myIndex (0-based)
Bytes 2-3: d[0] as uint16 (d × 10000, so 1.0 → 10000)
Bytes 4-5: d[1] as uint16
Bytes 6-7: d[2] as uint16
Total: 8 bytes
```

> [!info] Why encode duty as uint16 (d × 10000)?
> Precision: d × 10000 can represent values like 0.5304 → 5304. Resolution is 0.0001 duty = 0.01% — more than sufficient. Float32 would use 4 bytes per value, requiring 2 separate frames. This way all 3 duties fit in 6 bytes.

### ADMM Frame (CAN ID: 0x780 + nodeId)

```
Byte 0: myIndex (0-based)
Bytes 1-4: d[i] as float32
Bytes 5-6: u[i] as int16 (u × 1000)
Total: 7 bytes
```

### Dual Decomp Frame (CAN ID: 0x7C0 + nodeId)

```
Byte 0: myIndex (0-based)
Bytes 1-4: d[i] as float32
Total: 5 bytes
```

---

## The Hub Concept

Any node connected to a PC via USB serial can act as the **hub** — the gateway between human commands and the CAN network.

```
Operator types:               Hub receives:
"r 2 20"   ──────────►  Hub  ──────► CAN frame: CMD_SET_REF to Node 2
                                       payload: lux×100 = 2000
Node 2 receives, sets ref, sends back reply via CAN
Hub receives CAN reply ──────────► prints "r 2 20.0" to serial
```

This means the entire 3-node network can be controlled from a single USB cable connected to any one node.

### Hub command → CAN frame translation examples:

| Serial Command | CAN Frame Generated |
|----------------|---------------------|
| `r 2 20` | CMD_SET_REF to Node 2, payload=2000 |
| `u 1 2048` | CMD_LED_SET_PWM to Node 1, payload=2048 |
| `o 3 h` | CMD_SET_OCCUPANCY to Node 3, payload='h' |
| `C 1 2.0` | CMD_SET_COST to Node 1, payload=200 |
| `g y 2` | CAN_QUERY to Node 2, code='y' (lux) |
| `R` | CMD_RESTART broadcast to 0x37F |

### Remote queries (g command)

When you type `g y 2` (get lux from Node 2):
1. Hub sends Query frame to Node 2 (CAN 0x502)
2. Node 2 receives it, reads its `filteredLux`, sends Reply frame (CAN 0x582)
3. Hub receives Reply, prints "y 2 20.015" to serial
4. If no reply in 1 second → timeout error printed

---

## Boot Sequence and Discovery

### When does discovery happen?

After the user enters the node ID and total node count over serial, the node enters **WAKEUP_DISCOVERY_OPEN**. For 5 seconds (WAKEUP_WINDOW_MS = 5000), it:
- Broadcasts Hello frames every 500 ms
- Sends CMD_ANNOUNCE commands to 0x7F (broadcast) to wake up peers

Any node that receives these updates its peer table (tracks other nodes' IDs, lux, duty, reference).

After 5 seconds → **WAKEUP_DISCOVERY_STABLE**. The coordinator (Node 1, isCoordinator = (nodeId == 1)) triggers automatic calibration 500 ms later.

> [!info] Why 5 seconds for discovery?
> It must be long enough that even a slow-booting node (one that the user took 4 seconds to type its ID into) will have been detected. 5 seconds gives ample time for all nodes to boot and announce themselves.

---

## Common Exam Questions

> [!question] "What is CAN bus and why did you use it?"
> CAN (Controller Area Network) is a multi-master serial bus with built-in collision arbitration, error detection, and differential signaling. We use it because: (1) we need multi-master communication — any node can initiate a message; (2) differential signaling makes it robust to electromagnetic interference; (3) automatic error detection and retransmission ensures reliability; (4) arbitration is deterministic (lower ID always wins); (5) it operates over a single wire pair regardless of number of nodes.

> [!question] "Explain CAN arbitration."
> When two nodes transmit simultaneously, each monitors the bus while transmitting its ID bits. A dominant (0) bit overpowers a recessive (1). If a node transmits a 1 but sees a 0 on the bus, it immediately stops — another node has a lower ID and wins. The loser retries after the winner finishes. This is non-destructive: the winner's frame is undamaged. Lower ID = higher priority = guaranteed to win arbitration.

> [!question] "How does a CAN message get from Node 1's control loop to Node 2?"
> Core 0 calls enqueueTxFrame(), which adds the frame to the TX ring buffer. serviceCore0ToCore1Fifo() packs it into 3 uint32 words and pushes it through the RP2040 hardware FIFO. Core 1 receives these words via serviceCore1IncomingFifo(), reassembles the CAN frame, and calls canBus.sendMsgBuf(). The MCP2515 transmits the frame onto the CAN bus. Node 2's MCP2515 receives it, pulls INT low. Core 1 on Node 2 drains the RX buffer, packs the frame into the FIFO towards Core 0. Core 0 calls serviceFifoFromCore1(), which unpacks and dispatches via handleReceivedCanFrame().

> [!question] "What is the hub?"
> The hub is whichever node is connected to the PC via USB serial. It translates between human-readable serial commands (like 'r 2 20' to set Node 2's reference to 20 lux) and CAN bus frames. The entire 3-node network can be controlled from a single USB connection to any one node.

> [!question] "Why are Hello frames at CAN ID 0x100 and algorithm frames at 0x700?"
> Priority. In CAN, lower IDs win arbitration. Hello frames carry heartbeat data essential for network management — they must always get through, even under heavy algorithm traffic. By assigning them the lowest ID in our protocol (0x100), they always beat algorithm data frames (0x700+) in arbitration. This guarantees network topology awareness is never starved by optimization data.

> [!question] "How is the calibration plan distributed reliably?"
> The coordinator broadcasts the plan (Plan A + Plan B frames) 5 times, with 60 ms between broadcasts. All 5 repetitions contain the same plan. If a node misses the first transmission (e.g., it was briefly busy), it will receive one of the 4 subsequent repeats. Since nodes wait for a scheduled start time anyway, receiving the plan late (within the first few hundred ms) still gives them time to participate.

---

Back to [[SCDEEC Home]]
