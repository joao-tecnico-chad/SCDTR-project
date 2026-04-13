# 5 - Dual Core Architecture

> [!abstract] Summary
> The RP2040 has two CPU cores. We assign Core 0 to all control logic and Core 1 exclusively to the CAN bus driver. They communicate via lock-free hardware FIFOs. This solves the SPI contention problem and bounds CAN latency independently of control computation.

Back to [[SCDEEC Home]]

---

## The Problem: SPI Contention

### What is SPI?

SPI (Serial Peripheral Interface) is a protocol for short-distance chip-to-chip communication. The Pico talks to the MCP2515 over SPI at 10 MHz. To send a CAN frame, the Pico must:
1. Assert the CS (chip select) pin LOW
2. Send a "write to TX buffer" command byte
3. Send the CAN ID bytes
4. Send the data bytes (up to 8)
5. Send the "request transmit" command
6. Release CS HIGH

This sequence takes about 10–20 microseconds on the SPI bus.

### Why is this a problem on a single core?

Imagine Core 0 is in the middle of step 3 (sending CAN ID bytes) when the 100 Hz control timer fires and the control loop tries to also access the SPI bus. The SPI state machine is now corrupted — the MCP2515 receives garbled data and either ignores the frame or, worse, transmits garbage onto the CAN bus.

This is a **race condition** — two things competing for the same resource with unpredictable consequences.

> [!example] Analogy: Two people grabbing the same pen
> If you're writing a sentence and someone else grabs the pen mid-word to write something else, neither message makes sense. You need one person to finish before the other starts.

### The naïve fix: mutexes (and why we didn't use them)

A mutex (mutual exclusion lock) is a programming construct: "only one thread can hold this lock at a time." If the CAN driver holds the SPI mutex, the control loop waits. When the CAN driver releases it, the control loop proceeds.

Problem: **waiting** in a real-time system is dangerous. The control loop must run every 10 ms. If it has to wait 3 ms for a CAN frame to finish transmitting, the control timing is degraded. Worst case: a burst of CAN traffic could cause the control loop to miss its deadline entirely.

### Our solution: dedicated core

By putting ALL CAN/SPI operations on Core 1 and ALL control logic on Core 0, we eliminate contention entirely. Core 0 never touches SPI. Core 1 never runs control loops. **Each core owns its resources exclusively.** No locks needed.

---

## Core Responsibilities

### Core 0 — Control and Logic
- PI controller at 100 Hz (timer-driven)
- Calibration state machine
- Wakeup / discovery state machine
- Distributed algorithms (Consensus, ADMM, Dual Decomp)
- Serial command parser (hub interface)
- Hello heartbeat sending (via FIFO to Core 1)
- Processing received CAN frames (dispatching to handlers)
- Streaming telemetry data

### Core 1 — CAN Exclusively
- MCP2515 SPI driver
- CAN interrupt handler (GPIO 20 interrupt, FALLING edge)
- RX: drain MCP2515 buffers → pack frames → push to Core 0 via FIFO
- TX: receive frames from Core 0 via FIFO → call canBus.sendMsgBuf()
- Health monitoring (periodic MCP2515 error register check every 250 ms)

---

## Inter-Core Communication: The FIFO

### What is the RP2040 hardware FIFO?

The RP2040 chip has a dedicated hardware FIFO (First-In, First-Out queue) between the two cores. It holds up to 8 words of 32 bits each, and it's **lock-free by design**: one core can write while the other reads simultaneously without any synchronization primitives needed.

The key property: **only Core 0 writes to the Core0→Core1 direction, and only Core 1 writes to the Core1→Core0 direction.** This is what makes it safe without locks.

### Challenge: a CAN frame is 20 bytes, but FIFO words are 4 bytes

A CAN frame has:
- ID: 2 bytes (11-bit + padding)
- Length: 1 byte
- Data: up to 8 bytes

Total: ~11 bytes = needs 3 × 4-byte (uint32) words.

**Packing scheme:** Each frame is encoded as exactly 3 uint32 words:

```
Word 0 (bits 31-0):
  [31:24] = FifoMessageKind (FIFO_TX_FRAME or FIFO_DIAG)
  [23:8]  = CAN ID (16-bit)
  [7:0]   = data length (1 byte)

Word 1 (bits 31-0):
  data[0], data[1], data[2], data[3]  (4 bytes)

Word 2 (bits 31-0):
  data[4], data[5], data[6], data[7]  (4 bytes)
```

The receiving core reads exactly 3 words to reconstruct one complete CAN frame.

### Core 0 → Core 1 path (TX):

```
Core 0 creates a CAN frame
        │
        ▼
enqueueTxFrame(id, payload, len)
  → adds to txQueue[] ring buffer (24 frames deep)
        │
        ▼ (later, in serviceCore0ToCore1Fifo())
Pack into 3 uint32 words
  → rp2040.fifo.push(word0)
  → rp2040.fifo.push(word1)
  → rp2040.fifo.push(word2)
        │
        ▼ (Core 1's loop1())
serviceCore1IncomingFifo()
  → reads word0, checks kind = FIFO_TX_FRAME
  → reads word1, word2
  → reconstructs CanFrame
  → canBus.sendMsgBuf(frame.id, 0, frame.len, frame.data)
        │
        ▼
MCP2515 transmits frame onto CAN bus
```

### Core 1 → Core 0 path (RX):

```
CAN frame arrives on bus
        │
MCP2515 asserts INT pin LOW (GPIO 20)
        │
        ▼
canIrqHandler() fires (interrupt service routine)
  → sets canIrqPending = true
        │
        ▼ (Core 1's loop1())
if (canIrqPending) drainCanRxCore1()
  → reads all available frames from MCP2515
  → for each frame: enqueueCore1EventFrame()
        │
        ▼
serviceCore1ToCore0Fifo()
  → packs frame into 3 uint32 words
  → pushes to Core 1→Core 0 FIFO
        │
        ▼ (Core 0's loop())
serviceFifoFromCore1()
  → reads 3 words, reconstructs CanFrame
  → handleReceivedCanFrame(frame)
        │
        ▼
Dispatch by CAN ID:
  0x1xx → handleHelloFrame()
  0x3xx → handleCommandFrame()
  0x4xx → handleCalibFrame()
  0x5xx → handleQueryFrame()
  0x58x → handleReplyFrame()
  0x6xx → handleStreamFrame()
  0x68x → handleGainExchangeFrame()
  0x7xx → handleConsensusFrame()
  0x78x → handleADMMFrame()
  0x7Cx → handleDualDecompFrame()
```

### Why FIFOs instead of shared memory?

If both cores read and write the same array, you need locks to prevent one core from reading a partially-written value. Locks have overhead and can cause priority inversion. The hardware FIFO avoids all this:
- **Lock-free:** hardware guarantees atomic push/pop
- **Bounded:** if the FIFO is full, the sender knows immediately (push returns false)
- **Zero-copy:** no need to copy data — push from one core, pop from the other
- **Interrupt-safe:** the FIFO is separate from normal memory, no cache coherency issues

---

## The Cooperative Scheduler (Core 0 loop)

Core 0 does not use a Real-Time Operating System (RTOS). Instead, `loop()` runs an infinite loop and calls service functions in order. Each service function checks whether it needs to do anything, does a small amount of work, and returns quickly. This is called a **cooperative scheduler**.

```c
void loop() {
  serviceSerialNonblocking();       // Check for serial commands (non-blocking)
  serviceFifoFromCore1();           // Process received CAN frames
  serviceCore0ToCore1Fifo();        // Push queued TX frames to Core 1
  serviceWakeupStateMachine();      // Handle discovery (timer-based)
  serviceCalibrationStateMachine(); // Handle calibration (timer-based FSM)
  serviceCalibrationPlanBroadcast();// Repeat plan broadcasts
  serviceControlTask();             // THE 100 Hz PI loop
  serviceConsensus();               // Consensus algorithm (timer-based)
  serviceADMM();                    // ADMM algorithm (timer-based)
  serviceDualDecomp();              // Dual decomp algorithm (timer-based)
  servicePeriodicHello();           // Heartbeat every 1 second
  serviceStreamingOutput();         // Telemetry if streaming enabled
  serviceDiagnostics();             // Timeout detection
}
```

Each iteration of `loop()` takes microseconds (much less than 1 ms when there's nothing to do). The loop runs at roughly thousands of iterations per second.

**Key insight:** The control task isn't called every loop iteration — it checks a flag (`controlDueCount > 0`) set by the hardware timer. So the control loop timing depends on the hardware timer, not on loop iteration speed.

### The control timer:

```c
add_repeating_timer_ms(-10, controlTimerCallback, nullptr, &controlTimer);
```

The negative sign means the callback fires 10 ms after the previous callback's **end** (not start). This prevents drift if the callback takes a few microseconds.

```c
bool controlTimerCallback(repeating_timer_t *rt) {
    controlDueCount++;    // Atomically signal that control is needed
    return true;          // Keep repeating
}
```

The callback just increments a counter. The actual control work happens in `serviceControlTask()` when Core 0 gets to it in the loop. If `controlDueCount > 1`, it means the loop was too slow and we missed a control cycle (overrun — logged and counted).

> [!warning] The "overrun" scenario
> If a CAN frame burst takes 3 ms to process and the timer fires twice in that window, controlDueCount becomes 2. serviceControlTask() will execute the control step but also increment controlOverrunCount. This is rare in practice because the loop is very fast, but it's important to monitor.

---

## Boot Sequence: From Power-On to Normal Operation

Here's the complete sequence from when you plug in the USB cable:

```
Power on / USB connected
        │
        ▼
Core 0: setup()
  - Serial.begin(115200)
  - analogReadResolution(12) — 12-bit ADC
  - analogWriteResolution(12) — 12-bit PWM
  - pinMode(LED, OUTPUT), pinMode(LDR, INPUT)
  - setLedPwm(0) — LED off
  - readLuxFiltered() — initial LDR reading
  - start hardware repeating timer at 10 ms
  - print "Introduz o ID do no (1-8):"

Core 1: setup1()  [runs simultaneously]
  - SPI.setRX/setCS/setSCK/setTX
  - SPI.begin()
  - canBus.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ)
  - canBus.setMode(MCP_NORMAL)
  - attachInterrupt(CAN_INT, canIrqHandler, FALLING)

        │
Auto-config: read flash UID → lookup table → nodeId assigned
        │ (no manual input needed — prints "Auto-configured as Node X")
        ▼
startupState = STARTUP_READY
        │
wakeupState = WAKEUP_BOOT → resetRuntimeState() → WAKEUP_DISCOVERY_OPEN
        │
        ▼ (5 seconds: broadcast Hello + CMD_ANNOUNCE every 500 ms)
wakeupState = WAKEUP_DISCOVERY_STABLE
        │
        ▼ (coordinator = Node 1, 500 ms later)
isCoordinator → scheduleCalibrationPlanBroadcast()
             → startCalibrationSession()
             → wakeupState = WAKEUP_RUN
        │
        ▼
Calibration runs (~4.15 s):
  WAIT_START_TIME → BASELINE_SETTLE → BASELINE_MEASURE
  → SLOT_SETTLE 1 → SLOT_MEASURE 1
  → SLOT_SETTLE 2 → SLOT_MEASURE 2
  → SLOT_SETTLE 3 → SLOT_MEASURE 3
  → FINISHED (gainRow computed, gainsReady = true)
        │
        ▼
Gain exchange via CAN (5 seconds window):
  Each node broadcasts K row, baseline, L_ref, cost
  Each node receives peers' data
  allGainsReceived() → initConsensus() / initADMM() / initDualDecomp()
        │
        ▼
NORMAL OPERATION:
  - PI control loop at 100 Hz
  - Distributed algorithm at 10 Hz (CONSENSUS_PERIOD_MS = 100)
  - Hello heartbeat every 1 s
  - Serial commands accepted
```

---

## State Machine: WakeupState

The wakeup FSM has 4 states:

| State | What's happening | How we exit |
|-------|-----------------|-------------|
| WAKEUP_BOOT | Initial state, reset all runtime state | Immediately → DISCOVERY_OPEN |
| WAKEUP_DISCOVERY_OPEN | Broadcasting Hello every 500 ms | After 5 seconds → DISCOVERY_STABLE |
| WAKEUP_DISCOVERY_STABLE | Discovery done; coordinator schedules calibration | Immediately → WAKEUP_RUN |
| WAKEUP_RUN | Normal operation | Never (until restart command) |

---

## Common Exam Questions

> [!question] "Why did you dedicate Core 1 to CAN?"
> The MCP2515 uses SPI for communication. If the control loop (Core 0) and the CAN driver share one core, an SPI access for CAN transmission or reception could start during a critical section of the control loop, corrupting the SPI state and causing lost or garbled CAN frames. By dedicating Core 1 entirely to CAN/SPI, we achieve zero contention — Core 0 never touches SPI, Core 1 never runs control algorithms.

> [!question] "How do the two cores communicate?"
> Through the RP2040's hardware FIFO — two 8-word (32-bit) lock-free queues, one in each direction. CAN frames are packed into 3 uint32 words (word 0 = kind + ID + length, words 1-2 = 8 data bytes) and pushed into the FIFO. The receiving core reads these 3 words and reconstructs the CAN frame. No mutexes or locks are needed because each FIFO has exactly one writer and one reader.

> [!question] "What is a cooperative scheduler and how does it work here?"
> A cooperative scheduler is an execution model where multiple tasks share a single thread. Each task must check whether it has work to do, perform a small unit of work, and return quickly. In our loop(), 13 service functions are called in sequence every iteration. None of them block — they check timing conditions and return immediately if it's not their turn. The 100 Hz control task is the only one driven by a hardware timer flag; all others check wall-clock time (millis()).

> [!question] "What happens if the control loop misses its 10 ms deadline?"
> The hardware timer sets controlDueCount. If the loop is slow and the timer fires again before the first fire is handled, controlDueCount becomes 2. serviceControlTask() detects this (pending > 1), increments controlOverrunCount, runs the control step once (not twice), and logs the overrun. In practice, overruns are rare because loop() is very fast when there's nothing to do.

> [!question] "Why is the RP2040 used here and not a single-core microcontroller?"
> A single-core microcontroller would require complex time-slicing between the control loop and the CAN driver, with mutexes to prevent SPI contention. Every mutex acquisition adds latency and complexity. The RP2040's dual-core design lets us solve this architecturally: complete separation of concerns, no shared resources that need locking, and bounded latency on both cores independently.

---

Back to [[SCDEEC Home]]
