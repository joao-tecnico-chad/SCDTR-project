# 9 - Code Walkthrough

> [!abstract] Summary
> The firmware is split into 11 header files, each handling one concern. This page maps every file to its responsibility, traces the most important data flows, explains key functions, and gives you a script for navigating the code during a presentation.

Back to [[SCDEEC Home]]

---

## Project Structure

Unlike the old monolithic sketch, the modular version uses separate header files. All files are in `CAN_ORDER_modular/`:

| File | Responsibility | Key functions |
|------|---------------|---------------|
| `CAN_ORDER_modular.ino` | Entry point, global variables, Core 1 CAN service | `setup()`, `loop()`, `setup1()`, `loop1()`, `drainCanRxCore1()` |
| `hardware_config.h` | All constants: pins, timing, gains, IDs | Constants only |
| `protocol_defs.h` | Enums, structs, extern declarations | All type definitions |
| `sensing.h` | LDR → lux conversion, LED PWM, peer table | `readLuxFiltered()`, `setLedPwm()`, `updatePeer()` |
| `can_transport.h` | FIFO packing/unpacking, TX/RX queues, frame builders | `enqueueTxFrame()`, `serviceFifoFromCore1()`, `sendHelloFrame()` |
| `calibration.h` | Calibration FSM, wakeup FSM, plan broadcast | `serviceCalibrationStateMachine()`, `serviceWakeupStateMachine()` |
| `consensus.h` | Consensus algorithm + gain exchange | `solveLocalConsensus()`, `serviceConsensus()`, `sendGainRow()` |
| `admm.h` | ADMM algorithm | `admmUpdateD()`, `admmUpdateZ()`, `admmUpdateU()`, `serviceADMM()` |
| `dual_decomp.h` | Dual decomposition algorithm | `ddUpdatePrimal()`, `ddUpdateDual()`, `serviceDualDecomp()` |
| `can_protocol.h` | Receive frame dispatch | `handleReceivedCanFrame()`, all `handle*Frame()` functions |
| `control.h` | PI controller, hello, streaming, diagnostics | `runControlStep()`, `servicePeriodicHello()` |
| `serial_ui.h` | Serial command parser | `handleLineCommand()`, `serviceSerialNonblocking()` |

---

## Global State (protocol_defs.h + main .ino)

Understanding the global variables is key to understanding the firmware.

### Core identity and startup:
```c
uint8_t nodeId = 1;           // This node's ID (1-3, set by user at boot)
uint8_t totalNodes = 1;       // How many nodes in network (set by user)
bool isCoordinator = true;    // True if nodeId == 1
StartupState startupState;    // WAIT_NODE_ID → WAIT_TOTAL_NODES → READY
WakeupState wakeupState;      // BOOT → DISCOVERY_OPEN → STABLE → RUN
```

### Sensing and control:
```c
float filteredLux;            // Current lux reading (LPF applied)
uint16_t localPwm;            // Current LED PWM value (0-4095)
float refLux;                 // Current lux reference/target
float piIntegral;             // PI integral accumulator
float energyCost = 1.0f;      // This node's energy cost coefficient
```

### Metrics:
```c
float energyJ;                // Accumulated energy (J)
float visibilityErrorIntegral; // Sum of (max(0, ref - lux)) per sample
float flickerIntegral;         // Sum of duty direction reversal magnitudes
uint32_t metricSampleCount;    // Number of control samples (for averaging)
```

### Calibration:
```c
CalibrationContext calib;     // Holds gainRow[], baselineLux, state, timers
```

### Algorithm state (all in one structure, shared):
```c
ConsensusState cons;          // Holds K, d, d_avg, L_ref, c — shared by all 3 algorithms
ADMMState admm;               // ADMM-specific: z[], u[] arrays
DualDecompState dd;           // DD-specific: lambda[], alpha
```

---

## Data Flow: Sensor Reading to LED Control

Here's the complete path from photons hitting the LDR to photons coming out of the LED:

```
LDR photons → voltage change
        │
        ▼
GP26 ADC: 32 samples, average
  raw = analogRead(LDR_PIN) × 32 times
  avg_raw = sum / 32
        │
        ▼
Voltage: V = avg_raw / 4095 × 3.3V
Resistance: R_ldr = 10000 × (3.3 - V) / V
Lux: rawLux = 10^((log10(R_ldr) - B) / -0.7)
        │
        ▼
Low-pass filter:
  filteredLux = 0.15 × rawLux + 0.85 × filteredLux_prev
        │
        ▼
PI Controller:
  ff_duty = (refLux - calib.baselineLux) / (calib.gainRow[nodeId] × 4095)
  P = 0.01 × (1.0 × refLux - filteredLux)
  piIntegral += 0.11 × 0.01 × (refLux - filteredLux)
  [anti-windup applied]
  duty = ff_duty + P + piIntegral (clamped to [0,1])
        │
        ▼
LED PWM:
  localPwm = (uint16_t)(duty × 4095)
  analogWrite(GP15, localPwm)
        │
        ▼
LED → photons → desk surface → reflected → LDR
         (loop closes)
```

---

## Data Flow: Algorithm → Reference Update

```
After calibration: gainRow[] available, gainsReady = true
        │
        ▼
Gain exchange via CAN:
  sendGainRow() → CAN frames → peers receive
  handleGainExchangeFrame() → fills cons.K[i][j], cons.o, cons.L_ref, cons.c
  allGainsReceived() → initConsensus() called
        │
        ▼  (every CONSENSUS_PERIOD_MS = 100 ms)
serviceConsensus():
  solveLocalConsensus():
    d_min = max over k of (L_ref[k] - o[k] - Σ_{j≠i} K[k][j] × d_avg[j]) / K[k][i]
    d_unc = d_avg[i] - c[i] / 2.0
    cons.d[i] = max(d_unc, d_min), clamped [0,1]
  sendConsensusProposal() → CAN frame → peers
  (wait for peers' frames to arrive via CAN → handleConsensusFrame())
  updateConsensusAverage():
    d_avg[j] = mean of all received d[j] proposals
  Convergence check: max |Δd_avg| < 0.001
        │
        ▼
Apply to PI reference:
  estLux = o[myIdx] + Σ_j K[myIdx][j] × d_avg[j]
  refLux = estLux
        │
        ▼
Next PI control step uses new refLux
        (see Data Flow above)
```

---

## Key Functions Deep Dive

### `readLuxFiltered()` (sensing.h)

Takes 32 ADC readings, converts to lux via:
1. Average → voltage
2. Voltage → LDR resistance (voltage divider formula)
3. Resistance → lux (power-law: `powf(10, (log10(R) - B) / M)`)
4. Apply exponential moving average (α = 0.15)

The per-node B_PARAM values are: Node 1 = 6.293, Node 2 = 5.928, Node 3 = 5.364. These compensate for individual LDR manufacturing variation.

### `runControlStep()` (control.h)

Called at 100 Hz. The full sequence:
1. `readLuxFiltered()` — refresh sensor
2. Accumulate metrics: energyJ, visibilityErrorIntegral, flickerIntegral
3. `pushHistory()` — circular buffer for `b` command
4. `captureCalibrationSample()` — if calibrating, accumulate into average
5. PI + feedforward (only if feedbackEnabled && !calib.active)
6. Execution time measurement (overrun detection)

### `serviceCalibrationStateMachine()` (calibration.h)

Non-blocking FSM. Checks `millis()` against `calib.phaseDeadlineMs`. When deadline passes, transitions to next state. Never blocks. Called every loop iteration — only does work when it's time.

The measurement accumulation happens in `captureCalibrationSample()` (called from `runControlStep()`), which adds `filteredLux` to `calib.measureAccumulator` every 10 ms during the MEASURE states.

### `solveLocalConsensus()` (consensus.h)

The optimization core. For each illuminance constraint k (k = 0, 1, 2):
1. Compute how much light node k gets from all OTHER nodes' average duties
2. Compute minimum duty for THIS node that satisfies constraint k
3. Take maximum over all k → tightest constraint

Then compare to cost-optimal unconstrained duty. Take the larger (feasibility wins).

### `handleReceivedCanFrame()` (can_protocol.h)

The CAN message dispatcher. Extracts the "base" from the CAN ID and dispatches:

```c
uint16_t base = frame.id & 0x780;  // mask off lower 7 bits (node ID)
switch (base) {
    case 0x100: handleHelloFrame(); break;
    case 0x300: handleCommandFrame(); break;
    case 0x400: handleCalibFrame(); break;
    case 0x500: handleQueryFrame(); break;
    case 0x580: handleReplyFrame(); break;
    case 0x600: handleStreamFrame(); break;
    case 0x680: handleGainExchangeFrame(); break;
    case 0x700: handleConsensusFrame(); break;
    case 0x780: handleADMMFrame(); break;
    case 0x7C0: handleDualDecompFrame(); break;
}
```

### `handleLineCommand()` (serial_ui.h)

The human interface. Parses commands received over USB serial. For commands targeting a remote node (e.g., `r 2 20`), it either:
- Directly creates and sends a CAN command frame (if hub = this node)
- Or changes local state (if targeting this node)

For `g` (get) commands targeting a remote node, it sends a CAN query and registers `pendingRemoteQuery` to track the expected reply.

---

## Serial Command Reference (Complete)

### Setting values:
| Command | Syntax | Effect |
|---------|--------|--------|
| `u` | `u <n> <pwm>` | Set node n's LED to pwm (0-4095) |
| `r` | `r <n> <lux>` | Set node n's lux reference |
| `o` | `o <n> <l/h/o>` | Set occupancy: l=low, h=high, o=off |
| `C` | `C <n> <cost>` | Set energy cost coefficient |
| `O` | `O <lux>` | Set HIGH occupancy bound (broadcast) |
| `U` | `U <lux>` | Set LOW occupancy bound (broadcast) |
| `a` | `a <n> <0/1>` | Anti-windup enable/disable |
| `f` | `f <n> <0/1>` | Feedback enable/disable |
| `A` | `A <0-3>` | Algorithm: 0=PI, 1=Consensus, 2=ADMM, 3=DD |

### Getting values:
| Command | Variable | Returns |
|---------|----------|---------|
| `g y <n>` | y | Current lux (filtered) |
| `g u <n>` | u | Current PWM value |
| `g r <n>` | r | Current reference lux |
| `g E <n>` | E | Accumulated energy (J) |
| `g V <n>` | V | Average visibility error (lux) |
| `g F <n>` | F | Average flicker metric |
| `g o <n>` | o | Occupancy state char |
| `g d <n>` | d | Background lux (baseline from calibration) |
| `g p <n>` | p | Instantaneous power (W) |
| `g t <n>` | t | Uptime (seconds) |
| `g O <n>` | O | HIGH occupancy bound |
| `g U <n>` | U | LOW occupancy bound |
| `g L <n>` | L | Currently active lower bound |
| `g C <n>` | C | Energy cost coefficient |

### System commands:
| Command | Effect |
|---------|--------|
| `R` | Full restart: reset state, re-run wakeup + calibration |
| `RM` | Reset metrics only (energy, visibility, flicker) |
| `c` | Manually trigger calibration (coordinator only) |
| `rpt` | Print calibration report: K matrix, baselines |
| `s y <n>` | Start streaming lux for node n |
| `s u <n>` | Start streaming duty for node n |
| `S y <n>` | Stop lux stream |
| `b` | Print recent history buffer (600 samples at 100 Hz = 6 s) |
| `help` / `h` / `?` | Print help text |

---

## How to Navigate the Code During Presentation

### Recommended order:

**1. Start with `loop()` (CAN_ORDER_modular.ino, line ~165)**
- Show the cooperative scheduler
- "This is the main loop — no RTOS, just 13 service functions called in order"
- "Each function checks if it has work to do and returns immediately if not"

**2. Jump to `runControlStep()` (control.h, line ~20)**
- "This runs every 10 ms, triggered by the hardware timer"
- "First reads the LDR with 32-sample oversampling and low-pass filter"
- Show feedforward: `ff_duty = (refLux - calib.baselineLux) / staticGain`
- Show anti-windup: `piIntegral += (PI_DT / PI_TT) * (clampedOutput - rawOutput)`

**3. Jump to `solveLocalConsensus()` (consensus.h, line ~64)**
- "Each node runs this every 100 ms"
- Trace through the `d_min` computation: "checking all 3 illuminance constraints"
- Show `d_unc = d_avg[i] - c[i] / CONS_RHO`: "cost-optimal unconstrained"
- "Feasibility wins: take the max"

**4. Jump to `serviceCalibrationStateMachine()` (calibration.h, line ~188)**
- "Non-blocking FSM — never blocks, just checks millis()"
- Walk through the switch statement: "baseline settle → measure → slot settle → measure × N → done"
- "finishCalibration() computes the gain row: (slotLux - baseline) / calPwm"

**5. Jump to `handleReceivedCanFrame()` (can_protocol.h)**
- "Every received CAN frame comes through here"
- Show the dispatch switch: "ID range tells us the message type"
- Show `handleGainExchangeFrame()`: "this is how we share K matrix entries"

---

## Architecture Summary Diagram

```
                    ┌──────────── Core 0 ────────────────────────────────┐
                    │                                                     │
   USB Serial ←──►  │  serviceSerialNonblocking()                        │
                    │       ↓ handleLineCommand()                         │
                    │  serviceFifoFromCore1()                             │
                    │       ↓ handleReceivedCanFrame()                    │
                    │            ↓ handleHelloFrame()                     │
                    │            ↓ handleCommandFrame()                   │
                    │            ↓ handleConsensusFrame() → updateAverage │
                    │  serviceControlTask()                               │
                    │       ↓ runControlStep()                            │
                    │            ↓ readLuxFiltered()  ← GP26/ADC          │
                    │            ↓ feedforward + PI                       │
                    │            ↓ setLedPwm()        → GP15/PWM          │
                    │  serviceConsensus() → solveLocalConsensus()         │
                    │       ↓ enqueueTxFrame(CONSENSUS frame)             │
                    │  serviceCore0ToCore1Fifo() → pushes to FIFO         │
                    └─────────────────────────────────────────────────────┘
                                            ↕ RP2040 Hardware FIFO
                    ┌──────────── Core 1 ────────────────────────────────┐
                    │                                                     │
                    │  loop1():                                           │
                    │    serviceCore1IncomingFifo()                       │
                    │         ↓ canBus.sendMsgBuf()    → MCP2515 → bus    │
                    │    if (canIrqPending) drainCanRxCore1()             │
                    │         ↓ canBus.readMsgBuf()   ← MCP2515 ← bus    │
                    │         ↓ enqueueCore1EventFrame()                  │
                    │    serviceCore1ToCore0Fifo()                        │
                    │         ↓ pushes to FIFO                           │
                    └─────────────────────────────────────────────────────┘
```

---

## Common Exam Questions

> [!question] "Walk me through how a CAN frame gets from Node 1's control loop to Node 2's handler."
> Node 1 calls enqueueTxFrame(id, data, len), which adds the frame to a ring buffer. serviceCore0ToCore1Fifo() packs the frame into 3 uint32 words and pushes them through the RP2040 hardware FIFO. On Core 1, serviceCore1IncomingFifo() pops these 3 words, reconstructs the CAN frame, and calls canBus.sendMsgBuf() to transmit on the bus. On Node 2, the MCP2515 receives the frame and asserts INT. Core 1 on Node 2 calls drainCanRxCore1(), reads the frame from MCP2515, and pushes it through the FIFO to Core 0. serviceFifoFromCore1() unpacks it and calls handleReceivedCanFrame(), which dispatches to the appropriate handler.

> [!question] "How does the firmware know it's running at 100 Hz?"
> A hardware repeating timer (add_repeating_timer_ms(-10, callback)) fires every 10 ms. The callback is minimal: it increments controlDueCount atomically. In the main loop, serviceControlTask() checks if controlDueCount > 0, decrements it, and calls runControlStep(). The negative sign in the timer period means the interval is measured from the END of the previous callback, preventing cumulative drift.

> [!question] "Why are all algorithm state variables global?"
> Because the firmware runs as a single-file embedded program without dynamic memory allocation. All state must exist for the entire program lifetime. Globals are allocated in static RAM (SRAM) at compile time. The RP2040 has 264 KB SRAM — more than enough for our 3-node algorithm state (a few hundred bytes total).

> [!question] "How does the serial hub command 'r 2 20' reach Node 2?"
> handleLineCommand() parses the command: variable='r', targetNode=2, value=20.0. Since targetNode != nodeId, the hub path is taken: it constructs a CAN command frame (CMD_SET_REF, payload = (uint16)(20.0 × 100) = 2000) and calls enqueueTxFrame() for CAN ID 0x302. This frame travels through Core 1 to the CAN bus. Node 2 receives it, handleCommandFrame() processes CMD_SET_REF, and sets refLux = 20.0.

---

Back to [[SCDEEC Home]]
