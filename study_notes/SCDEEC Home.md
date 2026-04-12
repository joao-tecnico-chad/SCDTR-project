# SCDTR - Distributed Cooperative Illumination Control
> **Course:** Distributed Real-Time Control Systems (SCDTR), IST 2025/2026
> **Authors:** Joao Rocha, Ricardo Gaspar, Diogo Costa
> **Status:** Phase 2 — Distributed control with CAN bus + optimization algorithms

---

> [!tip] How to use these notes
> If you have zero background, read the files in order 1→10. If you just need a quick refresher before walking into the exam room, jump straight to [[10 - Oral Exam Cheat Sheet]]. Each file has "Common exam questions" at the bottom with model answers.

---

## What is this project in one sentence?

Three smart desk lamps that **talk to each other over a shared wire (CAN bus)** and **cooperatively solve an optimization problem** every 100 milliseconds to minimize electricity use while keeping every desk bright enough for work.

---

## The Ten Topics — Quick Map

### [[1 - Project Overview]]
**The big picture.** What problem are we solving, what are the phases, what does the math look like from 10,000 feet, why does cross-coupling make the optimization interesting (and also why it frustrates us in our specific box). Read this first.

**Key takeaway:** The optimization problem is `min Σ c_i * d_i` subject to `K*d + o ≥ L_ref`. All three algorithms solve this same problem; they just take different routes.

---

### [[2 - Hardware]]
**What's in the box and why.** The Raspberry Pi Pico (RP2040 chip), MCP2515 CAN controller, LED, LDR light sensor. Circuit diagrams, pin assignments, why 16 MHz crystal, why 5V for MCP2515, what a voltage divider is, what differential signaling means.

**Key takeaway:** Every component choice has a reason. The MCP2515 is a dedicated CAN chip that offloads protocol handling from the microcontroller. The 16 MHz crystal is mandatory — 8 MHz gives wrong baud rate.

---

### [[3 - Linear Model & Calibration]]
**The equation everything depends on.** `L = K*d + o`. What each symbol means, why the relationship is linear (R² > 0.977), how calibration measures K automatically at boot using a round-robin LED sweep. The full K matrix with actual measured values.

**Key takeaway:**
```
K = [29.50  12.53   4.03]    o = [0.32]
    [ 4.01  95.04  11.66]        [0.23]
    [ 1.31   9.09  72.23]        [0.03]
```
Diagonal dominant = each LED mainly lights its own desk. Cross-coupling only ~4–17%.

---

### [[4 - CAN Bus Communication]]
**The communication backbone.** CAN bus from scratch: what it is, why we use it, how arbitration works (lower ID wins), how our 11-bit IDs are structured, what each message type does, how one USB cable controls the whole network via the hub node.

**Key takeaway:** CAN is multi-master with collision-free arbitration. Our protocol assigns ID ranges by priority: Hello (0x100) always gets through; algorithm traffic (0x700+) has lowest priority.

---

### [[5 - Dual Core Architecture]]
**Why two cores, and how they share work safely.** SPI contention problem, FIFO queues for lock-free inter-core communication, the cooperative scheduler in `loop()`, the full boot sequence from power-on to normal operation, state machines explained.

**Key takeaway:** Core 0 = control + logic. Core 1 = CAN exclusively. They communicate through the RP2040's hardware FIFO. No mutexes needed because each core owns its own data.

---

### [[6 - PI Controller]]
**How the LED is driven.** PID control from scratch with intuitive analogies, feedforward explained, anti-windup explained, actual gains (Kp=0.01, Ki=0.11), why the step response is < 100 ms, the 100 Hz timing.

**Key takeaway:** Feedforward = "I know the model, so I can jump straight to approximately the right duty cycle." PI = "Now correct the small remaining error." Anti-windup = "Don't let the integral go crazy when we're saturated."

---

### [[7 - Distributed Algorithms]]
**The three optimization algorithms.** Consensus ADMM (our main one), ADMM, and Dual Decomposition. Explains the optimization problem from scratch, walks through each algorithm iteration by iteration, compares them, explains why equal costs give the same result as PI, why unequal costs barely change anything in our box.

**Key takeaway:** All three converge to the same point with equal costs (proves correctness). With unequal costs the differences are < 2% in our box because K[3][1] = 1.31 — Node 1 barely reaches Node 3.

---

### [[8 - Results Explained]]
**What each experiment shows.** Seven tests with actual measured numbers. For each: what we did, what the data shows, what it proves, how to answer if asked. Includes actual CSV data values.

**Key takeaway:** Step response reaches target in one control cycle (10 ms with feedforward). All three algorithms give lux within ±0.1 lux of reference. Disturbance recovery < 1.5 s.

---

### [[9 - Code Walkthrough]]
**How to navigate the firmware.** File structure of the modular Arduino project, what each `.h` file does, key functions explained with their data flow, the serial command reference, how to demo the code during the presentation.

**Key takeaway:** The firmware is split into 11 header files. Start a demo from `loop()` in `CAN_ORDER_modular.ino` and follow the call chain down.

---

### [[10 - Oral Exam Cheat Sheet]]
**Everything you need to survive the exam.** Ready-made answers to 20+ likely questions, numbers to have memorized, things NOT to say, diagrams you should be able to draw on a whiteboard, edge-case questions and their answers.

**Key takeaway:** Know these 9 numbers cold: 100 Hz, 500 kbps, 4.15 s, < 100 ms, 0.977–0.993, Kp=0.01, Ki=0.11, rho=2.0, 50 iterations.

---

## System Architecture at a Glance

```
┌─────────────────────────────────────────────────────────┐
│                    Physical Box                          │
│                                                          │
│  LED₁   LED₂   LED₃    ← ceiling, shining down          │
│   │      │      │                                        │
│  LDR₁  LDR₂  LDR₃    ← desk surface sensors             │
└─────────────────────────────────────────────────────────┘
        │      │      │
   ┌────┴──┐ ┌─┴────┐ ┌┴─────┐
   │ Pico 1│ │Pico 2│ │Pico 3│   ← each with MCP2515
   │Core0  │ │Core0 │ │Core0 │
   │Core1  │ │Core1 │ │Core1 │
   └───┬───┘ └──┬───┘ └──┬───┘
       │        │         │
  ─────┴────────┴─────────┴──── CAN bus (500 kbps)
  [120R]                   [120R]
```

---

## Critical Numbers — Memorize These

| Parameter | Value | Context |
|-----------|-------|---------|
| Control loop rate | **100 Hz** (10 ms) | PI controller update |
| CAN bus speed | **500 kbps** | MCP2515 setting |
| Calibration time | **~4.15 s** | 3 nodes |
| Step response | **< 100 ms** | With feedforward |
| Model fit R² | **0.977 – 0.993** | All 3 nodes |
| PI Kp | **0.01** | Proportional gain |
| PI Ki | **0.11** | Integral gain |
| Consensus ρ (rho) | **2.0** | Penalty parameter |
| Max iterations | **50** | All algorithms |
| Calibration PWM | **2800/4095 ≈ 68%** duty | Per-slot LED level |
| Self-gain K[1][1] | **29.50 lux/duty** | Node 1 |
| Self-gain K[2][2] | **95.04 lux/duty** | Node 2 |
| Self-gain K[3][3] | **72.23 lux/duty** | Node 3 |
