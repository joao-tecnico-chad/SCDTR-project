# 10 - Oral Exam Cheat Sheet

> [!abstract] Summary
> Everything you need in one place. Ready-made answers to 25+ likely questions, the 9 numbers to know cold, diagrams to draw on a whiteboard, things to NOT say, and edge-case questions with tricky but correct answers.

Back to [[SCDEEC Home]]

---

## The 9 Numbers — Know These Cold

| What | Value | Memorization tip |
|------|-------|-----------------|
| Control loop rate | **100 Hz / 10 ms** | "100 times per second" |
| CAN bus speed | **500 kbps** | Standard automotive CAN |
| Calibration time | **~4.15 s** | 850 ms baseline + 3×1100 ms |
| Step response | **< 100 ms** | "One control cycle with feedforward" |
| Model R² | **0.977 – 0.993** | "Over 97% linear" |
| PI Kp | **0.01** | Small — feedforward handles the bulk |
| PI Ki | **0.11** | Integral corrects residual |
| Consensus ρ | **2.0** | Penalty / convergence tradeoff |
| Max iterations | **50** | At 100 ms each = 5 seconds maximum |

---

## Quick-Answer Section — 25 Likely Questions

### System Architecture

> [!question] "Describe the overall system."
> Three Raspberry Pi Picos (RP2040), each controlling one LED desk lamp. Each node has an LDR sensor and an MCP2515 CAN controller. The RP2040 runs two cores: Core 0 for all control logic (PI controller at 100 Hz, distributed optimization, serial interface, calibration state machine), and Core 1 exclusively for the MCP2515 CAN bus driver. Three nodes communicate over a 500 kbps CAN bus. One node can connect to a PC via USB serial and act as a hub that translates human commands to CAN frames.

> [!question] "Why two cores on the same chip?"
> The MCP2515 communicates over SPI. If CAN driver and control loop shared one core, SPI accesses from CAN could interrupt the control loop mid-transaction, corrupting the SPI state machine. By dedicating Core 1 entirely to CAN and SPI, we achieve zero contention — each core owns its resources completely. No mutexes needed, bounded CAN latency independent of control computation.

> [!question] "What is the coordinator and what does it do?"
> The coordinator is always Node 1 (the node with ID 1 — lowest ID). It has two special responsibilities: (1) during the wakeup/discovery phase, it triggers automatic calibration after the 5-second discovery window by broadcasting the calibration plan to all nodes; (2) in CAN arbitration, lower IDs win, so Node 1 also has the highest priority Hello heartbeat.

---

### Hardware

> [!question] "What does the MCP2515 do and why do you need it?"
> The RP2040 has no built-in CAN controller. The MCP2515 is an external CAN controller chip (by Microchip) that handles all CAN protocol details: frame encoding/decoding, CRC generation/checking, bit stuffing, collision detection, automatic retransmission, and error counting. It connects to the Pico via SPI at 10 MHz. When a frame arrives, it asserts an interrupt pin (GPIO 20) to wake Core 1.

> [!question] "Why exactly 16 MHz crystal for the MCP2515?"
> The MCP2515 uses the crystal to derive the CAN bit timing. All timing registers (CNFR1, CNFR2, CNFR3) are pre-calculated for a specific crystal frequency in the mcp_can library. With 16 MHz, the library correctly generates 500 kbps. With 8 MHz, the derived baud rate would be 250 kbps — all nodes would fail to communicate with each other because they'd be at different speeds.

> [!question] "How does the LDR measure lux?"
> The LDR is in a voltage divider with a 10 kΩ fixed resistor. The ADC (12-bit, 0–3.3V) reads the voltage at their junction. Higher light = lower LDR resistance = lower voltage. The firmware: (1) averages 32 ADC readings; (2) converts to voltage; (3) computes LDR resistance from voltage divider formula; (4) applies the calibrated power-law model lux = 10^((log10(R) - B) / M) where M = -0.7, B is node-specific; (5) applies exponential low-pass filter with α = 0.15.

> [!question] "Why is common ground critical for CAN?"
> CAN receivers measure the differential voltage between CAN_H and CAN_L. If two nodes have different ground potentials (e.g., one Pico's GND is 0.5V above another's), the absolute levels of CAN_H and CAN_L shift, potentially pushing the common-mode voltage out of specification for the MCP2515's receiver. This causes bit errors or complete failure to receive. All nodes must share a ground wire for the CAN voltage levels to be correctly referenced.

---

### Linear Model and Calibration

> [!question] "Explain the linear model L = K*d + o."
> L is a 3×1 vector of illuminance at each sensor (lux). d is a 3×1 vector of LED duty cycles (0–1). K is a 3×3 gain matrix: K[i][j] = lux contributed to sensor i when LED j runs at 100% duty. o is a 3×1 vector of background illuminance (ambient light with all LEDs off). The model says: illuminance at each desk is the sum of contributions from all LEDs plus ambient light. We verified this is linear with R² > 0.977.

> [!question] "What are the actual K matrix values?"
> ```
> K = [29.50  12.53   4.03]    o = [0.32]
>     [ 4.01  95.04  11.66]        [0.23]
>     [ 1.31   9.09  72.23]        [0.03]
> ```
> Diagonal dominant: each LED mainly lights its own desk (29.5, 95.0, 72.2). Cross-coupling is 4–17% of self-gain.

> [!question] "How does automatic calibration work?"
> Round-robin: (1) All LEDs off → 250 ms settle → 600 ms measure → record background o_i for each sensor. (2) For each source node j (j = 1, 2, 3): LED j on at 2800/4095 ≈ 68% duty → 250+250 ms settle → 600 ms measure → record slot lux. Gain: K_row[j] = (slotLux - baseline) / (2800/4095). (3) Each node broadcasts its gain row, baseline, reference, and cost over CAN. Total: ~4.15 seconds. Automatic at every boot — no manual calibration needed.

> [!question] "Why is calibration done at 68% duty, not 100%?"
> At 100% duty, some LEDs approach thermal saturation, causing brightness to drift during the 600 ms measurement window. At 68%, we still get a strong signal (much above noise floor) while staying in the reliable operating region. Additionally, the linear model is most accurate in the middle of the operating range.

---

### PI Controller

> [!question] "Explain the feedforward term."
> The feedforward uses the calibrated model inverted: if I want L_ref lux, and I know L = K_ii × d + o_i, then d = (L_ref - o_i) / K_ii. For Node 1 targeting 20 lux: d = (20 - 0.32) / 29.50 = 0.667. This duty is applied immediately on the first control cycle, jumping straight to the answer without waiting for the integral to wind up. The PI controller only corrects the small residual error (~2–5% from model imperfections).

> [!question] "What is anti-windup and why is it needed?"
> When the duty cycle saturates at 0 or 1 (actuator limits), the PI integral keeps accumulating error even though the actuator can't do anything more. This "windup" causes large overshoot when conditions change. Back-calculation anti-windup: whenever the raw output is clamped, reduce the integral by (clamp_amount / Tt) × dt. This prevents accumulation when saturated. Tt = 0.15 s sets the rate of unwind.

> [!question] "What does each PI gain do? Kp=0.01, Ki=0.11?"
> Kp = 0.01: The P term reacts proportionally to current error. A 5 lux error → 0.05 duty added. Small value because feedforward handles most of the action — large Kp would cause oscillation on top of feedforward. Ki = 0.11: The integral accumulates error. A 1 lux persistent error → 0.11 duty per second of steady-state correction. Eliminates the offset that P alone cannot.

---

### Distributed Algorithms

> [!question] "Explain Consensus ADMM step by step."
> Starting from the average duty from the previous iteration (d_avg):
> 1. For each illuminance constraint k, compute the minimum duty for this node that keeps desk k's lux ≥ reference (given all other nodes' average duties). Take the tightest constraint as d_min.
> 2. Compute the cost-optimal unconstrained duty: d_unc = d_avg[i] - c_i / ρ.
> 3. Proposed duty = max(d_unc, d_min) — feasibility always wins.
> 4. Broadcast proposal over CAN.
> 5. Average received proposals + own proposal → new d_avg.
> 6. Repeat. Converges when d_avg changes by less than 0.001.

> [!question] "How is ADMM different from Consensus?"
> Standard ADMM has an explicit separation: d-update minimizes cost (local, closed-form: d_i = z_i - u_i - c_i/ρ), z-update projects onto the feasible set (iterative Dykstra projection ensuring illuminance constraints), u-update accumulates the d-z discrepancy (dual variable ascent). Consensus ADMM bundles cost and constraints into one local solve. ADMM has stronger theoretical convergence guarantees but more complex bookkeeping (3 variables: d, z, u vs. just d and d_avg).

> [!question] "Explain Dual Decomposition."
> Assign a "shadow price" λ_k ≥ 0 to each illuminance constraint. The price represents how expensive it would be to violate that constraint. Each node does gradient descent on its duty: d_i -= α × (c_i - Σ_k λ_k K[k][i]). If the shadow prices are high (constraints violated), this pushes duty up (provide more light). Then all nodes update shadow prices: λ_k increases when constraint k is violated, decreases when satisfied. The step size α decays geometrically (0.995 per iteration) for convergence.

> [!question] "Why do all algorithms give the same answer with equal costs?"
> With c = [1,1,1] and equal references, the optimization problem has a unique solution (the minimum-energy point on the feasible boundary). All three algorithms are correct implementations of distributed convex optimization — they all find the same global minimum. The fact that they agree validates the implementations.

> [!question] "Why doesn't unequal cost produce bigger duty differences?"
> K[3][1] = 1.31 lux/duty: even at 100% duty, Node 1 adds only 1.31 lux to Node 3's desk. Node 3 needs 20 lux and its self-gain K[3][3] = 72.23. The minimum duty for Node 3 is: (20 - 0.03 - 1.31×d₁ - 9.09×d₂) / 72.23 ≈ 0.25–0.27 regardless of d₁. The optimizer can't do better than this physically — it's a box geometry limitation.

---

### Results

> [!question] "What were your R² values and what do they mean?"
> R² = 0.977 (Node 1), 0.990 (Node 2), 0.993 (Node 3). R² is the coefficient of determination — it measures how well the linear model explains the variation in measured data. R² = 1.0 would be perfect linearity. Our values mean the linear model explains 97.7%–99.3% of the variance. The remaining 0.7%–2.3% is due to LDR nonlinearity (photoresistors have slightly curved log-log response), temperature drift, and measurement noise.

> [!question] "What happened during the step response test?"
> We changed Node 1's reference from 5 to 25 lux. In the streaming data (100 ms resolution), the system was already at 25.109 lux at the next sample — 100 ms later. This demonstrates that the feedforward enables near-instantaneous tracking. The actual settling is within one 10 ms control cycle; our logging resolution of 100 ms means we can only say "< 100 ms."

> [!question] "What did the disturbance rejection test show?"
> Two tests: (1) Opening the box lid: ambient illuminance at sensors changes suddenly. The PI controller detects the error and compensates within 2–3 control cycles. (2) Forcing Node 2 to 80% duty: Node 2's cross-illumination (K[1][2] × 0.8 = 10 extra lux) raises Node 1's measured lux. Node 1's PI immediately reduces its duty from ~0.41 to ~0.15 to maintain 20 lux. This 63% duty reduction demonstrates correct cross-coupling compensation.

---

### Design Choices

> [!question] "Why CAN bus instead of I2C or UART?"
> CAN: multi-master (any node transmits without a bus master), differential signaling (noise-immune), automatic retransmission on error, deterministic arbitration (lower ID wins), up to 1 Mbps, designed for electrically noisy automotive environments. I2C: master-slave only (needs one node to always be master), not differential, limited to short distances. UART: point-to-point only (can't broadcast to 3 nodes without a hub). For 3 independent nodes in a real-time control context, CAN is clearly the best choice.

> [!question] "Why use cooperative scheduling instead of RTOS?"
> RTOS adds significant complexity (task switching overhead, priority inversion, stack allocation for each task). Our system is simple enough that a cooperative loop works perfectly: the control task is the only hard-deadline task (100 Hz), and it's timer-driven. All other tasks are polling/event-driven and don't need strict timing. The cooperative approach is simpler to reason about and debug, has zero scheduling overhead, and is sufficient for our requirements.

> [!question] "Why implement three algorithms when one would do?"
> To validate correctness (they all give the same answer with equal costs), to compare practical behavior (convergence speed, robustness, sensitivity to parameters), and to demonstrate understanding of different optimization methodologies. Consensus ADMM proved most robust in our box. ADMM showed sensitivity to rapid algorithm switching. Dual Decomposition required careful step-size tuning.

---

## Things NOT to Say

> [!danger] Avoid these phrases — they invite follow-up questions you don't want
- ❌ "The algorithm doesn't work" → ✅ "The algorithm works correctly; the weak cross-coupling in our physical setup limits the observable cost differentiation"
- ❌ "We couldn't get faster data" → ✅ "The 100 ms serial streaming resolution is below the actual 10 ms control resolution; the system operates at 100 Hz"
- ❌ "ADMM was broken" → ✅ "ADMM showed sensitivity to rapid mid-run algorithm switching; when run from initialization it was stable"
- ❌ "The results are all the same" → ✅ "The results validate our implementations — identical outcomes with equal costs confirm all algorithms correctly solve the same LP"
- ❌ "We don't know why R² isn't 1.0" → ✅ "The small deviation from linearity is due to LDR photoresistor nonlinearity at extreme duty levels and temperature drift during the measurement sweep"

---

## Whiteboard Diagrams

### 1. Full system architecture
```
        Core 0                    Core 1
  ┌─────────────────┐       ┌──────────────────┐
  │ PI Control 100Hz │       │ CAN TX/RX        │
  │ Algorithms 10Hz  │◄─────►│ MCP2515 SPI      │
  │ Serial/Hub       │ FIFO  │ IRQ Handler      │
  │ Calibration FSM  │       │ Health Monitor   │
  └─────────────────┘       └──────────────────┘
          │                          │
      USB Serial               CAN Bus (500kbps)
          │                    ┌─────┴──────┐
         PC               Node 2        Node 3
```

### 2. Calibration timeline
```
t=0     t=1.5s    t=2.35s   t=2.95s  t=4.05s  t=4.65s  t=5.75s  t=6.35s
  Plan  |  Wait   |Base|Meas|  Gap  |Slot1M|  Gap  |Slot2M|  Gap  |Slot3M
─────────────────────────────────────────────────────────────────────────►
                              ↑ 250ms settle each side
```

### 3. K matrix heatmap (qualitative)
```
       LED 1    LED 2    LED 3
LDR 1: [HIGH    MED      LOW  ]   ← Node 1 mainly sees its own LED
LDR 2: [LOW     HIGH     MED  ]   ← Node 2 mainly sees its own LED
LDR 3: [VLow   LOW      HIGH  ]   ← Node 3 mainly sees its own LED
```
Numbers: diagonals 29.5 / 95.0 / 72.2, biggest cross K[1][2]=12.53

### 4. Consensus iteration
```
Iter 1: Node1 proposes [0.51, 0.17, 0.27]
        Node2 proposes [0.52, 0.17, 0.27]
        Node3 proposes [0.52, 0.17, 0.27]
        Average: [0.517, 0.170, 0.270]

Iter 2: Similar (small adjustments)
...
Iter 10: Converged: max |Δ| < 0.001
```

---

## Edge-Case Questions (Tricky but Fair)

> [!question] "What happens if one node fails during calibration?"
> The calibration plan has a 5-second timeout for gain exchange. If a node's slot is skipped (it fails before turning on its LED), the other nodes record their background lux as that slot's measurement — resulting in K[j][i] ≈ 0 for that node. The algorithm will then know this node contributes nothing (it's effectively absent), and the remaining nodes run their own control independently.

> [!question] "What happens if the CAN bus is fully loaded and frames are dropped?"
> The TX ring buffer (txQueueLen = 24 frames) absorbs bursts. If the buffer overflows, the oldest frame is dropped (with an error counter incrementing). For algorithm frames (proposals), a dropped frame means one iteration uses stale data from a peer — the algorithm continues with the previous iteration's value for that peer. Consensus still converges (just slower). For Hello frames, if 8 consecutive ones are missed (8 seconds), the peer is removed from the table.

> [!question] "Could your system work with N nodes instead of 3?"
> Yes. The protocol supports up to MAX_NODES = 8. The calibration would take 850 + N × 1100 ms. The gain matrix would be N×N. The consensus algorithm loops over all N nodes. The CAN ID ranges (7 bits for node ID) support up to 127 nodes. The main limitation is the 3-element array in ConsensusState (CONS_MAX_NODES = 3) — this would need to be changed to N.

> [!question] "What is the convergence tolerance and why that value?"
> CONS_TOL = 1e-3 (0.001 duty). This represents 0.1% duty cycle difference — roughly 0.03 lux change at a sensor with self-gain 30 lux/duty. This is well below our measurement noise and below any perceptible change in LED brightness. Tighter tolerance would require more iterations without any practical benefit. Looser tolerance (e.g., 0.01) would mean nodes might differ by 0.3 lux — acceptable in practice but not demonstrably optimal.

> [!question] "How does the anti-windup parameter Tt = 0.15 work?"
> Tt is the "tracking time constant." The anti-windup correction applied per step is (PI_DT / Tt) × (clamped - raw) = (0.01 / 0.15) × clamping_amount. So each 10 ms step, the integral is adjusted by 6.67% of the clamping amount. Complete unwind of a fully saturated integral (if the clamping amount is, say, 0.5) takes about: 0.15 s. Smaller Tt = faster unwind but more aggressive (can destabilize). Larger Tt = slower unwind but smoother.

> [!question] "Why does the coordinator send the calibration plan 5 times?"
> CAN has built-in error detection and retransmission at the physical layer, but if a node is in a brief period of high CPU load (finishing startup, initializing state), it might miss one reception. Sending 5 times with 60 ms gaps ensures that even a node that's busy for up to 240 ms will receive at least one copy. The plan is small (2 frames × 8 bytes) so the redundancy cost is negligible.

> [!question] "What would happen if rho was set to 0 in Consensus?"
> Division by zero in `d_unc = d_avg[i] - c[i] / CONS_RHO`. But conceptually: with ρ → 0, the cost term dominates infinitely — every node wants to set its duty to 0 (free energy). The unconstrained optimum becomes -∞. The constraint clamp `d_min` would catch this and force d_i to the minimum feasible value. But the algorithm would oscillate because every iteration the unconstrained term tries to pull duty to zero. ρ must be positive for the penalty to create a smooth landscape.

---

## 60-Second Summary (for when they cut you off)

> "We built 3 smart desk lamps that control their LEDs cooperatively. Each lamp has a Raspberry Pi Pico — Core 0 runs a 100 Hz PI controller with feedforward for instant tracking, Core 1 handles the MCP2515 CAN bus at 500 kbps. At boot, nodes calibrate automatically in ~4.15 s by round-robin LED sweeping to measure the 3×3 gain matrix K. The system's light model L = K*d + o was verified with R² > 0.977. Three distributed algorithms — Consensus ADMM, ADMM, and Dual Decomposition — minimize total energy subject to minimum illuminance constraints. With equal costs all algorithms converge to the same point, validating the implementations. Step responses settle within 100 ms thanks to feedforward."

---

Back to [[SCDEEC Home]]
