# 1 - Project Overview

> [!abstract] Summary
> Three Raspberry Pi Picos each control an LED desk lamp. They measure ambient light with an LDR sensor, communicate over CAN bus, and cooperate to minimize total energy while keeping each desk above its required illuminance. This is the Phase 2 version — Phase 1 had no communication.

Back to [[SCDEEC Home]]

---

## The Physical Setup

Imagine a rectangular wooden box (think shoebox, but bigger). Inside the box:
- **Ceiling:** 3 white LEDs, each pointing straight down at a different desk area
- **Floor/desks:** 3 LDR light sensors, one under each LED
- **Walls:** painted white so light bounces and spreads

Each LED-plus-sensor pair is one **node**. Each node has a **Raspberry Pi Pico** microcontroller and an **MCP2515 CAN bus controller**.

> [!info] Why a box instead of real offices?
> Because we can control exactly what happens. In a box, we know how much light each LED produces and how it spreads. We can open the lid to simulate an external disturbance, change references to simulate occupancy changes, and run repeatable experiments. In a real office, there's sunlight from windows, people walking around, reflections from whiteboards — all uncontrolled.

---

## The Problem in Plain English

Each desk has two possible states:
- **Occupied (HIGH):** Someone is working there → needs **30 lux minimum**
- **Unoccupied (LOW):** Nobody there → only needs **10 lux minimum** (for safety/navigation)

Each LED consumes energy proportional to its brightness (duty cycle). Running all LEDs at full brightness all the time is wasteful. We want the **minimum energy that still satisfies the minimum brightness requirements**.

The complication: **LEDs don't just light their own desk.** Node 2's LED also sends some light to Node 1's sensor and Node 3's sensor. This "spillover" is called **cross-coupling**. If we ignore it, we might overshoot or undershoot. We must model it.

> [!example] Analogy: Sharing a pizza
> Imagine 3 people sharing a pizza. Each person needs at least 2 slices to be full. But here's the twist: when Person 1 eats, some crumbs fall onto Person 2's plate too. So Person 2 might need fewer slices of their own. The optimization problem is: "What's the minimum total pizza slices everyone needs to eat (spend energy), knowing that eating creates spillover?"

---

## Phase 1 vs Phase 2

### Phase 1 — Each lamp works alone (no communication)

In Phase 1, each Pico had an LED and an LDR. It ran a simple PI controller: read the sensor, compare to reference, adjust the LED. No CAN bus, no knowledge of other nodes, no cooperation.

**Limitation:** Node 1 has no idea what Node 2's LED is doing. If Node 2 suddenly floods Node 1's sensor with extra light, Node 1's PI controller will compensate (reduce its own duty), but it doesn't proactively coordinate. And Phase 1 has no way to minimize total energy across all nodes — it just keeps each node at its reference.

### Phase 2 — Cooperative control (our project)

Phase 2 adds:
1. **CAN bus** — all nodes can talk to each other
2. **Automatic calibration** — the system measures K itself at boot
3. **Distributed optimization algorithms** — three different algorithms for finding the minimum-energy solution
4. **Hub / PC interface** — one node connects to a PC via USB and translates between human commands and CAN frames
5. **Wakeup protocol** — nodes discover each other automatically, no hardcoded knowledge of peers

> [!info] Why "distributed"?
> "Distributed" means there is no central computer that sees everything and computes the solution. Instead, each node knows only its own part of the problem (its own row of the gain matrix K, its own reference, its own cost). The nodes exchange proposals over CAN and iteratively converge to the global answer. This is more realistic for large-scale deployments where a central server would be a single point of failure.

---

## The Optimization Problem — Math from Scratch

### Step 1: Variables

Let's define:
- $d_i$ = **duty cycle** of LED $i$ — a number from 0 (off) to 1 (full brightness)
- $L_i$ = **illuminance** measured at sensor $i$ (in lux)
- $L_{ref,i}$ = **minimum required illuminance** at desk $i$ (10 or 30 lux)
- $c_i$ = **energy cost coefficient** for node $i$ (default = 1.0 for all)

### Step 2: The model

We measure that illuminance depends linearly on all duty cycles:

$$L_i = K_{i1} d_1 + K_{i2} d_2 + K_{i3} d_3 + o_i$$

In matrix form:

$$\mathbf{L} = \mathbf{K} \cdot \mathbf{d} + \mathbf{o}$$

Where $\mathbf{K}$ is the **3×3 gain matrix** and $\mathbf{o}$ is the background illuminance (ambient light with all LEDs off).

### Step 3: The objective

We want to minimize total weighted energy:

$$\min_{\mathbf{d}} \; \sum_{i=1}^{3} c_i \cdot d_i$$

In our case, power $\propto$ duty cycle (linear LED model, 1W at full duty), so minimizing duty = minimizing energy.

### Step 4: The constraints

Every desk must get enough light:

$$(\mathbf{K} \cdot \mathbf{d} + \mathbf{o})_i \geq L_{ref,i} \quad \forall i$$

Duty cycles must be physically valid:

$$0 \leq d_i \leq 1 \quad \forall i$$

### Complete formulation:

$$\min_{\mathbf{d}} \sum_i c_i d_i \quad \text{s.t.} \quad \mathbf{K}\mathbf{d} + \mathbf{o} \geq \mathbf{L}_{ref}, \quad \mathbf{0} \leq \mathbf{d} \leq \mathbf{1}$$

This is a **Linear Program (LP)**. It has a unique optimal solution when the cost vector is not orthogonal to the constraint boundary (which happens in practice).

> [!info] Why LP and not something harder?
> The objective function is linear (Σ c_i d_i is a linear function of d). The constraints are also linear (K*d + o ≥ L_ref is just a set of linear inequalities). Linear Programs can be solved very efficiently. Our distributed algorithms exploit this structure — the local subproblems are also LPs or simple quadratics.

---

## The Gain Matrix — What It Means Physically

From our measurements (see [[3 - Linear Model & Calibration]]):

```
          From LED 1    From LED 2    From LED 3
Sensor 1: [ 29.50        12.53         4.03  ]
Sensor 2: [  4.01        95.04        11.66  ]
Sensor 3: [  1.31         9.09        72.23  ]
```

Reading any entry K[i][j]: **"When LED j is at 100% duty, it contributes K[i][j] lux to sensor i"**

Observations:
- **Diagonal values are large:** Each LED mostly lights its own desk (29.5, 95.0, 72.2 lux at full duty)
- **Off-diagonal values are small:** Cross-coupling is weak (1.31 to 12.53 lux)
- **Not symmetric:** K[1][2] = 12.53 but K[2][1] = 4.01 — the box geometry is asymmetric

> [!warning] Why does this limit the optimizer?
> The cross-coupling is only about 4-17% of the self-gain. This means even if Node 1 runs at 100% duty, it contributes at most 4 lux to Node 3 (K[3][1] = 1.31 lux/duty × 1.0 = 1.31 lux total). Node 3 needs 20 lux. Its self-gain is 72.23. So Node 3 must run at about 0.26 duty regardless of what Node 1 does. The optimization has very little room to redistribute load.

---

## Key Design Decisions and Why We Made Them

| Decision | Why |
|----------|-----|
| CAN bus at 500 kbps | Robust, differential signaling, built-in error detection, industry standard |
| Dual-core RP2040 | Separate CAN SPI driver from control loop to avoid bus contention |
| Linear model L = Kd + o | Verified experimentally (R² > 0.977); simple enough for embedded optimization |
| Automatic calibration | Removes need for manual tuning; works even if LEDs/box change |
| 100 Hz control rate | Fast enough for all disturbances; LDR settles in < 5 ms |
| Three optimization algorithms | For comparison; validates that implementations are correct (same answer = good) |
| Cooperative scheduler | No RTOS needed; simple and predictable on bare metal |

---

## Performance Metrics Tracked

The firmware continuously computes three metrics to evaluate quality of service:

### 1. Energy (J)
Accumulated electrical energy since last reset:
$$E = \sum_k P_k \cdot \Delta t = \sum_k \left(\frac{\text{pwm}_k}{4095}\right) \cdot 1 \text{W} \cdot 0.01 \text{s}$$

### 2. Visibility Error (lux)
Average deficit below reference — measures how well we satisfy the constraint:
$$V = \frac{1}{N} \sum_k \max(0, L_{ref} - L_k)$$
Perfect control → V = 0 (never under the reference).

### 3. Flicker Metric
Counts duty-cycle direction reversals (changes from increasing to decreasing or vice versa). High flicker = jittery LED = uncomfortable for users:
$$F = \sum_k |\Delta d_k| + |\Delta d_{k-1}| \text{ when direction changes}$$

---

## Common Exam Questions

> [!question] "What is the overall goal of this project?"
> To implement and compare three distributed optimization algorithms on a network of three smart LED desk lamps. Each lamp has its own microcontroller. They communicate over CAN bus, measure illuminance with LDR sensors, and cooperate to minimize total energy consumption while maintaining minimum illuminance requirements at each desk.

> [!question] "What is the optimization problem you are solving?"
> We minimize the weighted sum of duty cycles (which is proportional to energy) subject to: (1) the illuminance at every desk is above its minimum reference, computed using the calibrated linear model L = K*d + o; and (2) duty cycles are between 0 and 1. This is a linear program.

> [!question] "Why did you use three algorithms?"
> To compare them in practice. With equal cost coefficients, all three should converge to the same operating point — that validates correctness. Then we observe differences in convergence speed, robustness, and sensitivity to step size / penalty parameters. Consensus ADMM proved most robust in our system.

> [!question] "What are the metrics you use to evaluate performance?"
> Energy (accumulated electrical energy in joules), visibility error (average lux deficit below reference, should be near zero), and flicker (duty cycle direction reversals, should be zero in steady state).

> [!question] "Why is the cross-coupling weak in your box?"
> The LEDs are mounted in 3D-printed fixtures on the box ceiling that constrain the light beam. Each LED primarily illuminates the desk directly below it. The physical distance and angle between nodes limits how much light one LED contributes to a non-adjacent sensor.

---

Back to [[SCDEEC Home]]
