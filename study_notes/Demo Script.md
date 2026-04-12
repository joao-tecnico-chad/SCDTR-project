# Demo Script for Oral Presentation

> [!tip] Before starting
> Connect all 3 Picos via USB. Open terminal: `python3 scripts/terminal.py`, select Pico 1.

---

## 1. Show Help
```
help
```
*"Here's our full command interface -- 35 commands for controlling, querying, and monitoring the system."*

---

## 2. Restart & Watch Boot Sequence
```
R
```
Wait for:
```
ack
Wakeup complete. Peers discovered.
Starting calibration...
Calibration complete in 4150 ms
Gain exchange complete. System ready.
```
*"The system auto-configures from flash UID -- no manual setup. Wakeup discovers peers over CAN, then calibration measures the gain matrix automatically in 4.15 seconds."*

---

## 3. Read All Nodes
```
g y 1
g y 2
g y 3
```
*"All nodes reading ambient light. Now let's set a target."*

---

## 4. Set Reference & Enable Feedback
```
r 1 20
r 2 20
r 3 20
f 1 1
f 2 1
f 3 1
g y 1
g y 2
g y 3
```
*"All 3 desks at 20 lux. The feedforward instantly calculates the right duty from the calibrated model."*

---

## 5. Show Duty Differences
```
g u 1
g u 2
g u 3
```
*"Node 2 uses least duty (PWM ~700) because it has the highest self-gain (K=95). Node 1 needs more (PWM ~2100) because its self-gain is only 29.5."*

---

## 6. Step Response
```
r 1 5
g y 1
r 1 25
g y 1
```
*"Step response -- feedforward gives instant tracking. The duty jumps straight to the correct value in one 10ms control cycle, no waiting for the integral."*

---

## 7. Occupancy Change
```
r 1 20
o 1 h
g r 1
g y 1
```
*"Occupancy HIGH -- reference automatically jumps to 30 lux."*

```
o 1 l
g r 1
g y 1
```
*"Back to LOW -- 10 lux. Simulates a person leaving the desk."*

---

## 8. Algorithm Switching
```
r 1 20
A 0
g y 1
```
*"PI only -- no communication between nodes."*

```
A 1
g y 1
g y 2
g y 3
```
*"Consensus ADMM -- nodes cooperate via CAN to minimize energy."*

```
A 2
g y 1
```
*"ADMM -- alternating direction method of multipliers."*

```
A 3
g y 1
```
*"Dual decomposition. All maintain 20 lux -- same result with equal costs, validating all implementations."*

---

## 9. Unequal Costs
```
A 1
C 1 1
C 2 2
C 3 3
g u 1
g u 2
g u 3
```
*"Unequal costs. The optimizer shifts load slightly towards the cheapest node. Effect is small (~1%) because cross-coupling is weak in our box -- K[3][1]=1.31 vs K[3][3]=72.23."*

```
C 1 1
C 2 1
C 3 1
```

---

## 10. Performance Metrics
```
RM
```
Wait 5 seconds:
```
g E 1
g V 1
g F 1
```
*"Energy accumulates over time. Visibility error near zero means the illuminance constraint is satisfied. Flicker ~0.0003 means negligible duty oscillation -- the controller is stable."*

---

## 11. Disturbance Rejection (Physical)
Open the box lid. Wait 3 seconds:
```
g y 1
g y 2
g y 3
```
*"Still at 20 lux despite the disturbance -- the PI controller compensates by adjusting duty cycles."*

Close the lid.

---

## 12. Calibration Report
```
rpt
```
*"Shows the measured gain matrix K. Diagonal dominant -- each LED mainly lights its own desk."*

---

## 13. Last-Minute Buffer
```
g b y 1
```
*"60 seconds of lux history at 100 Hz -- 6000 data points stored locally."*

---

## 14. Final Restart (Show Full Auto-Boot)
```
R
```
Wait for all messages:
```
Auto-configured as Node 1 (UID key: 0xB627)
Wakeup complete. Peers discovered.
Starting calibration...
Calibration complete in 4150 ms
Gain exchange complete. System ready.
```
*"No manual configuration needed. Each Pico auto-identifies from its unique flash ID, discovers peers, calibrates the system model, and is ready for cooperative control -- all automatically."*

---

## Key Numbers to Mention

| What | Value |
|------|-------|
| Control rate | 100 Hz |
| CAN speed | 500 kbps |
| Calibration time | ~4.15 s |
| Step response | < 100 ms |
| Model R² | 0.977 -- 0.993 |
| Steady-state flicker | ~0.0003 s⁻¹ |
| Consensus rho | 2.0 |
| Max iterations | 50 |

---

Back to [[SCDEEC Home]]
