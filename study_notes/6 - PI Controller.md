# 6 - PI Controller with Feedforward and Anti-Windup

> [!abstract] Summary
> The LED duty cycle is controlled by a discrete PI controller running at 100 Hz. It has a feedforward term that gives near-instant reference tracking, and back-calculation anti-windup to prevent integrator saturation. Actual gains: Kp = 0.01, Ki = 0.11, Tt = 0.15.

Back to [[SCDEEC Home]]

---

## Why a Controller at All?

You might think: "We have the linear model L = K*d + o. Just solve for d and set the LED to that duty. Done." And indeed, that's what the **feedforward** term does. So why do we also need a feedback controller?

Because the model is not perfect:
1. **LDR calibration has ~3% error** (R² = 0.977, not 1.0)
2. **Background light o changes** if someone opens a window or turns on other lights
3. **LED output varies with temperature** — an LED that's been on for 10 minutes is slightly dimmer than when it first turned on
4. **Other nodes change their duty** (cross-coupling changes what our sensor reads)

The feedforward gives us a great starting point (very close to correct), and the PI feedback **corrects the residual error** caused by all these imperfections.

> [!example] Analogy: GPS + speedometer
> Feedforward is like GPS — it knows approximately where you are based on a map (the model). PI feedback is like the steering corrections you make moment-to-moment because the GPS map isn't perfectly accurate. Together, you stay on course.

---

## PID Control from Scratch

### The error signal

The **error** is the difference between what you want and what you have:
$$e(t) = L_{ref} - L_{measured}(t)$$

- If $e > 0$: lux is below reference → need to increase duty → LED should get brighter
- If $e < 0$: lux is above reference → need to decrease duty → LED should get dimmer
- If $e = 0$: perfect, hold current duty

### P: Proportional term

The P term says: **react proportionally to how bad the current error is**.

$$u_P = K_p \cdot e(t)$$

With $K_p = 0.01$: if the error is 5 lux, the P term adds 0.05 to the duty output (5% extra duty).

**What P can and can't do:**
- Fast response to errors ✓
- Gets you close to the reference ✓
- Leaves a small "steady-state error" if the reference requires a non-zero steady duty ✗

> [!example] Analogy: Shower temperature
> You turn the knob proportional to how cold you feel. If you're very cold, you turn it a lot. If you're slightly cold, you turn it a little. But you'll always undershoot slightly — you end up just a bit cold because if you were exactly at perfect temperature, you'd stop turning, but then the water is still slightly cold.

### I: Integral term

The I term says: **accumulate the error over time and correct for it**.

$$u_I = K_i \int_0^t e(\tau) d\tau$$

In discrete form (each 10 ms step):
$$u_I(k) = u_I(k-1) + K_i \cdot \Delta t \cdot e(k)$$

The integral keeps growing as long as there's any error, no matter how small. Eventually it gets large enough to eliminate the error completely. When $e = 0$, the integral stops changing — it "remembers" exactly how much duty was needed.

**What I fixes:**
- Steady-state error → with integral, error → 0 guaranteed (for stable systems) ✓
- Slow DC disturbances (like ambient light change) ✓

### D: Derivative term

We don't use D in this project (Kd = 0). The D term reacts to the rate of change of error, which is useful for overshoot reduction but makes the controller sensitive to sensor noise. Since our LDR is noisy (rapidly fluctuating ADC readings), D would amplify that noise into the duty cycle output, causing jitter.

---

## Feedforward: The Key Innovation

### The idea

The PI controller by itself starts at zero duty and integrates up. This takes many control cycles to reach the reference. With the feedforward, we can jump straight to approximately the right duty:

$$u_{ff} = \frac{L_{ref} - o_i}{K_{ii}}$$

This is just the linear model inverted: if we want $L_i = L_{ref}$, and we know $L_i = K_{ii} \cdot d_i + o_i$ (ignoring cross-coupling for the feedforward, since our own LED dominates), then:

$$d_i = \frac{L_{ref} - o_i}{K_{ii}}$$

### Example with Node 1 (reference = 20 lux):

$$u_{ff} = \frac{20.0 - 0.32}{29.50} = \frac{19.68}{29.50} = 0.667$$

So immediately on the next control cycle, the LED jumps to 66.7% duty. The measured lux will be very close to 20, and the PI only needs to correct the small residual.

### In the code (control.h):

```c
// Convert per-node calibrated gain to lux/duty
float staticGain = calib.gainRow[nodeId] * 4095.0f;
if (staticGain > 0.01f) {
    ff_duty = (refLux - calib.baselineLux) / staticGain;
}
ff_duty = constrain(ff_duty, 0.0f, 1.0f);
```

Note: `calib.gainRow[nodeId]` is in lux/PWM_count. Multiplying by 4095 converts to lux/duty.

### Effect on step response

**Without feedforward:** Starting at duty=0, the error is 20 lux. PI integrates at 0.11 per second per lux. It takes several seconds to reach the reference.

**With feedforward:** First cycle, duty jumps to 0.667. Error drops to ~0 instantly. PI makes tiny corrections. **Settling time < 100 ms.**

From our measured step response data (step_response_ff.csv):
- At t=3.316s, reference changes from 5.0 → 25.0 lux
- At t=3.416s (100 ms later), lux is already 25.109 — already at target

---

## Anti-Windup: Preventing Integrator Saturation

### The windup problem

The integral term accumulates error. This is usually good. But consider: suppose the reference is 50 lux, but the LED at 100% duty can only produce 30 lux (the box ceiling is too dim). The error never reaches zero. The integral keeps growing and growing: 100, 1000, 10000...

When the reference later drops back to 15 lux, the integral is now huge and positive. Even though the LED should dim down immediately, the huge integral keeps the duty at 100% until the integral unwinds all the way down. This is **integrator windup**, and it causes massive overshoot and sluggish response.

### Back-calculation anti-windup

The fix: whenever the controller output is **clamped** (saturated at 0 or 1), reduce the integral by the amount it was clamped.

```c
float rawOutput = ff_duty + P + piIntegral;
float clampedOutput = constrain(rawOutput, 0.0f, 1.0f);

// Anti-windup: reduce integral by (overshoot / Tt)
if (antiWindupEnabled) {
    piIntegral += (PI_DT / PI_TT) * (clampedOutput - rawOutput);
}

float duty = ff_duty + P + piIntegral;
duty = constrain(duty, 0.0f, 1.0f);
```

- When `rawOutput` is within [0,1]: `clampedOutput == rawOutput`, so the extra term is zero. No effect.
- When `rawOutput > 1.0`: `clampedOutput - rawOutput` is negative → integral decreases → prevents windup.
- When `rawOutput < 0`: `clampedOutput - rawOutput` is positive → integral increases toward zero.

The parameter **Tt = 0.15** (tracking time constant) controls how quickly the anti-windup acts. Smaller Tt = faster unwind.

> [!example] Analogy: Speed-limited GPS
> Without anti-windup, the GPS tries to make you speed up by 1000 mph to "catch up" on all the time you were stuck in traffic. With anti-windup, it knows you're speed-limited and adjusts its expectation — it doesn't accumulate an impossible debt.

---

## Set-Point Weighting

There's one more subtlety in the code:

```c
float P = PI_KP * (PI_B_SP * refLux - filteredLux);
```

The proportional term uses `b * refLux - filteredLux` instead of just `error = refLux - filteredLux`. With b = 1.0 (our setting: `PI_B_SP = 1.0`), this is equivalent to standard P. But if b < 1, the P term reacts less aggressively to reference steps (the reference is "attenuated" in the P term) while still rejecting disturbances fully (because disturbances appear as changes in `filteredLux`, which has full weight). This can reduce overshoot on large step changes. We use b = 1.0, so it's standard.

---

## The 100 Hz Control Loop — Step by Step

Every 10 ms, the hardware timer fires and sets `controlDueCount++`. When Core 0 reaches `serviceControlTask()` in its loop, it calls `runControlStep()`:

### What happens in one control step:

```
1. Read LDR sensor (32-sample average + low-pass filter, α=0.15)
   → filteredLux updated

2. Update metrics:
   energyJ += P_led * 0.01 s   (energy meter)
   visibilityError += max(0, ref - lux)  (quality metric)
   flicker detection: check if duty direction reversed

3. Push to history buffer (600 samples @ 100 Hz = 6s of data)

4. captureCalibrationSample() (during calibration: accumulate lux average)

5. If feedbackEnabled AND not calibrating:
   a. Compute feedforward: ff = (ref - o) / K_ii
   b. Compute P term: P = Kp * (b*ref - lux)
   c. Update integral: piIntegral += Ki * dt * error
   d. Apply anti-windup
   e. Output: duty = ff + P + piIntegral, clamped to [0,1]
   f. Set LED PWM: localPwm = (uint16_t)(duty * 4095)

6. Record execution time (overrun detection)
```

### Why α = 0.15 for the low-pass filter?

The filter constant α = 0.15 means:
- 85% of the output is the previous filtered value
- 15% is the new raw measurement

This effectively gives the filter a **time constant** of:
$$\tau = \frac{-\Delta t}{\ln(1 - \alpha)} = \frac{-0.01}{\ln(0.85)} \approx 0.063 \text{ s} = 63 \text{ ms}$$

Disturbances faster than ~63 ms are attenuated. The control bandwidth is well above 63 ms (step responses settle in 100 ms), so the filter doesn't significantly slow down the controller's tracking. It just removes high-frequency ADC noise.

---

## Actual PI Gains and What They Mean

From `hardware_config.h`:
```c
const float PI_KP = 0.01f;   // Proportional gain
const float PI_KI = 0.11f;   // Integral gain
const float PI_TT = 0.15f;   // Anti-windup time constant
const float PI_DT = 0.01f;   // 10 ms sampling period
```

> [!warning] Note: these gains are LOWER than the original notes said (0.5 and 0.3)
> The actual code uses Kp=0.01 and Ki=0.11. This is intentional — with the feedforward providing most of the control action, the PI gains only need to correct small residual errors. Large gains would cause oscillation.

### Gain intuition:
- **Kp = 0.01:** A 5 lux error → P adds 0.05 duty (5% brighter/dimmer). Moderate reaction.
- **Ki = 0.11:** Per second, the integral adds Ki × error to duty. A 1 lux error → 0.11 duty per second accumulates → eliminates steady-state error in ~1 second.

### Why these low values work:
Feedforward handles ~95% of the control action. The PI only needs small gains to handle the remaining ~5% residual. Using high PI gains on top of feedforward would cause oscillation because the system would be "over-determined."

---

## Performance Metrics Computed in the Control Loop

| Metric | Formula | Target |
|--------|---------|--------|
| **Energy (J)** | $E += (pwm/4095) \times 1W \times 0.01s$ | Minimize |
| **Visibility error** | $V += \max(0, L_{ref} - L)$ per sample | → 0 |
| **Flicker** | $F += \|Δd_k\| + \|Δd_{k-1}\|$ when direction reverses | → 0 |

These accumulate over the experiment period. To compare algorithms fairly, use the `RM` command to reset them at the same time for each test.

---

## Common Exam Questions

> [!question] "What is a PI controller and why not PID?"
> A PI controller computes a weighted sum of the current error (proportional term) and the accumulated past error (integral term). The integral eliminates steady-state error by continuously pushing the output in the direction of the error until it reaches zero. We omit the derivative term because our LDR sensor is noisy — the D term would amplify high-frequency noise into the duty cycle output, causing visible LED jitter.

> [!question] "What is feedforward and why does it help?"
> Feedforward uses the calibrated linear model (L = K*d + o) to compute, in open-loop, the duty cycle expected to achieve the target lux: d_ff = (ref - o) / K_ii. This gives an immediate best-guess duty without waiting for the integral to wind up. In practice, it reduces settling time from several seconds (pure PI) to one control cycle (~100 ms). The PI only corrects the small residual error due to model imperfections.

> [!question] "What is anti-windup and when does it matter?"
> Anti-windup prevents the integral term from growing unboundedly when the actuator (LED) is saturated. If the reference requires more light than the LED can produce, the error is always positive, and the integral keeps growing. When the reference decreases later, the large integral holds the duty at maximum, causing overshoot. Back-calculation anti-windup feeds the clamping difference back into the integral, preventing growth during saturation. It matters most during large step changes or reference values outside the achievable range.

> [!question] "What is the actual control loop rate and why 100 Hz?"
> 100 Hz (10 ms period). This rate is chosen because: (1) it's fast enough to track all practical disturbances (the box can't change illuminance faster than ~50 ms); (2) the LDR settles in ~5 ms, so sampling faster than 100 Hz doesn't improve accuracy; (3) it matches the Nyquist criterion for our system bandwidth.

> [!question] "What is the set-point weighting parameter b?"
> Set-point weighting b scales the reference in the proportional term: P = Kp * (b*ref - lux). With b = 1.0 (standard), the P term reacts fully to both reference changes and disturbances. With b < 1, reference steps produce less overshoot (because the P term sees a "softer" step) while disturbances (which appear as changes in lux, not in ref) are rejected with full proportional gain. We use b = 1.0.

---

Back to [[SCDEEC Home]]
