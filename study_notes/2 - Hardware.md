# 2 - Hardware

> [!abstract] Summary
> Each node has 4 main components: Raspberry Pi Pico (RP2040), MCP2515 CAN controller, LED, and LDR sensor. This page explains what each one does, why we chose it, and exactly how they're connected.

Back to [[SCDEEC Home]]

---

## Component 1: Raspberry Pi Pico (RP2040)

### What is it?
A small microcontroller board (about the size of a USB stick) built around the **RP2040** chip made by Raspberry Pi Ltd. It costs about €4.

### Key specs that matter for our project:
- **Dual-core ARM Cortex-M0+** at 133 MHz — this is why we can run two independent "programs" simultaneously (one for control, one for CAN)
- **12-bit ADC** on 3 channels — reads voltages from 0 to 3.3V with 4096 levels of precision
- **12-bit PWM** on almost every pin — drives the LED with 4096 levels (0 = off, 4095 = full brightness)
- **SPI interface** — used to talk to the MCP2515 at 10 MHz
- **264 KB SRAM, 2 MB Flash** — enough for our firmware
- **USB** — used for serial communication with the PC hub

### Why RP2040 specifically?
The dual-core feature is the key reason. We have two tasks that must NOT interfere with each other:
1. The **control loop** at 100 Hz (needs predictable timing)
2. The **CAN bus driver** (uses SPI, responds to interrupts)

If both shared one core, the SPI accesses from CAN could happen during a critical section of the control loop and corrupt data. By putting CAN on Core 1 exclusively, we **eliminate all contention**.

> [!example] Analogy: Two cashiers in a shop
> Imagine one cashier (Core 0) handles customers (control loop), and another cashier (Core 1) handles phone orders (CAN). They share the same till (global state), but each cashier has their own till drawer (FIFO queue). They leave notes for each other in the drawer. No arguing over who controls the till at any moment.

---

## Component 2: MCP2515 CAN Controller

### What is it?
A dedicated chip made by Microchip Technology that handles the entire CAN bus protocol — frame encoding, bit timing, error detection, message buffering — so the microcontroller doesn't have to do it in software.

### Why do we need a separate chip?
The RP2040 does NOT have a built-in CAN controller. We need an external chip. The MCP2515 communicates with the Pico via SPI (4 wires: SCK, MOSI, MISO, CS) and signals when a frame arrives by pulling the INT pin low.

### Critical hardware requirements:

> [!warning] 16 MHz crystal — mandatory
> The MCP2515 needs an external crystal to generate its clock. We use **16 MHz**. If you put an 8 MHz crystal, the baud rate calculation inside the chip will be wrong (it will try to run at 500 kbps but actually run at 250 kbps), and no nodes will be able to communicate. This is a common hardware mistake.

> [!warning] 5V power supply — mandatory
> The MCP2515 is a **5V device**. The Pico GPIO pins are 3.3V, but the MCP2515's VCC must be 5V. We power it from the Pico's **VBUS pin**, which is connected directly to the USB 5V supply. Powering it from the 3.3V rail would give unreliable operation.

### What does it do exactly?
1. **Receives** a CAN frame on the bus (differential signal on CAN_H and CAN_L wires)
2. **Decodes** the frame bits, checks CRC, verifies acknowledge
3. **Stores** the frame in one of two receive buffers (up to 2 frames waiting)
4. **Pulls INT pin LOW** to wake up the Pico via interrupt
5. **Transmits** frames when asked by Pico via SPI

### Internal MCP2515 structure:
```
CAN_H ──┐
        ├── Differential receiver → bit decoder → CRC check → RX Buffer 0
CAN_L ──┘                                                  → RX Buffer 1
                                                              ↕ SPI (10 MHz)
         TX Buffer 0 ──┐                                   Pico (Core 1)
         TX Buffer 1 ──┼── bit encoder → CRC gen → CAN_H
         TX Buffer 2 ──┘                          CAN_L
                                    INT pin (GPIO 20) ← signals frame received
```

---

## Component 3: LED

### What is it?
A standard white LED. In the context of this project, it's a **luminaire** — a light source mounted to illuminate a desk.

### How is it driven?
Via **PWM (Pulse-Width Modulation)**. The Pico's GPIO pin switches the LED on and off at high frequency (much faster than the eye can see, so it looks like a constant brightness). The fraction of time it's on is the **duty cycle**:
- Duty = 0.0 → LED off
- Duty = 0.5 → LED at 50% apparent brightness
- Duty = 1.0 → LED fully on

The RP2040 uses 12-bit PWM, so the software sets a value from 0 to 4095:
```
PWM = 0     → 0% duty   → 0 lux
PWM = 2048  → 50% duty  → ~K_ii * 0.5 lux
PWM = 4095  → 100% duty → ~K_ii lux
```

### Power model
We assume the LED consumes up to **1 Watt at full duty**. Actual power:

$$P = 1 \text{W} \times \frac{\text{PWM}}{4095}$$

This linear power model is used for the energy metric.

> [!info] Why isn't LED brightness actually linear with duty?
> LEDs are slightly nonlinear — doubling the current doesn't quite double the light output. But over the 10–100% duty range we operate in, the linear approximation is good enough. The calibration process measures the actual lux-vs-duty relationship and finds R² > 0.977, confirming acceptable linearity.

---

## Component 4: LDR (Light Dependent Resistor)

### What is it?
An **LDR** (also called a photoresistor or photocell) is a resistor whose resistance decreases when light hits it. In bright light, resistance is low (maybe 1 kΩ). In darkness, resistance is high (maybe 100 kΩ or more).

### Why LDR instead of a "real" lux sensor?
LDRs are cheap, simple, and have good dynamic range. The main downside is their **nonlinearity** (the lux-resistance relationship is a power law, not linear), but we calibrate this out. A proper lux sensor (like a TSL2561) would give digital lux directly but costs more and has a narrower range.

### The voltage divider circuit:

```
3.3V ─────────────────┐
                      │
                   [R_FIXED = 10kΩ]   ← fixed resistor
                      │
                      ├──── ADC pin (GP26)   ← reads V_adc
                      │
                   [R_LDR]             ← LDR, changes with light
                      │
GND ──────────────────┘
```

**How it works:** The voltage at the ADC pin depends on the ratio of R_LDR to R_FIXED. In bright light, R_LDR is small, so most of the voltage drops across R_FIXED → V_adc is LOW. In darkness, R_LDR is large → V_adc is HIGH.

Wait, that seems backwards from what you'd expect. Let me trace through:
- **Bright light:** R_LDR ↓ → voltage divider: V_adc = 3.3V × R_LDR/(R_FIXED + R_LDR) → small value
- **Dark:** R_LDR ↑ → V_adc → large value

So **high ADC reading = dark, low ADC reading = bright**.

### Converting ADC value → lux (step by step):

**Step 1:** Convert raw ADC count to voltage:
$$V = \frac{\text{raw}}{4095} \times 3.3 \text{V}$$

**Step 2:** Convert voltage to LDR resistance (voltage divider formula):
$$R_{LDR} = R_{FIXED} \times \frac{3.3 - V}{V} = 10000 \times \frac{3.3 - V}{V}$$

**Step 3:** Convert resistance to lux using the LDR's calibrated power-law model:
$$\text{lux} = 10^{(\log_{10}(R_{LDR}) - B) / M}$$

Where for our LDRs:
- M = -0.7 (slope — negative because resistance decreases as lux increases)
- B = node-specific: 6.293 (Node 1), 5.928 (Node 2), 5.364 (Node 3)

**Step 4:** Low-pass filter to remove noise:
$$\text{lux}_{filtered} = 0.15 \times \text{lux}_{raw} + 0.85 \times \text{lux}_{filtered,prev}$$

### Oversampling for noise reduction

Before any of the above, the firmware takes **32 ADC readings and averages them**. The 12-bit ADC has quantization noise (last bit or two flickers randomly). Averaging 32 samples reduces noise by $\sqrt{32} \approx 5.7\times$ — effectively giving us about 14.5 bits of precision.

---

## Pin Connections (per node)

| GPIO Pin | Function | Notes |
|----------|----------|-------|
| **GP15** | LED PWM output | Drives the LED via transistor/directly |
| **GP16** | SPI MISO | Data from MCP2515 to Pico |
| **GP17** | SPI CS (CAN chip select) | Active LOW; selects MCP2515 |
| **GP18** | SPI SCK | SPI clock at 10 MHz |
| **GP19** | SPI MOSI | Data from Pico to MCP2515 |
| **GP20** | CAN INT | Falling edge = frame received |
| **GP26** | LDR ADC input | ADC channel 0 |
| **VBUS** | 5V power | Powers MCP2515 VCC |
| **GND** | Common ground | Must be shared with all nodes on CAN |

---

## The Physical Box

### Structure:
```
Top view (ceiling, looking down):
┌─────────────────────────────┐
│   [LED 1]  [LED 2]  [LED 3] │  ← mounted in 3D-printed baffles
│                             │
│   (LDR 1) (LDR 2) (LDR 3)  │  ← on the desk surface/floor
└─────────────────────────────┘
```

### Side view (light paths):
```
[LED 1] ─── direct ──────────────────────────────► (LDR 1)
   │
   └─── cross-coupling (weak) ──────────────────► (LDR 2)
                                                   (LDR 3)
```

### Why white walls?
White walls diffuse the light (Lambertian reflection), making the light distribution more uniform and predictable. The linear model assumption requires that illuminance scales linearly with duty, which is more accurately true when light mixes through diffuse reflection.

### 3D-printed LED fixtures:
Each LED sits in a small printed plastic baffle that directs its beam downward. This **reduces the cross-coupling** (already weak) and prevents light from escaping the box through cracks. It also makes calibration more repeatable.

### CAN bus termination:
CAN bus requires **120 Ω termination resistors** at each end of the cable to prevent signal reflections. Think of it like a microphone cable — if you leave the far end open, the signal echoes back and corrupts the data. The 120 Ω resistor "absorbs" the signal at each end.

**Important:** Only the TWO END nodes get termination resistors. The middle node does NOT get one.

```
Node 1 ────────────────── Node 2 ────────────────── Node 3
[120Ω]                    (no R)                    [120Ω]
  CAN_H ──────────────────────────────────────────────────
  CAN_L ──────────────────────────────────────────────────
  GND   ──────────────────────────────────────────────────
```

> [!warning] Common GND is critical
> If the nodes don't share a common ground, the differential signals are measured relative to different reference voltages, and communication fails. All three nodes must have their GND pins connected together on the same wire as CAN_H and CAN_L.

---

## How the Whole Node Fits Together

```
                    ┌─────────────────────────────────────┐
USB ←───────────►  │          Raspberry Pi Pico           │
(to PC/hub)         │                                     │
                    │  GP26 ◄─────────────────── [LDR]    │
                    │                             [10kΩ]  │
                    │  GP15 ─────────────────────► [LED]  │
                    │                                     │
                    │  GP16(MISO)  ┐                      │
                    │  GP17(CS)    ├──── SPI ──► [MCP2515]│
                    │  GP18(SCK)   │                      │
                    │  GP19(MOSI) ┘                       │
                    │                                     │
                    │  GP20(INT) ◄──────────────[MCP2515] │
                    │                                     │
                    │  VBUS ──────────────────► MCP2515   │
                    │              5V VCC                 │
                    └─────────────────────────────────────┘
                                         ↕
                                 [MCP2515] ──── CAN_H ──── other nodes
                                          ──── CAN_L ──── other nodes
                                          ──── GND   ──── other nodes
```

---

## Common Exam Questions

> [!question] "Why does the MCP2515 need a 16 MHz crystal?"
> The MCP2515 uses the crystal frequency to derive the CAN bit timing (tq — time quanta). It divides the crystal frequency to achieve the desired baud rate. For 500 kbps operation with the library we use (mcp_can), the register values are pre-calculated assuming a 16 MHz crystal. Using 8 MHz would halve the effective baud rate to 250 kbps, making the node incompatible with the rest of the network.

> [!question] "Why is the MCP2515 powered at 5V but the Pico runs at 3.3V?"
> The MCP2515 is a legacy chip designed in the CMOS 5V era. While its I/O pins are nominally 5V tolerant, its internal analog CAN transceiver requires 5V supply to generate the correct differential voltage levels on CAN_H and CAN_L (2.5V common mode, ±2V swing). Running at 3.3V would cause it to produce out-of-spec voltage levels that other nodes might not receive correctly.

> [!question] "Why use LDRs instead of proper lux sensors?"
> LDRs are cheaper, have a wider dynamic range than typical consumer lux ICs, and are sufficient for our purpose since we calibrate the conversion function (power-law model). The calibration process measures the actual lux-resistance relationship for each specific LDR, so sensor-to-sensor variation is compensated.

> [!question] "How does the LDR voltage divider work?"
> The LDR and a 10kΩ fixed resistor form a voltage divider from 3.3V to GND. The ADC reads the voltage at the junction. When the LDR resistance decreases (bright light), this voltage decreases. The firmware inverts the conversion: rLdr = 10000 × (3.3 - V) / V, then applies the calibrated power-law model to get lux.

> [!question] "Why does the CAN bus need 120Ω termination?"
> CAN_H and CAN_L form a differential transmission line with 120 Ω characteristic impedance. Without termination at each end, signal reflections bounce back and can corrupt bits. The 120 Ω resistors at each end absorb the signal energy, preventing reflections. Only the two physical endpoints of the bus get resistors — the middle node does not.

> [!question] "What happens if nodes don't share a common ground?"
> CAN uses differential signaling: the receiver detects the voltage difference (CAN_H − CAN_L). If two nodes have different ground potentials, the absolute voltages on CAN_H and CAN_L shift relative to each other. This can cause the receiver to misinterpret bits or the common-mode voltage to go out of specification, causing communication failure.

---

Back to [[SCDEEC Home]]
