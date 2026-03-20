import serial
import time

PORT = '/dev/cu.usbmodem101'
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)
ser.reset_input_buffer()

def cmd(c):
    ser.write((c + '\n').encode())
    time.sleep(0.3)
    r = ser.readline().decode('utf-8', errors='ignore').strip()
    return r

def collect_y(dur):
    data = []
    t0 = time.time()
    while time.time() - t0 < dur:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line.startswith('s y 1 '):
            p = line.split()
            if len(p) >= 5:
                data.append((float(p[3]), int(p[4])))
    return data

def collect_yu(dur):
    dy, du = [], []
    t0 = time.time()
    while time.time() - t0 < dur:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line.startswith('s y 1 '):
            p = line.split()
            if len(p) >= 5: dy.append((float(p[3]), int(p[4])))
        elif line.startswith('s u 1 '):
            p = line.split()
            if len(p) >= 5: du.append((float(p[3]), int(p[4])))
    return dy, du

# =============================================
# Restore tuned gains (g2 overwrites them)
# =============================================
print("Setting up tuned controller...")
for c in ['K 1 0.01 0.11', 'B 1 1.0', 'Tt 1 0.15', 'FF 1 0', 'P 1 0', 'MF 1 2']:
    cmd(c)

# =============================================
# TEST 1: Step response (HIGH -> LOW -> HIGH)
# =============================================
print("\n=== TEST 1: STEP RESPONSE ===")
cmd('o 1 h')
time.sleep(8)
ser.reset_input_buffer()
cmd('s y 1'); cmd('s u 1')
dy1, du1 = collect_yu(2)
ser.write(b'o 1 l\n')
dy2, du2 = collect_yu(6)
ser.write(b'o 1 h\n')
dy3, du3 = collect_yu(6)
cmd('S y 1'); cmd('S u 1')

with open('test_step.txt', 'w') as f:
    f.write("# Step response: HIGH->LOW->HIGH\n")
    for v, t in dy1 + dy2 + dy3:
        f.write(f"y {v:.2f} {t}\n")
    for v, t in du1 + du2 + du3:
        f.write(f"u {v:.4f} {t}\n")
print(f"  Saved {len(dy1+dy2+dy3)} y + {len(du1+du2+du3)} u samples to test_step.txt")

# =============================================
# TEST 2: Steady-state metrics (60s at HIGH)
# =============================================
print("\n=== TEST 2: STEADY-STATE METRICS (60s) ===")
cmd('o 1 o')
time.sleep(1)
cmd('o 1 h')
print("  Waiting 60s...")
time.sleep(60)
ser.reset_input_buffer()

with open('test_metrics.txt', 'w') as f:
    f.write("# Steady-state metrics after 60s at HIGH\n")
    for c in ['g E 1', 'g V 1', 'g F 1', 'g y 1', 'g u 1', 'g t 1', 'g p 1']:
        ser.reset_input_buffer()
        ser.write((c + '\n').encode())
        time.sleep(0.5)
        r = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"  {c} -> {r}")
        f.write(f"{c} -> {r}\n")
print("  Saved to test_metrics.txt")

# =============================================
# TEST 3: Flicker over time (30s streaming duty at HIGH)
# =============================================
print("\n=== TEST 3: FLICKER / DUTY STREAMING (30s at HIGH) ===")
cmd('o 1 h')
time.sleep(3)
ser.reset_input_buffer()
cmd('s u 1')
du_flicker = []
t0 = time.time()
while time.time() - t0 < 30:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line.startswith('s u 1 '):
        p = line.split()
        if len(p) >= 5:
            du_flicker.append((float(p[3]), int(p[4])))
cmd('S u 1')

with open('test_flicker.txt', 'w') as f:
    f.write("# Duty cycle streaming at HIGH (30s) for flicker analysis\n")
    for v, t in du_flicker:
        f.write(f"{v:.4f} {t}\n")
print(f"  Saved {len(du_flicker)} duty samples to test_flicker.txt")

# =============================================
# TEST 4: Jitter (10s streaming timestamps)
# =============================================
print("\n=== TEST 4: JITTER (10s) ===")
ser.reset_input_buffer()
cmd('s y 1')
jy = collect_y(10)
cmd('S y 1')

with open('test_jitter.txt', 'w') as f:
    f.write("# Jitter: inter-sample intervals (ms)\n")
    if len(jy) > 1:
        intervals = [jy[i+1][1] - jy[i][1] for i in range(len(jy)-1)]
        intervals = [iv for iv in intervals if iv > 0]
        f.write(f"# samples={len(intervals)} mean={sum(intervals)/len(intervals):.2f} min={min(intervals)} max={max(intervals)}\n")
        for iv in intervals:
            f.write(f"{iv}\n")
        print(f"  {len(intervals)} intervals, mean={sum(intervals)/len(intervals):.2f}ms, min={min(intervals)}, max={max(intervals)}")
print("  Saved to test_jitter.txt")

# =============================================
# TEST 5: Anti-windup comparison
# =============================================
print("\n=== TEST 5: ANTI-WINDUP ON vs OFF ===")

for aw_val, label in [(1, "on"), (0, "off")]:
    cmd(f'a 1 {aw_val}')
    cmd('o 1 h')
    time.sleep(6)
    ser.reset_input_buffer()
    cmd('s y 1')
    d1 = collect_y(2)
    ser.write(b'o 1 l\n')
    d2 = collect_y(6)
    ser.write(b'o 1 h\n')
    d3 = collect_y(6)
    cmd('S y 1')
    with open(f'test_aw_{label}.txt', 'w') as f:
        f.write(f"# Anti-windup {label.upper()}\n")
        for v, t in d1 + d2 + d3:
            f.write(f"{v:.2f} {t}\n")
    print(f"  AW {label}: {len(d1+d2+d3)} samples saved to test_aw_{label}.txt")

cmd('a 1 1')  # restore

# =============================================
# TEST 6: Feedback ON vs Feedforward only
# =============================================
print("\n=== TEST 6: FEEDBACK ON vs FEEDFORWARD ONLY ===")

# Feedback ON (PI, no FF)
cmd('f 1 1'); cmd('FF 1 0')
cmd('o 1 h'); time.sleep(6)
ser.reset_input_buffer(); cmd('s y 1')
d1 = collect_y(2)
ser.write(b'o 1 l\n')
d2 = collect_y(6)
ser.write(b'o 1 h\n')
d3 = collect_y(6)
cmd('S y 1')
with open('test_fb_on.txt', 'w') as f:
    f.write("# Feedback ON (PI, no FF)\n")
    for v, t in d1 + d2 + d3:
        f.write(f"{v:.2f} {t}\n")
print(f"  FB ON: {len(d1+d2+d3)} samples saved to test_fb_on.txt")

# Feedforward only (no PI)
cmd('FF 1 1'); cmd('f 1 0')
cmd('o 1 h'); time.sleep(6)
ser.reset_input_buffer(); cmd('s y 1')
d1 = collect_y(2)
ser.write(b'o 1 l\n')
d2 = collect_y(6)
ser.write(b'o 1 h\n')
d3 = collect_y(6)
cmd('S y 1')
with open('test_fb_off.txt', 'w') as f:
    f.write("# Feedforward only (no PI)\n")
    for v, t in d1 + d2 + d3:
        f.write(f"{v:.2f} {t}\n")
print(f"  FF only: {len(d1+d2+d3)} samples saved to test_fb_off.txt")

# Restore
cmd('f 1 1'); cmd('FF 1 0')

# =============================================
# TEST 7: Disturbance rejection
# =============================================
print("\n=== TEST 7: DISTURBANCE REJECTION ===")
print(">>> OPEN the box lid for ~2 seconds when prompted, then CLOSE it <<<")
cmd('o 1 h')
time.sleep(5)
ser.reset_input_buffer()
cmd('s y 1'); cmd('s u 1')

print("  Recording 5s baseline...")
dy_dist, du_dist = collect_yu(5)

input("  >>> OPEN THE LID NOW, wait 2s, then CLOSE and press Enter <<<")

print("  Recording 15s recovery...")
dy2, du2 = collect_yu(15)
dy_dist += dy2; du_dist += du2
cmd('S y 1'); cmd('S u 1')

with open('test_disturbance.txt', 'w') as f:
    f.write("# Disturbance rejection (lid open/close)\n")
    for v, t in dy_dist:
        f.write(f"y {v:.2f} {t}\n")
    for v, t in du_dist:
        f.write(f"u {v:.4f} {t}\n")
print(f"  Saved {len(dy_dist)} y + {len(du_dist)} u samples to test_disturbance.txt")

# =============================================
# DONE
# =============================================
cmd('o 1 o')
ser.close()

print("\n" + "=" * 50)
print("ALL TESTS COMPLETE")
print("=" * 50)
print("Files saved:")
print("  test_step.txt        - step response")
print("  test_metrics.txt     - steady-state metrics")
print("  test_flicker.txt     - duty cycle for flicker analysis")
print("  test_jitter.txt      - jitter intervals")
print("  test_aw_on.txt       - anti-windup ON")
print("  test_aw_off.txt      - anti-windup OFF")
print("  test_fb_on.txt       - feedback ON")
print("  test_fb_off.txt      - feedforward only")
print("  test_disturbance.txt - disturbance rejection")
