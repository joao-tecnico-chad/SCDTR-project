import serial
import time

ser = serial.Serial('/dev/cu.usbmodem1101', 115200, timeout=1)
time.sleep(2)

# Flush any leftover data
ser.reset_input_buffer()

print("=" * 50)
print("TEST 1: Steady-state metrics (60 seconds)")
print("=" * 50)
print("Keep the box SEALED for this test.")
input("Press Enter to start...")

# Turn off, reset metrics, then turn on HIGH
ser.write(b'o 1 o\n')
time.sleep(1)
ser.reset_input_buffer()

ser.write(b'o 1 h\n')
time.sleep(0.5)
ser.readline()  # consume ack

print("Controller running at HIGH. Waiting 60 seconds...")
for i in range(60, 0, -10):
    print(f"  {i}s remaining...")
    time.sleep(10)

# Collect metrics
metrics = {}
for cmd in [b'g E 1\n', b'g V 1\n', b'g F 1\n', b'g y 1\n', b'g u 1\n', b'g t 1\n']:
    ser.write(cmd)
    time.sleep(0.3)
    resp = ser.readline().decode('utf-8', errors='ignore').strip()
    print(f"  {resp}")
    metrics[cmd.decode().strip()] = resp

# Save metrics
with open('metrics.txt', 'w') as f:
    f.write("=== Steady-state metrics (60s at HIGH) ===\n")
    for k, v in metrics.items():
        f.write(f"{k} -> {v}\n")

print("\nMetrics saved to metrics.txt")

print("\n" + "=" * 50)
print("TEST 2: Disturbance rejection")
print("=" * 50)
print("When prompted, OPEN the box lid for ~2 seconds, then CLOSE it.")
print("The script will capture the controller's response.")
input("Press Enter when ready (box should be sealed)...")

# Start streaming
ser.write(b'o 1 h\n')
time.sleep(1)
ser.reset_input_buffer()

ser.write(b's y 1\n')
time.sleep(0.2)
ser.readline()  # consume ack
ser.write(b's u 1\n')
time.sleep(0.2)
ser.readline()  # consume ack

print("Streaming started. Recording 5 seconds of baseline...")
disturbance_data = []
start = time.time()

# 5 seconds baseline
while time.time() - start < 5:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line and line.startswith('s '):
        disturbance_data.append(line)

print(">>> NOW OPEN THE BOX LID for ~2 seconds, then CLOSE it <<<")

# Record for 15 more seconds (disturbance + recovery)
while time.time() - start < 20:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line and line.startswith('s '):
        disturbance_data.append(line)

# Stop streaming
ser.write(b'S y 1\n')
time.sleep(0.2)
ser.write(b'S u 1\n')
time.sleep(0.2)

# Save disturbance data
with open('disturbance_test.txt', 'w') as f:
    f.write("# Disturbance rejection test\n")
    f.write("# Baseline: 0-5s, Lid open: ~5-7s, Recovery: 7-20s\n")
    for line in disturbance_data:
        f.write(line + '\n')

print(f"\nDisturbance test saved to disturbance_test.txt ({len(disturbance_data)} samples)")

# Also collect jitter data
print("\n" + "=" * 50)
print("TEST 3: Jitter measurement (10 seconds)")
print("=" * 50)

ser.reset_input_buffer()
ser.write(b's y 1\n')
time.sleep(0.2)
ser.readline()

jitter_data = []
start = time.time()
while time.time() - start < 10:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line and line.startswith('s y'):
        # Extract timestamp
        parts = line.split()
        if len(parts) >= 5:
            jitter_data.append(int(parts[4]))

ser.write(b'S y 1\n')
time.sleep(0.2)

# Compute jitter stats
if len(jitter_data) > 1:
    intervals = [jitter_data[i+1] - jitter_data[i] for i in range(len(jitter_data)-1)]
    intervals = [i for i in intervals if i > 0]  # filter zero intervals
    avg = sum(intervals) / len(intervals)
    mn, mx = min(intervals), max(intervals)
    
    with open('jitter.txt', 'w') as f:
        f.write("# Jitter measurement\n")
        f.write(f"# Samples: {len(intervals)}\n")
        f.write(f"# Mean interval: {avg:.2f} ms\n")
        f.write(f"# Min: {mn} ms, Max: {mx} ms\n")
        f.write("# Interval values (ms):\n")
        for iv in intervals:
            f.write(f"{iv}\n")
    
    print(f"  Samples: {len(intervals)}")
    print(f"  Mean interval: {avg:.2f} ms (target: 100 ms)")
    print(f"  Min: {mn} ms, Max: {mx} ms")
    print(f"  Saved to jitter.txt")

ser.write(b'o 1 o\n')
time.sleep(0.5)
ser.close()

print("\n" + "=" * 50)
print("ALL TESTS COMPLETE")
print("Files saved: metrics.txt, disturbance_test.txt, jitter.txt")
print("=" * 50)