import serial
import time

ser = serial.Serial('/dev/cu.usbmodem1101', 115200, timeout=1)
time.sleep(2)
ser.reset_input_buffer()

def run_step_test(label, setup_cmds, duration_ms=8000, filename=None):
    """Run a HIGH->LOW step and capture the response."""
    print(f"\n{'='*50}")
    print(f"TEST: {label}")
    print(f"{'='*50}")
    
    # Turn off and reset
    ser.write(b'o 1 o\n')
    time.sleep(1)
    ser.reset_input_buffer()
    
    # Apply setup commands
    for cmd in setup_cmds:
        print(f"  Sending: {cmd}")
        ser.write((cmd + '\n').encode())
        time.sleep(0.3)
        resp = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"  Response: {resp}")
    
    # Start at HIGH, let it settle
    ser.write(b'o 1 h\n')
    time.sleep(0.5)
    ser.readline()
    print(f"  Settling at HIGH for 10s...")
    time.sleep(10)
    
    # Start streaming
    ser.reset_input_buffer()
    ser.write(b's y 1\n')
    time.sleep(0.2)
    ser.readline()
    ser.write(b's u 1\n')
    time.sleep(0.2)
    ser.readline()
    
    data = []
    start = time.time()
    
    # Record 3s baseline at HIGH
    print(f"  Recording 3s baseline at HIGH...")
    while time.time() - start < 3:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line and line.startswith('s '):
            data.append(line)
    
    # Switch to LOW
    print(f"  Switching to LOW...")
    ser.write(b'o 1 l\n')
    time.sleep(0.1)
    
    # Record transition
    while time.time() - start < 3 + duration_ms/1000:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line and line.startswith('s '):
            data.append(line)
    
    # Switch back to HIGH
    print(f"  Switching to HIGH...")
    ser.write(b'o 1 h\n')
    time.sleep(0.1)
    
    # Record recovery
    while time.time() - start < 3 + 2*duration_ms/1000:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line and line.startswith('s '):
            data.append(line)
    
    # Stop streaming
    ser.write(b'S y 1\n')
    time.sleep(0.2)
    ser.write(b'S u 1\n')
    time.sleep(0.2)
    
    # Save
    if filename:
        with open(filename, 'w') as f:
            f.write(f"# {label}\n")
            for line in data:
                f.write(line + '\n')
        print(f"  Saved {len(data)} samples to {filename}")
    
    return data

# =============================================
# TEST 1: Anti-windup comparison
# =============================================
print("\n" + "="*60)
print("ANTI-WINDUP COMPARISON")
print("="*60)
input("Press Enter to start anti-windup tests (box must be sealed)...")

# With anti-windup ON (default)
run_step_test(
    "Anti-windup ON",
    ["a 1 1", "f 1 1"],
    duration_ms=8000,
    filename="antiwindup_on.txt"
)

# With anti-windup OFF
run_step_test(
    "Anti-windup OFF",
    ["a 1 0", "f 1 1"],
    duration_ms=8000,
    filename="antiwindup_off.txt"
)

# Restore anti-windup
ser.write(b'a 1 1\n')
time.sleep(0.3)

# =============================================
# TEST 2: Feedforward comparison
# =============================================
print("\n" + "="*60)
print("FEEDFORWARD COMPARISON")
print("="*60)
input("Press Enter to start feedforward tests...")

# With feedback ON (PI + feedforward, default)
run_step_test(
    "PI + feedforward (default)",
    ["a 1 1", "f 1 1"],
    duration_ms=8000,
    filename="setpoint_weight_05.txt"
)

# With feedback OFF (feedforward only)
run_step_test(
    "Feedforward only (no feedback)",
    ["f 1 0", "a 1 1"],
    duration_ms=8000,
    filename="feedforward_only.txt"
)

# Restore
ser.write(b'f 1 1\n')
time.sleep(0.3)
ser.write(b'a 1 1\n')
time.sleep(0.3)
ser.write(b'o 1 o\n')
time.sleep(0.3)

ser.close()

print("\n" + "="*60)
print("ALL COMPARISON TESTS COMPLETE")
print("="*60)
print("Files saved:")
print("  antiwindup_on.txt")
print("  antiwindup_off.txt")
print("  setpoint_weight_05.txt")
print("  feedforward_only.txt")