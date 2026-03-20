import serial
import time

ser = serial.Serial('/dev/cu.usbmodem1101', 115200, timeout=1)
time.sleep(2)
ser.reset_input_buffer()

def run_step_test(label, b_sp_val, duration_ms=8000, filename=None):
    """Run a HIGH->LOW->HIGH step with a given b_sp value."""
    print(f"\n{'='*50}")
    print(f"TEST: {label} (b_sp={b_sp_val})")
    print(f"{'='*50}")
    
    # Turn off and reset
    ser.write(b'o 1 o\n')
    time.sleep(1)
    ser.reset_input_buffer()
    
    # Set b_sp
    cmd = f'B 1 {b_sp_val}\n'.encode()
    print(f"  Sending: B 1 {b_sp_val}")
    ser.write(cmd)
    time.sleep(0.3)
    resp = ser.readline().decode('utf-8', errors='ignore').strip()
    print(f"  Response: {resp}")
    
    # Verify
    ser.write(b'g B 1\n')
    time.sleep(0.3)
    resp = ser.readline().decode('utf-8', errors='ignore').strip()
    print(f"  Verify b_sp: {resp}")
    
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
            f.write(f"# {label} (b_sp={b_sp_val})\n")
            for line in data:
                f.write(line + '\n')
        print(f"  Saved {len(data)} samples to {filename}")
    
    return data

# =============================================
# SET-POINT WEIGHTING COMPARISON
# =============================================
print("\n" + "="*60)
print("SET-POINT WEIGHTING COMPARISON")
print("="*60)
input("Press Enter to start (box must be sealed)...")

run_step_test("b_sp = 0.0 (no SP weight)", 0.0, filename="bsp_00.txt")
run_step_test("b_sp = 0.5 (default)",       0.5, filename="bsp_05.txt")
run_step_test("b_sp = 1.0 (full SP weight)", 1.0, filename="bsp_10.txt")

# Restore default
ser.write(b'B 1 0.5\n')
time.sleep(0.3)
ser.write(b'o 1 o\n')
time.sleep(0.3)
ser.close()

print("\n" + "="*60)
print("DONE")
print("="*60)
print("Files: bsp_00.txt, bsp_05.txt, bsp_10.txt")
print("Upload all 3 and I'll add the comparison plot to the report.")