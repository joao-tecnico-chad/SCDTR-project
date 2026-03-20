import serial
import time
import sys

PORT = '/dev/cu.usbmodem101'
BAUD = 115200

def connect():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()
    ser.write(b'g y 1\n')
    time.sleep(0.3)
    resp = ser.readline().decode('utf-8', errors='ignore').strip()
    print(f"Connected. Current lux: {resp}")
    return ser

def send_cmd(ser, cmd):
    ser.write((cmd + '\n').encode())
    time.sleep(0.3)
    resp = ser.readline().decode('utf-8', errors='ignore').strip()
    print(f"  [{cmd}] -> {resp}")
    return resp

def run_step_test(ser, hold_s=5):
    """Run HIGH->LOW->HIGH step, return parsed y data with timestamps."""
    send_cmd(ser, 'o 1 o')
    time.sleep(0.5)
    send_cmd(ser, 'o 1 h')
    print(f"  Settling at HIGH for 8s...")
    time.sleep(8)

    ser.reset_input_buffer()
    send_cmd(ser, 's y 1')
    send_cmd(ser, 's u 1')

    data = []

    def collect(duration):
        t0 = time.time()
        while time.time() - t0 < duration:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('s y 1 ') or line.startswith('s u 1 '):
                parts = line.split()
                if len(parts) >= 5:
                    data.append((parts[1], float(parts[3]), int(parts[4])))

    print("  Recording 2s baseline at HIGH...")
    collect(2)

    print("  -> LOW")
    ser.write(b'o 1 l\n')
    collect(hold_s)

    print("  -> HIGH")
    ser.write(b'o 1 h\n')
    collect(hold_s)

    send_cmd(ser, 'S y 1')
    send_cmd(ser, 'S u 1')
    send_cmd(ser, 'o 1 o')

    return data

def analyze(data, label=""):
    y_data = [(v, t) for typ, v, t in data if typ == 'y']
    if len(y_data) < 20:
        print(f"  Not enough data ({len(y_data)} samples)")
        return {}

    values = [v for v, t in y_data]
    timestamps = [t for v, t in y_data]

    results = {}

    # Find HIGH->LOW transition (first big drop)
    for i in range(1, len(values)):
        if values[i] < values[i-1] - 3.0 and values[i-1] > 30:
            results['down'] = analyze_transition(values, timestamps, i, target=10.0, direction='down')
            break

    # Find LOW->HIGH transition (first big rise after the down)
    start_search = results.get('down', {}).get('switch_idx', len(values)//2) + 20
    for i in range(start_search, len(values)):
        if values[i] > values[i-1] + 3.0 and values[i-1] < 20:
            results['up'] = analyze_transition(values, timestamps, i, target=40.0, direction='up')
            break

    print(f"\n  {'='*50}")
    print(f"  RESULTS {label}")
    print(f"  {'='*50}")
    for key, r in results.items():
        direction = "HIGH->LOW" if key == 'down' else "LOW->HIGH"
        st = r.get('settling_s')
        ov = r.get('overshoot_pct', 0)
        ss = r.get('ss_error', 0)
        if st is not None:
            status = "OK" if st < 0.5 else "SLOW"
            print(f"  {direction}: settling={st:.3f}s [{status}]  overshoot={ov:.1f}%  SS_err={ss:.2f} lux")
        else:
            print(f"  {direction}: did not settle within window")
    print(f"  {'='*50}")

    return results

def analyze_transition(values, timestamps, switch_idx, target, direction):
    band = max(abs(target * 0.05), 0.5)
    switch_ts = timestamps[switch_idx]

    post_vals = values[switch_idx:]
    post_ts = timestamps[switch_idx:]

    # Find settling time: scan from end, find last sample outside band
    settled_idx = len(post_vals)  # default: never settled
    for i in range(len(post_vals) - 1, -1, -1):
        if abs(post_vals[i] - target) > band:
            settled_idx = i + 1
            break
    else:
        settled_idx = 0  # always within band

    if settled_idx < len(post_ts):
        settling_ms = post_ts[settled_idx] - switch_ts
        settling_s = settling_ms / 1000.0
    else:
        settling_s = None

    # Overshoot
    if direction == 'down':
        extreme = min(post_vals) if post_vals else target
        pre_val = values[max(0, switch_idx - 1)]
        step_size = pre_val - target
        overshoot_pct = (target - extreme) / step_size * 100 if step_size > 0 else 0
    else:
        extreme = max(post_vals) if post_vals else target
        pre_val = values[max(0, switch_idx - 1)]
        step_size = target - pre_val
        overshoot_pct = (extreme - target) / step_size * 100 if step_size > 0 else 0

    # Steady-state error
    if len(post_vals) >= 20:
        ss_vals = post_vals[-20:]
        ss_error = abs(sum(ss_vals) / len(ss_vals) - target)
    else:
        ss_error = 0

    return {
        'settling_s': settling_s,
        'overshoot_pct': max(0, overshoot_pct),
        'ss_error': ss_error,
        'switch_idx': switch_idx,
    }

def save_data(data, filename):
    with open(filename, 'w') as f:
        for typ, val, ts in data:
            if typ == 'y':
                f.write(f"y {val:.2f} {ts}\n")
            else:
                f.write(f"u {val:.4f} {ts}\n")
    print(f"  Saved {len(data)} samples to {filename}")


if __name__ == '__main__':
    ser = connect()

    # Parse args: pid_tune.py [kp] [ki] [hold] [--no-smith] [--bsp=val] [--tt=val]
    args = sys.argv[1:]
    kp = ki = None
    hold = 5
    extra_cmds = []

    positional = []
    for a in args:
        if a == '--no-smith':
            extra_cmds.append('P 1 0')
        elif a.startswith('--bsp='):
            extra_cmds.append(f'B 1 {a.split("=")[1]}')
        elif a.startswith('--tt='):
            extra_cmds.append(f'Tt 1 {a.split("=")[1]}')
        else:
            positional.append(a)

    if len(positional) >= 1: kp = float(positional[0])
    if len(positional) >= 2: ki = float(positional[1])
    if len(positional) >= 3: hold = int(positional[2])

    # Apply settings
    for cmd in extra_cmds:
        send_cmd(ser, cmd)

    if kp is not None:
        if ki is not None:
            send_cmd(ser, f'K 1 {kp} {ki}')
        else:
            send_cmd(ser, f'K 1 {kp}')

    label = f"Kp={kp} Ki={ki}" if kp else "defaults"
    for cmd in extra_cmds:
        label += f" {cmd}"
    print(f"\n--- Step test: {label}, hold={hold}s ---")

    data = run_step_test(ser, hold_s=hold)

    fname = f"tune_kp{kp}_ki{ki}.txt" if kp else "tune_defaults.txt"
    save_data(data, fname)
    analyze(data, label)

    ser.close()
