"""
Shared serial utilities for SCDTR Phase 2 test scripts.
Connects to the hub node via USB serial, sends commands,
parses responses and streaming data.
"""

import serial
import serial.tools.list_ports
import time
import sys
import re


def find_pico_port():
    """Auto-detect the Pico's serial port."""
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "pico" in desc or "2e8a" in hwid:  # Raspberry Pi VID
            return p.device
    # Fallback: first available port
    if ports:
        return ports[0].device
    return None


def connect(port=None, baud=115200, timeout=2.0):
    """Connect to the Pico. Auto-detects port if not specified."""
    if port is None:
        port = find_pico_port()
        if port is None:
            print("ERROR: No serial port found. Is the Pico connected?")
            sys.exit(1)
    print(f"Connecting to {port} at {baud} baud...")
    ser = serial.Serial(port, baud, timeout=timeout)
    time.sleep(0.5)  # let the connection settle
    ser.reset_input_buffer()
    return ser


def wait_ready(ser, timeout=30):
    """Wait for 'Ready.' message after boot/calibration."""
    start = time.time()
    while time.time() - start < timeout:
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if line:
            print(f"  [{line}]")
        if "Ready." in line:
            return True
    print("WARNING: Timed out waiting for Ready.")
    return False


def send(ser, cmd):
    """Send a command and return the response line."""
    ser.reset_input_buffer()
    ser.write(f"{cmd}\n".encode())
    time.sleep(0.05)
    resp = ser.readline().decode('utf-8', errors='replace').strip()
    return resp


def send_ack(ser, cmd):
    """Send a command that should return 'ack'. Returns True on success."""
    resp = send(ser, cmd)
    if "ack" in resp.lower():
        return True
    print(f"  WARNING: expected ack for '{cmd}', got '{resp}'")
    return False


def query(ser, cmd):
    """Send a getter command and parse the numeric value from the response."""
    resp = send(ser, cmd)
    # Response format: "<var> <id> <value>"
    parts = resp.split()
    if len(parts) >= 3:
        try:
            return float(parts[2])
        except ValueError:
            pass
    # Try to extract any float from response
    match = re.search(r'[-+]?\d*\.?\d+', resp)
    if match:
        return float(match.group())
    print(f"  WARNING: could not parse value from '{resp}'")
    return None


def stream(ser, var, node_id, duration_s, downsample=1):
    """
    Start streaming, collect data, stop streaming.
    var: 'l' (lux) or 'd' (duty)
    Returns list of (value, time_ms) tuples.
    """
    data = []
    ser.reset_input_buffer()

    # Start streaming
    send_ack(ser, f"s {var} {node_id}")

    start = time.time()
    count = 0
    while time.time() - start < duration_s:
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if not line:
            continue
        # Expected: "s <var> <id> <value> <time_ms>"
        if line.startswith(f"s {var}"):
            parts = line.split()
            if len(parts) >= 5:
                try:
                    value = float(parts[3])
                    t_ms = int(parts[4])
                    count += 1
                    if count % downsample == 0:
                        data.append((value, t_ms))
                except (ValueError, IndexError):
                    pass

    # Stop streaming
    send_ack(ser, f"S {var} {node_id}")
    time.sleep(0.1)
    ser.reset_input_buffer()

    return data


def stream_dual(ser, node_id, duration_s):
    """
    Stream both lux and duty simultaneously.
    Returns (lux_data, duty_data) where each is [(value, time_ms), ...].
    """
    lux_data = []
    duty_data = []
    ser.reset_input_buffer()

    send_ack(ser, f"s l {node_id}")
    send_ack(ser, f"s d {node_id}")

    start = time.time()
    while time.time() - start < duration_s:
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) >= 5 and parts[0] == 's':
            try:
                value = float(parts[3])
                t_ms = int(parts[4])
                if parts[1] == 'l':
                    lux_data.append((value, t_ms))
                elif parts[1] == 'd':
                    duty_data.append((value, t_ms))
            except (ValueError, IndexError):
                pass

    send_ack(ser, f"S l {node_id}")
    send_ack(ser, f"S d {node_id}")
    time.sleep(0.1)
    ser.reset_input_buffer()

    return lux_data, duty_data


def save_csv(filename, headers, *columns):
    """Save columns to a CSV file."""
    with open(filename, 'w') as f:
        f.write(",".join(headers) + "\n")
        n = min(len(c) for c in columns)
        for i in range(n):
            row = ",".join(str(columns[j][i]) for j in range(len(columns)))
            f.write(row + "\n")
    print(f"  Saved {filename} ({n} rows)")


def save_stream_csv(filename, data, var_name="value"):
    """Save streaming data [(value, time_ms), ...] to CSV."""
    with open(filename, 'w') as f:
        f.write(f"time_ms,{var_name}\n")
        for value, t_ms in data:
            f.write(f"{t_ms},{value}\n")
    print(f"  Saved {filename} ({len(data)} rows)")


def get_metrics(ser, node_id):
    """Query all performance metrics for a node."""
    e = query(ser, f"g e {node_id}")
    v = query(ser, f"g v {node_id}")
    f = query(ser, f"g f {node_id}")
    return {"energy": e, "visibility": v, "flicker": f}


def get_dist_state(ser):
    """Query distributed controller state (via 'D' command)."""
    ser.reset_input_buffer()
    ser.write(b"D\n")
    time.sleep(0.1)
    lines = []
    while ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if line:
            lines.append(line)
    return "\n".join(lines)


def set_algorithm(ser, alg_id):
    """Set the distributed algorithm. 0=none, 1=consensus, 2=dual, 3=admm."""
    alg_names = {0: "none", 1: "consensus", 2: "dual_decomp", 3: "admm"}
    print(f"  Setting algorithm: {alg_names.get(alg_id, alg_id)}")
    return send_ack(ser, f"A {alg_id}")


def set_occupancy(ser, node_id, state):
    """Set occupancy. 0=off, 1=unoccupied(low), 2=occupied(high)."""
    return send_ack(ser, f"o {node_id} {state}")


def set_cost(ser, node_id, cost):
    """Set energy cost coefficient for a node."""
    return send_ack(ser, f"C {node_id} {cost}")


def wait_converge(ser, timeout=10):
    """Wait for distributed algorithm to converge."""
    start = time.time()
    while time.time() - start < timeout:
        state = get_dist_state(ser)
        if "Conv: 1" in state:
            return True
        time.sleep(0.5)
    print("  WARNING: Algorithm did not converge within timeout")
    return False
