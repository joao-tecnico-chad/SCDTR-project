#!/usr/bin/env python3
"""
Comprehensive test suite for the distributed illumination system.
Tests every serial command, control algorithm, metrics, streaming,
buffer, and remote CAN queries across 3 connected Pico nodes.

Protocol:
  - On connect, each Pico expects: node ID (1-8), then total nodes (1-8).
  - After configuration, wait for wakeup + CAN calibration (~15s).
  - Then the full command set is available (see 'help' output).
"""

import serial
import time
import sys
from datetime import datetime

# ======================== Configuration ========================

PORTS = {
    1: '/dev/cu.usbmodem1101',
    2: '/dev/cu.usbmodem1201',
    3: '/dev/cu.usbmodem1301',
}
BAUD = 115200
NODE_IDS = [1, 2, 3]
TOTAL_NODES = 3

# Previous test baseline (before PWM fix)
PREV_LUX_BASELINE = 20.0
PREV_LUX_ACCURACY = 0.1

# ======================== Helpers ==============================

passed = 0
failed = 0
skipped = 0
results_log = []


def log_result(test_name, status, detail=""):
    global passed, failed, skipped
    if status == "PASS":
        passed += 1
        tag = "\033[92mPASS\033[0m"
    elif status == "FAIL":
        failed += 1
        tag = "\033[91mFAIL\033[0m"
    else:
        skipped += 1
        tag = "\033[93mSKIP\033[0m"
    line = f"  [{tag}] {test_name}"
    if detail:
        line += f"  --  {detail}"
    print(line)
    results_log.append((test_name, status, detail))


def cmd(ser, command, delay=0.3):
    """Send a command and read the response."""
    ser.reset_input_buffer()
    ser.write(f'{command}\n'.encode())
    time.sleep(delay)
    result = b''
    deadline = time.time() + 2.0
    while time.time() < deadline:
        chunk = ser.read(4096)
        if chunk:
            result += chunk
            time.sleep(0.03)
        else:
            if result:
                break
            time.sleep(0.05)
    text = result.decode('utf-8', errors='replace').strip()
    # Filter CAN error noise and empty lines
    lines = [l for l in text.split('\n')
             if not l.startswith('can_err') and l.strip()]
    return '\n'.join(lines)


def parse_float(response):
    """Extract a float from the last meaningful line of a response."""
    for line in reversed(response.split('\n')):
        parts = line.strip().split()
        if parts:
            try:
                return float(parts[-1])
            except ValueError:
                continue
    return None


def parse_int(response):
    """Extract an int from the last meaningful line."""
    for line in reversed(response.split('\n')):
        parts = line.strip().split()
        if parts:
            try:
                return int(parts[-1])
            except ValueError:
                try:
                    return int(float(parts[-1]))
                except ValueError:
                    continue
    return None


# ======================== Connection & Setup ====================

def connect_and_configure():
    """Connect to all 3 Picos and send node ID + total nodes config."""
    ports = {}
    for nid, port in PORTS.items():
        try:
            s = serial.Serial(port, BAUD, timeout=1)
            time.sleep(0.5)
            s.reset_input_buffer()
            ports[nid] = s
            print(f"  Connected to Node {nid} on {port}")
        except Exception as e:
            print(f"  FAILED Node {nid} ({port}): {e}")

    if len(ports) < TOTAL_NODES:
        return ports

    # Configure each node: send node ID, then total nodes
    for nid in NODE_IDS:
        s = ports[nid]
        s.reset_input_buffer()

        # Send node ID
        s.write(f'{nid}\n'.encode())
        time.sleep(1)
        resp = s.read(4096).decode('utf-8', errors='replace')
        print(f"  Node {nid} ID config: {resp.strip()[:80]}")

        # Send total nodes
        s.write(f'{TOTAL_NODES}\n'.encode())
        time.sleep(1)
        resp = s.read(4096).decode('utf-8', errors='replace')
        print(f"  Node {nid} total config: {resp.strip()[:80]}")

    return ports


def close_all(ports):
    for s in ports.values():
        s.close()


# ======================== Test Functions =======================

def test_01_basic_control(ports):
    """Set ref=20 on all nodes, verify lux readings ~20."""
    print("\n--- Test 1: Basic Control (ref=20 on all nodes) ---")

    for n in NODE_IDS:
        cmd(ports[n], f'r {n} 20', delay=0.2)
        cmd(ports[n], f'f {n} 1', delay=0.2)

    print("  Waiting 3s for settling...")
    time.sleep(3)

    for n in NODE_IDS:
        resp = cmd(ports[n], f'g y {n}', delay=0.3)
        lux = parse_float(resp)
        if lux is not None and 10.0 < lux < 35.0:
            log_result(f"Node {n} lux at ref=20", "PASS",
                       f"lux={lux:.2f} (expected ~20)")
        else:
            log_result(f"Node {n} lux at ref=20", "FAIL",
                       f"lux={lux} (raw: {resp})")


def test_02_algorithms(ports):
    """Switch to each of the 4 algorithms, verify lux ~20."""
    print("\n--- Test 2: All 4 Algorithms ---")

    alg_names = {0: "PI Only", 1: "Consensus", 2: "ADMM", 3: "Dual Decomp"}

    for alg_id, alg_name in alg_names.items():
        resp = cmd(ports[1], f'A {alg_id}', delay=0.5)
        time.sleep(3)

        # Set refs and feedback
        for n in NODE_IDS:
            cmd(ports[n], f'r {n} 20', delay=0.1)
            cmd(ports[n], f'f {n} 1', delay=0.1)

        time.sleep(3)

        lux_vals = []
        for n in NODE_IDS:
            resp = cmd(ports[n], f'g y {n}', delay=0.3)
            lux = parse_float(resp)
            lux_vals.append(lux)

        all_ok = all(l is not None and 5.0 < l < 50.0 for l in lux_vals)
        detail = ", ".join(
            f"N{i+1}={v:.2f}" if v is not None else f"N{i+1}=None"
            for i, v in enumerate(lux_vals)
        )
        log_result(f"Algorithm {alg_id} ({alg_name})",
                   "PASS" if all_ok else "FAIL", detail)

    # Reset to PI only
    cmd(ports[1], 'A 0', delay=0.3)


def test_03_step_response(ports):
    """Step response: node 1 ref 5 -> 25."""
    print("\n--- Test 3: Step Response (Node 1: ref 5 -> 25) ---")

    cmd(ports[1], 'A 0', delay=0.3)
    for n in NODE_IDS:
        cmd(ports[n], f'f {n} 1', delay=0.1)
        cmd(ports[n], f'r {n} 5', delay=0.1)

    time.sleep(3)

    resp = cmd(ports[1], f'g y 1', delay=0.3)
    lux_before = parse_float(resp)
    log_result("Step: lux at ref=5",
               "PASS" if lux_before is not None and lux_before < 15 else "FAIL",
               f"lux={lux_before:.2f}" if lux_before else f"raw={resp}")

    cmd(ports[1], 'r 1 25', delay=0.1)
    time.sleep(3)

    resp = cmd(ports[1], f'g y 1', delay=0.3)
    lux_after = parse_float(resp)
    ok = lux_after is not None and 15.0 < lux_after < 45.0
    log_result("Step: lux at ref=25",
               "PASS" if ok else "FAIL",
               f"lux={lux_after:.2f}" if lux_after else f"raw={resp}")

    if lux_before is not None and lux_after is not None:
        delta = lux_after - lux_before
        log_result("Step: lux increased",
                   "PASS" if delta > 5 else "FAIL",
                   f"delta={delta:.2f}")


def test_04_occupancy(ports):
    """Occupancy change: node 1 LOW -> HIGH -> LOW."""
    print("\n--- Test 4: Occupancy Change (Node 1) ---")

    cmd(ports[1], 'A 0', delay=0.3)
    for n in NODE_IDS:
        cmd(ports[n], f'f {n} 1', delay=0.1)

    # Set to LOW
    cmd(ports[1], 'o 1 l', delay=0.3)
    time.sleep(2)

    resp = cmd(ports[1], 'g r 1', delay=0.3)
    ref_low = parse_float(resp)
    resp = cmd(ports[1], 'g y 1', delay=0.3)
    lux_low = parse_float(resp)
    log_result("Occupancy LOW: ref",
               "PASS" if ref_low is not None else "FAIL",
               f"ref={ref_low}")
    log_result("Occupancy LOW: lux",
               "PASS" if lux_low is not None else "FAIL",
               f"lux={lux_low:.2f}" if lux_low else "None")

    # Set to HIGH
    cmd(ports[1], 'o 1 h', delay=0.3)
    time.sleep(2)

    resp = cmd(ports[1], 'g r 1', delay=0.3)
    ref_high = parse_float(resp)
    resp = cmd(ports[1], 'g y 1', delay=0.3)
    lux_high = parse_float(resp)
    log_result("Occupancy HIGH: ref",
               "PASS" if ref_high is not None else "FAIL",
               f"ref={ref_high}")
    log_result("Occupancy HIGH: lux",
               "PASS" if lux_high is not None else "FAIL",
               f"lux={lux_high:.2f}" if lux_high else "None")

    # Check ref increased
    if ref_low is not None and ref_high is not None:
        log_result("Occupancy: ref_high > ref_low",
                   "PASS" if ref_high > ref_low else "FAIL",
                   f"low={ref_low:.1f}, high={ref_high:.1f}")

    # Set back to LOW
    cmd(ports[1], 'o 1 l', delay=0.3)
    time.sleep(1)

    resp = cmd(ports[1], 'g r 1', delay=0.3)
    ref_back = parse_float(resp)
    log_result("Occupancy: back to LOW",
               "PASS" if ref_back is not None else "FAIL",
               f"ref={ref_back}")


def test_05_unequal_costs(ports):
    """Set costs C1=1, C2=2, C3=3, verify duty allocation."""
    print("\n--- Test 5: Unequal Costs [1, 2, 3] ---")

    # Use consensus
    cmd(ports[1], 'A 1', delay=0.5)
    time.sleep(1)

    cmd(ports[1], 'C 1 1', delay=0.2)
    cmd(ports[1], 'C 2 2', delay=0.2)
    cmd(ports[1], 'C 3 3', delay=0.2)
    for n in NODE_IDS:
        cmd(ports[n], f'r {n} 20', delay=0.1)
        cmd(ports[n], f'f {n} 1', delay=0.1)

    print("  Waiting 5s for convergence...")
    time.sleep(5)

    duties = {}
    for n in NODE_IDS:
        resp = cmd(ports[n], f'g u {n}', delay=0.3)
        d = parse_float(resp)
        duties[n] = d
        log_result(f"Node {n} duty (cost={n})",
                   "PASS" if d is not None else "FAIL",
                   f"PWM={d}" if d is not None else f"raw={resp}")

    # Check: node 1 (lowest cost) should have highest duty
    if all(v is not None for v in duties.values()):
        if duties[1] >= duties[3]:
            log_result("Cost allocation: N1(c=1) >= N3(c=3)", "PASS",
                       f"PWM1={duties[1]}, PWM3={duties[3]}")
        else:
            log_result("Cost allocation: N1(c=1) >= N3(c=3)", "FAIL",
                       f"PWM1={duties[1]}, PWM3={duties[3]}")

    # Reset costs and algorithm
    cmd(ports[1], 'C 1 1', delay=0.1)
    cmd(ports[1], 'C 2 1', delay=0.1)
    cmd(ports[1], 'C 3 1', delay=0.1)
    cmd(ports[1], 'A 0', delay=0.3)


def test_06_metrics(ports):
    """Reset metrics, wait, then read energy/visibility/flicker."""
    print("\n--- Test 6: Performance Metrics ---")

    for n in NODE_IDS:
        cmd(ports[n], f'f {n} 1', delay=0.1)
        cmd(ports[n], f'r {n} 20', delay=0.1)

    # Reset metrics
    resp = cmd(ports[1], 'RM', delay=0.3)
    log_result("Reset metrics (RM)",
               "PASS" if 'err' not in resp.lower() else "FAIL",
               f"response={resp}")

    print("  Waiting 5s for metric accumulation...")
    time.sleep(5)

    resp = cmd(ports[1], 'g E 1', delay=0.3)
    energy = parse_float(resp)
    log_result("Metric: energy (g E 1)",
               "PASS" if energy is not None and energy > 0 else "FAIL",
               f"energy={energy}" if energy is not None else f"raw={resp}")

    resp = cmd(ports[1], 'g V 1', delay=0.3)
    vis = parse_float(resp)
    log_result("Metric: visibility (g V 1)",
               "PASS" if vis is not None else "FAIL",
               f"visibility={vis}" if vis is not None else f"raw={resp}")

    resp = cmd(ports[1], 'g F 1', delay=0.3)
    flicker = parse_float(resp)
    log_result("Metric: flicker (g F 1)",
               "PASS" if flicker is not None else "FAIL",
               f"flicker={flicker}" if flicker is not None else f"raw={resp}")


def test_07_all_get_commands(ports):
    """Test every get command from the spec."""
    print("\n--- Test 7: All Get Commands ---")

    # Ensure known state
    cmd(ports[1], 'A 0', delay=0.3)
    for n in NODE_IDS:
        cmd(ports[n], f'f {n} 1', delay=0.1)
        cmd(ports[n], f'r {n} 20', delay=0.1)
    time.sleep(2)

    # g y <i> - lux (all nodes)
    for n in NODE_IDS:
        resp = cmd(ports[n], f'g y {n}', delay=0.3)
        v = parse_float(resp)
        log_result(f"g y {n} (lux)", "PASS" if v is not None else "FAIL",
                   f"{v:.2f}" if v is not None else f"raw={resp}")

    # g u <i> - PWM (all nodes)
    for n in NODE_IDS:
        resp = cmd(ports[n], f'g u {n}', delay=0.3)
        v = parse_float(resp)
        log_result(f"g u {n} (PWM)", "PASS" if v is not None else "FAIL",
                   f"{v}" if v is not None else f"raw={resp}")

    # g r <i> - reference (all nodes)
    for n in NODE_IDS:
        resp = cmd(ports[n], f'g r {n}', delay=0.3)
        v = parse_float(resp)
        log_result(f"g r {n} (reference)", "PASS" if v is not None else "FAIL",
                   f"{v:.2f}" if v is not None else f"raw={resp}")

    # Single-node get commands on node 1
    get_tests = [
        ('g o 1', 'occupancy'),
        ('g a 1', 'anti-windup state'),
        ('g f 1', 'feedback state'),
        ('g d 1', 'background illuminance'),
        ('g p 1', 'power'),
        ('g t 1', 'elapsed time'),
        ('g E 1', 'energy'),
        ('g V 1', 'visibility error'),
        ('g F 1', 'flicker'),
        ('g O 1', 'HIGH bound'),
        ('g U 1', 'LOW bound'),
        ('g L 1', 'current lower bound'),
        ('g C 1', 'cost coefficient'),
    ]

    for get_cmd, label in get_tests:
        resp = cmd(ports[1], get_cmd, delay=0.3)
        v = parse_float(resp)
        if v is None:
            v = parse_int(resp)
        ok = v is not None and 'err' not in resp.lower()
        log_result(f"{get_cmd} ({label})", "PASS" if ok else "FAIL",
                   f"{v}" if v is not None else f"raw={resp}")


def test_08_buffer(ports):
    """Test buffer dump command."""
    print("\n--- Test 8: Buffer Command (g b y 1) ---")

    # Let some data accumulate
    time.sleep(1)

    resp = cmd(ports[1], 'g b y 1', delay=3.0)
    # Expect comma-separated float values
    if resp and ',' in resp:
        parts = resp.replace('\n', '').split(',')
        try:
            vals = [float(p.strip()) for p in parts[:10] if p.strip()]
            log_result("Buffer dump (g b y 1)", "PASS",
                       f"{len(parts)} values, first 5: {[f'{v:.1f}' for v in vals[:5]]}")
        except ValueError:
            log_result("Buffer dump (g b y 1)", "FAIL",
                       f"Could not parse. First 100 chars: {resp[:100]}")
    elif resp and any(c.isdigit() for c in resp):
        log_result("Buffer dump (g b y 1)", "PASS",
                   f"Got data: {resp[:120]}")
    else:
        log_result("Buffer dump (g b y 1)", "FAIL",
                   f"raw={resp[:200]}")


def test_09_set_commands(ports):
    """Test various set commands and read back."""
    print("\n--- Test 9: Set Commands (write then read back) ---")

    # u <node> <pwm> - set PWM directly
    resp = cmd(ports[1], 'u 1 2048', delay=0.3)
    log_result("Set PWM: u 1 2048",
               "PASS" if 'ack' in resp.lower() or 'err' not in resp.lower() else "FAIL",
               f"response={resp}")
    time.sleep(0.5)
    resp = cmd(ports[1], 'g u 1', delay=0.3)
    v = parse_float(resp)
    log_result("Read back PWM: g u 1",
               "PASS" if v is not None and abs(v - 2048) < 200 else "FAIL",
               f"got {v}" if v is not None else f"raw={resp}")

    # Restore feedback
    cmd(ports[1], 'f 1 1', delay=0.2)

    # r <node> <lux> - set reference
    cmd(ports[1], 'r 1 15', delay=0.2)
    time.sleep(0.5)
    resp = cmd(ports[1], 'g r 1', delay=0.3)
    v = parse_float(resp)
    log_result("Set ref: r 1 15 -> g r 1",
               "PASS" if v is not None and abs(v - 15.0) < 2.0 else "FAIL",
               f"got {v}" if v is not None else f"raw={resp}")

    # o <node> <l/h> - occupancy
    cmd(ports[1], 'o 1 h', delay=0.3)
    resp = cmd(ports[1], 'g o 1', delay=0.3)
    v_str = resp.strip().split()[-1] if resp.strip() else ""
    log_result("Set occupancy HIGH: o 1 h -> g o 1",
               "PASS" if v_str and ('h' in v_str.lower() or v_str == '2' or 'HIGH' in resp.upper()) else "FAIL",
               f"got: {resp}")

    cmd(ports[1], 'o 1 l', delay=0.3)
    resp = cmd(ports[1], 'g o 1', delay=0.3)
    v_str = resp.strip().split()[-1] if resp.strip() else ""
    log_result("Set occupancy LOW: o 1 l -> g o 1",
               "PASS" if v_str and ('l' in v_str.lower() or v_str == '1' or 'LOW' in resp.upper()) else "FAIL",
               f"got: {resp}")

    # a <node> <0/1> - anti-windup
    cmd(ports[1], 'a 1 1', delay=0.2)
    resp = cmd(ports[1], 'g a 1', delay=0.3)
    log_result("Set anti-windup on: a 1 1",
               "PASS" if '1' in resp else "FAIL",
               f"response={resp}")

    cmd(ports[1], 'a 1 0', delay=0.2)
    resp = cmd(ports[1], 'g a 1', delay=0.3)
    log_result("Set anti-windup off: a 1 0",
               "PASS" if '0' in resp else "FAIL",
               f"response={resp}")

    # f <node> <0/1> - feedback
    cmd(ports[1], 'f 1 0', delay=0.2)
    resp = cmd(ports[1], 'g f 1', delay=0.3)
    log_result("Set feedback off: f 1 0",
               "PASS" if '0' in resp else "FAIL",
               f"response={resp}")

    cmd(ports[1], 'f 1 1', delay=0.2)
    resp = cmd(ports[1], 'g f 1', delay=0.3)
    log_result("Set feedback on: f 1 1",
               "PASS" if '1' in resp else "FAIL",
               f"response={resp}")

    # C <node> <cost> - cost coefficient
    cmd(ports[1], 'C 1 3.5', delay=0.2)
    resp = cmd(ports[1], 'g C 1', delay=0.3)
    v = parse_float(resp)
    log_result("Set cost: C 1 3.5 -> g C 1",
               "PASS" if v is not None and abs(v - 3.5) < 0.5 else "FAIL",
               f"got {v}" if v is not None else f"raw={resp}")

    # O <lux> - HIGH occupancy bound
    cmd(ports[1], 'O 50', delay=0.2)
    resp = cmd(ports[1], 'g O 1', delay=0.3)
    v = parse_float(resp)
    log_result("Set HIGH bound: O 50 -> g O 1",
               "PASS" if v is not None and abs(v - 50) < 5 else "FAIL",
               f"got {v}" if v is not None else f"raw={resp}")

    # U <lux> - LOW occupancy bound
    cmd(ports[1], 'U 10', delay=0.2)
    resp = cmd(ports[1], 'g U 1', delay=0.3)
    v = parse_float(resp)
    log_result("Set LOW bound: U 10 -> g U 1",
               "PASS" if v is not None and abs(v - 10) < 5 else "FAIL",
               f"got {v}" if v is not None else f"raw={resp}")

    # Restore defaults
    cmd(ports[1], 'C 1 1', delay=0.1)
    cmd(ports[1], 'r 1 20', delay=0.1)
    cmd(ports[1], 'f 1 1', delay=0.1)
    cmd(ports[1], 'a 1 1', delay=0.1)


def test_10_streaming(ports):
    """Test streaming start/stop."""
    print("\n--- Test 10: Streaming (s y 1 / S y 1) ---")

    cmd(ports[1], 'f 1 1', delay=0.1)
    cmd(ports[1], 'r 1 20', delay=0.1)
    time.sleep(1)

    # Start streaming lux
    ports[1].reset_input_buffer()
    resp_start = cmd(ports[1], 's y 1', delay=0.5)
    log_result("Stream start: s y 1",
               "PASS" if 'ack' in resp_start.lower() or 'err' not in resp_start.lower() else "FAIL",
               f"response={resp_start}")

    # Collect streaming data for 2 seconds
    time.sleep(2)
    stream_data = b''
    deadline = time.time() + 1.0
    while time.time() < deadline:
        chunk = ports[1].read(4096)
        if chunk:
            stream_data += chunk
        else:
            break
    stream_text = stream_data.decode('utf-8', errors='replace')

    line_count = len([l for l in stream_text.split('\n') if l.strip()])
    has_stream = line_count > 3
    log_result("Stream data received",
               "PASS" if has_stream else "FAIL",
               f"{line_count} lines, sample: {stream_text[:150].strip()}")

    # Stop streaming
    resp_stop = cmd(ports[1], 'S y 1', delay=0.5)
    log_result("Stream stop: S y 1",
               "PASS" if 'ack' in resp_stop.lower() or 'err' not in resp_stop.lower() else "FAIL",
               f"response={resp_stop}")

    # Verify stream stopped
    time.sleep(1)
    ports[1].reset_input_buffer()
    time.sleep(1)
    leftover = ports[1].read(4096)
    leftover_text = leftover.decode('utf-8', errors='replace').strip()
    log_result("Stream stopped (no more data)",
               "PASS" if len(leftover_text) < 50 else "FAIL",
               f"remaining bytes: {len(leftover_text)}")


def test_11_help_command(ports):
    """Test help command."""
    print("\n--- Test 11: Help Command ---")

    resp = cmd(ports[1], 'help', delay=0.5)
    ok = '=== SERIAL COMMAND REFERENCE ===' in resp
    log_result("Help command",
               "PASS" if ok else "FAIL",
               f"starts with correct header: {ok}, length={len(resp)}")


def test_12_remote_query(ports):
    """Test remote query: from node 1, query node 2 and node 3 lux via hub."""
    print("\n--- Test 12: Remote CAN Query (hub forwarding) ---")

    for n in NODE_IDS:
        cmd(ports[n], f'r {n} 20', delay=0.1)
        cmd(ports[n], f'f {n} 1', delay=0.1)
    time.sleep(2)

    # From node 1 serial, query node 2 lux
    resp2 = cmd(ports[1], 'g y 2', delay=1.0)
    v2 = parse_float(resp2)
    if v2 is not None and v2 > 0:
        log_result("Remote query: g y 2 from node 1", "PASS", f"lux={v2:.2f}")
    elif 'timeout' in resp2.lower() or 'err' in resp2.lower():
        log_result("Remote query: g y 2 from node 1", "FAIL",
                   f"Hub forwarding issue: {resp2}")
    else:
        log_result("Remote query: g y 2 from node 1", "FAIL", f"raw={resp2}")

    # From node 1 serial, query node 3 lux
    resp3 = cmd(ports[1], 'g y 3', delay=1.0)
    v3 = parse_float(resp3)
    if v3 is not None and v3 > 0:
        log_result("Remote query: g y 3 from node 1", "PASS", f"lux={v3:.2f}")
    elif 'timeout' in resp3.lower() or 'err' in resp3.lower():
        log_result("Remote query: g y 3 from node 1", "FAIL",
                   f"Hub forwarding issue: {resp3}")
    else:
        log_result("Remote query: g y 3 from node 1", "FAIL", f"raw={resp3}")


def test_13_algorithm_switching(ports):
    """Test algorithm switching with A command."""
    print("\n--- Test 13: Algorithm Switching (A command) ---")

    for alg_id in [0, 1, 2, 3]:
        resp = cmd(ports[1], f'A {alg_id}', delay=0.5)
        ok = 'ack' in resp.lower() or 'err' not in resp.lower()
        log_result(f"A {alg_id}",
                   "PASS" if ok else "FAIL",
                   f"response={resp}")

    cmd(ports[1], 'A 0', delay=0.3)


def test_14_calibration_report(ports):
    """Test calibration report command."""
    print("\n--- Test 14: Calibration Report (rpt command) ---")

    resp = cmd(ports[1], 'rpt', delay=1.0)
    has_data = len(resp) > 10 and ('K' in resp or 'gain' in resp.lower() or
                                    'calib' in resp.lower() or '[' in resp)
    log_result("Calibration report (rpt)",
               "PASS" if has_data else "FAIL",
               f"response (first 300 chars): {resp[:300]}")


def test_15_invalid_commands(ports):
    """Test error handling for invalid commands."""
    print("\n--- Test 15: Error Handling ---")

    resp = cmd(ports[1], 'Z', delay=0.3)
    log_result("Invalid command 'Z'",
               "PASS" if 'err' in resp.lower() or 'invalid' in resp.lower() or 'Valor' in resp else "FAIL",
               f"response={resp}")

    resp = cmd(ports[1], 'g z 1', delay=0.3)
    log_result("Invalid get 'g z 1'",
               "PASS" if 'err' in resp.lower() or 'invalid' in resp.lower() or 'Valor' in resp else "FAIL",
               f"response={resp}")


def test_16_pwm_fix_comparison(ports):
    """Compare current lux accuracy to previous baseline (before PWM fix)."""
    print("\n--- Test 16: PWM Fix Comparison ---")

    cmd(ports[1], 'A 0', delay=0.3)
    for n in NODE_IDS:
        cmd(ports[n], f'f {n} 1', delay=0.1)
        cmd(ports[n], f'r {n} 20', delay=0.1)
    time.sleep(3)

    print(f"  Previous baseline: ~{PREV_LUX_BASELINE:.1f} lux +/- {PREV_LUX_ACCURACY}")

    for n in NODE_IDS:
        readings = []
        for _ in range(5):
            resp = cmd(ports[n], f'g y {n}', delay=0.2)
            v = parse_float(resp)
            if v is not None:
                readings.append(v)
            time.sleep(0.1)

        if readings:
            avg = sum(readings) / len(readings)
            spread = max(readings) - min(readings)
            error = abs(avg - PREV_LUX_BASELINE)
            log_result(f"Node {n} accuracy (ref=20)",
                       "PASS" if error < 5.0 else "FAIL",
                       f"avg={avg:.2f}, spread={spread:.2f}, error_from_20={error:.2f}")
        else:
            log_result(f"Node {n} accuracy (ref=20)", "FAIL", "No valid readings")


def test_17_restart(ports):
    """Test restart command R."""
    print("\n--- Test 17: Restart (R command) ---")
    print("  Sending R (restart). Waiting 20s for reboot + recalibration...")

    cmd(ports[1], 'R', delay=1.0)
    time.sleep(20)

    # After restart, the Pico may need reconfiguration (node ID + total)
    # Try reading lux first; if it fails, reconfigure
    resp = cmd(ports[1], 'g y 1', delay=0.5)
    lux = parse_float(resp)

    if lux is None or 'Valor' in (resp or '') or 'invalid' in (resp or '').lower():
        print("  Node likely rebooted to config mode. Reconfiguring...")
        # Reconfigure all nodes
        for nid in NODE_IDS:
            s = ports[nid]
            s.reset_input_buffer()
            s.write(f'{nid}\n'.encode())
            time.sleep(1)
            s.read(4096)
            s.write(f'{TOTAL_NODES}\n'.encode())
            time.sleep(1)
            s.read(4096)

        print("  Waiting 15s for calibration after restart...")
        time.sleep(15)

        resp = cmd(ports[1], 'g y 1', delay=0.5)
        lux = parse_float(resp)

    log_result("Post-restart lux",
               "PASS" if lux is not None and lux > 0 else "FAIL",
               f"lux={lux:.2f}" if lux else f"raw={resp}")


# ======================== Main =================================

def main():
    print("=" * 70)
    print(f"  COMPREHENSIVE DISTRIBUTED ILLUMINATION SYSTEM TEST")
    print(f"  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 70)

    # Connect and configure
    print("\n[Step 1] Connecting to all 3 Picos...")
    ports = connect_and_configure()
    if len(ports) < TOTAL_NODES:
        print(f"\nERROR: Only {len(ports)}/{TOTAL_NODES} Picos connected. Aborting.")
        close_all(ports)
        sys.exit(1)

    # Wait for calibration
    print(f"\n[Step 2] Waiting 15s for wakeup + CAN calibration...")
    time.sleep(15)

    # Verify all nodes are alive
    alive_count = 0
    for n in NODE_IDS:
        resp = cmd(ports[n], f'g y {n}', delay=0.5)
        v = parse_float(resp)
        if v is not None:
            alive_count += 1
            print(f"  Node {n}: alive (lux={v:.2f})")
        else:
            print(f"  Node {n}: NOT RESPONDING ({resp[:80]})")

    if alive_count < TOTAL_NODES:
        print(f"\nWARNING: Only {alive_count}/{TOTAL_NODES} nodes responding.")

    # Run all tests
    print("\n" + "=" * 70)
    print("  RUNNING TESTS")
    print("=" * 70)

    test_01_basic_control(ports)
    test_02_algorithms(ports)
    test_03_step_response(ports)
    test_04_occupancy(ports)
    test_05_unequal_costs(ports)
    test_06_metrics(ports)
    test_07_all_get_commands(ports)
    test_08_buffer(ports)
    test_09_set_commands(ports)
    test_10_streaming(ports)
    test_11_help_command(ports)
    test_12_remote_query(ports)
    test_13_algorithm_switching(ports)
    test_14_calibration_report(ports)
    test_15_invalid_commands(ports)
    test_16_pwm_fix_comparison(ports)
    test_17_restart(ports)

    # Cleanup
    print("\n\nCleaning up...")
    for n in NODE_IDS:
        cmd(ports[n], f'A 0', delay=0.2)
        cmd(ports[n], f'f {n} 1', delay=0.1)
        cmd(ports[n], f'r {n} 20', delay=0.1)
        cmd(ports[n], f'C {n} 1', delay=0.1)

    close_all(ports)

    # Summary
    total = passed + failed + skipped
    print("\n" + "=" * 70)
    print(f"  TEST SUMMARY")
    print("=" * 70)
    print(f"  Total:   {total}")
    print(f"  PASSED:  {passed}")
    print(f"  FAILED:  {failed}")
    print(f"  SKIPPED: {skipped}")
    if total > 0:
        print(f"  Rate:    {100*passed/total:.1f}%")
    print("=" * 70)

    if failed > 0:
        print("\n  FAILED TESTS:")
        for name, status, detail in results_log:
            if status == "FAIL":
                print(f"    - {name}: {detail}")

    print()
    return 1 if failed > 0 else 0


if __name__ == '__main__':
    sys.exit(main())
