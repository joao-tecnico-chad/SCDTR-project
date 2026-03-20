#!/usr/bin/env python3
"""
Test distributed disturbance rejection.

Sets consensus mode with all desks occupied, waits for convergence,
then streams lux from all nodes for 30s. At the 10s mark the user is
prompted to open the box window (external light disturbance); at the
15s mark the user is prompted to close it. The test measures recovery
time and steady-state error after the disturbance.

Exit code 0 = PASS, 1 = FAIL.
"""

import argparse
import csv
import os
import sys
import time

from serial_utils import (
    connect,
    query,
    send_ack,
    set_algorithm,
    set_cost,
    set_occupancy,
    stream,
    wait_converge,
)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = os.path.join(SCRIPT_DIR, "disturbance_distributed.csv")

NODE_IDS = [1, 2, 3]
LUX_TOLERANCE = 2.0
CONVERGENCE_TIMEOUT_S = 30
TOTAL_DURATION_S = 30
DISTURBANCE_OPEN_S = 10
DISTURBANCE_CLOSE_S = 15
RECOVERY_DEADLINE_S = 5.0


def stream_all_timed(ser, node_ids, duration_s, disturbance_open_s, disturbance_close_s):
    """
    Stream lux from all nodes simultaneously for the given duration.
    At disturbance_open_s, prompt the user to open the window.
    At disturbance_close_s, prompt the user to close the window.
    Returns dict: node_id -> [(lux, time_ms, wall_elapsed_s), ...]
    """
    data = {nid: [] for nid in node_ids}
    ser.reset_input_buffer()

    # Start streaming lux on all nodes
    for nid in node_ids:
        send_ack(ser, f"s l {nid}")
        time.sleep(0.05)

    start = time.time()
    prompted_open = False
    prompted_close = False

    while True:
        elapsed = time.time() - start
        if elapsed >= duration_s:
            break

        # User prompts at the appropriate times
        if not prompted_open and elapsed >= disturbance_open_s:
            print("\n>>> OPEN THE BOX WINDOW NOW <<<\n")
            prompted_open = True

        if not prompted_close and elapsed >= disturbance_close_s:
            print("\n>>> CLOSE THE WINDOW NOW <<<\n")
            prompted_close = True

        line = ser.readline().decode('utf-8', errors='replace').strip()
        if not line:
            continue

        parts = line.split()
        if len(parts) >= 5 and parts[0] == 's' and parts[1] == 'l':
            try:
                nid = int(parts[2])
                value = float(parts[3])
                t_ms = int(parts[4])
                if nid in data:
                    data[nid].append((value, t_ms, round(elapsed, 3)))
            except (ValueError, IndexError):
                pass

    # Stop streaming
    for nid in node_ids:
        send_ack(ser, f"S l {nid}")
    time.sleep(0.1)
    ser.reset_input_buffer()

    return data


def measure_recovery(node_data, ref, disturbance_close_wall_s, tolerance, deadline_s):
    """
    Measure time to recover to within tolerance of ref after the
    disturbance window closes. Returns (recovery_time_s, steady_error).
    recovery_time_s is None if the node never recovers within deadline.
    """
    # Find samples after the close prompt
    post_close = [(v, w) for v, _, w in node_data if w >= disturbance_close_wall_s]
    if not post_close:
        return None, None

    # Find first time within tolerance and staying within tolerance
    recovery_time = None
    for i, (v, w) in enumerate(post_close):
        if abs(v - ref) <= tolerance:
            # Check remaining samples stay within tolerance
            remaining = post_close[i:]
            if all(abs(rv - ref) <= tolerance for rv, _ in remaining):
                recovery_time = w - disturbance_close_wall_s
                break

    # Steady-state error: average error over last 3 seconds
    last_3s_start = post_close[-1][1] - 3.0
    tail = [abs(v - ref) for v, w in post_close if w >= last_3s_start]
    steady_err = sum(tail) / len(tail) if tail else None

    return recovery_time, steady_err


def main():
    parser = argparse.ArgumentParser(description="Test distributed disturbance rejection")
    parser.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    args = parser.parse_args()

    ser = connect(port=args.port)

    print("\n=== Distributed Disturbance Rejection Test ===\n")

    # Step 1: Set algorithm to consensus
    print("1. Setting algorithm to consensus (A 1)...")
    if not set_algorithm(ser, 1):
        print("   WARNING: Algorithm set command not acknowledged.")
    time.sleep(0.3)

    # Step 2: Set all desks occupied with equal costs
    print("2. Setting all desks occupied with equal costs...")
    for nid in NODE_IDS:
        set_occupancy(ser, nid, 2)
        time.sleep(0.1)
    for nid in NODE_IDS:
        set_cost(ser, nid, 1.0)
        time.sleep(0.1)

    # Step 3: Wait for convergence
    print(f"3. Waiting for consensus convergence (timeout {CONVERGENCE_TIMEOUT_S}s)...")
    converged = wait_converge(ser, timeout=CONVERGENCE_TIMEOUT_S)
    if converged:
        print("   Consensus converged.")
    else:
        print("   WARNING: Convergence not confirmed; continuing anyway.")

    time.sleep(1.0)

    # Step 4: Read references
    print("4. Reading references...")
    refs = {}
    for nid in NODE_IDS:
        ref = query(ser, f"g r {nid}")
        refs[nid] = ref
        if ref is not None:
            print(f"   Node {nid}: reference = {ref:.2f} lux")
        else:
            print(f"   Node {nid}: could not read reference")

    # Step 5: Stream with disturbance
    print(f"\n5. Streaming lux from all nodes for {TOTAL_DURATION_S}s...")
    print(f"   Disturbance prompt at {DISTURBANCE_OPEN_S}s, close at {DISTURBANCE_CLOSE_S}s.")
    print("   Wait for prompts and act accordingly.\n")

    data = stream_all_timed(
        ser, NODE_IDS, TOTAL_DURATION_S, DISTURBANCE_OPEN_S, DISTURBANCE_CLOSE_S
    )

    # Step 6: Analyze recovery
    print("\n6. Analyzing recovery...")
    all_ok = True
    results = []

    for nid in NODE_IDS:
        ref = refs.get(nid)
        node_data = data.get(nid, [])
        if ref is None or not node_data:
            print(f"   Node {nid}: no data or no reference, FAIL")
            results.append((nid, ref, None, None, False))
            all_ok = False
            continue

        recovery_time, steady_err = measure_recovery(
            node_data, ref, DISTURBANCE_CLOSE_S, LUX_TOLERANCE, RECOVERY_DEADLINE_S
        )

        recovered = recovery_time is not None and recovery_time <= RECOVERY_DEADLINE_S
        status = "PASS" if recovered else "FAIL"

        rt_str = f"{recovery_time:.2f}s" if recovery_time is not None else "N/A"
        se_str = f"{steady_err:.2f}" if steady_err is not None else "N/A"
        print(f"   Node {nid}: recovery_time={rt_str}, steady_error={se_str} lux [{status}]")

        results.append((nid, ref, recovery_time, steady_err, recovered))
        if not recovered:
            all_ok = False

    # Step 7: Save results
    print(f"\n7. Saving results to {CSV_FILE}...")
    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        # Summary
        writer.writerow(["node_id", "reference_lux", "recovery_time_s", "steady_error_lux", "pass"])
        for row in results:
            writer.writerow(row)
        writer.writerow([])
        # Detailed stream data
        writer.writerow(["node_id", "time_ms", "lux", "wall_elapsed_s"])
        for nid in NODE_IDS:
            for value, t_ms, wall in data.get(nid, []):
                writer.writerow([nid, t_ms, value, wall])
    print(f"   Saved {CSV_FILE}")

    # Summary
    print()
    if all_ok:
        print(f"RESULT: PASS - All nodes recovered within {RECOVERY_DEADLINE_S}s.\n")
        ser.close()
        return 0
    else:
        failed = [r[0] for r in results if not r[4]]
        print(f"RESULT: FAIL - Nodes that did not recover in time: {failed}\n")
        ser.close()
        return 1


if __name__ == "__main__":
    sys.exit(main())
