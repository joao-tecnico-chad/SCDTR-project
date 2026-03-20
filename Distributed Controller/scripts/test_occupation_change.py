#!/usr/bin/env python3
"""
Test occupation state transitions under distributed control.

Starts all desks unoccupied, waits for convergence, records baseline,
then changes node 1 to occupied, waits for convergence, records metrics,
then changes node 1 back to unoccupied. Streams lux during transitions
and checks that settling time is within the allowed limit.

Exit code 0 = PASS, 1 = FAIL.
"""

import argparse
import csv
import os
import sys
import time

from serial_utils import (
    connect,
    get_metrics,
    query,
    send_ack,
    set_algorithm,
    set_cost,
    set_occupancy,
    wait_converge,
)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = os.path.join(SCRIPT_DIR, "occupation_change.csv")

NODE_IDS = [1, 2, 3]
LUX_TOLERANCE = 2.0
CONVERGENCE_TIMEOUT_S = 30
MAX_SETTLING_S = 3.0
TRANSITION_STREAM_S = 8


def stream_all_nodes(ser, node_ids, duration_s):
    """
    Stream lux from all nodes simultaneously.
    Returns dict: node_id -> [(lux, time_ms, wall_elapsed_s), ...]
    """
    data = {nid: [] for nid in node_ids}
    ser.reset_input_buffer()

    for nid in node_ids:
        send_ack(ser, f"s l {nid}")
        time.sleep(0.05)

    start = time.time()
    while time.time() - start < duration_s:
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) >= 5 and parts[0] == 's' and parts[1] == 'l':
            try:
                nid = int(parts[2])
                value = float(parts[3])
                t_ms = int(parts[4])
                elapsed = time.time() - start
                if nid in data:
                    data[nid].append((value, t_ms, round(elapsed, 3)))
            except (ValueError, IndexError):
                pass

    for nid in node_ids:
        send_ack(ser, f"S l {nid}")
    time.sleep(0.1)
    ser.reset_input_buffer()

    return data


def measure_settling_time(node_data, target_ref, tolerance, start_wall=0.0):
    """
    Find how long it takes for values to settle within tolerance of
    target_ref after start_wall. Returns settling time in seconds,
    or None if it never settles.
    """
    post = [(v, w) for v, _, w in node_data if w >= start_wall]
    if not post:
        return None

    # Find the first sample within tolerance such that all subsequent
    # samples also stay within tolerance.
    for i, (v, w) in enumerate(post):
        if abs(v - target_ref) <= tolerance:
            remaining = post[i:]
            if all(abs(rv - target_ref) <= tolerance for rv, _ in remaining):
                return w - start_wall

    return None


def main():
    parser = argparse.ArgumentParser(description="Test occupation state transitions")
    parser.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    args = parser.parse_args()

    ser = connect(port=args.port)

    print("\n=== Occupation Change Test ===\n")

    # Step 1: Set algorithm to consensus
    print("1. Setting algorithm to consensus (A 1)...")
    if not set_algorithm(ser, 1):
        print("   WARNING: Algorithm set command not acknowledged.")
    time.sleep(0.3)

    # Step 2: Start all desks unoccupied
    print("2. Setting all desks to unoccupied (state 1)...")
    for nid in NODE_IDS:
        set_occupancy(ser, nid, 1)
        time.sleep(0.1)
    for nid in NODE_IDS:
        set_cost(ser, nid, 1.0)
        time.sleep(0.1)

    # Step 3: Wait for convergence
    print(f"3. Waiting for convergence (timeout {CONVERGENCE_TIMEOUT_S}s)...")
    converged = wait_converge(ser, timeout=CONVERGENCE_TIMEOUT_S)
    if converged:
        print("   Converged.")
    else:
        print("   WARNING: Convergence not confirmed; continuing.")
    time.sleep(1.0)

    # Step 4: Record baseline references and metrics
    print("4. Recording baseline (unoccupied)...")
    baseline_refs = {}
    baseline_metrics = {}
    for nid in NODE_IDS:
        ref = query(ser, f"g r {nid}")
        baseline_refs[nid] = ref
        baseline_metrics[nid] = get_metrics(ser, nid)
        ref_str = f"{ref:.2f}" if ref is not None else "N/A"
        print(f"   Node {nid}: ref={ref_str} lux, metrics={baseline_metrics[nid]}")
    time.sleep(0.5)

    # Step 5: Transition -- change node 1 to occupied, stream during transition
    print(f"\n5. Changing node 1 to occupied and streaming ({TRANSITION_STREAM_S}s)...")
    # Start streaming before the change so we capture the transition
    all_stream_data = []

    # Phase A: node 1 -> occupied
    ser.reset_input_buffer()
    for nid in NODE_IDS:
        send_ack(ser, f"s l {nid}")
        time.sleep(0.05)

    # Give a short baseline window, then trigger the change
    phase_a_data = {nid: [] for nid in NODE_IDS}
    start = time.time()
    change_sent = False
    change_wall = None

    while time.time() - start < TRANSITION_STREAM_S:
        elapsed = time.time() - start

        # After 1s of baseline, send the occupancy change
        if not change_sent and elapsed >= 1.0:
            set_occupancy(ser, 1, 2)
            change_wall = elapsed
            change_sent = True
            print(f"   Sent 'o 1 2' at t={elapsed:.1f}s")

        line = ser.readline().decode('utf-8', errors='replace').strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) >= 5 and parts[0] == 's' and parts[1] == 'l':
            try:
                nid = int(parts[2])
                value = float(parts[3])
                t_ms = int(parts[4])
                if nid in phase_a_data:
                    phase_a_data[nid].append((value, t_ms, round(elapsed, 3)))
            except (ValueError, IndexError):
                pass

    for nid in NODE_IDS:
        send_ack(ser, f"S l {nid}")
    time.sleep(0.1)
    ser.reset_input_buffer()

    # Wait for convergence after occupation change
    print("   Waiting for convergence after occupation change...")
    wait_converge(ser, timeout=CONVERGENCE_TIMEOUT_S)
    time.sleep(0.5)

    # Read occupied references
    print("   Reading occupied references...")
    occupied_refs = {}
    occupied_metrics = {}
    for nid in NODE_IDS:
        ref = query(ser, f"g r {nid}")
        occupied_refs[nid] = ref
        occupied_metrics[nid] = get_metrics(ser, nid)
        ref_str = f"{ref:.2f}" if ref is not None else "N/A"
        print(f"   Node {nid}: ref={ref_str} lux, metrics={occupied_metrics[nid]}")

    # Step 6: Transition back -- change node 1 to unoccupied
    print(f"\n6. Changing node 1 back to unoccupied and streaming ({TRANSITION_STREAM_S}s)...")
    phase_b_data = {nid: [] for nid in NODE_IDS}

    ser.reset_input_buffer()
    for nid in NODE_IDS:
        send_ack(ser, f"s l {nid}")
        time.sleep(0.05)

    start = time.time()
    change_sent = False
    change_wall_b = None

    while time.time() - start < TRANSITION_STREAM_S:
        elapsed = time.time() - start

        if not change_sent and elapsed >= 1.0:
            set_occupancy(ser, 1, 1)
            change_wall_b = elapsed
            change_sent = True
            print(f"   Sent 'o 1 1' at t={elapsed:.1f}s")

        line = ser.readline().decode('utf-8', errors='replace').strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) >= 5 and parts[0] == 's' and parts[1] == 'l':
            try:
                nid = int(parts[2])
                value = float(parts[3])
                t_ms = int(parts[4])
                if nid in phase_b_data:
                    phase_b_data[nid].append((value, t_ms, round(elapsed, 3)))
            except (ValueError, IndexError):
                pass

    for nid in NODE_IDS:
        send_ack(ser, f"S l {nid}")
    time.sleep(0.1)
    ser.reset_input_buffer()

    # Step 7: Analyze settling times
    print("\n7. Analyzing settling times...")
    all_ok = True
    results = []

    # Phase A: unoccupied -> occupied (target is occupied ref)
    print("   Phase A: unoccupied -> occupied")
    for nid in NODE_IDS:
        target = occupied_refs.get(nid)
        nd = phase_a_data.get(nid, [])
        if target is None or not nd:
            print(f"   Node {nid}: no data or reference, FAIL")
            results.append(("A_unocc_to_occ", nid, target, None, False))
            all_ok = False
            continue

        settling = measure_settling_time(nd, target, LUX_TOLERANCE, start_wall=change_wall or 1.0)
        ok = settling is not None and settling <= MAX_SETTLING_S
        status = "PASS" if ok else "FAIL"
        st_str = f"{settling:.2f}s" if settling is not None else "N/A"
        print(f"   Node {nid}: target={target:.2f}, settling={st_str} [{status}]")
        results.append(("A_unocc_to_occ", nid, target, settling, ok))
        if not ok:
            all_ok = False

    # Phase B: occupied -> unoccupied (target is baseline ref)
    print("   Phase B: occupied -> unoccupied")
    for nid in NODE_IDS:
        target = baseline_refs.get(nid)
        nd = phase_b_data.get(nid, [])
        if target is None or not nd:
            print(f"   Node {nid}: no data or reference, FAIL")
            results.append(("B_occ_to_unocc", nid, target, None, False))
            all_ok = False
            continue

        settling = measure_settling_time(nd, target, LUX_TOLERANCE, start_wall=change_wall_b or 1.0)
        ok = settling is not None and settling <= MAX_SETTLING_S
        status = "PASS" if ok else "FAIL"
        st_str = f"{settling:.2f}s" if settling is not None else "N/A"
        print(f"   Node {nid}: target={target:.2f}, settling={st_str} [{status}]")
        results.append(("B_occ_to_unocc", nid, target, settling, ok))
        if not ok:
            all_ok = False

    # Step 8: Save results
    print(f"\n8. Saving results to {CSV_FILE}...")
    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        # Summary
        writer.writerow(["phase", "node_id", "target_lux", "settling_time_s", "pass"])
        for row in results:
            writer.writerow(row)
        writer.writerow([])
        # Baseline and occupied metrics
        writer.writerow(["node_id", "baseline_ref", "occupied_ref",
                         "baseline_energy", "occupied_energy"])
        for nid in NODE_IDS:
            writer.writerow([
                nid,
                baseline_refs.get(nid),
                occupied_refs.get(nid),
                baseline_metrics.get(nid, {}).get("energy"),
                occupied_metrics.get(nid, {}).get("energy"),
            ])
        writer.writerow([])
        # Phase A stream data
        writer.writerow(["phase", "node_id", "time_ms", "lux", "wall_elapsed_s"])
        for nid in NODE_IDS:
            for value, t_ms, wall in phase_a_data.get(nid, []):
                writer.writerow(["A", nid, t_ms, value, wall])
        for nid in NODE_IDS:
            for value, t_ms, wall in phase_b_data.get(nid, []):
                writer.writerow(["B", nid, t_ms, value, wall])
    print(f"   Saved {CSV_FILE}")

    # Summary
    print()
    if all_ok:
        print(f"RESULT: PASS - All transitions settled within {MAX_SETTLING_S}s.\n")
        ser.close()
        return 0
    else:
        failed = [(r[0], r[1]) for r in results if not r[4]]
        print(f"RESULT: FAIL - Failed transitions: {failed}\n")
        ser.close()
        return 1


if __name__ == "__main__":
    sys.exit(main())
