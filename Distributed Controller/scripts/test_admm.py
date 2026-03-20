#!/usr/bin/env python3
"""
Test the ADMM distributed algorithm.

Sets all desks to occupied with equal costs, enables ADMM mode,
waits for convergence, then streams lux from each node and checks
whether references are met within tolerance.

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
CSV_FILE = os.path.join(SCRIPT_DIR, "admm_results.csv")

NODE_IDS = [1, 2, 3]
LUX_TOLERANCE = 2.0
CONVERGENCE_TIMEOUT_S = 30
STREAM_DURATION_S = 10


def main():
    parser = argparse.ArgumentParser(description="Test ADMM algorithm")
    parser.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    args = parser.parse_args()

    ser = connect(port=args.port)

    print("\n=== ADMM Algorithm Test ===\n")

    # Step 1: Set algorithm to ADMM
    print("1. Setting algorithm to ADMM (A 3)...")
    if not set_algorithm(ser, 3):
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
    print(f"3. Waiting for ADMM convergence (timeout {CONVERGENCE_TIMEOUT_S}s)...")
    converged = wait_converge(ser, timeout=CONVERGENCE_TIMEOUT_S)
    if converged:
        print("   ADMM converged.")
    else:
        print("   WARNING: Convergence not confirmed; continuing with measurements.")

    time.sleep(1.0)

    # Step 4: Read references and stream lux for each node
    print(f"4. Reading references and streaming lux ({STREAM_DURATION_S}s per node)...\n")

    all_results = []
    all_stream_data = {}
    all_ok = True

    for nid in NODE_IDS:
        ref = query(ser, f"g r {nid}")
        if ref is None:
            print(f"   Node {nid}: could not read reference, skipping.")
            all_results.append((nid, None, None, None, False))
            all_ok = False
            continue

        print(f"   Node {nid}: reference = {ref:.2f} lux")
        print(f"   Streaming lux for {STREAM_DURATION_S}s...")
        data = stream(ser, "l", nid, STREAM_DURATION_S)

        if not data:
            print(f"   Node {nid}: no stream data received.")
            all_results.append((nid, ref, None, None, False))
            all_ok = False
            continue

        values = [v for v, _ in data]
        mean_lux = sum(values) / len(values)
        errors = [abs(v - ref) for v in values]
        max_err = max(errors)
        meets_ref = max_err <= LUX_TOLERANCE

        status = "PASS" if meets_ref else "FAIL"
        print(f"   Node {nid}: mean={mean_lux:.2f} lux, max_error={max_err:.2f} lux [{status}]")

        all_results.append((nid, round(ref, 3), round(mean_lux, 3), round(max_err, 3), meets_ref))
        all_stream_data[nid] = data

        if not meets_ref:
            all_ok = False

        time.sleep(0.3)

    # Step 5: Save results
    print(f"\n5. Saving results to {CSV_FILE}...")
    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["node_id", "reference_lux", "mean_lux", "max_error_lux", "pass"])
        for row in all_results:
            writer.writerow(row)
        writer.writerow([])
        writer.writerow(["node_id", "time_ms", "lux"])
        for nid, data in all_stream_data.items():
            for value, t_ms in data:
                writer.writerow([nid, t_ms, value])
    print(f"   Saved {CSV_FILE}")

    # Summary
    print()
    if all_ok:
        print(f"RESULT: PASS - All nodes meet reference within {LUX_TOLERANCE} lux tolerance.\n")
        ser.close()
        return 0
    else:
        failed = [r[0] for r in all_results if not r[4]]
        print(f"RESULT: FAIL - Nodes not meeting reference: {failed}\n")
        ser.close()
        return 1


if __name__ == "__main__":
    sys.exit(main())
