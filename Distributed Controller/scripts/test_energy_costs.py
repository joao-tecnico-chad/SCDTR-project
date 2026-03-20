#!/usr/bin/env python3
"""
Test identical vs different energy costs under distributed control.

Uses the consensus algorithm with all desks occupied.
  Test 1: Equal costs (c=1.0 for all) -- record duty cycles.
  Test 2: Different costs (c1=1, c2=2, c3=3) -- record duty cycles.

Verifies that with different costs, cheaper nodes (lower cost) have
higher duty cycles than more expensive nodes.

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
    set_algorithm,
    set_cost,
    set_occupancy,
    stream,
    wait_converge,
)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = os.path.join(SCRIPT_DIR, "energy_costs.csv")

NODE_IDS = [1, 2, 3]
CONVERGENCE_TIMEOUT_S = 30
STREAM_DURATION_S = 10

EQUAL_COSTS = {1: 1.0, 2: 1.0, 3: 1.0}
DIFFERENT_COSTS = {1: 1.0, 2: 2.0, 3: 3.0}


def run_cost_test(ser, test_name, costs):
    """
    Set costs, wait for convergence, stream duty from each node.
    Returns dict: node_id -> {cost, mean_duty, ref, energy, visibility, flicker}.
    """
    print(f"\n--- {test_name} ---")

    # Set costs
    for nid in NODE_IDS:
        set_cost(ser, nid, costs[nid])
        time.sleep(0.1)
    print(f"   Costs set: {costs}")

    # Wait for re-convergence
    print(f"   Waiting for convergence (timeout {CONVERGENCE_TIMEOUT_S}s)...")
    converged = wait_converge(ser, timeout=CONVERGENCE_TIMEOUT_S)
    if converged:
        print("   Converged.")
    else:
        print("   WARNING: Convergence not confirmed; continuing with measurements.")
    time.sleep(1.0)

    # Collect per-node data
    results = {}
    for nid in NODE_IDS:
        ref = query(ser, f"g r {nid}")
        duty = query(ser, f"g d {nid}")
        metrics = get_metrics(ser, nid)

        # Stream duty cycle for a more accurate mean
        print(f"   Node {nid}: streaming duty for {STREAM_DURATION_S}s...")
        data = stream(ser, "d", nid, STREAM_DURATION_S)

        mean_duty = None
        if data:
            values = [v for v, _ in data]
            mean_duty = sum(values) / len(values)

        results[nid] = {
            "cost": costs[nid],
            "instant_duty": duty,
            "mean_duty": round(mean_duty, 4) if mean_duty is not None else None,
            "ref": ref,
            "energy": metrics.get("energy"),
            "visibility": metrics.get("visibility"),
            "flicker": metrics.get("flicker"),
        }

        duty_str = f"{mean_duty:.4f}" if mean_duty is not None else "N/A"
        ref_str = f"{ref:.2f}" if ref is not None else "N/A"
        print(f"   Node {nid}: cost={costs[nid]}, mean_duty={duty_str}, ref={ref_str}")

        time.sleep(0.3)

    return converged, results


def main():
    parser = argparse.ArgumentParser(description="Test energy cost effects on duty cycles")
    parser.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    args = parser.parse_args()

    ser = connect(port=args.port)

    print("\n=== Energy Costs Test ===\n")

    # Step 1: Set algorithm to consensus
    print("1. Setting algorithm to consensus (A 1)...")
    if not set_algorithm(ser, 1):
        print("   WARNING: Algorithm set command not acknowledged.")
    time.sleep(0.3)

    # Step 2: Set all desks occupied
    print("2. Setting all desks to occupied...")
    for nid in NODE_IDS:
        set_occupancy(ser, nid, 2)
        time.sleep(0.1)

    # Step 3: Test 1 -- equal costs
    print("\n3. Test 1: Equal costs...")
    conv1, results_equal = run_cost_test(ser, "Equal Costs (c=1.0 for all)", EQUAL_COSTS)

    # Step 4: Test 2 -- different costs
    print("\n4. Test 2: Different costs...")
    conv2, results_diff = run_cost_test(ser, "Different Costs (c1=1, c2=2, c3=3)", DIFFERENT_COSTS)

    # Step 5: Verify cost ordering
    print("\n5. Verifying cost ordering...\n")

    # With different costs, lower cost nodes should have higher duty
    diff_duties = []
    for nid in NODE_IDS:
        md = results_diff[nid]["mean_duty"]
        cost = results_diff[nid]["cost"]
        if md is not None:
            diff_duties.append((nid, cost, md))

    # Sort by cost ascending
    diff_duties.sort(key=lambda x: x[1])

    print("   Different-cost duty cycles (sorted by cost):")
    for nid, cost, duty in diff_duties:
        print(f"   Node {nid}: cost={cost}, mean_duty={duty:.4f}")

    cost_ordering_ok = True
    if len(diff_duties) >= 2:
        # Check that lower cost implies higher or equal duty
        for i in range(len(diff_duties) - 1):
            nid_a, cost_a, duty_a = diff_duties[i]
            nid_b, cost_b, duty_b = diff_duties[i + 1]
            if cost_a < cost_b and duty_a < duty_b:
                print(f"   WARNING: Node {nid_a} (cost={cost_a}) has lower duty ({duty_a:.4f}) "
                      f"than node {nid_b} (cost={cost_b}, duty={duty_b:.4f})")
                cost_ordering_ok = False
    else:
        print("   WARNING: Not enough duty data to verify ordering.")
        cost_ordering_ok = False

    if cost_ordering_ok:
        print("   Cost ordering respected: cheaper nodes have higher (or equal) duty.")
    else:
        print("   Cost ordering NOT respected.")

    # Print comparison table
    print("\n\n=== Comparison Table ===\n")
    header = f"{'Test':<18} {'Node':<5} {'Cost':>6} {'MeanDuty':>10} {'Ref':>8} {'Energy':>8}"
    print(header)
    print("-" * len(header))

    for nid in NODE_IDS:
        r = results_equal[nid]
        d_str = f"{r['mean_duty']:.4f}" if r['mean_duty'] is not None else "N/A"
        ref_str = f"{r['ref']:.2f}" if r['ref'] is not None else "N/A"
        e_str = f"{r['energy']:.3f}" if r['energy'] is not None else "N/A"
        print(f"{'Equal costs':<18} {nid:<5} {r['cost']:>6.1f} {d_str:>10} {ref_str:>8} {e_str:>8}")

    for nid in NODE_IDS:
        r = results_diff[nid]
        d_str = f"{r['mean_duty']:.4f}" if r['mean_duty'] is not None else "N/A"
        ref_str = f"{r['ref']:.2f}" if r['ref'] is not None else "N/A"
        e_str = f"{r['energy']:.3f}" if r['energy'] is not None else "N/A"
        print(f"{'Different costs':<18} {nid:<5} {r['cost']:>6.1f} {d_str:>10} {ref_str:>8} {e_str:>8}")

    # Step 6: Save to CSV
    print(f"\n6. Saving results to {CSV_FILE}...")
    fieldnames = ["test", "node_id", "cost", "mean_duty", "instant_duty",
                   "reference_lux", "energy", "visibility", "flicker"]
    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for nid in NODE_IDS:
            r = results_equal[nid]
            writer.writerow({
                "test": "equal_costs",
                "node_id": nid,
                "cost": r["cost"],
                "mean_duty": r["mean_duty"],
                "instant_duty": r["instant_duty"],
                "reference_lux": r["ref"],
                "energy": r["energy"],
                "visibility": r["visibility"],
                "flicker": r["flicker"],
            })
        for nid in NODE_IDS:
            r = results_diff[nid]
            writer.writerow({
                "test": "different_costs",
                "node_id": nid,
                "cost": r["cost"],
                "mean_duty": r["mean_duty"],
                "instant_duty": r["instant_duty"],
                "reference_lux": r["ref"],
                "energy": r["energy"],
                "visibility": r["visibility"],
                "flicker": r["flicker"],
            })
    print(f"  Saved {CSV_FILE}")

    # Summary
    print()
    all_ok = conv1 and conv2 and cost_ordering_ok
    if all_ok:
        print("RESULT: PASS - Cost ordering respected; cheaper nodes use higher duty.\n")
        ser.close()
        return 0
    else:
        reasons = []
        if not conv1:
            reasons.append("equal-cost test did not converge")
        if not conv2:
            reasons.append("different-cost test did not converge")
        if not cost_ordering_ok:
            reasons.append("cost ordering not respected")
        print(f"RESULT: FAIL - {'; '.join(reasons)}.\n")
        ser.close()
        return 1


if __name__ == "__main__":
    sys.exit(main())
