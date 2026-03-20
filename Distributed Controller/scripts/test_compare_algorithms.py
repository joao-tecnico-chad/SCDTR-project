#!/usr/bin/env python3
"""
Compare all 3 distributed algorithms side-by-side.

For each algorithm (consensus, dual_decomp, admm), runs three scenarios:
  A) All occupied, equal costs (c=1.0)
  B) All occupied, different costs (c1=1.0, c2=2.0, c3=3.0)
  C) Mixed occupancy (node1=occupied, node2=unoccupied, node3=occupied)

Collects energy, visibility, and flicker metrics for each combination.
Prints a comparison table and saves to algorithm_comparison.csv.

Exit code 0 = PASS (all algorithms converge), 1 = FAIL.
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
CSV_FILE = os.path.join(SCRIPT_DIR, "algorithm_comparison.csv")

NODE_IDS = [1, 2, 3]
CONVERGENCE_TIMEOUT_S = 30
STREAM_DURATION_S = 10

ALGORITHMS = {
    1: "consensus",
    2: "dual_decomp",
    3: "admm",
}

SCENARIOS = {
    "A": {
        "description": "All occupied, equal costs (c=1.0)",
        "occupancy": {1: 2, 2: 2, 3: 2},
        "costs": {1: 1.0, 2: 1.0, 3: 1.0},
    },
    "B": {
        "description": "All occupied, different costs (c1=1, c2=2, c3=3)",
        "occupancy": {1: 2, 2: 2, 3: 2},
        "costs": {1: 1.0, 2: 2.0, 3: 3.0},
    },
    "C": {
        "description": "Mixed occupancy (n1=occ, n2=unocc, n3=occ)",
        "occupancy": {1: 2, 2: 1, 3: 2},
        "costs": {1: 1.0, 2: 1.0, 3: 1.0},
    },
}


def run_scenario(ser, alg_id, scenario_key, scenario):
    """
    Configure the system for one algorithm + scenario combination,
    wait for convergence, stream lux, and collect metrics.
    Returns (converged, node_metrics) where node_metrics is
    {node_id: {energy, visibility, flicker, mean_lux, ref}}.
    """
    alg_name = ALGORITHMS[alg_id]
    print(f"\n--- Algorithm: {alg_name} | Scenario {scenario_key}: {scenario['description']} ---")

    # Set algorithm
    set_algorithm(ser, alg_id)
    time.sleep(0.3)

    # Set occupancy
    for nid in NODE_IDS:
        set_occupancy(ser, nid, scenario["occupancy"][nid])
        time.sleep(0.1)

    # Set costs
    for nid in NODE_IDS:
        set_cost(ser, nid, scenario["costs"][nid])
        time.sleep(0.1)

    # Wait for convergence
    print(f"   Waiting for convergence (timeout {CONVERGENCE_TIMEOUT_S}s)...")
    converged = wait_converge(ser, timeout=CONVERGENCE_TIMEOUT_S)
    if converged:
        print("   Converged.")
    else:
        print("   WARNING: Convergence not confirmed; continuing with measurements.")
    time.sleep(1.0)

    # Collect metrics per node
    node_results = {}
    for nid in NODE_IDS:
        ref = query(ser, f"g r {nid}")
        metrics = get_metrics(ser, nid)

        # Stream lux
        print(f"   Node {nid}: streaming lux for {STREAM_DURATION_S}s...")
        data = stream(ser, "l", nid, STREAM_DURATION_S)

        mean_lux = None
        if data:
            values = [v for v, _ in data]
            mean_lux = sum(values) / len(values)

        node_results[nid] = {
            "ref": ref,
            "mean_lux": round(mean_lux, 3) if mean_lux is not None else None,
            "energy": metrics.get("energy"),
            "visibility": metrics.get("visibility"),
            "flicker": metrics.get("flicker"),
        }

        ref_str = f"{ref:.2f}" if ref is not None else "N/A"
        lux_str = f"{mean_lux:.2f}" if mean_lux is not None else "N/A"
        print(f"   Node {nid}: ref={ref_str}, mean_lux={lux_str}, "
              f"E={metrics.get('energy')}, V={metrics.get('visibility')}, "
              f"F={metrics.get('flicker')}")

        time.sleep(0.3)

    return converged, node_results


def main():
    parser = argparse.ArgumentParser(description="Compare all 3 distributed algorithms")
    parser.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    args = parser.parse_args()

    ser = connect(port=args.port)

    print("\n=== Algorithm Comparison Test ===\n")

    # Collect all results: (alg_name, scenario_key, node_id, converged, ref, mean_lux, energy, visibility, flicker)
    all_rows = []
    all_converged = True

    for alg_id, alg_name in ALGORITHMS.items():
        for scenario_key, scenario in SCENARIOS.items():
            converged, node_results = run_scenario(ser, alg_id, scenario_key, scenario)

            if not converged:
                all_converged = False

            for nid in NODE_IDS:
                nr = node_results[nid]
                all_rows.append({
                    "algorithm": alg_name,
                    "scenario": scenario_key,
                    "node_id": nid,
                    "converged": converged,
                    "reference_lux": nr["ref"],
                    "mean_lux": nr["mean_lux"],
                    "energy": nr["energy"],
                    "visibility": nr["visibility"],
                    "flicker": nr["flicker"],
                })

    # Print comparison table
    print("\n\n=== Comparison Table ===\n")
    header = f"{'Algorithm':<14} {'Scen':<5} {'Node':<5} {'Conv':<5} {'Ref':>8} {'MeanLux':>8} {'Energy':>8} {'Visib':>8} {'Flicker':>8}"
    print(header)
    print("-" * len(header))
    for row in all_rows:
        ref_s = f"{row['reference_lux']:.2f}" if row['reference_lux'] is not None else "N/A"
        lux_s = f"{row['mean_lux']:.2f}" if row['mean_lux'] is not None else "N/A"
        e_s = f"{row['energy']:.3f}" if row['energy'] is not None else "N/A"
        v_s = f"{row['visibility']:.3f}" if row['visibility'] is not None else "N/A"
        f_s = f"{row['flicker']:.3f}" if row['flicker'] is not None else "N/A"
        conv_s = "Y" if row['converged'] else "N"
        print(f"{row['algorithm']:<14} {row['scenario']:<5} {row['node_id']:<5} {conv_s:<5} "
              f"{ref_s:>8} {lux_s:>8} {e_s:>8} {v_s:>8} {f_s:>8}")

    # Save to CSV
    print(f"\nSaving results to {CSV_FILE}...")
    fieldnames = ["algorithm", "scenario", "node_id", "converged",
                   "reference_lux", "mean_lux", "energy", "visibility", "flicker"]
    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(all_rows)
    print(f"  Saved {CSV_FILE} ({len(all_rows)} rows)")

    # Summary
    print()
    if all_converged:
        print("RESULT: PASS - All algorithms converged in all scenarios.\n")
        ser.close()
        return 0
    else:
        failed = set()
        for row in all_rows:
            if not row["converged"]:
                failed.add(f"{row['algorithm']}/scenario_{row['scenario']}")
        print(f"RESULT: FAIL - Some algorithm/scenario combinations did not converge: {sorted(failed)}\n")
        ser.close()
        return 1


if __name__ == "__main__":
    sys.exit(main())
