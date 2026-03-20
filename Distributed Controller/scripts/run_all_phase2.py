#!/usr/bin/env python3
"""
Master test runner for SCDTR Phase 2.

Runs all test scripts in sequence, prints a summary table with
pass/fail status and duration, and exits with code 0 if all pass.

Usage:
  python run_all_phase2.py [--port /dev/ttyACM0]
"""

import argparse
import os
import subprocess
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Test scripts in execution order
TEST_SCRIPTS = [
    "test_boot.py",
    "test_canbus.py",
    "test_calibration.py",
    "test_consensus.py",
    "test_dual_decomp.py",
    "test_admm.py",
    "test_compare_algorithms.py",
    "test_energy_costs.py",
    "test_occupation_change.py",
    "test_disturbance_distributed.py",
]


def run_test(script_name, port=None):
    """
    Run a single test script via subprocess.
    Returns (passed, duration_s, returncode).
    """
    script_path = os.path.join(SCRIPT_DIR, script_name)

    if not os.path.isfile(script_path):
        print(f"  WARNING: Script not found: {script_path}")
        return False, 0.0, -1

    cmd = [sys.executable, script_path]
    if port:
        cmd.extend(["--port", port])

    print(f"\n{'=' * 60}")
    print(f"Running: {script_name}")
    print(f"{'=' * 60}")

    start = time.time()
    try:
        result = subprocess.run(
            cmd,
            cwd=SCRIPT_DIR,
            timeout=300,  # 5-minute timeout per test
        )
        duration = time.time() - start
        passed = result.returncode == 0
        return passed, duration, result.returncode
    except subprocess.TimeoutExpired:
        duration = time.time() - start
        print(f"  ERROR: {script_name} timed out after 300s")
        return False, duration, -2
    except Exception as e:
        duration = time.time() - start
        print(f"  ERROR: {script_name} raised exception: {e}")
        return False, duration, -3


def main():
    parser = argparse.ArgumentParser(description="SCDTR Phase 2 master test runner")
    parser.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    args = parser.parse_args()

    print("\n" + "=" * 60)
    print("  SCDTR Phase 2 - Full Test Suite")
    print("=" * 60)
    if args.port:
        print(f"  Serial port: {args.port}")
    else:
        print("  Serial port: auto-detect")
    print(f"  Tests to run: {len(TEST_SCRIPTS)}")
    print()

    results = []
    total_start = time.time()

    for script_name in TEST_SCRIPTS:
        passed, duration, returncode = run_test(script_name, port=args.port)
        results.append({
            "test": script_name,
            "passed": passed,
            "duration": duration,
            "returncode": returncode,
        })

    total_duration = time.time() - total_start

    # Print summary table
    print("\n\n" + "=" * 60)
    print("  TEST SUMMARY")
    print("=" * 60)
    print()

    header = f"  {'Test':<40} {'Status':<8} {'Duration':>10}"
    print(header)
    print("  " + "-" * (len(header) - 2))

    n_pass = 0
    n_fail = 0
    for r in results:
        status = "PASS" if r["passed"] else "FAIL"
        duration_str = f"{r['duration']:.1f}s"
        marker = " " if r["passed"] else "*"
        print(f" {marker}{r['test']:<40} {status:<8} {duration_str:>10}")
        if r["passed"]:
            n_pass += 1
        else:
            n_fail += 1

    print("  " + "-" * (len(header) - 2))
    print(f"  {'TOTAL':<40} {'':<8} {total_duration:>9.1f}s")
    print()
    print(f"  Passed: {n_pass}/{len(results)}")
    print(f"  Failed: {n_fail}/{len(results)}")

    if n_fail > 0:
        failed_names = [r["test"] for r in results if not r["passed"]]
        print(f"\n  Failed tests:")
        for name in failed_names:
            print(f"    - {name}")

    print()
    if n_fail == 0:
        print("  OVERALL: PASS\n")
        return 0
    else:
        print("  OVERALL: FAIL\n")
        return 1


if __name__ == "__main__":
    sys.exit(main())
