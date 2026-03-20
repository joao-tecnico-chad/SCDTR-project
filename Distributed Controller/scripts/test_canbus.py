#!/usr/bin/env python3
"""
Test CAN-BUS communication between nodes.

Sends getter commands targeting each remote node via the hub and verifies
that responses arrive within 500 ms.  Measures round-trip latency and
saves results to canbus_latency.csv.

Exit code 0 = PASS, 1 = FAIL.
"""

import argparse
import csv
import os
import sys
import time

from serial_utils import connect, send, wait_ready

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = os.path.join(SCRIPT_DIR, "canbus_latency.csv")

NODE_IDS = [1, 2, 3]
LATENCY_LIMIT_MS = 500
REPETITIONS = 5


def measure_latency(ser, cmd):
    """Send a command and return (response, latency_ms)."""
    ser.reset_input_buffer()
    t0 = time.perf_counter()
    ser.write(f"{cmd}\n".encode())
    resp = ser.readline().decode("utf-8", errors="replace").strip()
    t1 = time.perf_counter()
    latency_ms = (t1 - t0) * 1000
    return resp, latency_ms


def main():
    parser = argparse.ArgumentParser(description="Test CAN-BUS communication")
    parser.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    args = parser.parse_args()

    ser = connect(port=args.port)

    # Drain any leftover output
    time.sleep(0.3)
    ser.reset_input_buffer()

    results = []  # (node_id, rep, cmd, response, latency_ms, ok)
    all_ok = True

    print("\n=== CAN-BUS Communication Test ===\n")

    for nid in NODE_IDS:
        cmd = f"g l {nid}"
        print(f"Node {nid} - command: '{cmd}'")
        for rep in range(1, REPETITIONS + 1):
            resp, lat = measure_latency(ser, cmd)
            ok = bool(resp) and lat < LATENCY_LIMIT_MS
            status = "OK" if ok else "FAIL"
            print(f"  rep {rep}: response='{resp}' latency={lat:.1f} ms [{status}]")
            results.append((nid, rep, cmd, resp, round(lat, 2), ok))
            if not ok:
                all_ok = False
            time.sleep(0.05)
        print()

    # Save CSV
    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["node_id", "rep", "command", "response", "latency_ms", "pass"])
        for row in results:
            writer.writerow(row)
    print(f"Results saved to {CSV_FILE}")

    # Summary
    latencies = [r[4] for r in results if r[5]]
    if latencies:
        avg = sum(latencies) / len(latencies)
        mx = max(latencies)
        print(f"\nLatency  avg={avg:.1f} ms  max={mx:.1f} ms  (limit={LATENCY_LIMIT_MS} ms)")

    if all_ok:
        print("\nRESULT: PASS - All nodes responded within latency limit.\n")
        ser.close()
        return 0
    else:
        failed_nodes = sorted({r[0] for r in results if not r[5]})
        print(f"\nRESULT: FAIL - Nodes with issues: {failed_nodes}\n")
        ser.close()
        return 1


if __name__ == "__main__":
    sys.exit(main())
