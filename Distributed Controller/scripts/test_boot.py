#!/usr/bin/env python3
"""
Test network boot and node discovery.

Sends the "W" reboot command, waits for the system to reach "Ready.",
then queries how many nodes were discovered and their IDs.

Exit code 0 = PASS (3 nodes found), 1 = FAIL.
"""

import argparse
import os
import re
import sys
import time

from serial_utils import connect, send, wait_ready

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
EXPECTED_NODES = 3
BOOT_TIMEOUT_S = 60


def collect_boot_log(ser, timeout):
    """Read all lines until 'Ready.' or timeout. Return (lines, ready_found)."""
    lines = []
    ready = False
    start = time.time()
    while time.time() - start < timeout:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if line:
            lines.append(line)
            print(f"  [{line}]")
        if "Ready." in line:
            ready = True
            break
    return lines, ready


def discover_nodes(ser):
    """Try to identify active nodes by querying lux on IDs 1-5."""
    found = []
    for nid in range(1, 6):
        resp = send(ser, f"g l {nid}")
        if resp and "err" not in resp.lower():
            # Check if we got a numeric value back
            parts = resp.split()
            if len(parts) >= 3:
                try:
                    float(parts[2])
                    found.append(nid)
                    continue
                except ValueError:
                    pass
            # Fallback: any number in response
            if re.search(r"[-+]?\d*\.?\d+", resp):
                found.append(nid)
        time.sleep(0.05)
    return found


def main():
    parser = argparse.ArgumentParser(description="Test network boot and node discovery")
    parser.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    args = parser.parse_args()

    ser = connect(port=args.port)

    print("\n=== Network Boot Test ===\n")

    # Trigger reboot
    print("Sending reboot command (W)...")
    ser.reset_input_buffer()
    ser.write(b"W\n")

    # Wait for Ready
    print("Waiting for system boot (timeout {}s)...".format(BOOT_TIMEOUT_S))
    boot_lines, ready = collect_boot_log(ser, BOOT_TIMEOUT_S)

    if not ready:
        print("\nRESULT: FAIL - System did not reach 'Ready.' within timeout.\n")
        ser.close()
        return 1

    print("\nSystem is ready. Discovering nodes...")
    time.sleep(0.5)

    # Discover nodes
    nodes = discover_nodes(ser)
    print(f"  Discovered nodes: {nodes} (count={len(nodes)})")

    # Also try the D (distributed state) command for extra info
    ser.reset_input_buffer()
    ser.write(b"D\n")
    time.sleep(0.2)
    while ser.in_waiting:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if line:
            print(f"  [D] {line}")

    ser.close()

    if len(nodes) >= EXPECTED_NODES:
        print(f"\nRESULT: PASS - {len(nodes)} nodes discovered (expected {EXPECTED_NODES}).\n")
        return 0
    else:
        print(f"\nRESULT: FAIL - Only {len(nodes)} nodes discovered (expected {EXPECTED_NODES}).\n")
        return 1


if __name__ == "__main__":
    sys.exit(main())
