#!/usr/bin/env python3
"""
Test system calibration.

Sends "R" to trigger recalibration, waits for completion, reads back
the K (gain) matrix via the "K" command, validates its properties, and
saves results to calibration_matrix.csv.

Validation rules:
  - K[i][i] > 0   (positive self-gain)
  - K[i][j] >= 0   (non-negative cross-coupling)
  - o[i] >= 0      (non-negative background lux)

Exit code 0 = PASS, 1 = FAIL.
"""

import argparse
import csv
import os
import re
import sys
import time

from serial_utils import connect, wait_ready, send

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = os.path.join(SCRIPT_DIR, "calibration_matrix.csv")

NUM_NODES = 3
CALIBRATION_TIMEOUT_S = 120


def read_k_matrix(ser):
    """
    Send 'K' command and parse the gain matrix and background offsets.
    Returns (K, o) where K is a list of rows and o is a list of offsets,
    or (None, None) on failure.
    """
    ser.reset_input_buffer()
    ser.write(b"K\n")
    time.sleep(0.5)

    lines = []
    deadline = time.time() + 5
    while time.time() < deadline:
        if ser.in_waiting:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                lines.append(line)
                print(f"  [K] {line}")
        else:
            time.sleep(0.05)
            if not ser.in_waiting:
                break

    # Parse the output
    # Expected formats vary; try to extract numbers from each line
    k_matrix = []
    offsets = []

    for line in lines:
        numbers = re.findall(r"[-+]?\d*\.?\d+", line)
        lower = line.lower()

        if "o" in lower and ("background" in lower or "offset" in lower or "o[" in lower or "o =" in lower):
            offsets = [float(n) for n in numbers]
        elif len(numbers) >= NUM_NODES:
            # Likely a K-matrix row
            k_matrix.append([float(n) for n in numbers[:NUM_NODES]])

    # If we could not parse structured output, try treating all lines as matrix rows
    if not k_matrix:
        for line in lines:
            nums = re.findall(r"[-+]?\d*\.?\d+", line)
            if len(nums) >= 2:
                k_matrix.append([float(n) for n in nums])

    # Try to separate offsets from the last column if matrix is wider
    if not offsets and k_matrix and len(k_matrix[0]) > NUM_NODES:
        offsets = [row[-1] for row in k_matrix]
        k_matrix = [row[:NUM_NODES] for row in k_matrix]

    # If still no offsets, try extracting from individual queries
    if not offsets:
        print("  Trying to query offsets individually...")
        for nid in range(1, NUM_NODES + 1):
            resp = send(ser, f"g o {nid}")
            match = re.search(r"[-+]?\d*\.?\d+", resp)
            if match:
                offsets.append(float(match.group()))

    if len(k_matrix) < NUM_NODES:
        print(f"  WARNING: Expected {NUM_NODES} K-matrix rows, got {len(k_matrix)}")
        return None, None

    return k_matrix, offsets


def validate_matrix(k_matrix, offsets):
    """Validate K matrix and offsets. Returns (valid, issues)."""
    issues = []

    for i in range(len(k_matrix)):
        row = k_matrix[i]
        # Self-gain must be positive
        if i < len(row) and row[i] <= 0:
            issues.append(f"K[{i}][{i}] = {row[i]} (expected > 0)")

        # Cross-coupling must be non-negative
        for j in range(len(row)):
            if j != i and row[j] < 0:
                issues.append(f"K[{i}][{j}] = {row[j]} (expected >= 0)")

    for i, o_val in enumerate(offsets):
        if o_val < 0:
            issues.append(f"o[{i}] = {o_val} (expected >= 0)")

    return len(issues) == 0, issues


def save_matrix(k_matrix, offsets):
    """Save K matrix and offsets to CSV."""
    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        header = [f"K_col{j+1}" for j in range(len(k_matrix[0]))]
        header.append("offset")
        writer.writerow(header)
        for i, row in enumerate(k_matrix):
            csv_row = list(row)
            csv_row.append(offsets[i] if i < len(offsets) else "")
            writer.writerow(csv_row)
    print(f"  Saved {CSV_FILE}")


def main():
    parser = argparse.ArgumentParser(description="Test system calibration")
    parser.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    args = parser.parse_args()

    ser = connect(port=args.port)

    print("\n=== Calibration Test ===\n")

    # Trigger recalibration
    print("Sending recalibration command (R)...")
    ser.reset_input_buffer()
    ser.write(b"R\n")

    print(f"Waiting for calibration to complete (timeout {CALIBRATION_TIMEOUT_S}s)...")
    ready = wait_ready(ser, timeout=CALIBRATION_TIMEOUT_S)

    if not ready:
        print("\nRESULT: FAIL - Calibration did not complete within timeout.\n")
        ser.close()
        return 1

    time.sleep(0.5)

    # Read K matrix
    print("\nReading K matrix...")
    k_matrix, offsets = read_k_matrix(ser)

    if k_matrix is None:
        print("\nRESULT: FAIL - Could not read K matrix.\n")
        ser.close()
        return 1

    print(f"\nK matrix ({len(k_matrix)}x{len(k_matrix[0])}):")
    for i, row in enumerate(k_matrix):
        print(f"  Row {i+1}: {row}")
    print(f"  Offsets: {offsets}")

    # Validate
    valid, issues = validate_matrix(k_matrix, offsets)

    # Save regardless of validity
    save_matrix(k_matrix, offsets)

    if valid:
        print("\nRESULT: PASS - K matrix is valid.\n")
        ser.close()
        return 0
    else:
        print("\nValidation issues:")
        for issue in issues:
            print(f"  - {issue}")
        print("\nRESULT: FAIL - K matrix has invalid entries.\n")
        ser.close()
        return 1


if __name__ == "__main__":
    sys.exit(main())
