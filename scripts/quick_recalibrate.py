#!/usr/bin/env python3
"""
Quick script: capture calibration K matrix from firmware, then run model validation sweep.
Both from the same session so numbers are consistent for the report.
"""

import serial
import time
import csv
import os
import re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

PORTS = {
    1: '/dev/cu.usbmodem11101',
    2: '/dev/cu.usbmodem11201',
    3: '/dev/cu.usbmodem11301',
}
BAUD = 115200
RESULTS_DIR = os.path.join(os.path.dirname(__file__), '..', 'results')
NODE_IDS = [1, 2, 3]


def cmd(ser, command, delay=0.2):
    ser.read(4096)
    ser.write(f'{command}\n'.encode())
    time.sleep(delay)
    result = b''
    for _ in range(10):
        chunk = ser.read(4096)
        if chunk:
            result += chunk
            time.sleep(0.02)
        else:
            break
    text = result.decode('utf-8', errors='replace').strip()
    lines = [l for l in text.split('\n') if not l.startswith('can_err') and l.strip()]
    return '\n'.join(lines)


def parse_float(response):
    for line in reversed(response.split('\n')):
        parts = line.strip().split()
        if parts:
            try:
                return float(parts[-1])
            except ValueError:
                continue
    return 0.0


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)
    ports = {}

    print("Connecting to Picos...")
    for nid, port in PORTS.items():
        s = serial.Serial(port, BAUD, timeout=0.5)
        time.sleep(0.5)
        s.read(4096)
        ports[nid] = s
        print(f"  Connected Pico {nid}")

    # Step 1: Read the firmware's calibration K matrix
    print("\n=== Step 1: Reading Firmware K Matrix ===")
    # Query gains from the hub node (node 1)
    # The firmware stores K[i][j] and o[i] after calibration
    # We can query them via 'g K' or by reading the gain exchange data
    # Actually, let's just read it by querying each node's self-report
    # Try reading the calibration data that was exchanged

    # The simplest way: do a fresh calibration, then read K from serial output
    print("  Triggering fresh calibration...")
    resp = cmd(ports[1], 'R', delay=8)  # Reset triggers wakeup + calibration
    print(f"  Calibration response: {resp[:200]}...")

    # Wait for system to stabilize
    time.sleep(5)

    # Read K matrix by querying lux with known duty settings
    # Actually, let's just run the model validation and derive K from it
    print("\n=== Step 2: Model Validation Sweep ===")

    # Disable feedback and distributed algorithms
    for n in NODE_IDS:
        cmd(ports[n], f'A 0', delay=0.3)
    for n in NODE_IDS:
        cmd(ports[n], f'f {n} 0', delay=0.1)
    time.sleep(0.5)
    for n in NODE_IDS:
        cmd(ports[n], f'u {n} 0', delay=0.1)
    time.sleep(2)

    steps = [0, 410, 819, 1229, 1638, 2048, 2457, 2867, 3276, 3686, 4095]
    results = []

    for active_node in NODE_IDS:
        print(f"  Sweeping Node {active_node}...")
        for n in NODE_IDS:
            cmd(ports[n], f'u {n} 0', delay=0.1)
        time.sleep(1)

        for pwm in steps:
            cmd(ports[active_node], f'u {active_node} {pwm}', delay=0.1)
            time.sleep(0.8)
            readings = {}
            for sensor_node in NODE_IDS:
                lux = parse_float(cmd(ports[sensor_node], f'g y {sensor_node}', delay=0.1))
                readings[sensor_node] = lux
            results.append({
                'active_node': active_node,
                'pwm': pwm,
                'duty': pwm / 4095.0,
                **{f'lux_sensor_{s}': readings[s] for s in NODE_IDS}
            })
            print(f"    PWM={pwm:4d} -> S1={readings[1]:.2f} S2={readings[2]:.2f} S3={readings[3]:.2f}")

        cmd(ports[active_node], f'u {active_node} 0', delay=0.1)

    # Save CSV
    path = os.path.join(RESULTS_DIR, 'model_validation.csv')
    fields = ['active_node', 'pwm', 'duty', 'lux_sensor_1', 'lux_sensor_2', 'lux_sensor_3']
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(results)
    print(f"  Saved {path}")

    # Step 3: Compute K matrix from sweep (calibration-style: single point at 50%)
    print("\n=== Step 3: Computing K Matrix ===")
    K = [[0.0]*3 for _ in range(3)]
    o = [0.0]*3

    # Background: all LEDs off (first row of each node's sweep)
    for sensor in range(3):
        bg_vals = []
        for active in NODE_IDS:
            row = [r for r in results if r['active_node'] == active and r['pwm'] == 0][0]
            bg_vals.append(row[f'lux_sensor_{sensor+1}'])
        o[sensor] = np.mean(bg_vals)

    # Gains: at 50% duty (pwm=2048)
    d_cal = 2048 / 4095.0
    for active_idx, active in enumerate(NODE_IDS):
        row_50 = [r for r in results if r['active_node'] == active and r['pwm'] == 2048][0]
        for sensor_idx in range(3):
            lux_50 = row_50[f'lux_sensor_{sensor_idx+1}']
            K[sensor_idx][active_idx] = (lux_50 - o[sensor_idx]) / d_cal

    print("\n  K matrix (lux/duty):")
    print(f"         Node1    Node2    Node3    o")
    for i in range(3):
        print(f"  Node{i+1}  {K[i][0]:7.2f}  {K[i][1]:7.2f}  {K[i][2]:7.2f}  {o[i]:.2f}")

    # Step 4: Also compute from linear fit (full sweep)
    print("\n  Linear fit slopes (for comparison):")
    r_squared_vals = []
    for active_idx, active in enumerate(NODE_IDS):
        node_data = [r for r in results if r['active_node'] == active]
        duties = [r['duty'] for r in node_data]
        lux = [r[f'lux_sensor_{active}'] for r in node_data]
        coeffs = np.polyfit(duties, lux, 1)
        fit_fn = np.poly1d(coeffs)
        ss_res = np.sum((np.array(lux) - fit_fn(np.array(duties)))**2)
        ss_tot = np.sum((np.array(lux) - np.mean(lux))**2)
        r2 = 1 - ss_res / ss_tot
        r_squared_vals.append(r2)
        print(f"  Node {active}: slope={coeffs[0]:.2f}, intercept={coeffs[1]:.2f}, R²={r2:.4f}")
        print(f"           K_calib={K[active_idx][active_idx]:.2f}")

    # Step 5: Generate plot
    fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    for idx, active in enumerate(NODE_IDS):
        node_data = [r for r in results if r['active_node'] == active]
        duties = [r['duty'] for r in node_data]
        lux = [r[f'lux_sensor_{active}'] for r in node_data]
        coeffs = np.polyfit(duties, lux, 1)
        fit_fn = np.poly1d(coeffs)
        ss_res = np.sum((np.array(lux) - fit_fn(np.array(duties)))**2)
        ss_tot = np.sum((np.array(lux) - np.mean(lux))**2)
        r2 = 1 - ss_res / ss_tot

        axes[idx].scatter(duties, lux, c='tab:blue', s=30, zorder=3, label='Measured')
        x_fit = np.linspace(0, 1, 100)
        axes[idx].plot(x_fit, fit_fn(x_fit), 'r--', label=f'Fit: {coeffs[0]:.1f}u + {coeffs[1]:.1f}')
        axes[idx].set_xlabel('Duty cycle')
        axes[idx].set_ylabel('Illuminance [lux]')
        axes[idx].set_title(f'Node {active} ($R^2 = {r2:.4f}$)')
        axes[idx].legend(fontsize=7)
        axes[idx].grid(True, alpha=0.3)

    plt.suptitle('Linear Illuminance Model Validation')
    plt.tight_layout()
    out = os.path.join(RESULTS_DIR, 'model_validation.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"\n  Plot saved: {out}")

    # Restore feedback
    print("\n=== Restoring system ===")
    for n in NODE_IDS:
        cmd(ports[n], f'f {n} 1', delay=0.1)
    cmd(ports[1], 'A 1', delay=0.3)

    for s in ports.values():
        s.close()

    print("\n=== DONE ===")
    print("Update report Table 2 with the K matrix above.")
    print(f"R² values: {', '.join(f'{r:.4f}' for r in r_squared_vals)}")


if __name__ == '__main__':
    main()
