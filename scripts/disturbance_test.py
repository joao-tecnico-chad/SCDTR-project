#!/usr/bin/env python3
"""
Disturbance rejection test: record lux/duty while user opens and closes the box.
Press Enter at each prompt to mark the disturbance events.
"""

import serial
import time
import csv
import os
import sys
import threading
import matplotlib.pyplot as plt
import numpy as np

PORTS = {
    1: '/dev/cu.usbmodem11101',
    2: '/dev/cu.usbmodem11201',
    3: '/dev/cu.usbmodem11301',
}
BAUD = 115200
RESULTS_DIR = os.path.join(os.path.dirname(__file__), '..', 'results')
NODE_IDS = [1, 2, 3]


def cmd(ser, command, delay=0.1):
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
    # Filter out can_err and other noise lines
    lines = [l for l in text.split('\n') if not l.startswith('can_err') and l.strip()]
    return '\n'.join(lines)


def parse_float(response):
    """Extract last numeric value from a command response."""
    for line in reversed(response.split('\n')):
        parts = line.strip().split()
        if parts:
            try:
                return float(parts[-1])
            except ValueError:
                continue
    return None


def get_lux(ser, nid):
    r = cmd(ser, f'g y {nid}', delay=0.1)
    return parse_float(r)


def get_duty(ser, nid):
    r = cmd(ser, f'g u {nid}', delay=0.1)
    v = parse_float(r)
    if v is not None:
        return v / 4095.0
    return None


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)

    # Connect
    ports = {}
    for nid, port in PORTS.items():
        try:
            s = serial.Serial(port, BAUD, timeout=0.1)
            time.sleep(0.5)
            s.read(4096)
            ports[nid] = s
            print(f"  Connected to Pico {nid} on {port}")
        except Exception as e:
            print(f"  FAILED Pico {nid}: {e}")
    if len(ports) < 3:
        print("ERROR: Not all Picos connected.")
        return

    # Set reference to 20 lux, feedback ON
    ref = 20.0
    print(f"\nSetting all nodes to ref={ref} with feedback ON...")
    for n in NODE_IDS:
        cmd(ports[n], f'r {n} {ref}', delay=0.1)
        cmd(ports[n], f'f {n} 1', delay=0.1)
    print("Waiting 5s for steady state...")
    time.sleep(5)

    data = []
    start = time.time()
    stop_flag = threading.Event()

    def record_loop():
        while not stop_flag.is_set():
            for n in NODE_IDS:
                if stop_flag.is_set():
                    break
                t_s = time.time() - start
                lux = get_lux(ports[n], n)
                duty = get_duty(ports[n], n)
                if lux is not None and duty is not None:
                    data.append({'time_s': round(t_s, 3), 'node_id': n, 'lux': lux, 'duty': duty, 'ref': ref})

    recorder = threading.Thread(target=record_loop, daemon=True)
    recorder.start()

    print("\nRecording started. Keep the box CLOSED for now.")
    input(">>> Press ENTER when you OPEN the box...")
    t_open = time.time() - start
    print(f"  [BOX OPENED at t={t_open:.1f}s]")

    input(">>> Press ENTER when you CLOSE the box...")
    t_close = time.time() - start
    print(f"  [BOX CLOSED at t={t_close:.1f}s]")

    print("Recording recovery for 15 more seconds...")
    time.sleep(15)
    stop_flag.set()
    recorder.join(timeout=5)

    print(f"\nRecorded {len(data)} samples.")
    print(f"  Open  at t={t_open:.1f}s")
    print(f"  Close at t={t_close:.1f}s")

    # Save CSV
    csv_path = os.path.join(RESULTS_DIR, 'disturbance_rejection.csv')
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['time_s', 'node_id', 'lux', 'duty', 'ref'])
        writer.writeheader()
        writer.writerows(data)
    print(f"  Saved {csv_path}")

    # Save event times
    events_path = os.path.join(RESULTS_DIR, 'disturbance_events.txt')
    with open(events_path, 'w') as f:
        f.write(f"open={t_open:.3f}\n")
        f.write(f"close={t_close:.3f}\n")
    print(f"  Saved {events_path}")

    # Plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    colors = {1: 'tab:blue', 2: 'tab:orange', 3: 'tab:green'}

    for n in NODE_IDS:
        nd = [d for d in data if d['node_id'] == n]
        ts = [d['time_s'] for d in nd]
        lux = [d['lux'] for d in nd]
        duty = [d['duty'] for d in nd]
        ax1.plot(ts, lux, color=colors[n], label=f'Node {n}', linewidth=1)
    ax1.axhline(y=ref, color='gray', linestyle='--', alpha=0.5, label=f'Ref ({ref})')
    ax1.axvline(x=t_open, color='red', linestyle='--', alpha=0.7, label='Box opened')
    ax1.axvline(x=t_close, color='green', linestyle='--', alpha=0.7, label='Box closed')
    ax1.set_ylabel('Illuminance [lux]')
    ax1.set_title('Disturbance Rejection: Box Open/Close')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    for n in NODE_IDS:
        nd = [d for d in data if d['node_id'] == n]
        ts = [d['time_s'] for d in nd]
        duty = [d['duty'] for d in nd]
        ax2.plot(ts, duty, color=colors[n], label=f'Node {n}', linewidth=1)
    ax2.axvline(x=t_open, color='red', linestyle='--', alpha=0.7)
    ax2.axvline(x=t_close, color='green', linestyle='--', alpha=0.7)
    ax2.set_ylabel('Duty Cycle')
    ax2.set_xlabel('Time [s]')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(RESULTS_DIR, 'disturbance_rejection.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"  Plot saved: {out}")

    # Cleanup
    for s in ports.values():
        s.close()
    print("\nDone!")


if __name__ == '__main__':
    main()
