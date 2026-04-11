#!/usr/bin/env python3
"""
Programmatic disturbance test: force one node's duty to 0.8 (open-loop),
then release back to feedback. Fast polling on disturbed node only.
"""

import serial
import time
import csv
import os
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

DISTURB_NODE = 2        # Node to disturb
OBSERVE_NODE = 1        # Neighbour to observe cross-coupling effect
DISTURB_DUTY_PWM = 3276 # ~0.8 duty
SETTLE_BEFORE = 5.0
DISTURB_DURATION = 6.0
SETTLE_AFTER = 6.0


def cmd(ser, command, delay=0.1):
    ser.read(4096)
    ser.write(f'{command}\n'.encode())
    time.sleep(delay)
    result = b''
    for _ in range(8):
        chunk = ser.read(4096)
        if chunk:
            result += chunk
            time.sleep(0.015)
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


def fast_read(ser, nid):
    """Read lux and duty from a node as fast as possible."""
    lux = parse_float(cmd(ser, f'g y {nid}', delay=0.05))
    duty = parse_float(cmd(ser, f'g u {nid}', delay=0.05)) / 4095.0
    return lux, duty


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)
    ports = {}

    print("Connecting to Picos...")
    for nid, port in PORTS.items():
        s = serial.Serial(port, BAUD, timeout=0.2)
        time.sleep(0.5)
        s.read(4096)
        ports[nid] = s
        print(f"  Connected Pico {nid}")

    # Setup: all nodes ref=20, feedback on, consensus
    print("\nSetup: ref=20, feedback on, consensus...")
    for n in [1, 2, 3]:
        cmd(ports[n], f'A 1', delay=0.2)
        cmd(ports[n], f'r {n} 20', delay=0.1)
        cmd(ports[n], f'f {n} 1', delay=0.1)
        cmd(ports[n], f'C {n} 1', delay=0.1)
    time.sleep(3)

    total_duration = SETTLE_BEFORE + DISTURB_DURATION + SETTLE_AFTER
    t_on = SETTLE_BEFORE
    t_off = SETTLE_BEFORE + DISTURB_DURATION

    data = []
    start = time.time()
    disturb_applied = False
    disturb_released = False

    print(f"\nFast polling Node {OBSERVE_NODE} (neighbour) for {total_duration:.0f}s...")
    print(f"  Disturbance ON at t={t_on:.0f}s (Node {DISTURB_NODE} forced to ~0.8)")
    print(f"  Disturbance OFF at t={t_off:.0f}s (feedback restored)")

    while True:
        t = time.time() - start
        if t >= total_duration:
            break

        # Apply disturbance via Node 2's own port (non-blocking for observer)
        if not disturb_applied and t >= t_on:
            print(f"  [{t:.1f}s] Disturbance ON")
            cmd(ports[DISTURB_NODE], f'f {DISTURB_NODE} 0', delay=0.05)
            cmd(ports[DISTURB_NODE], f'u {DISTURB_NODE} {DISTURB_DUTY_PWM}', delay=0.05)
            disturb_applied = True

        if not disturb_released and t >= t_off:
            print(f"  [{t:.1f}s] Disturbance OFF")
            cmd(ports[DISTURB_NODE], f'f {DISTURB_NODE} 1', delay=0.05)
            disturb_released = True

        # Fast poll the OBSERVER node (via its own serial port - no hub relay)
        lux, duty = fast_read(ports[OBSERVE_NODE], OBSERVE_NODE)
        t_now = time.time() - start
        data.append({
            'time_s': round(t_now, 3),
            'node_id': OBSERVE_NODE,
            'lux': lux,
            'duty': duty,
            'ref': 20.0
        })

    # Also do a quick snapshot of all nodes at the end
    print("\n  Final state:")
    for n in [1, 2, 3]:
        lux, duty = fast_read(ports[n], n)
        print(f"    Node {n}: lux={lux:.1f}, duty={duty:.3f}")

    # Save CSV
    csv_path = os.path.join(RESULTS_DIR, 'disturbance_programmatic.csv')
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['time_s', 'node_id', 'lux', 'duty', 'ref'])
        w.writeheader()
        w.writerows(data)
    print(f"\n  Saved {csv_path} ({len(data)} samples)")

    # Plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    ts = [d['time_s'] for d in data]
    lx = [d['lux'] for d in data]
    dt = [d['duty'] for d in data]

    ax1.plot(ts, lx, 'b-', linewidth=1, label=f'Node {OBSERVE_NODE} lux')
    ax1.axhline(y=20, color='gray', linestyle='--', alpha=0.5, label='Ref (20)')
    ax1.axvline(x=t_on, color='red', linestyle='--', alpha=0.7, label=f'Node {DISTURB_NODE} forced to 0.8')
    ax1.axvline(x=t_off, color='green', linestyle='--', alpha=0.7, label=f'Node {DISTURB_NODE} feedback restored')

    ax2.plot(ts, dt, 'b-', linewidth=1, label=f'Node {OBSERVE_NODE} duty')
    ax2.axvline(x=t_on, color='red', linestyle='--', alpha=0.7)
    ax2.axvline(x=t_off, color='green', linestyle='--', alpha=0.7)

    ax1.set_ylabel('Illuminance [lux]')
    ax1.set_title(f'Programmatic Disturbance: Node {DISTURB_NODE} duty forced to 0.8, observing Node {OBSERVE_NODE}')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    ax2.set_ylabel('Duty Cycle')
    ax2.set_xlabel('Time [s]')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    png_path = os.path.join(RESULTS_DIR, 'disturbance_programmatic.png')
    plt.savefig(png_path, dpi=150)
    plt.close()
    print(f"  Plot saved: {png_path}")

    # Restore
    for n in [1, 2, 3]:
        cmd(ports[n], f'f {n} 1', delay=0.1)
    for s in ports.values():
        s.close()

    print("\nDone!")


if __name__ == '__main__':
    main()
