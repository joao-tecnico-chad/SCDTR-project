#!/usr/bin/env python3
"""
Complete test suite for Phase 2 report.
Collects: calibration, model validation, algorithm comparison,
step response, occupancy change, and generates all plots.
"""

import serial
import time
import csv
import os
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

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


class Controller:
    def __init__(self):
        self.ports = {}
        os.makedirs(RESULTS_DIR, exist_ok=True)

    def connect(self):
        for nid, port in PORTS.items():
            s = serial.Serial(port, BAUD, timeout=0.5)
            time.sleep(0.5)
            s.read(4096)
            self.ports[nid] = s
            print(f"  Connected Pico {nid}")

    def close(self):
        for s in self.ports.values():
            s.close()
        self.ports.clear()

    def reset_metrics(self):
        for n in NODE_IDS:
            cmd(self.ports[n], 'RM', delay=0.1)

    def set_algorithm(self, alg_id):
        for n in NODE_IDS:
            cmd(self.ports[n], f'A {alg_id}', delay=0.3)

    def set_ref(self, nid, ref):
        cmd(self.ports[nid], f'r {nid} {ref}', delay=0.1)

    def set_feedback(self, nid, on):
        cmd(self.ports[nid], f'f {nid} {1 if on else 0}', delay=0.1)

    def set_occupancy(self, nid, state):
        cmd(self.ports[nid], f'o {nid} {state}', delay=0.1)

    def set_cost(self, nid, cost):
        cmd(self.ports[nid], f'C {nid} {cost}', delay=0.1)

    def set_duty(self, nid, pwm):
        cmd(self.ports[nid], f'u {nid} {pwm}', delay=0.1)

    def get_lux(self, nid):
        return parse_float(cmd(self.ports[nid], f'g y {nid}', delay=0.1))

    def get_duty(self, nid):
        return parse_float(cmd(self.ports[nid], f'g u {nid}', delay=0.1)) / 4095.0

    def get_ref(self, nid):
        return parse_float(cmd(self.ports[nid], f'g r {nid}', delay=0.1))

    def get_metrics(self, nid):
        e = parse_float(cmd(self.ports[nid], f'g E {nid}', delay=0.1))
        v = parse_float(cmd(self.ports[nid], f'g V {nid}', delay=0.1))
        f = parse_float(cmd(self.ports[nid], f'g F {nid}', delay=0.1))
        return {'energy': e, 'visibility': v, 'flicker': f}

    def poll_single(self, nid, duration_s, t_offset=0, ref_override=None):
        data = []
        ref = ref_override if ref_override is not None else self.get_ref(nid)
        start = time.time()
        while time.time() - start < duration_s:
            t_s = time.time() - start + t_offset
            lux = self.get_lux(nid)
            duty = self.get_duty(nid)
            data.append({'time_s': round(t_s, 3), 'node_id': nid, 'lux': lux, 'duty': duty, 'ref': ref})
        return data


def test_model_validation(ctrl):
    """Sweep duty cycle 0-100% for each node, measure lux at all sensors. Validates linear model."""
    print("\n=== Model Validation (Linearity Check) ===")

    # Disable feedback and distributed algorithms on all nodes
    ctrl.set_algorithm(0)
    for n in NODE_IDS:
        ctrl.set_feedback(n, False)
    time.sleep(0.5)
    for n in NODE_IDS:
        ctrl.set_duty(n, 0)
    time.sleep(2)

    steps = [0, 410, 819, 1229, 1638, 2048, 2457, 2867, 3276, 3686, 4095]
    results = []

    for active_node in NODE_IDS:
        print(f"  Sweeping Node {active_node}...")
        # All LEDs off first
        for n in NODE_IDS:
            ctrl.set_duty(n, 0)
        time.sleep(1)

        for pwm in steps:
            ctrl.set_duty(active_node, pwm)
            time.sleep(0.8)  # settle
            readings = {}
            for sensor_node in NODE_IDS:
                lux = ctrl.get_lux(sensor_node)
                readings[sensor_node] = lux
            results.append({
                'active_node': active_node,
                'pwm': pwm,
                'duty': pwm / 4095.0,
                **{f'lux_sensor_{s}': readings[s] for s in NODE_IDS}
            })
            print(f"    PWM={pwm:4d} -> S1={readings[1]:.2f} S2={readings[2]:.2f} S3={readings[3]:.2f}")

        ctrl.set_duty(active_node, 0)

    # Save CSV
    path = os.path.join(RESULTS_DIR, 'model_validation.csv')
    fields = ['active_node', 'pwm', 'duty', 'lux_sensor_1', 'lux_sensor_2', 'lux_sensor_3']
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(results)
    print(f"  Saved {path}")

    # Plot: self-gain linearity for each node
    fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    for idx, active in enumerate(NODE_IDS):
        node_data = [r for r in results if r['active_node'] == active]
        duties = [r['duty'] for r in node_data]
        lux = [r[f'lux_sensor_{active}'] for r in node_data]

        # Linear fit
        coeffs = np.polyfit(duties, lux, 1)
        fit_line = np.poly1d(coeffs)
        r_squared = 1 - np.sum((np.array(lux) - fit_line(np.array(duties)))**2) / np.sum((np.array(lux) - np.mean(lux))**2)

        axes[idx].scatter(duties, lux, c='tab:blue', s=30, zorder=3, label='Measured')
        x_fit = np.linspace(0, 1, 100)
        axes[idx].plot(x_fit, fit_line(x_fit), 'r--', label=f'Fit: {coeffs[0]:.1f}u + {coeffs[1]:.1f}')
        axes[idx].set_xlabel('Duty cycle')
        axes[idx].set_ylabel('Illuminance [lux]')
        axes[idx].set_title(f'Node {active} ($R^2 = {r_squared:.4f}$)')
        axes[idx].legend(fontsize=7)
        axes[idx].grid(True, alpha=0.3)

    plt.suptitle('Linear Illuminance Model Validation')
    plt.tight_layout()
    out = os.path.join(RESULTS_DIR, 'model_validation.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"  Plot: {out}")


def test_algorithms_equal(ctrl):
    """Test all 4 algorithms with equal costs, 20s each, metrics reset between."""
    print("\n=== Algorithm Comparison (Equal Costs) ===")
    algs = [('PI Only', 0), ('Consensus', 1), ('ADMM', 2), ('Dual Decomp', 3)]
    results = []

    for alg_name, alg_id in algs:
        print(f"  [{alg_name}]...")
        ctrl.set_algorithm(alg_id)
        for n in NODE_IDS:
            ctrl.set_ref(n, 20)
            ctrl.set_feedback(n, True)
            ctrl.set_cost(n, 1.0)
        ctrl.reset_metrics()
        time.sleep(20)

        for n in NODE_IDS:
            m = ctrl.get_metrics(n)
            lux = ctrl.get_lux(n)
            duty = ctrl.get_duty(n)
            results.append({
                'alg': alg_name, 'node': n, 'lux': lux, 'duty': duty, 'ref': 20.0,
                'energy': m['energy'], 'vis': m['visibility'], 'flicker': m['flicker'],
            })
            print(f"    N{n}: lux={lux:.1f} duty={duty:.4f} E={m['energy']:.2f} V={m['visibility']:.4f}")

    path = os.path.join(RESULTS_DIR, 'alg_comparison_equal.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['alg', 'node', 'lux', 'duty', 'ref', 'energy', 'vis', 'flicker'])
        w.writeheader()
        w.writerows(results)
    print(f"  Saved {path}")

    # Plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
    alg_names = [a[0] for a in algs]
    x = np.arange(len(NODE_IDS))
    width = 0.2
    for ai, alg in enumerate(alg_names):
        vals = [r['energy'] for r in results if r['alg'] == alg]
        ax1.bar(x + ai * width, vals, width, label=alg)
    ax1.set_ylabel('Energy [J]')
    ax1.set_xticks(x + width * 1.5)
    ax1.set_xticklabels([f'N{n}' for n in NODE_IDS])
    ax1.legend(fontsize=7)
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Energy (20 s)')

    for ai, alg in enumerate(alg_names):
        vals = [r['vis'] for r in results if r['alg'] == alg]
        ax2.bar(x + ai * width, vals, width, label=alg)
    ax2.set_ylabel('Visibility Error [lux]')
    ax2.set_xticks(x + width * 1.5)
    ax2.set_xticklabels([f'N{n}' for n in NODE_IDS])
    ax2.legend(fontsize=7)
    ax2.grid(True, alpha=0.3)
    ax2.set_title('Visibility Error')

    plt.suptitle('Algorithm Comparison: Equal Costs [1,1,1], ref = 20 lux')
    plt.tight_layout()
    out = os.path.join(RESULTS_DIR, 'alg_comparison_equal.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"  Plot: {out}")


def test_unequal_costs(ctrl):
    """Test consensus with unequal costs."""
    print("\n=== Unequal Costs [1,2,3] (Consensus) ===")
    ctrl.set_algorithm(1)
    for n in NODE_IDS:
        ctrl.set_cost(n, n)
        ctrl.set_ref(n, 20)
        ctrl.set_feedback(n, True)
    ctrl.reset_metrics()
    time.sleep(25)

    results = []
    for n in NODE_IDS:
        lux = ctrl.get_lux(n)
        duty = ctrl.get_duty(n)
        m = ctrl.get_metrics(n)
        results.append({'alg': 'Consensus', 'node': n, 'cost': n, 'lux': lux, 'duty': duty, 'energy': m['energy']})
        print(f"  N{n} (c={n}): lux={lux:.1f} duty={duty:.4f} E={m['energy']:.2f}")

    path = os.path.join(RESULTS_DIR, 'alg_comparison_unequal.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['alg', 'node', 'cost', 'lux', 'duty', 'energy'])
        w.writeheader()
        w.writerows(results)
    print(f"  Saved {path}")

    # Plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
    x = np.arange(len(NODE_IDS))
    colors = ['tab:blue', 'tab:orange', 'tab:green']
    ax1.bar(x, [r['duty'] for r in results], 0.5, color=colors)
    ax1.set_ylabel('Duty Cycle')
    ax1.set_xticks(x)
    ax1.set_xticklabels([f'N{n}\n(c={n})' for n in NODE_IDS])
    ax1.set_title('Duty Allocation')
    ax1.grid(True, alpha=0.3)

    ax2.bar(x, [r['energy'] for r in results], 0.5, color=colors)
    ax2.set_ylabel('Energy [J]')
    ax2.set_xticks(x)
    ax2.set_xticklabels([f'N{n}\n(c={n})' for n in NODE_IDS])
    ax2.set_title('Energy Consumption')
    ax2.grid(True, alpha=0.3)

    plt.suptitle('Consensus with Unequal Costs [1, 2, 3], ref = 20 lux')
    plt.tight_layout()
    out = os.path.join(RESULTS_DIR, 'duty_unequal_costs.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"  Plot: {out}")


def test_step_response(ctrl):
    """Step response node 1: ref 5->25, fast single-node polling."""
    print("\n=== Step Response (Node 1: 5 -> 25) ===")
    ctrl.set_algorithm(0)
    for n in NODE_IDS:
        ctrl.set_ref(n, 5)
        ctrl.set_feedback(n, True)
    time.sleep(5)

    print("  Baseline (5s)...")
    data = ctrl.poll_single(1, 5, t_offset=0, ref_override=5.0)
    t_step = data[-1]['time_s'] + 0.1 if data else 5

    print("  Step -> 25...")
    ctrl.set_ref(1, 25)
    data += ctrl.poll_single(1, 10, t_offset=t_step, ref_override=25.0)

    path = os.path.join(RESULTS_DIR, 'step_response_ff.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['time_s', 'node_id', 'lux', 'duty', 'ref'])
        w.writeheader()
        w.writerows(data)
    print(f"  Saved {path}")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    ts = [d['time_s'] for d in data]
    ax1.plot(ts, [d['lux'] for d in data], 'tab:blue', lw=1.2, label='Node 1 lux')
    ax1.plot(ts, [d['ref'] for d in data], 'r--', alpha=0.6, label='Reference')
    ax1.axvline(x=t_step, color='gray', ls=':', alpha=0.5)
    ax1.set_ylabel('Illuminance [lux]')
    ax1.set_title('Step Response: Node 1 ref 5 → 25 (PI + Feedforward)')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    ax2.plot(ts, [d['duty'] for d in data], 'tab:blue', lw=1.2, label='Node 1 duty')
    ax2.axvline(x=t_step, color='gray', ls=':', alpha=0.5)
    ax2.set_ylabel('Duty Cycle')
    ax2.set_xlabel('Time [s]')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(RESULTS_DIR, 'step_response_ff.png'), dpi=150)
    plt.close()
    print(f"  Plot saved")


def test_occupancy(ctrl):
    """Occupancy change node 1: LOW->HIGH->LOW."""
    print("\n=== Occupancy Change (Node 1) ===")
    ctrl.set_algorithm(0)
    for n in NODE_IDS:
        ctrl.set_occupancy(n, 'l')
        ctrl.set_feedback(n, True)
    time.sleep(5)

    low_ref = ctrl.get_ref(1)
    print(f"  LOW ref = {low_ref}")
    data = ctrl.poll_single(1, 5, t_offset=0, ref_override=low_ref)
    t1 = data[-1]['time_s'] + 0.1 if data else 5

    print("  -> HIGH")
    ctrl.set_occupancy(1, 'h')
    time.sleep(0.2)
    high_ref = ctrl.get_ref(1)
    print(f"  HIGH ref = {high_ref}")
    data += ctrl.poll_single(1, 10, t_offset=t1, ref_override=high_ref)
    t2 = data[-1]['time_s'] + 0.1

    print("  -> LOW")
    ctrl.set_occupancy(1, 'l')
    time.sleep(0.2)
    data += ctrl.poll_single(1, 8, t_offset=t2, ref_override=low_ref)

    path = os.path.join(RESULTS_DIR, 'occupancy_change.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['time_s', 'node_id', 'lux', 'duty', 'ref'])
        w.writeheader()
        w.writerows(data)
    print(f"  Saved {path}")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    ts = [d['time_s'] for d in data]
    ax1.plot(ts, [d['lux'] for d in data], 'tab:blue', lw=1.2, label='Node 1 lux')
    ax1.plot(ts, [d['ref'] for d in data], 'r--', alpha=0.6, label='Reference')
    ax1.axvline(x=t1, color='red', ls=':', alpha=0.5, label='HIGH')
    ax1.axvline(x=t2, color='green', ls=':', alpha=0.5, label='LOW')
    ax1.set_ylabel('Illuminance [lux]')
    ax1.set_title('Occupancy Change: Node 1 LOW → HIGH → LOW')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    ax2.plot(ts, [d['duty'] for d in data], 'tab:blue', lw=1.2)
    ax2.axvline(x=t1, color='red', ls=':', alpha=0.5)
    ax2.axvline(x=t2, color='green', ls=':', alpha=0.5)
    ax2.set_ylabel('Duty Cycle')
    ax2.set_xlabel('Time [s]')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(RESULTS_DIR, 'occupancy_change.png'), dpi=150)
    plt.close()
    print(f"  Plot saved")


def main():
    print(f"Full Report Test Suite - {datetime.now().strftime('%Y-%m-%d %H:%M')}")
    print("=" * 60)

    ctrl = Controller()
    ctrl.connect()

    if len(ctrl.ports) < 3:
        print("ERROR: Not all Picos connected.")
        ctrl.close()
        return

    # Calibrate
    print("\nCalibrating...")
    cmd(ctrl.ports[1], 'c 0', delay=2)
    time.sleep(28)
    for n in NODE_IDS:
        r = cmd(ctrl.ports[n], 'rpt', delay=1)
        lines = r.split('\n')[:5]
        for l in lines:
            print(f"  N{n}: {l}")

    # Run all tests
    test_model_validation(ctrl)
    test_algorithms_equal(ctrl)
    test_unequal_costs(ctrl)
    test_step_response(ctrl)
    test_occupancy(ctrl)

    # Cleanup
    ctrl.set_algorithm(0)
    for n in NODE_IDS:
        ctrl.set_feedback(n, False)
        ctrl.set_duty(n, 0)
    ctrl.close()

    print(f"\n{'='*60}")
    print(f"All tests complete. Results in {RESULTS_DIR}")


if __name__ == '__main__':
    main()
