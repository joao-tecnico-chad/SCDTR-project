#!/usr/bin/env python3
"""
Comprehensive test suite for SCDTR report data.
Resets metrics between each algorithm test for fair comparison.
"""

import serial
import time
import csv
import os
import sys
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
SETTLE_TIME = 20  # seconds for all algorithms (equal measurement time)


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
            try:
                s = serial.Serial(port, BAUD, timeout=0.5)
                time.sleep(0.5)
                s.read(4096)
                self.ports[nid] = s
                print(f"  Connected to Pico {nid}")
            except Exception as e:
                print(f"  FAILED Pico {nid}: {e}")

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

    def get_lux(self, nid):
        return parse_float(cmd(self.ports[nid], f'g y {nid}', delay=0.1))

    def get_duty(self, nid):
        v = parse_float(cmd(self.ports[nid], f'g u {nid}', delay=0.1))
        return v / 4095.0

    def get_ref(self, nid):
        return parse_float(cmd(self.ports[nid], f'g r {nid}', delay=0.1))

    def get_metrics(self, nid):
        e = parse_float(cmd(self.ports[nid], f'g E {nid}', delay=0.1))
        v = parse_float(cmd(self.ports[nid], f'g V {nid}', delay=0.1))
        f = parse_float(cmd(self.ports[nid], f'g F {nid}', delay=0.1))
        return {'energy': e, 'visibility': v, 'flicker': f}

    def poll_all(self, duration_s, t_offset=0):
        data = []
        refs = {}
        for n in NODE_IDS:
            refs[n] = self.get_ref(n)
        start = time.time()
        while time.time() - start < duration_s:
            for n in NODE_IDS:
                t_s = time.time() - start + t_offset
                lux = self.get_lux(n)
                duty = self.get_duty(n)
                data.append({
                    'time_s': round(t_s, 3),
                    'node_id': n,
                    'lux': lux,
                    'duty': duty,
                    'ref': refs.get(n, 0),
                })
        return data

    def poll_single(self, nid, duration_s, t_offset=0, ref_override=None):
        """Fast polling of a single node for better time resolution."""
        data = []
        ref = ref_override if ref_override is not None else self.get_ref(nid)
        start = time.time()
        while time.time() - start < duration_s:
            t_s = time.time() - start + t_offset
            lux = self.get_lux(nid)
            duty = self.get_duty(nid)
            data.append({
                'time_s': round(t_s, 3),
                'node_id': nid,
                'lux': lux,
                'duty': duty,
                'ref': ref,
            })
        return data


def test_equal_costs(ctrl):
    """Test all 4 algorithms with equal costs [1,1,1], ref=20."""
    print("\n" + "="*60)
    print("TEST 1: Algorithm Comparison (Equal Costs [1,1,1])")
    print("="*60)

    algs = [('PI Only', 0), ('Consensus', 1), ('ADMM', 2), ('Dual Decomp', 3)]
    results = []

    for alg_name, alg_id in algs:
        print(f"\n  [{alg_name}] Setting up...")
        ctrl.set_algorithm(alg_id)
        for n in NODE_IDS:
            ctrl.set_ref(n, 20)
            ctrl.set_feedback(n, True)
            ctrl.set_cost(n, 1.0)

        ctrl.reset_metrics()
        settle = SETTLE_TIME
        print(f"  Waiting {settle}s for convergence + metric accumulation...")
        time.sleep(settle)

        for n in NODE_IDS:
            m = ctrl.get_metrics(n)
            lux = ctrl.get_lux(n)
            duty = ctrl.get_duty(n)
            results.append({
                'alg': alg_name, 'node': n, 'lux': lux,
                'duty': duty, 'ref': 20.0,
                'energy': m['energy'], 'vis': m['visibility'], 'flicker': m['flicker'],
            })
            print(f"    Node {n}: lux={lux:.1f} duty={duty:.4f} E={m['energy']:.2f} V={m['visibility']:.4f} F={m['flicker']:.6f}")

    path = os.path.join(RESULTS_DIR, 'alg_comparison_equal.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['alg', 'node', 'lux', 'duty', 'ref', 'energy', 'vis', 'flicker'])
        w.writeheader()
        w.writerows(results)
    print(f"\n  Saved {path}")

    # Plot
    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    alg_names = [a[0] for a in algs]
    x = np.arange(len(NODE_IDS))
    width = 0.2

    for idx, metric in enumerate(['energy', 'vis', 'flicker']):
        for ai, alg in enumerate(alg_names):
            vals = [r[metric] for r in results if r['alg'] == alg]
            axes[idx].bar(x + ai * width, vals, width, label=alg)
        axes[idx].set_ylabel(metric.replace('vis', 'Visibility Error').replace('energy', 'Energy [J]').replace('flicker', 'Flicker'))
        axes[idx].set_xticks(x + width * 1.5)
        axes[idx].set_xticklabels([f'N{n}' for n in NODE_IDS])
        axes[idx].legend(fontsize=7)
        axes[idx].grid(True, alpha=0.3)

    plt.suptitle('Algorithm Comparison: Equal Costs [1,1,1], ref=20 lux')
    plt.tight_layout()
    out = os.path.join(RESULTS_DIR, 'alg_comparison_equal.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"  Plot: {out}")
    return results


def test_unequal_costs(ctrl):
    """Test distributed algorithms with costs [1,2,3]."""
    print("\n" + "="*60)
    print("TEST 2: Unequal Costs [1,2,3]")
    print("="*60)

    algs = [('Consensus', 1), ('ADMM', 2), ('Dual Decomp', 3)]
    results = []

    for alg_name, alg_id in algs:
        print(f"\n  [{alg_name}] Setting costs [1,2,3]...")
        ctrl.set_algorithm(alg_id)
        for n in NODE_IDS:
            ctrl.set_cost(n, n)  # costs: 1, 2, 3
            ctrl.set_ref(n, 20)
            ctrl.set_feedback(n, True)

        ctrl.reset_metrics()
        print(f"  Waiting {SETTLE_TIME}s for convergence...")
        time.sleep(SETTLE_TIME)

        for n in NODE_IDS:
            lux = ctrl.get_lux(n)
            duty = ctrl.get_duty(n)
            m = ctrl.get_metrics(n)
            results.append({
                'alg': alg_name, 'node': n, 'cost': n,
                'lux': lux, 'duty': duty,
                'energy': m['energy'],
            })
            print(f"    Node {n} (cost={n}): lux={lux:.1f} duty={duty:.4f} E={m['energy']:.2f}")

    path = os.path.join(RESULTS_DIR, 'alg_comparison_unequal.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['alg', 'node', 'cost', 'lux', 'duty', 'energy'])
        w.writeheader()
        w.writerows(results)
    print(f"\n  Saved {path}")

    # Plot duty allocation
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))
    alg_names = [a[0] for a in algs]
    x = np.arange(len(NODE_IDS))
    width = 0.25

    for ai, alg in enumerate(alg_names):
        duties = [r['duty'] for r in results if r['alg'] == alg]
        ax1.bar(x + ai * width, duties, width, label=alg)
    ax1.set_ylabel('Duty Cycle')
    ax1.set_xticks(x + width)
    ax1.set_xticklabels([f'N{n}\n(c={n})' for n in NODE_IDS])
    ax1.legend(fontsize=8)
    ax1.set_title('Duty Allocation')
    ax1.grid(True, alpha=0.3)

    for ai, alg in enumerate(alg_names):
        energies = [r['energy'] for r in results if r['alg'] == alg]
        ax2.bar(x + ai * width, energies, width, label=alg)
    ax2.set_ylabel('Energy [J]')
    ax2.set_xticks(x + width)
    ax2.set_xticklabels([f'N{n}\n(c={n})' for n in NODE_IDS])
    ax2.legend(fontsize=8)
    ax2.set_title('Energy Consumption')
    ax2.grid(True, alpha=0.3)

    plt.suptitle('Unequal Costs [1,2,3]: Duty & Energy Allocation')
    plt.tight_layout()
    out = os.path.join(RESULTS_DIR, 'duty_unequal_costs.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"  Plot: {out}")
    return results


def test_occupancy_change(ctrl):
    """Test occupancy change LOW -> HIGH -> LOW on node 1. Fast single-node polling."""
    print("\n" + "="*60)
    print("TEST 3: Occupancy Change (Node 1: LOW -> HIGH -> LOW)")
    print("="*60)

    ctrl.set_algorithm(0)
    for n in NODE_IDS:
        ctrl.set_occupancy(n, 'l')
        ctrl.set_feedback(n, True)
    time.sleep(5)

    # Get LOW ref
    low_ref = ctrl.get_ref(1)
    print(f"  LOW ref = {low_ref}")

    print("  Recording LOW state (5s, node 1)...")
    data = ctrl.poll_single(1, 5, t_offset=0, ref_override=low_ref)
    t1 = data[-1]['time_s'] + 0.1 if data else 5

    print("  Setting node 1 to HIGH...")
    ctrl.set_occupancy(1, 'h')
    time.sleep(0.2)
    high_ref = ctrl.get_ref(1)
    print(f"  HIGH ref = {high_ref}")
    data += ctrl.poll_single(1, 10, t_offset=t1, ref_override=high_ref)
    t2 = data[-1]['time_s'] + 0.1 if data else t1 + 10

    print("  Setting node 1 back to LOW...")
    ctrl.set_occupancy(1, 'l')
    time.sleep(0.2)
    data += ctrl.poll_single(1, 8, t_offset=t2, ref_override=low_ref)

    path = os.path.join(RESULTS_DIR, 'occupancy_change.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['time_s', 'node_id', 'lux', 'duty', 'ref'])
        w.writeheader()
        w.writerows(data)
    print(f"  Saved {path}")

    # Plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    ts = [d['time_s'] for d in data]
    lux = [d['lux'] for d in data]
    duty = [d['duty'] for d in data]
    ref = [d['ref'] for d in data]

    ax1.plot(ts, lux, 'tab:blue', label='Node 1 lux', linewidth=1.2)
    ax1.plot(ts, ref, 'r--', alpha=0.6, label='Reference')
    ax1.axvline(x=t1, color='red', linestyle=':', alpha=0.5, label='HIGH')
    ax1.axvline(x=t2, color='green', linestyle=':', alpha=0.5, label='LOW')
    ax1.set_ylabel('Illuminance [lux]')
    ax1.set_title('Occupancy Change: Node 1 LOW → HIGH → LOW')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    ax2.plot(ts, duty, 'tab:blue', label='Node 1 duty', linewidth=1.2)
    ax2.axvline(x=t1, color='red', linestyle=':', alpha=0.5)
    ax2.axvline(x=t2, color='green', linestyle=':', alpha=0.5)
    ax2.set_ylabel('Duty Cycle')
    ax2.set_xlabel('Time [s]')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(RESULTS_DIR, 'occupancy_change.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"  Plot: {out}")


def test_step_response(ctrl):
    """Step response with feedforward: ref 5 -> 25 on node 1. Fast single-node polling."""
    print("\n" + "="*60)
    print("TEST 4: Step Response (Node 1: ref 5 -> 25)")
    print("="*60)

    ctrl.set_algorithm(0)
    for n in NODE_IDS:
        ctrl.set_ref(n, 5)
        ctrl.set_feedback(n, True)
    time.sleep(5)

    print("  Recording baseline ref=5 (5s, node 1 only)...")
    data = ctrl.poll_single(1, 5, t_offset=0, ref_override=5.0)
    t_step = data[-1]['time_s'] + 0.1 if data else 5

    print("  Step: node 1 ref -> 25...")
    ctrl.set_ref(1, 25)
    data += ctrl.poll_single(1, 10, t_offset=t_step, ref_override=25.0)

    path = os.path.join(RESULTS_DIR, 'step_response_ff.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['time_s', 'node_id', 'lux', 'duty', 'ref'])
        w.writeheader()
        w.writerows(data)
    print(f"  Saved {path}")

    # Plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    ts = [d['time_s'] for d in data]
    lux = [d['lux'] for d in data]
    duty = [d['duty'] for d in data]
    ref = [d['ref'] for d in data]

    ax1.plot(ts, lux, 'tab:blue', label='Node 1 lux', linewidth=1.2)
    ax1.plot(ts, ref, 'r--', alpha=0.6, label='Reference')
    ax1.axvline(x=t_step, color='gray', linestyle=':', alpha=0.5)
    ax1.set_ylabel('Illuminance [lux]')
    ax1.set_title('Step Response: Node 1 ref 5 → 25 (PI + Feedforward)')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    ax2.plot(ts, duty, 'tab:blue', label='Node 1 duty', linewidth=1.2)
    ax2.axvline(x=t_step, color='gray', linestyle=':', alpha=0.5)
    ax2.set_ylabel('Duty Cycle')
    ax2.set_xlabel('Time [s]')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(RESULTS_DIR, 'step_response_ff.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"  Plot: {out}")


def main():
    print(f"SCDTR Comprehensive Test Suite - {datetime.now().strftime('%Y-%m-%d %H:%M')}")
    print("="*60)

    ctrl = Controller()
    ctrl.connect()

    if len(ctrl.ports) < 3:
        print("ERROR: Not all 3 Picos connected.")
        ctrl.close()
        return

    test_equal_costs(ctrl)
    test_unequal_costs(ctrl)
    test_step_response(ctrl)
    test_occupancy_change(ctrl)

    # Cleanup
    ctrl.set_algorithm(0)
    for n in NODE_IDS:
        cmd(ctrl.ports[n], f'f {n} 0', delay=0.1)
        cmd(ctrl.ports[n], f'u {n} 0', delay=0.1)

    ctrl.close()
    print(f"\n{'='*60}")
    print(f"All tests complete. Results in {RESULTS_DIR}")
    print(f"{'='*60}")


if __name__ == '__main__':
    main()
