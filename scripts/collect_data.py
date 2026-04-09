#!/usr/bin/env python3
"""
SCDTR Data Collection Script
Runs test scenarios on 3-node distributed illumination system,
collects streaming data, and generates plots for the report.
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


class SystemController:
    def __init__(self):
        self.ports = {}
        os.makedirs(RESULTS_DIR, exist_ok=True)

    def connect_all(self):
        for nid, port in PORTS.items():
            try:
                s = serial.Serial(port, BAUD, timeout=0.1)
                time.sleep(0.5)
                s.read(4096)
                self.ports[nid] = s
                print(f"  Connected to Pico {nid} on {port}")
            except Exception as e:
                print(f"  FAILED Pico {nid}: {e}")

    def close_all(self):
        for s in self.ports.values():
            s.close()
        self.ports.clear()

    def cmd(self, nid, command, delay=0.1):
        s = self.ports[nid]
        s.read(4096)
        s.write(f'{command}\n'.encode())
        time.sleep(delay)
        # Read with retries
        result = b''
        for _ in range(5):
            chunk = s.read(4096)
            if chunk:
                result += chunk
                break
            time.sleep(0.02)
        return result.decode('utf-8', errors='replace').strip()

    def configure_nodes(self):
        print("Configuring nodes...")
        for nid in NODE_IDS:
            # Try configuring — if already configured, this is harmless
            r = self.cmd(nid, str(nid), delay=0.5)
            if 'ID configurado' in r:
                self.cmd(nid, '3', delay=0.5)
            # Verify by querying lux
            r = self.cmd(nid, f'g y {nid}', delay=0.5)
            print(f"  Pico {nid}: {r}")
        time.sleep(1)
        print("  All nodes configured.")

    def calibrate(self):
        print("Running calibration...")
        self.cmd(1, 'c 0', delay=2)
        time.sleep(25)
        for n in NODE_IDS:
            rpt = self.cmd(n, 'rpt', delay=1)
            print(f"  Pico {n}: {rpt}")

    def set_ref(self, nid, ref):
        self.cmd(nid, f'r {nid} {ref}')

    def set_feedback(self, nid, on):
        self.cmd(nid, f'f {nid} {1 if on else 0}')

    def set_occupancy(self, nid, state):
        self.cmd(nid, f'o {nid} {state}')

    def set_algorithm(self, alg_id):
        for n in NODE_IDS:
            self.cmd(n, f'A {alg_id}')

    def set_cost(self, nid, cost):
        self.cmd(nid, f'C {nid} {cost}')

    def get_lux(self, nid):
        r = self.cmd(nid, f'g y {nid}', delay=0.05)
        try:
            return float(r.split()[-1])
        except:
            return 0.0

    def get_duty(self, nid):
        r = self.cmd(nid, f'g u {nid}', delay=0.05)
        try:
            return int(r.split()[-1])
        except:
            return 0

    def get_metrics(self, nid):
        e = self.cmd(nid, f'g E {nid}')
        v = self.cmd(nid, f'g V {nid}')
        f = self.cmd(nid, f'g F {nid}')
        try:
            return {
                'energy': float(e.split()[-1]),
                'visibility': float(v.split()[-1]),
                'flicker': float(f.split()[-1]),
            }
        except:
            return {'energy': 0, 'visibility': 0, 'flicker': 0}

    def stream_all(self, duration_s, t_offset_ms=0):
        """Poll lux and duty from all nodes using direct serial to each Pico."""
        data = []
        cached_refs = {}
        for n in NODE_IDS:
            ref_r = self.cmd(n, f'g r {n}', delay=0.1)
            try:
                cached_refs[n] = float(ref_r.split()[-1])
            except:
                cached_refs[n] = 0.0

        start = time.time()
        while time.time() - start < duration_s:
            for n in NODE_IDS:
                t_ms = int((time.time() - start) * 1000) + t_offset_ms
                lux = self.get_lux(n)
                duty = self.get_duty(n)
                data.append({
                    'time_ms': t_ms,
                    'node_id': n,
                    'lux': lux,
                    'duty': duty,
                    'ref': cached_refs.get(n, 0.0),
                })
        # Update refs at end
        for n in NODE_IDS:
            ref_r = self.cmd(n, f'g r {n}', delay=0.1)
            try:
                ref = float(ref_r.split()[-1])
            except:
                ref = cached_refs.get(n, 0.0)
            for d in data:
                if d['node_id'] == n:
                    d['ref'] = ref
        return data

    def save_csv(self, data, filename):
        path = os.path.join(RESULTS_DIR, filename)
        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['time_ms', 'node_id', 'lux', 'duty', 'ref'])
            writer.writeheader()
            writer.writerows(data)
        print(f"  Saved {path}")
        return path


def plot_time_series(csv_path, title, output_name):
    """Plot lux and duty vs time for all nodes."""
    data = {n: {'t': [], 'lux': [], 'duty': [], 'ref': []} for n in NODE_IDS}
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            n = int(row['node_id'])
            data[n]['t'].append(float(row['time_ms']) / 1000.0)
            data[n]['lux'].append(float(row['lux']))
            data[n]['duty'].append(int(row['duty']) / 4095.0)
            data[n]['ref'].append(float(row['ref']))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    colors = {1: 'tab:blue', 2: 'tab:orange', 3: 'tab:green'}

    for n in NODE_IDS:
        ax1.plot(data[n]['t'], data[n]['lux'], color=colors[n], label=f'Node {n} lux')
        ax1.plot(data[n]['t'], data[n]['ref'], '--', color=colors[n], alpha=0.5, label=f'Node {n} ref')
    ax1.set_ylabel('Illuminance [lux]')
    ax1.legend(fontsize=8)
    ax1.set_title(title)
    ax1.grid(True, alpha=0.3)

    for n in NODE_IDS:
        ax2.plot(data[n]['t'], data[n]['duty'], color=colors[n], label=f'Node {n}')
    ax2.set_ylabel('Duty cycle')
    ax2.set_xlabel('Time [s]')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(RESULTS_DIR, output_name)
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"  Plot saved: {out}")


def run_step_response(ctrl):
    print("\n=== Test: Step Response ===")
    for n in NODE_IDS:
        ctrl.set_ref(n, 5)
        ctrl.set_feedback(n, True)
    time.sleep(3)

    print("  Recording baseline (5s)...")
    data = ctrl.stream_all(5, t_offset_ms=0)
    t_step = data[-1]['time_ms'] + 500 if data else 5000

    print("  Step: ref -> 25 on node 1")
    ctrl.set_ref(1, 25)
    data += ctrl.stream_all(10, t_offset_ms=t_step)

    path = ctrl.save_csv(data, 'step_response.csv')
    plot_time_series(path, 'Step Response: Node 1 ref 5 -> 25', 'step_response.png')


def run_occupancy_change(ctrl):
    print("\n=== Test: Occupancy Change ===")
    for n in NODE_IDS:
        ctrl.set_occupancy(n, 'l')
        ctrl.set_feedback(n, True)
    time.sleep(3)

    print("  Recording LOW state (5s)...")
    data = ctrl.stream_all(5, t_offset_ms=0)
    t1 = data[-1]['time_ms'] + 500 if data else 5000

    print("  Changing node 1 to HIGH")
    ctrl.set_occupancy(1, 'h')
    data += ctrl.stream_all(8, t_offset_ms=t1)
    t2 = data[-1]['time_ms'] + 500 if data else t1 + 8000

    print("  Changing node 1 back to LOW")
    ctrl.set_occupancy(1, 'l')
    data += ctrl.stream_all(5, t_offset_ms=t2)

    path = ctrl.save_csv(data, 'occupancy_change.csv')
    plot_time_series(path, 'Occupancy Change: Node 1 LOW -> HIGH -> LOW', 'occupancy_change.png')


def run_algorithm_comparison(ctrl):
    print("\n=== Test: Algorithm Comparison ===")
    results = []

    for alg_name, alg_id in [('PI Only', 0), ('Consensus', 1)]:
        print(f"  Testing {alg_name}...")
        ctrl.set_algorithm(alg_id)
        for n in NODE_IDS:
            ctrl.set_ref(n, 20)
            ctrl.set_feedback(n, True)
            ctrl.set_cost(n, 1.0)

        if alg_id == 1:
            time.sleep(15)  # consensus needs time to converge + gain exchange
        else:
            time.sleep(5)

        for n in NODE_IDS:
            m = ctrl.get_metrics(n)
            lux = ctrl.get_lux(n)
            duty = ctrl.get_duty(n)
            results.append({
                'algorithm': alg_name,
                'node_id': n,
                'lux': lux,
                'duty': duty,
                'energy': m['energy'],
                'visibility': m['visibility'],
                'flicker': m['flicker'],
            })
            print(f"    Node {n}: lux={lux:.1f} duty={duty} E={m['energy']:.2f} V={m['visibility']:.4f} F={m['flicker']:.6f}")

    path = os.path.join(RESULTS_DIR, 'algorithm_comparison.csv')
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['algorithm', 'node_id', 'lux', 'duty', 'energy', 'visibility', 'flicker'])
        writer.writeheader()
        writer.writerows(results)
    print(f"  Saved {path}")

    # Plot comparison
    fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    algs = ['PI Only', 'Consensus']
    x = np.arange(len(NODE_IDS))
    width = 0.35

    for idx, metric in enumerate(['energy', 'visibility', 'flicker']):
        for ai, alg in enumerate(algs):
            vals = [r[metric] for r in results if r['algorithm'] == alg]
            axes[idx].bar(x + ai * width, vals, width, label=alg)
        axes[idx].set_xlabel('Node')
        axes[idx].set_ylabel(metric.capitalize())
        axes[idx].set_xticks(x + width / 2)
        axes[idx].set_xticklabels([f'N{n}' for n in NODE_IDS])
        axes[idx].legend(fontsize=8)
        axes[idx].grid(True, alpha=0.3)

    plt.suptitle('Algorithm Comparison: PI Only vs Consensus')
    plt.tight_layout()
    out = os.path.join(RESULTS_DIR, 'algorithm_comparison.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"  Plot saved: {out}")


def run_cost_comparison(ctrl):
    print("\n=== Test: Cost Comparison ===")
    results = []

    for cost_name, costs in [('Equal [1,1,1]', [1, 1, 1]), ('Unequal [1,2,3]', [1, 2, 3])]:
        print(f"  Testing costs {cost_name}...")
        ctrl.set_algorithm(1)  # consensus
        for n in NODE_IDS:
            ctrl.set_cost(n, costs[n - 1])
            ctrl.set_ref(n, 20)
            ctrl.set_feedback(n, True)
        time.sleep(15)

        for n in NODE_IDS:
            m = ctrl.get_metrics(n)
            lux = ctrl.get_lux(n)
            duty = ctrl.get_duty(n)
            results.append({
                'costs': cost_name,
                'node_id': n,
                'lux': lux,
                'duty': duty / 4095.0,
                'energy': m['energy'],
            })
            print(f"    Node {n}: cost={costs[n-1]} lux={lux:.1f} duty={duty/4095:.3f} E={m['energy']:.2f}")

    path = os.path.join(RESULTS_DIR, 'cost_comparison.csv')
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['costs', 'node_id', 'lux', 'duty', 'energy'])
        writer.writeheader()
        writer.writerows(results)
    print(f"  Saved {path}")

    # Plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
    x = np.arange(len(NODE_IDS))
    width = 0.35

    for ci, (cname, _) in enumerate([('Equal [1,1,1]', None), ('Unequal [1,2,3]', None)]):
        duties = [r['duty'] for r in results if r['costs'] == cname]
        ax1.bar(x + ci * width, duties, width, label=cname)
    ax1.set_ylabel('Duty Cycle')
    ax1.set_xticks(x + width / 2)
    ax1.set_xticklabels([f'Node {n}' for n in NODE_IDS])
    ax1.legend(fontsize=8)
    ax1.set_title('Duty Allocation')
    ax1.grid(True, alpha=0.3)

    for ci, (cname, _) in enumerate([('Equal [1,1,1]', None), ('Unequal [1,2,3]', None)]):
        energies = [r['energy'] for r in results if r['costs'] == cname]
        ax2.bar(x + ci * width, energies, width, label=cname)
    ax2.set_ylabel('Energy [J]')
    ax2.set_xticks(x + width / 2)
    ax2.set_xticklabels([f'Node {n}' for n in NODE_IDS])
    ax2.legend(fontsize=8)
    ax2.set_title('Energy Consumption')
    ax2.grid(True, alpha=0.3)

    plt.suptitle('Cost Comparison: Equal vs Unequal Costs')
    plt.tight_layout()
    out = os.path.join(RESULTS_DIR, 'cost_comparison.png')
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"  Plot saved: {out}")


def main():
    print(f"SCDTR Data Collection - {datetime.now().strftime('%Y-%m-%d %H:%M')}")
    print("=" * 50)

    ctrl = SystemController()
    ctrl.connect_all()

    if len(ctrl.ports) < 3:
        print("ERROR: Not all 3 Picos connected. Aborting.")
        ctrl.close_all()
        return

    ctrl.configure_nodes()
    ctrl.calibrate()

    # Run all tests
    run_step_response(ctrl)
    run_occupancy_change(ctrl)
    run_algorithm_comparison(ctrl)
    run_cost_comparison(ctrl)

    # Final cleanup
    for n in NODE_IDS:
        ctrl.cmd(n, f'f {n} 0')
        ctrl.cmd(n, f'u {n} 0')
        ctrl.cmd(n, 'A 0')

    ctrl.close_all()
    print(f"\n=== All tests complete. Results in {RESULTS_DIR} ===")


if __name__ == '__main__':
    main()
