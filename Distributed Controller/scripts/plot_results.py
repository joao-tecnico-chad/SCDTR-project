#!/usr/bin/env python3
"""
Plotting utility for SCDTR Phase 2 CSV results.

Generates publication-quality figures from test CSV data using matplotlib.

Functions:
  plot_step_response     - time vs lux/duty
  plot_convergence       - iterations vs cost/error
  plot_algorithm_comparison - bar chart of metrics per algorithm
  plot_disturbance       - time series with disturbance window
  plot_occupation_transition - time series with transition markers

CLI usage:
  python plot_results.py <csv_file> <plot_type> [output.png]

Plot types: step_response, convergence, algorithm_comparison,
            disturbance, occupation_transition
"""

import argparse
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def _read_csv(csv_file):
    """Read a CSV file and return a list of dicts."""
    with open(csv_file, "r") as f:
        reader = csv.DictReader(f)
        return list(reader)


def _safe_float(val):
    """Convert to float, returning None for empty/invalid values."""
    if val is None or val == "" or val == "None" or val == "N/A":
        return None
    try:
        return float(val)
    except (ValueError, TypeError):
        return None


def plot_step_response(csv_file, output_png):
    """
    Plot time vs lux and duty from streaming data.
    Expects CSV columns: time_ms, lux (or value), and optionally duty.
    Also supports multi-node format: node_id, time_ms, lux.
    """
    rows = _read_csv(csv_file)
    if not rows:
        print(f"No data in {csv_file}")
        return

    fig, ax1 = plt.subplots(figsize=(10, 5))

    # Detect format
    if "node_id" in rows[0]:
        # Multi-node format
        nodes = sorted(set(r.get("node_id", "") for r in rows if r.get("node_id")))
        for node in nodes:
            node_rows = [r for r in rows if r.get("node_id") == node]
            times = [_safe_float(r.get("time_ms")) for r in node_rows]
            lux = [_safe_float(r.get("lux") or r.get("value")) for r in node_rows]
            # Filter out None values
            pairs = [(t, l) for t, l in zip(times, lux) if t is not None and l is not None]
            if pairs:
                ts, ls = zip(*pairs)
                ts_s = [t / 1000.0 for t in ts]
                ax1.plot(ts_s, ls, label=f"Node {node}", linewidth=0.8)
    else:
        times = [_safe_float(r.get("time_ms")) for r in rows]
        lux = [_safe_float(r.get("lux") or r.get("value")) for r in rows]
        pairs = [(t, l) for t, l in zip(times, lux) if t is not None and l is not None]
        if pairs:
            ts, ls = zip(*pairs)
            ts_s = [t / 1000.0 for t in ts]
            ax1.plot(ts_s, ls, "b-", label="Lux", linewidth=0.8)

    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Lux")
    ax1.set_title("Step Response")
    ax1.legend(loc="best")
    ax1.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_png, dpi=150)
    plt.close(fig)
    print(f"Saved {output_png}")


def plot_convergence(csv_file, output_png):
    """
    Plot iterations vs cost or error.
    Expects CSV columns: iteration, cost (and/or error).
    """
    rows = _read_csv(csv_file)
    if not rows:
        print(f"No data in {csv_file}")
        return

    fig, ax = plt.subplots(figsize=(10, 5))

    iters = [_safe_float(r.get("iteration")) for r in rows]
    costs = [_safe_float(r.get("cost")) for r in rows]
    errors = [_safe_float(r.get("error")) for r in rows]

    if any(c is not None for c in costs):
        pairs = [(i, c) for i, c in zip(iters, costs) if i is not None and c is not None]
        if pairs:
            xs, ys = zip(*pairs)
            ax.plot(xs, ys, "b-o", label="Cost", markersize=2)

    if any(e is not None for e in errors):
        ax2 = ax.twinx()
        pairs = [(i, e) for i, e in zip(iters, errors) if i is not None and e is not None]
        if pairs:
            xs, ys = zip(*pairs)
            ax2.plot(xs, ys, "r-s", label="Error", markersize=2)
            ax2.set_ylabel("Error", color="r")
            ax2.legend(loc="upper right")

    ax.set_xlabel("Iteration")
    ax.set_ylabel("Cost")
    ax.set_title("Algorithm Convergence")
    ax.legend(loc="upper left")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_png, dpi=150)
    plt.close(fig)
    print(f"Saved {output_png}")


def plot_algorithm_comparison(csv_file, output_png):
    """
    Bar chart comparing algorithms on energy, visibility, and flicker.
    Expects CSV columns: algorithm, scenario, node_id, energy, visibility, flicker.
    Aggregates by algorithm (mean across nodes and scenarios).
    """
    rows = _read_csv(csv_file)
    if not rows:
        print(f"No data in {csv_file}")
        return

    # Aggregate per algorithm
    alg_data = {}
    for r in rows:
        alg = r.get("algorithm", "unknown")
        if alg not in alg_data:
            alg_data[alg] = {"energy": [], "visibility": [], "flicker": []}
        for metric in ("energy", "visibility", "flicker"):
            val = _safe_float(r.get(metric))
            if val is not None:
                alg_data[alg][metric].append(val)

    algorithms = sorted(alg_data.keys())
    metrics = ["energy", "visibility", "flicker"]
    n_metrics = len(metrics)

    fig, axes = plt.subplots(1, n_metrics, figsize=(4 * n_metrics, 5))
    if n_metrics == 1:
        axes = [axes]

    colors = {"consensus": "#2196F3", "dual_decomp": "#FF9800", "admm": "#4CAF50"}

    for ax, metric in zip(axes, metrics):
        means = []
        labels = []
        bar_colors = []
        for alg in algorithms:
            vals = alg_data[alg][metric]
            mean_val = sum(vals) / len(vals) if vals else 0
            means.append(mean_val)
            labels.append(alg)
            bar_colors.append(colors.get(alg, "#999999"))

        bars = ax.bar(labels, means, color=bar_colors, edgecolor="black", linewidth=0.5)
        ax.set_title(metric.capitalize())
        ax.set_ylabel(metric.capitalize())
        for bar, val in zip(bars, means):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                    f"{val:.3f}", ha="center", va="bottom", fontsize=8)
        ax.grid(True, axis="y", alpha=0.3)

    fig.suptitle("Algorithm Comparison", fontsize=14, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    fig.savefig(output_png, dpi=150)
    plt.close(fig)
    print(f"Saved {output_png}")


def plot_disturbance(csv_file, output_png):
    """
    Plot time series with disturbance window highlighted.
    Expects CSV columns: node_id, time_ms (or wall_elapsed_s), lux.
    The disturbance window is drawn between 10s and 15s by default.
    """
    rows = _read_csv(csv_file)
    if not rows:
        print(f"No data in {csv_file}")
        return

    # Filter to stream data rows (those with wall_elapsed_s)
    stream_rows = [r for r in rows if r.get("wall_elapsed_s") is not None and r.get("wall_elapsed_s") != ""]
    if not stream_rows:
        # Fallback: try time_ms based
        stream_rows = [r for r in rows if r.get("time_ms") is not None and r.get("time_ms") != ""]

    fig, ax = plt.subplots(figsize=(12, 5))

    nodes = sorted(set(r.get("node_id", "") for r in stream_rows if r.get("node_id")))
    for node in nodes:
        node_rows = [r for r in stream_rows if r.get("node_id") == node]
        if "wall_elapsed_s" in node_rows[0] and node_rows[0]["wall_elapsed_s"]:
            times = [_safe_float(r["wall_elapsed_s"]) for r in node_rows]
        else:
            times = [_safe_float(r.get("time_ms")) for r in node_rows]
            times = [t / 1000.0 if t is not None else None for t in times]
        lux = [_safe_float(r.get("lux")) for r in node_rows]

        pairs = [(t, l) for t, l in zip(times, lux) if t is not None and l is not None]
        if pairs:
            ts, ls = zip(*pairs)
            ax.plot(ts, ls, label=f"Node {node}", linewidth=0.8)

    # Draw disturbance window
    ax.axvspan(10, 15, alpha=0.15, color="red", label="Disturbance window")
    ax.axvline(10, color="red", linestyle="--", linewidth=0.8, alpha=0.6)
    ax.axvline(15, color="red", linestyle="--", linewidth=0.8, alpha=0.6)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Lux")
    ax.set_title("Disturbance Rejection")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_png, dpi=150)
    plt.close(fig)
    print(f"Saved {output_png}")


def plot_occupation_transition(csv_file, output_png):
    """
    Plot time series with occupation transition markers.
    Expects CSV columns: phase, node_id, time_ms, lux, wall_elapsed_s.
    Draws vertical lines at transition points.
    """
    rows = _read_csv(csv_file)
    if not rows:
        print(f"No data in {csv_file}")
        return

    # Filter to stream data rows (those with phase and wall_elapsed_s)
    stream_rows = [r for r in rows
                   if r.get("phase") in ("A", "B")
                   and r.get("wall_elapsed_s") is not None
                   and r.get("wall_elapsed_s") != ""]

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=False)

    for ax, phase, title in zip(axes, ("A", "B"),
                                  ("Unoccupied -> Occupied", "Occupied -> Unoccupied")):
        phase_rows = [r for r in stream_rows if r.get("phase") == phase]
        nodes = sorted(set(r.get("node_id", "") for r in phase_rows if r.get("node_id")))

        for node in nodes:
            node_rows = [r for r in phase_rows if r.get("node_id") == node]
            times = [_safe_float(r.get("wall_elapsed_s")) for r in node_rows]
            lux = [_safe_float(r.get("lux")) for r in node_rows]

            pairs = [(t, l) for t, l in zip(times, lux) if t is not None and l is not None]
            if pairs:
                ts, ls = zip(*pairs)
                ax.plot(ts, ls, label=f"Node {node}", linewidth=0.8)

        # Transition marker at ~1s (when occupancy change is sent)
        ax.axvline(1.0, color="red", linestyle="--", linewidth=1.0,
                   label="Occupancy change", alpha=0.7)

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Lux")
        ax.set_title(f"Phase {phase}: {title}")
        ax.legend(loc="best")
        ax.grid(True, alpha=0.3)

    fig.suptitle("Occupation Transitions", fontsize=14, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(output_png, dpi=150)
    plt.close(fig)
    print(f"Saved {output_png}")


PLOT_TYPES = {
    "step_response": plot_step_response,
    "convergence": plot_convergence,
    "algorithm_comparison": plot_algorithm_comparison,
    "disturbance": plot_disturbance,
    "occupation_transition": plot_occupation_transition,
}


def main():
    parser = argparse.ArgumentParser(
        description="Generate plots from SCDTR Phase 2 CSV data",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Plot types: " + ", ".join(sorted(PLOT_TYPES.keys())),
    )
    parser.add_argument("csv_file", help="Input CSV file")
    parser.add_argument("plot_type", choices=sorted(PLOT_TYPES.keys()),
                        help="Type of plot to generate")
    parser.add_argument("output", nargs="?", default=None,
                        help="Output PNG file (default: <csv_basename>_<plot_type>.png)")
    args = parser.parse_args()

    if not os.path.isfile(args.csv_file):
        print(f"ERROR: File not found: {args.csv_file}")
        return 1

    output = args.output
    if output is None:
        base = os.path.splitext(os.path.basename(args.csv_file))[0]
        output = f"{base}_{args.plot_type}.png"

    plot_fn = PLOT_TYPES[args.plot_type]
    plot_fn(args.csv_file, output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
