#!/usr/bin/env python3
"""
Compare the tree planner, the product automaton and the random sampler across every sweep.

Reads the per-sweep combined_long.csv files (one row per run per planner), merges them into
one long CSV, and draws four figures:

  1. cost_quality_scatter.png  - computation time against makespan ratio, the trade-off itself
  2. scaling_curves.png        - how each planner's runtime grows with the swept variable
  3. gap_distribution.png      - spread of makespan ratio per planner, per sweep
  4. coverage.png              - fraction of instances each planner actually solved

Run from the organizedtests directory:  python3 plot_model_comparison.py
"""

import csv
import glob
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT_DIR = "comparison"
MERGED_CSV = os.path.join(OUT_DIR, "all_configurations_long.csv")

METHODS = ["tree", "product", "sampler"]
LABELS = {"tree": "Tree planner", "product": "Product automaton", "sampler": "Random sampler"}
COLORS = {"tree": "#1f77b4", "product": "#d62728", "sampler": "#2ca02c"}
MARKERS = {"tree": "o", "product": "s", "sampler": "^"}

NUMERIC = ["computation_time_seconds", "makespan", "makespan_ratio", "memory_MB",
           "num_robots", "num_automaton_states", "num_automaton_edges", "num_ts_regions",
           "avg_capabilities_per_robot", "capability_homogeneity"]


def load_rows():
    """Merge every sweep's combined_long.csv into one list of dicts."""
    paths = sorted(glob.glob(os.path.join("*", "data", "combined_long.csv")))
    if not paths:
        raise SystemExit("No combined_long.csv files found. Run the sweeps first.")

    rows = []
    for path in paths:
        with open(path, newline="") as f:
            source_rows = list(csv.DictReader(f))

        for row in source_rows:
            for key in NUMERIC:
                value = row.get(key, "")
                row[key] = float(value) if value not in ("", None) else None
            row["solved"] = row.get("solved", "0") == "1"
            rows.append(row)
        print("  read %-60s %s" % (path, "ok"))
    return rows


def write_merged(rows):
    os.makedirs(OUT_DIR, exist_ok=True)
    if not rows:
        return
    with open(MERGED_CSV, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        for row in rows:
            out = dict(row)
            out["solved"] = 1 if row["solved"] else 0
            writer.writerow(out)
    print("\nMerged %d rows -> %s" % (len(rows), MERGED_CSV))


def by_method(rows, predicate=None):
    groups = {m: [] for m in METHODS}
    for row in rows:
        if row["method"] in groups and (predicate is None or predicate(row)):
            groups[row["method"]].append(row)
    return groups


# --------------------------------------------------------------------------- 1
def plot_cost_quality(rows):
    """Computation time against makespan ratio. Log time because the product spans decades."""
    groups = by_method(rows, lambda r: r["solved"] and r["makespan_ratio"] and
                       r["computation_time_seconds"] and r["computation_time_seconds"] > 0)

    fig, ax = plt.subplots(figsize=(9, 6))
    for method in ("tree", "product"):
        pts = groups[method]
        if not pts:
            continue
        ax.scatter([p["computation_time_seconds"] for p in pts],
                   [p["makespan_ratio"] for p in pts],
                   label="%s (n=%d)" % (LABELS[method], len(pts)),
                   c=COLORS[method], marker=MARKERS[method],
                   alpha=0.45, s=34, edgecolors="none")

    ax.axhline(1.0, color="grey", linestyle="--", linewidth=1)
    ax.annotate("best result on each instance", xy=(0.02, 1.0), xycoords=("axes fraction", "data"),
                fontsize=8, color="grey", va="bottom")
    ax.set_xscale("log")
    ax.set_xlabel("Computation time (s, log scale)")
    ax.set_ylabel("Makespan relative to best on that instance")
    ax.set_title("Cost against solution quality\nlower and further left is better")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, alpha=0.3, which="both")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "cost_quality_scatter.png"), dpi=150)
    plt.close(fig)


# --------------------------------------------------------------------------- 2
def plot_scaling(rows):
    """Runtime against each sweep's own independent variable, one panel per sweep."""
    sweeps = sorted({r["sweep"] for r in rows})
    cols = 2
    panels = (len(sweeps) + cols - 1) // cols
    fig, axes = plt.subplots(panels, cols, figsize=(13, 4 * panels), squeeze=False)

    for n, sweep in enumerate(sweeps):
        ax = axes[n // cols][n % cols]
        sweep_rows = [r for r in rows if r["sweep"] == sweep]
        var = sweep_rows[0]["independent_variable"] if sweep_rows else ""

        for method in ("tree", "product"):
            # mean runtime at each value of the swept variable, solved instances only
            buckets = {}
            for r in sweep_rows:
                if r["method"] != method or not r["solved"]:
                    continue
                try:
                    x = float(r["independent_value"])
                except (TypeError, ValueError):
                    continue
                buckets.setdefault(x, []).append(r["computation_time_seconds"])
            if not buckets:
                continue
            xs = sorted(buckets)
            ys = [sum(buckets[x]) / len(buckets[x]) for x in xs]
            ax.plot(xs, ys, marker=MARKERS[method], color=COLORS[method],
                    label=LABELS[method], linewidth=1.8, markersize=5)

        ax.set_yscale("log")
        ax.set_title(sweep)
        ax.set_xlabel(var)
        ax.set_ylabel("Computation time (s, log)")
        ax.grid(True, alpha=0.3, which="both")
        ax.legend(fontsize=8)

    for n in range(len(sweeps), panels * cols):
        axes[n // cols][n % cols].axis("off")

    fig.suptitle("Runtime scaling. A curve that stops early is a planner that could not continue",
                 fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(os.path.join(OUT_DIR, "scaling_curves.png"), dpi=150)
    plt.close(fig)


# --------------------------------------------------------------------------- 3
def plot_gap_distribution(rows):
    """Box plots of makespan ratio. The sampler is stochastic, so its spread is the finding."""
    sweeps = sorted({r["sweep"] for r in rows})
    fig, ax = plt.subplots(figsize=(max(9, 2.2 * len(sweeps)), 6))

    positions, data, colors, ticks, tick_pos = [], [], [], [], []
    slot = 0
    for sweep in sweeps:
        group_start = slot
        for method in METHODS:
            values = [r["makespan_ratio"] for r in rows
                      if r["sweep"] == sweep and r["method"] == method
                      and r["solved"] and r["makespan_ratio"]]
            if values:
                positions.append(slot)
                data.append(values)
                colors.append(COLORS[method])
            slot += 1
        tick_pos.append((group_start + slot - 1) / 2.0)
        ticks.append(sweep)
        slot += 1  # gap between sweeps

    if data:
        bp = ax.boxplot(data, positions=positions, widths=0.75, patch_artist=True,
                        showfliers=True, flierprops=dict(marker=".", markersize=3, alpha=0.4))
        for patch, color in zip(bp["boxes"], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.55)
        for median in bp["medians"]:
            median.set_color("black")

    ax.axhline(1.0, color="grey", linestyle="--", linewidth=1)
    ax.set_xticks(tick_pos)
    ax.set_xticklabels(ticks, rotation=20, ha="right")
    ax.set_ylabel("Makespan relative to best on that instance")
    ax.set_title("Distribution of the makespan gap", pad=28)
    handles = [plt.Line2D([0], [0], color=COLORS[m], linewidth=8, alpha=0.55, label=LABELS[m])
               for m in METHODS]
    ax.legend(handles=handles, fontsize=9, loc="lower center",
              bbox_to_anchor=(0.5, 1.02), ncol=3, frameon=False)
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "gap_distribution.png"), dpi=150)
    plt.close(fig)


# --------------------------------------------------------------------------- 4
def plot_coverage(rows):
    """Fraction of instances each planner solved. The product's missing data is the result."""
    sweeps = sorted({r["sweep"] for r in rows})
    fig, ax = plt.subplots(figsize=(max(9, 2.0 * len(sweeps)), 5.5))

    width = 0.24
    for n, method in enumerate(METHODS):
        fractions = []
        for sweep in sweeps:
            attempted = [r for r in rows if r["sweep"] == sweep and r["method"] == method]
            solved = [r for r in attempted if r["solved"]]
            fractions.append(100.0 * len(solved) / len(attempted) if attempted else 0.0)
        offsets = [i + (n - 1) * width for i in range(len(sweeps))]
        bars = ax.bar(offsets, fractions, width, label=LABELS[method],
                      color=COLORS[method], alpha=0.85)
        for bar, value in zip(bars, fractions):
            ax.text(bar.get_x() + bar.get_width() / 2, value + 1.5, "%.0f" % value,
                    ha="center", fontsize=7)

    ax.set_xticks(range(len(sweeps)))
    ax.set_xticklabels(sweeps, rotation=20, ha="right")
    ax.set_ylabel("Instances solved (%)")
    ax.set_ylim(0, 108)
    ax.set_title("Coverage: how many instances each planner could handle", pad=28)
    ax.legend(fontsize=9, loc="lower center", bbox_to_anchor=(0.5, 1.02),
              ncol=3, frameon=False)
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "coverage.png"), dpi=150)
    plt.close(fig)


def main():
    print("Reading sweep data...")
    rows = load_rows()
    os.makedirs(OUT_DIR, exist_ok=True)
    write_merged(rows)

    solved_counts = {m: sum(1 for r in rows if r["method"] == m and r["solved"]) for m in METHODS}
    total = len({(r["sweep"], r["run_id"]) for r in rows})
    print("\n%d instances across %d sweeps" % (total, len({r["sweep"] for r in rows})))
    for m in METHODS:
        print("  %-18s solved %d" % (LABELS[m], solved_counts[m]))

    print("\nDrawing figures...")
    plot_cost_quality(rows)
    plot_scaling(rows)
    plot_gap_distribution(rows)
    plot_coverage(rows)
    print("Wrote 4 figures to %s/" % OUT_DIR)


if __name__ == "__main__":
    main()
