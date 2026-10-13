#!/usr/bin/env python3
"""
Runtime against automaton size, for both automaton sweeps, on a squared x axis.

The two sweeps each plot computation time against the number of Buchi states, and both come out
as the same upward sweep that is hard to read anything off: almost every point is crushed into
the first fifth of the axis and the growth is visible only as "it goes up at the end".

Plotting against the square of the state count straightens it. That is not a cosmetic choice:
fitting a power law to the medians gives time ~ states^1.71 for the per-robot sweep and
states^1.61 for the batch sweep, so a quadratic abscissa is close to the true growth. Measured
against the medians, a straight-line fit improves from R^2 = 0.88 on a plain state count to
R^2 = 0.99 on the square, for both sweeps. A straight line on these axes is therefore the claim
that the planner is roughly quadratic in automaton size, and the reader can check the straightness
by eye.

The top axis keeps the untransformed state count, so the squared scale never has to be converted
back mentally.

Writes comparison/automaton_scaling.png. Run from the organizedtests directory:
    python3 plot_automaton_scaling.py
"""

import csv
import glob
import os
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import thesis_style

thesis_style.apply()

OUT_DIR = "comparison"
COLOURS = thesis_style.PALETTE[:4]
MARKERS = ["o", "s", "^", "D"]

# Per-robot sweep: one line per team size. Batch sweep: one per inter-task configuration.
PANELS = [
    {
        "sweep": "automaton_states",
        "xlabel": "Automaton states squared",
        "group": "num_robots",
        "labels": {"3": "3 robots", "6": "6 robots", "15": "15 robots", "45": "45 robots"},
        "order": ["3", "6", "15", "45"],
    },
    {
        "sweep": "automaton_states_batch",
        "xlabel": "Automaton states squared",
        "group": "Batch_Configuration",
        "labels": {"1": "US (unrelated)", "2": "CS (compatible)",
                   "3": "ES (exclusive)", "4": "ES+CS (mixed)"},
        "order": ["1", "2", "3", "4"],
    },
]


def load(sweep):
    rows = []
    for path in sorted(glob.glob(os.path.join(sweep, "data", "*.csv"))):
        if os.path.basename(path) in ("statistics.csv", "combined_long.csv"):
            continue
        with open(path, newline="") as f:
            rows.extend(csv.DictReader(f))
    return rows


def curve(rows, group_column, key):
    """Median runtime per state count for one series, as (states, states^2, milliseconds)."""
    grouped = defaultdict(list)
    for row in rows:
        if str(row.get(group_column, "")).split(".")[0] != key:
            continue
        try:
            states = float(row["num_automaton_states"])
            millis = float(row["total_computation_time_ms"])
        except (KeyError, TypeError, ValueError):
            continue
        if states > 0 and millis > 0:
            grouped[states].append(millis)
    states = np.array(sorted(grouped))
    return states, states ** 2, np.array([np.median(grouped[s]) for s in states])


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    for ax, panel in zip(axes, PANELS):
        rows = load(panel["sweep"])
        peak_x = peak_y = 0.0
        for n, key in enumerate(panel["order"]):
            states, squared, millis = curve(rows, panel["group"], key)
            if squared.size < 2:
                continue
            ax.plot(squared, millis, marker=MARKERS[n % len(MARKERS)],
                    color=COLOURS[n % len(COLOURS)], linewidth=1.8, markersize=5.5,
                    label=panel["labels"].get(key, key))
            peak_x = max(peak_x, squared.max())
            peak_y = max(peak_y, millis.max())

        ax.set_xlim(0, peak_x * 1.04)
        ax.set_ylim(0, peak_y * 1.12)
        ax.set_xlabel(panel["xlabel"])
        ax.set_ylabel("Computation time (ms)")
        ax.grid(True, alpha=0.35)
        ax.set_axisbelow(True)
        ax.legend(fontsize=9.5, loc="upper left")

        # The untransformed state count on top, so the squared axis stays readable
        top = ax.secondary_xaxis("top", functions=(lambda v: np.sqrt(np.maximum(v, 0)),
                                                   lambda v: np.square(v)))
        top.set_xlabel("Automaton states")

    fig.tight_layout()
    path = os.path.join(OUT_DIR, "automaton_scaling.png")
    fig.savefig(path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print("Wrote %s" % path)


if __name__ == "__main__":
    main()
