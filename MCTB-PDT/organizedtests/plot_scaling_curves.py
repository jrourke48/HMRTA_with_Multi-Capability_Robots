#!/usr/bin/env python3
"""
Runtime scaling: how the tree planner's computation time grows with each independent variable.

One panel per sweep, log time against the variable that sweep varies, median over whatever the
sweep holds constant.

Read from the wide per-configuration CSVs rather than combined_long.csv, which is what fixes the
empty automaton_states panel the earlier version of this figure had: combined_long records
whatever value storeRun was handed as its "independent variable", and most sweeps end up with a
column name there while automaton_states ends up with the bare ids 1..16. Grouping on that field
produced sixteen one-point groups and an empty axis. The wide CSVs carry the parameter columns
under their own names, so the variable is unambiguous.

Writes comparison/scaling_curves.png. Run from the organizedtests directory:
    python3 plot_scaling_curves.py
"""

import csv
import glob
import os
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
# Shared thesis styling: Times New Roman, print palette, titles suppressed in favour of captions
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
import thesis_style as _thesis_style
_thesis_style.apply()
import numpy as np

OUT_DIR = "comparison"
TREE = "#c2501f"
INK, INK_MUTED, SURFACE = "#000000", "#444444", "#ffffff"

# sweep -> (the column it sweeps, axis label, panel title)
#
# The two automaton sweeps are deliberately absent. Runtime grows roughly 2,500-fold across their
# range, which needs a log axis, and a log axis next to these four would flatter them: it turns a
# half-millisecond wobble into a visible slope. These four are the fleet and environment
# variables, and the point of the figure is that none of them moves the runtime at all.
SWEEPS = [
    ("number_robots", "num_robots", "Robots", "Number of robots"),
    ("transition_system_regions", "num_ts_regions", "Transition-system regions", "TS regions"),
    ("average_capabilities", "average_capabilities", "Capabilities per robot",
     "Average capabilities"),
    ("robot_homogeneity", "robot_homogeneity", "Robot homogeneity", "Robot homogeneity"),
]

# Four automata per panel rather than one averaged line. A single median curve cannot show
# whether a flat trend is genuinely flat or an average of configurations moving in opposite
# directions, and these panels exist to claim flatness.
CONFIGS_PER_PANEL = 4
CONFIG_COLOURS = ["#0072B2", "#D55E00", "#009E73", "#CC79A7"]
CONFIG_MARKERS = ["o", "s", "^", "D"]

TIME_COLUMN = "total_computation_time_ms"

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Nimbus Roman No9 L", "Liberation Serif", "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "font.size": 11,
    "axes.labelsize": 11.5,
    "axes.titlesize": 12.5,
    "legend.fontsize": 9.5,
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "axes.edgecolor": INK,
    "axes.linewidth": 0.8,
    "text.color": INK,
    "axes.labelcolor": INK,
    "xtick.color": INK,
    "ytick.color": INK,
    "xtick.direction": "in",
    "ytick.direction": "in",
    "grid.color": "#cccccc",
    "grid.linewidth": 0.5,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "legend.frameon": False,
})


def load(sweep):
    """Every configuration row of one sweep, from its per-configuration CSVs."""
    rows = []
    for path in sorted(glob.glob(os.path.join(sweep, "data", "*.csv"))):
        if os.path.basename(path) in ("statistics.csv", "combined_long.csv"):
            continue
        with open(path, newline="") as f:
            rows.extend(csv.DictReader(f))
    return rows


def series_points(rows, xcolumn):
    """Median runtime in milliseconds per x value, over the rows given."""
    grouped = defaultdict(list)
    for row in rows:
        try:
            x = float(row[xcolumn])
            y = float(row[TIME_COLUMN])
        except (KeyError, TypeError, ValueError):
            continue
        if y > 0:
            grouped[x].append(y)
    xs = sorted(grouped)
    return xs, [float(np.median(grouped[x])) for x in xs]


def pick_configs(rows, count=CONFIGS_PER_PANEL):
    """Spread the chosen automata across the available range rather than taking the first few."""
    ids = sorted({row["automaton_id"] for row in rows if row.get("automaton_id")},
                 key=lambda v: int(v))
    if len(ids) <= count:
        return ids
    step = (len(ids) - 1) / float(count - 1)
    return [ids[int(round(n * step))] for n in range(count)]


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(13, 8.5))
    drawn = []

    for ax, (sweep, xcolumn, xlabel, title) in zip(axes.flat, SWEEPS):
        rows = load(sweep)
        peak = 0.0
        chosen = pick_configs(rows)
        for n, automaton in enumerate(chosen):
            xs, ys = series_points([r for r in rows if r.get("automaton_id") == automaton],
                                   xcolumn)
            # One x value is a dot, not a scaling curve
            if len(xs) < 2:
                continue
            ax.plot(xs, ys, marker=CONFIG_MARKERS[n % len(CONFIG_MARKERS)],
                    color=CONFIG_COLOURS[n % len(CONFIG_COLOURS)], linewidth=1.8,
                    markersize=5.5, label="Automaton %s" % automaton)
            peak = max(peak, max(ys))

        # Linear and anchored at zero. Auto-scaled, a span of 9.2ms to 10.8ms fills the panel and
        # reads as a trend; against zero it reads as the flat line it is, which is the result.
        ax.set_ylim(0, peak * 1.35 if peak else 1.0)
        ax.set_xlabel(xlabel)
        ax.set_ylabel("Computation time (ms)")
        ax.set_title(title)
        ax.grid(True, which="both", alpha=0.35)
        ax.set_axisbelow(True)
        ax.legend(fontsize=9)
        drawn.append("  %-26s automata %s" % (sweep, ", ".join(chosen)))

    fig.tight_layout()
    path = os.path.join(OUT_DIR, "scaling_curves.png")
    fig.savefig(path, dpi=300, bbox_inches="tight")
    plt.close(fig)

    print("\n".join(drawn))
    print("\nWrote %s" % path)


if __name__ == "__main__":
    main()
