#!/usr/bin/env python3
"""
Makespan against each fleet and environment variable, four automata per panel.

The companion to plot_scaling_curves.py, which plots the same four sweeps against computation
time. Runtime barely responds to any of these variables; makespan responds to all of them, and
putting the two figures beside each other is the point: the planner's cost is set by the
specification, the plan's cost is set by the fleet and the environment.

Four automata per panel rather than one averaged line, for the same reason as the runtime figure.
An average can hide configurations moving in opposite directions, and these panels are claims
about direction. The automata are spread across each sweep's range rather than taken in order.

Linear axes anchored at zero, so the size of each effect is readable rather than just its shape.

Writes comparison/makespan_curves.png. Run from the organizedtests directory:
    python3 plot_makespan_curves.py
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
MAKESPAN_COLUMN = "tree_makespan_seconds"
SAMPLER_COLUMN = "random_sampling_makespan_seconds"
CONFIGS_PER_PANEL = 4
CONFIG_COLOURS = thesis_style.PALETTE[:4]
CONFIG_MARKERS = ["o", "s", "^", "D"]

# sweep -> (swept column, axis label, output filename)
SWEEPS = [
    ("number_robots", "num_robots", "Robots", "makespan_number_robots.png"),
    ("transition_system_regions", "num_ts_regions", "Transition-system regions",
     "makespan_ts_regions.png"),
    ("average_capabilities", "average_capabilities", "Capabilities per robot",
     "makespan_average_capabilities.png"),
    ("robot_homogeneity", "robot_homogeneity", "Robot homogeneity",
     "makespan_robot_homogeneity.png"),
]

# Automata 3 and 5 both step sharply partway along the homogeneity axis, which reads as an effect
# but is the team size changing underneath: this metric is distinct capabilities over robots, so
# its high values exist only for 6-robot teams and its low values only for 16-robot ones. The step
# is that switch. Named explicitly here rather than spread across the range by pick_configs.
PINNED = {"robot_homogeneity": ["1", "2", "4", "6"]}


def load(sweep):
    rows = []
    for path in sorted(glob.glob(os.path.join(sweep, "data", "*.csv"))):
        if os.path.basename(path) in ("statistics.csv", "combined_long.csv"):
            continue
        with open(path, newline="") as f:
            rows.extend(csv.DictReader(f))
    return rows


def series_points(rows, xcolumn, ycolumn):
    """Median makespan in seconds per x value, for one planner's column."""
    grouped = defaultdict(list)
    for row in rows:
        try:
            x = float(row[xcolumn])
            y = float(row[ycolumn])
        except (KeyError, TypeError, ValueError):
            continue
        if y > 0:
            grouped[x].append(y)
    xs = sorted(grouped)
    return xs, [float(np.median(grouped[x])) for x in xs]


def pick_configs(rows, sweep, count=CONFIGS_PER_PANEL):
    """The pinned automata for this sweep, else a spread across the available range."""
    available = {row["automaton_id"] for row in rows if row.get("automaton_id")}
    if sweep in PINNED:
        return [a for a in PINNED[sweep] if a in available]
    ids = sorted(available, key=lambda v: int(v))
    if len(ids) <= count:
        return ids
    step = (len(ids) - 1) / float(count - 1)
    return [ids[int(round(n * step))] for n in range(count)]


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    for sweep, xcolumn, xlabel, filename in SWEEPS:
        rows = load(sweep)
        chosen = pick_configs(rows, sweep)
        fig, ax = plt.subplots(figsize=(7.6, 5.4))
        peak = 0.0

        for n, automaton in enumerate(chosen):
            subset = [r for r in rows if r.get("automaton_id") == automaton]
            colour = CONFIG_COLOURS[n % len(CONFIG_COLOURS)]
            marker = CONFIG_MARKERS[n % len(CONFIG_MARKERS)]

            # Colour carries the automaton, line style carries the planner, so eight lines stay
            # readable: a dotted line is always the sampler of the solid line it sits beside
            for ycolumn, style, fill in ((MAKESPAN_COLUMN, "-", colour),
                                         (SAMPLER_COLUMN, ":", "none")):
                xs, ys = series_points(subset, xcolumn, ycolumn)
                if len(xs) < 2:
                    continue
                ax.plot(xs, ys, linestyle=style, marker=marker, color=colour,
                        markerfacecolor=fill, markeredgecolor=colour, linewidth=1.7,
                        markersize=5.5,
                        label="Automaton %s" % automaton if style == "-" else None)
                peak = max(peak, max(ys))

        ax.set_ylim(0, peak * 1.15 if peak else 1.0)
        ax.set_xlabel(xlabel)
        ax.set_ylabel("Makespan (s)")
        ax.grid(True, alpha=0.35)
        ax.set_axisbelow(True)

        # Two legends: which automaton a colour is, and which planner a line style is
        automata_legend = ax.legend(fontsize=9, loc="upper right")
        ax.add_artist(automata_legend)
        planner_keys = [
            matplotlib.lines.Line2D([], [], color=thesis_style.INK_MUTED, linestyle="-",
                                    label="Tree planner"),
            matplotlib.lines.Line2D([], [], color=thesis_style.INK_MUTED, linestyle=":",
                                    label="Random sampler"),
        ]
        ax.legend(handles=planner_keys, fontsize=9, loc="upper center")

        fig.tight_layout()
        path = os.path.join(OUT_DIR, filename)
        fig.savefig(path, dpi=300, bbox_inches="tight")
        plt.close(fig)
        print("  %-28s automata %-14s -> %s" % (sweep, ", ".join(chosen), path))


if __name__ == "__main__":
    main()
