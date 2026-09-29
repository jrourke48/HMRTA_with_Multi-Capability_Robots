#!/usr/bin/env python3
"""
3D makespan plots for every organized test
For each test folder this makes two plots, saved in <test>/Plots/:
  1. x vs (total required capabilities / total robot capabilities) vs task allocation makespan
  2. x vs number of atomic propositions vs task allocation makespan
x is the number of robots, except for tests where the robot count is fixed,
which use the variable that test sweeps instead (TS regions, batch configuration).
The number of atomic propositions is not in the CSVs, so it is counted from the
LTL formula of createTestInfiniteBuchiAutomatonN() in each test's .cpp file.

Usage: python plot_3d_makespan.py [test_folder ...]   (default: all tests)
"""

import os
import re
import sys
import csv
import glob
from collections import defaultdict
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import cm

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# test folder -> (CSV glob, x-axis label, function row, filename -> x value)
TESTS = {
    'number_robots': ('num_robots_automaton_id_*.csv', 'Number of Robots',
                      lambda row, fname: int(row['num_robots'])),
    'automaton_states': ('automaton_states_num_robots_*.csv', 'Number of Robots',
                         lambda row, fname: int(row['num_robots'])),
    'average_capabilities': ('automaton_states_automaton_id_*.csv', 'Number of Robots',
                             lambda row, fname: int(row['num_robots'])),
    'robot_homogeneity': ('automaton_states_automaton_id_*.csv', 'Number of Robots',
                          lambda row, fname: int(row['num_robots'])),
    # Robot count is fixed at 6 in this test
    'transition_system_regions': ('ts_regions_automaton_id_*.csv', 'Number of TS Regions',
                                  lambda row, fname: int(row['num_ts_regions'])),
    # Robot count is fixed at 45 in this test
    'automaton_states_batch': ('automaton_states_batch_Batch_Configuration_*.csv', 'Batch Configuration',
                               lambda row, fname: int(re.search(r'Configuration_(\d+)', fname).group(1))),
}


def count_atomic_propositions(test_dir):
    """Map automaton id -> number of distinct atomic propositions in its LTL formula"""
    ap_counts = {}
    for cpp_file in glob.glob(os.path.join(test_dir, 'test_*.cpp')):
        with open(cpp_file, 'r', encoding='utf-8') as f:
            source = f.read()
        for match in re.finditer(r'BuchiAutomaton\*\s+createTestInfiniteBuchiAutomaton(\d+)\(\)\s*\{', source):
            body = source[match.end():source.find('\n}', match.end())]
            formula = re.search(r'string ltl_str\s*=\s*(.*?);\s*\n', body, re.S)
            if formula:
                ap_counts[int(match.group(1))] = len(set(re.findall(r'p(\d+)', formula.group(1))))
    return ap_counts


def load_rows(test_dir, csv_pattern, x_of):
    """Return a list of (x, capability ratio, automaton id, makespan) for every run"""
    rows = []
    for csv_file in sorted(glob.glob(os.path.join(test_dir, 'data', csv_pattern))):
        with open(csv_file, 'r') as f:
            for row in csv.DictReader(f):
                makespan = row.get('tree_makespan_seconds', '').strip()
                robot_caps = float(row.get('total_robot_capabilities', 0) or 0)
                if not makespan or robot_caps == 0:
                    continue
                ratio = float(row['total_required_capabilities']) / robot_caps
                rows.append((x_of(row, os.path.basename(csv_file)), ratio, int(row['automaton_id']), float(makespan)))
    return rows


def plot_3d(points, x_label, y_label, title, out_path):
    """points: list of (x, y, makespan). Surface through the mean makespan at each (x, y), dots for every run."""
    means = defaultdict(list)
    for x, y, z in points:
        means[(x, round(y, 6))].append(z)
    mx, my, mz = zip(*[(x, y, np.mean(zs)) for (x, y), zs in means.items()])
    xs, ys, zs = zip(*points)
    vmax = max(zs)

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')

    # Single-hue sequential color: light = short makespan, dark = long makespan
    # Draw a surface only when every x was run with every y (a full grid); otherwise a surface
    # would interpolate shapes that are not in the data, so show the runs as dots instead
    unique_x, unique_y = sorted(set(mx)), sorted(set(my))
    is_grid = len(unique_x) >= 2 and len(unique_y) >= 2 and len(means) == len(unique_x) * len(unique_y)
    surface = None
    if is_grid:
        X, Y = np.meshgrid(unique_x, unique_y)
        Z = np.array([[np.mean(means[(x, y)]) for x in unique_x] for y in unique_y])
        surface = ax.plot_surface(X, Y, Z, cmap=cm.Blues, vmin=0, vmax=vmax,
                                  edgecolor='#52514e', linewidth=0.4, alpha=0.85, antialiased=True)
    else:
        # Thin drop lines to the floor so each dot's (x, y) position can be read
        for x, y, z in zip(xs, ys, zs):
            ax.plot([x, x], [y, y], [0, z], color='#a9a8a3', linewidth=0.6)
    dots = ax.scatter(xs, ys, zs, c=zs, cmap=cm.Blues, vmin=0, vmax=vmax,
                      edgecolor='#0b0b0b', linewidth=0.5, s=18, depthshade=False)

    ax.set_title(title, fontsize=15, fontweight='bold')
    ax.set_xlabel(x_label, fontsize=12, fontweight='bold', labelpad=10)
    ax.set_ylabel(y_label, fontsize=12, fontweight='bold', labelpad=10)
    ax.set_zlabel('Task Allocation Makespan (s)', fontsize=12, fontweight='bold', labelpad=12)
    # Tick at every data value, skipping values too close to the previous tick to read
    def data_ticks(values):
        values = sorted(set(values))
        min_gap = (values[-1] - values[0]) / 12 if len(values) > 1 else 0
        ticks = []
        for v in values:
            if not ticks or v - ticks[-1] >= min_gap:
                ticks.append(v)
        return ticks
    if len(set(xs)) <= 12:
        ax.set_xticks(data_ticks(xs))
    if len(set(ys)) <= 16:
        ax.set_yticks(data_ticks(ys))
    ax.set_zlim(bottom=0)
    ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f'{v:g}' if float(v).is_integer() else f'{v:.2f}'))
    ax.tick_params(axis='both', labelsize=9)
    ax.view_init(elev=22, azim=50)
    ax.set_box_aspect(None, zoom=0.88)

    colorbar = fig.colorbar(surface if surface is not None else dots, ax=ax, shrink=0.6, pad=0.1)
    colorbar.set_label('Makespan (s)', fontsize=11)

    plt.tight_layout()
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f"Plot saved as {os.path.relpath(out_path, BASE_DIR)}")


def plot_test(test_name):
    csv_pattern, x_label, x_of = TESTS[test_name]
    test_dir = os.path.join(BASE_DIR, test_name)
    rows = load_rows(test_dir, csv_pattern, x_of)
    if not rows:
        print(f"Skipping {test_name}: no CSV data found")
        return
    ap_counts = count_atomic_propositions(test_dir)
    plots_dir = os.path.join(test_dir, 'Plots')
    os.makedirs(plots_dir, exist_ok=True)
    pretty = test_name.replace('_', ' ').title()

    plot_3d([(x, ratio, z) for x, ratio, _, z in rows],
            x_label, 'Required / Robot Capabilities',
            f'{pretty}: {x_label} vs Capability Ratio vs Makespan',
            os.path.join(plots_dir, f'{test_name}_capability_ratio_makespan_3d.png'))

    missing = sorted({a for _, _, a, _ in rows if a not in ap_counts})
    if missing:
        print(f"  Warning: no LTL formula found for automata {missing} in {test_name}, left out of the AP plot")
    ap_points = [(x, ap_counts[a], z) for x, _, a, z in rows if a in ap_counts]
    if ap_points:
        plot_3d(ap_points, x_label, 'Number of Atomic Propositions',
                f'{pretty}: {x_label} vs Atomic Propositions vs Makespan',
                os.path.join(plots_dir, f'{test_name}_atomic_propositions_makespan_3d.png'))


if __name__ == '__main__':
    selected = sys.argv[1:] or list(TESTS)
    for name in selected:
        if name not in TESTS:
            print(f"Unknown test '{name}', choose from: {', '.join(TESTS)}")
            continue
        plot_test(name)
