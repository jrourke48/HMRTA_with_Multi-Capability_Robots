#!/usr/bin/env python3
"""
3D plot of number of robots vs total required capabilities vs makespan
Reads the TestRunManager CSV exports from /data (one file per automaton)
Each automaton has a fixed number of required capabilities, so together the
automata form a grid of (robots, required capabilities) -> task allocation makespan
"""

import os
import glob
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
# Shared thesis styling: Times New Roman, print palette, titles suppressed in favour of captions
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
import thesis_style as _thesis_style
_thesis_style.apply()

os.makedirs('Plots', exist_ok=True)

csv_files = glob.glob(os.path.join("data", "num_robots_automaton_id_*.csv"))
if not csv_files:
    print("Error: No CSV files found in data/")
    exit(1)

# (num_robots, required_capabilities) -> makespan
tree_points = {}
sampler_points = {}
for csv_file in sorted(csv_files):
    with open(csv_file, 'r') as f:
        for row in csv.DictReader(f):
            robots = int(row['num_robots'])
            capabilities = int(row['total_required_capabilities'])
            tree_makespan = row.get('tree_makespan_seconds', '').strip()
            sampler_makespan = row.get('random_sampling_makespan_seconds', '').strip()
            if tree_makespan:
                tree_points[(robots, capabilities)] = float(tree_makespan)
            if sampler_makespan:
                sampler_points[(robots, capabilities)] = float(sampler_makespan)

def plot_surface(points, makespan_label, output_filename):
    robot_counts = sorted({r for r, _ in points})
    capability_counts = sorted({c for _, c in points})

    # Grid for the surface, NaN where a combination was not run
    X, Y = np.meshgrid(robot_counts, capability_counts)
    Z = np.full(X.shape, np.nan)
    for i, c in enumerate(capability_counts):
        for j, r in enumerate(robot_counts):
            Z[i, j] = points.get((r, c), np.nan)

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')

    # Single-hue sequential color: light = short makespan, dark = long makespan
    surface = ax.plot_surface(X, Y, Z, cmap=cm.Blues, vmin=0, vmax=np.nanmax(Z),
                              edgecolor='#52514e', linewidth=0.4, alpha=0.9, antialiased=True)
    xs, ys, zs = zip(*[(r, c, m) for (r, c), m in points.items()])
    ax.scatter(xs, ys, zs, color='#0b0b0b', s=12, depthshade=False)

    ax.set_title(f'Number of Robots vs Required Capabilities vs {makespan_label}', fontsize=16)
    ax.set_xlabel('Number of Robots', fontsize=12, labelpad=10)
    ax.set_ylabel('Total Required Capabilities', fontsize=12, labelpad=10)
    ax.set_zlabel(f'{makespan_label} Makespan (s)', fontsize=12, labelpad=12)
    ax.set_xticks(robot_counts)
    ax.set_yticks(capability_counts)
    ax.tick_params(axis='y', labelsize=9)
    ax.view_init(elev=22, azim=50)
    ax.set_box_aspect(None, zoom=0.88)

    colorbar = fig.colorbar(surface, ax=ax, shrink=0.6, pad=0.1)
    colorbar.set_label('Makespan (s)', fontsize=11)

    plt.tight_layout()
    plt.savefig(output_filename, dpi=300, bbox_inches='tight')
    print(f'Plot saved as {output_filename}')
    plt.close(fig)

plot_surface(tree_points, 'Task Allocation', 'Plots/robots_capabilities_makespan_3d.png')
if sampler_points:
    plot_surface(sampler_points, 'Random Sampling', 'Plots/robots_capabilities_sampler_makespan_3d.png')
