#!/usr/bin/env python3
"""
Plot robot homogeneity vs computation time and makespan from TestRunManager CSV exports
Reads from robot_homogeneity/data/
Shows scaling across 6 automata and 3 robot configurations (6, 10, 16) with varying robot homogeneity
"""

import matplotlib.pyplot as plt
# Shared thesis styling: Times New Roman, print palette, titles suppressed in favour of captions
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
import thesis_style as _thesis_style
_thesis_style.apply()
import os
import glob
import csv


# Metrics averaged when a configuration appears on more than one CSV row.
COLLAPSE_FIELDS = ['total_computation_time_ms', 'tree_makespan_seconds',
                   'product_makespan_seconds', 'random_sampling_makespan_seconds']


def collapse_rows(rows, x_key, value_keys=COLLAPSE_FIELDS):
    """Average the metrics across every row sharing the same value of x_key.

    One configuration is normally one row, but it may appear several times (the
    sampler was once swept over several search budgets, writing a row each).
    Averaging yields exactly one point per x either way, so the series cannot
    double back on itself and draw a zig-zag across repeated x values.
    """
    buckets, order = {}, []
    for row in rows:
        key = row.get(x_key)
        if key not in buckets:
            buckets[key] = []
            order.append(key)
        buckets[key].append(row)

    collapsed = []
    for key in order:
        group = buckets[key]
        merged = dict(group[0])
        for field in value_keys:
            values = []
            for row in group:
                text = (row.get(field) or '').strip()
                if not text:
                    continue
                try:
                    values.append(float(text))
                except ValueError:
                    pass
            merged[field] = repr(sum(values) / len(values)) if values else ''
        collapsed.append(merged)
    return collapsed


# Helper function to filter product makespans
def filter_product_makespans(product_makespans, makespans, threshold=2.0):
    """
    Filter product makespans to only plot values that are <= threshold * tree makespan
    Returns a list with invalid values replaced with None (matplotlib will skip them)
    """
    filtered = []
    for pm, tm in zip(product_makespans, makespans):
        if pm > 0 and tm > 0 and pm > threshold * tm:
            filtered.append(None)  # Skip astronomical values
        else:
            filtered.append(pm)
    return filtered

# Create Plots directory if it doesn't exist
os.makedirs('Plots', exist_ok=True)

# Get all CSV files from data folder
output_dir = "data"
csv_files = sorted(set(
    glob.glob(os.path.join(output_dir, "robot_homogeneity_automaton_id_*.csv"))
    + glob.glob(os.path.join(output_dir, "automaton_states_automaton_id_*.csv"))
))

if not csv_files:
    print(f"Error: No CSV files found in {output_dir}/")
    exit(1)

# Parse data from CSV files - organized by (automaton_id, robot_count)
config_data = {}

for csv_file in sorted(csv_files):
    filename = os.path.basename(csv_file)
    # Extract automaton_id and num_robots from filename
    # Accept the previous category prefix for CSVs exported before the category fix.
    prefix = "robot_homogeneity_automaton_id_"
    if filename.startswith("automaton_states_automaton_id_"):
        prefix = "automaton_states_automaton_id_"
    parts = filename.replace(prefix, "").replace(".csv", "").split("_num_robots_")
    automaton_id = int(parts[0])
    robot_count = int(parts[1])
    config_key = (automaton_id, robot_count)
    
    config_data[config_key] = {
        'homogeneities': [],
        'computation_times': [],
        'makespans': [],
        'product_makespans': [],
        'sampling_makespans': []
    }
    
    # Read CSV file
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        rows_list = collapse_rows(list(reader), 'robot_homogeneity')
        
    # Sort by robot_homogeneity
    rows_list.sort(key=lambda x: float(x['robot_homogeneity']))
    
    for row in rows_list:
        homogeneity = float(row['robot_homogeneity'])
        computation_time = float(row['total_computation_time_ms'])
        makespan = float(row['tree_makespan_seconds']) if 'tree_makespan_seconds' in row and row['tree_makespan_seconds'].strip() else 0
        product_makespan = float(row['product_makespan_seconds']) if 'product_makespan_seconds' in row and row['product_makespan_seconds'].strip() else 0
        sampling_makespan = float(row.get('random_sampling_makespan_seconds') or 0)
        
        config_data[config_key]['homogeneities'].append(homogeneity)
        config_data[config_key]['computation_times'].append(computation_time)
        config_data[config_key]['makespans'].append(makespan)
        config_data[config_key]['product_makespans'].append(product_makespan)
        config_data[config_key]['sampling_makespans'].append(sampling_makespan)

# Define colors for each robot configuration
robot_colors = {6: '#0072B2', 10: '#D55E00', 16: '#009E73'}
robot_markers = {6: 'o', 10: 's', 16: '^'}

# Define colors for each automaton
automaton_colors = ['#0072B2', '#D55E00', '#009E73', '#CC79A7', '#762a83', '#8c510a',
                     '#E69F00', '#6b6b6b', '#c98700', '#56B4E9', '#e5bcd3', '#ba94c1',
                     '#c5a784', '#f2cf7f', '#6b6b6b', '#762a83']

# ============================================================================
# FIGURE 1: Computation Time vs Robot Homogeneity (All Configs Combined)
# ============================================================================
fig1, ax1 = plt.subplots(figsize=(12, 7))
fig1.suptitle('Robot Homogeneity vs Computation Time (All Configurations)', fontsize=16)

for (automaton_id, robot_count) in sorted(config_data.keys()):
    data = config_data[(automaton_id, robot_count)]
    label = f'Auto {automaton_id}, {robot_count} Robots'
    ax1.plot(data['homogeneities'], data['computation_times'], 
             marker=robot_markers[robot_count], markersize=8, 
             linewidth=2.5, label=label)

ax1.set_xlabel('Robot Homogeneity (Independent Caps / Num Robots)', fontsize=12)
ax1.set_ylabel('Computation Time (ms)', fontsize=12)
ax1.grid(True, alpha=0.3)
ax1.legend(loc='best', fontsize=9)

plt.tight_layout()
plt.savefig('Plots/robot_homogeneity_computation_time_all.png', dpi=300, bbox_inches='tight')
print("✓ Plot saved as Plots/robot_homogeneity_computation_time_all.png")
plt.close(fig1)

# ============================================================================
# FIGURE 2: Makespan vs Robot Homogeneity (All Configs Combined)
# ============================================================================
fig2, ax2 = plt.subplots(figsize=(12, 7))
fig2.suptitle('Robot Homogeneity vs Makespan (All Configurations)', fontsize=16)

for (automaton_id, robot_count) in sorted(config_data.keys()):
    data = config_data[(automaton_id, robot_count)]
    label = f'Auto {automaton_id}, {robot_count} Robots (Task Allocation)'
    ax2.plot(data['homogeneities'], data['makespans'], 
             marker=robot_markers[robot_count], markersize=8, 
             linewidth=2.5, label=label)
    
    # Add product makespan overlay if non-zero values exist (only if within 2x tree makespan)
    filtered_product_makespans = filter_product_makespans(data['product_makespans'], data['makespans'])
    if any(pm > 0 for pm in filtered_product_makespans):
        label_prod = f'Auto {automaton_id}, {robot_count} Robots (Product)'
        ax2.plot(data['homogeneities'], filtered_product_makespans, 
                marker=robot_markers[robot_count], markersize=6, 
                linewidth=2.5, linestyle='--', label=label_prod)

    if any(sm > 0 for sm in data['sampling_makespans']):
        ax2.plot(data['homogeneities'], [sm if sm > 0 else None for sm in data['sampling_makespans']],
                 marker=robot_markers[robot_count], linewidth=2, linestyle=':',
                 label=f'Auto {automaton_id}, {robot_count} Robots (Sampler)')
    
    # Add value labels on points
    for hom, m in zip(data['homogeneities'], data['makespans']):
        if m > 0:
            ax2.text(hom, m, f'{m:.0f}s', ha='center', va='bottom', fontsize=7)

ax2.set_xlabel('Robot Homogeneity (Independent Caps / Num Robots)', fontsize=12)
ax2.set_ylabel('Makespan (seconds)', fontsize=12)
ax2.grid(True, alpha=0.3)
ax2.legend(loc='best', fontsize=8)

plt.tight_layout()
plt.savefig('Plots/robot_homogeneity_makespan_all.png', dpi=300, bbox_inches='tight')
print("✓ Plot saved as Plots/robot_homogeneity_makespan_all.png")
plt.close(fig2)

# ============================================================================
# FIGURE 3-8: Computation Time by Automaton (Combined Robot Counts)
# ============================================================================
max_automaton_id = max([cfg[0] for cfg in config_data.keys()], default=6)
for automaton_id in range(1, max_automaton_id + 1):
    fig, ax = plt.subplots(figsize=(12, 7))
    fig.suptitle(f'Automaton {automaton_id}: Robot Homogeneity vs Computation Time (All Robot Counts)', 
                 fontsize=16)
    
    for robot_count in [6, 10, 16]:
        config_key = (automaton_id, robot_count)
        if config_key in config_data:
            data = config_data[config_key]
            ax.plot(data['homogeneities'], data['computation_times'], 
                    marker=robot_markers[robot_count], markersize=8, 
                    linewidth=2.5, color=robot_colors[robot_count],
                    label=f'{robot_count} Robots')
    
    ax.set_xlabel('Robot Homogeneity (Independent Caps / Num Robots)', fontsize=12)
    ax.set_ylabel('Computation Time (ms)', fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=10)
    
    plt.tight_layout()
    plt.savefig(f'Plots/automaton_{automaton_id}_computation_time_combined.png', dpi=300, bbox_inches='tight')
    print(f"✓ Plot saved as Plots/automaton_{automaton_id}_computation_time_combined.png")
    plt.close(fig)

# ============================================================================
# FIGURE 9-14: Makespan by Automaton (Combined Robot Counts)
# ============================================================================
for automaton_id in range(1, max_automaton_id + 1):
    fig, ax = plt.subplots(figsize=(12, 7))
    fig.suptitle(f'Automaton {automaton_id}: Robot Homogeneity vs Makespan (All Robot Counts)', 
                 fontsize=16)
    
    for robot_count in [6, 10, 16]:
        config_key = (automaton_id, robot_count)
        if config_key in config_data:
            data = config_data[config_key]
            ax.plot(data['homogeneities'], data['makespans'], 
                    marker=robot_markers[robot_count], markersize=8, 
                    linewidth=2.5, color=robot_colors[robot_count],
                    label=f'{robot_count} Robots (Task Allocation)')
            
            # Add product makespan overlay if non-zero values exist (only if within 2x tree makespan)
            filtered_product_makespans = filter_product_makespans(data['product_makespans'], data['makespans'])
            if any(pm > 0 for pm in filtered_product_makespans):
                ax.plot(data['homogeneities'], filtered_product_makespans, 
                        marker=robot_markers[robot_count], markersize=6, 
                        linewidth=2.5, color=robot_colors[robot_count], linestyle='--',
                        label=f'{robot_count} Robots (Product)')
            if any(sm > 0 for sm in data['sampling_makespans']):
                ax.plot(data['homogeneities'], [sm if sm > 0 else None for sm in data['sampling_makespans']],
                        marker=robot_markers[robot_count], linewidth=2, color=robot_colors[robot_count],
                        linestyle=':', label=f'{robot_count} Robots (Sampler)')
    
    ax.set_xlabel('Robot Homogeneity (Independent Caps / Num Robots)', fontsize=12)
    ax.set_ylabel('Makespan (seconds)', fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=9)
    
    plt.tight_layout()
    plt.savefig(f'Plots/automaton_{automaton_id}_makespan_combined.png', dpi=300, bbox_inches='tight')
    print(f"✓ Plot saved as Plots/automaton_{automaton_id}_makespan_combined.png")
    plt.close(fig)

print("\n✓ All plots generated successfully!")
