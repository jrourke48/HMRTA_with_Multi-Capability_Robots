#!/usr/bin/env python3
"""
Plot robot homogeneity vs computation time and makespan from TestRunManager CSV exports
Reads from robot_homogeneity/data/
Shows scaling across 6 automata and 3 robot configurations (6, 10, 16) with varying robot homogeneity
"""

import matplotlib.pyplot as plt
import os
import glob
import csv

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
csv_files = glob.glob(os.path.join(output_dir, "automaton_states_automaton_id_*.csv"))

if not csv_files:
    print(f"Error: No CSV files found in {output_dir}/")
    exit(1)

# Parse data from CSV files - organized by (automaton_id, robot_count)
config_data = {}

for csv_file in sorted(csv_files):
    filename = os.path.basename(csv_file)
    # Extract automaton_id and num_robots from filename
    # Format: automaton_states_automaton_id_N_num_robots_R.csv
    parts = filename.replace("automaton_states_automaton_id_", "").replace(".csv", "").split("_num_robots_")
    automaton_id = int(parts[0])
    robot_count = int(parts[1])
    config_key = (automaton_id, robot_count)
    
    config_data[config_key] = {
        'homogeneities': [],
        'computation_times': [],
        'makespans': [],
        'product_makespans': []
    }
    
    # Read CSV file
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        rows_list = list(reader)
        
    # Sort by robot_homogeneity
    rows_list.sort(key=lambda x: float(x['robot_homogeneity']))
    
    for row in rows_list:
        homogeneity = float(row['robot_homogeneity'])
        computation_time = float(row['total_computation_time_ms'])
        makespan = float(row['tree_makespan_seconds']) if 'tree_makespan_seconds' in row and row['tree_makespan_seconds'].strip() else 0
        product_makespan = float(row['product_makespan_seconds']) if 'product_makespan_seconds' in row and row['product_makespan_seconds'].strip() else 0
        
        config_data[config_key]['homogeneities'].append(homogeneity)
        config_data[config_key]['computation_times'].append(computation_time)
        config_data[config_key]['makespans'].append(makespan)
        config_data[config_key]['product_makespans'].append(product_makespan)

# Define colors for each robot configuration
robot_colors = {6: '#1f77b4', 10: '#ff7f0e', 16: '#2ca02c'}
robot_markers = {6: 'o', 10: 's', 16: '^'}

# Define colors for each automaton
automaton_colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b',
                     '#e377c2', '#7f7f7f', '#bcbd22', '#17becf', '#ff9896', '#c5b0d5',
                     '#c49c94', '#f7b6d2', '#7f7f7f', '#9467bd']

# ============================================================================
# FIGURE 1: Computation Time vs Robot Homogeneity (All Configs Combined)
# ============================================================================
fig1, ax1 = plt.subplots(figsize=(12, 7))
fig1.suptitle('Robot Homogeneity vs Computation Time (All Configurations)', fontsize=16, fontweight='bold')

for (automaton_id, robot_count) in sorted(config_data.keys()):
    data = config_data[(automaton_id, robot_count)]
    label = f'Auto {automaton_id}, {robot_count} Robots'
    ax1.plot(data['homogeneities'], data['computation_times'], 
             marker=robot_markers[robot_count], markersize=8, 
             linewidth=2.5, label=label)

ax1.set_xlabel('Robot Homogeneity (Independent Caps / Num Robots)', fontsize=12, fontweight='bold')
ax1.set_ylabel('Computation Time (ms)', fontsize=12, fontweight='bold')
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
fig2.suptitle('Robot Homogeneity vs Makespan (All Configurations)', fontsize=16, fontweight='bold')

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
    
    # Add value labels on points
    for hom, m in zip(data['homogeneities'], data['makespans']):
        if m > 0:
            ax2.text(hom, m, f'{m:.0f}s', ha='center', va='bottom', fontsize=7)

ax2.set_xlabel('Robot Homogeneity (Independent Caps / Num Robots)', fontsize=12, fontweight='bold')
ax2.set_ylabel('Makespan (seconds)', fontsize=12, fontweight='bold')
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
                 fontsize=16, fontweight='bold')
    
    for robot_count in [6, 10, 16]:
        config_key = (automaton_id, robot_count)
        if config_key in config_data:
            data = config_data[config_key]
            ax.plot(data['homogeneities'], data['computation_times'], 
                    marker=robot_markers[robot_count], markersize=8, 
                    linewidth=2.5, color=robot_colors[robot_count],
                    label=f'{robot_count} Robots')
    
    ax.set_xlabel('Robot Homogeneity (Independent Caps / Num Robots)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Computation Time (ms)', fontsize=12, fontweight='bold')
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
                 fontsize=16, fontweight='bold')
    
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
    
    ax.set_xlabel('Robot Homogeneity (Independent Caps / Num Robots)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Makespan (seconds)', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=9)
    
    plt.tight_layout()
    plt.savefig(f'Plots/automaton_{automaton_id}_makespan_combined.png', dpi=300, bbox_inches='tight')
    print(f"✓ Plot saved as Plots/automaton_{automaton_id}_makespan_combined.png")
    plt.close(fig)

print("\n✓ All plots generated successfully!")
