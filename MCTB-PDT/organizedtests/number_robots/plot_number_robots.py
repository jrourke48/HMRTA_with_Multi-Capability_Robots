#!/usr/bin/env python3
"""
Plot number of robots vs computation time and makespan from TestRunManager CSV exports
Reads from /data
Shows scaling across 6 automata with varying robot counts
"""

import matplotlib.pyplot as plt
import os
import glob
import csv

# Create Plots directory if it doesn't exist
os.makedirs('Plots', exist_ok=True)

# Get all CSV files from data folder
output_dir = "data"
csv_files = glob.glob(os.path.join(output_dir, "num_robots_automaton_id_*.csv"))

if not csv_files:
    print(f"Error: No CSV files found in {output_dir}/")
    exit(1)

# Parse data from CSV files - organized by automaton_id
automaton_data = {}

for csv_file in sorted(csv_files):
    filename = os.path.basename(csv_file)
    # Extract automaton_id from filename
    # Format: number_robots_automaton_id_N.csv
    automaton_id = int(filename.replace("num_robots_automaton_id_", "").replace(".csv", ""))
    automaton_data[automaton_id] = {
        'robot_counts': [],
        'computation_times': [],
        'makespans': [],
        'product_makespans': []
    }
    
    # Read CSV file
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        rows_list = list(reader)
        
    # Sort by num_robots to ensure ordering
    rows_list.sort(key=lambda x: int(x['num_robots']))
    
    for row in rows_list:
        robot_count = int(row['num_robots'])
        computation_time = float(row['total_computation_time_ms'])
        makespan = float(row['tree_makespan_seconds']) if 'tree_makespan_seconds' in row and row['tree_makespan_seconds'].strip() else 0
        product_makespan = float(row['product_makespan_seconds']) if 'product_makespan_seconds' in row and row['product_makespan_seconds'].strip() else 0
        
        automaton_data[automaton_id]['robot_counts'].append(robot_count)
        automaton_data[automaton_id]['computation_times'].append(computation_time)
        automaton_data[automaton_id]['makespans'].append(makespan)
        automaton_data[automaton_id]['product_makespans'].append(product_makespan)

# Define colors and markers for each automaton
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b',
          '#e377c2', '#7f7f7f', '#bcbd22', '#17becf', '#ff9896', '#c5b0d5',
          '#c49c94', '#f7b6d2', '#7f7f7f', '#9467bd']
markers = ['o', 's', '^', 'D', 'v', 'p', '*', 'h', 'H', '+', 'x', 'X', 'd', '|', '_', '.']

# ============================================================================
# FIGURE 1: Computation Time vs Number of Robots (All Automata)
# ============================================================================
fig1, ax1 = plt.subplots(figsize=(12, 7))
fig1.suptitle('Number of Robots vs Computation Time', fontsize=16, fontweight='bold')

for automaton_id in sorted(automaton_data.keys()):
    data = automaton_data[automaton_id]
    ax1.plot(data['robot_counts'], data['computation_times'], 
             marker=markers[automaton_id-1], markersize=8, 
             linewidth=2.5, color=colors[automaton_id-1], 
             label=f'Automaton {automaton_id}')
    
    # Add value labels on points
    for r, t in zip(data['robot_counts'], data['computation_times']):
        ax1.text(r, t, f'{t:.2f}ms', ha='center', va='bottom', fontsize=9)

ax1.set_xlabel('Number of Robots', fontsize=12, fontweight='bold')
ax1.set_ylabel('Computation Time (ms)', fontsize=12, fontweight='bold')
ax1.grid(True, alpha=0.3)
ax1.legend(loc='best', fontsize=10)

plt.tight_layout()
plt.savefig('Plots/number_robots_computation_time.png', dpi=300, bbox_inches='tight')
print("✓ Plot saved as Plots/number_robots_computation_time.png")
plt.close(fig1)

# ============================================================================
# FIGURE 2: Makespan vs Number of Robots (All Automata)
# ============================================================================
fig2, ax2 = plt.subplots(figsize=(12, 7))
fig2.suptitle('Number of Robots vs Makespan', fontsize=16, fontweight='bold')

for automaton_id in sorted(automaton_data.keys()):
    data = automaton_data[automaton_id]
    ax2.plot(data['robot_counts'], data['makespans'], 
             marker=markers[automaton_id-1], markersize=8, 
             linewidth=2.5, color=colors[automaton_id-1], 
             label=f'Automaton {automaton_id} (Task Allocation)')
    
    # Add product makespan overlay if non-zero values exist
    if any(pm > 0 for pm in data['product_makespans']):
        ax2.plot(data['robot_counts'], data['product_makespans'], 
                marker=markers[automaton_id-1], markersize=6, 
                linewidth=2.5, color=colors[automaton_id-1], linestyle='--',
                label=f'Automaton {automaton_id} (Product)')
    
    # Add value labels on points
    for r, m in zip(data['robot_counts'], data['makespans']):
        if m > 0:
            ax2.text(r, m, f'{m:.0f}s', ha='center', va='bottom', fontsize=8)

ax2.set_xlabel('Number of Robots', fontsize=12, fontweight='bold')
ax2.set_ylabel('Makespan (seconds)', fontsize=12, fontweight='bold')
ax2.grid(True, alpha=0.3)
ax2.legend(loc='best', fontsize=9)

plt.tight_layout()
plt.savefig('Plots/number_robots_makespan.png', dpi=300, bbox_inches='tight')
print("✓ Plot saved as Plots/number_robots_makespan.png")
plt.close(fig2)

# ============================================================================
# FIGURE 3-8: Individual Automaton Plots (Computation Time)
# ============================================================================
for automaton_id in sorted(automaton_data.keys()):
    data = automaton_data[automaton_id]
    fig, ax = plt.subplots(figsize=(10, 6))
    fig.suptitle(f'Automaton {automaton_id}: Number of Robots vs Computation Time', 
                 fontsize=14, fontweight='bold')
    
    ax.plot(data['robot_counts'], data['computation_times'], 
            marker=markers[automaton_id-1], markersize=10, 
            linewidth=2.5, color=colors[automaton_id-1])
    ax.set_xlabel('Number of Robots', fontsize=12, fontweight='bold')
    ax.set_ylabel('Computation Time (ms)', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    # Add value labels
    for r, t in zip(data['robot_counts'], data['computation_times']):
        ax.text(r, t, f'{t:.2f}ms', ha='center', va='bottom', fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(f'Plots/automaton_{automaton_id}_computation_time.png', dpi=300, bbox_inches='tight')
    print(f"✓ Plot saved as Plots/automaton_{automaton_id}_computation_time.png")
    plt.close(fig)

# ============================================================================
# FIGURE 9-14: Individual Automaton Plots (Makespan)
# ============================================================================
for automaton_id in sorted(automaton_data.keys()):
    data = automaton_data[automaton_id]
    if any(m > 0 for m in data['makespans']):
        fig, ax = plt.subplots(figsize=(10, 6))
        fig.suptitle(f'Automaton {automaton_id}: Number of Robots vs Makespan', 
                     fontsize=14, fontweight='bold')
        
        ax.plot(data['robot_counts'], data['makespans'], 
                marker=markers[automaton_id-1], markersize=10, 
                linewidth=2.5, color=colors[automaton_id-1], 
                label='Task Allocation Makespan')
        
        # Add product makespan overlay if non-zero values exist
        if any(pm > 0 for pm in data['product_makespans']):
            ax.plot(data['robot_counts'], data['product_makespans'], 
                    marker='D', markersize=8, 
                    linewidth=2.5, color='#d62728', linestyle='--',
                    label='Product Automaton Makespan')
        
        ax.set_xlabel('Number of Robots', fontsize=12, fontweight='bold')
        ax.set_ylabel('Makespan (seconds)', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=11, loc='best')
        
        # Add value labels
        for r, m in zip(data['robot_counts'], data['makespans']):
            if m > 0:
                ax.text(r, m, f'{m:.0f}s', ha='center', va='bottom', fontweight='bold')
        
        plt.tight_layout()
        plt.savefig(f'Plots/automaton_{automaton_id}_makespan.png', dpi=300, bbox_inches='tight')
        print(f"✓ Plot saved as Plots/automaton_{automaton_id}_makespan.png")
        plt.close(fig)

print("\n✓ All plots generated successfully!")
