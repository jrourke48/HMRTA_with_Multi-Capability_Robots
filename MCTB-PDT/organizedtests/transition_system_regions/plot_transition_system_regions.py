#!/usr/bin/env python3
"""
Plot transition system regions vs computation time and makespan from TestRunManager CSV exports
Reads from transition_system_regions/data/
Shows scaling across 6 automata with varying TS regions
"""

import matplotlib.pyplot as plt
import os
import glob
import csv

# Create Plots directory if it doesn't exist
os.makedirs('Plots', exist_ok=True)

# Get all CSV files from data folder
output_dir = "data"
csv_files = glob.glob(os.path.join(output_dir, "ts_regions_automaton_id_*.csv"))

if not csv_files:
    print(f"Error: No CSV files found in {output_dir}/")
    exit(1)

# Parse data from CSV files - organized by automaton_id
automaton_data = {}

for csv_file in sorted(csv_files):
    filename = os.path.basename(csv_file)
    # Extract automaton_id from filename
    # Format: automaton_states_automaton_id_N.csv
    automaton_id = int(filename.replace("ts_regions_automaton_id_", "").replace(".csv", ""))
    automaton_data[automaton_id] = {
        'ts_regions': [],
        'computation_times': [],
        'makespans': []
    }
    
    # Read CSV file
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        rows_list = list(reader)
        
    # Sort by num_ts_regions to ensure ordering
    rows_list.sort(key=lambda x: int(x['num_ts_regions']))
    
    for row in rows_list:
        ts_regions = int(row['num_ts_regions'])
        computation_time = float(row['total_computation_time_ms'])
        makespan = float(row['tree_makespan_seconds']) if 'tree_makespan_seconds' in row and row['tree_makespan_seconds'].strip() else 0
        
        automaton_data[automaton_id]['ts_regions'].append(ts_regions)
        automaton_data[automaton_id]['computation_times'].append(computation_time)
        automaton_data[automaton_id]['makespans'].append(makespan)

# Define colors and markers for each automaton
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b',
          '#e377c2', '#7f7f7f', '#bcbd22', '#17becf', '#ff9896', '#c5b0d5',
          '#c49c94', '#f7b6d2', '#7f7f7f', '#9467bd']
markers = ['o', 's', '^', 'D', 'v', 'p', '*', 'h', 'H', '+', 'x', 'X', 'd', '|', '_', '.']

# ============================================================================
# FIGURE 1: Computation Time vs TS Regions (All Automata)
# ============================================================================
fig1, ax1 = plt.subplots(figsize=(12, 7))
fig1.suptitle('Transition System Regions vs Computation Time', fontsize=16, fontweight='bold')

for automaton_id in sorted(automaton_data.keys()):
    data = automaton_data[automaton_id]
    ax1.plot(data['ts_regions'], data['computation_times'], 
             marker=markers[automaton_id-1], markersize=8, 
             linewidth=2.5, color=colors[automaton_id-1], 
             label=f'Automaton {automaton_id}')
    
    # Add value labels on points
    for r, t in zip(data['ts_regions'], data['computation_times']):
        ax1.text(r, t, f'{t:.2f}ms', ha='center', va='bottom', fontsize=9)

ax1.set_xlabel('Number of TS Regions', fontsize=12, fontweight='bold')
ax1.set_ylabel('Computation Time (ms)', fontsize=12, fontweight='bold')
ax1.grid(True, alpha=0.3)
ax1.legend(loc='best', fontsize=10)

plt.tight_layout()
plt.savefig('Plots/ts_regions_computation_time.png', dpi=300, bbox_inches='tight')
print("✓ Plot saved as Plots/ts_regions_computation_time.png")
plt.close(fig1)

# ============================================================================
# FIGURE 2: Makespan vs TS Regions (All Automata)
# ============================================================================
fig2, ax2 = plt.subplots(figsize=(12, 7))
fig2.suptitle('Transition System Regions vs Makespan', fontsize=16, fontweight='bold')

for automaton_id in sorted(automaton_data.keys()):
    data = automaton_data[automaton_id]
    ax2.plot(data['ts_regions'], data['makespans'], 
             marker=markers[automaton_id-1], markersize=8, 
             linewidth=2.5, color=colors[automaton_id-1], 
             label=f'Automaton {automaton_id}')
    
    # Add value labels on points
    for r, m in zip(data['ts_regions'], data['makespans']):
        if m > 0:
            ax2.text(r, m, f'{m:.0f}s', ha='center', va='bottom', fontsize=9)

ax2.set_xlabel('Number of TS Regions', fontsize=12, fontweight='bold')
ax2.set_ylabel('Makespan (seconds)', fontsize=12, fontweight='bold')
ax2.grid(True, alpha=0.3)
ax2.legend(loc='best', fontsize=10)

plt.tight_layout()
plt.savefig('Plots/ts_regions_makespan.png', dpi=300, bbox_inches='tight')
print("✓ Plot saved as Plots/ts_regions_makespan.png")
plt.close(fig2)

# ============================================================================
# FIGURE 3-8: Individual Automaton Plots (Computation Time)
# ============================================================================
for automaton_id in sorted(automaton_data.keys()):
    data = automaton_data[automaton_id]
    fig, ax = plt.subplots(figsize=(10, 6))
    fig.suptitle(f'Automaton {automaton_id}: TS Regions vs Computation Time', 
                 fontsize=14, fontweight='bold')
    
    ax.plot(data['ts_regions'], data['computation_times'], 
            marker=markers[automaton_id-1], markersize=10, 
            linewidth=2.5, color=colors[automaton_id-1])
    ax.set_xlabel('Number of TS Regions', fontsize=12, fontweight='bold')
    ax.set_ylabel('Computation Time (ms)', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    # Add value labels
    for r, t in zip(data['ts_regions'], data['computation_times']):
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
        fig.suptitle(f'Automaton {automaton_id}: TS Regions vs Makespan', 
                     fontsize=14, fontweight='bold')
        
        ax.plot(data['ts_regions'], data['makespans'], 
                marker=markers[automaton_id-1], markersize=10, 
                linewidth=2.5, color=colors[automaton_id-1])
        ax.set_xlabel('Number of TS Regions', fontsize=12, fontweight='bold')
        ax.set_ylabel('Makespan (seconds)', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        
        # Add value labels
        for r, m in zip(data['ts_regions'], data['makespans']):
            if m > 0:
                ax.text(r, m, f'{m:.0f}s', ha='center', va='bottom', fontweight='bold')
        
        plt.tight_layout()
        plt.savefig(f'Plots/automaton_{automaton_id}_makespan.png', dpi=300, bbox_inches='tight')
        print(f"✓ Plot saved as Plots/automaton_{automaton_id}_makespan.png")
        plt.close(fig)

print("\n✓ All plots generated successfully!")
