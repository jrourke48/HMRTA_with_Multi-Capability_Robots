#!/usr/bin/env python3
"""
Plot automaton states vs computation time from TestRunManager CSV exports
Reads from test_results/automaton_states/exports/
Supports 4 robot configurations: 3, 6, 15, and 45 robots
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

# Get all CSV files from exports folder
output_dir = "data"
csv_files = glob.glob(os.path.join(output_dir, "automaton_states_num_robots_*.csv"))

if not csv_files:
    print(f"Error: No CSV files found in {output_dir}/")
    exit(1)

# Parse data from CSV files
data_3robots = {}
data_6robots = {}
data_15robots = {}
data_45robots = {}

for csv_file in sorted(csv_files):
    filename = os.path.basename(csv_file)
    # Extract robot count from filename
    # Format: automaton_states_num_robots_N.csv
    robots = int(filename.replace("automaton_states_num_robots_", "").replace(".csv", ""))
    
    # Read CSV file
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            automaton_id = int(row['automaton_id'])
            
            # Extract key metrics
            metrics = {
                'automaton_states': int(row['num_automaton_states']),
                'total_computation_time_ms': float(row['total_computation_time_ms']),
                'makespan': float(row['tree_makespan_seconds']) if 'tree_makespan_seconds' in row and row['tree_makespan_seconds'] else 0,
                'product_makespan': float(row['product_makespan_seconds']) if 'product_makespan_seconds' in row and row['product_makespan_seconds'] else 0,
                'sampling_makespan': float(row['random_sampling_makespan_seconds'])
                                     if row.get('random_sampling_makespan_seconds') else 0
            }
            
            # Store in appropriate dict
            if robots == 3:
                data_3robots[automaton_id] = metrics
            elif robots == 6:
                data_6robots[automaton_id] = metrics
            elif robots == 15:
                data_15robots[automaton_id] = metrics
            elif robots == 45:
                data_45robots[automaton_id] = metrics

# Extract data for plotting
automaton_ids_3 = sorted(data_3robots.keys())
automaton_ids_6 = sorted(data_6robots.keys())
automaton_ids_15 = sorted(data_15robots.keys())
automaton_ids_45 = sorted(data_45robots.keys())

times_3robots = [data_3robots[aid]["total_computation_time_ms"] for aid in automaton_ids_3]
states_3robots = [data_3robots[aid]["automaton_states"] for aid in automaton_ids_3]
makespans_3robots = [data_3robots[aid]["makespan"] for aid in automaton_ids_3]
product_makespans_3robots = [data_3robots[aid]["product_makespan"] for aid in automaton_ids_3]
sampling_makespans_3robots = [data_3robots[aid]["sampling_makespan"] for aid in automaton_ids_3]

times_6robots = [data_6robots[aid]["total_computation_time_ms"] for aid in automaton_ids_6]
states_6robots = [data_6robots[aid]["automaton_states"] for aid in automaton_ids_6]
makespans_6robots = [data_6robots[aid]["makespan"] for aid in automaton_ids_6]
product_makespans_6robots = [data_6robots[aid]["product_makespan"] for aid in automaton_ids_6]
sampling_makespans_6robots = [data_6robots[aid]["sampling_makespan"] for aid in automaton_ids_6]

times_15robots = [data_15robots[aid]["total_computation_time_ms"] for aid in automaton_ids_15]
states_15robots = [data_15robots[aid]["automaton_states"] for aid in automaton_ids_15]
makespans_15robots = [data_15robots[aid]["makespan"] for aid in automaton_ids_15]
product_makespans_15robots = [data_15robots[aid]["product_makespan"] for aid in automaton_ids_15]
sampling_makespans_15robots = [data_15robots[aid]["sampling_makespan"] for aid in automaton_ids_15]

times_45robots = [data_45robots[aid]["total_computation_time_ms"] for aid in automaton_ids_45]
states_45robots = [data_45robots[aid]["automaton_states"] for aid in automaton_ids_45]
makespans_45robots = [data_45robots[aid]["makespan"] for aid in automaton_ids_45]
product_makespans_45robots = [data_45robots[aid]["product_makespan"] for aid in automaton_ids_45]
sampling_makespans_45robots = [data_45robots[aid]["sampling_makespan"] for aid in automaton_ids_45]

# Create separate figures for each robot count
# ============================================================================
# FIGURE 1: 3-Robot Environment
# ============================================================================
fig1, ax1 = plt.subplots(figsize=(10, 6))
fig1.suptitle('Automaton Complexity vs Computation Time (3-Robot Team)', fontsize=16)

ax1.plot(states_3robots, times_3robots, marker='o', markersize=8, 
         linewidth=2.5, color='#0072B2', label='3-Robot Team')
ax1.set_xlabel('Number of Automaton States', fontsize=12)
ax1.set_ylabel('Computation Time (ms)', fontsize=12)
ax1.grid(True, alpha=0.3)

# Add value labels on points
for i, (s, t) in enumerate(zip(states_3robots, times_3robots)):
    ax1.text(s, t, f'{t:.2f}ms', ha='center', va='bottom')

plt.tight_layout()
plt.savefig('Plots/automaton_computation_time_3robots.png', dpi=300, bbox_inches='tight')
print("✓ Plot saved as Plots/automaton_computation_time_3robots.png")
plt.close(fig1)

# ============================================================================
# FIGURE 2: 6-Robot Environment
# ============================================================================
fig2, ax2 = plt.subplots(figsize=(10, 6))
fig2.suptitle('Automaton Complexity vs Computation Time (6-Robot Team)', fontsize=16)

ax2.plot(states_6robots, times_6robots, marker='s', markersize=8, 
         linewidth=2.5, color='#D55E00', label='6-Robot Team')
ax2.set_xlabel('Number of Automaton States', fontsize=12)
ax2.set_ylabel('Computation Time (ms)', fontsize=12)
ax2.grid(True, alpha=0.3)

# Add value labels on points
for i, (s, t) in enumerate(zip(states_6robots, times_6robots)):
    ax2.text(s, t, f'{t:.2f}ms', ha='center', va='bottom')

plt.tight_layout()
plt.savefig('Plots/automaton_computation_time_6robots.png', dpi=300, bbox_inches='tight')
print("✓ Plot saved as Plots/automaton_computation_time_6robots.png")
plt.close(fig2)

# ============================================================================
# FIGURE 3: 15-Robot Environment
# ============================================================================
fig3, ax3 = plt.subplots(figsize=(10, 6))
fig3.suptitle('Automaton Complexity vs Computation Time (15-Robot Team)', fontsize=16)

ax3.plot(states_15robots, times_15robots, marker='^', markersize=8, 
         linewidth=2.5, color='#009E73', label='15-Robot Team')
ax3.set_xlabel('Number of Automaton States', fontsize=12)
ax3.set_ylabel('Computation Time (ms)', fontsize=12)
ax3.grid(True, alpha=0.3)

# Add value labels on points
for i, (s, t) in enumerate(zip(states_15robots, times_15robots)):
    ax3.text(s, t, f'{t:.2f}ms', ha='center', va='bottom')

plt.tight_layout()
plt.savefig('Plots/automaton_computation_time_15robots.png', dpi=300, bbox_inches='tight')
print("✓ Plot saved as Plots/automaton_computation_time_15robots.png")
plt.close(fig3)

# ============================================================================
# FIGURE 4: 45-Robot Environment
# ============================================================================
fig4, ax4 = plt.subplots(figsize=(10, 6))
fig4.suptitle('Automaton Complexity vs Computation Time (45-Robot Team)', fontsize=16)

ax4.plot(states_45robots, times_45robots, marker='D', markersize=8, 
         linewidth=2.5, color='#CC79A7', label='45-Robot Team')
ax4.set_xlabel('Number of Automaton States', fontsize=12)
ax4.set_ylabel('Computation Time (ms)', fontsize=12)
ax4.grid(True, alpha=0.3)

# Add value labels on points
for i, (s, t) in enumerate(zip(states_45robots, times_45robots)):
    ax4.text(s, t, f'{t:.2f}ms', ha='center', va='bottom')

plt.tight_layout()
plt.savefig('Plots/automaton_computation_time_45robots.png', dpi=300, bbox_inches='tight')
print("✓ Plot saved as Plots/automaton_computation_time_45robots.png")
plt.close(fig4)

# ============================================================================
# FIGURE 5: Makespan vs Automaton States - 3-Robot Environment
# ============================================================================
if any(makespans_3robots):
    fig5, ax5 = plt.subplots(figsize=(10, 6))
    fig5.suptitle('Automaton Complexity vs Makespan (3-Robot Team)', fontsize=16)

    ax5.plot(states_3robots, makespans_3robots, marker='o', markersize=8, 
             linewidth=2.5, color='#0072B2', label='Tree Makespan (Task Allocation)')
    
    # Add product makespan overlay if non-zero values exist (only if within 2x tree makespan)
    filtered_product_makespans_3 = filter_product_makespans(product_makespans_3robots, makespans_3robots)
    if any(pm > 0 for pm in filtered_product_makespans_3):
        ax5.plot(states_3robots, filtered_product_makespans_3, marker='s', markersize=8, 
                 linewidth=2.5, color='#CC79A7', linestyle='--', label='Product Automaton Makespan')
    

    # Sampler runs on every instance, so it is plotted unfiltered wherever it found a plan
    if any(sm > 0 for sm in sampling_makespans_3robots):
        ax5.plot(states_3robots, [sm if sm > 0 else None for sm in sampling_makespans_3robots],
                 marker='^', markersize=8, linewidth=2.5, color='#009E73', linestyle=':',
                 label='Random Sampling Makespan')
    ax5.set_xlabel('Number of Automaton States', fontsize=12)
    ax5.set_ylabel('Makespan (seconds)', fontsize=12)
    ax5.grid(True, alpha=0.3)
    ax5.legend(fontsize=11, loc='best')

    # Add value labels on points
    for i, (s, m) in enumerate(zip(states_3robots, makespans_3robots)):
        ax5.text(s, m, f'{m:.2f}s', ha='center', va='bottom', fontsize=9)

    plt.tight_layout()
    plt.savefig('Plots/automaton_makespan_3robots.png', dpi=300, bbox_inches='tight')
    print("✓ Plot saved as Plots/automaton_makespan_3robots.png")
    plt.close(fig5)

# ============================================================================
# FIGURE 6: Makespan vs Automaton States - 6-Robot Environment
# ============================================================================
if any(makespans_6robots):
    fig6, ax6 = plt.subplots(figsize=(10, 6))
    fig6.suptitle('Automaton Complexity vs Makespan (6-Robot Team)', fontsize=16)

    ax6.plot(states_6robots, makespans_6robots, marker='s', markersize=8, 
             linewidth=2.5, color='#D55E00', label='Tree Makespan (Task Allocation)')
    
    # Add product makespan overlay if non-zero values exist (only if within 2x tree makespan)
    filtered_product_makespans_6 = filter_product_makespans(product_makespans_6robots, makespans_6robots)
    if any(pm > 0 for pm in filtered_product_makespans_6):
        ax6.plot(states_6robots, filtered_product_makespans_6, marker='^', markersize=8, 
                 linewidth=2.5, color='#CC79A7', linestyle='--', label='Product Automaton Makespan')
    

    # Sampler runs on every instance, so it is plotted unfiltered wherever it found a plan
    if any(sm > 0 for sm in sampling_makespans_6robots):
        ax6.plot(states_6robots, [sm if sm > 0 else None for sm in sampling_makespans_6robots],
                 marker='^', markersize=8, linewidth=2.5, color='#009E73', linestyle=':',
                 label='Random Sampling Makespan')
    ax6.set_xlabel('Number of Automaton States', fontsize=12)
    ax6.set_ylabel('Makespan (seconds)', fontsize=12)
    ax6.grid(True, alpha=0.3)
    ax6.legend(fontsize=11, loc='best')

    # Add value labels on points
    for i, (s, m) in enumerate(zip(states_6robots, makespans_6robots)):
        ax6.text(s, m, f'{m:.2f}s', ha='center', va='bottom', fontsize=9)

    plt.tight_layout()
    plt.savefig('Plots/automaton_makespan_6robots.png', dpi=300, bbox_inches='tight')
    print("✓ Plot saved as Plots/automaton_makespan_6robots.png")
    plt.close(fig6)

# ============================================================================
# FIGURE 7: Makespan vs Automaton States - 15-Robot Environment
# ============================================================================
if any(makespans_15robots):
    fig7, ax7 = plt.subplots(figsize=(10, 6))
    fig7.suptitle('Automaton Complexity vs Makespan (15-Robot Team)', fontsize=16)

    ax7.plot(states_15robots, makespans_15robots, marker='^', markersize=8, 
             linewidth=2.5, color='#009E73', label='Tree Makespan (Task Allocation)')
    
    # Add product makespan overlay if non-zero values exist (only if within 2x tree makespan)
    filtered_product_makespans_15 = filter_product_makespans(product_makespans_15robots, makespans_15robots)
    if any(pm > 0 for pm in filtered_product_makespans_15):
        ax7.plot(states_15robots, filtered_product_makespans_15, marker='D', markersize=8, 
                 linewidth=2.5, color='#CC79A7', linestyle='--', label='Product Automaton Makespan')
    

    # Sampler runs on every instance, so it is plotted unfiltered wherever it found a plan
    if any(sm > 0 for sm in sampling_makespans_15robots):
        ax7.plot(states_15robots, [sm if sm > 0 else None for sm in sampling_makespans_15robots],
                 marker='^', markersize=8, linewidth=2.5, color='#009E73', linestyle=':',
                 label='Random Sampling Makespan')
    ax7.set_xlabel('Number of Automaton States', fontsize=12)
    ax7.set_ylabel('Makespan (seconds)', fontsize=12)
    ax7.grid(True, alpha=0.3)
    ax7.legend(fontsize=11, loc='best')

    # Add value labels on points
    for i, (s, m) in enumerate(zip(states_15robots, makespans_15robots)):
        ax7.text(s, m, f'{m:.2f}s', ha='center', va='bottom', fontsize=9)

    plt.tight_layout()
    plt.savefig('Plots/automaton_makespan_15robots.png', dpi=300, bbox_inches='tight')
    print("✓ Plot saved as Plots/automaton_makespan_15robots.png")
    plt.close(fig7)

# ============================================================================
# FIGURE 8: Makespan vs Automaton States - 45-Robot Environment
# ============================================================================
if any(makespans_45robots):
    fig8, ax8 = plt.subplots(figsize=(10, 6))
    fig8.suptitle('Automaton Complexity vs Makespan (45-Robot Team)', fontsize=16)

    ax8.plot(states_45robots, makespans_45robots, marker='D', markersize=8, 
             linewidth=2.5, color='#CC79A7', label='Tree Makespan (Task Allocation)')
    
    # Add product makespan overlay if non-zero values exist (only if within 2x tree makespan)
    filtered_product_makespans_45 = filter_product_makespans(product_makespans_45robots, makespans_45robots)
    if any(pm > 0 for pm in filtered_product_makespans_45):
        ax8.plot(states_45robots, filtered_product_makespans_45, marker='*', markersize=12, 
                 linewidth=2.5, color='#009E73', linestyle='--', label='Product Automaton Makespan')
    

    # Sampler runs on every instance, so it is plotted unfiltered wherever it found a plan
    if any(sm > 0 for sm in sampling_makespans_45robots):
        ax8.plot(states_45robots, [sm if sm > 0 else None for sm in sampling_makespans_45robots],
                 marker='^', markersize=8, linewidth=2.5, color='#009E73', linestyle=':',
                 label='Random Sampling Makespan')
    ax8.set_xlabel('Number of Automaton States', fontsize=12)
    ax8.set_ylabel('Makespan (seconds)', fontsize=12)
    ax8.grid(True, alpha=0.3)
    ax8.legend(fontsize=11, loc='best')

    # Add value labels on points
    for i, (s, m) in enumerate(zip(states_45robots, makespans_45robots)):
        ax8.text(s, m, f'{m:.2f}s', ha='center', va='bottom', fontsize=9)

    plt.tight_layout()
    plt.savefig('Plots/automaton_makespan_45robots.png', dpi=300, bbox_inches='tight')
    print("✓ Plot saved as Plots/automaton_makespan_45robots.png")
    plt.close(fig8)

# ============================================================================
# FIGURE 9: Comparison of All Robot Configurations - Computation Time
# ============================================================================
fig5, ax5 = plt.subplots(figsize=(12, 7))
fig5.suptitle('Automaton Complexity vs Computation Time (All Configurations)', fontsize=16)

ax5.plot(states_3robots, times_3robots, marker='o', markersize=8, 
         linewidth=2.5, color='#0072B2', label='3-Robot Team')
ax5.plot(states_6robots, times_6robots, marker='s', markersize=8, 
         linewidth=2.5, color='#D55E00', label='6-Robot Team')
ax5.plot(states_15robots, times_15robots, marker='^', markersize=8, 
         linewidth=2.5, color='#009E73', label='15-Robot Team')
ax5.plot(states_45robots, times_45robots, marker='D', markersize=8, 
         linewidth=2.5, color='#CC79A7', label='45-Robot Team')

ax5.set_xlabel('Number of Automaton States', fontsize=12)
ax5.set_ylabel('Computation Time (ms)', fontsize=12)
ax5.legend(fontsize=11, loc='best')
ax5.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('Plots/automaton_computation_time_all_configs.png', dpi=300, bbox_inches='tight')
print("✓ Plot saved as Plots/automaton_computation_time_all_configs.png")
plt.close(fig5)

# ============================================================================
# FIGURE 10: Comparison of All Robot Configurations - Makespan
# ============================================================================
fig10, ax10 = plt.subplots(figsize=(12, 7))
fig10.suptitle('Automaton Complexity vs Makespan (All Configurations)', fontsize=16)

if any(makespans_3robots):
    ax10.plot(states_3robots, makespans_3robots, marker='o', markersize=8, 
             linewidth=2.5, color='#0072B2', label='3-Robot (Task Allocation)')
filtered_product_3_combined = filter_product_makespans(product_makespans_3robots, makespans_3robots)
if any(pm > 0 for pm in filtered_product_3_combined):
    ax10.plot(states_3robots, filtered_product_3_combined, marker='o', markersize=6, 
             linewidth=2.5, color='#0072B2', linestyle='--', label='3-Robot (Product)')
if any(sm > 0 for sm in sampling_makespans_3robots):
    ax10.plot(states_3robots, [sm if sm > 0 else None for sm in sampling_makespans_3robots], marker='^',
             linewidth=2, color='#0072B2', linestyle=':', label='3-Robot (Sampler)')
    
if any(makespans_6robots):
    ax10.plot(states_6robots, makespans_6robots, marker='s', markersize=8, 
             linewidth=2.5, color='#D55E00', label='6-Robot (Task Allocation)')
filtered_product_6_combined = filter_product_makespans(product_makespans_6robots, makespans_6robots)
if any(pm > 0 for pm in filtered_product_6_combined):
    ax10.plot(states_6robots, filtered_product_6_combined, marker='s', markersize=6, 
             linewidth=2.5, color='#D55E00', linestyle='--', label='6-Robot (Product)')
if any(sm > 0 for sm in sampling_makespans_6robots):
    ax10.plot(states_6robots, [sm if sm > 0 else None for sm in sampling_makespans_6robots], marker='^',
             linewidth=2, color='#D55E00', linestyle=':', label='6-Robot (Sampler)')
    
if any(makespans_15robots):
    ax10.plot(states_15robots, makespans_15robots, marker='^', markersize=8, 
             linewidth=2.5, color='#009E73', label='15-Robot (Task Allocation)')
filtered_product_15_combined = filter_product_makespans(product_makespans_15robots, makespans_15robots)
if any(pm > 0 for pm in filtered_product_15_combined):
    ax10.plot(states_15robots, filtered_product_15_combined, marker='^', markersize=6, 
             linewidth=2.5, color='#009E73', linestyle='--', label='15-Robot (Product)')
if any(sm > 0 for sm in sampling_makespans_15robots):
    ax10.plot(states_15robots, [sm if sm > 0 else None for sm in sampling_makespans_15robots], marker='D',
             linewidth=2, color='#009E73', linestyle=':', label='15-Robot (Sampler)')
    
if any(makespans_45robots):
    ax10.plot(states_45robots, makespans_45robots, marker='D', markersize=8, 
             linewidth=2.5, color='#CC79A7', label='45-Robot (Task Allocation)')
filtered_product_45_combined = filter_product_makespans(product_makespans_45robots, makespans_45robots)
if any(pm > 0 for pm in filtered_product_45_combined):
    ax10.plot(states_45robots, filtered_product_45_combined, marker='D', markersize=6, 
             linewidth=2.5, color='#CC79A7', linestyle='--', label='45-Robot (Product)')
if any(sm > 0 for sm in sampling_makespans_45robots):
    ax10.plot(states_45robots, [sm if sm > 0 else None for sm in sampling_makespans_45robots], marker='^',
             linewidth=2, color='#CC79A7', linestyle=':', label='45-Robot (Sampler)')

ax10.set_xlabel('Number of Automaton States', fontsize=12)
ax10.set_ylabel('Makespan (seconds)', fontsize=12)
ax10.legend(fontsize=10, loc='best')
ax10.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('Plots/automaton_makespan_all_configs.png', dpi=300, bbox_inches='tight')
print("✓ Plot saved as Plots/automaton_makespan_all_configs.png")
plt.close(fig10)

# Print summary
print("\n" + "="*80)
print("SUMMARY STATISTICS")
print("="*80 + "\n")

print("3-Robot Environment:")
for aid in automaton_ids_3:
    states = data_3robots[aid]["automaton_states"]
    t = data_3robots[aid]["total_computation_time_ms"]
    m = data_3robots[aid]["makespan"]
    print(f"  Automaton {aid}: {states} states, {t:.2f} ms, makespan: {m:.2f}s")

print("\n6-Robot Environment:")
for aid in automaton_ids_6:
    states = data_6robots[aid]["automaton_states"]
    t = data_6robots[aid]["total_computation_time_ms"]
    m = data_6robots[aid]["makespan"]
    print(f"  Automaton {aid}: {states} states, {t:.2f} ms, makespan: {m:.2f}s")

print("\n15-Robot Environment:")
for aid in automaton_ids_15:
    states = data_15robots[aid]["automaton_states"]
    t = data_15robots[aid]["total_computation_time_ms"]
    m = data_15robots[aid]["makespan"]
    print(f"  Automaton {aid}: {states} states, {t:.2f} ms, makespan: {m:.2f}s")

print("\n45-Robot Environment:")
for aid in automaton_ids_45:
    states = data_45robots[aid]["automaton_states"]
    t = data_45robots[aid]["total_computation_time_ms"]
    m = data_45robots[aid]["makespan"]
    print(f"  Automaton {aid}: {states} states, {t:.2f} ms, makespan: {m:.2f}s")

print("\n" + "="*80 + "\n")
