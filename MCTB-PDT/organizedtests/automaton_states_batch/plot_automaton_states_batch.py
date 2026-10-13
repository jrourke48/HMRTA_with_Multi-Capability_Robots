#!/usr/bin/env python3
"""
Plot automaton states vs computation time from TestRunManager CSV exports
Reads from data/ folder for batch configuration test results
Supports 4 batch configurations: US, CS, ES, and ES+CS
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

# Get all CSV files from data folder with batch configuration pattern
output_dir = "data"
csv_files = glob.glob(os.path.join(output_dir, "automaton_states_batch_Batch_Configuration_*.csv"))

if not csv_files:
    print(f"Error: No CSV files found in {output_dir}/")
    print(f"Looking for pattern: automaton_states_batch_Batch_Configuration_*.csv")
    exit(1)

# Parse data from CSV files - batch configurations
data_us = {}      # Unrelated tasks
data_cs = {}      # Compatible tasks
data_es = {}      # Exclusive tasks
data_es_cs = {}   # Mixed ES+CS

for csv_file in sorted(csv_files):
    filename = os.path.basename(csv_file)
    # Extract configuration number from filename
    # Format: automaton_states_batch_Batch_Configuration_N.csv
    config_num = int(filename.replace("automaton_states_batch_Batch_Configuration_", "").replace(".csv", ""))
    
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
                'product_makespan': float(row['product_makespan_seconds']) if 'product_makespan_seconds' in row and row['product_makespan_seconds'] else 0
            }
            
            # Store in appropriate dict based on configuration
            if config_num == 1:
                data_us[automaton_id] = metrics
            elif config_num == 2:
                data_cs[automaton_id] = metrics
            elif config_num == 3:
                data_es[automaton_id] = metrics
            elif config_num == 4:
                data_es_cs[automaton_id] = metrics

# Extract data for plotting
automaton_ids_us = sorted(data_us.keys())
automaton_ids_cs = sorted(data_cs.keys())
automaton_ids_es = sorted(data_es.keys())
automaton_ids_es_cs = sorted(data_es_cs.keys())

times_us = [data_us[aid]["total_computation_time_ms"] for aid in automaton_ids_us]
states_us = [data_us[aid]["automaton_states"] for aid in automaton_ids_us]
makespans_us = [data_us[aid]["makespan"] for aid in automaton_ids_us]
product_makespans_us = [data_us[aid]["product_makespan"] for aid in automaton_ids_us]

times_cs = [data_cs[aid]["total_computation_time_ms"] for aid in automaton_ids_cs]
states_cs = [data_cs[aid]["automaton_states"] for aid in automaton_ids_cs]
makespans_cs = [data_cs[aid]["makespan"] for aid in automaton_ids_cs]
product_makespans_cs = [data_cs[aid]["product_makespan"] for aid in automaton_ids_cs]

times_es = [data_es[aid]["total_computation_time_ms"] for aid in automaton_ids_es]
states_es = [data_es[aid]["automaton_states"] for aid in automaton_ids_es]
makespans_es = [data_es[aid]["makespan"] for aid in automaton_ids_es]
product_makespans_es = [data_es[aid]["product_makespan"] for aid in automaton_ids_es]

times_es_cs = [data_es_cs[aid]["total_computation_time_ms"] for aid in automaton_ids_es_cs]
states_es_cs = [data_es_cs[aid]["automaton_states"] for aid in automaton_ids_es_cs]
makespans_es_cs = [data_es_cs[aid]["makespan"] for aid in automaton_ids_es_cs]
product_makespans_es_cs = [data_es_cs[aid]["product_makespan"] for aid in automaton_ids_es_cs]

# Create separate figures for each batch configuration
# ============================================================================
# FIGURE 1: Unrelated Tasks (US)
# ============================================================================
if data_us:
    fig1, ax1 = plt.subplots(figsize=(10, 6))
    fig1.suptitle('Automaton Complexity vs Computation Time (Unrelated Tasks)', fontsize=16)

    ax1.plot(states_us, times_us, marker='o', markersize=8, 
             linewidth=2.5, color='#0072B2', label='US Configuration')
    ax1.set_xlabel('Number of Automaton States', fontsize=12)
    ax1.set_ylabel('Computation Time (ms)', fontsize=12)
    ax1.grid(True, alpha=0.3)

    # Add value labels on points
    for i, (s, t) in enumerate(zip(states_us, times_us)):
        ax1.text(s, t, f'{t:.2f}ms', ha='center', va='bottom')

    plt.tight_layout()
    plt.savefig('Plots/automaton_computation_time_us.png', dpi=300, bbox_inches='tight')
    print("✓ Plot saved as Plots/automaton_computation_time_us.png")
    plt.close(fig1)

# ============================================================================
# FIGURE 2: Compatible Tasks (CS)
# ============================================================================
if data_cs:
    fig2, ax2 = plt.subplots(figsize=(10, 6))
    fig2.suptitle('Automaton Complexity vs Computation Time (Compatible Tasks)', fontsize=16)

    ax2.plot(states_cs, times_cs, marker='s', markersize=8, 
             linewidth=2.5, color='#D55E00', label='CS Configuration')
    ax2.set_xlabel('Number of Automaton States', fontsize=12)
    ax2.set_ylabel('Computation Time (ms)', fontsize=12)
    ax2.grid(True, alpha=0.3)

    # Add value labels on points
    for i, (s, t) in enumerate(zip(states_cs, times_cs)):
        ax2.text(s, t, f'{t:.2f}ms', ha='center', va='bottom')

    plt.tight_layout()
    plt.savefig('Plots/automaton_computation_time_cs.png', dpi=300, bbox_inches='tight')
    print("✓ Plot saved as Plots/automaton_computation_time_cs.png")
    plt.close(fig2)

# ============================================================================
# FIGURE 3: Exclusive Tasks (ES)
# ============================================================================
if data_es:
    fig3, ax3 = plt.subplots(figsize=(10, 6))
    fig3.suptitle('Automaton Complexity vs Computation Time (Exclusive Tasks)', fontsize=16)

    ax3.plot(states_es, times_es, marker='^', markersize=8, 
             linewidth=2.5, color='#009E73', label='ES Configuration')
    ax3.set_xlabel('Number of Automaton States', fontsize=12)
    ax3.set_ylabel('Computation Time (ms)', fontsize=12)
    ax3.grid(True, alpha=0.3)

    # Add value labels on points
    for i, (s, t) in enumerate(zip(states_es, times_es)):
        ax3.text(s, t, f'{t:.2f}ms', ha='center', va='bottom')

    plt.tight_layout()
    plt.savefig('Plots/automaton_computation_time_es.png', dpi=300, bbox_inches='tight')
    print("✓ Plot saved as Plots/automaton_computation_time_es.png")
    plt.close(fig3)

# ============================================================================
# FIGURE 4: Mixed ES+CS
# ============================================================================
if data_es_cs:
    fig4, ax4 = plt.subplots(figsize=(10, 6))
    fig4.suptitle('Automaton Complexity vs Computation Time (Mixed ES+CS)', fontsize=16)

    ax4.plot(states_es_cs, times_es_cs, marker='D', markersize=8, 
             linewidth=2.5, color='#CC79A7', label='ES+CS Configuration')
    ax4.set_xlabel('Number of Automaton States', fontsize=12)
    ax4.set_ylabel('Computation Time (ms)', fontsize=12)
    ax4.grid(True, alpha=0.3)

    # Add value labels on points
    for i, (s, t) in enumerate(zip(states_es_cs, times_es_cs)):
        ax4.text(s, t, f'{t:.2f}ms', ha='center', va='bottom')

    plt.tight_layout()
    plt.savefig('Plots/automaton_computation_time_es_cs.png', dpi=300, bbox_inches='tight')
    print("✓ Plot saved as Plots/automaton_computation_time_es_cs.png")
    plt.close(fig4)

# ============================================================================
# FIGURE 5: Makespan vs Automaton States - Unrelated Tasks (US)
# ============================================================================
if data_us and any(makespans_us):
    fig5, ax5 = plt.subplots(figsize=(10, 6))
    fig5.suptitle('Automaton Complexity vs Makespan (Unrelated Tasks)', fontsize=16)

    ax5.plot(states_us, makespans_us, marker='o', markersize=8, 
             linewidth=2.5, color='#0072B2', label='US Configuration')
    ax5.set_xlabel('Number of Automaton States', fontsize=12)
    ax5.set_ylabel('Makespan', fontsize=12)
    ax5.grid(True, alpha=0.3)

    # Add value labels on points
    for i, (s, m) in enumerate(zip(states_us, makespans_us)):
        ax5.text(s, m, f'{m:.2f}', ha='center', va='bottom')

    plt.tight_layout()
    plt.savefig('Plots/automaton_makespan_us.png', dpi=300, bbox_inches='tight')
    print("✓ Plot saved as Plots/automaton_makespan_us.png")
    plt.close(fig5)

# ============================================================================
# FIGURE 6: Makespan vs Automaton States - Compatible Tasks (CS)
# ============================================================================
if data_cs and any(makespans_cs):
    fig6, ax6 = plt.subplots(figsize=(10, 6))
    fig6.suptitle('Automaton Complexity vs Makespan (Compatible Tasks)', fontsize=16)

    ax6.plot(states_cs, makespans_cs, marker='s', markersize=8, 
             linewidth=2.5, color='#D55E00', label='CS Configuration')
    ax6.set_xlabel('Number of Automaton States', fontsize=12)
    ax6.set_ylabel('Makespan', fontsize=12)
    ax6.grid(True, alpha=0.3)

    # Add value labels on points
    for i, (s, m) in enumerate(zip(states_cs, makespans_cs)):
        ax6.text(s, m, f'{m:.2f}', ha='center', va='bottom')

    plt.tight_layout()
    plt.savefig('Plots/automaton_makespan_cs.png', dpi=300, bbox_inches='tight')
    print("✓ Plot saved as Plots/automaton_makespan_cs.png")
    plt.close(fig6)

# ============================================================================
# FIGURE 7: Makespan vs Automaton States - Exclusive Tasks (ES)
# ============================================================================
if data_es and any(makespans_es):
    fig7, ax7 = plt.subplots(figsize=(10, 6))
    fig7.suptitle('Automaton Complexity vs Makespan (Exclusive Tasks)', fontsize=16)

    ax7.plot(states_es, makespans_es, marker='^', markersize=8, 
             linewidth=2.5, color='#009E73', label='ES Configuration')
    ax7.set_xlabel('Number of Automaton States', fontsize=12)
    ax7.set_ylabel('Makespan', fontsize=12)
    ax7.grid(True, alpha=0.3)

    # Add value labels on points
    for i, (s, m) in enumerate(zip(states_es, makespans_es)):
        ax7.text(s, m, f'{m:.2f}', ha='center', va='bottom')

    plt.tight_layout()
    plt.savefig('Plots/automaton_makespan_es.png', dpi=300, bbox_inches='tight')
    print("✓ Plot saved as Plots/automaton_makespan_es.png")
    plt.close(fig7)

# ============================================================================
# FIGURE 8: Makespan vs Automaton States - Mixed ES+CS
# ============================================================================
if data_es_cs and any(makespans_es_cs):
    fig8, ax8 = plt.subplots(figsize=(10, 6))
    fig8.suptitle('Automaton Complexity vs Makespan (Mixed ES+CS)', fontsize=16)

    ax8.plot(states_es_cs, makespans_es_cs, marker='D', markersize=8, 
             linewidth=2.5, color='#CC79A7', label='ES+CS Configuration')
    ax8.set_xlabel('Number of Automaton States', fontsize=12)
    ax8.set_ylabel('Makespan', fontsize=12)
    ax8.grid(True, alpha=0.3)

    # Add value labels on points
    for i, (s, m) in enumerate(zip(states_es_cs, makespans_es_cs)):
        ax8.text(s, m, f'{m:.2f}', ha='center', va='bottom')

    plt.tight_layout()
    plt.savefig('Plots/automaton_makespan_es_cs.png', dpi=300, bbox_inches='tight')
    print("✓ Plot saved as Plots/automaton_makespan_es_cs.png")
    plt.close(fig8)

# ============================================================================
# FIGURE 9: Comparison of All Batch Configurations - Computation Time
# ============================================================================
fig5, ax5 = plt.subplots(figsize=(12, 7))
fig5.suptitle('Automaton Complexity vs Computation Time (All Batch Configurations)', fontsize=16)

if data_us:
    ax5.plot(states_us, times_us, marker='o', markersize=8, 
             linewidth=2.5, color='#0072B2', label='US (Unrelated)')
if data_cs:
    ax5.plot(states_cs, times_cs, marker='s', markersize=8, 
             linewidth=2.5, color='#D55E00', label='CS (Compatible)')
if data_es:
    ax5.plot(states_es, times_es, marker='^', markersize=8, 
             linewidth=2.5, color='#009E73', label='ES (Exclusive)')
if data_es_cs:
    ax5.plot(states_es_cs, times_es_cs, marker='D', markersize=8, 
             linewidth=2.5, color='#CC79A7', label='ES+CS (Mixed)')

ax5.set_xlabel('Number of Automaton States', fontsize=12)
ax5.set_ylabel('Computation Time (ms)', fontsize=12)
ax5.legend(fontsize=11, loc='best')
ax5.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('Plots/automaton_computation_time_all_configs.png', dpi=300, bbox_inches='tight')
print("✓ Plot saved as Plots/automaton_computation_time_all_configs.png")
plt.close(fig5)

# ============================================================================
# FIGURE 10: Comparison of All Batch Configurations - Makespan
# ============================================================================
fig9, ax9 = plt.subplots(figsize=(12, 7))
fig9.suptitle('Automaton Complexity vs Makespan (All Batch Configurations)', fontsize=16)

if data_us and any(makespans_us):
    ax9.plot(states_us, makespans_us, marker='o', markersize=8, 
             linewidth=2.5, color='#0072B2', label='US (Unrelated)')
if data_cs and any(makespans_cs):
    ax9.plot(states_cs, makespans_cs, marker='s', markersize=8, 
             linewidth=2.5, color='#D55E00', label='CS (Compatible)')
if data_es and any(makespans_es):
    ax9.plot(states_es, makespans_es, marker='^', markersize=8, 
             linewidth=2.5, color='#009E73', label='ES (Exclusive)')
if data_es_cs and any(makespans_es_cs):
    ax9.plot(states_es_cs, makespans_es_cs, marker='D', markersize=8, 
             linewidth=2.5, color='#CC79A7', label='ES+CS (Mixed)')

ax9.set_xlabel('Number of Automaton States', fontsize=12)
ax9.set_ylabel('Makespan', fontsize=12)
ax9.legend(fontsize=11, loc='best')
ax9.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('Plots/automaton_makespan_all_configs.png', dpi=300, bbox_inches='tight')
print("✓ Plot saved as Plots/automaton_makespan_all_configs.png")
plt.close(fig9)

# Print summary
print("\n" + "="*80)
print("SUMMARY STATISTICS - BATCH CONFIGURATIONS")
print("="*80 + "\n")

if data_us:
    print("Unrelated Tasks (US) Configuration:")
    for aid in automaton_ids_us:
        states = data_us[aid]["automaton_states"]
        t = data_us[aid]["total_computation_time_ms"]
        m = data_us[aid]["makespan"]
        print(f"  Automaton {aid}: {states} states, {t:.2f} ms, makespan: {m:.2f}")

if data_cs:
    print("\nCompatible Tasks (CS) Configuration:")
    for aid in automaton_ids_cs:
        states = data_cs[aid]["automaton_states"]
        t = data_cs[aid]["total_computation_time_ms"]
        m = data_cs[aid]["makespan"]
        print(f"  Automaton {aid}: {states} states, {t:.2f} ms, makespan: {m:.2f}")

if data_es:
    print("\nExclusive Tasks (ES) Configuration:")
    for aid in automaton_ids_es:
        states = data_es[aid]["automaton_states"]
        t = data_es[aid]["total_computation_time_ms"]
        m = data_es[aid]["makespan"]
        print(f"  Automaton {aid}: {states} states, {t:.2f} ms, makespan: {m:.2f}")

if data_es_cs:
    print("\nMixed ES+CS Configuration:")
    for aid in automaton_ids_es_cs:
        states = data_es_cs[aid]["automaton_states"]
        t = data_es_cs[aid]["total_computation_time_ms"]
        m = data_es_cs[aid]["makespan"]
        print(f"  Automaton {aid}: {states} states, {t:.2f} ms, makespan: {m:.2f}")

print("\n" + "="*80 + "\n")
