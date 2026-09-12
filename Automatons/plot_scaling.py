#!/usr/bin/env python3
"""
Plot ProductAutomaton scaling analysis
"""
import csv
import glob
import matplotlib.pyplot as plt
import numpy as np
import os

# Read CSV file
csv_file = "output/product_automaton_scaling.csv"

if not os.path.exists(csv_file):
    print(f"Error: {csv_file} not found")
    exit(1)

# Read data from CSV
data = {
    'num_robots': [],
    'product_states': [],
    'product_edges': [],
    'accepting_states': [],
    'state_ratio': []
}

print("ProductAutomaton Scaling Data:")
print("num_robots | product_states | product_edges | accepting_states | state_ratio")
print("-" * 80)

with open(csv_file, newline='') as f:
    reader = csv.DictReader(f)
    for row in reader:
        num_robots = int(row['num_robots'])
        product_states = int(row['product_states'])
        product_edges = int(row['product_edges'])
        accepting_states = int(row['accepting_states'])
        state_ratio = float(row['state_ratio'])
        
        data['num_robots'].append(num_robots)
        data['product_states'].append(product_states)
        data['product_edges'].append(product_edges)
        data['accepting_states'].append(accepting_states)
        data['state_ratio'].append(state_ratio)
        
        print(f"{num_robots:10d} | {product_states:14,d} | {product_edges:13,d} | {accepting_states:16,d} | {state_ratio:9.2f}%")

print()

# Create single plot: Automaton States vs Number of Robots
fig, ax = plt.subplots(figsize=(10, 6))

# Plot actual data
ax.plot(data['num_robots'], data['product_states'], 'o-', linewidth=2, markersize=10, color='#1f77b4', label='Actual States')

# Plot theoretical 5^n
theoretical_5n = [4*5**n for n in data['num_robots']]
ax.plot(data['num_robots'], theoretical_5n, 's--', linewidth=2, markersize=8, color='#d62728', label='NBA*TS^n (Theoretical)')

ax.set_xlabel('Number of Robots', fontsize=12, fontweight='bold')
ax.set_ylabel('States', fontsize=12, fontweight='bold')
ax.set_title('Product Automaton State Growth vs Robot Count', fontsize=14, fontweight='bold')
ax.grid(True, alpha=0.3)
ax.set_xticks(range(1, 7))
ax.set_yscale('log')  # Log scale to better visualize exponential growth
ax.legend(fontsize=11)

# Add value labels on actual data points
for i, (x, y) in enumerate(zip(data['num_robots'], data['product_states'])):
    ax.text(x, y, f"{y:,}", ha='center', va='bottom', fontsize=9, fontweight='bold')

plt.tight_layout()
plt.savefig('output/automaton_scaling_analysis.png', dpi=300, bbox_inches='tight')
print("✓ Saved plot to output/automaton_scaling_analysis.png")

# Show plot
plt.show()

# Print summary statistics
print("\n=== Summary Statistics ===")
print("Product Automaton Growth:")
print(f"  1 robot:  {data['product_states'][0]:,} states")
print(f"  6 robots: {data['product_states'][-1]:,} states")
if data['product_states'][0] > 0:
    growth = data['product_states'][-1] / data['product_states'][0]
    print(f"  Growth factor: {growth:.1f}x")
print()

# Check if states equal 2^16 for 6 robots
if data['product_states'][-1] == 65536:
    print("⚠ 6 robots: product_states = 65536 = 2^16 (state space saturated at 16-bit size)")
    print("  This suggests missing states in the state space representation")
else:
    print(f"Note: 6 robots produce {data['product_states'][-1]:,} states (not 2^16)")

