#!/usr/bin/env python3
"""
Freeze the product automaton numbers before the sweeps stop computing them.

The sweeps now run only the tree planner: the product automaton cost hours and only ever built on
a handful of small configurations, and the sampler moved to the budget study. But the
product-efficiency table still needs those numbers, and they will not be regenerated.

They do not need to be. The product automaton is exact and deterministic - for a given
configuration it has one answer - so a recorded value stays valid however many times the tree is
re-measured around it. This script lifts every row where a product was actually built into one
archive file, keyed by sweep and configuration, and merge_external_data.py puts those columns
back into the regenerated CSVs.

Run from the organizedtests directory:  python3 archive_product_data.py
"""

import csv
import glob
import os

ARCHIVE = os.path.join("comparison", "product_archive.csv")

# The columns the product-efficiency table and the plots read back
PRODUCT_COLUMNS = [
    "full_product_automaton_nodes",
    "full_product_automaton_edges",
    "full_product_automaton_memory_MB",
    "product_computation_time_ms",
    "product_makespan_seconds",
    "explored_product_ratio",
    "tree_product_ratio",
    "memory_reduction_ratio",
    "optimality_gap_percent",
    "runtime_speedup",
    "state_space_reduction",
]

# Per-sweep parameter columns, in the order each sweep's CSV carries them. The join key.
KEY_COLUMNS = {
    "automaton_states": ["automaton_id", "num_robots"],
    "automaton_states_batch": ["automaton_id", "num_robots"],
    "number_robots": ["automaton_id", "num_robots"],
    "transition_system_regions": ["automaton_id", "num_ts_regions"],
    "average_capabilities": ["automaton_id", "num_robots", "average_capabilities"],
    "robot_homogeneity": ["automaton_id", "num_robots", "robot_homogeneity"],
}


def number(row, column):
    try:
        return float(row.get(column, "") or 0.0)
    except ValueError:
        return 0.0


def main():
    records = {}
    skipped_no_product = 0

    for path in sorted(glob.glob(os.path.join("*", "data", "*.csv"))):
        sweep = path.split(os.sep)[0]
        if sweep not in KEY_COLUMNS:
            continue
        if os.path.basename(path) in ("statistics.csv", "combined_long.csv"):
            continue

        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                # A row with no product is one the size guard skipped - nothing to preserve
                if number(row, "full_product_automaton_nodes") <= 0:
                    skipped_no_product += 1
                    continue

                key = [sweep] + [row.get(c, "") for c in KEY_COLUMNS[sweep]]
                identity = tuple(key)
                # The same configuration appears in more than one file in some sweeps; the
                # product figures are identical across those rows, so the first is enough
                if identity in records:
                    continue

                record = {
                    "sweep": sweep,
                    "params": ";".join("%s=%s" % (c, row.get(c, ""))
                                       for c in KEY_COLUMNS[sweep]),
                }
                for column in PRODUCT_COLUMNS:
                    record[column] = row.get(column, "")
                records[identity] = record

    if not records:
        raise SystemExit("No product rows found. Has the data already been regenerated? "
                         "Restore from an organizedtests/_backup_* directory first.")

    os.makedirs("comparison", exist_ok=True)
    fields = ["sweep", "params"] + PRODUCT_COLUMNS
    with open(ARCHIVE, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for identity in sorted(records):
            writer.writerow(records[identity])

    by_sweep = {}
    for record in records.values():
        by_sweep[record["sweep"]] = by_sweep.get(record["sweep"], 0) + 1

    print("Archived %d configurations that actually built a product:" % len(records))
    for sweep in sorted(by_sweep):
        print("  %-28s %d" % (sweep, by_sweep[sweep]))
    print("\n%d rows had no product (skipped by the size guard) and were ignored."
          % skipped_no_product)
    print("Wrote %s" % ARCHIVE)


if __name__ == "__main__":
    main()
