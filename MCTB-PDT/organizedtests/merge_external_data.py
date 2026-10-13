#!/usr/bin/env python3
"""
Put the sampler and product columns back into the sweep CSVs after a tree-only run.

The sweeps now measure the tree planner and nothing else. The other two planners still belong in
the same tables, so they are joined in here from where they actually live:

  sampler   budget_study/data/<sweep>.csv at the 100x budget - the same multiple of the tree's
            runtime the sweeps used to apply - averaged over the budget study's trials. Quoting
            an anytime method needs a stated budget, and the budget study states five of them.
  product   comparison/product_archive.csv, frozen by archive_product_data.py. The product
            automaton is exact and deterministic, so a recorded value does not go stale when the
            tree is re-measured around it.

The point of doing this as a merge rather than inside the sweeps is that the sweeps would
otherwise have to read the budget study, which reads the sweeps - a cycle that cannot be run from
a clean checkout. As a post-processing step the order is just: sweeps, budget study, merge.

Every plot script and the product-efficiency table read these CSVs unchanged.

Run from the organizedtests directory:  python3 merge_external_data.py
"""

import csv
import glob
import os
import statistics

BUDGET_DATA = os.path.join("..", "budget_study", "data")
PRODUCT_ARCHIVE = os.path.join("comparison", "product_archive.csv")
TOP_BUDGET = 100.0

# Per-sweep parameter columns, in the order each CSV carries them - the join key
KEY_COLUMNS = {
    "automaton_states": ["automaton_id", "num_robots"],
    "automaton_states_batch": ["automaton_id", "num_robots"],
    "number_robots": ["automaton_id", "num_robots"],
    "transition_system_regions": ["automaton_id", "num_ts_regions"],
    "average_capabilities": ["automaton_id", "num_robots", "average_capabilities"],
    "robot_homogeneity": ["automaton_id", "num_robots", "robot_homogeneity"],
}

PRODUCT_COLUMNS = [
    "full_product_automaton_nodes", "full_product_automaton_edges",
    "full_product_automaton_memory_MB", "product_computation_time_ms",
    "product_makespan_seconds", "explored_product_ratio", "tree_product_ratio",
    "memory_reduction_ratio", "optimality_gap_percent", "runtime_speedup",
    "state_space_reduction",
]


def params_key(values):
    return ";".join(values)


# --------------------------------------------------------------------------- sources
def load_sampler():
    """The budget study's 100x rows, averaged per configuration."""
    table = {}
    for path in sorted(glob.glob(os.path.join(BUDGET_DATA, "*.csv"))):
        sweep = os.path.splitext(os.path.basename(path))[0]
        if sweep not in KEY_COLUMNS:
            continue                      # sampler_wins.csv and friends are not measurements
        groups = {}
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                try:
                    if float(row["budget_multiplier"]) != TOP_BUDGET:
                        continue
                    makespan = float(row["sampler_makespan"])
                except (KeyError, TypeError, ValueError):
                    continue
                if makespan <= 0:
                    continue              # the sampler found nothing; not a makespan
                key = params_key([row[c] for c in KEY_COLUMNS[sweep]])
                groups.setdefault(key, []).append(row)

        for key, rows in groups.items():
            makespans = [float(r["sampler_makespan"]) for r in rows]
            seconds = [float(r["sampler_time_s"]) for r in rows]
            table[(sweep, key)] = {
                "random_sampling_makespan_seconds": statistics.mean(makespans),
                "random_sampling_computation_time_seconds": statistics.mean(seconds),
                "random_sampling_iterations":
                    int(round(statistics.mean(float(r["sampler_iterations"]) for r in rows))),
                "random_sampling_memory_MB":
                    statistics.mean(float(r["sampler_memory_MB"]) for r in rows),
                "random_sampling_budget_multiplier": TOP_BUDGET,
                "random_sampling_makespan_stddev":
                    statistics.pstdev(makespans) if len(makespans) > 1 else 0.0,
                "random_sampling_computation_time_stddev":
                    statistics.pstdev(seconds) if len(seconds) > 1 else 0.0,
                "_trials": len(rows),
            }
    return table


def load_product():
    if not os.path.exists(PRODUCT_ARCHIVE):
        return {}
    table = {}
    with open(PRODUCT_ARCHIVE, newline="") as f:
        for row in csv.DictReader(f):
            key = ";".join(part.split("=", 1)[1] for part in row["params"].split(";"))
            table[(row["sweep"], key)] = {c: row.get(c, "") for c in PRODUCT_COLUMNS}
    return table


# --------------------------------------------------------------------------- wide CSVs
def merge_wide(path, sweep, sampler, product):
    """Fill the sampler and product columns of one per-configuration CSV.

    Rewritten positionally, not through DictWriter: these headers carry `num_robots` twice, and a
    dict round-trip would silently collapse the duplicate and shift every later column.
    """
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    if len(rows) < 2:
        return 0, 0

    header = rows[0]

    def index_of(name):
        return header.index(name) if name in header else None

    key_idx = [index_of(c) for c in KEY_COLUMNS[sweep]]
    if any(i is None for i in key_idx):
        return 0, 0

    matched_sampler = matched_product = 0
    for row in rows[1:]:
        if len(row) < len(header):
            continue
        key = params_key([row[i] for i in key_idx])

        found = sampler.get((sweep, key))
        if found:
            for column, value in found.items():
                if column.startswith("_"):
                    continue
                at = index_of(column)
                if at is not None:
                    row[at] = ("%g" % value) if isinstance(value, float) else str(value)
            matched_sampler += 1

        found = product.get((sweep, key))
        if found:
            for column, value in found.items():
                at = index_of(column)
                if at is not None and value != "":
                    row[at] = value
            matched_product += 1

    with open(path, "w", newline="") as f:
        csv.writer(f).writerows(rows)
    return matched_sampler, matched_product


# --------------------------------------------------------------------------- long CSV
def merge_long(path, sweep, sampler, product):
    """Re-add the sampler and product rows to combined_long.csv, one row per (run, method).

    The sweeps now store tree runs only, so this file comes out with a single method. The
    comparison figures expect all three, and `makespan_ratio` is relative to the best makespan on
    that instance, so it has to be recomputed once the other two are back.
    """
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
        fields = rows[0].keys() if rows else []
    if not rows:
        return 0

    fields = list(fields)
    tree_rows = [r for r in rows if r.get("method") == "tree"]
    if not tree_rows:
        return 0

    # The long format does not carry the sweep's parameter columns under their own names, so the
    # join goes through the wide CSVs on the descriptor columns both exports share.
    #
    # Not on run_id: exportCombinedCSV emits a row per stored run while exportByConfiguration
    # emits one per distinct configuration, and four of the sweeps store each configuration
    # several times. Keying on run_id would match only the handful that happen to appear in both.
    DESCRIPTOR = ["num_robots", "num_automaton_states", "num_automaton_edges", "num_ts_regions",
                  "avg_capabilities_per_robot", "capability_homogeneity"]

    wide_key = {}
    for wide_path in sorted(glob.glob(os.path.join(os.path.dirname(path), "*.csv"))):
        if os.path.basename(wide_path) in ("combined_long.csv", "statistics.csv"):
            continue
        with open(wide_path, newline="") as f:
            reader = csv.reader(f)
            header = next(reader, None)
            if not header:
                continue
            key_idx = [header.index(c) for c in KEY_COLUMNS[sweep] if c in header]
            if len(key_idx) != len(KEY_COLUMNS[sweep]):
                continue
            # The duplicated num_robots column means index() finds the first; the long export
            # writes the same value, so either occurrence joins correctly
            desc_idx = [header.index(c) for c in DESCRIPTOR if c in header]
            if len(desc_idx) != len(DESCRIPTOR):
                continue
            for row in reader:
                if len(row) <= max(key_idx + desc_idx):
                    continue
                wide_key[tuple(row[i] for i in desc_idx)] = params_key(
                    [row[i] for i in key_idx])

    def descriptor_of(row):
        return tuple(row.get(c, "") for c in DESCRIPTOR)

    added = 0
    rebuilt = []
    for tree_row in tree_rows:
        key = wide_key.get(descriptor_of(tree_row))
        group = [tree_row]

        found = product.get((sweep, key)) if key else None
        if found:
            makespan = float(found.get("product_makespan_seconds") or 0.0)
            row = dict(tree_row)
            row["method"] = "product"
            row["computation_time_seconds"] = "%g" % (
                float(found.get("product_computation_time_ms") or 0.0) / 1000.0)
            row["makespan"] = "%g" % makespan
            row["memory_MB"] = found.get("full_product_automaton_memory_MB", "0")
            row["solved"] = "1" if makespan > 0 else "0"
            row["sampler_budget_multiplier"] = "0"
            group.append(row)

        found = sampler.get((sweep, key)) if key else None
        if found:
            row = dict(tree_row)
            row["method"] = "sampler"
            row["computation_time_seconds"] = "%g" % found[
                "random_sampling_computation_time_seconds"]
            row["makespan"] = "%g" % found["random_sampling_makespan_seconds"]
            row["memory_MB"] = "%g" % found["random_sampling_memory_MB"]
            row["solved"] = "1"
            row["sampler_budget_multiplier"] = "%g" % TOP_BUDGET
            group.append(row)
            added += 1

        # makespan_ratio is relative to the best plan anyone found on this instance
        solved = [float(r["makespan"]) for r in group
                  if r.get("solved") == "1" and float(r["makespan"] or 0) > 0]
        best = min(solved) if solved else 0.0
        for row in group:
            value = float(row["makespan"] or 0)
            row["makespan_ratio"] = ("%g" % (value / best)) if best > 0 and value > 0 else ""
        rebuilt.extend(group)

    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rebuilt)
    return added


# --------------------------------------------------------------------------- driver
def main():
    sampler = load_sampler()
    product = load_product()
    print("Loaded %d sampler configurations at the %gx budget and %d archived products.\n"
          % (len(sampler), TOP_BUDGET, len(product)))

    total_s = total_p = total_long = 0
    for sweep in sorted(KEY_COLUMNS):
        data_dir = os.path.join(sweep, "data")
        if not os.path.isdir(data_dir):
            continue
        swept_s = swept_p = 0
        for path in sorted(glob.glob(os.path.join(data_dir, "*.csv"))):
            name = os.path.basename(path)
            if name in ("statistics.csv", "combined_long.csv"):
                continue
            matched_s, matched_p = merge_wide(path, sweep, sampler, product)
            swept_s += matched_s
            swept_p += matched_p

        long_path = os.path.join(data_dir, "combined_long.csv")
        long_added = merge_long(long_path, sweep, sampler, product) \
            if os.path.exists(long_path) else 0

        print("  %-28s sampler %4d rows, product %3d rows, combined_long +%d"
              % (sweep, swept_s, swept_p, long_added))
        total_s += swept_s
        total_p += swept_p
        total_long += long_added

    print("\nFilled %d sampler and %d product rows across the sweeps." % (total_s, total_p))
    if total_s == 0:
        print("No sampler rows matched. Has the budget study been run for these configurations?")


if __name__ == "__main__":
    main()
