#!/usr/bin/env python3
"""
Which input variables actually drive the makespan?

That is the question the sweeps exist to answer, and it cannot be read off a single correlation
table. The sweeps are one-factor-at-a-time designs over different grids, so pooling them mixes
the designs: a variable can look influential only because it happens to co-vary with the
specification being solved. So this reports three views, and only agreement between them counts:

  pooled          every configuration together. Descriptive, and confounded - read it as "what
                  varies alongside makespan", never as "what causes it".
  within-automaton the specification is held fixed and the fleet varies. This is where a real
                  fleet effect has to show up, because the automaton cannot explain it.
  within-fleet     robot count and capabilities held fixed, the specification varies. The mirror
                  image: where a real specification effect has to show up.

Correlations are Spearman, so a monotone-but-curved relationship is not missed and a single
large instance cannot drag a line around.

Run from the organizedtests directory:  python3 makespan_drivers.py
"""

import csv
import glob
import os
import re
from collections import defaultdict

import numpy as np
from scipy import stats

SWEEPS = ["automaton_states", "number_robots", "transition_system_regions",
          "average_capabilities", "robot_homogeneity"]

# Candidate drivers, grouped by what they describe
SPEC_VARS = [
    ("num_automaton_states", "Buchi states"),
    ("num_automaton_edges", "Buchi edges"),
    ("num_atomic_propositions", "atomic propositions"),
    ("total_required_capabilities", "required capabilities (total)"),
    ("independent_required_capabilities", "required capabilities (distinct)"),
    ("num_ts_regions", "transition-system regions"),
]
FLEET_VARS = [
    ("num_robots", "robots"),
    ("total_robot_capabilities", "fleet capabilities (total)"),
    ("independent_robot_capabilities", "fleet capabilities (distinct)"),
    ("avg_capabilities_per_robot", "capabilities per robot"),
    ("capability_homogeneity", "capability homogeneity"),
]
ALL_VARS = SPEC_VARS + FLEET_VARS

TARGETS = [("tree_makespan_seconds", "tree"),
           ("random_sampling_makespan_seconds", "sampler")]


def atomic_propositions():
    """automaton id -> AP count, per sweep, parsed from each sweep's LTL strings.

    The CSVs never record this, and the same automaton id means different formulas in different
    sweeps, so it has to be counted per sweep from the source the sweep actually compiled.
    """
    table = {}
    for sweep in SWEEPS:
        path = os.path.join(sweep, "test_%s.cpp" % sweep)
        if not os.path.exists(path):
            continue
        source = open(path, encoding="utf-8", errors="replace").read()
        counts = {}
        for match in re.finditer(
                r"BuchiAutomaton\*\s+createTestInfiniteBuchiAutomaton(\d+)\(\)\s*\{", source):
            body = source[match.end():match.end() + 6000]
            formula = re.search(r"string ltl_str\s*=\s*(.*?);\s*\n", body, re.S)
            if formula:
                counts[match.group(1)] = len(set(re.findall(r"p(\d+)", formula.group(1))))
        table[sweep] = counts
    return table


def load():
    """One row per configuration, from the wide CSVs.

    Not combined_long.csv: four sweeps store each configuration five times there, which would
    quintuple every sample size and make every correlation look far more certain than it is.
    """
    ap_table = atomic_propositions()
    rows = []
    for sweep in SWEEPS:
        for path in sorted(glob.glob(os.path.join(sweep, "data", "*.csv"))):
            if os.path.basename(path) in ("statistics.csv", "combined_long.csv"):
                continue
            with open(path, newline="") as f:
                for raw in csv.DictReader(f):
                    row = {"sweep": sweep}
                    try:
                        for column, _ in ALL_VARS + TARGETS:
                            if column == "num_atomic_propositions":
                                continue
                            row[column] = float(raw[column])
                        row["automaton_id"] = raw["automaton_id"]
                    except (KeyError, TypeError, ValueError):
                        continue
                    if row["tree_makespan_seconds"] <= 0:
                        continue
                    row["num_atomic_propositions"] = float(
                        ap_table.get(sweep, {}).get(raw["automaton_id"], "nan"))
                    rows.append(row)
    return rows


def spearman(rows, column, target):
    """Spearman rho and p, or None when the column does not vary within this slice."""
    pairs = [(r[column], r[target]) for r in rows
             if r[column] == r[column] and r[target] == r[target] and r[target] > 0]
    # Four points is a weak correlation on its own, and is meant to be: these slices are read in
    # aggregate through the agreement count, and demanding more would silently discard the
    # narrowest slices, which are exactly the ones that control a confounder most tightly
    if len(pairs) < 4:
        return None
    x = np.array([p[0] for p in pairs])
    y = np.array([p[1] for p in pairs])
    if np.unique(x).size < 2 or np.unique(y).size < 2:
        return None
    rho, p = stats.spearmanr(x, y)
    if rho != rho:
        return None
    return rho, p, len(pairs)


def stars(p):
    return "***" if p < 0.001 else "**" if p < 0.01 else "*" if p < 0.05 else ""


def report_pooled(rows):
    print("=" * 78)
    print("POOLED - every configuration together (confounded; read as association only)")
    print("=" * 78)
    print("%-36s %18s %18s" % ("", "tree", "sampler"))
    for column, label in ALL_VARS:
        cells = []
        for target, _ in TARGETS:
            result = spearman(rows, column, target)
            cells.append("        --        " if result is None
                         else "%+6.2f%-3s  n=%4d" % (result[0], stars(result[1]), result[2]))
        print("%-36s %18s %18s" % (label, cells[0], cells[1]))


def controlled(rows, keyfunc, title, variables):
    """Correlate each variable inside slices where the confounder is held fixed.

    Controlling on every other column at once does not work: the inputs are algebraically linked
    (total fleet capability is robots times capabilities per robot, and the Buchi counts are all
    fixed once the automaton is), so fixing the others fixes the variable under test and nothing
    is left to correlate. Two complementary slices are what the one-factor-at-a-time design
    actually supports: hold the specification fixed and let the fleet move, then the reverse.

    Groups with only one slice are reported rather than dropped - that is how a near-perfect
    effect of region count went missing on the first pass.

    One slice proves nothing, so the agreement count is what to read: how many slices push the
    same way.
    """
    print()
    print("=" * 78)
    print(title)
    print("=" * 78)
    print("%-34s %-28s %-28s" % ("", "tree", "sampler"))
    print("%-34s %-28s %-28s" % ("", "rho   agree   span", "rho   agree   span"))

    for column, label in variables:
        groups = defaultdict(list)
        for row in rows:
            groups[keyfunc(row)].append(row)

        cells = []
        for target, _ in TARGETS:
            values, spans = [], []
            for group in groups.values():
                result = spearman(group, column, target)
                if result is None:
                    continue
                values.append(result[0])
                # How much the makespan actually moves across the slice. A rank correlation
                # cannot tell 105->106 seconds from 700->190, and both look like rho = 1.
                ys = [r[target] for r in group if r[target] > 0]
                if len(ys) > 1 and min(ys) > 0:
                    spans.append(100.0 * (max(ys) - min(ys)) / min(ys))
            if not values:
                cells.append("        --                  ")
                continue
            median = float(np.median(values))
            same = sum(1 for v in values if (v > 0) == (median > 0) and abs(v) > 0.05)
            span = float(np.median(spans)) if spans else float("nan")
            cells.append("%+5.2f  %2d/%-2d  %6.0f%%     " % (median, same, len(values), span))
        print("%-34s %-28s %-28s" % (label, cells[0], cells[1]))


def saturation(rows):
    """Is the makespan floor set by a capability supply/demand ratio rather than by raw counts?

    Team size and capabilities per robot both stop helping part-way through their range, which
    looks like the grid simply ran out of room. It is not: both are routes to the same quantity,
    the fleet's total capability measured against what the specification demands. Expressed as
    that ratio the two saturations become one, and the floor arrives at the same place in every
    sweep and for both planners.

    Makespans are normalised by the best each automaton ever achieves, because absolute travel
    time varies with the automaton and its environment and would otherwise dominate the pooling.
    """
    for row in rows:
        row["supply_demand"] = (row["total_robot_capabilities"]
                                / max(row["total_required_capabilities"], 1e-9))

    best = defaultdict(lambda: {"tree": float("inf"), "sampler": float("inf")})
    for row in rows:
        key = (row["sweep"], row["automaton_id"])
        best[key]["tree"] = min(best[key]["tree"], row["tree_makespan_seconds"])
        if row["random_sampling_makespan_seconds"] > 0:
            best[key]["sampler"] = min(best[key]["sampler"],
                                       row["random_sampling_makespan_seconds"])

    bands = [(0, 0.5, "<0.5"), (0.5, 1.0, "0.5-1"), (1.0, 1.5, "1-1.5"),
             (1.5, 2.0, "1.5-2"), (2.0, 4.0, "2-4"), (4.0, float("inf"), ">4")]

    print()
    print("=" * 78)
    print("CAPABILITY SUPPLY vs DEMAND - fleet capability / capability the specification needs")
    print("=" * 78)
    print("makespan normalised to the best each automaton achieves; 1.00 means at the floor")
    print()
    print("  %-8s %7s %10s %10s" % ("ratio", "n", "tree", "sampler"))
    for low, high, label in bands:
        chunk = [r for r in rows if low <= r["supply_demand"] < high]
        if not chunk:
            continue
        tree = np.median([r["tree_makespan_seconds"] / best[(r["sweep"], r["automaton_id"])]["tree"]
                          for r in chunk])
        sampled = [r["random_sampling_makespan_seconds"]
                   / best[(r["sweep"], r["automaton_id"])]["sampler"]
                   for r in chunk if r["random_sampling_makespan_seconds"] > 0
                   and best[(r["sweep"], r["automaton_id"])]["sampler"] < float("inf")]
        print("  %-8s %7d %10.2f %10.2f"
              % (label, len(chunk), tree, np.median(sampled) if sampled else float("nan")))

    short = [r for r in rows
             if r["independent_robot_capabilities"] < r["independent_required_capabilities"]]
    print("\n  Configurations whose fleet lacks a required capability type: %d of %d."
          % (len(short), len(rows)))
    print("  Feasibility is therefore never the constraint - what the ratio measures is how many")
    print("  tasks the fleet can staff at once rather than one after another.")


def main():
    rows = load()
    print("%d configurations across %d sweeps\n" % (len(rows), len(SWEEPS)))

    report_pooled(rows)

    # Specification fixed: whatever is left is the fleet talking. Region count belongs here too -
    # the transition-system sweep varies it with one automaton and one fleet throughout.
    controlled(rows, lambda r: (r["sweep"], r["automaton_id"]),
               "SPECIFICATION FIXED - fleet (and region count) varies within one automaton",
               FLEET_VARS + [("num_ts_regions", "transition-system regions")])

    # Team size fixed as well. The capability measures are per-robot ratios, so their attainable
    # range depends on the robot count - a 16-robot team cannot reach the diversity a 6-robot team
    # does. Holding the specification alone therefore lets team size masquerade as a capability
    # effect, and it does: the apparent diversity effect above disappears here.
    controlled(rows, lambda r: (r["sweep"], r["automaton_id"], r["num_robots"]),
               "SPECIFICATION AND TEAM SIZE FIXED - only the capability mix varies",
               [v for v in FLEET_VARS if v[0] != "num_robots"])

    # Fleet fixed: whatever is left is the specification talking
    controlled(rows, lambda r: (r["sweep"], r["num_robots"], r["total_robot_capabilities"]),
               "FLEET FIXED - specification varies at a fixed robot count and capability budget",
               SPEC_VARS)

    print()
    saturation(rows)

    print()
    print("rho is Spearman; agree = slices pushing the same way; span = median makespan range")
    print("across a slice, as a percentage. A large rho with a small span is rank noise on a")
    print("flat response, not an effect worth reporting.")
    print("'agree' counts groups whose correlation has the same sign as the median and |rho|>0.05.")


if __name__ == "__main__":
    main()
