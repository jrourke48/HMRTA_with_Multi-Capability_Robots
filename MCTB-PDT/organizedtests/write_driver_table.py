#!/usr/bin/env python3
"""
The driver ranking as a table: comparison/makespan_drivers.{csv,tex}.

One row per input variable, with the correlation, how consistently it holds, and - the column
that stops the table lying - how far the makespan actually moves. A variable can post a high
correlation on a response that spans one percent, and without the span beside it that row reads
like a result.

The control column says which slice the row was measured in, because these numbers mean nothing
pooled: region count reads +0.01 across the whole suite and -1.00 once the automaton is fixed.

Run from the organizedtests directory:  python3 write_driver_table.py
"""

import csv
import os
from collections import defaultdict

import numpy as np

import makespan_drivers as md

OUT_DIR = "comparison"
BS = chr(92)   # a literal backslash; writing "\\" here has been mangled by the shell before

# variable, label, control description, grouping key
ROWS = [
    ("num_ts_regions", "Transition-system regions", "automaton",
     lambda r: (r["sweep"], r["automaton_id"])),
    ("total_robot_capabilities", "Fleet capability (total)", "automaton + team size",
     lambda r: (r["sweep"], r["automaton_id"], r["num_robots"])),
    ("num_robots", "Team size", "automaton",
     lambda r: (r["sweep"], r["automaton_id"])),
    ("avg_capabilities_per_robot", "Capabilities per robot", "automaton",
     lambda r: (r["sweep"], r["automaton_id"])),
    ("num_atomic_propositions", "Atomic propositions", "fleet",
     lambda r: (r["sweep"], r["num_robots"], r["total_robot_capabilities"])),
    ("independent_required_capabilities", "Required capabilities (distinct)", "fleet",
     lambda r: (r["sweep"], r["num_robots"], r["total_robot_capabilities"])),
    ("total_required_capabilities", "Required capabilities (total)", "fleet",
     lambda r: (r["sweep"], r["num_robots"], r["total_robot_capabilities"])),
    ("num_automaton_states", "B\\\"uchi states", "fleet",
     lambda r: (r["sweep"], r["num_robots"], r["total_robot_capabilities"])),
    ("independent_robot_capabilities", "Capability diversity", "automaton + team size",
     lambda r: (r["sweep"], r["automaton_id"], r["num_robots"])),
]


def measure(rows, column, keyfunc, target):
    groups = defaultdict(list)
    for row in rows:
        groups[keyfunc(row)].append(row)
    values, spans = [], []
    for group in groups.values():
        result = md.spearman(group, column, target)
        if result is None:
            continue
        values.append(result[0])
        ys = [r[target] for r in group if r[target] > 0]
        if len(ys) > 1 and min(ys) > 0:
            spans.append(100.0 * (max(ys) - min(ys)) / min(ys))
    if not values:
        return None
    median = float(np.median(values))
    agree = sum(1 for v in values if (v > 0) == (median > 0) and abs(v) > 0.05)
    return median, agree, len(values), (float(np.median(spans)) if spans else float("nan"))


def verdict(tree):
    """What the row supports, judged on the effect size rather than the correlation."""
    span = tree[3]
    if span < 10:
        return "no effect"
    if abs(tree[0]) >= 0.8 and tree[1] >= 0.8 * tree[2]:
        return "strong"
    if abs(tree[0]) >= 0.3 and tree[1] >= 0.7 * tree[2]:
        return "moderate"
    return "weak"


def latex(records):
    caption = ("Input variables ranked by their effect on makespan. $" + BS + "rho$ is "
               "Spearman's rank correlation inside slices holding the stated control fixed; "
               "agreement counts the slices sharing the sign; span is the median range of the "
               "makespan across a slice. A large $" + BS + "rho$ on a small span is rank "
               "ordering of noise, not an effect.")
    rule = BS + BS
    lines = [
        BS + "begin{table}[htbp]",
        BS + "centering",
        BS + "caption{" + caption + "}",
        BS + "label{tab:makespan_drivers}",
        BS + "small",
        BS + "begin{tabular}{llrrrrrrl}",
        BS + "toprule",
        ("Variable & Controlled for & " + BS + "multicolumn{3}{c}{Tree planner} & "
         + BS + "multicolumn{3}{c}{Random sampler} & Verdict " + rule),
        BS + "cmidrule(lr){3-5}" + BS + "cmidrule(lr){6-8}",
        (" & & $" + BS + "rho$ & agree & span & $" + BS + "rho$ & agree & span & " + rule),
        BS + "midrule",
    ]
    for r in records:
        lines.append("%s & %s & %s & %s & %s%s%% & %s & %s & %s%s%% & %s %s" % (
            r["variable"], r["controlled_for"], r["tree_rho"], r["tree_agreement"],
            r["tree_span_pct"], BS, r["sampler_rho"], r["sampler_agreement"],
            r["sampler_span_pct"], BS, r["verdict"], rule))
    lines += [BS + "bottomrule", BS + "end{tabular}", BS + "end{table}"]
    return lines


def main():
    rows = md.load()
    os.makedirs(OUT_DIR, exist_ok=True)

    records = []
    for column, label, control, keyfunc in ROWS:
        tree = measure(rows, column, keyfunc, "tree_makespan_seconds")
        samp = measure(rows, column, keyfunc, "random_sampling_makespan_seconds")
        if tree is None:
            continue
        records.append({
            "variable": label,
            "controlled_for": control,
            "tree_rho": "%.2f" % tree[0],
            "tree_agreement": "%d/%d" % (tree[1], tree[2]),
            "tree_span_pct": "%.0f" % tree[3],
            "sampler_rho": "%.2f" % samp[0] if samp else "",
            "sampler_agreement": "%d/%d" % (samp[1], samp[2]) if samp else "",
            "sampler_span_pct": "%.0f" % samp[3] if samp else "",
            "verdict": verdict(tree),
        })
    order = {"strong": 0, "moderate": 1, "weak": 2, "no effect": 3}
    records.sort(key=lambda r: (order[r["verdict"]], -abs(float(r["tree_rho"]))))

    path = os.path.join(OUT_DIR, "makespan_drivers.csv")
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(records[0].keys()))
        writer.writeheader()
        writer.writerows(records)
    print("Wrote %s" % path)

    path = os.path.join(OUT_DIR, "makespan_drivers.tex")
    with open(path, "w") as f:
        f.write("\n".join(latex(records)) + "\n")
    print("Wrote %s\n" % path)

    print("%-34s %-22s %8s %8s %9s  %s"
          % ("variable", "controlled for", "tree rho", "agree", "span", "verdict"))
    for r in records:
        print("%-34s %-22s %8s %8s %8s%%  %s" % (
            r["variable"].replace(BS + '"u', "u"), r["controlled_for"], r["tree_rho"],
            r["tree_agreement"], r["tree_span_pct"], r["verdict"]))


if __name__ == "__main__":
    main()
