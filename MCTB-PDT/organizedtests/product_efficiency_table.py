#!/usr/bin/env python3
"""
Quantify what the tree planner saves against the exact product automaton.

One row per configuration in which a product automaton was actually constructed AND solved.
Makespan is deliberately not considered - this table is about cost, not solution quality.

Only a handful of configurations qualify, so every one is listed individually rather than
summarised: with four data points a median says less than the four numbers themselves.

Two filters decide what qualifies:

  * `full_product_automaton_nodes > 0` - the product was built at all. Four of the six sweeps
    skip it entirely via their `shouldSkip` size guard, and within those four it is skipped for
    every team larger than five robots.
  * `product_makespan_seconds < UINT32_MAX/2` - the product also found an accepting run.
    `ProductAutomaton::OptimalAcceptingPath()` returns UINT32_MAX as a failure sentinel and the
    sweeps store it unguarded, so a failed build still records a real node count and a real
    computation time. Those runs would otherwise inflate the speedup with searches that found
    nothing.

The same configuration is measured by more than one sweep (the sweeps share automata and
environments), which is a repeat measurement of one problem, not two data points. Rows are
merged on the problem's identity and the timings averaged.

Reads the per-sweep CSVs written by TestRunManager::exportByConfiguration and writes
    comparison/product_efficiency.csv    one row per configuration
    comparison/product_efficiency.tex    the same table as booktabs LaTeX

Run from the organizedtests directory:  python3 product_efficiency_table.py
"""

import csv
import glob
import os
import statistics

OUT_DIR = "comparison"
SENTINEL = 2 ** 32 / 2
SKIP_FILES = {"statistics.csv", "combined_long.csv"}


def number(row, column):
    text = (row.get(column) or "").strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def collect():
    """Every solved product build, merged across sweeps by the problem it describes."""
    problems = {}
    # Attempts are counted on the problem's identity too, so a configuration that two
    # sweeps both attempt is one attempt, matching how the solved rows are merged.
    attempted = set()

    for csv_path in sorted(glob.glob(os.path.join("*", "data", "*.csv"))):
        if os.path.basename(csv_path) in SKIP_FILES:
            continue
        sweep = csv_path.split(os.sep)[0]

        with open(csv_path, newline="") as f:
            rows = list(csv.DictReader(f))

        for row in rows:
            nodes = number(row, "full_product_automaton_nodes")
            edges = number(row, "full_product_automaton_edges")
            makespan = number(row, "product_makespan_seconds")
            tree_nodes = number(row, "total_nodes_planning")
            product_ms = number(row, "product_computation_time_ms")
            tree_ms = number(row, "total_computation_time_ms")
            if not nodes or nodes <= 0:
                continue

            # Identity of the underlying problem. Node and edge counts are deterministic, so two
            # sweeps reporting the same counts for the same automaton and team size are running
            # the same instance.
            identity = (int(number(row, "num_automaton_states") or 0),
                        int(row.get("automaton_id") or 0),
                        int(number(row, "num_robots") or 0),
                        int(nodes), int(edges or 0))
            attempted.add(identity)
            if makespan is None or makespan >= SENTINEL:
                continue            # the product ran but found no accepting run
            if not tree_ms or tree_ms <= 0 or product_ms is None or not tree_nodes:
                continue

            entry = problems.setdefault(identity, {
                "automaton_states": identity[0],
                "automaton_id": identity[1],
                "num_robots": identity[2],
                "product_nodes": identity[3],
                "product_edges": identity[4],
                "tree_nodes": int(tree_nodes),
                "product_ms": [],
                "tree_ms": [],
                "sweeps": [],
            })
            entry["product_ms"].append(product_ms)
            entry["tree_ms"].append(tree_ms)
            entry["sweeps"].append(sweep)

    return problems, len(attempted)


def finalise(entry):
    """Average the repeat measurements and derive the reductions."""
    product_ms = statistics.mean(entry["product_ms"])
    tree_ms = statistics.mean(entry["tree_ms"])
    nodes, edges, tree_nodes = entry["product_nodes"], entry["product_edges"], entry["tree_nodes"]
    return {
        "automaton_id": entry["automaton_id"],
        "automaton_states": entry["automaton_states"],
        "num_robots": entry["num_robots"],
        "product_nodes": nodes,
        "product_edges": edges,
        "tree_nodes": tree_nodes,
        # A planning tree with N nodes has exactly N-1 edges, so this is not an estimate
        "tree_edges": tree_nodes - 1,
        "product_time_ms": product_ms,
        "tree_time_ms": tree_ms,
        "speedup": product_ms / tree_ms,
        "node_reduction_pct": 100.0 * (1.0 - tree_nodes / nodes),
        "edge_reduction_pct": 100.0 * (1.0 - (tree_nodes - 1) / edges) if edges else float("nan"),
        "measurements": len(entry["product_ms"]),
    }


FIELDS = ["automaton_id", "automaton_states", "num_robots",
          "product_nodes", "product_edges", "tree_nodes", "tree_edges",
          "product_time_ms", "tree_time_ms", "speedup",
          "node_reduction_pct", "edge_reduction_pct", "measurements"]


def write_csv(records):
    path = os.path.join(OUT_DIR, "product_efficiency.csv")
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        for record in records:
            writer.writerow({k: ("%.4f" % v if isinstance(v, float) else v)
                             for k, v in record.items()})
    return path


def write_latex(records, attempted):
    path = os.path.join(OUT_DIR, "product_efficiency.tex")
    solved = len(records)
    lines = [
        "% Generated by organizedtests/product_efficiency_table.py - do not edit by hand.",
        "% Requires: \\usepackage{booktabs}",
        "\\begin{table}[t]",
        "\\centering",
        "\\caption{Cost of the tree planner against the exact product automaton, on every",
        "configuration where the product automaton was both small enough to construct and able to",
        "find an accepting run (%d of %d attempted builds, all at three robots over six regions)." % (solved, attempted),
        "A planning tree of $N$ nodes has exactly $N-1$ edges, so the tree's edge count is exact",
        "rather than estimated. Solution quality is not considered here.}",
        "\\label{tab:product_efficiency}",
        "\\footnotesize",
        "\\begin{tabular}{@{}rrrrrrrrrrr@{}}",
        "\\toprule",
        "& & \\multicolumn{3}{c}{\\textbf{Product automaton}} & "
        "\\multicolumn{3}{c}{\\textbf{Tree planner}} & "
        "\\multicolumn{2}{c}{\\textbf{Reduction}} & \\\\",
        "\\cmidrule(lr){3-5}\\cmidrule(lr){6-8}\\cmidrule(lr){9-10}",
        "\\textbf{Aut.} & \\textbf{NBA} & nodes & edges & time (s) "
        "& nodes & edges & time (ms) & nodes & edges & \\textbf{Speedup} \\\\",
        "& \\textbf{states} & & & & & & & & & \\\\",
        "\\midrule",
    ]
    for r in records:
        lines.append(
            "%d & %d & %s & %s & %.2f & %d & %d & %.3f & %.2f\\%% & %.2f\\%% "
            "& $%s\\times$ \\\\" % (
                r["automaton_id"], r["automaton_states"],
                "{:,}".format(r["product_nodes"]), "{:,}".format(r["product_edges"]),
                r["product_time_ms"] / 1000.0,
                r["tree_nodes"], r["tree_edges"], r["tree_time_ms"],
                r["node_reduction_pct"], r["edge_reduction_pct"],
                # {,} not a bare comma: in math mode LaTeX treats "," as a list separator and
                # adds a thin space after it, so $3,748$ would set as "3, 748"
                "{:,}".format(int(round(r["speedup"]))).replace(",", "{,}")))
    lines += [
        "\\bottomrule",
        "\\end{tabular}",
        "",
        "\\vspace{0.5em}",
        "\\begin{minipage}{0.95\\textwidth}\\footnotesize",
        "Speedup is the product automaton's construction and search time divided by the tree",
        "planner's total computation time on the same instance, and ranges from",
        "$%s\\times$ to $%s\\times$." % (
            "{:,}".format(int(round(min(r["speedup"] for r in records)))).replace(",", "{,}"),
            "{:,}".format(int(round(max(r["speedup"] for r in records)))).replace(",", "{,}")),
        "The smallest speedup is on the largest specification, where the tree planner's own cost",
        "has grown by a factor of %.0f while the product automaton's has not; four points are too"
        % (max(r["tree_time_ms"] for r in records) / min(r["tree_time_ms"] for r in records)),
        "few to call that a trend, but the direction is consistent across them.",
        "The remaining %d attempted builds returned no accepting run and are excluded."
        % (attempted - solved),
        "\\end{minipage}",
        "\\end{table}",
        "",
    ]
    with open(path, "w") as f:
        f.write("\n".join(lines))
    return path


def main():
    problems, attempted = collect()
    if not problems:
        raise SystemExit("No sweep CSVs with a solved product automaton were found.")

    os.makedirs(OUT_DIR, exist_ok=True)
    records = sorted((finalise(e) for e in problems.values()),
                     key=lambda r: (r["automaton_states"], r["automaton_id"]))

    print("Product automaton vs tree planner, per configuration")
    print("(%d of %d attempted product builds found an accepting run)\n" % (len(records), attempted))

    print("%-5s %-6s %-7s %11s %11s %11s %11s" % (
        "aut", "NBA", "robots", "prod nodes", "prod edges", "tree nodes", "tree edges"))
    print("-" * 70)
    for r in records:
        print("%-5d %-6d %-7d %11s %11s %11d %11d" % (
            r["automaton_id"], r["automaton_states"], r["num_robots"],
            "{:,}".format(r["product_nodes"]), "{:,}".format(r["product_edges"]),
            r["tree_nodes"], r["tree_edges"]))

    print("\n%-5s %13s %13s %10s %11s %11s" % (
        "aut", "prod time(s)", "tree time(ms)", "speedup", "node red", "edge red"))
    print("-" * 68)
    for r in records:
        print("%-5d %13.2f %13.3f %9.0fx %10.2f%% %10.3f%%" % (
            r["automaton_id"], r["product_time_ms"] / 1000.0, r["tree_time_ms"],
            r["speedup"], r["node_reduction_pct"], r["edge_reduction_pct"]))

    speedups = [r["speedup"] for r in records]
    print("\nSpeedup %0.0fx to %0.0fx, median %0.0fx."
          % (min(speedups), max(speedups), statistics.median(speedups)))
    print("%d attempted builds found no accepting run and are excluded." % (attempted - len(records)))
    print("\nWrote %s" % write_csv(records))
    print("Wrote %s" % write_latex(records, attempted))


if __name__ == "__main__":
    main()
