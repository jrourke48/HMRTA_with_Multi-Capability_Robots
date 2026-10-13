#!/usr/bin/env python3
"""
The makespan trends the per-sweep figures cannot show.

Each sweep plots its own variable against makespan, which is the right way to read one factor but
leaves out everything that only appears when the sweeps are put side by side: how specification
complexity compares across measures, and how capability supply and demand interact. These figures
are cross-sweep, so they are the ones that answer "what actually drives the makespan".

Absolute makespan varies with the automaton and its environment - a long mission is long whatever
planner runs it - so anything pooled across automata is normalised by the best makespan that
automaton ever achieves. A value of 1.00 means "at this specification's floor".

  1 saturation.png        normalised makespan against capability supply / demand. The headline:
                          two separate saturations (team size, capabilities per robot) are one
                          curve once expressed as a ratio.
  2 specification.png     the four specification measures against makespan, so the reader can see
                          that proposition count predicts better than automaton size.
  3 supply_demand_3d.png  required capability against fleet capability. The diagonal is supply =
                          demand, and the floor is everything above it.
  4 propositions_3d.png   proposition count against team size: the two strongest variables that
                          are varied independently across the suite.

Writes to comparison/. Run from the organizedtests directory:
    python3 plot_makespan_trends.py
"""

import os
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers the 3d projection)

import makespan_drivers as md

OUT_DIR = "comparison"
TREE, SAMPLER = "#c2501f", "#256abf"
INK, INK_MUTED, SURFACE = "#000000", "#444444", "#ffffff"

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Nimbus Roman No9 L", "Liberation Serif", "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "font.size": 11,
    "axes.labelsize": 12,
    "legend.fontsize": 10,
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "axes.edgecolor": INK,
    "axes.linewidth": 0.8,
    "text.color": INK,
    "axes.labelcolor": INK,
    "xtick.color": INK,
    "ytick.color": INK,
    "xtick.direction": "in",
    "ytick.direction": "in",
    "grid.color": "#cccccc",
    "grid.linewidth": 0.5,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "legend.frameon": False,
})


def prepare():
    """Load, add the supply/demand ratio, and normalise makespan per automaton."""
    rows = md.load()
    floor = defaultdict(lambda: {"tree": float("inf"), "sampler": float("inf")})
    for row in rows:
        row["supply_demand"] = (row["total_robot_capabilities"]
                                / max(row["total_required_capabilities"], 1e-9))
        key = (row["sweep"], row["automaton_id"])
        floor[key]["tree"] = min(floor[key]["tree"], row["tree_makespan_seconds"])
        if row["random_sampling_makespan_seconds"] > 0:
            floor[key]["sampler"] = min(floor[key]["sampler"],
                                        row["random_sampling_makespan_seconds"])
    for row in rows:
        key = (row["sweep"], row["automaton_id"])
        row["tree_norm"] = row["tree_makespan_seconds"] / floor[key]["tree"]
        row["sampler_norm"] = (row["random_sampling_makespan_seconds"] / floor[key]["sampler"]
                               if row["random_sampling_makespan_seconds"] > 0
                               and floor[key]["sampler"] < float("inf") else float("nan"))
    return rows


def band(values):
    return np.median(values), np.percentile(values, 25), np.percentile(values, 75)


# --------------------------------------------------------------------------- 1
def plot_saturation(rows):
    """Normalised makespan against supply/demand, with the interquartile band."""
    fig, ax = plt.subplots(figsize=(8.8, 5.4))
    edges = [0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 3.0, 5.0, 10.0, 40.0]
    centres, series = [], {"tree": [], "sampler": []}

    for low, high in zip(edges[:-1], edges[1:]):
        chunk = [r for r in rows if low <= r["supply_demand"] < high]
        if len(chunk) < 4:
            continue
        centres.append(np.sqrt(max(low, 0.05) * high))        # geometric centre, log axis
        series["tree"].append(band([r["tree_norm"] for r in chunk]))
        sampled = [r["sampler_norm"] for r in chunk if r["sampler_norm"] == r["sampler_norm"]]
        series["sampler"].append(band(sampled) if sampled else (np.nan, np.nan, np.nan))

    for name, colour, marker in (("tree", TREE, "o"), ("sampler", SAMPLER, "s")):
        med = np.array([v[0] for v in series[name]])
        lo = np.array([v[1] for v in series[name]])
        hi = np.array([v[2] for v in series[name]])
        ax.fill_between(centres, lo, hi, color=colour, alpha=0.15, linewidth=0)
        ax.plot(centres, med, marker=marker, color=colour, linewidth=2, markersize=7,
                label="Tree planner" if name == "tree" else "Random sampler")

    ax.axhline(1.0, color=INK_MUTED, linestyle="--", linewidth=0.8)
    ax.axvspan(1.5, 2.0, color="#b5b4ae", alpha=0.35, linewidth=0)
    ax.text(1.72, ax.get_ylim()[1] * 0.92, "floor reached", ha="center", fontsize=9.5,
            color=INK_MUTED)

    ax.set_xscale("log")
    ax.set_xlabel("Fleet capability $\\div$ capability the specification requires")
    ax.set_ylabel("Makespan $\\div$ best achieved for that specification")
    ax.grid(True, which="both", alpha=0.35)
    ax.set_axisbelow(True)
    ax.legend(loc="upper right")

    fig.tight_layout()
    path = os.path.join(OUT_DIR, "saturation.png")
    fig.savefig(path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    return path


# --------------------------------------------------------------------------- 2
def plot_specification(rows):
    """Specification complexity against makespan, split by whether the fleet is provisioned.

    Pooled, these measures look like noise, and that is the finding rather than a defect: above
    the capability knee every configuration already sits at its floor, so no specification
    measure can move it, and mixing those rows in buries the effect under a flat majority.
    Separating under-provisioned fleets from provisioned ones makes the interaction visible -
    complexity costs makespan only while there is too little capability to absorb it.
    """
    measures = [("num_atomic_propositions", "Atomic propositions"),
                ("independent_required_capabilities", "Required capabilities (distinct)"),
                ("total_required_capabilities", "Required capabilities (total)"),
                ("num_automaton_states", "Büchi states")]
    regimes = [("starved", lambda r: r["supply_demand"] < 1.0, "#8c2d04", "o",
                "Fleet under-provisioned ($<1\\times$)"),
               ("provisioned", lambda r: r["supply_demand"] >= 1.5, "#4393c3", "s",
                "Fleet provisioned ($\\geq1.5\\times$)")]

    fig, axes = plt.subplots(1, 4, figsize=(16.5, 4.6), sharey=True)
    for ax, (column, label) in zip(axes, measures):
        for _, predicate, colour, marker, legend in regimes:
            values = defaultdict(list)
            for row in rows:
                if row[column] == row[column] and predicate(row):
                    values[row[column]].append(row["tree_norm"])

            xs, med, lo, hi = [], [], [], []
            for level in sorted(values):
                chunk = values[level]
                if len(chunk) < 4:
                    continue
                m, q1, q3 = band(chunk)
                xs.append(level)
                med.append(m)
                lo.append(m - q1)
                hi.append(q3 - m)
            if xs:
                ax.errorbar(xs, med, yerr=[lo, hi], marker=marker, color=colour, linewidth=1.8,
                            markersize=6, capsize=3, elinewidth=1, label=legend)

        ax.axhline(1.0, color=INK_MUTED, linestyle="--", linewidth=0.8)
        # The Buchi panel needs a log x to be readable at all; the rest are linear,
        # so each axis says which it is rather than leaving the reader to infer it
        scale = "log scale" if column == "num_automaton_states" else "linear scale"
        ax.set_xlabel("%s (%s)" % (label, scale))
        ax.grid(True, alpha=0.35)
        ax.set_axisbelow(True)

    axes[3].set_xscale("log")
    axes[0].set_ylabel("Tree makespan $\\div$ best for that specification")
    axes[0].legend(loc="upper left")

    fig.tight_layout()
    path = os.path.join(OUT_DIR, "specification_measures.png")
    fig.savefig(path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    return path


# --------------------------------------------------------------------------- 3 and 4
def surface(rows, xcol, ycol, xlabel, ylabel, filename, logx=False, logy=False,
            diagonal=False):
    """A trisurf over the configurations that exist.

    plot_surface needs a filled rectangular grid and these pairings are 60-80% dense, so a regular
    grid would be mostly holes - which is why the existing 3-D figures read as empty. A
    triangulation spans only the cells that were actually run.
    """
    cells = defaultdict(list)
    for row in rows:
        if row[xcol] != row[xcol] or row[ycol] != row[ycol]:
            continue
        cells[(row[xcol], row[ycol])].append(row["tree_norm"])

    xs = np.array([k[0] for k in cells], dtype=float)
    ys = np.array([k[1] for k in cells], dtype=float)
    zs = np.array([float(np.median(v)) for v in cells.values()])

    # A couple of cells hold a single starved configuration and spike to ten-plus, which flattens
    # the rest of the surface into an unreadable sheet. Clipped at the 95th percentile: the ridge
    # still reads as the ridge, and the plain keeps its shape.
    cap = float(np.percentile(zs, 95))
    clipped = int((zs > cap).sum())
    zs = np.minimum(zs, cap)

    fig = plt.figure(figsize=(9.5, 7))
    ax = fig.add_subplot(111, projection="3d")

    px = np.log10(xs) if logx else xs
    py = np.log10(ys) if logy else ys
    surf = ax.plot_trisurf(px, py, zs, cmap="YlOrRd", linewidth=0.3, edgecolor="#55555533",
                           antialiased=True, alpha=0.95)
    ax.scatter(px, py, zs, color=INK, s=5, depthshade=False)

    if diagonal:
        # supply = demand. Everything above this line is a fleet that carries more capability
        # than the specification asks for, and that is where the floor lives
        line = np.linspace(max(px.min(), py.min()), min(px.max(), py.max()), 40)
        ax.plot(line, line, np.full_like(line, zs.min()), color=INK, linewidth=1.6,
                linestyle="--", zorder=10)

    ax.set_xlabel("\n" + (("$\\log_{10}$ " if logx else "") + xlabel), linespacing=2.6)
    ax.set_ylabel("\n" + (("$\\log_{10}$ " if logy else "") + ylabel), linespacing=2.6)
    ax.set_zlabel("\nMakespan $\\div$ best", linespacing=2.6)
    ax.view_init(elev=26, azim=-128)
    ax.grid(True)
    fig.colorbar(surf, ax=ax, shrink=0.55, pad=0.09, label="Makespan $\\div$ best")

    ax.set_zlim(bottom=1.0)

    path = os.path.join(OUT_DIR, filename)
    fig.savefig(path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    return path, len(cells), clipped


def main():
    rows = prepare()
    os.makedirs(OUT_DIR, exist_ok=True)
    print("%d configurations\n" % len(rows))

    print("Wrote %s" % plot_saturation(rows))
    print("Wrote %s" % plot_specification(rows))

    path, cells, clipped = surface(rows, "total_required_capabilities", "total_robot_capabilities",
                          "required capability", "fleet capability",
                          "supply_demand_3d.png", logy=True, diagonal=False)
    print("Wrote %s  (%d filled cells, %d clipped at the 95th pct)" % (path, cells, clipped))

    path, cells, clipped = surface(rows, "num_atomic_propositions", "num_robots",
                          "atomic propositions", "robots",
                          "propositions_3d.png", logy=True)
    print("Wrote %s  (%d filled cells, %d clipped at the 95th pct)" % (path, cells, clipped))


if __name__ == "__main__":
    main()
