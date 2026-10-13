#!/usr/bin/env python3
"""
Retrofit thesis_style onto every sweep plotting script, once.

Each script hardcodes the tab10 hex codes and imports pyplot its own way, so the styling cannot
be applied purely through rcParams. This rewrites two things in place:

  the colours   the tab10 codes are swapped for the Okabe-Ito equivalents slot for slot, so a
                script that chose "the third colour" still gets the third colour. rcParams alone
                would not reach them, because they are written out as string literals.
  the import    a shim after the pyplot import that puts organizedtests on sys.path (the scripts
                run with their own sweep directory as the working directory) and calls apply().

Idempotent: a script that already imports thesis_style is left alone.

Run once from the organizedtests directory:  python3 apply_thesis_style.py
"""

import io
import os
import re

import thesis_style

SCRIPTS = [
    "plot_3d_makespan.py",
    "plot_scaling_curves.py",
    os.path.join("automaton_states", "plot_automaton_states.py"),
    os.path.join("automaton_states_batch", "plot_automaton_states_batch.py"),
    os.path.join("average_capabilities", "plot_average_capabilities.py"),
    os.path.join("number_robots", "plot_number_robots.py"),
    os.path.join("number_robots", "plot_robots_capabilities_makespan_3d.py"),
    os.path.join("robot_homogeneity", "plot_robot_homogeneity.py"),
    os.path.join("transition_system_regions", "plot_transition_system_regions.py"),
]

# tab10 in its own order -> the new palette in its order, so relative identity is preserved
COLOUR_MAP = {
    "#1f77b4": "#0072B2", "#ff7f0e": "#D55E00", "#2ca02c": "#009E73",
    "#d62728": "#CC79A7", "#9467bd": "#762a83", "#8c564b": "#8c510a",
    "#e377c2": "#E69F00", "#7f7f7f": "#6b6b6b", "#bcbd22": "#c98700",
    "#17becf": "#56B4E9",
    # the pale tab20 companions some scripts use for a second line of the same series
    "#aec7e8": "#7fb8dc", "#ffbb78": "#eaa37f", "#98df8a": "#7fcfb9",
    "#ff9896": "#e5bcd3", "#c5b0d5": "#ba94c1", "#c49c94": "#c5a784",
    "#f7b6d2": "#f2cf7f", "#dbdb8d": "#f2cf7f", "#9edae5": "#aad9f4",
}

SHIM = """
# Shared thesis styling: Times New Roman, print palette, titles suppressed in favour of captions
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
import thesis_style as _thesis_style
_thesis_style.apply()
"""


def retrofit(path):
    source = io.open(path, "r", newline="", encoding="utf-8", errors="surrogateescape").read()
    if "thesis_style" in source:
        return "already styled"

    term = "\r\n" if "\r\n" in source else "\n"
    changed = 0
    for old, new in COLOUR_MAP.items():
        for variant in (old, old.upper()):
            if variant in source:
                changed += source.count(variant)
                source = source.replace(variant, new)

    # After the last pyplot import, so rcParams land on an initialised pyplot
    lines = source.split(term)
    anchor = max((n for n, line in enumerate(lines)
                  if re.match(r"\s*(import matplotlib|from matplotlib)", line)), default=None)
    if anchor is None:
        return "no matplotlib import found - skipped"
    lines[anchor + 1:anchor + 1] = SHIM.strip("\n").split("\n")

    io.open(path, "w", newline="", encoding="utf-8", errors="surrogateescape").write(
        term.join(lines))
    return "styled, %d colour literals remapped" % changed


def main():
    for path in SCRIPTS:
        if not os.path.exists(path):
            print("  %-58s missing" % path)
            continue
        print("  %-58s %s" % (path, retrofit(path)))
    print("\nPalette: %s" % ", ".join(thesis_style.PALETTE))


if __name__ == "__main__":
    main()
