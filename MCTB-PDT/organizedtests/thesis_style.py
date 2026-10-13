#!/usr/bin/env python3
"""
One look for every figure in organizedtests: Times New Roman, a print palette, and no titles.

Each sweep's plotting script grew its own styling, so the suite came out in the default
matplotlib sans-serif with the default tab10 colours and a bold title on every figure. Three
problems with that for a thesis:

  titles      a figure in a thesis is introduced by its caption. An in-figure title repeats the
              caption at a larger size and in a different typeface, so these are suppressed - see
              the note on set_title below.
  typeface    body text is serif, so figures in a sans-serif read as pasted in from elsewhere.
  colour      the tab10 defaults are not colour-vision safe. Its green and orange sit 0.7 ΔE
              apart under protanopia, which is indistinguishable; printed greyscale they are the
              same value too. This module uses Okabe-Ito, which is designed for exactly that.

Okabe-Ito scores one warning rather than a pass on adjacent-pair CVD separation (7.6 ΔE, in the
6-8 band). That band is usable only when colour is not the sole channel carrying identity: these
scripts already give every series its own marker and a legend entry, so it is.

Import and call apply() before plotting:

    import thesis_style
    thesis_style.apply()
"""

import matplotlib
import matplotlib.pyplot as plt

# Okabe-Ito, extended with two further hues for the sweeps that draw up to ten series.
# Order matters: the validator checks neighbouring pairs, and this ordering keeps the
# red-green confusions apart.
PALETTE = [
    "#0072B2",  # blue
    "#D55E00",  # vermillion
    "#009E73",  # bluish green
    "#CC79A7",  # reddish purple
    "#E69F00",  # orange
    "#56B4E9",  # sky blue
    "#8c510a",  # brown
    "#762a83",  # purple
]

# Tints of the above, for scripts that drew a second lighter series per colour
TINTS = {
    "#0072B2": "#7fb8dc", "#D55E00": "#eaa37f", "#009E73": "#7fcfb9",
    "#CC79A7": "#e5bcd3", "#E69F00": "#f2cf7f", "#56B4E9": "#aad9f4",
    "#8c510a": "#c5a784", "#762a83": "#ba94c1",
}

INK, INK_MUTED, SURFACE = "#000000", "#444444", "#ffffff"

_applied = False


def _suppress_titles():
    """Make set_title and suptitle no-ops.

    Done by replacement rather than by deleting the calls from eight scripts: the calls are
    informative where they sit in the source, and a caption-bearing thesis figure simply should
    not render them. Anything that genuinely needs an in-figure label can set it through
    ax.text, which is untouched.
    """
    if getattr(matplotlib.axes.Axes.set_title, "_thesis_suppressed", False):
        return

    def noop_title(self, *args, **kwargs):
        return self.title

    def noop_suptitle(self, *args, **kwargs):
        return None

    noop_title._thesis_suppressed = True
    matplotlib.axes.Axes.set_title = noop_title
    matplotlib.figure.Figure.suptitle = noop_suptitle


def apply():
    """Set the rcParams and suppress titles. Safe to call more than once."""
    global _applied
    _suppress_titles()
    if _applied:
        return

    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Nimbus Roman No9 L", "Liberation Serif",
                       "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "font.size": 11,
        "axes.labelsize": 12,
        "axes.titlesize": 12,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
        "legend.fontsize": 10,
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "savefig.facecolor": SURFACE,
        "axes.edgecolor": INK,
        "axes.linewidth": 0.8,
        "axes.labelcolor": INK,
        "text.color": INK,
        "xtick.color": INK,
        "ytick.color": INK,
        "xtick.direction": "in",
        "ytick.direction": "in",
        "axes.prop_cycle": plt.cycler(color=PALETTE),
        "grid.color": "#cccccc",
        "grid.linewidth": 0.5,
        "grid.alpha": 0.6,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "legend.frameon": False,
        "lines.linewidth": 1.9,
        "lines.markersize": 6,
        "figure.dpi": 110,
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
    })
    _applied = True
