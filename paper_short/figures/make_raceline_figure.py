#!/usr/bin/env python3
"""Fig. 4 -- the optimised raceline used for every experiment in the paper.

Reads ``traj_race_cl.csv`` (the byte-identical output of the TUM
global_racetrajectory_optimization package shipped inside Carla_ASU_Bridge)
and renders three panels:

  (a) track layout, coloured by lateral demand             [sequential, one hue]
  (b) reference speed profile vs arc length                [one series]
  (c) lateral demand v_x^2|kappa| vs arc length, against
      the grip the optimiser ASSUMED and the grip the
      vehicle actually HAS                                 [one series + 2 refs]

Panel (c) is the point of the figure: the optimiser was handed a flat
5.0 m/s^2 envelope in ``inputs/veh_dyn_info/ggv.csv`` (0.51 g) against a plant
that measures ~1.0 g. The raceline saturates that envelope -- it peaks at
0.51 g -- but that is only ~51 % of the grip the vehicle actually has, so no
segment of the lap is infeasible; the conservatism is left on the table.

Panel (a) is coloured by lateral demand, NOT by speed: with v_max binding almost
everywhere the speed spans 0.09 m/s and a speed colour scale would carry no
information.

Design rules followed (see the dataviz skill):
  * no dual axes -- speed and demand are separate panels, never twinned;
  * speed is a magnitude, so it gets ONE hue light->dark, never a rainbow;
  * single-series panels carry no legend box; the two reference levels are
    direct-labelled, so meaning is never colour-alone;
  * colours are the validated defaults (series-1 blue, status-critical red);
    the pair passes the CVD and normal-vision floors on a light surface.

Usage:  python3 make_raceline_figure.py [path/to/traj_race_cl.csv]
Writes: raceline.pdf  (vector, sized for an IEEE two-column figure*)
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap, Normalize

# --- validated palette -------------------------------------------------------
INK        = "#0b0b0b"   # text-primary
INK_2      = "#52514e"   # text-secondary
SERIES_1   = "#2a78d6"   # categorical slot 1 / sequential step 450
CRITICAL   = "#d03b3b"   # status: critical (never reused as a series colour)
GRID       = "#d8d7d3"
# sequential blue ramp, steps 200 -> 700. Started at 200 rather than 100 so the
# slow end of the track stays visible as a thin line on white paper.
SEQ_STEPS  = ["#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]
SEQ        = LinearSegmentedColormap.from_list("speed", SEQ_STEPS)

G = 9.81
AY_ASSUMED = 5.0   # inputs/veh_dyn_info/ggv.csv -- flat at every speed
AY_MEASURED = 9.81  # measured steady-state ceiling of the CARLA asurt_fsai

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "DejaVu Serif"],
    "font.size": 7,
    "axes.labelsize": 7,
    "axes.titlesize": 7.5,
    "xtick.labelsize": 6.5,
    "ytick.labelsize": 6.5,
    "axes.edgecolor": INK_2,
    "axes.linewidth": 0.5,
    "xtick.color": INK_2,
    "ytick.color": INK_2,
    "text.color": INK,
    "axes.labelcolor": INK,
    "figure.dpi": 200,
})


def load(path):
    d = np.loadtxt(path, delimiter=";", comments="#")
    s, x, y, psi, kappa, vx, ax = d.T
    return s, x, y, kappa, vx


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default = os.path.join(here, "..", "..", "traj_race_cl.csv")
    path = sys.argv[1] if len(sys.argv) > 1 else default
    s, x, y, kappa, vx = load(path)

    ay = vx ** 2 * np.abs(kappa)
    lap = s[-1] + np.median(np.diff(s))
    s_km = s / 1000.0

    fig = plt.figure(figsize=(7.16, 3.05))          # IEEE \textwidth
    gs = fig.add_gridspec(2, 2, width_ratios=[1.0, 2.45], height_ratios=[0.62, 1.38],
                          wspace=0.20, hspace=0.34,
                          left=0.005, right=0.985, top=0.90, bottom=0.135)
    ax_map = fig.add_subplot(gs[:, 0])
    ax_v = fig.add_subplot(gs[0, 1])
    ax_a = fig.add_subplot(gs[1, 1], sharex=ax_v)

    # ---------------- (a) track layout, coloured by reference speed ----------
    pts = np.array([x, y]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap=SEQ, norm=Normalize(0.0, ay.max()),
                        linewidth=1.5, capstyle="round")
    lc.set_array(ay[:-1])
    ax_map.add_collection(lc)
    ax_map.set_aspect("equal")
    ax_map.autoscale_view()
    ax_map.margins(0.06)
    ax_map.axis("off")

    # start / finish and the corner that sets the whole feasibility argument
    ax_map.plot(x[0], y[0], "o", ms=3.2, mfc="white", mec=INK, mew=0.7, zorder=5)
    ax_map.annotate("start/finish", (x[0], y[0]), textcoords="offset points",
                    xytext=(6, -1), fontsize=6, color=INK_2)
    i = int(np.argmax(np.abs(kappa)))
    ax_map.plot(x[i], y[i], "o", ms=3.2, mfc="white", mec=CRITICAL, mew=0.9, zorder=5)
    ax_map.annotate("tightest corner, highest demand\n"
                    r"$R=%.1f$ m, %.2f g" % (1 / abs(kappa[i]), ay[i] / G),
                    (x[i], y[i]), textcoords="offset points", xytext=(7, -6),
                    fontsize=6, color=CRITICAL)

    # Horizontal, under the map. A vertical bar here would sit in the gutter and
    # collide with the y-labels of panels (b)/(c).
    cb = fig.colorbar(lc, ax=ax_map, orientation="horizontal",
                      fraction=0.042, pad=0.03, aspect=22)
    cb.set_label(r"$v_x^2|\kappa|$ [m/s$^2$]", fontsize=6.5, color=INK, labelpad=1.5)
    cb.ax.tick_params(labelsize=6, color=INK_2, length=2, pad=1.5)
    cb.outline.set_linewidth(0.4)
    cb.outline.set_edgecolor(INK_2)

    ax_map.set_title("(a) raceline, %.2f km lap, coloured by lateral demand"
                     % (lap / 1000.0), fontsize=7.5, color=INK, pad=3)

    # ---------------- (b) speed profile --------------------------------------
    ax_v.plot(s_km, vx, color=SERIES_1, lw=0.9)
    # Autoscaling here would zoom onto 0.09 m/s of variation and make a flat
    # profile look eventful. Fix the range so "pinned at the cap" is the reading.
    ax_v.set_ylim(0, vx.max() * 1.18)
    ax_v.axhline(vx.max(), color=INK, lw=0.7, ls=":", dashes=(1, 1.6))
    ax_v.annotate(r"$v_{\max}=%.0f$ m/s, binding over %.0f%% of the lap"
                  % (vx.max(), 100.0 * np.mean(vx > vx.max() - 0.01)),
                  xy=(0.012, vx.max()), xycoords=("axes fraction", "data"),
                  xytext=(0, 2.0), textcoords="offset points",
                  fontsize=6, color=INK, va="bottom")
    ax_v.set_ylabel(r"$v_x$ [m/s]")
    ax_v.set_title("(b) reference speed profile", fontsize=7.5, color=INK, pad=3)
    ax_v.grid(True, lw=0.4, color=GRID)
    ax_v.set_axisbelow(True)
    ax_v.tick_params(labelbottom=False)
    for side in ("top", "right"):
        ax_v.spines[side].set_visible(False)

    # ---------------- (c) lateral demand vs available grip -------------------
    ax_a.fill_between(s_km, 0, ay, where=ay > AY_MEASURED, color=CRITICAL,
                      alpha=0.20, linewidth=0, interpolate=True)
    ax_a.plot(s_km, ay, color=SERIES_1, lw=0.9)
    ax_a.axhline(AY_ASSUMED, color=CRITICAL, lw=0.9, ls="--", dashes=(4, 2))
    ax_a.axhline(AY_MEASURED, color=INK, lw=0.9, ls=":", dashes=(1, 1.6))

    # Direct labels: the two levels must not be distinguished by colour alone.
    # Both texts are formatted from the constants so they cannot drift from the
    # lines they name. Both sit above their line: below the assumed envelope is
    # where the trace lives, so a label there gets spikes drawn through it.
    ax_a.annotate(r"assumed by the optimiser: %.1f m/s$^2$ (%.2f g)"
                  % (AY_ASSUMED, AY_ASSUMED / G),
                  xy=(0.012, AY_ASSUMED), xycoords=("axes fraction", "data"),
                  xytext=(0, 2.0), textcoords="offset points",
                  fontsize=6, color=CRITICAL, va="bottom")
    ax_a.annotate(r"measured ceiling: %.1f m/s$^2$ ($\approx$%.1f g)"
                  % (AY_MEASURED, AY_MEASURED / G),
                  xy=(0.012, AY_MEASURED), xycoords=("axes fraction", "data"),
                  xytext=(0, 2.0), textcoords="offset points", va="bottom",
                  fontsize=6, color=INK)

    # The peak is the whole reading of the panel: mark it and state where it
    # sits relative to both levels. The callout is lifted into the empty band
    # between the two lines, clear of the trace and of both level labels.
    j = int(np.argmax(ay))
    over = ay > AY_MEASURED
    ax_a.plot(s_km[j], ay[j], "o", ms=3.0, mfc="none", mec=CRITICAL, mew=0.9,
              zorder=6)
    if over.any():
        seg_m = float(over.sum()) * float(np.median(np.diff(s_km))) * 1000.0
        peak_txt = ("peak %.2f g; over the measured ceiling\nfor %.0f m of %.0f m (%.1f%%)"
                    % (ay[j] / G, seg_m, s_km[-1] * 1000.0, 100.0 * over.mean()))
    else:
        peak_txt = ("peak %.2f g (%.1f m/s$^2$) at $s=%.2f$ km: %.0f%% of the\n"
                    r"assumed envelope, %.0f%% of the measured ceiling"
                    % (ay[j] / G, ay[j], s_km[j],
                       100.0 * ay[j] / AY_ASSUMED, 100.0 * ay[j] / AY_MEASURED))
    y_mid = 0.5 * (AY_ASSUMED + AY_MEASURED)
    ax_a.annotate(peak_txt,
                  xy=(s_km[j], ay[j]), xytext=(s_km[j], y_mid),
                  ha="center", va="center",
                  fontsize=6, color=CRITICAL,
                  arrowprops=dict(arrowstyle="-", lw=0.45, color=CRITICAL,
                                  shrinkA=3.0, shrinkB=2.5))
    ax_a.set_ylabel(r"$v_x^2|\kappa|$ [m/s$^2$]")
    ax_a.set_xlabel("arc length $s$ [km]")
    ax_a.set_title("(c) lateral demand against available grip",
                   fontsize=7.5, color=INK, pad=3)
    ax_a.grid(True, lw=0.4, color=GRID)
    ax_a.set_axisbelow(True)
    # Both reference levels must stay on-panel whichever of them is higher.
    ax_a.set_ylim(0, max(AY_ASSUMED, AY_MEASURED, ay.max()) * 1.20)
    ax_a.set_xlim(0, s_km[-1])
    for side in ("top", "right"):
        ax_a.spines[side].set_visible(False)

    out = os.path.join(here, "raceline.pdf")
    fig.savefig(out, format="pdf", bbox_inches="tight", pad_inches=0.01)
    print("wrote", out)

    # numbers quoted in the paper -- printed so the text can be checked
    frac = 100.0 * np.mean(ay > AY_MEASURED)
    print(f"  lap {lap:.1f} m, {len(s)} pts @ {np.median(np.diff(s)):.3f} m")
    print(f"  vx  {vx.min():.2f}-{vx.max():.2f} m/s")
    print(f"  |kappa|max {np.abs(kappa).max():.4f} 1/m -> R {1/np.abs(kappa).max():.1f} m")
    print(f"  ay  max {ay.max():.2f} m/s^2 = {ay.max()/G:.3f} g, p99 {np.percentile(ay,99):.2f}")
    print(f"  {frac:.1f}% of the lap demands more than the measured ceiling")


if __name__ == "__main__":
    main()
