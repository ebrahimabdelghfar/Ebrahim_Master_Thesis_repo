#!/usr/bin/env python3
"""
Plot the MPC adaptive prediction-step (dynamic dt) concept.

Visualises how the prediction time step adapts to vehicle speed so
that the MPC horizon covers a roughly constant *look-ahead distance*
regardless of speed:

    dt = clamp( horizon_distance_m / (N * max(vx, v_floor)), dt_min, dt_max )

Three sub-plots:
  1. dt(v)   — the adaptive time step as a function of reference speed
  2. L(v)    — the resulting look-ahead distance  L = N · dt · v
  3. A side-by-side schematic comparing the prediction "fans" at a slow
     speed and a fast speed, illustrating that the spatial coverage stays
     approximately the same even though the temporal spacing changes.

Saves the figure to docs/images/dynamic_dt.png.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch

# ── Parameters (match config/mpc_path_tracking.yaml defaults) ──────────────
N = 30                   # prediction steps
HORIZON_DIST_M = 3.0     # target look-ahead distance [m]
DT_MIN = 0.02            # s
DT_MAX = 0.08            # s
V_FLOOR = 0.5            # m/s
V_MAX = 7.0              # m/s (for axis limit)

# Speed sweep
v = np.linspace(0.0, V_MAX + 1.0, 500)
v_eff = np.maximum(v, V_FLOOR)

# ── Compute dt(v) and look-ahead distance L(v) ────────────────────────────
dt = np.clip(HORIZON_DIST_M / (N * v_eff), DT_MIN, DT_MAX)
L = N * dt * v_eff  # effective look-ahead [m]

# Identify the three regimes for shading
# Regime 1: dt is clamped to dt_max  (low speed)
# Regime 2: dt is in the linear region
# Regime 3: dt is clamped to dt_min  (high speed)
v_low_thresh = HORIZON_DIST_M / (N * DT_MAX)     # below this → dt = dt_max
v_high_thresh = HORIZON_DIST_M / (N * DT_MIN)    # above this → dt = dt_min

# ── Style ──────────────────────────────────────────────────────────────────
try:
    plt.style.use('seaborn-v0_8-whitegrid')
except OSError:
    plt.style.use('seaborn-whitegrid')

fig, axes = plt.subplots(3, 1, figsize=(10, 13),
                         gridspec_kw={'height_ratios': [3, 3, 2.5], 'hspace': 0.4})
fig.patch.set_facecolor('#fafafa')
ax_dt, ax_L, ax_fan = axes

# Colours
C_DT      = '#E91E63'   # pink-red for dt
C_L       = '#2196F3'   # blue for look-ahead
C_REGIME  = '#FF9800'   # orange for regime boundaries
C_FAN_LO  = '#7B1FA2'   # purple for slow-speed fan
C_FAN_HI  = '#00897B'   # teal for fast-speed fan

# ── Panel 1: dt vs speed ──────────────────────────────────────────────────
ax_dt.plot(v, dt * 1000, '-', color=C_DT, lw=2.8, zorder=5,
           label=r'$\mathrm{dt}(v)$')

# Regime shading
ax_dt.axvspan(0, v_low_thresh, alpha=0.07, color=C_REGIME, zorder=0)
ax_dt.axvspan(v_high_thresh, v[-1], alpha=0.07, color='#4CAF50', zorder=0)

# Clamp lines
ax_dt.axhline(DT_MAX * 1000, color='#888888', ls='--', lw=1.3, alpha=0.7)
ax_dt.axhline(DT_MIN * 1000, color='#888888', ls='--', lw=1.3, alpha=0.7)
ax_dt.text(v[-1] - 0.05, DT_MAX * 1000 + 2, f'$dt_{{max}}$ = {DT_MAX*1000:.0f} ms',
           ha='right', fontsize=9, color='#555555')
ax_dt.text(v[-1] - 0.05, DT_MIN * 1000 + 2, f'$dt_{{min}}$ = {DT_MIN*1000:.0f} ms',
           ha='right', fontsize=9, color='#555555')

# Vertical threshold lines
ax_dt.axvline(v_low_thresh, color=C_REGIME, lw=1.3, ls=':', alpha=0.6)
ax_dt.axvline(v_high_thresh, color='#4CAF50', lw=1.3, ls=':', alpha=0.6)

# Regime labels
ax_dt.text(v_low_thresh / 2, ax_dt.get_ylim()[0] + 5 if ax_dt.get_ylim()[0] > 0 else 25,
           'clamped\n$dt_{max}$', ha='center', fontsize=8.5,
           color='#E65100', fontstyle='italic', alpha=0.8)

ax_dt.set_ylabel('Prediction step  $dt$  [ms]', fontsize=12, fontweight='bold')
ax_dt.set_xlabel('Reference speed  $v_{ref}$  [m/s]', fontsize=12, fontweight='bold')
ax_dt.set_title('Adaptive MPC Prediction Step  (Dynamic $dt$)', fontsize=15,
                fontweight='bold', pad=15)
ax_dt.legend(loc='upper right', fontsize=10, framealpha=0.9)
ax_dt.set_ylim(0, DT_MAX * 1000 + 40)

# Elaborative annotations
ax_dt.annotate('Slow driving:\nLarge time steps', xy=(v_low_thresh/2, DT_MAX*1000), 
               xytext=(1.5, 95),
               arrowprops=dict(arrowstyle='->', color='#E65100', lw=1.5),
               fontsize=10, color='#E65100', fontweight='bold', fontstyle='italic')

ax_dt.annotate('Fast driving:\nTime step shrinks\nto avoid looking\ntoo far ahead', 
               xy=(4.5, np.interp(4.5, v, dt)*1000), xytext=(3.2, 45),
               ha='center', va='bottom',
               arrowprops=dict(arrowstyle='->', color=C_DT, lw=1.5),
               fontsize=10, color=C_DT, fontweight='bold', fontstyle='italic')

# ── Panel 2: look-ahead distance vs speed ─────────────────────────────────
ax_L.plot(v, L, '-', color=C_L, lw=2.8, zorder=5,
          label=r'$L(v) = N \cdot \mathrm{dt}(v) \cdot v$')
ax_L.axhline(HORIZON_DIST_M, color='#888888', ls='--', lw=1.3, alpha=0.7)
ax_L.text(v[-1] - 0.05, HORIZON_DIST_M + 0.15,
          f'target $L$ = {HORIZON_DIST_M:.1f} m',
          ha='right', fontsize=9, color='#555555')

# Regime shading
ax_L.axvspan(0, v_low_thresh, alpha=0.07, color=C_REGIME, zorder=0)
ax_L.axvspan(v_high_thresh, v[-1], alpha=0.07, color='#4CAF50', zorder=0)
ax_L.axvline(v_low_thresh, color=C_REGIME, lw=1.3, ls=':', alpha=0.6)
ax_L.axvline(v_high_thresh, color='#4CAF50', lw=1.3, ls=':', alpha=0.6)

ax_L.set_ylabel('Look-ahead distance  $L$  [m]', fontsize=12, fontweight='bold')
ax_L.set_xlabel('Reference speed  $v_{ref}$  [m/s]', fontsize=12, fontweight='bold')
ax_L.legend(loc='lower right', fontsize=10, framealpha=0.9)
ax_L.set_ylim(0, max(L) + 1.5)

# Regime labels in look-ahead panel
mid_linear = (v_low_thresh + min(v_high_thresh, v[-1])) / 2
ax_L.text(mid_linear, HORIZON_DIST_M + 0.55,
          r'$dt = \frac{L_{target}}{N \cdot v}$  (ideal region)',
          ha='center', fontsize=9, color='#1565C0', fontstyle='italic', alpha=0.85)

# ── Panel 3: fan schematic ─────────────────────────────────────────────────
ax_fan.set_xlim(-0.5, 13)
ax_fan.set_ylim(-0.5, 3.5)
ax_fan.set_aspect('equal')
ax_fan.axis('off')

def draw_fan(ax, x0, y0, v_val, label, color, y_label):
    """Draw a row of prediction-step markers along a 'road'."""
    v_e = max(v_val, V_FLOOR)
    dt_val = np.clip(HORIZON_DIST_M / (N * v_e), DT_MIN, DT_MAX)
    step_dist = dt_val * v_val  # spatial spacing between prediction nodes
    total_L = N * step_dist

    # Road line
    road_len = max(total_L + 0.5, 7.0)
    ax.plot([x0, x0 + road_len], [y0, y0], '-', color='#BDBDBD', lw=6,
            solid_capstyle='round', zorder=1)

    # Vehicle marker
    ax.plot(x0, y0, 's', color=color, ms=14, zorder=6)
    ax.text(x0, y0 - 0.35, 'ego', ha='center', fontsize=8, color=color, fontweight='bold')

    # Prediction nodes
    for k in range(1, N + 1):
        xk = x0 + k * step_dist
        if xk > x0 + road_len:
            break
        size = max(4, 10 - 0.4 * k)
        alpha_val = max(0.25, 1.0 - 0.05 * k)
        ax.plot(xk, y0, 'o', color=color, ms=size, alpha=alpha_val, zorder=5)

    # Bracket for total look-ahead
    brace_y = y0 + 0.35
    ax.annotate('', xy=(x0, brace_y), xytext=(x0 + total_L, brace_y),
                arrowprops=dict(arrowstyle='<->', color=color, lw=1.5))
    ax.text(x0 + total_L / 2, brace_y + 0.18,
            f'$L$ = {total_L:.1f} m',
            ha='center', fontsize=9, color=color, fontweight='bold')

    # Label
    ax.text(x0 - 0.4, y0, label, ha='right', va='center', fontsize=10,
            color=color, fontweight='bold')

    # dt annotation — point to the gap from below to avoid overlapping 'ego'
    ax.annotate(f'$dt$={dt_val*1000:.0f} ms', 
                xy=(x0 + step_dist * 1.5, y0 - 0.15), 
                xytext=(x0 + 1.5, y0 - 0.55),
                arrowprops=dict(arrowstyle='->', color='#757575', lw=1.0),
                ha='center', fontsize=8.5, color='#616161', fontweight='bold')

    return dt_val, total_L

# Slow speed scenario
v_slow = 1.5
dt_s, L_s = draw_fan(ax_fan, 1.5, 2.5, v_slow,
                      f'$v$ = {v_slow:.0f} m/s', C_FAN_LO, 2.5)

# Fast speed scenario
v_fast = 7.0
dt_f, L_f = draw_fan(ax_fan, 1.5, 0.7, v_fast,
                      f'$v$ = {v_fast:.0f} m/s', C_FAN_HI, 0.7)

ax_fan.set_title('Spatial prediction coverage at different speeds',
                 fontsize=11, fontstyle='italic', pad=6, color='#424242')

# ── Formula box on panel 1 ────────────────────────────────────────────────
formula_text = (
    r"$\mathrm{dt}(v) = \mathrm{clamp}\!\left("
    r"\frac{L_{target}}{N \cdot \max(v,\, v_{floor})}"
    r",\; dt_{min},\; dt_{max}\right)$"
)
props = dict(boxstyle='round,pad=0.5', facecolor='white',
             edgecolor='#BDBDBD', alpha=0.95)
ax_dt.text(0.98, 0.55, formula_text, transform=ax_dt.transAxes, fontsize=11,
           verticalalignment='bottom', horizontalalignment='right', bbox=props)

# (X-labels are now set explicitly on each panel)

plt.tight_layout()

# ── Save ───────────────────────────────────────────────────────────────────
out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'images')
os.makedirs(out_dir, exist_ok=True)
out_path = os.path.join(out_dir, 'dynamic_dt.png')
fig.savefig(out_path, dpi=180, bbox_inches='tight', facecolor=fig.get_facecolor())
print(f"Saved to {out_path}")
plt.close(fig)
