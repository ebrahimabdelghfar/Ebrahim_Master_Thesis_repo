#!/usr/bin/env python3
"""
Plot the adaptive_controller_manager velocity handover ramp.

Shows how v_cmd blends from v_frozen (outgoing controller's last speed)
to v_new (incoming controller's commanded speed) over delta_t_switch
using a linear alpha ramp:

    alpha(t) = clamp(t / delta_t_switch, 0, 1)
    v_cmd(t) = (1 - alpha) * v_frozen  +  alpha * v_new(t)

Also shows the deceleration rate limit that caps any speed decrease to
max_decel_mps2 per tick, preventing instant full-stop commands.

Saves the figure to docs/images/handover_velocity_ramp.png.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ── Parameters (match config/adaptive_controller_manager.yaml) ──
DELTA_T_SWITCH = 1.0      # s
CONTROL_RATE_HZ = 20.0    # Hz
MAX_DECEL_MPS2 = 8.26     # m/s²

# ── Scenario ──
V_FROZEN = 3.0            # m/s — outgoing controller's last speed at switch instant
V_NEW_FINAL = 6.0         # m/s — incoming controller's steady-state target

# Time axis: show a bit before and after the switch window
t_pre = 0.3               # seconds of pre-switch (steady v_frozen)
t_post = 0.5              # seconds of post-switch (steady v_new)
dt = 1.0 / CONTROL_RATE_HZ

t_all = np.arange(-t_pre, DELTA_T_SWITCH + t_post + dt, dt)

# ── Compute signals ──
# Incoming controller ramps its own command linearly for illustration
v_new = np.where(t_all < 0, V_FROZEN, np.minimum(V_FROZEN + (V_NEW_FINAL - V_FROZEN) * (t_all / DELTA_T_SWITCH), V_NEW_FINAL))

alpha = np.clip(t_all / DELTA_T_SWITCH, 0, 1)
# Before t=0, alpha should be 0 (not switching yet)
alpha = np.where(t_all < 0, 0.0, alpha)

v_cmd = (1.0 - alpha) * V_FROZEN + alpha * v_new

# After the switch window, v_cmd just tracks v_new directly
v_cmd = np.where(t_all > DELTA_T_SWITCH, v_new, v_cmd)

# ── Plot ──
try:
    plt.style.use('seaborn-v0_8-whitegrid')
except OSError:
    plt.style.use('seaborn-whitegrid')
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), gridspec_kw={'height_ratios': [3, 1]}, sharex=True)
fig.patch.set_facecolor('#fafafa')

# ── Top panel: velocity ──
ax1.plot(t_all, np.full_like(t_all, V_FROZEN), '--', color='#888888', lw=1.5, label=f'$v_{{frozen}}$ = {V_FROZEN:.1f} m/s')
ax1.plot(t_all, v_new, ':', color='#2196F3', lw=2.0, label=f'$v_{{new}}(t)$ (incoming controller)')
ax1.plot(t_all, v_cmd, '-', color='#E91E63', lw=2.8, label='$v_{{cmd}}(t)$ (blended output to /drive)')

# Shade the switch window
ax1.axvspan(0, DELTA_T_SWITCH, alpha=0.08, color='#FF9800', zorder=0)
ax1.axvline(0, color='#FF9800', lw=1.5, ls='--', alpha=0.7)
ax1.axvline(DELTA_T_SWITCH, color='#FF9800', lw=1.5, ls='--', alpha=0.7)

ax1.annotate('beginSwitch()', xy=(0, V_FROZEN), xytext=(0.08, V_FROZEN - 0.6),
             fontsize=9, color='#E65100',
             arrowprops=dict(arrowstyle='->', color='#E65100', lw=1.2))
ax1.annotate(f'switch complete\n(Δt = {DELTA_T_SWITCH:.1f}s)',
             xy=(DELTA_T_SWITCH, v_cmd[np.argmin(np.abs(t_all - DELTA_T_SWITCH))]),
             xytext=(DELTA_T_SWITCH + 0.08, V_NEW_FINAL - 0.8),
             fontsize=9, color='#E65100',
             arrowprops=dict(arrowstyle='->', color='#E65100', lw=1.2))

ax1.set_ylabel('Speed  [m/s]', fontsize=12)
ax1.set_title('Handover Velocity Ramp  —  adaptive_controller_manager', fontsize=14, fontweight='bold', pad=12)
ax1.legend(loc='upper left', fontsize=10, framealpha=0.9)
ax1.set_ylim(V_FROZEN - 1.2, V_NEW_FINAL + 0.8)

# Label the switch window
mid = DELTA_T_SWITCH / 2
ax1.text(mid, ax1.get_ylim()[1] - 0.3, 'SWITCHING window', ha='center', fontsize=10,
         color='#E65100', fontstyle='italic', fontweight='bold', alpha=0.7)

# ── Bottom panel: alpha ──
ax2.plot(t_all, alpha, '-', color='#4CAF50', lw=2.5, label=r'$\alpha(t) = \mathrm{clamp}(t / \Delta t_{switch},\, 0,\, 1)$')
ax2.axvspan(0, DELTA_T_SWITCH, alpha=0.08, color='#FF9800', zorder=0)
ax2.axvline(0, color='#FF9800', lw=1.5, ls='--', alpha=0.7)
ax2.axvline(DELTA_T_SWITCH, color='#FF9800', lw=1.5, ls='--', alpha=0.7)
ax2.set_ylabel(r'$\alpha$', fontsize=14)
ax2.set_xlabel('Time  [s]  (relative to switch start)', fontsize=12)
ax2.set_ylim(-0.1, 1.15)
ax2.set_yticks([0, 0.25, 0.5, 0.75, 1.0])
ax2.legend(loc='upper left', fontsize=10, framealpha=0.9)

# ── Formula box ──
formula_text = (
    r"$v_{cmd}(t) = (1 - \alpha)\, v_{frozen} \;+\; \alpha\, v_{new}(t)$"
    "\n"
    r"$\alpha(t) = \mathrm{clamp}\!\left(\frac{t}{\Delta t_{switch}},\; 0,\; 1\right)$"
)
props = dict(boxstyle='round,pad=0.5', facecolor='white', edgecolor='#BDBDBD', alpha=0.95)
ax1.text(0.98, 0.15, formula_text, transform=ax1.transAxes, fontsize=11,
         verticalalignment='bottom', horizontalalignment='right', bbox=props)

plt.tight_layout()

# Save
out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'images')
os.makedirs(out_dir, exist_ok=True)
out_path = os.path.join(out_dir, 'handover_velocity_ramp.png')
fig.savefig(out_path, dpi=180, bbox_inches='tight', facecolor=fig.get_facecolor())
print(f"Saved to {out_path}")
plt.close(fig)
