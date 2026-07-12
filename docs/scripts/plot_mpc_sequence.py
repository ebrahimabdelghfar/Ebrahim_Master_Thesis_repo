#!/usr/bin/env python3
"""
Generate an academic sequence diagram for the MPC execution flow using Matplotlib.
This allows for full LaTeX math rendering of the equations.
Saves the figure to docs/images/mpc_sequence.png.
"""

import os
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Use seaborn styles for a clean academic look
try:
    plt.style.use('seaborn-v0_8-white')
except OSError:
    plt.style.use('seaborn-white')

# Participants and their X coordinates - spaced out to avoid text overlap
participants = {
    "ROS 2 Context": 0,
    "MpcNode": 4.0,
    "MpcController": 8.0,
    "VehicleModel": 12.0,
    "SolverInterface\n(OSQP)": 16.0
}

# Setup figure
fig, ax = plt.subplots(figsize=(16, 16))
ax.set_xlim(-1, 17)
ax.set_ylim(-32, 2)
ax.axis('off')

# Colors
COLOR_LIFELINE = "#BDBDBD"
COLOR_ACTIVATE = "#E0E0E0"
COLOR_ARROW = "#424242"
COLOR_NOTE_BG = "#FFF9C4"
COLOR_NOTE_EDGE = "#FBC02D"
COLOR_LOOP_BG = "#FAFAFA"
COLOR_LOOP_EDGE = "#E0E0E0"

# Draw lifelines and headers
for name, x in participants.items():
    ax.plot([x, x], [0, -31], color=COLOR_LIFELINE, linestyle='--', zorder=0)
    # Header box
    ax.add_patch(patches.Rectangle((x - 1.4, 0), 2.8, 1.2, facecolor='white', edgecolor='black', zorder=3))
    ax.text(x, 0.6, name, ha='center', va='center', fontsize=11, fontweight='bold', zorder=4)

y = -1.0
dy = 1.0

# Helper functions
def draw_arrow(x_start, x_end, y_val, text, dashed=False):
    style = "-|>"
    ls = "dashed" if dashed else "solid"
    
    # Add a little padding to the arrow ends so they don't touch the lifeline exactly
    pad = 0.15 if x_start != x_end else 0
    dx = x_end - x_start
    if dx > 0:
        sx, ex = x_start + pad, x_end - pad
    elif dx < 0:
        sx, ex = x_start - pad, x_end + pad
    else:
        # Self-arrow
        sx = x_start + 0.1
        ax.annotate("", xy=(x_start, y_val - 0.5), xytext=(sx, y_val),
                    arrowprops=dict(arrowstyle="->", color=COLOR_ARROW, lw=1.5, ls=ls, connectionstyle="bar,angle=180,fraction=-0.3"))
        ax.text(x_start + 0.5, y_val - 0.25, text, ha='left', va='center', fontsize=11)
        return y_val - 1.0

    ax.annotate("", xy=(ex, y_val), xytext=(sx, y_val),
                arrowprops=dict(arrowstyle=style, color=COLOR_ARROW, lw=1.5, ls=ls))
    ax.text((sx + ex) / 2, y_val + 0.2, text, ha='center', va='bottom', fontsize=11)
    return y_val - dy

def draw_note(x_center, y_val, text, width=3.8):
    lines = text.count('\n') + 1
    height = 0.6 + lines * 0.4
    rect = patches.Rectangle((x_center - width/2, y_val - height/2), width, height, 
                             facecolor=COLOR_NOTE_BG, edgecolor=COLOR_NOTE_EDGE, zorder=2)
    ax.add_patch(rect)
    ax.text(x_center, y_val, text, ha='center', va='center', fontsize=10.5)
    return y_val - height/2 - 0.4

def draw_activation(x, y_start, y_end):
    rect = patches.Rectangle((x - 0.1, y_end), 0.2, y_start - y_end, 
                             facecolor=COLOR_ACTIVATE, edgecolor='gray', zorder=1)
    ax.add_patch(rect)

# ── Sequence Diagram Steps ──

y = draw_note((participants["ROS 2 Context"] + participants["SolverInterface\n(OSQP)"]) / 2, y, 
              r"Control Cycle (Triggered at horizon.control_rate_hz)", width=10)

y = draw_arrow(participants["ROS 2 Context"], participants["MpcNode"], y, "Timer Callback")
y = draw_arrow(participants["MpcNode"], participants["MpcNode"], y, r"Fetch latest Odometry (state $x_0$)")
y = draw_arrow(participants["MpcNode"], participants["MpcNode"], y, r"Fetch Track Waypoints (reference)")

y -= 0.4
y = draw_note(participants["MpcNode"] + 2.0, y, 
              "Compute Adaptive Prediction Step\n"
              r"$dt = \mathrm{clamp}\left(\frac{L}{N \cdot v}, dt_{min}, dt_{max}\right)$", width=3.6)
y -= 0.4

y_activate_ctrl = y
y = draw_arrow(participants["MpcNode"], participants["MpcController"], y, r"computeOptimalCommand(state, reference, $dt$)")

# Loop start
y_loop_top = y + 0.3
y -= 0.6
x_loop = participants["MpcController"] - 1.5
w_loop = participants["VehicleModel"] - x_loop + 1.5

y_activate_model = y
y = draw_arrow(participants["MpcController"], participants["VehicleModel"], y, r"computeDiscreteLinearization($x_{ref}, u_{ref}, dt$)")

y -= 0.2
y = draw_note(participants["VehicleModel"] - 1.9, y, "RK4 Integration +\nFinite Difference Jacobians", width=3.5)
y -= 0.2

y = draw_arrow(participants["VehicleModel"], participants["MpcController"], y, r"$A_d, B_d, c$", dashed=True)
draw_activation(participants["VehicleModel"], y_activate_model, y + 0.5)

y = draw_arrow(participants["MpcController"], participants["MpcController"], y, r"Build State Cost ($Q$) & Input Cost ($R, R_{rate}$)")

y -= 0.4
y = draw_note(participants["MpcController"] + 2.0, y, r"Map path-relative errors ($e_y, e_\psi$)" + "\n" + r"into full state cost $P, q$", width=3.6)
y -= 0.4

# Loop end
y_loop_bottom = y + 0.3
loop_rect = patches.Rectangle((x_loop, y_loop_bottom), w_loop, y_loop_top - y_loop_bottom, facecolor=COLOR_LOOP_BG, edgecolor=COLOR_LOOP_EDGE, zorder=-1)
ax.add_patch(loop_rect)
ax.text(x_loop + 0.1, y_loop_top - 0.1, "loop", fontweight='bold', bbox=dict(facecolor='white', edgecolor=COLOR_LOOP_EDGE, boxstyle='round,pad=0.2'))
ax.text(x_loop + 1.0, y_loop_top - 0.1, r"Over Prediction Horizon $N$", fontstyle='italic')

y -= 0.5

y_activate_solv = y
y = draw_arrow(participants["MpcController"], participants["SolverInterface\n(OSQP)"], y, "buildAndSolveQP()")

y -= 0.2
y = draw_note(participants["SolverInterface\n(OSQP)"] - 1.9, y, 
              r"$\min \frac{1}{2} z^T P z + q^T z$" + "\n" + r"s.t. $l \leq A z \leq u$", width=3.5)
y -= 0.2

y = draw_arrow(participants["SolverInterface\n(OSQP)"], participants["SolverInterface\n(OSQP)"], y, "Run ADMM (OSQP)")

y = draw_arrow(participants["SolverInterface\n(OSQP)"], participants["MpcController"], y, r"Optimal solution $z^* = [x_0..x_N, u_0..u_{N-1}]$", dashed=True)
draw_activation(participants["SolverInterface\n(OSQP)"], y_activate_solv, y + 0.5)

y = draw_arrow(participants["MpcController"], participants["MpcNode"], y, r"Solution status, cost, and command $u_0^*$", dashed=True)
draw_activation(participants["MpcController"], y_activate_ctrl, y + 0.5)

# Alt start
y_alt_top = y + 0.3
y -= 0.6
x_alt = participants["ROS 2 Context"] - 0.5
w_alt = participants["MpcNode"] - x_alt + 1.5

y = draw_arrow(participants["MpcNode"], participants["ROS 2 Context"], y, "Publish AckermannDrive to /mpc/drive_cmd")
y = draw_arrow(participants["MpcNode"], participants["ROS 2 Context"], y, "Publish predicted horizon to /mpc/predicted_path")

y -= 0.3
ax.axhline(y, xmin=0.03, xmax=0.30, color=COLOR_LOOP_EDGE, linestyle='--')
ax.text(x_alt + 0.8, y + 0.2, "Solver Failed / Stale Data", fontstyle='italic')
y -= 0.6

y = draw_note((participants["ROS 2 Context"] + participants["MpcNode"]) / 2, y, "Apply fallback policy\n(hold_last, zero_command, brake)", width=3.8)
y -= 0.4

y = draw_arrow(participants["MpcNode"], participants["ROS 2 Context"], y, "Publish fallback AckermannDrive")

# Alt end
y_alt_bottom = y - 0.5
alt_rect = patches.Rectangle((x_alt, y_alt_bottom), w_alt, y_alt_top - y_alt_bottom, fill=False, edgecolor=COLOR_LOOP_EDGE, zorder=-1)
ax.add_patch(alt_rect)
ax.text(x_alt + 0.1, y_alt_top - 0.1, "alt", fontweight='bold', bbox=dict(facecolor='white', edgecolor=COLOR_LOOP_EDGE, boxstyle='round,pad=0.2'))
ax.text(x_alt + 0.8, y_alt_top - 0.1, "Solve Successful", fontstyle='italic')

y -= 1.0
y = draw_arrow(participants["MpcNode"], participants["ROS 2 Context"], y, "Publish diagnostic /mpc/status")

# Set dynamic limits to ensure bottom isn't cut off
ax.set_ylim(y - 2.0, 2)
plt.tight_layout()

# Save
out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'images')
os.makedirs(out_dir, exist_ok=True)
out_path = os.path.join(out_dir, 'mpc_sequence.png')
fig.savefig(out_path, dpi=200, bbox_inches='tight', facecolor='white')
print(f"Saved academic sequence diagram to {out_path}")
plt.close(fig)
