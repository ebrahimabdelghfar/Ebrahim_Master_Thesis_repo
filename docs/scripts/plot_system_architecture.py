#!/usr/bin/env python3
"""
Academic system-architecture diagram for adaptive_controller_manager.
Uses routing channels and color-coded arrows to avoid overlaps.
"""
import os, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

try:    plt.style.use("seaborn-v0_8-white")
except: plt.style.use("seaborn-white")

plt.rcParams.update({"font.family": "sans-serif",
                      "font.sans-serif": ["DejaVu Sans"],
                      "mathtext.fontset": "dejavusans"})

# ── Colours ──
C_EXT   = "#BBDEFB"; C_EXT_E  = "#1565C0"
C_MOD   = "#C8E6C9"; C_MOD_E  = "#2E7D32"
C_FSM   = "#FFE0B2"; C_FSM_E  = "#E65100"
C_GATE  = "#FFCDD2"; C_GATE_E = "#C62828"
C_PAR   = "#E1BEE7"; C_PAR_E  = "#6A1B9A"
C_OUT   = "#C5CAE9"; C_OUT_E  = "#283593"
C_MGR   = "#FAFAFA"; C_MGR_E  = "#424242"

# Arrow color categories
A_IN   = "#1565C0"   # blue:   external inputs
A_PIPE = "#2E7D32"   # green:  internal pipeline
A_CTRL = "#E65100"   # orange: control outputs (Start_Working)
A_SRV  = "#6A1B9A"   # purple: service calls
A_DATA = "#455A64"   # grey:   data passthrough

fig, ax = plt.subplots(figsize=(28, 26))
ax.set_xlim(0, 28)
ax.set_ylim(0, 26)
ax.set_aspect("equal")
ax.axis("off")
fig.patch.set_facecolor("white")

# ═══════════════════════════  Helpers  ═══════════════════
def box(cx, cy, w, h, text, bg, ec, fs=9.5, sub=None, sub_fs=7.5,
        sub_col="#555", zorder=3):
    ax.add_patch(FancyBboxPatch((cx-w/2, cy-h/2), w, h,
        boxstyle="round,pad=0.12", facecolor=bg, edgecolor=ec,
        lw=1.6, zorder=zorder))
    if sub:
        ax.text(cx, cy+0.22, text, ha="center", va="center",
                fontsize=fs, fontweight="bold", zorder=zorder+1)
        ax.text(cx, cy-0.25, sub, ha="center", va="center",
                fontsize=sub_fs, fontstyle="italic", color=sub_col,
                zorder=zorder+1)
    else:
        ax.text(cx, cy, text, ha="center", va="center",
                fontsize=fs, fontweight="bold", zorder=zorder+1)

def draw_arrow(pts, label="", color=A_PIPE, lw=1.5, dashed=False,
               lx=None, ly=None, lha="center", lva="bottom", lfs=8,
               lcol=None, zorder=4):
    """Orthogonal arrow through waypoints with label at (lx,ly)."""
    if lcol is None: lcol = color
    ls = "--" if dashed else "-"
    for i in range(len(pts)-2):
        ax.plot([pts[i][0], pts[i+1][0]], [pts[i][1], pts[i+1][1]],
                color=color, lw=lw, ls=ls, zorder=zorder,
                solid_capstyle="round")
    ax.annotate("", xy=pts[-1], xytext=pts[-2],
                arrowprops=dict(arrowstyle="-|>", color=color, lw=lw,
                                linestyle=ls, mutation_scale=14),
                zorder=zorder)
    if label and lx is not None:
        ax.text(lx, ly, label, ha=lha, va=lva, fontsize=lfs,
                color=lcol, zorder=zorder+2,
                bbox=dict(fc="white", ec="none", alpha=0.92, pad=1.5))

# ═══════════════════════════  LAYOUT  ═══════════════════
# External nodes
OD_X, OD_Y   = 2.5, 24      # Odometry
RL_X, RL_Y   = 2.5, 22      # Raceline
PP_X, PP_Y   = 10, 24       # Pure Pursuit
MC_X, MC_Y   = 20, 24       # MPC Controller
SI_X, SI_Y   = 2.5, 13      # SysID
VH_X, VH_Y   = 14, 1.5      # Vehicle
BM_X, BM_Y   = 26, 13       # Benchmark

# Internal modules (inside manager box)
SC_X, SC_Y   = 11, 19       # Subscription Callbacks
PM_X, PM_Y   = 21, 19       # Parameter Manager
TE_X, TE_Y   = 8, 16        # Track Error
TM_X, TM_Y   = 21, 16       # Tire Param Mediator
CM_X, CM_Y   = 8, 13        # Convergence Monitor
AG_X, AG_Y   = 14, 13       # Arming Gate
SR_X, SR_Y   = 21, 10       # Switch Ramp
FS_X, FS_Y   = 14, 8.5      # FSM
OA_X, OA_Y   = 14, 5        # Output Arbiter
DB_X, DB_Y   = 23, 8.5      # Debug Publishers

# Manager box
MBX, MBY, MBW, MBH = 5, 3.5, 20, 18

# Routing channels (dedicated x/y lanes for arrows to avoid crossing)
CH_L1 = 5.8    # left channel 1 (for input arrows going down)
CH_L2 = 6.5    # left channel 2
CH_R1 = 24.2   # right channel (for output arrows going up)
CH_R2 = 24.8   # right channel 2

# ════════════════  1. TITLE  ════════════════
ax.text(14, 25.5, "System Architecture — adaptive_controller_manager",
        ha="center", fontsize=18, fontweight="bold", color="#212121")
ax.text(14, 25.0,
        "ROS 2 node arbitrating Pure Pursuit / MPC with "
        "online tire-parameter identification",
        ha="center", fontsize=11, fontstyle="italic", color="#757575")

# ════════════════  2. MANAGER BOX  ════════════════
ax.add_patch(FancyBboxPatch((MBX, MBY), MBW, MBH,
    boxstyle="round,pad=0.4", facecolor=C_MGR, edgecolor=C_MGR_E,
    lw=2.8, zorder=0))
ax.text(MBX+MBW/2, MBY+MBH-0.5,
        "adaptive_controller_manager  (ManagerNode)",
        ha="center", fontsize=14, fontweight="bold",
        fontstyle="italic", color=C_MGR_E, zorder=1)

# ════════════════  3. EXTERNAL NODES  ════════════════
box(OD_X, OD_Y, 2.8, 0.85, "Odometry Source", C_EXT, C_EXT_E, sub="/odom")
box(RL_X, RL_Y, 2.8, 0.85, "Raceline Publisher", C_EXT, C_EXT_E,
    sub="/raceline_waypoints")
box(PP_X, PP_Y, 3.0, 0.85, "Pure Pursuit", C_EXT, C_EXT_E,
    sub="pure_pursuit node")
box(MC_X, MC_Y, 3.0, 0.85, "MPC Controller", C_EXT, C_EXT_E,
    sub="mpc_path_tracking node")
box(SI_X, SI_Y, 2.8, 0.85, "On-Track SysID", C_EXT, C_EXT_E,
    sub="identification node")
box(VH_X, VH_Y, 3.5, 0.85, "Vehicle / Simulator", C_EXT, C_EXT_E,
    sub="AckermannDriveStamped")
box(BM_X, BM_Y, 2.2, 0.85, "Tire Force\nBenchmark", C_EXT, C_EXT_E,
    fs=8.5, sub="(optional)")

# ════════════════  4. INTERNAL MODULES  ════════════════
box(SC_X, SC_Y, 5.0, 0.85, "Subscription Callbacks", C_MOD, C_MOD_E,
    sub="odom · waypoints · pp/mpc cmds · health")

box(PM_X, PM_Y, 3.5, 0.85, "Parameter Manager", C_PAR, C_PAR_E,
    sub="TopicsConfig · SafetyConfig · TireBounds")

box(TE_X, TE_Y, 3.5, 0.85, "Track Error Computation", C_MOD, C_MOD_E)
ax.text(TE_X, TE_Y-0.7, r"$\rightarrow\; e_y,\; \theta_{err},\; \kappa_{max}$",
        ha="center", fontsize=8.5, fontstyle="italic", color="#2E7D32")

box(TM_X, TM_Y, 3.5, 0.85, "Tire Param Mediator", C_MOD, C_MOD_E,
    sub="validate → store → forward")

box(CM_X, CM_Y, 3.5, 0.85, "Convergence Monitor", C_MOD, C_MOD_E)
ax.text(CM_X, CM_Y-0.7, r"$\rightarrow\; \bar{\dot{e}}_y$ convergence flag",
        ha="center", fontsize=8.5, fontstyle="italic", color="#2E7D32")

box(AG_X, AG_Y, 5.0, 1.3, "Arming Gate", C_GATE, C_GATE_E, fs=11)
ax.text(AG_X, AG_Y-0.55,
        r"$v_x > v_{min}$  ·  $|e_y| < e_{y,max}$  ·  "
        r"$|\theta| < \theta_{max}$"
        "\n"
        r"$\bar{\dot{e}}_y < 0$  ·  "
        r"$\kappa_{ahead} < \kappa_{max}$",
        ha="center", fontsize=8, fontstyle="italic", color="#B71C1C",
        linespacing=1.5)

box(SR_X, SR_Y, 3.5, 0.85, "Switch Ramp", C_MOD, C_MOD_E,
    sub="velocity blend")
ax.text(SR_X, SR_Y-0.7,
        r"$v_{cmd}\!=\!(1\!-\!\alpha)\,v_{frozen}"
        r" + \alpha\,v_{new}$",
        ha="center", fontsize=8, fontstyle="italic", color="#2E7D32")

box(FS_X, FS_Y, 6.0, 1.4, "Finite State Machine", C_FSM, C_FSM_E, fs=12)
ax.text(FS_X, FS_Y-0.55,
        "BOOTSTRAP_PP → RUNNING_PP → SWITCHING_TO_MPC\n"
        "→ RUNNING_MPC → SWITCHING_TO_PP → EMERGENCY_HALT",
        ha="center", fontsize=7.5, fontstyle="italic", color="#E65100",
        linespacing=1.3)

box(OA_X, OA_Y, 5.0, 0.85, "Output Arbiter  (computeOutput)", C_OUT, C_OUT_E)
ax.text(OA_X, OA_Y-0.7,
        r"decel rate-limit:  $\Delta v \leq a_{max} \cdot \Delta t$",
        ha="center", fontsize=8, fontstyle="italic", color="#283593")

box(DB_X, DB_Y, 2.2, 0.9, "Debug\nPublishers", "#ECEFF1", "#9E9E9E",
    fs=8.5, sub="manager/state\nRViz markers", sub_fs=6.5)

# ════════════════════════════════════════════════════════
#  5. ARROWS  — All carefully routed through channels
# ════════════════════════════════════════════════════════

# ──── EXTERNAL INPUTS (blue) ────

# Odom → Sub Callbacks: down channel CH_L1, then right
draw_arrow([(OD_X+1.4, OD_Y), (CH_L1, OD_Y), (CH_L1, SC_Y),
            (SC_X-2.5, SC_Y)],
           label="/odom", lx=CH_L1-0.3, ly=SC_Y+2, lha="right",
           color=A_IN, lfs=8.5)

# Raceline → Sub Callbacks: down channel CH_L2, then right
draw_arrow([(RL_X+1.4, RL_Y), (CH_L2, RL_Y), (CH_L2, SC_Y-0.2),
            (SC_X-2.5, SC_Y-0.2)],
           label="/raceline_waypoints", lx=CH_L2-0.3, ly=SC_Y+0.8,
           lha="right", color=A_IN, lfs=8)

# PP cmds → Sub Callbacks: straight down
draw_arrow([(PP_X, PP_Y-0.42), (PP_X, SC_Y+0.42)],
           label="pp/drive_cmd\npp_state · pp_health",
           lx=PP_X+0.3, ly=SC_Y+2, lha="left", color=A_IN, lfs=7.8)

# MPC cmds → Sub Callbacks: down then left along y=20.5
draw_arrow([(MC_X, MC_Y-0.42), (MC_X, SC_Y+1.5),
            (SC_X+2.5, SC_Y+1.5), (SC_X+2.5, SC_Y+0.42)],
           label="mpc/drive_cmd · /mpc/status",
           lx=16, ly=SC_Y+1.8, color=A_IN, lfs=8)

# SysID first_run → Sub Callbacks: up channel CH_L1
draw_arrow([(SI_X+1.4, SI_Y+0.2), (CH_L1, SI_Y+0.2),
            (CH_L1, SC_Y-0.35), (SC_X-2.5, SC_Y-0.35)],
           label="sysid/first_run", lx=CH_L1-0.3, ly=SI_Y+2,
           lha="right", color=A_IN, lfs=7.8)

# SysID update_params → Tire Mediator: right along y=13 then up
draw_arrow([(SI_X+1.4, SI_Y-0.2), (CH_L2, SI_Y-0.2),
            (CH_L2, TM_Y-0.8), (TM_X-1.75, TM_Y-0.8),
            (TM_X-1.75, TM_Y-0.42)],
           label="sysid/update_params\n(service server)",
           lx=12, ly=TM_Y-1.1, color=A_SRV, lfs=7.8)

# ──── INTERNAL PIPELINE (green) ────

# Sub Callbacks → Track Error
draw_arrow([(SC_X-1.5, SC_Y-0.42), (SC_X-1.5, TE_Y+1.2),
            (TE_X, TE_Y+1.2), (TE_X, TE_Y+0.42)],
           label="odom, waypoints", lx=9, ly=TE_Y+1.45,
           color=A_PIPE, lfs=8)

# Sub Callbacks → Arming Gate (vx, health): straight down center
draw_arrow([(SC_X+1, SC_Y-0.42), (SC_X+1, AG_Y+1.3),
            (AG_X, AG_Y+1.3), (AG_X, AG_Y+0.65)],
           label=r"odom ($v_x$), health flags",
           lx=SC_X+1.3, ly=AG_Y+3, lha="left", color=A_PIPE, lfs=7.8)

# Track Error → Convergence: straight down
draw_arrow([(TE_X, TE_Y-0.42), (TE_X, CM_Y+0.42)],
           label=r"$e_y$", lx=TE_X+0.5, ly=TE_Y-1.5,
           lha="left", color=A_PIPE, lfs=9)

# Track Error → Arming Gate: right then down
draw_arrow([(TE_X+1.75, TE_Y-0.2), (AG_X-1, TE_Y-0.2),
            (AG_X-1, AG_Y+0.65)],
           label=r"$e_y$, $\theta_{err}$, $\kappa_{max}$",
           lx=10.5, ly=TE_Y+0.05, color=A_PIPE, lfs=7.8)

# Convergence → Arming Gate: straight right
draw_arrow([(CM_X+1.75, CM_Y), (AG_X-2.5, CM_Y)],
           label="convergence flag",
           lx=10.5, ly=CM_Y+0.3, color=A_PIPE, lfs=8)

# Arming Gate → FSM: straight down
draw_arrow([(AG_X, AG_Y-0.65), (AG_X, FS_Y+0.7)],
           label="arming\ndecision", lx=AG_X+0.5, ly=AG_Y-2.2,
           lha="left", color=A_PIPE, lfs=8)

# Arming Gate → Tire Mediator: right then up (gate pass)
draw_arrow([(AG_X+2.5, AG_Y+0.3), (TM_X-2.5, AG_Y+0.3),
            (TM_X-2.5, TM_Y-0.42)],
           label="gate pass", lx=17.5, ly=AG_Y+0.55,
           color=A_PIPE, lfs=7.8)

# Tire Mediator → FSM: down then left
draw_arrow([(TM_X-0.5, TM_Y-0.42), (TM_X-0.5, FS_Y+0.3),
            (FS_X+3, FS_Y+0.3)],
           label="stored_version + ack",
           lx=TM_X-0.2, ly=AG_Y-2.5, lha="left", color=A_PIPE, lfs=7.8)

# FSM → Switch Ramp: right then up
draw_arrow([(FS_X+3, FS_Y-0.3), (SR_X-2.5, FS_Y-0.3),
            (SR_X-2.5, SR_Y), (SR_X-1.75, SR_Y)],
           label="beginSwitch()", lx=17.5, ly=FS_Y-0.05,
           color=A_PIPE, lfs=8)

# Switch Ramp → Output Arbiter: down then left
draw_arrow([(SR_X, SR_Y-0.42), (SR_X, OA_Y), (OA_X+2.5, OA_Y)],
           label=r"$v_{cmd}$, $\delta$",
           lx=SR_X+0.3, ly=SR_Y-2.5, lha="left", color=A_PIPE, lfs=8)

# FSM → Output Arbiter: straight down
draw_arrow([(FS_X, FS_Y-0.7), (FS_X, OA_Y+0.42)],
           label="active state", lx=FS_X-0.5, ly=FS_Y-2.5,
           lha="right", color=A_PIPE, lfs=8)

# Sub Callbacks cmd passthrough → Output Arbiter: far-left channel
draw_arrow([(SC_X-2.5, SC_Y-0.42), (SC_X-2.5, OA_Y),
            (OA_X-2.5, OA_Y)],
           label="pp_cmd, mpc_cmd",
           lx=SC_X-2.8, ly=AG_Y-2.5, lha="right", color=A_DATA,
           lfs=7.8)

# ──── CONTROL OUTPUTS (orange) ────

# FSM → PP: Start_Working_pp — left then up on CH_L2+1
CH_PP = 7.2
draw_arrow([(FS_X-3, FS_Y), (CH_PP, FS_Y), (CH_PP, PP_Y),
            (PP_X-1.5, PP_Y)],
           label="Start_Working_pp", lx=CH_PP-0.3, ly=FS_Y+3.5,
           lha="right", color=A_CTRL, lfs=8)

# FSM → MPC: Start_Working_mpc — right then up on CH_R1
draw_arrow([(FS_X+3, FS_Y-0.1), (CH_R1, FS_Y-0.1),
            (CH_R1, MC_Y), (MC_X+1.5, MC_Y)],
           label="Start_Working_mpc", lx=CH_R1+0.3, ly=FS_Y+3.5,
           lha="left", color=A_CTRL, lfs=8)

# ──── SERVICE OUTPUTS (purple) ────

# Tire Mediator → MPC: up on CH_R2
draw_arrow([(TM_X+1.75, TM_Y), (CH_R2, TM_Y),
            (CH_R2, MC_Y-0.2), (MC_X+1.5, MC_Y-0.2)],
           label="mpc/update_params\n(service client)",
           lx=CH_R2+0.3, ly=TM_Y+3, lha="left", color=A_SRV, lfs=7.8)

# Tire Mediator → Benchmark: right (dashed)
draw_arrow([(TM_X+1.75, TM_Y-0.3), (BM_X, TM_Y-0.3),
            (BM_X, BM_Y+0.42)],
           label="benchmark/\nupdate_params",
           lx=BM_X+0.3, ly=TM_Y-1.5, lha="left", color=A_SRV,
           lfs=7, dashed=True)

# ──── OUTPUT (black) ────

# Output Arbiter → Vehicle: straight down
draw_arrow([(OA_X, OA_Y-0.42), (OA_X, VH_Y+0.42)],
           label="/drive", lx=OA_X+0.5, ly=OA_Y-2, lha="left",
           color="#000000", lfs=10, lw=2.0)

# FSM → Debug
draw_arrow([(FS_X+3, FS_Y+0.2), (DB_X-1.1, FS_Y+0.2),
            (DB_X-1.1, DB_Y)],
           label="state, errors", lx=19, ly=FS_Y+0.45,
           color=A_DATA, lfs=7)

# ════════════════  6. LEGEND  ════════════════
LX, LY = 7, OA_Y+1.2
# Color legend
items = [
    (C_EXT,  C_EXT_E,  "External ROS 2 Node"),
    (C_MOD,  C_MOD_E,  "Internal Module"),
    (C_FSM,  C_FSM_E,  "Finite State Machine"),
    (C_GATE, C_GATE_E, "Safety / Arming Gate"),
    (C_PAR,  C_PAR_E,  "Configuration"),
    (C_OUT,  C_OUT_E,  "Output Stage"),
]
ax.add_patch(FancyBboxPatch((LX-0.6, LY-len(items)*0.55+0.15),
    3.8, len(items)*0.55+0.25, boxstyle="round,pad=0.12",
    facecolor="white", edgecolor="#BDBDBD", lw=0.8, zorder=5, alpha=0.95))
for i, (bg, ec, lbl) in enumerate(items):
    ly = LY - i * 0.55
    ax.add_patch(FancyBboxPatch((LX-0.25, ly-0.18), 0.45, 0.36,
        boxstyle="round,pad=0.04", facecolor=bg, edgecolor=ec,
        lw=0.8, zorder=6))
    ax.text(LX+0.5, ly, lbl, ha="left", va="center", fontsize=8.5,
            zorder=7)

# Arrow type legend
ALX = LX - 0.6
ALY = LY - len(items)*0.55 - 0.6
arrow_items = [
    (A_IN,   "External input (topic)"),
    (A_PIPE, "Internal data flow"),
    (A_CTRL, "Control output"),
    (A_SRV,  "Service call"),
]
for i, (col, lbl) in enumerate(arrow_items):
    aly = ALY - i * 0.5
    ax.plot([ALX, ALX+0.6], [aly, aly], color=col, lw=2, zorder=6)
    ax.annotate("", xy=(ALX+0.6, aly), xytext=(ALX+0.35, aly),
                arrowprops=dict(arrowstyle="-|>", color=col, lw=1.5),
                zorder=6)
    ax.text(ALX+0.9, aly, lbl, ha="left", va="center", fontsize=8,
            zorder=7)

# ════════════════  SAVE  ════════════════
plt.tight_layout()
out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "..", "images")
os.makedirs(out_dir, exist_ok=True)
for ext in ("png", "pdf"):
    p = os.path.join(out_dir,
                     f"adaptive_controller_system_architecture.{ext}")
    fig.savefig(p, dpi=200, bbox_inches="tight", facecolor="white")
    print(f"Saved → {p}")
plt.close(fig)
