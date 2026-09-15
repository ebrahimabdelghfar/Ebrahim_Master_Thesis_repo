#!/usr/bin/env python3
"""Generate baseline_vs_ours.pdf: the baseline pipeline's virtual-data step
against the excitation-aware one, on the full-scale vehicle.

Everything here is analytic or an offline round-trip through the repository's
own Pacejka solver -- no recorded run, no trained network, no simulator. The
residual network is held at zero so that the rollout configuration is the only
variable, which is exactly the isolation experiment of Section VI-C.

Panel (a): reachable friction mu_reach = v^2 * delta_sweep / (g L).
Panel (b): round-trip identified D against rollout speed, known tire in.
Panel (c): slip-angle / lateral-force coverage of the synthetic sweep.

Usage:  python3 paper/figures/make_baseline_comparison.py
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(REPO, "On-Track-SysID", "src"))

from helpers.solve_pacejka import solve_pacejka, PACEJKA_BOUNDS  # noqa: E402
from helpers.pacejka_formula import pacejka_formula  # noqa: E402

G = 9.81

# --- plant, as measured from the simulator's own per-wheel telemetry ---------
VEHICLE = dict(l_f=0.738142, l_r=0.795262, l_wb=1.533404, m=269.6, I_z=51.1)
TRUE_F = [7.0760, 1.3460, 1.0090, -2.0000]   # B, C, D, E
TRUE_R = [7.8730, 1.3830, 1.0020, -1.0240]

# --- the two rollout configurations under comparison -------------------------
SWEEP_DELTA = 0.4          # rad, baseline sweep amplitude, held fixed here
BASELINE_SPEED_CLIP = (2.0, 4.0)   # m/s, the baseline's hardcoded clip
OURS_SPEED_MIN = 7.0       # m/s, pacejka_rollout.speed_min
L_SCALED = 0.32            # m, 1:10 platform wheelbase


def mu_reach(v, delta_sweep, wheelbase):
    return v ** 2 * delta_sweep / (G * wheelbase)


def rollout(speed, delta_max, coeff_f, coeff_r, timesteps=500, dt=0.02):
    """simulated_data_gen() with the residual network held at zero."""
    l_f, l_r, l_wb = VEHICLE["l_f"], VEHICLE["l_r"], VEHICLE["l_wb"]
    m, I_z = VEHICLE["m"], VEHICLE["I_z"]
    F_zf = m * G * l_r / l_wb
    F_zr = m * G * l_f / l_wb

    v_x = np.full(timesteps, float(speed))
    v_y = np.zeros(timesteps)
    omega = np.zeros(timesteps)
    delta = np.linspace(0.0, delta_max, timesteps)

    for t in range(timesteps - 1):
        a_f = -np.arctan((v_y[t] + omega[t] * l_f) / v_x[t]) + delta[t]
        a_r = -np.arctan((v_y[t] - omega[t] * l_r) / v_x[t])
        F_f = pacejka_formula(coeff_f, a_f, F_zf)
        F_r = pacejka_formula(coeff_r, a_r, F_zr)
        v_y[t + 1] = v_y[t] + dt / m * (F_r + F_f * np.cos(delta[t]) - m * v_x[t] * omega[t])
        omega[t + 1] = omega[t] + dt / I_z * (F_f * l_f * np.cos(delta[t]) - F_r * l_r)

    return v_x, v_y, omega, delta


def refit(speed, delta_max=SWEEP_DELTA):
    """Roll a known tire out at `speed` and fit it back with the repo solver."""
    v_x, v_y, omega, delta = rollout(speed, delta_max, TRUE_F, TRUE_R)
    model = dict(VEHICLE)
    model.update(
        C_Pf_model=list(TRUE_F), C_Pr_model=list(TRUE_R),
        C_Pf_prior=list(TRUE_F), C_Pr_prior=list(TRUE_R),
        pacejka_method="trf", pacejka_num_starts=8,
        pacejka_prior_weight=0.05, pacejka_seed=0,
    )
    return solve_pacejka(model, v_x, v_y, omega, delta)


def axle_scatter(speed, delta_max=SWEEP_DELTA):
    """Front-axle (alpha, Fy) the fit actually sees at this rollout speed."""
    v_x, v_y, omega, delta = rollout(speed, delta_max, TRUE_F, TRUE_R)
    l_f, l_r, l_wb, m = VEHICLE["l_f"], VEHICLE["l_r"], VEHICLE["l_wb"], VEHICLE["m"]
    alpha_f = -np.arctan((v_y + omega * l_f) / v_x) + delta
    F_yf = m * l_r * v_x * omega / (l_wb * np.cos(delta))
    keep = (np.abs(omega) <= 5) & (np.abs(alpha_f) <= 0.5)
    return alpha_f[keep], F_yf[keep]


def main():
    speeds = np.arange(2.0, 20.01, 1.0)
    fits = {v: refit(v) for v in speeds}
    D_f = np.array([fits[v][0][2] for v in speeds])
    D_r = np.array([fits[v][1][2] for v in speeds])

    C_BASE, C_OURS, C_TRUE = "#b2182b", "#2166ac", "#333333"
    fig, ax = plt.subplots(1, 3, figsize=(11.0, 3.1))

    # (a) reachable friction ------------------------------------------------
    v = np.linspace(0.5, 20, 400)
    ax[0].plot(v, mu_reach(v, SWEEP_DELTA, VEHICLE["l_wb"]), color=C_TRUE,
               label=r"full scale, $L=1.53$ m")
    ax[0].plot(v, mu_reach(v, SWEEP_DELTA, L_SCALED), color=C_TRUE, ls="--",
               label=r"1:10, $L=0.32$ m")
    ax[0].axhline(1.0, color="#666666", ls=":", lw=1.0)
    ax[0].text(0.4, 1.05, r"plant peak $\mu\approx1.0$", fontsize=7, color="#666666")
    ax[0].axvspan(*BASELINE_SPEED_CLIP, color=C_BASE, alpha=0.16, lw=0)
    ax[0].axvspan(OURS_SPEED_MIN, 20, color=C_OURS, alpha=0.10, lw=0)
    ax[0].plot([4.0], [mu_reach(4.0, SWEEP_DELTA, VEHICLE["l_wb"])], "o",
               color=C_BASE, ms=5)
    ax[0].annotate(r"$\mu_{\mathrm{reach}}=0.43$", (4.0, 0.43),
                   textcoords="offset points", xytext=(8, -4), fontsize=7, color=C_BASE)
    ax[0].set_xlim(0, 20)
    ax[0].set_ylim(0, 3.0)
    ax[0].set_xlabel(r"rollout speed $v$ [m/s]")
    ax[0].set_ylabel(r"$\mu_{\mathrm{reach}}=v^{2}\delta_{\mathrm{sweep}}/(gL)$")
    ax[0].set_title(r"(a) what the synthetic rollout can demand", fontsize=8.5)
    ax[0].legend(fontsize=7, loc="upper left", frameon=False)

    # (b) round-trip identifiability ----------------------------------------
    ax[1].axhline(PACEJKA_BOUNDS[0][2], color="#999999", ls=":", lw=1.0)
    ax[1].axhline(PACEJKA_BOUNDS[1][2], color="#999999", ls=":", lw=1.0)
    ax[1].text(19.6, PACEJKA_BOUNDS[0][2] + 0.03, "box bound", fontsize=6.5,
               color="#999999", ha="right")
    ax[1].axhline(TRUE_F[2], color=C_TRUE, lw=1.2)
    ax[1].text(19.6, TRUE_F[2] + 0.05, r"true $D$", fontsize=7, color=C_TRUE, ha="right")
    ax[1].axvspan(*BASELINE_SPEED_CLIP, color=C_BASE, alpha=0.16, lw=0,
                  label="baseline rollout clip")
    ax[1].axvspan(OURS_SPEED_MIN, 20, color=C_OURS, alpha=0.10, lw=0,
                  label="excitation-aware rollout")
    ax[1].plot(speeds, D_f, "o-", color=C_BASE, ms=3.5, lw=1.2, label=r"identified $D_f$")
    ax[1].plot(speeds, D_r, "s--", color=C_OURS, ms=3.5, lw=1.2, label=r"identified $D_r$")
    ax[1].set_xlim(1, 20.5)
    ax[1].set_ylim(0.25, 2.15)
    ax[1].set_xlabel(r"rollout speed $v$ [m/s]")
    ax[1].set_ylabel(r"identified peak factor $D$")
    ax[1].set_title(r"(b) known tire in, fitted tire out", fontsize=8.5)
    ax[1].legend(fontsize=6.5, loc="lower right", frameon=False, ncol=1)

    # (c) excitation entering the fit ---------------------------------------
    l_r, l_wb, m = VEHICLE["l_r"], VEHICLE["l_wb"], VEHICLE["m"]
    F_zf = m * G * l_r / l_wb
    a_grid = np.linspace(0, 0.35, 300)
    ax[2].plot(a_grid, pacejka_formula(TRUE_F, a_grid, F_zf), color=C_TRUE, lw=1.4,
               label="true front tire")
    for speed, color, lab in ((4.0, C_BASE, r"baseline, $v=4$ m/s"),
                              (12.0, C_OURS, r"ours, $v=12$ m/s")):
        a, F = axle_scatter(speed)
        ax[2].plot(a, F, color=color, lw=2.4, alpha=0.75, label=lab)
    a_peak = a_grid[int(np.argmax(pacejka_formula(TRUE_F, a_grid, F_zf)))]
    ax[2].axvline(a_peak, color="#999999", ls=":", lw=1.0)
    ax[2].text(a_peak + 0.005, 200, r"peak-force $\alpha$", fontsize=6.5, color="#999999",
               rotation=90, va="bottom")
    ax[2].set_xlim(0, 0.35)
    ax[2].set_xlabel(r"front slip angle $\alpha_f$ [rad]")
    ax[2].set_ylabel(r"$F_{y,f}$ [N]")
    ax[2].set_title(r"(c) coverage the Pacejka fit sees", fontsize=8.5)
    ax[2].legend(fontsize=7, loc="lower right", frameon=False)

    for a in ax:
        a.tick_params(labelsize=7.5)
        for side in ("top", "right"):
            a.spines[side].set_visible(False)

    fig.tight_layout()
    out = os.path.join(os.path.dirname(__file__), "baseline_vs_ours.pdf")
    fig.savefig(out, bbox_inches="tight")
    print("wrote", out)

    print("\nrollout speed [m/s]   D_f     D_r    railed?")
    for v_ in speeds:
        cf, cr = fits[v_]
        railed = sum(
            abs(c - b) < 1e-3
            for coeffs in (cf, cr)
            for c, b in zip(coeffs, PACEJKA_BOUNDS[0])
        ) + sum(
            abs(c - b) < 1e-3
            for coeffs in (cf, cr)
            for c, b in zip(coeffs, PACEJKA_BOUNDS[1])
        )
        print(f"{v_:6.1f}            {cf[2]:6.3f}  {cr[2]:6.3f}   {railed}")


if __name__ == "__main__":
    main()
