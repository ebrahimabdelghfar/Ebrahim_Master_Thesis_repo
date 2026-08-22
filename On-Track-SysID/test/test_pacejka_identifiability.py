"""Regression tests for the degenerate Pacejka fit (2026-08-22).

Three separate identifications in this repo's history collapsed onto the
coefficient box edges and were only caught downstream, as an oscillating or
unsolvable MPC. The signature is always the same: the data (here, the
synthetic steering sweep that solve_pacejka() actually fits) never reaches
the tire's peak, so only the product B*C*D is identifiable and the solver
parks the individual coefficients wherever the bounds stop it.

These tests reproduce the mechanism in-process, with no ROS and no torch.
"""
import os
import sys

import numpy as np
import pytest
import yaml

_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src')
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from helpers.pacejka_formula import pacejka_formula          # noqa: E402
from helpers.solve_pacejka import PACEJKA_BOUNDS, solve_pacejka  # noqa: E402

G = 9.81
PARAMS_YAML = os.path.join(os.path.dirname(_SRC), 'params', 'pacejka_params.yaml')
MODEL_TXT = os.path.join(os.path.dirname(_SRC), 'models', 'SIM', 'SIM_pacejka.txt')


def _load(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def _vehicle(cpf, cpr, params=None):
    veh = _load(MODEL_TXT)
    model = {
        'C_Pf_model': list(cpf), 'C_Pr_model': list(cpr),
        'l_f': veh['l_f'], 'l_r': veh['l_r'], 'l_wb': veh['l_wb'],
        'm': veh['m'], 'I_z': veh['I_z'], 'racecar_version': 'SIM',
        'pacejka_method': 'trf', 'pacejka_num_starts': 4, 'pacejka_seed': 0,
        'pacejka_prior_weight': _load(PARAMS_YAML)['pacejka_solver'].get('prior_weight', 0.0),
    }
    model.update(params or {})
    return model


def _rollout(model, vx_const, timesteps=500, dt=0.02, delta_max=0.4):
    """simulated_data_gen()'s integration with the residual NN set to zero."""
    lf, lr, lwb, m, Iz = model['l_f'], model['l_r'], model['l_wb'], model['m'], model['I_z']
    Fzf, Fzr = m * G * lr / lwb, m * G * lf / lwb
    vy, om = np.zeros(timesteps), np.zeros(timesteps)
    vx = np.full(timesteps, vx_const)
    delta = np.linspace(0.0, delta_max, timesteps)
    for t in range(timesteps - 1):
        af = -np.arctan((vy[t] + om[t] * lf) / vx[t]) + delta[t]
        ar = -np.arctan((vy[t] - om[t] * lr) / vx[t])
        Ff = pacejka_formula(model['C_Pf_model'], af, Fzf)
        Fr = pacejka_formula(model['C_Pr_model'], ar, Fzr)
        vy[t + 1] = vy[t] + dt * (Fr + Ff * np.cos(delta[t]) - m * vx[t] * om[t]) / m
        om[t + 1] = om[t] + dt * (Ff * lf * np.cos(delta[t]) - Fr * lr) / Iz
    return vx, vy, om, delta


def _on_bound(coeffs, tol=1e-3):
    lo, hi = PACEJKA_BOUNDS
    return [name for i, name in enumerate('BCDE')
            if abs(coeffs[i] - lo[i]) < tol or abs(coeffs[i] - hi[i]) < tol]


def test_configured_prior_is_inside_the_bounds():
    """A prior on (or outside) a bound makes the solver start on the rail.

    pacejka_params.yaml used to ship D = 0.3882 / 0.3752 against a D lower
    bound of 0.4, so np.clip() in _fit_pacejka_axle put iteration 1 exactly
    on the rail with a single start to escape from.
    """
    prior = _load(PARAMS_YAML)['pacejka_model']
    lo, hi = PACEJKA_BOUNDS
    for key in ('C_Pf_model', 'C_Pr_model'):
        for i, name in enumerate('BCDE'):
            v = prior[key][i]
            assert lo[i] < v < hi[i], (
                f"{key}[{name}] = {v} is on or outside its bound "
                f"({lo[i]}, {hi[i]}) - the fit will start on the rail")


def test_shipped_config_rollout_reaches_the_tire_peak():
    """pacejka_rollout must let the synthetic sweep saturate the tires.

    Reachable mu is v^2 * sweep_delta_max / (g * l_wb); below the prior's own
    D the sweep stays on the linear ramp and D is unidentifiable.
    """
    cfg = _load(PARAMS_YAML)
    roll = cfg.get('pacejka_rollout', {})
    veh = _load(MODEL_TXT)
    v = float(roll.get('speed_min', 2.0))
    delta_max = float(roll.get('sweep_delta_max', 0.4))
    mu_kin = v ** 2 * delta_max / (G * veh['l_wb'])
    d_prior = max(cfg['pacejka_model']['C_Pf_model'][2], cfg['pacejka_model']['C_Pr_model'][2])
    assert mu_kin >= d_prior, (
        f"rollout at speed_min={v} m/s reaches only mu={mu_kin:.2f} but the prior peaks at "
        f"D={d_prior:.2f}")


@pytest.mark.parametrize('cpf,cpr', [
    ([7.076, 1.346, 1.009, -2.0], [7.873, 1.383, 1.002, -1.024]),   # measured SIM prior
    ([6.3, 1.4, 1.5, 0.0], [6.3, 1.4, 1.5, 0.0]),                   # a higher-grip tire
])
def test_fit_recovers_the_prior_at_operating_speed(cpf, cpr):
    """At the speed the car is actually driven, nothing lands on a bound."""
    model = _vehicle(cpf, cpr)
    vx, vy, om, delta = _rollout(model, 13.0)
    got_f, got_r = solve_pacejka(model, vx, vy, om, delta)
    assert not _on_bound(got_f), f"front coefficients on a bound: {got_f}"
    assert not _on_bound(got_r), f"rear coefficients on a bound: {got_r}"
    # D is the coefficient the MPC's grip gate keys off - it has to come back.
    assert got_f[2] == pytest.approx(cpf[2], rel=0.25)
    assert got_r[2] == pytest.approx(cpr[2], rel=0.25)


def test_unregularised_low_speed_rollout_rails_D():
    """Documents the original failure - do not 'fix' this by relaxing a bound.

    At 4 m/s (the old hardcoded np.clip(mean(vx), 2.0, 4)) the sweep can only
    reach mu = 0.43 on this vehicle, so with no prior term a D = 1.5 tire
    comes back as D <= 0.8 with coefficients pinned on the box.
    """
    model = _vehicle([6.3, 1.4, 1.5, 0.0], [6.3, 1.4, 1.5, 0.0],
                     {'pacejka_prior_weight': 0.0})
    vx, vy, om, delta = _rollout(model, 4.0)
    got_f, got_r = solve_pacejka(model, vx, vy, om, delta)
    assert got_f[2] < 1.0 or got_r[2] < 1.0
    assert _on_bound(got_f) or _on_bound(got_r)


def test_regularised_uninformative_rollout_returns_the_prior():
    """When the data cannot see D, the fit must keep the prior, not a rail.

    Same non-informative 4 m/s rollout as above, with the shipped
    prior_weight: the answer is allowed to be uninformative, but it has to be
    uninformative in the direction of the nominal model rather than the box
    edge, because the MPC's grip gate keys off D.
    """
    cpf = cpr = [6.3, 1.4, 1.5, 0.0]
    model = _vehicle(cpf, cpr)
    vx, vy, om, delta = _rollout(model, 4.0)
    got_f, got_r = solve_pacejka(model, vx, vy, om, delta)
    assert not _on_bound(got_f), f"front coefficients on a bound: {got_f}"
    assert not _on_bound(got_r), f"rear coefficients on a bound: {got_r}"
    assert got_f[2] == pytest.approx(cpf[2], rel=0.2)
    assert got_r[2] == pytest.approx(cpr[2], rel=0.2)


def test_grip_ceiling_matches_the_measured_vehicle():
    """The shipped prior must reproduce the grip measured on the real plant.

    Measured on the CARLA asurt_fsai on 2026-08-22 from CARLA's own per-wheel
    telemetry over a limit run: peak axle mu 1.05, steady-state grip ceiling
    (D_f*F_zf + D_r*F_zr)/m ~= 1.0 g.
    """
    cfg = _load(PARAMS_YAML)['pacejka_model']
    veh = _load(MODEL_TXT)
    Fzf = veh['m'] * G * veh['l_r'] / veh['l_wb']
    Fzr = veh['m'] * G * veh['l_f'] / veh['l_wb']
    ceiling = (cfg['C_Pf_model'][2] * Fzf + cfg['C_Pr_model'][2] * Fzr) / veh['m']
    assert 0.9 * G <= ceiling <= 1.15 * G, f"grip ceiling {ceiling:.2f} m/s^2 off the measured 1.0 g"


def test_wheelbase_is_the_sum_of_the_axle_distances():
    """analyse_tires() uses l_wb for F_z and l_f + l_r for the force split."""
    veh = _load(MODEL_TXT)
    assert veh['l_wb'] == pytest.approx(veh['l_f'] + veh['l_r'], abs=1e-6)
