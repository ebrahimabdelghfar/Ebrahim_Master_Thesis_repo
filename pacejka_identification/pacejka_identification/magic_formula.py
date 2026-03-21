#!/usr/bin/env python3
"""
Pacejka Magic Formula tire model functions.

Implements the standard Magic Formula:
    Y(x) = Fz * D * sin(C * arctan(B*x - E*(B*x - arctan(B*x))))

where x is the slip variable (slip angle α for lateral, slip ratio κ for longitudinal)
and [B, C, D, E] are the coefficients to be identified.

Reference: Pacejka, H.B. "Tire and Vehicle Dynamics", 3rd Edition.
"""

import numpy as np


def pacejka_formula(params, slip, fz):
    """
    General Pacejka Magic Formula.

    Parameters
    ----------
    params : array-like of length 4
        [B, C, D, E] coefficients.
    slip : float or ndarray
        Slip variable (slip angle α in rad, or longitudinal slip ratio κ).
    fz : float or ndarray
        Normal force (vertical load) in Newtons.

    Returns
    -------
    float or ndarray
        Computed force or moment.
    """
    B, C, D, E = params[0], params[1], params[2], params[3]
    x = B * slip
    return fz * D * np.sin(C * np.arctan(x - E * (x - np.arctan(x))))


def pacejka_fy(params, alpha, fz):
    """
    Lateral force Fy from slip angle α and normal load Fz.

    Fy = Fz * D * sin(C * arctan(B*α - E*(B*α - arctan(B*α))))
    """
    return pacejka_formula(params, alpha, fz)


def pacejka_fx(params, kappa, fz):
    """
    Longitudinal force Fx from slip ratio κ and normal load Fz.

    Fx = Fz * D * sin(C * arctan(B*κ - E*(B*κ - arctan(B*κ))))
    """
    return pacejka_formula(params, kappa, fz)


def pacejka_mz(params, alpha, fz):
    """
    Self-aligning torque Mz from slip angle α and normal load Fz.

    Mz = Fz * D * sin(C * arctan(B*α - E*(B*α - arctan(B*α))))
    """
    return pacejka_formula(params, alpha, fz)
