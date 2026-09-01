"""
S4D (diagonal Structured State Space) temporal-residual model.

This is the S4D variant (Gu, Goel & Re, "On the Parameterization and
Initialization of Diagonal State Space Models", arXiv:2206.11893) -- NOT the
original FFT/Cauchy-kernel S4 (Gu et al., arXiv:2111.00396). Both share the
same HiPPO-derived complex-diagonal state-transition matrix, but S4D needs
neither S4's Cauchy kernel nor its FFT convolution: a diagonal A makes the
impulse response K_l = Re(C . exp(A*dt*l) . Bbar) a closed-form Vandermonde
product, and at this pipeline's window length (W = 20) the cheapest way to
apply it is a dense causal Toeplitz matmul, not an FFT. Don't relabel this
as "the" S4 paper's exact algorithm.

Two call contracts:
  - forward(x): x (batch, W, d_in) -> (batch, 2), state always init to zero
    internally. Matches every other arch's `nn_model(X_train)` contract
    (used by train_residual_nn()/compute_physics_informed_loss()).
  - init_state(batch) / step(x_t, state): single-timestep stateful API, used
    only by train_model.py's simulated_data_gen() to carry hidden state
    across its 500-step open-loop rollout.

Real-valued arithmetic only (no torch.complex64) - every complex quantity
(state, A, C) is carried as a (*_re, *_im) real tensor pair with complex
algebra hand-expanded, to avoid any dtype/autograd friction from partial
complex-tensor support.
"""

import math

import torch
import torch.nn as nn


class S4DBlock(nn.Module):
    """H independent SISO S4D channels, complex state dimension N each."""

    def __init__(self, H, N, dt_min=0.001, dt_max=0.1):
        super().__init__()
        self.H = H
        self.N = N

        # S4D-Lin init: closed-form diagonal approximation to HiPPO-LegS.
        # Re(A_n) = -0.5 (uniform exponential memory decay), Im(A_n) = pi*n
        # (linearly increasing oscillation frequency per state index).
        n = torch.arange(N, dtype=torch.float32)
        log_A_real_init = torch.log(torch.full((N,), 0.5))
        A_imag_init = math.pi * n
        self.log_A_real = nn.Parameter(log_A_real_init.unsqueeze(0).repeat(H, 1))
        self.A_imag = nn.Parameter(A_imag_init.unsqueeze(0).repeat(H, 1))

        self.C_re = nn.Parameter(torch.randn(H, N) / math.sqrt(N))
        self.C_im = nn.Parameter(torch.randn(H, N) / math.sqrt(N))
        self.D = nn.Parameter(torch.zeros(H))

        # Per-channel discretization step, log-uniform initialized in
        # [dt_min, dt_max]. This is a free SSM hyperparameter, DISTINCT from
        # nn_params.yaml's `sample_dt` (the physical 1/rate integration step
        # used elsewhere in this package's Pacejka rollout) - don't conflate
        # the two "dt"s.
        self.dt_min = dt_min
        self.dt_max = dt_max
        log_dt_init = torch.rand(H) * (math.log(dt_max) - math.log(dt_min)) + math.log(dt_min)
        self.log_dt = nn.Parameter(log_dt_init)

    def _dt_A(self):
        """(A, A*dt) as real/imag pairs, each (H, N). Shared by the recurrent
        step and the convolution kernel."""
        A_re = -torch.exp(self.log_A_real)  # (H, N), always < 0 -> stable
        A_im = self.A_imag  # (H, N)

        dt = torch.exp(self.log_dt).clamp(self.dt_min, self.dt_max)  # (H,)
        dt = dt.unsqueeze(-1)  # (H, 1), broadcasts against (H, N)
        return A_re, A_im, A_re * dt, A_im * dt

    def _discretize(self):
        A_re, A_im, dtA_re, dtA_im = self._dt_A()

        # Zero-order hold: Abar = exp(A*dt). |Abar| = exp(Re(A)*dt) < 1 for
        # any dt > 0 since Re(A) < 0 by construction - unconditionally
        # stable, no extra clipping needed.
        decay = torch.exp(dtA_re)
        Abar_re = decay * torch.cos(dtA_im)
        Abar_im = decay * torch.sin(dtA_im)

        # Bbar = (Abar - 1) / A  (B fixed = 1, standard minimal-S4D
        # simplification - halves param count for negligible expressiveness
        # loss at this scale, since C is already fully learnable).
        p_re, p_im = Abar_re - 1.0, Abar_im
        q_re, q_im = A_re, A_im
        denom = q_re * q_re + q_im * q_im
        Bbar_re = (p_re * q_re + p_im * q_im) / denom
        Bbar_im = (p_im * q_re - p_re * q_im) / denom

        return Abar_re, Abar_im, Bbar_re, Bbar_im

    def _kernel(self, L):
        """Causal impulse response K[h, l] = Re(C_h * Abar_h^l * Bbar_h), (H, L).

        Same LTI system _recur_step iterates: from zero initial state,
        y_t = sum_{l<=t} K[:, l] * u_{t-l} + D * u_t. Abar^l = exp(A*dt*l) has
        a closed form, so the whole length-L response is one Vandermonde
        product rather than an L-step Python scan.
        """
        _, _, Bbar_re, Bbar_im = self._discretize()
        _, _, dtA_re, dtA_im = self._dt_A()

        CB_re = self.C_re * Bbar_re - self.C_im * Bbar_im
        CB_im = self.C_re * Bbar_im + self.C_im * Bbar_re

        l = torch.arange(L, device=dtA_re.device, dtype=dtA_re.dtype)
        mag = torch.exp(dtA_re.unsqueeze(-1) * l)  # (H, N, L)
        ang = dtA_im.unsqueeze(-1) * l
        K = CB_re.unsqueeze(-1) * (mag * torch.cos(ang)) - CB_im.unsqueeze(-1) * (mag * torch.sin(ang))
        return K.sum(1)  # (H, L)

    def init_state(self, batch_size, device=None, dtype=None):
        shape = (batch_size, self.H, self.N)
        state_re = torch.zeros(shape, device=device, dtype=dtype)
        state_im = torch.zeros(shape, device=device, dtype=dtype)
        return state_re, state_im

    def _recur_step(self, u_t, state_re, state_im, coeffs):
        Abar_re, Abar_im, Bbar_re, Bbar_im = coeffs
        u = u_t.unsqueeze(-1)  # (batch, H, 1), broadcasts against (H, N)

        new_re = Abar_re * state_re - Abar_im * state_im + Bbar_re * u
        new_im = Abar_re * state_im + Abar_im * state_re + Bbar_im * u

        y_t = (self.C_re * new_re - self.C_im * new_im).sum(-1) + self.D * u_t
        return y_t, new_re, new_im

    def step(self, u_t, state_re, state_im):
        """u_t: (batch, H) single timestep -> y_t: (batch, H), new state."""
        coeffs = self._discretize()
        y_t, new_re, new_im = self._recur_step(u_t, state_re, state_im, coeffs)
        return y_t, (new_re, new_im)

    def _causal_toeplitz(self, W, device, dtype):
        """(H, W, W) lower-triangular Toeplitz of the impulse response, so that
        y = T @ u is the causal convolution from a zero initial state."""
        K = self._kernel(W)  # (H, W)
        idx = torch.arange(W, device=device)
        lag = idx.unsqueeze(1) - idx.unsqueeze(0)  # (W, W), t - s
        return K[:, lag.clamp(min=0)] * (lag >= 0).to(dtype)

    def forward_sequence(self, u):
        """u: (batch, W, H) -> y_seq: (batch, W, H), zero initial state."""
        T = self._causal_toeplitz(u.shape[1], u.device, u.dtype)
        y = torch.matmul(T, u.permute(0, 2, 1).unsqueeze(-1)).squeeze(-1)  # (batch, H, W)
        return y.permute(0, 2, 1) + self.D * u

    def forward_last(self, u):
        """u: (batch, W, H) -> y at the final timestep only: (batch, H).

        The stack's last layer only ever reads y_seq[:, -1, :], so contracting
        the reversed kernel straight onto the window skips the other W-1
        outputs entirely.
        """
        W = u.shape[1]
        K_rev = self._kernel(W).flip(-1)  # (H, W), K[h, W-1-s]
        y_last = torch.einsum('hs,bsh->bh', K_rev, u)
        return y_last + self.D * u[:, -1, :]


class S4DResidual(nn.Module):
    """Input proj -> stacked (S4DBlock + residual + LeakyReLU) -> output proj to [d_vy, d_omega]."""

    def __init__(self, d_in, H=4, N=4, num_layers=1, dt_min=0.001, dt_max=0.1, leaky_relu_slope=0.01):
        super().__init__()
        self.d_in = d_in
        self.H = H
        self.num_layers = num_layers

        self.input_proj = nn.Linear(d_in, H)
        self.layers = nn.ModuleList([S4DBlock(H, N, dt_min, dt_max) for _ in range(num_layers)])
        self.act = nn.LeakyReLU(leaky_relu_slope)
        self.output_proj = nn.Linear(H, 2)

    def forward(self, x):
        """x: (batch, W, d_in) -> (batch, 2), residual at the window's last timestep."""
        h = self.act(self.input_proj(x))  # (batch, W, H)
        for layer in self.layers[:-1]:
            h = h + self.act(layer.forward_sequence(h))
        last = h[:, -1, :] + self.act(self.layers[-1].forward_last(h))  # (batch, H)
        return self.output_proj(last)

    def init_state(self, batch_size, device=None, dtype=None):
        return [layer.init_state(batch_size, device=device, dtype=dtype) for layer in self.layers]

    def step(self, x_t, state):
        """x_t: (batch, d_in) single timestep, state from init_state()/previous step().

        Returns (out: (batch, 2), new_state).
        """
        h = self.act(self.input_proj(x_t))  # (batch, H)
        new_state = []
        for layer, (state_re, state_im) in zip(self.layers, state):
            y_t, (new_re, new_im) = layer.step(h, state_re, state_im)
            h = h + self.act(y_t)
            new_state.append((new_re, new_im))
        out = self.output_proj(h)
        return out, new_state
