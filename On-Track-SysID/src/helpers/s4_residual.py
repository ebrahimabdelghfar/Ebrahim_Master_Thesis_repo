"""
S4D (diagonal Structured State Space) temporal-residual model.

This is the S4D variant (Gu, Goel & Re, "On the Parameterization and
Initialization of Diagonal State Space Models", arXiv:2206.11893) -- NOT the
original FFT/Cauchy-kernel S4 (Gu et al., arXiv:2111.00396). Both share the
same HiPPO-derived complex-diagonal state-transition matrix, but S4D is
evaluated with a plain causal recurrent scan instead of an FFT convolution,
which is the right complexity/risk tradeoff for this pipeline's short
(~1500-sample) training sequences, vs. S4's machinery which targets
sequences of thousands-to-millions of steps. Don't relabel this as "the" S4
paper's exact algorithm.

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

    def _discretize(self):
        A_re = -torch.exp(self.log_A_real)  # (H, N), always < 0 -> stable
        A_im = self.A_imag  # (H, N)

        dt = torch.exp(self.log_dt).clamp(self.dt_min, self.dt_max)  # (H,)
        dt = dt.unsqueeze(-1)  # (H, 1), broadcasts against (H, N)

        # Zero-order hold: Abar = exp(A*dt). |Abar| = exp(Re(A)*dt) < 1 for
        # any dt > 0 since Re(A) < 0 by construction - unconditionally
        # stable, no extra clipping needed.
        decay = torch.exp(A_re * dt)
        Abar_re = decay * torch.cos(A_im * dt)
        Abar_im = decay * torch.sin(A_im * dt)

        # Bbar = (Abar - 1) / A  (B fixed = 1, standard minimal-S4D
        # simplification - halves param count for negligible expressiveness
        # loss at this scale, since C is already fully learnable).
        p_re, p_im = Abar_re - 1.0, Abar_im
        q_re, q_im = A_re, A_im
        denom = q_re * q_re + q_im * q_im
        Bbar_re = (p_re * q_re + p_im * q_im) / denom
        Bbar_im = (p_im * q_re - p_re * q_im) / denom

        return Abar_re, Abar_im, Bbar_re, Bbar_im

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

    def forward_sequence(self, u, state_re=None, state_im=None):
        """u: (batch, W, H) -> y_seq: (batch, W, H), final (state_re, state_im)."""
        batch = u.shape[0]
        if state_re is None:
            state_re, state_im = self.init_state(batch, device=u.device, dtype=u.dtype)

        coeffs = self._discretize()
        outputs = []
        for t in range(u.shape[1]):
            y_t, state_re, state_im = self._recur_step(u[:, t, :], state_re, state_im, coeffs)
            outputs.append(y_t)
        y_seq = torch.stack(outputs, dim=1)  # (batch, W, H)
        return y_seq, (state_re, state_im)


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
        for layer in self.layers:
            y_seq, _ = layer.forward_sequence(h)
            h = h + self.act(y_seq)
        last = h[:, -1, :]  # (batch, H)
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
