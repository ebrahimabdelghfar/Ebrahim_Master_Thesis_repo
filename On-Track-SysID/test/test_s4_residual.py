"""The S4D convolution path must stay identical to the recurrent scan.

S4DBlock's training path evaluates the SSM as a closed-form impulse response
convolved with the window (`_kernel` / `forward_sequence` / `forward_last`),
while `step()` still iterates the recurrence one sample at a time for
simulated_data_gen()'s stateful rollout. Those are two implementations of one
linear system, and nothing downstream would notice them drifting apart - the
Pacejka fit would just start returning different coefficients. These tests
pin them together, in float64 so the comparison is about the algebra rather
than float32 rounding.
"""
import os
import sys

import torch

_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src')
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from helpers.s4_residual import S4DBlock, S4DResidual  # noqa: E402

TOL = 1e-9


def _scan(block, u):
    """The reference: an explicit L-step recurrence from zero initial state."""
    state_re, state_im = block.init_state(u.shape[0], device=u.device, dtype=u.dtype)
    coeffs = block._discretize()
    outputs = []
    for t in range(u.shape[1]):
        y_t, state_re, state_im = block._recur_step(u[:, t, :], state_re, state_im, coeffs)
        outputs.append(y_t)
    return torch.stack(outputs, dim=1)


def _perturbed_block(H, N, seed):
    torch.manual_seed(seed)
    block = S4DBlock(H, N).double()
    with torch.no_grad():
        for p in block.parameters():
            p.add_(torch.randn_like(p) * 0.2)  # move off the structured init
    return block


def test_conv_matches_recurrent_scan():
    for seed, (W, H, N) in enumerate([(1, 4, 4), (2, 4, 4), (20, 4, 4), (20, 3, 6), (50, 2, 8)]):
        block = _perturbed_block(H, N, seed)
        u = torch.randn(7, W, H, dtype=torch.float64)
        ref = _scan(block, u)
        assert (ref - block.forward_sequence(u)).abs().max() < TOL
        assert (ref[:, -1, :] - block.forward_last(u)).abs().max() < TOL


def test_step_matches_conv():
    block = _perturbed_block(4, 4, 11)
    u = torch.randn(5, 20, 4, dtype=torch.float64)
    state = block.init_state(5, dtype=torch.float64)
    outputs = []
    for t in range(u.shape[1]):
        y_t, state = block.step(u[:, t, :], *state)
        outputs.append(y_t)
    assert (torch.stack(outputs, dim=1) - block.forward_sequence(u)).abs().max() < TOL


def test_stack_forward_and_gradients_match_scan():
    for num_layers in (1, 2, 3):
        torch.manual_seed(num_layers)
        model = S4DResidual(d_in=4, H=4, N=4, num_layers=num_layers).double()
        x = torch.randn(32, 20, 4, dtype=torch.float64)

        h = model.act(model.input_proj(x))
        for layer in model.layers:
            h = h + model.act(_scan(layer, h))
        ref = model.output_proj(h[:, -1, :])
        assert (ref - model(x)).abs().max() < TOL

        ref.pow(2).sum().backward()
        ref_grads = {k: v.grad.clone() for k, v in model.named_parameters()}
        model.zero_grad()
        model(x).pow(2).sum().backward()
        for k, v in model.named_parameters():
            assert (v.grad - ref_grads[k]).abs().max() < 1e-8, k
