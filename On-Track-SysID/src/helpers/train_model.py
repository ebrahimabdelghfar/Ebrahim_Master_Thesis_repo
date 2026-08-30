"""
Residual NN for On-Track Pacejka SysID.
Paper: arXiv:2411.17508v1 — Learning-Based On-Track SysID for Scaled Autonomous Racing.

Supported architectures (set via nn_params.yaml -> nn_architecture):
  baseline        : 4->8->2  MLP, 58 params  [paper original]
  wide            : 4->16->2 MLP, 114 params [drop-in, weight_decay regularized]
  physics_inputs  : 6->8->2  MLP, 74 params  [slip-angle augmented inputs, RECOMMENDED]
  ensemble        : 3x(4->8->2), 174 params  [averaged ensemble, best noise robustness]
  s4              : S4D temporal-residual (see helpers/s4_residual.py), ~102-110 params
                    [windowed sequence input, carries hidden state through
                    simulated_data_gen() - see s4_residual.py's module docstring
                    for the S4D-vs-S4 scoping note. Not combinable with ensemble.]

NN role: intermediate residual corrector only.
Final model artifact: Pacejka parameters [Bf,Cf,Df,Ef,Br,Cr,Dr,Er].
Signal contract: inputs [vx,vy,omega,delta], outputs [e_vy,e_omega].
Units: m/s, m/s, rad/s, rad -> m/s, rad/s.
"""

import os
import time
import numpy as np
import matplotlib.pyplot as plt
import torch
import torch.nn as nn
import yaml
from scipy.signal import butter, filtfilt
from torch.optim import Adam
from torch.utils.data import TensorDataset, DataLoader
from helpers.generate_predictions import generate_predictions
from helpers.generate_inputs_errors import generate_inputs_errors, generate_sequence_windows
from helpers.data_processing import compute_slip_angles
from helpers.pacejka_formula import pacejka_formula
from helpers.plot_results import plot_results
from helpers.solve_pacejka import solve_pacejka, PACEJKA_BOUNDS
from helpers.save_model import save
from helpers.load_model import get_dotdict
from helpers.simulate_model import LookupGenerator
from helpers.s4_residual import S4DResidual
from tqdm import tqdm

# Use ament_index for package path lookup
try:
    from ament_index_python.packages import get_package_share_directory
    USE_AMENT = True
except ImportError:
    USE_AMENT = False

import logging
logger = logging.getLogger(__name__)

def log_info(msg):
    logger.info(msg)
    print(f"[INFO] {msg}")

def log_warn(msg):
    logger.warning(msg)
    print(f"[WARN] {msg}")

PACEJKA_COEFF_NAMES = ('B', 'C', 'D', 'E')


def warn_railed_fit(axle, coeffs, rtol=1e-3):
    """Warn when an identified axle sits on the edge of PACEJKA_BOUNDS.

    A coefficient that stops exactly on its box constraint was not identified
    by the data - the optimiser simply ran out of room, and the value carries
    no information beyond the bound itself. Two of these on one axle is enough
    to invent an axle stiffness that the car does not have: the 2026-08-30 fit
    returned front [19.18, 2.2, 2.0, -3.0] (C, D and E all railed, B at 96 % of
    its bound), which is C_front = 103 kN/rad against C_rear = 17 kN/rad - an
    oversteering model whose critical speed is 16.4 m/s, below the MPC's own
    speed_max. mpc_node rejects it, so the adaptive stack keeps falling back to
    the startup prior with nothing saying why.
    """
    lb, ub = PACEJKA_BOUNDS
    railed = []
    for name, value, low, high in zip(PACEJKA_COEFF_NAMES, coeffs, lb, ub):
        span = max(high - low, 1e-9)
        if abs(value - low) <= rtol * span:
            railed.append(f"{name}={value:.4g} (lower bound {low})")
        elif abs(value - high) <= rtol * span:
            railed.append(f"{name}={value:.4g} (upper bound {high})")
    if railed:
        log_warn(
            f"{axle} Pacejka fit is DEGENERATE: {len(railed)}/4 coefficients stopped on "
            f"their bounds - {', '.join(railed)}. These are not identified values, they are "
            "the box constraint. The excitation does not determine this axle; widen the "
            "steering/speed sweep or re-collect data before trusting the result."
        )
    return railed


def resolve_device(params: dict) -> torch.device:
    """nn_params.yaml's `device` key: auto | cpu | cuda. `auto` (default) picks
    cuda if torch.cuda.is_available(), else cpu. `cuda` requested with no GPU
    available falls back to cpu with a warning rather than crashing."""
    choice = (params or {}).get('device', 'auto')
    if choice == 'cpu':
        return torch.device('cpu')
    if choice == 'cuda':
        if not torch.cuda.is_available():
            log_warn("nn_params.yaml device: cuda requested but torch.cuda.is_available() "
                      "is False (no GPU, or a CPU-only torch build) - falling back to cpu.")
            return torch.device('cpu')
        return torch.device('cuda')
    return torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

def get_package_path(package_name='on_track_sys_id'):
    if USE_AMENT:
        try:
            return get_package_share_directory(package_name)
        except Exception:
            pass
    current_dir = os.path.dirname(os.path.abspath(__file__))
    package_root = os.path.dirname(os.path.dirname(current_dir))
    if os.path.exists(os.path.join(package_root, 'params')):
        return package_root
    package_root = os.path.dirname(current_dir)
    if os.path.exists(os.path.join(package_root, 'params')):
        return package_root
    return current_dir

def filter_data(training_data, model):
    b, a = butter(N=3, Wn=0.1, btype='low')
    training_data[:,0] = filtfilt(b, a, training_data[:,0])
    training_data[:,1] = filtfilt(b, a, training_data[:,1])
    training_data[:,2] = filtfilt(b, a, training_data[:,2])
    training_data[:,3] = filtfilt(b, a, training_data[:,3])
    training_data[:,3] = np.roll(training_data[:,3], 5)
    training_data = training_data[5:,:]
    if model["racecar_version"] != "SIM":
        training_data[:,1] = training_data[:,1] + model["l_r"] * training_data[:,2]
    return training_data

def negate_data(training_data):
    negate_data = np.zeros((2*training_data.shape[0], training_data.shape[1]))
    negate_data[:,0] = np.append(training_data[:,0], training_data[:,0])
    negate_data[:,1] = np.append(training_data[:,1], -training_data[:,1])
    negate_data[:,2] = np.append(training_data[:,2], -training_data[:,2])
    negate_data[:,3] = np.append(training_data[:,3], -training_data[:,3])
    return negate_data

def process_data(training_data, model):
    filtered_data = filter_data(training_data, model)
    negated_data = negate_data(filtered_data)
    return negated_data

def torch_pacejka_force(params, alpha, F_z):
    B = float(params[0])
    C = float(params[1])
    D = float(params[2])
    E = float(params[3])
    return F_z * D * torch.sin(C * torch.atan(B * alpha - E * (B * alpha - torch.atan(B * alpha))))

def _last_step(X):
    """(N,W,d) windowed s4 input -> (N,d) at the window's last timestep; (N,d) passes through."""
    return X[:, -1, :] if X.dim() == 3 else X

def compute_physics_informed_loss(nn_model, X_train, y_train, model, dt, lambda_cfg):
    criterion = nn.MSELoss()
    outputs = nn_model(X_train)
    data_loss = criterion(outputs, y_train)

    X_state = _last_step(X_train)
    v_x = X_state[:, 0]
    v_y = X_state[:, 1]
    omega = X_state[:, 2]
    delta = X_state[:, 3]

    m = float(model['m'])
    I_z = float(model['I_z'])
    l_f = float(model['l_f'])
    l_r = float(model['l_r'])
    l_wb = float(model['l_wb'])
    g_ = 9.81
    F_zf = m * g_ * l_r / l_wb
    F_zr = m * g_ * l_f / l_wb

    v_x_safe = torch.where(torch.abs(v_x) < 0.3, torch.sign(v_x) * 0.3, v_x)
    v_x_safe = torch.where(v_x_safe == 0.0, torch.full_like(v_x_safe, 0.3), v_x_safe)

    alpha_f_nom = -torch.atan((v_y + omega * l_f) / v_x_safe) + delta
    alpha_r_nom = -torch.atan((v_y - omega * l_r) / v_x_safe)
    F_f_nom = torch_pacejka_force(model['C_Pf_model'], alpha_f_nom, F_zf)
    F_r_nom = torch_pacejka_force(model['C_Pr_model'], alpha_r_nom, F_zr)

    v_y_dot_nom = (1.0 / m) * (F_r_nom + F_f_nom * torch.cos(delta) - m * v_x * omega)
    omega_dot_nom = (1.0 / I_z) * (F_f_nom * l_f * torch.cos(delta) - F_r_nom * l_r)
    v_y_next_nom = v_y + v_y_dot_nom * dt
    omega_next_nom = omega + omega_dot_nom * dt

    v_y_next = v_y_next_nom + outputs[:, 0]
    omega_next = omega_next_nom + outputs[:, 1]
    v_y_dot = (v_y_next - v_y) / dt
    omega_dot = (omega_next - omega) / dt

    alpha_f_next = -torch.atan((v_y_next + omega_next * l_f) / v_x_safe) + delta
    alpha_r_next = -torch.atan((v_y_next - omega_next * l_r) / v_x_safe)
    F_f_next = torch_pacejka_force(model['C_Pf_model'], alpha_f_next, F_zf)
    F_r_next = torch_pacejka_force(model['C_Pr_model'], alpha_r_next, F_zr)

    lat_dyn_residual = m * v_y_dot + m * v_x * omega_next - (F_r_next + F_f_next * torch.cos(delta))
    yaw_dyn_residual = I_z * omega_dot - (F_f_next * l_f * torch.cos(delta) - F_r_next * l_r)

    steady_vy_dot_th = float(lambda_cfg.get('steady_vy_dot_threshold', 0.8))
    steady_omega_dot_th = float(lambda_cfg.get('steady_omega_dot_threshold', 6.0))
    steady_mask = (torch.abs(v_y_dot) < steady_vy_dot_th) & (torch.abs(omega_dot) < steady_omega_dot_th)

    if torch.any(steady_mask):
        steady_loss = torch.mean(lat_dyn_residual[steady_mask] ** 2) + torch.mean(yaw_dyn_residual[steady_mask] ** 2)
    else:
        steady_loss = torch.mean(lat_dyn_residual ** 2) + torch.mean(yaw_dyn_residual ** 2)

    X_mirror = X_train.clone()
    X_mirror[..., 1] = -X_mirror[..., 1]
    X_mirror[..., 2] = -X_mirror[..., 2]
    X_mirror[..., 3] = -X_mirror[..., 3]
    if X_mirror.shape[-1] >= 6:
        X_mirror[..., 4] = -X_mirror[..., 4]
        X_mirror[..., 5] = -X_mirror[..., 5]
    outputs_mirror = nn_model(X_mirror)
    symmetry_loss = torch.mean((outputs + outputs_mirror) ** 2)

    X_smooth = X_train.detach().clone().requires_grad_(True)
    outputs_smooth = nn_model(X_smooth)
    grad_vy = torch.autograd.grad(outputs_smooth[:, 0].sum(), X_smooth, create_graph=True)[0]
    grad_omega = torch.autograd.grad(outputs_smooth[:, 1].sum(), X_smooth, create_graph=True)[0]
    smoothness_loss = torch.mean(grad_vy ** 2) + torch.mean(grad_omega ** 2)

    lambda_steady = float(lambda_cfg.get('lambda_steady', 0.1))
    lambda_symmetry = float(lambda_cfg.get('lambda_symmetry', 0.05))
    lambda_smoothness = float(lambda_cfg.get('lambda_smoothness', 1e-4))

    total_loss = data_loss + lambda_steady * steady_loss + lambda_symmetry * symmetry_loss + lambda_smoothness * smoothness_loss
    loss_terms = {
        'data': float(data_loss.detach().cpu()),
        'steady': float(steady_loss.detach().cpu()),
        'symmetry': float(symmetry_loss.detach().cpu()),
        'smoothness': float(smoothness_loss.detach().cpu())
    }
    return total_loss, loss_terms

# --- ARCHITECTURES ---
def init_weights(m):
    if isinstance(m, nn.Linear):
        nn.init.normal_(m.weight, mean=0.0, std=0.01)
        if m.bias is not None:
            nn.init.constant_(m.bias, 0.0)

class BaselineMLP(nn.Module):
    def __init__(self, leaky_relu_slope):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(4, 8),
            nn.LeakyReLU(leaky_relu_slope),
            nn.Linear(8, 2)
        )
        self.apply(init_weights)
    def forward(self, x):
        return self.net(x)

class WideMLP(nn.Module):
    def __init__(self, leaky_relu_slope):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(4, 16),
            nn.LeakyReLU(leaky_relu_slope),
            nn.Linear(16, 2)
        )
        self.apply(init_weights)
    def forward(self, x):
        return self.net(x)

class PhysicsInputMLP(nn.Module):
    def __init__(self, leaky_relu_slope, lf, lr):
        super().__init__()
        self.lf = lf
        self.lr = lr
        self.net = nn.Sequential(
            nn.Linear(6, 8),
            nn.LeakyReLU(leaky_relu_slope),
            nn.Linear(8, 2)
        )
        self.apply(init_weights)
    def forward(self, x):
        return self.net(x)

class EnsembleMLP(nn.Module):
    def __init__(self, leaky_relu_slope, n_members, base_seed):
        super().__init__()
        self.n_members = n_members
        self.base_seed = base_seed
        self.members = nn.ModuleList([
            nn.Sequential(
                nn.Linear(4, 8),
                nn.LeakyReLU(leaky_relu_slope),
                nn.Linear(8, 2)
            ) for _ in range(n_members)
        ])
        self.apply(init_weights)
    def forward(self, x):
        preds = torch.stack([member(x) for member in self.members], dim=0)
        return torch.mean(preds, dim=0)

def build_model(params: dict) -> nn.Module:
    arch = params.get('nn_architecture', 'baseline')
    slope = params.get('leaky_relu_slope', 0.01)
    if arch == 'baseline':
        return BaselineMLP(slope)
    elif arch == 'wide':
        return WideMLP(slope)
    elif arch == 'physics_inputs':
        lf = params.get('physics_inputs_lf', 0.15795)
        lr = params.get('physics_inputs_lr', 0.17205)
        return PhysicsInputMLP(slope, lf, lr)
    elif arch == 'ensemble':
        n_members = params.get('ensemble_n_members', 3)
        base_seed = params.get('ensemble_base_seed', 42)
        return EnsembleMLP(slope, n_members, base_seed)
    elif arch == 's4':
        s4_cfg = params.get('s4', {}) or {}
        d_in = 6 if s4_cfg.get('use_physics_inputs', False) else 4
        return S4DResidual(
            d_in=d_in,
            H=s4_cfg.get('num_channels', 4),
            N=s4_cfg.get('state_dim', 4),
            num_layers=s4_cfg.get('num_layers', 1),
            dt_min=s4_cfg.get('dt_min', 0.001),
            dt_max=s4_cfg.get('dt_max', 0.1),
            leaky_relu_slope=slope,
        )
    else:
        raise ValueError(f"Unknown architecture: {arch}")

def preprocess_inputs(X_raw, params: dict, vehicle_model=None) -> np.ndarray:
    arch = params.get('nn_architecture', 'baseline')
    s4_cfg = params.get('s4', {}) or {}
    augment = (arch == 'physics_inputs') or (arch == 's4' and s4_cfg.get('use_physics_inputs', False))

    if isinstance(X_raw, torch.Tensor):
        X_np = X_raw.detach().cpu().numpy()
    else:
        X_np = np.asarray(X_raw)

    if not augment:
        return X_np

    if vehicle_model is not None:
        lf = vehicle_model['l_f']
        lr = vehicle_model['l_r']
    else:
        lf = params.get('physics_inputs_lf', 0.15795)
        lr = params.get('physics_inputs_lr', 0.17205)

    # Ellipsis-indexed: works uniformly whether X_np is a single row (4,),
    # a flat batch (N,4), or an s4 window batch (N,W,4) - compute_slip_angles
    # is already elementwise numpy, so it's rank-agnostic too.
    vx, vy, omega, delta = X_np[..., 0], X_np[..., 1], X_np[..., 2], X_np[..., 3]
    alpha_f, alpha_r = compute_slip_angles(vx, vy, omega, delta, lf, lr)

    if X_np.ndim == 1:
        return np.concatenate([X_np, [alpha_f, alpha_r]])
    return np.concatenate([X_np, alpha_f[..., np.newaxis], alpha_r[..., np.newaxis]], axis=-1)

def train_residual_nn(X_train_raw, y_train, params: dict, vehicle_model=None, current_iteration=None, total_iterations=None) -> nn.Module:
    device = resolve_device(params)
    model = build_model(params).to(device)

    X_train_np = preprocess_inputs(X_train_raw, params, vehicle_model)
    X_t = torch.tensor(X_train_np, dtype=torch.float32, device=device)
    y_t = (torch.tensor(y_train, dtype=torch.float32, device=device) if not isinstance(y_train, torch.Tensor)
           else y_train.clone().detach().to(dtype=torch.float32, device=device))

    arch = params.get('nn_architecture', 'baseline')
    epochs = params.get('epochs', 200)
    lr = params.get('learning_rate', 0.0005)
    batch_size = params.get('batch_size', -1)
    weight_decay = 1e-4 if arch == 'wide' else params.get('weight_decay', 0.0)
    early_stopping = params.get('early_stopping', False)
    patience = params.get('early_stopping_patience', 20)
    min_delta = params.get('early_stopping_min_delta', 1e-6)
    log_per_iteration = params.get('log_per_iteration', True)
    loss_mode = params.get('loss_mode', 'mse')
    physics_loss_cfg = params.get('physics_loss', {})
    sample_dt = float(params.get('sample_dt', 0.02))

    if log_per_iteration:
        log_info(f"Training on device: {device}")

    if arch == 'ensemble':
        members_to_train = list(model.members)
        seeds = [model.base_seed + i for i in range(model.n_members)]
    else:
        members_to_train = [model]
        seeds = [42]
    
    for m_idx, member in enumerate(members_to_train):
        torch.manual_seed(seeds[m_idx])
        np.random.seed(seeds[m_idx])
        optimizer = Adam(member.parameters(), lr=lr, weight_decay=weight_decay)
        criterion = nn.MSELoss()
        
        best_loss = float('inf')
        patience_counter = 0
        best_state = None

        if batch_size > 0:
            dataset = TensorDataset(X_t, y_t)
            loader = DataLoader(dataset, batch_size=batch_size, shuffle=True)
            
        desc_str = "Epochs:"
        if current_iteration is not None and total_iterations is not None:
            desc_str = f"Iter: {current_iteration}/{total_iterations}"
        if arch == 'ensemble':
            desc_str += f", Memb: {m_idx+1}/{len(members_to_train)}"
        
        pbar = tqdm(total=epochs, desc=desc_str, ascii=True)
        
        for epoch in range(1, epochs + 1):
            pbar.update(1)
            member.train()
            epoch_loss = 0.0
            
            if batch_size > 0:
                for xb, yb in loader:
                    optimizer.zero_grad()
                    if loss_mode == 'physics_informed' and vehicle_model is not None:
                        loss, terms = compute_physics_informed_loss(member, xb, yb, vehicle_model, sample_dt, physics_loss_cfg)
                    else:
                        out = member(xb)
                        loss = criterion(out, yb)
                    loss.backward()
                    optimizer.step()
                    epoch_loss += loss.item() * xb.size(0)
                epoch_loss /= len(X_t)
            else:
                optimizer.zero_grad()
                if loss_mode == 'physics_informed' and vehicle_model is not None:
                    loss, terms = compute_physics_informed_loss(member, X_t, y_t, vehicle_model, sample_dt, physics_loss_cfg)
                else:
                    out = member(X_t)
                    loss = criterion(out, y_t)
                loss.backward()
                optimizer.step()
                epoch_loss = loss.item()
            
            if early_stopping:
                if best_loss - epoch_loss > min_delta:
                    best_loss = epoch_loss
                    patience_counter = 0
                    best_state = {k: v.detach().clone() for k, v in member.state_dict().items()}
                else:
                    patience_counter += 1

                if patience_counter >= patience:
                    break

        pbar.close()

        if early_stopping and best_state is not None:
            member.load_state_dict(best_state)

        if log_per_iteration:
            print(f"Arch: {arch} | Member {m_idx} | iter loss: {epoch_loss:.6f}")
    
    return model

def simulated_data_gen(nn_model, vehicle_model, avg_vel, nn_params, delta_max=None):
    C_Pf_model = vehicle_model['C_Pf_model']
    C_Pr_model = vehicle_model['C_Pr_model']

    l_f = vehicle_model['l_f']
    l_r = vehicle_model['l_r']
    l_wb = vehicle_model['l_wb']
    m = vehicle_model['m']
    I_z = vehicle_model['I_z']
    F_zf = m * 9.81 * l_r / l_wb
    F_zr = m * 9.81 * l_f / l_wb
    dt = float(nn_params.get('sample_dt', 0.02))
    ode_solver = nn_params.get('ode_solver', 'euler').lower()
    arch = nn_params.get('nn_architecture', 'baseline')
    # Same device the model was trained on (train_residual_nn already resolved
    # nn_params['device'] and moved the model there) - reading it back off the
    # model itself keeps this in sync without re-running device resolution
    # (and its cuda-unavailable fallback logic) a second time.
    device = next(nn_model.parameters()).device

    timesteps = 500

    v_y = np.zeros(timesteps)
    omega = np.zeros(timesteps)
    v_x = np.ones(timesteps)*avg_vel
    # Caller normally passes rollout_delta_max()'s data-bounded value; the
    # config ceiling is the fallback for direct callers.
    if delta_max is None:
        delta_max = float((vehicle_model.get('pacejka_rollout', {}) or {}).get('sweep_delta_max', 0.4))
    delta = np.linspace(0.0, delta_max, timesteps)

    nn_model.eval()

    # arch=='s4' carries hidden state across this whole 500-step rollout.
    # simulated_data_gen() is called fresh from nn_train()'s outer loop every
    # iteration with a newly-trained nn_model, so this local variable resets
    # automatically each outer iteration - intentional, do NOT hoist it out
    # to persist across nn_train() iterations, since each iteration's Pacejka
    # model has changed and stale state would carry over stale dynamics.
    s4_state = nn_model.init_state(1, device=device) if arch == 's4' else None
    
    def dynamics(vy_val, omega_val, vx_val, delta_val):
        alpha_f_val = -np.arctan((vy_val + omega_val * l_f) / vx_val) + delta_val
        alpha_r_val = -np.arctan((vy_val - omega_val * l_r) / vx_val)

        F_f_val = pacejka_formula(C_Pf_model, alpha_f_val, F_zf)
        F_r_val = pacejka_formula(C_Pr_model, alpha_r_val, F_zr)
        
        v_y_dot = (1/m) * (F_r_val + F_f_val * np.cos(delta_val) - m * vx_val * omega_val)
        omega_dot = (1/I_z) * (F_f_val * l_f * np.cos(delta_val) - F_r_val * l_r)
        
        return v_y_dot, omega_dot
    
    for t in range(timesteps-1):
        vx_t = v_x[t]
        vy_t = v_y[t]
        omega_t = omega[t]
        delta_t = delta[t]
        
        if ode_solver == 'euler':
            v_y_dot, omega_dot = dynamics(vy_t, omega_t, vx_t, delta_t)
            vy_next = vy_t + v_y_dot * dt
            omega_next = omega_t + omega_dot * dt
        elif ode_solver == 'rk2':
            k1_vy, k1_omega = dynamics(vy_t, omega_t, vx_t, delta_t)
            vy_mid = vy_t + k1_vy * dt
            omega_mid = omega_t + k1_omega * dt
            
            delta_next = delta[t+1]
            k2_vy, k2_omega = dynamics(vy_mid, omega_mid, vx_t, delta_next)
            
            vy_next = vy_t + (k1_vy + k2_vy) * dt / 2.0
            omega_next = omega_t + (k1_omega + k2_omega) * dt / 2.0
        elif ode_solver == 'rk4':
            k1_vy, k1_omega = dynamics(vy_t, omega_t, vx_t, delta_t)
            
            delta_half = (delta[t] + delta[t+1])/2.0
            vy_k2 = vy_t + k1_vy * dt / 2.0
            omega_k2 = omega_t + k1_omega * dt / 2.0
            k2_vy, k2_omega = dynamics(vy_k2, omega_k2, vx_t, delta_half)
            
            vy_k3 = vy_t + k2_vy * dt / 2.0
            omega_k3 = omega_t + k2_omega * dt / 2.0
            k3_vy, k3_omega = dynamics(vy_k3, omega_k3, vx_t, delta_half)
            
            delta_next = delta[t+1]
            vy_k4 = vy_t + k3_vy * dt
            omega_k4 = omega_t + k3_omega * dt
            k4_vy, k4_omega = dynamics(vy_k4, omega_k4, vx_t, delta_next)
            
            vy_next = vy_t + (k1_vy + 2*k2_vy + 2*k3_vy + k4_vy) * dt / 6.0
            omega_next = omega_t + (k1_omega + 2*k2_omega + 2*k3_omega + k4_omega) * dt / 6.0
        else:
            raise ValueError(f"Unknown ODE solver: {ode_solver}")
            
        input_raw = torch.tensor([vx_t, vy_t, omega_t, delta_t], dtype=torch.float32)
        input_proc = preprocess_inputs(input_raw, nn_params, vehicle_model)
        input_proc_t = torch.tensor(input_proc, dtype=torch.float32, device=device)
        if input_proc_t.dim() == 1:
            input_proc_t = input_proc_t.unsqueeze(0)
            
        with torch.no_grad():
            if arch == 's4':
                predicted_means, s4_state = nn_model.step(input_proc_t, s4_state)
            else:
                predicted_means = nn_model(input_proc_t)
            if predicted_means.dim() == 2:
                predicted_means = predicted_means.squeeze(0)

        v_y[t+1] = vy_next + predicted_means[0].item()
        omega[t+1] = omega_next + predicted_means[1].item()
    
    return v_x, v_y, omega, delta

def get_model_param(racecar_version):
    package_path = get_package_path('on_track_sys_id')
    yaml_file = os.path.join(package_path, 'params', 'pacejka_params.yaml')
    with open(yaml_file, 'r') as file:
        pacejka_params = yaml.safe_load(file)

    solver_cfg = pacejka_params.get('pacejka_solver', {})
        
    yaml_file = os.path.join(package_path, 'models', racecar_version, racecar_version + '_pacejka.txt')
    with open(yaml_file, 'r') as file:
        vehicle_params = yaml.safe_load(file)

    model = {
        "C_Pf_model": pacejka_params['pacejka_model']['C_Pf_model'],
        "C_Pr_model": pacejka_params['pacejka_model']['C_Pr_model'],
        "C_Pf_ref": pacejka_params['pacejka_ref']['C_Pf_ref'],
        "C_Pr_ref": pacejka_params['pacejka_ref']['C_Pr_ref'],
        "pacejka_method": solver_cfg.get('method', 'trf'),
        "pacejka_loss": solver_cfg.get('loss', 'soft_l1'),
        "pacejka_f_scale": solver_cfg.get('f_scale', None),
        "pacejka_x_scale": solver_cfg.get('x_scale', 'jac'),
        "pacejka_max_nfev": solver_cfg.get('max_nfev', None),
        "pacejka_num_starts": solver_cfg.get('num_starts', 1),
        "pacejka_prior_weight": solver_cfg.get('prior_weight', 0.0),
        "pacejka_start_jitter": solver_cfg.get('start_jitter', 0.05),
        "pacejka_seed": solver_cfg.get('seed', None),
        "pacejka_de_popsize": solver_cfg.get('de_popsize', None),
        "pacejka_de_maxiter": solver_cfg.get('de_maxiter', None),
        "friction_warm_start": pacejka_params.get('friction_warm_start', {}),
        "pacejka_rollout": pacejka_params.get('pacejka_rollout', {}),
        "m": vehicle_params['m'],
        "I_z": vehicle_params['I_z'],
        "l_f": vehicle_params['l_f'],
        "l_r": vehicle_params['l_r'],
        "l_wb": vehicle_params['l_wb'],
        "racecar_version": racecar_version
    }
    return model

def get_nn_params():
    package_path = get_package_path('on_track_sys_id')
    yaml_file = os.path.join(package_path, 'params', 'nn_params.yaml')
    with open(yaml_file, 'r') as file:
        nn_params = yaml.safe_load(file)
    return nn_params

def generate_training_set(training_data, model, nn_params=None, segment_len=None):
    nn_params = nn_params or {}
    dt = float(nn_params.get('sample_dt', 0.02))
    v_y_next_pred, omega_next_pred = generate_predictions(training_data, model, dt=dt)
    X_train, y_train = generate_inputs_errors(v_y_next_pred, omega_next_pred, training_data)

    if nn_params.get('nn_architecture', 'baseline') == 's4':
        s4_cfg = nn_params.get('s4', {}) or {}
        window_size = int(s4_cfg.get('sequence_length', 20))
        if segment_len is None:
            raise ValueError("generate_training_set: segment_len is required for nn_architecture: s4")
        X_train, y_train = generate_sequence_windows(X_train, y_train, window_size, segment_len)

    return X_train, y_train

def rollout_speed(training_data, model):
    """Speed used for simulated_data_gen()'s synthetic steering sweep.

    solve_pacejka() never sees the measured trace - it sees the rollout of
    (nominal Pacejka + residual NN) that simulated_data_gen() produces while
    sweeping delta from 0 to sweep_delta_max at this fixed speed. The peak
    lateral force that rollout can ever produce is therefore capped by the
    kinematic ceiling of that speed,

        mu_kin = v^2 * delta_max / (g * l_wb),

    because analyse_tires() reconstructs the "measured" force from
    m * v_x * omega, and in steady state omega = v_x * delta / l_wb. Below
    mu_kin the tire curve is still on its linear ramp, so only the product
    B*C*D is identifiable and the solver slides D onto its box edge.

    This used to be hardcoded as np.clip(mean(v_x), 2.0, 4.0) - an F1TENTH
    leftover. Measured on the CARLA asurt_fsai (l_wb 1.5334 m, sweep to
    0.4 rad) that ceiling is mu_kin = 0.43 at 4 m/s, and a rollout-and-fit at
    that speed returns Df = 0.400 / Dr = 0.400 (both exactly the bound) from
    a D = 1.5 prior as well as from the yaml prior. At the vehicle's real
    13 m/s the same fit returns Df = 1.555 / Dr = 1.403 off a D = 1.5 prior,
    no coefficient on a bound. See test/test_pacejka_identifiability.py.
    """
    cfg = (model.get('pacejka_rollout', {}) or {})
    v_meas = float(np.mean(training_data[:, 0]))
    v_min = float(cfg.get('speed_min', 2.0))
    v_max = cfg.get('speed_max', None)
    avg_vel = max(v_meas, v_min)
    if v_max is not None:
        avg_vel = min(avg_vel, float(v_max))
    if avg_vel != v_meas:
        log_info(f"Pacejka rollout speed clamped {v_meas:.2f} -> {avg_vel:.2f} m/s "
                 f"(pacejka_rollout.speed_min/speed_max)")

    delta_max = float(cfg.get('sweep_delta_max', 0.4))
    mu_kin = avg_vel ** 2 * delta_max / (9.81 * model['l_wb'])
    d_prior = max(model['C_Pf_model'][2], model['C_Pr_model'][2])
    if mu_kin < d_prior:
        log_warn(
            f"Pacejka rollout at {avg_vel:.2f} m/s can only reach mu = {mu_kin:.2f} "
            f"(kinematic ceiling of v^2*{delta_max}/(g*l_wb)) but the tire prior peaks at "
            f"D = {d_prior:.2f}. The synthetic sweep never leaves the linear ramp, so D is "
            "unidentifiable and the fit will slide onto the D bound. Raise "
            "pacejka_rollout.speed_min or collect data at a higher speed.")
    return avg_vel


def rollout_delta_max(training_data, model):
    """End of simulated_data_gen()'s steering sweep, bounded by the data.

    The rollout is (nominal Pacejka + residual NN). The NN is only trained on
    the steering range the car was actually driven through - on a pure-pursuit
    lap of traj_race_cl.csv that is |delta| <= 0.06 rad - so sweeping it out
    to the configured 0.4 rad asks it to invent the whole nonlinear part of
    the tire curve. Measured consequence (2026-08-22): the fit reads the
    extrapolated force as extra grip, and because each iteration's answer
    becomes the next one's nominal model, six iterations walked D from 1.01
    to 1.87 - a claimed 1.7 g on a vehicle that measures 1.0 g.

    So the sweep stops at sweep_delta_extrapolation times the largest steering
    angle actually seen (99th percentile, to ignore single-sample spikes),
    capped by sweep_delta_max. Beyond that range D is simply not identifiable
    from this manoeuvre and the prior term in solve_pacejka holds it, which is
    the honest answer: only limit excitation can measure peak grip.
    """
    cfg = (model.get('pacejka_rollout', {}) or {})
    hard_max = float(cfg.get('sweep_delta_max', 0.4))
    factor = float(cfg.get('sweep_delta_extrapolation', 1.5))
    floor = float(cfg.get('sweep_delta_min', 0.05))
    delta_obs = float(np.percentile(np.abs(training_data[:, 3]), 99))
    delta_max = min(hard_max, max(floor, factor * delta_obs))
    log_info(f"Pacejka rollout sweep: delta 0 -> {delta_max:.3f} rad "
             f"(p99 |delta| observed {delta_obs:.3f} rad x {factor}, cap {hard_max})")
    return delta_max


def nn_train(training_data, racecar_version, save_LUT_name, plot_model, warm_start_mu=None):
    model = get_model_param(racecar_version)
    nn_params = get_nn_params()

    if warm_start_mu is not None:
        d_lower, d_upper = PACEJKA_BOUNDS[0][2], PACEJKA_BOUNDS[1][2]
        d_val = float(np.clip(warm_start_mu, d_lower, d_upper))
        # mu_hat is peak UTILISED friction, which is a lower bound on the
        # available friction (see friction_warmstart._mu_from_accel). Applying
        # it as a replacement let a gentle lap - 0.04 g median, 0.63 g peak
        # measured on this vehicle - overwrite a physically sane prior with a
        # near-zero D. It is a floor, never a ceiling.
        for key in ('C_Pf_model', 'C_Pr_model'):
            if d_val > model[key][2]:
                log_info(
                    f"Cold-start friction warm-start: raising {key} D initial guess "
                    f"{model[key][2]:.4f} -> {d_val:.4f} (mu_hat={warm_start_mu:.4f})")
                model[key][2] = d_val
            else:
                log_info(
                    f"Cold-start friction warm-start: keeping {key} D initial guess "
                    f"{model[key][2]:.4f} (mu_hat={warm_start_mu:.4f} is only a lower bound "
                    "and does not exceed it)")

    num_of_iterations = nn_params.get('num_of_iterations', 6)
    arch = nn_params.get('nn_architecture', 'baseline')

    # filter_data() (inside process_data) trims a fixed 5 rows; negate_data()
    # then concatenates the filtered trace with its mirrored copy. segment_len
    # is each of those two physically-continuous halves' length, needed by
    # generate_training_set()'s s4 windowing to avoid windowing across the
    # boundary between them.
    segment_len = training_data.shape[0] - 5
    if arch == 's4':
        window_size = int((nn_params.get('s4', {}) or {}).get('sequence_length', 20))
        if segment_len - 1 < window_size:
            raise ValueError(
                f"nn_architecture: s4 needs segment_len-1={segment_len - 1} >= "
                f"s4.sequence_length={window_size} - increase data_collection_duration "
                "or lower s4.sequence_length in nn_params.yaml")

    training_data = process_data(training_data, model)

    avg_vel = rollout_speed(training_data, model)
    sweep_delta_max = rollout_delta_max(training_data, model)

    # The co-identification loop feeds each iteration's answer in as the next
    # iteration's nominal model, which is what lets the residual NN and the
    # Pacejka fit converge together. The REGULARISER's target must not move
    # with it, or the prior term degenerates into "stay near wherever you
    # drifted to" and the drift compounds: measured on 2026-08-22 a run that
    # started from D = 1.009/1.002 ended at D = 1.540/1.872 after 6
    # iterations, i.e. a claimed 1.7 g on a vehicle measured at 1.0 g.
    model['C_Pf_prior'] = list(model['C_Pf_model'])
    model['C_Pr_prior'] = list(model['C_Pr_model'])

    for i in range(1, num_of_iterations+1):
        if i == num_of_iterations:
            plot_model = True

        X_train, y_train = generate_training_set(training_data, model, nn_params, segment_len=segment_len)
        
        log_info(f"Training iteration {i}/{num_of_iterations} | Architecture: {arch} | Loss Mode: {nn_params.get('loss_mode', 'mse')}")
        
        t0 = time.perf_counter()
        nn_model = train_residual_nn(X_train, y_train, nn_params, vehicle_model=model, current_iteration=i, total_iterations=num_of_iterations)
        t1 = time.perf_counter()
        log_info(f"Training completed in {t1 - t0:.3f} seconds.")
        
        v_x, v_y, omega, delta = simulated_data_gen(
            nn_model, model, avg_vel, nn_params, delta_max=sweep_delta_max)
        C_Pf_identified, C_Pr_identified = solve_pacejka(model, v_x, v_y, omega, delta)

        # Print the identified Pacejka coefficients clearly using ROS logger
        log_info(f"=========== ITERATION {i} RESULTS ===========")
        log_info(f"C_Pf_identified: {C_Pf_identified}")
        log_info(f"C_Pr_identified: {C_Pr_identified}")
        log_info(f"==========================================")
        warn_railed_fit('front', C_Pf_identified)
        warn_railed_fit('rear', C_Pr_identified)
        
        if plot_model:
            saved_path = plot_results(
                model, v_x, v_y, omega, delta, C_Pf_identified, C_Pr_identified, i, racecar_version)
            log_info(f"Saved Pacejka fit plot to: {saved_path}")
            
        model['C_Pf_model'] = C_Pf_identified
        model['C_Pr_model'] = C_Pr_identified
                
    model_name = racecar_version +"_pacejka"
    car_model = get_dotdict(model_name)
    car_model.C_Pf = C_Pf_identified
    car_model.C_Pr = C_Pr_identified
    save(car_model)
    
    log_info("LUT is being generated...")
    LookupGenerator(racecar_version, save_LUT_name).run_generator()

    # Returned so the caller submits what was IDENTIFIED. get_model_param()
    # reads C_P*_model back out of params/pacejka_params.yaml, which nothing
    # ever writes, so a caller that re-reads it after training gets the static
    # prior - on_track_sys_id did exactly that and pushed the prior to
    # adaptive_controller_manager on every cycle (measured 2026-08-22: the
    # service received [7.076, 1.346, 1.009, -2.0] while this function had
    # just saved [4.341, 2.178, 1.540, -1.657] to SIM_pacejka.txt).
    return C_Pf_identified, C_Pr_identified
