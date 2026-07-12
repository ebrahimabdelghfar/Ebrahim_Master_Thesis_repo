"""
Residual NN for On-Track Pacejka SysID.
Paper: arXiv:2411.17508v1 — Learning-Based On-Track SysID for Scaled Autonomous Racing.

Supported architectures (set via nn_params.yaml -> nn_architecture):
  baseline        : 4->8->2  MLP, 58 params  [paper original]
  wide            : 4->16->2 MLP, 114 params [drop-in, weight_decay regularized]
  physics_inputs  : 6->8->2  MLP, 74 params  [slip-angle augmented inputs, RECOMMENDED]
  ensemble        : 3x(4->8->2), 174 params  [averaged ensemble, best noise robustness]

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
from helpers.generate_inputs_errors import generate_inputs_errors
from helpers.data_processing import compute_slip_angles
from helpers.pacejka_formula import pacejka_formula
from helpers.plot_results import plot_results
from helpers.solve_pacejka import solve_pacejka
from helpers.save_model import save
from helpers.load_model import get_dotdict
from helpers.simulate_model import LookupGenerator
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

def compute_physics_informed_loss(nn_model, X_train, y_train, model, dt, lambda_cfg):
    criterion = nn.MSELoss()
    outputs = nn_model(X_train)
    data_loss = criterion(outputs, y_train)

    v_x = X_train[:, 0]
    v_y = X_train[:, 1]
    omega = X_train[:, 2]
    delta = X_train[:, 3]

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
    X_mirror[:, 1] = -X_mirror[:, 1]
    X_mirror[:, 2] = -X_mirror[:, 2]
    X_mirror[:, 3] = -X_mirror[:, 3]
    if X_mirror.shape[1] >= 6:
        X_mirror[:, 4] = -X_mirror[:, 4]
        X_mirror[:, 5] = -X_mirror[:, 5]
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
    else:
        raise ValueError(f"Unknown architecture: {arch}")

def preprocess_inputs(X_raw, params: dict, vehicle_model=None) -> np.ndarray:
    arch = params.get('nn_architecture', 'baseline')
    
    if isinstance(X_raw, torch.Tensor):
        X_np = X_raw.detach().cpu().numpy()
    else:
        X_np = np.asarray(X_raw)
        
    if arch == 'physics_inputs':
        if vehicle_model is not None:
            lf = vehicle_model['l_f']
            lr = vehicle_model['l_r']
        else:
            lf = params.get('physics_inputs_lf', 0.15795)
            lr = params.get('physics_inputs_lr', 0.17205)
        
        if X_np.ndim == 1:
            vx, vy, omega, delta = X_np[0], X_np[1], X_np[2], X_np[3]
        else:
            vx, vy, omega, delta = X_np[:, 0], X_np[:, 1], X_np[:, 2], X_np[:, 3]
            
        alpha_f, alpha_r = compute_slip_angles(vx, vy, omega, delta, lf, lr)
        
        if X_np.ndim == 1:
            return np.concatenate([X_np, [alpha_f, alpha_r]])
        else:
            return np.column_stack([X_np, alpha_f, alpha_r])
    else:
        return X_np

def train_residual_nn(X_train_raw, y_train, params: dict, vehicle_model=None, current_iteration=None, total_iterations=None) -> nn.Module:
    model = build_model(params)
    
    X_train_np = preprocess_inputs(X_train_raw, params, vehicle_model)
    X_t = torch.tensor(X_train_np, dtype=torch.float32)
    y_t = torch.tensor(y_train, dtype=torch.float32) if not isinstance(y_train, torch.Tensor) else y_train.clone().detach().to(torch.float32)
    
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
                        loss, terms = compute_physics_informed_loss(member, xb, yb, vehicle_model, 0.02, physics_loss_cfg)
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
                    loss, terms = compute_physics_informed_loss(member, X_t, y_t, vehicle_model, 0.02, physics_loss_cfg)
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
                else:
                    patience_counter += 1
                
                if patience_counter >= patience:
                    break
                    
        pbar.close()
        
        if log_per_iteration:
            print(f"Arch: {arch} | Member {m_idx} | iter loss: {epoch_loss:.6f}")
    
    return model

def simulated_data_gen(nn_model, vehicle_model, avg_vel, nn_params):
    C_Pf_model = vehicle_model['C_Pf_model']
    C_Pr_model = vehicle_model['C_Pr_model']

    l_f = vehicle_model['l_f']
    l_r = vehicle_model['l_r']
    l_wb = vehicle_model['l_wb']
    m = vehicle_model['m']
    I_z = vehicle_model['I_z']
    F_zf = m * 9.81 * l_r / l_wb
    F_zr = m * 9.81 * l_f / l_wb
    dt = 0.02
    ode_solver = nn_params.get('ode_solver', 'euler').lower()

    timesteps = 500
    
    v_y = np.zeros(timesteps)
    omega = np.zeros(timesteps)
    v_x = np.ones(timesteps)*avg_vel
    delta = np.linspace(0.0, 0.4, timesteps)
    
    nn_model.eval()
    
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
        input_proc_t = torch.tensor(input_proc, dtype=torch.float32)
        if input_proc_t.dim() == 1:
            input_proc_t = input_proc_t.unsqueeze(0)
            
        with torch.no_grad():
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
        "pacejka_start_jitter": solver_cfg.get('start_jitter', 0.05),
        "pacejka_seed": solver_cfg.get('seed', None),
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

def generate_training_set(training_data, model):
    v_y_next_pred, omega_next_pred = generate_predictions(training_data, model)
    X_train, y_train = generate_inputs_errors(v_y_next_pred, omega_next_pred, training_data)
    return X_train, y_train

def nn_train(training_data, racecar_version, save_LUT_name, plot_model):
    model = get_model_param(racecar_version)
    nn_params = get_nn_params()
    
    num_of_iterations = nn_params.get('num_of_iterations', 6)
    arch = nn_params.get('nn_architecture', 'baseline')
    
    training_data = process_data(training_data, model)   
     
    avg_vel = np.mean(training_data[:,0])
    avg_vel = np.clip(avg_vel, 2.0, 4)
    
    for i in range(1, num_of_iterations+1):
        if i == num_of_iterations:
            plot_model = True
            
        X_train, y_train = generate_training_set(training_data, model)
        
        log_info(f"Training iteration {i}/{num_of_iterations} | Architecture: {arch} | Loss Mode: {nn_params.get('loss_mode', 'mse')}")
        
        t0 = time.perf_counter()
        nn_model = train_residual_nn(X_train, y_train, nn_params, vehicle_model=model, current_iteration=i, total_iterations=num_of_iterations)
        t1 = time.perf_counter()
        log_info(f"Training completed in {t1 - t0:.3f} seconds.")
        
        v_x, v_y, omega, delta = simulated_data_gen(nn_model, model, avg_vel, nn_params)
        C_Pf_identified, C_Pr_identified = solve_pacejka(model, v_x, v_y, omega, delta)

        # Print the identified Pacejka coefficients clearly using ROS logger
        log_info(f"=========== ITERATION {i} RESULTS ===========")
        log_info(f"C_Pf_identified: {C_Pf_identified}")
        log_info(f"C_Pr_identified: {C_Pr_identified}")
        log_info(f"==========================================")
        
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
