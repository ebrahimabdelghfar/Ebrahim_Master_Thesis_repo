import os
import numpy as np
import matplotlib.pyplot as plt
import torch
import torch.nn as nn
import yaml
from scipy.signal import butter, filtfilt
from torch.optim import Adam
from helpers.generate_predictions import generate_predictions
from helpers.generate_inputs_errors import generate_inputs_errors
from helpers.SimpleNN import SimpleNN
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

# For logging (works with or without ROS2)
import logging
logger = logging.getLogger(__name__)

def log_info(msg):
    """Log info message - works with or without ROS2"""
    logger.info(msg)
    print(f"[INFO] {msg}")

def log_warn(msg):
    """Log warning message - works with or without ROS2"""
    logger.warning(msg)
    print(f"[WARN] {msg}")

def get_package_path(package_name='on_track_sys_id'):
    """Get package path using ament_index or fallback to relative path"""
    if USE_AMENT:
        try:
            return get_package_share_directory(package_name)
        except Exception:
            pass
    
    # Fallback: try to find the package directory relative to this file
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # Go up from helpers to src, then to package root
    package_root = os.path.dirname(os.path.dirname(current_dir))
    if os.path.exists(os.path.join(package_root, 'params')):
        return package_root
    
    # Another fallback - check if we're in the src folder
    package_root = os.path.dirname(current_dir)
    if os.path.exists(os.path.join(package_root, 'params')):
        return package_root
    
    return current_dir

def filter_data(training_data, model):
    """
    Filter training data.

    Applies a low-pass Butterworth filter to each column of the training data array.
    """
    b, a = butter(N=3, Wn=0.1, btype='low')
    training_data[:,0] = filtfilt(b, a, training_data[:,0]) # v_x, longitudinal velocity
    training_data[:,1] = filtfilt(b, a, training_data[:,1]) # v_y, lateral velocity
    training_data[:,2] = filtfilt(b, a, training_data[:,2]) # omega, yaw rate
    training_data[:,3] = filtfilt(b, a, training_data[:,3]) # delta, steering angle
    training_data[:,3] = np.roll(training_data[:,3], 5)
    training_data = training_data[5:,:]
    
    # If the model is not a simulation, adjusts lateral velocity based on the car's rear axle length.
    if model["racecar_version"] != "SIM":
        training_data[:,1] = training_data[:,1] + model["l_r"] * training_data[:,2]
    
    return training_data

def negate_data(training_data):
    """
    Negate training data along the y-axis.

    Negates the lateral velocity (v_y), yaw rate (omega) and steering angle (delta) components of the training data
    to generate additional training samples with the opposite direction.
    """
    negate_data = np.zeros((2*training_data.shape[0], training_data.shape[1]))
    negate_data[:,0] = np.append(training_data[:,0], training_data[:,0])
    negate_data[:,1] = np.append(training_data[:,1], -training_data[:,1])
    negate_data[:,2] = np.append(training_data[:,2], -training_data[:,2])
    negate_data[:,3] = np.append(training_data[:,3], -training_data[:,3])
    
    return negate_data

def process_data(training_data, model):
    """
    Process training data.
    Filters and negates the training data to prepare it for training the neural network.
    """
    filtered_data = filter_data(training_data, model)
    negated_data = negate_data(filtered_data)
    
    return negated_data

def torch_pacejka_force(params, alpha, F_z):
    """Compute Pacejka lateral force with torch tensors."""
    B = float(params[0])
    C = float(params[1])
    D = float(params[2])
    E = float(params[3])
    return F_z * D * torch.sin(C * torch.atan(B * alpha - E * (B * alpha - torch.atan(B * alpha))))

def compute_physics_informed_loss(nn_model, X_train, y_train, model, dt, lambda_cfg):
    """
    Composite physics-informed loss used for ablation against plain MSE.
    """
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

def simulated_data_gen(nn_model, model, avg_vel):
    C_Pf_model = model['C_Pf_model']
    C_Pr_model = model['C_Pr_model']

    l_f = model['l_f']
    l_r = model['l_r']
    l_wb = model['l_wb']
    m = model['m']
    I_z = model['I_z']
    F_zf = m * 9.81 * l_r / l_wb
    F_zr = m * 9.81 * l_f / l_wb
    dt = 0.02 # 0.02 for 50 Hz

    timesteps = 500 # Number of timesteps to simulate
    
    v_y = np.zeros(timesteps)  # Initial lateral velocity
    omega = np.zeros(timesteps)  # Initial yaw rate
    alpha_f = np.zeros(timesteps)  # Initial lateral velocity
    alpha_r = np.zeros(timesteps)  # Initial yaw rate
    
    v_x = np.ones(timesteps)*avg_vel  # Constant longitudinal velocity
    delta = np.linspace(0.0, 0.4, timesteps)
    
    # Simulation loop
    for t in range(timesteps-1):
        alpha_f[t] = -np.arctan((v_y[t] + omega[t] * l_f) / v_x[t]) + delta[t]
        alpha_r[t] = -np.arctan((v_y[t] - omega[t] * l_r) / v_x[t])

        # Calculate Pacejka lateral forces
        F_f = pacejka_formula(C_Pf_model, alpha_f[t], F_zf)
        F_r = pacejka_formula(C_Pr_model, alpha_r[t], F_zr)
        input = torch.tensor([v_x[t], v_y[t], omega[t], delta[t]], dtype=torch.float32)

        # Making predictions
        with torch.no_grad():
            predicted_means = nn_model(input)
        # Update vehicle states using the dynamics equations
        v_y_dot = (1/m) * (F_r + F_f * np.cos(delta[t]) - m * v_x[t]* omega[t])
        omega_dot = (1/I_z) * (F_f * l_f * np.cos(delta[t]) - F_r * l_r)

        # Euler integration for the next state
        v_y[t+1] = v_y[t] + v_y_dot * dt + predicted_means[0]
        omega[t+1] = omega[t] + omega_dot * dt + predicted_means[1]
    
    return v_x, v_y, omega, delta

def get_model_param(racecar_version):
    """
    Retrieve model parameters for a given racecar version.
    Loads Pacejka tire and vehicle parameters from YAML files and constructs a model dictionary.
    
    Returns:
        dict: Model parameters including tire and vehicle properties.
    """
    package_path = get_package_path('on_track_sys_id')
    yaml_file = os.path.join(package_path, 'params', 'pacejka_params.yaml')
    with open(yaml_file, 'r') as file:
        pacejka_params = yaml.safe_load(file)

    solver_cfg = pacejka_params.get('pacejka_solver', {})
        
    # Load vehicle parameters
    yaml_file = os.path.join(package_path, 'models', racecar_version, racecar_version + '_pacejka.txt')
    with open(yaml_file, 'r') as file:
        vehicle_params = yaml.safe_load(file)

    # Construct model dictionary
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
    """
    Retrieve neural network parameters.

    Loads neural network parameters from a YAML file.

    Returns:
        dict: Neural network parameters.
    """
    package_path = get_package_path('on_track_sys_id')
    yaml_file = os.path.join(package_path, 'params', 'nn_params.yaml')
    with open(yaml_file, 'r') as file:
        nn_params = yaml.safe_load(file)
        
    return nn_params

def generate_training_set(training_data, model):
    """
    Generate training set for neural network training.

    Predicts the next step's lateral velocity and yaw rate using the vehicle model.
    Constructs input tensors and error tensors for training the neural network.

    Args:
        training_data (numpy.ndarray): Input training data with shape (n_samples, n_features).
        model (dict): Dictionary containing vehicle model parameters.

    Returns:
        tuple: Tuple containing input tensor and target error tensor for training.
    """
    
    # Generate predictions for the next step's lateral velocity and yaw rate
    v_y_next_pred, omega_next_pred = generate_predictions(training_data, model)
    
    # Generate input tensors and error tensors for training
    X_train, y_train = generate_inputs_errors(v_y_next_pred, omega_next_pred, training_data)
    
    return X_train, y_train

def nn_train(training_data, racecar_version, save_LUT_name, plot_model):
    """
    Train the neural network.
    
    Trains the neural network using the provided training data and model parameters.
    After training, it simulates the car behavior with the trained model and identifies
    Pacejka tire model coefficients. Then it iteratively refines the model and repeats
    the training process. Finally, it saves the trained model and generates a
    Look-Up Table (LUT) for the controller. 

    """
    # Get model and neural network parameters
    model = get_model_param(racecar_version)
    nn_params = get_nn_params()
    num_of_epochs = nn_params['num_of_epochs']
    lr = nn_params['lr']
    weight_decay = nn_params['weight_decay']
    num_of_iterations = nn_params['num_of_iterations']
    loss_mode = nn_params.get('loss_mode', 'mse')
    physics_loss_cfg = nn_params.get('physics_loss', {})
    dt = 0.02

    training_data = process_data(training_data, model)   
     
    avg_vel = np.mean(training_data[:,0]) # Defining average velocity for the simulation, NN will have more accurate predictions
    avg_vel = np.clip(avg_vel, 2.0, 4) # Clip average velocity to be within a reasonable range (2.75 m/s to 4 m/s)
    
    # Iterative training loop
    for i in range(1, num_of_iterations+1):
        if i == num_of_iterations: # Determine if it's the last iteration to enable plotting (if plot_model is False)
            plot_model = True
            
        # Process training data and generate inputs and targets
        X_train, y_train = generate_training_set(training_data, model)
        
        # Initialize the network
        nn_model = SimpleNN(weight_decay = weight_decay)

        # Optimizer and ablation-ready loss selection
        optimizer = Adam(nn_model.parameters(), lr=lr)

        if loss_mode not in ['mse', 'physics_informed']:
            log_warn(f"Unknown loss_mode '{loss_mode}'. Falling back to 'mse'.")
            loss_mode = 'mse'

        log_info(f"Training iteration {i}/{num_of_iterations} with loss_mode={loss_mode}")

        nn_model.train()
        pbar = tqdm(total=num_of_epochs, desc=f"Iteration: {i}/{num_of_iterations}, Epoch:", ascii=True)

        # Training loop
        for epoch in range(1, num_of_epochs+1):
            pbar.update(1)
            # Forward pass on training data
            if loss_mode == 'physics_informed':
                train_loss, loss_terms = compute_physics_informed_loss(
                    nn_model=nn_model,
                    X_train=X_train,
                    y_train=y_train,
                    model=model,
                    dt=dt,
                    lambda_cfg=physics_loss_cfg
                )
            else:
                outputs = nn_model(X_train)
                train_loss = nn.MSELoss()(outputs, y_train)

            optimizer.zero_grad()
            train_loss.backward()
            optimizer.step()
            
            # If it's the last epoch, simulate car behavior and identify model coefficients
            if (epoch == num_of_epochs):
                pbar.close()
                nn_model.eval()
                if loss_mode == 'physics_informed':
                    log_info(
                        f"Final epoch losses | total={float(train_loss.detach().cpu()):.6f}, "
                        f"data={loss_terms['data']:.6f}, steady={loss_terms['steady']:.6f}, "
                        f"symmetry={loss_terms['symmetry']:.6f}, smoothness={loss_terms['smoothness']:.6f}"
                    )
                v_x, v_y, omega, delta = simulated_data_gen(nn_model, model, avg_vel)   
                C_Pf_identified, C_Pr_identified = solve_pacejka(model, v_x, v_y, omega, delta)

                print(f"C_Pf_identified at Iteration {i}:", C_Pf_identified)
                print(f"C_Pr_identified at Iteration {i}:", C_Pr_identified)
                
                if plot_model:
                    log_warn("Close the plot window (press Q) to continue... ")
                    plot_results(model, v_x, v_y, omega, delta, C_Pf_identified, C_Pr_identified, i)   
                    
                # Update model with identified coefficients
                model['C_Pf_model'] = C_Pf_identified
                model['C_Pr_model'] = C_Pr_identified
                
    # Save the trained model with identified coefficients
    model_name = racecar_version +"_pacejka"
    car_model = get_dotdict(model_name)
    car_model.C_Pf = C_Pf_identified
    car_model.C_Pr = C_Pr_identified
    save(car_model)
    
    # Generate Look-Up Table (LUT) with the updated model
    log_info("LUT is being generated...")
    LookupGenerator(racecar_version, save_LUT_name).run_generator()
