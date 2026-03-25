import numpy as np

def compute_slip_angles(vx, vy, omega, delta, lf, lr):
    """
    Computes front and rear slip angles using a pure numpy implementation.
    Clips vx to avoid division by zero (minimum 0.01 m/s).
    
    Args:
        vx (np.ndarray or float): longitudinal velocity [m/s]
        vy (np.ndarray or float): lateral velocity [m/s]
        omega (np.ndarray or float): yaw rate [rad/s]
        delta (np.ndarray or float): steering angle [rad]
        lf (float): CG-to-front-axle distance [m]
        lr (float): CG-to-rear-axle distance [m]
        
    Returns:
        alpha_f, alpha_r (tuple of np.ndarray): computed front and rear slip angles.
    """
    # Ensure vx is at least 0.01 or at most -0.01 (handling positive and negative appropriately)
    # However, since longitudinal velocity is usually positive, limiting to max(0.01, vx) is safer
    # or clipping by 0.01 magnitude.
    # We will use np.where or np.clip, but since it's a signed velocity, let's use:
    if isinstance(vx, np.ndarray):
        vx_safe = np.where(np.abs(vx) < 0.01, np.sign(vx) * 0.01, vx)
        vx_safe = np.where(vx_safe == 0.0, 0.01, vx_safe)  # handle true 0.0 case
    else:
        vx_safe = np.sign(vx) * 0.01 if abs(vx) < 0.01 else vx
        if vx_safe == 0.0:
            vx_safe = 0.01

    alpha_f = delta - np.arctan2(vy + lf * omega, vx_safe)
    alpha_r = -np.arctan2(vy - lr * omega, vx_safe)
    
    return alpha_f, alpha_r
