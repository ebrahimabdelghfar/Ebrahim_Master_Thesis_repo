# Estimation Benchmark ROS2 Package

This package provides a framework for real-time validation and benchmarking of vehicle dynamics state estimation natively inside ROS2. It was designed to validate the predictive quality of dynamic single-track models iteratively identified by the `On-Track-SysID` package.

## Core Concepts

The benchmarking node continuously listens to odometry (`/odom`) and steering commands (`/drive`). To compute error metrics, it compares **what the model predicts will happen** with **what actually happens on the track**.

### How Prediction Benchmarking Works (Pseudo-Code)

Because sensors and commands arrive at different times and frequencies, the node maintains a synchronized, timestamped sliding window (circular buffer) of historical states.

#### 1. Data Buffering & Time Synchronization
```python
# Runs continuously as /odom and /drive messages arrive
function on_sensor_data(v_x, v_y, omega, delta, current_timestamp):
    # Store the state in a circular buffer
    buffer.append({
        time: current_timestamp,
        v_x: v_x, 
        v_y: v_y, 
        omega: omega, 
        delta: delta
    })
    
    # Run the benchmarking matching logic
    run_benchmarking_step()
```

#### 2. Multi-Step Benchmarking Loop
For every new odometry sample, we look back exactly $N$ steps into the buffer. We take that historical state and ask the model to predict forward $N$ steps. We then compare that prediction to the *current* measured state.

```python
function run_benchmarking_step():
    current_state = buffer.last()
    
    # Evaluate multiple prediction horizons simultaneously (e.g., 1-step, 5-step, 10-step)
    for N in [1, 5, 10]:
        if buffer.length <= N:
            continue # Not enough history buffered yet
            
        # 1. Get the historic state from exactly N temporal steps ago
        historical_state = buffer.get_from_end(index = N)
        
        # 2. Predict forward N steps STARTING from the historical state.
        #    Note: Longitudinal velocity (v_x) and steering (delta) are assumed
        #    to remain constant over the open-loop prediction horizon.
        v_y_predicted, omega_predicted = predict_multi_step(
            initial_v_y = historical_state.v_y,
            initial_omega = historical_state.omega,
            constant_v_x = historical_state.v_x,
            constant_delta = historical_state.delta,
            steps_to_predict = N,
            dt = 0.02
        )
        
        # 3. Compare the multi-step prediction against the ACTUAL current state
        track_metrics(
            N_step, 
            real_v_y = current_state.v_y,          predicted_v_y = v_y_predicted,
            real_omega = current_state.omega,      predicted_omega = omega_predicted
        )
```

#### 3. Open-Loop Roll-Forward (The Physics Engine)
This is the core physics prediction implemented in `multi_step_predictor.py`. It discretely steps the continuous Pacejka differential equations forward in time using a **configurable numerical integrator**.

Crucially, in a multi-step prediction ($N > 1$), **the model feeds its own predictions back into itself** for the next step, without any interim sensor corrections.

All integrators share the same dynamics function $f(x)$ that computes the state derivatives:

```python
function dynamics(v_x, v_y, omega, delta) -> (v_y_dot, omega_dot):
    # 1. Calculate slip angles (kinematics)
    alpha_front = -arctan( (v_y + omega * length_front) / v_x ) + delta
    alpha_rear  = -arctan( (v_y - omega * length_rear)  / v_x )
    
    # 2. Calculate tire lateral forces using Pacejka Magic Formula
    Force_front = Pacejka(C_Pf, alpha_front, F_z_front)
    Force_rear  = Pacejka(C_Pr, alpha_rear,  F_z_rear)
    
    # 3. Calculate accelerations (dynamics derivatives)
    v_y_dot   = (1 / mass) * (Force_rear + Force_front * cos(delta) - mass * v_x * omega)
    omega_dot = (1 / inertia_z) * (Force_front * length_front * cos(delta) - Force_rear * length_rear)
    
    return v_y_dot, omega_dot
```

The integrator then uses this to step the state forward. The `integration_method` config parameter selects which:

```python
function predict_multi_step(initial_v_y, initial_omega, v_x, delta, steps, dt, method):
    v_y = initial_v_y
    omega = initial_omega
    step_fn = get_integrator(method)  # "euler", "heun", or "rk4"
    
    for step = 1 to steps:
        v_y, omega = step_fn(v_x, v_y, omega, delta, dt)  # Each calls dynamics() internally
        
    return v_y, omega
```

### Configurable Integration Methods

The package supports three numerical integrators, set via `integration_method` in `benchmark_config.yaml`:

| Method | Order | Dynamics Evals / Step | Formula | Best For |
|--------|-------|----------------------|---------|----------|
| `euler` | 1st | 1 | $x_{k+1} = x_k + dt \cdot f(x_k)$ | Speed, paper baseline |
| `heun` | 2nd | 2 | $x_{k+1} = x_k + \frac{dt}{2}(f(x_k) + f(x_k + dt \cdot f(x_k)))$ | Accuracy/speed balance |
| `rk4` | 4th | 4 | $x_{k+1} = x_k + \frac{dt}{6}(k_1 + 2k_2 + 2k_3 + k_4)$ | Maximum accuracy |

**How each works in pseudo-code:**

```python
# --- Forward Euler (1st-order) ---
function step_euler(v_x, v_y, omega, delta, dt):
    k1_vy, k1_om = dynamics(v_x, v_y, omega, delta)
    return v_y + dt * k1_vy, omega + dt * k1_om

# --- Heun / Improved Euler (2nd-order) ---
function step_heun(v_x, v_y, omega, delta, dt):
    k1_vy, k1_om = dynamics(v_x, v_y, omega, delta)                            # Slope at start
    k2_vy, k2_om = dynamics(v_x, v_y + dt*k1_vy, omega + dt*k1_om, delta)      # Slope at end
    return v_y + dt/2 * (k1_vy + k2_vy), omega + dt/2 * (k1_om + k2_om)        # Average

# --- Classic Runge-Kutta (4th-order) ---
function step_rk4(v_x, v_y, omega, delta, dt):
    k1_vy, k1_om = dynamics(v_x, v_y,                     omega,                     delta)
    k2_vy, k2_om = dynamics(v_x, v_y + dt/2 * k1_vy,      omega + dt/2 * k1_om,      delta)
    k3_vy, k3_om = dynamics(v_x, v_y + dt/2 * k2_vy,      omega + dt/2 * k2_om,      delta)
    k4_vy, k4_om = dynamics(v_x, v_y + dt   * k3_vy,      omega + dt   * k3_om,      delta)
    return v_y   + dt/6 * (k1_vy + 2*k2_vy + 2*k3_vy + k4_vy),
           omega + dt/6 * (k1_om + 2*k2_om + 2*k3_om + k4_om)
```

**Why does this matter?** Euler uses one slope sample (the start), Heun averages the start and end slopes, and RK4 samples four points across the interval. With stiff tire dynamics and aggressive cornering, higher-order methods produce significantly more accurate predictions per step.

---

## Example: A 10-Step Prediction Walkthrough

Let's look at exactly what happens when the car is driving and we evaluate a **10-step prediction** at 50 Hz ($dt = 0.02$ seconds). A 10-step horizon means predicting $0.20$ seconds into the future.

### Background Setup
At $T = 1.00s$, the car is cornering. Our sensors give us:
- **Velocity ($v_x$)**: $3.0$ m/s
- **Lateral Velocity ($v_y$)**: $0.1$ m/s
- **Yaw Rate ($\omega$)**: $0.2$ rad/s
- **Steering Command ($\delta$)**: $0.15$ rad

This data is saved in our historical buffer.

### The Present Moment ($T = 1.20s$)
Exactly $0.20$ seconds later ($10$ steps $\times 0.02s$), our sensors report the *actual* new state of the car:
- **Actual $v_y$**: $0.14$ m/s
- **Actual $\omega$**: $0.25$ rad/s

Did our model know this was going to happen? To find out, the benchmarking node pulls the historical data from $T = 1.00s$ out of the buffer and runs the `predict_multi_step` function.

### The Open-Loop Simulation (Inside the node)
The algorithm locks in the steering angle ($0.15$ rad) and the speed ($3.0$ m/s) from $T=1.00s$. It then simulates the physics for 10 loops:

* **Loop 1 (Predicting $T=1.02s$):**
  Uses $v_y=0.10, \omega=0.20$. Calculates slip angles, pushes them through Pacejka tire curves to get lateral forces, and calculates acceleration. Euler integration updates the state. 
  *Result*: predicted $v_y=0.105, \omega=0.206$

* **Loop 2 (Predicting $T=1.04s$):**
  Uses the *predicted* $v_y=0.105, \omega=0.206$ from Loop 1 (no sensor update here!). Calculates new slip angles and forces.
  *Result*: predicted $v_y=0.110, \omega=0.211$

* ...(Loops 3 through 9 continue feeding predictions into themselves)...

* **Loop 10 (Predicting $T=1.20s$):**
  Uses the *predicted* states from Loop 9. Calculates final forces and integration.
  *Final Result*: predicted $v_y=0.138, \omega=0.246$

### The Benchmarking Evaluation
The algorithm immediately compares the final prediction against the actual recorded state at $T=1.20s$:
- **Prediction**: $v_y=0.138, \omega=0.246$
- **Actual**: $v_y=0.140, \omega=0.250$
- **Errors**: $e_{vy} = 0.002$ m/s, $e_{\omega} = 0.004$ rad/s

These errors are then fed into Welford's online algorithm to instantly update the running benchmarking statistics (RMSE, MAE, R², etc.) published on the ROS2 topic for the `10-step` metric.

By running this continuously across hundreds of samples, the package provides a highly robust, statistically significant measure of how accurate the Pacejka vehicle model truly is over a given time horizon.

---

## Launch Instructions

```bash
# Basic start with RK4 (default, waits for On-Track-SysID to finish training)
ros2 launch estimation_benchmark estimation_benchmark.launch.py

# Use Heun integrator instead
ros2 launch estimation_benchmark estimation_benchmark.launch.py integration_method:=heun

# Use Euler (matches original paper implementation)
ros2 launch estimation_benchmark estimation_benchmark.launch.py integration_method:=euler

# With CarMaker ground truth tire force benchmarking enabled
ros2 launch estimation_benchmark estimation_benchmark.launch.py enable_tire_force_benchmark:=true
```
