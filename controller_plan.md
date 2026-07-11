# Overview

I want to create a 2 ros2 packages, the first package will be called [ontroller_manager] will switch between 2 models for controlling the vehicle (pure pursuit and mpc) the pure pursuit will be operate intially when the system start to identify the model parameters, after that the parameter identified in from [on track system](On-Track-SysID/) after collecting data and parameters are updated it should switch safely the mpc controller.

The second package will be MPC controller for the car to replace the pure pursuit controller [pure_pursuit](pure_pursuit/), it should take the same inputs as the pure pursuit controller and produce the same outputs, it should also have a safety mechanism to switch back to pure pursuit if the MPC controller fails.


adjust the [pure_pursuit] and [On-Track-SysID] to suit the new controller archeticture.

# Step-by-Step Workflow

### 1. Initial State & Baseline Control
When the system first boots up, the exact dynamic parameters of the vehicle are unknown.
* The **System Identification** node signals the **Controller Manager** that this is the initial phase using the `First_run` (`std_msgs/bool`) topic.
* Because the advanced model isn't ready yet, the **Controller Manager** sends a `Start_Working` (`std_msgs/bool`) signal to the **Pure Pursuit Node**.
* The **Pure Pursuit Node** begins publishing steering and velocity commands via the `drive` (`ackermann_msgs/msg/AckermannDriveStamped`) topic directly to the **Car**. It also continuously reports its operational status back to the manager via `pp_state`.

### 2. Parameter Estimation
While the vehicle is being driven by the Pure Pursuit controller, the system is actively learning.
* The **System Identification** node observes the vehicle's behavior and calculates its dynamic properties.
* Once it calculates a reliable model, it sends the `estimated_param` (`std_msgs/Float32MultiArray`) to the **Controller Manager**.
* It also triggers the `Is_param_updated` (`std_msgs/bool`) flag to notify the manager that new, accurate parameters are available.

### 3. Controller Handover (Switching to MPC)
Once the system parameters are identified, the manager orchestrates a transition to the more advanced controller.
* The **Controller Manager** forwards the newly learned parameters to the **Model Predictive Node** via the `Identified_param` (`std_msgs/Float32MultiArray`) topic.
* It sends a `Param_updated` (`std_msgs/bool`) flag to the MPC node to confirm the parameters are fresh.
* The manager then activates the MPC by sending a `Start_Working` (`std_msgs/bool`) signal to the **Model Predictive Node** (and implicitly or explicitly stops the Pure Pursuit node to prevent conflicting commands).
* The **Model Predictive Node** takes over, utilizing the identified parameters to calculate optimal trajectories, and begins sending its own `drive` commands (`ackermann_msgs/msg/AckermannDriveStamped`) to the **Car**. It reports its status back to the manager via `mpc_state`.
* The **System Identification** node still observes the vehicle's behavior and calculates its dynamic properties again for any-change.
* Once the parameters update via in loop identification, it sends the `estimated_param` (`std_msgs/Float32MultiArray`) to the **Controller Manager**.
* It also triggers the `Is_param_updated` (`std_msgs/bool`) flag to notify the manager that new, accurate parameters are available after the MPC has reterived this parameter it should toogle it and system identification begin another learning.

# Safety Requirements for Switching

To ensure a safe transition between the Pure Pursuit controller and the MPC controller, the following conditions must be met before switching:

## 1. Vehicle State Validation

### A. Vehicle Speed Threshold
-   **Condition**: The vehicle's longitudinal velocity ($v_x$) must be above a minimum threshold to ensure sufficient traction and steering authority for the MPC controller.
-   **Requirement**: $v_x > v_{min}$ (e.g., 0.5 m/s).
-   **Action**: If $v_x \leq v_{min}$, the Pure Pursuit controller must remain active.

### B. Track Alignment
-   **Condition**: The vehicle must be sufficiently aligned with the track center.
-   **Requirement**: Lateral error $|e_y| < e_{y,max}$ (e.g., 0.1 m) AND heading error $|\Delta\theta| < \theta_{max}$ (e.g., 0.1 radians).
-   **Action**: If the vehicle is too far from the track, Pure Pursuit should maintain control until the vehicle is properly positioned.

## 2. System Health Checks

### A. MPC Solver Health
-   **Condition**: The MPC solver must be fully initialized and capable of finding a valid solution.
-   **Requirement**: The MPC node must publish a `is_healthy` status topic (boolean).
-   **Action**: Only switch to MPC if `is_healthy == true`.
-   **Action**: If `is_healthy == false`, Pure Pursuit should remain active or if MPC is active it should switch to Pure Pursuit safely untill solver return healthy signal again.

### B. Data Freshness
-   **Condition**: The vehicle state data must be recent to ensure the MPC model uses current information.
-   **Requirement**: Timestamp of the last received state $\Delta t_{state} < \Delta t_{max}$ (e.g., 0.1s).
-   **Action**: If data is stale, continue with the current controller (or a failsafe) until fresh data is received.

## 3. Control Performance Validation

### A. Error Convergence
-   **Condition**: The lateral error ($e_y$) must be decreasing (converging) under the current controller, indicating stable operation.
-   **Requirement**: The rate of change of lateral error $\frac{de_y}{dt} < 0$.
-   **Action**: Do not switch to MPC if the vehicle is oscillating or the error is increasing (unless MPC is explicitly required to handle the oscillation).

## 4. Smooth Handover Mechanism

### A. State Prediction
-   **Condition**: Before switching, the MPC controller must predict the initial state to ensure a smooth start.
-   **Requirement**: MPC publishes `predicted_initial_state`.
-   **Action**: Compare predicted vs. actual initial states. If the mismatch is large, delay the switch.

### B. Proportional Handover
-   **Condition**: Avoid sudden changes in control output.
-   **Requirement**: Implement a smooth transition function for the steering command over a short time interval $\Delta t_{switch}$.

## 5. Emergency Failsafes

### A. Controller Timeout
-   **Condition**: If the current controller fails to provide a valid command for a specified duration.
-   **Requirement**: Timeout $\Delta t_{timeout} = 1.0s$.
-   **Action**: If timeout occurs, attempt switch to the other controller.

### B. Hardcoded Safety Parameters
-   **Condition**: In case of complete system failure.
-   **Requirement**: Maintain a hardcoded list of "safe" steering angles for known track segments (if applicable) or default to minimal steering.


Rule:
- Use the DO-MPC library at first research and understand it [DoMpc](https://www.do-mpc.com/en/latest/)
- add to the MPC package a config file to able to control the MPC parameters.
- add to the controller manager a config file to able to control the controller manager parameters.