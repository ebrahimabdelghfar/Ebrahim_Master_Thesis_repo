I want you to act as a senior ROS2 autonomous vehicle control engineer and MPC architect.

Your task is to design and specify a production-quality ROS2 MPC package for vehicle path tracking.

## Objective
Create a ROS2 package that implements Model Predictive Control (MPC) for tracking a raceline / trajectory, similar in interface to a [pure_pursuit](pure_pursuit/), but using a dynamic bicycle model. The package must be modular, configurable, and suitable for real vehicle integration and simulator testing.

## Main requirements
1. The package must subscribe to a raceline / reference trajectory topic, similar to how a [pure_pursuit](pure_pursuit/) controller consumes the track or planned path.
2. The package must publish steering commands and velocity commands, and optionally predicted trajectory / debug topics.
3. The package must include a configuration file (YAML) that allows tuning of:
   - state cost matrix / state weights
   - input cost matrix / input weights
   - terminal cost
   - topic names
   - vehicle parameters
   - MPC horizon
   - sampling time (must be adaptive and maintain system stability)
   - solver settings
   - actuator limits and rate limits
   - all model parameters used in the equations
4. The controller should use a dynamic bicycle vehicle model, not kinematic model.
5. The tire model should be based on identified parameters from an on-track system identification process, especially front and rear tire parameters from the Magic Formula / Pacejka model `[[Bf , Cf , Df , Ef , Br , Cr , Dr , Er ]]` per axis.
6. The design must support future extension to real vehicle deployment, logging, validation, and benchmarking.

## What I want from you
First, perform a research-style design study before proposing code.

Then provide the result in the following structure:

### 1. Best MPC approach
Recommend the best practical MPC formulation for this use case:
- linear MPC
- linear time-varying MPC
- nonlinear MPC

Explain which one should be used first for an automotive path tracking controller with a dynamic bicycle model, and justify the tradeoff between accuracy, robustness, implementation complexity, and real-time feasibility. A practical staged recommendation is preferred.

### 2. Vehicle model
Define the correct equations for a dynamic bicycle model for path tracking, including:
- vehicle states
- control inputs
- slip angles
- lateral tire forces
- yaw dynamics
- lateral dynamics
- path-relative error states if appropriate
- continuous-time model
- discretization approach

Also explain when to use:
- linear tire model with cornering stiffness
- Pacejka / Magic Formula tire model
- gain scheduling or online linearization

### 3. Solver selection
Recommend the best solver for ROS2 real-time MPC implementation and compare options such as:
- OSQP
- qpOASES
- ACADOS
- IPOPT
- CVXGEN

### 4. ROS2 package architecture
Design the ROS2 package architecture with:
- node responsibilities
- subscribed topics
- published topics
- parameter files
- launch files
- message types
- debug topics
- diagnostics
- fallback behavior

I want a clean package design that is easy to test and extend.

Please include suggested files/classes such as:
- mpc_node.cpp
- mpc_controller.cpp
- vehicle_model.cpp
- solver_interface.cpp
- reference_trajectory_handler.cpp
- parameter_manager.cpp
- debug_publisher.cpp

### 5. Configuration design
Propose a clean YAML parameter structure for:
- ROS topics
- cost matrices
- horizon settings
- vehicle geometry and mass properties
- tire model parameters
- actuator dynamics
- constraints
- solver options
- debug options

Make the configuration practical and maintainable.

### 6. Interfaces
Define the expected input/output interfaces clearly:
- subscribed raceline / trajectory topic
- odometry / vehicle state topic
- steering feedback topic
- output steering command topic
- optional acceleration command topic
- predicted trajectory topic
- controller status topic

### 7. Deliverables
I want the final answer to include:
- architecture recommendation
- equations
- solver recommendation
- ROS2 package structure
- YAML config example
- implementation roadmap
- practical engineering advice
- common pitfalls to avoid

## Constraints
- Prefer practical engineering decisions over purely academic ones.
- Target ROS2 and C++ primarily.
- Assume this is for a racecar / autonomous vehicle path tracking controller.
- Assume the raceline is already available from another node.
- Focus first on lateral and longitudinal control.
- Prioritize a design that can run in real time.

## Important
Do not jump directly into code.
First give the best system design, solver choice, and mathematically correct equations.