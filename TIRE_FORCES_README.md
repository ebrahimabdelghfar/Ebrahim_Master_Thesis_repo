# Ground Truth Pacejka Tire Model Publisher

## Overview
This implementation publishes ground truth tire forces and slip data from the CarMaker Pacejka tire model simulation to ROS2.

## Published Topics

### `/tire_forces` (hellocm_msgs/TireForcesArray)
Published at **100Hz** with comprehensive tire data for all four wheels.

#### Message Structure
Each tire message contains:

**Forces (in wheel coordinate system FrW):**
- `fx` - Longitudinal force (braking/traction) [N]
- `fy` - Lateral force (cornering) [N]
- `fz` - Normal force (vertical load) [N]

**Torques (in wheel coordinate system FrW):**
- `mx` - Overturning moment [Nm]
- `my` - Rolling resistance moment [Nm]
- `mz` - Self-aligning torque (Mz) [Nm]

**Pacejka Model Slip Parameters:**
- `slip_angle` - Slip angle (alpha) [rad] - ISO 8855 7.1.2
- `long_slip` - Longitudinal slip (kappa) [-] - ISO 8855 7.4.6
- `turn_slip` - Turn slip [1/m]

**Kinematic Parameters:**
- `inclination_angle` - Camber/inclination angle [rad]
- `effective_radius` - Effective belt radius [m]
- `belt_velocity` - Rotational rubber velocity at contact point [m/s]

**Contact Point (global frame):**
- `contact_point_x`, `contact_point_y`, `contact_point_z` [m]

**Road Properties:**
- `mu_road` - Road friction coefficient [-]
- `on_road` - Boolean indicating if tire is on the road

**Additional State:**
- `rim_rotation_speed` - Rim angular velocity [rad/s]
- `angular_velocity` - Tire rotational speed [rad/s]
- `tire_index` - Tire identification (0=FL, 1=FR, 2=RL, 3=RR)

## Usage

### Subscribing to Tire Forces
```python
import rclpy
from rclpy.node import Node
from hellocm_msgs.msg import TireForcesArray

class TireDataSubscriber(Node):
    def __init__(self):
        super().__init__('tire_data_subscriber')
        self.subscription = self.create_subscription(
            TireForcesArray,
            'tire_forces',
            self.tire_callback,
            10)
    
    def tire_callback(self, msg):
        # Access front left tire data
        fl = msg.front_left
        print(f"Front Left - Fx: {fl.fx:.2f} N, Fy: {fl.fy:.2f} N, Fz: {fl.fz:.2f} N")
        print(f"Slip Angle: {fl.slip_angle:.4f} rad, Long Slip: {fl.long_slip:.4f}")
        print(f"Self-aligning torque (Mz): {fl.mz:.2f} Nm")
        
        # Access other tires
        # msg.front_right, msg.rear_left, msg.rear_right

def main():
    rclpy.init()
    node = TireDataSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
```

### Viewing Messages in Terminal
```bash
# Echo all tire forces data
ros2 topic echo /tire_forces

# View message info
ros2 interface show hellocm_msgs/msg/TireForcesArray
ros2 interface show hellocm_msgs/msg/TireForces

# Check publishing rate
ros2 topic hz /tire_forces
```

## Technical Details

### Data Source
The tire data is extracted from CarMaker's `tTire` structure, specifically from the `tTireIF` interface which contains:
- Forces and torques calculated by the Pacejka tire model
- Slip parameters (alpha, kappa) used as inputs to the model
- Contact point kinematics
- Road surface properties

### Coordinate System
Forces and torques are expressed in the **wheel coordinate system (FrW)**:
- **X-axis**: Longitudinal direction (forward)
- **Y-axis**: Lateral direction (left)
- **Z-axis**: Normal direction (up)

### Tire Indexing
CarMaker uses the following tire indices:
- **0** = Front Left (FL)
- **1** = Front Right (FR)
- **2** = Rear Left (RL)
- **3** = Rear Right (RR)

## Applications

This ground truth data can be used for:

1. **Vehicle Dynamics Parameter Estimation**
   - Estimate tire stiffness coefficients
   - Validate lateral and longitudinal dynamics models
   - System identification for tire models

2. **Control Algorithm Development**
   - Validate traction control systems
   - Develop anti-lock braking systems (ABS)
   - Electronic stability control (ESC)

3. **State Estimation Validation**
   - Compare estimated vs. ground truth slip angles
   - Validate friction estimation algorithms
   - Benchmark tire force observers

4. **Data-Driven Modeling**
   - Train neural networks for tire force prediction
   - Learn tire model parameters
   - Develop adaptive control strategies

## Notes

- The publisher runs at 100Hz by default (can be adjusted in `CMNode_ROS2_HelloCM.cpp`)
- Data is only valid when the simulation is running
- Forces are computed by CarMaker's Pacejka tire model implementation
- The `on_road` flag indicates if the tire has ground contact

## Related Topics

- `/tire_steering` - Tire steering angles (100Hz)
- `/odom` - Ground truth odometry (100Hz)
- `/VehicleControl` - Vehicle control inputs

## Files Modified

1. `hellocm_msgs/msg/TireForces.msg` - Individual tire data message
2. `hellocm_msgs/msg/TireForcesArray.msg` - All four tires array message
3. `hellocm_cmnode/src/CMNode_ROS2_HelloCM.hpp` - Header with callback declaration
4. `hellocm_cmnode/src/CMNode_ROS2_HelloCM.cpp` - Implementation of tire forces publisher
