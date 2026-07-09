#!/usr/bin/env python3
import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QSlider, QPushButton, QDoubleSpinBox, QGroupBox)
from PyQt5.QtCore import Qt, QTimer
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import collections

class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes1 = self.fig.add_subplot(211)
        self.axes2 = self.fig.add_subplot(212)
        self.fig.tight_layout(pad=3.0)
        super(MplCanvas, self).__init__(self.fig)

class TuningNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_tuning_gui')
        
        self.target_vel = 0.0
        self.current_vel = 0.0
        self.control_effort = 0.0
        
        # Subscriptions
        self.create_subscription(Float64, '/debug/target_velocity', self.target_cb, 10)
        self.create_subscription(Float64, '/debug/current_velocity', self.current_cb, 10)
        self.create_subscription(Float64, '/debug/control_effort', self.effort_cb, 10)
        
        # Parameter client for the pure_pursuit node
        self.param_client = self.create_client(SetParameters, '/pure_pursuit/set_parameters')
        
    def target_cb(self, msg):
        self.target_vel = msg.data
        
    def current_cb(self, msg):
        self.current_vel = msg.data
        
    def effort_cb(self, msg):
        self.control_effort = msg.data

    def send_parameter(self, name, value, param_type):
        """Send a parameter update to the pure_pursuit node (non-blocking)."""
        if not self.param_client.service_is_ready():
            self.get_logger().warn(
                f'Service /pure_pursuit/set_parameters not available! '
                f'Is the pure_pursuit node running?'
            )
            return
            
        req = SetParameters.Request()
        param = Parameter()
        param.name = name
        param.value = ParameterValue()
        param.value.type = param_type
        if param_type == ParameterType.PARAMETER_DOUBLE:
            param.value.double_value = float(value)
            
        req.parameters = [param]
        future = self.param_client.call_async(req)
        future.add_done_callback(
            lambda fut: self._log_result(fut, name, value)
        )

    def _log_result(self, future, name, value):
        """Log the result of a set_parameters service call."""
        try:
            response = future.result()
            if response.results and response.results[0].successful:
                self.get_logger().info(f'✓ Parameter {name} set to {value}')
            else:
                reason = response.results[0].reason if response.results else 'unknown'
                self.get_logger().warn(f'✗ Failed to set {name}: {reason}')
        except Exception as e:
            self.get_logger().error(f'Service call failed for {name}: {e}')


class TuningApp(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Pure Pursuit PID Tuning")
        self.resize(800, 800)
        
        # Data for plots
        self.max_len = 100
        self.time_data = collections.deque(maxlen=self.max_len)
        self.target_data = collections.deque(maxlen=self.max_len)
        self.current_data = collections.deque(maxlen=self.max_len)
        self.effort_data = collections.deque(maxlen=self.max_len)
        self.t = 0
        
        self.init_ui()
        
        # Plot update timer
        self.plot_timer = QTimer()
        self.plot_timer.setInterval(100) # 10 Hz
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start()

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)
        
        # Parameters group
        param_group = QGroupBox("PID Parameters")
        param_layout = QVBoxLayout()
        
        self.params = {
            'kp_vel': {'range': (0.0, 10.0), 'default': 1.0, 'step': 0.1},
            'ki_vel': {'range': (0.0, 5.0), 'default': 0.0, 'step': 0.01},
            'kd_vel': {'range': (0.0, 5.0), 'default': 0.0, 'step': 0.01},
            'lookahead_distance': {'range': (0.5, 5.0), 'default': 1.5, 'step': 0.1}
        }
        
        self.spinboxes = {}
        for p_name, p_info in self.params.items():
            row = QHBoxLayout()
            label = QLabel(p_name)
            label.setMinimumWidth(150)
            row.addWidget(label)
            
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(p_info['range'][0] / p_info['step']))
            slider.setMaximum(int(p_info['range'][1] / p_info['step']))
            slider.setValue(int(p_info['default'] / p_info['step']))
            
            spinbox = QDoubleSpinBox()
            spinbox.setRange(p_info['range'][0], p_info['range'][1])
            spinbox.setSingleStep(p_info['step'])
            spinbox.setValue(p_info['default'])
            
            self.spinboxes[p_name] = spinbox
            
            # Connect
            slider.valueChanged.connect(lambda v, sb=spinbox, step=p_info['step']: sb.setValue(v * step))
            spinbox.valueChanged.connect(lambda v, sl=slider, step=p_info['step']: sl.setValue(int(v / step)))
            
            # Send param button
            btn = QPushButton("Set")
            btn.clicked.connect(lambda checked, name=p_name, sb=spinbox: self.send_param(name, sb.value()))
            
            row.addWidget(slider)
            row.addWidget(spinbox)
            row.addWidget(btn)
            param_layout.addLayout(row)

        # "Set All" button
        set_all_btn = QPushButton("Set All Parameters")
        set_all_btn.setStyleSheet("font-weight: bold; padding: 5px;")
        set_all_btn.clicked.connect(self.send_all_params)
        param_layout.addWidget(set_all_btn)

        param_group.setLayout(param_layout)
        layout.addWidget(param_group)
        
        # Plot canvas
        self.canvas = MplCanvas(self, width=5, height=6, dpi=100)
        layout.addWidget(self.canvas)
        
        self.line_target, = self.canvas.axes1.plot([], [], 'r-', label='Target Vel')
        self.line_current, = self.canvas.axes1.plot([], [], 'b-', label='Current Vel')
        self.canvas.axes1.legend(loc='upper right')
        self.canvas.axes1.set_ylabel('Velocity (m/s)')
        self.canvas.axes1.set_title('Velocity Tracking')
        self.canvas.axes1.grid(True)
        
        self.line_effort, = self.canvas.axes2.plot([], [], 'g-', label='Control Effort')
        self.canvas.axes2.legend(loc='upper right')
        self.canvas.axes2.set_xlabel('Time Steps (10Hz)')
        self.canvas.axes2.set_ylabel('Effort')
        self.canvas.axes2.set_title('PID Control Effort')
        self.canvas.axes2.grid(True)

    def send_param(self, name, value):
        self.ros_node.send_parameter(name, value, ParameterType.PARAMETER_DOUBLE)

    def send_all_params(self):
        """Send all current parameter values at once."""
        for name, spinbox in self.spinboxes.items():
            self.send_param(name, spinbox.value())

    def update_plot(self):
        self.t += 1
        self.time_data.append(self.t)
        self.target_data.append(self.ros_node.target_vel)
        self.current_data.append(self.ros_node.current_vel)
        self.effort_data.append(self.ros_node.control_effort)
        
        self.line_target.set_data(list(self.time_data), list(self.target_data))
        self.line_current.set_data(list(self.time_data), list(self.current_data))
        self.canvas.axes1.relim()
        self.canvas.axes1.autoscale_view()
        
        self.line_effort.set_data(list(self.time_data), list(self.effort_data))
        self.canvas.axes2.relim()
        self.canvas.axes2.autoscale_view()
        
        self.canvas.draw()

def main(args=None):
    rclpy.init(args=args)
    ros_node = TuningNode()
    
    # Spin the ROS node in a background thread so service calls
    # (call_async futures) are processed without blocking the Qt event loop.
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()
    
    app = QApplication(sys.argv)
    ex = TuningApp(ros_node)
    ex.show()
    
    ret = app.exec_()
    
    rclpy.shutdown()
    spin_thread.join(timeout=2.0)
    sys.exit(ret)

if __name__ == '__main__':
    main()
