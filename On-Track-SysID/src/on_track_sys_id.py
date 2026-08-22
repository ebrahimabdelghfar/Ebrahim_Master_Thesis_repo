#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import numpy as np
import os
import yaml
import csv
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32, Float64MultiArray, String, Bool
from sensor_msgs.msg import Imu
from datetime import datetime
import sys # Ensure sys is imported
import torch
from tqdm import tqdm

from ament_index_python.packages import get_package_share_directory
from adaptive_controller_interfaces.srv import IdentifiedParam

# Import helpers - handle both installed and development paths
try:
    from helpers.train_model import nn_train, get_model_param, resolve_device
    from helpers.pacejka_formula import pacejka_formula
    from helpers.benchmarking_metrics import OnlineBenchmark
    from helpers.friction_warmstart import estimate_mu_from_buffer, estimate_mu_from_imu
except ImportError:
    import sys
    # Add the src directory to path for development
    src_path = os.path.dirname(os.path.abspath(__file__))
    if src_path not in sys.path:
        sys.path.insert(0, src_path)
    from helpers.train_model import nn_train, get_model_param, resolve_device
    from helpers.pacejka_formula import pacejka_formula
    from helpers.benchmarking_metrics import OnlineBenchmark
    from helpers.friction_warmstart import estimate_mu_from_buffer, estimate_mu_from_imu


class OnTrackSysId(Node):
    def __init__(self):
        super().__init__('on_track_sys_id')
        # Declare parameters
        self.declare_parameter('racecar_version', 'SIM')
        self.declare_parameter('save_LUT_name', 'NUCx_on_track_pacejka')
        self.declare_parameter('plot_model', False)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('ackermann_cmd_topic', '/drive')
        # Measured road-wheel angle. The /drive command is a SETPOINT: on the
        # CARLA asurt_fsai the achieved angle is 0.76 +/- 0.03 of the
        # commanded one (measured 2026-08-22 against
        # /sim/feedback/steering_angles' wheel_FL/FR joints over 13-21 m/s),
        # which biases alpha_f high by ~25 % and the identified front
        # cornering stiffness low by the same factor: fitting the same run
        # with the command gave C_front = 13725 N/rad against CARLA's own
        # 17198 N/rad, and with the measured angle 18486 N/rad.
        # Empty string = disabled, use the command (previous behaviour).
        self.declare_parameter('steering_feedback_topic', '')
        self.declare_parameter('steering_feedback_units', 'rad')   # rad | deg
        self.declare_parameter('steering_feedback_scale', 1.0)
        self.declare_parameter('steering_feedback_timeout_s', 1.0)
        self.declare_parameter('benchmarking_log_interval', 100)
        self.declare_parameter('reidentification_interval_s', 30.0)
        # If enabled, every accepted identification cycle is also forwarded
        # (best-effort, fire-and-forget) to tire_force_benchmark via the same
        # IdentifiedParam service contract adaptive_controller_manager uses
        # for its own benchmark_update_params_enable/service - see
        # forward_to_benchmark(). Off by default.
        self.declare_parameter('benchmark_update_params_enable', False)
        self.declare_parameter('benchmark_update_params_service', 'benchmark/update_params')
        # Get parameters
        self.racecar_version = self.get_parameter('racecar_version').value
        self.save_LUT_name = self.get_parameter('save_LUT_name').value
        self.plot_model = self.get_parameter('plot_model').value
        odom_topic = self.get_parameter('odom_topic').value
        ackermann_topic = self.get_parameter('ackermann_cmd_topic').value
        self.bench_log_interval = self.get_parameter('benchmarking_log_interval').value
        self.steering_fb_topic = self.get_parameter('steering_feedback_topic').value
        units = str(self.get_parameter('steering_feedback_units').value).lower()
        if units not in ('rad', 'deg'):
            raise ValueError(f"steering_feedback_units must be 'rad' or 'deg', got {units!r}")
        self.steering_fb_gain = float(self.get_parameter('steering_feedback_scale').value) * \
            (np.pi / 180.0 if units == 'deg' else 1.0)
        self.steering_fb_timeout_s = float(self.get_parameter('steering_feedback_timeout_s').value)
        self._steering_fb_value = None
        self._steering_fb_seen = False
        self._steering_fb_ticks = 0
        self._steering_fb_warned = False
        self.benchmark_update_params_enable = bool(
            self.get_parameter('benchmark_update_params_enable').value)
        benchmark_update_params_service = self.get_parameter('benchmark_update_params_service').value
        # Print parameters
        self.get_logger().info(f"Racecar_version: {self.racecar_version}")
        self.get_logger().info(f"Save_LUT_name: {self.save_LUT_name}")
        self.get_logger().info(f"Plot_model: {self.plot_model}")
        self.get_logger().info(f"Odom_topic: {odom_topic}")
        self.get_logger().info(f"Ackermann_topic: {ackermann_topic}")
        # Initialize variables
        self.rate = 50
        self.package_path = get_package_share_directory('on_track_sys_id')
        # Load parameters
        self.load_parameters()
        self.log_torch_device()
        self.setup_data_storage()

        # Sensor-stream QoS. The /odom publisher (simulator / CarMaker bridge)
        # offers BEST_EFFORT, while a subscription created with a plain depth
        # requests RELIABLE. That request is incompatible with a best-effort
        # offer, so DDS refuses the match, silently delivers nothing and only
        # logs:
        #   "New publisher discovered on topic '/odom', offering incompatible
        #    QoS. No messages will be received from it.
        #    Last incompatible policy: RELIABILITY"
        # Requesting BEST_EFFORT is compatible with BOTH best-effort and
        # reliable publishers, so it is the safe request for high-rate sensor
        # data. depth stays 1: this node always wants the freshest sample.
        qos_sensor = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscribe to the topics using the loaded parameter values
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_cb,
            qos_sensor
        )
        self.ackermann_sub = self.create_subscription(
            AckermannDriveStamped,
            ackermann_topic,
            self.ackermann_cb,
            1
        )

        # Measured road-wheel angle, when the platform publishes one. Same
        # BEST_EFFORT reasoning as qos_sensor above - this is sensor feedback.
        if self.steering_fb_topic:
            self.steering_fb_sub = self.create_subscription(
                Float32, self.steering_fb_topic, self.steering_fb_cb, qos_sensor)
            self.get_logger().info(
                f"Using measured steering from {self.steering_fb_topic} "
                f"(units={units}, scale={self.get_parameter('steering_feedback_scale').value}); "
                f"{ackermann_topic} is used only as a fallback if it goes stale.")
        else:
            self.get_logger().warn(
                "steering_feedback_topic is empty - identifying against the /drive steering "
                "SETPOINT. Any actuator gain or lag is then absorbed into the front tire "
                "coefficients.")

        # Publishers for estimated state, sensor state, and error
        self.est_state_pub = self.create_publisher(Float64MultiArray, '/estimated_state', 1)
        self.sensor_state_pub = self.create_publisher(Float64MultiArray, '/sensor_state', 1)
        self.error_pub = self.create_publisher(Float64MultiArray, '/estimation_error', 1)

        # Academic benchmarking publishers
        self.bench_vy_pub = self.create_publisher(
            Float64MultiArray, '/benchmarking/vy_metrics', 1)
        self.bench_omega_pub = self.create_publisher(
            Float64MultiArray, '/benchmarking/omega_metrics', 1)
        self.bench_summary_pub = self.create_publisher(
            String, '/benchmarking/summary', 1)

        # True until the first successful identification, then false for the
        # node's life. Latched so a late-starting subscriber (e.g.
        # adaptive_controller_manager) still sees the current value.
        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.first_run_pub = self.create_publisher(Bool, 'sysid/first_run', qos_latched)
        self.first_run_pub.publish(Bool(data=True))

        # Submits identified tire params to adaptive_controller_manager;
        # replaces the old one-shot latched /sysid/training_complete String.
        self.update_params_cli = self.create_client(IdentifiedParam, 'sysid/update_params')

        # Always created (cheap) - only ever called if
        # benchmark_update_params_enable is true, see forward_to_benchmark().
        self.benchmark_update_params_cli = self.create_client(
            IdentifiedParam, benchmark_update_params_service)

        # Online benchmarking accumulators
        self.bench_vy = OnlineBenchmark(name='v_y (lateral velocity)')
        self.bench_omega = OnlineBenchmark(name='omega (yaw rate)')
        self.bench_sample_count = 0

        # Load model parameters for estimation
        try:
            self.model_params = get_model_param(self.racecar_version)
            self.init_model_constants()
        except Exception as e:
            self.get_logger().warn(f"Could not load model parameters: {e}")
            self.model_params = None

        # Cold-start friction warm-start (see maybe_compute_warm_start_mu()) -
        # applies only once, to the node's first-ever identification cycle.
        self._is_first_identification = True
        warm_start_cfg = (self.model_params or {}).get('friction_warm_start', {})
        if warm_start_cfg.get('enable', False) and warm_start_cfg.get('accel_source', 'finite_diff') == 'imu':
            imu_topic = warm_start_cfg.get('imu_topic', '/imu')
            # Same reasoning as qos_sensor above: IMU drivers almost always
            # publish best-effort, and this subscription would fail the same
            # silent way.
            self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_cb, qos_sensor)

        self.prev_v_y = 0.0
        self.prev_omega = 0.0
        self.last_time = self.get_clock().now()
        self.current_time = self.get_clock().now()

        # Create timer for main loop
        timer_period = 1.0 / self.rate
        self.loop_timer = self.create_timer(timer_period, self.loop_callback)
        
        # State tracking. Phases: 'COLLECTING' (initial fill of the rolling
        # data buffer) -> 'TRAINING' (run identification, submit params) ->
        # 'AWAITING_ACK' (non-blocking wait for adaptive_controller_manager's
        # response) -> 'RUNNING' (estimation + periodic re-identification
        # every reidentification_interval_s). collect_data() keeps refreshing
        # the rolling buffer in every phase past 'COLLECTING' so each
        # periodic retrain uses fresh on-track data, not the original window.
        self.phase = 'COLLECTING'
        self.pending_update_future = None
        self.next_reid_time = None
        self.pbar = None

    def init_model_constants(self):
        mp = self.model_params
        self.m = mp['m']
        self.I_z = mp['I_z']
        self.l_f = mp['l_f']
        self.l_r = mp['l_r']
        self.l_wb = mp['l_wb']
        self.F_zf = self.m * 9.81 * self.l_r / self.l_wb
        self.F_zr = self.m * 9.81 * self.l_f / self.l_wb
        self.C_Pf_model = mp['C_Pf_model']
        self.C_Pr_model = mp['C_Pr_model']

    def setup_data_storage(self):
        """
        Set up storage for collected data.
        """
        self.data_duration = self.nn_params['data_collection_duration']
        self.timesteps = self.data_duration * self.rate
        self.data = np.zeros((self.timesteps, 4))
        self.counter = 0
        self.current_state = np.zeros(4)
        self._collection_logged = False

        # Friction warm-start (accel_source: imu path) - a rolling buffer of
        # [a_x, a_y] kept in exact lockstep with self.data (appended at the
        # same collect_data() ticks), so state/accel samples are 1:1 aligned
        # by construction regardless of the IMU's actual publish rate
        # (zero-order-hold: each tick uses the most recent IMU sample seen).
        # Harmless to always allocate even if accel_source: finite_diff.
        self._imu_accel_buffer = np.zeros((self.timesteps, 2))
        self._latest_imu_accel = np.zeros(2)
        self._imu_msg_count = 0

    def load_parameters(self):
        """
        Load neural network parameters from params/nn_params.yaml
        """
        yaml_file = os.path.join(self.package_path, 'params', 'nn_params.yaml')
        with open(yaml_file, 'r') as file:
            self.nn_params = yaml.safe_load(file)

    def log_torch_device(self):
        """
        Logs which device (CPU or GPU) torch will actually train/simulate on,
        resolved the same way train_model.py's resolve_device() will resolve
        it at training time (nn_params.yaml's `device: auto|cpu|cuda`) - so
        this is a truthful preview, not a separate/possibly-inconsistent check.

        Also printed to stderr (not just get_logger().info()): sys_id.launch.py
        runs this node with `--log-level warn` (suppresses INFO-level rclpy
        logs entirely - see that launch file's own comment) and `output='log'`
        (keeps this node's stdout out of the live `ros2 launch` console,
        routing it to the per-run log file instead). stderr is the one
        channel that's still empirically visible on the launch console under
        that config (e.g. scipy's import-time UserWarning shows up there) -
        mirroring onto it is what actually makes this line visible live,
        rather than only ever landing in a log file nobody's tailing.
        """
        device = resolve_device(self.nn_params)
        if device.type == 'cuda':
            gpu_name = torch.cuda.get_device_name(device)
            msg = (f"Torch device: cuda ({gpu_name}, torch {torch.__version__}, "
                   f"CUDA {torch.version.cuda})")
        else:
            msg = f"Torch device: cpu (torch {torch.__version__})"
        self.get_logger().info(msg)
        print(f"[on_track_sys_id] {msg}", file=sys.stderr, flush=True)

    def export_data_as_csv(self):
        """
        Export collected data as a CSV file.
        """
        # Auto-export without prompting in ROS2
        self.get_logger().info("Exporting data as CSV...")
        data_dir = os.path.join(self.package_path, 'data', self.racecar_version)
        if not os.path.exists(data_dir):
            os.makedirs(data_dir)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_file = os.path.join(data_dir, f'{self.racecar_version}_sys_id_data_{timestamp}.csv')
        
        # Write data to CSV file    
        with open(csv_file, mode='w') as file:
            writer = csv.writer(file)
            writer.writerow(['v_x', 'v_y', 'omega', 'delta'])
            for row in self.data:
                writer.writerow(row)
        self.get_logger().info(f"DATA HAS BEEN EXPORTED TO: {csv_file}")

    def odom_cb(self, msg):
        self.current_state[0] = msg.twist.twist.linear.x
        self.current_state[1] = msg.twist.twist.linear.y
        self.current_state[2] = msg.twist.twist.angular.z
        self.current_time = rclpy.time.Time.from_msg(msg.header.stamp)
        
    def ackermann_cb(self, msg):
        # Only the fallback when a measured steering angle is configured and
        # fresh - see steering_fb_cb / _steering_is_fresh.
        if not self._steering_is_fresh():
            self.current_state[3] = msg.drive.steering_angle

    def steering_fb_cb(self, msg):
        self._steering_fb_value = float(msg.data) * self.steering_fb_gain
        self._steering_fb_ticks = 0
        self._steering_fb_seen = True
        self.current_state[3] = self._steering_fb_value

    def _steering_is_fresh(self):
        """Staleness in LOOP TICKS, not wall-clock seconds.

        A training cycle blocks this node's single-threaded executor for tens
        of seconds, during which no callback runs at all. Wall-clock ageing
        reports every stream as dead the moment training returns (measured:
        30-35 s "dropouts" on a stream whose real worst gap is 185 ms).
        Counting ticks of this node's own 50 Hz loop makes the check immune to
        that, because the loop is stalled by exactly the same amount.
        """
        if not self.steering_fb_topic or not self._steering_fb_seen:
            return False
        max_ticks = max(1, int(self.steering_fb_timeout_s * self.rate))
        if self._steering_fb_ticks > max_ticks:
            if not self._steering_fb_warned:
                self.get_logger().warn(
                    f"No message on {self.steering_fb_topic} for {self._steering_fb_ticks} loop "
                    f"ticks (> {self.steering_fb_timeout_s:.1f} s) - falling back to the /drive "
                    "steering setpoint.")
                self._steering_fb_warned = True
            return False
        self._steering_fb_warned = False
        return True

    def imu_cb(self, msg):
        self._latest_imu_accel[0] = msg.linear_acceleration.x
        self._latest_imu_accel[1] = msg.linear_acceleration.y
        self._imu_msg_count += 1

    def collect_data(self):
        """
        Collects data during simulation. Called every tick regardless of
        phase (not just during initial collection) so the rolling buffer
        stays fresh for periodic re-identification in the RUNNING phase.
        """
        if self.current_state[0] > 1:  # Only collect data when the car is moving
            self.data = np.roll(self.data, -1, axis=0)
            self.data[-1] = self.current_state
            self._imu_accel_buffer = np.roll(self._imu_accel_buffer, -1, axis=0)
            self._imu_accel_buffer[-1] = self._latest_imu_accel
            self.counter += 1

            # Log progress bar every 2% to avoid spamming too much but keeping it fluid
            update_interval = max(1, self.timesteps // 50)
            if not self._collection_logged and (
                self.counter % update_interval == 0 or self.counter == self.timesteps
            ):
                percent = (self.counter / self.timesteps) * 100
                bar_length = 20
                filled_length = int(bar_length * self.counter // self.timesteps)
                bar = '=' * filled_length + '-' * (bar_length - filled_length)
                self.get_logger().info(f"Collecting data: [{bar}] {percent:.1f}% ({self.counter}/{self.timesteps})")

        else:
            # Show waiting message occasionally
            if self.counter == 0 and not hasattr(self, '_waiting_logged'):
                self.get_logger().info("Waiting for car to move (velocity > 1 m/s)...")
                self._waiting_logged = True

        if self.counter >= self.timesteps:
            # Guarded so this doesn't spam every tick once counter stays
            # >= timesteps between periodic re-identification cycles.
            if not self._collection_logged:
                self.get_logger().info("Data collection completed.")
                self._collection_logged = True
            return True
        return False
            
    def maybe_compute_warm_start_mu(self):
        """
        Non-vision friction warm-start (see helpers/friction_warmstart.py):
        estimates a cold-start D_f/D_r initial guess from the buffer this
        node already collects, replacing the static pacejka_params.yaml
        default for the FIRST identification cycle only. Must be called
        (and read self.data) BEFORE run_nn_train() - train_model.py's
        filter_data() mutates its training_data argument's columns in place,
        and self.data is passed by reference, so reading after training
        would silently see Butterworth-filtered data instead of raw odom.
        """
        if not self._is_first_identification or self.model_params is None:
            return None
        cfg = self.model_params.get('friction_warm_start', {})
        if not cfg.get('enable', False):
            return None

        l_f, l_r = self.model_params['l_f'], self.model_params['l_r']
        accel_source = cfg.get('accel_source', 'finite_diff')

        if accel_source == 'imu':
            if self._imu_msg_count == 0:
                # By the time this runs, data_collection_duration seconds
                # have already elapsed - if the IMU topic were real, at
                # least one message would have arrived by now. No need for
                # a separate imu_wait_timeout_s timer on top of that.
                self.get_logger().warn(
                    f"friction_warm_start.accel_source is 'imu' but no message ever "
                    f"arrived on {cfg.get('imu_topic', '/imu')} - falling back to finite_diff.")
                mu_hat, n_used = estimate_mu_from_buffer(self.data.copy(), l_f, l_r, 1.0 / self.rate, cfg)
            else:
                mu_hat, n_used = estimate_mu_from_imu(
                    self.data.copy(), self._imu_accel_buffer.copy(), l_f, l_r, cfg)
        else:
            mu_hat, n_used = estimate_mu_from_buffer(self.data.copy(), l_f, l_r, 1.0 / self.rate, cfg)

        if mu_hat is None:
            self.get_logger().warn(
                f"Friction warm-start: only {n_used} low-slip samples "
                f"(< min_samples={cfg.get('min_samples', 20)}) - falling back to "
                "static pacejka_params.yaml D default.")
            return None

        self.get_logger().info(
            f"Friction warm-start ({accel_source}): mu_hat={mu_hat:.4f} from {n_used} low-slip samples.")
        return mu_hat

    def run_nn_train(self, warm_start_mu=None):
        """
        Initiates training of the neural network using collected data.
        """
        self.get_logger().info("Training neural network...")
        return nn_train(self.data, self.racecar_version, self.save_LUT_name, self.plot_model,
                        warm_start_mu=warm_start_mu)


    def publish_estimates(self):
        """
        Calculates and publishes estimated state and error using the identified parameters.
        """
        if self.model_params is None:
            return

        # Check if we have new data based on timestamp
        if self.current_time <= self.last_time:
            return

        # Calculate dt in seconds
        dt = (self.current_time - self.last_time).nanoseconds / 1e9
        
        # If dt is too small or negative, skip
        if dt <= 0.00001:
            return
            
        # If dt is too large, reset
        if dt > 0.2:
            self.last_time = self.current_time
            self.prev_v_y = self.current_state[1]
            self.prev_omega = self.current_state[2]
            return

        self.last_time = self.current_time

        v_x = self.current_state[0]
        v_y_real = self.current_state[1]
        omega_real = self.current_state[2]
        delta = self.current_state[3]

        # Skip if car is stopped
        if v_x < 0.1:
            self.prev_v_y = v_y_real
            self.prev_omega = omega_real
            return

        # One-Step Prediction using REAL previous measurements
        alpha_f = -np.arctan((self.prev_v_y + self.prev_omega * self.l_f) / v_x) + delta
        alpha_r = -np.arctan((self.prev_v_y - self.prev_omega * self.l_r) / v_x)

        # Pacejka forces
        F_f = pacejka_formula(self.C_Pf_model, alpha_f, self.F_zf)
        F_r = pacejka_formula(self.C_Pr_model, alpha_r, self.F_zr)

        # Dynamics derivatives
        v_y_dot = (1/self.m) * (F_r + F_f * np.cos(delta) - self.m * v_x * self.prev_omega)
        omega_dot = (1/self.I_z) * (F_f * self.l_f * np.cos(delta) - F_r * self.l_r)

        # Predicted next state
        v_y_pred = self.prev_v_y + v_y_dot * dt
        omega_pred = self.prev_omega + omega_dot * dt

        # Store current real state for next prediction
        self.prev_v_y = v_y_real
        self.prev_omega = omega_real

        # Publish Sensor State
        sensor_msg = Float64MultiArray()
        sensor_msg.data = [v_x, v_y_real, omega_real, delta]
        self.sensor_state_pub.publish(sensor_msg)

        # Publish Estimated/Predicted State
        est_msg = Float64MultiArray()
        est_msg.data = [v_x, v_y_pred, omega_pred]
        self.est_state_pub.publish(est_msg)

        # Publish Error (backward-compatible)
        err_msg = Float64MultiArray()
        err_msg.data = [abs(v_y_real - v_y_pred), abs(omega_real - omega_pred)]
        self.error_pub.publish(err_msg)

        # --- Academic Benchmarking ---
        self.bench_vy.update(v_y_real, v_y_pred)
        self.bench_omega.update(omega_real, omega_pred)
        self.bench_sample_count += 1

        # Publish metric arrays [RMSE, MAE, NRMSE, MaxAE, Bias, StdDev, R²]
        vy_metrics_msg = Float64MultiArray()
        vy_metrics_msg.data = self.bench_vy.get_metrics_array()
        self.bench_vy_pub.publish(vy_metrics_msg)

        omega_metrics_msg = Float64MultiArray()
        omega_metrics_msg.data = self.bench_omega.get_metrics_array()
        self.bench_omega_pub.publish(omega_metrics_msg)

        # Publish human-readable summary
        summary_msg = String()
        summary_msg.data = (
            self.bench_vy.get_summary_string() + '\n' +
            self.bench_omega.get_summary_string()
        )
        self.bench_summary_pub.publish(summary_msg)

        # Periodic logging to terminal
        if self.bench_sample_count % self.bench_log_interval == 0:
            self.get_logger().info(
                '\n' + self.bench_vy.get_summary_string() +
                '\n' + self.bench_omega.get_summary_string()
            )

    def run_identification_cycle(self):
        """
        Trains on the current data buffer, reloads model constants, and
        submits the identified tire params to adaptive_controller_manager
        via the sysid/update_params service (non-blocking - see
        handle_pending_ack for the response side). Runs once for the
        initial bootstrap identification, then again every
        reidentification_interval_s while in the RUNNING phase.
        """
        if not self.update_params_cli.service_is_ready():
            # adaptive_controller_manager isn't up yet (or restarted) -
            # retry next tick. Cheap check, so no need to also skip the
            # (expensive) retrain below more than one tick at a time.
            return

        warm_start_mu = self.maybe_compute_warm_start_mu()
        identified = self.run_nn_train(warm_start_mu=warm_start_mu)
        self._is_first_identification = False
        self.export_data_as_csv()

        self.get_logger().info("Reloading parameters for estimation...")
        try:
            self.model_params = get_model_param(self.racecar_version)
            self.init_model_constants()
            self.get_logger().info("Parameters reloaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to reload parameters: {e}")

        # get_model_param() re-reads C_P*_model from pacejka_params.yaml, which
        # is the static prior and NOT what was just identified (nothing writes
        # that file back). Take the identified set straight from nn_train, so
        # both the estimator below and the service request carry it.
        if identified is not None:
            self.C_Pf_model = [float(v) for v in identified[0]]
            self.C_Pr_model = [float(v) for v in identified[1]]
            self.get_logger().info(
                f"Identified C_Pf={self.C_Pf_model} C_Pr={self.C_Pr_model}")
        else:
            self.get_logger().warn(
                "nn_train returned no coefficients - submitting the static prior instead.")


        self.last_time = self.current_time
        # Re-arm the rolling-window progress counter/log for the next
        # collection cycle - the data buffer itself keeps rolling
        # continuously regardless (see collect_data), this only affects
        # the cosmetic "collecting data" progress log.
        self.counter = 0
        self._collection_logged = False

        request = IdentifiedParam.Request()
        request.param_values = [float(v) for v in self.C_Pf_model] + \
            [float(v) for v in self.C_Pr_model]
        self.pending_update_future = self.update_params_cli.call_async(request)
        self.phase = 'AWAITING_ACK'
        self.get_logger().info(
            "Submitted identified params via sysid/update_params, awaiting ack...")

        # Idempotent after the first identification - harmless to republish
        # on every subsequent re-identification cycle too.
        self.first_run_pub.publish(Bool(data=False))

        if self.benchmark_update_params_enable:
            self.forward_to_benchmark(request.param_values)

    def forward_to_benchmark(self, param_values):
        """
        Best-effort forward of a freshly-identified tire param set to a
        passive benchmarking node (tire_force_benchmark), independent of the
        adaptive_controller_manager arming FSM - unlike sysid/update_params,
        this consumer isn't actuating anything, so it should see every
        identification immediately rather than wait for a handover window.
        Fire-and-forget: no ack means just a warning, no retry (the next
        re-identification cycle naturally resends anyway).
        """
        if not self.benchmark_update_params_cli.service_is_ready():
            self.get_logger().warn(
                "benchmark_update_params_enable is true but "
                f"{self.benchmark_update_params_cli.srv_name} is not available - "
                "is tire_force_benchmark running?")
            return

        request = IdentifiedParam.Request()
        # param_values here is read back off another IdentifiedParam.Request
        # (rosidl's fixed-size float array field), which returns a numpy
        # ndarray - list(...) alone yields numpy.float32 elements, and this
        # field's setter strictly requires each element be a Python float.
        request.param_values = [float(v) for v in param_values]

        def _on_response(future):
            response = future.result()
            if response is None or not response.ack:
                self.get_logger().warn(
                    "benchmark/update_params was not acked by tire_force_benchmark")

        future = self.benchmark_update_params_cli.call_async(request)
        future.add_done_callback(_on_response)

    def handle_pending_ack(self):
        """
        Non-blocking check of the sysid/update_params future. A literal
        blocking wait (e.g. spin_until_future_complete) from inside this
        timer callback would deadlock, since this node spins on a single
        default executor thread - so the ack is polled across ticks
        instead, while estimation keeps running in the meantime.
        """
        if self.pending_update_future is None or not self.pending_update_future.done():
            return

        future = self.pending_update_future
        self.pending_update_future = None
        try:
            response = future.result()
            ack = bool(response.ack) if response is not None else False
        except Exception as e:
            self.get_logger().error(f"sysid/update_params call failed: {e}")
            ack = False

        interval_s = self.get_parameter('reidentification_interval_s').value
        self.next_reid_time = self.get_clock().now() + Duration(seconds=interval_s)

        if ack:
            self.get_logger().info("Identified params acked by adaptive_controller_manager.")
        else:
            # Manager rejected the submission (implausible values) or the
            # call itself failed. Keep the previous (already-applied)
            # model and just retry at the next normal interval, rather
            # than immediately retraining in a tight loop.
            self.get_logger().warn(
                "sysid/update_params was not acked - keeping previous model, "
                "will retry at the next reidentification interval.")
        self.phase = 'RUNNING'

    def loop_callback(self):
        """
        Main loop callback - handles data collection, training, ack-waiting,
        and estimation phases.
        """
        self._steering_fb_ticks += 1

        # Runs in every phase past initial collection so the rolling buffer
        # stays fresh for periodic re-identification (see collect_data).
        collection_filled = self.collect_data()

        if self.phase == 'COLLECTING':
            if collection_filled:
                self.phase = 'TRAINING'
            return

        if self.phase == 'TRAINING':
            self.run_identification_cycle()
            return

        if self.phase == 'AWAITING_ACK':
            # Estimation shouldn't stall just because the ack hasn't
            # landed yet - AWAITING_ACK is a sub-state layered on RUNNING.
            self.publish_estimates()
            self.handle_pending_ack()
            return

        # RUNNING
        self.publish_estimates()
        if self.next_reid_time is not None and self.get_clock().now() >= self.next_reid_time:
            self.phase = 'TRAINING'


def main(args=None):
    rclpy.init(args=args)
    
    node = OnTrackSysId()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()