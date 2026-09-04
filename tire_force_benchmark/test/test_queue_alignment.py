"""Regression tests for tire_force_benchmark_node.py.

sim_manager_msgs (the CARLA bridge's TireForces message package) lives in a
separate workspace that is not sourced in this dev environment, so it's
stubbed into sys.modules before importing the node module.
Node.create_subscription/create_publisher are monkeypatched so tests can
construct the node and drive its callbacks directly without a live ROS graph.
adaptive_controller_interfaces IS a real, built workspace package (unlike
sim_manager_msgs) - run these tests with the workspace sourced,
e.g. `source install/setup.bash` before `pytest`, or `IdentifiedParam` import
below will fail.
"""
import json
import math
import sys
import types
from pathlib import Path

import pytest
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from adaptive_controller_interfaces.srv import IdentifiedParam
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String


def _stub_sim_manager_msgs():
    if 'sim_manager_msgs.msg' in sys.modules:
        return sys.modules['sim_manager_msgs.msg']

    sim_manager_msgs = types.ModuleType('sim_manager_msgs')
    sim_manager_msgs_msg = types.ModuleType('sim_manager_msgs.msg')

    class _Stamp:
        def __init__(self):
            self.sec = 0
            self.nanosec = 0

    class TireForces:
        def __init__(self):
            self.stamp = _Stamp()
            self.wheel_names = ['FL', 'FR', 'RL', 'RR']
            self.slip_angle = [0.0] * 4
            self.tire_friction = [0.0] * 4
            self.wheel_speed = [0.0] * 4
            self.slip_ratio = [0.0] * 4
            self.normal_load = [0.0] * 4
            self.lateral_force = [0.0] * 4
            self.longitudinal_force = [0.0] * 4
            self.wheel_torque = [0.0] * 4

    sim_manager_msgs_msg.TireForces = TireForces
    sim_manager_msgs.msg = sim_manager_msgs_msg
    sys.modules['sim_manager_msgs'] = sim_manager_msgs
    sys.modules['sim_manager_msgs.msg'] = sim_manager_msgs_msg
    return sim_manager_msgs_msg


TireForces = _stub_sim_manager_msgs().TireForces

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tire_force_benchmark import tire_force_benchmark_node as node_module  # noqa: E402
from tire_force_benchmark.tire_force_benchmark_node import TireForceBenchmarkNode  # noqa: E402


def _ros_args(overrides):
    args = ['--ros-args']
    for key, value in overrides.items():
        if isinstance(value, bool):
            value_str = 'true' if value else 'false'
        elif isinstance(value, (list, tuple)):
            value_str = '[' + ','.join(str(v) for v in value) + ']'
        else:
            value_str = str(value)
        args += ['-p', f'{key}:={value_str}']
    return args


class _FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _FakeSubscription:
    """Distinct truthy object per create_subscription call, so tests can
    assert whether a subscription was (not) created via `is (not) None`."""


class NodeUnderTest:
    """Context manager: builds a TireForceBenchmarkNode with param overrides,
    with create_subscription/create_publisher stubbed so no live ROS graph
    or real hellocm_msgs type support is needed."""

    def __init__(self, overrides):
        self.overrides = overrides
        self._orig_create_subscription = Node.create_subscription
        self._orig_create_publisher = Node.create_publisher
        self.node = None

    def __enter__(self):
        rclpy.init(args=_ros_args(self.overrides))
        Node.create_subscription = lambda self, msg_type, topic, cb, qos: _FakeSubscription()
        Node.create_publisher = lambda self, msg_type, topic, qos: _FakePublisher()
        try:
            self.node = TireForceBenchmarkNode()
        except Exception:
            Node.create_subscription = self._orig_create_subscription
            Node.create_publisher = self._orig_create_publisher
            rclpy.shutdown()
            raise
        return self.node

    def __exit__(self, exc_type, exc_value, traceback):
        Node.create_subscription = self._orig_create_subscription
        Node.create_publisher = self._orig_create_publisher
        if self.node is not None:
            self.node.destroy_node()
        rclpy.shutdown()
        return False


def _make_tire_msg(index, fy=None, fz=10.0, slip_angle=0.01, tire_friction=1.05):
    msg = TireForces()
    msg.stamp.sec = index
    msg.stamp.nanosec = 0
    msg.tire_friction = [float(tire_friction)] * 4
    value = float(index) if fy is None else float(fy)
    msg.lateral_force = [value] * 4
    msg.normal_load = [float(fz)] * 4
    msg.slip_angle = [float(slip_angle)] * 4
    return msg


def _make_string_msg(payload):
    msg = String()
    msg.data = json.dumps(payload)
    return msg


def _make_est_msg(index, offset=100.0):
    msg = Float64MultiArray()
    msg.data = [offset + index] * 4
    return msg


def _make_odom_msg(sec, nanosec, v_x, v_y, omega):
    msg = Odometry()
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.twist.twist.linear.x = v_x
    msg.twist.twist.linear.y = v_y
    msg.twist.twist.angular.z = omega
    return msg


def _make_ackermann_msg(delta):
    msg = AckermannDriveStamped()
    msg.drive.steering_angle = delta
    return msg


# --- min_fz_threshold mandatory ---

def test_min_fz_threshold_missing_raises():
    with pytest.raises(RuntimeError):
        with NodeUnderTest({}):
            pass


def test_min_fz_threshold_present_constructs_and_filters():
    with NodeUnderTest({'min_fz_threshold': 5.0}) as node:
        assert node.min_fz == pytest.approx(5.0)

        assert node._use_sample(0.01, 4.9) is False
        assert node._use_sample(0.01, 5.1) is True


# --- standstill frames (slip and forces published as exactly 0) ---

def test_standstill_samples_are_dropped():
    with NodeUnderTest({
        'min_fz_threshold': 1.0,
        'enable_state_benchmarking': False,
        'c_pf': [6.63, 1.1052, 0.4316, 0.5193],
        'c_pr': [7.8594, 1.5468, 0.3589, 0.5631],
    }) as node:
        # normal_load stays live at rest, so only the all-zero slip/force
        # pattern distinguishes a parked frame from a real one.
        node.tire_forces_callback(_make_tire_msg(0, fy=0.0, fz=700.0, slip_angle=0.0))
        assert node.metrics['fl_fy'].metrics()['n_samples'] == 0

        node.tire_forces_callback(_make_tire_msg(1, fy=120.0, fz=700.0, slip_angle=0.02))
        assert node.metrics['fl_fy'].metrics()['n_samples'] == 1


# --- enable_force_benchmarking toggle ---

def test_force_benchmarking_disabled_skips_no_subscriptions_and_callbacks():
    with NodeUnderTest({
        'min_fz_threshold': 1.0,
        'enable_force_benchmarking': False,
        'enable_state_benchmarking': False,
        'c_pf': [6.63, 1.1052, 0.4316, 0.5193],
        'c_pr': [7.8594, 1.5468, 0.3589, 0.5631],
    }) as node:
        assert node.enable_force_benchmarking is False
        assert node.sub is None
        assert node.ext_sub is None
        assert node.estimation_pub is None

        node.tire_forces_callback(_make_tire_msg(0, fy=1.0))
        assert node.metrics['fl_fy'].metrics()['n_samples'] == 0


def test_force_benchmarking_enabled_by_default():
    with NodeUnderTest({'min_fz_threshold': 1.0, 'enable_state_benchmarking': False}) as node:
        assert node.enable_force_benchmarking is True
        assert node.sub is not None


# --- queue alignment: lead=0 regression + lead=2 (the bug) ---

def test_queue_alignment_lead_zero():
    with NodeUnderTest({
        'min_fz_threshold': 1.0,
        'benchmark_mode': 'external_topic',
        'external_prediction_lead_samples': 0,
        'enable_state_benchmarking': False,
    }) as node:
        recorded = []
        node._benchmark_and_publish = lambda stamp_sec, fy_gt, fy_est, publish_estimate: recorded.append(
            (fy_gt[0], fy_est[0])
        )

        for i in range(5):
            node.tire_forces_callback(_make_tire_msg(i))
            node.estimated_fy_callback(_make_est_msg(i))

        assert recorded == [(float(k), 100.0 + k) for k in range(5)]


def test_queue_alignment_lead_two():
    with NodeUnderTest({
        'min_fz_threshold': 1.0,
        'benchmark_mode': 'external_topic',
        'external_prediction_lead_samples': 2,
        'enable_state_benchmarking': False,
    }) as node:
        recorded = []
        node._benchmark_and_publish = lambda stamp_sec, fy_gt, fy_est, publish_estimate: recorded.append(
            (fy_gt[0], fy_est[0])
        )

        for i in range(8):
            node.tire_forces_callback(_make_tire_msg(i))
            node.estimated_fy_callback(_make_est_msg(i))

        # estimate[k] must pair with ground_truth[k+lead] - this drifted
        # further apart every iteration before the fix (over-popping the GT
        # queue by lead+1 instead of sliding it by exactly 1).
        expected = [(float(k + 2), 100.0 + k) for k in range(6)]
        assert recorded == expected


# --- state estimation math ---

def test_state_prediction_matches_reference_formula():
    m, I_z, l_f, l_r, l_wb = 3.5, 0.0627, 0.17, 0.155, 0.325
    c_pf = [6.63, 1.1052, 0.4316, 0.5193]
    c_pr = [7.8594, 1.5468, 0.3589, 0.5631]

    with NodeUnderTest({
        'min_fz_threshold': 1.0,
        'm': m, 'I_z': I_z, 'l_f': l_f, 'l_r': l_r, 'l_wb': l_wb,
        'c_pf': c_pf, 'c_pr': c_pr,
    }) as node:
        assert node.enable_state_benchmarking is True
        assert node.have_identified_params is True

        captured = []
        node._benchmark_state = lambda stamp_sec, v_x, v_y_real, omega_real, delta, v_y_pred, omega_pred, v_y_prev, omega_prev: (
            captured.append((v_y_pred, omega_pred))
        )

        prev_v_y, prev_omega, delta, v_x = 0.05, 0.3, 0.05, 2.0
        node.odom_callback(_make_odom_msg(0, 0, v_x, prev_v_y, prev_omega))
        node.ackermann_callback(_make_ackermann_msg(delta))

        dt = 0.02
        node.odom_callback(_make_odom_msg(0, int(dt * 1e9), v_x, 0.06, 0.32))

        from tire_force_benchmark.tire_force_benchmark_node import pacejka_formula

        g = 9.81
        f_zf = m * g * l_r / l_wb
        f_zr = m * g * l_f / l_wb
        alpha_f = -math.atan((prev_v_y + prev_omega * l_f) / v_x) + delta
        alpha_r = -math.atan((prev_v_y - prev_omega * l_r) / v_x)
        f_f = float(pacejka_formula(c_pf, alpha_f, f_zf))
        f_r = float(pacejka_formula(c_pr, alpha_r, f_zr))
        v_y_dot = (1.0 / m) * (f_r + f_f * math.cos(delta) - m * v_x * prev_omega)
        omega_dot = (1.0 / I_z) * (f_f * l_f * math.cos(delta) - f_r * l_r)
        expected_v_y_pred = prev_v_y + v_y_dot * dt
        expected_omega_pred = prev_omega + omega_dot * dt

        assert len(captured) == 1
        assert captured[0][0] == pytest.approx(expected_v_y_pred, rel=1e-6)
        assert captured[0][1] == pytest.approx(expected_omega_pred, rel=1e-6)


def test_state_benchmarking_disabled_without_vehicle_constants():
    with NodeUnderTest({'min_fz_threshold': 1.0}) as node:
        assert node.enable_state_benchmarking is False


# --- plot export smoke test ---

def test_plot_export_creates_expected_files(tmp_path):
    plot_dir = tmp_path / 'plots'
    with NodeUnderTest({
        'min_fz_threshold': 1.0,
        'plot_output_dir': str(plot_dir),
        'm': 3.5, 'I_z': 0.0627, 'l_f': 0.17, 'l_r': 0.155, 'l_wb': 0.325,
        'c_pf': [6.63, 1.1052, 0.4316, 0.5193],
        'c_pr': [7.8594, 1.5468, 0.3589, 0.5631],
    }) as node:
        for i in range(5):
            node.tire_forces_callback(_make_tire_msg(i, fy=1.0 + 0.1 * i))
        node.ackermann_callback(_make_ackermann_msg(0.02))
        for i in range(5):
            node.odom_callback(_make_odom_msg(i, 0, 2.0, 0.05 + 0.001 * i, 0.3))

    expected_files = [
        'tire_forces_timeseries.png',
        'vehicle_states_timeseries.png',
        'tire_forces_error_hist.png',
        'vehicle_states_error_hist.png',
        'friction_mu_timeseries.png',
        'friction_mu_error_hist.png',
        'metrics_summary.png',
    ]
    for name in expected_files:
        f = plot_dir / name
        assert f.exists(), f'missing {name}'
        assert f.stat().st_size > 0


# --- CARLA PhysX nominal curve ---

def test_physx_lateral_force_matches_the_simulators_own_model():
    # PxVehicleComputeTireForceDefault at zero long. slip / camber, with
    # CARLA's asurt_fsai wheel values: saturates at exactly mu*Fz once
    # K = latStiff*|tan(alpha)|/(mu*Fz) reaches 3, and its initial slope is
    # latStiff = Fz * lat_stiff_value * S1(3/lat_stiff_max_load).
    mu, fz = 1.05, 650.0
    lat_stiff = fz * 17.0 * (1.5 - 1.5 ** 2 / 3.0 + 1.5 ** 3 / 27.0)

    slope = float(
        node_module.physx_lateral_force(1e-4, fz, mu, 17.0, 2.0) / 1e-4
    )
    assert slope == pytest.approx(lat_stiff, rel=1e-3)

    alpha_peak = math.atan(3.0 * mu * fz / lat_stiff)
    peak = float(node_module.physx_lateral_force(alpha_peak, fz, mu, 17.0, 2.0))
    assert peak == pytest.approx(mu * fz, rel=1e-3)
    # No falloff past the peak - this is what a Magic Formula cannot reproduce.
    beyond = float(node_module.physx_lateral_force(2.0 * alpha_peak, fz, mu, 17.0, 2.0))
    assert beyond == pytest.approx(mu * fz, rel=1e-3)

    assert node_module.physx_lateral_force(-0.05, fz, mu, 17.0, 2.0) == pytest.approx(
        -node_module.physx_lateral_force(0.05, fz, mu, 17.0, 2.0)
    )


def test_each_identification_iteration_gets_its_own_nominal_plot(tmp_path):
    # The shutdown export only shows the last identified model, so every set
    # of params pushed on identified_params_service gets a numbered snapshot.
    plot_dir = tmp_path / 'plots'
    with NodeUnderTest({
        'min_fz_threshold': 1.0,
        'enable_state_benchmarking': False,
        'plot_output_dir': str(plot_dir),
        'm': 3.5, 'I_z': 0.0627, 'l_f': 0.17, 'l_r': 0.155, 'l_wb': 0.325,
    }) as node:
        node.tire_forces_callback(_make_tire_msg(0, fy=1.0, tire_friction=0.8))

        for scale in (1.0, 1.2):
            node.handle_identified_params(
                _make_identified_param_request(
                    [6.63 * scale, 1.1052, 0.4316, 0.5193,
                     7.8594 * scale, 1.5468, 0.3589, 0.5631]),
                IdentifiedParam.Response(),
            )

        first = plot_dir / 'pacejka_identified_vs_nominal_iter_01.png'
        second = plot_dir / 'pacejka_identified_vs_nominal_iter_02.png'
        assert first.exists() and first.stat().st_size > 0
        assert second.exists() and second.stat().st_size > 0
        # Different params must give different figures - the whole point.
        assert first.read_bytes() != second.read_bytes()


def test_nominal_plot_uses_measured_friction_and_needs_no_prior_model(tmp_path):
    # nominal_source 'carla_physx' must not depend on nominal_model_file, and
    # must take mu from the telemetry rather than a configured value.
    plot_dir = tmp_path / 'plots'
    with NodeUnderTest({
        'min_fz_threshold': 1.0,
        'plot_output_dir': str(plot_dir),
        'm': 3.5, 'I_z': 0.0627, 'l_f': 0.17, 'l_r': 0.155, 'l_wb': 0.325,
        'c_pf': [6.63, 1.1052, 0.4316, 0.5193],
        'c_pr': [7.8594, 1.5468, 0.3589, 0.5631],
    }) as node:
        assert node.nominal_source == 'carla_physx'
        assert node.nominal_c_pf is None
        for i in range(5):
            node.tire_forces_callback(_make_tire_msg(i, fy=1.0 + 0.1 * i, tire_friction=0.8))
        assert node._nominal_friction('front') == pytest.approx(0.8)

    f = plot_dir / 'pacejka_identified_vs_nominal.png'
    assert f.exists() and f.stat().st_size > 0


def test_vehicle_physics_topic_overrides_the_configured_stiffness():
    # The bridge's latched /sim/feedback/vehicle_physics wins over the yaml
    # params, so changing the vehicle config cannot leave the curve stale.
    with NodeUnderTest({'min_fz_threshold': 1.0, 'enable_state_benchmarking': False}) as node:
        assert node._nominal_lat_stiff('front') == (17.0, 2.0)

        wheels = [{'name': n, 'lat_stiff_value': v, 'lat_stiff_max_load': 3.0}
                  for n, v in zip(['FL', 'FR', 'RL', 'RR'], [25.0, 25.0, 11.0, 11.0])]
        node.vehicle_physics_callback(_make_string_msg({'mass': 240.0, 'wheels': wheels}))

        assert node._nominal_lat_stiff('front') == (25.0, 3.0)
        assert node._nominal_lat_stiff('rear') == (11.0, 3.0)

        node.vehicle_physics_callback(_make_string_msg({'not': 'physics'}))
        assert node._nominal_lat_stiff('front') == (25.0, 3.0)


def test_mu_benchmark_scores_identified_d_against_reported_friction():
    # The identified Pacejka D is the axle peak friction, so it is scored
    # directly against the tire_friction the telemetry reports - and only
    # once an identification exists.
    with NodeUnderTest({
        'min_fz_threshold': 1.0, 'enable_state_benchmarking': False,
    }) as node:
        node.tire_forces_callback(_make_tire_msg(0, fy=1.0, tire_friction=0.80))
        assert node.metrics['front_mu'].metrics()['n_samples'] == 0

        node.handle_identified_params(
            _make_identified_param_request([6.63, 1.1052, 0.4316, 0.5193,
                                            7.8594, 1.5468, 0.3589, 0.5631]),
            IdentifiedParam.Response(),
        )
        node.tire_forces_callback(_make_tire_msg(1, fy=1.0, tire_friction=0.80))

        assert node.metrics['front_mu'].metrics()['n_samples'] == 1
        assert node.metrics['front_mu'].metrics()['bias'] == pytest.approx(0.80 - 0.4316)
        assert node.metrics['rear_mu'].metrics()['bias'] == pytest.approx(0.80 - 0.3589)


def test_nominal_friction_follows_a_runtime_change():
    # /sim/control/tire_friction can change grip mid-run: the nominal curve
    # must be drawn at the latest friction, not an average of both levels.
    with NodeUnderTest({'min_fz_threshold': 1.0, 'enable_state_benchmarking': False}) as node:
        node.tire_forces_callback(_make_tire_msg(0, fy=100.0, tire_friction=1.05))
        node.tire_forces_callback(_make_tire_msg(1, fy=100.0, tire_friction=1.05))
        node.tire_forces_callback(_make_tire_msg(2, fy=60.0, tire_friction=0.60))

        assert node._nominal_friction('front') == pytest.approx(0.60)
        assert node.mu_min['front'] == pytest.approx(0.60)
        assert node.mu_max['front'] == pytest.approx(1.05)


# --- identified-params service gating (no hardcoded c_pf/c_pr) ---

def _make_identified_param_request(values):
    req = IdentifiedParam.Request()
    req.param_values = [float(v) for v in values]
    return req


def test_internal_pacejka_waits_then_activates_on_service_call():
    with NodeUnderTest({'min_fz_threshold': 1.0, 'enable_state_benchmarking': False}) as node:
        assert node.have_identified_params is False

        node.tire_forces_callback(_make_tire_msg(0, fy=1.0))
        assert node.metrics['fl_fy'].metrics()['n_samples'] == 0

        response = node.handle_identified_params(
            _make_identified_param_request([6.63, 1.1052, 0.4316, 0.5193, 7.8594, 1.5468, 0.3589, 0.5631]),
            IdentifiedParam.Response(),
        )
        assert response.ack is True
        assert node.have_identified_params is True
        assert node.c_pf == pytest.approx([6.63, 1.1052, 0.4316, 0.5193])
        assert node.c_pr == pytest.approx([7.8594, 1.5468, 0.3589, 0.5631])

        node.tire_forces_callback(_make_tire_msg(1, fy=1.0))
        assert node.metrics['fl_fy'].metrics()['n_samples'] == 1


def test_identified_params_service_rejects_wrong_length():
    # IdentifiedParam.param_values is a fixed-size float32[8] field - the
    # real generated type refuses to even construct a request with a
    # different length, so the defensive length check in
    # handle_identified_params (mirroring the same pattern already in
    # adaptive_controller_manager's onSysidUpdateParams) is exercised here
    # via a duck-typed stand-in rather than a real IdentifiedParam.Request.
    class _BadRequest:
        param_values = [1.0, 2.0, 3.0, 4.0]

    with NodeUnderTest({'min_fz_threshold': 1.0, 'enable_state_benchmarking': False}) as node:
        response = node.handle_identified_params(_BadRequest(), IdentifiedParam.Response())
        assert response.ack is False
        assert node.have_identified_params is False
