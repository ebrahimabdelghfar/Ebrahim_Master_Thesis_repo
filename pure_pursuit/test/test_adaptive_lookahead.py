#!/usr/bin/env python3
"""
Geometry tests for the adaptive lookahead and the lookahead-point search.

The node is imported with the ROS packages stubbed out so the pure geometry
can be exercised without a running graph; the methods under test only touch
self.waypoints/curvatures/seg_len and the pose, which are set directly.
"""
import importlib.util
import math
import os
import sys
import types

import numpy as np
import pytest

_STUBS = [
    'rclpy', 'rclpy.node', 'rclpy.qos', 'nav_msgs', 'nav_msgs.msg',
    'visualization_msgs', 'visualization_msgs.msg', 'std_msgs', 'std_msgs.msg',
    'rcl_interfaces', 'rcl_interfaces.msg', 'tf_compat', 'geometry_msgs',
    'geometry_msgs.msg', 'ackermann_msgs', 'ackermann_msgs.msg',
    'ament_index_python', 'ament_index_python.packages', 'f1tenth_msgs',
    'f1tenth_msgs.msg', 'matplotlib', 'matplotlib.pyplot',
]


def _load_node_class():
    for name in _STUBS:
        module = types.ModuleType(name)
        module.__getattr__ = lambda attr: type(attr, (), {})
        sys.modules[name] = module
    sys.modules['rclpy.node'].Node = type('Node', (), {})
    sys.modules['matplotlib'].patches = types.ModuleType('patches')

    path = os.path.join(os.path.dirname(__file__), '..', 'src',
                        'pure_pursuit_node.py')
    spec = importlib.util.spec_from_file_location('pure_pursuit_node', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.PurePursuitNode


PurePursuitNode = _load_node_class()

RADIUS = 20.0
CIRCLE = np.stack([
    RADIUS * np.cos(np.linspace(0, 2 * math.pi, 400, endpoint=False)),
    RADIUS * np.sin(np.linspace(0, 2 * math.pi, 400, endpoint=False)),
], axis=1)
STRAIGHT = np.stack([np.arange(0.0, 400.0, 0.5), np.zeros(800)], axis=1)


def make_node(xy, vel, **kwargs):
    node = object.__new__(PurePursuitNode)
    node.waypoints = np.hstack([xy, np.zeros((len(xy), 2))])
    node.seg_len = np.linalg.norm(np.roll(xy, -1, axis=0) - xy, axis=1)
    node.curvatures = PurePursuitNode.compute_curvature(xy)
    node.vel = vel
    node.adaptive_lookahead = kwargs.get('adaptive', True)
    node.lookahead_gain = kwargs.get('gain', 0.35)
    node.lookahead_min = kwargs.get('lmin', 2.5)
    node.lookahead_max = kwargs.get('lmax', 12.0)
    node.lookahead_curvature_gain = kwargs.get('kgain', 5.0)
    node.LOOKAHEAD = kwargs.get('fixed', 5.5)
    node.xc, node.yc = float(xy[0][0]), float(xy[0][1])
    return node


def test_curvature_of_a_circle_is_one_over_radius():
    kappa = PurePursuitNode.compute_curvature(CIRCLE)
    assert kappa == pytest.approx(1.0 / RADIUS, abs=1e-4)


def test_curvature_of_a_straight_is_zero():
    assert PurePursuitNode.compute_curvature(STRAIGHT).max() < 1e-9


def test_target_point_lies_on_the_lookahead_circle():
    node = make_node(CIRCLE, 15.0)
    target, idx = node.find_target_point(0, 5.0)
    distance = math.hypot(target[0] - node.xc, target[1] - node.yc)
    # The crossing is interpolated linearly along the chord, so it is exact on
    # a straight and first-order on a curve.
    assert distance == pytest.approx(5.0, abs=1e-4)
    assert 0 < idx < 40


def test_search_wraps_past_the_last_waypoint():
    node = make_node(CIRCLE, 15.0)
    node.xc, node.yc = float(CIRCLE[-1][0]), float(CIRCLE[-1][1])
    target, idx = node.find_target_point(len(CIRCLE) - 1, 5.0)
    distance = math.hypot(target[0] - node.xc, target[1] - node.yc)
    assert distance == pytest.approx(5.0, abs=1e-4)
    assert idx < 40


def test_target_from_the_first_waypoint_is_the_one_ahead():
    # The previous search returned the *last* waypoint whenever it settled on
    # index 0, aiming the car backwards down the raceline.
    node = make_node(STRAIGHT, 15.0)
    _, idx = node.find_target_point(0, 5.0)
    assert idx == 10


def test_car_farther_off_the_path_than_the_lookahead_aims_at_the_nearest_point():
    node = make_node(CIRCLE, 15.0)
    node.xc, node.yc = 0.0, 0.0  # circle centre, RADIUS from every waypoint
    target, idx = node.find_target_point(0, 5.0)
    assert idx == 0
    assert target == pytest.approx(CIRCLE[0])


def test_search_terminates_when_the_path_fits_inside_the_lookahead():
    tiny = np.array([[0.0, 0.0], [0.5, 0.0], [0.5, 0.5], [0.0, 0.5]])
    node = make_node(tiny, 15.0)
    _, idx = node.find_target_point(0, 50.0)
    assert idx < len(tiny)


def test_lookahead_scales_with_speed_and_shortens_in_corners():
    node = make_node(CIRCLE, 15.0)
    assert node.compute_lookahead(0) == pytest.approx(
        (0.35 * 15.0) / (1.0 + 5.0 * (1.0 / RADIUS)))


def test_lookahead_is_unshortened_on_a_straight():
    assert make_node(STRAIGHT, 15.0).compute_lookahead(0) == pytest.approx(5.25)


def test_zero_curvature_gain_disables_the_corner_shortening():
    assert make_node(CIRCLE, 15.0, kgain=0.0).compute_lookahead(0) == \
        pytest.approx(5.25)


def test_lookahead_is_clamped_at_both_ends():
    assert make_node(CIRCLE, 0.0).compute_lookahead(0) == 2.5
    assert make_node(STRAIGHT, 100.0).compute_lookahead(0) == 12.0


def test_adaptive_disabled_falls_back_to_the_fixed_distance():
    assert make_node(CIRCLE, 15.0, adaptive=False).compute_lookahead(0) == 5.5
