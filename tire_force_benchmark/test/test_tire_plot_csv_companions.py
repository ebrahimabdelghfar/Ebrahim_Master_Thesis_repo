"""Every exported figure must ship a same-named CSV holding its numbers.

benchmark_runner/compare_scenarios.py reads those CSVs rather than re-deriving
any metric, so a figure without one silently drops out of the cross-scenario
comparison instead of failing loudly.
"""
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from test_queue_alignment import NodeUnderTest  # noqa: E402
from test_queue_alignment import _make_odom_msg  # noqa: E402
from test_queue_alignment import _make_tire_msg  # noqa: E402

VEHICLE = {
    'm': 269.6,
    'I_z': 51.1,
    'l_f': 0.738142,
    'l_r': 0.795362,
    'l_wb': 1.533504,
}


def _drive(node, samples=40):
    """Feed enough force and odometry samples to populate every history buffer.

    Odometry steps by 20 ms: the state predictor drops any gap over 0.2 s as
    untrustworthy timing, so one-second stamps would leave the vehicle-state
    figures empty.
    """
    for i in range(samples):
        node.tire_forces_callback(
            _make_tire_msg(i, fy=120.0 + 3.0 * i, fz=700.0, slip_angle=0.01 + 0.0005 * i))
        node.ackermann_callback(_make_drive_msg(0.05))
        node.odom_callback(
            _make_odom_msg(0, int(i * 20e6), v_x=8.0, v_y=0.2 + 0.001 * i, omega=0.15))


def _export_once(node):
    node._export_plots()
    # destroy_node() would export a second identical set on the way out.
    node.plot_output_dir = ''


def _make_drive_msg(steering):
    from ackermann_msgs.msg import AckermannDriveStamped
    msg = AckermannDriveStamped()
    msg.drive.steering_angle = steering
    return msg


def _read(path):
    with Path(path).open(newline='', encoding='utf-8') as handle:
        return list(csv.reader(handle))


def test_every_exported_png_has_a_csv_companion(tmp_path):
    overrides = dict(VEHICLE)
    overrides.update({
        'min_fz_threshold': 1.0,
        'plot_output_dir': str(tmp_path),
        'c_pf': [6.63, 1.1052, 1.05, 0.5193],
        'c_pr': [7.8594, 1.5468, 1.02, 0.5631],
    })
    with NodeUnderTest(overrides) as node:
        _drive(node)
        _export_once(node)

    pngs = sorted(p.name for p in tmp_path.glob('*.png'))
    assert pngs, 'export produced no figures at all'
    for png in tmp_path.glob('*.png'):
        companion = png.with_suffix('.csv')
        assert companion.is_file(), f'{png.name} has no CSV companion'
        rows = _read(companion)
        assert len(rows) >= 2, f'{companion.name} has a header but no data'


def test_metrics_summary_csv_is_keyed_by_signal(tmp_path):
    """The comparison joins on signal_key, not on the display label."""
    overrides = dict(VEHICLE)
    overrides.update({
        'min_fz_threshold': 1.0,
        'plot_output_dir': str(tmp_path),
        'c_pf': [6.63, 1.1052, 1.05, 0.5193],
        'c_pr': [7.8594, 1.5468, 1.02, 0.5631],
    })
    with NodeUnderTest(overrides) as node:
        _drive(node)
        _export_once(node)

    rows = _read(tmp_path / 'metrics_summary.csv')
    assert rows[0][:2] == ['signal_key', 'Signal']
    keys = {row[0] for row in rows[1:]}
    for expected in ('front_sum_fy', 'rear_sum_fy', 'v_y', 'omega', 'front_mu', 'rear_mu'):
        assert expected in keys, f'{expected} missing from metrics_summary.csv'
