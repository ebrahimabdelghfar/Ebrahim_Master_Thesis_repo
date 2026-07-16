#!/usr/bin/env python3
"""Academic benchmark of adaptive_controller_manager's PP/MPC switching FSM.

Passively subscribes to topics adaptive_controller_manager/pure_pursuit/mpc_path_tracking
already publish - no changes to any of those packages. See docs/adaptive_controller_benchmark.md
for the metric definitions, the FSM state-name coupling, and citations.
"""
import csv
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile

from ackermann_msgs.msg import AckermannDriveStamped
from f1tenth_msgs.msg import WaypointArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_msgs.msg import String

from adaptive_controller_benchmark.online_metrics import HandoverHistoryBuffer
from adaptive_controller_benchmark.online_metrics import LapTracker
from adaptive_controller_benchmark.online_metrics import STATE_COLORS
from adaptive_controller_benchmark.online_metrics import SignalStats
from adaptive_controller_benchmark.online_metrics import SwitchTracker

# Must match adaptive_controller_manager/src/manager_node.cpp::fsmStateName() literally -
# see online_metrics.py's module docstring for why this is a string coupling, not an enum.
STATE_ORDER = [
    'BOOTSTRAP_PP', 'RUNNING_PP', 'SWITCHING_TO_MPC', 'RUNNING_MPC',
    'SWITCHING_TO_PP', 'EMERGENCY_HALT',
]
PP_ACTIVE_STATE = 'RUNNING_PP'
MPC_ACTIVE_STATE = 'RUNNING_MPC'
EMERGENCY_STATE = 'EMERGENCY_HALT'


class AdaptiveControllerBenchmarkNode(Node):
    def __init__(self):
        super().__init__('adaptive_controller_benchmark_node')

        self.declare_parameter('manager_state_topic', 'manager/state')
        self.declare_parameter('lateral_error_topic', 'manager/debug/lateral_error')
        self.declare_parameter('heading_error_topic', 'manager/debug/heading_error')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('pp_drive_topic', 'pp/drive_cmd')
        self.declare_parameter('mpc_drive_topic', 'mpc/drive_cmd')
        self.declare_parameter('waypoint_topic', '/raceline_waypoints')
        self.declare_parameter('mpc_solve_time_topic', '/mpc/debug/solve_time_ms')
        self.declare_parameter('log_interval', 200)
        self.declare_parameter('csv_output_path', '')
        self.declare_parameter('plot_output_dir', '')
        self.declare_parameter('plot_max_points', 5000)

        manager_state_topic = str(self.get_parameter('manager_state_topic').value)
        lateral_error_topic = str(self.get_parameter('lateral_error_topic').value)
        heading_error_topic = str(self.get_parameter('heading_error_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        drive_topic = str(self.get_parameter('drive_topic').value)
        pp_drive_topic = str(self.get_parameter('pp_drive_topic').value)
        mpc_drive_topic = str(self.get_parameter('mpc_drive_topic').value)
        waypoint_topic = str(self.get_parameter('waypoint_topic').value)
        mpc_solve_time_topic = str(self.get_parameter('mpc_solve_time_topic').value)
        self.log_interval = max(1, int(self.get_parameter('log_interval').value))
        self.csv_output_path = str(self.get_parameter('csv_output_path').value)
        self.plot_output_dir = str(self.get_parameter('plot_output_dir').value)
        plot_max_points = int(self.get_parameter('plot_max_points').value)

        # Latest cached values from independent per-topic callbacks. manager/state is
        # published every control-loop tick regardless of track-error availability (see
        # manager_node.cpp::controlLoop() -> publishDebugTopics()), so it drives per-tick
        # processing here; everything else is a "latest known value" snapshot taken at
        # that instant - the same cross-topic aggregation idiom
        # tire_force_benchmark_node.py/estimation_benchmark_node.py already use.
        self.has_e_y = False
        self.e_y = 0.0
        self.heading_error = 0.0

        self.has_odom = False
        self.v_x = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0

        self.has_drive = False
        self.drive_speed = 0.0
        self.drive_steering = 0.0

        self.pp_steering = 0.0
        self.pp_speed = 0.0
        self.mpc_steering = 0.0
        self.mpc_speed = 0.0

        self.has_solve_time = False
        self.solve_time_ms = 0.0

        self.run_start_t = None
        self.last_tick_t = None

        self.overall_e_y = SignalStats('e_y (overall)')
        self.overall_heading = SignalStats('heading_error (overall)')
        self.overall_v_x = SignalStats('v_x (overall)')
        self.pp_e_y = SignalStats('e_y (RUNNING_PP)')
        self.pp_heading = SignalStats('heading_error (RUNNING_PP)')
        self.mpc_e_y = SignalStats('e_y (RUNNING_MPC)')
        self.mpc_heading = SignalStats('heading_error (RUNNING_MPC)')
        self.mpc_solve_time = SignalStats('mpc solve_time_ms')

        self.switch_tracker = SwitchTracker()
        self.lap_tracker = LapTracker()
        self.history = HandoverHistoryBuffer(plot_max_points)

        self.emergency_halt_count = 0
        self.emergency_halt_total_s = 0.0
        self._emergency_entered_at = None

        self.sample_count = 0

        waypoint_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped, drive_topic, self._drive_cb, 10)
        self.pp_drive_sub = self.create_subscription(
            AckermannDriveStamped, pp_drive_topic, self._pp_drive_cb, 10)
        self.mpc_drive_sub = self.create_subscription(
            AckermannDriveStamped, mpc_drive_topic, self._mpc_drive_cb, 10)
        self.lateral_error_sub = self.create_subscription(
            Float64, lateral_error_topic, self._lateral_error_cb, 10)
        self.heading_error_sub = self.create_subscription(
            Float64, heading_error_topic, self._heading_error_cb, 10)
        self.solve_time_sub = self.create_subscription(
            Float64, mpc_solve_time_topic, self._solve_time_cb, 10)
        self.waypoint_sub = self.create_subscription(
            WaypointArray, waypoint_topic, self._waypoint_cb, waypoint_qos)
        # manager/state drives per-tick processing - subscribed last so every cached
        # value above is already wired up before the first tick can fire.
        self.state_sub = self.create_subscription(
            String, manager_state_topic, self._state_cb, 10)

        self.csv_file = None
        self.csv_writer = None
        self._setup_csv_if_enabled()

        self.get_logger().info('adaptive_controller_benchmark_node started')
        self.get_logger().info(f'Driving tick topic: {manager_state_topic}')
        if self.plot_output_dir:
            self.get_logger().info(f'Plot export enabled -> {self.plot_output_dir} (on shutdown)')
        else:
            self.get_logger().info('plot_output_dir not set - plot export disabled')
        if self.csv_output_path:
            self.get_logger().info(f'CSV logging enabled -> {self.csv_output_path}')

    # ---------------- Cached-value callbacks ----------------

    def _odom_cb(self, msg: Odometry):
        self.has_odom = True
        self.v_x = msg.twist.twist.linear.x
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y

    def _drive_cb(self, msg: AckermannDriveStamped):
        self.has_drive = True
        self.drive_speed = float(msg.drive.speed)
        self.drive_steering = float(msg.drive.steering_angle)

    def _pp_drive_cb(self, msg: AckermannDriveStamped):
        self.pp_steering = float(msg.drive.steering_angle)
        self.pp_speed = float(msg.drive.speed)

    def _mpc_drive_cb(self, msg: AckermannDriveStamped):
        self.mpc_steering = float(msg.drive.steering_angle)
        self.mpc_speed = float(msg.drive.speed)

    def _lateral_error_cb(self, msg: Float64):
        self.has_e_y = True
        self.e_y = float(msg.data)

    def _heading_error_cb(self, msg: Float64):
        self.heading_error = float(msg.data)

    def _solve_time_cb(self, msg: Float64):
        self.has_solve_time = True
        self.solve_time_ms = float(msg.data)

    def _waypoint_cb(self, msg: WaypointArray):
        if not msg.waypoints:
            return
        x = [wp.x_m for wp in msg.waypoints]
        y = [wp.y_m for wp in msg.waypoints]
        s = [wp.s_m for wp in msg.waypoints]
        self.lap_tracker.set_waypoints(x, y, s)

    # ---------------- Per-tick processing (driven by manager/state) ----------------

    def _state_cb(self, msg: String):
        state = str(msg.data)
        t = self.get_clock().now().nanoseconds * 1e-9

        if self.run_start_t is None:
            self.run_start_t = t
            self.last_tick_t = t

        dt = t - self.last_tick_t
        t_run = t - self.run_start_t

        if state == EMERGENCY_STATE:
            if self._emergency_entered_at is None:
                self._emergency_entered_at = t
                self.emergency_halt_count += 1
        elif self._emergency_entered_at is not None:
            self.emergency_halt_total_s += t - self._emergency_entered_at
            self._emergency_entered_at = None

        if self.has_e_y:
            self.overall_e_y.update(self.e_y, dt, t_run)
            self.overall_heading.update(self.heading_error, dt, t_run)
            if state == PP_ACTIVE_STATE:
                self.pp_e_y.update(self.e_y, dt, t_run)
                self.pp_heading.update(self.heading_error, dt, t_run)
            elif state == MPC_ACTIVE_STATE:
                self.mpc_e_y.update(self.e_y, dt, t_run)
                self.mpc_heading.update(self.heading_error, dt, t_run)

        if self.has_odom:
            self.overall_v_x.update(self.v_x, dt, t_run)
            if self.lap_tracker.ready():
                self.lap_tracker.update(t, self.pos_x, self.pos_y)

        if self.has_solve_time:
            self.mpc_solve_time.update(self.solve_time_ms, dt, t_run)

        v_cmd = self.drive_speed if self.has_drive else 0.0
        steering_cmd = self.drive_steering if self.has_drive else 0.0
        self.switch_tracker.update(
            t_run, state, v_cmd, steering_cmd,
            extra={
                'pp_steering': self.pp_steering, 'mpc_steering': self.mpc_steering,
                'pp_speed': self.pp_speed, 'mpc_speed': self.mpc_speed,
            })

        self.history.add(
            t_run, state, self.e_y, self.heading_error, self.v_x, steering_cmd, v_cmd,
            self.solve_time_ms if self.has_solve_time else 0.0)

        if self.csv_writer is not None:
            self.csv_writer.writerow([
                f'{t_run:.4f}', state, f'{self.e_y:.5f}', f'{self.heading_error:.5f}',
                f'{self.v_x:.4f}', f'{steering_cmd:.5f}', f'{v_cmd:.4f}',
                f'{self.solve_time_ms:.4f}' if self.has_solve_time else '',
            ])

        self.last_tick_t = t
        self.sample_count += 1
        if self.sample_count % self.log_interval == 0:
            self._log_summary(t_run)

    # ---------------- Summary / CSV ----------------

    def _log_summary(self, t_run):
        e_y_m = self.overall_e_y.metrics()
        heading_m = self.overall_heading.metrics()
        sw = self.switch_tracker.summary()
        self.get_logger().info(
            f'[t={t_run:6.1f}s] N={e_y_m["n"]} RMS(e_y)={e_y_m["rms"]:.4f}m '
            f'Max|e_y|={e_y_m["max_abs"]:.4f}m RMS(heading)={heading_m["rms"]:.4f}rad '
            f'switches={sw["n_episodes"]} emergency_halts={self.emergency_halt_count} '
            f'laps={len(self.lap_tracker.lap_times)}')

    def _setup_csv_if_enabled(self):
        if self.csv_output_path == '':
            return
        output_path = Path(self.csv_output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = output_path.open('w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            't_run_s', 'state', 'e_y_m', 'heading_error_rad', 'v_x_mps',
            'steering_cmd_rad', 'speed_cmd_mps', 'mpc_solve_time_ms',
        ])
        self.get_logger().info(f'CSV logging enabled: {self.csv_output_path}')

    # ---------------- Plot export (fires on shutdown - see destroy_node()) ----------------

    def _export_plots(self):
        if not self.plot_output_dir:
            return
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().warn('matplotlib not available - skipping benchmark plot export.')
            return

        if self.run_start_t is None:
            self.get_logger().warn('No ticks were ever received - skipping benchmark plot export.')
            return

        if self._emergency_entered_at is not None:
            self.emergency_halt_total_s += self.last_tick_t - self._emergency_entered_at
            self._emergency_entered_at = None
        self.switch_tracker.finalize(self.last_tick_t - self.run_start_t)

        out_dir = Path(self.plot_output_dir)
        out_dir.mkdir(parents=True, exist_ok=True)

        self._plot_state_timeline(plt, out_dir / 'fsm_state_timeline.png')
        self._plot_tracking_error_timeseries(plt, out_dir / 'tracking_error_timeseries.png')
        self._plot_error_boxplot_by_controller(
            plt, out_dir / 'tracking_error_boxplot_by_controller.png')
        self._plot_speed_and_compute_cost(plt, out_dir / 'speed_and_compute_cost.png')
        self._plot_handover_transient(plt, out_dir / 'handover_transient.png')
        self._plot_lap_times(plt, out_dir / 'lap_times.png')
        self._plot_metrics_summary_table(plt, out_dir / 'metrics_summary_table.png')

        self.get_logger().info(f'Benchmark plots saved to: {out_dir}')

    def _shade_states(self, ax):
        """axvspan background per contiguous FSM-state run, colored per STATE_COLORS -
        ties tracking-error/speed behavior visually to which controller was active.
        Returns the distinct states shaded, in STATE_ORDER, for legend building."""
        h = self.history
        if not h.t:
            return []
        start_idx = 0
        n = len(h.t)
        seen = set()
        for i in range(1, n + 1):
            if i == n or h.state[i] != h.state[start_idx]:
                end_t = h.t[i - 1] if i - 1 > start_idx else h.t[start_idx] + 1e-3
                ax.axvspan(
                    h.t[start_idx], end_t,
                    color=STATE_COLORS.get(h.state[start_idx], '#cccccc'),
                    alpha=0.15, linewidth=0)
                seen.add(h.state[start_idx])
                start_idx = i
        return [s for s in STATE_ORDER if s in seen]

    def _add_state_legend(self, ax, states):
        """Legend mapping shaded background colors to FSM states (PP/MPC/switching/emergency zones)."""
        if not states:
            return
        import matplotlib.patches as mpatches
        handles = [
            mpatches.Patch(color=STATE_COLORS.get(s, '#cccccc'), alpha=0.3, label=s)
            for s in states
        ]
        ax.legend(handles=handles, loc='upper right', fontsize=6, framealpha=0.8, ncol=len(handles))

    def _plot_state_timeline(self, plt, save_path):
        h = self.history
        fig, ax = plt.subplots(figsize=(10, 2.6))
        if not h.t:
            ax.set_title('FSM state timeline (no data)')
            ax.axis('off')
        else:
            y_of = {s: i for i, s in enumerate(STATE_ORDER)}
            ys = [y_of.get(s, -1) for s in h.state]
            colors = [STATE_COLORS.get(s, '#cccccc') for s in h.state]
            ax.scatter(h.t, ys, c=colors, s=6, marker='s')
            ax.set_yticks(list(y_of.values()))
            ax.set_yticklabels(STATE_ORDER, fontsize=8)
            ax.set_xlabel('Time [s]')
            ax.set_title('Active FSM state over time')
            ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(save_path, dpi=150)
        plt.close(fig)

    def _plot_tracking_error_timeseries(self, plt, save_path):
        h = self.history
        fig, axes = plt.subplots(2, 1, figsize=(10, 5), sharex=True)
        if not h.t:
            for ax in axes:
                ax.set_title('(no data)')
                ax.axis('off')
        else:
            states_seen = self._shade_states(axes[0])
            self._shade_states(axes[1])
            axes[0].plot(h.t, h.e_y, color='tab:blue', linewidth=1.0)
            axes[0].set_ylabel('e_y [m]')
            axes[0].set_title('Lateral tracking error (background shaded by active FSM state)')
            axes[0].grid(True, alpha=0.3)
            self._add_state_legend(axes[0], states_seen)
            axes[1].plot(h.t, h.heading_error, color='tab:red', linewidth=1.0)
            axes[1].set_ylabel('heading error [rad]')
            axes[1].set_xlabel('Time [s]')
            axes[1].set_title('Heading tracking error')
            axes[1].grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(save_path, dpi=150)
        plt.close(fig)

    def _plot_error_boxplot_by_controller(self, plt, save_path):
        h = self.history
        pp_e_y = [abs(e) for e, s in zip(h.e_y, h.state) if s == PP_ACTIVE_STATE]
        mpc_e_y = [abs(e) for e, s in zip(h.e_y, h.state) if s == MPC_ACTIVE_STATE]
        pp_heading = [abs(e) for e, s in zip(h.heading_error, h.state) if s == PP_ACTIVE_STATE]
        mpc_heading = [abs(e) for e, s in zip(h.heading_error, h.state) if s == MPC_ACTIVE_STATE]

        fig, axes = plt.subplots(1, 2, figsize=(9, 4))
        for ax, pp_vals, mpc_vals, label, unit in (
            (axes[0], pp_e_y, mpc_e_y, '|e_y|', 'm'),
            (axes[1], pp_heading, mpc_heading, '|heading error|', 'rad'),
        ):
            data = [v for v in (pp_vals, mpc_vals) if v]
            labels = [n for n, v in ((PP_ACTIVE_STATE, pp_vals), (MPC_ACTIVE_STATE, mpc_vals)) if v]
            if not data:
                ax.set_title(f'{label} (no data)')
                ax.axis('off')
                continue
            ax.boxplot(data, labels=labels)
            ax.set_ylabel(f'{label} [{unit}]')
            ax.grid(True, alpha=0.3)
        fig.suptitle('Tracking error by active controller')
        fig.tight_layout()
        fig.savefig(save_path, dpi=150)
        plt.close(fig)

    def _plot_speed_and_compute_cost(self, plt, save_path):
        h = self.history
        fig, ax1 = plt.subplots(figsize=(10, 4))
        if not h.t:
            ax1.set_title('(no data)')
            ax1.axis('off')
        else:
            states_seen = self._shade_states(ax1)
            ax1.plot(h.t, h.v_x, color='tab:green', linewidth=1.0)
            ax1.set_xlabel('Time [s]')
            ax1.set_ylabel('v_x [m/s]', color='tab:green')
            ax1.tick_params(axis='y', labelcolor='tab:green')
            ax1.grid(True, alpha=0.3)
            self._add_state_legend(ax1, states_seen)

            ax2 = ax1.twinx()
            ax2.plot(h.t, h.solve_time_ms, color='tab:purple', linewidth=0.8, alpha=0.8)
            ax2.set_ylabel('MPC solve_time_ms [ms]', color='tab:purple')
            ax2.tick_params(axis='y', labelcolor='tab:purple')
        fig.suptitle('Speed profile vs. MPC compute cost')
        fig.tight_layout()
        fig.savefig(save_path, dpi=150)
        plt.close(fig)

    def _plot_handover_transient(self, plt, save_path):
        episodes = self.switch_tracker.episodes
        fig, axes = plt.subplots(1, 2, figsize=(11, 4))
        if not episodes:
            for ax in axes:
                ax.set_title('(no switching episodes recorded)')
                ax.axis('off')
            fig.tight_layout()
            fig.savefig(save_path, dpi=150)
            plt.close(fig)
            return

        for idx, ep in enumerate(episodes):
            ts = [s[0] for s in ep['samples']]
            v_cmds = [s[1] for s in ep['samples']]
            style = '-' if ep['to_state'] == 'SWITCHING_TO_MPC' else '--'
            axes[0].plot(
                ts, v_cmds, style, linewidth=1.0, alpha=0.8,
                label=f"#{idx} -> {ep['to_state']}" if idx < 6 else None)
        axes[0].set_xlabel('Time since switch start [s]')
        axes[0].set_ylabel('Arbitrated v_cmd [m/s]')
        axes[0].set_title('Handover speed ramp per switching episode')
        axes[0].grid(True, alpha=0.3)
        if len(episodes) <= 6:
            axes[0].legend(fontsize=7, loc='best')

        jumps = [abs(ep['entry_steering'] - ep['exit_steering']) for ep in episodes]
        colors = [STATE_COLORS.get(ep['to_state'], '#cccccc') for ep in episodes]
        axes[1].bar(range(len(episodes)), jumps, color=colors)
        axes[1].set_xlabel('Switching episode index')
        axes[1].set_ylabel('|steering jump| [rad]')
        axes[1].set_title('Steering discontinuity at switch instant')
        axes[1].grid(True, alpha=0.3)

        fig.tight_layout()
        fig.savefig(save_path, dpi=150)
        plt.close(fig)

    def _plot_lap_times(self, plt, save_path):
        laps = self.lap_tracker.lap_times
        if not laps:
            self.get_logger().info('No completed laps observed - skipping lap_times.png.')
            return
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.bar(range(1, len(laps) + 1), laps, color='tab:blue')
        ax.set_xlabel('Lap number')
        ax.set_ylabel('Lap time [s]')
        ax.set_title('Lap-wise lap time')
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(save_path, dpi=150)
        plt.close(fig)

    def _plot_metrics_summary_table(self, plt, save_path):
        rows = []
        for label, e_y_stats, heading_stats in (
            ('Overall', self.overall_e_y, self.overall_heading),
            (PP_ACTIVE_STATE, self.pp_e_y, self.pp_heading),
            (MPC_ACTIVE_STATE, self.mpc_e_y, self.mpc_heading),
        ):
            e = e_y_stats.metrics()
            h = heading_stats.metrics()
            if e['n'] == 0:
                rows.append([label, '0', '-', '-', '-', '-', '-'])
                continue
            rows.append([
                label, str(e['n']), f"{e['rms']:.4f}", f"{e['max_abs']:.4f}",
                f"{e['itae']:.2f}", f"{h['rms']:.4f}", f"{h['max_abs']:.4f}",
            ])

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 4.5))
        ax1.axis('off')
        table1 = ax1.table(
            cellText=rows,
            colLabels=['Controller', 'N', 'RMS e_y [m]', 'Max|e_y| [m]', 'ITAE e_y',
                       'RMS heading [rad]', 'Max|heading| [rad]'],
            loc='center', cellLoc='center')
        table1.auto_set_font_size(False)
        table1.set_fontsize(9)
        table1.auto_set_column_width(col=list(range(7)))
        table1.scale(1, 1.4)
        ax1.set_title('Tracking-error metrics by active controller', fontsize=11)

        sw = self.switch_tracker.summary()
        dwell_rows = [
            [state, f"{d['n']}", f"{d['mean']:.2f}", f"{d['max']:.2f}"]
            for state, d in sw['dwell'].items()
        ]
        laps = self.lap_tracker.lap_times
        lap_text = (
            f"laps={len(laps)} best={min(laps):.2f}s mean={sum(laps)/len(laps):.2f}s"
            if laps else 'laps=0'
        )
        solve = self.mpc_solve_time.metrics()
        footer = (
            f"transitions={sw['transitions']} | emergency_halts={self.emergency_halt_count} "
            f"(total {self.emergency_halt_total_s:.2f}s) | {lap_text} | "
            f"mpc_solve_time_ms: rms/max = "
            f"{(solve['rms'] if solve['n'] else 0.0):.2f}/{solve['max_abs']:.2f}"
        )

        ax2.axis('off')
        if dwell_rows:
            table2 = ax2.table(
                cellText=dwell_rows,
                colLabels=['State', 'N dwell intervals', 'Mean dwell [s]', 'Max dwell [s]'],
                loc='upper center', cellLoc='center')
            table2.auto_set_font_size(False)
            table2.set_fontsize(9)
            table2.auto_set_column_width(col=list(range(4)))
            table2.scale(1, 1.4)
        ax2.text(0.5, 0.02, footer, ha='center', va='bottom', fontsize=8, wrap=True,
                  transform=ax2.transAxes)
        ax2.set_title('Switching / dwell-time / lap / compute-cost summary', fontsize=11)

        fig.tight_layout()
        fig.savefig(save_path, dpi=150)
        plt.close(fig)

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
        self._export_plots()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveControllerBenchmarkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
