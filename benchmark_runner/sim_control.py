"""Lifecycle control of the CARLA bridge, plus topic-readiness waits.

The bridge (`carla_telemetry_cpp`, node name `ASU_RT_Carla_Telemetry_Node`) is a
`rclcpp_lifecycle::LifecycleNode`. Its `on_cleanup` sets `has_cleaned_up_`, so a
second `configure` reuses the running CARLA world instead of reloading the map -
which is what makes configure/activate ... deactivate/cleanup the right
per-scenario reset rather than restarting the server between scenarios.

This module never starts or stops the CARLA server or the bridge process. The
user launches those by hand (`make launch_carla_sim AUTO_START=false` in
Carla_ASU_Bridge); if the lifecycle services are absent we say so and stop.
"""
import time

from lifecycle_msgs.msg import State
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.srv import GetState

BRIDGE_NODE = 'ASU_RT_Carla_Telemetry_Node'

TRANSITIONS = {
    'configure': Transition.TRANSITION_CONFIGURE,
    'activate': Transition.TRANSITION_ACTIVATE,
    'deactivate': Transition.TRANSITION_DEACTIVATE,
    'cleanup': Transition.TRANSITION_CLEANUP,
}

STATE_NAMES = {
    State.PRIMARY_STATE_UNKNOWN: 'unknown',
    State.PRIMARY_STATE_UNCONFIGURED: 'unconfigured',
    State.PRIMARY_STATE_INACTIVE: 'inactive',
    State.PRIMARY_STATE_ACTIVE: 'active',
    State.PRIMARY_STATE_FINALIZED: 'finalized',
}


class SimNotRunning(RuntimeError):
    pass


class BridgeLifecycle:
    """Thin ChangeState/GetState client pair for the bridge."""

    def __init__(self, node, bridge_node=BRIDGE_NODE):
        self.node = node
        self.bridge_node = bridge_node
        self._change = node.create_client(ChangeState, f'/{bridge_node}/change_state')
        self._get = node.create_client(GetState, f'/{bridge_node}/get_state')

    def require_running(self, timeout_s=10.0):
        """Fail loudly and usefully when the simulator was not started."""
        if not self._get.wait_for_service(timeout_sec=timeout_s):
            raise SimNotRunning(
                f'/{self.bridge_node}/get_state did not answer within {timeout_s:.0f}s. '
                'Start the simulator first, in its own terminal:\n'
                '  cd /home/ebrahim/Carla_ASU_Bridge && make launch_carla_sim AUTO_START=false\n'
                'The runner drives its lifecycle but never starts or kills it.')
        self._change.wait_for_service(timeout_sec=timeout_s)

    def state(self, timeout_s=10.0):
        future = self._get.call_async(GetState.Request())
        result = self._spin_until(future, timeout_s)
        if result is None:
            raise SimNotRunning(f'/{self.bridge_node}/get_state timed out')
        return result.current_state.id

    def state_name(self):
        return STATE_NAMES.get(self.state(), 'unknown')

    def transition(self, name, timeout_s=120.0):
        """Request one lifecycle transition. Returns True if it was taken.

        `on_configure` reloads the map and respawns the vehicle on the first
        call, which is why the default timeout is generous.
        """
        request = ChangeState.Request()
        request.transition.id = TRANSITIONS[name]
        future = self._change.call_async(request)
        result = self._spin_until(future, timeout_s)
        if result is None:
            raise SimNotRunning(f'{name} on /{self.bridge_node} timed out after {timeout_s:.0f}s')
        return bool(result.success)

    def ensure_unconfigured(self):
        """Walk the bridge back to `unconfigured` from wherever it currently is."""
        current = self.state()
        if current == State.PRIMARY_STATE_ACTIVE:
            self.transition('deactivate')
            current = self.state()
        if current == State.PRIMARY_STATE_INACTIVE:
            self.transition('cleanup')

    def _spin_until(self, future, timeout_s):
        # The caller keeps this node on an executor running in its own thread,
        # so the future is completed there - poll rather than spin, which would
        # re-enter the executor from a second thread.
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if time.monotonic() > deadline:
                return None
            time.sleep(0.02)
        return future.result()


def wait_for_publisher(node, topic, timeout_s, poll_s=0.5):
    """Block until at least one publisher exists on `topic`. False on timeout."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if node.count_publishers(topic) > 0:
            return True
        time.sleep(poll_s)
    return node.count_publishers(topic) > 0
