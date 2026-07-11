#!/usr/bin/env bash
# Full-stack simulation integration test for adaptive_controller_manager.
#
# Exercises, against the real f1tenth_simulator on YasMarina:
#   1. Baseline PP driving + forced handover to MPC (arming gates pass naturally)
#   2. Injected MPC failure mid-RUNNING_MPC -> fallback to PP
#   3. Both controllers unhealthy -> EMERGENCY_HALT -> recovery
#
# Every scenario starts and ends with a full process cleanup, verified before
# the next scenario begins. Not wired into colcon test - needs a live
# simulator and wall-clock waits colcon's test framework isn't built for.
#
# Usage: ./sim_integration_test.sh   (run from anywhere; paths are absolute)

REPO_ROOT="/home/ebrahim/Ebrahim_Master_Thesis_repo"
LOG_DIR="/tmp/adaptive_controller_manager_integration_test"
mkdir -p "$LOG_DIR"

PROC_PATTERN="adaptive_controller_manager_node|mpc_path_tracking/mpc_node|pure_pursuit/pure_pursuit_node.py|on_track_sys_id.py|f1tenth_simulator/simulator|f1tenth_simulator/track_publisher.py|map_server|lifecycle_manager|robot_state_publisher"

FAIL_COUNT=0
PASS_COUNT=0

log() { echo "[$(date '+%H:%M:%S')] $*"; }
pass() { PASS_COUNT=$((PASS_COUNT + 1)); log "PASS: $*"; }
fail() { FAIL_COUNT=$((FAIL_COUNT + 1)); log "FAIL: $*"; }

cleanup() {
  log "cleanup: sending SIGINT to any ros2 launch processes..."
  pkill -INT -f "ros2 launch" 2>/dev/null
  sleep 2
  log "cleanup: force-killing any remaining stack processes..."
  pkill -9 -f "$PROC_PATTERN" 2>/dev/null
  sleep 1
  if pgrep -f "$PROC_PATTERN" > /dev/null 2>&1; then
    log "CLEANUP FAILED - processes still alive:"
    pgrep -af "$PROC_PATTERN"
    exit 1
  fi
  log "cleanup: confirmed clean."
}

trap cleanup EXIT

# ROS2's own setup.bash scripts reference unset variables internally, so
# source them BEFORE turning on -u for the rest of this script's own logic.
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
export MPLBACKEND=Agg
THIRDPARTY_INSTALL_ROOT="$REPO_ROOT/install/thirdparty_lib"
export LD_LIBRARY_PATH="$THIRDPARTY_INSTALL_ROOT/osqp-0.6.3/lib:$THIRDPARTY_INSTALL_ROOT/osqp-eigen-0.8.1/lib:$THIRDPARTY_INSTALL_ROOT/casadi-3.7.0/lib:$THIRDPARTY_INSTALL_ROOT/acados-0.5.5/lib:$THIRDPARTY_INSTALL_ROOT/Ipopt-stable-3.14/lib:${LD_LIBRARY_PATH:-}"

set -u

# Prints the last value seen on a topic, or empty string if none within $2 seconds.
read_topic_once() {
  timeout "${2:-2}" ros2 topic echo "$1" --once 2>/dev/null
}

manager_state() {
  read_topic_once /manager/state 2 | grep "^data:" | sed "s/data: //"
}

# wait_for_state TARGET TIMEOUT_S
wait_for_state() {
  local target="$1" timeout_s="$2" start now current
  start=$(date +%s)
  while true; do
    current=$(manager_state)
    if [ "$current" = "$target" ]; then
      log "reached state=$target after $(( $(date +%s) - start ))s"
      return 0
    fi
    now=$(date +%s)
    if [ $(( now - start )) -ge "$timeout_s" ]; then
      log "TIMEOUT after ${timeout_s}s waiting for state=$target (last seen: '$current')"
      log "diagnostic: last e_y=$(read_topic_once /manager/debug/lateral_error 1 | tr -d '\n'), heading_error=$(read_topic_once /manager/debug/heading_error 1 | tr -d '\n')"
      return 1
    fi
    sleep 1
  done
}

launch_full_stack() {
  local stack_log="$LOG_DIR/stack_$1.log"
  log "launching sim_test.launch.py (map_name:=YasMarina), logging to $stack_log..."
  nohup ros2 launch adaptive_controller_manager sim_test.launch.py map_name:=YasMarina \
    > "$stack_log" 2>&1 &
  disown
  if ! wait_for_state "BOOTSTRAP_PP" 20 && ! wait_for_state "RUNNING_PP" 5; then
    log "stack did not come up - tail of $stack_log:"
    tail -60 "$stack_log"
    return 1
  fi
  return 0
}

# check_no_nan FILE LABEL
check_no_nan() {
  if grep -qi "nan\|inf" "$1" 2>/dev/null; then
    fail "$2: NaN/Inf found in $1"
    return 1
  fi
  pass "$2: no NaN/Inf"
  return 0
}

# check_bounded FILE LABEL MAX_ABS
check_bounded() {
  local max_abs_seen
  max_abs_seen=$(awk '{v=$1; if (v<0) v=-v; if (v>m) m=v} END{print m+0}' "$1")
  if awk -v v="$max_abs_seen" -v max="$3" 'BEGIN{exit !(v>max)}'; then
    fail "$2: max |value|=$max_abs_seen exceeds bound $3"
    return 1
  fi
  pass "$2: max |value|=$max_abs_seen within bound $3"
  return 0
}

log "===== SCENARIO 1+2: baseline + handover + MPC-failure fallback ====="
cleanup
if launch_full_stack "s1s2"; then
  wait_for_state "RUNNING_PP" 15 && pass "Scenario1: reached RUNNING_PP" || fail "Scenario1: did not reach RUNNING_PP"

  log "Scenario1: logging ~15s of PP-only driving..."
  timeout 15 ros2 topic echo /drive --field drive.speed > "$LOG_DIR/s1_pp_speed.log" 2>&1
  timeout 1 ros2 topic echo /odom --field pose.pose.position.x > "$LOG_DIR/s1_pp_x.log" 2>&1
  check_no_nan "$LOG_DIR/s1_pp_speed.log" "Scenario1 PP /drive.speed"
  check_bounded "$LOG_DIR/s1_pp_speed.log" "Scenario1 PP /drive.speed" 30.0

  log "Scenario1: injecting validated SIM Pacejka tire params via sysid/update_params..."
  ros2 service call /sysid/update_params adaptive_controller_interfaces/srv/IdentifiedParam \
    "{param_values: [2.4128, 4.8155, 0.5922, 5.0, 14.4445, 1.2129, 0.6842, 0.8526]}" \
    > "$LOG_DIR/s1_service_call.log" 2>&1
  cat "$LOG_DIR/s1_service_call.log"

  if wait_for_state "SWITCHING_TO_MPC" 45 || wait_for_state "RUNNING_MPC" 5; then
    pass "Scenario1: armed and began switch to MPC"

    log "Scenario1: logging /drive during the switch window..."
    timeout 3 ros2 topic echo /drive --field drive.steering_angle > "$LOG_DIR/s1_switch_steer.log" 2>&1
    timeout 3 ros2 topic echo /drive --field drive.speed > "$LOG_DIR/s1_switch_speed.log" 2>&1
    check_no_nan "$LOG_DIR/s1_switch_steer.log" "Scenario1 switch steering"

    if wait_for_state "RUNNING_MPC" 10; then
      pass "Scenario1: reached RUNNING_MPC"

      log "Scenario1: logging ~15s of MPC-only driving..."
      timeout 15 ros2 topic echo /drive --field drive.speed > "$LOG_DIR/s1_mpc_speed.log" 2>&1
      check_no_nan "$LOG_DIR/s1_mpc_speed.log" "Scenario1 MPC /drive.speed"
      check_bounded "$LOG_DIR/s1_mpc_speed.log" "Scenario1 MPC /drive.speed" 30.0

      pub_count=$(ros2 topic info /drive --verbose 2>/dev/null | grep "Publisher count" | awk '{print $3}')
      if [ "$pub_count" = "1" ]; then
        pass "Scenario1: /drive has exactly 1 publisher"
      else
        fail "Scenario1: /drive publisher count=$pub_count (expected 1)"
      fi

      log "===== SCENARIO 2: injected MPC failure -> fallback to PP ====="
      log "Scenario2: killing mpc_node..."
      pkill -9 -f "mpc_path_tracking/mpc_node" 2>/dev/null
      kill_time=$(date +%s)
      if wait_for_state "SWITCHING_TO_PP" 5 || wait_for_state "RUNNING_PP" 5; then
        recover_time=$(date +%s)
        pass "Scenario2: fell back to PP within $(( recover_time - kill_time ))s of killing MPC"
      else
        fail "Scenario2: did not fall back to PP after MPC was killed"
      fi
    else
      fail "Scenario1: did not reach RUNNING_MPC after switch began"
    fi
  else
    fail "Scenario1: arming gates never passed (params rejected, or PP tracking too poor to satisfy e_y/theta/convergence gates)"
  fi
else
  fail "Scenario1: stack failed to come up"
fi
cleanup

log "===== SCENARIO 3: both controllers unhealthy -> EMERGENCY_HALT -> recovery ====="
if launch_full_stack "s3"; then
  wait_for_state "RUNNING_PP" 15 && pass "Scenario3: reached RUNNING_PP" || fail "Scenario3: did not reach RUNNING_PP"

  log "Scenario3: killing pure_pursuit (MPC was never armed, so both are now unhealthy)..."
  pkill -9 -f "pure_pursuit/pure_pursuit_node.py" 2>/dev/null
  kill_time=$(date +%s)
  if wait_for_state "EMERGENCY_HALT" 5; then
    halt_time=$(date +%s)
    pass "Scenario3: reached EMERGENCY_HALT within $(( halt_time - kill_time ))s"

    speed=$(read_topic_once /drive 2 | grep "speed:" | awk '{print $2}')
    steer=$(read_topic_once /drive 2 | grep "steering_angle:" | awk '{print $2}')
    if [ "${speed:-x}" = "0.0" ] && [ "${steer:-x}" = "0.0" ]; then
      pass "Scenario3: /drive is zero-throttle/zero-steering during EMERGENCY_HALT"
    else
      fail "Scenario3: /drive during EMERGENCY_HALT was speed=$speed steer=$steer (expected 0.0/0.0)"
    fi

    log "Scenario3: restarting pure_pursuit to confirm recovery..."
    nohup ros2 launch pure_pursuit pure_pursuit.launch.py standalone_mode:=false \
      > "$LOG_DIR/s3_pp_restart.log" 2>&1 &
    disown
    if wait_for_state "BOOTSTRAP_PP" 10 || wait_for_state "RUNNING_PP" 5; then
      pass "Scenario3: recovered out of EMERGENCY_HALT after PP restarted"
    else
      fail "Scenario3: did not recover after PP restarted"
    fi
  else
    fail "Scenario3: did not reach EMERGENCY_HALT after both controllers unhealthy"
  fi
else
  fail "Scenario3: stack failed to come up"
fi
cleanup

log "===== RESULTS: $PASS_COUNT passed, $FAIL_COUNT failed ====="
[ "$FAIL_COUNT" -eq 0 ]
