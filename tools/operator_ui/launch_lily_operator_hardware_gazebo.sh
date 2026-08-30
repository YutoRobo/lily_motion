#!/usr/bin/env bash
set -u

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$THIS_DIR/../.." && pwd)"
NORMAL_LAUNCHER="$THIS_DIR/launch_lily_operator.sh"
READINESS="$ROOT/tools/gazebo/runtime_readiness.py"
ROS_SETUP="${LILY_ROS_SETUP:-/opt/ros/melodic/setup.bash}"
CATKIN_SETUP="${LILY_CATKIN_SETUP:-$HOME/catkin_ws/devel/setup.bash}"
LOG_ROOT="${LILY_OPERATOR_LOG_ROOT:-$ROOT/runtime_logs/operator_ui}"

WORLD_PACKAGE="${LILY_GAZEBO_WORLD_PACKAGE:-lily_octpus_gazebo}"
WORLD_LAUNCH="${LILY_GAZEBO_WORLD_LAUNCH:-lily_octpus_world.launch}"
CONTROL_PACKAGE="${LILY_GAZEBO_CONTROL_PACKAGE:-lily_octpus_control}"
CONTROL_LAUNCH="${LILY_GAZEBO_CONTROL_LAUNCH:-lily_octpus_control.launch}"
CONTROL_DELAY_SEC="${LILY_GAZEBO_CONTROL_DELAY_SEC:-2.0}"
GAZEBO_TIMEOUT_SEC="${LILY_GAZEBO_READY_TIMEOUT_SEC:-30.0}"
CONTROL_TIMEOUT_SEC="${LILY_GAZEBO_CONTROL_TIMEOUT_SEC:-30.0}"
GAZEBO_INTERP_DURATION="${LILY_GAZEBO_INTERP_DURATION:-0.100}"
GAZEBO_UPDATE_PERIOD="${LILY_GAZEBO_UPDATE_PERIOD:-0.002}"

BOOT_LOG_DIR="$LOG_ROOT/launcher"
mkdir -p "$BOOT_LOG_DIR"
BOOT_LOG="$BOOT_LOG_DIR/hardware_gazebo_launcher_$(date +%Y%m%d_%H%M%S).log"

ROSCORE_PID=""
WORLD_PID=""
CONTROL_PID=""
INTERPOLATOR_PID=""
ROSCORE_STARTED=0
WORLD_STARTED=0
CONTROL_STARTED=0
INTERPOLATOR_STARTED=0
CLEANUP_DONE=0

log() {
  printf '%s %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*" | tee -a "$BOOT_LOG"
}

show_error() {
  local msg="$1"
  log "ERROR: $msg"
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title="Lily Operator (Hardware + Gazebo)" --text="$msg" >/dev/null 2>&1 || true
  elif command -v xmessage >/dev/null 2>&1; then
    xmessage -center "Lily Operator (Hardware + Gazebo)\n\n$msg" >/dev/null 2>&1 || true
  fi
}

source_setup_file() {
  local setup_file="$1"
  local label="$2"
  local rc

  log "Loading $label: $setup_file"
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  rc=$?
  set -u

  if [ "$rc" -ne 0 ]; then
    show_error "Failed to load $label: $setup_file"
    return "$rc"
  fi
  log "Loaded $label: $setup_file"
}

setup_ros_environment() {
  if [ ! -f "$ROS_SETUP" ]; then
    show_error "ROS setup file was not found: $ROS_SETUP"
    return 1
  fi
  source_setup_file "$ROS_SETUP" "ROS environment" || return 1

  if [ -f "$CATKIN_SETUP" ]; then
    source_setup_file "$CATKIN_SETUP" "catkin workspace" || return 1
  else
    show_error "Catkin setup was not found: $CATKIN_SETUP"
    return 1
  fi
}

ros_master_available() {
  if ! command -v rosparam >/dev/null 2>&1; then
    return 1
  fi
  if command -v timeout >/dev/null 2>&1; then
    timeout 1 rosparam list >/dev/null 2>&1
  else
    rosparam list >/dev/null 2>&1
  fi
}

ensure_roscore() {
  if ros_master_available; then
    log "ROS master already available at ${ROS_MASTER_URI:-default}; reusing it"
    return 0
  fi

  log "ROS master not reachable; starting roscore"
  roscore >>"$BOOT_LOG" 2>&1 &
  ROSCORE_PID=$!
  ROSCORE_STARTED=1

  local i
  for i in $(seq 1 30); do
    if ros_master_available; then
      log "roscore ready (pid=$ROSCORE_PID)"
      return 0
    fi
    if ! kill -0 "$ROSCORE_PID" >/dev/null 2>&1; then
      show_error "roscore exited during startup. See $BOOT_LOG"
      return 1
    fi
    sleep 0.2
  done

  show_error "roscore did not become ready. See $BOOT_LOG"
  return 1
}

ros_node_exists() {
  local node_name="$1"
  if command -v timeout >/dev/null 2>&1; then
    timeout 1 rosnode list 2>/dev/null | grep -Fxq "$node_name"
  else
    rosnode list 2>/dev/null | grep -Fxq "$node_name"
  fi
}

validate_dependencies() {
  local cmd
  for cmd in python2 roslaunch rosnode rospack; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
      show_error "Required command was not found after loading ROS: $cmd"
      return 1
    fi
  done
  if [ ! -f "$READINESS" ]; then
    show_error "Gazebo readiness helper was not found: $READINESS"
    return 1
  fi
  if ! rospack find "$WORLD_PACKAGE" >/dev/null 2>&1; then
    show_error "Gazebo package was not found by rospack: $WORLD_PACKAGE"
    return 1
  fi
  if ! rospack find "$CONTROL_PACKAGE" >/dev/null 2>&1; then
    show_error "Gazebo control package was not found by rospack: $CONTROL_PACKAGE"
    return 1
  fi
}

wait_gazebo_ready() {
  python2 "$READINESS" gazebo --timeout-sec "$GAZEBO_TIMEOUT_SEC" 2>&1 | tee -a "$BOOT_LOG"
  return ${PIPESTATUS[0]}
}

controller_status() {
  local output
  local rc
  output="$(python2 "$READINESS" controller-status 2>&1)"
  rc=$?
  log "$output"
  return "$rc"
}

wait_controllers_ready() {
  python2 "$READINESS" controllers --timeout-sec "$CONTROL_TIMEOUT_SEC" 2>&1 | tee -a "$BOOT_LOG"
  return ${PIPESTATUS[0]}
}

start_or_reuse_world() {
  # A short readiness probe avoids duplicating a Gazebo instance that is
  # already fully running before this launcher starts.
  if python2 "$READINESS" gazebo --timeout-sec 0.3 >>"$BOOT_LOG" 2>&1; then
    log "Gazebo world already ready; reusing existing Gazebo"
    WORLD_STARTED=0
    return 0
  fi

  # If the canonical Gazebo node exists but the world is not ready, treat it as
  # a partial/stale startup instead of launching a second Gazebo instance.
  if ros_node_exists "/gazebo"; then
    show_error "A /gazebo node exists but Gazebo readiness checks are incomplete. Close/fix the existing Gazebo session before using the combined launcher."
    return 1
  fi

  log "Starting Gazebo world: roslaunch $WORLD_PACKAGE $WORLD_LAUNCH"
  roslaunch "$WORLD_PACKAGE" "$WORLD_LAUNCH" >>"$BOOT_LOG" 2>&1 &
  WORLD_PID=$!
  WORLD_STARTED=1

  if ! wait_gazebo_ready; then
    if ! kill -0 "$WORLD_PID" >/dev/null 2>&1; then
      show_error "Gazebo world roslaunch exited during startup. See $BOOT_LOG"
    else
      show_error "Gazebo did not become ready within $GAZEBO_TIMEOUT_SEC s. See $BOOT_LOG"
    fi
    return 1
  fi

  log "Gazebo world ready (roslaunch pid=$WORLD_PID)"
}

start_or_reuse_control() {
  local rc

  controller_status
  rc=$?
  if [ "$rc" -eq 0 ]; then
    log "All 24 Gazebo controller command subscribers already ready; reusing existing control launch"
    CONTROL_STARTED=0
    return 0
  fi
  if [ "$rc" -eq 3 ]; then
    show_error "Gazebo controller topology is partial. Refusing to start a duplicate control launch. See $BOOT_LOG for the missing topics."
    return 1
  fi
  if [ "$rc" -ne 2 ]; then
    show_error "Could not inspect Gazebo controller readiness. See $BOOT_LOG"
    return 1
  fi

  log "Gazebo ready; waiting $CONTROL_DELAY_SEC s before starting controllers"
  sleep "$CONTROL_DELAY_SEC"

  log "Starting Gazebo controllers: roslaunch $CONTROL_PACKAGE $CONTROL_LAUNCH"
  roslaunch "$CONTROL_PACKAGE" "$CONTROL_LAUNCH" >>"$BOOT_LOG" 2>&1 &
  CONTROL_PID=$!
  CONTROL_STARTED=1

  if ! wait_controllers_ready; then
    if ! kill -0 "$CONTROL_PID" >/dev/null 2>&1; then
      show_error "Gazebo control roslaunch exited during startup. See $BOOT_LOG"
    else
      show_error "All 24 Gazebo controllers did not become ready within $CONTROL_TIMEOUT_SEC s. See $BOOT_LOG"
    fi
    return 1
  fi

  log "All 24 Gazebo controller command subscribers ready (roslaunch pid=$CONTROL_PID)"
}

start_or_reuse_interpolator() {
  local node_name="/lily_gazebo_mcu_position_interpolator"

  if ros_node_exists "$node_name"; then
    log "Gazebo MCU interpolator already running; reusing $node_name"
    INTERPOLATOR_STARTED=0
    return 0
  fi

  log "Starting Gazebo MCU interpolator: interp=$GAZEBO_INTERP_DURATION s update=$GAZEBO_UPDATE_PERIOD s"
  cd "$ROOT" || return 1
  python2 tools/gazebo/mcu_position_interpolator_node.py \
    --input-topic /cmdForJetson \
    --interp-duration-sec "$GAZEBO_INTERP_DURATION" \
    --update-period-sec "$GAZEBO_UPDATE_PERIOD" \
    >>"$BOOT_LOG" 2>&1 &
  INTERPOLATOR_PID=$!
  INTERPOLATOR_STARTED=1

  local i
  for i in $(seq 1 30); do
    if ros_node_exists "$node_name"; then
      log "Gazebo MCU interpolator ready (pid=$INTERPOLATOR_PID)"
      return 0
    fi
    if ! kill -0 "$INTERPOLATOR_PID" >/dev/null 2>&1; then
      show_error "Gazebo MCU interpolator exited during startup. See $BOOT_LOG"
      return 1
    fi
    sleep 0.2
  done

  show_error "Gazebo MCU interpolator did not become ready. See $BOOT_LOG"
  return 1
}

stop_started_process() {
  local label="$1"
  local pid="$2"
  local started="$3"
  local i

  if [ "$started" -ne 1 ] || [ -z "$pid" ]; then
    return 0
  fi
  if ! kill -0 "$pid" >/dev/null 2>&1; then
    return 0
  fi

  log "Stopping launcher-started $label (pid=$pid)"
  kill "$pid" >/dev/null 2>&1 || true
  for i in $(seq 1 25); do
    if ! kill -0 "$pid" >/dev/null 2>&1; then
      wait "$pid" >/dev/null 2>&1 || true
      return 0
    fi
    sleep 0.2
  done

  log "$label did not exit after SIGTERM; sending SIGKILL to launcher-owned pid=$pid"
  kill -9 "$pid" >/dev/null 2>&1 || true
  wait "$pid" >/dev/null 2>&1 || true
}

cleanup() {
  if [ "$CLEANUP_DONE" -eq 1 ]; then
    return
  fi
  CLEANUP_DONE=1
  stop_started_process "Gazebo MCU interpolator" "$INTERPOLATOR_PID" "$INTERPOLATOR_STARTED"
  stop_started_process "Gazebo control roslaunch" "$CONTROL_PID" "$CONTROL_STARTED"
  stop_started_process "Gazebo world roslaunch" "$WORLD_PID" "$WORLD_STARTED"
  stop_started_process "roscore" "$ROSCORE_PID" "$ROSCORE_STARTED"
}

main() {
  log "Lily Operator Hardware + Gazebo launcher start"
  log "Repository: $ROOT"
  log "Architecture: $(uname -m)"
  log "World launch: roslaunch $WORLD_PACKAGE $WORLD_LAUNCH"
  log "Control launch: roslaunch $CONTROL_PACKAGE $CONTROL_LAUNCH"
  log "Control delay: $CONTROL_DELAY_SEC s"

  setup_ros_environment || return 1
  ensure_roscore || return 1
  validate_dependencies || return 1

  if ros_node_exists "/lily_operator"; then
    show_error "The normal Lily Operator is already running. Close it before starting the combined Hardware + Gazebo launcher."
    return 1
  fi

  cd "$ROOT" || return 1
  start_or_reuse_world || return 1
  start_or_reuse_control || return 1
  start_or_reuse_interpolator || return 1

  log "Gazebo path ready. Starting the unchanged normal Lily Operator launcher."
  bash "$NORMAL_LAUNCHER"
  local rc=$?
  log "Normal Lily Operator launcher exited with code $rc"
  return "$rc"
}

trap cleanup EXIT
trap 'exit 130' INT TERM
main "$@"
