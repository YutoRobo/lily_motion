#!/usr/bin/env bash
set -u

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$THIS_DIR/../.." && pwd)"
ROS_SETUP="${LILY_ROS_SETUP:-/opt/ros/melodic/setup.bash}"
CATKIN_SETUP="${LILY_CATKIN_SETUP:-$HOME/catkin_ws/devel/setup.bash}"
LOG_ROOT="${LILY_OPERATOR_LOG_ROOT:-$ROOT/runtime_logs/operator_ui}"
GAZEBO_INTERP_DURATION="${LILY_GAZEBO_INTERP_DURATION:-0.100}"
GAZEBO_UPDATE_PERIOD="${LILY_GAZEBO_UPDATE_PERIOD:-0.002}"
BOOT_LOG_DIR="$LOG_ROOT/launcher"
mkdir -p "$BOOT_LOG_DIR"
BOOT_LOG="$BOOT_LOG_DIR/gazebo_launcher_$(date +%Y%m%d_%H%M%S).log"
GAZEBO_BRIDGE_PID=""
GAZEBO_BRIDGE_STARTED=0

log() {
  printf '%s %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*" | tee -a "$BOOT_LOG"
}

show_error() {
  local msg="$1"
  log "ERROR: $msg"
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title="Lily Operator (Gazebo)" --text="$msg" >/dev/null 2>&1 || true
  elif command -v xmessage >/dev/null 2>&1; then
    xmessage -center "Lily Operator (Gazebo)\n\n$msg" >/dev/null 2>&1 || true
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
  return 0
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
    log "Catkin setup not found; continuing without it: $CATKIN_SETUP"
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
    log "ROS master already available at ${ROS_MASTER_URI:-default}"
    return 0
  fi

  if ! command -v roscore >/dev/null 2>&1; then
    show_error "roscore was not found after loading the ROS environment."
    return 1
  fi

  log "ROS master not reachable; starting roscore"
  roscore >>"$BOOT_LOG" 2>&1 &
  local roscore_pid=$!

  local i
  for i in $(seq 1 30); do
    if ros_master_available; then
      log "roscore ready (pid=$roscore_pid)"
      return 0
    fi
    sleep 0.2
  done

  show_error "roscore did not become ready. See $BOOT_LOG"
  return 1
}

ros_node_exists() {
  local node_name="$1"
  if ! command -v rosnode >/dev/null 2>&1; then
    return 1
  fi
  if command -v timeout >/dev/null 2>&1; then
    timeout 1 rosnode list 2>/dev/null | grep -Fxq "$node_name"
  else
    rosnode list 2>/dev/null | grep -Fxq "$node_name"
  fi
}

start_gazebo_bridge() {
  local node_name="/lily_gazebo_mcu_position_interpolator"

  if ros_node_exists "$node_name"; then
    log "Gazebo MCU interpolator already running; reusing $node_name"
    GAZEBO_BRIDGE_STARTED=0
    return 0
  fi

  log "Starting Gazebo MCU interpolator: interp=$GAZEBO_INTERP_DURATION s update=$GAZEBO_UPDATE_PERIOD s"
  python2 tools/gazebo/mcu_position_interpolator_node.py \
    --input-topic /cmdForJetson \
    --interp-duration-sec "$GAZEBO_INTERP_DURATION" \
    --update-period-sec "$GAZEBO_UPDATE_PERIOD" \
    >>"$BOOT_LOG" 2>&1 &
  GAZEBO_BRIDGE_PID=$!
  GAZEBO_BRIDGE_STARTED=1

  local i
  for i in $(seq 1 30); do
    if ros_node_exists "$node_name"; then
      log "Gazebo MCU interpolator ready (pid=$GAZEBO_BRIDGE_PID)"
      return 0
    fi
    if ! kill -0 "$GAZEBO_BRIDGE_PID" >/dev/null 2>&1; then
      show_error "Gazebo MCU interpolator exited during startup. See $BOOT_LOG"
      return 1
    fi
    sleep 0.2
  done

  show_error "Gazebo MCU interpolator did not become ready. See $BOOT_LOG"
  return 1
}

cleanup() {
  if [ "$GAZEBO_BRIDGE_STARTED" -eq 1 ] && [ -n "$GAZEBO_BRIDGE_PID" ]; then
    if kill -0 "$GAZEBO_BRIDGE_PID" >/dev/null 2>&1; then
      log "Stopping launcher-started Gazebo MCU interpolator (pid=$GAZEBO_BRIDGE_PID)"
      kill "$GAZEBO_BRIDGE_PID" >/dev/null 2>&1 || true
      wait "$GAZEBO_BRIDGE_PID" >/dev/null 2>&1 || true
    fi
  fi
}

main() {
  log "Lily Operator Gazebo launcher start"
  log "Repository: $ROOT"
  log "Architecture: $(uname -m)"
  log "CAN/StateMachine: disabled by Gazebo mode"

  setup_ros_environment || return 1
  ensure_roscore || return 1

  if ! command -v python2 >/dev/null 2>&1; then
    show_error "python2 was not found."
    return 1
  fi

  cd "$ROOT" || return 1
  start_gazebo_bridge || return 1

  log "Starting Gazebo-only Operator Motion UI"
  python2 tools/operator_ui/lily_operator_gazebo.py >>"$BOOT_LOG" 2>&1
  local rc=$?
  log "Gazebo Operator UI exited with code $rc"
  return "$rc"
}

trap cleanup EXIT INT TERM
main "$@"
