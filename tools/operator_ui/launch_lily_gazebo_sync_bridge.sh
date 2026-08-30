#!/usr/bin/env bash
set -u

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$THIS_DIR/../.." && pwd)"
CAN_IF="${LILY_CAN_CHANNEL:-can0}"
ROS_SETUP="${LILY_ROS_SETUP:-/opt/ros/melodic/setup.bash}"
CATKIN_SETUP="${LILY_CATKIN_SETUP:-$HOME/catkin_ws/devel/setup.bash}"
INTERP_SEC="${LILY_GAZEBO_INTERP_DURATION:-0.100}"
UPDATE_SEC="${LILY_GAZEBO_UPDATE_PERIOD:-0.002}"
COALESCE_SEC="${LILY_GAZEBO_CAN_COALESCE_SEC:-0.002}"
LOG_ROOT="${LILY_OPERATOR_LOG_ROOT:-$ROOT/runtime_logs/operator_ui}"
BOOT_LOG_DIR="$LOG_ROOT/launcher"
mkdir -p "$BOOT_LOG_DIR"
BOOT_LOG="$BOOT_LOG_DIR/gazebo_sync_bridge_$(date +%Y%m%d_%H%M%S).log"

log() {
  printf '%s %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*" | tee -a "$BOOT_LOG"
}

show_error() {
  local msg="$1"
  log "ERROR: $msg"
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title="Lily Gazebo Sync Bridge" --text="$msg" >/dev/null 2>&1 || true
  elif command -v xmessage >/dev/null 2>&1; then
    xmessage -center "Lily Gazebo Sync Bridge\n\n$msg" >/dev/null 2>&1 || true
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
    log "Catkin setup not found; continuing without it: $CATKIN_SETUP"
  fi
}

validate_can_ready() {
  if ! command -v ip >/dev/null 2>&1; then
    show_error "'ip' command was not found."
    return 1
  fi
  if ! ip link show "$CAN_IF" >/dev/null 2>&1; then
    show_error "CAN interface '$CAN_IF' was not found. Start the normal Lily Operator first."
    return 1
  fi
  if ! ip link show "$CAN_IF" | head -n 1 | grep -q '<[^>]*UP[^>]*>'; then
    show_error "$CAN_IF is not UP. Start the normal Lily Operator first so its existing CAN setup path prepares the interface."
    return 1
  fi
  log "$CAN_IF is UP; sync bridge will only observe it through candump"
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

ros_node_exists() {
  local node_name="$1"
  rosnode list 2>/dev/null | grep -Fxq "$node_name"
}

main() {
  log "Lily Gazebo Sync Bridge launcher start"
  log "Repository: $ROOT"
  log "CAN observe channel: $CAN_IF"
  log "Gazebo interpolation: $INTERP_SEC s"
  log "Gazebo update period: $UPDATE_SEC s"
  log "CAN burst coalesce: $COALESCE_SEC s"

  setup_ros_environment || return 1
  validate_can_ready || return 1

  if ! command -v candump >/dev/null 2>&1; then
    show_error "candump was not found. Install/use can-utils before starting the sync bridge."
    return 1
  fi
  if ! command -v python2 >/dev/null 2>&1; then
    show_error "python2 was not found."
    return 1
  fi
  if ! ros_master_available; then
    show_error "ROS master is not reachable. Start the normal Lily Operator first."
    return 1
  fi
  if ! command -v rosnode >/dev/null 2>&1; then
    show_error "rosnode was not found after loading ROS."
    return 1
  fi
  if ! ros_node_exists "/lily_operator"; then
    show_error "The normal Lily Operator (/lily_operator) is not running. Start 'Lily Operator' first; the sync bridge does not replace or modify it."
    return 1
  fi
  if ros_node_exists "/lily_gazebo_mcu_position_interpolator"; then
    show_error "Gazebo-only /cmdForJetson interpolator is running. Close 'Lily Operator (Gazebo)' before hardware synchronization; sync mode must keep /cmdForJetson subscribed only by the normal StateMachine."
    return 1
  fi
  if ros_node_exists "/lily_can_gazebo_sync_bridge"; then
    show_error "Lily CAN->Gazebo sync bridge is already running."
    return 1
  fi

  cd "$ROOT" || return 1
  log "Starting receive-only CAN->Gazebo bridge; Lily Operator and StateMachine are unchanged"
  exec python2 tools/gazebo/can_position_sync_bridge.py \
    --can-channel "$CAN_IF" \
    --interp-duration-sec "$INTERP_SEC" \
    --update-period-sec "$UPDATE_SEC" \
    --coalesce-sec "$COALESCE_SEC" \
    2>&1 | tee -a "$BOOT_LOG"
}

main "$@"
