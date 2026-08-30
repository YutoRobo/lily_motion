#!/usr/bin/env bash
set -u

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$THIS_DIR/../.." && pwd)"
CAN_IF="${LILY_CAN_CHANNEL:-can0}"
CAN_BITRATE="${LILY_CAN_BITRATE:-500000}"
ROS_SETUP="${LILY_ROS_SETUP:-/opt/ros/melodic/setup.bash}"
CATKIN_SETUP="${LILY_CATKIN_SETUP:-$HOME/catkin_ws/devel/setup.bash}"
LOG_ROOT="${LILY_OPERATOR_LOG_ROOT:-$ROOT/runtime_logs/operator_ui}"
BOOT_LOG_DIR="$LOG_ROOT/launcher"
mkdir -p "$BOOT_LOG_DIR"
BOOT_LOG="$BOOT_LOG_DIR/launcher_$(date +%Y%m%d_%H%M%S).log"

log() {
  printf '%s %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*" | tee -a "$BOOT_LOG"
}

show_error() {
  local msg="$1"
  log "ERROR: $msg"
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title="Lily Operator" --text="$msg" >/dev/null 2>&1 || true
  elif command -v xmessage >/dev/null 2>&1; then
    xmessage -center "Lily Operator\n\n$msg" >/dev/null 2>&1 || true
  fi
}

find_ip_cmd() {
  if [ -x /sbin/ip ]; then
    printf '%s\n' /sbin/ip
  elif [ -x /bin/ip ]; then
    printf '%s\n' /bin/ip
  else
    command -v ip 2>/dev/null || true
  fi
}

find_modprobe_cmd() {
  if [ -x /sbin/modprobe ]; then
    printf '%s\n' /sbin/modprobe
  elif [ -x /usr/sbin/modprobe ]; then
    printf '%s\n' /usr/sbin/modprobe
  else
    command -v modprobe 2>/dev/null || true
  fi
}

validate_can_name() {
  case "$CAN_IF" in
    *[!A-Za-z0-9_.:-]*|'')
      show_error "Invalid CAN interface name: $CAN_IF"
      return 1
      ;;
  esac
  return 0
}

is_vcan_mode() {
  case "$CAN_IF" in
    vcan*) return 0 ;;
  esac

  if ip -details link show "$CAN_IF" >/dev/null 2>&1; then
    ip -details link show "$CAN_IF" 2>/dev/null | grep -Eq '(^|[[:space:]])vcan([[:space:]]|$)'
    return $?
  fi
  return 1
}

configure_vcan() {
  local ip_cmd
  ip_cmd="$(find_ip_cmd)"
  if [ -z "$ip_cmd" ]; then
    show_error "'ip' command was not found. Install iproute2."
    return 1
  fi

  if ip link show "$CAN_IF" >/dev/null 2>&1; then
    local details
    details="$(ip -details link show "$CAN_IF" 2>/dev/null || true)"
    if ! printf '%s\n' "$details" | grep -Eq '(^|[[:space:]])vcan([[:space:]]|$)'; then
      show_error "Interface '$CAN_IF' exists but is not a vcan interface."
      return 1
    fi

    if printf '%s\n' "$details" | head -n 1 | grep -q '<[^>]*UP[^>]*>'; then
      log "$CAN_IF already UP (virtual CAN; no bitrate setting)"
      return 0
    fi

    if ! command -v pkexec >/dev/null 2>&1; then
      show_error "Bringing $CAN_IF up needs administrator permission, but 'pkexec' was not found."
      return 1
    fi

    log "Bringing existing virtual CAN interface $CAN_IF up (administrator authentication may appear)"
    pkexec "$ip_cmd" link set "$CAN_IF" up >>"$BOOT_LOG" 2>&1 || {
      show_error "Failed to bring $CAN_IF up. See $BOOT_LOG"
      return 1
    }
  else
    local modprobe_cmd
    modprobe_cmd="$(find_modprobe_cmd)"
    if [ -z "$modprobe_cmd" ]; then
      show_error "'modprobe' was not found; cannot create $CAN_IF."
      return 1
    fi
    if ! command -v pkexec >/dev/null 2>&1; then
      show_error "Creating $CAN_IF needs administrator permission, but 'pkexec' was not found."
      return 1
    fi

    log "Creating virtual CAN interface $CAN_IF (administrator authentication may appear)"
    pkexec /bin/sh -c \
      "'$modprobe_cmd' vcan && '$ip_cmd' link add dev '$CAN_IF' type vcan && '$ip_cmd' link set '$CAN_IF' up" \
      >>"$BOOT_LOG" 2>&1 || {
        show_error "Failed to create $CAN_IF. See $BOOT_LOG"
        return 1
      }
  fi

  if ! ip link show "$CAN_IF" >/dev/null 2>&1; then
    show_error "$CAN_IF was not found after vcan setup."
    return 1
  fi
  if ! ip -details link show "$CAN_IF" 2>/dev/null | grep -Eq '(^|[[:space:]])vcan([[:space:]]|$)'; then
    show_error "$CAN_IF is not a vcan interface after setup."
    return 1
  fi
  if ! ip -details link show "$CAN_IF" 2>/dev/null | head -n 1 | grep -q '<[^>]*UP[^>]*>'; then
    show_error "$CAN_IF is not UP after setup."
    return 1
  fi

  log "$CAN_IF virtual CAN ready (no bitrate setting)"
  return 0
}

configure_physical_can() {
  local ip_cmd
  ip_cmd="$(find_ip_cmd)"
  if [ -z "$ip_cmd" ]; then
    show_error "'ip' command was not found. Install iproute2."
    return 1
  fi

  if ! ip link show "$CAN_IF" >/dev/null 2>&1; then
    show_error "CAN interface '$CAN_IF' was not found."
    return 1
  fi

  local details
  details="$(ip -details link show "$CAN_IF" 2>/dev/null || true)"
  local bitrate_ok=0
  local up_ok=0
  if printf '%s\n' "$details" | grep -Eq "bitrate[[:space:]]+$CAN_BITRATE([[:space:]]|$)"; then
    bitrate_ok=1
  fi
  if printf '%s\n' "$details" | head -n 1 | grep -q '<[^>]*UP[^>]*>'; then
    up_ok=1
  fi

  if [ "$bitrate_ok" -eq 1 ] && [ "$up_ok" -eq 1 ]; then
    log "$CAN_IF already UP at $CAN_BITRATE bit/s"
    return 0
  fi

  if ! command -v pkexec >/dev/null 2>&1; then
    show_error "CAN setup needs administrator permission, but 'pkexec' was not found."
    return 1
  fi

  log "Configuring $CAN_IF at $CAN_BITRATE bit/s (administrator authentication may appear)"
  if [ "$bitrate_ok" -eq 1 ]; then
    pkexec "$ip_cmd" link set "$CAN_IF" up >>"$BOOT_LOG" 2>&1 || {
      show_error "Failed to bring $CAN_IF up. See $BOOT_LOG"
      return 1
    }
  else
    pkexec /bin/sh -c \
      "'$ip_cmd' link set '$CAN_IF' down 2>/dev/null || true; '$ip_cmd' link set '$CAN_IF' type can bitrate '$CAN_BITRATE' && '$ip_cmd' link set '$CAN_IF' up" \
      >>"$BOOT_LOG" 2>&1 || {
        show_error "Failed to configure $CAN_IF at $CAN_BITRATE bit/s. See $BOOT_LOG"
        return 1
      }
  fi

  details="$(ip -details link show "$CAN_IF" 2>/dev/null || true)"
  if ! printf '%s\n' "$details" | grep -Eq "bitrate[[:space:]]+$CAN_BITRATE([[:space:]]|$)"; then
    show_error "$CAN_IF did not report bitrate $CAN_BITRATE after setup."
    return 1
  fi
  if ! printf '%s\n' "$details" | head -n 1 | grep -q '<[^>]*UP[^>]*>'; then
    show_error "$CAN_IF is not UP after setup."
    return 1
  fi
  log "$CAN_IF configured successfully"
}

configure_can() {
  validate_can_name || return 1

  if is_vcan_mode; then
    log "CAN mode: virtual ($CAN_IF)"
    configure_vcan
  else
    log "CAN mode: physical ($CAN_IF, $CAN_BITRATE bit/s)"
    configure_physical_can
  fi
}

source_setup_file() {
  local setup_file="$1"
  local label="$2"
  local rc

  log "Loading $label: $setup_file"

  # ROS/catkin setup scripts are not guaranteed to be compatible with
  # `set -u` (nounset). Temporarily disable it while sourcing them, then
  # restore the launcher's stricter mode afterward.
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

  # Do not let an unreachable/stale ROS_MASTER_URI stall desktop startup.
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
  ROSCORE_PID=$!
  export ROSCORE_PID

  local i
  for i in $(seq 1 30); do
    if ros_master_available; then
      log "roscore ready (pid=$ROSCORE_PID)"
      return 0
    fi
    sleep 0.2
  done

  show_error "roscore did not become ready. See $BOOT_LOG"
  return 1
}

main() {
  log "Lily Operator launcher start"
  log "Repository: $ROOT"
  log "Architecture: $(uname -m)"
  log "Requested CAN interface: $CAN_IF"

  configure_can || return 1
  setup_ros_environment || return 1
  ensure_roscore || return 1

  if ! command -v python2 >/dev/null 2>&1; then
    show_error "python2 was not found."
    return 1
  fi

  cd "$ROOT" || return 1
  log "Starting integrated Operator UI on $CAN_IF"
  exec python2 tools/operator_ui/lily_operator_integrated.py \
    --can-interface socketcan \
    --can-channel "$CAN_IF" \
    --can-bitrate "$CAN_BITRATE" \
    >>"$BOOT_LOG" 2>&1
}

main "$@"
