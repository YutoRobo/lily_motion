#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PUBLISHER="${ROOT}/tools/publish_cmdforjetson_jsonl.py"
GAZEBO_REPLAY="${ROOT}/tools/gazebo/run_v3_0_gazebo_replay.py"
AIR_ENTRY_LOG="${ROOT}/testdata/hardware_trial_air_entry_only/air_entry_and_hold_only_commands.jsonl"
ROLL_BODY_LOG="${ROOT}/data/reference_candidates/v3_0_42c_candidate_02_softlimit_94p8/commands.jsonl"
GAZEBO_RUNTIME_DIR="${ROOT}/testdata/staged_gazebo_manual_runtime"
MODE="hardware"
HARDWARE_RATE_HZ="3"
GAZEBO_RATE_HZ="2"
ENABLE_FULL_ROLL=0
DRY_RUN=0

usage() {
  cat <<'EOF'
Usage: tools/run_hardware_staged_manual.sh [options]

Options:
  --mode hardware|gazebo   Select hardware publisher mode or Gazebo replay mode. Default: hardware.
  --gazebo                 Compatibility alias for --mode gazebo.
  --enable-full-roll       Enable the full roll body stage. Disabled by default.
  --rate HZ                Hardware /cmdForJetson publish rate. Default: 3.
  --gazebo-rate HZ         Gazebo replay rate. Default: 2.
  --dry-run                Print planned commands only.
  --help, -h               Show this help.

Hardware mode wraps tools/publish_cmdforjetson_jsonl.py and publishes
/ui/leg_command run/stop around each stage. Gazebo mode calls
tools/gazebo/run_v3_0_gazebo_replay.py and does not publish /ui/leg_command,
/cmdForJetson, open can0, or start tools/can_interface.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      if [[ $# -lt 2 ]]; then
        echo "error: --mode requires hardware or gazebo" >&2
        exit 2
      fi
      MODE="$2"
      shift 2
      ;;
    --gazebo)
      MODE="gazebo"
      shift
      ;;
    --enable-full-roll)
      ENABLE_FULL_ROLL=1
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    --rate)
      if [[ $# -lt 2 ]]; then
        echo "error: --rate requires a value" >&2
        exit 2
      fi
      HARDWARE_RATE_HZ="$2"
      shift 2
      ;;
    --gazebo-rate)
      if [[ $# -lt 2 ]]; then
        echo "error: --gazebo-rate requires a value" >&2
        exit 2
      fi
      GAZEBO_RATE_HZ="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "error: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${MODE}" in
  hardware|gazebo)
    ;;
  *)
    echo "error: --mode must be hardware or gazebo: ${MODE}" >&2
    exit 2
    ;;
esac

print_cmd() {
  printf '[dry-run]'
  printf ' %q' "$@"
  printf '\n'
}

publish_ui_command() {
  local command="$1"
  if [[ "${DRY_RUN}" -eq 1 ]]; then
    print_cmd rostopic pub -1 /ui/leg_command std_msgs/String "data: '${command}'"
    return 0
  fi
  rostopic pub -1 /ui/leg_command std_msgs/String "data: '${command}'"
}

publish_stop_and_exit() {
  echo
  echo "Stopping staged manual wrapper."
  if [[ "${MODE}" == "hardware" ]]; then
    publish_ui_command stop || true
  else
    echo "Gazebo mode: no /ui/leg_command stop is published."
  fi
  exit 130
}

trap publish_stop_and_exit INT TERM

stage_log_path() {
  local stage_id="$1"
  printf '%s/%s_commands.jsonl' "${GAZEBO_RUNTIME_DIR}" "${stage_id}"
}

prepare_gazebo_command_log() {
  local stage_id="$1"
  local source_log="$2"
  local start_index="${3:-}"
  local max_frames="${4:-}"
  local out_log
  out_log="$(stage_log_path "${stage_id}")"

  if [[ -z "${start_index}" && -z "${max_frames}" ]]; then
    printf '%s' "${source_log}"
    return 0
  fi

  if [[ -z "${start_index}" ]]; then
    start_index=0
  fi

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    if [[ -n "${max_frames}" ]]; then
      echo "[dry-run] would create ${out_log} from ${source_log} start ${start_index} count ${max_frames}" >&2
    else
      echo "[dry-run] would create ${out_log} from ${source_log} start ${start_index} count none" >&2
    fi
    printf '%s' "${out_log}"
    return 0
  fi

  mkdir -p "${GAZEBO_RUNTIME_DIR}"
  if [[ -n "${max_frames}" ]]; then
    awk -v start_index="${start_index}" -v max_frames="${max_frames}" 'NF { if (count >= start_index && emitted < max_frames) { print; emitted += 1 } count += 1; if (emitted >= max_frames) exit }' "${source_log}" > "${out_log}"
  else
    awk -v start_index="${start_index}" 'NF { if (count >= start_index) print; count += 1 }' "${source_log}" > "${out_log}"
  fi
  printf '%s' "${out_log}"
}

run_hardware_publisher() {
  local command_log="$1"
  local start_index="${2:-}"
  local max_frames="${3:-}"
  local cmd=(python "${PUBLISHER}" --command-log "${command_log}" --rate "${HARDWARE_RATE_HZ}")
  if [[ -n "${start_index}" ]]; then
    cmd+=(--start-index "${start_index}")
  fi
  if [[ -n "${max_frames}" ]]; then
    cmd+=(--max-frames "${max_frames}")
  fi

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    print_cmd "${cmd[@]}"
    return 0
  fi
  "${cmd[@]}"
}

run_gazebo_replay() {
  local stage_id="$1"
  local command_log="$2"
  local start_index="${3:-}"
  local max_frames="${4:-}"
  local gazebo_log
  gazebo_log="$(prepare_gazebo_command_log "${stage_id}" "${command_log}" "${start_index}" "${max_frames}")"
  local cmd=(python "${GAZEBO_REPLAY}" --command-log "${gazebo_log}" --strict-command-log-input --rate "${GAZEBO_RATE_HZ}")

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    print_cmd "${cmd[@]}"
    return 0
  fi
  "${cmd[@]}"
}

prompt_stage() {
  local stage_name="$1"
  local detail="$2"
  echo
  echo "Stage: ${stage_name}"
  echo "${detail}"
  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "Dry run: auto-printing this stage command."
    return 0
  fi
  echo "Press Enter to start this stage."
  if [[ "${MODE}" == "hardware" ]]; then
    echo "Type 'stop' to publish stop and exit. Type 'q' to exit without publishing stop."
  else
    echo "Type 'stop' to exit. Type 'q' to exit. Gazebo mode publishes no /ui/leg_command."
  fi
  local reply=""
  read -r reply || reply="q"
  case "${reply}" in
    "")
      return 0
      ;;
    stop|STOP)
      if [[ "${MODE}" == "hardware" ]]; then
        publish_ui_command stop
      else
        echo "Gazebo mode: stop requested; no /ui/leg_command stop is published."
      fi
      exit 0
      ;;
    q|Q)
      echo "Exit requested. No stage started."
      exit 0
      ;;
    *)
      echo "Unrecognized input: ${reply}" >&2
      echo "Stage not started." >&2
      return 1
      ;;
  esac
}

run_stage() {
  local stage_id="$1"
  local stage_name="$2"
  local command_log="$3"
  local start_index="${4:-}"
  local max_frames="${5:-}"
  local detail="$6"

  while ! prompt_stage "${stage_name}" "${detail}"; do
    :
  done

  set +e
  if [[ "${MODE}" == "hardware" ]]; then
    echo "Publishing run before stage: ${stage_name}"
    publish_ui_command run
    echo "Publishing command log for stage: ${stage_name}"
    if [[ -n "${start_index}" || -n "${max_frames}" ]]; then
      echo "Stage range: start ${start_index:-0}, count ${max_frames:-none}"
    fi
    run_hardware_publisher "${command_log}" "${start_index}" "${max_frames}"
    local stage_status=$?
    echo "Publishing stop after stage: ${stage_name}"
    publish_ui_command stop
  else
    echo "Running Gazebo replay for stage: ${stage_name}"
    if [[ -n "${start_index}" || -n "${max_frames}" ]]; then
      echo "Stage range: start ${start_index:-0}, count ${max_frames:-none}"
    fi
    run_gazebo_replay "${stage_id}" "${command_log}" "${start_index}" "${max_frames}"
    local stage_status=$?
  fi
  set -e

  if [[ "${stage_status}" -ne 0 ]]; then
    echo "error: stage failed: ${stage_name}" >&2
    return "${stage_status}"
  fi
}

if [[ ! -f "${PUBLISHER}" ]]; then
  echo "error: missing publisher: ${PUBLISHER}" >&2
  exit 1
fi
if [[ ! -f "${GAZEBO_REPLAY}" ]]; then
  echo "error: missing Gazebo replay tool: ${GAZEBO_REPLAY}" >&2
  exit 1
fi
if [[ ! -f "${AIR_ENTRY_LOG}" ]]; then
  echo "error: missing air-entry command log: ${AIR_ENTRY_LOG}" >&2
  exit 1
fi
if [[ ! -f "${ROLL_BODY_LOG}" ]]; then
  echo "error: missing roll-body command log: ${ROLL_BODY_LOG}" >&2
  exit 1
fi

if [[ "${MODE}" == "gazebo" && "${DRY_RUN}" -ne 1 ]]; then
  mkdir -p "${GAZEBO_RUNTIME_DIR}"
fi

echo "Hardware staged manual wrapper"
echo "Repo: ${ROOT}"
echo "Mode: ${MODE}"
echo "Hardware publisher: ${PUBLISHER}"
echo "Hardware rate: ${HARDWARE_RATE_HZ} Hz"
echo "Gazebo replay: ${GAZEBO_REPLAY}"
echo "Gazebo rate: ${GAZEBO_RATE_HZ} Hz"
echo "Gazebo runtime dir: ${GAZEBO_RUNTIME_DIR}"
echo "Full roll enabled: ${ENABLE_FULL_ROLL}"
echo "Dry run: ${DRY_RUN}"
echo
echo "Preconditions:"
echo "- roscore is running when required by the selected mode."
echo "- Hardware mode assumes tools/can_interface/statemachine/main.py is already running only when the operator intends hardware operation."
echo "- Gazebo mode does not start tools/can_interface and does not publish /ui/leg_command or /cmdForJetson."
echo "- can0 setup is handled outside this wrapper and is never opened here."
echo "- external/can_interface is not used."

run_stage \
  "air_entry_hold" \
  "air-entry + hold only" \
  "${AIR_ENTRY_LOG}" \
  "" \
  "" \
  "Uses only the staged air-entry and touchdown-hold command log. Confirm posture and STOP behavior before proceeding."

run_stage \
  "roll_0_50" \
  "roll frames 0-50" \
  "${ROLL_BODY_LOG}" \
  "0" \
  "50" \
  "Uses roll body frames 0-49 only. This is the first-quarter RF2/RF3 near-contact motion check."

echo
echo "Mandatory visual check after roll frames 0-50:"
echo "- Confirm first-quarter RF2/RF3-region near-contact clearance, no abnormal contact, no unexpected posture jump, and STOP behavior."
echo "- Hardware roll is prohibited if Gazebo visual near-contact is observed, even when the primitive scan reports ok."
echo "- Continue only if the operator accepts the observed posture and motion."

run_stage \
  "roll_50_100" \
  "roll frames 50-100" \
  "${ROLL_BODY_LOG}" \
  "50" \
  "50" \
  "Uses roll body frames 50-99 only. Continue only after the 0-50 visual check passed."

if [[ "${ENABLE_FULL_ROLL}" -eq 1 ]]; then
  run_stage \
    "roll_100_end" \
    "roll frames 100-end" \
    "${ROLL_BODY_LOG}" \
    "100" \
    "" \
    "Uses roll body frames 100 through the end. This stage is disabled by default and requires --enable-full-roll."
else
  echo
  echo "roll frames 100-end stage skipped. It is disabled by default."
  echo "Re-run with --enable-full-roll only after staged checks pass."
fi

echo
echo "Staged manual wrapper complete."
