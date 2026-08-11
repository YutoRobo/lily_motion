# -*- coding: utf-8 -*-
import rospy
import can
import struct
import time
import math
import os
import sys

CAN_INTERFACE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if CAN_INTERFACE_DIR not in sys.path:
    sys.path.insert(0, CAN_INTERFACE_DIR)
from motion_check import (DEFAULT_MOTION_CHECK_CONFIG, build_motion_values,
                          build_return_values)
from std_msgs.msg import String
from sensor_msgs.msg import JointState


NUM_LEGS = 24
CONNECT_TIMEOUT_SEC = 3.0
ALIGNMENT_TIMEOUT_SEC = 30.0

DEFAULT_HOME_STEP = 0.005
HOME_STEP_MIN = 1e-6
HOME_STEP_MAX = 0.1

JOINT_LIMITS_RAD = [
    (-6.283185307179586, 6.283185307179586),  # base_clause +/-360deg
    (-1.6580627893946132, 1.6580627893946132),  # thigh +/-95deg
    (-2.6179938779914944, 2.6179938779914944),  # tibia +/-150deg
]
JOINT_NAMES = ["base_clause", "thigh", "tibia"]

LOW_FREQ_STATE_NAMES = {
    0: "start",
    1: "aliment_standby",
    2: "aliment",
    3: "get_home",
    4: "run_standby",
    5: "run",
    6: "servo_off",
    7: "aliment_error",
    8: "error",
}

ERROR_ID_NAMES = {
    0: "NOMINAL",
    1: "POS_CMD_NOT_READY",
    2: "POS_CMD_JUMP_ERR",
    3: "POS_CMD_OUT_OF_RANGE_ERR",
    4: "POS_OUT_OF_RANGE_ERR",
    5: "POS_ERROR_OUT_OF_RANGE_ERR",
    6: "POS_CMD_LUG_ERR",
    7: "FAULT_YET_ERR",
    8: "ALIMENT_ERR",
    9: "ALIMENT_STOP_ERR",
    10: "ACT_POS_OUT_OF_RANGE_ERR",
    11: "OTHER_AXIS_ERR",
    12: "FAULT_YET_INITIALISE_ERR",
    13: "OTHER",
}
INITIALIZATION_ERROR_IDS = frozenset((8, 9, 12))


def low_freq_state_name(value):
    return LOW_FREQ_STATE_NAMES.get(value, "unknown_state_%d" % value)


def error_id_name(value):
    return ERROR_ID_NAMES.get(value, "unknown_error_%d" % value)


class LegInfo(object):
    def __init__(self, leg_id):
        self.leg_id = leg_id
        self.connected = False
        self.last_seen = 0.0

        # connected means discovered in MCU aliment_standby and ready to ALIGN.
        # 0x0FF is not a whole-session liveness heartbeat.
        self.heartbeat_seen_once = False
        self.discovered_once_in_current_session = False
        self.use_selection_initialized = False
        self.use_manually_selected = False
        self.awaiting_heartbeat = False

        self.aligned = False
        self.homed = False
        self.aligned_in_current_session = False
        self.homed_in_current_session = False
        self.running_in_current_session = False
        self.run_command_sent_in_current_session = False
        self.diagnostic_run_command_sent = False
        self.diagnostic_run_q0_rad = None
        self.runtime_error_latched = False
        self.last_logical_position_command_rad = None
        self.alignment_in_progress = False
        self.alignment_request_generation = 0
        self.alignment_deadline = 0.0
        self.initialization_error_latched = False
        self.last_error_code = None
        self.last_error_name = None
        self.last_error_low_freq_state = None
        self.last_error_low_freq_state_name = None
        self.last_error_extension = ()

        # 手動原点調整位置（絶対float）
        self.home_pos = 0.0

        self.state_str = "Disconnected"


class StateMachine(object):
    """
    UI仕様に合わせた StateMachine

    Subscribe:
      - /ui/leg_command (String)    ex) "use:3:1", "align:3", "home_move:3:-1", "set_home:3", "home_step:0.002", "run", "stop"
      - /cmdForJetson (JointState)  24要素位置指令（RUN成立後、Use=True軸のみ送信）

    Publish:
      - /ui/leg_status (String)     ex) "3,Connected"
    """

    def __init__(self, bus):
        self.bus = bus

        # 脚テーブル
        self.legs = [LegInfo(i) for i in range(NUM_LEGS)]

        # UI Use=True の初期化・HOME・RUN成立判定・CAN送信対象。
        self.active_joints = set()

        # RUNモード
        self.is_run = False
        self.can_interface_ok = True
        self.send_error_latched = False
        self.error_latched = False
        self.global_error_details = []
        self.alignment_generation = 0
        self.stop_in_progress = False

        # Non-blocking, strictly single-axis RUN Motion Check state.
        self.motion_check_config = DEFAULT_MOTION_CHECK_CONFIG
        self.motion_check_active = False
        self.motion_check_axis = None
        self.motion_check_direction = None
        self.motion_check_q0 = None
        self.motion_check_values = []
        self.motion_check_index = 0
        self.motion_check_next_send_time = 0.0
        self.motion_check_complete_time = None
        self.motion_check_mode = None
        self.motion_check_last_failed_position = None
        self.last_external_position_command_time = None
        self.external_position_active_window_sec = 0.5

        # 手動原点調整のステップ量
        self.home_step = DEFAULT_HOME_STEP

        # ROS
        self.status_pub = rospy.Publisher("/ui/leg_status", String, queue_size=50)
        self.use_status_pub = rospy.Publisher(
            "/ui/leg_use_status", String, queue_size=50)
        self.motion_check_status_pub = rospy.Publisher(
            "/ui/motion_check_status", String, queue_size=20)
        self.diagnostic_targets_pub = rospy.Publisher(
            "/ui/diagnostic_targets", String, queue_size=20)
        self.diagnostic_status_pub = rospy.Publisher(
            "/ui/diagnostic_status", String, queue_size=20)
        rospy.Subscriber("/ui/leg_command", String, self.ui_command_callback)
        rospy.Subscriber("/cmdForJetson", JointState, self.coordinate_callback)

        rospy.loginfo(
            "StateMachine initialized. Listening /ui/leg_command and "
            "/cmdForJetson; CAN fan-out follows Use=True axes.")

    # =========================================================
    # CAN helpers
    # =========================================================
    def _send_can_message(self, arbitration_id, data):
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            return True
        except Exception as e:
            self.can_interface_ok = False
            self.send_error_latched = True
            rospy.logerr("CAN send error: %s", e)
            if self.is_run:
                self.handle_stop_request("can_send_error")
            return False

    def encode_float_to_bytes(self, fval):
        return list(bytearray(struct.pack('<f', float(fval))))

    # ---- CAN commands
    def send_alignment_request(self, leg_id):
        # alignment request: 0x000 + leg_id
        can_id = 0x000 + leg_id
        rospy.loginfo("[CAN] Alignment request leg=%d (ID=0x%03X)", leg_id, can_id)
        return self._send_can_message(can_id, [0] * 8)

    def send_manual_home_command(self, leg_id, abs_pos):
        # manual home adjust: 0x200 + leg_id, payload: [0,0,0,0] + float32(abs_pos)
        can_id = 0x200 + leg_id
        data = [0, 0, 0, 0] + self.encode_float_to_bytes(abs_pos)
        rospy.loginfo("[CAN] Manual home leg=%d abs_pos=%.6f (ID=0x%03X)", leg_id, abs_pos, can_id)
        return self._send_can_message(can_id, data)

    def send_set_home_command(self, leg_id):
        # set home: 0x300 + leg_id
        can_id = 0x300 + leg_id
        rospy.loginfo("[CAN] Set home leg=%d (ID=0x%03X)", leg_id, can_id)
        return self._send_can_message(can_id, [0] * 8)

    def send_run_start_command(self):
        active = self._active_joint_ids()
        if not active:
            rospy.logwarn("[CAN] RUN start rejected: no active joints")
            return False
        rospy.loginfo("[CAN] Run start active_joints=%s", active)
        ok = True
        for axis in active:
            ok = self._send_can_message(0x600 + axis, [0] * 8) and ok
        return ok

    def _position_command_data(self, position_rad):
        return [0, 0, 0, 0] + self.encode_float_to_bytes(position_rad)

    def send_position_command_one(self, leg_id, position_rad):
        """Send POS_SET to exactly one axis and update q only on success."""
        data = self._position_command_data(position_rad)
        if not self._send_can_message(0x400 | leg_id, data):
            self.motion_check_last_failed_position = float(position_rad)
            rospy.logerr(
                "[UI] RUN motion check single-axis send failed axis=%d position=%.6f",
                leg_id, position_rad)
            return False
        self.legs[leg_id].last_logical_position_command_rad = float(
            position_rad)
        return True

    def send_position_command_all(self, positions):
        """Send a 24-element command to Use=True axes only."""
        if positions is None or len(positions) != NUM_LEGS:
            rospy.logwarn("[CAN] Position command length invalid (ID=0x400+i)")
            return False
        active = self._active_joint_ids()
        if not active:
            rospy.logwarn("[CAN] Position command ignored: no active joints")
            return False
        ok = True
        for i in active:
            sent = self._send_can_message(
                0x400 + i, self._position_command_data(positions[i]))
            if sent:
                self.legs[i].last_logical_position_command_rad = float(
                    positions[i])
            ok = sent and ok
        return ok

    # =========================================================
    # UI <-> StateMachine
    # =========================================================
    def publish_leg_status(self, leg_id, state_str):
        self.status_pub.publish("{},{}".format(leg_id, state_str))

    def publish_leg_use_status(self, leg_id):
        active = 1 if self._is_active(leg_id) else 0
        self.use_status_pub.publish("%d,%d" % (leg_id, active))

    def set_leg_state(self, leg_id, new_state):
        leg = self.legs[leg_id]
        if leg.state_str != new_state:
            leg.state_str = new_state
            self.publish_leg_status(leg_id, new_state)

    def ui_command_callback(self, msg):
        """
        /ui/leg_command: String
          - "use:<leg>:0" or "use:<leg>:1"
          - "align:<leg>"
          - "home_move:<leg>:-1" or "home_move:<leg>:+1"
          - "set_home:<leg>"
          - "home_step:<float>"
          - "run"
          - "stop"
        """
        s = msg.data.strip()
        if not s:
            return

        if s.startswith("motion_check_start:"):
            parts = s.split(":")
            if len(parts) != 3:
                self._publish_motion_check_status(
                    "rejected: invalid_start_command")
                return
            try:
                axis = int(parts[1])
                direction = 1 if parts[2] == "+" else (
                    -1 if parts[2] == "-" else 0)
            except Exception:
                direction = 0
                axis = -1
            self.start_motion_check(axis, direction)
            return

        if s.startswith("diagnostic_run:"):
            try:
                axis = int(s.split(":", 1)[1])
            except Exception:
                self._publish_diagnostic_status(-1, "rejected: invalid_axis")
                return
            self.submit_axis_command("ui", axis, "diagnostic_run")
            return

        if s == "motion_check_cancel":
            self.cancel_run_motion_check()
            return

        # RUN
        if s == "run":
            self.handle_run_request()
            return

        # STOP
        if s == "stop":
            self.submit_axis_command("ui", None, "stop")
            return

        # use:<leg>:0|1
        if s.startswith("use:"):
            parts = s.split(":")
            if len(parts) != 3:
                rospy.logwarn("[UI] invalid use command: %s", s)
                return
            try:
                leg_id = int(parts[1])
                active = self._parse_active_flag(parts[2])
            except:
                rospy.logwarn("[UI] invalid use params: %s", s)
                return
            if 0 <= leg_id < NUM_LEGS:
                self.handle_use_selection(leg_id, active)
            return

        # home_step
        if s.startswith("home_step:"):
            try:
                val = float(s.split(":", 1)[1])
                val = max(HOME_STEP_MIN, min(HOME_STEP_MAX, val))
                self.home_step = val
                rospy.loginfo("[UI] home_step set to %.6f", self.home_step)
            except:
                rospy.logwarn("[UI] invalid home_step command: %s", s)
            return

        # align (all Use axes) or align:<leg>
        if s == "align":
            self.handle_alignment_request()
            return
        if s.startswith("align:"):
            try:
                leg_id = int(s.split(":", 1)[1])
            except:
                rospy.logwarn("[UI] invalid align command: %s", s)
                return
            if 0 <= leg_id < NUM_LEGS:
                self.handle_alignment_request([leg_id])
            return

        # home_move:<leg>:<dir>
        if s.startswith("home_move:"):
            parts = s.split(":")
            if len(parts) != 3:
                rospy.logwarn("[UI] invalid home_move command: %s", s)
                return
            try:
                leg_id = int(parts[1])
                direction = int(parts[2])
            except:
                rospy.logwarn("[UI] invalid home_move params: %s", s)
                return
            if direction not in (-1, 1):
                rospy.logwarn("[UI] home_move direction must be -1 or +1: %s", s)
                return
            if 0 <= leg_id < NUM_LEGS:
                self.handle_home_move(leg_id, direction)
            return

        # set_home:<leg>
        if s.startswith("set_home:"):
            try:
                leg_id = int(s.split(":", 1)[1])
            except:
                rospy.logwarn("[UI] invalid set_home command: %s", s)
                return
            if 0 <= leg_id < NUM_LEGS:
                self.handle_set_home(leg_id)
            return

        rospy.logwarn("[UI] unknown command: %s", s)

    def _parse_active_flag(self, s):
        v = s.strip().lower()
        if v in ("1", "true", "on", "yes"):
            return True
        if v in ("0", "false", "off", "no"):
            return False
        raise ValueError("invalid active flag")

    def _session_started(self):
        return self.is_run or any(
            leg.alignment_in_progress or leg.aligned_in_current_session
            or leg.homed_in_current_session
            or leg.run_command_sent_in_current_session for leg in self.legs)

    def handle_use_selection(self, leg_id, active):
        active = bool(active)
        leg = self.legs[leg_id]
        if self.motion_check_active:
            rospy.logwarn("[UI] Use change rejected: motion_check_active leg=%d",
                          leg_id)
            self.publish_leg_use_status(leg_id)
            return False
        if self._is_active(leg_id) == active:
            leg.use_selection_initialized = True
            leg.use_manually_selected = True
            self.publish_leg_use_status(leg_id)
            return True
        # Use changes after ALIGN starts are forbidden. Publish the retained
        # value so UI rolls back a rejected checkbox action.
        if self._session_started():
            rospy.logwarn("[UI] Use change rejected during active session leg=%d",
                          leg_id)
            self.publish_leg_use_status(leg_id)
            return False
        if active:
            self.active_joints.add(leg_id)
        else:
            self.active_joints.discard(leg_id)
        leg.use_selection_initialized = True
        leg.use_manually_selected = True
        self.publish_leg_use_status(leg_id)
        rospy.loginfo("[UI] Use=%s leg=%d active_joints=%s",
                      active, leg_id, self._active_joint_ids())
        return True

    def _enable_use_on_first_discovery(self, leg_id):
        leg = self.legs[leg_id]
        if (leg.use_selection_initialized or self._session_started()):
            return False
        self.active_joints.add(leg_id)
        leg.use_selection_initialized = True
        self.publish_leg_use_status(leg_id)
        rospy.loginfo("[CAN] leg%d discovered; Use enabled by default", leg_id)
        return True

    def _active_joint_ids(self):
        return sorted(i for i in self.active_joints if 0 <= i < NUM_LEGS)

    def _inactive_joint_ids(self):
        active = set(self._active_joint_ids())
        return [i for i in range(NUM_LEGS) if i not in active]

    def _is_active(self, leg_id):
        return leg_id in self.active_joints

    def _publish_all_leg_status(self):
        for leg in self.legs:
            self.publish_leg_status(leg.leg_id, leg.state_str)
            self.publish_leg_use_status(leg.leg_id)
        self._publish_diagnostic_targets()

    def alignment_pending(self):
        return [i for i in self._active_joint_ids()
                if not self.legs[i].aligned_in_current_session]

    def alignment_complete(self):
        active = self._active_joint_ids()
        return bool(active) and all(
            self.legs[i].aligned_in_current_session for i in active)

    def alignment_status_summary(self):
        active = self._active_joint_ids()
        aligned = [i for i in active if self.legs[i].aligned_in_current_session]
        pending = [i for i in active if i not in aligned]
        return "ALIGN incomplete: pending=%s, aligned=%s" % (pending, aligned)

    def handle_alignment_request(self, requested_ids=None, now=None):
        if self.motion_check_active:
            rospy.logwarn("[ALIGN] rejected: RUN motion check active")
            return []
        if now is None:
            now = time.time()
        requested = self._active_joint_ids() if requested_ids is None else [
            i for i in requested_ids if self._is_active(i)]
        candidates = []
        pending_heartbeat = []
        for leg_id in requested:
            leg = self.legs[leg_id]
            if leg.aligned_in_current_session or leg.alignment_in_progress:
                continue
            if (not leg.connected
                    or (now - leg.last_seen) > CONNECT_TIMEOUT_SEC):
                leg.connected = False
                pending_heartbeat.append(leg_id)
                self.set_leg_state(leg_id, "Disconnected")
                continue
            candidates.append(leg_id)

        if not candidates:
            if self.alignment_complete():
                rospy.loginfo("[ALIGN] already complete; no requests sent")
            else:
                rospy.logwarn("[ALIGN] %s pending_heartbeat=%s",
                              self.alignment_status_summary(), pending_heartbeat)
            return []

        self.alignment_generation += 1
        generation = self.alignment_generation
        sent = []
        for leg_id in candidates:
            leg = self.legs[leg_id]
            if not self.send_alignment_request(leg_id):
                continue
            leg.alignment_in_progress = True
            leg.alignment_request_generation = generation
            leg.alignment_deadline = now + ALIGNMENT_TIMEOUT_SEC
            leg.awaiting_heartbeat = False
            self.set_leg_state(leg_id, "Aligning")
            sent.append(leg_id)
        rospy.loginfo("[ALIGN] generation=%d sent=%s %s",
                      generation, sent, self.alignment_status_summary())
        return sent

    def _home_blocking_reasons(self):
        reasons = []
        active = self._active_joint_ids()
        if not active:
            return ["no_active_joints"]
        if self.error_latched:
            reasons.append("mcu_error_latched")
        for leg_id in active:
            leg = self.legs[leg_id]
            if not leg.aligned_in_current_session:
                reasons.append("leg%d_missing_aligned" % leg_id)
            if leg.alignment_in_progress:
                reasons.append("leg%d_alignment_in_progress" % leg_id)
            if leg.initialization_error_latched:
                reasons.append("leg%d_initialization_error" % leg_id)
        return reasons

    def _run_blocking_reasons(self):
        reasons = []
        active = self._active_joint_ids()
        if not active:
            return ["no_active_joints"]
        if not self.can_interface_ok:
            reasons.append("can_interface_error")
        if self.send_error_latched:
            reasons.append("pc_send_error")
        if self.error_latched:
            reasons.append("mcu_error_latched")
        for leg_id in active:
            leg = self.legs[leg_id]
            if leg.alignment_in_progress:
                reasons.append("leg%d_alignment_in_progress" % leg_id)
            if not leg.aligned_in_current_session:
                reasons.append("leg%d_missing_aligned" % leg_id)
            if not leg.homed_in_current_session:
                reasons.append("leg%d_missing_homed" % leg_id)
        return reasons

    def handle_run_request(self):
        if self.motion_check_active:
            rospy.logwarn("[UI] RUN rejected: motion_check_active")
            return False
        reasons = self._run_blocking_reasons()
        if reasons:
            self.is_run = False
            rospy.logwarn("[UI] RUN rejected: %s", ",".join(reasons))
            self._publish_all_leg_status()
            return False
        if not self.send_run_start_command():
            self.is_run = False
            return False
        self.is_run = True
        for leg_id in self._active_joint_ids():
            self.legs[leg_id].running_in_current_session = True
            self.legs[leg_id].run_command_sent_in_current_session = True
            self.set_leg_state(leg_id, "Running")
        rospy.loginfo("[UI] RUN accepted active_joints=%s", self._active_joint_ids())
        self._publish_all_leg_status()
        return True

    def handle_stop_request(self, reason="stop"):
        self.stop_in_progress = True
        if self.motion_check_active:
            self._abort_motion_check("stop_%s" % reason)
        if self.is_run:
            rospy.logwarn("[UI] STOP requested (%s) -> is_run=False", reason)
        self.is_run = False
        for leg in self.legs:
            diagnostic_was_sent = leg.diagnostic_run_command_sent
            leg.running_in_current_session = False
            leg.run_command_sent_in_current_session = False
            leg.diagnostic_run_command_sent = False
            if diagnostic_was_sent:
                self._publish_diagnostic_status(leg.leg_id, "stopped")
            if leg.homed_in_current_session:
                self.set_leg_state(leg.leg_id, "Homed")
        self._publish_all_leg_status()
        self.stop_in_progress = False
        return True

    def _position_limit_violation_all(self, positions):
        for i in self._active_joint_ids():
            joint_index = i % 3
            lo, hi = JOINT_LIMITS_RAD[joint_index]
            try:
                v = float(positions[i])
            except Exception:
                return "position[%d] is not numeric" % i
            if math.isnan(v) or math.isinf(v):
                return "position[%d] is non-finite" % i
            if v < lo or v > hi:
                return "position[%d] %s %.6f rad outside [%.6f, %.6f]" % (
                    i, JOINT_NAMES[joint_index], v, lo, hi)
        return None

    def _inactive_position_limit_warnings(self, positions):
        warnings = []
        for i in self._inactive_joint_ids():
            joint_index = i % 3
            lo, hi = JOINT_LIMITS_RAD[joint_index]
            try:
                v = float(positions[i])
            except Exception:
                warnings.append("position[%d] inactive non-numeric" % i)
                continue
            if v < lo or v > hi:
                warnings.append("position[%d] inactive %s %.6f rad outside [%.6f, %.6f]" % (
                    i, JOINT_NAMES[joint_index], v, lo, hi))
        return warnings

    # =========================================================
    # Home operations
    # =========================================================
    def handle_home_move(self, leg_id, direction):
        """
        Use=TrueかつAligned状態の脚だけ◀▶操作できる
        direction: -1 or +1
        送る値は位置指令と同じ float（絶対値）
        """
        leg = self.legs[leg_id]

        if self.motion_check_active:
            rospy.logwarn("[HOME] home_move rejected: motion_check_active")
            return False
        if not self._is_active(leg_id):
            rospy.logwarn("[HOME] leg=%d is Use=False -> ignore home_move", leg_id)
            return

        if not leg.aligned:
            rospy.logwarn("[HOME] leg=%d is not Aligned -> ignore home_move", leg_id)
            return

        leg.home_pos += float(direction) * float(self.home_step)
        self.send_manual_home_command(leg_id, leg.home_pos)

    def handle_set_home(self, leg_id):
        if self.motion_check_active:
            rospy.logwarn("[HOME] SET HOME rejected: motion_check_active")
            return False
        if not self._is_active(leg_id):
            rospy.logwarn("[HOME] leg=%d is Use=False -> ignore set_home", leg_id)
            return False
        leg = self.legs[leg_id]
        reasons = []
        if self.error_latched:
            reasons.append("mcu_error_latched")
        if not leg.aligned_in_current_session:
            reasons.append("leg%d_missing_aligned" % leg_id)
        if leg.alignment_in_progress:
            reasons.append("leg%d_alignment_in_progress" % leg_id)
        if leg.initialization_error_latched:
            reasons.append("leg%d_initialization_error" % leg_id)
        if reasons:
            rospy.logwarn("[HOME] SET HOME rejected: %s", ",".join(reasons))
            return False
        if not self.send_set_home_command(leg_id):
            return False
        leg.homed = True
        leg.homed_in_current_session = True
        leg.last_logical_position_command_rad = 0.0
        self.set_leg_state(leg_id, "Homed")
        return True

    # =========================================================
    # Diagnostic selected-axis RUN
    # =========================================================
    def diagnostic_axis_label(self, axis):
        if not 0 <= axis < NUM_LEGS:
            return "axis%d" % axis
        joint = JOINT_NAMES[axis % 3].replace("_clause", "").title()
        return "Leg %d %s (axis%d)" % (axis // 3 + 1, joint, axis)

    def get_diagnostic_target_axes(self):
        return [axis for axis in range(NUM_LEGS)
                if self._is_active(axis)
                and self.legs[axis].discovered_once_in_current_session
                and not self.legs[axis].runtime_error_latched]

    def _publish_diagnostic_targets(self):
        fields = ["%d|%s" % (axis, self.diagnostic_axis_label(axis))
                  for axis in self.get_diagnostic_target_axes()]
        self.diagnostic_targets_pub.publish(";".join(fields))

    def _publish_diagnostic_status(self, axis, status):
        self.diagnostic_status_pub.publish("%d|%s" % (axis, status))

    def _external_position_publisher_active(self, now):
        return (self.last_external_position_command_time is not None
                and now - self.last_external_position_command_time
                <= self.external_position_active_window_sec)

    def can_start_diagnostic_run(self, axis, now=None):
        if now is None:
            now = time.time()
        if not 0 <= axis < NUM_LEGS:
            return False, ["invalid_axis"]
        leg = self.legs[axis]
        checks = (
            (not self._is_active(axis), "axis%d_use_false" % axis),
            (not leg.discovered_once_in_current_session,
             "axis%d_not_discovered" % axis),
            (not leg.aligned_in_current_session,
             "axis%d_not_aligned" % axis),
            (not leg.homed_in_current_session, "axis%d_not_homed" % axis),
            (leg.alignment_in_progress,
             "axis%d_alignment_in_progress" % axis),
            (leg.initialization_error_latched,
             "axis%d_initialization_error" % axis),
            (leg.runtime_error_latched, "axis%d_runtime_error" % axis),
            (leg.last_logical_position_command_rad is None,
             "last_position_unknown"),
            (leg.diagnostic_run_command_sent,
             "axis%d_diagnostic_run_already_sent" % axis),
            (not self.can_interface_ok, "can_interface_error"),
            (self.send_error_latched, "pc_send_error"),
            (self.error_latched, "mcu_error_latched"),
            (self.stop_in_progress, "stop_in_progress"),
            (self.motion_check_active, "motion_check_active"),
            (self._external_position_publisher_active(now),
             "position_publisher_active"),
        )
        reasons = [reason for failed, reason in checks if failed]
        return not reasons, reasons

    def get_axis_diagnostic_status(self, axis, now=None):
        if not 0 <= axis < NUM_LEGS:
            return "Invalid axis"
        leg = self.legs[axis]
        if leg.runtime_error_latched:
            return "Error"
        if leg.diagnostic_run_command_sent:
            return "Diagnostic RUN sent"
        allowed, unused_reasons = self.can_start_diagnostic_run(axis, now)
        if allowed:
            return "Diagnostic RUN ready"
        return leg.state_str

    def start_diagnostic_run(self, axis, now=None, source="internal"):
        allowed, reasons = self.can_start_diagnostic_run(axis, now)
        if not allowed:
            reason = ",".join(reasons)
            rospy.logwarn("Diagnostic RUN rejected: %s", reason)
            self._publish_diagnostic_status(axis, "rejected: %s" % reason)
            return False
        if not self._send_can_message(0x600 | axis, [0] * 8):
            self._publish_diagnostic_status(axis, "rejected: pc_send_error")
            return False
        leg = self.legs[axis]
        leg.run_command_sent_in_current_session = True
        leg.diagnostic_run_command_sent = True
        leg.diagnostic_run_q0_rad = float(
            leg.last_logical_position_command_rad)
        rospy.loginfo("[CAN] Diagnostic RUN command sent source=%s axis=%d ID=0x%03X",
                      source, axis, 0x600 | axis)
        self._publish_diagnostic_status(axis, "Diagnostic RUN sent")
        return True

    def can_request_axis_position(self, axis, position_rad, now=None,
                                  source="external"):
        if now is None:
            now = time.time()
        if not 0 <= axis < NUM_LEGS:
            return False, ["invalid_axis"]
        leg = self.legs[axis]
        reasons = []
        checks = (
            (not self._is_active(axis), "axis%d_use_false" % axis),
            (not leg.discovered_once_in_current_session,
             "axis%d_not_discovered" % axis),
            (not leg.aligned_in_current_session,
             "axis%d_not_aligned" % axis),
            (not leg.homed_in_current_session, "axis%d_not_homed" % axis),
            (not leg.run_command_sent_in_current_session,
             "axis%d_run_command_not_sent" % axis),
            (leg.alignment_in_progress,
             "axis%d_alignment_in_progress" % axis),
            (leg.initialization_error_latched,
             "axis%d_initialization_error" % axis),
            (leg.runtime_error_latched, "axis%d_runtime_error" % axis),
            (leg.last_logical_position_command_rad is None,
             "last_position_unknown"),
            (not self.can_interface_ok, "can_interface_error"),
            (self.send_error_latched, "pc_send_error"),
            (self.error_latched, "mcu_error_latched"),
            (self.stop_in_progress, "stop_in_progress"),
            (self.motion_check_active and source != "motion_check",
             "motion_check_active"),
        )
        reasons.extend(reason for failed, reason in checks if failed)
        try:
            value = float(position_rad)
        except Exception:
            reasons.append("position_not_numeric")
            return False, reasons
        if math.isnan(value) or math.isinf(value):
            reasons.append("non_finite_position")
        else:
            lo, hi = JOINT_LIMITS_RAD[axis % 3]
            if value < lo or value > hi:
                reasons.append("joint_limit")
            if leg.last_logical_position_command_rad is not None:
                jump = abs(value - leg.last_logical_position_command_rad)
                if jump >= math.radians(4.0):
                    reasons.append("command_jump_4deg")
        return not reasons, reasons

    def request_axis_position(self, axis, position_rad, source="external",
                              now=None):
        allowed, reasons = self.can_request_axis_position(
            axis, position_rad, now=now, source=source)
        if not allowed:
            rospy.logwarn("Axis position rejected source=%s axis=%s: %s",
                          source, axis, ",".join(reasons))
            return False, reasons
        if not self.send_position_command_one(axis, float(position_rad)):
            return False, ["pc_send_error"]
        if source == "external":
            self.last_external_position_command_time = (
                time.time() if now is None else now)
        rospy.loginfo("[CAN] Axis position source=%s axis=%d value=%.6f",
                      source, axis, float(position_rad))
        return True, []

    def submit_axis_command(self, source, axis, command_type, value=None,
                            now=None):
        """Common safety and dispatch API for UI and external inputs."""
        if command_type == "stop":
            return self.handle_stop_request("%s_stop" % source), []
        if command_type == "diagnostic_run":
            if self.start_diagnostic_run(axis, now=now, source=source):
                return True, []
            unused_allowed, reasons = self.can_start_diagnostic_run(axis, now)
            return False, reasons or ["pc_send_error"]
        if command_type == "position_offset":
            if not 0 <= axis < NUM_LEGS:
                return False, ["invalid_axis"]
            q0 = self.legs[axis].diagnostic_run_q0_rad
            if q0 is None:
                return False, ["diagnostic_q0_unknown"]
            try:
                value = q0 + float(value)
            except Exception:
                return False, ["position_not_numeric"]
            command_type = "position"
        if command_type == "position":
            return self.request_axis_position(
                axis, value, source=source, now=now)
        return False, ["unknown_command_type"]

    # =========================================================
    # RUN Motion Check (single-axis, non-blocking)
    # =========================================================
    def _publish_motion_check_status(self, status):
        self.motion_check_status_pub.publish(status)

    def _motion_check_rejection_reasons(self, axis, now):
        if not 0 <= axis < NUM_LEGS:
            return ["invalid_axis"]
        reasons = []
        leg = self.legs[axis]
        checks = (
            (not self._is_active(axis), "axis%d_use_false" % axis),
            (not leg.aligned_in_current_session, "axis%d_not_aligned" % axis),
            (not leg.homed_in_current_session, "axis%d_not_homed" % axis),
            (not (leg.diagnostic_run_command_sent
                  or (self.is_run and leg.running_in_current_session)),
             "axis%d_run_command_not_sent" % axis),
            (leg.alignment_in_progress, "axis%d_alignment_in_progress" % axis),
            (leg.initialization_error_latched,
             "axis%d_initialization_error" % axis),
            (self.error_latched, "mcu_error_latched"),
            (not self.can_interface_ok, "can_interface_error"),
            (self.send_error_latched, "pc_send_error"),
            (self.motion_check_active, "motion_check_active"),
            (leg.last_logical_position_command_rad is None,
             "last_position_unknown"),
        )
        reasons.extend(reason for failed, reason in checks if failed)
        if self._external_position_publisher_active(now):
            reasons.append("position_publisher_active")
        return reasons

    def _validate_motion_values(self, axis, q0, values):
        lo, hi = JOINT_LIMITS_RAD[axis % 3]
        previous = q0
        for value in values:
            if math.isnan(value) or math.isinf(value):
                return "non_finite_position"
            if value < lo or value > hi:
                return "joint_limit"
            jump = abs(value - previous)
            if jump >= math.radians(4.0):
                return "command_jump_4deg"
            if abs(jump - self.motion_check_config.step_rad) > 1e-8:
                return "unexpected_step"
            previous = value
        return None

    def can_start_motion_check(self, axis, now=None):
        if now is None:
            now = time.time()
        reasons = self._motion_check_rejection_reasons(axis, now)
        return not reasons, reasons

    def start_motion_check(self, axis, direction, now=None):
        if now is None:
            now = time.time()
        reasons = self._motion_check_rejection_reasons(axis, now)
        if direction not in (-1, 1):
            reasons.append("invalid_direction")
        if reasons:
            reason = ",".join(reasons)
            rospy.logwarn("RUN motion check rejected: %s", reason)
            self._publish_motion_check_status("rejected: %s" % reason)
            return False
        leg = self.legs[axis]
        q0 = float(leg.last_logical_position_command_rad)
        try:
            values = build_motion_values(q0, direction, self.motion_check_config)
        except ValueError as exc:
            self._publish_motion_check_status(
                "rejected: sequence_invalid_%s" % exc)
            return False
        violation = self._validate_motion_values(axis, q0, values)
        if violation:
            rospy.logwarn("RUN motion check rejected: %s", violation)
            self._publish_motion_check_status("rejected: %s" % violation)
            return False
        self.motion_check_active = True
        self.motion_check_axis = axis
        self.motion_check_direction = direction
        self.motion_check_q0 = q0
        self.motion_check_values = list(values)
        self.motion_check_index = 0
        self.motion_check_next_send_time = (
            now + self.motion_check_config.command_period_sec)
        self.motion_check_complete_time = None
        self.motion_check_mode = "out_and_back"
        self.motion_check_last_failed_position = None
        sign = "+" if direction > 0 else "-"
        rospy.loginfo(
            "[UI] RUN motion check start axis=%d direction=%s q0=%.3f amplitude=%.3f",
            axis, sign, q0, self.motion_check_config.amplitude_rad)
        self._publish_motion_check_status(
            "active: axis%d direction=%s q0=%.3f" % (axis, sign, q0))
        return True

    # Backward-compatible name used by existing normal RUN tests/callers.
    def start_run_motion_check(self, axis, direction, now=None):
        return self.start_motion_check(axis, direction, now)

    def _motion_check_safe_to_return(self):
        if not self.motion_check_active or self.motion_check_axis is None:
            return False
        leg = self.legs[self.motion_check_axis]
        run_ready = (leg.diagnostic_run_command_sent
                     or (self.is_run and leg.running_in_current_session))
        return (self.can_interface_ok and not self.send_error_latched
                and not self.error_latched and run_ready
                and leg.aligned_in_current_session
                and leg.homed_in_current_session)

    def cancel_run_motion_check(self, now=None):
        if now is None:
            now = time.time()
        if not self.motion_check_active:
            self._publish_motion_check_status("rejected: not_active")
            return False
        axis = self.motion_check_axis
        if not self._motion_check_safe_to_return():
            self._abort_motion_check("cancel_unsafe_no_return")
            self.handle_stop_request("motion_check_cancel_unsafe")
            return False
        current = self.legs[axis].last_logical_position_command_rad
        values = build_return_values(current, self.motion_check_q0,
                                     self.motion_check_config)
        rospy.logwarn("[UI] RUN motion check cancelled; returning axis%d to q0", axis)
        self.motion_check_values = values
        self.motion_check_index = 0
        self.motion_check_mode = "cancel_return"
        self.motion_check_complete_time = None
        if values:
            self.motion_check_next_send_time = now + self.motion_check_config.command_period_sec
        else:
            self.motion_check_complete_time = now + self.motion_check_config.end_hold_sec
        self._publish_motion_check_status("returning: axis%d q0=%.3f" % (axis, self.motion_check_q0))
        return True

    def _abort_motion_check(self, reason, failed_position=None):
        if not self.motion_check_active:
            return
        axis = self.motion_check_axis
        last_position = (self.legs[axis].last_logical_position_command_rad
                         if axis is not None else None)
        self.motion_check_last_failed_position = failed_position
        rospy.logerr("[UI] RUN motion check aborted axis=%s reason=%s last_sent=%s failed=%s",
                     axis, reason, last_position, failed_position)
        self.motion_check_active = False
        self.motion_check_values = []
        self.motion_check_index = 0
        self.motion_check_complete_time = None
        self.motion_check_mode = "aborted"
        self._publish_motion_check_status("aborted: axis%s %s" % (axis, reason))

    def _complete_motion_check(self):
        axis = self.motion_check_axis
        q0 = self.motion_check_q0
        self.motion_check_active = False
        self.motion_check_values = []
        self.motion_check_index = 0
        self.motion_check_complete_time = None
        self.motion_check_mode = "complete"
        rospy.loginfo("[UI] RUN motion check complete axis=%d returned_to=%.3f", axis, q0)
        self._publish_motion_check_status("complete: axis%d returned_to=%.3f" % (axis, q0))

    def _execute_motion_check(self, now):
        if not self.motion_check_active:
            return
        axis = self.motion_check_axis
        if not self._motion_check_safe_to_return():
            self._abort_motion_check("runtime_safety_condition_lost")
            self.handle_stop_request("motion_check_safety_condition_lost")
            return
        if self.motion_check_complete_time is not None:
            if now >= self.motion_check_complete_time:
                self._complete_motion_check()
            return
        if now < self.motion_check_next_send_time:
            return
        if self.motion_check_index >= len(self.motion_check_values):
            self.motion_check_complete_time = now + self.motion_check_config.end_hold_sec
            return
        value = self.motion_check_values[self.motion_check_index]
        sent, unused_reasons = self.submit_axis_command(
            "motion_check", axis, "position", value=value, now=now)
        if not sent:
            self._abort_motion_check("position_send_failed", value)
            return
        self.motion_check_index += 1
        if self.motion_check_index >= len(self.motion_check_values):
            self.motion_check_complete_time = now + self.motion_check_config.end_hold_sec
            return
        outward_count = int(round(self.motion_check_config.amplitude_rad
                                  / self.motion_check_config.step_rad))
        hold = (self.motion_check_config.end_hold_sec
                if (self.motion_check_mode == "out_and_back"
                    and self.motion_check_index == outward_count)
                else 0.0)
        self.motion_check_next_send_time = (
            now + (hold if hold > 0.0
                   else self.motion_check_config.command_period_sec))

    # =========================================================
    # External position commands (RUN mode)
    # =========================================================
    def coordinate_callback(self, msg):
        """
        /cmdForJetson (JointState)
        RUN中はUse=True軸のみへ送信する。positionは24要素必須。
        """
        self.last_external_position_command_time = time.time()
        if self.motion_check_active:
            self._abort_motion_check("external_position_publisher_active")
            self.handle_stop_request("external_position_during_motion_check")
            return
        if not self.is_run:
            return
        try:
            pos = list(msg.position)
        except:
            return
        if len(pos) != NUM_LEGS:
            rospy.logwarn("[RUN] position length=%d != %d -> ignore", len(pos), NUM_LEGS)
            return
        if not self._active_joint_ids():
            rospy.logwarn("[RUN] active_joints is empty -> ignore position")
            return
        violation = self._position_limit_violation_all(pos)
        if violation:
            rospy.logerr("[RUN] hardware_limit_v2 reject: %s", violation)
            return
        self.send_position_command_all(pos)

    # =========================================================
    # CAN receive
    # =========================================================
    def can_callback(self, msg):
        """Decode 0x0FF standby discovery, ALIGN result, and 0x0EE error."""
        if msg is None:
            return
        arb = msg.arbitration_id
        data = list(msg.data)

        if arb == 0x0FF:
            if len(data) < 1:
                rospy.logwarn("[CONN] malformed 0x0FF DLC=%d", len(data))
                return
            leg_id = int(data[0])
            if 0 <= leg_id < NUM_LEGS:
                self.handle_connection_ping(leg_id)
            else:
                rospy.logwarn("[CONN] invalid 0x0FF leg=%d", leg_id)
            return

        if 0x100 <= arb < 0x100 + NUM_LEGS:
            if len(data) < 8:
                rospy.logwarn("[ALIGN] malformed result ID=0x%03X DLC=%d",
                              arb, len(data))
                return
            self.handle_alignment_result(arb - 0x100, int(data[7]))
            return

        if arb == 0x0EE:
            if len(data) != 8:
                rospy.logwarn("[ERROR] rejected 0x0EE DLC=%d expected=8",
                              len(data))
                return
            leg_id = int(data[0])
            if not 0 <= leg_id < NUM_LEGS:
                rospy.logwarn("[ERROR] rejected 0x0EE leg=%d", leg_id)
                return

            low_freq_state = int(data[1])
            extension = tuple(int(value) for value in data[2:7])
            error_code = int(data[7])
            state_name = low_freq_state_name(low_freq_state)
            name = error_id_name(error_code)
            if low_freq_state not in LOW_FREQ_STATE_NAMES:
                rospy.logwarn("[ERROR] leg=%d unknown lowFreqState=%s",
                              leg_id, state_name)
            if error_code not in ERROR_ID_NAMES:
                rospy.logwarn("[ERROR] leg=%d unknown errorID=%s",
                              leg_id, name)
            if any(extension):
                rospy.logwarn("[ERROR] leg=%d nonzero extension=%s",
                              leg_id, list(extension))
            self.handle_mcu_error(
                leg_id, error_code, low_freq_state, extension)
            return

        # DEBUG_MODE 0x0DD is intentionally ignored.
        return

    def _record_mcu_error(self, leg_id, error_code, low_freq_state,
                          extension):
        leg = self.legs[leg_id]
        leg.last_error_code = error_code
        leg.last_error_name = error_id_name(error_code)
        leg.last_error_low_freq_state = low_freq_state
        leg.last_error_low_freq_state_name = low_freq_state_name(
            low_freq_state)
        leg.last_error_extension = tuple(extension)
        return leg

    def _mark_alignment_failed(self, leg_id, reason, error_code=None):
        leg = self.legs[leg_id]
        leg.connected = False
        leg.aligned = False
        leg.homed = False
        leg.aligned_in_current_session = False
        leg.homed_in_current_session = False
        leg.alignment_in_progress = False
        leg.alignment_deadline = 0.0
        leg.running_in_current_session = False
        leg.run_command_sent_in_current_session = False
        leg.diagnostic_run_command_sent = False
        leg.awaiting_heartbeat = True
        leg.initialization_error_latched = True
        leg.last_error_code = error_code
        if error_code is not None:
            leg.last_error_name = error_id_name(error_code)
        leg.home_pos = 0.0
        display = "ALIGN incomplete"
        if error_code is not None:
            display += ": %s waiting heartbeat" % error_id_name(error_code)
        self.set_leg_state(leg_id, display)
        rospy.logwarn("[ALIGN] leg=%d failed reason=%s; awaiting 0x0FF",
                      leg_id, reason)
        if self._is_active(leg_id):
            rospy.logwarn("[ALIGN] %s", self.alignment_status_summary())
        if self.is_run and self._is_active(leg_id):
            self.handle_stop_request("alignment_failure_leg_%d" % leg_id)

    def handle_mcu_error(self, leg_id, error_code, low_freq_state=0,
                         extension=()):
        leg = self._record_mcu_error(
            leg_id, error_code, low_freq_state, extension)
        name = leg.last_error_name
        state_name = leg.last_error_low_freq_state_name

        if error_code == 0:
            rospy.loginfo("[ERROR] axis%d NOMINAL state=%s extension=%s",
                          leg_id, state_name, list(extension))
            return "nominal"

        if error_code in INITIALIZATION_ERROR_IDS:
            if self._is_active(leg_id):
                self._mark_alignment_failed(
                    leg_id, "%s state=%s" % (name, state_name), error_code)
                rospy.logwarn(
                    "[ERROR] axis%d %s: waiting heartbeat for retry",
                    leg_id, name)
            else:
                leg.initialization_error_latched = True
                leg.awaiting_heartbeat = True
                self.set_leg_state(
                    leg_id, "Error: %s ignored for gates Use=False" % name)
                rospy.logwarn(
                    "[ERROR] axis%d %s ignored for gates: Use=False",
                    leg_id, name)
            return "initialization"

        # Known runtime errors 1..7, 10, 11, 13 and all future unknown
        # error IDs are system-fatal until an explicit recovery operation.
        self.error_latched = True
        leg.runtime_error_latched = True
        leg.running_in_current_session = False
        leg.run_command_sent_in_current_session = False
        leg.diagnostic_run_command_sent = False
        self._publish_diagnostic_status(leg_id, "Error: %s" % name)
        detail = {
            "leg_id": leg_id,
            "error_id": error_code,
            "error_name": name,
            "low_freq_state": low_freq_state,
            "low_freq_state_name": state_name,
            "extension": tuple(extension),
            "use": self._is_active(leg_id),
        }
        self.global_error_details.append(detail)
        self.set_leg_state(leg_id, "Error: %s system error" % name)
        rospy.logerr("[ERROR] axis%d %s: system error state=%s extension=%s",
                     leg_id, name, state_name, list(extension))
        if self.is_run or self.motion_check_active:
            self.handle_stop_request(
                "mcu_system_error_leg_%d_%s" % (leg_id, name))
        return "runtime"

    def handle_connection_ping(self, leg_id, now=None):
        if now is None:
            now = time.time()
        leg = self.legs[leg_id]
        was_awaiting_heartbeat = leg.awaiting_heartbeat

        # 0x0FF can still be received while an ALIGN request is in flight.
        # Treat it only as connection freshness in that window. Cancelling the
        # request here makes the following normal 0x100+axis result look stale.
        if leg.alignment_in_progress:
            leg.connected = True
            leg.last_seen = now
            leg.heartbeat_seen_once = True
            leg.awaiting_heartbeat = False
            return

        unexpected = (leg.aligned_in_current_session
                      or leg.homed_in_current_session
                      or leg.run_command_sent_in_current_session
                      or leg.diagnostic_run_command_sent
                      or (self.is_run and self._is_active(leg_id)))

        if unexpected:
            rospy.logwarn("leg%d_unexpected_heartbeat_after_alignment", leg_id)
            leg.aligned = False
            leg.homed = False
            leg.aligned_in_current_session = False
            leg.homed_in_current_session = False
            leg.alignment_in_progress = False
            leg.alignment_deadline = 0.0
            leg.running_in_current_session = False
            leg.run_command_sent_in_current_session = False
            leg.diagnostic_run_command_sent = False
            leg.home_pos = 0.0
            if (self.is_run or self.motion_check_active) and self._is_active(leg_id):
                self.handle_stop_request(
                    "unexpected_heartbeat_after_alignment_leg_%d" % leg_id)

        first_discovery = not leg.discovered_once_in_current_session
        leg.discovered_once_in_current_session = True
        if first_discovery:
            self._enable_use_on_first_discovery(leg_id)

        leg.connected = True
        leg.last_seen = now
        leg.heartbeat_seen_once = True
        leg.awaiting_heartbeat = False
        self.set_leg_state(leg_id, "Connected")
        if first_discovery:
            rospy.loginfo(
                "[CONN] leg=%d standby heartbeat discovered; ready for ALIGN",
                leg_id)
        elif was_awaiting_heartbeat:
            rospy.loginfo(
                "[CONN] leg=%d standby heartbeat recovered; ready for ALIGN",
                leg_id)

    def handle_alignment_result(self, leg_id, success_flag, now=None):
        if now is None:
            now = time.time()
        leg = self.legs[leg_id]
        if not self._is_active(leg_id):
            rospy.logwarn("[ALIGN] ignored result for Use=False leg=%d", leg_id)
            return False
        if not leg.alignment_in_progress:
            rospy.logwarn("[ALIGN] ignored stale/unsolicited result leg=%d", leg_id)
            return False
        if now > leg.alignment_deadline:
            rospy.logwarn("[ALIGN] ignored expired generation=%d leg=%d",
                          leg.alignment_request_generation, leg_id)
            self._mark_alignment_failed(leg_id, "deadline_expired")
            return False

        if success_flag != 1:
            self._mark_alignment_failed(leg_id, "negative_result")
            return False

        leg.alignment_in_progress = False
        leg.alignment_deadline = 0.0
        leg.aligned = True
        leg.homed = False
        leg.running_in_current_session = False
        leg.run_command_sent_in_current_session = False
        leg.diagnostic_run_command_sent = False
        leg.aligned_in_current_session = True
        leg.homed_in_current_session = False
        leg.initialization_error_latched = False
        leg.awaiting_heartbeat = False
        self.set_leg_state(leg_id, "Aligned")
        rospy.loginfo("[ALIGN] generation=%d leg=%d success",
                      leg.alignment_request_generation, leg_id)
        if self.alignment_complete():
            rospy.loginfo("[ALIGN] all Use axes complete")
        else:
            rospy.logwarn("[ALIGN] %s", self.alignment_status_summary())
        return True

    # =========================================================
    # Periodic
    # =========================================================
    def execute(self, now=None):
        """Apply deadlines only while waiting in standby or for ALIGN result."""
        if now is None:
            now = time.time()
        for leg in self.legs:
            if leg.alignment_in_progress and now > leg.alignment_deadline:
                self._mark_alignment_failed(leg.leg_id, "alignment_timeout")
                continue

            # 0x0FF freshness is meaningful only before an ALIGN request, while
            # the MCU is expected to remain in aliment_standby.
            standby_wait = (
                leg.connected
                and not leg.alignment_in_progress
                and not leg.aligned_in_current_session
                and not leg.homed_in_current_session)
            if standby_wait and (now - leg.last_seen) > CONNECT_TIMEOUT_SEC:
                rospy.logwarn("[CONN] leg=%d standby heartbeat timeout",
                              leg.leg_id)
                leg.connected = False
                self.set_leg_state(leg.leg_id, "Disconnected")

        self._execute_motion_check(now)
        self._publish_all_leg_status()
