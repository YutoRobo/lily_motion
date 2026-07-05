# -*- coding: utf-8 -*-
import rospy
import can
import struct
import time
from std_msgs.msg import String
from sensor_msgs.msg import JointState


NUM_LEGS = 24
CONNECT_TIMEOUT_SEC = 3.0

DEFAULT_HOME_STEP = 0.005
HOME_STEP_MIN = 1e-6
HOME_STEP_MAX = 0.1

JOINT_LIMITS_RAD = [
    (-6.283185307179586, 6.283185307179586),  # base_clause +/-360deg
    (-1.6580627893946132, 1.6580627893946132),  # thigh +/-95deg
    (-2.6179938779914944, 2.6179938779914944),  # tibia +/-150deg
]
JOINT_NAMES = ["base_clause", "thigh", "tibia"]


class LegInfo(object):
    def __init__(self, leg_id):
        self.leg_id = leg_id
        self.connected = False
        self.last_seen = 0.0

        self.aligned = False
        self.homed = False

        # 手動原点調整位置（絶対float）
        self.home_pos = 0.0

        self.state_str = "Disconnected"  # Disconnected / Connected / Aligned / Homed


class StateMachine(object):
    """
    UI仕様に合わせた StateMachine

    Subscribe:
      - /ui/leg_command (String)    ex) "use:3:1", "align:3", "home_move:3:-1", "set_home:3", "home_step:0.002", "run", "stop"
      - /cmdForJetson (JointState)  24要素位置指令（RUN中、Use=True軸のみ送信）

    Publish:
      - /ui/leg_status (String)     ex) "3,Connected"
    """

    def __init__(self, bus):
        self.bus = bus

        # 脚テーブル
        self.legs = [LegInfo(i) for i in range(NUM_LEGS)]

        # UI Use=True の制御対象軸。RUN gate/CAN送信はこの集合だけを見る。
        self.active_joints = set()

        # RUNモード
        self.is_run = False

        # 手動原点調整のステップ量
        self.home_step = DEFAULT_HOME_STEP

        # ROS
        self.status_pub = rospy.Publisher("/ui/leg_status", String, queue_size=50)
        rospy.Subscriber("/ui/leg_command", String, self.ui_command_callback)
        rospy.Subscriber("/cmdForJetson", JointState, self.coordinate_callback)

        rospy.loginfo("StateMachine initialized. Listening /ui/leg_command, /cmdForJetson.")

    # =========================================================
    # CAN helpers
    # =========================================================
    def _send_can_message(self, arbitration_id, data):
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            return True
        except can.CanError as e:
            rospy.logerr("CAN send error: %s", e)
            return False

    def encode_float_to_bytes(self, fval):
        return list(struct.pack('<f', float(fval)))

    # ---- CAN commands
    def send_alignment_request(self, leg_id):
        # alignment request: 0x000 + leg_id
        can_id = 0x000 + leg_id
        rospy.loginfo("[CAN] Alignment request leg=%d (ID=0x%03X)", leg_id, can_id)
        self._send_can_message(can_id, [0] * 8)

    def send_manual_home_command(self, leg_id, abs_pos):
        # manual home adjust: 0x200 + leg_id, payload: [0,0,0,0] + float32(abs_pos)
        can_id = 0x200 + leg_id
        data = [0, 0, 0, 0] + self.encode_float_to_bytes(abs_pos)
        rospy.loginfo("[CAN] Manual home leg=%d abs_pos=%.6f (ID=0x%03X)", leg_id, abs_pos, can_id)
        self._send_can_message(can_id, data)

    def send_set_home_command(self, leg_id):
        # set home: 0x300 + leg_id
        can_id = 0x300 + leg_id
        rospy.loginfo("[CAN] Set home leg=%d (ID=0x%03X)", leg_id, can_id)
        self._send_can_message(can_id, [0] * 8)

    def send_run_start_command(self):
        # run start: 0x600 + active joint id
        active = self._active_joint_ids()
        rospy.loginfo("[CAN] Run start active_joints=%s", active)
        for i in active:
            self._send_can_message(0x600 + i, [0] * 8)

    def send_position_command_active(self, positions):
        """
        positions: list/tuple length == NUM_LEGS
        send: 0x400+i, payload: [0,0,0,0] + float32(pos[i]) only for active_joints
        """
        if positions is None or len(positions) != NUM_LEGS:
            rospy.logwarn("[CAN] Position command length invalid (ID=0x400+i)")
            return
        for i in self._active_joint_ids():
            can_id = 0x400 + i
            data = [0, 0, 0, 0] + self.encode_float_to_bytes(positions[i])
            self._send_can_message(can_id, data)

    # =========================================================
    # UI <-> StateMachine
    # =========================================================
    def publish_leg_status(self, leg_id, state_str):
        self.status_pub.publish("{},{}".format(leg_id, state_str))

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

        # RUN
        if s == "run":
            self.handle_run_request()
            return

        # STOP
        if s == "stop":
            self.handle_stop_request("ui_stop")
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

        # align:<leg>
        if s.startswith("align:"):
            try:
                leg_id = int(s.split(":", 1)[1])
            except:
                rospy.logwarn("[UI] invalid align command: %s", s)
                return
            if 0 <= leg_id < NUM_LEGS:
                if not self._is_active(leg_id):
                    rospy.logwarn("[ALIGN] leg=%d is Use=False -> ignore align", leg_id)
                    return
                self.send_alignment_request(leg_id)
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

    def handle_use_selection(self, leg_id, active):
        if active:
            self.active_joints.add(leg_id)
            rospy.loginfo("[UI] Use=True leg=%d active_joints=%s", leg_id, self._active_joint_ids())
        else:
            if leg_id in self.active_joints:
                self.active_joints.remove(leg_id)
            rospy.loginfo("[UI] Use=False leg=%d active_joints=%s", leg_id, self._active_joint_ids())
            if self.is_run and not self.active_joints:
                self.handle_stop_request("active_joints_empty")

    def _active_joint_ids(self):
        return sorted([i for i in self.active_joints if 0 <= i < NUM_LEGS])

    def _inactive_joint_ids(self):
        active = set(self._active_joint_ids())
        return [i for i in range(NUM_LEGS) if i not in active]

    def _is_active(self, leg_id):
        return leg_id in self.active_joints

    def _publish_all_leg_status(self):
        for leg in self.legs:
            self.publish_leg_status(leg.leg_id, leg.state_str)

    def _run_blocking_reasons(self):
        reasons = []
        active = self._active_joint_ids()
        if not active:
            return ["no_active_joints"]
        for leg_id in active:
            leg = self.legs[leg_id]
            missing = []
            if not leg.connected:
                missing.append("connected")
            if not leg.aligned:
                missing.append("aligned")
            if not leg.homed:
                missing.append("homed")
            if missing:
                reasons.append("leg%d_missing_%s" % (leg.leg_id, "+".join(missing)))
        return reasons

    def handle_run_request(self):
        reasons = self._run_blocking_reasons()
        if reasons:
            self.is_run = False
            rospy.logwarn("[UI] RUN rejected: %s", ",".join(reasons))
            self._publish_all_leg_status()
            return False
        self.is_run = True
        rospy.loginfo("[UI] RUN accepted active_joints=%s -> is_run=True", self._active_joint_ids())
        self.send_run_start_command()
        self._publish_all_leg_status()
        return True

    def handle_stop_request(self, reason="stop"):
        if self.is_run:
            rospy.logwarn("[UI] STOP requested (%s) -> is_run=False", reason)
        else:
            rospy.loginfo("[UI] STOP requested (%s) while already stopped", reason)
        self.is_run = False
        self._publish_all_leg_status()
        return True

    def _position_limit_violation(self, positions):
        for i in self._active_joint_ids():
            joint_index = i % 3
            lo, hi = JOINT_LIMITS_RAD[joint_index]
            try:
                v = float(positions[i])
            except Exception:
                return "position[%d] is not numeric" % i
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

        if not self._is_active(leg_id):
            rospy.logwarn("[HOME] leg=%d is Use=False -> ignore home_move", leg_id)
            return

        if not leg.aligned:
            rospy.logwarn("[HOME] leg=%d is not Aligned -> ignore home_move", leg_id)
            return

        leg.home_pos += float(direction) * float(self.home_step)
        self.send_manual_home_command(leg_id, leg.home_pos)

    def handle_set_home(self, leg_id):
        leg = self.legs[leg_id]
        if not self._is_active(leg_id):
            rospy.logwarn("[HOME] leg=%d is Use=False -> ignore set_home", leg_id)
            return
        if not leg.aligned:
            rospy.logwarn("[HOME] leg=%d is not Aligned -> ignore set_home", leg_id)
            return

        self.send_set_home_command(leg_id)
        leg.homed = True
        self.set_leg_state(leg_id, "Homed")

    # =========================================================
    # External position commands (RUN mode)
    # =========================================================
    def coordinate_callback(self, msg):
        """
        /cmdForJetson (JointState)
        RUN中はUse=Trueのactive_jointsだけに送信する。positionは従来通り24要素必須。
        """
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
        violation = self._position_limit_violation(pos)
        if violation:
            rospy.logerr("[RUN] hardware_limit_v2 reject: %s", violation)
            return
        inactive_warnings = self._inactive_position_limit_warnings(pos)
        if inactive_warnings:
            rospy.logwarn("[RUN] inactive joint values outside hardware_limit_v2 ignored: %s", ";".join(inactive_warnings[:8]))
        self.send_position_command_active(pos)

    # =========================================================
    # CAN receive
    # =========================================================
    def can_callback(self, msg):
        """
        CAN受信をここで処理する
        - 0x0FF: 接続通知（data[0]=leg_id）
        - 0x100+leg: アライメント結果（data[7]==1 success）
        """
        if msg is None:
            return

        arb = msg.arbitration_id

        # Connection ping (0x0FF)
        if arb == 0x0FF and len(msg.data) >= 1:
            leg_id = int(msg.data[0])
            if 0 <= leg_id < NUM_LEGS:
                self.handle_connection_ping(leg_id)
            return

        # Alignment result (0x100 + leg)
        if 0x100 <= arb < 0x100 + NUM_LEGS and len(msg.data) >= 8:
            leg_id = arb - 0x100
            result = int(msg.data[7])
            self.handle_alignment_result(leg_id, result)
            return

        # ほか必要なら追加（0x500など）
        return

    def handle_connection_ping(self, leg_id):
        now = time.time()
        leg = self.legs[leg_id]
        was_connected = leg.connected

        leg.connected = True
        leg.last_seen = now

        # 未接続→接続の瞬間に通知（UIでUse自動ONになる）
        if not was_connected:
            rospy.loginfo("[CONN] leg=%d Connected", leg_id)
            # 接続したら基本状態はConnected（aligned/homedは未確定扱い）
            leg.aligned = False
            leg.homed = False
            leg.home_pos = 0.0
            self.set_leg_state(leg_id, "Connected")

    def handle_alignment_result(self, leg_id, success_flag):
        leg = self.legs[leg_id]
        if success_flag == 1:
            rospy.loginfo("[ALIGN] leg=%d success", leg_id)
            leg.aligned = True
            self.set_leg_state(leg_id, "Aligned")
        else:
            rospy.logwarn("[ALIGN] leg=%d FAILED (MCU may reboot)", leg_id)
            leg.aligned = False
            leg.homed = False
            leg.home_pos = 0.0
            # 失敗時はUI表示をConnectedに戻す（MCUが再起動して0x0FF送ってくる想定でもOK）
            # ここをDisconnectedにするかは運用次第。今回は「操作上再ALIGNしやすい」方針でConnectedへ。
            if leg.connected:
                self.set_leg_state(leg_id, "Connected")

    # =========================================================
    # Periodic
    # =========================================================
    def execute(self):
        """
        main.py から周期呼び出しされる想定
        - 接続タイムアウト監視（3秒ルール）
        """
        now = time.time()
        for leg in self.legs:
            if leg.connected and (now - leg.last_seen) > CONNECT_TIMEOUT_SEC:
                rospy.logwarn("[CONN] leg=%d timeout -> Disconnected", leg.leg_id)
                was_active = self._is_active(leg.leg_id)
                leg.connected = False
                leg.aligned = False
                leg.homed = False
                leg.home_pos = 0.0
                self.set_leg_state(leg.leg_id, "Disconnected")
                if self.is_run and was_active:
                    self.handle_stop_request("active_connection_timeout_leg_%d" % leg.leg_id)

        # Periodically republish state for UI recovery.
        self._publish_all_leg_status()
