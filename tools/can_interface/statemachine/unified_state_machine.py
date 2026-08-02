# -*- coding: utf-8 -*-
"""Production StateMachine with /cmdForJetson as the sole position input."""
from __future__ import division

import math
import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from state_machine import (
    DEFAULT_HOME_STEP,
    JOINT_LIMITS_RAD,
    JOINT_NAMES,
    NUM_LEGS,
    LegInfo,
    StateMachine as LegacyStateMachine,
)
from motion_check import DEFAULT_MOTION_CHECK_CONFIG


class StateMachine(LegacyStateMachine):
    """Use-filtered production StateMachine.

    Position commands enter only through /cmdForJetson as a 24-element
    sensor_msgs/JointState. Use=True determines which axes receive RUN and
    POS_SET CAN frames. Use=False axes retain stable indexes in the incoming
    vector but receive no RUN or position CAN traffic.

    The historical /can/axis_command subscriber is intentionally not created.
    """

    def __init__(self, bus):
        self.bus = bus

        self.legs = [LegInfo(i) for i in range(NUM_LEGS)]
        self.active_joints = set()

        self.is_run = False
        self.can_interface_ok = True
        self.send_error_latched = False
        self.error_latched = False
        self.global_error_details = []
        self.alignment_generation = 0
        self.stop_in_progress = False

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

        self.home_step = DEFAULT_HOME_STEP

        self.status_pub = rospy.Publisher(
            "/ui/leg_status", String, queue_size=50)
        self.use_status_pub = rospy.Publisher(
            "/ui/leg_use_status", String, queue_size=50)
        self.motion_check_status_pub = rospy.Publisher(
            "/ui/motion_check_status", String, queue_size=20)
        self.diagnostic_targets_pub = rospy.Publisher(
            "/ui/diagnostic_targets", String, queue_size=20)
        self.diagnostic_status_pub = rospy.Publisher(
            "/ui/diagnostic_status", String, queue_size=20)

        rospy.Subscriber(
            "/ui/leg_command", String, self.ui_command_callback)
        rospy.Subscriber(
            "/cmdForJetson", JointState, self.coordinate_callback)

        rospy.loginfo(
            "Unified command mode enabled: position input=/cmdForJetson; "
            "CAN fan-out follows Use=True axes")

    def external_axis_command_callback(self, msg):
        """Reject direct calls to the retired external position callback."""
        text = getattr(msg, "data", "")
        rospy.logwarn(
            "[DEPRECATED] external axis command ignored (%s). "
            "Publish a 24-element JointState to /cmdForJetson instead.",
            text)
        return False

    def send_run_start_command(self):
        """Send RUN only to Use=True axes."""
        active = self._active_joint_ids()
        if not active:
            rospy.logwarn("[CAN] RUN start rejected: no active joints")
            return False
        rospy.loginfo("[CAN] Run start active_joints=%s", active)
        ok = True
        for axis in active:
            ok = self._send_can_message(0x600 + axis, [0] * 8) and ok
        return ok

    def send_position_command_all(self, positions):
        """Send a 24-element logical command only to Use=True axes."""
        if positions is None or len(positions) != NUM_LEGS:
            rospy.logwarn("[CAN] Position command length invalid (expected 24)")
            return False
        active = self._active_joint_ids()
        if not active:
            rospy.logwarn("[CAN] Position command ignored: no active joints")
            return False
        ok = True
        for axis in active:
            value = float(positions[axis])
            sent = self._send_can_message(
                0x400 + axis, self._position_command_data(value))
            if sent:
                self.legs[axis].last_logical_position_command_rad = value
            ok = sent and ok
        return ok

    def _position_limit_violation_all(self, positions):
        """Validate only axes that will actually receive CAN position frames."""
        for axis in self._active_joint_ids():
            joint_index = axis % 3
            lo, hi = JOINT_LIMITS_RAD[joint_index]
            try:
                value = float(positions[axis])
            except Exception:
                return "position[%d] is not numeric" % axis
            if math.isnan(value) or math.isinf(value):
                return "position[%d] is non-finite" % axis
            if value < lo or value > hi:
                return (
                    "position[%d] %s %.6f rad outside [%.6f, %.6f]" %
                    (axis, JOINT_NAMES[joint_index], value, lo, hi))
        return None
