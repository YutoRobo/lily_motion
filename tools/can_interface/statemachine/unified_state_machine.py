# -*- coding: utf-8 -*-
"""Production StateMachine with /cmdForJetson as the only position input.

The legacy StateMachine still contains the historical /can/axis_command
subscriber for compatibility with old tests and tools. This production
subclass disables that position path and makes normal RUN fan-out obey the
UI Use selection.
"""
from __future__ import division

import math
import rospy

from state_machine import (JOINT_LIMITS_RAD, JOINT_NAMES, NUM_LEGS,
                           StateMachine as LegacyStateMachine)


class StateMachine(LegacyStateMachine):
    """Use-filtered production StateMachine.

    Position commands enter only through /cmdForJetson as a 24-element
    sensor_msgs/JointState. Use=True determines which axes receive RUN and
    POS_SET CAN frames. Use=False axes retain stable indexes in the incoming
    24-element vector but receive no RUN or position CAN traffic.
    """

    def __init__(self, bus):
        LegacyStateMachine.__init__(self, bus)
        rospy.loginfo(
            "Unified command mode enabled: /cmdForJetson is the only "
            "external position input; CAN fan-out follows Use=True axes")

    def external_axis_command_callback(self, msg):
        """Reject the retired /can/axis_command position interface."""
        text = getattr(msg, "data", "")
        rospy.logwarn(
            "[DEPRECATED] /can/axis_command ignored (%s). "
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
