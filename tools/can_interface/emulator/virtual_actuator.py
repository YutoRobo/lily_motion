# -*- coding: utf-8 -*-
"""Hardware-independent model of one actuator MCU."""
from __future__ import print_function

import struct
import time

try:
    from .scenario import ActuatorScenario
except (ImportError, ValueError):
    from scenario import ActuatorScenario


START = 0
ALIMENT_STANDBY = 1
ALIMENT = 2
GET_HOME = 3
RUN_STANDBY = 4
RUN = 5
SERVO_OFF = 6
ALIMENT_ERROR = 7
ERROR = 8

STATE_NAMES = {
    START: "start",
    ALIMENT_STANDBY: "aliment_standby",
    ALIMENT: "aliment",
    GET_HOME: "get_home",
    RUN_STANDBY: "run_standby",
    RUN: "run",
    SERVO_OFF: "servo_off",
    ALIMENT_ERROR: "aliment_error",
    ERROR: "error",
}


class FrameRecord(object):
    def __init__(self, timestamp, arbitration_id, data, action, state,
                 position_rad=None):
        self.timestamp = float(timestamp)
        self.arbitration_id = int(arbitration_id)
        self.data = tuple(int(v) for v in data)
        self.action = str(action)
        self.state = int(state)
        self.position_rad = position_rad


class TransitionRecord(object):
    def __init__(self, timestamp, old_state, new_state, reason):
        self.timestamp = float(timestamp)
        self.old_state = int(old_state)
        self.new_state = int(new_state)
        self.reason = str(reason)


class VirtualActuator(object):
    def __init__(self, axis, transmit, scenario=None, heartbeat_period=1.0,
                 align_delay=0.25, reset_delay=0.5, now=None, logger=None):
        if not 0 <= int(axis) <= 23:
            raise ValueError("axis must be in 0..23")
        self.axis = int(axis)
        self.transmit = transmit
        self.scenario = scenario or ActuatorScenario()
        self.heartbeat_period = float(heartbeat_period)
        self.align_delay = float(align_delay)
        self.reset_delay = float(reset_delay)
        if self.heartbeat_period <= 0.0:
            raise ValueError("heartbeat_period must be positive")
        if self.align_delay < 0.0 or self.reset_delay < 0.0:
            raise ValueError("delays must be non-negative")
        self.logger = logger
        self.low_freq_state = START
        self.last_position_command_rad = None
        self.align_attempt_count = 0
        self.heartbeat_enabled = False
        self.current_error_id = 0
        self.reset_count = 0
        self.received_frame_history = []
        self.transmitted_frame_history = []
        self.state_transition_history = []
        self.align_due = None
        self.reset_due = None
        self.run_reset_due = None
        self.next_heartbeat_due = None
        self.start(time.time() if now is None else now)

    @property
    def state_name(self):
        return STATE_NAMES[self.low_freq_state]

    def _log(self, message):
        line = "axis=%d %s" % (self.axis, message)
        if self.logger is not None:
            self.logger(line)

    def _transition(self, new_state, now, reason):
        old = self.low_freq_state
        if old == new_state:
            return
        self.low_freq_state = int(new_state)
        self.state_transition_history.append(
            TransitionRecord(now, old, new_state, reason))
        self._log("state %s -> %s reason=%s" % (
            STATE_NAMES[old], STATE_NAMES[new_state], reason))
        self._set_heartbeat(new_state == ALIMENT_STANDBY, now)

    def _set_heartbeat(self, enabled, now):
        enabled = bool(enabled)
        if enabled == self.heartbeat_enabled:
            return
        self.heartbeat_enabled = enabled
        if enabled:
            self.next_heartbeat_due = float(now)
            self._log("heartbeat start")
        else:
            self.next_heartbeat_due = None
            self._log("heartbeat stop")

    def start(self, now):
        self._transition(ALIMENT_STANDBY, now, "boot_complete")

    def _tx(self, arbitration_id, data, action, now):
        payload = list(data)
        self.transmit(arbitration_id, payload)
        self.transmitted_frame_history.append(
            FrameRecord(now, arbitration_id, payload, action,
                        self.low_freq_state))
        self._log("TX id=0x%03X %s" % (arbitration_id, action))

    def _record_rx(self, arbitration_id, data, action, now,
                   position_rad=None):
        self.received_frame_history.append(
            FrameRecord(now, arbitration_id, data, action,
                        self.low_freq_state, position_rad))
        suffix = ""
        if position_rad is not None:
            suffix = " position=%.6f" % position_rad
        self._log("RX id=0x%03X%s %s state=%s" % (
            arbitration_id, suffix, action, self.state_name))

    def _decode_position(self, data):
        if len(data) != 8 or list(data[:4]) != [0, 0, 0, 0]:
            return None
        return struct.unpack("<f", struct.pack("4B", *data[4:8]))[0]

    def handles_id(self, arbitration_id):
        return arbitration_id in (
            self.axis,
            0x200 | self.axis,
            0x300 | self.axis,
            0x400 | self.axis,
            0x600 | self.axis)

    def receive(self, arbitration_id, data, now=None):
        now = time.time() if now is None else float(now)
        arbitration_id = int(arbitration_id)
        data = list(data)
        if not self.handles_id(arbitration_id):
            return False

        if arbitration_id == self.axis:
            if len(data) != 8:
                self._record_rx(arbitration_id, data, "ignored_bad_dlc", now)
            elif self.low_freq_state != ALIMENT_STANDBY:
                self._record_rx(
                    arbitration_id, data, "align_ignored_wrong_state", now)
            else:
                self._record_rx(arbitration_id, data, "align_accepted", now)
                self.align_attempt_count += 1
                self._transition(ALIMENT, now, "align_start")
                self.align_due = now + self.align_delay
            return True

        if arbitration_id == (0x200 | self.axis):
            position = self._decode_position(data)
            if position is None:
                self._record_rx(arbitration_id, data, "ignored_bad_payload", now)
            elif self.low_freq_state == GET_HOME:
                self.last_position_command_rad = position
                self._record_rx(
                    arbitration_id, data, "home_position_accepted", now,
                    position)
            else:
                self._record_rx(
                    arbitration_id, data, "home_position_ignored", now,
                    position)
            return True

        if arbitration_id == (0x300 | self.axis):
            if len(data) == 8 and self.low_freq_state == GET_HOME:
                self._record_rx(
                    arbitration_id, data, "home_complete_accepted", now)
                self._transition(RUN_STANDBY, now, "home_complete")
            else:
                self._record_rx(
                    arbitration_id, data, "home_complete_ignored", now)
            return True

        if arbitration_id == (0x600 | self.axis):
            if len(data) == 8 and self.low_freq_state == RUN_STANDBY:
                self._record_rx(arbitration_id, data, "run_accepted", now)
                self._transition(RUN, now, "run_start")
                if self.scenario.reset_after_run_sec is not None:
                    self.run_reset_due = (
                        now + self.scenario.reset_after_run_sec)
                    self._log("reset after run scheduled")
            else:
                self._record_rx(arbitration_id, data, "run_ignored", now)
            return True

        if arbitration_id == (0x400 | self.axis):
            position = self._decode_position(data)
            accepted = (position is not None
                        and self.low_freq_state in (GET_HOME, RUN))
            if accepted:
                self.last_position_command_rad = position
                action = "position_accepted"
            else:
                action = ("ignored_bad_payload" if position is None
                          else "position_ignored")
            self._record_rx(
                arbitration_id, data, action, now, position)
            return True
        return False

    def _finish_alignment(self, now):
        self.align_due = None
        if self.scenario.alignment_should_fail(self.align_attempt_count):
            error_id = self.scenario.initialization_error_id
            self.current_error_id = error_id
            self._transition(ALIMENT_ERROR, now, "align_failure")
            payload = [
                self.axis, ALIMENT_ERROR, 0, 0, 0, 0, 0, error_id]
            self._tx(0x0EE, payload, "align_error_%d" % error_id, now)
            self.reset_due = now + self.reset_delay
            return
        self.current_error_id = 0
        self._transition(GET_HOME, now, "align_success")
        self._tx(0x100 | self.axis, [0, 0, 0, 0, 0, 0, 0, 1],
                 "align_success", now)

    def reset(self, now, reason="injected_reset"):
        self.reset_count += 1
        self.align_due = None
        self.reset_due = None
        self.run_reset_due = None
        self.current_error_id = 0
        self._transition(START, now, reason)
        self._transition(ALIMENT_STANDBY, now, "reset_boot_complete")
        self._log("reset count=%d" % self.reset_count)

    def inject_error(self, error_id, now=None):
        now = time.time() if now is None else float(now)
        error_id = int(error_id)
        if not 0 <= error_id <= 255:
            raise ValueError("error_id must be in 0..255")
        self.current_error_id = error_id
        payload = [
            self.axis, self.low_freq_state, 0, 0, 0, 0, 0, error_id]
        self._tx(0x0EE, payload, "injected_error_%d" % error_id, now)
        if error_id not in (0, 8, 9, 12):
            self._transition(ERROR, now, "runtime_error_%d" % error_id)

    def tick(self, now=None):
        now = time.time() if now is None else float(now)
        if self.align_due is not None and now >= self.align_due:
            self._finish_alignment(now)
        if self.reset_due is not None and now >= self.reset_due:
            self.reset(now, "align_failure_reset")
        if self.run_reset_due is not None and now >= self.run_reset_due:
            self.reset(now, "reset_after_run")
        if self.heartbeat_enabled:
            while (self.next_heartbeat_due is not None
                   and now >= self.next_heartbeat_due):
                due = self.next_heartbeat_due
                self._tx(
                    0x0FF, [self.axis, 0, 0, 0, 0, 0, 0, 0],
                    "heartbeat", due)
                self.next_heartbeat_due += self.heartbeat_period

    def summary(self):
        position = ("unknown" if self.last_position_command_rad is None
                    else "%.6f" % self.last_position_command_rad)
        return ("axis=%d state=%s last_position=%s align_attempts=%d "
                "resets=%d" % (
                    self.axis, self.state_name, position,
                    self.align_attempt_count, self.reset_count))
