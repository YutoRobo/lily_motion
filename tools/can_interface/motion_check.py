# -*- coding: utf-8 -*-
"""Pure, hardware-independent sequence generation for RUN Motion Check."""
from __future__ import division

import math


class MotionCheckConfig(object):
    def __init__(self, step_rad=0.005, amplitude_rad=0.020,
                 command_period_sec=0.100, end_hold_sec=0.500,
                 repeat_count=1):
        self.step_rad = float(step_rad)
        self.amplitude_rad = float(amplitude_rad)
        self.command_period_sec = float(command_period_sec)
        self.end_hold_sec = float(end_hold_sec)
        self.repeat_count = int(repeat_count)
        self._validate()

    def _validate(self):
        values = (self.step_rad, self.amplitude_rad,
                  self.command_period_sec, self.end_hold_sec)
        if any(math.isnan(v) or math.isinf(v) for v in values):
            raise ValueError("motion check config must be finite")
        if (self.step_rad <= 0.0 or self.amplitude_rad <= 0.0
                or self.command_period_sec <= 0.0
                or self.end_hold_sec < 0.0
                or self.repeat_count != 1):
            raise ValueError("invalid motion check config")
        steps = self.amplitude_rad / self.step_rad
        if abs(steps - round(steps)) > 1e-9:
            raise ValueError("amplitude must be an exact step multiple")

    @property
    def speed_rad_per_sec(self):
        return self.step_rad / self.command_period_sec


DEFAULT_MOTION_CHECK_CONFIG = MotionCheckConfig()


def build_motion_values(q0, direction, config=None):
    """Return outward and return commands, excluding the initial q0."""
    config = config or DEFAULT_MOTION_CHECK_CONFIG
    q0 = float(q0)
    if math.isnan(q0) or math.isinf(q0):
        raise ValueError("q0 must be finite")
    if direction not in (-1, 1):
        raise ValueError("direction must be -1 or +1")
    count = int(round(config.amplitude_rad / config.step_rad))
    outward = [
        q0 + direction * config.step_rad * index
        for index in range(1, count + 1)]
    returning = [
        q0 + direction * config.step_rad * index
        for index in range(count - 1, -1, -1)]
    return outward + returning


def build_timed_commands(q0, direction, start_time, config=None):
    """Return (due_time, value) pairs with a hold at maximum amplitude."""
    config = config or DEFAULT_MOTION_CHECK_CONFIG
    values = build_motion_values(q0, direction, config)
    count = int(round(config.amplitude_rad / config.step_rad))
    commands = []
    due = float(start_time) + config.command_period_sec
    for index, value in enumerate(values):
        if index == count:
            due += config.end_hold_sec
        commands.append((due, value))
        due += config.command_period_sec
    complete_time = commands[-1][0] + config.end_hold_sec
    return commands, complete_time


def build_return_values(current, q0, config=None):
    """Return safe step-sized values from current command back to q0."""
    config = config or DEFAULT_MOTION_CHECK_CONFIG
    current = float(current)
    q0 = float(q0)
    if any(math.isnan(v) or math.isinf(v) for v in (current, q0)):
        raise ValueError("return positions must be finite")
    delta = q0 - current
    if abs(delta) <= 1e-12:
        return []
    direction = 1 if delta > 0.0 else -1
    values = []
    value = current
    while abs(q0 - value) > config.step_rad + 1e-9:
        value += direction * config.step_rad
        values.append(value)
    if abs(q0 - value) > 1e-12:
        values.append(q0)
    return values
