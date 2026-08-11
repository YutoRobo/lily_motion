# -*- coding: utf-8 -*-
"""Gazebo-only emulator of the actuator/MCU target interpolation stage.

The shared command publisher stops at ``/cmdForJetson``.  Real hardware consumes
that topic through the CAN interface and real MCU.  Gazebo consumes the same
topic through a separate emulator node that uses the pure logic in this module.

The interpolation duration and update period are parameters because MCU firmware
timing may change in the future.  Python 2.7 and Python 3 are supported.
"""
from __future__ import division, print_function

import math

from lily_motion_v3.command_timing import linear_interpolate_command


def timing_relationship(target_period_sec, interpolation_duration_sec,
                        tolerance_sec=1e-9):
    """Classify target cadence versus actuator interpolation duration."""
    target_period_sec = float(target_period_sec)
    interpolation_duration_sec = float(interpolation_duration_sec)
    if target_period_sec <= 0.0:
        raise ValueError('target_period_sec must be positive')
    if interpolation_duration_sec < 0.0:
        raise ValueError('interpolation_duration_sec must be >= 0')
    delta = target_period_sec - interpolation_duration_sec
    if abs(delta) <= float(tolerance_sec):
        return 'matched'
    if delta > 0.0:
        return 'hold_after_interpolation'
    return 'new_target_before_interpolation_complete'


def _validate_timing(target_period_sec, interpolation_duration_sec,
                     update_period_sec):
    target_period_sec = float(target_period_sec)
    interpolation_duration_sec = float(interpolation_duration_sec)
    update_period_sec = float(update_period_sec)
    if target_period_sec <= 0.0:
        raise ValueError('target_period_sec must be positive')
    if interpolation_duration_sec < 0.0:
        raise ValueError('interpolation_duration_sec must be >= 0')
    if update_period_sec <= 0.0:
        raise ValueError('update_period_sec must be positive')
    return target_period_sec, interpolation_duration_sec, update_period_sec


def _finite_vector(command, expected_length=None):
    values = [float(v) for v in command]
    if expected_length is not None and len(values) != int(expected_length):
        raise ValueError('command length %d != %d' % (
            len(values), int(expected_length)))
    for value in values:
        if value != value or abs(value) == float('inf'):
            raise ValueError('command contains NaN or inf')
    return values


class OnlineLinearActuatorInterpolator(object):
    """Stateful online emulator for the current MCU target interpolation.

    ``set_target`` models target arrival at the MCU boundary.  The current MCU
    semantics are intentionally preserved: when a new target arrives,
    ``previous_target`` becomes the *old target* (not the current interpolated
    output), the new target is stored, and interpolation restarts from alpha=0.

    The first received target is assumed to be the already-held starting pose.
    This matches the staged operating contract used here: air-entry starts at
    HOME, and roll starts from the held air-entry endpoint.

    ``command_at`` is time-based rather than call-count-based.  Therefore the
    Gazebo node may use ROS simulated time without assuming each timer callback
    occurs exactly on the requested update period.
    """
    def __init__(self, interpolation_duration_sec=0.100,
                 expected_length=24):
        interpolation_duration_sec = float(interpolation_duration_sec)
        if interpolation_duration_sec < 0.0:
            raise ValueError('interpolation_duration_sec must be >= 0')
        self.interpolation_duration_sec = interpolation_duration_sec
        self.expected_length = int(expected_length)
        if self.expected_length <= 0:
            raise ValueError('expected_length must be positive')
        self.reset()

    def reset(self):
        self.previous_target = None
        self.current_target = None
        self.target_set_time_sec = None
        self.target_count = 0

    def has_target(self):
        return self.current_target is not None

    def set_target(self, command, time_sec):
        command = _finite_vector(command, self.expected_length)
        time_sec = float(time_sec)
        if self.current_target is None:
            self.previous_target = list(command)
            self.current_target = list(command)
        else:
            # Match current MCU semantics: restart from the old target value.
            self.previous_target = list(self.current_target)
            self.current_target = list(command)
        self.target_set_time_sec = time_sec
        self.target_count += 1

    def interpolation_alpha_at(self, time_sec):
        if self.current_target is None:
            return None
        if self.interpolation_duration_sec <= 0.0:
            return 1.0
        elapsed = float(time_sec) - float(self.target_set_time_sec)
        return min(1.0, max(0.0, elapsed / self.interpolation_duration_sec))

    def command_at(self, time_sec):
        if self.current_target is None:
            return None
        alpha = self.interpolation_alpha_at(time_sec)
        return linear_interpolate_command(
            self.previous_target, self.current_target, alpha)


def simulate_linear_actuator_records(target_records, target_period_sec,
                                     interpolation_duration_sec,
                                     update_period_sec,
                                     initial_command=None):
    """Offline reference simulation using the same MCU interpolation semantics.

    This function is retained for deterministic tests and diagnostics.  The live
    Gazebo path uses :class:`OnlineLinearActuatorInterpolator` through a separate
    ROS node subscribed to ``/cmdForJetson``.
    """
    target_period_sec, interpolation_duration_sec, update_period_sec = \
        _validate_timing(
            target_period_sec, interpolation_duration_sec, update_period_sec)

    targets = [dict(record) for record in target_records]
    if not targets:
        return []
    for i, record in enumerate(targets):
        if 'joint_command_rad' not in record:
            raise ValueError('target record %d has no joint_command_rad' % i)

    first_command = _finite_vector(targets[0]['joint_command_rad'])
    if initial_command is None:
        previous_target = list(first_command)
        current_target = list(first_command)
    else:
        previous_target = _finite_vector(
            initial_command, expected_length=len(first_command))
        current_target = list(first_command)

    previous_target_index = None
    current_target_index = 0
    target_set_time = 0.0
    next_target_index = 1

    final_target_arrival = (len(targets) - 1) * target_period_sec
    final_time = final_target_arrival + interpolation_duration_sec
    if interpolation_duration_sec == 0.0:
        final_time = final_target_arrival

    output = []
    sample_index = 0
    t = 0.0
    eps = max(1e-12, update_period_sec * 1e-9)

    while t <= final_time + eps:
        while next_target_index < len(targets):
            arrival = next_target_index * target_period_sec
            if arrival > t + eps:
                break
            previous_target = list(current_target)
            previous_target_index = current_target_index
            current_target = _finite_vector(
                targets[next_target_index]['joint_command_rad'],
                expected_length=len(previous_target))
            current_target_index = next_target_index
            target_set_time = arrival
            next_target_index += 1

        if interpolation_duration_sec <= 0.0:
            alpha = 1.0
        else:
            alpha = (t - target_set_time) / interpolation_duration_sec
            alpha = min(1.0, max(0.0, alpha))

        command = linear_interpolate_command(
            previous_target, current_target, alpha)
        source_record = dict(targets[current_target_index])
        source_record['joint_command_rad'] = command
        source_record['joint_command_deg'] = [math.degrees(v) for v in command]
        source_record['actuator_sample_index'] = sample_index
        source_record['actuator_time_sec'] = t
        source_record['actuator_interpolation_alpha'] = alpha
        source_record['actuator_target_record_index'] = current_target_index
        source_record['actuator_previous_target_record_index'] = \
            previous_target_index
        source_record['actuator_target_period_sec'] = target_period_sec
        source_record['actuator_interpolation_duration_sec'] = \
            interpolation_duration_sec
        source_record['actuator_update_period_sec'] = update_period_sec
        output.append(source_record)

        sample_index += 1
        t = sample_index * update_period_sec

    if output:
        last_time = float(output[-1]['actuator_time_sec'])
        if final_time - last_time > eps:
            final_record = dict(targets[-1])
            final_command = _finite_vector(targets[-1]['joint_command_rad'])
            final_record['joint_command_rad'] = final_command
            final_record['joint_command_deg'] = [math.degrees(v) for v in final_command]
            final_record['actuator_sample_index'] = sample_index
            final_record['actuator_time_sec'] = final_time
            final_record['actuator_interpolation_alpha'] = 1.0
            final_record['actuator_target_record_index'] = len(targets) - 1
            final_record['actuator_previous_target_record_index'] = \
                len(targets) - 2 if len(targets) > 1 else None
            final_record['actuator_target_period_sec'] = target_period_sec
            final_record['actuator_interpolation_duration_sec'] = \
                interpolation_duration_sec
            final_record['actuator_update_period_sec'] = update_period_sec
            output.append(final_record)

    return output
