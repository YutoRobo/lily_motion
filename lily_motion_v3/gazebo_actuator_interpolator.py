# -*- coding: utf-8 -*-
"""Gazebo-only emulator of the actuator/MCU target interpolation stage.

The shared transport target stream stops before this module.  Real hardware
sends that stream to the MCU.  Gazebo inserts this emulator instead so the
simulation can approximate MCU-side interpolation without duplicating the
source/transport pipeline.

The timing parameters are deliberately configurable because MCU firmware timing
may change in the future.
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


def simulate_linear_actuator_records(target_records, target_period_sec,
                                     interpolation_duration_sec,
                                     update_period_sec,
                                     initial_command=None):
    """Emulate the current MCU target interpolation semantics for Gazebo.

    The current firmware updates ``previous_target`` from the old target when a
    new target arrives, assigns the new target, and restarts interpolation.  The
    emulator intentionally preserves that behavior even when a new target
    arrives before the previous interpolation duration has completed.

    No integer-ratio assumption is made between target period, interpolation
    duration, and internal update period.
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

    first_command = [float(v) for v in targets[0]['joint_command_rad']]
    if initial_command is None:
        previous_target = list(first_command)
        current_target = list(first_command)
    else:
        previous_target = [float(v) for v in initial_command]
        if len(previous_target) != len(first_command):
            raise ValueError('initial_command length does not match target')
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
            current_target = [
                float(v) for v in
                targets[next_target_index]['joint_command_rad']
            ]
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
            final_command = [float(v) for v in targets[-1]['joint_command_rad']]
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
