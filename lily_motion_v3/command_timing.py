# -*- coding: utf-8 -*-
"""Shared command timing utilities for hardware and Gazebo replay.

The frozen command log is the single source trajectory.  This module separates
three timing layers without hard-coding the current MCU values:

1. source/keyframe timing (represented by the frozen JSONL order),
2. transport target timing (for example Jetson -> MCU),
3. actuator interpolation timing (MCU-side target interpolation).

Both Python 2.7 and Python 3 are supported.
"""
from __future__ import division, print_function

import math

from lily_motion_v3.command_resampler import resample_command_records


def linear_interpolate_command(command0, command1, alpha):
    """Linearly interpolate two equal-length command vectors."""
    if len(command0) != len(command1):
        raise ValueError('command lengths differ: %d != %d' % (
            len(command0), len(command1)))
    alpha = float(alpha)
    return [
        float(a) + (float(b) - float(a)) * alpha
        for a, b in zip(command0, command1)
    ]


def resample_transport_records(records, factor=1, segment_key=None):
    """Build the transport target stream from source command records.

    This deliberately delegates to the project's existing command resampler so
    Gazebo and the hardware publisher use one transport-resampling definition.
    """
    factor = int(factor)
    if factor < 1:
        raise ValueError('factor must be >= 1')
    return resample_command_records(
        records, factor=factor, segment_key=segment_key)


def timing_relationship(target_period_sec, interpolation_duration_sec,
                        tolerance_sec=1e-9):
    """Classify transport cadence versus actuator interpolation duration."""
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


def _validate_actuator_timing(target_period_sec, interpolation_duration_sec,
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
    """Emulate the current MCU's target-to-target linear interpolation.

    Target records arrive every ``target_period_sec``.  On each arrival the
    current MCU semantics are modeled as::

        previous_target = old_target
        target = new_target
        interpolation restarts from alpha=0

    The actuator command is sampled every ``update_period_sec``.  Importantly,
    no assumption is made that the periods are equal or integer multiples.
    Therefore this function can also expose the behavior when a new target
    arrives before the previous interpolation has completed.

    If ``initial_command`` is omitted, the first target is assumed to already be
    the held actuator command.  This matches staged replay that starts from a
    known pose (HOME for air-entry, or the held air-entry endpoint for roll).
    """
    target_period_sec, interpolation_duration_sec, update_period_sec = \
        _validate_actuator_timing(
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

    # Include the final settled sample even when periods are not integer ratios.
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
        source_record['joint_command_deg'] = [
            math.degrees(v) for v in command]
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

    # If the fixed sampling grid did not land on the exact final settle time,
    # append one exact endpoint sample.  This keeps the final target explicit.
    if output:
        last_time = float(output[-1]['actuator_time_sec'])
        if final_time - last_time > eps:
            final_record = dict(targets[-1])
            final_command = [float(v) for v in targets[-1]['joint_command_rad']]
            final_record['joint_command_rad'] = final_command
            final_record['joint_command_deg'] = [
                math.degrees(v) for v in final_command]
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
