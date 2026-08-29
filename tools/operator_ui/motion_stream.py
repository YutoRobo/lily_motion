# -*- coding: utf-8 -*-
from __future__ import division, print_function

import math
import os

from lily_motion_v3.command_stream import (
    DEFAULT_POSITION_LENGTH,
    prepare_transport_stream,
    transport_stream_sha256,
)

POSITION_LENGTH = DEFAULT_POSITION_LENGTH
MAX_COMMAND_JUMP_RAD = math.radians(4.0)

JOINT_LIMITS_RAD = [
    (-6.283185307179586, 6.283185307179586),
    (-1.6580627893946132, 1.6580627893946132),
    (-2.6179938779914944, 2.6179938779914944),
]


class MotionStreamError(ValueError):
    pass


def _is_finite(value):
    return not (math.isnan(value) or math.isinf(value))


def _as_position(record, record_index):
    try:
        values = list(record['joint_command_rad'])
    except Exception:
        raise MotionStreamError(
            'transport frame %d has no joint_command_rad' % record_index)
    if len(values) != POSITION_LENGTH:
        raise MotionStreamError(
            'transport frame %d position length=%d expected=%d' % (
                record_index, len(values), POSITION_LENGTH))
    converted = []
    for axis, raw in enumerate(values):
        try:
            value = float(raw)
        except Exception:
            raise MotionStreamError(
                'transport frame %d axis%d is not numeric' % (
                    record_index, axis))
        if not _is_finite(value):
            raise MotionStreamError(
                'transport frame %d axis%d is non-finite' % (
                    record_index, axis))
        lo, hi = JOINT_LIMITS_RAD[axis % 3]
        if value < lo or value > hi:
            raise MotionStreamError(
                'transport frame %d axis%d %.6f rad outside [%.6f, %.6f]' % (
                    record_index, axis, value, lo, hi))
        converted.append(value)
    return converted


def _max_step(positions):
    max_delta = 0.0
    max_axis = None
    max_frame = None
    for frame_index in range(1, len(positions)):
        before = positions[frame_index - 1]
        after = positions[frame_index]
        for axis in range(POSITION_LENGTH):
            delta = abs(after[axis] - before[axis])
            if delta > max_delta:
                max_delta = delta
                max_axis = axis
                max_frame = frame_index
    return max_delta, max_axis, max_frame


def load_motion_stream(path, resample_factor):
    path = os.path.abspath(path)
    if not os.path.isfile(path):
        raise MotionStreamError('file does not exist: %s' % path)
    try:
        resample_factor = int(resample_factor)
    except Exception:
        raise MotionStreamError('resample_factor must be an integer')
    if resample_factor < 1:
        raise MotionStreamError('resample_factor must be >= 1')

    try:
        source_records, transport_records = prepare_transport_stream(
            path, resample_factor=resample_factor)
    except Exception as exc:
        raise MotionStreamError('failed to build transport stream: %s' % exc)

    if not source_records:
        raise MotionStreamError('source JSONL has no command frames')
    if not transport_records:
        raise MotionStreamError('transport stream is empty')

    positions = [
        _as_position(record, index)
        for index, record in enumerate(transport_records)
    ]
    max_step_rad, max_step_axis, max_step_frame = _max_step(positions)
    if max_step_rad >= MAX_COMMAND_JUMP_RAD:
        raise MotionStreamError(
            'transport jump %.6f rad (%.3f deg) at frame %s axis%s '
            'is >= 4 deg' % (
                max_step_rad, math.degrees(max_step_rad),
                max_step_frame, max_step_axis))

    return {
        'path': path,
        'source_frame_count': len(source_records),
        'transport_frame_count': len(transport_records),
        'resample_factor': resample_factor,
        'transport_sha256': transport_stream_sha256(transport_records),
        'transport_records': transport_records,
        'positions': positions,
        'first_position': list(positions[0]),
        'last_position': list(positions[-1]),
        'max_step_rad': max_step_rad,
        'max_step_axis': max_step_axis,
        'max_step_frame': max_step_frame,
    }


def continuity(reference_position, first_position):
    if reference_position is None:
        return None
    if len(reference_position) != POSITION_LENGTH:
        raise MotionStreamError(
            'continuity reference length=%d expected=%d' % (
                len(reference_position), POSITION_LENGTH))
    if len(first_position) != POSITION_LENGTH:
        raise MotionStreamError(
            'first position length=%d expected=%d' % (
                len(first_position), POSITION_LENGTH))

    max_delta = -1.0
    max_axis = None
    for axis in range(POSITION_LENGTH):
        try:
            before = float(reference_position[axis])
            after = float(first_position[axis])
        except Exception:
            raise MotionStreamError('continuity value is not numeric')
        if not _is_finite(before) or not _is_finite(after):
            raise MotionStreamError('continuity value is non-finite')
        delta = abs(after - before)
        if delta > max_delta:
            max_delta = delta
            max_axis = axis
    return {
        'max_delta_rad': max_delta,
        'max_delta_deg': math.degrees(max_delta),
        'axis': max_axis,
        'pass': max_delta < MAX_COMMAND_JUMP_RAD,
    }
