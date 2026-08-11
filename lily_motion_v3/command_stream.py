# -*- coding: utf-8 -*-
"""Single shared source->transport command stream for Gazebo and hardware.

This module is intentionally unaware of ROS, CAN, Gazebo, and MCU internals.
Both execution backends consume the exact same transport target records built
here.  Backend-specific behavior begins only after ``prepare_transport_stream``.

Python 2.7 and Python 3 are supported.
"""
from __future__ import division, print_function

import hashlib
import json

from lily_motion_v3.command_timing import resample_transport_records


POSITION_KEYS = ('joint_command_rad', 'position', 'joint_positions_rad')
DEFAULT_POSITION_LENGTH = 24


def extract_position(record, record_index, position_length=DEFAULT_POSITION_LENGTH):
    """Return a normalized floating-point joint vector and its source key."""
    for key in POSITION_KEYS:
        if key not in record:
            continue
        pos = record[key]
        if not isinstance(pos, list):
            raise ValueError('record %d key %s is not a list' % (
                record_index, key))
        if len(pos) != int(position_length):
            raise ValueError('record %d key %s length %d != %d' % (
                record_index, key, len(pos), int(position_length)))
        return [float(v) for v in pos], key
    raise ValueError('record %d has none of %s' % (
        record_index, ','.join(POSITION_KEYS)))


def load_source_records(path, start_index=0, max_source_frames=None,
                        position_length=DEFAULT_POSITION_LENGTH):
    """Load selected source JSONL records while preserving their metadata.

    Selection happens before transport interpolation.  Every selected record is
    normalized to ``joint_command_rad`` while the original fields (roll_index,
    phase_name, etc.) remain available for diagnostics and segment handling.
    """
    start_index = int(start_index or 0)
    if start_index < 0:
        raise ValueError('start_index must be >= 0')
    if max_source_frames is not None and int(max_source_frames) <= 0:
        raise ValueError('max_source_frames must be positive when provided')

    records = []
    with open(path) as f:
        for line_index, line in enumerate(f):
            if not line.strip():
                continue
            if line_index < start_index:
                continue
            record = json.loads(line)
            position, source_key = extract_position(
                record, line_index, position_length=position_length)
            normalized = dict(record)
            normalized['joint_command_rad'] = position
            normalized.setdefault(
                'frame_index', normalized.get('command_index', line_index))
            normalized['_command_stream_source_line_index'] = line_index
            normalized['_command_stream_source_key'] = source_key
            records.append(normalized)
            if max_source_frames is not None and \
                    len(records) >= int(max_source_frames):
                break
    return records


def build_transport_records(source_records, resample_factor=1,
                            segment_key=None):
    """Create the backend-independent transport target stream."""
    factor = int(resample_factor)
    if factor < 1:
        raise ValueError('resample_factor must be >= 1')
    return resample_transport_records(
        source_records, factor=factor, segment_key=segment_key)


def prepare_transport_stream(path, resample_factor=1, start_index=0,
                             max_source_frames=None, segment_key=None,
                             position_length=DEFAULT_POSITION_LENGTH):
    """Return ``(source_records, transport_records)`` for either backend."""
    source_records = load_source_records(
        path,
        start_index=start_index,
        max_source_frames=max_source_frames,
        position_length=position_length)
    transport_records = build_transport_records(
        source_records,
        resample_factor=resample_factor,
        segment_key=segment_key)
    return source_records, transport_records


def transport_stream_sha256(records):
    """Stable digest of only the transport joint target vectors.

    This allows Jetson and Gazebo dry-runs to prove that the MCU-input target
    stream is byte-for-byte equivalent at the normalized command level.
    """
    digest = hashlib.sha256()
    for record in records:
        payload = json.dumps(
            [float(v) for v in record['joint_command_rad']],
            separators=(',', ':'),
            allow_nan=False)
        if not isinstance(payload, bytes):
            payload = payload.encode('utf-8')
        digest.update(payload)
        digest.update(b'\n')
    return digest.hexdigest()
