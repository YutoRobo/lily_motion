#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Compatibility wrapper for the shared command-stream Jetson backend.

New code should use ``tools/run_v3_0_command_stream.py --backend jetson``.
This wrapper keeps the established hardware command line while delegating all
source loading, transport interpolation, and publishing to the same canonical
runner used by Gazebo.
"""
from __future__ import division, print_function

import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
for path in (ROOT, TOOLS_DIR):
    if path not in sys.path:
        sys.path.insert(0, path)

from lily_motion_v3.command_stream import (
    DEFAULT_POSITION_LENGTH,
    build_transport_records,
    extract_position as _extract_position,
    load_source_records,
)
from lily_motion_v3.command_timing import linear_interpolate_command
from run_v3_0_command_stream import main as shared_command_stream_main


POSITION_LENGTH = DEFAULT_POSITION_LENGTH


def extract_position(record, record_index):
    return _extract_position(record, record_index, position_length=POSITION_LENGTH)


def iter_positions(path, start_index=0, max_frames=None):
    records = load_source_records(
        path,
        start_index=start_index,
        max_source_frames=max_frames,
        position_length=POSITION_LENGTH)
    for record in records:
        yield (
            int(record['_command_stream_source_line_index']),
            list(record['joint_command_rad']),
            record['_command_stream_source_key'],
        )


def interpolate_position(position0, position1, alpha):
    if len(position0) != POSITION_LENGTH or len(position1) != POSITION_LENGTH:
        raise ValueError(
            'interpolation requires two %d-element positions' % POSITION_LENGTH)
    return linear_interpolate_command(position0, position1, alpha)


def _selected_records(path, start_index=0, max_frames=None):
    return load_source_records(
        path,
        start_index=start_index,
        max_source_frames=max_frames,
        position_length=POSITION_LENGTH)


def iter_resampled_positions(path, start_index=0, max_frames=None,
                             resample_factor=1, segment_key=None):
    source_records = _selected_records(
        path, start_index=start_index, max_frames=max_frames)
    records = build_transport_records(
        source_records,
        resample_factor=resample_factor,
        segment_key=segment_key)
    for record in records:
        source_index = int(record.get(
            'source_frame_index', record.get('frame_index', 0)))
        yield (
            source_index,
            record.get('next_source_frame_index'),
            list(record['joint_command_rad']),
            record.get('_command_stream_source_key', 'joint_command_rad'),
            float(record.get('interpolation_alpha', 0.0)),
        )


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Compatibility wrapper: publish the shared transport command stream as sensor_msgs/JointState to /cmdForJetson.')
    parser.add_argument('--command-log', required=True)
    parser.add_argument('--topic', default='/cmdForJetson')
    parser.add_argument('--rate', type=float, required=True)
    parser.add_argument('--start-index', type=int, default=0)
    parser.add_argument('--max-frames', type=int, default=None)
    parser.add_argument('--resample-factor', type=int, default=1)
    parser.add_argument('--segment-key', default='')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    translated = [
        '--backend', 'jetson',
        '--command-log', args.command_log,
        '--transport-rate', str(args.rate),
        '--transport-resample-factor', str(args.resample_factor),
        '--start-index', str(args.start_index),
        '--topic', args.topic,
    ]
    if args.max_frames is not None:
        translated.extend(['--max-source-frames', str(args.max_frames)])
    if args.segment_key:
        translated.extend(['--segment-key', args.segment_key])
    return shared_command_stream_main(translated)


if __name__ == '__main__':
    raise SystemExit(main())
