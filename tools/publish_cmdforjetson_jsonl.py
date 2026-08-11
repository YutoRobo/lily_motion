#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Canonical shared command publisher for both real hardware and Gazebo.

This program always performs the same upstream path::

    staged/frozen JSONL
      -> shared source normalization
      -> shared transport resampling
      -> /cmdForJetson

It contains no Gazebo/MCU-emulation branch.  Real hardware consumes
``/cmdForJetson`` through the CAN interface.  Gazebo consumes the exact same
topic through ``tools/gazebo/mcu_position_interpolator_node.py``.
"""
from __future__ import division, print_function

import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.command_stream import (
    POSITION_KEYS,
    DEFAULT_POSITION_LENGTH,
    extract_position,
    prepare_transport_stream,
    transport_stream_sha256,
)

POSITION_LENGTH = DEFAULT_POSITION_LENGTH


def iter_positions(path, start_index=0, max_frames=None):
    """Backward-compatible source-frame iterator."""
    source_records, _transport_records = prepare_transport_stream(
        path,
        resample_factor=1,
        start_index=start_index,
        max_source_frames=max_frames)
    for record in source_records:
        yield (
            int(record.get('_command_stream_source_line_index',
                           record.get('frame_index', 0))),
            list(record['joint_command_rad']),
            record.get('_command_stream_source_key', 'joint_command_rad'),
        )


def interpolate_position(position0, position1, alpha):
    """Backward-compatible helper retained for existing pure tests."""
    from lily_motion_v3.command_timing import linear_interpolate_command
    if len(position0) != POSITION_LENGTH or len(position1) != POSITION_LENGTH:
        raise ValueError(
            'interpolation requires two %d-element positions' % POSITION_LENGTH)
    return linear_interpolate_command(position0, position1, alpha)


def iter_resampled_positions(path, start_index=0, max_frames=None,
                             resample_factor=1, segment_key=None):
    """Backward-compatible iterator backed by the single shared stream builder."""
    _source_records, transport_records = prepare_transport_stream(
        path,
        resample_factor=resample_factor,
        start_index=start_index,
        max_source_frames=max_frames,
        segment_key=segment_key)
    for record in transport_records:
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
        description='Publish the shared JSONL transport command stream as sensor_msgs/JointState to /cmdForJetson. This script never opens CAN and contains no Gazebo branch.')
    parser.add_argument(
        '--command-log', required=True,
        help='JSONL command file containing joint_command_rad, position, or joint_positions_rad')
    parser.add_argument('--topic', default='/cmdForJetson')
    parser.add_argument('--rate', type=float, required=True,
                        help='Transport target publish rate in Hz')
    parser.add_argument('--start-index', type=int, default=0)
    parser.add_argument(
        '--max-frames', type=int, default=None,
        help='Maximum number of source JSONL frames before interpolation')
    parser.add_argument(
        '--resample-factor', type=int, default=1,
        help='Shared linear transport resampling factor. factor=1 preserves source frames; factor=2 inserts one midpoint. Increase --rate by the same factor to preserve source time scale.')
    parser.add_argument(
        '--segment-key', default='',
        help='Optional source metadata key used to prevent interpolation across segment boundaries')
    parser.add_argument('--dry-run', action='store_true',
                        help='Build and report the exact /cmdForJetson stream without ROS publishing')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.rate <= 0.0:
        raise SystemExit('--rate must be positive')
    if args.start_index < 0:
        raise SystemExit('--start-index must be >= 0')
    if args.max_frames is not None and args.max_frames <= 0:
        raise SystemExit('--max-frames must be positive when provided')
    if args.resample_factor < 1:
        raise SystemExit('--resample-factor must be >= 1')

    segment_key = args.segment_key.strip() or None
    source_records, transport_records = prepare_transport_stream(
        args.command_log,
        resample_factor=args.resample_factor,
        start_index=args.start_index,
        max_source_frames=args.max_frames,
        segment_key=segment_key)
    digest = transport_stream_sha256(transport_records)

    print('source_frames=%d' % len(source_records))
    print('transport_frames=%d resample_factor=%d rate_hz=%.6f' % (
        len(transport_records), args.resample_factor, args.rate))
    print('transport_sha256=%s' % digest)
    print('output_topic=%s' % args.topic)

    if args.dry_run:
        print('dry_run=true published_count=0')
        return 0

    import rospy
    from sensor_msgs.msg import JointState

    rospy.init_node('publish_cmdforjetson_jsonl', anonymous=True)
    pub = rospy.Publisher(args.topic, JointState, queue_size=10)
    rate = rospy.Rate(args.rate)

    count = 0
    for record in transport_records:
        if rospy.is_shutdown():
            break
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position = list(record['joint_command_rad'])
        pub.publish(msg)
        count += 1
        rate.sleep()

    rospy.loginfo(
        'publish_cmdforjetson_jsonl done: frames=%d topic=%s command_log=%s resample_factor=%d rate_hz=%.6f transport_sha256=%s',
        count,
        args.topic,
        args.command_log,
        args.resample_factor,
        args.rate,
        digest)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
