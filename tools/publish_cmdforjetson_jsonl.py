#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division, print_function

import argparse
import json
import sys

POSITION_KEYS = ('joint_command_rad', 'position', 'joint_positions_rad')
POSITION_LENGTH = 24


def extract_position(record, record_index):
    for key in POSITION_KEYS:
        if key in record:
            pos = record[key]
            if not isinstance(pos, list):
                raise ValueError('record %d key %s is not a list' % (record_index, key))
            if len(pos) != POSITION_LENGTH:
                raise ValueError('record %d key %s length %d != %d' % (
                    record_index, key, len(pos), POSITION_LENGTH))
            return [float(v) for v in pos], key
    raise ValueError('record %d has none of %s' % (record_index, ','.join(POSITION_KEYS)))


def iter_positions(path, start_index=0, max_frames=None):
    emitted = 0
    with open(path) as f:
        for line_index, line in enumerate(f):
            line = line.strip()
            if not line:
                continue
            if line_index < start_index:
                continue
            record = json.loads(line)
            pos, key = extract_position(record, line_index)
            yield line_index, pos, key
            emitted += 1
            if max_frames is not None and emitted >= max_frames:
                break


def interpolate_position(position0, position1, alpha):
    if len(position0) != POSITION_LENGTH or len(position1) != POSITION_LENGTH:
        raise ValueError('interpolation requires two %d-element positions' % POSITION_LENGTH)
    alpha = float(alpha)
    return [
        float(a) + (float(b) - float(a)) * alpha
        for a, b in zip(position0, position1)
    ]


def iter_resampled_positions(path, start_index=0, max_frames=None,
                             resample_factor=1):
    """Yield selected JSONL positions with optional linear interpolation.

    ``start_index`` and ``max_frames`` apply to source JSONL records before
    interpolation. ``resample_factor=1`` preserves the legacy publisher output.
    ``resample_factor=2`` inserts one midpoint between adjacent source records.
    In general, N source records produce ``(N - 1) * factor + 1`` output
    positions when N > 0.
    """
    factor = int(resample_factor or 1)
    if factor < 1:
        raise ValueError('resample_factor must be >= 1')

    source = iter_positions(path, start_index=start_index, max_frames=max_frames)
    try:
        previous_index, previous_position, previous_key = next(source)
    except StopIteration:
        return

    for next_index, next_position, next_key in source:
        for step in range(factor):
            alpha = float(step) / float(factor)
            if step == 0:
                position = list(previous_position)
            else:
                position = interpolate_position(
                    previous_position, next_position, alpha)
            yield (
                previous_index,
                next_index,
                position,
                previous_key,
                alpha,
            )
        previous_index = next_index
        previous_position = next_position
        previous_key = next_key

    yield previous_index, None, list(previous_position), previous_key, 0.0


def parse_args(argv):
    parser = argparse.ArgumentParser(description='Publish JSONL command positions as sensor_msgs/JointState to /cmdForJetson. This script never opens CAN.')
    parser.add_argument('--command-log', required=True, help='JSONL command file containing joint_command_rad, position, or joint_positions_rad')
    parser.add_argument('--topic', default='/cmdForJetson')
    parser.add_argument('--rate', type=float, required=True, help='Publish rate in Hz')
    parser.add_argument('--start-index', type=int, default=0)
    parser.add_argument('--max-frames', type=int, default=None,
                        help='Maximum number of source JSONL frames before interpolation')
    parser.add_argument('--resample-factor', type=int, default=1,
                        help='Linear interpolation factor between adjacent source frames. factor=1 preserves legacy behavior; factor=2 inserts one midpoint. Increase --rate by the same factor to preserve the source time scale.')
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

    import rospy
    from sensor_msgs.msg import JointState

    rospy.init_node('publish_cmdforjetson_jsonl', anonymous=True)
    pub = rospy.Publisher(args.topic, JointState, queue_size=10)
    rate = rospy.Rate(args.rate)

    count = 0
    for source_index, next_source_index, position, source_key, alpha in iter_resampled_positions(
            args.command_log,
            start_index=args.start_index,
            max_frames=args.max_frames,
            resample_factor=args.resample_factor):
        if rospy.is_shutdown():
            break
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position = position
        pub.publish(msg)
        count += 1
        rospy.loginfo(
            'published %s frame source_index=%d next_source_index=%s alpha=%.6f source=%s count=%d',
            args.topic,
            source_index,
            str(next_source_index),
            alpha,
            source_key,
            count)
        rate.sleep()
    rospy.loginfo(
        'publish_cmdforjetson_jsonl done: frames=%d topic=%s command_log=%s resample_factor=%d rate_hz=%.6f',
        count,
        args.topic,
        args.command_log,
        args.resample_factor,
        args.rate)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
