#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

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


def parse_args(argv):
    parser = argparse.ArgumentParser(description='Publish JSONL command positions as sensor_msgs/JointState to /cmdForJetson. This script never opens CAN.')
    parser.add_argument('--command-log', required=True, help='JSONL command file containing joint_command_rad, position, or joint_positions_rad')
    parser.add_argument('--topic', default='/cmdForJetson')
    parser.add_argument('--rate', type=float, required=True, help='Publish rate in Hz')
    parser.add_argument('--start-index', type=int, default=0)
    parser.add_argument('--max-frames', type=int, default=None)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.rate <= 0.0:
        raise SystemExit('--rate must be positive')
    if args.start_index < 0:
        raise SystemExit('--start-index must be >= 0')
    if args.max_frames is not None and args.max_frames <= 0:
        raise SystemExit('--max-frames must be positive when provided')

    import rospy
    from sensor_msgs.msg import JointState

    rospy.init_node('publish_cmdforjetson_jsonl', anonymous=True)
    pub = rospy.Publisher(args.topic, JointState, queue_size=10)
    rate = rospy.Rate(args.rate)

    count = 0
    for record_index, position, source_key in iter_positions(args.command_log, args.start_index, args.max_frames):
        if rospy.is_shutdown():
            break
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position = position
        pub.publish(msg)
        count += 1
        rospy.loginfo('published %s frame record_index=%d source=%s count=%d', args.topic, record_index, source_key, count)
        rate.sleep()
    rospy.loginfo('publish_cmdforjetson_jsonl done: frames=%d topic=%s command_log=%s', count, args.topic, args.command_log)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
