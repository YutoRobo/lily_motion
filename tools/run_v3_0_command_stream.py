#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Run one shared transport command stream against Jetson or Gazebo.

The program path is identical through source loading and transport resampling:

    frozen/staged JSONL -> shared transport target stream

Only then does ``--backend`` branch:

    jetson -> /cmdForJetson -> CAN -> real MCU
    gazebo -> Gazebo-only MCU interpolation emulator -> Gazebo joint topics

Python 2.7 and Python 3 are supported.
"""
from __future__ import division, print_function

import argparse
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.command_resampler import full_command_diagnostics
from lily_motion_v3.command_stream import (
    prepare_transport_stream,
    transport_stream_sha256,
)


def _validate_args(args):
    if not os.path.exists(args.command_log):
        raise IOError('command log does not exist: %s' % args.command_log)
    if args.transport_resample_factor < 1:
        raise SystemExit('--transport-resample-factor must be >= 1')
    if args.transport_rate <= 0.0:
        raise SystemExit('--transport-rate must be positive')
    if args.start_index < 0:
        raise SystemExit('--start-index must be >= 0')
    if args.max_source_frames is not None and args.max_source_frames <= 0:
        raise SystemExit('--max-source-frames must be positive when provided')
    if args.hold_start_sec < 0.0 or args.hold_end_sec < 0.0:
        raise SystemExit('hold durations must be >= 0')
    if args.backend == 'gazebo':
        if args.actuator_interp_duration_sec < 0.0:
            raise SystemExit('--actuator-interp-duration-sec must be >= 0')
        if args.actuator_update_period_sec <= 0.0:
            raise SystemExit('--actuator-update-period-sec must be positive')


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description='Run the same source->transport command stream against Jetson or Gazebo; Gazebo alone inserts MCU interpolation emulation.')
    ap.add_argument('--backend', choices=('jetson', 'gazebo'), required=True)
    ap.add_argument('--command-log', required=True)
    ap.add_argument('--transport-resample-factor', type=int, default=1)
    ap.add_argument('--transport-rate', type=float, required=True,
                    help='Shared Jetson->MCU-equivalent target rate in Hz')
    ap.add_argument('--start-index', type=int, default=0)
    ap.add_argument('--max-source-frames', type=int, default=None)
    ap.add_argument('--segment-key', default='')
    ap.add_argument('--topic', default='/cmdForJetson',
                    help='Jetson backend JointState topic')
    ap.add_argument('--actuator-interp-duration-sec', type=float, default=0.100,
                    help='Gazebo backend only: MCU interpolation duration model')
    ap.add_argument('--actuator-update-period-sec', type=float, default=0.002,
                    help='Gazebo backend only: MCU interpolation update-period model')
    ap.add_argument('--hold-start-sec', type=float, default=0.0)
    ap.add_argument('--hold-end-sec', type=float, default=0.0)
    ap.add_argument('--diagnose-command-log', action='store_true')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--dry-run-sleep', action='store_true')
    ap.add_argument('--verbose', action='store_true')
    return ap.parse_args(argv)


def _sleep_function(period_sec, dry_run=False, dry_run_sleep=False):
    if dry_run:
        return time.sleep if dry_run_sleep else (lambda _sec: None)
    import rospy
    rate = rospy.Rate(1.0 / float(period_sec))
    return lambda _sec: rate.sleep()


def _publish_vector_records(records, publish_vector, period_sec,
                            hold_start_sec=0.0, hold_end_sec=0.0,
                            dry_run=False, dry_run_sleep=False):
    if not records:
        return 0
    sleep_fn = _sleep_function(
        period_sec, dry_run=dry_run, dry_run_sleep=dry_run_sleep)
    count = 0
    start_ticks = int(round(float(hold_start_sec) / float(period_sec)))
    end_ticks = int(round(float(hold_end_sec) / float(period_sec)))
    first = list(records[0]['joint_command_rad'])
    last = list(records[-1]['joint_command_rad'])

    for _ in range(start_ticks):
        publish_vector(first)
        count += 1
        sleep_fn(period_sec)
    for record in records:
        publish_vector(record['joint_command_rad'])
        count += 1
        sleep_fn(period_sec)
    for _ in range(end_ticks):
        publish_vector(last)
        count += 1
        sleep_fn(period_sec)
    return count


def _run_jetson_backend(args, transport_records):
    period_sec = 1.0 / float(args.transport_rate)
    if args.dry_run:
        published = _publish_vector_records(
            transport_records,
            lambda _vector: None,
            period_sec,
            hold_start_sec=args.hold_start_sec,
            hold_end_sec=args.hold_end_sec,
            dry_run=True,
            dry_run_sleep=args.dry_run_sleep)
        print('backend=jetson dry_run=true published_count=%d' % published)
        return published

    import rospy
    from sensor_msgs.msg import JointState
    rospy.init_node('lily_motion_v3_shared_command_stream_jetson', anonymous=True)
    publisher = rospy.Publisher(args.topic, JointState, queue_size=10)

    def publish_vector(vector):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position = list(vector)
        publisher.publish(msg)

    published = _publish_vector_records(
        transport_records,
        publish_vector,
        period_sec,
        hold_start_sec=args.hold_start_sec,
        hold_end_sec=args.hold_end_sec)
    rospy.loginfo(
        'shared command stream done: backend=jetson frames=%d topic=%s',
        published, args.topic)
    return published


def _run_gazebo_backend(args, transport_records):
    from lily_motion_v3.gazebo_actuator_interpolator import (
        simulate_linear_actuator_records,
        timing_relationship,
    )
    from lily_motion_v3.ros_bridge import GazeboCommandPublisher

    target_period_sec = 1.0 / float(args.transport_rate)
    actuator_records = simulate_linear_actuator_records(
        transport_records,
        target_period_sec=target_period_sec,
        interpolation_duration_sec=args.actuator_interp_duration_sec,
        update_period_sec=args.actuator_update_period_sec)
    relation = timing_relationship(
        target_period_sec, args.actuator_interp_duration_sec)
    print(
        'actuator_frames=%d interp_duration_sec=%.6f update_period_sec=%.6f relation=%s' % (
            len(actuator_records),
            args.actuator_interp_duration_sec,
            args.actuator_update_period_sec,
            relation))
    if args.verbose and actuator_records:
        print('first_actuator_time_sec=%.9f last_actuator_time_sec=%.9f' % (
            actuator_records[0]['actuator_time_sec'],
            actuator_records[-1]['actuator_time_sec']))
    if args.diagnose_command_log:
        print('actuator_diagnostics=%s' % json.dumps(
            full_command_diagnostics(actuator_records), sort_keys=True))

    if args.dry_run:
        published = _publish_vector_records(
            actuator_records,
            lambda _vector: None,
            args.actuator_update_period_sec,
            hold_start_sec=args.hold_start_sec,
            hold_end_sec=args.hold_end_sec,
            dry_run=True,
            dry_run_sleep=args.dry_run_sleep)
        print('backend=gazebo dry_run=true published_count=%d' % published)
        return published

    import rospy
    rospy.init_node('lily_motion_v3_shared_command_stream_gazebo', anonymous=True)
    publisher = GazeboCommandPublisher()
    published = _publish_vector_records(
        actuator_records,
        publisher.publish,
        args.actuator_update_period_sec,
        hold_start_sec=args.hold_start_sec,
        hold_end_sec=args.hold_end_sec)
    print('backend=gazebo published_count=%d' % published)
    return published


def main(argv=None):
    args = parse_args(argv)
    _validate_args(args)
    segment_key = args.segment_key.strip() or None
    source_records, transport_records = prepare_transport_stream(
        args.command_log,
        resample_factor=args.transport_resample_factor,
        start_index=args.start_index,
        max_source_frames=args.max_source_frames,
        segment_key=segment_key)

    transport_digest = transport_stream_sha256(transport_records)
    print('source_frames=%d' % len(source_records))
    print('transport_frames=%d transport_factor=%d transport_rate_hz=%.6f' % (
        len(transport_records),
        args.transport_resample_factor,
        args.transport_rate))
    print('transport_sha256=%s' % transport_digest)
    print('backend=%s' % args.backend)

    if args.diagnose_command_log:
        print('transport_diagnostics=%s' % json.dumps(
            full_command_diagnostics(transport_records), sort_keys=True))

    if args.backend == 'jetson':
        return _run_jetson_backend(args, transport_records)
    return _run_gazebo_backend(args, transport_records)


if __name__ == '__main__':
    raise SystemExit(main())
