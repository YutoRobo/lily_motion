#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Replay a frozen command log in Gazebo through the hardware timing model.

This is the hardware-equivalent timing path:

    frozen source command log
      -> shared transport resampling
      -> transport target stream
      -> configurable actuator linear-interpolation emulation
      -> Gazebo joint topics

The MCU timing values are parameters, not constants.  The current MCU can be
represented by transport_rate=10 Hz, interpolation_duration=0.100 s, and
update_period=0.002 s, but other values can be evaluated without changing code.
"""
from __future__ import division, print_function

import argparse
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.command_resampler import full_command_diagnostics
from lily_motion_v3.command_timing import (
    resample_transport_records,
    simulate_linear_actuator_records,
    timing_relationship,
)
from lily_motion_v3.ros_bridge import GazeboCommandPublisher, MockPublisher


def load_records(path, max_source_frames=None):
    records = []
    with open(path) as f:
        for i, line in enumerate(f):
            if not line.strip():
                continue
            rec = json.loads(line)
            if 'joint_command_rad' not in rec:
                raise ValueError(
                    'record %d has no joint_command_rad: %s' % (i, path))
            rec = dict(rec)
            rec.setdefault('frame_index', rec.get('command_index', i))
            records.append(rec)
            if max_source_frames is not None and \
                    len(records) >= max_source_frames:
                break
    return records


def build_hardware_equivalent_records(
        source_records,
        transport_resample_factor,
        transport_rate_hz,
        actuator_interp_duration_sec,
        actuator_update_period_sec,
        segment_key=None):
    if transport_rate_hz <= 0.0:
        raise ValueError('transport_rate_hz must be positive')
    transport_records = resample_transport_records(
        source_records,
        factor=transport_resample_factor,
        segment_key=segment_key)
    actuator_records = simulate_linear_actuator_records(
        transport_records,
        target_period_sec=1.0 / float(transport_rate_hz),
        interpolation_duration_sec=actuator_interp_duration_sec,
        update_period_sec=actuator_update_period_sec)
    return transport_records, actuator_records


def _publish_command(publisher, command):
    publisher.publish(command)


def publish_records(records, publisher, publish_period_sec,
                    hold_start_sec=0.0, hold_end_sec=0.0,
                    dry_run=False, dry_run_sleep=False):
    if publish_period_sec <= 0.0:
        raise ValueError('publish_period_sec must be positive')
    if not records:
        return 0

    if dry_run:
        sleep_fn = time.sleep if dry_run_sleep else (lambda _sec: None)
    else:
        import rospy
        rate = rospy.Rate(1.0 / publish_period_sec)
        sleep_fn = lambda _sec: rate.sleep()

    count = 0
    start_ticks = int(round(max(0.0, hold_start_sec) / publish_period_sec))
    end_ticks = int(round(max(0.0, hold_end_sec) / publish_period_sec))

    first = list(records[0]['joint_command_rad'])
    last = list(records[-1]['joint_command_rad'])

    for _ in range(start_ticks):
        _publish_command(publisher, first)
        count += 1
        sleep_fn(publish_period_sec)

    for rec in records:
        _publish_command(publisher, rec['joint_command_rad'])
        count += 1
        sleep_fn(publish_period_sec)

    for _ in range(end_ticks):
        _publish_command(publisher, last)
        count += 1
        sleep_fn(publish_period_sec)
    return count


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description='Replay a frozen command log in Gazebo using the same transport target stream as hardware plus configurable MCU interpolation emulation.')
    ap.add_argument('--command-log', required=True)
    ap.add_argument('--transport-resample-factor', type=int, default=2,
                    help='Shared source->transport linear resampling factor')
    ap.add_argument('--transport-rate', type=float, default=10.0,
                    help='Transport target rate in Hz (Jetson->MCU equivalent)')
    ap.add_argument('--segment-key', default='',
                    help='Optional segment boundary key for transport resampling; usually leave empty for hardware-equivalent replay')
    ap.add_argument('--actuator-interp-duration-sec', type=float, default=0.100,
                    help='Actuator target interpolation duration; configurable to match MCU firmware')
    ap.add_argument('--actuator-update-period-sec', type=float, default=0.002,
                    help='Actuator internal interpolation update period; configurable to match MCU firmware')
    ap.add_argument('--max-source-frames', type=int, default=None)
    ap.add_argument('--hold-start-sec', type=float, default=2.0)
    ap.add_argument('--hold-end-sec', type=float, default=2.0)
    ap.add_argument('--diagnose-command-log', action='store_true')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--dry-run-sleep', action='store_true')
    ap.add_argument('--verbose', action='store_true')
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if not os.path.exists(args.command_log):
        raise IOError('command log does not exist: %s' % args.command_log)
    if args.transport_resample_factor < 1:
        raise SystemExit('--transport-resample-factor must be >= 1')
    if args.transport_rate <= 0.0:
        raise SystemExit('--transport-rate must be positive')
    if args.actuator_interp_duration_sec < 0.0:
        raise SystemExit('--actuator-interp-duration-sec must be >= 0')
    if args.actuator_update_period_sec <= 0.0:
        raise SystemExit('--actuator-update-period-sec must be positive')
    if args.max_source_frames is not None and args.max_source_frames <= 0:
        raise SystemExit('--max-source-frames must be positive when provided')

    source_records = load_records(
        args.command_log, max_source_frames=args.max_source_frames)
    segment_key = args.segment_key.strip() or None
    transport_records, actuator_records = build_hardware_equivalent_records(
        source_records,
        transport_resample_factor=args.transport_resample_factor,
        transport_rate_hz=args.transport_rate,
        actuator_interp_duration_sec=args.actuator_interp_duration_sec,
        actuator_update_period_sec=args.actuator_update_period_sec,
        segment_key=segment_key)

    relation = timing_relationship(
        1.0 / args.transport_rate,
        args.actuator_interp_duration_sec)
    print('source_frames=%d' % len(source_records))
    print('transport_frames=%d transport_factor=%d transport_rate_hz=%.6f' % (
        len(transport_records),
        args.transport_resample_factor,
        args.transport_rate))
    print('actuator_frames=%d interp_duration_sec=%.6f update_period_sec=%.6f relation=%s' % (
        len(actuator_records),
        args.actuator_interp_duration_sec,
        args.actuator_update_period_sec,
        relation))

    if args.diagnose_command_log:
        print('transport_diagnostics=%s' % json.dumps(
            full_command_diagnostics(transport_records), sort_keys=True))
        print('actuator_diagnostics=%s' % json.dumps(
            full_command_diagnostics(actuator_records), sort_keys=True))

    if args.verbose and actuator_records:
        print('first_actuator_time_sec=%.9f last_actuator_time_sec=%.9f' % (
            actuator_records[0]['actuator_time_sec'],
            actuator_records[-1]['actuator_time_sec']))

    if args.dry_run:
        publisher = MockPublisher()
    else:
        import rospy
        rospy.init_node('lily_motion_v3_hardware_equivalent_replay', anonymous=True)
        publisher = GazeboCommandPublisher()

    published = publish_records(
        actuator_records,
        publisher,
        publish_period_sec=args.actuator_update_period_sec,
        hold_start_sec=args.hold_start_sec,
        hold_end_sec=args.hold_end_sec,
        dry_run=args.dry_run,
        dry_run_sleep=args.dry_run_sleep)
    print('published_count=%d' % published)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
