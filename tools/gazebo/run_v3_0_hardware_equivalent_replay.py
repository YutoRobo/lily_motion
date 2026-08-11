#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Compatibility wrapper for the shared command-stream Gazebo backend.

New code should use ``tools/run_v3_0_command_stream.py --backend gazebo``.
This file retains the previous Gazebo command line while delegating the complete
source->transport path to the same canonical runner used by Jetson.  Only the
Gazebo backend inserts the MCU interpolation emulator.
"""
from __future__ import print_function

import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TOOLS_DIR = os.path.join(ROOT, 'tools')
for path in (ROOT, TOOLS_DIR):
    if path not in sys.path:
        sys.path.insert(0, path)

from run_v3_0_command_stream import main as shared_command_stream_main


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description='Compatibility wrapper for shared command stream --backend gazebo')
    ap.add_argument('--command-log', required=True)
    ap.add_argument('--transport-resample-factor', type=int, default=2)
    ap.add_argument('--transport-rate', type=float, default=10.0)
    ap.add_argument('--start-index', type=int, default=0)
    ap.add_argument('--segment-key', default='')
    ap.add_argument('--actuator-interp-duration-sec', type=float, default=0.100)
    ap.add_argument('--actuator-update-period-sec', type=float, default=0.002)
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
    translated = [
        '--backend', 'gazebo',
        '--command-log', args.command_log,
        '--transport-resample-factor', str(args.transport_resample_factor),
        '--transport-rate', str(args.transport_rate),
        '--start-index', str(args.start_index),
        '--actuator-interp-duration-sec', str(args.actuator_interp_duration_sec),
        '--actuator-update-period-sec', str(args.actuator_update_period_sec),
        '--hold-start-sec', str(args.hold_start_sec),
        '--hold-end-sec', str(args.hold_end_sec),
    ]
    if args.segment_key:
        translated.extend(['--segment-key', args.segment_key])
    if args.max_source_frames is not None:
        translated.extend(['--max-source-frames', str(args.max_source_frames)])
    if args.diagnose_command_log:
        translated.append('--diagnose-command-log')
    if args.dry_run:
        translated.append('--dry-run')
    if args.dry_run_sleep:
        translated.append('--dry-run-sleep')
    if args.verbose:
        translated.append('--verbose')
    return shared_command_stream_main(translated)


if __name__ == '__main__':
    raise SystemExit(main())
