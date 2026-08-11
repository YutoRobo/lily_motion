#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import argparse
import os
import subprocess
import sys

DEFAULT_CANDIDATE = 'data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075'


def repo_root():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


def normalize_stage(value):
    text = str(value).strip().lower().replace('roll-to-', '').replace('roll_to_', '')
    text = text.replace('of4', '').replace('/4', '')
    try:
        stage = int(text)
    except Exception:
        raise ValueError('stage must be 1,2,3,4 (also accepts 2/4 or roll-to-2of4)')
    if stage not in (1, 2, 3, 4):
        raise ValueError('stage must be one of 1,2,3,4')
    return stage


def q(path):
    return "'" + path.replace("'", "'\\''") + "'"


def print_cmd(cmd):
    print(' '.join(q(str(x)) if (' ' in str(x) or '/' in str(x)) else str(x) for x in cmd))


def run(cmd, dry_run=False):
    print_cmd(cmd)
    if dry_run:
        return 0
    return subprocess.call(cmd)


def main(argv=None):
    ap = argparse.ArgumentParser(description='Replay the exact same cumulative quarter-roll JSONL in Gazebo or on hardware.')
    ap.add_argument('--mode', choices=('gazebo', 'hardware'), required=True)
    ap.add_argument('--stage', required=True, help='1,2,3,4; 2/4 and roll-to-2of4 are also accepted')
    ap.add_argument('--candidate-dir', default=DEFAULT_CANDIDATE,
                    help='Candidate directory containing staged/roll_to_Nof4_commands.jsonl')
    ap.add_argument('--rate', type=float, default=None,
                    help='Replay/publish rate. Defaults: Gazebo 15 Hz, hardware 3 Hz.')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--confirm-hardware', action='store_true',
                    help='Required in hardware mode. Confirms the operator intentionally wants /cmdForJetson motion.')
    args = ap.parse_args(sys.argv[1:] if argv is None else argv)

    try:
        stage = normalize_stage(args.stage)
    except ValueError as exc:
        print('error: %s' % exc, file=sys.stderr)
        return 2
    if args.rate is not None and args.rate <= 0:
        print('error: --rate must be positive', file=sys.stderr)
        return 2
    if args.mode == 'hardware' and not args.confirm_hardware and not args.dry_run:
        print('error: hardware mode requires --confirm-hardware', file=sys.stderr)
        return 2

    root = repo_root()
    candidate_dir = args.candidate_dir
    if not os.path.isabs(candidate_dir):
        candidate_dir = os.path.join(root, candidate_dir)
    command_log = os.path.join(candidate_dir, 'staged', 'roll_to_%dof4_commands.jsonl' % stage)
    if not os.path.isfile(command_log) and not args.dry_run:
        print('error: missing semantic stage log: %s' % command_log, file=sys.stderr)
        print('Run tools/build_roll_quarter_stages.py first.', file=sys.stderr)
        return 2

    rate = args.rate if args.rate is not None else (15.0 if args.mode == 'gazebo' else 3.0)
    print('mode:', args.mode)
    print('stage: %d/4 cumulative' % stage)
    print('command_log:', command_log)
    print('rate_hz:', rate)
    print('IMPORTANT: Gazebo and hardware use this exact same JSONL file.')

    if args.mode == 'gazebo':
        cmd = [sys.executable,
               os.path.join(root, 'tools', 'gazebo', 'run_v3_0_gazebo_replay.py'),
               '--command-log', command_log,
               '--strict-command-log-input',
               '--rate', str(rate),
               '--hold-start-sec', '2.0',
               '--hold-end-sec', '2.0',
               '--diagnose-command-log']
        return run(cmd, args.dry_run)

    publisher = [sys.executable,
                 os.path.join(root, 'tools', 'publish_cmdforjetson_jsonl.py'),
                 '--command-log', command_log,
                 '--rate', str(rate)]
    run_ui = ['rostopic', 'pub', '-1', '/ui/leg_command', 'std_msgs/String', "data: 'run'"]
    stop_ui = ['rostopic', 'pub', '-1', '/ui/leg_command', 'std_msgs/String', "data: 'stop'"]
    if run(run_ui, args.dry_run) != 0:
        return 1
    try:
        return run(publisher, args.dry_run)
    finally:
        run(stop_ui, args.dry_run)


if __name__ == '__main__':
    raise SystemExit(main())
