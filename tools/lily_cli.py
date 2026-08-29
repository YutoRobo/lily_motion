#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Unified operator entry point for lily_motion.

Python 2.7 compatible. This module is intentionally a thin wrapper around
existing validated tools; it does not duplicate motion/CAN runtime logic.
"""
from __future__ import division, print_function

import argparse
import json
import os
import shlex
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROFILE_PATH = os.path.join(ROOT, 'config', 'lily_cli_profile.json')


def _load_profile():
    with open(PROFILE_PATH, 'r') as f:
        profile = json.load(f)
    if int(profile.get('schema_version', 0)) != 1:
        raise RuntimeError('unsupported lily CLI profile schema')
    return profile


def _repo_path(*parts):
    return os.path.join(ROOT, *parts)


def _shell_quote(text):
    if not text:
        return "''"
    safe = 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_./-,:='
    if all(ch in safe for ch in text):
        return text
    return "'" + text.replace("'", "'\\''") + "'"


def _display_cmd(cmd):
    if hasattr(shlex, 'quote'):
        return ' '.join(shlex.quote(str(x)) for x in cmd)
    return ' '.join(_shell_quote(str(x)) for x in cmd)


def _run(cmd, cwd=ROOT):
    print('$ %s' % _display_cmd(cmd))
    return subprocess.call(cmd, cwd=cwd)


def _capture(cmd, cwd=ROOT):
    try:
        proc = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE)
        out, err = proc.communicate()
        if not isinstance(out, str):
            out = out.decode('utf-8', 'replace')
        if not isinstance(err, str):
            err = err.decode('utf-8', 'replace')
        return proc.returncode, out.strip(), err.strip()
    except Exception as exc:
        return 127, '', str(exc)


def _which(name):
    rc, out, _err = _capture(['which', name])
    return out if rc == 0 and out else None


def _stage_path(profile, stage):
    rel = profile['stages'][stage]
    return os.path.join(ROOT, profile['candidate'], rel)


def _print_check(ok, label, detail=''):
    state = 'OK' if ok else 'FAIL'
    suffix = (' - ' + detail) if detail else ''
    print('[%s] %s%s' % (state, label, suffix))


def cmd_status(_args):
    profile = _load_profile()
    rc, branch, _ = _capture(['git', 'rev-parse', '--abbrev-ref', 'HEAD'])
    _print_check(rc == 0, 'git repository', branch if rc == 0 else '')
    rc, dirty, _ = _capture(['git', 'status', '--porcelain'])
    if rc == 0:
        print('[%s] working tree%s' % ('WARN' if dirty else 'OK',
                                     ' - dirty' if dirty else ' - clean'))

    print('profile: %s' % os.path.relpath(PROFILE_PATH, ROOT))
    print('candidate: %s' % profile['candidate'])
    print('transport: resample_factor=%d rate_hz=%s' % (
        int(profile['transport']['resample_factor']),
        profile['transport']['rate_hz']))

    missing = []
    for stage in sorted(profile['stages']):
        if not os.path.isfile(_stage_path(profile, stage)):
            missing.append(stage)
    detail = ('missing: %s' % ','.join(missing)
              if missing else '%d available' % len(profile['stages']))
    _print_check(not missing, 'configured staged files', detail)
    return 0 if not missing else 1


def cmd_doctor(args):
    profile = _load_profile()
    failures = 0
    warnings = 0

    checks = [
        (os.path.isfile(PROFILE_PATH), 'CLI profile',
         os.path.relpath(PROFILE_PATH, ROOT)),
        (bool(_which('python2')), 'python2', _which('python2') or 'not found'),
        (os.path.isfile('/opt/ros/melodic/setup.bash'), 'ROS Melodic setup',
         '/opt/ros/melodic/setup.bash'),
    ]
    if args.target == 'real':
        checks.extend([
            (bool(_which('candump')), 'candump', _which('candump') or 'not found'),
            (bool(_which('ip')), 'ip command', _which('ip') or 'not found'),
        ])
    for ok, label, detail in checks:
        _print_check(ok, label, detail)
        if not ok:
            failures += 1

    for stage in sorted(profile['stages']):
        if not os.path.isfile(_stage_path(profile, stage)):
            _print_check(False, 'stage ' + stage, _stage_path(profile, stage))
            failures += 1

    if args.target == 'real' and _which('ip'):
        rc, out, err = _capture(
            ['ip', '-details', 'link', 'show', args.interface])
        ok = rc == 0
        _print_check(ok, args.interface + ' exists', err if not ok else '')
        if not ok:
            failures += 1
        else:
            first_line = out.split('\n')[0] if out else ''
            is_up = ('UP' in first_line)
            bitrate_ok = ('bitrate 500000' in out)
            _print_check(is_up, args.interface + ' link state',
                         'UP' if is_up else 'not UP')
            _print_check(bitrate_ok, args.interface + ' bitrate',
                         '500000' if bitrate_ok else 'not 500000 / unknown')
            if not is_up or not bitrate_ok:
                failures += 1

    if _which('rostopic'):
        rc, _out, err = _capture(['rostopic', 'info', '/cmdForJetson'])
        if rc == 0:
            print('[OK] /cmdForJetson is visible')
        else:
            print('[WARN] /cmdForJetson is not currently visible - %s' %
                  (err or 'ROS master/runtime may be stopped'))
            warnings += 1
    else:
        print('[WARN] rostopic not in current PATH; source ROS/catkin before runtime checks')
        warnings += 1

    if failures:
        print('NOT READY - %d required check(s) failed' % failures)
        return 1
    if warnings:
        print('READY WITH WARNINGS - %d runtime check(s) unresolved' % warnings)
    else:
        print('READY')
    return 0


def cmd_viewer(args):
    cmd = [
        sys.executable,
        _repo_path('tools', 'diagnostics',
                   'realtime_position_debug_viewer_ui.py'),
        '--interface', args.interface,
        '--duration-sec', str(args.duration_sec),
    ]
    if args.axes:
        cmd += ['--axes', args.axes]
    else:
        cmd += ['--leg-index', str(args.leg_index)]
    if args.no_csv:
        cmd.append('--no-csv')
    elif args.csv:
        cmd += ['--csv', args.csv]
    return _run(cmd)


def cmd_config(args):
    cmd = [
        sys.executable,
        _repo_path('tools', 'mcu_config', 'lily_mcu_config_editor.py'),
        '--interface', args.interface,
        '--axes', args.axes,
    ]
    return _run(cmd)


def cmd_test_axis(args):
    cmd = [
        sys.executable,
        _repo_path('tools', 'publish_cmdforjetson_single_axis_test.py'),
        '--axis', str(args.axis),
        '--direction', args.direction,
        '--amplitude-rad', str(args.amplitude_rad),
        '--step-rad', str(args.step_rad),
        '--period-sec', str(args.period_sec),
        '--start-hold-sec', str(args.start_hold_sec),
        '--peak-hold-sec', str(args.peak_hold_sec),
        '--end-hold-sec', str(args.end_hold_sec),
    ]
    if not args.execute:
        print('COMMAND ONLY - add --execute to publish /cmdForJetson')
        print('$ %s' % _display_cmd(cmd))
        return 0
    return _run(cmd)


def cmd_test_leg(args):
    cmd = [
        sys.executable,
        _repo_path('tools', 'publish_cmdforjetson_one_leg_test.py'),
        '--leg-index', str(args.leg_index),
        '--mode', args.mode,
        '--direction', args.direction,
        '--centers-rad', args.centers_rad,
        '--amplitude-rad', str(args.amplitude_rad),
        '--step-rad', str(args.step_rad),
        '--period-sec', str(args.period_sec),
    ]
    if not args.execute:
        print('COMMAND ONLY - add --execute to publish /cmdForJetson')
        print('$ %s' % _display_cmd(cmd))
        return 0
    return _run(cmd)


def cmd_play(args):
    profile = _load_profile()
    command_log = _stage_path(profile, args.stage)
    if not os.path.isfile(command_log):
        print('ERROR: staged file does not exist: %s' % command_log,
              file=sys.stderr)
        return 2
    resample = (args.resample_factor if args.resample_factor is not None
                else int(profile['transport']['resample_factor']))
    rate = (args.rate if args.rate is not None
            else float(profile['transport']['rate_hz']))
    cmd = [
        sys.executable,
        _repo_path('tools', 'publish_cmdforjetson_jsonl.py'),
        '--command-log', command_log,
        '--resample-factor', str(resample),
        '--rate', str(rate),
    ]
    if not args.execute:
        cmd.append('--dry-run')
        print('DRY RUN - add --execute to publish /cmdForJetson')
    return _run(cmd)


def build_parser():
    profile = _load_profile()
    parser = argparse.ArgumentParser(
        prog='lily',
        description='Unified operator CLI for lily_motion (Python 2 compatible)')
    sub = parser.add_subparsers(dest='command')

    p = sub.add_parser('status',
                       help='show repository and current CLI profile status')
    p.set_defaults(func=cmd_status)

    p = sub.add_parser('doctor', help='check host/runtime prerequisites')
    p.add_argument('target', choices=('real', 'gazebo'), nargs='?',
                   default='real')
    p.add_argument('--interface', default=profile.get('can_interface', 'can0'))
    p.set_defaults(func=cmd_doctor)

    p = sub.add_parser(
        'viewer', help='start receive-only MCU position debug viewer')
    p.add_argument('--interface', default=profile.get('can_interface', 'can0'))
    group = p.add_mutually_exclusive_group()
    group.add_argument('--leg-index', type=int, default=3)
    group.add_argument('--axes', default='')
    p.add_argument('--duration-sec', type=float, default=5.0)
    p.add_argument('--csv', default='')
    p.add_argument('--no-csv', action='store_true')
    p.set_defaults(func=cmd_viewer)

    p = sub.add_parser('config', help='open MCU configuration GUI')
    p.add_argument('--interface', default=profile.get('can_interface', 'can0'))
    p.add_argument('--axes', default='0-23')
    p.set_defaults(func=cmd_config)

    test = sub.add_parser('test',
                          help='build or execute safe test publishers')
    test_sub = test.add_subparsers(dest='test_kind')

    p = test_sub.add_parser('axis', help='single-axis out-and-back test')
    p.add_argument('axis', type=int)
    p.add_argument('--direction', choices=('plus', 'minus'), default='plus')
    p.add_argument('--amplitude-rad', type=float, default=0.002)
    p.add_argument('--step-rad', type=float, default=0.001)
    p.add_argument('--period-sec', type=float, default=0.500)
    p.add_argument('--start-hold-sec', type=float, default=1.000)
    p.add_argument('--peak-hold-sec', type=float, default=1.000)
    p.add_argument('--end-hold-sec', type=float, default=1.000)
    p.add_argument('--execute', action='store_true',
                   help='actually publish; default is command preview only')
    p.set_defaults(func=cmd_test_axis)

    p = test_sub.add_parser('leg', help='one-leg three-axis test')
    p.add_argument('leg_index', type=int)
    p.add_argument('--mode', choices=('individual', 'coordinated', 'all'),
                   default='individual')
    p.add_argument('--direction', choices=('plus', 'minus', 'both'),
                   default='plus')
    p.add_argument('--centers-rad', default='0,0,0')
    p.add_argument('--amplitude-rad', type=float, default=0.002)
    p.add_argument('--step-rad', type=float, default=0.001)
    p.add_argument('--period-sec', type=float, default=0.500)
    p.add_argument('--execute', action='store_true',
                   help='actually publish; default is command preview only')
    p.set_defaults(func=cmd_test_leg)

    p = sub.add_parser(
        'play', help='validate or execute a configured staged motion')
    p.add_argument('stage', choices=sorted(profile['stages'].keys()))
    p.add_argument('--resample-factor', type=int, default=None)
    p.add_argument('--rate', type=float, default=None)
    p.add_argument('--execute', action='store_true',
                   help='actually publish; default calls publisher with --dry-run')
    p.set_defaults(func=cmd_play)

    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    if not hasattr(args, 'func'):
        parser.print_help()
        return 2
    return int(args.func(args) or 0)


if __name__ == '__main__':
    raise SystemExit(main())
