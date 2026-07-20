#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _run(cmd):
    p = subprocess.Popen(cmd, cwd=ROOT)
    rc = p.wait()
    if rc != 0:
        raise RuntimeError('command failed with rc=%s: %s' % (rc, ' '.join(cmd)))


def _ensure_dir(path):
    if path and not os.path.isdir(path):
        os.makedirs(path)


def main():
    ap = argparse.ArgumentParser(description='Generate provisional baseline v2 command logs: v3.0.42C case27 + x8 smooth_window=40.')
    ap.add_argument('--output-dir', default='testdata/provisional_baseline_v2')
    ap.add_argument('--python', default=sys.executable or 'python')
    ap.add_argument('--skip-constraints', action='store_true', help='Skip constraint evaluation in the raw generation step.')
    args = ap.parse_args()

    out_dir = args.output_dir
    _ensure_dir(out_dir)
    raw_log = os.path.join(out_dir, 'provisional_baseline_v2_raw_commands.jsonl')
    raw_report = os.path.join(out_dir, 'provisional_baseline_v2_generation_report.json')
    filtered_log = os.path.join(out_dir, 'provisional_baseline_v2_x8_sw40_commands.jsonl')
    smooth_report = os.path.join(out_dir, 'provisional_baseline_v2_resample_summary.json')
    manifest = os.path.join(out_dir, 'provisional_baseline_v2_manifest.json')

    # v3.0.42C case27 parameters.
    gen_cmd = [
        args.python, 'archive/v3_experiment_scripts/run_v3_0_pure_legacy_repeated_roll.py',
        '--surface-sequence', '1,5,6,2,1',
        '--move-dist', '0.40',
        '--support-dist', '0.77',
        '--legacy-body-z', '0.40',
        '--max-step', '30',
        '--goal2-dist-front', '0.40',
        '--goal2-x-scale', '0.95',
        '--goal2-pitch-scale', '0.90',
        '--goal2-landing-z', '0.00',
        '--goal3-lift-z', '0.05',
        '--goal3-target-x', '0.20',
        '--goal4-target-x', '0.05',
        '--goal5-x-scale', '1.00',
        '--goal5-pitch-scale', '1.00',
        '--rf1-current-angle-anchor',
        '--output-command-log', raw_log,
        '--report-output', raw_report,
    ]
    if args.skip_constraints:
        gen_cmd.append('--skip-constraints')

    smooth_cmd = [
        args.python, 'tools/command_generation/run_v3_0_resample_commands.py',
        '--input', raw_log,
        '--output', filtered_log,
        '--resample-factor', '8',
        '--smooth-window', '40',
        '--segment-key', '',
        '--diagnose-boundaries',
    ]

    _run(gen_cmd)
    with open(smooth_report, 'w') as f:
        p = subprocess.Popen(smooth_cmd, cwd=ROOT, stdout=f)
        rc = p.wait()
    if rc != 0:
        raise RuntimeError('resample command failed with rc=%s: %s' % (rc, ' '.join(smooth_cmd)))

    info = {
        'baseline_name': 'provisional_baseline_v2',
        'description': 'v3.0.42C case27; second-joint angle <=95 deg in filtered evaluation; x8 resample; smooth_window=40; segment_key none.',
        'parameters': {
            'surface_sequence': '1,5,6,2,1',
            'move_dist': 0.40,
            'support_dist': 0.77,
            'legacy_body_z': 0.40,
            'max_step': 30,
            'goal2_dist_front': 0.40,
            'goal2_x_scale': 0.95,
            'goal2_pitch_scale': 0.90,
            'goal2_landing_z': 0.00,
            'goal3_lift_z': 0.05,
            'goal3_target_x': 0.20,
            'goal4_target_x': 0.05,
            'goal5_x_scale': 1.00,
            'goal5_pitch_scale': 1.00,
            'rf1_current_angle_anchor': True,
            'resample_factor': 8,
            'smooth_window': 40,
            'segment_key': '',
        },
        'artifacts': {
            'raw_command_log': raw_log,
            'filtered_command_log': filtered_log,
            'generation_report': raw_report,
            'resample_summary': smooth_report,
        },
        'gazebo_replay_command': 'python tools/gazebo/run_v3_0_gazebo_replay.py --command-log %s --strict-command-log-input --rate 15 --hold-start-sec 2.0 --hold-end-sec 2.0 --diagnose-command-log' % filtered_log,
    }
    with open(manifest, 'w') as f:
        json.dump(info, f, indent=2, sort_keys=True)
    print(json.dumps(info, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
