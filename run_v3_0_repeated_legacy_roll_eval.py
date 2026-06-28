#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse, json, os, sys, warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.command_resampler import write_command_records, resample_command_records, moving_average_command_records
from lily_motion_v3.legacy_state_machine_emulator import LegacyStateMachineConfig, LegacyStateMachineEmulator
from lily_motion_v3.legacy_constraint_evaluator import LegacyConstraintEvaluator
from lily_motion_v3.repeated_roll_connection import connection_report


def parse_float_list(s):
    return [float(x) for x in str(s).split(',') if str(x).strip()]


def parse_int_list(s):
    return [int(x) for x in str(s).split(',') if str(x).strip()]


def main():
    ap = argparse.ArgumentParser(description='Evaluate repeated legacy quarter-roll connection over a surface sequence such as 1,5,6,2,1.')
    ap.add_argument('--surface-sequence', default='1,5,6,2,1')
    ap.add_argument('--move-dist', type=float, default=0.4)
    ap.add_argument('--support-dist', type=float, default=0.7)
    ap.add_argument('--max-step', type=int, default=30)
    ap.add_argument('--legacy-body-z', type=float, default=0.35)
    ap.add_argument('--initialize-step', type=int, default=100)
    ap.add_argument('--include-initialize', action='store_true')
    ap.add_argument('--goal2-dist-front', type=float, default=0.4)
    ap.add_argument('--goal2-x-scale', type=float, default=1.0)
    ap.add_argument('--goal2-pitch-scale', type=float, default=1.0)
    ap.add_argument('--goal2-landing-z', type=float, default=0.0)
    ap.add_argument('--goal3-lift-z', type=float, default=0.05)
    ap.add_argument('--goal3-target-x', type=float, default=0.2)
    ap.add_argument('--goal4-target-x', type=float, default=0.05)
    ap.add_argument('--goal5-x-scale', type=float, default=1.0)
    ap.add_argument('--goal5-pitch-scale', type=float, default=1.0)
    ap.add_argument('--resample-factor', type=int, default=1)
    ap.add_argument('--smooth-window', type=int, default=1)
    ap.add_argument('--second-joint-abs-max-deg', type=float, default=95.0)
    ap.add_argument('--inter-leg-limit', type=float, default=0.04)
    ap.add_argument('--ground-tolerance', type=float, default=1e-4)
    ap.add_argument('--constraint-stride', type=int, default=4, help='Evaluate constraints every N frames for speed. Use 1 for full evaluation.')
    ap.add_argument('--skip-constraints', action='store_true', help='Skip full constraint geometry evaluation and report only connection metrics.')
    ap.add_argument('--output-command-log', default=None)
    ap.add_argument('--report-output', default=None)
    args = ap.parse_args()

    # The vendored legacy runtime prints setup messages such as link lengths.
    # Keep stdout as clean JSON for shell pipelines; send runtime chatter to stderr.
    _json_stdout = sys.stdout
    sys.stdout = sys.stderr

    seq = parse_int_list(args.surface_sequence)
    cfg = LegacyStateMachineConfig(
        move_dist=args.move_dist,
        support_dist=args.support_dist,
        max_step=args.max_step,
        surface_id=seq[0],
        z=args.legacy_body_z,
        initialize_step=args.initialize_step,
        include_initialize=False,
        goal2_dist_front=args.goal2_dist_front,
        goal2_x_scale=args.goal2_x_scale,
        goal2_pitch_scale=args.goal2_pitch_scale,
        goal2_landing_z=args.goal2_landing_z,
        goal3_lift_z=args.goal3_lift_z,
        goal3_target_x=args.goal3_target_x,
        goal4_target_x=args.goal4_target_x,
        goal5_x_scale=args.goal5_x_scale,
        goal5_pitch_scale=args.goal5_pitch_scale,
    )
    emu = LegacyStateMachineEmulator(cfg)
    generation_error = None
    candidate_completed = True
    try:
        records = emu.run_forward_repeated(surface_sequence=seq, include_initialize=args.include_initialize)
    except Exception as e:
        generation_error = {'type': e.__class__.__name__, 'message': str(e)}
        candidate_completed = False
        records = list(getattr(emu, '_records', []))
    raw_count = len(records)
    if args.resample_factor and args.resample_factor > 1:
        records = resample_command_records(records, factor=args.resample_factor)
    if args.smooth_window and args.smooth_window > 1:
        records = moving_average_command_records(records, window=args.smooth_window)
    if args.output_command_log:
        write_command_records(records, args.output_command_log)

    if args.skip_constraints:
        constraints = {'skipped': True}
    else:
        ev = LegacyConstraintEvaluator(
            second_joint_limit_deg=args.second_joint_abs_max_deg,
            ground_tol=args.ground_tolerance,
            inter_leg_limit_m=args.inter_leg_limit,
            default_body_z=args.legacy_body_z,
        )
        stride = max(1, int(args.constraint_stride))
        eval_records = records[::stride]
        if records and eval_records[-1] is not records[-1]:
            eval_records = eval_records + [records[-1]]
        constraints = ev.evaluate(eval_records)
        constraints['constraint_stride'] = stride
        constraints['evaluated_frame_count'] = len(eval_records)
        constraints['constraint_note'] = 'Use --constraint-stride 1 for exact full-frame constraint evaluation.'
    conn = connection_report(records, default_body_z=args.legacy_body_z)
    report = {
        'version_note': 'v3.0.27: repeated legacy roll connection evaluation over multiple surface transitions.',
        'profile': 'repeated_legacy_roll_connection',
        'surface_sequence': seq,
        'quarter_roll_count': max(0, len(seq)-1),
        'candidate_completed': candidate_completed,
        'generation_error': generation_error,
        'raw_frame_count_before_resampling': raw_count,
        'frame_count': len(records),
        'parameters': {
            'move_dist': args.move_dist,
            'support_dist': args.support_dist,
            'legacy_body_z': args.legacy_body_z,
            'max_step': args.max_step,
            'goal2_pitch_scale': args.goal2_pitch_scale,
            'goal2_x_scale': args.goal2_x_scale,
            'goal5_pitch_scale': args.goal5_pitch_scale,
            'goal5_x_scale': args.goal5_x_scale,
            'resample_factor': args.resample_factor,
            'smooth_window': args.smooth_window,
            'constraint_stride': args.constraint_stride,
        },
        'constraints': constraints,
        'connection': conn,
        'output_command_log': args.output_command_log,
    }
    if args.report_output:
        d = os.path.dirname(args.report_output)
        if d and not os.path.isdir(d):
            os.makedirs(d)
        with open(args.report_output, 'w') as f:
            json.dump(report, f, indent=2, sort_keys=True)
    sys.stdout = _json_stdout
    print(json.dumps(report, indent=2, sort_keys=True))

if __name__ == '__main__':
    main()
