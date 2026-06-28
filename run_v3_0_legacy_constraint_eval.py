#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse, json, os, sys
ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.command_resampler import load_command_records, write_command_records, resample_command_records, moving_average_command_records
from lily_motion_v3.legacy_state_machine_emulator import LegacyStateMachineConfig, LegacyStateMachineEmulator, write_jsonl
from lily_motion_v3.legacy_constraint_evaluator import LegacyConstraintEvaluator


def _generate_records(args):
    cfg = LegacyStateMachineConfig(
        move_dist=args.move_dist,
        support_dist=args.support_dist,
        max_step=args.max_step,
        surface_id=args.surface_id,
        z=args.legacy_body_z,
        initialize_step=args.initialize_step,
        include_initialize=args.include_initialize,
        goal2_dist_front=args.goal2_dist_front,
        goal2_x_scale=args.goal2_x_scale,
        goal2_pitch_scale=args.goal2_pitch_scale,
        goal2_landing_z=args.goal2_landing_z,
        goal3_lift_z=args.goal3_lift_z,
        goal3_target_x=args.goal3_target_x,
        goal4_target_x=args.goal4_target_x,
        goal5_x_scale=getattr(args, 'goal5_x_scale', 1.0),
        goal5_pitch_scale=getattr(args, 'goal5_pitch_scale', 1.0),
    )
    emu = LegacyStateMachineEmulator(cfg)
    return emu.run_forward_roll()


def main():
    ap = argparse.ArgumentParser(description='Evaluate legacy-state-machine command constraints: second joint, ground clearance, and inter-leg proximity.')
    ap.add_argument('--command-log', default=None, help='Existing JSONL with joint_command_rad. If omitted, generate legacy-state-machine commands first.')
    ap.add_argument('--output-command-log', default=None, help='Optional path to save generated/processed command log.')
    ap.add_argument('--report-output', default=None, help='Optional JSON report path.')
    ap.add_argument('--surface-id', type=int, default=1)
    ap.add_argument('--move-dist', type=float, default=0.4)
    ap.add_argument('--support-dist', type=float, default=0.7)
    ap.add_argument('--max-step', type=int, default=30)
    ap.add_argument('--legacy-body-z', type=float, default=0.35)
    ap.add_argument('--initialize-step', type=int, default=100)
    ap.add_argument('--include-initialize', action='store_true')
    ap.add_argument('--goal2-dist-front', type=float, default=0.4, help='RF-2 landing x margin. Default matches legacy code.')
    ap.add_argument('--goal2-x-scale', type=float, default=1.0, help='Scale RF-2 support x progress. Diagnostic knob.')
    ap.add_argument('--goal2-pitch-scale', type=float, default=1.0, help='Scale RF-2 body pitch progress. Diagnostic knob.')
    ap.add_argument('--goal2-landing-z', type=float, default=0.0, help='RF-2 landing target z. Default matches legacy code.')
    ap.add_argument('--goal3-lift-z', type=float, default=0.05, help='RF-3 middle-pair lift height. Default matches legacy code.')
    ap.add_argument('--goal3-target-x', type=float, default=0.2, help='RF-3 middle-pair x target. Default matches legacy code.')
    ap.add_argument('--goal4-target-x', type=float, default=0.05, help='RF-4 middle-pair landing x target. Default matches legacy code.')
    ap.add_argument('--goal5-x-scale', type=float, default=1.0, help='Scale RF-5 support x progress. Diagnostic knob.')
    ap.add_argument('--goal5-pitch-scale', type=float, default=1.0, help='Scale RF-5 body pitch progress. Diagnostic knob.')
    ap.add_argument('--resample-factor', type=int, default=1)
    ap.add_argument('--smooth-window', type=int, default=1)
    ap.add_argument('--second-joint-abs-max-deg', type=float, default=95.0)
    ap.add_argument('--ground-z', type=float, default=0.0)
    ap.add_argument('--ground-tolerance', type=float, default=1e-4)
    ap.add_argument('--inter-leg-limit', type=float, default=0.04)
    ap.add_argument('--top-n', type=int, default=20)
    args = ap.parse_args()

    if args.command_log:
        records = load_command_records(args.command_log)
        command_source = args.command_log
    else:
        records = _generate_records(args)
        command_source = 'generated_legacy_state_machine'

    if args.resample_factor and args.resample_factor > 1:
        records = resample_command_records(records, factor=args.resample_factor)
    if args.smooth_window and args.smooth_window > 1:
        records = moving_average_command_records(records, window=args.smooth_window)

    if args.output_command_log:
        write_command_records(records, args.output_command_log)

    ev = LegacyConstraintEvaluator(
        second_joint_limit_deg=args.second_joint_abs_max_deg,
        ground_z=args.ground_z,
        ground_tol=args.ground_tolerance,
        inter_leg_limit_m=args.inter_leg_limit,
        default_body_z=args.legacy_body_z,
    )
    report = ev.evaluate(records, top_n=args.top_n)
    report.update({
        'version_note': 'v3.0.25: legacy-state-machine constraint evaluation using vendored legacy FK; no old-project import.',
        'profile': 'legacy_constraint_evaluation',
        'command_source': command_source,
        'output_command_log': args.output_command_log,
        'surface_id': args.surface_id,
        'move_dist': args.move_dist,
        'support_dist': args.support_dist,
        'max_step': args.max_step,
        'include_initialize': args.include_initialize,
        'resample_factor': args.resample_factor,
        'smooth_window': args.smooth_window,
        'goal2_dist_front': args.goal2_dist_front,
        'goal2_x_scale': args.goal2_x_scale,
        'goal2_pitch_scale': args.goal2_pitch_scale,
        'goal2_landing_z': args.goal2_landing_z,
        'goal3_lift_z': args.goal3_lift_z,
        'goal3_target_x': args.goal3_target_x,
        'goal4_target_x': args.goal4_target_x,
        'goal5_x_scale': args.goal5_x_scale,
        'goal5_pitch_scale': args.goal5_pitch_scale,
    })
    if args.report_output:
        d = os.path.dirname(args.report_output)
        if d and not os.path.isdir(d):
            os.makedirs(d)
        with open(args.report_output, 'w') as f:
            json.dump(report, f, indent=2, sort_keys=True)
    print(json.dumps(report, indent=2, sort_keys=True))

if __name__ == '__main__':
    main()
