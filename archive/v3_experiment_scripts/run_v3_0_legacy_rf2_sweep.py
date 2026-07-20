#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division
import argparse, json, os, sys, itertools, math
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.legacy_state_machine_emulator import LegacyStateMachineConfig, LegacyStateMachineEmulator, write_jsonl
from lily_motion_v3.command_resampler import resample_command_records, moving_average_command_records, write_command_records
from lily_motion_v3.legacy_constraint_evaluator import LegacyConstraintEvaluator


def parse_float_list(text):
    if text is None or str(text).strip() == '':
        return []
    return [float(x) for x in str(text).split(',') if str(x).strip() != '']


def score_report(report):
    # Hard priority: remove second-joint violations first, then avoid ground and inter-leg issues.
    # Keep the max second-joint angle in the score even after violations disappear.
    return (
        report.get('second_joint_violation_count', 0) * 1000000.0 +
        max(0.0, report.get('max_second_joint_deg', 0.0) - report.get('second_joint_limit_deg', 95.0)) * 10000.0 +
        report.get('ground_penetration_count', 0) * 1000.0 +
        report.get('inter_leg_near_count', 0) * 100.0 +
        report.get('max_second_joint_deg', 0.0)
    )


def generate_records(args, case):
    cfg = LegacyStateMachineConfig(
        move_dist=args.move_dist,
        support_dist=args.support_dist,
        max_step=args.max_step,
        surface_id=args.surface_id,
        z=args.legacy_body_z,
        initialize_step=args.initialize_step,
        include_initialize=args.include_initialize,
        goal2_dist_front=case['goal2_dist_front'],
        goal2_x_scale=case['goal2_x_scale'],
        goal2_pitch_scale=case['goal2_pitch_scale'],
        goal2_landing_z=case['goal2_landing_z'],
        goal3_lift_z=case['goal3_lift_z'],
        goal3_target_x=case['goal3_target_x'],
        goal4_target_x=case['goal4_target_x'],
    )
    emu = LegacyStateMachineEmulator(cfg)
    records = emu.run_forward_roll()
    if args.resample_factor and args.resample_factor > 1:
        records = resample_command_records(records, factor=args.resample_factor)
    if args.smooth_window and args.smooth_window > 1:
        records = moving_average_command_records(records, window=args.smooth_window)
    return records


def main():
    ap = argparse.ArgumentParser(description='Sweep narrow RF-2/nearby legacy-roll parameters to reduce second-joint violations without changing the state-machine structure.')
    ap.add_argument('--surface-id', type=int, default=1)
    ap.add_argument('--move-dist', type=float, default=0.4)
    ap.add_argument('--support-dist', type=float, default=0.7)
    ap.add_argument('--max-step', type=int, default=30)
    ap.add_argument('--legacy-body-z', type=float, default=0.35)
    ap.add_argument('--initialize-step', type=int, default=100)
    ap.add_argument('--include-initialize', action='store_true')
    ap.add_argument('--goal2-dist-fronts', default='0.30,0.35,0.40,0.45,0.50')
    ap.add_argument('--goal2-x-scales', default='0.8,1.0,1.2')
    ap.add_argument('--goal2-pitch-scales', default='0.8,0.9,1.0,1.1')
    ap.add_argument('--goal2-landing-zs', default='0.0,0.03,0.06')
    ap.add_argument('--goal3-lift-zs', default='0.05')
    ap.add_argument('--goal3-target-xs', default='0.2')
    ap.add_argument('--goal4-target-xs', default='0.05')
    ap.add_argument('--resample-factor', type=int, default=1)
    ap.add_argument('--smooth-window', type=int, default=1)
    ap.add_argument('--second-joint-abs-max-deg', type=float, default=95.0)
    ap.add_argument('--ground-z', type=float, default=0.0)
    ap.add_argument('--ground-tolerance', type=float, default=1e-4)
    ap.add_argument('--inter-leg-limit', type=float, default=0.04)
    ap.add_argument('--top-n', type=int, default=10)
    ap.add_argument('--output', default='testdata/v3_0_25_rf2_sweep.json')
    ap.add_argument('--best-command-output', default=None)
    ap.add_argument('--best-report-output', default=None)
    args = ap.parse_args()

    lists = {
        'goal2_dist_front': parse_float_list(args.goal2_dist_fronts),
        'goal2_x_scale': parse_float_list(args.goal2_x_scales),
        'goal2_pitch_scale': parse_float_list(args.goal2_pitch_scales),
        'goal2_landing_z': parse_float_list(args.goal2_landing_zs),
        'goal3_lift_z': parse_float_list(args.goal3_lift_zs),
        'goal3_target_x': parse_float_list(args.goal3_target_xs),
        'goal4_target_x': parse_float_list(args.goal4_target_xs),
    }
    keys = ['goal2_dist_front','goal2_x_scale','goal2_pitch_scale','goal2_landing_z','goal3_lift_z','goal3_target_x','goal4_target_x']
    ev = LegacyConstraintEvaluator(
        second_joint_limit_deg=args.second_joint_abs_max_deg,
        ground_z=args.ground_z,
        ground_tol=args.ground_tolerance,
        inter_leg_limit_m=args.inter_leg_limit,
        default_body_z=args.legacy_body_z,
    )
    results = []
    best = None
    best_records = None
    best_report = None
    for vals in itertools.product(*[lists[k] for k in keys]):
        case = dict(zip(keys, vals))
        records = generate_records(args, case)
        report = ev.evaluate(records, top_n=3)
        sc = score_report(report)
        item = {
            'case': case,
            'score': sc,
            'frame_count': report.get('frame_count'),
            'max_second_joint_deg': report.get('max_second_joint_deg'),
            'second_joint_violation_count': report.get('second_joint_violation_count'),
            'ground_penetration_count': report.get('ground_penetration_count'),
            'inter_leg_near_count': report.get('inter_leg_near_count'),
            'worst_second_joint': report.get('worst_second_joint'),
            'worst_inter_leg_distance': report.get('worst_inter_leg_distance'),
            'phase_summary': report.get('phase_summary'),
        }
        results.append(item)
        if best is None or sc < best['score']:
            best = item
            best_records = records
            best_report = report
    results_sorted = sorted(results, key=lambda x: x['score'])
    out = {
        'version_note': 'v3.0.25: RF-2 focused diagnostic sweep for legacy state-machine commands; defaults preserve the supplied legacy controller.',
        'case_count': len(results_sorted),
        'best_case': best,
        'top_cases': results_sorted[:max(1, args.top_n)],
        'sweep_parameters': lists,
        'global_parameters': {
            'surface_id': args.surface_id,
            'move_dist': args.move_dist,
            'support_dist': args.support_dist,
            'max_step': args.max_step,
            'legacy_body_z': args.legacy_body_z,
            'resample_factor': args.resample_factor,
            'smooth_window': args.smooth_window,
            'second_joint_abs_max_deg': args.second_joint_abs_max_deg,
            'inter_leg_limit': args.inter_leg_limit,
        },
    }
    d = os.path.dirname(args.output)
    if d and not os.path.isdir(d):
        os.makedirs(d)
    with open(args.output, 'w') as f:
        json.dump(out, f, indent=2, sort_keys=True)
    if args.best_command_output and best_records is not None:
        write_command_records(best_records, args.best_command_output)
    if args.best_report_output and best_report is not None:
        dd = os.path.dirname(args.best_report_output)
        if dd and not os.path.isdir(dd):
            os.makedirs(dd)
        with open(args.best_report_output, 'w') as f:
            json.dump(best_report, f, indent=2, sort_keys=True)
    print(json.dumps({
        'case_count': len(results_sorted),
        'best_case': best,
        'output': args.output,
        'best_command_output': args.best_command_output,
        'best_report_output': args.best_report_output,
    }, indent=2, sort_keys=True))

if __name__ == '__main__':
    main()
