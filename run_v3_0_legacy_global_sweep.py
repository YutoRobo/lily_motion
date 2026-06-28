#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division
import argparse, json, os, sys, itertools
ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.legacy_state_machine_emulator import LegacyStateMachineConfig, LegacyStateMachineEmulator
from lily_motion_v3.command_resampler import resample_command_records, moving_average_command_records, write_command_records
from lily_motion_v3.legacy_constraint_evaluator import LegacyConstraintEvaluator
from run_v3_0_legacy_rf2_sweep import parse_float_list, score_report


def generate_records(args, case):
    cfg = LegacyStateMachineConfig(
        move_dist=case['move_dist'],
        support_dist=case['support_dist'],
        max_step=args.max_step,
        surface_id=args.surface_id,
        z=case['legacy_body_z'],
        initialize_step=args.initialize_step,
        include_initialize=args.include_initialize,
        goal2_dist_front=args.goal2_dist_front,
        goal2_x_scale=args.goal2_x_scale,
        goal2_pitch_scale=case['goal2_pitch_scale'],
        goal2_landing_z=args.goal2_landing_z,
        goal3_lift_z=args.goal3_lift_z,
        goal3_target_x=args.goal3_target_x,
        goal4_target_x=args.goal4_target_x,
        goal5_x_scale=args.goal5_x_scale,
        goal5_pitch_scale=args.goal5_pitch_scale,
    )
    records = LegacyStateMachineEmulator(cfg).run_forward_roll()
    if args.resample_factor and args.resample_factor > 1:
        records = resample_command_records(records, factor=args.resample_factor)
    if args.smooth_window and args.smooth_window > 1:
        records = moving_average_command_records(records, window=args.smooth_window)
    return records


def main():
    ap = argparse.ArgumentParser(description='Global posture sweep for legacy roll: support_dist/body_z/move_dist/goal2_pitch_scale.')
    ap.add_argument('--surface-id', type=int, default=1)
    ap.add_argument('--support-dists', default='0.60,0.65,0.70,0.75')
    ap.add_argument('--legacy-body-zs', default='0.30,0.35,0.40')
    ap.add_argument('--move-dists', default='0.30,0.35,0.40')
    ap.add_argument('--goal2-pitch-scales', default='0.70,0.85,1.0')
    ap.add_argument('--max-step', type=int, default=30)
    ap.add_argument('--initialize-step', type=int, default=100)
    ap.add_argument('--include-initialize', action='store_true')
    # Keep the previous RF-2/3/4 knobs fixed unless explicitly changed.
    ap.add_argument('--goal2-dist-front', type=float, default=0.3)
    ap.add_argument('--goal2-x-scale', type=float, default=1.0)
    ap.add_argument('--goal2-landing-z', type=float, default=0.0)
    ap.add_argument('--goal3-lift-z', type=float, default=0.05)
    ap.add_argument('--goal3-target-x', type=float, default=0.2)
    ap.add_argument('--goal4-target-x', type=float, default=0.05)
    ap.add_argument('--goal5-x-scale', type=float, default=1.0)
    ap.add_argument('--goal5-pitch-scale', type=float, default=1.0)
    ap.add_argument('--resample-factor', type=int, default=1)
    ap.add_argument('--smooth-window', type=int, default=1)
    ap.add_argument('--second-joint-abs-max-deg', type=float, default=95.0)
    ap.add_argument('--ground-z', type=float, default=0.0)
    ap.add_argument('--ground-tolerance', type=float, default=1e-4)
    ap.add_argument('--inter-leg-limit', type=float, default=0.04)
    ap.add_argument('--top-n', type=int, default=10)
    ap.add_argument('--output', default='testdata/v3_0_26_global_sweep.json')
    ap.add_argument('--best-command-output', default=None)
    ap.add_argument('--best-report-output', default=None)
    args = ap.parse_args()

    lists = {
        'support_dist': parse_float_list(args.support_dists),
        'legacy_body_z': parse_float_list(args.legacy_body_zs),
        'move_dist': parse_float_list(args.move_dists),
        'goal2_pitch_scale': parse_float_list(args.goal2_pitch_scales),
    }
    keys = ['support_dist', 'legacy_body_z', 'move_dist', 'goal2_pitch_scale']
    results=[]; best=None; best_records=None; best_report=None
    for vals in itertools.product(*[lists[k] for k in keys]):
        case = dict(zip(keys, vals))
        records = generate_records(args, case)
        ev = LegacyConstraintEvaluator(
            second_joint_limit_deg=args.second_joint_abs_max_deg,
            ground_z=args.ground_z,
            ground_tol=args.ground_tolerance,
            inter_leg_limit_m=args.inter_leg_limit,
            default_body_z=case['legacy_body_z'],
        )
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
            best=item; best_records=records; best_report=report
    results_sorted=sorted(results, key=lambda x: x['score'])
    out={
        'version_note':'v3.0.26: global posture sweep plus optional fixed RF-2/RF-5 knobs.',
        'case_count':len(results_sorted),
        'best_case':best,
        'top_cases':results_sorted[:max(1,args.top_n)],
        'sweep_parameters':lists,
        'fixed_parameters':{
            'surface_id':args.surface_id,'max_step':args.max_step,
            'goal2_dist_front':args.goal2_dist_front,'goal2_x_scale':args.goal2_x_scale,'goal2_landing_z':args.goal2_landing_z,
            'goal3_lift_z':args.goal3_lift_z,'goal3_target_x':args.goal3_target_x,'goal4_target_x':args.goal4_target_x,
            'goal5_x_scale':args.goal5_x_scale,'goal5_pitch_scale':args.goal5_pitch_scale,
            'resample_factor':args.resample_factor,'smooth_window':args.smooth_window,
            'second_joint_abs_max_deg':args.second_joint_abs_max_deg,'inter_leg_limit':args.inter_leg_limit,
        }
    }
    d=os.path.dirname(args.output)
    if d and not os.path.isdir(d): os.makedirs(d)
    with open(args.output,'w') as f: json.dump(out,f,indent=2,sort_keys=True)
    if args.best_command_output and best_records is not None:
        write_command_records(best_records,args.best_command_output)
    if args.best_report_output and best_report is not None:
        d=os.path.dirname(args.best_report_output)
        if d and not os.path.isdir(d): os.makedirs(d)
        with open(args.best_report_output,'w') as f: json.dump(best_report,f,indent=2,sort_keys=True)
    print(json.dumps({'case_count':len(results_sorted),'best_case':best,'output':args.output,'best_command_output':args.best_command_output,'best_report_output':args.best_report_output},indent=2,sort_keys=True))

if __name__ == '__main__':
    main()
