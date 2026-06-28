#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse, json, os, sys, warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.legacy_state_machine_emulator import LegacyStateMachineConfig, LegacyStateMachineEmulator, write_jsonl, command_diagnostics
from lily_motion_v3.command_resampler import (
    load_command_records, write_command_records, resample_command_records,
    moving_average_command_records, full_command_diagnostics, boundary_transition_diagnostics)
from lily_motion_v3.legacy_constraint_evaluator import LegacyConstraintEvaluator


def parse_int_list(s):
    return [int(x) for x in str(s).split(',') if str(x).strip()]


def _ensure_dir(path):
    d = os.path.dirname(path)
    if d and not os.path.isdir(d):
        os.makedirs(d)


def _subsample(records, stride):
    stride = max(1, int(stride or 1))
    out = records[::stride]
    if records and (not out or out[-1] is not records[-1]):
        out = out + [records[-1]]
    return out


def _evaluate_constraints(records, args, label):
    try:
        eval_records = _subsample(records, args.constraint_stride)
        ev = LegacyConstraintEvaluator(
            second_joint_limit_deg=args.second_joint_abs_max_deg,
            ground_z=args.ground_z,
            ground_tol=args.ground_tolerance,
            inter_leg_limit_m=args.inter_leg_limit,
            default_body_z=args.legacy_body_z,
            leg_radius_m=args.inter_leg_link_radius,
            inter_leg_safety_margin_m=args.inter_leg_safety_margin,
            joint_housing_radius_m=args.inter_leg_joint_housing_radius,
            joint_housing_safety_margin_m=args.inter_leg_joint_housing_safety_margin,
        )
        rep = ev.evaluate(eval_records, top_n=args.top_n)
        rep['label'] = label
        rep['constraint_stride'] = max(1, int(args.constraint_stride or 1))
        rep['evaluated_frame_count'] = len(eval_records)
        rep['source_frame_count'] = len(records)
        return rep
    except Exception as e:
        return {
            'label': label,
            'error': {'type': e.__class__.__name__, 'message': str(e)},
            'source_frame_count': len(records),
        }


def _make_legacy_repeated_records(args):
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
        rf1_current_angle_anchor=args.rf1_current_angle_anchor,
    )
    emu = LegacyStateMachineEmulator(cfg)
    try:
        records = emu.run_forward_repeated(surface_sequence=seq, include_initialize=args.include_initialize)
        err = None
        completed = True
    except Exception as e:
        records = list(getattr(emu, '_records', []))
        err = {'type': e.__class__.__name__, 'message': str(e)}
        completed = False
    return records, completed, err


def main():
    ap = argparse.ArgumentParser(description='v3.0.38 baseline evaluation: generate v3.0.36 pure legacy repeated roll, smooth the actual Gazebo command, and evaluate raw/filtered constraints including second-joint floor clearance.')
    ap.add_argument('--command-log', default=None, help='Optional existing raw JSONL command log. If omitted, generate baseline pure legacy repeated roll.')
    ap.add_argument('--surface-sequence', default='1,5,6,2,1')
    ap.add_argument('--move-dist', type=float, default=0.4)
    ap.add_argument('--support-dist', type=float, default=0.7)
    ap.add_argument('--legacy-body-z', type=float, default=0.35)
    ap.add_argument('--max-step', type=int, default=30)
    ap.add_argument('--initialize-step', type=int, default=100)
    ap.add_argument('--include-initialize', action='store_true')
    ap.add_argument('--rf1-current-angle-anchor', action='store_true', default=True,
                    help='Use v3.0.36 RF-1 current-angle anchor. Default: enabled.')
    ap.add_argument('--no-rf1-current-angle-anchor', action='store_false', dest='rf1_current_angle_anchor')
    ap.add_argument('--goal2-dist-front', type=float, default=0.4)
    ap.add_argument('--goal2-x-scale', type=float, default=1.0)
    ap.add_argument('--goal2-pitch-scale', type=float, default=1.0)
    ap.add_argument('--goal2-landing-z', type=float, default=0.0)
    ap.add_argument('--goal3-lift-z', type=float, default=0.05)
    ap.add_argument('--goal3-target-x', type=float, default=0.2)
    ap.add_argument('--goal4-target-x', type=float, default=0.05)
    ap.add_argument('--goal5-x-scale', type=float, default=1.0)
    ap.add_argument('--goal5-pitch-scale', type=float, default=1.0)

    ap.add_argument('--resample-factor', type=int, default=8)
    ap.add_argument('--smooth-window', type=int, default=40)
    ap.add_argument('--segment-key', default='', help='Usually leave empty for the current baseline: smooth across the whole 4-roll command stream.')
    ap.add_argument('--second-joint-abs-max-deg', type=float, default=95.0)
    ap.add_argument('--ground-z', type=float, default=0.0)
    ap.add_argument('--ground-tolerance', type=float, default=1e-4)
    ap.add_argument('--inter-leg-limit', type=float, default=0.04,
                    help='Backward-compatible near threshold. v3.0.39+ uses max(this, 2*radius+margin).')
    ap.add_argument('--inter-leg-link-radius', type=float, default=0.015,
                    help='Capsule radius for each leg link [m]. Collision threshold is 2*radius.')
    ap.add_argument('--inter-leg-safety-margin', type=float, default=0.010,
                    help='Additional safety margin [m]. Required clearance is max(inter_leg_limit, 2*radius+margin).')
    ap.add_argument('--inter-leg-joint-housing-radius', type=float, default=0.030,
                    help='Sphere radius for second-third joint/lower-link-root housing [m].')
    ap.add_argument('--inter-leg-joint-housing-safety-margin', type=float, default=0.005,
                    help='Safety margin for joint housing vs other-leg link clearance [m].')
    ap.add_argument('--constraint-stride', type=int, default=8, help='Evaluate every Nth frame for speed; final frame is always included.')
    ap.add_argument('--top-n', type=int, default=20)

    ap.add_argument('--output-raw-command-log', default='testdata/v3_0_37_baseline_raw_commands.jsonl')
    ap.add_argument('--output-filtered-command-log', default='testdata/v3_0_37_baseline_x8_sw40_commands.jsonl')
    ap.add_argument('--report-output', default='testdata/v3_0_37_baseline_filtered_constraint_report.json')
    args = ap.parse_args()

    json_stdout = sys.stdout
    sys.stdout = sys.stderr

    if args.command_log:
        raw_records = load_command_records(args.command_log)
        candidate_completed = None
        generation_error = None
        command_source = args.command_log
    else:
        raw_records, candidate_completed, generation_error = _make_legacy_repeated_records(args)
        command_source = 'generated_v3_0_36_pure_legacy_repeated_roll'

    if args.output_raw_command_log:
        write_jsonl(raw_records, args.output_raw_command_log)

    segment_key = args.segment_key.strip() or None
    filtered_records = resample_command_records(raw_records, factor=args.resample_factor, segment_key=segment_key)
    filtered_records = moving_average_command_records(filtered_records, window=args.smooth_window, segment_key=segment_key)
    if args.output_filtered_command_log:
        write_command_records(filtered_records, args.output_filtered_command_log)

    raw_diag = full_command_diagnostics(raw_records)
    filtered_diag = full_command_diagnostics(filtered_records)
    raw_boundaries = boundary_transition_diagnostics(raw_records, segment_key='roll_index')
    filtered_boundaries = boundary_transition_diagnostics(filtered_records, segment_key='roll_index')

    raw_constraints = _evaluate_constraints(raw_records, args, 'raw')
    filtered_constraints = _evaluate_constraints(filtered_records, args, 'filtered_gazebo_command')

    report = {
        'version_note': 'v3.0.40: baseline filtered constraint evaluation with part-wise floor clearance and capsule-based inter-leg collision distance. The second-joint/knee point is transformed to world coordinates with body pitch before checking the fixed Gazebo ground plane z=0. smooth_window=40 baseline; smoothing is applied to the full 4-roll command stream unless --segment-key is explicitly set.',
        'profile': 'baseline_filtered_constraint_evaluation',
        'command_source': command_source,
        'candidate_completed': candidate_completed,
        'generation_error': generation_error,
        'baseline_parameters': {
            'surface_sequence': args.surface_sequence,
            'move_dist': args.move_dist,
            'support_dist': args.support_dist,
            'legacy_body_z': args.legacy_body_z,
            'max_step': args.max_step,
            'rf1_current_angle_anchor': args.rf1_current_angle_anchor,
            'resample_factor': args.resample_factor,
            'smooth_window': args.smooth_window,
            'segment_key': args.segment_key,
            'constraint_stride': args.constraint_stride,
            'inter_leg_link_radius': args.inter_leg_link_radius,
            'inter_leg_safety_margin': args.inter_leg_safety_margin,
            'inter_leg_limit': args.inter_leg_limit,
            'inter_leg_joint_housing_radius': args.inter_leg_joint_housing_radius,
            'inter_leg_joint_housing_safety_margin': args.inter_leg_joint_housing_safety_margin,
        },
        'raw_command_log': args.output_raw_command_log,
        'filtered_command_log': args.output_filtered_command_log,
        'raw': {
            'frame_count': len(raw_records),
            'command_diagnostics': {
                'max_adjacent_delta_deg': raw_diag.get('max_adjacent_delta_deg'),
                'max_delta_deg': raw_diag.get('max_delta_deg'),
                'worst_transition': raw_diag.get('worst_transition'),
            },
            'boundary_diagnostics': raw_boundaries,
            'constraints': raw_constraints,
        },
        'filtered': {
            'frame_count': len(filtered_records),
            'command_diagnostics': {
                'max_adjacent_delta_deg': filtered_diag.get('max_adjacent_delta_deg'),
                'max_delta_deg': filtered_diag.get('max_delta_deg'),
                'worst_transition': filtered_diag.get('worst_transition'),
            },
            'boundary_diagnostics': filtered_boundaries,
            'constraints': filtered_constraints,
        },
        'interpretation_note': 'Use filtered.filtered_command_log for Gazebo replay. Compare raw.constraints and filtered.constraints. For floor contact, prefer filtered.constraints.second_joint_clearance / clearance_by_part.second_joint over total ground_penetration_count, because foot penetration can be a smoothing artifact while second-joint floor penetration is a primary rejection criterion.',
    }

    if args.report_output:
        _ensure_dir(args.report_output)
        with open(args.report_output, 'w') as f:
            json.dump(report, f, indent=2, sort_keys=True)
    sys.stdout = json_stdout
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
