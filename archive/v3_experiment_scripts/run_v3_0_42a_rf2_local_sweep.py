#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division

"""v3.0.42A RF-2 local countermeasure sweep.

Roadmap position:
  Phase 3 -> Phase 4 preparation.

This script does not introduce a new gait generator.  It reuses the existing
LegacyStateMachineConfig RF-2 diagnostic knobs and evaluates 4 repeated rolls
with the same v3.0.41 second-joint localization logic.

Purpose:
  Find whether a minimal RF-2-only parameter change can reduce the remaining
  second-joint angle violation before changing global parameters such as
  body_z, support_dist, or move_dist.
"""

import argparse
import itertools
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.legacy_state_machine_emulator import write_jsonl
from lily_motion_v3.command_resampler import (
    resample_command_records, moving_average_command_records,
    write_command_records)
from lily_motion_v3.legacy_constraint_evaluator import LegacyConstraintEvaluator

import run_v3_0_41_second_joint_angle_localization as loc


def parse_float_list(text):
    if text is None or str(text).strip() == '':
        return []
    return [float(x) for x in str(text).split(',') if str(x).strip() != '']


def ensure_dir(path):
    d = os.path.dirname(path)
    if d and not os.path.isdir(d):
        os.makedirs(d)


class LocArgs(object):
    pass


def make_loc_args(args, case):
    a = LocArgs()
    a.surface_sequence = args.surface_sequence
    a.move_dist = args.move_dist
    a.support_dist = args.support_dist
    a.legacy_body_z = args.legacy_body_z
    a.max_step = args.max_step
    a.initialize_step = args.initialize_step
    a.include_initialize = args.include_initialize
    a.rf1_current_angle_anchor = args.rf1_current_angle_anchor
    a.goal2_dist_front = case['goal2_dist_front']
    a.goal2_x_scale = case['goal2_x_scale']
    a.goal2_pitch_scale = case['goal2_pitch_scale']
    a.goal2_landing_z = case['goal2_landing_z']
    a.goal3_lift_z = args.goal3_lift_z
    a.goal3_target_x = args.goal3_target_x
    a.goal4_target_x = args.goal4_target_x
    a.goal5_x_scale = args.goal5_x_scale
    a.goal5_pitch_scale = args.goal5_pitch_scale
    return a


def evaluate_constraint(records, args):
    stride = max(1, int(args.constraint_stride))
    eval_records = records[::stride]
    if records and (not eval_records or eval_records[-1] is not records[-1]):
        eval_records = eval_records + [records[-1]]
    ev = LegacyConstraintEvaluator(
        second_joint_limit_deg=args.second_joint_abs_max_deg,
        ground_tol=args.ground_tolerance,
        inter_leg_limit_m=args.inter_leg_limit,
        default_body_z=args.legacy_body_z)
    rep = ev.evaluate(eval_records)
    rep['constraint_stride'] = stride
    rep['evaluated_frame_count'] = len(eval_records)
    return rep


def summarize_source(records, source_label, args):
    return loc._analyze_source(
        records,
        source_label,
        args.second_joint_abs_max_deg,
        args.boundary_window,
        args.top_n)


def score_item(item, prefer_filtered):
    src = item['filtered'] if prefer_filtered and item.get('filtered') else item['raw']
    # Primary priority: lower maximum angle.  Secondary: fewer violation frames.
    # Keep penalties for obvious physical regressions when constraint summaries
    # are requested.
    score = src.get('max_abs_angle_deg', 999.0) * 1000000.0
    score += src.get('violation_frame_count', 999999) * 1000.0
    score += src.get('violation_count', 999999)
    con = item.get('constraint_filtered') or item.get('constraint_raw') or {}
    score += con.get('ground_penetration_count', 0) * 100000.0
    score += con.get('inter_leg_collision_count', 0) * 100000.0
    score += con.get('inter_leg_near_count', 0) * 10000.0
    return score


def compact_summary(src):
    worst = src.get('worst') or {}
    return {
        'frame_count': src.get('frame_count'),
        'second_joint_sample_count': src.get('second_joint_sample_count'),
        'max_abs_angle_deg': src.get('max_abs_angle_deg'),
        'max_excess_deg': src.get('max_excess_deg'),
        'violation_count': src.get('violation_count'),
        'violation_frame_count': src.get('violation_frame_count'),
        'violation_sample_rate': src.get('violation_sample_rate'),
        'violation_frame_rate': src.get('violation_frame_rate'),
        'worst': worst,
        'phase_summary': src.get('group_summary', {}).get('phase_name', []),
        'roll_phase_summary': src.get('group_summary', {}).get('roll_phase', [])[:12],
    }


def main():
    ap = argparse.ArgumentParser(description='v3.0.42A RF-2 local sweep for second-joint violation reduction. Diagnostic/candidate generation only.')
    ap.add_argument('--surface-sequence', default='1,5,6,2,1')
    ap.add_argument('--move-dist', type=float, default=0.4)
    ap.add_argument('--support-dist', type=float, default=0.7)
    ap.add_argument('--legacy-body-z', type=float, default=0.35)
    ap.add_argument('--max-step', type=int, default=30)
    ap.add_argument('--initialize-step', type=int, default=100)
    ap.add_argument('--include-initialize', action='store_true')
    ap.add_argument('--rf1-current-angle-anchor', action='store_true', default=True)
    ap.add_argument('--no-rf1-current-angle-anchor', action='store_false', dest='rf1_current_angle_anchor')

    # RF-2 local knobs.  Defaults include baseline plus small local perturbations.
    ap.add_argument('--goal2-dist-fronts', default='0.35,0.40,0.45')
    ap.add_argument('--goal2-x-scales', default='0.9,1.0,1.1')
    ap.add_argument('--goal2-pitch-scales', default='0.8,0.9,1.0')
    ap.add_argument('--goal2-landing-zs', default='0.0')

    # Keep adjacent phases at current baseline unless explicitly changed.
    ap.add_argument('--goal3-lift-z', type=float, default=0.05)
    ap.add_argument('--goal3-target-x', type=float, default=0.2)
    ap.add_argument('--goal4-target-x', type=float, default=0.05)
    ap.add_argument('--goal5-x-scale', type=float, default=1.0)
    ap.add_argument('--goal5-pitch-scale', type=float, default=1.0)

    ap.add_argument('--resample-factor', type=int, default=8)
    ap.add_argument('--smooth-window', type=int, default=40)
    ap.add_argument('--segment-key', default='', help='Leave empty for current baseline: do not split smoothing by roll_index.')
    ap.add_argument('--evaluate-filtered', action='store_true', default=True)
    ap.add_argument('--raw-only', action='store_false', dest='evaluate_filtered')
    ap.add_argument('--evaluate-constraints', action='store_true', help='Also run heavier FK constraint evaluator for each case.')
    ap.add_argument('--constraint-stride', type=int, default=4)
    ap.add_argument('--second-joint-abs-max-deg', type=float, default=95.0)
    ap.add_argument('--ground-tolerance', type=float, default=1e-4)
    ap.add_argument('--inter-leg-limit', type=float, default=0.04)
    ap.add_argument('--boundary-window', type=int, default=3)
    ap.add_argument('--top-n', type=int, default=10)
    ap.add_argument('--output', default='testdata/v3_0_42a_rf2_local_sweep.json')
    ap.add_argument('--best-raw-command-output', default='testdata/v3_0_42a_best_raw_commands.jsonl')
    ap.add_argument('--best-filtered-command-output', default='testdata/v3_0_42a_best_x8_sw40_commands.jsonl')
    args = ap.parse_args()

    lists = {
        'goal2_dist_front': parse_float_list(args.goal2_dist_fronts),
        'goal2_x_scale': parse_float_list(args.goal2_x_scales),
        'goal2_pitch_scale': parse_float_list(args.goal2_pitch_scales),
        'goal2_landing_z': parse_float_list(args.goal2_landing_zs),
    }
    keys = ['goal2_dist_front', 'goal2_x_scale', 'goal2_pitch_scale', 'goal2_landing_z']
    cases = [dict(zip(keys, vals)) for vals in itertools.product(*[lists[k] for k in keys])]

    results = []
    best = None
    best_raw_records = None
    best_filtered_records = None
    for idx, case in enumerate(cases):
        loc_args = make_loc_args(args, case)
        item = {'case_index': idx, 'case': case}
        try:
            raw_records, completed, error = loc._make_legacy_repeated_records(loc_args)
            item['candidate_completed'] = completed
            item['generation_error'] = error
            raw_summary = summarize_source(raw_records, 'raw', args)
            item['raw'] = compact_summary(raw_summary)
            if args.evaluate_constraints:
                item['constraint_raw'] = evaluate_constraint(raw_records, args)

            filtered_records = None
            if args.evaluate_filtered:
                segment_key = args.segment_key.strip() or None
                filtered_records = resample_command_records(raw_records, factor=args.resample_factor, segment_key=segment_key)
                filtered_records = moving_average_command_records(filtered_records, window=args.smooth_window, segment_key=segment_key)
                filtered_summary = summarize_source(filtered_records, 'filtered', args)
                item['filtered'] = compact_summary(filtered_summary)
                if args.evaluate_constraints:
                    item['constraint_filtered'] = evaluate_constraint(filtered_records, args)
            item['score'] = score_item(item, args.evaluate_filtered)
        except Exception as e:
            item['candidate_completed'] = False
            item['generation_error'] = {'type': e.__class__.__name__, 'message': str(e)}
            item['score'] = 1e99
            raw_records = None
            filtered_records = None
        results.append(item)
        if best is None or item['score'] < best['score']:
            best = item
            best_raw_records = raw_records
            best_filtered_records = filtered_records

    results_sorted = sorted(results, key=lambda x: x.get('score', 1e99))
    out = {
        'version_note': 'v3.0.42A: RF-2 local countermeasure sweep. This is a candidate-selection tool; it does not change the default baseline.',
        'roadmap_position': {
            'phase': 'Phase 3/4',
            'name': 'Select and test minimal RF-2 local countermeasure candidates',
            'previous': 'v3.0.41 localized the worst second-joint violation to RF-2 terminal landing.',
            'next': 'Inspect top cases, then run full constraint/Gazebo checks before adopting any candidate.',
        },
        'baseline_note': 'The baseline case is goal2_dist_front=0.4, goal2_x_scale=1.0, goal2_pitch_scale=1.0, goal2_landing_z=0.0.',
        'case_count': len(results_sorted),
        'sweep_parameters': lists,
        'global_parameters': {
            'surface_sequence': args.surface_sequence,
            'move_dist': args.move_dist,
            'support_dist': args.support_dist,
            'legacy_body_z': args.legacy_body_z,
            'max_step': args.max_step,
            'rf1_current_angle_anchor': args.rf1_current_angle_anchor,
            'resample_factor': args.resample_factor,
            'smooth_window': args.smooth_window,
            'segment_key': args.segment_key,
            'evaluate_filtered': args.evaluate_filtered,
            'evaluate_constraints': args.evaluate_constraints,
        },
        'best_case': best,
        'top_cases': results_sorted[:max(1, args.top_n)],
        'all_cases': results_sorted,
        'interpretation_notes': [
            'Do not adopt a case only because max_second_joint_deg improves.',
            'After this sweep, run full floor/inter-leg/joint-housing evaluations and Gazebo replay.',
            'If all RF-2 local cases remain above 95 deg, move to IK branch/cost or broader geometry parameters instead of over-tuning RF-2.',
        ],
    }
    ensure_dir(args.output)
    with open(args.output, 'w') as f:
        json.dump(out, f, indent=2, sort_keys=True)
    if args.best_raw_command_output and best_raw_records is not None:
        ensure_dir(args.best_raw_command_output)
        write_jsonl(best_raw_records, args.best_raw_command_output)
    if args.best_filtered_command_output and best_filtered_records is not None:
        ensure_dir(args.best_filtered_command_output)
        write_command_records(best_filtered_records, args.best_filtered_command_output)
    print(json.dumps({
        'case_count': len(results_sorted),
        'best_case': best,
        'output': args.output,
        'best_raw_command_output': args.best_raw_command_output,
        'best_filtered_command_output': args.best_filtered_command_output,
    }, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
