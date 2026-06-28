#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse, json, os, sys, warnings
warnings.filterwarnings('ignore', category=DeprecationWarning)
ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.command_resampler import write_command_records
from lily_motion_v3.constrained_roll_parameterizer import (
    parse_float_list, parse_profile_list, linspace, PITCH_PROFILES,
    find_periodic_move_dist, generate_repeated_records, evaluate_constraints,
    periodicity_for_records, score_case, is_valid_constrained_case, score_valid_case)


def parse_int_list(s):
    return [int(x) for x in str(s).split(',') if str(x).strip()]


def _ensure_dir(path):
    d = os.path.dirname(path)
    if d and not os.path.isdir(d):
        os.makedirs(d)


def main():
    ap = argparse.ArgumentParser(description='v3.0.30 valid-case-only constrained repeated roll sweep.')
    ap.add_argument('--surface-sequence', default='1,5,6,2,1')
    ap.add_argument('--surface-id', type=int, default=1)
    ap.add_argument('--support-dists', default='0.72,0.74,0.76')
    ap.add_argument('--legacy-body-zs', default='0.38,0.40,0.42')
    ap.add_argument('--pitch-profiles', default='legacy,balanced,late_roll')
    ap.add_argument('--move-dist-range', default='0.25,0.45')
    ap.add_argument('--move-dist-samples', type=int, default=41)
    ap.add_argument('--max-step', type=int, default=30)
    ap.add_argument('--periodicity-max-error', type=float, default=0.08)
    ap.add_argument('--periodicity-mean-error', type=float, default=0.04)
    ap.add_argument('--second-joint-abs-max-deg', type=float, default=95.0)
    ap.add_argument('--inter-leg-limit', type=float, default=0.04)
    ap.add_argument('--ground-tolerance', type=float, default=1e-4)
    ap.add_argument('--goal2-dist-front', type=float, default=0.3)
    ap.add_argument('--goal2-x-scale', type=float, default=1.0)
    ap.add_argument('--goal2-landing-z', type=float, default=0.0)
    ap.add_argument('--goal3-lift-z', type=float, default=0.05)
    ap.add_argument('--goal3-target-x', type=float, default=0.2)
    ap.add_argument('--goal4-target-x', type=float, default=0.05)
    ap.add_argument('--output', default='testdata/v3_0_30_constrained_valid_sweep.json')
    ap.add_argument('--best-command-output', default='testdata/v3_0_30_constrained_valid_best_repeated_commands.jsonl')
    ap.add_argument('--best-report-output', default='testdata/v3_0_30_constrained_valid_best_report.json')
    ap.add_argument('--top-n', type=int, default=10)
    args = ap.parse_args()

    # vendored legacy runtime prints link setup messages; keep JSON clean.
    _json_stdout = sys.stdout
    sys.stdout = sys.stderr

    support_dists = parse_float_list(args.support_dists)
    body_zs = parse_float_list(args.legacy_body_zs)
    profiles = parse_profile_list(args.pitch_profiles)
    rng = parse_float_list(args.move_dist_range)
    if len(rng) != 2:
        raise ValueError('--move-dist-range must be min,max')
    move_candidates = linspace(rng[0], rng[1], args.move_dist_samples)
    seq = parse_int_list(args.surface_sequence)

    fixed_kwargs = dict(
        goal2_dist_front=args.goal2_dist_front,
        goal2_x_scale=args.goal2_x_scale,
        goal2_landing_z=args.goal2_landing_z,
        goal3_lift_z=args.goal3_lift_z,
        goal3_target_x=args.goal3_target_x,
        goal4_target_x=args.goal4_target_x,
    )
    all_cases = []
    periodicity_admissible = []
    valid_cases = []
    rejected = 0
    rejected_after_periodicity = 0
    best = None
    best_records = []

    for sd in support_dists:
        for bz in body_zs:
            for profile in profiles:
                perbest = find_periodic_move_dist(
                    args.surface_id, sd, bz, args.max_step, profile,
                    move_candidates, args.periodicity_max_error, args.periodicity_mean_error,
                    **fixed_kwargs)
                case_base = {
                    'support_dist': sd,
                    'legacy_body_z': bz,
                    'pitch_profile_name': profile,
                    'pitch_profile': PITCH_PROFILES[profile],
                    'move_dist': None if perbest is None else perbest['move_dist'],
                    'move_dist_source': 'periodicity_minimization',
                    'periodicity': None if perbest is None else perbest['periodicity'],
                    'admissible_by_periodicity': False if perbest is None else bool(perbest.get('admissible')),
                }
                if perbest is None or not perbest.get('admissible'):
                    rejected += 1
                    all_cases.append(case_base)
                    continue
                try:
                    repeated_records = generate_repeated_records(seq, args.surface_id, sd, bz, perbest['move_dist'], args.max_step, profile, **fixed_kwargs)
                    constraints = evaluate_constraints(repeated_records, bz, args.second_joint_abs_max_deg, args.inter_leg_limit, args.ground_tolerance)
                    repeated_periodicity = periodicity_for_records(repeated_records, bz)
                    repeated_report = {'candidate_completed': True, 'frame_count': len(repeated_records), 'periodicity': repeated_periodicity}
                except Exception as e:
                    repeated_records = list(perbest.get('records') or [])
                    constraints = {'error': str(e), 'max_second_joint_deg': 999.0, 'second_joint_violation_count': 999999, 'ground_penetration_count': 999999, 'inter_leg_near_count': 999999}
                    repeated_report = {'candidate_completed': False, 'error': {'type': e.__class__.__name__, 'message': str(e)}}
                case = dict(case_base)
                case['constraints'] = constraints
                case['repeated_roll'] = repeated_report
                case['score'] = score_case(case['periodicity'], constraints, repeated_report)
                periodicity_admissible.append(case)
                all_cases.append(case)
                if is_valid_constrained_case(case):
                    case['valid_case'] = True
                    case['valid_score'] = score_valid_case(case)
                    valid_cases.append(case)
                    if best is None or case['valid_score'] < best['valid_score']:
                        best = case
                        best_records = repeated_records
                else:
                    case['valid_case'] = False
                    rejected_after_periodicity += 1

    top_cases = sorted(valid_cases, key=lambda c: c.get('valid_score', 1e99))[:args.top_n]
    report = {
        'version_note': 'v3.0.30: valid-case-only constrained repeated sweep. Periodic candidates with constraint/repeated-roll errors are rejected, not penalized.',
        'profile': 'constrained_roll_sweep',
        'surface_sequence': seq,
        'case_count': len(all_cases),
        'periodicity_admissible_case_count': len(periodicity_admissible),
        'valid_case_count': len(valid_cases),
        'rejected_by_periodicity_count': rejected,
        'rejected_after_periodicity_count': rejected_after_periodicity,
        'periodicity_thresholds': {'max_error_m': args.periodicity_max_error, 'mean_error_m': args.periodicity_mean_error},
        'move_dist_candidates': {'range': rng, 'samples': args.move_dist_samples},
        'fixed_parameters': fixed_kwargs,
        'best_case': best,
        'top_valid_cases': top_cases,
        'top_cases': top_cases,
        'all_cases_summary': [
            {
                'support_dist': c.get('support_dist'), 'legacy_body_z': c.get('legacy_body_z'),
                'pitch_profile_name': c.get('pitch_profile_name'), 'move_dist': c.get('move_dist'),
                'admissible_by_periodicity': c.get('admissible_by_periodicity'),
                'periodicity_max_error_m': None if not c.get('periodicity') else c['periodicity'].get('max_error_m'),
                'periodicity_mean_error_m': None if not c.get('periodicity') else c['periodicity'].get('mean_error_m'),
                'score': c.get('score'), 'valid_case': c.get('valid_case', False), 'valid_score': c.get('valid_score')
            } for c in all_cases
        ],
        'best_command_output': args.best_command_output,
        'best_report_output': args.best_report_output,
    }

    if best and args.best_command_output:
        # v3.0.30: this is the repeated-roll command log, not the single 1/4-roll log.
        write_command_records(best_records, args.best_command_output)
    if args.best_report_output:
        _ensure_dir(args.best_report_output)
        with open(args.best_report_output, 'w') as f:
            json.dump(best, f, indent=2, sort_keys=True)
    if args.output:
        _ensure_dir(args.output)
        with open(args.output, 'w') as f:
            json.dump(report, f, indent=2, sort_keys=True)
    sys.stdout = _json_stdout
    print(json.dumps(report, indent=2, sort_keys=True))

if __name__ == '__main__':
    main()
