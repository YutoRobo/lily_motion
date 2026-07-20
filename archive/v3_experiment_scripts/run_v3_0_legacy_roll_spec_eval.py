#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse, json, os, sys
ROOT=os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path: sys.path.insert(0, ROOT)
from lily_motion_v3.legacy_roll_spec_generator import LegacyRollSpecCandidateGenerator, LegacyRollSpecGenerationConfig
from lily_motion_v3.whole_roll_evaluator import WholeRollEvaluator, WholeRollEvaluationConfig


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--summary-only', action='store_true')
    ap.add_argument('--surface-id', type=int, default=1)
    ap.add_argument('--move-dist', type=float, default=0.4)
    ap.add_argument('--support-dist', type=float, default=0.7)
    ap.add_argument('--max-step', type=int, default=30)
    ap.add_argument('--legacy-body-z', type=float, default=0.35)
    ap.add_argument('--filter-window', type=int, default=3)
    ap.add_argument('--ground-z', type=float, default=0.0)
    ap.add_argument('--contact-drift-soft-limit', type=float, default=0.05)
    ap.add_argument('--contact-drift-hard-limit', type=float, default=0.15)
    args=ap.parse_args()
    cfg=LegacyRollSpecGenerationConfig(move_dist=args.move_dist, support_dist=args.support_dist, max_step=args.max_step, surface_id=args.surface_id, z=args.legacy_body_z, ground_z=args.ground_z)
    gen=LegacyRollSpecCandidateGenerator(config=cfg)
    cand=gen.generate_forward_one_roll(surface_id=args.surface_id)
    whole=WholeRollEvaluator(gen.robot_model, WholeRollEvaluationConfig(filter_window=args.filter_window, ground_z=args.ground_z, contact_drift_soft_limit_m=args.contact_drift_soft_limit, contact_drift_hard_limit_m=args.contact_drift_hard_limit)).evaluate(cand)
    fr=whole['filtered_command']['geometry']
    summary={
        'profile':'legacy_roll_spec',
        'candidate_completed': cand.report.task_success.get('completed'),
        'surface_start': args.surface_id,
        'surface_after': cand.report.task_success.get('surface_after'),
        'frame_count': len(cand.frames),
        'phase_names': [p.name for p in cand.phases],
        'generator_ik_failure_count': cand.report.ik_reachability.get('ik_failure_count'),
        'raw_ground_penetration_count': cand.report.ground_clearance.get('penetration_count'),
        'filtered_penetration_count': fr['ground_clearance']['penetration_count'],
        'filtered_min_clearance_m': fr['ground_clearance']['min_clearance_m'],
        'filtered_max_second_joint_deg': fr['joint_limit']['max_abs_second_joint_deg'],
        'filtered_max_joint_delta_deg': whole['filtered_command']['max_joint_delta_deg'],
        'whole_roll_success_by_filtered_geometry': whole['whole_roll_success_by_filtered_geometry'],
        'notes': cand.report.notes,
    }
    print(json.dumps(summary if args.summary_only else {'summary':summary,'candidate':cand.to_dict(),'whole':whole}, indent=2, sort_keys=True))
if __name__=='__main__':
    main()
