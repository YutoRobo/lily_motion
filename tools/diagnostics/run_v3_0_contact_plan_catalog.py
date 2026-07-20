#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Evaluate contact-plan variants with a fixed small parameter set.

v3.0.13 uses this as a contact-set design tool.  It is intentionally not a
continuous-parameter optimizer; it answers the first structural question:
which support set fails least badly over a full roll evaluation?
"""
from __future__ import print_function
import argparse
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.v3_roll_concept_generator import CONTACT_PLAN_VARIANTS, build_forward_roll_concept
from lily_motion_v3.v3_roll_candidate_generator import V3RollCandidateGenerator, V3RollGenerationConfig
from lily_motion_v3.whole_roll_evaluator import WholeRollEvaluator, WholeRollEvaluationConfig
from run_v3_0_goal_oriented_sweep import summarize_whole


def parse_strings(text):
    return [x.strip() for x in str(text).split(',') if x.strip()]


def phase_support_summary(variant):
    phases = build_forward_roll_concept(1, variant)
    rows = []
    for p in phases:
        cs = p.contact_state
        rows.append({
            "phase_name": p.name,
            "support_legs": list(cs.support_legs),
            "candidate_support_legs": list(cs.candidate_support_legs),
            "lift_legs": list(cs.lift_legs),
            "clearance_legs": list(cs.clearance_legs),
            "transfer_legs": list(cs.transfer_legs),
        })
    return rows


def evaluate_variant(variant, args):
    cfg = V3RollGenerationConfig(
        steps_per_phase=args.steps_per_phase,
        lift_height=args.lift_height,
        clearance_height=args.clearance_height,
        candidate_support_shift_x=args.candidate_support_shift_x,
        candidate_support_drop_z=args.candidate_support_drop_z,
        body_roll_pitch_rad=math.radians(args.body_roll_pitch_deg),
        enable_body_roll_pose_search=True,
        body_roll_search_x_offsets=[float(x) for x in args.body_roll_search_x_offsets.split(',') if x.strip()],
        body_roll_search_z_offsets=[float(x) for x in args.body_roll_search_z_offsets.split(',') if x.strip()],
        ground_z=args.ground_z,
        min_inter_leg_clearance_m=args.min_inter_leg_clearance,
        enable_contact_lock_generation=True,
        contact_plan_variant=variant,
    )
    gen = V3RollCandidateGenerator(config=cfg)
    cand = gen.generate_forward_one_roll(surface_id=1)
    whole = WholeRollEvaluator(gen.robot_model, WholeRollEvaluationConfig(
        filter_window=args.filter_window,
        ground_z=args.ground_z,
        min_inter_leg_clearance_m=args.min_inter_leg_clearance,
        contact_drift_soft_limit_m=args.contact_drift_soft_limit,
        contact_drift_hard_limit_m=args.contact_drift_hard_limit,
        second_joint_abs_max_deg=gen.robot_model.leg_config.second_joint_abs_max_deg,
        contact_preserving_filter=args.contact_preserving_filter,
    )).evaluate(cand)
    case = {
        "contact_plan_variant": variant,
        "steps_per_phase": args.steps_per_phase,
        "lift_height": args.lift_height,
        "clearance_height": args.clearance_height,
        "candidate_support_shift_x": args.candidate_support_shift_x,
        "candidate_support_drop_z": args.candidate_support_drop_z,
        "body_roll_pitch_deg": args.body_roll_pitch_deg,
        "filter_window": args.filter_window,
    }
    s = summarize_whole(case, whole)
    s["phase_support_summary"] = phase_support_summary(variant)
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--contact-plan-variants", default=','.join(CONTACT_PLAN_VARIANTS))
    ap.add_argument("--steps-per-phase", type=int, default=6)
    ap.add_argument("--lift-height", type=float, default=0.08)
    ap.add_argument("--clearance-height", type=float, default=0.06)
    ap.add_argument("--candidate-support-shift-x", type=float, default=0.04)
    ap.add_argument("--candidate-support-drop-z", type=float, default=-0.02)
    ap.add_argument("--body-roll-pitch-deg", type=float, default=90.0)
    ap.add_argument("--filter-window", type=int, default=3)
    ap.add_argument("--body-roll-search-x-offsets", default="-0.20,-0.10,0.0,0.10,0.20")
    ap.add_argument("--body-roll-search-z-offsets", default="-0.10,0.0,0.10,0.20,0.30,0.40")
    ap.add_argument("--contact-preserving-filter", action="store_true")
    ap.add_argument("--contact-drift-soft-limit", type=float, default=0.05)
    ap.add_argument("--contact-drift-hard-limit", type=float, default=0.15)
    ap.add_argument("--min-inter-leg-clearance", type=float, default=0.05)
    ap.add_argument("--ground-z", type=float, default=0.0)
    ap.add_argument("--output", default="testdata/v3_0_13_contact_plan_catalog.json")
    args = ap.parse_args()

    variants = parse_strings(args.contact_plan_variants)
    results = [evaluate_variant(v, args) for v in variants]
    results.sort(key=lambda r: r["score"])
    out = {
        "version": "v3.0.13",
        "purpose": "Contact-plan catalog sweep.  Keep continuous parameters mostly fixed and compare support-set assumptions.",
        "case_count": len(results),
        "best_case": results[0] if results else None,
        "results": results,
    }
    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.output, 'w') as f:
        json.dump(out, f, indent=2, sort_keys=True)
    print(json.dumps({
        "version": out["version"],
        "case_count": out["case_count"],
        "best_case": out["best_case"],
        "output": args.output,
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
