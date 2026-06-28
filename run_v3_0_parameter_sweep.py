#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Small whole-roll parameter sweep for v3.

This is intentionally simple: it sweeps a few gait-generation parameters and
judges the full generated trajectory after moving-average filtering.
"""
from __future__ import print_function
import argparse
import itertools
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.v3_roll_candidate_generator import V3RollCandidateGenerator, V3RollGenerationConfig
from lily_motion_v3.synchronized_roll_generator import SynchronizedRollCandidateGenerator, SynchronizedRollGenerationConfig
from lily_motion_v3.whole_roll_evaluator import WholeRollEvaluator, WholeRollEvaluationConfig


def parse_floats(text):
    return [float(x.strip()) for x in str(text).split(',') if x.strip()]


def parse_ints(text):
    return [int(x.strip()) for x in str(text).split(',') if x.strip()]


def summarize_case(case, whole):
    fr = whole["filtered_command"]["geometry"]
    raw = whole["raw_command"]
    filt = whole["filtered_command"]
    contact = whole["contact_lock"]
    return {
        "case": dict(case),
        "candidate_completed": whole["candidate_completed"],
        "whole_roll_success_by_filtered_geometry": whole["whole_roll_success_by_filtered_geometry"],
        "generator_ik_failure_count": fr["ik_failure_count_from_generator"],
        "filtered_penetration_count": fr["ground_clearance"]["penetration_count"],
        "filtered_min_clearance_m": fr["ground_clearance"]["min_clearance_m"],
        "filtered_near_count": fr["inter_leg_clearance"]["near_count"],
        "filtered_max_second_joint_deg": fr["joint_limit"]["max_abs_second_joint_deg"],
        "raw_max_joint_delta_deg": raw["max_joint_delta_deg"],
        "filtered_max_joint_delta_deg": filt["max_joint_delta_deg"],
        "contact_drift_violation_count": contact["contact_drift_violation_count"],
        "contact_drift_soft_violation_count": contact["contact_drift_soft_violation_count"],
        "contact_drift_hard_violation_count": contact["contact_drift_hard_violation_count"],
        "contact_drift_soft_excess_sum_m": contact["contact_drift_soft_excess_sum_m"],
        "contact_drift_hard_excess_sum_m": contact["contact_drift_hard_excess_sum_m"],
        "max_contact_drift_m": contact["max_contact_drift_m"],
        "filter_projection_failure_count": whole["filter"]["contact_preserving_projection"].get("projection_failure_count", 0),
        "filter_projected_count": whole["filter"]["contact_preserving_projection"].get("projected_count", 0),
    }


def case_score(s):
    # Lower is better.  Feasibility dominates; geometry/contact then smoothness.
    return (
        s["generator_ik_failure_count"] * 100000.0 +
        s["filtered_penetration_count"] * 20000.0 +
        s["filtered_near_count"] * 5000.0 +
        s.get("contact_drift_hard_violation_count", 0) * 5000.0 +
        s.get("contact_drift_hard_excess_sum_m", 0.0) * 50000.0 +
        s.get("contact_drift_soft_excess_sum_m", 0.0) * 5000.0 +
        max(0.0, (s["filtered_max_second_joint_deg"] or 0.0) - 95.0) * 1000.0 +
        (s["filtered_max_joint_delta_deg"] or 0.0)
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trajectory-modes", default="phase", help="Comma-separated: phase,synchronized")
    ap.add_argument("--synchronized-steps", default="72", help="Comma-separated synchronized frame counts")
    ap.add_argument("--roll-start-s", type=float, default=0.35)
    ap.add_argument("--roll-end-s", type=float, default=0.85)
    ap.add_argument("--contact-plan-variants", default="default,next_only_roll")
    ap.add_argument("--steps-per-phase", default="6,8")
    ap.add_argument("--lift-heights", default="0.06,0.08,0.10")
    ap.add_argument("--clearance-heights", default="0.05,0.08")
    ap.add_argument("--candidate-support-shift-xs", default="0.02,0.04,0.06")
    ap.add_argument("--candidate-support-drop-zs", default="-0.04,-0.02,0.0")
    ap.add_argument("--body-roll-pitch-deg", type=float, default=90.0)
    ap.add_argument("--body-roll-search-x-offsets", default="-0.20,-0.10,0.0,0.10,0.20")
    ap.add_argument("--body-roll-search-z-offsets", default="-0.10,0.0,0.10,0.20,0.30,0.40")
    ap.add_argument("--filter-window", type=int, default=5, help="Single filter window; ignored when --filter-windows is non-empty")
    ap.add_argument("--filter-windows", default="", help="Comma-separated moving-average windows, e.g. 3,5,7,9")
    ap.add_argument("--contact-preserving-filter", action="store_true")
    ap.add_argument("--contact-drift-warn", type=float, default=None, help="Deprecated alias for --contact-drift-soft-limit")
    ap.add_argument("--contact-drift-soft-limit", type=float, default=0.05)
    ap.add_argument("--contact-drift-hard-limit", type=float, default=0.15)
    ap.add_argument("--min-inter-leg-clearance", type=float, default=0.05)
    ap.add_argument("--ground-z", type=float, default=0.0)
    ap.add_argument("--no-contact-lock-generation", action="store_true")
    ap.add_argument("--output", default="testdata/v3_0_9_parameter_sweep_summary.json")
    args = ap.parse_args()

    trajectory_modes = [x.strip() for x in str(args.trajectory_modes).split(',') if x.strip()]
    synchronized_step_values = parse_ints(args.synchronized_steps)
    variant_values = [x.strip() for x in str(args.contact_plan_variants).split(',') if x.strip()]
    step_values = parse_ints(args.steps_per_phase)
    lift_values = parse_floats(args.lift_heights)
    clearance_values = parse_floats(args.clearance_heights)
    shift_values = parse_floats(args.candidate_support_shift_xs)
    drop_values = parse_floats(args.candidate_support_drop_zs)
    filter_window_values = parse_ints(args.filter_windows) if str(args.filter_windows).strip() else [int(args.filter_window)]
    x_offsets = parse_floats(args.body_roll_search_x_offsets)
    z_offsets = parse_floats(args.body_roll_search_z_offsets)

    results = []
    for mode, sync_steps, variant, steps, lift_h, clearance_h, shift_x, drop_z, filt_win in itertools.product(trajectory_modes, synchronized_step_values, variant_values, step_values, lift_values, clearance_values, shift_values, drop_values, filter_window_values):
        case = {
            "trajectory_mode": mode,
            "synchronized_steps": sync_steps if mode == "synchronized" else None,
            "contact_plan_variant": variant,
            "steps_per_phase": steps,
            "lift_height": lift_h,
            "clearance_height": clearance_h,
            "candidate_support_shift_x": shift_x,
            "candidate_support_drop_z": drop_z,
            "filter_window": filt_win,
        }
        cfg_cls = SynchronizedRollGenerationConfig if mode == "synchronized" else V3RollGenerationConfig
        cfg_kwargs = {}
        if mode == "synchronized":
            cfg_kwargs = {"synchronized_steps": sync_steps, "roll_start_s": args.roll_start_s, "roll_end_s": args.roll_end_s}
        cfg = cfg_cls(
            steps_per_phase=steps,
            lift_height=lift_h,
            clearance_height=clearance_h,
            candidate_support_shift_x=shift_x,
            candidate_support_drop_z=drop_z,
            body_roll_pitch_rad=math.radians(args.body_roll_pitch_deg),
            enable_body_roll_pose_search=True,
            body_roll_search_x_offsets=x_offsets,
            body_roll_search_z_offsets=z_offsets,
            ground_z=args.ground_z,
            min_inter_leg_clearance_m=args.min_inter_leg_clearance,
            enable_contact_lock_generation=not args.no_contact_lock_generation,
            contact_plan_variant=variant,
            **cfg_kwargs
        )
        gen_cls = SynchronizedRollCandidateGenerator if mode == "synchronized" else V3RollCandidateGenerator
        gen = gen_cls(config=cfg)
        cand = gen.generate_forward_one_roll(surface_id=1)
        whole = WholeRollEvaluator(gen.robot_model, WholeRollEvaluationConfig(
            filter_window=filt_win,
            ground_z=args.ground_z,
            min_inter_leg_clearance_m=args.min_inter_leg_clearance,
            contact_drift_soft_limit_m=(args.contact_drift_warn if args.contact_drift_warn is not None else args.contact_drift_soft_limit),
            contact_drift_hard_limit_m=args.contact_drift_hard_limit,
            second_joint_abs_max_deg=gen.robot_model.leg_config.second_joint_abs_max_deg,
            contact_preserving_filter=args.contact_preserving_filter,
        )).evaluate(cand)
        s = summarize_case(case, whole)
        s["score"] = case_score(s)
        results.append(s)

    results.sort(key=lambda r: r["score"])
    out = {
        "case_count": len(results),
        "best_case": results[0] if results else None,
        "top_cases": results[:20],
        "all_cases": results,
        "note": "v3.0.14 whole-roll parameter/contact-plan/filter-window sweep with optional synchronized-progress trajectory mode: contact drift is treated with soft/hard limits, not as a zero-drift constraint. This ranks candidate parameters for the current phase/contact design.",
    }
    d = os.path.dirname(args.output)
    if d and not os.path.isdir(d):
        os.makedirs(d)
    with open(args.output, "w") as f:
        json.dump(out, f, indent=2, sort_keys=True)
        f.write("\n")
    print(json.dumps({"output": args.output, "case_count": len(results), "best_case": out["best_case"]}, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
