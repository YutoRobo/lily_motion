#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Goal-oriented staged sweep for v3 whole-roll development.

This script is intentionally separate from the small parameter sweep.  Its job is
not just to find a local best score, but to make the current roadmap visible in
the output: v3-native search now, legacy-style compatibility later.
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
from lily_motion_v3.whole_roll_evaluator import WholeRollEvaluator, WholeRollEvaluationConfig


def parse_floats(text):
    return [float(x.strip()) for x in str(text).split(',') if x.strip()]


def parse_ints(text):
    return [int(x.strip()) for x in str(text).split(',') if x.strip()]


def parse_strings(text):
    return [x.strip() for x in str(text).split(',') if x.strip()]


def summarize_whole(case, whole):
    fr = whole["filtered_command"]["geometry"]
    raw = whole["raw_command"]
    filt = whole["filtered_command"]
    contact = whole["contact_lock"]
    raw_contact = whole.get("raw_contact_lock", {})
    filtered_contact = whole.get("filtered_contact_lock", {})
    summary = {
        "case": dict(case),
        "candidate_completed": whole["candidate_completed"],
        "whole_roll_success_by_filtered_geometry": whole["whole_roll_success_by_filtered_geometry"],
        "frame_count": whole["frame_count"],
        "generator_ik_failure_count": fr["ik_failure_count_from_generator"],
        "filtered_penetration_count": fr["ground_clearance"]["penetration_count"],
        "filtered_min_clearance_m": fr["ground_clearance"]["min_clearance_m"],
        "filtered_near_count": fr["inter_leg_clearance"]["near_count"],
        "filtered_min_inter_leg_distance_m": fr["inter_leg_clearance"]["min_distance_m"],
        "filtered_max_second_joint_deg": fr["joint_limit"]["max_abs_second_joint_deg"],
        "raw_max_joint_delta_deg": raw["max_joint_delta_deg"],
        "filtered_max_joint_delta_deg": filt["max_joint_delta_deg"],
        "raw_max_contact_drift_m": raw_contact.get("max_contact_drift_m"),
        "filtered_max_contact_drift_m": filtered_contact.get("max_contact_drift_m"),
        "contact_drift_violation_count": contact.get("contact_drift_violation_count", 0),
        "contact_drift_soft_violation_count": contact.get("contact_drift_soft_violation_count", 0),
        "contact_drift_hard_violation_count": contact.get("contact_drift_hard_violation_count", 0),
        "contact_drift_soft_excess_sum_m": contact.get("contact_drift_soft_excess_sum_m", 0.0),
        "contact_drift_hard_excess_sum_m": contact.get("contact_drift_hard_excess_sum_m", 0.0),
        "filter_projection_enabled": whole["filter"].get("contact_preserving_projection", {}).get("enabled", False),
        "filter_projected_count": whole["filter"].get("contact_preserving_projection", {}).get("projected_count", 0),
        "filter_projection_failure_count": whole["filter"].get("contact_preserving_projection", {}).get("projection_failure_count", 0),
    }
    summary["score"] = score_case(summary)
    summary["failure_signature"] = failure_signature(summary)
    return summary


def score_case(s):
    # Lower is better.  IK failure dominates, then geometry, then contact drift
    # and smoothness.  Raw jumps are intentionally not penalized strongly because
    # singular-pose flip-like raw commands are allowed if the filtered trajectory
    # is acceptable.
    second = s.get("filtered_max_second_joint_deg") or 0.0
    drift_hard = s.get("contact_drift_hard_violation_count", 0)
    return (
        s.get("generator_ik_failure_count", 0) * 100000.0 +
        s.get("filtered_penetration_count", 0) * 25000.0 +
        s.get("filtered_near_count", 0) * 10000.0 +
        drift_hard * 5000.0 +
        s.get("contact_drift_hard_excess_sum_m", 0.0) * 50000.0 +
        s.get("contact_drift_soft_excess_sum_m", 0.0) * 2500.0 +
        max(0.0, second - 95.0) * 2000.0 +
        (s.get("filtered_max_joint_delta_deg") or 0.0) * 5.0
    )


def failure_signature(s):
    out = []
    if s.get("generator_ik_failure_count", 0) > 0:
        out.append("ik_failure")
    if s.get("filtered_penetration_count", 0) > 0:
        out.append("ground_penetration")
    if s.get("filtered_near_count", 0) > 0:
        out.append("inter_leg_near")
    if (s.get("filtered_max_second_joint_deg") or 0.0) > 95.0:
        out.append("second_joint_limit")
    if s.get("contact_drift_hard_violation_count", 0) > 0:
        out.append("hard_contact_drift")
    if (s.get("filtered_max_joint_delta_deg") or 0.0) > 120.0:
        out.append("filtered_discontinuity")
    return out or ["no_major_failure"]


def default_values_for_mode(mode):
    if mode == "quick":
        return {
            "contact_plan_variants": "front_pair_roll,rear_pair_roll,diagonal_front_roll,diagonal_rear_roll,four_corner_roll,x_cross_roll",
            "steps_per_phase": "6",
            "lift_heights": "0.08",
            "clearance_heights": "0.06",
            "candidate_support_shift_xs": "0.04",
            "candidate_support_drop_zs": "-0.02",
            "body_roll_pitch_degs": "60,90",
            "filter_windows": "3,5",
        }
    if mode == "broad":
        return {
            "contact_plan_variants": "default,next_only_roll,six_support_roll,front_pair_roll,rear_pair_roll,upper_front_pair_roll,upper_rear_pair_roll,lower_front_pair_roll,lower_rear_pair_roll,diagonal_front_roll,diagonal_rear_roll,four_corner_roll,x_cross_roll,upper_quad_roll,lower_quad_roll",
            "steps_per_phase": "5,6,8",
            "lift_heights": "0.06,0.08,0.10,0.12",
            "clearance_heights": "0.04,0.06,0.08,0.10",
            "candidate_support_shift_xs": "0.00,0.04,0.08,0.12",
            "candidate_support_drop_zs": "-0.06,-0.04,-0.02,0.0,0.02",
            "body_roll_pitch_degs": "45,60,75,90",
            "filter_windows": "3,5,7",
        }
    return {}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["quick", "broad", "custom"], default="quick")
    ap.add_argument("--contact-plan-variants", default=None)
    ap.add_argument("--steps-per-phase", default=None)
    ap.add_argument("--lift-heights", default=None)
    ap.add_argument("--clearance-heights", default=None)
    ap.add_argument("--candidate-support-shift-xs", default=None)
    ap.add_argument("--candidate-support-drop-zs", default=None)
    ap.add_argument("--body-roll-pitch-degs", default=None)
    ap.add_argument("--body-roll-search-x-offsets", default="-0.20,-0.10,0.0,0.10,0.20")
    ap.add_argument("--body-roll-search-z-offsets", default="-0.10,0.0,0.10,0.20,0.30,0.40")
    ap.add_argument("--filter-windows", default=None)
    ap.add_argument("--contact-preserving-filter", action="store_true")
    ap.add_argument("--contact-drift-soft-limit", type=float, default=0.05)
    ap.add_argument("--contact-drift-hard-limit", type=float, default=0.15)
    ap.add_argument("--min-inter-leg-clearance", type=float, default=0.05)
    ap.add_argument("--ground-z", type=float, default=0.0)
    ap.add_argument("--max-cases", type=int, default=0, help="Optional safety cap; 0 means no cap")
    ap.add_argument("--output", default="testdata/v3_0_12_goal_oriented_sweep.json")
    args = ap.parse_args()

    defaults = default_values_for_mode(args.mode)
    values = {
        "contact_plan_variants": args.contact_plan_variants or defaults.get("contact_plan_variants", "front_pair_roll"),
        "steps_per_phase": args.steps_per_phase or defaults.get("steps_per_phase", "6"),
        "lift_heights": args.lift_heights or defaults.get("lift_heights", "0.08"),
        "clearance_heights": args.clearance_heights or defaults.get("clearance_heights", "0.06"),
        "candidate_support_shift_xs": args.candidate_support_shift_xs or defaults.get("candidate_support_shift_xs", "0.04"),
        "candidate_support_drop_zs": args.candidate_support_drop_zs or defaults.get("candidate_support_drop_zs", "-0.02"),
        "body_roll_pitch_degs": args.body_roll_pitch_degs or defaults.get("body_roll_pitch_degs", "90"),
        "filter_windows": args.filter_windows or defaults.get("filter_windows", "5"),
    }

    variant_values = parse_strings(values["contact_plan_variants"])
    step_values = parse_ints(values["steps_per_phase"])
    lift_values = parse_floats(values["lift_heights"])
    clearance_values = parse_floats(values["clearance_heights"])
    shift_values = parse_floats(values["candidate_support_shift_xs"])
    drop_values = parse_floats(values["candidate_support_drop_zs"])
    pitch_values = parse_floats(values["body_roll_pitch_degs"])
    filter_values = parse_ints(values["filter_windows"])
    x_offsets = parse_floats(args.body_roll_search_x_offsets)
    z_offsets = parse_floats(args.body_roll_search_z_offsets)

    results = []
    case_index = 0
    for variant, steps, lift_h, clearance_h, shift_x, drop_z, pitch_deg, filt_win in itertools.product(
            variant_values, step_values, lift_values, clearance_values, shift_values, drop_values, pitch_values, filter_values):
        if args.max_cases and case_index >= args.max_cases:
            break
        case_index += 1
        case = {
            "contact_plan_variant": variant,
            "steps_per_phase": steps,
            "lift_height": lift_h,
            "clearance_height": clearance_h,
            "candidate_support_shift_x": shift_x,
            "candidate_support_drop_z": drop_z,
            "body_roll_pitch_deg": pitch_deg,
            "filter_window": filt_win,
        }
        cfg = V3RollGenerationConfig(
            steps_per_phase=steps,
            lift_height=lift_h,
            clearance_height=clearance_h,
            candidate_support_shift_x=shift_x,
            candidate_support_drop_z=drop_z,
            body_roll_pitch_rad=math.radians(pitch_deg),
            enable_body_roll_pose_search=True,
            body_roll_search_x_offsets=x_offsets,
            body_roll_search_z_offsets=z_offsets,
            ground_z=args.ground_z,
            min_inter_leg_clearance_m=args.min_inter_leg_clearance,
            enable_contact_lock_generation=True,
            contact_plan_variant=variant,
        )
        gen = V3RollCandidateGenerator(config=cfg)
        cand = gen.generate_forward_one_roll(surface_id=1)
        whole = WholeRollEvaluator(gen.robot_model, WholeRollEvaluationConfig(
            filter_window=filt_win,
            ground_z=args.ground_z,
            min_inter_leg_clearance_m=args.min_inter_leg_clearance,
            contact_drift_soft_limit_m=args.contact_drift_soft_limit,
            contact_drift_hard_limit_m=args.contact_drift_hard_limit,
            second_joint_abs_max_deg=gen.robot_model.leg_config.second_joint_abs_max_deg,
            contact_preserving_filter=args.contact_preserving_filter,
        )).evaluate(cand)
        results.append(summarize_whole(case, whole))

    results.sort(key=lambda r: r["score"])
    by_signature = {}
    for r in results:
        key = "+".join(r["failure_signature"])
        by_signature[key] = by_signature.get(key, 0) + 1

    best_by_pitch = {}
    for r in results:
        p = str(r["case"]["body_roll_pitch_deg"])
        if p not in best_by_pitch or r["score"] < best_by_pitch[p]["score"]:
            best_by_pitch[p] = r

    out = {
        "version": "v3.0.13",
        "mode": args.mode,
        "roadmap": {
            "stage_now": "v3-native whole-roll search with soft contact drift and filtered geometry evaluation",
            "stage_next": "contact plan redesign if IK failures remain dominant",
            "stage_later": "legacy-style RF parameter compatibility layer for direct comparison with the traditional program",
            "gazebo_role": "failure visualization until a plausible filtered trajectory exists",
        },
        "input_space": values,
        "case_count": len(results),
        "best_case": results[0] if results else None,
        "top_cases": results[:20],
        "best_by_pitch_deg": best_by_pitch,
        "failure_signature_counts": by_signature,
        "all_cases": results,
    }
    d = os.path.dirname(args.output)
    if d and not os.path.isdir(d):
        os.makedirs(d)
    with open(args.output, "w") as f:
        json.dump(out, f, indent=2, sort_keys=True)
        f.write("\n")
    print(json.dumps({
        "output": args.output,
        "case_count": len(results),
        "best_case": out["best_case"],
        "failure_signature_counts": by_signature,
        "roadmap_next": out["roadmap"]["stage_next"],
        "roadmap_later": out["roadmap"]["stage_later"],
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
