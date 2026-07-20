#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Run v3 whole-roll raw/filtered/contact-lock evaluation."""
from __future__ import print_function
import argparse
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.v3_roll_candidate_generator import V3RollCandidateGenerator, V3RollGenerationConfig
from lily_motion_v3.synchronized_roll_generator import SynchronizedRollCandidateGenerator, SynchronizedRollGenerationConfig
from lily_motion_v3.whole_roll_evaluator import WholeRollEvaluator, WholeRollEvaluationConfig


def _parse_float_list(text):
    out = []
    for part in str(text).split(','):
        part = part.strip()
        if part:
            out.append(float(part))
    return out


def build_generation_config(args):
    cls = SynchronizedRollGenerationConfig if args.trajectory_mode == "synchronized" else V3RollGenerationConfig
    extra = {}
    if args.trajectory_mode == "synchronized":
        extra = {"synchronized_steps": args.synchronized_steps, "roll_start_s": args.roll_start_s, "roll_end_s": args.roll_end_s}
    return cls(
        steps_per_phase=args.steps_per_phase,
        lift_height=args.lift_height,
        clearance_height=args.clearance_height,
        candidate_support_shift_x=args.candidate_support_shift_x,
        candidate_support_drop_z=args.candidate_support_drop_z,
        body_roll_pitch_rad=math.radians(args.body_roll_pitch_deg),
        body_roll_x_shift=args.body_roll_x_shift,
        body_roll_z_shift=args.body_roll_z_shift,
        enable_body_roll_pose_search=not args.disable_body_roll_pose_search,
        body_roll_search_x_offsets=_parse_float_list(args.body_roll_search_x_offsets),
        body_roll_search_z_offsets=_parse_float_list(args.body_roll_search_z_offsets),
        ground_z=args.ground_z,
        auto_align_initial_ground=not args.no_auto_align_initial_ground,
        min_inter_leg_clearance_m=args.min_inter_leg_clearance,
        min_target_point_clearance_m=args.min_target_point_clearance,
        enable_contact_lock_generation=not args.no_contact_lock_generation,
        contact_plan_variant=args.contact_plan_variant,
        **extra
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--surface-id", type=int, default=1)
    ap.add_argument("--trajectory-mode", choices=["phase", "synchronized"], default="phase")
    ap.add_argument("--synchronized-steps", type=int, default=72)
    ap.add_argument("--roll-start-s", type=float, default=0.35)
    ap.add_argument("--roll-end-s", type=float, default=0.85)
    ap.add_argument("--contact-plan-variant", default="default")
    ap.add_argument("--steps-per-phase", type=int, default=8)
    ap.add_argument("--lift-height", type=float, default=0.08)
    ap.add_argument("--clearance-height", type=float, default=0.06)
    ap.add_argument("--candidate-support-shift-x", type=float, default=0.04)
    ap.add_argument("--candidate-support-drop-z", type=float, default=-0.02)
    ap.add_argument("--body-roll-pitch-deg", type=float, default=90.0)
    ap.add_argument("--body-roll-x-shift", type=float, default=0.0)
    ap.add_argument("--body-roll-z-shift", type=float, default=0.0)
    ap.add_argument("--disable-body-roll-pose-search", action="store_true")
    ap.add_argument("--body-roll-search-x-offsets", default="-0.20,-0.10,0.0,0.10,0.20")
    ap.add_argument("--body-roll-search-z-offsets", default="-0.10,0.0,0.10,0.20,0.30,0.40")
    ap.add_argument("--ground-z", type=float, default=0.0)
    ap.add_argument("--no-auto-align-initial-ground", action="store_true")
    ap.add_argument("--min-inter-leg-clearance", type=float, default=0.05)
    ap.add_argument("--min-target-point-clearance", type=float, default=0.04)
    ap.add_argument("--no-contact-lock-generation", action="store_true")
    ap.add_argument("--filter-window", type=int, default=5)
    ap.add_argument("--contact-preserving-filter", action="store_true")
    ap.add_argument("--contact-drift-warn", type=float, default=None, help="Deprecated alias for --contact-drift-soft-limit")
    ap.add_argument("--contact-drift-soft-limit", type=float, default=0.05)
    ap.add_argument("--contact-drift-hard-limit", type=float, default=0.15)
    ap.add_argument("--output", default="")
    ap.add_argument("--summary-only", action="store_true")
    args = ap.parse_args()

    gen_cls = SynchronizedRollCandidateGenerator if args.trajectory_mode == "synchronized" else V3RollCandidateGenerator
    gen = gen_cls(config=build_generation_config(args))
    cand = gen.generate_forward_one_roll(surface_id=args.surface_id)
    evaluator = WholeRollEvaluator(gen.robot_model, WholeRollEvaluationConfig(
        filter_window=args.filter_window,
        ground_z=args.ground_z,
        min_inter_leg_clearance_m=args.min_inter_leg_clearance,
        contact_drift_soft_limit_m=(args.contact_drift_warn if args.contact_drift_warn is not None else args.contact_drift_soft_limit),
        contact_drift_hard_limit_m=args.contact_drift_hard_limit,
        second_joint_abs_max_deg=gen.robot_model.leg_config.second_joint_abs_max_deg,
        contact_preserving_filter=args.contact_preserving_filter,
    ))
    whole = evaluator.evaluate(cand)
    data = {
        "candidate_report_task_success": cand.report.task_success,
        "candidate_report_joint_limit": cand.report.joint_limit,
        "candidate_report_ik_reachability": cand.report.ik_reachability,
        "whole_roll_evaluation": whole,
    }
    if args.summary_only:
        fr = whole["filtered_command"]["geometry"]
        data = {
            "contact_plan_variant": args.contact_plan_variant,
            "trajectory_mode": args.trajectory_mode,
            "contact_lock_generation_enabled": not args.no_contact_lock_generation,
            "candidate_completed": whole["candidate_completed"],
            "whole_roll_success_by_filtered_geometry": whole["whole_roll_success_by_filtered_geometry"],
            "frame_count": whole["frame_count"],
            "filter": whole["filter"],
            "filter_projection_failure_count": whole["filter"]["contact_preserving_projection"].get("projection_failure_count", 0),
            "filter_projected_count": whole["filter"]["contact_preserving_projection"].get("projected_count", 0),
            "raw_max_joint_delta_deg": whole["raw_command"]["max_joint_delta_deg"],
            "filtered_max_joint_delta_deg": whole["filtered_command"]["max_joint_delta_deg"],
            "filtered_penetration_count": fr["ground_clearance"]["penetration_count"],
            "filtered_min_clearance_m": fr["ground_clearance"]["min_clearance_m"],
            "filtered_near_count": fr["inter_leg_clearance"]["near_count"],
            "filtered_max_second_joint_deg": fr["joint_limit"]["max_abs_second_joint_deg"],
            "raw_contact_drift_violation_count": whole["raw_contact_lock"]["contact_drift_violation_count"],
            "raw_max_contact_drift_m": whole["raw_contact_lock"]["max_contact_drift_m"],
            "contact_drift_soft_limit_m": whole["filter"]["contact_drift_soft_limit_m"],
            "contact_drift_hard_limit_m": whole["filter"]["contact_drift_hard_limit_m"],
            "filtered_contact_drift_violation_count": whole["filtered_contact_lock"]["contact_drift_violation_count"],
            "filtered_contact_drift_soft_violation_count": whole["filtered_contact_lock"]["contact_drift_soft_violation_count"],
            "filtered_contact_drift_hard_violation_count": whole["filtered_contact_lock"]["contact_drift_hard_violation_count"],
            "filtered_contact_drift_soft_excess_sum_m": whole["filtered_contact_lock"]["contact_drift_soft_excess_sum_m"],
            "filtered_contact_drift_hard_excess_sum_m": whole["filtered_contact_lock"]["contact_drift_hard_excess_sum_m"],
            "filtered_max_contact_drift_m": whole["filtered_contact_lock"]["max_contact_drift_m"],
            "contact_drift_violation_count": whole["contact_lock"]["contact_drift_violation_count"],
            "contact_drift_hard_violation_count": whole["contact_lock"]["contact_drift_hard_violation_count"],
            "max_contact_drift_m": whole["contact_lock"]["max_contact_drift_m"],
            "generator_ik_failure_count": fr["ik_failure_count_from_generator"],
            "failure_diagnosis": {
                "dominant_failure_category": whole["failure_diagnosis"].get("dominant_failure_category"),
                "generator_ik_failure_by_phase": whole["failure_diagnosis"]["categories"]["generator_ik_failure"]["histogram"]["by_phase"],
                "generator_ik_failure_by_leg": whole["failure_diagnosis"]["categories"]["generator_ik_failure"]["histogram"]["by_leg"],
                "filtered_penetration_by_phase": whole["failure_diagnosis"]["categories"]["filtered_penetration"]["histogram"]["by_phase"],
            },
        }
    text = json.dumps(data, indent=2, sort_keys=True)
    if args.output:
        d = os.path.dirname(args.output)
        if d and not os.path.isdir(d):
            os.makedirs(d)
        with open(args.output, "w") as f:
            f.write(text)
            f.write("\n")
    print(text)


if __name__ == "__main__":
    main()
