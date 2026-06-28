#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Evaluate a legacy-style v3-core roll profile.

This script is intentionally independent of the old project.  It maps legacy
parameters into a v3-core candidate and evaluates it with the same whole-roll
raw/filtered/contact-drift metrics as v3-native profiles.
"""
from __future__ import print_function
import argparse
import json
import math
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.legacy_style_generator import LegacyStyleRollCandidateGenerator, LegacyStyleRollGenerationConfig
from lily_motion_v3.whole_roll_evaluator import WholeRollEvaluator, WholeRollEvaluationConfig


def _parse_float_list(text):
    return [float(x.strip()) for x in str(text).split(',') if x.strip()]


def build_config(args):
    return LegacyStyleRollGenerationConfig(
        legacy_step_scale=args.step_scale,
        legacy_splited_num=args.splited_num,
        legacy_rf2_pitch_scale=args.rf2_pitch_scale,
        legacy_rf2_x_scale=args.rf2_x_scale,
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
        min_inter_leg_clearance_m=args.min_inter_leg_clearance,
        min_target_point_clearance_m=args.min_target_point_clearance,
        enable_contact_lock_generation=not args.no_contact_lock_generation,
    )


def summarize(args, whole):
    fr = whole["filtered_command"]["geometry"]
    return {
        "profile": "legacy_style",
        "note": "v3-core legacy-style scaffold; no old-project imports or legacy IK calls.",
        "legacy_params": {
            "direction": "forward",
            "step_scale": args.step_scale,
            "splited_num": args.splited_num,
            "rf2_pitch_scale": args.rf2_pitch_scale,
            "rf2_x_scale": args.rf2_x_scale,
            "filter_window": args.filter_window,
        },
        "candidate_completed": whole["candidate_completed"],
        "whole_roll_success_by_filtered_geometry": whole["whole_roll_success_by_filtered_geometry"],
        "frame_count": whole["frame_count"],
        "filter": whole["filter"],
        "raw_max_joint_delta_deg": whole["raw_command"]["max_joint_delta_deg"],
        "filtered_max_joint_delta_deg": whole["filtered_command"]["max_joint_delta_deg"],
        "filtered_penetration_count": fr["ground_clearance"]["penetration_count"],
        "filtered_min_clearance_m": fr["ground_clearance"]["min_clearance_m"],
        "filtered_near_count": fr["inter_leg_clearance"]["near_count"],
        "filtered_max_second_joint_deg": fr["joint_limit"]["max_abs_second_joint_deg"],
        "generator_ik_failure_count": fr["ik_failure_count_from_generator"],
        "raw_max_contact_drift_m": whole["raw_contact_lock"]["max_contact_drift_m"],
        "filtered_max_contact_drift_m": whole["filtered_contact_lock"]["max_contact_drift_m"],
        "filtered_contact_drift_soft_violation_count": whole["filtered_contact_lock"]["contact_drift_soft_violation_count"],
        "filtered_contact_drift_hard_violation_count": whole["filtered_contact_lock"]["contact_drift_hard_violation_count"],
        "failure_diagnosis": {
            "dominant_failure_category": whole["failure_diagnosis"].get("dominant_failure_category"),
            "generator_ik_failure_by_phase": whole["failure_diagnosis"]["categories"]["generator_ik_failure"]["histogram"]["by_phase"],
            "generator_ik_failure_by_leg": whole["failure_diagnosis"]["categories"]["generator_ik_failure"]["histogram"]["by_leg"],
            "filtered_penetration_by_phase": whole["failure_diagnosis"]["categories"]["filtered_penetration"]["histogram"]["by_phase"],
        },
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step-scale", type=float, default=1.5)
    ap.add_argument("--splited-num", type=int, default=10)
    ap.add_argument("--rf2-pitch-scale", type=float, default=1.0)
    ap.add_argument("--rf2-x-scale", type=float, default=1.0)
    ap.add_argument("--lift-height", type=float, default=0.12)
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
    ap.add_argument("--min-inter-leg-clearance", type=float, default=0.05)
    ap.add_argument("--min-target-point-clearance", type=float, default=0.04)
    ap.add_argument("--no-contact-lock-generation", action="store_true")
    ap.add_argument("--filter-window", type=int, default=3)
    ap.add_argument("--contact-preserving-filter", action="store_true")
    ap.add_argument("--contact-drift-soft-limit", type=float, default=0.05)
    ap.add_argument("--contact-drift-hard-limit", type=float, default=0.15)
    ap.add_argument("--output", default="")
    ap.add_argument("--summary-only", action="store_true")
    args = ap.parse_args()

    gen = LegacyStyleRollCandidateGenerator(config=build_config(args))
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
    data = summarize(args, whole) if args.summary_only else {
        "candidate": cand.to_dict(),
        "whole_roll_evaluation": whole,
        "summary": summarize(args, whole),
    }
    text = json.dumps(data, indent=2, sort_keys=True)
    if args.output:
        d = os.path.dirname(args.output)
        if d and not os.path.isdir(d):
            os.makedirs(d)
        with open(args.output, "w") as f:
            f.write(text + "\n")
    print(text)


if __name__ == "__main__":
    main()
