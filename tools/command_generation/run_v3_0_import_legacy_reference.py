#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Import an actual legacy/reference joint command log into v3-core.

This is the recommended route when a real legacy run exists: do not guess the
motion; import the command sequence, evaluate it with v3-core, visualize it, and
replay it in Gazebo.
"""
from __future__ import print_function
import argparse
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.reference_importer import candidate_from_reference_file, candidate_to_json_file
from lily_motion_v3.robot_model import RobotModel
from lily_motion_v3.whole_roll_evaluator import WholeRollEvaluator, WholeRollEvaluationConfig
from lily_motion_v3.gazebo_export import V3GazeboCommandExporter
from lily_motion_v3.command_filter import filter_joint_trajectory


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True, help="JSONL/JSON/CSV containing a legacy/reference joint command trajectory")
    ap.add_argument("--input-format", choices=["auto", "jsonl", "json", "csv"], default="auto")
    ap.add_argument("--input-unit", choices=["rad", "deg"], default="rad")
    ap.add_argument("--ground-z", type=float, default=0.0)
    ap.add_argument("--no-auto-align-initial-ground", action="store_true")
    ap.add_argument("--filter-window", type=int, default=3)
    ap.add_argument("--contact-drift-soft-limit", type=float, default=0.05)
    ap.add_argument("--contact-drift-hard-limit", type=float, default=0.15)
    ap.add_argument("--min-inter-leg-clearance", type=float, default=0.05)
    ap.add_argument("--candidate-output", default="testdata/imported_reference_candidate.json")
    ap.add_argument("--command-output", default="testdata/imported_reference_commands.jsonl")
    ap.add_argument("--command-source", choices=["raw", "filtered"], default="raw",
                    help="raw preserves the imported command exactly; filtered applies v3 moving average")
    ap.add_argument("--summary-only", action="store_true")
    args = ap.parse_args()

    robot_model = RobotModel()
    cand = candidate_from_reference_file(
        args.input,
        input_format=args.input_format,
        input_unit=args.input_unit,
        robot_model=robot_model,
        ground_z=args.ground_z,
        auto_align_initial_ground=not args.no_auto_align_initial_ground,
    )
    candidate_to_json_file(cand, args.candidate_output)

    evaluator = WholeRollEvaluator(robot_model, WholeRollEvaluationConfig(
        filter_window=args.filter_window,
        ground_z=args.ground_z,
        min_inter_leg_clearance_m=args.min_inter_leg_clearance,
        contact_drift_soft_limit_m=args.contact_drift_soft_limit,
        contact_drift_hard_limit_m=args.contact_drift_hard_limit,
        second_joint_abs_max_deg=robot_model.leg_config.second_joint_abs_max_deg,
    ))
    whole = evaluator.evaluate(cand)

    # Optional immediate command export for Gazebo replay.
    exporter = V3GazeboCommandExporter(robot_model)
    frames = cand.frames
    if args.command_source == "filtered":
        qmaps = filter_joint_trajectory(cand.frames, args.filter_window)
    else:
        qmaps = [f.joint_angles for f in cand.frames]
    dirname = os.path.dirname(args.command_output)
    if dirname and not os.path.isdir(dirname):
        os.makedirs(dirname)
    with open(args.command_output, "w") as f:
        for i, frame in enumerate(frames):
            class Proxy(object):
                pass
            p = Proxy()
            p.joint_angles = qmaps[i]
            cmd = exporter.frame_to_joint_state_order(p)
            rec = {
                "command_index": i,
                "command_source": args.command_source,
                "frame_index": frame.frame_index,
                "phase_name": frame.phase_name,
                "joint_command_rad": cmd,
            }
            f.write(json.dumps(rec, sort_keys=True))
            f.write("\n")

    fr = whole["filtered_command"]["geometry"]
    summary = {
        "version_note": "v3.0.20: imported reference trajectory support; no old-project code is called.",
        "input": args.input,
        "candidate_output": args.candidate_output,
        "command_output": args.command_output,
        "command_source": args.command_source,
        "frame_count": len(cand.frames),
        "candidate_completed": cand.report.task_success.get("completed"),
        "geometry_evaluation_note": cand.report.task_success.get("geometry_evaluation_note"),
        "whole_roll_success_by_filtered_geometry": whole["whole_roll_success_by_filtered_geometry"],
        "filtered_penetration_count": fr["ground_clearance"]["penetration_count"],
        "filtered_min_clearance_m": fr["ground_clearance"]["min_clearance_m"],
        "filtered_near_count": fr["inter_leg_clearance"]["near_count"],
        "filtered_max_second_joint_deg": fr["joint_limit"]["max_abs_second_joint_deg"],
        "filtered_second_joint_violation_count": fr["joint_limit"].get("second_joint_violation_count"),
        "filtered_max_contact_drift_m": whole["filtered_contact_lock"].get("max_contact_drift_m"),
        "filtered_contact_drift_hard_violation_count": whole["filtered_contact_lock"].get("contact_drift_hard_violation_count"),
        "failure_diagnosis": whole.get("failure_diagnosis"),
    }
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
