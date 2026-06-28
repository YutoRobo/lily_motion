#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Export v3 roll candidate frames to Gazebo command JSONL.

v3.0.18:
  * supports both v3-native and legacy-style profiles;
  * supports raw or filtered command export;
  * remains ROS-free; replay is handled by run_v3_0_gazebo_replay.py.
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

from lily_motion_v3.v3_roll_candidate_generator import V3RollCandidateGenerator, V3RollGenerationConfig
from lily_motion_v3.synchronized_roll_generator import SynchronizedRollCandidateGenerator, SynchronizedRollGenerationConfig
from lily_motion_v3.legacy_style_generator import LegacyStyleRollCandidateGenerator, LegacyStyleRollGenerationConfig
from lily_motion_v3.legacy_roll_spec_generator import LegacyRollSpecCandidateGenerator, LegacyRollSpecGenerationConfig
from lily_motion_v3.gazebo_export import V3GazeboCommandExporter, frames_until_invalid
from lily_motion_v3.whole_roll_evaluator import WholeRollEvaluator, WholeRollEvaluationConfig
from lily_motion_v3.command_filter import filter_joint_trajectory, filter_joint_trajectory_contact_reproject
from lily_motion_v3.reference_importer import candidate_from_json_file


def _parse_float_list(text):
    out = []
    for part in str(text).split(','):
        part = part.strip()
        if part:
            out.append(float(part))
    return out


def build_native_candidate(args):
    cls = SynchronizedRollGenerationConfig if args.trajectory_mode == "synchronized" else V3RollGenerationConfig
    extra = {}
    if args.trajectory_mode == "synchronized":
        extra = {
            "synchronized_steps": args.synchronized_steps,
            "roll_start_s": args.roll_start_s,
            "roll_end_s": args.roll_end_s,
        }
    cfg = cls(
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
    gen_cls = SynchronizedRollCandidateGenerator if args.trajectory_mode == "synchronized" else V3RollCandidateGenerator
    gen = gen_cls(config=cfg)
    return gen, gen.generate_forward_one_roll(surface_id=args.surface_id)


def build_legacy_candidate(args):
    cfg = LegacyStyleRollGenerationConfig(
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
        auto_align_initial_ground=not args.no_auto_align_initial_ground,
        min_inter_leg_clearance_m=args.min_inter_leg_clearance,
        min_target_point_clearance_m=args.min_target_point_clearance,
        enable_contact_lock_generation=not args.no_contact_lock_generation,
    )
    gen = LegacyStyleRollCandidateGenerator(config=cfg)
    return gen, gen.generate_forward_one_roll(surface_id=args.surface_id)


class _ImportedReferenceContext(object):
    def __init__(self, robot_model):
        self.robot_model = robot_model


def build_legacy_roll_spec_candidate(args):
    cfg = LegacyRollSpecGenerationConfig(
        move_dist=args.move_dist,
        support_dist=args.support_dist,
        max_step=args.max_step,
        surface_id=args.surface_id,
        z=args.legacy_body_z,
        ground_z=args.ground_z,
    )
    gen = LegacyRollSpecCandidateGenerator(config=cfg)
    return gen, gen.generate_forward_one_roll(surface_id=args.surface_id)


def build_imported_reference_candidate(args):
    if not args.candidate:
        raise ValueError("--candidate is required when --profile imported_reference")
    robot_model, cand = candidate_from_json_file(args.candidate)
    return _ImportedReferenceContext(robot_model), cand


def build_candidate(args):
    if args.profile == "imported_reference":
        return build_imported_reference_candidate(args)
    if args.profile == "legacy_roll_spec":
        return build_legacy_roll_spec_candidate(args)
    if args.profile == "legacy_style":
        return build_legacy_candidate(args)
    return build_native_candidate(args)


def filtered_joint_maps_for_candidate(args, gen, cand):
    if args.contact_preserving_filter:
        joint_maps, projection = filter_joint_trajectory_contact_reproject(
            cand.frames, gen.robot_model, args.filter_window)
        filter_type = "moving_average_unwrapped_angles_contact_reproject"
    else:
        joint_maps = filter_joint_trajectory(cand.frames, args.filter_window)
        projection = {"enabled": False}
        filter_type = "moving_average_unwrapped_angles"
    return joint_maps, {"type": filter_type, "window": args.filter_window, "contact_preserving_projection": projection}


class _ExportFrameProxy(object):
    """Frame-like object with a replacement joint map for export only."""
    def __init__(self, frame, joint_angles):
        self._frame = frame
        self.joint_angles = joint_angles
        self.frame_index = frame.frame_index
        self.phase_name = frame.phase_name
        self.phase_index = frame.phase_index
        self.phase_step_index = frame.phase_step_index
        self.phase_step_count = frame.phase_step_count
        self.base_pose = frame.base_pose
        self.leg_roles = frame.leg_roles
        self.diagnostics = frame.diagnostics


def frames_for_command_source(args, gen, cand):
    if args.command_source == "raw":
        return list(cand.frames), {"type": "raw"}
    joint_maps, filter_info = filtered_joint_maps_for_candidate(args, gen, cand)
    return [_ExportFrameProxy(f, joint_maps[i]) for i, f in enumerate(cand.frames)], filter_info


def frame_meta(frame, command_index, command_source, invalid_reasons=None):
    return {
        "command_index": command_index,
        "command_source": command_source,
        "frame_index": frame.frame_index,
        "phase_name": frame.phase_name,
        "phase_index": frame.phase_index,
        "phase_step_index": frame.phase_step_index,
        "phase_step_count": frame.phase_step_count,
        "base_pose": dict(frame.base_pose),
        "leg_roles": dict((str(k), v) for k, v in frame.leg_roles.items()),
        "invalid_reasons": list(invalid_reasons or []),
    }


def evaluate_for_summary(args, gen, cand):
    return WholeRollEvaluator(gen.robot_model, WholeRollEvaluationConfig(
        filter_window=args.filter_window,
        ground_z=args.ground_z,
        min_inter_leg_clearance_m=args.min_inter_leg_clearance,
        contact_drift_soft_limit_m=args.contact_drift_soft_limit,
        contact_drift_hard_limit_m=args.contact_drift_hard_limit,
        second_joint_abs_max_deg=gen.robot_model.leg_config.second_joint_abs_max_deg,
        contact_preserving_filter=args.contact_preserving_filter,
    )).evaluate(cand)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", choices=["native", "legacy_style", "legacy_roll_spec", "imported_reference"], default="native",
                    help="native exports v3-native candidates; legacy_style exports the old qualitative scaffold; legacy_roll_spec exports the table-based reproduction of lily_controller.roll(); imported_reference exports an imported candidate")
    ap.add_argument("--surface-id", type=int, default=1)
    ap.add_argument("--candidate", default="", help="candidate JSON path for --profile imported_reference")
    ap.add_argument("--trajectory-mode", choices=["phase", "synchronized"], default="phase",
                    help="native profile only")
    ap.add_argument("--synchronized-steps", type=int, default=72)
    ap.add_argument("--roll-start-s", type=float, default=0.35)
    ap.add_argument("--roll-end-s", type=float, default=0.85)
    ap.add_argument("--contact-plan-variant", default="default", help="native profile only")
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

    # Legacy-style profile knobs.  They are ignored by native unless future code explicitly maps them.
    ap.add_argument("--step-scale", type=float, default=1.5)
    ap.add_argument("--splited-num", type=int, default=10)
    ap.add_argument("--move-dist", type=float, default=0.4, help="legacy_roll_spec: old controller move_dist")
    ap.add_argument("--support-dist", type=float, default=0.7, help="legacy_roll_spec: old controller support_dist")
    ap.add_argument("--max-step", type=int, default=30, help="legacy_roll_spec: old controller max_step")
    ap.add_argument("--legacy-body-z", type=float, default=0.35, help="legacy_roll_spec: old controller initial body height z")
    ap.add_argument("--rf2-pitch-scale", type=float, default=1.0)
    ap.add_argument("--rf2-x-scale", type=float, default=1.0)

    ap.add_argument("--command-source", choices=["raw", "filtered"], default="filtered")
    ap.add_argument("--filter-window", type=int, default=3)
    ap.add_argument("--contact-preserving-filter", action="store_true")
    ap.add_argument("--contact-drift-soft-limit", type=float, default=0.05)
    ap.add_argument("--contact-drift-hard-limit", type=float, default=0.15)
    ap.add_argument("--output", default="testdata/v3_exported_commands.jsonl")
    ap.add_argument("--include-invalid-frame", action="store_true",
                    help="include the first invalid frame in the exported preview")
    ap.add_argument("--allow-invalid-frames", action="store_true",
                    help="export all frames even if IK/ground/base-pose failures exist")
    args = ap.parse_args()

    gen, cand = build_candidate(args)
    export_frames_all, command_filter_info = frames_for_command_source(args, gen, cand)
    frames = list(export_frames_all)
    first_invalid = None
    if not args.allow_invalid_frames:
        frames, first_invalid = frames_until_invalid(frames, include_invalid=args.include_invalid_frame)

    exporter = V3GazeboCommandExporter(gen.robot_model)
    dirname = os.path.dirname(args.output)
    if dirname and not os.path.isdir(dirname):
        os.makedirs(dirname)
    with open(args.output, "w") as f:
        for i, frame in enumerate(frames):
            cmd = exporter.frame_to_joint_state_order(frame)
            rec = frame_meta(frame, i, args.command_source)
            rec["joint_command_rad"] = cmd
            f.write(json.dumps(rec, sort_keys=True))
            f.write("\n")

    whole = evaluate_for_summary(args, gen, cand)
    fr = whole["filtered_command"]["geometry"]
    summary = {
        "version_note": "v3.0.21: export supports native, legacy_style, legacy_roll_spec, and imported_reference profiles; no old-project calls.",
        "profile": args.profile,
        "trajectory_mode": args.trajectory_mode if args.profile == "native" else args.profile,
        "command_source": args.command_source,
        "command_filter": command_filter_info,
        "output": args.output,
        "exported_command_count": len(frames),
        "candidate_frame_count": len(cand.frames),
        "candidate_completed": cand.report.task_success.get("completed"),
        "first_invalid_frame": first_invalid,
        "whole_roll_success_by_filtered_geometry": whole["whole_roll_success_by_filtered_geometry"],
        "filtered_penetration_count": fr["ground_clearance"]["penetration_count"],
        "filtered_near_count": fr["inter_leg_clearance"]["near_count"],
        "filtered_max_second_joint_deg": fr["joint_limit"]["max_abs_second_joint_deg"],
        "generator_ik_failure_count": fr["ik_failure_count_from_generator"],
        "filtered_max_contact_drift_m": whole["filtered_contact_lock"]["max_contact_drift_m"],
        "filtered_contact_drift_hard_violation_count": whole["filtered_contact_lock"].get("contact_drift_hard_violation_count"),
        "report_task_success": cand.report.task_success,
    }
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
