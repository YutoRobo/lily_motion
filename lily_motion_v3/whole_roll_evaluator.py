# -*- coding: utf-8 -*-
"""Trajectory-level evaluator for v3 roll candidates.

This module evaluates the whole roll rather than judging each frame in
isolation.  It explicitly separates raw and filtered joint commands, and checks
contact-lock drift after filtering.
"""
from __future__ import division
import math

from lily_motion_v3.command_filter import filter_joint_trajectory, filter_joint_trajectory_contact_reproject, max_joint_step_deg
from lily_motion_v3.contact_lock import ContactLockTracker
from lily_motion_v3.geometry import segment_segment_distance
from lily_motion_v3 import leg_role as R
from lily_motion_v3.failure_diagnosis import summarize_failure_diagnosis


class WholeRollEvaluationConfig(object):
    def __init__(self, filter_window=5, ground_z=0.0, min_inter_leg_clearance_m=0.05,
                 contact_drift_warn_m=0.03, second_joint_abs_max_deg=95.0,
                 contact_preserving_filter=False, contact_drift_soft_limit_m=None,
                 contact_drift_hard_limit_m=0.15):
        self.filter_window = int(filter_window)
        self.ground_z = float(ground_z)
        self.min_inter_leg_clearance_m = float(min_inter_leg_clearance_m)
        # v3.0.11: contact drift is not a zero constraint.  The soft limit is
        # a scoring/warning threshold; the hard limit is the pass/fail threshold.
        self.contact_drift_warn_m = float(contact_drift_warn_m if contact_drift_soft_limit_m is None else contact_drift_soft_limit_m)
        self.contact_drift_soft_limit_m = self.contact_drift_warn_m
        self.contact_drift_hard_limit_m = (None if contact_drift_hard_limit_m is None else float(contact_drift_hard_limit_m))
        self.second_joint_abs_max_deg = float(second_joint_abs_max_deg)
        self.contact_preserving_filter = bool(contact_preserving_filter)


class WholeRollEvaluator(object):
    def __init__(self, robot_model, config=None):
        self.robot_model = robot_model
        self.config = config or WholeRollEvaluationConfig(
            second_joint_abs_max_deg=robot_model.leg_config.second_joint_abs_max_deg)

    def evaluate(self, candidate):
        frames = list(candidate.frames or [])
        raw_joint_maps = [f.joint_angles for f in frames]
        if self.config.contact_preserving_filter:
            filtered_joint_maps, filter_projection = filter_joint_trajectory_contact_reproject(
                frames, self.robot_model, self.config.filter_window)
            filter_type = "moving_average_unwrapped_angles_contact_reproject"
        else:
            filtered_joint_maps = filter_joint_trajectory(frames, self.config.filter_window)
            filter_projection = {"enabled": False}
            filter_type = "moving_average_unwrapped_angles"
        raw_max_delta, raw_max_rec = max_joint_step_deg(raw_joint_maps)
        filt_max_delta, filt_max_rec = max_joint_step_deg(filtered_joint_maps)
        raw_geom = self._evaluate_geometry(frames, raw_joint_maps)
        filt_geom = self._evaluate_geometry(frames, filtered_joint_maps)
        raw_contact = ContactLockTracker(
            self.robot_model,
            drift_warn_m=self.config.contact_drift_soft_limit_m,
            drift_hard_limit_m=self.config.contact_drift_hard_limit_m).evaluate(frames, raw_joint_maps)
        filtered_contact = ContactLockTracker(
            self.robot_model,
            drift_warn_m=self.config.contact_drift_soft_limit_m,
            drift_hard_limit_m=self.config.contact_drift_hard_limit_m).evaluate(frames, filtered_joint_maps)
        completed_by_filtered = (
            filt_geom["ik_failure_count_from_generator"] == 0 and
            filt_geom["ground_clearance"]["penetration_count"] == 0 and
            filt_geom["inter_leg_clearance"]["near_count"] == 0 and
            filt_geom["joint_limit"]["second_joint_violation_count"] == 0 and
            filtered_contact.get("contact_drift_hard_violation_count", filtered_contact["contact_drift_violation_count"]) == 0
        )
        result = {
            "version_note": "v3.0.15: v3-core whole-roll evaluation with raw/filtered geometry, soft/hard contact-drift limits, and failure diagnosis.",
            "candidate_completed": bool(candidate.report.task_success.get("completed", False)),
            "frame_count": len(frames),
            "filter": {
                "type": filter_type,
                "window": self.config.filter_window,
                "contact_preserving_projection": filter_projection,
                "contact_drift_soft_limit_m": self.config.contact_drift_soft_limit_m,
                "contact_drift_hard_limit_m": self.config.contact_drift_hard_limit_m,
            },
            "raw_command": {
                "max_joint_delta_deg": raw_max_delta,
                "max_joint_delta_record": raw_max_rec,
                "geometry": raw_geom,
            },
            "filtered_command": {
                "max_joint_delta_deg": filt_max_delta,
                "max_joint_delta_record": filt_max_rec,
                "geometry": filt_geom,
            },
            "raw_contact_lock": raw_contact,
            "filtered_contact_lock": filtered_contact,
            # Backward-compatible alias: contact_lock means filtered contact lock.
            "contact_lock": filtered_contact,
            "whole_roll_success_by_filtered_geometry": completed_by_filtered,
        }
        result["failure_diagnosis"] = summarize_failure_diagnosis(candidate, result)
        return result

    def _evaluate_geometry(self, frames, joint_maps):
        min_clearance = None
        penetration_records = []
        min_inter_leg = None
        near_records = []
        max_second = 0.0
        second_violation_records = []
        for idx, frame in enumerate(frames):
            qmap = joint_maps[idx]
            for leg_id, q in qmap.items():
                second_deg = abs(math.degrees(q[1]))
                if second_deg > max_second:
                    max_second = second_deg
                if second_deg > self.config.second_joint_abs_max_deg + 1e-9:
                    second_violation_records.append({
                        "frame_index": frame.frame_index,
                        "phase_name": frame.phase_name,
                        "phase_step_index": frame.phase_step_index,
                        "leg_id": int(leg_id),
                        "leg_name": self.robot_model.leg_name(leg_id),
                        "second_joint_deg": second_deg,
                    })
                pts = self.robot_model.link_positions_world(leg_id, q, frame.base_pose)
                for point_name, p in pts.items():
                    clearance = p[2] - self.config.ground_z
                    if min_clearance is None or clearance < min_clearance:
                        min_clearance = clearance
                    if clearance < -1e-9:
                        penetration_records.append({
                            "frame_index": frame.frame_index,
                            "phase_name": frame.phase_name,
                            "phase_step_index": frame.phase_step_index,
                            "leg_id": int(leg_id),
                            "leg_name": self.robot_model.leg_name(leg_id),
                            "role": frame.leg_roles.get(int(leg_id), R.OTHER),
                            "point_name": point_name,
                            "point_world": list(p),
                            "clearance_m": clearance,
                        })
            segments = []
            for leg_id, q in qmap.items():
                segments.extend(self.robot_model.leg_segments_world(leg_id, q, frame.base_pose))
            frame_min = None
            frame_min_rec = None
            for i in range(len(segments)):
                for j in range(i + 1, len(segments)):
                    a = segments[i]
                    b = segments[j]
                    if a["leg_id"] == b["leg_id"]:
                        continue
                    d, ca, cb = segment_segment_distance(a["a"], a["b"], b["a"], b["b"])
                    if frame_min is None or d < frame_min:
                        frame_min = d
                        frame_min_rec = {
                            "frame_index": frame.frame_index,
                            "phase_name": frame.phase_name,
                            "phase_step_index": frame.phase_step_index,
                            "distance_m": d,
                            "closest_point_a": ca,
                            "closest_point_b": cb,
                            "segment_a": {"leg_id": a["leg_id"], "leg_name": a["leg_name"], "segment_name": a["segment_name"]},
                            "segment_b": {"leg_id": b["leg_id"], "leg_name": b["leg_name"], "segment_name": b["segment_name"]},
                        }
            if frame_min is not None:
                if min_inter_leg is None or frame_min < min_inter_leg:
                    min_inter_leg = frame_min
                if frame_min < self.config.min_inter_leg_clearance_m:
                    near_records.append(frame_min_rec)
        return {
            "ik_failure_count_from_generator": self._count_generator_ik_failures(frames),
            "ground_clearance": {
                "ground_z": self.config.ground_z,
                "min_clearance_m": min_clearance,
                "penetration_count": len(penetration_records),
                "top_penetration_records": penetration_records[:20],
            },
            "inter_leg_clearance": {
                "threshold_m": self.config.min_inter_leg_clearance_m,
                "min_distance_m": min_inter_leg,
                "near_count": len(near_records),
                "top_near_records": near_records[:20],
            },
            "joint_limit": {
                "second_joint_abs_max_deg": self.config.second_joint_abs_max_deg,
                "max_abs_second_joint_deg": max_second,
                "second_joint_violation_count": len(second_violation_records),
                "top_second_joint_violations": second_violation_records[:20],
            },
        }

    @staticmethod
    def _count_generator_ik_failures(frames):
        count = 0
        for f in frames:
            diag = f.diagnostics or {}
            ik_failures = diag.get("ik_failures", [])
            count += len(ik_failures)
        return count
