# -*- coding: utf-8 -*-
"""Contact-lock tracking for v3 trajectory-level evaluation.

A support foot is treated as contact-locked from the first frame in which its
role is SUPPORT until the role leaves SUPPORT.  During that interval its FK foot
position should stay close to the lock point in world coordinates.
"""
from __future__ import division

from lily_motion_v3.transforms import vec_sub, norm
from lily_motion_v3 import leg_role as R


class ContactLockTracker(object):
    def __init__(self, robot_model, drift_warn_m=0.03, drift_hard_limit_m=None):
        self.robot_model = robot_model
        self.drift_warn_m = float(drift_warn_m)
        # Hard limit is used for pass/fail.  Soft/warn limit is used for scoring.
        self.drift_hard_limit_m = (None if drift_hard_limit_m is None else float(drift_hard_limit_m))

    def evaluate(self, frames, joint_maps=None):
        frames = list(frames or [])
        joint_maps = list(joint_maps or [f.joint_angles for f in frames])
        locks = {}
        lock_records = []
        violations = []
        hard_violations = []
        soft_excess_sum = 0.0
        hard_excess_sum = 0.0
        max_drift = 0.0
        max_record = None

        for idx, frame in enumerate(frames):
            qmap = joint_maps[idx]
            active_support_ids = set()
            for leg_id, role in frame.leg_roles.items():
                leg_id = int(leg_id)
                if role == R.SUPPORT:
                    active_support_ids.add(leg_id)
                    foot = self.robot_model.foot_position_world(leg_id, qmap[leg_id], frame.base_pose)
                    if leg_id not in locks:
                        locks[leg_id] = {
                            "lock_point_world": list(foot),
                            "start_frame_index": frame.frame_index,
                            "start_phase_name": frame.phase_name,
                            "start_phase_step_index": frame.phase_step_index,
                        }
                    lock = locks[leg_id]
                    drift = norm(vec_sub(foot, lock["lock_point_world"]))
                    rec = {
                        "frame_index": frame.frame_index,
                        "phase_name": frame.phase_name,
                        "phase_step_index": frame.phase_step_index,
                        "leg_id": leg_id,
                        "leg_name": self.robot_model.leg_name(leg_id),
                        "drift_m": drift,
                        "foot_world": list(foot),
                        "lock_point_world": list(lock["lock_point_world"]),
                        "lock_start_frame_index": lock["start_frame_index"],
                        "lock_start_phase_name": lock["start_phase_name"],
                    }
                    if drift > max_drift:
                        max_drift = drift
                        max_record = rec
                    if drift > self.drift_warn_m:
                        violations.append(rec)
                        soft_excess_sum += drift - self.drift_warn_m
                    if self.drift_hard_limit_m is not None and drift > self.drift_hard_limit_m:
                        hard_violations.append(rec)
                        hard_excess_sum += drift - self.drift_hard_limit_m
                    lock_records.append(rec)
            # Release locks for legs no longer in SUPPORT.
            for leg_id in list(locks.keys()):
                if leg_id not in active_support_ids:
                    del locks[leg_id]

        return {
            "drift_warn_m": self.drift_warn_m,
            "drift_soft_limit_m": self.drift_warn_m,
            "drift_hard_limit_m": self.drift_hard_limit_m,
            "max_contact_drift_m": max_drift,
            "max_contact_drift_record": max_record,
            # Backward-compatible alias: violation_count means soft-limit violation.
            "contact_drift_violation_count": len(violations),
            "contact_drift_soft_violation_count": len(violations),
            "contact_drift_hard_violation_count": len(hard_violations),
            "contact_drift_soft_excess_sum_m": soft_excess_sum,
            "contact_drift_hard_excess_sum_m": hard_excess_sum,
            "top_contact_drift_violations": violations[:20],
            "top_contact_drift_hard_violations": hard_violations[:20],
            "top_contact_lock_records": lock_records[:20],
        }
