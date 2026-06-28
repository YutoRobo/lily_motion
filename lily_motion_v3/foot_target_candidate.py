# -*- coding: utf-8 -*-
"""Foot-target candidate generation and local selection for v3 roll planning."""
from __future__ import division
import math

from lily_motion_v3 import leg_role as R
from lily_motion_v3.geometry import distance
from lily_motion_v3.transforms import vec_add, angle_delta


class FootTargetCandidate(object):
    def __init__(self, leg_id, target, reason, selected_ik=None,
                 feasible=False, score=None, violations=None, metrics=None):
        self.leg_id = int(leg_id)
        self.target = [float(target[0]), float(target[1]), float(target[2])]
        self.reason = str(reason)
        self.selected_ik = selected_ik
        self.feasible = bool(feasible)
        self.score = score
        self.violations = list(violations or [])
        self.metrics = dict(metrics or {})

    def to_dict(self):
        return {
            "leg_id": self.leg_id,
            "target": list(self.target),
            "reason": self.reason,
            "feasible": self.feasible,
            "score": self.score,
            "violations": list(self.violations),
            "metrics": dict(self.metrics),
            "selected_ik": None if self.selected_ik is None else self.selected_ik.to_dict(),
        }


class CandidateFootTargetConfig(object):
    def __init__(self, lift_height=0.08, clearance_height=0.06,
                 candidate_support_shift_x=0.04, candidate_support_drop_z=-0.02,
                 lateral_offsets=None, x_offsets=None, z_offsets=None,
                 min_point_clearance_m=0.04, min_target_z=-0.60,
                 continuity_weight=1.0, second_joint_margin_weight=0.05,
                 clearance_weight=0.10):
        self.lift_height = float(lift_height)
        self.clearance_height = float(clearance_height)
        self.candidate_support_shift_x = float(candidate_support_shift_x)
        self.candidate_support_drop_z = float(candidate_support_drop_z)
        self.lateral_offsets = list(lateral_offsets or [-0.03, 0.0, 0.03])
        self.x_offsets = list(x_offsets or [-0.03, 0.0, 0.03])
        self.z_offsets = list(z_offsets or [-0.02, 0.0, 0.02])
        self.min_point_clearance_m = float(min_point_clearance_m)
        self.min_target_z = float(min_target_z)
        self.continuity_weight = float(continuity_weight)
        self.second_joint_margin_weight = float(second_joint_margin_weight)
        self.clearance_weight = float(clearance_weight)


class CandidateFootTargetGenerator(object):
    """Generate and choose portable foot-target candidates.

    This intentionally does not know the old controller.  It uses only v3 RobotModel,
    IK candidates, previous joint continuity, second-joint margin, and a simple
    point-distance clearance to other current foot targets.  Full segment/capsule
    collision checking is handled by the motion report after a full frame exists.
    """
    def __init__(self, robot_model, config=None):
        self.robot_model = robot_model
        self.config = config or CandidateFootTargetConfig()

    def generate_candidates(self, leg_id, role, current_target):
        role = str(role)
        leg_id = int(leg_id)
        base = list(current_target)
        out = []

        if role == R.LIFT:
            for dx in self.config.x_offsets:
                for dy in self.config.lateral_offsets:
                    for dz in [self.config.lift_height * 0.75,
                               self.config.lift_height,
                               self.config.lift_height * 1.25]:
                        out.append(FootTargetCandidate(leg_id, vec_add(base, [dx, dy, dz]), "lift_grid"))
        elif role == R.CLEARANCE:
            for dy in self.config.lateral_offsets:
                for dz in [self.config.clearance_height,
                           self.config.clearance_height * 1.5]:
                    out.append(FootTargetCandidate(leg_id, vec_add(base, [0.0, dy, dz]), "clearance_grid"))
        elif role == R.CANDIDATE_SUPPORT:
            sign = 1.0 if leg_id % 2 == 0 else -1.0
            for dx_extra in self.config.x_offsets:
                for dy in self.config.lateral_offsets:
                    for dz_extra in self.config.z_offsets:
                        out.append(FootTargetCandidate(
                            leg_id,
                            vec_add(base, [sign * self.config.candidate_support_shift_x + dx_extra,
                                           dy,
                                           self.config.candidate_support_drop_z + dz_extra]),
                            "candidate_support_grid"))
        else:
            out.append(FootTargetCandidate(leg_id, base, "keep_current"))
        return out

    def choose_target(self, leg_id, role, current_target, previous_q=None,
                      other_current_targets=None):
        candidates = self.generate_candidates(leg_id, role, current_target)
        evaluated = []
        best = None
        for cand in candidates:
            self._evaluate_candidate(cand, previous_q, other_current_targets or {})
            evaluated.append(cand)
            if cand.feasible and (best is None or cand.score < best.score):
                best = cand
        if best is None:
            # Preserve the least bad diagnostic candidate for reporting.
            return None, evaluated
        return best, evaluated

    def _evaluate_candidate(self, cand, previous_q, other_current_targets):
        violations = []
        metrics = {}
        if cand.target[2] < self.config.min_target_z:
            violations.append("target_z_below_min")

        selected = self.robot_model.select_ik_body(cand.leg_id, cand.target, previous_q=previous_q)
        cand.selected_ik = selected
        if selected is None:
            violations.append("no_feasible_ik")
        else:
            second_margin = self.robot_model.leg_config.second_joint_abs_max_rad - abs(selected.q[1])
            metrics["second_joint_margin_deg"] = math.degrees(second_margin)
            if previous_q is not None:
                metrics["joint_delta_sum_deg"] = sum(
                    abs(math.degrees(angle_delta(selected.q[i], previous_q[i]))) for i in range(3))
            else:
                metrics["joint_delta_sum_deg"] = 0.0

        min_point_distance = None
        for other_leg_id, other_target in other_current_targets.items():
            if int(other_leg_id) == cand.leg_id:
                continue
            d = distance(cand.target, other_target)
            if min_point_distance is None or d < min_point_distance:
                min_point_distance = d
        if min_point_distance is not None:
            metrics["min_point_distance_to_other_targets_m"] = min_point_distance
            if min_point_distance < self.config.min_point_clearance_m:
                violations.append("target_point_clearance_below_min")

        cand.violations = violations
        cand.metrics = metrics
        cand.feasible = len(violations) == 0
        if cand.feasible:
            score = 0.0
            score += self.config.continuity_weight * metrics.get("joint_delta_sum_deg", 0.0)
            score -= self.config.second_joint_margin_weight * metrics.get("second_joint_margin_deg", 0.0)
            score -= self.config.clearance_weight * metrics.get("min_point_distance_to_other_targets_m", 0.0)
            cand.score = score
        else:
            cand.score = None
