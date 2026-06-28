# -*- coding: utf-8 -*-
"""Project-contained FK/IK for the v3 insect-style 3-DOF leg."""
from __future__ import division
import math

from lily_motion_v3.transforms import clamp, angle_delta


class IkCandidate(object):
    def __init__(self, q, branch_name, reachable=True, acos_argument=None,
                 reason=""):
        self.q = [float(q[0]), float(q[1]), float(q[2])]
        self.branch_name = str(branch_name)
        self.reachable = bool(reachable)
        self.acos_argument = acos_argument
        self.reason = str(reason)
        self.score = None
        self.violations = []

    def to_dict(self):
        return {
            "q": list(self.q),
            "branch_name": self.branch_name,
            "reachable": self.reachable,
            "acos_argument": self.acos_argument,
            "reason": self.reason,
            "score": self.score,
            "violations": list(self.violations),
        }


class LegKinematics(object):
    def __init__(self, config):
        self.config = config

    def forward_kinematics(self, q):
        """Return foot position in the leg mount frame."""
        q0, q1, q2 = float(q[0]), float(q[1]), float(q[2])
        coxa = self.config.coxa_length
        L1 = self.config.thigh_length
        L2 = self.config.tibia_length

        # Planar position after coxa offset.  Positive z is upward.
        radial = coxa + L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
        z = L1 * math.sin(q1) + L2 * math.sin(q1 + q2)
        x = math.cos(q0) * radial
        y = math.sin(q0) * radial
        return [x, y, z]


    def link_positions(self, q):
        """Return representative joint/link points in the leg mount frame.

        Points are: mount origin, coxa end, knee, foot.  They are not exact
        collision geometry; they are portable geometric primitives used by the
        v3 planner/evaluator before Gazebo-specific collision checks exist.
        """
        q0, q1, q2 = float(q[0]), float(q[1]), float(q[2])
        coxa = self.config.coxa_length
        L1 = self.config.thigh_length
        L2 = self.config.tibia_length
        c0 = math.cos(q0)
        s0 = math.sin(q0)

        p0 = [0.0, 0.0, 0.0]
        p1 = [c0 * coxa, s0 * coxa, 0.0]
        r2 = coxa + L1 * math.cos(q1)
        p2 = [c0 * r2, s0 * r2, L1 * math.sin(q1)]
        r3 = coxa + L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
        p3 = [c0 * r3, s0 * r3, L1 * math.sin(q1) + L2 * math.sin(q1 + q2)]
        return {
            "mount": p0,
            "coxa_end": p1,
            "knee": p2,
            "foot": p3,
        }

    def inverse_kinematics_candidates(self, target):
        """Return elbow-up/down IK candidates for a target in leg frame.

        No candidate is silently selected here.  Selection is handled by
        select_candidate() so constraints and previous joint continuity are
        explicit and testable.
        """
        x, y, z = float(target[0]), float(target[1]), float(target[2])
        coxa = self.config.coxa_length
        L1 = self.config.thigh_length
        L2 = self.config.tibia_length

        q0 = math.atan2(y, x)
        radial_total = math.sqrt(x * x + y * y)
        r = radial_total - coxa
        d2 = r * r + z * z
        if L1 <= 0.0 or L2 <= 0.0:
            return [IkCandidate([q0, 0.0, 0.0], "invalid_link", False, None,
                                "link length must be positive")]

        D = (d2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
        if D < -1.0 or D > 1.0:
            # Return clipped candidates for diagnostic continuity, but mark them
            # unreachable so the planner can reject this foot target.
            Dc = clamp(D, -1.0, 1.0)
            candidates = self._candidates_from_D(q0, r, z, Dc, D)
            for c in candidates:
                c.reachable = False
                c.reason = "acos argument outside [-1, 1]"
            return candidates
        return self._candidates_from_D(q0, r, z, D, D)

    def _candidates_from_D(self, q0, r, z, D_for_acos, D_original):
        out = []
        for sign, name in [(1.0, "elbow_positive"), (-1.0, "elbow_negative")]:
            q2 = sign * math.acos(D_for_acos)
            q1 = math.atan2(z, r) - math.atan2(
                self.config.tibia_length * math.sin(q2),
                self.config.thigh_length + self.config.tibia_length * math.cos(q2))
            out.append(IkCandidate([q0, q1, q2], name, True, D_original, ""))
        return out

    def select_candidate(self, candidates, previous_q=None, prefer_second_joint_margin=True):
        """Select one IK candidate using explicit constraints.

        Hard filters:
          - reachable
          - absolute joint limits
          - second joint <= configured limit

        Score:
          - previous joint continuity, if previous_q is supplied
          - second-joint margin as a tie breaker
        """
        best = None
        for cand in candidates:
            cand.violations = self._candidate_violations(cand)
            if not cand.reachable or cand.violations:
                cand.score = None
                continue
            score = 0.0
            if previous_q is not None:
                score += sum(abs(angle_delta(cand.q[i], previous_q[i])) for i in range(3))
            if prefer_second_joint_margin:
                margin = self.config.second_joint_abs_max_rad - abs(cand.q[1])
                score -= 0.01 * margin
            cand.score = score
            if best is None or cand.score < best.score:
                best = cand
        return best

    def _candidate_violations(self, cand):
        violations = []
        if not cand.reachable:
            violations.append("unreachable")
        for i, q in enumerate(cand.q):
            lim = math.radians(self.config.joint_abs_max_deg[i])
            if abs(q) > lim + 1e-9:
                violations.append("joint_%d_abs_limit" % i)
        if abs(cand.q[1]) > self.config.second_joint_abs_max_rad + 1e-9:
            violations.append("second_joint_abs_limit")
        return violations
