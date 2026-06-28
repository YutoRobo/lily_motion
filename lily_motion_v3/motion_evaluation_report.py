# -*- coding: utf-8 -*-
"""Unified report schema for v2/v3 motion evaluations."""


class MotionEvaluationReport(object):
    def __init__(self):
        self.task_success = {}
        self.joint_limit = {}
        self.ground_clearance = {}
        self.inter_leg_clearance = {}
        self.support_consistency = {}
        self.motion_discontinuity = {}
        self.ik_reachability = {}
        self.base_pose_search = {}
        self.notes = []

    def to_dict(self):
        return {
            "task_success": self.task_success,
            "joint_limit": self.joint_limit,
            "ground_clearance": self.ground_clearance,
            "inter_leg_clearance": self.inter_leg_clearance,
            "support_consistency": self.support_consistency,
            "motion_discontinuity": self.motion_discontinuity,
            "ik_reachability": self.ik_reachability,
            "base_pose_search": self.base_pose_search,
            "notes": list(self.notes),
        }
