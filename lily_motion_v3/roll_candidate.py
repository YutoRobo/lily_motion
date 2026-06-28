# -*- coding: utf-8 -*-
"""Data objects for project-contained v3 roll candidates."""
from __future__ import division


class V3MotionFrame(object):
    def __init__(self, frame_index, phase_index, phase_name, phase_step_index,
                 phase_step_count, contact_state, base_pose, leg_roles,
                 foot_targets_body, joint_angles, diagnostics=None, foot_targets_world=None):
        self.frame_index = int(frame_index)
        self.phase_index = int(phase_index)
        self.phase_name = str(phase_name)
        self.phase_step_index = int(phase_step_index)
        self.phase_step_count = int(phase_step_count)
        self.contact_state = contact_state
        self.base_pose = dict(base_pose or {})
        self.leg_roles = dict(leg_roles or {})
        self.foot_targets_body = dict(foot_targets_body or {})
        self.foot_targets_world = dict(foot_targets_world or {})
        self.joint_angles = dict(joint_angles or {})
        self.diagnostics = dict(diagnostics or {})

    def to_dict(self):
        return {
            "frame_index": self.frame_index,
            "phase_index": self.phase_index,
            "phase_name": self.phase_name,
            "phase_step_index": self.phase_step_index,
            "phase_step_count": self.phase_step_count,
            "contact_state": self.contact_state.to_dict() if self.contact_state else None,
            "base_pose": dict(self.base_pose),
            "leg_roles": dict((str(k), v) for k, v in self.leg_roles.items()),
            "foot_targets_body": dict((str(k), list(v)) for k, v in self.foot_targets_body.items()),
            "foot_targets_world": dict((str(k), list(v)) for k, v in self.foot_targets_world.items()),
            "joint_angles": dict((str(k), list(v)) for k, v in self.joint_angles.items()),
            "diagnostics": dict(self.diagnostics),
        }


class V3RollCandidate(object):
    def __init__(self, direction, phases, frames, report):
        self.direction = str(direction)
        self.phases = list(phases or [])
        self.frames = list(frames or [])
        self.report = report

    def to_dict(self):
        return {
            "direction": self.direction,
            "phase_count": len(self.phases),
            "frame_count": len(self.frames),
            "phases": [p.to_dict() for p in self.phases],
            "frames": [f.to_dict() for f in self.frames],
            "report": self.report.to_dict() if self.report else None,
        }
