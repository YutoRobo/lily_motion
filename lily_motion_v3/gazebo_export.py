# -*- coding: utf-8 -*-
"""Export project-contained v3 frames to existing Gazebo/JointState order.

This module deliberately uses leg names as the bridge because v3 leg ids are
self-contained and do not match the legacy/Gazebo leg-id constants.  The output
order follows lily_motion_v3.interface_config.JOINT_STATE_ORDER so existing
Gazebo controller topics can be reused.
"""
from __future__ import division

from lily_motion_v3.interface_config import JOINT_STATE_ORDER, LEG_NAMES_BY_ID, NUM_JOINTS


class V3GazeboCommandExporter(object):
    def __init__(self, v3_robot_model=None):
        self.v3_robot_model = v3_robot_model
        if v3_robot_model is not None:
            self.v3_leg_id_by_name = dict((m.leg_name, m.leg_id) for m in v3_robot_model.mounts)
        else:
            # Project-contained v3 default order.
            self.v3_leg_id_by_name = {
                "TRF": 0, "TRH": 1, "BRF": 2, "BRH": 3,
                "TLF": 4, "TLH": 5, "BLF": 6, "BLH": 7,
            }

    def frame_to_joint_state_order(self, frame):
        """Return a 24-rad command from a V3MotionFrame or frame dict."""
        if hasattr(frame, "joint_angles"):
            joint_angles = frame.joint_angles
        else:
            joint_angles = frame.get("joint_angles", {})
        out = []
        for legacy_leg_id, joint_index in JOINT_STATE_ORDER:
            leg_name = LEG_NAMES_BY_ID[legacy_leg_id]
            if leg_name not in self.v3_leg_id_by_name:
                raise KeyError("v3 model has no leg named %s" % leg_name)
            v3_leg_id = self.v3_leg_id_by_name[leg_name]
            q = self._get_leg_q(joint_angles, v3_leg_id)
            out.append(float(q[joint_index]))
        if len(out) != NUM_JOINTS:
            raise RuntimeError("exported joint command length must be %d" % NUM_JOINTS)
        return out

    def _get_leg_q(self, joint_angles, leg_id):
        if leg_id in joint_angles:
            return joint_angles[leg_id]
        key = str(leg_id)
        if key in joint_angles:
            return joint_angles[key]
        raise KeyError("missing v3 joint_angles for leg_id %s" % leg_id)


def frame_is_invalid(frame, stop_on_ik_failure=True, stop_on_ground_penetration=True,
                     stop_on_base_pose_failure=True):
    """Return (invalid_bool, reasons) for preview/replay stopping."""
    if hasattr(frame, "diagnostics"):
        diag = frame.diagnostics or {}
    else:
        diag = frame.get("diagnostics", {}) or {}
    reasons = []
    if stop_on_ik_failure and diag.get("ik_failures"):
        reasons.append("ik_failure")
    if stop_on_ground_penetration:
        g = diag.get("ground_clearance") or {}
        if g.get("penetrating"):
            reasons.append("ground_penetration")
    if stop_on_base_pose_failure:
        ts = (diag.get("target_selection") or {})
        bps = ts.get("base_pose_search") or diag.get("base_pose_search") or {}
        if bps and not bps.get("selected_feasible", True):
            reasons.append("base_pose_search_failure")
    return (len(reasons) > 0), reasons


def frames_until_invalid(frames, include_invalid=False, **kwargs):
    """Return frames up to first invalid frame plus metadata."""
    selected = []
    first_invalid = None
    for frame in frames:
        invalid, reasons = frame_is_invalid(frame, **kwargs)
        if invalid:
            first_invalid = {
                "frame_index": getattr(frame, "frame_index", None) if hasattr(frame, "frame_index") else frame.get("frame_index"),
                "phase_name": getattr(frame, "phase_name", None) if hasattr(frame, "phase_name") else frame.get("phase_name"),
                "phase_step_index": getattr(frame, "phase_step_index", None) if hasattr(frame, "phase_step_index") else frame.get("phase_step_index"),
                "reasons": reasons,
            }
            if include_invalid:
                selected.append(frame)
            break
        selected.append(frame)
    return selected, first_invalid
