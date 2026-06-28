# -*- coding: utf-8 -*-
"""Self-contained v3 robot model: mounts + per-leg FK/IK."""
from __future__ import division

from lily_motion_v3.leg_config import LegKinematicConfig, default_octpus_mounts
from lily_motion_v3.leg_kinematics import LegKinematics
from lily_motion_v3.transforms import rot_z, mat_vec_mul, mat_transpose, vec_add, vec_sub, rpy_matrix


class RobotModel(object):
    def __init__(self, leg_config=None, mounts=None):
        self.leg_config = leg_config or LegKinematicConfig()
        self.mounts = mounts or default_octpus_mounts()
        self.kinematics = LegKinematics(self.leg_config)
        self.mount_by_id = dict((m.leg_id, m) for m in self.mounts)
        self.mount_by_name = dict((m.leg_name, m) for m in self.mounts)

    def leg_name(self, leg_id):
        return self.mount_by_id[int(leg_id)].leg_name

    def leg_id(self, leg_name):
        return self.mount_by_name[str(leg_name)].leg_id

    def foot_position_body(self, leg_id, q):
        mount = self.mount_by_id[int(leg_id)]
        local_p = self.kinematics.forward_kinematics(q)
        R = rot_z(mount.yaw_rad)
        return vec_add(mount.position, mat_vec_mul(R, local_p))

    def ik_candidates_body(self, leg_id, foot_target_body):
        mount = self.mount_by_id[int(leg_id)]
        R = rot_z(mount.yaw_rad)
        Rt = mat_transpose(R)
        target_local = mat_vec_mul(Rt, vec_sub(foot_target_body, mount.position))
        return self.kinematics.inverse_kinematics_candidates(target_local)

    def select_ik_body(self, leg_id, foot_target_body, previous_q=None):
        cands = self.ik_candidates_body(leg_id, foot_target_body)
        return self.kinematics.select_candidate(cands, previous_q=previous_q)


    def base_rotation_matrix(self, base_pose):
        return rpy_matrix(
            float(base_pose.get("roll", 0.0)),
            float(base_pose.get("pitch", 0.0)),
            float(base_pose.get("yaw", 0.0)),
        )

    def base_position(self, base_pose):
        return [
            float(base_pose.get("x", 0.0)),
            float(base_pose.get("y", 0.0)),
            float(base_pose.get("z", 0.0)),
        ]

    def body_point_to_world(self, point_body, base_pose):
        R = self.base_rotation_matrix(base_pose)
        return vec_add(self.base_position(base_pose), mat_vec_mul(R, point_body))

    def world_point_to_body(self, point_world, base_pose):
        R = self.base_rotation_matrix(base_pose)
        Rt = mat_transpose(R)
        return mat_vec_mul(Rt, vec_sub(point_world, self.base_position(base_pose)))

    def foot_position_world(self, leg_id, q, base_pose):
        return self.body_point_to_world(self.foot_position_body(leg_id, q), base_pose)


    def link_positions_body(self, leg_id, q):
        """Return representative leg link points in the body frame."""
        mount = self.mount_by_id[int(leg_id)]
        local_points = self.kinematics.link_positions(q)
        R = rot_z(mount.yaw_rad)
        out = {}
        for name, p in local_points.items():
            out[name] = vec_add(mount.position, mat_vec_mul(R, p))
        return out

    def leg_segments_body(self, leg_id, q):
        """Return named representative segments for a leg in the body frame."""
        pts = self.link_positions_body(leg_id, q)
        return [
            {"leg_id": int(leg_id), "leg_name": self.leg_name(leg_id), "segment_name": "mount_to_coxa", "a": pts["mount"], "b": pts["coxa_end"]},
            {"leg_id": int(leg_id), "leg_name": self.leg_name(leg_id), "segment_name": "coxa_to_knee", "a": pts["coxa_end"], "b": pts["knee"]},
            {"leg_id": int(leg_id), "leg_name": self.leg_name(leg_id), "segment_name": "knee_to_foot", "a": pts["knee"], "b": pts["foot"]},
        ]


    def link_positions_world(self, leg_id, q, base_pose):
        body_points = self.link_positions_body(leg_id, q)
        return dict((name, self.body_point_to_world(p, base_pose))
                    for name, p in body_points.items())

    def leg_segments_world(self, leg_id, q, base_pose):
        pts = self.link_positions_world(leg_id, q, base_pose)
        return [
            {"leg_id": int(leg_id), "leg_name": self.leg_name(leg_id), "segment_name": "mount_to_coxa", "a": pts["mount"], "b": pts["coxa_end"]},
            {"leg_id": int(leg_id), "leg_name": self.leg_name(leg_id), "segment_name": "coxa_to_knee", "a": pts["coxa_end"], "b": pts["knee"]},
            {"leg_id": int(leg_id), "leg_name": self.leg_name(leg_id), "segment_name": "knee_to_foot", "a": pts["knee"], "b": pts["foot"]},
        ]
