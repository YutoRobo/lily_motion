# -*- coding: utf-8 -*-
"""Configuration objects for project-contained v3 kinematics."""
from __future__ import division
import math

from lily_motion_v3.robot_geometry import COXA_LENGTH, THIGH_LENGTH, TIBIA_LENGTH


class LegKinematicConfig(object):
    """Length and joint-limit configuration for one insect-style 3-DOF leg.

    Joint convention used by v3:
      q0: yaw around local +z
      q1: thigh pitch in the yaw-selected vertical plane
      q2: tibia pitch relative to thigh

    The default lengths follow the current URDF geometry, with the first/coxa
    link corrected to 0.075 m.
    """

    def __init__(self, coxa_length=COXA_LENGTH, thigh_length=THIGH_LENGTH, tibia_length=TIBIA_LENGTH,
                 second_joint_abs_max_deg=95.0,
                 joint_abs_max_deg=None):
        self.coxa_length = float(coxa_length)
        self.thigh_length = float(thigh_length)
        self.tibia_length = float(tibia_length)
        self.second_joint_abs_max_deg = float(second_joint_abs_max_deg)
        if joint_abs_max_deg is None:
            joint_abs_max_deg = [180.0, 180.0, 180.0]
        self.joint_abs_max_deg = [float(v) for v in joint_abs_max_deg]

    @property
    def second_joint_abs_max_rad(self):
        return math.radians(self.second_joint_abs_max_deg)


class LegMount(object):
    """Mount pose of a leg in body frame.

    Only yaw rotation is supported initially.  This is intentional: v3 Step 0
    should be simple, explicit, and independent from xacro/legacy code.
    """

    def __init__(self, leg_id, leg_name, position, yaw_rad):
        self.leg_id = int(leg_id)
        self.leg_name = str(leg_name)
        self.position = [float(position[0]), float(position[1]), float(position[2])]
        self.yaw_rad = float(yaw_rad)


# Conventional 8-leg order used by the current project logs.
DEFAULT_LEG_NAMES = ["TRF", "TRH", "BRF", "BRH", "TLF", "TLH", "BLF", "BLH"]


def default_octpus_mounts(body_half_x=0.2, body_half_y=0.2, body_half_z=0.2):
    """Return a simple symmetric 8-leg mount set.

    This is not claimed to be exact URDF geometry.  It is a self-contained
    project default that can be replaced by a checked-in config file later.
    """
    bx = float(body_half_x)
    by = float(body_half_y)
    bz = float(body_half_z)
    return [
        LegMount(0, "TRF", [ bx, -by,  bz], 0.0),
        LegMount(1, "TRH", [-bx, -by,  bz], math.pi),
        LegMount(2, "BRF", [ bx, -by, -bz], 0.0),
        LegMount(3, "BRH", [-bx, -by, -bz], math.pi),
        LegMount(4, "TLF", [ bx,  by,  bz], 0.0),
        LegMount(5, "TLH", [-bx,  by,  bz], math.pi),
        LegMount(6, "BLF", [ bx,  by, -bz], 0.0),
        LegMount(7, "BLH", [-bx,  by, -bz], math.pi),
    ]
