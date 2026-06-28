# -*- coding: utf-8 -*-
"""External interface constants for the project-contained v3 package.

This file intentionally duplicates the small amount of Gazebo/JointState order
metadata needed by v3 exporters so that v3 does not import the older
``lily_motion`` package.
"""
from __future__ import division

LEG_NAMES_BY_ID = {
    0: "BLF",
    1: "BLH",
    2: "BRF",
    3: "BRH",
    4: "TLF",
    5: "TLH",
    6: "TRF",
    7: "TRH",
}

JOINT_BASE_CLAUSE = 0
JOINT_THIGH = 1
JOINT_TIBIA = 2
JOINTS_PER_LEG = 3
NUM_LEGS = 8
NUM_JOINTS = 24

# Existing Gazebo/JointState order used by the Lily model.
# Tuples are (external_leg_id, joint_index).  v3 maps these by leg name rather
# than by integer id, because v3's internal leg order is intentionally separate.
JOINT_STATE_ORDER = [
    (2, JOINT_BASE_CLAUSE), (2, JOINT_THIGH), (2, JOINT_TIBIA),  # BRF
    (0, JOINT_BASE_CLAUSE), (0, JOINT_THIGH), (0, JOINT_TIBIA),  # BLF
    (1, JOINT_BASE_CLAUSE), (1, JOINT_THIGH), (1, JOINT_TIBIA),  # BLH
    (3, JOINT_BASE_CLAUSE), (3, JOINT_THIGH), (3, JOINT_TIBIA),  # BRH
    (6, JOINT_BASE_CLAUSE), (6, JOINT_THIGH), (6, JOINT_TIBIA),  # TRF
    (4, JOINT_BASE_CLAUSE), (4, JOINT_THIGH), (4, JOINT_TIBIA),  # TLF
    (5, JOINT_BASE_CLAUSE), (5, JOINT_THIGH), (5, JOINT_TIBIA),  # TLH
    (7, JOINT_BASE_CLAUSE), (7, JOINT_THIGH), (7, JOINT_TIBIA),  # TRH
]

GAZEBO_JOINT_TOPICS_IN_JOINT_STATE_ORDER = [
    "BRF_base_clause_controller/command",
    "BRF_thigh_controller/command",
    "BRF_tibia_controller/command",
    "BLF_base_clause_controller/command",
    "BLF_thigh_controller/command",
    "BLF_tibia_controller/command",
    "BLH_base_clause_controller/command",
    "BLH_thigh_controller/command",
    "BLH_tibia_controller/command",
    "BRH_base_clause_controller/command",
    "BRH_thigh_controller/command",
    "BRH_tibia_controller/command",
    "TRF_base_clause_controller/command",
    "TRF_thigh_controller/command",
    "TRF_tibia_controller/command",
    "TLF_base_clause_controller/command",
    "TLF_thigh_controller/command",
    "TLF_tibia_controller/command",
    "TLH_base_clause_controller/command",
    "TLH_thigh_controller/command",
    "TLH_tibia_controller/command",
    "TRH_base_clause_controller/command",
    "TRH_thigh_controller/command",
    "TRH_tibia_controller/command",
]
