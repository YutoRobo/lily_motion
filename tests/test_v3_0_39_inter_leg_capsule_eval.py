# -*- coding: utf-8 -*-
from __future__ import print_function

import math
import numpy as np

from lily_motion_v3.legacy_constraint_evaluator import LegacyConstraintEvaluator, _segment_distance


def test_segment_distance_detects_interior_closest_points():
    # Two perpendicular segments crossing at their interiors must have zero
    # distance.  The old endpoint-only proxy would miss this case.
    a0 = np.array([-1.0, 0.0, 0.0])
    a1 = np.array([1.0, 0.0, 0.0])
    b0 = np.array([0.0, -1.0, 0.0])
    b1 = np.array([0.0, 1.0, 0.0])
    assert abs(_segment_distance(a0, a1, b0, b1)) < 1e-9


def test_capsule_threshold_fields_exist_on_empty_report():
    ev = LegacyConstraintEvaluator(
        default_body_z=0.35,
        inter_leg_limit_m=0.04,
        leg_radius_m=0.015,
        inter_leg_safety_margin_m=0.010,
    )
    rep = ev.evaluate([], top_n=3)
    assert rep['inter_leg_link_radius_m'] == 0.015
    assert rep['inter_leg_collision_threshold_m'] == 0.03
    assert rep['inter_leg_required_clearance_m'] == 0.04
    assert rep['inter_leg_collision_count'] == 0
    assert rep['inter_leg_near_count'] == 0
    assert rep['inter_leg_collision']['method'] == 'capsule_segment_distance'


def test_joint_housing_threshold_fields_exist_on_empty_report():
    ev = LegacyConstraintEvaluator(
        default_body_z=0.35,
        inter_leg_limit_m=0.04,
        leg_radius_m=0.015,
        inter_leg_safety_margin_m=0.010,
        joint_housing_radius_m=0.030,
        joint_housing_safety_margin_m=0.005,
    )
    rep = ev.evaluate([], top_n=3)
    jh = rep['inter_leg_joint_housing_collision']
    assert rep['inter_leg_joint_housing_collision_count'] == 0
    assert rep['inter_leg_joint_housing_near_count'] == 0
    assert jh['method'] == 'joint_sphere_to_other_leg_link_capsule_distance'
    assert abs(jh['collision_threshold_m'] - 0.045) < 1e-12
    assert abs(jh['required_clearance_m'] - 0.050) < 1e-12
