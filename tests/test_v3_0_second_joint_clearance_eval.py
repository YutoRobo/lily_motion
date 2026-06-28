# -*- coding: utf-8 -*-
from __future__ import print_function

from lily_motion_v3.legacy_constraint_evaluator import LegacyConstraintEvaluator


def test_second_joint_clearance_fields_exist_on_empty_report():
    ev = LegacyConstraintEvaluator(default_body_z=0.35)
    rep = ev.evaluate([], top_n=3)
    assert 'clearance_by_part' in rep
    assert 'second_joint' in rep['clearance_by_part']
    assert 'foot' in rep['clearance_by_part']
    assert rep['second_joint_clearance']['point_name'] == 'second_joint'
    assert rep['foot_clearance']['point_name'] == 'foot'
    assert rep['second_joint_clearance']['penetration_count'] == 0
