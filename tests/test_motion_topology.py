# -*- coding: utf-8 -*-
from __future__ import division

import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OPERATOR_TOOLS = os.path.join(ROOT, 'tools', 'operator_ui')
if OPERATOR_TOOLS not in sys.path:
    sys.path.insert(0, OPERATOR_TOOLS)

from motion_topology import (
    GAZEBO_INTERPOLATOR_NODE,
    NORMAL_OPERATOR_NODE,
    check_subscriber_topology,
    connection_count_candidate_ok,
)


class MotionTopologyTest(unittest.TestCase):
    def test_normal_operator_state_machine_only(self):
        ok, reason = check_subscriber_topology(
            NORMAL_OPERATOR_NODE, [NORMAL_OPERATOR_NODE], 1)
        self.assertTrue(ok, reason)

    def test_normal_operator_state_machine_plus_gazebo(self):
        ok, reason = check_subscriber_topology(
            NORMAL_OPERATOR_NODE,
            [NORMAL_OPERATOR_NODE, GAZEBO_INTERPOLATOR_NODE],
            2)
        self.assertTrue(ok, reason)

    def test_normal_operator_rejects_gazebo_without_state_machine(self):
        ok, unused_reason = check_subscriber_topology(
            NORMAL_OPERATOR_NODE, [GAZEBO_INTERPOLATOR_NODE], 1)
        self.assertFalse(ok)

    def test_normal_operator_rejects_unknown_subscriber(self):
        ok, unused_reason = check_subscriber_topology(
            NORMAL_OPERATOR_NODE,
            [NORMAL_OPERATOR_NODE, '/unexpected_consumer'],
            2)
        self.assertFalse(ok)

    def test_normal_operator_rejects_connection_count_mismatch(self):
        ok, unused_reason = check_subscriber_topology(
            NORMAL_OPERATOR_NODE,
            [NORMAL_OPERATOR_NODE, GAZEBO_INTERPOLATOR_NODE],
            1)
        self.assertFalse(ok)

    def test_other_hosts_keep_exactly_one_rule(self):
        ok, reason = check_subscriber_topology(
            '/lily_operator_gazebo_ui', [GAZEBO_INTERPOLATOR_NODE], 1)
        self.assertTrue(ok, reason)
        ok, unused_reason = check_subscriber_topology(
            '/lily_operator_gazebo_ui',
            [GAZEBO_INTERPOLATOR_NODE, '/extra'],
            2)
        self.assertFalse(ok)

    def test_cheap_connection_precheck(self):
        self.assertTrue(connection_count_candidate_ok(NORMAL_OPERATOR_NODE, 1))
        self.assertTrue(connection_count_candidate_ok(NORMAL_OPERATOR_NODE, 2))
        self.assertFalse(connection_count_candidate_ok(NORMAL_OPERATOR_NODE, 0))
        self.assertFalse(connection_count_candidate_ok(NORMAL_OPERATOR_NODE, 3))
        self.assertTrue(connection_count_candidate_ok('/other', 1))
        self.assertFalse(connection_count_candidate_ok('/other', 2))


if __name__ == '__main__':
    unittest.main()
