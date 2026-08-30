# -*- coding: utf-8 -*-
from __future__ import division

import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GAZEBO_TOOLS = os.path.join(ROOT, 'tools', 'gazebo')
if GAZEBO_TOOLS not in sys.path:
    sys.path.insert(0, GAZEBO_TOOLS)

from runtime_readiness import (
    GAZEBO_MODEL_STATES_TOPIC,
    GAZEBO_WORLD_SERVICE,
    controller_status,
    expected_controller_topics,
    gazebo_ready,
)


class GazeboRuntimeReadinessTest(unittest.TestCase):
    def test_gazebo_requires_topic_and_service(self):
        self.assertTrue(gazebo_ready(
            [GAZEBO_MODEL_STATES_TOPIC], [GAZEBO_WORLD_SERVICE]))
        self.assertFalse(gazebo_ready([], [GAZEBO_WORLD_SERVICE]))
        self.assertFalse(gazebo_ready([GAZEBO_MODEL_STATES_TOPIC], []))

    def test_all_controller_topics_ready(self):
        topics = expected_controller_topics()
        ready, total, missing = controller_status(topics)
        self.assertEqual(total, 24)
        self.assertEqual(ready, total)
        self.assertEqual(missing, [])

    def test_partial_controller_topics_report_missing(self):
        topics = expected_controller_topics()
        ready, total, missing = controller_status(topics[:-1])
        self.assertEqual(total, 24)
        self.assertEqual(ready, 23)
        self.assertEqual(missing, [topics[-1]])

    def test_controller_topic_names_are_canonical(self):
        for topic in expected_controller_topics():
            self.assertTrue(topic.startswith('/'))


if __name__ == '__main__':
    unittest.main()
