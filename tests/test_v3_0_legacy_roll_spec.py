# -*- coding: utf-8 -*-
from __future__ import division
import unittest
from lily_motion_v3.legacy_roll_spec_generator import LegacyRollSpecCandidateGenerator, LegacyRollSpecGenerationConfig

class LegacyRollSpecTest(unittest.TestCase):
    def test_generates_old_five_goal_plus_adjustment_phases(self):
        gen=LegacyRollSpecCandidateGenerator(config=LegacyRollSpecGenerationConfig(max_step=10))
        cand=gen.generate_forward_one_roll(surface_id=1)
        self.assertEqual(len(cand.phases), 6)
        self.assertEqual(cand.phases[0].name, 'RF-1_Goal1_UpperLegPreSwing')
        self.assertEqual(cand.report.task_success['surface_after'], 5)
        self.assertEqual(cand.report.task_success['legacy_dependency'], False)
        self.assertTrue(len(cand.frames) > 0)

if __name__ == '__main__':
    unittest.main()
