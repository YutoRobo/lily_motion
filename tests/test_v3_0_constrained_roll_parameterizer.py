# -*- coding: utf-8 -*-
from __future__ import print_function
import unittest
from lily_motion_v3.constrained_roll_parameterizer import PITCH_PROFILES, linspace, _goal5_for_goal2

class TestConstrainedRollParameterizer(unittest.TestCase):
    def test_linspace(self):
        self.assertEqual(linspace(0, 1, 3), [0.0, 0.5, 1.0])

    def test_pitch_profiles_exist(self):
        self.assertIn('legacy', PITCH_PROFILES)
        self.assertIn('balanced', PITCH_PROFILES)
        self.assertIn('late_roll', PITCH_PROFILES)

    def test_goal5_compensation(self):
        g2 = 0.8
        g5 = _goal5_for_goal2(g2)
        total = 0.025 + 0.225 * g2 + 0.25 * g5
        self.assertAlmostEqual(total, 0.5, places=9)

if __name__ == '__main__':
    unittest.main()
