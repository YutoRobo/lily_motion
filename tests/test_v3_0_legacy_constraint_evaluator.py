# -*- coding: utf-8 -*-
from __future__ import print_function
import unittest
from lily_motion_v3.legacy_state_machine_emulator import LegacyStateMachineConfig, LegacyStateMachineEmulator
from lily_motion_v3.legacy_constraint_evaluator import LegacyConstraintEvaluator

class TestLegacyConstraintEvaluator(unittest.TestCase):
    def test_evaluate_generated_legacy_commands(self):
        cfg = LegacyStateMachineConfig(surface_id=1, max_step=10, initialize_step=5)
        records = LegacyStateMachineEmulator(cfg).run_forward_roll()
        self.assertTrue(len(records) > 0)
        report = LegacyConstraintEvaluator(second_joint_limit_deg=95.0, inter_leg_limit_m=0.02).evaluate(records, top_n=3)
        self.assertEqual(report['frame_count'], len(records))
        self.assertIn('max_second_joint_deg', report)
        self.assertIn('phase_summary', report)
        self.assertIn('worst_second_joint', report)
        self.assertGreaterEqual(report['max_second_joint_deg'], 0.0)

if __name__ == '__main__':
    unittest.main()
