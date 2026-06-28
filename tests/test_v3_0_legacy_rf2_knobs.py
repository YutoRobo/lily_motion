# -*- coding: utf-8 -*-
from __future__ import division
import unittest
from lily_motion_v3.legacy_state_machine_emulator import LegacyStateMachineConfig, LegacyStateMachineEmulator

class TestLegacyRf2Knobs(unittest.TestCase):
    def test_goal2_knobs_are_recorded_and_generate_commands(self):
        cfg = LegacyStateMachineConfig(max_step=10, goal2_dist_front=0.35, goal2_pitch_scale=0.8, goal2_landing_z=0.03)
        emu = LegacyStateMachineEmulator(cfg)
        records = emu.run_forward_roll()
        self.assertTrue(len(records) > 0)
        self.assertIn('joint_command_rad', records[0])
        self.assertEqual(len(records[0]['joint_command_rad']), 24)
        phases = set(r.get('phase_name') for r in records)
        self.assertIn('RF-2_Goal2_UpperLegLanding', phases)

if __name__ == '__main__':
    unittest.main()
