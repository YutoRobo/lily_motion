# -*- coding: utf-8 -*-
from __future__ import division
import unittest
from lily_motion_v3.legacy_state_machine_emulator import LegacyStateMachineConfig, LegacyStateMachineEmulator, command_diagnostics


class TestV322LegacyStateMachine(unittest.TestCase):
    def test_legacy_state_machine_generates_commands(self):
        cfg = LegacyStateMachineConfig(max_step=10, initialize_step=5, include_initialize=False)
        emu = LegacyStateMachineEmulator(cfg)
        recs = emu.run_forward_roll()
        self.assertTrue(len(recs) > 0)
        self.assertIn('joint_command_rad', recs[0])
        self.assertEqual(len(recs[0]['joint_command_rad']), 24)
        diag = command_diagnostics(recs)
        self.assertTrue(diag['nonzero_joint_count'] > 0)
        self.assertTrue(diag['max_delta_rad'] > 0.1)

if __name__ == '__main__':
    unittest.main()
