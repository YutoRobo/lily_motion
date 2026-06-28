# -*- coding: utf-8 -*-
import unittest
from lily_motion_v3.legacy_state_machine_emulator import LegacyStateMachineConfig, LegacyStateMachineEmulator
from lily_motion_v3.repeated_roll_connection import connection_report

class TestRepeatedRollConnection(unittest.TestCase):
    def test_repeated_records_have_roll_indices(self):
        cfg = LegacyStateMachineConfig(max_step=10, move_dist=0.3, support_dist=0.75, z=0.4)
        emu = LegacyStateMachineEmulator(cfg)
        records = emu.run_forward_repeated([1,5,6])
        self.assertTrue(len(records) > 0)
        self.assertIn('roll_index', records[-1])
        self.assertIn('surface_after', records[-1])
        self.assertEqual(records[-1]['surface_after'], 6)

    def test_connection_report(self):
        cfg = LegacyStateMachineConfig(max_step=10, move_dist=0.3, support_dist=0.75, z=0.4)
        emu = LegacyStateMachineEmulator(cfg)
        records = emu.run_forward_repeated([1,5])
        rep = connection_report(records, default_body_z=0.4)
        self.assertEqual(rep['boundary_count'], 1)
        self.assertIn('body_to_next_support_center_xy_m', rep['boundaries'][0])

if __name__ == '__main__':
    unittest.main()
