# -*- coding: utf-8 -*-
import unittest
from lily_motion_v3.legacy_state_machine_emulator import LegacyStateMachineConfig, LegacyStateMachineEmulator
from lily_motion_v3.legacy_ik_branch_diagnostics import LegacyIKBranchDiagnostics

class TestIKBranchDiagnostics(unittest.TestCase):
    def test_diagnose_returns_four_candidates(self):
        records = LegacyStateMachineEmulator(LegacyStateMachineConfig(max_step=10)).run_forward_roll()
        report = LegacyIKBranchDiagnostics().diagnose_frame(records, surface_id=1)
        self.assertIn('candidates', report)
        self.assertEqual(len(report['candidates']), 4)
        self.assertIn('diagnosis_hint', report)

if __name__ == '__main__':
    unittest.main()
