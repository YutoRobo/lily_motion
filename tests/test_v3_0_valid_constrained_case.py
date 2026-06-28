import unittest
from lily_motion_v3.constrained_roll_parameterizer import is_valid_constrained_case

class TestValidConstrainedCase(unittest.TestCase):
    def test_rejects_error_case(self):
        case = {
            'admissible_by_periodicity': True,
            'constraints': {'error': 'math domain error', 'max_second_joint_deg': 999.0,
                            'second_joint_violation_count': 999999,
                            'ground_penetration_count': 999999,
                            'inter_leg_near_count': 999999},
            'repeated_roll': {'candidate_completed': False, 'error': {'message': 'math domain error'}},
        }
        self.assertFalse(is_valid_constrained_case(case))

    def test_accepts_clean_case(self):
        case = {
            'admissible_by_periodicity': True,
            'constraints': {'max_second_joint_deg': 93.0,
                            'second_joint_violation_count': 0,
                            'ground_penetration_count': 0,
                            'inter_leg_near_count': 0},
            'repeated_roll': {'candidate_completed': True, 'frame_count': 276},
        }
        self.assertTrue(is_valid_constrained_case(case))

if __name__ == '__main__':
    unittest.main()
