import unittest
from lily_motion_v3.constrained_roll_parameterizer import (
    _is_finite_number,
    is_valid_constrained_case,
)

class TestFiniteNumberPython27Compatibility(unittest.TestCase):
    def test_accepts_finite_numeric_values(self):
        for value in (12.5, -12.5, 0, "93.0"):
            self.assertTrue(_is_finite_number(value), repr(value))

    def test_rejects_non_finite_and_non_numeric_values(self):
        for value in (float("nan"), float("inf"),
                      float("-inf"), "not-a-number"):
            self.assertFalse(_is_finite_number(value), repr(value))


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

    def test_accepts_clean_case_with_python27_finite_check(self):
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
