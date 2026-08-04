# -*- coding: utf-8 -*-
from __future__ import division

try:
    import imp
except ImportError:
    imp = None
import json
import math
import os
import tempfile
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ONE_LEG_PATH = os.path.join(
    ROOT, "tools", "publish_cmdforjetson_one_leg_test.py")
MAPPED_PATH = os.path.join(
    ROOT, "tools", "publish_cmdforjetson_mapped_axis_replay.py")


def load_source(name, path):
    if imp is not None:
        return imp.load_source(name, path)
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class OneLegPublisherPureTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_source(
            "one_leg_publisher_under_test", ONE_LEG_PATH)

    def test_leg_axes_and_three_finite_values(self):
        axes = self.module.build_leg_axes(3)
        self.assertEqual([9, 10, 11], axes)
        positions = self.module.build_position(
            axes, [0.001, 0.002, -0.001])
        self.assertEqual(24, len(positions))
        self.assertEqual(
            [0.001, 0.002, -0.001], [positions[i] for i in axes])
        self.assertEqual(
            3, sum(0 if math.isnan(value) else 1
                   for value in positions))

    def test_individual_sequence_keeps_all_three_axes_finite(self):
        axes, steps = self.module.build_sequence(
            3, "individual", "plus", [0.0, 0.0, 0.0],
            0.002, 0.001, 0.5, 1.0, 1.0, 1.0, 1.0)
        self.assertEqual([9, 10, 11], axes)
        self.assertEqual(
            [0.0, 0.0, 0.0],
            [steps[0]["positions"][i] for i in axes])
        self.assertEqual(
            [0.0, 0.0, 0.0],
            [steps[-1]["positions"][i] for i in axes])
        for step in steps:
            self.assertEqual(
                3, sum(0 if math.isnan(value) else 1
                       for value in step["positions"]))
        axis9_values = [step["positions"][9] for step in steps]
        axis10_values = [step["positions"][10] for step in steps]
        axis11_values = [step["positions"][11] for step in steps]
        self.assertIn(0.002, axis9_values)
        self.assertIn(0.002, axis10_values)
        self.assertIn(0.002, axis11_values)

    def test_coordinated_sequence_moves_all_three_together(self):
        axes, steps = self.module.build_sequence(
            3, "coordinated", "minus", [0.0, 0.0, 0.0],
            0.002, 0.001, 0.5, 1.0, 1.0, 1.0, 1.0)
        peaks = [[step["positions"][i] for i in axes]
                 for step in steps]
        self.assertIn([-0.002, -0.002, -0.002], peaks)

    def test_invalid_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            self.module.build_leg_axes(8)
        with self.assertRaises(ValueError):
            self.module.parse_triplet("0,0", "--centers-rad")
        with self.assertRaises(ValueError):
            self.module.build_offsets("plus", 0.021, 0.001)
        with self.assertRaises(ValueError):
            self.module.build_sequence(
                3, "bad", "plus", [0.0, 0.0, 0.0],
                0.002, 0.001, 0.5, 1.0, 1.0, 1.0, 1.0)


class MappedAxisPublisherPureTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_source(
            "mapped_axis_publisher_under_test", MAPPED_PATH)

    def _make_log(self, frames):
        handle = tempfile.NamedTemporaryFile(mode="w", delete=False)
        try:
            for frame in frames:
                handle.write(json.dumps({
                    "joint_command_rad": frame}) + "\n")
            return handle.name
        finally:
            handle.close()

    def test_load_and_map_relative_axis(self):
        frames = []
        for value in (1.0, 1.1, 0.8):
            frame = [0.0] * 24
            frame[3] = value
            frames.append(frame)
        path = self._make_log(frames)
        try:
            samples, keys = self.module.load_axis_samples(path, 3)
        finally:
            os.unlink(path)
        self.assertEqual([1.0, 1.1, 0.8], samples)
        self.assertEqual(["joint_command_rad"], keys)
        mapping = self.module.map_samples(samples, 0.0, 0.05, 0.02)
        self.assertAlmostEqual(0.0, mapping["mapped_values_rad"][0])
        self.assertAlmostEqual(0.005, mapping["mapped_values_rad"][1])
        self.assertAlmostEqual(-0.010, mapping["mapped_values_rad"][2])
        self.assertEqual(0, mapping["clipped_count"])

    def test_mapping_clips_and_inverts(self):
        mapping = self.module.map_samples(
            [0.0, 1.0, -1.0], 0.0, 1.0, 0.01, invert=True)
        self.assertEqual(
            [0.0, -0.01, 0.01], mapping["mapped_values_rad"])
        self.assertEqual(2, mapping["clipped_count"])

    def test_output_has_one_finite_physical_axis(self):
        positions = self.module.build_position(10, 0.004)
        self.assertEqual(24, len(positions))
        self.assertEqual(0.004, positions[10])
        self.assertEqual(
            1, sum(0 if math.isnan(value) else 1
                   for value in positions))

    def test_return_ramp_reaches_center_without_large_last_step(self):
        values = self.module.build_return_values(0.0035, 0.0, 0.001)
        self.assertEqual(0.0, values[-1])
        previous = 0.0035
        for value in values:
            self.assertLessEqual(
                abs(value - previous), 0.001 + 1e-12)
            previous = value

    def test_invalid_source_length_and_limit_rejected(self):
        path = self._make_log([[0.0] * 23])
        try:
            with self.assertRaises(ValueError):
                self.module.load_axis_samples(path, 0)
        finally:
            os.unlink(path)
        with self.assertRaises(ValueError):
            self.module.map_samples([0.0, 1.0], 0.0, 1.0, 0.021)


if __name__ == "__main__":
    unittest.main()
