# -*- coding: utf-8 -*-
from __future__ import division

try:
    import imp
except ImportError:
    imp = None
import json
import os
import tempfile
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PUBLISHER_PATH = os.path.join(ROOT, "tools", "publish_cmdforjetson_jsonl.py")


def load_source(name, path):
    if imp is not None:
        return imp.load_source(name, path)
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class JsonlPublisherResamplingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_source(
            "publish_cmdforjetson_jsonl_under_test", PUBLISHER_PATH)

    def _make_log(self, scalar_values):
        handle = tempfile.NamedTemporaryFile(mode="w", delete=False)
        try:
            for value in scalar_values:
                handle.write(json.dumps({
                    "joint_command_rad": [float(value)] * 24}) + "\n")
            return handle.name
        finally:
            handle.close()

    def _axis0_values(self, path, factor=1, start_index=0, max_frames=None):
        rows = list(self.module.iter_resampled_positions(
            path,
            start_index=start_index,
            max_frames=max_frames,
            resample_factor=factor))
        return [row[2][0] for row in rows], rows

    def test_factor_one_preserves_source_frames(self):
        path = self._make_log([0.0, 2.0, 4.0])
        try:
            values, rows = self._axis0_values(path, factor=1)
        finally:
            os.unlink(path)
        self.assertEqual([0.0, 2.0, 4.0], values)
        self.assertEqual(3, len(rows))

    def test_factor_two_inserts_midpoints_and_preserves_endpoints(self):
        path = self._make_log([0.0, 2.0, 4.0])
        try:
            values, rows = self._axis0_values(path, factor=2)
        finally:
            os.unlink(path)
        self.assertEqual([0.0, 1.0, 2.0, 3.0, 4.0], values)
        self.assertEqual(5, len(rows))
        self.assertEqual(0.5, rows[1][4])
        self.assertEqual(0.5, rows[3][4])

    def test_factor_ten_has_expected_output_count(self):
        path = self._make_log([0.0, 1.0, 2.0])
        try:
            values, _rows = self._axis0_values(path, factor=10)
        finally:
            os.unlink(path)
        self.assertEqual((3 - 1) * 10 + 1, len(values))
        self.assertAlmostEqual(0.1, values[1])
        self.assertAlmostEqual(1.9, values[-2])
        self.assertAlmostEqual(2.0, values[-1])

    def test_start_and_max_frames_apply_before_interpolation(self):
        path = self._make_log([0.0, 2.0, 4.0, 6.0])
        try:
            values, _rows = self._axis0_values(
                path, factor=2, start_index=1, max_frames=2)
        finally:
            os.unlink(path)
        self.assertEqual([2.0, 3.0, 4.0], values)

    def test_invalid_factor_is_rejected(self):
        path = self._make_log([0.0, 1.0])
        try:
            with self.assertRaises(ValueError):
                list(self.module.iter_resampled_positions(
                    path, resample_factor=0))
        finally:
            os.unlink(path)


if __name__ == "__main__":
    unittest.main()
