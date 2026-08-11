# -*- coding: utf-8 -*-
from __future__ import division

try:
    import imp
except ImportError:
    imp = None
try:
    from StringIO import StringIO
except ImportError:
    from io import StringIO
import json
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
RUNNER_PATH = os.path.join(ROOT, 'tools', 'run_v3_0_command_stream.py')

from lily_motion_v3.command_stream import (
    prepare_transport_stream,
    transport_stream_sha256,
)


def load_source(name, path):
    if imp is not None:
        return imp.load_source(name, path)
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SharedCommandStreamTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runner = load_source('shared_command_stream_runner_under_test', RUNNER_PATH)

    def _make_log(self):
        handle = tempfile.NamedTemporaryFile(mode='w', delete=False)
        try:
            for index, value in enumerate((0.0, 2.0, 4.0)):
                handle.write(json.dumps({
                    'frame_index': index,
                    'roll_index': 7,
                    'joint_command_rad': [value] * 24,
                }) + '\n')
            return handle.name
        finally:
            handle.close()

    def _run_and_capture(self, argv):
        previous = sys.stdout
        buffer = StringIO()
        try:
            sys.stdout = buffer
            result = self.runner.main(argv)
        finally:
            sys.stdout = previous
        return result, buffer.getvalue()

    def _extract_digest(self, output):
        for line in output.splitlines():
            if line.startswith('transport_sha256='):
                return line.split('=', 1)[1].strip()
        self.fail('transport_sha256 not found in output: %s' % output)

    def test_prepare_transport_preserves_metadata_and_midpoint(self):
        path = self._make_log()
        try:
            source, transport = prepare_transport_stream(
                path, resample_factor=2)
        finally:
            os.unlink(path)
        self.assertEqual(3, len(source))
        self.assertEqual(5, len(transport))
        self.assertEqual(7, transport[1]['roll_index'])
        self.assertAlmostEqual(1.0, transport[1]['joint_command_rad'][0])

    def test_stream_digest_is_stable(self):
        path = self._make_log()
        try:
            _source, transport1 = prepare_transport_stream(
                path, resample_factor=2)
            _source, transport2 = prepare_transport_stream(
                path, resample_factor=2)
        finally:
            os.unlink(path)
        self.assertEqual(
            transport_stream_sha256(transport1),
            transport_stream_sha256(transport2))

    def test_jetson_and_gazebo_dry_run_use_identical_transport_stream(self):
        path = self._make_log()
        common = [
            '--command-log', path,
            '--transport-resample-factor', '2',
            '--transport-rate', '10',
            '--dry-run',
        ]
        try:
            _result, jetson_output = self._run_and_capture(
                ['--backend', 'jetson'] + common)
            _result, gazebo_output = self._run_and_capture(
                ['--backend', 'gazebo'] + common + [
                    '--actuator-interp-duration-sec', '0.100',
                    '--actuator-update-period-sec', '0.002',
                ])
        finally:
            os.unlink(path)
        self.assertEqual(
            self._extract_digest(jetson_output),
            self._extract_digest(gazebo_output))
        self.assertIn('backend=jetson', jetson_output)
        self.assertIn('backend=gazebo', gazebo_output)


if __name__ == '__main__':
    unittest.main()
