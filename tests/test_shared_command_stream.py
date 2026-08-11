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
PUBLISHER_PATH = os.path.join(ROOT, 'tools', 'publish_cmdforjetson_jsonl.py')

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
        cls.publisher = load_source(
            'shared_jsonl_publisher_under_test', PUBLISHER_PATH)

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
            result = self.publisher.main(argv)
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

    def test_canonical_publisher_reports_exact_shared_stream_digest(self):
        path = self._make_log()
        try:
            _source, transport = prepare_transport_stream(
                path, resample_factor=2)
            expected = transport_stream_sha256(transport)
            result, output = self._run_and_capture([
                '--command-log', path,
                '--resample-factor', '2',
                '--rate', '10',
                '--dry-run',
            ])
        finally:
            os.unlink(path)
        self.assertEqual(0, result)
        self.assertEqual(expected, self._extract_digest(output))
        self.assertIn('output_topic=/cmdForJetson', output)
        self.assertNotIn('backend=', output)


if __name__ == '__main__':
    unittest.main()
