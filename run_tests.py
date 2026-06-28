#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Run v3-core tests only."""
from __future__ import print_function
import unittest

suite = unittest.defaultTestLoader.discover('tests', pattern='test_v3_0_*.py')
result = unittest.TextTestRunner(verbosity=2).run(suite)
raise SystemExit(0 if result.wasSuccessful() else 1)
