# -*- coding: utf-8 -*-
from __future__ import print_function
import os
import subprocess
import sys


def test_baseline_script_help_runs():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    script = os.path.join(root, 'run_v3_0_baseline_filtered_constraint_eval.py')
    p = subprocess.Popen([sys.executable, script, '--help'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = p.communicate()
    assert p.returncode == 0
    assert b'baseline' in out.lower()
