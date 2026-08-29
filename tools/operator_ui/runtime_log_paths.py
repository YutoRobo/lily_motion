#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import time


def _mkdir(path):
    if not os.path.isdir(path):
        os.makedirs(path)


def create_operator_log_session(log_root):
    """Create one timestamped Operator UI runtime-log session directory.

    Layout:
        <log_root>/<YYYYMMDD_HHMMSS>/
            monitor/
            motion/
            config/

    A numeric suffix is added only when two sessions are created during the same
    second.  The function returns absolute paths so log placement is independent
    of the shell's current working directory.
    """
    log_root = os.path.abspath(os.path.expanduser(log_root))
    _mkdir(log_root)

    stamp = time.strftime('%Y%m%d_%H%M%S')
    session_dir = os.path.join(log_root, stamp)
    suffix = 1
    while os.path.exists(session_dir):
        session_dir = os.path.join(log_root, '%s_%02d' % (stamp, suffix))
        suffix += 1

    _mkdir(session_dir)
    paths = {
        'root': log_root,
        'session': session_dir,
    }
    for name in ('monitor', 'motion', 'config'):
        path = os.path.join(session_dir, name)
        _mkdir(path)
        paths[name] = path
    return paths
