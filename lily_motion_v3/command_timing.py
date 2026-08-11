# -*- coding: utf-8 -*-
"""Shared transport-timing utilities for hardware and Gazebo replay.

The shared path ends at the transport target stream.  MCU/actuator interpolation
is intentionally not implemented here: real hardware performs it in firmware,
while Gazebo uses ``gazebo_actuator_interpolator`` after the shared stream.

Python 2.7 and Python 3 are supported.
"""
from __future__ import division, print_function

from lily_motion_v3.command_resampler import resample_command_records


def linear_interpolate_command(command0, command1, alpha):
    """Linearly interpolate two equal-length command vectors."""
    if len(command0) != len(command1):
        raise ValueError('command lengths differ: %d != %d' % (
            len(command0), len(command1)))
    alpha = float(alpha)
    return [
        float(a) + (float(b) - float(a)) * alpha
        for a, b in zip(command0, command1)
    ]


def resample_transport_records(records, factor=1, segment_key=None):
    """Build the backend-independent transport target stream.

    This delegates to the existing project resampler so hardware and Gazebo use
    one definition for source->transport interpolation.
    """
    factor = int(factor)
    if factor < 1:
        raise ValueError('factor must be >= 1')
    return resample_command_records(
        records, factor=factor, segment_key=segment_key)
