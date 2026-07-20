# -*- coding: utf-8 -*-
"""Shared robot geometry constants for v3 kinematics and legacy emulation."""
from __future__ import division

COXA_LENGTH = 0.075
THIGH_LENGTH = 0.3
TIBIA_LENGTH = 0.3

# Vendored legacy Leg expects a 1-based link vector: [unused, coxa, thigh, tibia].
LINK_LENGTHS = (0.0, COXA_LENGTH, THIGH_LENGTH, TIBIA_LENGTH)
