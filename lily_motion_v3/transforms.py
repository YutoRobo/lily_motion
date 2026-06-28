# -*- coding: utf-8 -*-
"""Small transform utilities for v3 kinematics.

This module intentionally avoids numpy and ROS dependencies so that v3 can run
in a plain Python environment.  It supports Python 2.7 and Python 3.x.
"""
from __future__ import division
import math


def vec_add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vec_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def norm(a):
    return math.sqrt(dot(a, a))


def mat_vec_mul(R, v):
    return [
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    ]


def mat_mul(A, B):
    out = []
    for i in range(3):
        row = []
        for j in range(3):
            row.append(A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j])
        out.append(row)
    return out


def mat_transpose(R):
    return [
        [R[0][0], R[1][0], R[2][0]],
        [R[0][1], R[1][1], R[2][1]],
        [R[0][2], R[1][2], R[2][2]],
    ]


def rot_x(roll):
    c = math.cos(roll)
    s = math.sin(roll)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def rot_y(pitch):
    c = math.cos(pitch)
    s = math.sin(pitch)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def rot_z(yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def rpy_matrix(roll, pitch, yaw):
    """Return R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    return mat_mul(rot_z(yaw), mat_mul(rot_y(pitch), rot_x(roll)))


def clamp(x, lo, hi):
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def angle_delta(a, b):
    """Shortest signed angular difference a - b."""
    return wrap_to_pi(a - b)
