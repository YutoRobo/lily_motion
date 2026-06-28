# -*- coding: utf-8 -*-
"""Small geometry helpers for project-contained v3 evaluation."""
from __future__ import division
import math


def dot(a, b):
    return sum(float(a[i]) * float(b[i]) for i in range(3))


def sub(a, b):
    return [float(a[i]) - float(b[i]) for i in range(3)]


def add(a, b):
    return [float(a[i]) + float(b[i]) for i in range(3)]


def scale(a, s):
    return [float(a[i]) * float(s) for i in range(3)]


def norm(a):
    return math.sqrt(dot(a, a))


def distance(a, b):
    return norm(sub(a, b))


def clamp01(x):
    return max(0.0, min(1.0, float(x)))


def point_segment_distance(p, a, b):
    ab = sub(b, a)
    denom = dot(ab, ab)
    if denom <= 1e-18:
        return distance(p, a), list(a)
    t = clamp01(dot(sub(p, a), ab) / denom)
    q = add(a, scale(ab, t))
    return distance(p, q), q


def segment_segment_distance(p1, q1, p2, q2):
    """Return distance and closest points between two 3D segments.

    Implementation follows the standard closest-segment calculation and keeps
    all dependencies local so the v3 package remains portable.
    """
    u = sub(q1, p1)
    v = sub(q2, p2)
    w = sub(p1, p2)
    a = dot(u, u)
    b = dot(u, v)
    c = dot(v, v)
    d = dot(u, w)
    e = dot(v, w)
    D = a * c - b * b
    SMALL = 1e-12

    sN = 0.0
    sD = D
    tN = 0.0
    tD = D

    if D < SMALL:
        sN = 0.0
        sD = 1.0
        tN = e
        tD = c
    else:
        sN = b * e - c * d
        tN = a * e - b * d
        if sN < 0.0:
            sN = 0.0
            tN = e
            tD = c
        elif sN > sD:
            sN = sD
            tN = e + b
            tD = c

    if tN < 0.0:
        tN = 0.0
        if -d < 0.0:
            sN = 0.0
        elif -d > a:
            sN = sD
        else:
            sN = -d
            sD = a
    elif tN > tD:
        tN = tD
        if (-d + b) < 0.0:
            sN = 0.0
        elif (-d + b) > a:
            sN = sD
        else:
            sN = -d + b
            sD = a

    sc = 0.0 if abs(sN) < SMALL else sN / sD
    tc = 0.0 if abs(tN) < SMALL else tN / tD
    cp1 = add(p1, scale(u, sc))
    cp2 = add(p2, scale(v, tc))
    return distance(cp1, cp2), cp1, cp2
