# -*- coding: utf-8 -*-
"""Small reusable constraints for v3 kinematic candidates."""
from __future__ import division
import math


class ConstraintResult(object):
    def __init__(self, ok, name, value=None, limit=None, message=""):
        self.ok = bool(ok)
        self.name = str(name)
        self.value = value
        self.limit = limit
        self.message = str(message)

    def to_dict(self):
        return {"ok": self.ok, "name": self.name, "value": self.value,
                "limit": self.limit, "message": self.message}


class JointLimitConstraint(object):
    def __init__(self, second_joint_abs_max_deg=95.0):
        self.second_joint_abs_max_deg = float(second_joint_abs_max_deg)

    def evaluate(self, q):
        val = abs(math.degrees(float(q[1])))
        ok = val <= self.second_joint_abs_max_deg + 1e-9
        return ConstraintResult(ok, "second_joint_abs", val,
                                self.second_joint_abs_max_deg,
                                "" if ok else "second joint exceeds limit")
