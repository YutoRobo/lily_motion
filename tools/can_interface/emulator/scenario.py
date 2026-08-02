# -*- coding: utf-8 -*-
"""CLI parsing and immutable scenario configuration."""
from __future__ import print_function


def parse_axis_spec(text):
    if text is None or not str(text).strip():
        raise ValueError("axis specification is empty")
    result = []
    seen = set()
    for token in str(text).split(","):
        token = token.strip()
        if not token:
            raise ValueError("empty axis token")
        if "-" in token:
            if token.count("-") != 1:
                raise ValueError("invalid axis range: %s" % token)
            start_text, end_text = token.split("-")
            try:
                start = int(start_text)
                end = int(end_text)
            except ValueError:
                raise ValueError("invalid axis range: %s" % token)
            if start > end:
                raise ValueError("reversed axis range: %s" % token)
            values = range(start, end + 1)
        else:
            try:
                values = (int(token),)
            except ValueError:
                raise ValueError("invalid axis: %s" % token)
        for axis in values:
            if not 0 <= axis <= 23:
                raise ValueError("axis out of range: %d" % axis)
            if axis in seen:
                raise ValueError("duplicate axis: %d" % axis)
            seen.add(axis)
            result.append(axis)
    if not result:
        raise ValueError("axis specification is empty")
    return result


def parse_axis_value_specs(values, value_name, value_parser=float):
    result = {}
    for item in values or ():
        parts = str(item).split(":")
        if len(parts) != 2:
            raise ValueError("invalid %s specification: %s" % (
                value_name, item))
        axes = parse_axis_spec(parts[0])
        try:
            value = value_parser(parts[1])
        except (TypeError, ValueError):
            raise ValueError("invalid %s value: %s" % (value_name, parts[1]))
        for axis in axes:
            if axis in result:
                raise ValueError("duplicate %s axis: %d" % (value_name, axis))
            result[axis] = value
    return result


class ActuatorScenario(object):
    def __init__(self, fail_attempts=None, fail_always=False,
                 initialization_error_id=8, reset_after_run_sec=None):
        self.fail_attempts = frozenset(fail_attempts or ())
        self.fail_always = bool(fail_always)
        self.initialization_error_id = int(initialization_error_id)
        self.reset_after_run_sec = (
            None if reset_after_run_sec is None
            else float(reset_after_run_sec))
        if self.initialization_error_id not in (8, 9, 12):
            raise ValueError("initialization error ID must be 8, 9, or 12")
        if (self.reset_after_run_sec is not None
                and self.reset_after_run_sec < 0.0):
            raise ValueError("reset_after_run_sec must be non-negative")

    def alignment_should_fail(self, attempt):
        return self.fail_always or int(attempt) in self.fail_attempts
