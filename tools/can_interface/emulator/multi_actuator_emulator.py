#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Run multiple actuator MCU emulators on one SocketCAN vcan channel."""
from __future__ import print_function

import argparse
import os
import sys
import time

try:
    from .scenario import (
        ActuatorScenario, parse_axis_spec, parse_axis_value_specs)
    from .virtual_actuator import VirtualActuator
except (ImportError, ValueError):
    EMULATOR_DIR = os.path.dirname(os.path.abspath(__file__))
    if EMULATOR_DIR not in sys.path:
        sys.path.insert(0, EMULATOR_DIR)
    from scenario import ActuatorScenario, parse_axis_spec, parse_axis_value_specs
    from virtual_actuator import VirtualActuator


class MultiActuatorEmulator(object):
    def __init__(self, axes, transmit, scenarios=None, heartbeat_period=1.0,
                 align_delay=0.25, reset_delay=0.5, now=None, logger=None):
        axes = list(axes)
        if not axes or len(set(axes)) != len(axes):
            raise ValueError("axes must be non-empty and unique")
        self.transmit = transmit
        self.logger = logger
        self.actuators = {}
        scenarios = scenarios or {}
        for axis in axes:
            self.actuators[axis] = VirtualActuator(
                axis, transmit, scenario=scenarios.get(axis),
                heartbeat_period=heartbeat_period,
                align_delay=align_delay, reset_delay=reset_delay,
                now=now, logger=logger)

    def receive(self, frame, now=None):
        arbitration_id = int(frame.arbitration_id)
        data = list(frame.data)
        handled = False
        for actuator in self.actuators.values():
            handled = actuator.receive(
                arbitration_id, data, now=now) or handled
        return handled

    def tick(self, now=None):
        for actuator in self.actuators.values():
            actuator.tick(now)

    def inject_error(self, axis, error_id, now=None):
        self.actuators[int(axis)].inject_error(error_id, now)

    def summaries(self):
        return [self.actuators[axis].summary()
                for axis in sorted(self.actuators)]


def build_scenarios(axes, fail_once=(), fail_always=(), fail_at=None,
                    injected_errors=None, reset_after_run=None,
                    initialization_error_id=8):
    fail_once = set(fail_once)
    fail_always = set(fail_always)
    fail_at = fail_at or {}
    injected_errors = injected_errors or {}
    reset_after_run = reset_after_run or {}
    if fail_once & fail_always:
        raise ValueError("axis cannot be both fail-once and fail-always")
    configured = fail_once | fail_always | set(fail_at) | set(
        injected_errors) | set(reset_after_run)
    unknown = configured - set(axes)
    if unknown:
        raise ValueError("scenario axis not enabled: %s" % sorted(unknown))
    scenarios = {}
    for axis in axes:
        scenarios[axis] = ActuatorScenario(
            fail_attempts=(set((1,)) if axis in fail_once else set())
            | set(fail_at.get(axis, ())),
            fail_always=axis in fail_always,
            initialization_error_id=initialization_error_id,
            reset_after_run_sec=reset_after_run.get(axis))
    return scenarios


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Multiple Lily actuator MCU emulator for SocketCAN vcan")
    parser.add_argument("--interface", default="vcan0",
                        help="SocketCAN channel; can0 is prohibited")
    parser.add_argument("--axes", required=True)
    parser.add_argument("--align-fail-once", action="append", default=[])
    parser.add_argument("--align-fail-always", action="append", default=[])
    parser.add_argument(
        "--align-fail-at", action="append", default=[],
        help="AXIS:ATTEMPT; repeat for multiple attempts")
    parser.add_argument("--inject-error", action="append", default=[])
    parser.add_argument("--reset-after-run", action="append", default=[])
    parser.add_argument(
        "--initialization-error-id", type=int, choices=(8, 9, 12), default=8)
    parser.add_argument("--heartbeat-period", type=float, default=1.0)
    parser.add_argument("--align-delay", type=float, default=0.25)
    parser.add_argument("--reset-delay", type=float, default=0.5)
    return parser.parse_args(argv)


def _combined_axis_specs(values):
    result = []
    seen = set()
    for value in values or ():
        for axis in parse_axis_spec(value):
            if axis in seen:
                raise ValueError("duplicate axis across options: %d" % axis)
            seen.add(axis)
            result.append(axis)
    return result


def parse_fail_at_specs(values):
    result = {}
    for item in values or ():
        parts = str(item).split(":")
        if len(parts) != 2:
            raise ValueError("invalid align-fail-at: %s" % item)
        axes = parse_axis_spec(parts[0])
        try:
            attempt = int(parts[1])
        except ValueError:
            raise ValueError("invalid alignment attempt: %s" % parts[1])
        if attempt <= 0:
            raise ValueError("alignment attempt must be positive")
        for axis in axes:
            result.setdefault(axis, set())
            if attempt in result[axis]:
                raise ValueError("duplicate align-fail-at: %s" % item)
            result[axis].add(attempt)
    return result


def validate_channel(channel):
    value = str(channel).strip()
    if not value:
        raise ValueError("SocketCAN channel is empty")
    if value == "can0":
        raise ValueError("can0 is prohibited for the emulator")
    if not value.startswith("vcan"):
        raise ValueError("emulator channel must be a vcan interface")
    return value


def main(argv=None):
    args = parse_args(argv)
    try:
        channel = validate_channel(args.interface)
        axes = parse_axis_spec(args.axes)
        fail_once = _combined_axis_specs(args.align_fail_once)
        fail_always = _combined_axis_specs(args.align_fail_always)
        fail_at = parse_fail_at_specs(args.align_fail_at)
        injected_errors = parse_axis_value_specs(
            args.inject_error, "inject-error", int)
        reset_after_run = parse_axis_value_specs(
            args.reset_after_run, "reset-after-run", float)
        scenarios = build_scenarios(
            axes, fail_once, fail_always, fail_at, injected_errors,
            reset_after_run, args.initialization_error_id)
    except ValueError as exc:
        raise SystemExit(str(exc))

    import can
    bus = can.interface.Bus(interface="socketcan", channel=channel)

    def log(line):
        print(line)

    def transmit(arbitration_id, data):
        bus.send(can.Message(
            arbitration_id=arbitration_id, data=data,
            is_extended_id=False))

    emulator = MultiActuatorEmulator(
        axes, transmit, scenarios=scenarios,
        heartbeat_period=args.heartbeat_period,
        align_delay=args.align_delay, reset_delay=args.reset_delay,
        now=time.time(), logger=log)
    now = time.time()
    for axis, error_id in injected_errors.items():
        emulator.inject_error(axis, error_id, now)

    print("emulator ready channel=%s axes=%s" % (channel, axes))
    try:
        while True:
            frame = bus.recv(timeout=0.02)
            now = time.time()
            if frame is not None:
                emulator.receive(frame, now)
            emulator.tick(now)
    except KeyboardInterrupt:
        pass
    finally:
        for line in emulator.summaries():
            print(line)
        bus.shutdown()


if __name__ == "__main__":
    main()
