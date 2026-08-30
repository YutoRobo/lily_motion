#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Mirror Lily position commands actually sent on CAN into Gazebo.

This bridge is intentionally outside Lily Operator / StateMachine.  It never
opens a CAN transmit API and never subscribes to /cmdForJetson.  Instead it runs
``candump -L`` in receive-only mode, observes position-command frames already
sent by StateMachine, reconstructs the 24-axis command vector, applies the same
Gazebo MCU-equivalent interpolation model used by the existing Gazebo path, and
publishes only to Gazebo joint controller topics.

Observed StateMachine position command format::

    CAN ID   : 0x400 + axis, axis 0..23
    DLC      : 8
    byte 0-3 : 0
    byte 4-7 : float32 little-endian target position [rad]

Because one logical 24-axis target is emitted as a short burst of individual CAN
frames, the bridge waits for a short quiet interval after the last position
frame before committing one Gazebo target.  Axes not present in a burst retain
their last command.  Initial logical HOME is zero for all axes.

Python 2.7 / ROS Melodic compatible.
"""
from __future__ import division, print_function

import argparse
import math
import os
import re
import struct
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.gazebo_actuator_interpolator import (
    OnlineLinearActuatorInterpolator,
)
from lily_motion_v3.ros_bridge import GazeboCommandPublisher


POSITION_LENGTH = 24
POSITION_CAN_BASE = 0x400
POSITION_CAN_LAST = POSITION_CAN_BASE + POSITION_LENGTH - 1

CANDUMP_RE = re.compile(
    r'^\s*\((?P<timestamp>[0-9]+(?:\.[0-9]+)?)\)\s+'
    r'(?P<interface>\S+)\s+'
    r'(?P<canid>[0-9A-Fa-f]+)\s+'
    r'\[(?P<dlc>\d+)\]\s+'
    r'(?P<data>(?:[0-9A-Fa-f]{2}(?:\s+|$))+)$'
)

CANDUMP_HASH_RE = re.compile(
    r'^\s*\((?P<timestamp>[0-9]+(?:\.[0-9]+)?)\)\s+'
    r'(?P<interface>\S+)\s+'
    r'(?P<canid>[0-9A-Fa-f]+)#(?P<data>[0-9A-Fa-f]*)\s*$'
)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Receive-only CAN position-command mirror to Gazebo')
    parser.add_argument('--can-channel', default='can0',
                        help='SocketCAN channel observed by candump (default: can0)')
    parser.add_argument('--interp-duration-sec', type=float, default=0.100,
                        help='Gazebo MCU-equivalent interpolation duration (default: 0.100)')
    parser.add_argument('--update-period-sec', type=float, default=0.002,
                        help='Gazebo controller output period (default: 0.002)')
    parser.add_argument('--coalesce-sec', type=float, default=0.002,
                        help='quiet interval used to group one CAN command burst (default: 0.002)')
    parser.add_argument('--gazebo-topic-prefix', default='',
                        help='optional prefix passed to GazeboCommandPublisher')
    return parser.parse_args(argv)


def _decode_float_le(data4):
    if len(data4) != 4:
        raise ValueError('float payload must contain 4 bytes')
    return struct.unpack('<f', struct.pack('4B', *[int(x) for x in data4]))[0]


def parse_candump_line(line):
    """Return (axis, position_rad) for Lily position-command frames, else None."""
    line = line.rstrip('\r\n')
    match = CANDUMP_RE.match(line)
    if match:
        data = [int(x, 16) for x in match.group('data').split()]
        can_id = int(match.group('canid'), 16)
    else:
        match = CANDUMP_HASH_RE.match(line)
        if not match:
            return None
        hx = match.group('data')
        if len(hx) % 2:
            return None
        data = [int(hx[i:i + 2], 16) for i in range(0, len(hx), 2)]
        can_id = int(match.group('canid'), 16)

    if can_id < POSITION_CAN_BASE or can_id > POSITION_CAN_LAST:
        return None
    if len(data) != 8:
        return None
    if any(int(x) != 0 for x in data[:4]):
        return None

    axis = can_id - POSITION_CAN_BASE
    position = _decode_float_le(data[4:8])
    if math.isnan(position) or math.isinf(position):
        return None
    return axis, float(position)


class PositionBurstAccumulator(object):
    """Coalesce per-axis CAN frames into logical 24-axis target snapshots."""

    def __init__(self, quiet_sec=0.002, initial_position=None):
        self.quiet_sec = float(quiet_sec)
        if self.quiet_sec < 0.0:
            raise ValueError('quiet_sec must be >= 0')
        if initial_position is None:
            initial_position = [0.0] * POSITION_LENGTH
        if len(initial_position) != POSITION_LENGTH:
            raise ValueError('initial_position must contain 24 values')
        self.latest = [float(v) for v in initial_position]
        self.pending = False
        self.last_rx_wall = None
        self.seen_axes = set()
        self.frame_count = 0
        self.target_count = 0

    def accept(self, axis, position_rad, wall_time_sec):
        axis = int(axis)
        if axis < 0 or axis >= POSITION_LENGTH:
            raise ValueError('axis out of range: %d' % axis)
        value = float(position_rad)
        if math.isnan(value) or math.isinf(value):
            raise ValueError('position must be finite')
        self.latest[axis] = value
        self.seen_axes.add(axis)
        self.frame_count += 1
        self.pending = True
        self.last_rx_wall = float(wall_time_sec)

    def take_if_quiet(self, wall_time_sec):
        if not self.pending or self.last_rx_wall is None:
            return None
        if float(wall_time_sec) - self.last_rx_wall < self.quiet_sec:
            return None
        self.pending = False
        self.target_count += 1
        return list(self.latest)


class CandumpPositionReader(threading.Thread):
    """Receive-only candump reader.  This class has no CAN transmit API."""

    def __init__(self, interface, callback):
        threading.Thread.__init__(self)
        self.daemon = True
        self.interface = str(interface)
        self.callback = callback
        self.stop_event = threading.Event()
        self.proc = None
        self.error = None

    def run(self):
        # 0x400 with mask 0x7E0 observes 0x400..0x41F; the parser narrows this
        # to the 24 valid Lily axes 0x400..0x417.
        cmd = ['candump', '-L', '%s,400:7E0' % self.interface]
        try:
            self.proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                bufsize=1)
        except Exception as exc:
            self.error = 'failed to start candump: %s' % exc
            return

        while not self.stop_event.is_set():
            line = self.proc.stdout.readline()
            if not line:
                if self.proc.poll() is not None:
                    err = ''
                    try:
                        err = self.proc.stderr.read().strip()
                    except Exception:
                        pass
                    self.error = 'candump exited: %s' % err
                    break
                time.sleep(0.01)
                continue
            parsed = parse_candump_line(line)
            if parsed is None:
                continue
            axis, position_rad = parsed
            self.callback(axis, position_rad, time.time())

    def terminate(self):
        self.stop_event.set()
        if self.proc is not None and self.proc.poll() is None:
            try:
                self.proc.terminate()
            except Exception:
                pass


class CanGazeboSyncBridge(object):
    def __init__(self, rospy, can_channel, interpolation_duration_sec,
                 update_period_sec, coalesce_sec, gazebo_topic_prefix=''):
        self.rospy = rospy
        self.lock = threading.Lock()
        self.accumulator = PositionBurstAccumulator(quiet_sec=coalesce_sec)
        self.interpolator = OnlineLinearActuatorInterpolator(
            interpolation_duration_sec=interpolation_duration_sec,
            expected_length=POSITION_LENGTH)
        self.gazebo_publisher = GazeboCommandPublisher(
            topic_prefix=gazebo_topic_prefix)
        self.reader = CandumpPositionReader(can_channel, self._on_can_position)
        self.update_period_sec = float(update_period_sec)
        if self.update_period_sec <= 0.0:
            raise ValueError('update_period_sec must be positive')
        self.last_reader_error_logged = False

        self.reader.start()
        self.timer = rospy.Timer(
            rospy.Duration(self.update_period_sec), self._update_callback)
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            'CAN->Gazebo sync bridge ready: can=%s interp=%.6f update=%.6f coalesce=%.6f; '
            'CAN input is candump receive-only and /cmdForJetson is untouched',
            can_channel,
            float(interpolation_duration_sec),
            self.update_period_sec,
            float(coalesce_sec))

    def _ros_time_sec(self):
        return self.rospy.Time.now().to_sec()

    def _on_can_position(self, axis, position_rad, wall_time_sec):
        with self.lock:
            self.accumulator.accept(axis, position_rad, wall_time_sec)

    def _update_callback(self, _event):
        if self.reader.error and not self.last_reader_error_logged:
            self.last_reader_error_logged = True
            self.rospy.logerr('CAN->Gazebo sync reader failed: %s', self.reader.error)

        now_wall = time.time()
        now_ros = self._ros_time_sec()
        with self.lock:
            target = self.accumulator.take_if_quiet(now_wall)
            if target is not None:
                self.interpolator.set_target(target, now_ros)
                target_count = self.accumulator.target_count
                seen_count = len(self.accumulator.seen_axes)
            else:
                target_count = None
                seen_count = None
            command = self.interpolator.command_at(now_ros)

        if target_count is not None:
            if target_count == 1:
                self.rospy.loginfo(
                    'CAN->Gazebo sync accepted first CAN target; observed_axes=%d/24',
                    seen_count)
            elif target_count % 100 == 0:
                self.rospy.loginfo(
                    'CAN->Gazebo sync targets=%d CAN_frames=%d observed_axes=%d/24',
                    target_count, self.accumulator.frame_count, seen_count)

        if command is None:
            return
        try:
            self.gazebo_publisher.publish(command)
        except Exception as exc:
            self.rospy.logerr_throttle(
                1.0, 'CAN->Gazebo sync publish failed: %s' % exc)

    def shutdown(self):
        try:
            self.reader.terminate()
        except Exception:
            pass


def main(argv=None):
    import rospy

    ros_argv = rospy.myargv(argv=sys.argv if argv is None else argv)
    args = parse_args(ros_argv[1:] if argv is None else ros_argv)
    if args.interp_duration_sec < 0.0:
        raise SystemExit('--interp-duration-sec must be >= 0')
    if args.update_period_sec <= 0.0:
        raise SystemExit('--update-period-sec must be positive')
    if args.coalesce_sec < 0.0:
        raise SystemExit('--coalesce-sec must be >= 0')

    rospy.init_node('lily_can_gazebo_sync_bridge', anonymous=False)
    CanGazeboSyncBridge(
        rospy,
        can_channel=args.can_channel,
        interpolation_duration_sec=args.interp_duration_sec,
        update_period_sec=args.update_period_sec,
        coalesce_sec=args.coalesce_sec,
        gazebo_topic_prefix=args.gazebo_topic_prefix)
    rospy.spin()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
