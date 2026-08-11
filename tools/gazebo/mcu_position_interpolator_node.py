#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Gazebo-only MCU position interpolation node.

Input is exactly the shared hardware command boundary::

    /cmdForJetson  (sensor_msgs/JointState, 24 positions)

The node applies the configurable MCU-equivalent target interpolation and sends
only the interpolated result to Gazebo joint controller topics.  It does not
load command files, resample trajectories, open CAN, or know about staged files.

The first received target is treated as the already-held starting pose.  After
input stops, the final target remains held until this node is stopped.
"""
from __future__ import division, print_function

import argparse
import os
import sys
import threading

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from lily_motion_v3.gazebo_actuator_interpolator import (
    OnlineLinearActuatorInterpolator,
)
from lily_motion_v3.ros_bridge import GazeboCommandPublisher


POSITION_LENGTH = 24


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Subscribe to /cmdForJetson, emulate MCU position interpolation, and publish to Gazebo joint topics.')
    parser.add_argument('--input-topic', default='/cmdForJetson')
    parser.add_argument('--interp-duration-sec', type=float, default=0.100,
                        help='MCU-equivalent interpolation duration after each new target')
    parser.add_argument('--update-period-sec', type=float, default=0.002,
                        help='Gazebo interpolation output update period')
    parser.add_argument('--gazebo-topic-prefix', default='',
                        help='Optional prefix passed to GazeboCommandPublisher')
    return parser.parse_args(argv)


class GazeboMcuInterpolationNode(object):
    def __init__(self, rospy, input_topic, interpolation_duration_sec,
                 update_period_sec, gazebo_topic_prefix=''):
        from sensor_msgs.msg import JointState

        self.rospy = rospy
        self.input_topic = str(input_topic)
        self.update_period_sec = float(update_period_sec)
        if self.update_period_sec <= 0.0:
            raise ValueError('update_period_sec must be positive')
        self.interpolator = OnlineLinearActuatorInterpolator(
            interpolation_duration_sec=interpolation_duration_sec,
            expected_length=POSITION_LENGTH)
        self.lock = threading.Lock()
        self.gazebo_publisher = GazeboCommandPublisher(
            topic_prefix=gazebo_topic_prefix)
        self.accepted_target_count = 0
        self.rejected_target_count = 0

        self.subscriber = rospy.Subscriber(
            self.input_topic,
            JointState,
            self._target_callback,
            queue_size=10)
        self.timer = rospy.Timer(
            rospy.Duration(self.update_period_sec),
            self._update_callback)

        rospy.loginfo(
            'Gazebo MCU interpolator ready: input=%s interp_duration=%.6f update_period=%.6f',
            self.input_topic,
            float(interpolation_duration_sec),
            self.update_period_sec)

    def _ros_time_sec(self):
        return self.rospy.Time.now().to_sec()

    def _target_callback(self, msg):
        now = self._ros_time_sec()
        try:
            with self.lock:
                self.interpolator.set_target(msg.position, now)
                self.accepted_target_count += 1
                count = self.accepted_target_count
        except Exception as exc:
            self.rejected_target_count += 1
            self.rospy.logerr(
                'Gazebo MCU interpolator rejected /cmdForJetson target: %s',
                exc)
            return

        if count == 1:
            self.rospy.loginfo(
                'Gazebo MCU interpolator accepted first target; treating it as held starting pose')

    def _update_callback(self, _event):
        now = self._ros_time_sec()
        with self.lock:
            command = self.interpolator.command_at(now)
        if command is None:
            return
        try:
            self.gazebo_publisher.publish(command)
        except Exception as exc:
            self.rospy.logerr_throttle(
                1.0, 'Gazebo MCU interpolator publish failed: %s' % exc)


def main(argv=None):
    import rospy

    ros_argv = rospy.myargv(argv=sys.argv if argv is None else argv)
    args = parse_args(ros_argv[1:] if argv is None else ros_argv)
    if args.interp_duration_sec < 0.0:
        raise SystemExit('--interp-duration-sec must be >= 0')
    if args.update_period_sec <= 0.0:
        raise SystemExit('--update-period-sec must be positive')

    rospy.init_node('lily_gazebo_mcu_position_interpolator', anonymous=False)
    GazeboMcuInterpolationNode(
        rospy,
        input_topic=args.input_topic,
        interpolation_duration_sec=args.interp_duration_sec,
        update_period_sec=args.update_period_sec,
        gazebo_topic_prefix=args.gazebo_topic_prefix)
    rospy.spin()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
