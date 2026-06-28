# -*- coding: utf-8 -*-
"""Small ROS/Gazebo bridge used only by optional v3 replay scripts.

The v3 evaluation and sweep pipeline does not require ROS.  This module keeps
Gazebo replay independent from the older ``lily_motion`` package; it only
requires ``rospy`` when replaying to a live ROS/Gazebo environment.
"""
from __future__ import division
import json
import re
import time

from lily_motion_v3.interface_config import GAZEBO_JOINT_TOPICS_IN_JOINT_STATE_ORDER


def split_csv(text):
    if not text:
        return []
    return [p.strip() for p in str(text).split(',') if p.strip()]


class MockPublisher(object):
    def __init__(self):
        self.published = []

    def publish(self, command):
        self.published.append(list(command))


class GazeboCommandPublisher(object):
    def __init__(self, topic_prefix=""):
        import rospy
        from std_msgs.msg import Float64
        self.publishers = []
        prefix = str(topic_prefix or "")
        for topic in GAZEBO_JOINT_TOPICS_IN_JOINT_STATE_ORDER:
            full_topic = prefix + topic
            if not full_topic.startswith('/'):
                full_topic = '/' + full_topic
            self.publishers.append(rospy.Publisher(full_topic, Float64, queue_size=1))
        # Give ROS a short moment to connect publishers before first command.
        try:
            rospy.sleep(0.2)
        except Exception:
            time.sleep(0.2)

    def publish(self, command):
        if len(command) != len(self.publishers):
            raise ValueError("expected %d joint commands, got %d" % (len(self.publishers), len(command)))
        for pub, value in zip(self.publishers, command):
            pub.publish(float(value))


class JetsonJointStatePublisher(object):
    """Optional publisher for downstream Jetson command consumers.

    The exact Jetson interface can vary by deployment.  This lightweight bridge
    publishes the 24-value command as a JSON string on /ui/leg_command so the
    script remains self-contained.  Use GazeboCommandPublisher for Gazebo joint
    controllers.
    """
    def __init__(self, topic="/ui/leg_command"):
        import rospy
        from std_msgs.msg import String
        self.publisher = rospy.Publisher(topic, String, queue_size=1)
        try:
            rospy.sleep(0.2)
        except Exception:
            time.sleep(0.2)

    def publish(self, command):
        self.publisher.publish(json.dumps({"joint_command_rad": list(command)}))


class CombinedCommandPublisher(object):
    def __init__(self, gazebo_publisher=None, jetson_publisher=None):
        self.gazebo_publisher = gazebo_publisher
        self.jetson_publisher = jetson_publisher

    def publish(self, command):
        if self.gazebo_publisher is not None:
            self.gazebo_publisher.publish(command)
        if self.jetson_publisher is not None:
            self.jetson_publisher.publish(command)


class GazeboLinkStateLogger(object):
    """Log selected /gazebo/link_states snapshots as JSONL."""
    def __init__(self, output_path, name_contains=None, name_regex=None, log_all_links=False):
        self.output_path = output_path
        self.name_contains = list(name_contains or [])
        self.name_regex = re.compile(name_regex) if name_regex else None
        self.log_all_links = bool(log_all_links)
        self.record_count = 0
        self.matched_names = set()
        self._last_msg = None
        self._sub = None
        self._fh = None

    def start(self):
        import rospy
        from gazebo_msgs.msg import LinkStates
        self._fh = open(self.output_path, 'w')
        self._sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self._callback, queue_size=1)
        try:
            rospy.sleep(0.2)
        except Exception:
            time.sleep(0.2)

    def _callback(self, msg):
        self._last_msg = msg

    def _match(self, name):
        if self.log_all_links:
            return True
        if self.name_contains and any(s in name for s in self.name_contains):
            return True
        if self.name_regex and self.name_regex.search(name):
            return True
        return False

    def snapshot(self, context=None):
        if self._fh is None or self._last_msg is None:
            return
        msg = self._last_msg
        links = []
        for name, pose, twist in zip(msg.name, msg.pose, msg.twist):
            if not self._match(name):
                continue
            self.matched_names.add(name)
            links.append({
                "name": name,
                "position": [pose.position.x, pose.position.y, pose.position.z],
                "orientation": [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                "linear": [twist.linear.x, twist.linear.y, twist.linear.z],
                "angular": [twist.angular.x, twist.angular.y, twist.angular.z],
            })
        rec = {"context": context or {}, "links": links}
        self._fh.write(json.dumps(rec, sort_keys=True))
        self._fh.write('\n')
        self._fh.flush()
        self.record_count += 1

    def close(self):
        try:
            if self._sub is not None:
                self._sub.unregister()
        except Exception:
            pass
        if self._fh is not None:
            self._fh.close()
            self._fh = None
