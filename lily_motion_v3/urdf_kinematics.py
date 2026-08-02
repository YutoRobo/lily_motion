# -*- coding: utf-8 -*-
"""URDF joint-chain forward kinematics for offline Lily diagnostics.

Transform order follows URDF: parent transform, joint origin xyz/rpy, then the
joint rotation about the normalized axis. Angles are radians.
"""
from __future__ import division

import math
import os
import xml.etree.ElementTree as ET

from lily_motion_v3.geometry import norm, sub
from lily_motion_v3.interface_config import (
    JOINT_STATE_ORDER,
    LEG_NAMES_BY_ID,
    NUM_JOINTS,
)
from lily_motion_v3.transforms import (
    mat_mul,
    mat_vec_mul,
    rpy_matrix,
    vec_add,
)


SUPPORTED_JOINT_TYPES = frozenset(("fixed", "revolute", "continuous"))
ACTUATED_JOINT_SUFFIXES = (
    "base_clause_joint",
    "thigh_joint",
    "tibia_joint",
)


def _identity_matrix():
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def _parse_vector(text, default, field_name, joint_name):
    if text is None:
        return list(default)
    try:
        values = [float(value) for value in text.split()]
    except (TypeError, ValueError):
        raise ValueError(
            "joint %s has invalid %s: %r" % (
                joint_name, field_name, text))
    if len(values) != 3:
        raise ValueError(
            "joint %s %s must contain exactly 3 numbers" % (
                joint_name, field_name))
    return values


def axis_rotation_matrix(axis, angle_rad):
    """Return a Rodrigues rotation matrix for a non-zero URDF axis."""
    axis = [float(value) for value in axis]
    magnitude = norm(axis)
    if magnitude <= 1e-15:
        raise ValueError("joint axis must not be a zero vector")
    x, y, z = [value / magnitude for value in axis]
    c = math.cos(float(angle_rad))
    s = math.sin(float(angle_rad))
    one_minus_c = 1.0 - c
    return [
        [c + x * x * one_minus_c,
         x * y * one_minus_c - z * s,
         x * z * one_minus_c + y * s],
        [y * x * one_minus_c + z * s,
         c + y * y * one_minus_c,
         y * z * one_minus_c - x * s],
        [z * x * one_minus_c - y * s,
         z * y * one_minus_c + x * s,
         c + z * z * one_minus_c],
    ]


class Transform(object):
    """Rigid transform represented as p_parent = R*p_local + translation."""

    def __init__(self, rotation=None, translation=None):
        self.rotation = rotation or _identity_matrix()
        self.translation = translation or [0.0, 0.0, 0.0]

    def then(self, child):
        return Transform(
            mat_mul(self.rotation, child.rotation),
            vec_add(
                self.translation,
                mat_vec_mul(self.rotation, child.translation)),
        )

    def point(self, local_point):
        return vec_add(
            self.translation,
            mat_vec_mul(self.rotation, local_point))

    def vector(self, local_vector):
        return mat_vec_mul(self.rotation, local_vector)


class UrdfJoint(object):
    def __init__(self, element):
        self.name = (element.get("name") or "").strip()
        if not self.name:
            raise ValueError("URDF joint is missing a name")
        self.joint_type = (element.get("type") or "").strip()
        if self.joint_type not in SUPPORTED_JOINT_TYPES:
            raise ValueError(
                "joint %s has unsupported type %r" % (
                    self.name, self.joint_type))

        parent_element = element.find("parent")
        child_element = element.find("child")
        self.parent_link = (
            parent_element.get("link", "").strip()
            if parent_element is not None else "")
        self.child_link = (
            child_element.get("link", "").strip()
            if child_element is not None else "")
        if not self.parent_link:
            raise ValueError(
                "joint %s is missing parent link" % self.name)
        if not self.child_link:
            raise ValueError(
                "joint %s is missing child link" % self.name)

        origins = element.findall("origin")
        if len(origins) > 1:
            raise ValueError(
                "joint %s has multiple origin elements" % self.name)
        origin_element = origins[0] if origins else None
        self.origin_xyz = _parse_vector(
            origin_element.get("xyz") if origin_element is not None else None,
            [0.0, 0.0, 0.0], "origin xyz", self.name)
        self.origin_rpy = _parse_vector(
            origin_element.get("rpy") if origin_element is not None else None,
            [0.0, 0.0, 0.0], "origin rpy", self.name)
        self.origin_transform = Transform(
            rpy_matrix(
                self.origin_rpy[0],
                self.origin_rpy[1],
                self.origin_rpy[2]),
            list(self.origin_xyz),
        )

        axis_element = element.find("axis")
        if self.joint_type in ("revolute", "continuous"):
            self.axis = _parse_vector(
                axis_element.get("xyz") if axis_element is not None else None,
                [1.0, 0.0, 0.0], "axis", self.name)
            if norm(self.axis) <= 1e-15:
                raise ValueError(
                    "joint %s has a zero axis" % self.name)
        else:
            self.axis = [0.0, 0.0, 0.0]

    def motion_transform(self, angle_rad):
        if self.joint_type == "fixed":
            return Transform()
        return Transform(axis_rotation_matrix(self.axis, angle_rad),
                         [0.0, 0.0, 0.0])


class UrdfLegKinematics(object):
    def __init__(self, joints_by_name, links):
        self.joints_by_name = dict(joints_by_name)
        self.links = frozenset(links)
        self.leg_names = tuple(
            LEG_NAMES_BY_ID[leg_id]
            for leg_id in sorted(LEG_NAMES_BY_ID)
            if self._has_leg(LEG_NAMES_BY_ID[leg_id]))
        if not self.leg_names:
            raise ValueError("no Lily leg joint chains were found")

    @classmethod
    def from_robot_description(cls, path):
        if not path:
            raise ValueError("robot description path is required")
        if not os.path.isfile(path):
            raise ValueError(
                "robot description file does not exist: %s" % path)
        try:
            with open(path, "r") as stream:
                xml_text = stream.read()
        except IOError as exc:
            raise ValueError(
                "cannot read robot description %s: %s" % (path, exc))
        try:
            root = ET.fromstring(xml_text)
        except ET.ParseError as exc:
            raise ValueError(
                "invalid robot description XML %s: %s" % (path, exc))
        if root.tag != "robot":
            raise ValueError(
                "robot description root element must be <robot>")

        links = set()
        for element in root.findall("link"):
            name = (element.get("name") or "").strip()
            if not name:
                raise ValueError("URDF link is missing a name")
            if name in links:
                raise ValueError("duplicate URDF link: %s" % name)
            links.add(name)

        joints = {}
        child_links = set()
        for element in root.findall("joint"):
            joint = UrdfJoint(element)
            if joint.name in joints:
                raise ValueError("duplicate URDF joint: %s" % joint.name)
            if joint.parent_link not in links:
                raise ValueError(
                    "joint %s references missing parent link %s" % (
                        joint.name, joint.parent_link))
            if joint.child_link not in links:
                raise ValueError(
                    "joint %s references missing child link %s" % (
                        joint.name, joint.child_link))
            if joint.child_link in child_links:
                raise ValueError(
                    "link has multiple parent joints: %s" %
                    joint.child_link)
            child_links.add(joint.child_link)
            joints[joint.name] = joint
        if not joints:
            raise ValueError("robot description contains no joints")
        return cls(joints, links)

    def _has_leg(self, leg_name):
        prefix = "%s_" % leg_name
        return any(
            name.startswith(prefix)
            for name in self.joints_by_name)

    def joint(self, joint_name):
        try:
            return self.joints_by_name[joint_name]
        except KeyError:
            raise ValueError("joint was not found: %s" % joint_name)

    def joint_chain(self, leg_name):
        """Return one topology-ordered chain for an existing Lily leg."""
        prefix = "%s_" % leg_name
        candidates = [
            joint for joint in self.joints_by_name.values()
            if joint.name.startswith(prefix)]
        if not candidates:
            raise ValueError(
                "joint chain was not found for leg %s" % leg_name)

        child_links = set(joint.child_link for joint in candidates)
        roots = [
            joint for joint in candidates
            if joint.parent_link not in child_links]
        if len(roots) != 1:
            raise ValueError(
                "leg %s must have exactly one chain root" % leg_name)

        candidate_by_parent = {}
        for joint in candidates:
            if joint.parent_link in candidate_by_parent:
                raise ValueError(
                    "leg %s joint chain branches at link %s" % (
                        leg_name, joint.parent_link))
            candidate_by_parent[joint.parent_link] = joint

        chain = []
        current = roots[0]
        visited = set()
        while current is not None:
            if current.name in visited:
                raise ValueError(
                    "cycle in joint chain for leg %s" % leg_name)
            visited.add(current.name)
            chain.append(current)
            current = candidate_by_parent.get(current.child_link)
        if len(chain) != len(candidates):
            raise ValueError(
                "leg %s contains disconnected joints" % leg_name)

        actuated = [
            joint for joint in chain
            if joint.joint_type in ("revolute", "continuous")]
        expected_names = [
            "%s_%s" % (leg_name, suffix)
            for suffix in ACTUATED_JOINT_SUFFIXES]
        if [joint.name for joint in actuated] != expected_names:
            raise ValueError(
                "leg %s actuated chain must be %s, got %s" % (
                    leg_name,
                    expected_names,
                    [joint.name for joint in actuated]))
        return chain

    def leg_joint_summary(self, leg_name):
        out = []
        for order, joint in enumerate(self.joint_chain(leg_name)):
            out.append({
                "order": order,
                "joint_name": joint.name,
                "joint_type": joint.joint_type,
                "parent_link": joint.parent_link,
                "child_link": joint.child_link,
                "origin_xyz": list(joint.origin_xyz),
                "origin_rpy": list(joint.origin_rpy),
                "axis": list(joint.axis),
            })
        return out

    def command_values_by_leg(self, values):
        if values is None or len(values) != NUM_JOINTS:
            actual = 0 if values is None else len(values)
            raise ValueError(
                "joint command must contain %d values, got %d" % (
                    NUM_JOINTS, actual))
        q_by_leg = dict(
            (name, [0.0, 0.0, 0.0])
            for name in LEG_NAMES_BY_ID.values())
        for command_index, mapping in enumerate(JOINT_STATE_ORDER):
            leg_id, joint_index = mapping
            leg_name = LEG_NAMES_BY_ID[int(leg_id)]
            q_by_leg[leg_name][int(joint_index)] = float(
                values[command_index])
        return q_by_leg

    def command_q_by_leg(self, record):
        values = record.get("joint_command_rad")
        if values is None:
            degrees = record.get("joint_command_deg")
            if degrees is None:
                raise ValueError(
                    "command record requires joint_command_rad or "
                    "joint_command_deg")
            if len(degrees) != NUM_JOINTS:
                raise ValueError(
                    "joint_command_deg must contain %d values, got %d" % (
                        NUM_JOINTS, len(degrees)))
            values = [
                math.radians(float(value))
                for value in degrees]
        return self.command_values_by_leg(values)

    def link_positions_body(self, leg_name, q):
        """Return joint origins, link endpoints and foot in body coordinates."""
        if q is None or len(q) != 3:
            raise ValueError(
                "leg %s requires exactly 3 joint angles" % leg_name)
        q = [float(value) for value in q]
        chain = self.joint_chain(leg_name)
        parent_transform = Transform()
        actuated_index = 0
        joint_records = []
        link_endpoints = []
        for order, joint in enumerate(chain):
            joint_transform = parent_transform.then(
                joint.origin_transform)
            axis_body = (
                joint_transform.vector(joint.axis)
                if joint.joint_type != "fixed" else [0.0, 0.0, 0.0])
            joint_records.append({
                "order": order,
                "joint_name": joint.name,
                "joint_type": joint.joint_type,
                "parent_link": joint.parent_link,
                "child_link": joint.child_link,
                "position": list(joint_transform.translation),
                "axis_body": axis_body,
            })
            angle = 0.0
            if joint.joint_type in ("revolute", "continuous"):
                angle = q[actuated_index]
                actuated_index += 1
            child_transform = joint_transform.then(
                joint.motion_transform(angle))
            link_endpoints.append({
                "link_name": joint.child_link,
                "position": list(child_transform.translation),
            })
            parent_transform = child_transform

        actuated_records = [
            record for record in joint_records
            if record["joint_type"] in ("revolute", "continuous")]
        fixed_after_actuated = [
            record for record in joint_records
            if (record["joint_type"] == "fixed"
                and record["order"] > actuated_records[-1]["order"])]
        if not fixed_after_actuated:
            raise ValueError(
                "leg %s chain requires a fixed foot endpoint" % leg_name)

        root = list(actuated_records[0]["position"])
        coxa_end = list(actuated_records[1]["position"])
        knee = list(actuated_records[2]["position"])
        foot = list(fixed_after_actuated[-1]["position"])
        return {
            "body_root": [0.0, 0.0, 0.0],
            "root": root,
            "coxa_end": coxa_end,
            "knee": knee,
            "foot": foot,
            "joint_positions": joint_records,
            "link_endpoints": link_endpoints,
        }

    def leg_segments_body(self, leg_name, q):
        points = self.link_positions_body(leg_name, q)
        return self.segments_from_positions(leg_name, points)

    @staticmethod
    def segments_from_positions(leg_name, points):
        return [
            {
                "leg_name": leg_name,
                "segment_name": "root_to_coxa_end",
                "a": list(points["root"]),
                "b": list(points["coxa_end"]),
            },
            {
                "leg_name": leg_name,
                "segment_name": "coxa_end_to_knee",
                "a": list(points["coxa_end"]),
                "b": list(points["knee"]),
            },
            {
                "leg_name": leg_name,
                "segment_name": "knee_to_foot",
                "a": list(points["knee"]),
                "b": list(points["foot"]),
            },
        ]

    def link_positions_world_from_record(self, record):
        q_by_leg = self.command_q_by_leg(record)
        base_pose = record.get("base_pose") or {}
        world_from_body = Transform(
            rpy_matrix(
                float(base_pose.get("roll", 0.0)),
                float(base_pose.get("pitch", 0.0)),
                float(base_pose.get("yaw", 0.0))),
            [
                float(base_pose.get("x", 0.0)),
                float(base_pose.get("y", 0.0)),
                float(base_pose.get("z", 0.0)),
            ])
        result = {}
        for leg_name in self.leg_names:
            body = self.link_positions_body(
                leg_name, q_by_leg[leg_name])
            world = {}
            for key in ("body_root", "root", "coxa_end", "knee", "foot"):
                world[key] = world_from_body.point(body[key])
            world["joint_positions"] = [
                dict(record_item,
                     position=world_from_body.point(
                         record_item["position"]),
                     axis_body=world_from_body.vector(
                         record_item["axis_body"]))
                for record_item in body["joint_positions"]]
            world["link_endpoints"] = [
                dict(record_item,
                     position=world_from_body.point(
                         record_item["position"]))
                for record_item in body["link_endpoints"]]
            result[leg_name] = world
        return result

    def leg_segments_world_from_record(self, record):
        positions = self.link_positions_world_from_record(record)
        segments = []
        for leg_name in self.leg_names:
            segments.extend(
                self.segments_from_positions(
                    leg_name, positions[leg_name]))
        return segments


def vector_almost_equal(a, b, tolerance=1e-9):
    return norm(sub(a, b)) <= float(tolerance)
