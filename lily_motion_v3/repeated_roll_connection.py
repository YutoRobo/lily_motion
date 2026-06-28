# -*- coding: utf-8 -*-
"""Connection diagnostics for repeated legacy quarter-rolls."""
from __future__ import division, print_function
import math
import numpy as np

from lily_motion_v3.legacy_constraint_evaluator import LegacyConstraintEvaluator
from lily_motion_v3.legacy_state_machine_emulator import LEGACY_ID_TO_NAME

# Goal-1 support sets for each starting surface in the supplied legacy forward roll.
START_SUPPORT_BY_SURFACE = {
    1: [1, 3, 0, 2],
    5: [0, 2, 4, 6],
    6: [4, 6, 5, 7],
    2: [5, 7, 1, 3],
}


def _xy(p):
    return np.array([float(p[0]), float(p[1])], dtype=float)


def _posture_xy(rec):
    bp = rec.get('base_pose') or {}
    return np.array([float(bp.get('x', 0.0)), float(bp.get('y', 0.0))], dtype=float)


def connection_report(records, default_body_z=0.35):
    """Return boundary metrics for quarter-roll connection quality.

    The main metric is the XY distance between the body projection and the
    centroid of the next surface's initial support set at each quarter-roll end.
    This is not a dynamic stability proof; it is a simple diagnostic for whether
    the terminal posture is a reasonable start for the next quarter roll.
    """
    if not records:
        return {'boundary_count': 0, 'boundaries': [], 'max_body_to_next_support_center_xy_m': None}
    ev = LegacyConstraintEvaluator(default_body_z=default_body_z)
    # Last record of each roll_index.
    by_roll = {}
    for rec in records:
        if 'roll_index' in rec and 'surface_after' in rec:
            by_roll[int(rec['roll_index'])] = rec
    boundaries = []
    max_err = 0.0
    worst = None
    for roll_index in sorted(by_roll.keys()):
        rec = by_roll[roll_index]
        next_surface = int(rec.get('surface_after'))
        support = START_SUPPORT_BY_SURFACE.get(next_surface, [])
        geom = ev._frame_geometry(rec)
        pts = [_xy(geom[leg_id]['foot_abs']) for leg_id in support if leg_id in geom]
        if pts:
            center = sum(pts) / float(len(pts))
        else:
            center = np.array([0.0, 0.0])
        body = _posture_xy(rec)
        err = float(np.linalg.norm(body - center))
        item = {
            'roll_index': roll_index,
            'frame_index': rec.get('frame_index'),
            'transition': rec.get('roll_surface_transition'),
            'phase_name': rec.get('phase_name'),
            'surface_after': next_surface,
            'next_support_legs': [{'leg_id': i, 'leg_name': LEGACY_ID_TO_NAME.get(i)} for i in support],
            'body_xy_m': [float(body[0]), float(body[1])],
            'next_support_center_xy_m': [float(center[0]), float(center[1])],
            'body_to_next_support_center_xy_m': err,
            'base_pose': rec.get('base_pose'),
        }
        boundaries.append(item)
        if err > max_err:
            max_err = err
            worst = item
    return {
        'boundary_count': len(boundaries),
        'max_body_to_next_support_center_xy_m': max_err if boundaries else None,
        'worst_connection_boundary': worst,
        'boundaries': boundaries,
        'connection_note': 'Centroid metric is a kinematic diagnostic on the ground-plane XY projection, not a full support-polygon stability proof.',
    }
