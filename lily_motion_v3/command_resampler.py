# -*- coding: utf-8 -*-
"""Command-log diagnostics and replay-oriented resampling utilities.

This module operates on JSONL records that contain ``joint_command_rad``.
It is intentionally independent from ROS/Gazebo so it can be used in tests,
command export, dry-run diagnostics, and replay preparation.

v3.0.32 adds segmented resampling/smoothing.  For repeated roll preview,
``segment_key='roll_index'`` prevents interpolation or moving-average windows
from crossing a surface/quarter-roll boundary.  This preserves the intended
singular-posture avoidance at roll boundaries while still smoothing within each
quarter-roll.
"""
from __future__ import division, print_function
import json
import math
import os

try:
    range = xrange
except NameError:  # pragma: no cover on py3 only
    pass


def load_command_records(path):
    records = []
    with open(path) as f:
        for i, line in enumerate(f):
            if not line.strip():
                continue
            rec = json.loads(line)
            if 'joint_command_rad' not in rec:
                raise ValueError('record %d has no joint_command_rad: %s' % (i, path))
            rec = dict(rec)
            rec.setdefault('frame_index', rec.get('command_index', i))
            records.append(rec)
    return records


def write_command_records(records, path):
    d = os.path.dirname(path)
    if d and not os.path.isdir(d):
        os.makedirs(d)
    with open(path, 'w') as f:
        for rec in records:
            f.write(json.dumps(rec, sort_keys=True))
            f.write('\n')


def _lerp(a, b, alpha):
    return a + (b - a) * alpha


def _interp_command(q0, q1, alpha):
    return [_lerp(float(a), float(b), alpha) for a, b in zip(q0, q1)]


def _same_segment(r0, r1, segment_key):
    if not segment_key:
        return True
    return r0.get(segment_key) == r1.get(segment_key)


def _split_by_segment(records, segment_key):
    if not segment_key or not records:
        return [records]
    groups = []
    current = [records[0]]
    last = records[0].get(segment_key)
    for r in records[1:]:
        val = r.get(segment_key)
        if val == last:
            current.append(r)
        else:
            groups.append(current)
            current = [r]
            last = val
    groups.append(current)
    return groups


def _renumber_records(records):
    out = []
    for i, r in enumerate(records):
        nr = dict(r)
        nr['resampled_index'] = i
        out.append(nr)
    return out


def resample_command_records(records, factor=1, segment_key=None):
    """Linearly interpolate command records.

    ``factor=1`` returns a shallow copy with normalized indices.  ``factor=4``
    inserts three evenly spaced command frames between every original pair.

    If ``segment_key`` is given, interpolation is performed independently within
    contiguous groups that share the same key.  For repeated rolls, use
    ``segment_key='roll_index'`` so no interpolated frame is inserted between
    the final frame of one quarter roll and the first frame of the next.
    """
    factor = int(factor or 1)
    if factor <= 1 or len(records) <= 1:
        out = []
        for i, r in enumerate(records):
            nr = dict(r)
            nr['resampled_index'] = i
            nr.setdefault('source_frame_index', r.get('frame_index', i))
            nr.setdefault('interpolation_alpha', 0.0)
            if segment_key:
                nr['resample_segment_key'] = segment_key
            out.append(nr)
        return out

    out = []
    out_index = 0
    groups = _split_by_segment(records, segment_key)
    for group_index, group in enumerate(groups):
        if len(group) == 1:
            r = dict(group[0])
            r['joint_command_rad'] = list(group[0]['joint_command_rad'])
            r['joint_command_deg'] = [math.degrees(v) for v in r['joint_command_rad']]
            r['resampled_index'] = out_index
            r['source_frame_index'] = group[0].get('frame_index', out_index)
            r['interpolation_alpha'] = 0.0
            r['resample_factor'] = factor
            if segment_key:
                r['resample_segment_key'] = segment_key
                r['resample_segment_group'] = group_index
            out.append(r)
            out_index += 1
            continue
        for i in range(len(group) - 1):
            r0 = group[i]
            r1 = group[i + 1]
            q0 = list(r0['joint_command_rad'])
            q1 = list(r1['joint_command_rad'])
            for k in range(factor):
                alpha = float(k) / float(factor)
                nr = dict(r0)
                nr['joint_command_rad'] = _interp_command(q0, q1, alpha)
                nr['joint_command_deg'] = [math.degrees(v) for v in nr['joint_command_rad']]
                nr['resampled_index'] = out_index
                nr['source_frame_index'] = r0.get('frame_index', i)
                nr['next_source_frame_index'] = r1.get('frame_index', i + 1)
                nr['interpolation_alpha'] = alpha
                nr['resample_factor'] = factor
                if segment_key:
                    nr['resample_segment_key'] = segment_key
                    nr['resample_segment_group'] = group_index
                out.append(nr)
                out_index += 1
        last = dict(group[-1])
        last['joint_command_rad'] = list(group[-1]['joint_command_rad'])
        last['joint_command_deg'] = [math.degrees(v) for v in last['joint_command_rad']]
        last['resampled_index'] = out_index
        last['source_frame_index'] = group[-1].get('frame_index', len(records)-1)
        last['interpolation_alpha'] = 0.0
        last['resample_factor'] = factor
        if segment_key:
            last['resample_segment_key'] = segment_key
            last['resample_segment_group'] = group_index
        out.append(last)
        out_index += 1
    return out


def _moving_average_command_records_one_segment(records, window=1):
    window = int(window or 1)
    if window <= 1 or len(records) <= 1:
        return [dict(r) for r in records]
    if window % 2 == 0:
        window += 1
    half = window // 2
    n = len(records[0]['joint_command_rad'])
    out = []
    for i, r in enumerate(records):
        lo = max(0, i - half)
        hi = min(len(records), i + half + 1)
        count = float(hi - lo)
        q = []
        for j in range(n):
            q.append(sum(float(records[k]['joint_command_rad'][j]) for k in range(lo, hi)) / count)
        nr = dict(r)
        nr['joint_command_rad'] = q
        nr['joint_command_deg'] = [math.degrees(v) for v in q]
        nr['smoothing_window'] = window
        out.append(nr)
    return out


def moving_average_command_records(records, window=1, segment_key=None):
    """Apply a centered moving average to joint commands.

    This is for Gazebo preview smoothness only.  It may slightly alter contact
    consistency, so diagnostics should be checked both before and after use.

    If ``segment_key`` is given, moving average windows are reset at segment
    boundaries.  For repeated roll preview, use ``segment_key='roll_index'`` so
    the surface-switch boundary is not averaged with the previous/next roll.
    """
    window = int(window or 1)
    if window <= 1 or len(records) <= 1:
        return [dict(r) for r in records]
    groups = _split_by_segment(records, segment_key)
    out = []
    for group_index, group in enumerate(groups):
        gout = _moving_average_command_records_one_segment(group, window=window)
        for r in gout:
            if segment_key:
                r['smoothing_segment_key'] = segment_key
                r['smoothing_segment_group'] = group_index
            out.append(r)
    return _renumber_records(out)


def _nearest_equivalent_angle(value, reference):
    """Return value + 2*pi*k closest to reference.

    This does not average across roll/surface boundaries. It only chooses an
    equivalent angular representation, so the represented joint posture is the
    same while the command time series becomes continuous for Gazebo preview.
    """
    value = float(value)
    reference = float(reference)
    twopi = 2.0 * math.pi
    # Python 2.7 has no math.isfinite.  Use x==x and not +/-inf instead.
    if (value != value) or (reference != reference):
        return value
    if abs(value) == float("inf") or abs(reference) == float("inf"):
        return value
    k = int(round((reference - value) / twopi))
    candidates = [value + twopi * (k + dk) for dk in (-1, 0, 1)]
    return min(candidates, key=lambda x: abs(x - reference))


def unwrap_continuous_command_records(records, key='joint_command_rad'):
    """Choose 2*pi-equivalent commands closest to the previous frame.

    This is different from moving average.  It does not mix commands from
    different roll_index/surface segments.  It only fixes angle-representation
    jumps such as +179 deg -> -179 deg that are the same physical posture but
    look like a 358 deg command jump to Gazebo controllers.
    """
    if not records:
        return []
    out = []
    prev = None
    for i, r in enumerate(records):
        nr = dict(r)
        q = [float(v) for v in nr[key]]
        if prev is None:
            uq = list(q)
        else:
            uq = [_nearest_equivalent_angle(v, p) for v, p in zip(q, prev)]
        nr[key] = uq
        nr['joint_command_deg'] = [math.degrees(v) for v in uq]
        nr['angle_unwrap_continuous'] = True
        nr['angle_unwrap_index'] = i
        out.append(nr)
        prev = uq
    return out


def boundary_transition_diagnostics(records, segment_key='roll_index', top_joints=8):
    """Report command jumps at segment boundaries.

    This isolates RF-6(final) -> next RF-1(first) jumps, which are hidden if
    only global adjacent-delta statistics are inspected.
    """
    top_joints = int(top_joints or 8)
    boundaries = []
    if len(records) < 2:
        return {'boundary_count': 0, 'boundaries': [], 'worst_boundary': None}
    for i in range(len(records)-1):
        r0, r1 = records[i], records[i+1]
        if r0.get(segment_key) == r1.get(segment_key):
            continue
        q0 = [float(v) for v in r0['joint_command_rad']]
        q1 = [float(v) for v in r1['joint_command_rad']]
        ds = [abs(b-a) for a,b in zip(q0,q1)]
        order = sorted(range(len(ds)), key=lambda j: ds[j], reverse=True)
        item = {
            'transition_index': i,
            'from_segment': r0.get(segment_key),
            'to_segment': r1.get(segment_key),
            'from_frame_index': r0.get('frame_index', i),
            'to_frame_index': r1.get('frame_index', i+1),
            'from_phase_name': r0.get('phase_name'),
            'to_phase_name': r1.get('phase_name'),
            'max_abs_delta_rad': max(ds) if ds else 0.0,
            'max_abs_delta_deg': math.degrees(max(ds)) if ds else 0.0,
            'top_joints': []
        }
        for j in order[:top_joints]:
            item['top_joints'].append({
                'joint_index': j,
                'from_rad': q0[j],
                'to_rad': q1[j],
                'delta_rad': q1[j]-q0[j],
                'abs_delta_rad': abs(q1[j]-q0[j]),
                'from_deg': math.degrees(q0[j]),
                'to_deg': math.degrees(q1[j]),
                'delta_deg': math.degrees(q1[j]-q0[j]),
                'abs_delta_deg': math.degrees(abs(q1[j]-q0[j])),
            })
        boundaries.append(item)
    worst = None
    if boundaries:
        worst = max(boundaries, key=lambda b: b['max_abs_delta_rad'])
    return {
        'boundary_count': len(boundaries),
        'segment_key': segment_key,
        'boundaries': boundaries,
        'worst_boundary': worst,
    }


def command_range_diagnostics(records):
    if not records:
        return {'frame_count': 0}
    n = len(records[0]['joint_command_rad'])
    mins = [min(float(r['joint_command_rad'][i]) for r in records) for i in range(n)]
    maxs = [max(float(r['joint_command_rad'][i]) for r in records) for i in range(n)]
    deltas = [maxs[i] - mins[i] for i in range(n)]
    return {
        'frame_count': len(records),
        'nonzero_joint_count': sum(1 for d in deltas if abs(d) > 1e-9),
        'max_delta_rad': max(deltas) if deltas else 0.0,
        'max_delta_deg': math.degrees(max(deltas)) if deltas else 0.0,
        'deltas_rad': deltas,
        'deltas_deg': [math.degrees(d) for d in deltas],
        'mins_rad': mins,
        'maxs_rad': maxs,
    }


def adjacent_delta_diagnostics(records):
    if len(records) < 2:
        return {
            'max_adjacent_delta_rad': 0.0,
            'max_adjacent_delta_deg': 0.0,
            'per_joint_max_adjacent_delta_rad': [],
            'per_joint_max_adjacent_delta_deg': [],
            'worst_transition': None,
            'phase_summary': [],
        }
    n = len(records[0]['joint_command_rad'])
    per_joint = [0.0] * n
    per_joint_at = [None] * n
    phase_acc = {}
    worst = {'abs_delta_rad': -1.0}
    for i in range(len(records) - 1):
        r0 = records[i]
        r1 = records[i+1]
        q0 = r0['joint_command_rad']
        q1 = r1['joint_command_rad']
        phase = str(r1.get('phase_name', r0.get('phase_name', 'unknown')))
        max_for_transition = 0.0
        max_joint = 0
        for j in range(n):
            d = abs(float(q1[j]) - float(q0[j]))
            if d > per_joint[j]:
                per_joint[j] = d
                per_joint_at[j] = i
            if d > max_for_transition:
                max_for_transition = d
                max_joint = j
        if max_for_transition > worst['abs_delta_rad']:
            worst = {
                'transition_index': i,
                'from_frame_index': r0.get('frame_index', i),
                'to_frame_index': r1.get('frame_index', i+1),
                'from_phase_name': r0.get('phase_name'),
                'to_phase_name': r1.get('phase_name'),
                'from_roll_index': r0.get('roll_index'),
                'to_roll_index': r1.get('roll_index'),
                'joint_index': max_joint,
                'abs_delta_rad': max_for_transition,
                'abs_delta_deg': math.degrees(max_for_transition),
            }
        acc = phase_acc.setdefault(phase, {'phase_name': phase, 'transition_count': 0, 'max_adjacent_delta_rad': 0.0, 'transition_index': None, 'joint_index': None})
        acc['transition_count'] += 1
        if max_for_transition > acc['max_adjacent_delta_rad']:
            acc['max_adjacent_delta_rad'] = max_for_transition
            acc['transition_index'] = i
            acc['joint_index'] = max_joint
    phases = []
    for k in sorted(phase_acc.keys()):
        a = phase_acc[k]
        a = dict(a)
        a['max_adjacent_delta_deg'] = math.degrees(a['max_adjacent_delta_rad'])
        phases.append(a)
    return {
        'max_adjacent_delta_rad': max(per_joint),
        'max_adjacent_delta_deg': math.degrees(max(per_joint)),
        'per_joint_max_adjacent_delta_rad': per_joint,
        'per_joint_max_adjacent_delta_deg': [math.degrees(v) for v in per_joint],
        'per_joint_worst_transition_index': per_joint_at,
        'worst_transition': worst,
        'phase_summary': phases,
    }


def full_command_diagnostics(records):
    d = command_range_diagnostics(records)
    d.update(adjacent_delta_diagnostics(records))
    return d
