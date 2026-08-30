# -*- coding: utf-8 -*-
"""Small pure helpers for /cmdForJetson subscriber safety policy.

The normal integrated Operator owns the hardware StateMachine subscriber under
/lily_operator.  It may optionally fan the same command stream out to the
existing Gazebo MCU interpolator.  Unknown subscribers remain rejected.

Other MotionPanel hosts keep the legacy exactly-one-subscriber rule.
Python 2.7 compatible.
"""
from __future__ import division

NORMAL_OPERATOR_NODE = '/lily_operator'
GAZEBO_INTERPOLATOR_NODE = '/lily_gazebo_mcu_position_interpolator'


def connection_count_candidate_ok(own_name, connection_count):
    """Cheap pre-check used for SEND-button enablement.

    Detailed node-name validation is performed immediately before SEND and while
    SEND is active.  The normal Operator may have StateMachine only (1) or
    StateMachine + Gazebo interpolator (2).  Other hosts retain exactly one.
    """
    count = int(connection_count)
    if str(own_name) == NORMAL_OPERATOR_NODE:
        return count in (1, 2)
    return count == 1


def check_subscriber_topology(own_name, subscribers, connection_count):
    """Return (ok, reason) for the allowed /cmdForJetson subscriber topology."""
    own_name = str(own_name)
    nodes = sorted(set(subscribers or []))
    count = int(connection_count)

    if own_name == NORMAL_OPERATOR_NODE:
        if NORMAL_OPERATOR_NODE not in nodes:
            return False, 'required StateMachine subscriber %s is missing' % NORMAL_OPERATOR_NODE

        allowed = set((NORMAL_OPERATOR_NODE, GAZEBO_INTERPOLATOR_NODE))
        unknown = [node for node in nodes if node not in allowed]
        if unknown:
            return False, 'unknown subscriber(s): %s' % ', '.join(unknown)

        if len(nodes) not in (1, 2):
            return False, 'expected StateMachine with optional Gazebo subscriber; found: %s' % (
                ', '.join(nodes) if nodes else 'none')

        if count != len(nodes):
            return False, 'publisher connection count %d does not match verified subscribers %d' % (
                count, len(nodes))

        return True, None

    # Preserve the existing rule for Gazebo-only and any legacy MotionPanel host.
    if len(nodes) != 1 or count != 1:
        return False, 'expected exactly one subscriber; found: %s' % (
            ', '.join(nodes) if nodes else 'none')
    return True, None
