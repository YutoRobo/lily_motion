# -*- coding: utf-8 -*-
"""Leg role helpers for v3."""
from lily_motion_v3 import leg_role as R


def roles_from_contact_state(contact_state, leg_ids):
    roles = dict((int(i), R.OTHER) for i in leg_ids)
    for i in contact_state.support_legs:
        roles[int(i)] = R.SUPPORT
    for i in contact_state.candidate_support_legs:
        roles[int(i)] = R.CANDIDATE_SUPPORT
    for i in contact_state.lift_legs:
        roles[int(i)] = R.LIFT
    for i in contact_state.clearance_legs:
        roles[int(i)] = R.CLEARANCE
    for i in contact_state.transfer_legs:
        roles[int(i)] = R.TRANSFER
    return roles
