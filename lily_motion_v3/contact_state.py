# -*- coding: utf-8 -*-
"""Contact-state data structures for v3 roll design."""


class ContactState(object):
    def __init__(self, surface_id, support_legs=None, candidate_support_legs=None,
                 lift_legs=None, clearance_legs=None, transfer_legs=None):
        self.surface_id = surface_id
        self.support_legs = list(support_legs or [])
        self.candidate_support_legs = list(candidate_support_legs or [])
        self.lift_legs = list(lift_legs or [])
        self.clearance_legs = list(clearance_legs or [])
        self.transfer_legs = list(transfer_legs or [])

    def to_dict(self):
        return {
            "surface_id": self.surface_id,
            "support_legs": list(self.support_legs),
            "candidate_support_legs": list(self.candidate_support_legs),
            "lift_legs": list(self.lift_legs),
            "clearance_legs": list(self.clearance_legs),
            "transfer_legs": list(self.transfer_legs),
        }
