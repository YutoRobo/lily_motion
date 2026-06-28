# -*- coding: utf-8 -*-
"""PhaseSpec defines v3 phases by purpose, not by legacy RF call order."""


class PhaseSpec(object):
    def __init__(self, name, purpose, contact_state, allowed_roles=None,
                 constraints=None, notes=""):
        self.name = str(name)
        self.purpose = str(purpose)
        self.contact_state = contact_state
        self.allowed_roles = list(allowed_roles or [])
        self.constraints = list(constraints or [])
        self.notes = str(notes)

    def to_dict(self):
        return {
            "name": self.name,
            "purpose": self.purpose,
            "contact_state": self.contact_state.to_dict() if self.contact_state else None,
            "allowed_roles": list(self.allowed_roles),
            "constraints": list(self.constraints),
            "notes": self.notes,
        }
