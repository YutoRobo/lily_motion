# -*- coding: utf-8 -*-
"""Minimal v3 roll concept phase generator.

v3.0.13 expands contact-plan variants.  The goal is to make the support-set
assumptions explicit and sweepable before trying to tune continuous parameters.
"""
from lily_motion_v3.contact_state import ContactState
from lily_motion_v3.phase_spec import PhaseSpec
from lily_motion_v3 import leg_role as R


CONTACT_PLAN_VARIANTS = [
    "default",
    "next_only_roll",
    "six_support_roll",
    "front_pair_roll",
    "rear_pair_roll",
    "upper_front_pair_roll",
    "upper_rear_pair_roll",
    "lower_front_pair_roll",
    "lower_rear_pair_roll",
    "diagonal_front_roll",
    "diagonal_rear_roll",
    "four_corner_roll",
    "x_cross_roll",
    "upper_quad_roll",
    "lower_quad_roll",
    "legacy_six_middle_roll",
]

# Leg IDs follow the project-contained v3 naming/order:
#   0 TRF, 1 TRH, 2 BRF, 3 BRH, 4 TLF, 5 TLH, 6 BLF, 7 BLH.
BOTTOM_SUPPORT = [1, 3, 0, 2]
TOP_CANDIDATE = [4, 6]
TOP_ALL = [4, 5, 6, 7]
BOTTOM_ALL = [0, 1, 2, 3]


def _support_plan(roll_support, transfer_support=None, lift_legs=None,
                  candidate_support=None, initial_support=None,
                  clearance=None, transfer_legs=None, posture_support=None):
    """Build the common phase support dictionary.

    The generator intentionally keeps this as explicit data, not hidden logic,
    because the current research question is which contact set can survive a
    full roll trajectory under IK, clearance, joint, and filter constraints.
    """
    initial_support = list(initial_support if initial_support is not None else BOTTOM_SUPPORT)
    candidate_support = list(candidate_support if candidate_support is not None else TOP_CANDIDATE)
    roll_support = list(roll_support)
    transfer_support = list(transfer_support if transfer_support is not None else candidate_support)
    posture_support = list(posture_support if posture_support is not None else transfer_support)
    if lift_legs is None:
        keep = set(roll_support)
        lift_legs = [i for i in range(8) if i not in keep]
    clearance = list(clearance if clearance is not None else [i for i in range(8) if i not in initial_support])
    if transfer_legs is None:
        transfer_legs = [i for i in roll_support if i not in transfer_support]
    return {
        "initial_support": initial_support,
        "clearance": clearance,
        "candidate_support": candidate_support,
        "lift_support": roll_support,
        "lift_legs": list(lift_legs),
        "roll_support": roll_support,
        "transfer_support": transfer_support,
        "transfer_legs": list(transfer_legs),
        "posture_support": posture_support,
    }


def _variant_sets(variant):
    variant = str(variant or "default")
    if variant == "legacy_rf_six_middle_roll":
        # Internal v3.0.19 variant used only by LegacyStyleRollCandidateGenerator
        # to obtain RF-named phases without changing the public catalog/tests for
        # legacy_six_middle_roll.
        pass
    elif variant not in CONTACT_PLAN_VARIANTS:
        raise ValueError("unknown contact plan variant: {}".format(variant))

    if variant == "default":
        return {
            "initial_support": [1, 3, 0, 2],
            "clearance": [5, 7, 4, 6],
            "candidate_support": [4, 6],
            "lift_support": [1, 3, 4, 6],
            "lift_legs": [0, 2],
            "roll_support": [1, 3, 4, 6],
            "transfer_support": [4, 6],
            "transfer_legs": [1, 3],
            "posture_support": [4, 6],
        }

    if variant in ("legacy_six_middle_roll", "legacy_rf_six_middle_roll"):
        # Compatibility scaffold for the legacy idea: keep a six-leg contact
        # set around the roll preparation, lift the two middle/transition legs,
        # then roll on the remaining four/contact-candidate legs.  This does
        # not call legacy code and is not numerically identical to RF-1..RF-6;
        # it is a v3-core representation of the same qualitative contact idea.
        return {
            "initial_support": [0, 1, 2, 3, 4, 6],
            "clearance": [5, 7],
            "candidate_support": [4, 6],
            "lift_support": [0, 1, 2, 3, 4, 6],
            "lift_legs": [0, 2],
            "roll_support": [1, 3, 4, 6],
            "transfer_support": [4, 6],
            "transfer_legs": [1, 3],
            "posture_support": [4, 6],
        }

    if variant == "next_only_roll":
        return _support_plan([4, 6], lift_legs=[0, 1, 2, 3], transfer_legs=[0, 1, 2, 3])
    if variant == "six_support_roll":
        return _support_plan([1, 3, 0, 2, 4, 6], clearance=[5, 7], lift_legs=[], transfer_legs=[1, 3, 0, 2])

    # Existing compact pair hypotheses.
    if variant == "front_pair_roll":
        return _support_plan([0, 4], lift_legs=[1, 2, 3, 6], transfer_support=[4, 6], transfer_legs=[0])
    if variant == "rear_pair_roll":
        return _support_plan([2, 6], lift_legs=[0, 1, 3, 4], transfer_support=[4, 6], transfer_legs=[2])

    # New v3.0.13 pair hypotheses: each one makes a different edge/corner
    # assumption explicit.  These are not claimed to be good gaits; they exist
    # to stop hiding contact-set choices inside parameter tuning.
    if variant == "upper_front_pair_roll":
        return _support_plan([0, 5], candidate_support=[5, 7], transfer_support=[5, 7], transfer_legs=[0])
    if variant == "upper_rear_pair_roll":
        return _support_plan([2, 7], candidate_support=[5, 7], transfer_support=[5, 7], transfer_legs=[2])
    if variant == "lower_front_pair_roll":
        return _support_plan([1, 4], candidate_support=[4, 6], transfer_support=[4, 6], transfer_legs=[1])
    if variant == "lower_rear_pair_roll":
        return _support_plan([3, 6], candidate_support=[4, 6], transfer_support=[4, 6], transfer_legs=[3])
    if variant == "diagonal_front_roll":
        return _support_plan([0, 7], candidate_support=[5, 7], transfer_support=[5, 7], transfer_legs=[0])
    if variant == "diagonal_rear_roll":
        return _support_plan([2, 5], candidate_support=[5, 7], transfer_support=[5, 7], transfer_legs=[2])

    # Four-leg hypotheses that keep a larger support polygon through the roll.
    if variant == "four_corner_roll":
        return _support_plan([0, 3, 4, 7], candidate_support=[4, 7], transfer_support=[4, 7], transfer_legs=[0, 3])
    if variant == "x_cross_roll":
        return _support_plan([1, 2, 5, 6], candidate_support=[5, 6], transfer_support=[5, 6], transfer_legs=[1, 2])
    if variant == "upper_quad_roll":
        return _support_plan([0, 1, 4, 5], candidate_support=[4, 5], transfer_support=[4, 5], transfer_legs=[0, 1])
    if variant == "lower_quad_roll":
        return _support_plan([2, 3, 6, 7], candidate_support=[6, 7], transfer_support=[6, 7], transfer_legs=[2, 3])
    raise AssertionError("unreachable")


def _build_legacy_rf_style_roll_concept(surface_id, s):
    """Return RF-named phases for the legacy-style adapter.

    This is still project-contained: it does not call the old RF functions.
    The intent is to preserve the old motion semantics in the common v3 schema:
    six-contact preparation, next-surface candidate placement, middle-pair
    lift/step-over, body roll through the singular/flip-prone zone, support
    transfer, and normalization for the next roll.
    """
    phases = []
    phases.append(PhaseSpec(
        "RF-1_StableSixContact",
        "Legacy-style RF-1: keep a six-leg contact set and confirm the starting support surface.",
        ContactState(surface_id, support_legs=s["initial_support"], clearance_legs=s["clearance"]),
        [R.SUPPORT, R.CLEARANCE],
        ["joint_limit", "ground_clearance", "inter_leg_clearance"],
        notes="v3-contained approximation of the old six-contact starting condition."
    ))
    phases.append(PhaseSpec(
        "RF-2_NextSurfacePreShape",
        "Legacy-style RF-2: pre-shape body pose and next-surface candidate legs before the main roll.",
        ContactState(surface_id, support_legs=s["initial_support"], candidate_support_legs=s["candidate_support"]),
        [R.SUPPORT, R.CANDIDATE_SUPPORT],
        ["ik_reachability", "joint_limit", "inter_leg_clearance"],
        notes="rf2_pitch_scale and rf2_x_scale act mainly in this phase."
    ))
    phases.append(PhaseSpec(
        "RF-3_LiftMiddlePair",
        "Legacy-style RF-3: lift/step the middle transition pair while the remaining contacts preserve the surface.",
        ContactState(surface_id, support_legs=s["lift_support"], lift_legs=s["lift_legs"]),
        [R.SUPPORT, R.LIFT],
        ["ground_clearance", "inter_leg_clearance", "motion_discontinuity", "joint_limit"],
        notes="This corresponds to the old six-contact state where the middle two legs are stepped."
    ))
    phases.append(PhaseSpec(
        "RF-4_BodyRollThroughSingular",
        "Legacy-style RF-4: rotate the body while keeping the active contact set locked; raw flip-like changes are allowed.",
        ContactState(surface_id, support_legs=s["roll_support"]),
        [R.SUPPORT],
        ["support_consistency", "joint_limit", "inter_leg_clearance"],
        notes="Raw 180deg-class changes are not automatically rejected; filtered geometry is evaluated downstream."
    ))
    phases.append(PhaseSpec(
        "RF-5_SupportTransfer",
        "Legacy-style RF-5: transfer support from old surface contacts to next-surface contacts.",
        ContactState(surface_id, support_legs=s["transfer_support"], transfer_legs=s["transfer_legs"]),
        [R.SUPPORT, R.TRANSFER],
        ["support_consistency", "ground_clearance", "joint_limit"],
    ))
    phases.append(PhaseSpec(
        "RF-6_PostureNormalization",
        "Legacy-style RF-6: normalize posture for the next roll start.",
        ContactState(surface_id, support_legs=s["posture_support"], clearance_legs=s["clearance"]),
        [R.SUPPORT, R.CLEARANCE],
        ["joint_limit", "inter_leg_clearance", "ik_reachability"],
    ))
    for p in phases:
        p.metadata = getattr(p, "metadata", {})
        p.metadata["contact_plan_variant"] = "legacy_six_middle_roll"
        p.metadata["variant_sets"] = s
        p.metadata["legacy_rf_style"] = True
    return phases


def build_forward_roll_concept(surface_id=1, contact_plan_variant="default"):
    """Return a role-based phase list for one forward roll."""
    s = _variant_sets(contact_plan_variant)
    if str(contact_plan_variant or "") == "legacy_rf_six_middle_roll":
        return _build_legacy_rf_style_roll_concept(surface_id, s)
    phases = []
    phases.append(PhaseSpec(
        "StableInitialContact",
        "Confirm the current support set before starting a roll.",
        ContactState(surface_id, support_legs=s["initial_support"]),
        [R.SUPPORT],
        ["joint_limit", "ground_clearance", "inter_leg_clearance"]
    ))
    phases.append(PhaseSpec(
        "ClearancePreparation",
        "Move non-support legs away from expected collision zones.",
        ContactState(surface_id, support_legs=s["initial_support"], clearance_legs=s["clearance"]),
        [R.SUPPORT, R.CLEARANCE],
        ["joint_limit", "ground_clearance", "inter_leg_clearance", "motion_discontinuity"]
    ))
    phases.append(PhaseSpec(
        "EstablishNextSupportCandidates",
        "Place future support candidates at feasible contact targets.",
        ContactState(surface_id, support_legs=s["initial_support"], candidate_support_legs=s["candidate_support"]),
        [R.SUPPORT, R.CANDIDATE_SUPPORT],
        ["ik_reachability", "joint_limit", "inter_leg_clearance"]
    ))
    phases.append(PhaseSpec(
        "LiftTransitionLegs",
        "Lift legs that should stop acting as contact legs before body roll.",
        ContactState(surface_id, support_legs=s["lift_support"], lift_legs=s["lift_legs"]),
        [R.SUPPORT, R.LIFT],
        ["ground_clearance", "inter_leg_clearance", "motion_discontinuity", "joint_limit"]
    ))
    phases.append(PhaseSpec(
        "ConstrainedBodyRoll",
        "Rotate the body while keeping the active support set geometrically consistent.",
        ContactState(surface_id, support_legs=s["roll_support"]),
        [R.SUPPORT],
        ["support_consistency", "joint_limit", "inter_leg_clearance"]
    ))
    phases.append(PhaseSpec(
        "SupportTransfer",
        "Transfer the support set from old legs to next-surface legs.",
        ContactState(surface_id, support_legs=s["transfer_support"], transfer_legs=s["transfer_legs"]),
        [R.SUPPORT, R.TRANSFER],
        ["support_consistency", "ground_clearance", "joint_limit"]
    ))
    phases.append(PhaseSpec(
        "PostureNormalization",
        "Normalize posture for the next roll without forcing legacy RF shape.",
        ContactState(surface_id, support_legs=s["posture_support"]),
        [R.SUPPORT, R.CLEARANCE],
        ["joint_limit", "inter_leg_clearance", "ik_reachability"]
    ))
    for p in phases:
        p.metadata = getattr(p, "metadata", {})
        p.metadata["contact_plan_variant"] = contact_plan_variant
        p.metadata["variant_sets"] = s
    return phases
