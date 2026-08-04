# Kinematics Link Length Update: 0.075 m

更新日: 2026-08-04

This document records the geometry correction that changed the first/coxa link length from the old 0.05 m assumption to 0.075 m.

Current integrated status and hardware ordering are defined by:

- [`../README.md`](../README.md)
- [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md)
- [`../data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/pre_hardware_decision.md`](../data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/pre_hardware_decision.md)

## 1. Current Shared Geometry

Source:

```text
lily_motion_v3/robot_geometry.py
```

```text
COXA_LENGTH  = 0.075 m
THIGH_LENGTH = 0.300 m
TIBIA_LENGTH = 0.300 m
LINK_LENGTHS = (0.0, 0.075, 0.3, 0.3)
```

These constants are used by current v3 kinematics and legacy-emulation paths that use the shared configuration.

## 2. Code Changes

The geometry update:

- added `lily_motion_v3/robot_geometry.py`
- changed `LegKinematicConfig()` defaults to shared geometry constants
- added `LegacyStateMachineConfig.link_lengths`
- changed `LegacyStateMachineEmulator._make_leg()` to use configured link lengths instead of hardcoded `[0, 0.05, 0.3, 0.3]`
- added regression tests for 0.075 m defaults and FK/IK round trip

## 3. Meaning Of The Update

The geometry change affects:

- future FK and IK calculations
- geometric diagnostics
- primitive link-distance evaluation
- candidate comparisons that depend on link positions

Existing command logs contain joint angles. They were not rewritten merely because the geometry model changed.

Therefore:

```text
old command log ≠ automatically invalid
old geometry-derived report = stale for exact 0.075 m geometry
```

A command log must still be re-evaluated with current geometry before adoption.

## 4. Remaining 0.05 References

Audit output is retained in:

```text
testdata/kinematics_link_length_audit/grep_0p05_after_update.txt
```

No active `lily_motion_v3` first-link hardcode remains at 0.05 m.

Remaining active-code `0.05` values may represent unrelated parameters such as:

- clearance thresholds
- contact drift limits
- lift or target coordinates
- smoothing constants
- warning thresholds

Known stale or historical 0.05 references include:

- `archive/v3_experiment_scripts/run_v3_0_42c_candidate03_local_refine.py`
- `docs/v3_0_project_contained_kinematics_notes.md`
- `docs/kinematics_link_length_audit_0p075.md`

These do not override the shared current geometry.

## 5. 0.075 m Re-Evaluation Results

| Dataset | Angle gate | Frames | Thigh max deg | Tibia max deg | Base max deg | Max adjacent delta deg | Primitive inter-leg min m | Primitive housing min m | Foot penetration count |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| candidate02_softlimit_94p8 | PASS | 2233 | 94.8 | 130.415820782 | 224.958527758 | 5.56001875295 | 0.165715128213 | 0.165715128213 | 4948 |
| candidate_022_wide | PASS | 2233 | 94.8 | 130.415820782 | 224.958527758 | 5.56001875295 | 0.184676148893 | 0.184676148893 | 4940 |

Foot penetration from this primitive evaluator remains informational. Previous reports showed that visually benign support-foot contact drift can be counted as penetration.

For the 0.075 m adoption decision, URDF-derived FK and Gazebo visual review are the primary geometry basis.

## 6. Candidate Decisions

### candidate02_softlimit_94p8

- existing joint-angle `commands.jsonl` was not rewritten
- the 0.075 m angle gate passes
- old 0.05 m primitive reports are stale for exact geometry
- retained as an important comparison and historical baseline
- not the current first pre-hardware candidate

### candidate_022_wide

- existing joint-angle `commands.jsonl` was not rewritten during the geometry correction
- the 0.075 m angle gate passes
- primitive inter-leg/housing minimum is larger than candidate02 in the recorded re-evaluation
- full-roll Gazebo review passed
- frozen as:

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
```

Current status:

```text
pre-hardware candidate: adopted
hardware full roll: not tested
```

The candidate must still follow staged hardware testing. Geometry adoption does not authorize full-roll-first execution.

## 7. Frozen Candidate Evidence

Manifest:

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/manifest.json
```

Summary:

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/summary.json
```

Recorded values include:

```text
coxa length: 0.075 m
command count: 2233
maximum second-joint angle: 94.8 deg
violations over 95 deg: 0
Gazebo full roll: PASS
URDF FK minimum recorded clearance: 0.019648551564994537 m
```

The one-time initial `urdf_worst_1144_1184` blow-up was not reproduced and remains a recorded note rather than a rejection reason.

## 8. Stale Report Policy

Treat all pre-update primitive FK/IK-derived reports using the old 0.05 m first-link default as stale for exact geometry decisions.

Examples:

- near-contact phase scans
- legacy constraint evaluations
- old housing-distance summaries
- old inter-leg primitive summaries

Do not delete them when they are needed for historical traceability. Mark them as old-geometry evidence and do not use them as the current source of truth.

Existing Gazebo command logs are not stale merely because they are joint-angle logs generated before the geometry update. Their geometric interpretation must be reviewed with the current URDF and evaluation path.

## 9. Verification Tests

Representative geometry tests:

```text
tests/test_v3_0_kinematics.py
```

Recommended checks after geometry-code modification:

```bash
python -m pytest -q tests/test_v3_0_kinematics.py
git diff --check
```

Use the Python environment appropriate to the diagnostic code being tested.

## 10. Safety State During The Geometry Update

The geometry update and candidate freeze recorded:

```text
data_reference_candidates_modified only by explicit candidate freeze
can0_opened=false
hardware_can_sent=false
external_can_interface_executed=false
```

These values describe the software evaluation/freeze work. They do not describe the later real axis10 single-axis test.

## 11. Current Boundary

The geometry decision is complete for the v3.0.44 candidate. The remaining uncertainty is hardware-dependent:

- physical sign mapping
- one-leg three-axis behavior
- multi-actuator synchronization
- load, current, vibration, heat, and mechanical deformation
- air-entry and touchdown behavior
- staged and full roll behavior

Those items are controlled by the hardware operation procedure, not by this geometry note.
