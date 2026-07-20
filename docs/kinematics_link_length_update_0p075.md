# Kinematics Link Length Update: 0.075 m

This update makes future FK/IK generation, diagnostics, and searches use the corrected first/coxa link length of 0.075 m. Existing command logs are joint-angle logs and were not rewritten.

## Code Changes

- Added `lily_motion_v3/robot_geometry.py` with `COXA_LENGTH = 0.075`, `THIGH_LENGTH = 0.3`, `TIBIA_LENGTH = 0.3`, and `LINK_LENGTHS = (0.0, 0.075, 0.3, 0.3)`.
- Updated `LegKinematicConfig()` defaults to use the shared geometry constants.
- Added `LegacyStateMachineConfig.link_lengths`, defaulting to `(0.0, 0.075, 0.3, 0.3)`.
- Updated `LegacyStateMachineEmulator._make_leg()` to call `lg.setLinkLength(list(self.config.link_lengths))` instead of hardcoding `[0, 0.05, 0.3, 0.3]`.
- Added regression tests for the 0.075 defaults and FK/IK round trip.

## 0.05 After Update

Raw results are in `testdata/kinematics_link_length_audit/grep_0p05_after_update.txt`.

No active `lily_motion_v3` first-link length hardcode remains at 0.05. Remaining active-code `0.05` values are different parameters, such as clearance thresholds, contact drift limits, `goal3_lift_z`, `goal4_target_x`, smoothing profile constants, and warning thresholds.

Known non-active/stale 0.05 references:

- `archive/v3_experiment_scripts/run_v3_0_42c_candidate03_local_refine.py`: archived old exploratory script with `COXA_LENGTH_M = 0.05`; keep as archive/stale unless that workflow is revived.
- `docs/v3_0_project_contained_kinematics_notes.md`: historical docs still mention coxa 0.05.
- `docs/kinematics_link_length_audit_0p075.md`: pre-update audit intentionally records the old stale state.

## 0.075 Re-evaluation

| dataset | angle gate | frames | thigh max deg | tibia max deg | base max deg | max adjacent delta deg | primitive inter-leg min m | primitive housing min m | foot penetration count |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| candidate02_softlimit_94p8 | True | 2233 | 94.8 | 130.415820782 | 224.958527758 | 5.56001875295 | 0.165715128213 | 0.165715128213 | 4948 |
| candidate_022_wide | True | 2233 | 94.8 | 130.415820782 | 224.958527758 | 5.56001875295 | 0.184676148893 | 0.184676148893 | 4940 |

Foot penetration remains informational for this primitive evaluator because previous reports already showed visual-benign contact drift can be counted as penetration. Gazebo visual review remains the hardware gate.

## Candidate Handling

- `candidate02_softlimit_94p8`: keep existing `commands.jsonl`; it was not rewritten. Treat old 0.05 primitive reports as stale, but the 0.075 re-eval angle gate passes.
- `candidate_022_wide`: keep existing `commands.jsonl`; it was not rewritten. Under 0.075 primitive re-eval, inter-leg/housing min distance is larger than candidate02 and angle gate passes. Continue Gazebo visual confirmation before adoption.

## Stale Reports

All primitive FK/IK-derived reports created before this update with the old 0.05 first-link default should be treated as stale for exact geometry. This includes near-contact phase scans, legacy constraint evals, and housing/inter-leg primitive summaries. Existing Gazebo replay command logs are not stale merely because of this change, because they contain joint-angle commands.

## Safety State

- data_reference_candidates_modified=false
- tools_can_interface_modified=false
- publish_cmdforjetson_jsonl_modified=false
- can0_opened=false
- hardware_can_sent=false
