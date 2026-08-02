# Kinematics Link Length Audit: first link 0.075 m

## Scope

This audit checks whether FK/IK code can handle the first link length changing from 0.05 m to 0.075 m. It does not modify canonical candidates, command logs, CAN tools, or publisher tools.

Safety state: `data/reference_candidates` unchanged, `tools/can_interface` unchanged, `tools/publish_cmdforjetson_jsonl.py` unchanged, `can0_opened=false`, `hardware_can_sent=false`.

## Short Answer

- Project-contained FK: algorithmically yes, because `LegKinematics.forward_kinematics()` reads `config.coxa_length`. The current default is stale at `0.05`.
- Project-contained IK: algorithmically yes, because `LegKinematics.inverse_kinematics_candidates()` reads `config.coxa_length`, `thigh_length`, and `tibia_length`. The current default is stale at `0.05`.
- Vendored legacy FK/IK: algorithmically yes, because `Leg.setLinkLength()` supplies `self.__L`, and FK/IK methods use `self.__L`. The active emulator setup is stale because it calls `setLinkLength([0, 0.05, 0.3, 0.3])`.

## First-Link 0.05 Findings

| Location | Status | Notes |
|---|---|---|
| `lily_motion_v3/leg_config.py` | stale active default | `LegKinematicConfig(coxa_length=0.05, thigh_length=0.3, tibia_length=0.3)` drives `RobotModel()` defaults. |
| `lily_motion_v3/legacy_state_machine_emulator.py` | stale active hardcode | `_make_leg()` calls `lg.setLinkLength([0, 0.05, 0.3, 0.3])`; this affects legacy command generation and legacy geometry evaluation. |
| `docs/v3_0_project_contained_kinematics_notes.md` | stale documentation | Documents coxa as 0.05 m. |
| `lily_motion_v3/legacy_runtime/end_efector_manager.py` | demo only | `setLinkLength([0, 1.0, 1.0, 1.0])` in local sample/demo path, not current runtime. |

## 0.05 Values That Are Not First-Link Length

Examples include `goal3_lift_z`, `goal4_target_x`, clearance thresholds, contact drift limits, inter-leg clearance defaults, and local Y escape sweep values. These should not be mass-replaced. See `testdata/kinematics_link_length_audit/grep_0p05_results.txt` for raw hits.

## FK/IK Design Assessment

Project-contained `LegKinematics` is already parameterized by `LegKinematicConfig`; changing first link length is a config/default issue, not an FK/IK formula issue. `RobotModel` callers that do not pass `leg_config` still inherit stale `0.05`.

Vendored legacy `Leg` also supports link length injection through `setLinkLength()`. The blocker is not the core `Leg` formula; it is `LegacyStateMachineEmulator._make_leg()` hardcoding the old vector.

## FK/IK-Adjacent Stale Evaluation Risk

`LegacyConstraintEvaluator` instantiates `LegacyStateMachineEmulator`, so its FK-derived geometry currently uses `0.05`. Therefore previous primitive geometry results, including near-contact scans and legacy constraint reports, should be treated as stale for exact link geometry until the emulator link length is updated/centralized and those reports are rerun.

Existing `commands.jsonl` files are joint command logs. The user has confirmed the current commands are visually OK in 0.075 URDF Gazebo, so this audit does not invalidate the command logs as Gazebo replay inputs. It invalidates FK/IK-derived diagnostics that assume the old first link length.

## Commonization Recommendation

Introduce one shared geometry source, for example:

- `LegKinematicConfig(coxa_length=0.075, thigh_length=0.3, tibia_length=0.3)` as the project-contained default.
- `LegacyStateMachineConfig(link_lengths=(0, 0.075, 0.3, 0.3))` and use it in `_make_leg()`.
- Add a small regression test that constructs both project-contained and legacy legs with 0.075 and confirms the first-link offset changes as expected.

## Immediate Fix Priority

No commands or canonical candidates should be changed in this audit. For future regeneration/evaluation, update these before trusting new FK/IK-derived reports:

1. `lily_motion_v3/legacy_state_machine_emulator.py` hardcoded `[0, 0.05, 0.3, 0.3]`.
2. `lily_motion_v3/leg_config.py` default `coxa_length=0.05`.
3. Stale documentation in `docs/v3_0_project_contained_kinematics_notes.md`.

## Outputs

- Raw grep results: `testdata/kinematics_link_length_audit/grep_0p05_results.txt`
- FK/IK call graph: `testdata/kinematics_link_length_audit/fk_ik_call_graph_summary.md`
- Dependency table: `testdata/kinematics_link_length_audit/link_length_dependency_table.csv`
