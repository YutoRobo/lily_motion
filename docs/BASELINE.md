# lily_motion baseline

## Current provisional baseline candidate

- Candidate name: v3.0.42c candidate_02 softlimit 94.8
- Path: data/reference_candidates/v3_0_42c_candidate_02_softlimit_94p8/commands.jsonl
- Origin: data/reference_candidates/v3_0_42c_candidate_02_x8_sw40/commands.jsonl
- Method: second-joint local softlimit postprocess
- Soft limit: 94.8 deg
- Gazebo replay: completed normally
- second_joint_max_deg: 94.8
- second_joint_violation_count: 0
- max_adjacent_delta: unchanged from candidate_02
- max_second_diff: unchanged from candidate_02
- inter-leg collision / near: 0 / 0
- joint-housing collision / near: 0 / 0
- Known issue: support-foot proxy penetration remains, unchanged from candidate_02
- Status: current provisional baseline candidate, not final baseline

This candidate is Gazebo-confirmed and currently preferred over baseline_v2 for
baseline work. It is not final because it is produced by a joint-space
postprocess and support-foot proxy penetration remains monitored.

## Previous / comparison reference

### baseline_v2_42c_case27_x8_sw40

baseline_v2_42c_case27_x8_sw40 is retained as a comparison reference, not as
the current baseline. Its second-joint numeric metrics are good, but Gazebo
visual replay showed a flip / floor-penetrating posture during the third or
fourth roll, so it is not a Gazebo-confirmed baseline.

## Comparison

| item | baseline_v2_42c_case27_x8_sw40 | candidate_02_x8_sw40 | candidate02_softlimit_94p8 |
| --- | --- | --- | --- |
| path | data/baselines/baseline_v2_42c_case27_x8_sw40/commands.jsonl | data/reference_candidates/v3_0_42c_candidate_02_x8_sw40/commands.jsonl | data/reference_candidates/v3_0_42c_candidate_02_softlimit_94p8/commands.jsonl |
| second_joint_max_deg | 93.9994725377589 | 95.97432281767107 | 94.8 |
| second_joint_violation_count | 0 | 198 | 0 |
| Gazebo replay result | visual replay showed flip / floor-penetrating posture during roll 3/4; not Gazebo-confirmed | completed normally | completed normally |
| max_adjacent_delta_deg | not rechecked in this document | 5.56001875294519 | 5.56001875294519 |
| max_second_diff_deg | not rechecked in this document | 3.787713927320219 | 3.787713927320219 |
| foot_min_clearance_m | not rechecked in this document | -0.03017711684493357 | -0.03017711684493357 |
| foot_penetration_count | not rechecked in this document | 1986 | 1986 |
| inter-leg collision / near | not rechecked in this document | 0 / 0 | 0 / 0 |
| housing collision / near | not rechecked in this document | 0 / 0 | 0 / 0 |
| status | previous / comparison reference only | source candidate; rejected as baseline due second-joint violation | current provisional baseline candidate, not final baseline |

## Gates

### Hard gate

- Gazebo replay failure
- Visible flip / floor-penetrating posture
- second_joint_max_deg > 95.0
- second_joint_violation_count > 0
- second_joint_penetration_count > 0
- inter_leg_collision_count > 0
- inter_leg_joint_housing_collision_count > 0
- Clear worsening of max_adjacent_delta
- Clear worsening of max_second_diff
- Swing-foot dragging / swing-foot penetration that matches Gazebo visual abnormality

### Soft / monitored gate

- Support-foot proxy penetration
- foot_penetration_count
- foot_min_clearance_m
- inter-leg near count
- housing near count
- effort spike
- postprocess correction amount
- modified sample count

## Previous current baseline note

- Baseline name: v3.0.36 RF-1 current-angle anchor + smooth_window=40
- surface_sequence: 1,5,6,2,1
- move_dist: 0.4
- support_dist: 0.7
- legacy_body_z: 0.35
- resample_factor: 8
- smooth_window: 40
- smoothing: across full 4-roll command sequence, not split by roll_index
- Gazebo visual result: acceptable
- Known issue: second joint angle exceeds 95 deg
- Remaining concern: possible middle/front leg visual interference at first quarter roll

## Do not break

- Ability to reproduce baseline command log
- Ability to return from experimental candidates to baseline
- JSONL command log compatibility
- Gazebo replay compatibility

## Provisional baseline verification command

Use the following command to verify the current provisional baseline candidate:

```bash
python run_v3_0_verify_provisional_baseline.py \
  --command-log data/reference_candidates/v3_0_42c_candidate_02_softlimit_94p8/commands.jsonl \
  --output-dir testdata/provisional_baseline_verify

## Provisional baseline Gazebo final check

Target:

- data/reference_candidates/v3_0_42c_candidate_02_softlimit_94p8/commands.jsonl

Result:

- Gazebo replay: OK
- No visible flip / floor-penetrating posture
- No abnormal motion around RF-4/RF-5
- Completed normally

Decision:

candidate02_softlimit_94p8 is confirmed as the current provisional baseline candidate.

## Current Provisional Baseline under hardware_limit_v2

Current provisional baseline candidate:

```text
data/reference_candidates/v3_0_42c_candidate_02_softlimit_94p8/commands.jsonl
```

This candidate is treated as the current provisional baseline under `hardware_limit_v2`.

### hardware_limit_v2 result

| Item                                                     |                Result |
| -------------------------------------------------------- | --------------------: |
| hard joint limit violation                               |                     0 |
| base_clause range                                        | -224.96 .. 224.96 deg |
| thigh range                                              |     -94.8 .. 94.8 deg |
| tibia range                                              | -130.03 .. 130.42 deg |
| base soft margin violation, `abs(base_clause) > 330 deg` |                     0 |
| base soft margin violation, `abs(base_clause) > 340 deg` |                     0 |
| second_joint violation                                   |                     0 |
| second_joint minimum clearance                           |             0.06693 m |
| inter-leg collision                                      |                     0 |
| housing collision                                        |                     0 |

### Status

`candidate02_softlimit_94p8` passes the hardware_limit_v2 hard gate.

It remains a Gazebo-feasible reference and is restored as the current provisional baseline candidate under hardware_limit_v2.

### Remaining monitored issue

The existing foot proxy clearance evaluator reports:

```text
foot_penetration_count = 1986
foot_min_clearance_m ≈ -0.03018
```

This is currently treated as a monitored metric, not as a hard gate failure, because previous localization indicates that it is mainly a support-foot proxy penetration issue rather than a swing-foot floor penetration issue.

This point should remain visible in reports, but it does not block provisional baseline status at this stage.
