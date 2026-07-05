## v3.0.42c candidate_02 Gazebo replay / second joint angle issue

### Input

data/reference_candidates/v3_0_42c_candidate_02_x8_sw40/commands.jsonl

### Replay / evaluation

Example replay command:

```bash
python run_v3_0_42e_effort_replay_plot.py \
  --command-log testdata/v3_0_42c_candidates/candidate_02_x8_sw40_commands.jsonl \
  --strict-command-log-input \
  --rate 5 \
  --hold-start-sec 2.0 \
  --hold-end-sec 2.0 \
  --diagnose-command-log \
  --effort-limit 40 \
  --output testdata/v3_0_42e_effort_candidate_02_rate5.json \
  --plot-dir testdata/v3_0_42e_effort_candidate_02_rate5_plots
```

### Result

- Gazebo replay completed normally.
- second_joint_max_deg: 95.97432281767107
- second_joint_violation_count: 198
- The violation is concentrated in RF-4_Goal4_LandMiddlePair and RF-5_Goal5_MainBodyRoll.
- candidate_02 is not accepted as a baseline because the second joint exceeds 95 deg.

## candidate02_softlimit_94p8 generation and validation

### Branch

experiment/candidate02-second-joint-softlimit

### Input

data/reference_candidates/v3_0_42c_candidate_02_x8_sw40/commands.jsonl

### Output

testdata/candidate02_softlimit/commands.jsonl

Promoted reference copy:

data/reference_candidates/v3_0_42c_candidate_02_softlimit_94p8/commands.jsonl

### Method

Second-joint local softlimit postprocess.

- target joint: joint_index == 1
- target phases:
  - RF-4_Goal4_LandMiddlePair
  - RF-5_Goal5_MainBodyRoll
- soft limit: 94.8 deg
- taper applied around violation intervals
- original candidate_02 command log was not modified

### Result

- second_joint_max_before_deg: 95.97432281767107
- second_joint_max_after_deg: 94.8
- violation_count_before: 198
- violation_count_after: 0
- modified_sample_count: 308
- modified_frame_count: 154
- max_abs_correction_deg: 1.1743228176710687
- max_adjacent_delta_before_deg: 5.56001875294519
- max_adjacent_delta_after_deg: 5.56001875294519
- max_second_diff_before_deg: 3.787713927320219
- max_second_diff_after_deg: 3.787713927320219
- warnings: []

### Constraint comparison

- second_joint_min_clearance_m: 0.0669304010825203
- second_joint_penetration_count: 0
- foot_min_clearance_m: -0.03017711684493357
- foot_penetration_count: 1986
- inter_leg_collision_count: 0
- inter_leg_near_count: 0
- inter_leg_joint_housing_collision_count: 0
- inter_leg_joint_housing_near_count: 0

### Gazebo replay

- Result: completed normally
- published_count: 4546
- preview_frame_count: 2233
- first_invalid_frame: null

### Decision

Promote candidate02_softlimit_94p8 to the next reference candidate.

It removes the second-joint 95 deg violation without worsening adjacent delta, second difference, floor/contact metrics, or inter-leg collision metrics.

It is not yet a final baseline because it is a joint-space postprocess candidate and foot penetration remains unresolved.

## candidate02_softlimit_94p8 foot penetration localization

### Branch

eval/candidate02-softlimit-foot-penetration

### Target

data/reference_candidates/v3_0_42c_candidate_02_softlimit_94p8/commands.jsonl

### Result

- foot_min_clearance_m: -0.03017711684493357
- foot_penetration_count: 1986
- worst:
  - roll_index: 2
  - phase_name: RF-2_Goal2_UpperLegLanding
  - leg_name: TLH
  - frame_index: 147
  - phase_step_index: 3
  - penetration_m: 0.03017711684493357

### Distribution

- RF-2_Goal2_UpperLegLanding: 1730
- RF-1_Goal1_UpperLegPreSwing: 192
- RF-1_Goal1_UpperLegPreSwing_CurrentAngleAnchor: 64
- RF-3/RF-4/RF-5/RF-6: 0

### Role

All foot penetration samples are support-side samples.

- support: 1986
- swing / landing_swing: 0

### Comparison with source candidate_02

The values are identical to the source candidate_02.

- source candidate_02 foot_min_clearance_m: -0.03017711684493357
- source candidate_02 foot_penetration_count: 1986
- softlimit foot_min_clearance_m: -0.03017711684493357
- softlimit foot_penetration_count: 1986

### Interpretation

The foot penetration is not caused by the second-joint softlimit postprocess.

It is concentrated in support feet during RF-1/RF-2 and is likely a legacy FK / body pose / ground-height proxy issue rather than a newly introduced swing-foot dragging issue.

Second joint penetration, inter-leg collision, and housing collision are all zero.

### Decision

Do not block candidate02_softlimit_94p8 adoption as the next provisional baseline candidate based only on this support-foot proxy penetration.

Treat swing-foot penetration, second-joint penetration, inter-leg collision, housing collision, and Gazebo visual failure as hard gates.
Treat support-foot proxy penetration as a monitored metric.

## candidate02_softlimit_94p8 provisional baseline decision

### Decision

candidate02_softlimit_94p8 is the current provisional baseline candidate.

It is selected because Gazebo replay completed normally, second_joint_max_deg is capped at 94.8, second_joint_violation_count is 0, max adjacent delta and max second difference did not worsen from candidate_02, and inter-leg / housing collision metrics are zero.

It is not a final baseline yet because it is produced by a joint-space postprocess and support-foot-only proxy penetration remains monitored.

## hardware_limit_v2 recheck and base_clause ±180 investigation

### Background

Earlier diagnostics treated base_clause ±180 deg as a hard hardware limit.
Under that assumption, `candidate02_softlimit_94p8` appeared to violate the base_clause limit.

However, later mechanical review clarified the actual structural joint ranges:

```text
base_clause: ±360 deg
thigh:       ±95 deg
tibia:       ±150 deg
```

The thigh joint may mechanically approach ±100 deg at the limit, but ±95 deg remains the normal hard gate.

### Original repo runtime check

The original repo `roll(Direction.FORWARD)` was executed four times without publishing to hardware or Gazebo. The execution surface sequence was:

```text
1,5,6,2,1
```

The runtime check showed:

```text
roll trace frames: 276
base servo range: [-380, 380] deg
internal IK angle exceeds ±180 deg: yes
servo target angle exceeds ±180 deg: yes
publish angle exceeds ±180 deg: yes
```

Representative publish angle ranges:

```text
TLF base_clause: -243.6683 .. 44.2951 deg
TRF base_clause: -44.2951 .. 243.6683 deg
base_clause violation count under old ±180 assumption: 154
all-joint violation count under old assumptions: 592
```

Therefore, the original roll program was not designed under a base_clause ±180 deg hardware assumption. The old ±180 diagnostic is retained only as historical context.

The uploaded original `lily_controller.py` also shows that the roll sequence repeatedly uses `support_solve_type=[-1,...]` and `lending_leg_type=[-1,-1]`, with no explicit base_clause ±180 deg clamp in the roll sequence.

### hardware_limit_v2 recheck

Target:

```text
data/reference_candidates/v3_0_42c_candidate_02_softlimit_94p8/commands.jsonl
```

Output:

```text
testdata/hardware_limit_v2_recheck/
```

Result:

```text
hard violation: 0
base soft margin violation >330/340 deg: 0
base_clause: -224.96 .. 224.96 deg
thigh: -94.8 .. 94.8 deg
tibia: -130.03 .. 130.42 deg
second_joint violation: 0
second_joint minimum clearance: 0.06693 m
inter-leg collision: 0
housing collision: 0
```

The candidate passes hardware_limit_v2 hard gates.

### Conclusion

`candidate02_softlimit_94p8` is restored as the current provisional baseline candidate under hardware_limit_v2.

The remaining foot proxy penetration metric is monitored, but it is not treated as a hard gate failure at this stage.
