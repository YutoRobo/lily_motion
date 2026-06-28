## v3.0.42c candidate_02 x8 sw40 Gazebo replay

### Input

data/reference_candidates/v3_0_42c_candidate_02_x8_sw40/commands.jsonl

### Replay command

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

## v3.0.42c candidate_02 x8 sw40 evaluation

### Input

data/reference_candidates/v3_0_42c_candidate_02_x8_sw40/commands.jsonl

### Gazebo replay

Replay command:

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

## candidate02_softlimit_94p8 evaluation

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
