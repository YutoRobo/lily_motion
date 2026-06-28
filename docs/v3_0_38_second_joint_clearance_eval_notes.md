# v3.0.38 second-joint clearance evaluation

## Purpose

v3.0.37 showed that the current Gazebo-preview baseline is visually good, but the old `ground_penetration_count` was too coarse.  With `smooth_window=40`, foot/contact-point drift can create small floor-penetration counts even when the motion looks acceptable.  The more important failure mode is the second joint, i.e. the knee/thigh joint region, entering the floor.

v3.0.38 therefore keeps the v3.0.36/v3.0.37 baseline motion and adds part-wise floor clearance reporting.

## Baseline kept

- pure legacy repeated roll
- RF-1 current-angle anchor enabled
- `surface_sequence=1,5,6,2,1`
- `move_dist=0.4`
- `support_dist=0.7`
- `legacy_body_z=0.35`
- `max_step>=30`
- `resample_factor=8`
- `smooth_window=40`
- no `segment_key`, so the moving average is applied to the full four-roll command stream

## What changed

`LegacyConstraintEvaluator` now reports:

```text
constraints.clearance_by_part.second_joint
constraints.second_joint_clearance
constraints.clearance_by_part.foot
constraints.foot_clearance
```

For each part:

```text
min_clearance_m
penetration_count
max_penetration_depth_m
worst
```

The old aggregate `ground_penetration_count` is still present for backward compatibility, but it should not be the main pass/fail criterion at this stage.

## Coordinate convention

The second-joint/knee point is first computed in the robot/body coordinate frame by vendored legacy FK.  It is then transformed by `TransformationRobotToABS(..., posture)`.  That transform includes the body pitch, so the reported second-joint world position already accounts for body rotation.

The floor itself is the fixed Gazebo/world plane:

```text
world_z = ground_z, usually 0.0
```

So the correct clearance criterion is:

```text
second_joint_clearance = second_joint_world_z - ground_z
```

not body-frame z.

## Recommended judgment

Primary floor criterion:

```text
filtered.constraints.second_joint_clearance.min_clearance_m >= 0
filtered.constraints.second_joint_clearance.penetration_count == 0
```

Foot penetration should be treated as a secondary diagnostic, because smoothing can move the command slightly away from exact contact-lock geometry.

## Example

```bash
python run_v3_0_baseline_filtered_constraint_eval.py \
  --surface-sequence 1,5,6,2,1 \
  --move-dist 0.4 \
  --support-dist 0.7 \
  --legacy-body-z 0.35 \
  --max-step 30 \
  --resample-factor 8 \
  --smooth-window 40 \
  --constraint-stride 8 \
  --output-raw-command-log testdata/v3_0_38_baseline_raw_commands.jsonl \
  --output-filtered-command-log testdata/v3_0_38_baseline_x8_sw40_commands.jsonl \
  --report-output testdata/v3_0_38_second_joint_clearance_report.json
```

Then inspect:

```bash
python - <<'PY'
import json
p = 'testdata/v3_0_38_second_joint_clearance_report.json'
d = json.load(open(p))
for key in ['raw', 'filtered']:
    c = d[key]['constraints']
    sj = c.get('second_joint_clearance', {})
    foot = c.get('foot_clearance', {})
    print('\n== %s ==' % key)
    print('max_second_joint_deg =', c.get('max_second_joint_deg'))
    print('second_joint_angle_violations =', c.get('second_joint_violation_count'))
    print('second_joint_min_clearance_m =', sj.get('min_clearance_m'))
    print('second_joint_penetration_count =', sj.get('penetration_count'))
    print('second_joint_max_penetration_depth_m =', sj.get('max_penetration_depth_m'))
    print('foot_min_clearance_m =', foot.get('min_clearance_m'))
    print('foot_penetration_count =', foot.get('penetration_count'))
PY
```
