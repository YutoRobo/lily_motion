# lily_motion baseline

## Current baseline

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
