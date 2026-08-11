# Pre-Hardware Software Baseline — 2026-08-12

## Purpose

This document freezes the software/configuration state immediately before staged real-hardware validation of the current Lily rolling candidate.

The trajectory candidate itself is not modified by this baseline record.

## Reference Candidate

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
```

Candidate properties retained from the frozen candidate manifest:

- command count: `2233`
- candidate command checksum SHA256: `e60c9de63287c5c198e78e11c1da89475b2293e6de45950cf09f5f2c170304a5`
- coxa length: `0.075 m`
- thigh length: `0.300 m`
- tibia length: `0.300 m`
- maximum second-joint angle: `94.8 deg`
- second-joint violations over `95 deg`: `0`
- full-roll Gazebo review: `PASS`
- hardware full roll: `NOT TESTED`

## Frozen Staged Inputs

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/
  air_entry_and_hold_only_commands.jsonl
  roll_0_50_commands.jsonl
  roll_50_100_commands.jsonl
  roll_100_300_commands.jsonl
  roll_300_end_commands.jsonl
```

## Canonical Shared Command Path

The maintained upstream path is common to both real hardware and Gazebo:

```text
staged JSONL
  -> tools/publish_cmdforjetson_jsonl.py
  -> shared source normalization
  -> shared transport resampling
  -> /cmdForJetson (sensor_msgs/JointState, 24 positions, rad)
```

There is no Gazebo-specific trajectory-generation branch in the canonical publisher.

Real hardware consumes `/cmdForJetson` through:

```text
/cmdForJetson
  -> tools/can_interface/statemachine/StateMachine
  -> CAN POSITION
  -> real MCU
```

Gazebo consumes the exact same `/cmdForJetson` boundary through:

```text
/cmdForJetson
  -> tools/gazebo/mcu_position_interpolator_node.py
  -> MCU-equivalent interpolation
  -> Gazebo joint command topics
```

## Pre-Hardware Transport Profile

The current hardware-trial candidate transport profile is:

```text
resample-factor = 2
transport rate  = 10 Hz
```

Interpretation:

- the frozen JSONL remains the trajectory/keyframe source;
- factor `2` inserts one linear midpoint between adjacent source targets;
- 10 Hz transport preserves approximately the original 5 Hz source trajectory time scale;
- this is a transport/replay policy, not a modification of the frozen candidate JSONL.

This profile is frozen for the upcoming staged hardware validation. It is not yet declared universally valid for future MCU firmware revisions.

## Current MCU-Equivalent Gazebo Profile

Gazebo-only MCU interpolation parameters used for the completed validation:

```text
interpolation duration = 0.100 s
update period          = 0.002 s
```

These parameters represent the current MCU behavior under evaluation and remain configurable. They are not architectural constants.

## Verified Common-Path Dry Run

For:

```text
staged/air_entry_and_hold_only_commands.jsonl
```

with:

```text
resample-factor = 2
rate            = 10 Hz
```

verified result:

```text
source_frames=135
transport_frames=269
transport_sha256=e1c00e23811f841e86ca4ff3fdc9a42c380e6537f6cf9623f97334a020f5a0fa
output_topic=/cmdForJetson
```

## Unit / Regression Test Result

Verified on Python 2 before freezing:

```text
tests/test_publish_cmdforjetson_jsonl_resampling.py   5 tests PASS
tests/test_command_timing.py                          7 tests PASS
tests/test_shared_command_stream.py                    3 tests PASS
tests/test_gazebo_mcu_interpolator_online.py           6 tests PASS
```

## Gazebo Validation Result

Using the canonical shared publisher and the independent Gazebo MCU interpolation node, the following sequence was executed successfully without changing the command-path architecture or timing parameters:

```text
HOME -> air-entry
     -> roll_0_50
     -> roll_50_100
     -> roll_100_300
     -> roll_300_end
```

Observed result:

- air-entry: `PASS`
- roll 0–50: `PASS`
- roll 50–100: `PASS`
- roll 100–300: `PASS`
- roll 300–end: `PASS`
- visible stage-boundary discontinuity: none observed
- final-pose hold after publisher completion: `PASS`
- Gazebo MCU interpolation node remained active across roll stage boundaries

This validates the current Gazebo-side execution architecture and the pre-hardware transport profile. It does not prove the real MCU / mechanism / CAN bus behavior under load.

## Canonical Runtime Files At Freeze

```text
lily_motion_v3/command_stream.py
lily_motion_v3/command_timing.py
lily_motion_v3/gazebo_actuator_interpolator.py
tools/publish_cmdforjetson_jsonl.py
tools/gazebo/mcu_position_interpolator_node.py
tools/can_interface/statemachine/
```

The previously introduced backend-switch runner and direct file-to-Gazebo hardware-equivalent runner are not canonical runtime paths and were removed before this baseline.

## Hardware Validation Status

The software is considered ready to start staged hardware validation.

This does **not** authorize immediate full-roll execution.

Required progression remains:

```text
single axis
-> one leg / three axes
-> suspended air-entry
-> controlled touchdown
-> roll 0–50
-> roll 50–100
-> roll 100–300
-> roll 300–end
-> full combined sequence only after prior stages pass
```

## Change-Control Rule

From this baseline forward, do not silently modify any of the following during hardware validation:

- reference candidate JSONL
- staged JSONL files
- resample factor
- transport rate
- `/cmdForJetson` message semantics
- CAN StateMachine position-command mapping
- MCU interpolation assumptions used for Gazebo comparison

If one of these must change because of a hardware finding, create a new baseline/version and record the reason rather than overwriting this baseline.