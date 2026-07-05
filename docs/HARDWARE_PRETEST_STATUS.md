# Hardware Pretest Status

Current date: 2026-07-06

## Status Summary

The software integration from initial posture to CAN frame conversion is complete in mock testing:

`HOME [0,0,0] -> air-entry -> touchdown hold -> candidate02 roll -> /cmdForJetson -> tools/can_interface StateMachine -> CAN frame payload`

This is a software/mock PASS only. Hardware testing has not been performed.

## Execution Target

Use only:

- `tools/can_interface/statemachine/main.py`
- `tools/can_interface/initUI/ui.py`
- `tools/publish_cmdforjetson_jsonl.py`

Do not execute:

- `external/can_interface/260102_usb_can_fast_alignment/`

`external/can_interface` is a legacy snapshot / pre-relocation reference only. It is not the maintained runtime path.

## Files

Canonical roll body, do not edit:

- `data/reference_candidates/v3_0_42c_candidate_02_softlimit_94p8/commands.jsonl`

First hardware air-entry-only file:

- `testdata/hardware_trial_air_entry_only/air_entry_and_hold_only_commands.jsonl`

Full staged sequence, final-stage only:

- `testdata/entry_touchdown_roll_sequence/combined_with_hold_commands.jsonl`

End-to-end mock evidence:

- `testdata/end_to_end_initial_pose_to_roll_can_check/summary.json`
- `testdata/end_to_end_initial_pose_to_roll_can_check/command_sequence_check.json`
- `testdata/end_to_end_initial_pose_to_roll_can_check/phase_boundary_check.json`
- `testdata/end_to_end_initial_pose_to_roll_can_check/use_all_24_can_check.json`
- `testdata/end_to_end_initial_pose_to_roll_can_check/use_4_axis_can_check.json`
- `testdata/end_to_end_initial_pose_to_roll_can_check/run_stop_gate_check.json`
- `testdata/end_to_end_initial_pose_to_roll_can_check/hardware_limit_report.json`

## Mock End-to-End PASS Results

From `testdata/end_to_end_initial_pose_to_roll_can_check/summary.json`:

- total frames: 2368
- air-entry: 120 frames
- touchdown hold: 15 frames
- candidate02 roll body: 2233 frames
- all frames have 24 `joint_command_rad` values
- hardware_limit_v2: PASS
- `/cmdForJetson` equivalent stream into `tools/can_interface`: PASS
- Use=True 0..23: every command frame emits `0x400..0x417`, 24 position CAN frames
- Use=True 0,1,2,3: every command frame emits only `0x400..0x403`, 4 position CAN frames
- Use=False axes emit 0 position CAN frames
- before RUN: 0 position CAN frames
- after RUN: position CAN frames are emitted
- after STOP: position CAN frames stop
- payload compare against representative `can_preview.jsonl`: 192/192 match
- `can0_opened=false`
- `hardware_can_sent=false`
- `external_can_interface_executed=false`

## Use=True Specification

The UI `Use` checkbox is the active joint selection.

- `Use=True` means active. ALIGN, HOME jog, SET HOME, RUN start, and RUN position frames are sent only to active joints.
- `Use=False` means inactive. The joint is excluded from the RUN gate and receives no position CAN frame.
- `/cmdForJetson.position` must still contain 24 rad values so indexes remain stable.
- Hardware limits are enforced for active joints. Inactive out-of-limit values are ignored for CAN send and logged as warnings.
- RUN is rejected if no joints are active.
- Disconnected inactive joints do not block RUN.

## CAN ID And Payload Specification

- ALIGN request TX: `0x000 + joint_index`
- ALIGN result RX: `0x100 + joint_index`
- HOME jog TX: `0x200 + joint_index`
- SET HOME TX: `0x300 + joint_index`
- position command TX: `0x400 + joint_index`
- RUN start TX: `0x600 + joint_index`
- position payload: `[0,0,0,0] + little-endian float32(rad)`
- position unit: rad

## Touchdown Offset Policy

Touchdown offset is not encoded into joint commands. It is an operational base/floor height margin while lowering the robot to the floor at the hold posture.

- analytical minimum pass: +0.013 m
- recommended operational value: +0.015 m
- first hardware trial option: +0.020 m

For the first hardware touchdown, use the +0.020 m equivalent safety option.

## Confirmed Software Safety Checks

- hardware_limit_v2 PASS for `combined_with_hold_commands.jsonl`
- RUN gate obeys Use=True active joint selection
- RUN before `/cmdForJetson` emits no position frames
- STOP sets `is_run=False` and stops subsequent position conversion
- `tools/can_interface` is the execution target
- `external/can_interface` was not executed during mock checks
- `can0` was not opened during mock checks
- no hardware CAN was sent during mock checks

## Hardware-Dependent Items Not Yet Verified

Hardware testing has not been performed. These remain unverified on real units:

- real CAN bus TX/RX
- real unit ACK/response behavior
- ALIGN real response
- HOME direction
- final real posture after SET HOME
- small-angle hardware motion
- air-entry hardware motion
- touchdown hardware contact and clearance
- candidate02 roll body hardware motion
- current, sound, vibration, heat, and mechanical interference under load

## Required Hardware Test Order

Do not start with the full combined file. The sequence must be:

1. small-angle test
2. air-entry + touchdown hold only
3. touchdown contact confirmation
4. short roll segment
5. full roll final confirmation

Initial hardware test must not run `testdata/entry_touchdown_roll_sequence/combined_with_hold_commands.jsonl`.
