# Hardware Operation Procedure

Current date: 2026-07-06

This document is intentionally explicit. Follow it in order. Do not skip directly to roll.

## 1. Core Rules

- `can0` must not be opened during dry-run, mock, documentation, or pretest checks.
- Hardware CAN must not be sent unless the operator is intentionally performing a hardware trial.
- `external/can_interface` is legacy reference only. Do not execute it.
- The only CAN execution target is `tools/can_interface/`.
- `data/reference_candidates` is the canonical source area. Do not edit it.
- Initial hardware trial must not stream `combined_with_hold_commands.jsonl`.
- Test order is: small-angle test -> air-entry only -> touchdown -> short roll -> full roll.

## 2. File Structure

Runtime code:

- `tools/can_interface/statemachine/main.py`
- `tools/can_interface/initUI/ui.py`
- `tools/publish_cmdforjetson_jsonl.py`

Canonical roll body, read-only:

- `data/reference_candidates/v3_0_42c_candidate_02_softlimit_94p8/commands.jsonl`

First hardware air-entry-only command log:

- `testdata/hardware_trial_air_entry_only/air_entry_and_hold_only_commands.jsonl`

Final-stage full sequence only:

- `testdata/entry_touchdown_roll_sequence/combined_with_hold_commands.jsonl`

## 3. CAN Specification

- ALIGN request: `0x000 + joint_index`
- ALIGN result RX: `0x100 + joint_index`
- HOME jog: `0x200 + joint_index`
- SET HOME: `0x300 + joint_index`
- position command: `0x400 + joint_index`
- RUN start: `0x600 + joint_index`
- position payload: `[0,0,0,0] + little-endian float32(rad)`
- position unit: rad

## 4. Use=True / Use=False Specification

Use checkbox state is the active joint selection.

- `/cmdForJetson.position` must always be 24 values.
- CAN frames are sent only for `Use=True` joints.
- `Use=False` joints receive no `0x400+i` position frame.
- `Use=True=0,1,2,3` emits only `0x400..0x403` position frames.
- `Use=False` disconnected joints do not block RUN.
- RUN is rejected if no joint is Use=True.
- RUN is allowed only when all Use=True joints are connected, aligned, and homed.

## 5. Pre-Hardware Prohibitions

Do not:

- execute `external/can_interface`
- edit `data/reference_candidates`
- run `combined_with_hold_commands.jsonl` on first hardware trial
- run air-entry before small-angle test
- run roll body before air-entry is confirmed
- press SET HOME before HOME direction and posture are visually checked
- roll before STOP behavior is verified
- set Use=True on a missing or unverified unit
- open `can0` during dry-run or mock checks

## 6. Terminal Layout

Use separate terminals:

- Terminal A: `roscore`
- Terminal B: CAN setup and `candump`
- Terminal C: StateMachine
- Terminal D: UI
- Terminal E: `/cmdForJetson` publisher
- Terminal F: emergency/manual rostopic commands

## 7. Common Preparation

Run in every ROS terminal:

```bash
cd <repo_root>
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

Replace `<repo_root>` with this repository root.

## 8. Start roscore

Terminal A:

```bash
roscore
```

## 9. CAN Preparation

Only do this when intentionally preparing hardware CAN. Do not run this during mock or dry-run checks.

Terminal B:

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
```

Monitor CAN:

```bash
candump -tz can0
```

## 10. vcan Verification Path

Use this before hardware when a SocketCAN path is needed without hardware CAN:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan 2>/dev/null || true
sudo ip link set up vcan0
```

Start StateMachine on `vcan0`:

```bash
python tools/can_interface/statemachine/main.py \
  --can-interface socketcan \
  --can-channel vcan0 \
  --can-bitrate 500000
```

## 11. StateMachine On Hardware CAN

Only after hardware CAN preparation and operator approval:

```bash
python tools/can_interface/statemachine/main.py \
  --can-interface socketcan \
  --can-channel can0 \
  --can-bitrate 500000
```

The maintained StateMachine path is `tools/can_interface/statemachine/main.py`. Do not run the `external/` copy.

## 12. Start UI

Terminal D:

```bash
python tools/can_interface/initUI/ui.py
```

The maintained UI path is `tools/can_interface/initUI/ui.py`.

## 13. Use Setting

Set four-axis Use=True example:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:0:1'"
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:1:1'"
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:2:1'"
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:3:1'"
```

Turn unused axes OFF:

```bash
for i in $(seq 4 23); do
  rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:'$i':0'"
done
```

Set all 24 axes Use=True only when all 24 units are installed and verified:

```bash
for i in $(seq 0 23); do
  rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:'$i':1'"
done
```

## 14. RUN Negative Test

Before CONNECT/ALIGN/HOME, RUN must be rejected:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'run'"
```

Expected: no `0x400+i` position frames. If active joints are not connected/aligned/homed, RUN is rejected.

## 15. CONNECT

Current StateMachine receives connection by CAN ping RX `0x0FF`; there is no implemented global `connect` UI command in `tools/can_interface`.

The following command is a negative/compatibility probe only and is expected to be ignored unless a future implementation adds it:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'connect'"
```

Actual CONNECT confirmation must come from unit ping RX and UI state changing to Connected.

## 16. ALIGN

The current implemented command is indexed: `align:<joint_index>`.

For four active axes:

```bash
for i in $(seq 0 3); do
  rostopic pub -1 /ui/leg_command std_msgs/String "data: 'align:'$i"
done
```

A global `align` command is not currently implemented and should be treated as a negative/compatibility probe:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'align'"
```

Expected CAN for Use=True 0..3: `0x000..0x003` only.

## 17. HOME Jog

There is no implemented global `home` command. HOME jog is indexed and directional.

For one small positive jog on four active axes:

```bash
for i in $(seq 0 3); do
  rostopic pub -1 /ui/leg_command std_msgs/String "data: 'home_move:'$i':1'"
done
```

For one small negative jog:

```bash
for i in $(seq 0 3); do
  rostopic pub -1 /ui/leg_command std_msgs/String "data: 'home_move:'$i':-1'"
done
```

The following global command is not implemented and should not be used as the real HOME action:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'home'"
```

Verify HOME direction visually before continuing.

## 18. SET HOME

SET HOME is indexed.

For four active axes:

```bash
for i in $(seq 0 3); do
  rostopic pub -1 /ui/leg_command std_msgs/String "data: 'set_home:'$i"
done
```

The following global command is not currently implemented and should be treated as a negative/compatibility probe:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'set_home'"
```

Current limitation: SET HOME has no MCU ACK in this software path. The StateMachine marks `homed=True` after command send. Treat this as operator-confirmed.

## 19. RUN

After Use=True axes are connected, aligned, and homed:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'run'"
```

Expected:

- RUN start sends `0x600 + joint_index` only for Use=True axes.
- `/cmdForJetson` position frames are converted only after RUN.
- Use=False axes receive no position frames.

## 20. STOP

STOP must be tested before motion tests:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'stop'"
```

Expected:

- `is_run=False`
- later `/cmdForJetson` messages produce no position CAN frames
- STOP itself sends no position frames

## 21. /cmdForJetson Publisher

The helper publisher reads JSONL and publishes `sensor_msgs/JointState` to `/cmdForJetson`. It never opens CAN. CAN conversion is done only by the StateMachine.

General form:

```bash
python tools/publish_cmdforjetson_jsonl.py \
  --command-log <commands.jsonl> \
  --rate <hz> \
  --start-index <n> \
  --max-frames <n>
```

The JSONL record must contain one of:

- `joint_command_rad`
- `position`
- `joint_positions_rad`

Each position must contain exactly 24 values.

## 22. Small-Angle Test

If this file does not exist yet, create and validate it before running hardware:

- `testdata/hardware_bringup_4unit_small_motion/four_unit_small_motion_commands.jsonl`

Command after it exists:

```bash
python tools/publish_cmdforjetson_jsonl.py \
  --command-log testdata/hardware_bringup_4unit_small_motion/four_unit_small_motion_commands.jsonl \
  --rate 5
```

Expected:

- only Use=True axes move
- direction is correct
- STOP immediately stops later position conversion
- no unexpected sound, current, heat, or jump

Do not run air-entry until small-angle motion is verified.

## 23. Air-Entry + Hold Only

This is the first staged posture test after small-angle motion.

Use:

- `testdata/hardware_trial_air_entry_only/air_entry_and_hold_only_commands.jsonl`

It contains 135 frames: 120 air-entry + 15 touchdown hold. It contains 0 roll-body frames.

Command:

```bash
python tools/publish_cmdforjetson_jsonl.py \
  --command-log testdata/hardware_trial_air_entry_only/air_entry_and_hold_only_commands.jsonl \
  --rate 5
```

Expected:

- robot is suspended in air at start
- no joint jumps
- final hold posture is candidate02 start posture
- no roll motion occurs

## 24. Touchdown Contact Confirmation

Touchdown offset is operational height margin, not a joint command.

Use at hold posture:

- analytical minimum: +0.013 m
- recommended operational: +0.015 m
- first hardware trial: +0.020 m

For first hardware trial, use +0.020 m equivalent. Lower the robot manually while holding candidate02 start posture.

Do not continue to roll until contact, clearance, stability, sound, and current are acceptable.

## 25. Short Roll Segment

Use the canonical roll body only after small-angle, air-entry, and touchdown checks pass.

50 frames:

```bash
python tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_42c_candidate_02_softlimit_94p8/commands.jsonl \
  --rate 3 \
  --start-index 0 \
  --max-frames 50
```

100 frames:

```bash
python tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_42c_candidate_02_softlimit_94p8/commands.jsonl \
  --rate 3 \
  --start-index 0 \
  --max-frames 100
```

Stop immediately on unexpected behavior.

## 26. Full Roll Final Confirmation

Do not run this during initial hardware trial.

Final-stage command only after all earlier stages pass:

```bash
python tools/publish_cmdforjetson_jsonl.py \
  --command-log testdata/entry_touchdown_roll_sequence/combined_with_hold_commands.jsonl \
  --rate 3
```

This file includes air-entry, hold, and full candidate02 roll. It is not the first hardware command file.

## 27. Execution Prohibitions

Never:

- run `external/can_interface`
- edit `data/reference_candidates`
- run full roll before small-angle, air-entry, touchdown, and short-roll checks
- set missing units to Use=True
- press SET HOME before HOME direction/posture is visually confirmed
- run roll before STOP is tested
- open `can0` in dry-run/mock context

## 28. PASS Conditions

Before moving to the next stage, all of these must hold:

- correct Use=True axes selected
- inactive axes receive no position CAN frames
- active axes connected/aligned/homed
- RUN negative test rejects before ready state
- STOP tested and working
- no unexpected CAN IDs in `candump`
- no unexpected mechanical motion
- no excessive current, heat, sound, or vibration
- operator can physically stop the system
- all observations recorded
