# Hardware Operation Procedure

更新日: 2026-08-04

この文書は、現行`master`でLily 8脚ロボットを段階的に確認するための実機操作手順である。順序を飛ばしてfull rollへ進んではならない。

## 1. Core Rules

- dry-run、mock、vcan試験では`can0`を開かない。
- 実機CAN送信は、操作者が意図して実機試験を行う場合だけ許可する。
- 現行CAN実行対象は`tools/can_interface/`だけである。
- `external/can_interface/`を実行しない。
- 本番位置指令入力は`/cmdForJetson`だけである。
- 削除済みの`/can/axis_command`経路を使用しない。
- `data/reference_candidates/`の正式候補を直接編集しない。
- 対象外軸は`Use=False`にする。
- 実機単軸試験では機体または対象脚を浮かせる。
- 非常停止またはUI STOPを直ちに操作できる状態にする。
- full sequenceは全段階の最後にだけ実施する。

## 2. Current Status And Next Stage

2026-08-04時点:

```text
vcan axis10 single-axis: PASS
vcan axis10,11,12 fan-out: PASS
real axis10 +0.002 rad: provisional PASS
```

次の実機確認順:

```text
axis10 negative 0.002 rad
→ axis10 positive/negative 0.005 rad
→ one complete leg, three axes
→ air-entry and hold
→ touchdown
→ split roll
→ final combined sequence
```

## 3. Maintained Runtime Files

```text
tools/can_interface/statemachine/main.py
tools/can_interface/initUI/ui.py
tools/publish_cmdforjetson_single_axis_test.py
tools/publish_cmdforjetson_jsonl.py
```

Current reference candidate:

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
```

Candidate-specific decision:

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
  pre_hardware_decision.md
```

## 4. CAN Specification

| Purpose | CAN ID |
|---|---:|
| standby heartbeat RX | `0x0FF` |
| ALIGN request TX | `0x000 + axis` |
| ALIGN result RX | `0x100 + axis` |
| HOME jog TX | `0x200 + axis` |
| SET HOME TX | `0x300 + axis` |
| POSITION TX | `0x400 + axis` |
| RUN TX | `0x600 + axis` |

Position payload:

```text
[0,0,0,0] + little-endian float32(rad)
```

Axis10 example:

```text
ALIGN:    0x00A
SET HOME: 0x30A
POSITION: 0x40A
RUN:      0x60A
```

## 5. ROS Interface

### UI command

```text
Topic: /ui/leg_command
Message: std_msgs/String
```

Implemented commands include:

```text
use:<axis>:0|1
align
align:<axis>
home_move:<axis>:-1|1
home_step:<rad>
set_home:<axis>
run
stop
```

`align` applies to all current`Use=True` axes. `align:<axis>` requests one indexed axis.

There is no implemented global `home` or global `set_home` command. HOME jog and SET HOME are indexed.

### Position command

```text
Topic: /cmdForJetson
Message: sensor_msgs/JointState
position: exactly 24 elements
unit: rad
```

StateMachine sends RUN and POSITION only to`Use=True` axes.

## 6. Use=True / Use=False

- `Use=True`: active axis; included in ALIGN, HOME, RUN, and POSITION safety gates.
- `Use=False`: inactive axis; receives no RUN or POSITION frame.
- RUN is rejected when no axis is active.
- RUN is accepted only when all active axes are aligned and homed in the current session.
- disconnected inactive axes do not block RUN.
- changing Use selection after the session starts should be avoided; STOP and restart the session instead.

## 7. Terminal Layout

Use separate terminals.

| Terminal | Purpose |
|---|---|
| A | `roscore` |
| B | CAN setup and `candump` |
| C | StateMachine |
| D | UI |
| E | `/cmdForJetson` publisher |
| F | manual STOP / status checks |

## 8. Common ROS Preparation

In every ROS terminal:

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

Confirm the repository and branch:

```bash
git status -sb
git log -1 --oneline
```

Record the commit used for every hardware test.

## 9. Start roscore

Terminal A:

```bash
roscore
```

## 10. vcan Verification Before Hardware

Create or reuse`vcan0`:

```bash
sudo modprobe vcan
ip link show vcan0 >/dev/null 2>&1 || sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
ip -details link show vcan0
```

Monitor:

```bash
candump -L vcan0
```

Start StateMachine on`vcan0`:

```bash
python2 tools/can_interface/statemachine/main.py \
  --can-interface socketcan \
  --can-channel vcan0 \
  --can-bitrate 500000
```

Do not replace`vcan0`with`can0`during this check.

## 11. Prepare Hardware CAN

Run only when intentionally starting a hardware trial.

Terminal B:

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0
```

Start a timestamped log:

```bash
candump -L can0 | tee /tmp/lily_hardware_can.log
```

Confirm visually that the active channel is`can0`, not`vcan0`.

## 12. Start StateMachine On Hardware CAN

Terminal C:

```bash
python2 tools/can_interface/statemachine/main.py \
  --can-interface socketcan \
  --can-channel can0 \
  --can-bitrate 500000
```

Expected startup log includes:

```text
CAN bus ready: interface=socketcan channel=can0 bitrate=500000
StateMachine initialized. Listening /ui/leg_command and /cmdForJetson
```

Stop immediately if the channel or bitrate is not as intended.

## 13. Start UI

Terminal D:

```bash
python2 tools/can_interface/initUI/ui.py
```

The UI requests operations through`/ui/leg_command`. CAN IDs and safety gates remain owned by StateMachine.

## 14. Confirm Connection

Connection is discovered from MCU standby heartbeat:

```text
0x0FF payload data[0] = axis
```

There is no required global`connect`command. Confirm that the intended axis becomes Connected in the UI or StateMachine log.

After ALIGN succeeds, standby heartbeat may stop. That alone is not a disconnection.

## 15. Select Use Axes

### Axis10 only

```bash
for i in $(seq 0 23); do
  rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:'$i':0'"
done

rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:10:1'"
```

Verify in the UI that only axis10 is active.

### One complete leg example

Axis numbering follows:

```text
axis = 3 × leg_index + joint_index
```

For axes9,10,11:

```bash
for i in $(seq 0 23); do
  rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:'$i':0'"
done

for i in 9 10 11; do
  rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:'$i':1'"
done
```

Do not set all 24 axes`Use=True`until every actuator is installed and individually verified.

## 16. RUN Negative Test

Before ALIGN and SET HOME, RUN must be rejected.

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'run'"
```

Expected:

- RUN rejected
- no`0x600 + axis`
- no`0x400 + axis`

Do not continue if RUN is accepted before active axes are ready.

## 17. ALIGN

### All Use=True axes

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'align'"
```

### One indexed axis

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'align:10'"
```

Expected axis10 traffic:

```text
TX 0x00A
RX 0x10A with success flag
```

On initialization error, review`0x0EE`, wait for standby heartbeat to return, and retry only the failed axis. Do not bypass the error latch.

## 18. HOME Jog

HOME jog is indexed and directional.

Set a conservative step:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'home_step:0.002'"
```

Positive jog:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'home_move:10:1'"
```

Negative jog:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'home_move:10:-1'"
```

Confirm the physical direction visually. Do not infer up/down or forward/back only from the sign; the visible direction depends on axis, URDF joint axis, mounting, and sign convention.

## 19. SET HOME

SET HOME is indexed.

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'set_home:10'"
```

Current limitation:

- this path has no separate MCU SET HOME ACK
- StateMachine marks the axis homed after successful command send
- the operator must visually confirm posture before RUN

For one complete leg:

```bash
for i in 9 10 11; do
  rostopic pub -1 /ui/leg_command std_msgs/String "data: 'set_home:'$i"
done
```

## 20. RUN

After all active axes are connected, aligned, and homed:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'run'"
```

Expected:

- one`0x600 + axis`frame per active axis
- StateMachine enters RUN
- later`/cmdForJetson`messages are converted only for active axes

RUN is not retransmitted on every position sample.

## 21. STOP

Test STOP before every motion stage.

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'stop'"
```

Expected:

-`is_run=False`
- later`/cmdForJetson`messages produce no POSITION frames
- active axes return to Homed display state on the PC side

STOP does not replace physical emergency isolation. Keep the physical stop path ready.

## 22. Single-Axis Publisher

The publisher never opens CAN and never issues ALIGN, HOME, RUN, or STOP.

### Positive 0.002 rad

```bash
python2 tools/publish_cmdforjetson_single_axis_test.py \
  --axis 10 \
  --direction plus \
  --amplitude-rad 0.002 \
  --step-rad 0.001 \
  --period-sec 0.500 \
  --start-hold-sec 1.000 \
  --peak-hold-sec 1.000 \
  --end-hold-sec 1.000
```

Command sequence:

```text
0.000 → 0.001 → 0.002 → 0.001 → 0.000 rad
```

### Negative 0.002 rad

Run only after the positive test passes.

```bash
python2 tools/publish_cmdforjetson_single_axis_test.py \
  --axis 10 \
  --direction minus \
  --amplitude-rad 0.002 \
  --step-rad 0.001 \
  --period-sec 0.500 \
  --start-hold-sec 1.000 \
  --peak-hold-sec 1.000 \
  --end-hold-sec 1.000
```

Command sequence:

```text
0.000 → -0.001 → -0.002 → -0.001 → 0.000 rad
```

### Positive and negative 0.005 rad

Run only after both`+/-0.002 rad`tests pass.

```bash
python2 tools/publish_cmdforjetson_single_axis_test.py \
  --axis 10 --direction plus \
  --amplitude-rad 0.005 --step-rad 0.001 \
  --period-sec 0.500 \
  --start-hold-sec 1.000 \
  --peak-hold-sec 1.000 \
  --end-hold-sec 1.000

python2 tools/publish_cmdforjetson_single_axis_test.py \
  --axis 10 --direction minus \
  --amplitude-rad 0.005 --step-rad 0.001 \
  --period-sec 0.500 \
  --start-hold-sec 1.000 \
  --peak-hold-sec 1.000 \
  --end-hold-sec 1.000
```

Issue STOP after each test and inspect the CAN log.

## 23. Single-Axis PASS Conditions

- only intended axis is`Use=True`
- RUN ID is only`0x600 + target axis`
- POSITION ID is only`0x400 + target axis`
- commanded direction matches the observed logical direction
- motion returns to center
- no other axis moves
- no`0x0EE`
- no`pc_send_error`
- no`can_interface_error`
- no abnormal sound, shock, current, heat, or vibration

## 24. One-Leg Three-Axis Test

Proceed only after single-axis positive/negative tests pass.

Recommended first one-leg test:

1. select exactly the three axes of one leg
2. ALIGN all selected axes
3. confirm HOME direction for each axis separately
4. SET HOME each axis
5. RUN
6. send one small axis motion at a time using the single-axis publisher while the other two axes remain finite and safe through the selected test method
7. verify STOP
8. only then test coordinated three-axis commands

A coordinated one-leg publisher or validated JSONL must use a full 24-element`JointState`. Do not improvise direct CAN commands.

## 25. JSONL Publisher

General form:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log <commands.jsonl> \
  --rate <hz> \
  --start-index <n> \
  --max-frames <n>
```

Accepted position keys:

```text
joint_command_rad
position
joint_positions_rad
```

Every frame must contain exactly 24 positions.

## 26. Air-Entry And Hold

Proceed only after one-leg three-axis confirmation.

Command log:

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
  staged/air_entry_and_hold_only_commands.jsonl
```

Command:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/air_entry_and_hold_only_commands.jsonl \
  --rate 5
```

Conditions:

- robot suspended in air at start
- no joint jump
- no unexpected contact
- final posture held stably
- no roll-body motion

## 27. Touchdown Confirmation

Touchdown height margin is an operational fixture/base-height condition, not an encoded joint offset.

At the final air-entry hold posture:

- lower the robot under controlled support
- confirm intended contact sequence
- confirm foot and link clearance
- confirm stable support and no unexpected slide
- monitor current, sound, vibration, and temperature
- record the actual base/floor height margin used

Do not continue to roll until contact is accepted.

## 28. Split Roll Stages

Use each dedicated staged file. Do not replace these with a start-index guess when a frozen staged file exists.

### Roll 0–50

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/roll_0_50_commands.jsonl \
  --rate 3
```

### Roll 50–100

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/roll_50_100_commands.jsonl \
  --rate 3
```

### Roll 100–300

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/roll_100_300_commands.jsonl \
  --rate 3
```

### Roll 300–end

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/roll_300_end_commands.jsonl \
  --rate 3
```

`roll_300_end`is a long final segment and must not be used before all shorter segments pass.

## 29. Final Combined Sequence

Final confirmation only:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/combined_with_hold_commands.jsonl \
  --rate 3
```

This file contains:

```text
air-entry + hold + complete 2233-frame roll body
```

Do not run it before all split roll stages pass.

## 30. Immediate Stop Conditions

Stop the current stage when any of the following occurs:

- unexpected axis motion
- posture jump or discontinuity
- failure to return to center
- unexpected contact or link interference
- abnormal sound, shock, current, vibration, or heat
- CAN frame for a`Use=False`axis
-`0x0EE`
- CAN send error
- CAN interface error
- command timing or rate concern
- UI state mismatch
- operator concern

After abnormal STOP, do not immediately restart. Save logs and inspect the cause.

## 31. Record For Every Hardware Trial

Record at least:

```text
date and time
operator
Git commit
Jetson/PC and OS
CAN interface and bitrate
active axes
ALIGN/HOME result
command file or publisher arguments
rate and hold values
CAN log path
observed motion direction
STOP result
sound/current/temperature observations
PASS/FAIL and reason
```

## 32. Related Documents

- [`../README.md`](../README.md)
- [`HARDWARE_PRETEST_STATUS.md`](HARDWARE_PRETEST_STATUS.md)
- [`Lily_8leg_Robot_Command_Reference.md`](Lily_8leg_Robot_Command_Reference.md)
- [`../data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/pre_hardware_decision.md`](../data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/pre_hardware_decision.md)
