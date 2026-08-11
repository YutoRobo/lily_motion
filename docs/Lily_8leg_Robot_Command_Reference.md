# Lily 8脚ロボット コマンド集

更新日: 2026-08-12  
対象: 現行 `master`

この文書は現行softwareの実行コマンド集である。実機の段階順序・安全判定は [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md) を優先する。

## 1. Repository / ROS

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
git status -sb
git log -1 --oneline

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

roscore
```

## 2. Canonical position path

```text
publisher
→ /cmdForJetson
  sensor_msgs/JointState
  position[24] [rad]
→ consumer
```

Hardware consumer:

```text
StateMachine → CAN → real MCU
```

Gazebo hardware-equivalent consumer:

```text
mcu_position_interpolator_node.py → Gazebo
```

詳細:

- [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md)

## 3. vcan

```bash
sudo modprobe vcan
ip link show vcan0 >/dev/null 2>&1 || sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
candump -L vcan0
```

Emulator axis10:

```bash
python2 tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 10
```

axes10,11,12:

```bash
python2 tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 10,11,12
```

StateMachine vcan:

```bash
python2 tools/can_interface/statemachine/main.py \
  --can-interface socketcan \
  --can-channel vcan0 \
  --can-bitrate 500000
```

## 4. Hardware CAN

```bash
ip -details link show can0
candump -L can0
```

StateMachine:

```bash
python2 tools/can_interface/statemachine/main.py \
  --can-interface socketcan \
  --can-channel can0 \
  --can-bitrate 500000
```

UI:

```bash
python2 tools/can_interface/initUI/ui.py
```

## 5. UI commands

Use:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:10:1'"
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:10:0'"
```

ALIGN:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'align'"
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'align:10'"
```

HOME jog:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'home_step:0.002'"
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'home_move:10:1'"
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'home_move:10:-1'"
```

SET HOME:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'set_home:10'"
```

RUN / STOP:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'run'"
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'stop'"
```

global `home` / global `set_home` は実装されていない。HOME jogとSET HOMEはaxis indexed。

## 6. CAN protocol summary

| purpose | ID |
|---|---:|
| standby heartbeat RX | `0x0FF` |
| ALIGN request TX | `0x000 + axis` |
| ALIGN result RX | `0x100 + axis` |
| HOME jog TX | `0x200 + axis` |
| SET HOME TX | `0x300 + axis` |
| POSITION TX | `0x400 + axis` |
| RUN TX | `0x600 + axis` |
| error RX | `0x0EE` |

POSITION payload:

```text
[0,0,0,0] + little-endian float32(position_rad)
```

## 7. Single-axis publisher

Positive 0.002:

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

Negative:

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

対象axisだけfinite、他23軸はNaN safety mask。

## 8. One-leg publisher

leg-index 3 = axes9,10,11。

individual plus:

```bash
python2 tools/publish_cmdforjetson_one_leg_test.py \
  --leg-index 3 \
  --mode individual \
  --direction plus \
  --centers-rad 0,0,0 \
  --amplitude-rad 0.002 \
  --step-rad 0.001 \
  --period-sec 0.500
```

individual minus:

```bash
python2 tools/publish_cmdforjetson_one_leg_test.py \
  --leg-index 3 \
  --mode individual \
  --direction minus \
  --amplitude-rad 0.002 \
  --step-rad 0.001 \
  --period-sec 0.500
```

coordinated:

```bash
python2 tools/publish_cmdforjetson_one_leg_test.py \
  --leg-index 3 \
  --mode coordinated \
  --direction plus \
  --amplitude-rad 0.002 \
  --step-rad 0.001 \
  --period-sec 0.500
```

3軸試験時はその3軸だけUse=True。

## 9. Mapped-axis diagnostic publisher

24軸command logの任意logical axis波形を、小振幅relative motionとして1 physical axisへ割り当てる診断tool:

```text
tools/publish_cmdforjetson_mapped_axis_replay.py
```

最初にdry-run:

```bash
python2 tools/publish_cmdforjetson_mapped_axis_replay.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/commands.jsonl \
  --logical-axis 0 \
  --physical-axis 10 \
  --confirm-physical-axis 10 \
  --rate 5 \
  --scale 0.01 \
  --limit-rad 0.005 \
  --return-step-rad 0.001 \
  --max-frames 50 \
  --dry-run
```

これはabsolute rolling posture再現ではない。waveform/ROS/CAN mapping診断用。

## 10. Canonical JSONL publisher

Help:

```bash
python2 tools/publish_cmdforjetson_jsonl.py --help
```

Current staged profile:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log <commands.jsonl> \
  --resample-factor 2 \
  --rate 10
```

Dry-run:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log <commands.jsonl> \
  --resample-factor 2 \
  --rate 10 \
  --dry-run
```

source subset:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log <commands.jsonl> \
  --start-index 0 \
  --max-frames 10 \
  --resample-factor 2 \
  --rate 10
```

Accepted source keys:

```text
joint_command_rad
position
joint_positions_rad
```

各source recordは24要素。

## 11. Current staged files

```text
BASE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged
```

```text
$BASE/air_entry_and_hold_only_commands.jsonl
$BASE/roll_0_50_commands.jsonl
$BASE/roll_50_100_commands.jsonl
$BASE/roll_100_300_commands.jsonl
$BASE/roll_300_end_commands.jsonl
$BASE/combined_with_hold_commands.jsonl
```

## 12. Air-entry

Dry-run:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/air_entry_and_hold_only_commands.jsonl \
  --resample-factor 2 \
  --rate 10 \
  --dry-run
```

Live:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/air_entry_and_hold_only_commands.jsonl \
  --resample-factor 2 \
  --rate 10
```

## 13. Split roll

0–50:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/roll_0_50_commands.jsonl \
  --resample-factor 2 \
  --rate 10
```

50–100:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/roll_50_100_commands.jsonl \
  --resample-factor 2 \
  --rate 10
```

100–300:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/roll_100_300_commands.jsonl \
  --resample-factor 2 \
  --rate 10
```

300–end:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/roll_300_end_commands.jsonl \
  --resample-factor 2 \
  --rate 10
```

## 14. Final combined

split roll実機PASS後だけ:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/combined_with_hold_commands.jsonl \
  --resample-factor 2 \
  --rate 10
```

## 15. Hardware-equivalent Gazebo

CAN StateMachineを停止した状態で、Gazebo robotを起動。

Terminal A:

```bash
python2 tools/gazebo/mcu_position_interpolator_node.py \
  --input-topic /cmdForJetson \
  --interp-duration-sec 0.100 \
  --update-period-sec 0.002
```

Terminal B:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log <staged-file> \
  --resample-factor 2 \
  --rate 10
```

split stage間でTerminal Aを再起動しない。

## 16. Development direct Gazebo replay

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --command-log <commands.jsonl> \
  --strict-command-log-input \
  --rate 15 \
  --diagnose-command-log
```

用途はtrajectory開発・診断・履歴再現。formal hardware-equivalent pathではない。

## 17. Current baseline

```text
candidate:
v3_0_44_candidate_022_wide_urdf0p075

pre-hardware software baseline:
3ff47e223c2ba67b3f6bf62de327f71de5226d86

transport:
factor=2 / 10 Hz

Gazebo MCU:
0.100 s / 0.002 s
```

- [`BASELINE.md`](BASELINE.md)
- [`BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md`](BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md)
