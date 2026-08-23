# Lily Copy/Paste Commands

更新日: 2026-08-23

このページは、**説明を最小限にしてコマンドをそのままコピーして実行するためのページ**である。

詳細:

- CAN接続 / MCU Config: [`CAN_MCU_CONFIG_GUIDE.md`](CAN_MCU_CONFIG_GUIDE.md)
- 実機試験順序: [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md)
- command / CAN IDの意味: [`COMMAND_REFERENCE.md`](COMMAND_REFERENCE.md)

---

# 0. Repository確認

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
git status -sb
git log -1 --oneline
```

---

# 1. 実機CANを接続する

CAN初期設定の正本:

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

確認:

```bash
ip -details link show can0
```

CAN monitor:

```bash
candump -L can0
```

---

# 2. MCU Configを確認・変更する

## 2.1 Axis 11だけ

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion/tools/mcu_config
python2 lily_mcu_config_editor.py --interface can0 --axes 11
```

## 2.2 24軸を一覧対象にする

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion/tools/mcu_config
python2 lily_mcu_config_editor.py --interface can0 --axes 0-23
```

通常はGUIを使用する。raw `cansend` は診断用。

Axis 11 Kp READ:

```bash
cansend can0 08B#0102010000000000
```

Axis 11 Kp=500 WRITE:

```bash
cansend can0 08B#02020100F4010000
```

SoftwareConfig SAVE:

```bash
cansend can0 08B#0302000000000000
```

Axis 11 gear ratio READ:

```bash
cansend can0 08B#0101010000000000
```

Axis 11 gear ratio=30.8 WRITE:

```bash
cansend can0 08B#020101006666F641
```

HardwareConfig SAVE:

```bash
cansend can0 08B#0301000000000000
```

HardwareConfig SAVE後はMCU電源を再投入する。

---

# 3. 実機runtimeを起動する

## 3.1 Terminal 1: ROS master

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roscore
```

## 3.2 Terminal 2: CAN StateMachine

CANをSection 1でUPした後に実行する。

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

python2 tools/can_interface/statemachine/main.py \
  --can-interface socketcan \
  --can-channel can0 \
  --can-bitrate 500000
```

## 3.3 Terminal 3: UI

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
python2 tools/can_interface/initUI/ui.py
```

## 3.4 Runtime確認

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosparam get /use_sim_time 2>/dev/null || true
rostopic info /cmdForJetson
```

実機時はGazebo MCU nodeを `/cmdForJetson` へ接続しない。

---

# 4. UI command

STOP:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'stop'"
```

Axis 10 Use ON:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:10:1'"
```

Axis 10 ALIGN:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'align:10'"
```

HOME jog step:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'home_step:0.002'"
```

HOME jog +:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'home_move:10:1'"
```

HOME jog -:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'home_move:10:-1'"
```

SET HOME:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'set_home:10'"
```

RUN:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'run'"
```

Axis 10 Use OFF:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:10:0'"
```

---

# 5. 単軸試験

Axis 10 +0.002 rad:

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

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

Axis 10 -0.002 rad:

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

---

# 6. 1脚試験

例: leg-index 3 = axes 9,10,11。

Individual +:

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

Individual -:

```bash
python2 tools/publish_cmdforjetson_one_leg_test.py \
  --leg-index 3 \
  --mode individual \
  --direction minus \
  --centers-rad 0,0,0 \
  --amplitude-rad 0.002 \
  --step-rad 0.001 \
  --period-sec 0.500
```

Coordinated +:

```bash
python2 tools/publish_cmdforjetson_one_leg_test.py \
  --leg-index 3 \
  --mode coordinated \
  --direction plus \
  --amplitude-rad 0.002 \
  --step-rad 0.001 \
  --period-sec 0.500
```

---

# 7. Current staged roll

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged
```

Air-entry:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/air_entry_and_hold_only_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

Risk 0-50:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_0_50_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

Risk 50-100:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_50_100_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

Risk 100-300:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_100_300_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

Risk 300-end:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_300_end_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

Semantic 1/4:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_to_1of4_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

Semantic 2/4:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_to_2of4_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

Semantic 3/4:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_to_3of4_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

Semantic 4/4:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_to_4of4_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

Combined:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/combined_with_hold_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

実機でどこまで進めてよいかは `HARDWARE_OPERATION_PROCEDURE.md` を優先する。

---

# 8. Gazebo

Terminal 1:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roscore
```

Terminal 2:

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

python2 tools/gazebo/mcu_position_interpolator_node.py \
  --input-topic /cmdForJetson \
  --interp-duration-sec 0.100 \
  --update-period-sec 0.002
```

Gazebo時はCAN StateMachineを起動しない。
