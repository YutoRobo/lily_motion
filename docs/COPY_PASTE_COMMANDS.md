# Lily Copy/Paste Commands

更新日: 2026-08-12

このページは、**説明を最小限にしてコマンドをそのままコピーして実行するためのページ**である。

詳細な意味・安全手順は次を参照する。

- [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md)
- [`COMMAND_REFERENCE.md`](COMMAND_REFERENCE.md)
- [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md)

**GazeboではCAN StateMachineを起動しない。実機CANではGazebo MCU nodeを起動しない。**

---

# 1. Gazebo

Gazebo上のLily本体・joint controllerが起動済みの状態から使用する。

## 1.1 Terminal 1: ROS master

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roscore
```

## 1.2 Terminal 2: Gazebo MCU-equivalent node

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

python2 tools/gazebo/mcu_position_interpolator_node.py \
  --input-topic /cmdForJetson \
  --interp-duration-sec 0.100 \
  --update-period-sec 0.002
```

## 1.3 `/cmdForJetson` consumer確認

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
rostopic info /cmdForJetson
```

Gazebo時は `mcu_position_interpolator_node.py` がconsumerであり、CAN StateMachineは接続しない。

---

## 1.4 Air-entryだけ

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged

python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/air_entry_and_hold_only_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

---

## 1.5 Risk-splitを最初から最後まで

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged

# HOME状態から開始
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/air_entry_and_hold_only_commands.jsonl" \
  --resample-factor 2 \
  --rate 10

python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_0_50_commands.jsonl" \
  --resample-factor 2 \
  --rate 10

python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_50_100_commands.jsonl" \
  --resample-factor 2 \
  --rate 10

python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_100_300_commands.jsonl" \
  --resample-factor 2 \
  --rate 10

python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_300_end_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

---

## 1.6 1/4回転

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged

# GazeboをHOME状態に戻してから実行
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/air_entry_and_hold_only_commands.jsonl" \
  --resample-factor 2 \
  --rate 10

python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_to_1of4_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

## 1.7 2/4回転

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged

# GazeboをHOME状態に戻してから実行
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/air_entry_and_hold_only_commands.jsonl" \
  --resample-factor 2 \
  --rate 10

python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_to_2of4_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

## 1.8 3/4回転

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged

# GazeboをHOME状態に戻してから実行
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/air_entry_and_hold_only_commands.jsonl" \
  --resample-factor 2 \
  --rate 10

python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_to_3of4_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

## 1.9 4/4回転

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged

# GazeboをHOME状態に戻してから実行
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/air_entry_and_hold_only_commands.jsonl" \
  --resample-factor 2 \
  --rate 10

python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_to_4of4_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

---

## 1.10 Combined full sequence

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged

# HOME状態から開始
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/combined_with_hold_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

---

## 1.11 Development direct Gazebo replay

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075

python tools/gazebo/run_v3_0_gazebo_replay.py \
  --command-log "$CANDIDATE/commands.jsonl" \
  --strict-command-log-input \
  --rate 15 \
  --diagnose-command-log
```

---

# 2. 実機CAN

実機では物理非常停止を使用可能な状態にし、[`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md) のstage順序を守る。

## 2.1 Terminal 1: ROS master

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roscore
```

## 2.2 Terminal 2: CAN StateMachine

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

ip -details link show can0

python2 tools/can_interface/statemachine/main.py \
  --can-interface socketcan \
  --can-channel can0 \
  --can-bitrate 500000
```

## 2.3 Terminal 3: UI

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

python2 tools/can_interface/initUI/ui.py
```

## 2.4 Terminal 4: CAN monitor

```bash
candump -L can0
```

## 2.5 実機runtime確認

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

rosparam get /use_sim_time 2>/dev/null || true
rostopic info /cmdForJetson
```

実機時はCAN StateMachineが `/cmdForJetson` のconsumerであり、Gazebo MCU nodeは接続しない。

---

## 2.6 STOP

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'stop'"
```

---

## 2.7 Axis 10: Use / ALIGN / HOME jog / SET HOME / RUN

Use ON:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:10:1'"
```

ALIGN:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'align:10'"
```

HOME jog step:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'home_step:0.002'"
```

HOME jog +:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'home_move:10:1'"
```

HOME jog -:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'home_move:10:-1'"
```

SET HOME:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'set_home:10'"
```

RUN:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'run'"
```

Use OFF:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:10:0'"
```

---

## 2.8 Axis 10 +0.002 rad

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

## 2.9 Axis 10 -0.002 rad

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

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

## 2.10 1脚 individual +

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

# leg-index 3 = axes 9,10,11
python2 tools/publish_cmdforjetson_one_leg_test.py \
  --leg-index 3 \
  --mode individual \
  --direction plus \
  --centers-rad 0,0,0 \
  --amplitude-rad 0.002 \
  --step-rad 0.001 \
  --period-sec 0.500
```

## 2.11 1脚 individual -

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

# leg-index 3 = axes 9,10,11
python2 tools/publish_cmdforjetson_one_leg_test.py \
  --leg-index 3 \
  --mode individual \
  --direction minus \
  --centers-rad 0,0,0 \
  --amplitude-rad 0.002 \
  --step-rad 0.001 \
  --period-sec 0.500
```

## 2.12 1脚 coordinated +

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

# leg-index 3 = axes 9,10,11
python2 tools/publish_cmdforjetson_one_leg_test.py \
  --leg-index 3 \
  --mode coordinated \
  --direction plus \
  --amplitude-rad 0.002 \
  --step-rad 0.001 \
  --period-sec 0.500
```

---

## 2.13 Air-entry dry-run

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged

python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/air_entry_and_hold_only_commands.jsonl" \
  --resample-factor 2 \
  --rate 10 \
  --dry-run
```

## 2.14 Air-entry live

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged

# 24軸のUse / ALIGN / HOME / RUN完了後、robotをsuspendした状態で実行
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/air_entry_and_hold_only_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

---

## 2.15 Risk-split 0-50

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged

python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_0_50_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

## 2.16 Risk-split 50-100

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged

python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_50_100_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

## 2.17 Risk-split 100-300

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged

python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_100_300_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

## 2.18 Risk-split 300-end

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged

python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_300_end_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

---

## 2.19 実機 1/4

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged

# risk-split full pathが実機PASSした後の独立trial
# HOME → suspended air-entry → touchdown → 1/4
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/air_entry_and_hold_only_commands.jsonl" \
  --resample-factor 2 \
  --rate 10

# ここでcontrolled touchdown
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_to_1of4_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

## 2.20 実機 2/4

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged

# risk-split full pathが実機PASSした後の独立trial
# HOME → suspended air-entry → touchdown → 2/4
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/air_entry_and_hold_only_commands.jsonl" \
  --resample-factor 2 \
  --rate 10

# ここでcontrolled touchdown
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_to_2of4_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

## 2.21 実機 3/4

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged

# risk-split full pathが実機PASSした後の独立trial
# HOME → suspended air-entry → touchdown → 3/4
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/air_entry_and_hold_only_commands.jsonl" \
  --resample-factor 2 \
  --rate 10

# ここでcontrolled touchdown
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_to_3of4_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

## 2.22 実機 4/4

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged

# risk-split full pathが実機PASSした後の独立trial
# HOME → suspended air-entry → touchdown → 4/4
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/air_entry_and_hold_only_commands.jsonl" \
  --resample-factor 2 \
  --rate 10

# ここでcontrolled touchdown
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_to_4of4_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

---

## 2.23 Final combined sequence

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged

# risk-split full pathが実機PASSした後のみ
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/combined_with_hold_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

---

# 3. よく使う確認コマンド

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
git status -sb
git log -1 --oneline
```

```bash
CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
wc -l "$CANDIDATE/commands.jsonl"
sha256sum "$CANDIDATE/commands.jsonl"
```

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
rostopic info /cmdForJetson
```

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
rostopic echo -n 1 /cmdForJetson
```
