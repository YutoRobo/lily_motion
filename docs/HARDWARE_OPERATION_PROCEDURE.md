# Lily 実機操作手順

更新日: 2026-08-12  
対象: `v3_0_44_candidate_022_wide_urdf0p075` staged hardware validation

この文書は現行実機試験の正本である。

## 1. Scope

対象runtime:

```text
staged JSONL
→ tools/publish_cmdforjetson_jsonl.py
→ /cmdForJetson
→ tools/can_interface/statemachine/StateMachine
→ CAN
→ real MCU
```

現行pre-hardware transport:

```text
resample-factor = 2
rate            = 10 Hz
```

**旧文書・旧runnerにある `--rate 3` / `--rate 5` を現行staged rollへ使用しない。**

## 2. Do not run

現行実機操作では次を使わない。

```text
archive/
external/can_interface/
tools/gazebo/mcu_position_interpolator_node.py
tools/gazebo/run_v3_0_gazebo_replay.py
```

Gazebo nodeを実機CAN StateMachineと同時に `/cmdForJetson` へ接続しない。

旧 `tools/run_hardware_staged_manual.sh` は現行common-path成立前のwrapperであり、現行entry pointではない。

## 3. Physical safety

必須:

- physical emergency isolationを即操作できる
- 可動範囲に人を入れない
- robotを安全にsuspendできるfixture
- 対象外axisを `Use=False`
- CAN cable / power / mechanical fastening確認
- 異音、衝撃、予期しないmotion、heat、vibrationで即中止
- stageごとにSTOP可能なoperator配置

PC側STOPはphysical emergency isolationの代替ではない。

## 4. Git / baseline check

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
git status -sb
git log -1 --oneline
```

実機trial recordにはcommit SHAを残す。

基準:

```text
pre-hardware software baseline:
3ff47e223c2ba67b3f6bf62de327f71de5226d86
```

masterに文書等の後続commitがある場合でも、trajectory/transport runtime差分がないことを確認する。

## 5. ROS

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roscore
```

実機では `/use_sim_time` を使用しない。

確認:

```bash
rosparam get /use_sim_time 2>/dev/null || true
```

`true`なら実機trial前に原因を確認する。

## 6. CAN

実機CANだけを使用:

```bash
ip -details link show can0
```

必要に応じて:

```bash
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

## 7. Use selection

trial対象axisだけ `Use=True`。

例 axis10だけ:

```bash
for i in $(seq 0 23); do
  rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:'$i':0'"
done

rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:10:1'"
```

one leg (axes9,10,11):

```bash
for i in $(seq 0 23); do
  rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:'$i':0'"
done

for i in 9 10 11; do
  rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:'$i':1'"
done
```

24-axis activeは各axis個別確認後だけ。

## 8. ALIGN / HOME / RUN / STOP

ALIGN:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'align'"
```

indexed ALIGN:

```bash
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

RUN:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'run'"
```

STOP:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'stop'"
```

RUNはactive axisがconnected/aligned/homedのときだけ成立することを確認する。

## 9. Single-axis validation

最初は専用publisherを使う。JSONL rollを単軸初回試験に使わない。

Positive:

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

両方PASS後だけ ±0.005 radへ進む。

各試験後STOP。

## 10. One-leg validation

axes9,10,11を例とする。

まず各axisを1本ずつsingle-axis publisherで確認する。

その後、3軸だけUse=Trueとして:

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

negative:

```bash
python2 tools/publish_cmdforjetson_one_leg_test.py \
  --leg-index 3 \
  --mode individual \
  --direction minus \
  --amplitude-rad 0.002 \
  --step-rad 0.001 \
  --period-sec 0.500
```

individual PASS後:

```bash
python2 tools/publish_cmdforjetson_one_leg_test.py \
  --leg-index 3 \
  --mode coordinated \
  --direction plus \
  --amplitude-rad 0.002 \
  --step-rad 0.001 \
  --period-sec 0.500
```

## 11. Before 24-axis staged motion

必須確認:

- 24 actuator installation
- axis mapping
- sign
- each HOME direction
- each SET HOME
- no unexpected `Use=True`
- physical suspension
- STOP test
- no Gazebo MCU subscriber
- `rostopic info /cmdForJetson` でconsumerを確認

実機時、意図したconsumerはCAN StateMachineのみ。

## 12. Dry-run the exact transport

air-entry:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/air_entry_and_hold_only_commands.jsonl \
  --resample-factor 2 \
  --rate 10 \
  --dry-run
```

baseline expected:

```text
source_frames=135
transport_frames=269
transport_sha256=e1c00e23811f841e86ca4ff3fdc9a42c380e6537f6cf9623f97334a020f5a0fa
output_topic=/cmdForJetson
dry_run=true published_count=0
```

異なる場合は実送信しない。

## 13. Air-entry

条件:

```text
robot suspended
all required axes aligned/homed
HOME logical posture = all 0 rad
RUN accepted
physical emergency path ready
```

実行:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/air_entry_and_hold_only_commands.jsonl \
  --resample-factor 2 \
  --rate 10
```

確認:

- initial jumpなし
- intended direction
- cable/fixture interferenceなし
- abnormal current/sound/vibration/heatなし
- final rolling-start postureで安定

publisher終了時にzero/STOPは送らない。実MCUがlast targetを保持するかをこのstageで必ず確認する。

異常時はSTOPし、touchdownへ進まない。

## 14. Controlled touchdown

air-entry final postureを保持したまま、fixture/base heightを制御して床へ接地させる。

確認:

- intended foot contact
- link/floor clearance
- unexpected slideなし
- supportが安定
- current/sound/vibration/heat
- actual base/floor marginを記録

joint commandで「touchdown offset」を勝手に追加しない。

## 15. Split roll

MCU/StateMachine sessionを不用意に再初期化せず、前stage final postureから続ける。

### Roll 0–50

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/roll_0_50_commands.jsonl \
  --resample-factor 2 \
  --rate 10
```

### Roll 50–100

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/roll_50_100_commands.jsonl \
  --resample-factor 2 \
  --rate 10
```

### Roll 100–300

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/roll_100_300_commands.jsonl \
  --resample-factor 2 \
  --rate 10
```

### Roll 300–end

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/roll_300_end_commands.jsonl \
  --resample-factor 2 \
  --rate 10
```

各stageでPASS判定後に次へ進む。

## 16. Final combined sequence

split rollがすべて実機PASSした後だけ:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/combined_with_hold_commands.jsonl \
  --resample-factor 2 \
  --rate 10
```

初回実機試験に使用しない。

## 17. Immediate stop criteria

1つでも該当したらstage中止:

- unexpected axis motion
- posture jump
- unexpected contact
- cable pull
- mechanism interference
- abnormal sound
- abnormal current
- abnormal vibration
- abnormal temperature
- `0x0EE`
- CAN send/interface error
- `/cmdForJetson` unexpected extra subscriber
- timing/rate concern
- operator concern

異常後にそのまま再RUNしない。logとphysical stateを確認する。

## 18. Record for every trial

最低限:

```text
date/time
operator
git commit
candidate path
staged file
resample factor
transport rate
active axes
ALIGN/HOME result
CAN log
observed sign/direction
current/sound/vibration/temperature
STOP behavior
PASS/FAIL
failure reason
```

## 19. Related documents

- [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md)
- [`BASELINE.md`](BASELINE.md)
- [`HARDWARE_PRETEST_STATUS.md`](HARDWARE_PRETEST_STATUS.md)
- [`Lily_8leg_Robot_Command_Reference.md`](Lily_8leg_Robot_Command_Reference.md)
