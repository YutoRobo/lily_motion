# Lily 8脚ロボット コマンドリファレンス

更新日: 2026-08-12  
対象: 現行 `master` 系

この文書は、**「何をしたいか」から実行コマンドを探すための索引兼コマンド集**である。

実機の試験順序・安全判定は必ず [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md) を優先する。
データ形式は [`COMMAND_DATA_FORMAT.md`](COMMAND_DATA_FORMAT.md)、runtime構成は [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md) を参照する。

---

## 0. 最初に見る表

| やりたいこと | 参照 |
|---|---|
| repository / ROSを準備する | [1. 共通準備](#1-共通準備) |
| CAN StateMachine / UIを起動する | [2. CANを起動する](#2-canを起動する) |
| 1軸だけ小さく動かす | [3. 単軸試験](#3-単軸試験) |
| 1脚3軸を確認する | [4. 1脚試験](#4-1脚試験) |
| HOMEから回転開始姿勢へ移る | [5. Air-entry](#5-air-entry) |
| 2026-08-12 baselineの既存分割rollを使う | [6. Baseline split roll](#6-baseline-split-roll) |
| 1/4・2/4・3/4・4/4回転ファイルを作る | [7. Semantic quarter stage生成](#7-semantic-quarter-stage生成) |
| Gazeboで1/4〜4/4を選んで確認する | [8. Quarter stageをGazeboで使う](#8-quarter-stageをgazeboで使う) |
| 実機で1/4〜4/4を使う | [9. Quarter stageを実機で使う](#9-quarter-stageを実機で使う) |
| full combined sequenceを実行する | [10. Final combined](#10-final-combined) |
| 任意1軸へ波形を縮小して割り当てる | [11. Mapped-axis診断](#11-mapped-axis診断) |
| trajectory開発用に直接Gazebo replayする | [12. Development direct Gazebo](#12-development-direct-gazebo) |
| UI command / CAN IDを調べる | [付録](#付録-a-ui-command) |

---

## 1. 共通準備

### 1.1 Repository

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
git status -sb
git log -1 --oneline
```

### 1.2 ROS

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roscore
```

実機では `/use_sim_time` が `true` でないことを確認する。

```bash
rosparam get /use_sim_time 2>/dev/null || true
```

### 1.3 現行candidateを変数化

以下を同じshellで設定すると、以後のcommandが短くなる。

```bash
CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged
```

現在のstaged transport profile:

```text
resample-factor = 2
rate            = 10 Hz
```

canonical publisher:

```text
tools/publish_cmdforjetson_jsonl.py
```

このpublisherはGazebo/CANを直接選ばない。

```text
JSONL
  ↓
publish_cmdforjetson_jsonl.py
  ↓
/cmdForJetson
```

その下流だけが実機/Gazeboで異なる。

---

## 2. CANを起動する

### 2.1 実機CAN

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

### 2.2 vcan emulator

```bash
sudo modprobe vcan
ip link show vcan0 >/dev/null 2>&1 || sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

axis10 emulator:

```bash
python2 tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 10
```

StateMachine:

```bash
python2 tools/can_interface/statemachine/main.py \
  --can-interface socketcan \
  --can-channel vcan0 \
  --can-bitrate 500000
```

---

## 3. 単軸試験

初回実機確認はrolling JSONLではなく専用publisherを使う。

axis10 positive 0.002 rad:

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

negative:

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

このpublisherは対象axisだけfinite、他23軸をNaNにする。
**exactly one axisだけ `Use=True`** とする。

---

## 4. 1脚試験

例: `leg-index 3 = axes 9,10,11`。

individual positive:

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

individual negative:

```bash
python2 tools/publish_cmdforjetson_one_leg_test.py \
  --leg-index 3 \
  --mode individual \
  --direction minus \
  --amplitude-rad 0.002 \
  --step-rad 0.001 \
  --period-sec 0.500
```

individual確認後のcoordinated test:

```bash
python2 tools/publish_cmdforjetson_one_leg_test.py \
  --leg-index 3 \
  --mode coordinated \
  --direction plus \
  --amplitude-rad 0.002 \
  --step-rad 0.001 \
  --period-sec 0.500
```

3軸試験時は対象3軸だけ `Use=True`。

---

## 5. Air-entry

### 5.1 Dry-run

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/air_entry_and_hold_only_commands.jsonl" \
  --resample-factor 2 \
  --rate 10 \
  --dry-run
```

2026-08-12 baseline expected:

```text
source_frames=135
transport_frames=269
transport_sha256=e1c00e23811f841e86ca4ff3fdc9a42c380e6537f6cf9623f97334a020f5a0fa
output_topic=/cmdForJetson
dry_run=true published_count=0
```

### 5.2 Live

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/air_entry_and_hold_only_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

実機ではrobotをsuspendした状態で実施し、終了後にcontrolled touchdownを行う。

---

## 6. Baseline split roll

ここは**2026-08-12 pre-hardware baselineでGazebo PASSした既存stage**である。

```text
roll_0_50_commands.jsonl
roll_50_100_commands.jsonl
roll_100_300_commands.jsonl
roll_300_end_commands.jsonl
```

これらはsemanticな1/4・2/4ではなく、段階確認用のframe-range splitである。

0–50:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_0_50_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

50–100:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_50_100_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

100–300:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_100_300_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

300–end:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_300_end_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

これらは前stageの最終姿勢から次stageへ続けて使用するための既存validation sequenceである。

---

## 7. Semantic quarter stage生成

### 7.1 目的

`commands.jsonl` 内の連続した `roll_index` blockを意味上の各回転として認識し、次の累積ファイルを生成する。

```text
roll_to_1of4_commands.jsonl  = 回転開始 → 1/4終了
roll_to_2of4_commands.jsonl  = 回転開始 → 2/4終了
roll_to_3of4_commands.jsonl  = 回転開始 → 3/4終了
roll_to_4of4_commands.jsonl  = 回転開始 → 4/4終了
```

**2233 frameを単純に4等分しない。** `roll_index` 境界を使う。

生成tool:

```text
tools/command_generation/build_roll_quarter_stages.py
```

このtoolはJSONLを生成するだけで、ROS publishもCAN openもしない。

### 7.2 まずdry-run

空の一時directoryを用意する。

```bash
QUARTER_DIR=$(mktemp -d /tmp/lily_quarter_stages.XXXXXX)
```

境界だけ検証:

```bash
python2 tools/command_generation/build_roll_quarter_stages.py \
  --command-log "$CANDIDATE/commands.jsonl" \
  --output-dir "$QUARTER_DIR" \
  --dry-run
```

現在candidateで期待するsemantic boundary:

```text
roll_index 0 : source 0    - 559   → cumulative  560 frames
roll_index 1 : source 560  - 1119  → cumulative 1120 frames
roll_index 2 : source 1120 - 1679  → cumulative 1680 frames
roll_index 3 : source 1680 - 2232  → cumulative 2233 frames
```

source SHA256 expected:

```text
e60c9de63287c5c198e78e11c1da89475b2293e6de45950cf09f5f2c170304a5
```

### 7.3 ファイル生成

同じ空directoryへ:

```bash
python2 tools/command_generation/build_roll_quarter_stages.py \
  --command-log "$CANDIDATE/commands.jsonl" \
  --output-dir "$QUARTER_DIR"
```

生成物:

```text
$QUARTER_DIR/roll_to_1of4_commands.jsonl
$QUARTER_DIR/roll_to_2of4_commands.jsonl
$QUARTER_DIR/roll_to_3of4_commands.jsonl
$QUARTER_DIR/roll_to_4of4_commands.jsonl
$QUARTER_DIR/quarter_stage_manifest.json
```

既存出力はデフォルトでは上書きしない。

### 7.4 累積stageの注意

`roll_to_2of4` は「第2 quarterだけ」ではない。

```text
roll start → quarter 1 → quarter 2
```

を含む。

したがって:

```text
roll_to_1of4 実行直後
  ↓
roll_to_2of4 をそのまま続けて実行
```

とはしない。

1/4、2/4、3/4、4/4は**同じrolling-start postureからどこまで回すかを選ぶ独立試験**として扱う。

---

## 8. Quarter stageをGazeboで使う

semantic quarter stageは現在の2026-08-12 frozen baselineには含まれない**新しいderived validation data**である。
実機へ使う前に、まずGazeboで同じ生成ファイルを確認する。

### 8.1 Gazebo MCU node

CAN StateMachineを停止した状態でGazebo robotを起動し、Terminal A:

```bash
python2 tools/gazebo/mcu_position_interpolator_node.py \
  --input-topic /cmdForJetson \
  --interp-duration-sec 0.100 \
  --update-period-sec 0.002
```

### 8.2 例: 1/4まで

rolling-start postureまでair-entryを実行した後、Terminal B:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$QUARTER_DIR/roll_to_1of4_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

### 8.3 例: 2/4まで

同じrolling-start postureから独立に開始して:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$QUARTER_DIR/roll_to_2of4_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

3/4、4/4もfile名だけ変更する。

```text
roll_to_3of4_commands.jsonl
roll_to_4of4_commands.jsonl
```

各stageで姿勢、境界、最終holdを確認する。

---

## 9. Quarter stageを実機で使う

**現時点ではsemantic quarter filesは2026-08-12 frozen pre-hardware baselineの正式staged inputではない。**

したがって、次を満たすまでは実機へ送らない。

1. builder test PASS
2. frozen candidateでboundary / SHA確認
3. 1/4 → 2/4 → 3/4 → 4/4をGazeboで個別確認
4. generated quarter filesを固定しchecksumを記録
5. 新しいvalidation/baselineとして変更点を記録

その後の実送信command自体はGazeboと同じである。

例 2/4:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log <frozen-quarter-stage-dir>/roll_to_2of4_commands.jsonl \
  --resample-factor 2 \
  --rate 10
```

違うのは `/cmdForJetson` のconsumerだけである。

```text
Gazebo: mcu_position_interpolator_node.py
Real:   StateMachine → CAN → real MCU
```

---

## 10. Final combined

既存split rollがすべて実機PASSした後だけ使用する。

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/combined_with_hold_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

初回実機確認には使用しない。

---

## 11. Mapped-axis診断

24軸command logの任意logical axis波形を、小振幅relative motionとして1 physical axisへ割り当てるdiagnostic tool:

```text
tools/publish_cmdforjetson_mapped_axis_replay.py
```

まずdry-run:

```bash
python2 tools/publish_cmdforjetson_mapped_axis_replay.py \
  --command-log "$CANDIDATE/commands.jsonl" \
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

これはabsolute rolling posture再現ではない。waveform / ROS / CAN mapping診断用。

---

## 12. Development direct Gazebo

trajectory開発・診断・履歴再現用:

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --command-log "$CANDIDATE/commands.jsonl" \
  --strict-command-log-input \
  --rate 15 \
  --diagnose-command-log
```

これはformal hardware-equivalent pathではない。

実機とのcommand path比較では:

```text
publish_cmdforjetson_jsonl.py
→ /cmdForJetson
→ mcu_position_interpolator_node.py
```

を使う。

---

## 13. Canonical JSONL publisher 詳細

Help:

```bash
python2 tools/publish_cmdforjetson_jsonl.py --help
```

一般形:

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

accepted source position keys:

```text
joint_command_rad
position
joint_positions_rad
```

すべて24要素。

---

# 付録 A. UI command

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

global `home` / global `set_home` は実装されていない。

---

# 付録 B. CAN protocol

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

---

# 付録 C. Current baseline

```text
candidate:
v3_0_44_candidate_022_wide_urdf0p075

pre-hardware software baseline:
3ff47e223c2ba67b3f6bf62de327f71de5226d86

current transport trial profile:
factor=2 / 10 Hz

Gazebo MCU profile:
0.100 s / 0.002 s
```

関連:

- [`BASELINE.md`](BASELINE.md)
- [`BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md`](BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md)
- [`COMMAND_DATA_FORMAT.md`](COMMAND_DATA_FORMAT.md)
- [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md)
