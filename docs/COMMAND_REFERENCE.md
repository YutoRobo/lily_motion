# Lily 8脚ロボット コマンドリファレンス

更新日: 2026-08-12  
対象: 現行 `master` 系

この文書は、**「何をしたいか」から実行コマンドを探すための索引兼コマンド集**である。

実機の安全手順は必ず [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md) を優先する。  
データ形式は [`COMMAND_DATA_FORMAT.md`](COMMAND_DATA_FORMAT.md)、runtime構成は [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md) を参照する。

---

## 0. 最初に見る表

| やりたいこと | 使うもの |
|---|---|
| repository / ROSを準備する | [1. 共通準備](#1-共通準備) |
| CAN StateMachine / UIを起動する | [2. CAN / UI](#2-can--ui) |
| 1軸だけ小さく動かす | [3. 単軸試験](#3-単軸試験) |
| 1脚3軸を確認する | [4. 1脚試験](#4-1脚試験) |
| HOMEから回転開始姿勢へ移る | [5. Air-entry](#5-air-entry) |
| 初回実機で安全にrollを刻む | [6. Risk-split roll](#6-risk-split-roll) |
| 1/4だけ回す | `roll_to_1of4_commands.jsonl` |
| 2/4まで回す | `roll_to_2of4_commands.jsonl` |
| 3/4まで回す | `roll_to_3of4_commands.jsonl` |
| 4/4まで回す | `roll_to_4of4_commands.jsonl` |
| Gazeboで1/4〜4/4を確認する | [7. Semantic quarter](#7-semantic-quarter) |
| 実機で1/4〜4/4を使う | [8. 実機semantic quarter](#8-実機semantic-quarter) |
| full combined sequenceを実行する | [9. Final combined](#9-final-combined) |
| quarter fileを再生成・検証する | [10. Builder / checksum](#10-builder--checksum) |
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

### 1.3 Candidate path

```bash
CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged
```

現行transport profile:

```text
resample-factor = 2
rate            = 10 Hz
```

canonical publisher:

```text
tools/publish_cmdforjetson_jsonl.py
```

共通path:

```text
JSONL
  ↓
publish_cmdforjetson_jsonl.py
  ↓
/cmdForJetson
  ├→ REAL: StateMachine → CAN → real MCU
  └→ GAZEBO: mcu_position_interpolator_node.py → Gazebo
```

publisher自身はGazebo / realを選ばない。

---

## 2. CAN / UI

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

### 2.2 Gazebo時

実機CAN StateMachineは停止する。

```bash
python2 tools/gazebo/mcu_position_interpolator_node.py \
  --input-topic /cmdForJetson \
  --interp-duration-sec 0.100 \
  --update-period-sec 0.002
```

**Gazebo MCU nodeと実機CAN StateMachineを同時に `/cmdForJetson` へ接続しない。**

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
  --centers-rad 0,0,0 \
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

HOMEはlogical originであり、現在は全24軸0 rad。  
Air-entryはHOMEからrolling-start postureへのJSONL遷移である。

### Dry-run

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/air_entry_and_hold_only_commands.jsonl" \
  --resample-factor 2 \
  --rate 10 \
  --dry-run
```

2026-08-12 expected:

```text
source_frames=135
transport_frames=269
transport_sha256=e1c00e23811f841e86ca4ff3fdc9a42c380e6537f6cf9623f97334a020f5a0fa
output_topic=/cmdForJetson
dry_run=true published_count=0
```

### Live

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/air_entry_and_hold_only_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

実機ではrobotをsuspendした状態で実施し、終了後にjoint commandを変えずcontrolled touchdownする。

---

## 6. Risk-split roll

これは**初回実機の安全進行用**である。

```text
roll_0_50_commands.jsonl
roll_50_100_commands.jsonl
roll_100_300_commands.jsonl
roll_300_end_commands.jsonl
```

semanticな1/4・2/4ではない。前stageの最終姿勢から次stageへ続ける。

### 0–50

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_0_50_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

### 50–100

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_50_100_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

### 100–300

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_100_300_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

### 300–end

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_300_end_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

初回実機では、この4段階を飛ばしてsemantic quarter/full combinedへ進まない。

---

## 7. Semantic quarter

### 7.1 正式frozen files

```text
$STAGED/roll_to_1of4_commands.jsonl
$STAGED/roll_to_2of4_commands.jsonl
$STAGED/roll_to_3of4_commands.jsonl
$STAGED/roll_to_4of4_commands.jsonl
$STAGED/quarter_stage_manifest.json
```

2026-08-12 data-freeze commit:

```text
2e42343dccf3b56066cdcc97e011dca328388a20
```

境界:

```text
1/4: source 0–559    =  560 frames
2/4: source 0–1119   = 1120 frames
3/4: source 0–1679   = 1680 frames
4/4: source 0–2232   = 2233 frames
```

`roll_index` の連続blockを使うため、2233 frameを単純4等分したものではない。

### 7.2 重要: quarterは累積

```text
roll_to_1of4 = rolling start → quarter 1 end
roll_to_2of4 = rolling start → quarter 2 end
roll_to_3of4 = rolling start → quarter 3 end
roll_to_4of4 = rolling start → quarter 4 end
```

したがって、

```text
roll_to_1of4
→ 続けて roll_to_2of4
```

とはしない。

2/4を試したい場合は**rolling-start postureから `roll_to_2of4` を1本実行**する。

### 7.3 Gazeboで使う

Gazebo MCU nodeを起動した状態で、rolling-start postureから任意の1本を実行する。

1/4:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_to_1of4_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

2/4:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_to_2of4_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

3/4:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_to_3of4_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

4/4:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_to_4of4_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

Current status:

```text
1/4 Gazebo PASS
2/4 Gazebo PASS
3/4 Gazebo PASS
4/4 Gazebo PASS
hardware NOT TESTED
```

`roll_to_4of4_commands.jsonl` は元 `commands.jsonl` とbyte-for-byte同一。

---

## 8. 実機semantic quarter

初回のrisk-split full pathがPASSした後に使用する。

基本trial:

```text
HOME
→ suspended air-entry
→ controlled touchdown
→ roll_to_Nof4
→ STOP / inspect / record
```

例 2/4:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_to_2of4_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

3/4や4/4へ切り替える場合も、原則として別trialとしてHOME→air-entry→touchdownから開始する。

**実機semantic quarterは現時点でNOT TESTED。**

---

## 9. Final combined

既存risk-split rollがすべて実機PASSした後だけ使用する。

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/combined_with_hold_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

初回実機確認には使用しない。

---

## 10. Builder / checksum

通常運用ではfrozen quarter filesをそのまま使う。  
再生成は検証・開発・新candidate作成時のみ。

builder:

```text
tools/command_generation/build_roll_quarter_stages.py
```

一時directoryへ生成:

```bash
QUARTER_DIR=$(mktemp -d /tmp/lily_quarter_stages.XXXXXX)

python2 tools/command_generation/build_roll_quarter_stages.py \
  --command-log "$CANDIDATE/commands.jsonl" \
  --output-dir "$QUARTER_DIR"
```

Expected SHA256:

```text
source / 4/4
e60c9de63287c5c198e78e11c1da89475b2293e6de45950cf09f5f2c170304a5

1/4
cf2f2592b6dd688a996b4bcc872509fa9ee3b85d8db53825ce2a01671a70dc58

2/4
3e54fdef3c3285b2d45f43b086081ce1dc659e7a87098981a5702561878e0bf0

3/4
2599ea79a90ae4746a10f6771589e50e0a5acf7d3a1e2e0f8e146b602cad3998
```

4/4 exact comparison:

```bash
cmp -s \
  "$CANDIDATE/commands.jsonl" \
  "$STAGED/roll_to_4of4_commands.jsonl" \
  && echo "4/4 EXACT MATCH"
```

frozen `staged/` を再生成するときに `--overwrite` を安易に使わない。

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

formal comparisonでは:

```text
publish_cmdforjetson_jsonl.py
→ /cmdForJetson
→ mcu_position_interpolator_node.py
```

を使う。

---

## 13. Canonical JSONL publisher

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

semantic-quarter data freeze:
2e42343dccf3b56066cdcc97e011dca328388a20

transport:
factor=2 / 10 Hz

Gazebo MCU:
0.100 s / 0.002 s
```

関連:

- [`BASELINE.md`](BASELINE.md)
- [`BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md`](BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md)
- [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md)
- [`COMMAND_DATA_FORMAT.md`](COMMAND_DATA_FORMAT.md)
- [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md)
