# Lily 8脚ロボット コマンド集

更新日: 2026-08-04  
対象: 現行`master`

この文書は、Lily 8脚ロボットの現行ソフトウェアを実行・確認するためのコマンド集である。ソフト全体像は[`../README.md`](../README.md)、実機の厳密な段階手順は[`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md)を参照する。

## 0. 表記

- **確認済み**: 単体試験、vcan、Gazebo、または実機で確認済み
- **要実機確認**: ソフトウェア経路は確認済みだが実機挙動は未確認
- **履歴再現用**: 現行候補ではなく、過去結果の再現・比較用

---

# 1. 共通操作

## 1.1 リポジトリへ移動

```bash
cd ~/Programs/PythonScripts/260522_lily_remake/lily_motion
```

## 1.2 Git状態

```bash
git status -sb
git log -1 --oneline
git diff --check
```

実機試験では使用コミットを必ず記録する。

## 1.3 ROS環境

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

## 1.4 roscore

```bash
roscore
```

---

# 2. 現行ソフトウェア経路

## 2.1 本番位置指令

```text
単軸試験／1脚試験／複数脚試験／回転歩容
                    ↓
/cmdForJetson
sensor_msgs/JointState
position: 24 elements
                    ↓
StateMachine
                    ↓
Use=True軸だけへRUN／POSITION CAN送信
```

削除済みの`/can/axis_command`を本番位置指令として使用しない。

## 2.2 実行対象

```text
tools/can_interface/statemachine/main.py
tools/can_interface/initUI/ui.py
tools/publish_cmdforjetson_single_axis_test.py
tools/publish_cmdforjetson_jsonl.py
```

`external/can_interface/`は旧スナップショットであり、現行実機操作には使用しない。

---

# 3. vcan0の準備

## 3.1 作成または再利用

```bash
sudo modprobe vcan
ip link show vcan0 >/dev/null 2>&1 || sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
ip -details link show vcan0
```

## 3.2 ログ

```bash
candump -L vcan0 | tee /tmp/lily_vcan.log
```

## 3.3 削除

```bash
sudo ip link delete vcan0
```

---

# 4. 複数アクチュエータ・エミュレータ

エミュレータはvcan専用であり、`can0`を拒否する。

## 4.1 axis10だけ

```bash
python2 tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 10
```

## 4.2 axis10,11,12

```bash
python2 tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 10,11,12
```

期待heartbeat:

```text
0FF#0A00000000000000
0FF#0B00000000000000
0FF#0C00000000000000
```

## 4.3 全24軸

```bash
python2 tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 0-23
```

## 4.4 ALIGN失敗を1回だけ注入

```bash
python2 tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 10,11,12 \
  --align-fail-once 11
```

期待動作:

```text
axis10,12: first ALIGN success
axis11: 0x0EE → reset → heartbeat return
axis11: second ALIGN success
```

## 4.5 その他のシナリオ

```bash
python2 tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 10,11,12 \
  --align-fail-always 12
```

```bash
python2 tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 10,11,12 \
  --align-fail-at 11:2
```

```bash
python2 tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 10,11,12 \
  --inject-error 12:2
```

```bash
python2 tools/can_interface/emulator/multi_actuator_emulator.py \
  --interface vcan0 \
  --axes 10,11,12 \
  --reset-after-run 10:2.0
```

詳細:

- [`../tools/can_interface/emulator/README.md`](../tools/can_interface/emulator/README.md)

---

# 5. StateMachineとUI

## 5.1 StateMachineをvcan0で起動

```bash
python2 tools/can_interface/statemachine/main.py \
  --can-interface socketcan \
  --can-channel vcan0 \
  --can-bitrate 500000
```

## 5.2 StateMachineをcan0で起動

実機試験時だけ実行する。

```bash
python2 tools/can_interface/statemachine/main.py \
  --can-interface socketcan \
  --can-channel can0 \
  --can-bitrate 500000
```

実行前:

```bash
ip -details link show can0
```

## 5.3 UI

```bash
python2 tools/can_interface/initUI/ui.py
```

## 5.4 UIコマンド

Topic:

```text
/ui/leg_command
```

Use:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:10:1'"
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'use:10:0'"
```

全Use=True軸へALIGN:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'align'"
```

指定軸へALIGN:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'align:10'"
```

HOME step:

```bash
rostopic pub -1 /ui/leg_command std_msgs/String "data: 'home_step:0.002'"
```

HOME jog:

```bash
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

実装上、global`align`は有効である。global`home`とglobal`set_home`は実装されていない。

---

# 6. axis10単軸試験

## 6.1 前提

```text
Connected
→ axis10だけUse=True
→ ALIGN
→ HOME方向確認
→ SET HOME
→ RUN
→ publisher
→ STOP
```

安全条件:

- 機体または対象脚を浮かせる
- axis10以外をUse=Falseにする
- 非常停止を直ちに操作できる状態にする
- 可動範囲へ人を入れない
- 異音、衝撃、別軸動作、原点未復帰、`0x0EE`で中止する

## 6.2 正方向0.002 rad

**確認済み: 実機で暫定PASS**

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

```text
0.000 → 0.001 → 0.002 → 0.001 → 0.000 rad
```

## 6.3 負方向0.002 rad

**要実機確認**

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

```text
0.000 → -0.001 → -0.002 → -0.001 → 0.000 rad
```

## 6.4 正負0.005 rad

**要実機確認。正負0.002 radが両方PASSした後だけ実施する。**

```bash
python2 tools/publish_cmdforjetson_single_axis_test.py \
  --axis 10 --direction plus \
  --amplitude-rad 0.005 --step-rad 0.001 \
  --period-sec 0.500 \
  --start-hold-sec 1.000 --peak-hold-sec 1.000 --end-hold-sec 1.000
```

```bash
python2 tools/publish_cmdforjetson_single_axis_test.py \
  --axis 10 --direction minus \
  --amplitude-rad 0.005 --step-rad 0.001 \
  --period-sec 0.500 \
  --start-hold-sec 1.000 --peak-hold-sec 1.000 --end-hold-sec 1.000
```

## 6.5 directionの意味

```text
plus:  center → center + amplitude → center
minus: center → center - amplitude → center
```

plus/minusは論理関節角の符号であり、上、下、前、後を直接意味しない。

## 6.6 publisher標準値

```text
center_rad       = 0.0
amplitude_rad    = 0.020
step_rad         = 0.005
period_sec       = 0.100
start_hold_sec   = 0.500
peak_hold_sec    = 0.500
end_hold_sec     = 0.500
```

実機初回に標準振幅0.020 radを使用しない。

## 6.7 axis10期待CAN

```text
ALIGN:    0x00A
SET HOME: 0x30A
RUN:      0x60A
POSITION: 0x40A
```

位置往復中に`0x40A`だけが繰り返されることは正常である。`0x60A`はRUN開始時だけ送信される。

---

# 7. 単軸publisherのROS形式

```text
Topic: /cmdForJetson
Message: sensor_msgs/JointState
position length: 24
selected axis: finite rad value
non-selected axes: NaN
```

StateMachineは`Use=True`軸だけを検査する。意図しない別軸がUse=Trueの場合、その軸のNaNによってフレーム全体がCAN送信前に拒否される。

---

# 8. 複数軸ファンアウト確認

## 8.1 確認済み結果

```text
axes: 10,11,12
RUN: 0x60A, 0x60B, 0x60C
POSITION: 0x40A, 0x40B, 0x40C
result: PASS
```

1つの24要素`JointState`から、3軸へ個別CANフレームが生成された。

これはソフトウェアのファンアウト確認であり、複数実アクチュエータの同期応答、24台の実バス負荷、機械応答は未確認である。

---

# 9. JSONL publisher

## 9.1 一般形

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log <commands.jsonl> \
  --rate <hz> \
  --start-index <n> \
  --max-frames <n>
```

Accepted keys:

```text
joint_command_rad
position
joint_positions_rad
```

各フレームは24要素でなければならない。

PublisherはCANを直接開かない。CAN変換はStateMachineだけが行う。

---

# 10. 現行pre-hardware候補

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
```

主な状態:

```text
command count: 2233
coxa: 0.075 m
thigh: 0.300 m
tibia: 0.300 m
second-joint max: 94.8 deg
violations over 95 deg: 0
Gazebo full roll: PASS
hardware full roll: not tested
```

詳細:

- [`../data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/pre_hardware_decision.md`](../data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/pre_hardware_decision.md)

---

# 11. 現行候補の段階実機コマンド

必ず[`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md)の順序と安全条件に従う。

## 11.1 Air-entry and hold only

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/air_entry_and_hold_only_commands.jsonl \
  --rate 5
```

## 11.2 Roll 0–50

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/roll_0_50_commands.jsonl \
  --rate 3
```

## 11.3 Roll 50–100

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/roll_50_100_commands.jsonl \
  --rate 3
```

## 11.4 Roll 100–300

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/roll_100_300_commands.jsonl \
  --rate 3
```

## 11.5 Roll 300–end

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/roll_300_end_commands.jsonl \
  --rate 3
```

## 11.6 Final combined sequence

全分割rollがPASSした後だけ実施する。

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/staged/combined_with_hold_commands.jsonl \
  --rate 3
```

Approved order:

```text
air-entry and hold
→ touchdown confirmation
→ roll 0–50
→ roll 50–100
→ roll 100–300
→ roll 300–end
→ final combined sequence
```

---

# 12. Gazebo確認

## 12.1 現行候補のstrict dry-run

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --dry-run \
  --command-log data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/commands.jsonl \
  --strict-command-log-input \
  --rate 15 \
  --hold-start-sec 0.0 \
  --hold-end-sec 0.0 \
  --repeat-last 0 \
  --diagnose-command-log
```

Frozen result: PASS.

## 12.2 candidate_022_wide full-roll review reproduction

```bash
bash testdata/visual_near_contact_local_fix_candidate/candidate_022_wide/gazebo_review_global/run_gazebo_review_global.sh \
  full_roll \
  normal
```

This is a Gazebo review workflow. Hardware execution uses the frozen files under`data/reference_candidates/`.

## 12.3 General Gazebo replay

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --command-log <commands.jsonl> \
  --strict-command-log-input \
  --rate 15 \
  --hold-start-sec 2.0 \
  --hold-end-sec 2.0 \
  --diagnose-command-log
```

---

# 13. 履歴候補の再現

v3.0.42 candidate02は比較・履歴上の重要な基準だが、現在の最初のpre-hardware候補ではない。

旧effort replay helperはarchiveへ移動済みである。

```bash
python archive/v3_experiment_scripts/run_v3_0_42e_effort_replay_plot.py \
  --command-log testdata/v3_0_42c_candidates/candidate_02_x8_sw40_commands.jsonl \
  --strict-command-log-input \
  --rate 5 \
  --hold-start-sec 2.0 \
  --hold-end-sec 2.0 \
  --diagnose-command-log \
  --effort-limit 40 \
  --output testdata/v3_0_42e_effort_candidate_02_rate5.json \
  --plot-dir testdata/v3_0_42e_effort_candidate_02_rate5_plots
```

これは履歴再現用であり、現行実機操作の入口ではない。

---

# 14. v3-core評価

## 14.1 Quick evaluation

```bash
python tools/diagnostics/run_v3_0_whole_roll_eval.py --summary-only
```

## 14.2 Failure diagnosis

```bash
python tools/diagnostics/run_v3_0_diagnose_failures.py
```

## 14.3 Parameter sweep

```bash
python tools/diagnostics/run_v3_0_parameter_sweep.py --help
```

## 14.4 Visualization

```bash
python tools/diagnostics/run_v3_0_visualize_roll.py --help
```

詳細:

- [`../README_V3_CORE.md`](../README_V3_CORE.md)

---

# 15. CANテスト

## 15.1 Unified path

```bash
python2 tests/test_can_cmdforjetson_unified_path.py
```

Verified:

```text
10/10 PASS
```

## 15.2 Safety and emulator regression

```bash
python2 tests/test_can_diagnostic_run.py
python2 tests/test_can_emulator_integration.py
python2 tests/test_can_legacy_alignment_retry.py
python2 tests/test_can_multi_actuator_emulator.py
```

## 15.3 All CAN tests

```bash
python2 -m unittest discover -s tests -p "test_can_*.py"
```

Verified:

```text
81/81 PASS
```

---

# 16. Syntax and diff checks

## 16.1 Python 2.7

```bash
python2 -m py_compile \
  tools/can_interface/emulator/virtual_actuator.py \
  tools/can_interface/emulator/multi_actuator_emulator.py \
  tools/can_interface/emulator/scenario.py \
  tools/can_interface/statemachine/state_machine.py \
  tools/publish_cmdforjetson_single_axis_test.py \
  tools/publish_cmdforjetson_jsonl.py
```

## 16.2 Python 3 syntax

```bash
python3 -m py_compile \
  tools/can_interface/emulator/virtual_actuator.py \
  tools/can_interface/emulator/multi_actuator_emulator.py \
  tools/can_interface/emulator/scenario.py \
  tools/can_interface/statemachine/state_machine.py \
  tools/publish_cmdforjetson_single_axis_test.py \
  tools/publish_cmdforjetson_jsonl.py
```

## 16.3 Git diff

```bash
git diff --check
```

## 16.4 Generated bytecode cleanup

```bash
find . -type d -name '__pycache__' -prune -exec rm -rf {} +
find . -type f -name '*.pyc' -delete
```

---

# 17. ENOBUFS調査

Known error:

```text
[Errno 105] No buffer space available
```

Search:

```bash
grep -RniE \
  'ENOBUFS|No buffer space|pc_send_error|can_interface_error' \
  . \
  --exclude-dir=.git
```

Notes:

- vcanでは実CANのACK、bit timing、送信キュー挙動を完全再現しない。
- 安全ゲートを無効化して回避しない。
- 24軸100 Hz相当はCAN負荷を実測して判断する。

---

# 18. 標準フロー

## 18.1 CANソフト統合だけ確認

```text
vcan0
→ candump
→ emulator
→ StateMachine on vcan0
→ UI
→ Use
→ ALIGN
→ SET HOME
→ RUN
→ /cmdForJetson publisher
→ STOP
```

## 18.2 現行実機段階

```text
axis10 negative 0.002 rad
→ axis10 positive/negative 0.005 rad
→ one leg, three axes
→ air-entry and hold
→ touchdown
→ split roll
→ final combined sequence
```

---

# 19. 安全上の注意

- vcan試験では`--can-channel vcan0`を明示する。
- 実機試験前に`can0`と`vcan0`を取り違えていないことを確認する。
- 実機単軸試験では対象脚を浮かせる。
- 対象外軸を`Use=False`にする。
- 未Aligned、未Homed、RUN前の位置指令を送らない。
- `pc_send_error`、`can_interface_error`、MCU errorを無効化しない。
- 予期しないstandby heartbeat復帰後はセッション状態を再確認する。
- full sequenceから開始しない。
- `archive/`と`external/can_interface/`を現行実機経路として使用しない。
- 試験終了後は必ずSTOPする。
