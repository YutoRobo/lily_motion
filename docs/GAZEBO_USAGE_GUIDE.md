# Lily Gazebo Usage Guide

更新日: 2026-08-23  
対象: `master`

この文書は、**LilyをGazeboで確認するときの入口と実行手順の正本**である。

すべての `tools/...`、`data/...` pathは `lily_motion/` repository root基準。

---

## 1. まず知っておくこと

現行のGazebo確認は、実機と同じ上流command pathを使う。

```text
staged JSONL
    ↓
tools/publish_cmdforjetson_jsonl.py
    ↓
/cmdForJetson
    ↓
tools/gazebo/mcu_position_interpolator_node.py
    ↓
Gazebo joint controller topics
    ↓
Lily model
```

実機では `/cmdForJetson` の下流がCAN StateMachineになる。

```text
REAL:
/cmdForJetson → CAN StateMachine → CAN → MCU

GAZEBO:
/cmdForJetson → Gazebo MCU interpolator → Gazebo
```

したがって、**同じJSONL / 同じJetson transport target列を実機とGazeboで比較できる**。

---

## 2. このrepositoryだけでは起動しないもの

現在の `tools/gazebo/` には次のPython toolsがある。

```text
mcu_position_interpolator_node.py
run_v3_0_gazebo_replay.py
run_v3_0_gazebo_touchdown_pose_check.py
```

一方、現行repositoryにはLily本体・Gazebo world・joint controllerを起動するlaunch fileは確認できない。

そのため、このガイドは次が**既に起動できる環境**から開始する。

```text
Gazebo
+ Lily robot model
+ Lily joint controllers
```

この外部Gazebo環境の起動方法は、現時点では本repositoryの正本化対象外。

---

## 3. Gazeboで起動してはいけないもの

Gazebo確認時はCAN StateMachineを `/cmdForJetson` に接続しない。

```text
Gazebo trial
  mcu_position_interpolator_node.py : ON
  CAN StateMachine                  : OFF
```

実機trialでは逆。

```text
Real hardware trial
  CAN StateMachine                  : ON
  Gazebo MCU interpolator           : OFF
```

同時に両方をsubscriberとして接続しない。

---

## 4. 基本起動順序

### Terminal 1: ROS master

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roscore
```

### 別途: Gazebo + Lily model + joint controllers

既存のGazebo環境側の手順で起動する。

本repositoryにはこのlaunch commandを現在保持していない。

### Terminal 2: Gazebo MCU-equivalent interpolator

repository rootで:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

python2 tools/gazebo/mcu_position_interpolator_node.py \
  --input-topic /cmdForJetson \
  --interp-duration-sec 0.100 \
  --update-period-sec 0.002
```

### Terminal 3: subscriber確認

```bash
rostopic info /cmdForJetson
```

Gazebo時、意図したconsumerは `mcu_position_interpolator_node.py`。

CAN StateMachineが同時にsubscriberになっていないことを確認する。

---

## 5. staged JSONLをGazeboへ送る

現行candidate:

```bash
CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
STAGED=$CANDIDATE/staged
```

### Air-entry

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/air_entry_and_hold_only_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

### Risk split example

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/roll_0_50_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

他のstaged fileも同じpublisherで送る。

実行コマンド一覧は [`COPY_PASTE_COMMANDS.md`](COPY_PASTE_COMMANDS.md) を参照する。

---

## 6. Gazebo MCU interpolatorの引数

対象:

```text
tools/gazebo/mcu_position_interpolator_node.py
```

| 引数 | default | 意味 |
|---|---:|---|
| `--input-topic` | `/cmdForJetson` | Jetson transport targetの入力topic |
| `--interp-duration-sec` | `0.100` | 新target受信後、Gazebo側でMCU相当の線形補間を行う時間 [s] |
| `--update-period-sec` | `0.002` | 補間結果をGazebo controllerへ更新する周期 [s] |
| `--gazebo-topic-prefix` | empty | `GazeboCommandPublisher` へ渡すoptional topic prefix |

注意:

```text
--interp-duration-sec
```

はGazebo側でMCU挙動を模擬するparameterであり、Jetsonの

```text
--resample-factor
--rate
```

とは別物。

また、実MCU Configの

```text
interpolation_time_ms
```

とも別設定値である。

---

## 7. 現行Gazebo comparison profile

現在、staged validationで使用してきたprofile:

```text
Jetson transport
  resample_factor = 2
  rate            = 10 Hz

Gazebo MCU equivalent
  interp_duration = 0.100 s
  update_period   = 0.002 s
```

このprofileはGazebo側で確認済みの比較条件として扱う。

実MCUの `interpolation_time_ms` と完全に同値であるとは仮定せず、実機側は別途確認する。

---

## 8. `resample_factor` とGazebo補間の違い

```text
source JSONL
q0 ---------------- q1

Jetson --resample-factor 2
q0 ------ midpoint ------ q1

Gazebo --interp-duration-sec
各transport targetを受け取った後、
Gazebo joint controllerへ向けて連続的に補間
```

つまり:

- `resample_factor`: **Jetsonが送るtarget点そのものを増やす**
- `interp-duration-sec`: **Gazebo側で1 targetへ滑らかに遷移する**

である。

Jetson側引数の詳細は [`JETSON_ARGUMENT_REFERENCE.md`](JETSON_ARGUMENT_REFERENCE.md) を参照する。

---

## 9. Development direct replay

次も存在する。

```text
tools/gazebo/run_v3_0_gazebo_replay.py
```

これはdevelopment / direct Gazebo replay用。

現行の実機比較では、原則として

```text
publish_cmdforjetson_jsonl.py
→ /cmdForJetson
→ mcu_position_interpolator_node.py
```

のshared pathを優先する。

理由は、実機とGazeboで `/cmdForJetson` より上流を共通化できるため。

---

## 10. Gazeboで確認するときのチェック

```text
[ ] Gazebo + Lily model + joint controllersが起動済み
[ ] roscore起動済み
[ ] mcu_position_interpolator_node.py起動済み
[ ] CAN StateMachineは起動していない
[ ] /cmdForJetson subscriberが意図どおり
[ ] 使用candidate / staged fileを確認
[ ] resample_factor / rateを確認
[ ] Gazebo interp duration / update periodを確認
```

---

## 11. 現在の不足

本repository内でGazeboを完全に自己完結させるには、まだ次が不足している。

```text
Gazebo world / Lily model / joint controllerの起動手順
または
それらをまとめたlaunch file
```

既存のGazebo環境が別repository / catkin packageにある場合、その正確な起動方法をこの文書へ接続すると、clone後の再現性がさらに高くなる。

---

## 12. 関連文書

- [`COPY_PASTE_COMMANDS.md`](COPY_PASTE_COMMANDS.md) — 実行コマンド
- [`JETSON_ARGUMENT_REFERENCE.md`](JETSON_ARGUMENT_REFERENCE.md) — Jetson側引数
- [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md) — 実機 / Gazeboのprogram境界
- [`CURRENT_BASELINE.md`](CURRENT_BASELINE.md) — current candidate / profile
- [`VALIDATION_STATUS.md`](VALIDATION_STATUS.md) — current verification status
