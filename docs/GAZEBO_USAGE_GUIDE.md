# Lily Gazebo Usage Guide

更新日: 2026-08-30  
対象: `feature/monitor-csv-load`

この文書は、LilyをGazeboで確認するときの現行手順をまとめる。

すべての `tools/...`、`data/...` pathは `lily_motion/` repository root基準。

---

## 1. 共通command境界

Gazeboと実機は、上流の24軸command境界 `/cmdForJetson` を共有する。

実機:

```text
JSONL / Lily Operator Motion
    ↓
/cmdForJetson
    ↓
/lily_operator StateMachine
    ↓
CAN
    ↓
MCU / hardware
```

Gazebo:

```text
JSONL / Motion publisher
    ↓
/cmdForJetson
    ↓
/lily_gazebo_mcu_position_interpolator
    ↓
24 Gazebo joint controller topics
    ↓
Lily model
```

`tools/gazebo/mcu_position_interpolator_node.py` が `/cmdForJetson` の24要素を受け、MCU相当の位置補間を行ってGazeboの各joint controller topicへ分配する。

---

## 2. Gazebo単独確認

Gazebo単独では `Lily Operator (Gazebo)` を使用できる。

```text
Lily Operator (Gazebo)
    ↓
/cmdForJetson
    ↓
Gazebo MCU interpolator
    ↓
Gazebo
```

このモードではCAN StateMachineを起動しない。

Gazebo model / world / joint controllersは既存のGazebo環境側で先に起動しておく。

---

## 3. 実機 + Gazebo同期確認

実機とGazeboを同じcommandで動かす場合、CANを後段でコピーするSync Bridgeは使用しない。

通常の `Lily Operator` と既存Gazebo MCU interpolatorを同一ROS master上で使用する。

```text
                         Lily Operator Motion
                                  ↓
                            /cmdForJetson
                                  │
                    ┌─────────────┴─────────────┐
                    ↓                           ↓
          /lily_operator StateMachine   Gazebo MCU interpolator
                    ↓                           ↓
                   CAN                 24 joint controller topics
                    ↓                           ↓
                  実MCU                        Gazebo
                    ↓
                   実機
```

これにより、1回のMotion SENDで同じ24軸target列が実機経路とGazebo経路へ配信される。

### 安全トポロジ

通常Operatorでは `/cmdForJetson` subscriberを次の組合せだけ許可する。

```text
必須:
  /lily_operator

追加で許可:
  /lily_gazebo_mcu_position_interpolator
```

したがって:

```text
StateMachineのみ                         -> OK
StateMachine + Gazebo MCU interpolator  -> OK
Gazeboのみ                              -> NG
未知subscriber追加                      -> NG
別の/cmdForJetson publisher            -> NG
```

Motion SEND中もトポロジを再確認し、許可されない構成へ変化した場合はposition出力を中断する。

---

## 4. 実機 + Gazebo同期の起動順序

### 1. Gazebo model / joint controllersを起動

既存のGazebo環境側の手順で起動する。

### 2. 通常の `Lily Operator` を起動

物理CAN `can0` を使用する。

Controlで通常どおり:

```text
Connected
  ↓
ALIGN
  ↓
HOME
  ↓
RUN
```

まで進める。

### 3. Gazebo MCU interpolatorを起動

repository rootで:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

python2 tools/gazebo/mcu_position_interpolator_node.py \
  --input-topic /cmdForJetson \
  --interp-duration-sec 0.100 \
  --update-period-sec 0.002
```

### 4. `/cmdForJetson` の接続を確認

```bash
rostopic info /cmdForJetson
```

実機 + Gazebo同期時の意図したsubscriberは:

```text
/lily_operator
/lily_gazebo_mcu_position_interpolator
```

の2つ。

### 5. MotionをSEND

通常の `Lily Operator` のMotion tabでJSONLを `LOAD / CHECK` し、SENDする。

別のpublisherを起動しない。

---

## 5. Gazebo MCU interpolator parameter

対象:

```text
tools/gazebo/mcu_position_interpolator_node.py
```

主な引数:

| 引数 | default | 意味 |
|---|---:|---|
| `--input-topic` | `/cmdForJetson` | 24軸target入力 |
| `--interp-duration-sec` | `0.100` | MCU相当の補間時間 [s] |
| `--update-period-sec` | `0.002` | Gazebo controllerへの出力周期 [s] |
| `--gazebo-topic-prefix` | empty | optional topic prefix |

同期比較では、可能なら実MCUの補間設定とGazebo側 `--interp-duration-sec` の関係を明示して評価する。

---

## 6. `resample_factor` とGazebo補間

Jetson/Operator側のresamplingとGazebo側補間は別処理。

```text
source JSONL
    ↓
resample_factor
    ↓
/cmdForJetson target列
    ↓
Gazebo MCU interpolation
    ↓
Gazebo controller update
```

- `resample_factor`: `/cmdForJetson` に送るtarget点を増やす
- `interp-duration-sec`: 各target受信後のMCU相当遷移をGazebo側で模擬する

---

## 7. 注意事項

- 実機 + Gazebo同期時は `Lily Operator (Gazebo)` を同時起動しない。
- CAN-to-Gazebo Sync Bridgeは廃止済みであり、使用しない。
- `/cmdForJetson` の別publisherを追加しない。
- 実機とGazeboは同じ上流targetを受けるが、ROS scheduling、CAN送信、MCU処理、Gazebo schedulingにより完全なハードリアルタイム同期ではない。

---

## 8. 関連文書

- `tools/operator_ui/DESKTOP_LAUNCHER.md`
- `docs/COPY_PASTE_COMMANDS.md`
- `docs/JETSON_ARGUMENT_REFERENCE.md`
- `docs/RUNTIME_ARCHITECTURE.md`
- `docs/CURRENT_BASELINE.md`
- `docs/VALIDATION_STATUS.md`
