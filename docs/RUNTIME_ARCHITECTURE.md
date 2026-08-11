# Lily Runtime Architecture

更新日: 2026-08-12  
状態: 現行設計

## 1. 目的

この文書は、Lilyの回転指令が「どこまで共通で、どこから実機/Gazebo固有になるか」を定義する。

最重要原則:

> **実機とGazeboは `/cmdForJetson` まで同じprogram path・同じtransport command streamを使用する。**

Gazeboのために別trajectory generatorや別backend runnerを作らない。

## 2. Canonical path

```text
staged / frozen JSONL
        ↓
tools/publish_cmdforjetson_jsonl.py
        ↓
lily_motion_v3.command_stream
  source normalization
        ↓
lily_motion_v3.command_timing
  transport resampling
        ↓
/cmdForJetson
sensor_msgs/JointState
position[24] [rad]
        │
        ├───────────────────────────────┐
        │                               │
      REAL                            GAZEBO
        │                               │
tools/can_interface/            tools/gazebo/
statemachine/StateMachine       mcu_position_interpolator_node.py
        │                               │
       CAN                      OnlineLinearActuatorInterpolator
        │                               │
     real MCU                           │
        │                         Gazebo Float64 topics
      motor
```

## 3. Shared boundary

共通部は `/cmdForJetson` publishまで。

canonical shared runtime files:

```text
lily_motion_v3/command_stream.py
lily_motion_v3/command_timing.py
tools/publish_cmdforjetson_jsonl.py
```

この層は次を行う。

- JSONL position keyの正規化
- 24軸length validation
- source frame selection
- linear transport resampling
- transport stream SHA256
- ROS `/cmdForJetson` publish

この層は次を行わない。

- CAN open
- CAN ID生成
- MCU simulation
- Gazebo joint topic publish
- hardware/Gazebo backend switch

## 4. Source trajectory と transport

frozen JSONLはsource trajectory / keyframe sequenceである。

現在のpre-hardware profile:

```text
source nominal cadence: 約5 Hz相当
resample-factor:        2
transport rate:         10 Hz
```

factor 2では、隣接source targetの間にlinear midpointを1点追加する。

概念:

```text
source:
q0 ---------------- q1 ---------------- q2
0 ms               200 ms              400 ms

transport factor=2:
q0 ----- q0.5 ----- q1 ----- q1.5 ----- q2
0 ms      100 ms    200 ms    300 ms    400 ms
```

したがって、source trajectoryそのものを書き換えずにcommand updateを細分化する。

これはmoving average等のtrajectory smoothingとは異なる。

## 5. Actuator / MCU interpolation

実MCUは外部position targetを内部control periodで補間してmotor側へ渡す。

現在比較対象としているprofile:

```text
external target interval:   0.100 s
MCU interpolation duration: 0.100 s
MCU update period:          0.002 s
```

この3値は**独立した設定概念**である。

将来MCU firmwareが変わった場合、

```text
transport target rate
MCU interpolation duration
MCU internal update period
```

を別々に変更・記録する。

## 6. Why factor=2 / 10 Hz

旧5 Hz sourceをそのまま外部送信すると、実MCUが約100 msで新targetへ補間した後、次のtargetまで残り約100 msをholdする可能性がある。

factor=2 / 10 Hzでは、

```text
t=0 ms      q0
t=100 ms    midpoint(q0,q1)
t=200 ms    q1
```

となり、100 ms MCU interpolationと整合した場合、元の200 ms区間を連続的に近似しやすい。

これは現在MCU挙動に対するpre-hardware仮説であり、実機で検証する。

## 7. Gazebo MCU-equivalent node

```text
tools/gazebo/mcu_position_interpolator_node.py
```

入力:

```text
/cmdForJetson
sensor_msgs/JointState
position length = 24
```

出力:

```text
24 Gazebo joint controller topics
std_msgs/Float64
```

現行profile:

```bash
python2 tools/gazebo/mcu_position_interpolator_node.py \
  --input-topic /cmdForJetson \
  --interp-duration-sec 0.100 \
  --update-period-sec 0.002
```

online interpolatorは実時間elapsed timeからalphaを計算し、timer callback回数そのものには依存しない。

期待semantic:

- 最初のtargetはstarting poseとしてhold
- 新targetで補間開始/restart
- target到達後はhold
- upstream publisher終了後もlast targetをhold

## 8. Canonical Gazebo validation

### Terminal A

Gazebo robotを通常どおり起動後:

```bash
python2 tools/gazebo/mcu_position_interpolator_node.py \
  --input-topic /cmdForJetson \
  --interp-duration-sec 0.100 \
  --update-period-sec 0.002
```

### Terminal B

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log <staged-file> \
  --resample-factor 2 \
  --rate 10
```

Gazebo MCU nodeはsplit stage間で再起動しない。前stageのlast targetをholdした状態から次stageを受ける。

## 9. Canonical hardware path

実機ではGazebo MCU nodeを起動しない。

```text
/cmdForJetson
→ tools/can_interface/statemachine/StateMachine
→ Use=True axes
→ POSITION CAN 0x400 + axis
→ real MCU
```

publisherはCANを直接開かない。

## 10. Consumer exclusivity

`/cmdForJetson` はaction command streamであるため、意図しないconsumerを同時起動しない。

Gazebo:

```text
ON  mcu_position_interpolator_node.py
OFF hardware CAN StateMachine
```

Hardware:

```text
ON  hardware CAN StateMachine
OFF mcu_position_interpolator_node.py
```

canonical publisherはpublish開始前にsubscriberを待つ。subscriberが複数の場合はwarningを出すため、試験を開始する前にconsumerを確認する。

## 11. Direct Gazebo replay

```text
tools/gazebo/run_v3_0_gazebo_replay.py
```

は残す。

用途:

- trajectory development
- visualization
- diagnostics
- historical reproduction

ただし、これは正式な `/cmdForJetson` common-path verificationではない。

## 12. Change control

2026-08-12 pre-hardware baselineで固定した比較条件:

```text
candidate:
v3_0_44_candidate_022_wide_urdf0p075

transport:
factor=2
rate=10 Hz

Gazebo MCU:
interp=0.100 s
update=0.002 s
```

実機で変更が必要になった場合:

1. baseline値を上書きしない
2. 変更理由を記録
3. 新profileでGazebo/common-path regression
4. 新baselineを作成
5. その後に次の実機stageへ進む
