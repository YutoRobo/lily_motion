# Lily Jetson Argument Reference

更新日: 2026-08-23  
対象: `master`

この文書は、**Jetson / host側で実行する現行programの引数の意味を確認するための正本**である。

すべての `tools/...` pathは `lily_motion/` repository root基準で記載する。

---

## 1. 最重要: Jetson transportとMCU interpolationは別物

Lilyの位置指令には、少なくとも次の3層がある。

```text
source JSONL
    ↓
Jetson transport resampling
  --resample-factor
  --rate
    ↓
/cmdForJetson
    ↓
StateMachine → CAN
    ↓
MCU target interpolation
  interpolation_time_ms
    ↓
motor position command
```

ここを混同しない。

- `--resample-factor`: **Jetson側**でsource JSONLの隣接frame間に追加targetを作る。
- `--rate`: **Jetson側**でtransport targetを何Hzでpublishするか。
- `interpolation_time_ms`: **MCU側**で受信した1つのtargetへ何msかけて線形遷移するか。

---

## 2. `--resample-factor` とは何か

対象program:

```text
tools/publish_cmdforjetson_jsonl.py
```

`--resample-factor F` は、隣接するsource command `q0 → q1` の間を **F分割**する。

### factor = 1

補間frameを追加しない。

```text
q0 -------- q1
```

### factor = 2

1つ中間点を追加する。

```text
q0 ---- midpoint ---- q1
      alpha = 0.5
```

つまり、source pair `q0, q1` に対してtransport側は概念的に:

```text
q0
0.5*q0 + 0.5*q1
q1
```

となる。

### factor = 4

3つの中間点を追加する。

```text
alpha = 0.00, 0.25, 0.50, 0.75, 1.00
```

segment分割を使わない単純な `N` source framesの場合、transport frame数は:

```text
transport_frames = F * (N - 1) + 1
```

例:

```text
source frames = 135
factor        = 2
transport     = 2*(135-1)+1 = 269 frames
```

これは現行air-entry dry-runの269 framesと一致する。

---

## 3. `--resample-factor` と `--rate` の関係

sourceの時間スケールを保ちたい場合、`resample_factor` を増やした分だけpublish rateも増やす。

概念的に、source frame間の時間は:

```text
source_pair_time ≈ resample_factor / rate
```

例えば:

```text
resample_factor = 1
rate            = 5 Hz
```

ならsource frame間は約0.2 s。

同じ時間スケールを:

```text
resample_factor = 2
rate            = 10 Hz
```

にすると、Jetsonは0.1 sごとにtargetを送るが、source `q0 → q1` は2 transport intervalで進むため、元の約0.2 sを維持する。

現行staged rollで使用している:

```text
--resample-factor 2
--rate 10
```

はこの考え方である。

### 注意

`resample_factor` だけ2倍にして `rate` を変えない場合、frame数だけ増えるためmotion全体はおおむね遅くなる。

逆に `rate` だけ上げると、同じtarget列をより短時間で送るためmotionは速くなる。

---

## 4. `--segment-key`

`publish_cmdforjetson_jsonl.py` のoptional argument。

```text
--segment-key <metadata key>
```

指定すると、そのmetadata値が変わる境界をまたいで線形補間しない。

例:

```text
--segment-key roll_index
```

なら、異なる `roll_index` 間に人工的な中間姿勢を作らない。

通常のfrozen staged fileでは、既に意図したstage境界でfileが分かれているため、指定が必要かはfile生成方針に従う。

---

# 5. `publish_cmdforjetson_jsonl.py`

用途:

```text
JSONL
→ transport resampling
→ sensor_msgs/JointState
→ /cmdForJetson
```

実機とGazeboで共通のcanonical publisher。

| 引数 | 必須 | default | 意味 |
|---|---|---|---|
| `--command-log` | yes | - | 入力JSONL file。`joint_command_rad` / `position` / `joint_positions_rad` の24要素を読む |
| `--topic` | no | `/cmdForJetson` | publish先ROS topic |
| `--rate` | yes | - | transport target publish rate [Hz] |
| `--start-index` | no | `0` | source JSONLの開始frame index |
| `--max-frames` | no | none | 補間前のsource frameを最大何frame読むか |
| `--resample-factor` | no | `1` | source隣接点間の線形分割数。2ならmidpointを1点追加 |
| `--segment-key` | no | empty | metadata境界を跨ぐ補間を禁止するkey |
| `--subscriber-wait-timeout-sec` | no | `5.0` | 最初の送信前にsubscriberを待つ最大時間 [s]。0で待機無効 |
| `--dry-run` | no | false | ROS publishせず、生成されるtransport frame数とSHAを確認 |

現行staged hardware execution例:

```bash
python2 tools/publish_cmdforjetson_jsonl.py \
  --command-log "$STAGED/air_entry_and_hold_only_commands.jsonl" \
  --resample-factor 2 \
  --rate 10
```

---

# 6. CAN StateMachine `main.py`

対象:

```text
tools/can_interface/statemachine/main.py
```

| 引数 | default | 意味 |
|---|---|---|
| `--can-interface` | `socketcan` | python-can backend名 |
| `--can-channel` | `can0` | 使用CAN interface名 |
| `--can-bitrate` | `500000` | CAN bitrate [bit/s] |

環境変数でも指定可能:

```text
LILY_CAN_INTERFACE
LILY_CAN_CHANNEL
LILY_CAN_BITRATE
```

通常の実機:

```bash
python2 tools/can_interface/statemachine/main.py \
  --can-interface socketcan \
  --can-channel can0 \
  --can-bitrate 500000
```

注意: `--can-bitrate` はLinux側 `can0` をUPする処理の代替ではない。事前に:

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

を実行する。

---

# 7. MCU Config Editor

対象:

```text
tools/mcu_config/lily_mcu_config_editor.py
```

| 引数 | default | 意味 |
|---|---|---|
| `--interface` | `can0` | `candump` / `cansend` で使用するCAN interface |
| `--axes` | `0-23` | GUIに表示・READ対象とするaxis指定 |

`--axes` 例:

```text
11
10-11
0,1,2,11
0-23
```

通常:

```bash
python2 tools/mcu_config/lily_mcu_config_editor.py --interface can0 --axes 11
```

---

# 8. Single-axis test publisher

対象:

```text
tools/publish_cmdforjetson_single_axis_test.py
```

対象axisだけfinite値、他23軸はNaNをpublishする。対象axisだけ `Use=True` にする。

| 引数 | default | 意味 |
|---|---|---|
| `--axis` | required | target axis `0..23` |
| `--direction` | `plus` | `plus` / `minus` |
| `--center-rad` | `0.0` | SET HOME後の論理中心 [rad] |
| `--amplitude-rad` | `0.020` | centerからの最大変位 [rad] |
| `--step-rad` | `0.005` | 1commandごとの変位increment [rad]。amplitudeの整数分割である必要あり |
| `--period-sec` | `0.100` | 通常step間隔 [s] |
| `--peak-hold-sec` | `0.500` | peakでの保持時間 [s] |
| `--start-hold-sec` | `0.500` | 開始center保持時間 [s] |
| `--end-hold-sec` | `0.500` | 終了center保持時間 [s] |
| `--subscriber-wait-sec` | `0.500` | publish開始前の待機 [s] |
| `--topic` | `/cmdForJetson` | 出力topic |

初回実機で現在使っている小振幅例は `0.002 rad`, step `0.001 rad`。

---

# 9. One-leg test publisher

対象:

```text
tools/publish_cmdforjetson_one_leg_test.py
```

選択legの3軸だけfinite値、他21軸はNaN。

axis mapping:

```text
leg 0 → axes 0,1,2
leg 1 → axes 3,4,5
...
leg 7 → axes 21,22,23
```

| 引数 | default | 意味 |
|---|---|---|
| `--leg-index` | required | leg index `0..7` |
| `--mode` | `individual` | `individual` / `coordinated` / `all` |
| `--direction` | `plus` | `plus` / `minus` / `both` |
| `--centers-rad` | `0,0,0` | base, thigh, tibiaの論理中心 [rad] |
| `--amplitude-rad` | `0.002` | test変位 [rad]。code上限は0.200 rad |
| `--step-rad` | `0.001` | command increment [rad] |
| `--period-sec` | `0.500` | 通常step間隔 [s] |
| `--start-hold-sec` | `1.000` | 開始保持 [s] |
| `--peak-hold-sec` | `1.000` | peak保持 [s] |
| `--between-motion-hold-sec` | `1.000` | individual軸間などの保持 [s] |
| `--end-hold-sec` | `1.000` | 終了保持 [s] |
| `--subscriber-wait-sec` | `0.500` | publish開始前待機 [s] |
| `--topic` | `/cmdForJetson` | 出力topic |

---

# 10. Mapped-axis diagnostic publisher

対象:

```text
tools/publish_cmdforjetson_mapped_axis_replay.py
```

JSONL中の1論理axisの**変位波形**だけを、小振幅へscaleして1物理axisへ割り当てるdiagnostic tool。

| 引数 | default | 意味 |
|---|---|---|
| `--command-log` | required | source JSONL |
| `--logical-axis` | required | sourceから読むaxis `0..23` |
| `--physical-axis` | required | 実際に動かすaxis `0..23` |
| `--confirm-physical-axis` | required | 誤指定防止。`--physical-axis` と完全一致必須 |
| `--rate` | required | publish rate [Hz] |
| `--center-rad` | `0.0` | physical axis中心 [rad] |
| `--scale` | `0.05` | source変位へ掛けるscale |
| `--invert` | false | source変位の符号反転 |
| `--limit-rad` | `0.010` | centerからの最大変位clip値。code上限0.020 rad |
| `--return-step-rad` | `0.001` | 最後にcenterへ戻すstep [rad] |
| `--start-index` | `0` | source開始index |
| `--max-frames` | none | source最大frame数 |
| `--start-hold-sec` | `1.0` | 初期値保持 [s] |
| `--end-hold-sec` | `1.0` | center復帰後保持 [s] |
| `--subscriber-wait-sec` | `0.5` | publish前待機 [s] |
| `--allow-clipping` | false | scale後の波形がlimitでclipしても実行を許可 |
| `--dry-run` | false | publishせずmapped range / clippingを確認 |
| `--topic` | `/cmdForJetson` | 出力topic |

---

# 11. 現行staged rollでまず覚える値

```text
Jetson transport
  resample_factor = 2
  rate            = 10 Hz

MCU
  interpolation_time_ms = 50 ms  (firmware default / Axis11 current baseline系)
```

この3つは役割が異なる。

概念例:

```text
source q0 -------------------------- q1

factor=2:
source q0 -------- midpoint -------- q1
          100 ms         100 ms     (rate=10 Hz)

MCU:
各target受信後、そのtargetへ interpolation_time_ms で内部補間
```

したがって `resample_factor=2` は、**MCU interpolationを2回行う、あるいはMCU interpolation時間を2倍にする設定ではない**。

---

# 12. 関連文書

- [`CAN_MCU_CONFIG_GUIDE.md`](CAN_MCU_CONFIG_GUIDE.md) — CAN接続 / Config操作
- [`MCU_PARAMETER_REFERENCE.md`](MCU_PARAMETER_REFERENCE.md) — MCU側Config parameterの意味
- [`COPY_PASTE_COMMANDS.md`](COPY_PASTE_COMMANDS.md) — 実行command
- [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md) — 実機試験順序
- [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md) — Jetson/MCU/Gazeboの境界
