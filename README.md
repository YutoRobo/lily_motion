# Lily 8脚ロボット ソフトウェア

更新日: 2026-08-12  
対象: `master`

Lilyは **8脚 × 3自由度 = 24軸** のロボットで、本リポジトリでは脚を使って本体を次の面へ倒しながら進む**回転移動**を中心に扱う。

このREADMEは**プロジェクト全体の入口**である。実機操作、正確なbaseline値、motion開発、JSONL作成、コマンド一覧、データ仕様はそれぞれ専用文書を正本とし、このREADMEには重複して持たせない。

実機を動かす場合は、必ず [`docs/HARDWARE_OPERATION_PROCEDURE.md`](docs/HARDWARE_OPERATION_PROCEDURE.md) を正本として使用する。

---

## 1. 最初にどこを読むか

| 目的 | 最初に読む文書 |
|---|---|
| プロジェクト全体を知りたい | この `README.md` |
| 文書の役割・読む順番を知りたい | [`docs/README.md`](docs/README.md) |
| 現在のcandidate / baselineを知りたい | [`docs/CURRENT_BASELINE.md`](docs/CURRENT_BASELINE.md) |
| 現在どこまで検証済みか | [`docs/VALIDATION_STATUS.md`](docs/VALIDATION_STATUS.md) |
| motion生成・評価programを使いたい | [`docs/MOTION_DEVELOPMENT_GUIDE.md`](docs/MOTION_DEVELOPMENT_GUIDE.md) |
| JSONLを新しく作りたい | [`docs/JSONL_CREATION_GUIDE.md`](docs/JSONL_CREATION_GUIDE.md) |
| 実機を動かしたい | [`docs/HARDWARE_OPERATION_PROCEDURE.md`](docs/HARDWARE_OPERATION_PROCEDURE.md) |
| 実行コマンドを探したい | [`docs/COMMAND_REFERENCE.md`](docs/COMMAND_REFERENCE.md) |
| Gazebo / 実機のruntimeを理解したい | [`docs/RUNTIME_ARCHITECTURE.md`](docs/RUNTIME_ARCHITECTURE.md) |
| JSONL field / candidate data仕様を知りたい | [`docs/COMMAND_DATA_FORMAT.md`](docs/COMMAND_DATA_FORMAT.md) |
| 関節可動域を確認したい | [`docs/HARDWARE_LIMITS.md`](docs/HARDWARE_LIMITS.md) |

文書全体の地図は [`docs/README.md`](docs/README.md) に集約する。

---

## 2. このプロジェクトでいう「回転」

通常の歩行だけではなく、脚姿勢と支持状態を切り替えながら本体を次の面へ移す。

```text
支持姿勢
  ↓
支持脚 / 遊脚を切り替える
  ↓
次の面へ移る脚姿勢を作る
  ↓
本体をrollさせる
  ↓
次の支持姿勢
```

1回の意味的な回転単位はcommand metadataの `roll_index` で追跡する。

実機へ最終的に送るのはmetadataではなく、24軸の関節指令値である。

```text
joint_command_rad : 24 values [rad]
```

詳細は [`docs/COMMAND_DATA_FORMAT.md`](docs/COMMAND_DATA_FORMAT.md) を参照する。

---

## 3. 開発から実行まで

```text
motion algorithm / parameters
        ↓
command generation
        ↓
generated candidate
        ↓
diagnostics / Gazebo / geometry evaluation
        ↓
reference candidate freeze
        ↓
staged execution data
        ↓
canonical transport
        ↓
/cmdForJetson
        ↓
real hardware / Gazebo MCU-equivalent
```

motion側の標準作業は [`docs/MOTION_DEVELOPMENT_GUIDE.md`](docs/MOTION_DEVELOPMENT_GUIDE.md)、JSONL作成例は [`docs/JSONL_CREATION_GUIDE.md`](docs/JSONL_CREATION_GUIDE.md) を使用する。

重要な区別は次の3つである。

```text
trajectory generation
transport timing
MCU-side interpolation
```

これらを同じ処理として扱わない。

---

## 4. 現在のreference candidate

current candidateの入口:

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
```

current candidateの**正確なstatus、SHA256、frame数、transport条件**は [`docs/CURRENT_BASELINE.md`](docs/CURRENT_BASELINE.md) を正本とする。

candidate固有の説明は:

- [`data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/README.md`](data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/README.md)

を参照する。

現在は従来のrisk-oriented splitに加え、同じsource trajectoryからsemantic quarterを固定している。

```text
risk-oriented sequential split
  roll_0_50
  roll_50_100
  roll_100_300
  roll_300_end

semantic cumulative stage
  roll_to_1of4
  roll_to_2of4
  roll_to_3of4
  roll_to_4of4
```

semantic quarterは**累積**である。例えば `roll_to_2of4` は第2区間だけではなく、rolling-startから2/4終了までを含む。

---

## 5. 現行runtimeの原則

Gazeboと実機は `/cmdForJetson` まで同じ経路を使う。

```text
frozen / staged JSONL
        ↓
tools/publish_cmdforjetson_jsonl.py
        ↓
lily_motion_v3.command_stream
        ↓
/cmdForJetson
        │
        ├─ REAL
        │    ↓
        │  tools/can_interface/statemachine/
        │    ↓
        │   CAN → real MCU → motor
        │
        └─ GAZEBO
             ↓
           tools/gazebo/mcu_position_interpolator_node.py
             ↓
           Gazebo joint controllers
```

原則:

- canonical publisherにGazebo/realのbackend分岐を作らない。
- Gazebo確認時はCAN StateMachineを同時に動かさない。
- 実機試験時はGazebo MCU nodeを同時に動かさない。
- `/cmdForJetson` に意図しない複数consumerを接続しない。

詳細は [`docs/RUNTIME_ARCHITECTURE.md`](docs/RUNTIME_ARCHITECTURE.md) を参照する。

---

## 6. 実機試験の考え方

初回実機試験は、いきなりfull rollを実行しない。

```text
single axis
  ↓
one leg
  ↓
suspended air-entry
  ↓
controlled touchdown
  ↓
risk-oriented split roll
  ↓
semantic quarter / full sequence
```

正確な順序、STOP条件、安全確認は [`docs/HARDWARE_OPERATION_PROCEDURE.md`](docs/HARDWARE_OPERATION_PROCEDURE.md) を使用する。

コマンドを探すだけの場合は [`docs/COMMAND_REFERENCE.md`](docs/COMMAND_REFERENCE.md) を使用する。

---

## 7. Repository map

```text
lily_motion/
├── README.md                    # project entry point
├── lily_motion_v3/              # motion / geometry / shared runtime core
├── tools/
│   ├── command_generation/      # command generation / derived stage tools
│   ├── diagnostics/             # evaluation / diagnostics
│   ├── gazebo/                  # Gazebo execution / MCU-equivalent path
│   ├── can_interface/           # current CAN StateMachine / UI / emulator
│   └── publish_cmdforjetson_*   # canonical and staged-test publishers
├── data/
│   ├── reference_candidates/    # reviewed/frozen candidates
│   └── baselines/               # retained comparison baselines
├── docs/                        # current docs + development history notes
├── tests/                       # regression tests
├── testdata/                    # exploratory/generated evaluation outputs
└── archive/                     # legacy scripts / stale operational docs
```

`docs/v3_0_*` は開発判断の履歴であり、current runtimeやcurrent baselineを上書きしない。

---

## 8. 文書の正本ルール

同じ数値やstatusを複数文書で正本化しない。

| 情報 | 正本 |
|---|---|
| current candidate / current baseline | `docs/CURRENT_BASELINE.md` |
| current verification status | `docs/VALIDATION_STATUS.md` |
| motion開発programの使い方 | `docs/MOTION_DEVELOPMENT_GUIDE.md` |
| JSONLの作成手順 | `docs/JSONL_CREATION_GUIDE.md` |
| 実機試験順序 / safety | `docs/HARDWARE_OPERATION_PROCEDURE.md` |
| exact commands | `docs/COMMAND_REFERENCE.md` |
| runtime architecture | `docs/RUNTIME_ARCHITECTURE.md` |
| JSON / JSONL / candidate data contract | `docs/COMMAND_DATA_FORMAT.md` |
| hardware joint limits | `docs/HARDWARE_LIMITS.md` |
| immutable historical baseline evidence | `docs/BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md` |

README類はこれらを**案内する**役割とし、変わりやすいSHA、frame数、試験結果を必要以上に複製しない。

旧文書名は過去リンク互換のためstubとして残す場合があるが、current正本として使用しない。

---

## 9. Change control

次をsilent変更しない。

- frozen candidate JSONL
- staged JSONL
- semantic quarter JSONL / manifest
- transport resample factor / rate
- `/cmdForJetson` semantics
- CAN StateMachine mapping
- Gazebo MCU interpolation assumptions
- hardware joint limit definition

変更が必要なら、新しいcandidate / baseline / validation recordとして明示的に残す。

---

## 10. 次に読む

motion developer:

```text
README.md
  ↓
docs/MOTION_DEVELOPMENT_GUIDE.md
  ↓
docs/JSONL_CREATION_GUIDE.md
  ↓
docs/COMMAND_DATA_FORMAT.md
```

実機operator:

```text
README.md
  ↓
docs/CURRENT_BASELINE.md
  ↓
docs/VALIDATION_STATUS.md
  ↓
docs/HARDWARE_OPERATION_PROCEDURE.md
  ↓
docs/COMMAND_REFERENCE.md
```
