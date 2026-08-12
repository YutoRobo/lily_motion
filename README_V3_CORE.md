# Lily Motion v3-core

更新日: 2026-08-12

`lily_motion_v3/` はLily回転移動の**motion生成・運動学・評価core**である。

この文書は**motion developer向けの入口**であり、実機操作、current baseline、exact commandの正本ではない。

実機・current状態を確認する場合は先に以下を参照する。

- project入口: [`README.md`](README.md)
- docs map: [`docs/README.md`](docs/README.md)
- current baseline: [`docs/BASELINE.md`](docs/BASELINE.md)
- runtime: [`docs/RUNTIME_ARCHITECTURE.md`](docs/RUNTIME_ARCHITECTURE.md)
- data contract: [`docs/COMMAND_DATA_FORMAT.md`](docs/COMMAND_DATA_FORMAT.md)
- hardware operation: [`docs/HARDWARE_OPERATION_PROCEDURE.md`](docs/HARDWARE_OPERATION_PROCEDURE.md)

---

## 1. v3-coreの責務

主な責務:

- robot geometry
- FK / IK
- contact plan
- RF phase handling
- repeated roll generation
- command sequence generation
- legacy/reference comparison
- constraint evaluation support
- command transportの共有pure logic

現行coreの代表module:

```text
lily_motion_v3/
├── robot_geometry.py
├── command_stream.py
├── command_timing.py
└── gazebo_actuator_interpolator.py
```

`command_stream.py` / `command_timing.py` はGazeboやCANを直接知らない。

Gazebo MCU-equivalent処理はGazebo側の責務として分離する。

---

## 2. Architecture boundary

motion coreから実行までの境界:

```text
motion algorithm
  ↓
source command JSONL
  ↓
review / diagnostics / Gazebo
  ↓
reference candidate freeze
  ↓
staged JSONL
  ↓
tools/publish_cmdforjetson_jsonl.py
  ↓
/cmdForJetson
```

ここから先だけがreal / Gazeboに分岐する。

```text
REAL:
  StateMachine → CAN → real MCU

GAZEBO:
  mcu_position_interpolator_node.py → Gazebo
```

詳細は [`docs/RUNTIME_ARCHITECTURE.md`](docs/RUNTIME_ARCHITECTURE.md) を正本とする。

---

## 3. Geometry

現行geometryの実装source:

```text
lily_motion_v3/robot_geometry.py
```

hardware joint limitの正本:

- [`docs/HARDWARE_LIMITS.md`](docs/HARDWARE_LIMITS.md)

link length更新の判断経緯:

- [`docs/kinematics_link_length_update_0p075.md`](docs/kinematics_link_length_update_0p075.md)

このREADMEには変わりうるgeometry数値を重複して正本化しない。

---

## 4. Data lifecycle

開発中のcommandと実機候補を分ける。

```text
algorithm / parameter change
  ↓
generated command JSONL
  ↓
testdata/ で評価
  ↓
diagnostics / Gazebo review
  ↓
reference candidateへfreeze
  ↓
staged / derived execution data
```

原則:

- exploratory outputは `testdata/`
- `data/reference_candidates/` はreview済み/frozen data
- frozen candidateを直接編集しない
- derived stageはsourceとの関係とchecksumを残す
- 生成できたことと実機承認を同一視しない

current candidateは [`docs/BASELINE.md`](docs/BASELINE.md) を参照する。

JSONL field、candidate directory、source/transport recordの意味は [`docs/COMMAND_DATA_FORMAT.md`](docs/COMMAND_DATA_FORMAT.md) を参照する。

---

## 5. Motion evaluation

### Whole-roll evaluation

```bash
python tools/diagnostics/run_v3_0_whole_roll_eval.py --summary-only
```

### Failure diagnosis

```bash
python tools/diagnostics/run_v3_0_diagnose_failures.py
```

### Visualization

```bash
python tools/diagnostics/run_v3_0_visualize_roll.py --help
```

### Parameter sweep

```bash
python tools/diagnostics/run_v3_0_parameter_sweep.py --help
```

評価結果を採用candidateへ反映する場合は、元parameter、geometry、source command、report、checksumを追跡できる形でfreezeする。

---

## 6. Command generation

代表tool:

```bash
python tools/command_generation/run_v3_0_export_commands.py --help
python tools/command_generation/run_v3_0_import_legacy_reference.py --help
python tools/command_generation/run_v3_0_resample_commands.py --help
python2 tools/command_generation/build_roll_quarter_stages.py --help
```

`build_roll_quarter_stages.py` はcontiguous `roll_index` blockを用いてsemantic cumulative stageを生成する。

```text
roll_to_1of4 = rolling-start → roll_index 0終了
roll_to_2of4 = rolling-start → roll_index 1終了
roll_to_3of4 = rolling-start → roll_index 2終了
roll_to_4of4 = rolling-start → roll_index 3終了
```

単純なframe数4等分ではない。

このtoolはdata生成のみを行い、ROS publishやCAN openはしない。

---

## 7. Gazeboの2用途

### Development direct replay

```text
tools/gazebo/run_v3_0_gazebo_replay.py
```

用途:

- trajectory開発
- posture目視
- diagnostic replay
- historical reproduction

これはformal hardware-equivalent pathではない。

### Hardware-equivalent runtime verification

実機と同じupstream command pathを確認する場合:

```text
staged JSONL
→ tools/publish_cmdforjetson_jsonl.py
→ /cmdForJetson
→ tools/gazebo/mcu_position_interpolator_node.py
→ Gazebo
```

transport profileやcurrent検証結果はこのREADMEへ固定せず、以下を参照する。

- [`docs/RUNTIME_ARCHITECTURE.md`](docs/RUNTIME_ARCHITECTURE.md)
- [`docs/BASELINE.md`](docs/BASELINE.md)
- [`docs/HARDWARE_PRETEST_STATUS.md`](docs/HARDWARE_PRETEST_STATUS.md)

---

## 8. Development rules

- hidden legacy runtime dependencyを増やさない
- current geometry sourceを明示する
- source trajectoryとtransport resamplingを混同しない
- transport timingとMCU interpolation timingを混同しない
- `/cmdForJetson` より上流にreal/Gazebo別trajectoryを作らない
- exploratory outputをreference candidateへ直接書き込まない
- current operation用commandをdevelopment direct replayと混同しない
- old `docs/v3_0_*` の前提をcurrent仕様として再利用しない

---

## 9. Historical boundary

```text
archive/
docs/v3_0_*
```

は履歴・再現用である。

過去noteに書かれたcandidate、geometry、limit、rate、smoothing条件は、そのversion時点の記録として読む。

current仕様は [`docs/README.md`](docs/README.md) に列挙されたauthoritative documentsを優先する。
