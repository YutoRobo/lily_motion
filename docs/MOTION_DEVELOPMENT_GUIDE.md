# Lily Motion Development Guide

更新日: 2026-08-12

この文書は、Lily回転motionを**開発・評価・JSONL化する人の正本ガイド**である。

「どのprogramがあるか」ではなく、**何をしたいか → 何を実行するか → 何が出るか → 次に何をするか**の順で整理する。

実機操作はこの文書の範囲外である。実機は [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md) を使用する。

JSONLを実際に作る具体例は [`JSONL_CREATION_GUIDE.md`](JSONL_CREATION_GUIDE.md) を参照する。

---

## 1. Motion開発の全体フロー

標準フローは次のとおり。

```text
current baseline確認
  ↓
変更したいmotion parameter / logicを決める
  ↓
1 candidateを生成してwhole-roll評価
  ↓
必要ならfailure diagnosis
  ↓
必要ならparameter sweep
  ↓
visualizationで姿勢を確認
  ↓
command JSONLへexport
  ↓
JSONL自体をdiagnose
  ↓
development direct Gazebo replay
  ↓
採否判断
  ↓
review済みcandidateとしてfreeze
  ↓
staged / derived execution dataを生成
  ↓
hardware-equivalent Gazebo
  ↓
実機側へ引き渡し
```

重要:

- `testdata/` は開発・探索用。
- `data/reference_candidates/` はreview済み/frozen data。
- JSONLが生成できたことと、reference candidateへ採用できることは別。
- reference candidateへfreezeした後はsilent編集しない。

current状態は [`CURRENT_BASELINE.md`](CURRENT_BASELINE.md) を正本とする。

---

## 2. 最初に見る表

| やりたいこと | 標準tool | 主な出力 |
|---|---|---|
| 1 candidateを生成して総合評価したい | `tools/diagnostics/run_v3_0_whole_roll_eval.py` | evaluation JSON / summary |
| 失敗原因を詳しく見たい | `tools/diagnostics/run_v3_0_diagnose_failures.py` | failure diagnosis |
| 複数parameterを探索したい | `tools/diagnostics/run_v3_0_parameter_sweep.py` | sweep ranking JSON |
| 姿勢を3Dで見たい | `tools/diagnostics/run_v3_0_visualize_roll.py` | PNG / HTML / manifest |
| command JSONLを作りたい | `tools/command_generation/run_v3_0_export_commands.py` | `*.jsonl` |
| 既存JSONLの角度範囲・隣接jumpを見たい | `tools/diagnostics/run_v3_0_command_diagnostics.py` | command diagnostics |
| JSONLをGazeboで直接見たい | `tools/gazebo/run_v3_0_gazebo_replay.py` | Gazebo replay |
| semantic 1/4〜4/4を作りたい | `tools/command_generation/build_roll_quarter_stages.py` | cumulative quarter JSONL + manifest |
| JSONLをofflineでsmooth/resampleしたい | `tools/command_generation/run_v3_0_resample_commands.py` | transformed JSONL |
| contact plan自体を調べたい | `tools/diagnostics/run_v3_0_contact_plan_catalog.py` | catalog |
| near-contactを局所解析したい | `tools/diagnostics/run_v3_0_near_contact_phase_scan.py` | phase scan |

---

## 3. 最重要: 各toolのparameterを揃える

motion系toolの一部は、historical developmentの都合で**default値が完全には揃っていない**。

例えば `whole_roll_eval.py`、`diagnose_failures.py`、`visualize_roll.py`、`export_commands.py` は同系統のgeneratorを使うが、defaultの `steps_per_phase`、`lift_height`、`body_roll_pitch_deg`、`filter_window` 等が異なる箇所がある。

したがってcandidateを比較するときは、default任せにせず、重要parameterをcommand lineで明示する。

例:

```text
surface_id
trajectory_mode
contact_plan_variant
steps_per_phase
lift_height
clearance_height
candidate_support_shift_x
candidate_support_drop_z
body_roll_pitch_deg
filter_window
contact_preserving_filter
```

**評価したparameterと、exportしたparameterが一致していることを必ず確認する。**

---

## 4. 1 candidateを作って評価する

標準入口:

```text
tools/diagnostics/run_v3_0_whole_roll_eval.py
```

このtoolは、generatorからcandidateを生成し、raw / filtered command、ground clearance、inter-leg clearance、joint limit、contact drift、IK failure等をまとめて評価する。

最小確認:

```bash
python tools/diagnostics/run_v3_0_whole_roll_eval.py \
  --summary-only
```

開発時はparameterを明示する。

例:

```bash
python tools/diagnostics/run_v3_0_whole_roll_eval.py \
  --surface-id 1 \
  --trajectory-mode phase \
  --contact-plan-variant default \
  --steps-per-phase 8 \
  --lift-height 0.08 \
  --clearance-height 0.06 \
  --candidate-support-shift-x 0.04 \
  --candidate-support-drop-z -0.02 \
  --body-roll-pitch-deg 90 \
  --filter-window 5 \
  --summary-only \
  --output testdata/tutorial/whole_roll_eval.json
```

このcommandは**使い方の例**であり、現在のfrozen 2233-frame candidateの生成parameterを意味しない。

主に見る項目:

```text
candidate_completed
whole_roll_success_by_filtered_geometry
generator_ik_failure_count
filtered_penetration_count
filtered_min_clearance_m
filtered_near_count
filtered_max_second_joint_deg
filtered_max_joint_delta_deg
filtered_max_contact_drift_m
```

---

## 5. Failure diagnosis

標準tool:

```text
tools/diagnostics/run_v3_0_diagnose_failures.py
```

whole-roll summaryだけでは原因が分からないときに使う。

例:

```bash
python tools/diagnostics/run_v3_0_diagnose_failures.py \
  --surface-id 1 \
  --trajectory-mode phase \
  --contact-plan-variant default \
  --steps-per-phase 8 \
  --lift-height 0.08 \
  --clearance-height 0.06 \
  --candidate-support-shift-x 0.04 \
  --candidate-support-drop-z -0.02 \
  --body-roll-pitch-deg 90 \
  --filter-window 5 \
  --output testdata/tutorial/failure_diagnosis.json
```

whole-roll evalと同じcandidateを調べたい場合は、parameterを一致させる。

---

## 6. Parameter sweep

標準tool:

```text
tools/diagnostics/run_v3_0_parameter_sweep.py
```

複数parameterの組合せを比較し、feasibility、geometry/contact、smoothnessを含むscoreでrankする。

探索範囲を小さく始める例:

```bash
python tools/diagnostics/run_v3_0_parameter_sweep.py \
  --trajectory-modes phase \
  --contact-plan-variants default \
  --steps-per-phase 6,8 \
  --lift-heights 0.06,0.08 \
  --clearance-heights 0.06 \
  --candidate-support-shift-xs 0.02,0.04 \
  --candidate-support-drop-zs=-0.02,0.0 \
  --filter-windows 3,5 \
  --output testdata/tutorial/parameter_sweep.json
```

探索結果の `best_case` をそのまま採用しない。

必ず:

```text
best / top cases
  ↓
whole-roll eval再確認
  ↓
failure diagnosis
  ↓
visualization
  ↓
Gazebo
```

と進める。

---

## 7. Visualization

標準tool:

```text
tools/diagnostics/run_v3_0_visualize_roll.py
```

candidateを再生成して、重要frameをPNG / HTMLへ出す。

例:

```bash
python tools/diagnostics/run_v3_0_visualize_roll.py \
  --surface-id 1 \
  --trajectory-mode phase \
  --contact-plan-variant default \
  --steps-per-phase 8 \
  --lift-height 0.08 \
  --clearance-height 0.06 \
  --candidate-support-shift-x 0.04 \
  --candidate-support-drop-z -0.02 \
  --body-roll-pitch-deg 90 \
  --filter-window 5 \
  --command-source filtered \
  --output-dir testdata/tutorial/visualization
```

ここでも評価時parameterを明示的に揃える。

---

## 8. JSONL export

標準tool:

```text
tools/command_generation/run_v3_0_export_commands.py
```

現行toolでは主に次のprofileを扱う。

```text
native
legacy_style
legacy_roll_spec
imported_reference
```

新規motion開発では基本的に `native` を入口とする。

例:

```bash
python tools/command_generation/run_v3_0_export_commands.py \
  --profile native \
  --surface-id 1 \
  --trajectory-mode phase \
  --contact-plan-variant default \
  --steps-per-phase 8 \
  --lift-height 0.08 \
  --clearance-height 0.06 \
  --candidate-support-shift-x 0.04 \
  --candidate-support-drop-z -0.02 \
  --body-roll-pitch-deg 90 \
  --command-source filtered \
  --filter-window 5 \
  --output testdata/tutorial/one_roll_commands.jsonl
```

この汎用exportは現状、**1 roll candidateの標準JSONL化入口**である。

現在のfrozen reference candidateは4 roll / 2233 frameであり、この1 commandだけでbyte-for-byte再生成できるものではない。詳細は [`JSONL_CREATION_GUIDE.md`](JSONL_CREATION_GUIDE.md) を参照する。

---

## 9. Existing JSONLをdiagnoseする

標準tool:

```text
tools/diagnostics/run_v3_0_command_diagnostics.py
```

例:

```bash
python tools/diagnostics/run_v3_0_command_diagnostics.py \
  --command-log testdata/tutorial/one_roll_commands.jsonl
```

主に確認するもの:

```text
frame_count
max_delta_deg
max_adjacent_delta_deg
worst_transition
phase_summary
per-joint min/max
```

generator評価と違い、このtoolは**すでに存在するJSONLそのもの**を見る。

---

## 10. Development direct Gazebo

JSONLの姿勢・動きの目視確認:

```text
tools/gazebo/run_v3_0_gazebo_replay.py
```

例:

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --command-log testdata/tutorial/one_roll_commands.jsonl \
  --strict-command-log-input \
  --rate 15 \
  --diagnose-command-log
```

これはtrajectory development用であり、formal hardware-equivalent pathではない。

実機等価比較は:

```text
JSONL
→ publish_cmdforjetson_jsonl.py
→ /cmdForJetson
→ mcu_position_interpolator_node.py
→ Gazebo
```

を使用する。詳細は [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md) を参照する。

---

## 11. Candidateをfreezeする

現状、探索結果をreference candidateへ自動昇格させる万能commandはない。

freeze前に最低限:

```text
1. source parameter / source commitを記録
2. command JSONLを固定
3. SHA256を記録
4. command diagnostics
5. geometry / joint limit確認
6. Gazebo visual review
7. manifest / summary / decision recordを作成
8. data/reference_candidates/<candidate>/ へ配置
```

current candidate packageを見本にする:

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
├── README.md
├── commands.jsonl
├── manifest.json
├── summary.json
├── pre_hardware_decision.md
├── reports/
└── staged/
```

candidate packageのdata contractは [`COMMAND_DATA_FORMAT.md`](COMMAND_DATA_FORMAT.md) を参照する。

---

## 12. Freeze後のderived data

semantic quarter生成:

```bash
python2 tools/command_generation/build_roll_quarter_stages.py \
  --command-log <candidate>/commands.jsonl \
  --output-dir <candidate>/staged
```

これは `roll_index` が4つのcontiguous blockとして存在するcandidate向けである。

生成物:

```text
roll_to_1of4_commands.jsonl
roll_to_2of4_commands.jsonl
roll_to_3of4_commands.jsonl
roll_to_4of4_commands.jsonl
quarter_stage_manifest.json
```

新規candidateでroll構成が異なる場合、`expected_roll_count`前提を確認してから使用する。

---

## 13. Tool分類

### 13.1 標準利用

```text
tools/diagnostics/run_v3_0_whole_roll_eval.py
tools/diagnostics/run_v3_0_diagnose_failures.py
tools/diagnostics/run_v3_0_parameter_sweep.py
tools/diagnostics/run_v3_0_visualize_roll.py
tools/diagnostics/run_v3_0_command_diagnostics.py
tools/command_generation/run_v3_0_export_commands.py
tools/command_generation/build_roll_quarter_stages.py
tools/gazebo/run_v3_0_gazebo_replay.py
```

### 13.2 補助・局所解析

```text
tools/diagnostics/run_v3_0_contact_plan_catalog.py
tools/diagnostics/run_v3_0_goal_oriented_sweep.py
tools/diagnostics/run_v3_0_near_contact_phase_scan.py
```

必要な問題が明確なときだけ使う。

### 13.3 Development transformation

```text
tools/command_generation/run_v3_0_resample_commands.py
```

既存JSONLをofflineでresample / smooth / unwrapする**Gazebo preview用変換tool**。

これはruntimeの:

```text
publish_cmdforjetson_jsonl.py --resample-factor ...
```

とは別物である。

reference candidateをこのtoolでsilent加工しない。

### 13.4 Legacy / historical compatibility

```text
tools/diagnostics/run_v3_0_legacy_style_eval.py
tools/diagnostics/run_v3_0_verify_provisional_baseline.py
tools/command_generation/run_v3_0_import_legacy_reference.py
tools/command_generation/run_v3_0_42c_candidate02_second_joint_softlimit.py
```

特に `run_v3_0_42c_candidate02_second_joint_softlimit.py` はcandidate02専用の履歴toolであり、新規candidateの一般的soft-limit処理として使わない。

---

## 14. 現在のfrozen candidateとの関係

current reference candidate:

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
```

このcandidateは過去の探索・local fixを経てfreezeされたもので、manifestにはsourceとして:

```text
testdata/visual_near_contact_local_fix_candidate/candidate_022_wide
```

source commitとして:

```text
178be686111962e3cf2741e47e495c8966bbe888
```

が記録されている。

現行repoでは、その探索過程の専用runnerはarchive / historical側へ整理されており、**現在の汎用export toolだけで2233-frame sourceを完全再構築することは保証されない**。

したがって:

- 現在の実機candidateを使う → frozen `commands.jsonl` を使う。
- 新しいmotionを開発する → このガイドの標準flowを使う。
- current candidateを完全再生成したい → historical provenanceを追跡する別の再現作業として扱う。

---

## 15. 開発時の禁止事項

- `data/reference_candidates/` へ探索outputを直接書かない。
- evaluationとexportでparameterをdefault任せにして変えない。
- offline smoothingとruntime transport resamplingを混同しない。
- development direct Gazeboで動いたことをhardware-equivalent PASSと扱わない。
- archiveの旧runnerをcurrent standard entry pointとして復活させない。
- frozen candidateを上書きして同じcandidate名を使い続けない。

---

## 16. 次に読む文書

JSONLを実際に作る:

- [`JSONL_CREATION_GUIDE.md`](JSONL_CREATION_GUIDE.md)

JSONL field / candidate package仕様:

- [`COMMAND_DATA_FORMAT.md`](COMMAND_DATA_FORMAT.md)

current candidate:

- [`CURRENT_BASELINE.md`](CURRENT_BASELINE.md)

実行architecture:

- [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md)

実機試験:

- [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md)
