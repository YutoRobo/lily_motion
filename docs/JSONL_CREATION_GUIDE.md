# Lily JSONL Creation Guide

更新日: 2026-08-12

この文書は、Lily motionの**command JSONLをどう作るか**を具体例で説明する。

対象は2つに分ける。

1. **現行codeから新しいdevelopment JSONLを作る方法**
2. **現在のfrozen reference candidateをどう扱うか**

この2つを混同しない。

motion開発全体の流れは [`MOTION_DEVELOPMENT_GUIDE.md`](MOTION_DEVELOPMENT_GUIDE.md) を参照する。

---

## 1. まず理解すること

Lilyのcommand JSONLは基本的に:

```text
1 line = 1 command record
```

である。

実行上の中心fieldは:

```text
joint_command_rad
```

で、24軸のpositionをradで持つ。

例の概念形:

```json
{
  "command_index": 0,
  "frame_index": 0,
  "phase_name": "...",
  "joint_command_rad": [0.0, 0.0, 0.0, "... total 24 values ..."]
}
```

正確なfield仕様は [`COMMAND_DATA_FORMAT.md`](COMMAND_DATA_FORMAT.md) を正本とする。

---

## 2. Current frozen JSONL

現在のreference candidate:

```bash
CANDIDATE=data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075
```

source command:

```text
$CANDIDATE/commands.jsonl
```

現在のidentity:

```text
frame count = 2233
SHA256      = e60c9de63287c5c198e78e11c1da89475b2293e6de45950cf09f5f2c170304a5
```

確認:

```bash
wc -l "$CANDIDATE/commands.jsonl"
sha256sum "$CANDIDATE/commands.jsonl"
```

これは**既にfreeze済みの実行候補**である。作り直したり上書きしたりせず、そのまま使う。

---

## 3. Current frozen JSONLは1 commandでは再生成できない

現在のmanifestにはsource provenanceとして:

```text
source_candidate:
testdata/visual_near_contact_local_fix_candidate/candidate_022_wide

source_commit:
178be686111962e3cf2741e47e495c8966bbe888
```

が残っている。

現在の2233-frame JSONLは、過去のrepeated-roll生成、candidate探索、局所修正、評価、freezeを経た成果物である。

現行の汎用:

```text
tools/command_generation/run_v3_0_export_commands.py
```

は、新しいcandidateをJSONL化する標準入口だが、現在のfrozen 4-roll / 2233-frame fileを**byte-for-byte再生成する専用commandではない**。

したがって、目的を分ける。

```text
current hardware candidateを使いたい
  → frozen commands.jsonlを使用

新しいmotion JSONLを作りたい
  → 現行generic generator/export flowを使用

current candidateを歴史的に完全再現したい
  → source commit / historical runner / archived development chainを追跡
```

---

## 4. 新しいdevelopment JSONLを作る最小例

最初にworking directoryを作る。

```bash
mkdir -p testdata/tutorial_jsonl
```

次の例は、現行native generatorから**1 rollのdevelopment candidate**を生成してJSONLへexportする。

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
  --output testdata/tutorial_jsonl/one_roll_commands.jsonl
```

このparameter setは**使い方を示すexample**であり、current frozen candidateの採用parameterそのものを意味しない。

出力:

```text
testdata/tutorial_jsonl/one_roll_commands.jsonl
```

---

## 5. Export前に同じparameterで評価する

本来はexportより先にwhole-roll evaluationを行う。

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
  --output testdata/tutorial_jsonl/whole_roll_eval.json
```

重要なのは、evaluationとexportで:

```text
surface-id
trajectory-mode
contact-plan-variant
steps-per-phase
lift-height
clearance-height
candidate-support-shift-x
candidate-support-drop-z
body-roll-pitch-deg
filter-window
```

等を一致させること。

各toolのdefault値が完全には同じでないため、candidate比較では重要parameterを明示する。

---

## 6. JSONLができた直後の確認

### 6.1 行数

```bash
wc -l testdata/tutorial_jsonl/one_roll_commands.jsonl
```

### 6.2 SHA256

```bash
sha256sum testdata/tutorial_jsonl/one_roll_commands.jsonl
```

SHAはcandidate採用前に記録する。

### 6.3 先頭record

```bash
head -n 1 testdata/tutorial_jsonl/one_roll_commands.jsonl
```

少なくとも `joint_command_rad` が24要素であることを確認する。

### 6.4 command diagnostics

```bash
python tools/diagnostics/run_v3_0_command_diagnostics.py \
  --command-log testdata/tutorial_jsonl/one_roll_commands.jsonl \
  > testdata/tutorial_jsonl/command_diagnostics.json
```

見る項目:

```text
frame_count
max_delta_deg
max_adjacent_delta_deg
worst_transition
phase_summary
joint min/max
```

---

## 7. Failureがある場合

同じparameterでfailure diagnosisを実行する。

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
  --output testdata/tutorial_jsonl/failure_diagnosis.json
```

修正後は別filename / 別directoryへ出力し、前candidateを上書きしない。

---

## 8. Visualization

同じparameterを使って静的visualizationを生成する。

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
  --output-dir testdata/tutorial_jsonl/visualization
```

このtoolはJSONLを読み直すのではなくcandidateを再生成するため、parameter一致が特に重要。

---

## 9. Development Gazebo replay

作ったJSONLそのものをGazeboで確認する。

```bash
python tools/gazebo/run_v3_0_gazebo_replay.py \
  --command-log testdata/tutorial_jsonl/one_roll_commands.jsonl \
  --strict-command-log-input \
  --rate 15 \
  --diagnose-command-log
```

ここでは:

- posture
- discontinuity
- collision / floor interactionの見た目
- start / end behavior

を確認する。

これはdevelopment direct replayであり、hardware-equivalent runtime validationとは別。

---

## 10. Offline smoothing / resamplingを行う場合

既存JSONLを開発上加工するtool:

```text
tools/command_generation/run_v3_0_resample_commands.py
```

例:

```bash
python tools/command_generation/run_v3_0_resample_commands.py \
  --input testdata/tutorial_jsonl/one_roll_commands.jsonl \
  --output testdata/tutorial_jsonl/one_roll_smoothed.jsonl \
  --resample-factor 1 \
  --smooth-window 3 \
  --diagnose-boundaries
```

このtoolは**source JSONL自体を新しいJSONLへ変換するdevelopment tool**である。

runtimeの:

```text
publish_cmdforjetson_jsonl.py --resample-factor 2
```

とは意味が異なる。

```text
offline resample/smooth
  = source trajectoryを変える

runtime transport resample
  = frozen sourceを変えず送信targetを補間する
```

reference candidateをoffline toolで加工する場合は、必ず新candidateとして扱う。

---

## 11. Legacy roll reproduction example

現行export toolには `legacy_roll_spec` profileも残っている。

例えばhistorical roll specificationの代表parameter:

```text
move_dist    = 0.4
support_dist = 0.7
max_step     = 30
body z       = 0.35
```

を使う場合:

```bash
python tools/command_generation/run_v3_0_export_commands.py \
  --profile legacy_roll_spec \
  --surface-id 1 \
  --move-dist 0.4 \
  --support-dist 0.7 \
  --max-step 30 \
  --legacy-body-z 0.35 \
  --command-source filtered \
  --filter-window 5 \
  --output testdata/tutorial_jsonl/legacy_spec_one_roll.jsonl
```

これは**historical scaffoldの1 roll reproduction example**であり、current frozen 2233-frame candidateそのものではない。

---

## 12. Reference candidateへ昇格するとき

`testdata/` のJSONLをいきなり `data/reference_candidates/` へcopyしない。

最低限、次を揃える。

```text
commands.jsonl
SHA256
source commit
motion/generation parameters
whole-roll evaluation
command diagnostics
visual/Gazebo review
geometry / joint-limit evidence
README
manifest.json
summary.json
adoption decision
```

current candidate packageを雛形として見る:

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
```

ただし数値や古いstatusをそのままcopyせず、新candidateの実測・評価結果で作る。

---

## 13. Repeated roll / semantic quarterについて

current frozen candidateは4つのcontiguous `roll_index` blockを持つ。

そのような**freeze済みrepeated-roll JSONL**ができた後、semantic cumulative quarterを作る場合:

```bash
python2 tools/command_generation/build_roll_quarter_stages.py \
  --command-log <candidate>/commands.jsonl \
  --output-dir <candidate>/staged \
  --dry-run
```

境界を確認した後:

```bash
python2 tools/command_generation/build_roll_quarter_stages.py \
  --command-log <candidate>/commands.jsonl \
  --output-dir <candidate>/staged
```

このbuilderはrepeated rollそのものを生成するtoolではない。

```text
repeated-roll source JSONLを作る
        ↓
freeze / review
        ↓
quarter builderでcumulative prefixを作る
```

という順序。

---

## 14. Current candidateを作成例として読むポイント

current candidateは、完成したpackageの見本として非常に有用。

確認順:

```text
README.md
  ↓
manifest.json
  ↓
summary.json
  ↓
commands.jsonl
  ↓
reports/
  ↓
staged/
```

特にmanifestで見るもの:

```text
candidate_name
source_candidate
source_commit
checksum_sha256
command_count
geometry source
angle gate result
Gazebo result
hardware status
```

summaryで見るもの:

```text
staged file ranges
staged checksums
strict replay dry-run evidence
```

この形を、新candidate freeze時の最低限のtemplateとして使う。

---

## 15. 将来改善すべきgap

現在のgeneric toolchainには、**current 4-roll trajectoryと同じclassのrepeated-roll source JSONLを1 commandで再生成し、parameter manifestまで自動保存するcanonical generator CLI**がない。

そのため今後motion algorithmを再開する際は、次の改善価値が高い。

```text
motion config file
  ↓
canonical repeated-roll generator
  ↓
commands.jsonl
  + generation_manifest.json
  + evaluation.json
```

これができれば、current candidateのようなtrajectoryも「どのcommandで作ったか」を完全に再現できるようになる。

現時点では、このgapを隠さず、frozen provenanceと現行generic flowを分けて管理する。

---

## 16. 関連文書

- [`MOTION_DEVELOPMENT_GUIDE.md`](MOTION_DEVELOPMENT_GUIDE.md) — motion tool全体の使い方
- [`COMMAND_DATA_FORMAT.md`](COMMAND_DATA_FORMAT.md) — JSONL / candidate data仕様
- [`CURRENT_BASELINE.md`](CURRENT_BASELINE.md) — current frozen candidate
- [`COMMAND_REFERENCE.md`](COMMAND_REFERENCE.md) — 実行command
- [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md) — runtime path
