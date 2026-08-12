# Lily Command Data Format

更新日: 2026-08-12  
状態: 現行データ仕様の入口

この文書は、Lilyの回転候補に含まれる主要なJSON / JSONLファイルと、command recordの意味を説明する。

実行architectureは [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md)、実機操作は [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md) を優先する。

## 1. JSON と JSONL

### JSON

`.json` は、候補全体の情報や評価結果を1つの構造化objectとして保存する。

例:

```text
manifest.json
summary.json
reports/*.json
```

### JSONL

`.jsonl` は JSON Lines 形式であり、**1行が1つのcommand record** である。

例:

```text
commands.jsonl
staged/air_entry_and_hold_only_commands.jsonl
staged/roll_0_50_commands.jsonl
```

概念上:

```text
line 0 = command record 0
line 1 = command record 1
line 2 = command record 2
...
```

大きなcommand sequenceを1 recordずつ読み込めるため、trajectory保存・診断・再生に使用している。

## 2. Candidate directory の意味

現在のpre-hardware reference candidate:

```text
data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/
```

主要構成:

```text
v3_0_44_candidate_022_wide_urdf0p075/
├── README.md
├── commands.jsonl
├── manifest.json
├── summary.json
├── pre_hardware_decision.md
├── staged/
│   ├── air_entry_and_hold_only_commands.jsonl
│   ├── combined_with_hold_commands.jsonl
│   ├── roll_0_50_commands.jsonl
│   ├── roll_50_100_commands.jsonl
│   ├── roll_100_300_commands.jsonl
│   └── roll_300_end_commands.jsonl
└── reports/
```

`data/reference_candidates/` に置かれたcandidateは、開発途中の一時出力ではなく、比較・検証・実機試験のために固定したreference dataである。

## 3. 各主要ファイルの役割

### 3.1 `commands.jsonl`

**回転軌道そのもののsource command sequence。**

現在candidateでは2233 source recordsを持つ。

各recordは原則として24軸の関節角指令を含む。

```text
commands.jsonl
       ↓
source trajectory / source keyframes
```

freeze後は直接編集しない。

### 3.2 `manifest.json`

**candidateの身元・由来・採用根拠を保存するmanifest。**

代表的な内容:

- `candidate_name`
- `checksum_sha256`
- `command_count`
- source candidate / source commit
- geometry
- joint-angle gate結果
- Gazebo review結果
- URDF FKに基づく評価情報
- candidate作成時のsafety state

つまり、

```text
「このcandidateは何者か」
```

を確認するファイルである。

注意: `manifest.json` は**candidate freeze時点のsnapshot**である。後日確定したtransport profileや実機試験結果を自動的に反映するものではない。

### 3.3 `summary.json`

**candidate directoryに含まれる主要データの索引・要約。**

現在candidateでは主に:

- candidate checksum
- command count
- Gazebo freeze時評価
- staged fileのpath
- staged fileのline count
- staged fileのchecksum
- source command range

を記録している。

`manifest.json`との役割差は:

```text
manifest.json = candidateのidentity / provenance / adoption evidence
summary.json  = candidate packageのinventory / stage summary
```

である。

`summary.json` 内の古いdry-run commandやrateは、そのcandidateをfreezeした時点の証跡であり、現在の実機operation parameterとは限らない。

現在のoperationは [`BASELINE.md`](BASELINE.md) と [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md) を優先する。

### 3.4 `pre_hardware_decision.md`

**人間向けの採用判断記録。**

数値だけでなく、なぜこのcandidateをpre-hardware candidateにしたか、どの順序で実機確認するかを記録する。

### 3.5 `staged/*.jsonl`

**実機を段階的に検証するための実行入力。**

現在は:

```text
air_entry_and_hold_only
roll_0_50
roll_50_100
roll_100_300
roll_300_end
combined_with_hold
```

に分けている。

`commands.jsonl` が回転bodyのsource sequenceであるのに対し、`staged/` は安全な試験順序に合わせた実行単位である。

### 3.6 `reports/`

candidate採用時の診断・比較・評価証跡。

実行commandそのものではない。

評価方式・geometry・閾値が更新された場合、古いreportはhistorical evidenceになることがある。現在の判断基準は現行documentとcurrent evaluatorを優先する。

## 4. Command record の最重要field

JSONLの各行には多くのfieldが入ることがあるが、すべてが実機へ送られるわけではない。

### 4.1 実行に使うposition field

canonical publisherが受理するposition keyは:

```text
joint_command_rad
position
joint_positions_rad
```

優先順位は上記順。

読み込み時に24要素のfloat vectorへ正規化し、内部では `joint_command_rad` として扱う。

現在のreference candidateでは `joint_command_rad` を正本として考える。

```text
joint_command_rad = [q0, q1, ..., q23]
```

- length: exactly 24
- unit: rad
- axis order: repositoryの24-axis mappingに従う

この24要素だけが最終的に `/cmdForJetson.position` のposition commandになる。

## 5. 主なmetadata field

metadataはdiagnostics、traceability、segment判定、人間の理解に使う。通常、そのままCAN position payloadには送られない。

| field | 役割 | 実機position値か |
|---|---|---|
| `joint_command_rad` | 24軸関節角指令 [rad] | **Yes** |
| `joint_command_deg` | 人間・診断用deg表現 | No |
| `frame_index` | source frame識別 | No |
| `command_index` | generator側command index | No |
| `roll_index` | multi-roll sequence内のroll block識別 | No |
| `phase_name` | そのframeのmotion phase名 | No |
| `phase_step_index` | phase内step位置 | No |
| `phase_step_count` | phase総step数 | No |
| `base_pose` | 生成・診断上のbody pose metadata | No |
| `surface_start` / `surface_after` | surface transition metadata | No |
| `sequence_phase` | air-entry / hold等のsequence分類 | No |
| `touchdown_target` | touchdown targetを示すsemantic flag | No |
| `touchdown_hold` | touchdown posture holdを示すsemantic flag | No |
| `roll_body_included` | roll body recordを含むかの識別 | No |

`base_pose` がJSONLに存在していても、`publish_cmdforjetson_jsonl.py` がbody poseを別commandとして実機へ送るわけではない。

## 6. Index field は同じ意味ではない

JSONLには複数のindexが存在することがある。

```text
command_index
frame_index
entry_index
phase_step_index
sequence_phase_index
roll_index
```

これらを単一の「時刻index」と考えない。

典型的な意味:

- `command_index`: command generatorが付けたsequence index
- `frame_index`: source recordを識別する標準的なframe index
- `entry_index`: air-entry sequence内部のindex
- `phase_step_index`: phase内部でのstep番号
- `sequence_phase_index`: staged sequence内部のphase位置
- `roll_index`: 何番目のroll blockかを表す分類値

record生成経路によって存在するfieldは異なる。

canonical runtimeが位置指令を送るために必要なのは、これらのindexではなく24要素のposition vectorである。

## 7. `interpolation_alpha` の注意

`interpolation_alpha` は**どの層のrecordかによって意味が変わり得る**。

例えばair-entry source JSONLでは、HOMEからtarget postureへ生成したときの補間進捗を表すmetadataとして存在する。

一方、transport resampling後のrecordでは、source frame `q0` と次source frame `q1` の間のtransport interpolation位置として使われる。

したがって:

```text
source JSONLの interpolation_alpha
≠ 必ずしもtransport interpolation alphaと同じ意味
```

recordを解析するときは、source recordかtransport-derived recordかを確認する。

## 8. Source record と transport record

### Source record

frozen/staged JSONLに保存されているrecord。

```text
source JSONL
q0 ---- q1 ---- q2
```

### Transport record

`tools/publish_cmdforjetson_jsonl.py` がsource JSONLを読み、必要ならlinear resamplingして作るruntime target record。

現在profile:

```text
resample-factor = 2
rate            = 10 Hz
```

factor 2では、同一segmentの隣接source target間に1つのlinear midpointを入れる。

概念:

```text
source:
q0 -------- q1 -------- q2

transport factor=2:
q0 -- m01 -- q1 -- m12 -- q2
```

source trajectoryを上書きする処理ではない。

## 9. Transport-derived field

resampling後には、診断用として次のfieldが付くことがある。

```text
resampled_index
source_frame_index
next_source_frame_index
interpolation_alpha
resample_factor
resample_segment_key
resample_segment_group
```

これらはruntime/diagnostic layerのmetadataであり、frozen source JSONLを書き換えるためのfieldではない。

## 10. Source trajectory と actuator interpolation は別

現在のtiming chain:

```text
frozen source JSONL
        ↓
transport linear resampling
        ↓
/cmdForJetson target stream
        ↓
actuator / MCU interpolation
```

現在pre-hardware profile:

```text
transport:
  factor = 2
  rate   = 10 Hz

Gazebo MCU-equivalent:
  interpolation duration = 0.100 s
  update period          = 0.002 s
```

この3種類を混同しない。

- source trajectory resolution
- transport target rate / resampling
- actuator internal interpolation

は独立した概念である。

## 11. Air-entry JSONL の読み方

`staged/air_entry_and_hold_only_commands.jsonl` のrecordには例えば:

```json
{
  "command_index": 0,
  "joint_command_rad": [0.0, 0.0, "... 24 axes ..."],
  "joint_command_deg": [0.0, 0.0, "..."],
  "phase_name": "AIR_ENTRY_HOME_TO_CANDIDATE02_START",
  "roll_index": -1,
  "interpolation_alpha": 0.0,
  "base_pose": {"x": 0.0, "y": 0.0, "z": 0.4, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
  "touchdown_target": false,
  "roll_body_included": false
}
```

という情報が入る。

実行上の中心は `joint_command_rad`。

`touchdown_target=true` は「このjoint postureをtouchdown targetとして扱う」というsemantic markerであり、ロボットbaseを自動的に床へ降ろすactuator commandではない。

実機touchdownは、air-entry final postureを保持したままfixture/base heightを物理的に制御して行う。

## 12. Checksum の意味

reference candidateではSHA256を使って、command/dataがfreeze時から変化していないことを確認する。

現在candidate `commands.jsonl` の記録値:

```text
e60c9de63287c5c198e78e11c1da89475b2293e6de45950cf09f5f2c170304a5
```

staged fileにも個別checksumがある。

checksumが変わった場合:

```text
「同じcandidate名だが中身が変わった」
```

可能性があるため、原因を確認する。

freeze済みcandidateを修正してchecksumを更新するのではなく、必要なら新candidate / new baselineとして扱う。

## 13. Data lifecycle

開発から実機までのデータの流れ:

```text
motion algorithm / parameters
        ↓
command generation
        ↓
candidate commands.jsonl
        ↓
diagnostics / Gazebo / geometry evaluation
        ↓
reference candidate freeze
  ├─ commands.jsonl
  ├─ manifest.json
  ├─ summary.json
  ├─ reports/
  └─ staged/*.jsonl
        ↓
canonical publisher
        ↓
transport stream
        ↓
/cmdForJetson
        ↓
real MCU or Gazebo MCU-equivalent consumer
```

## 14. `testdata/` との違い

```text
testdata/
```

は、探索中candidate、diagnostic output、plot、comparison、temporary validation result等を置く領域である。

すべてが正式な実機入力ではない。

```text
data/reference_candidates/
```

に明示的にfreezeされたものをreference candidateとして扱う。

## 15. 何を直接編集してはいけないか

current hardware validation中は、特に次をsilent editしない。

```text
reference candidate commands.jsonl
reference candidate staged/*.jsonl
candidate manifest / summaryのfreeze evidence
transport factor / rate
```

実機結果により変更が必要になった場合は、新しいcandidateまたはbaselineを作り、変更理由を記録する。

## 16. 初見者が読む順序

command dataを理解する場合:

```text
README.md
  ↓
RUNTIME_ARCHITECTURE.md
  ↓
COMMAND_DATA_FORMAT.md   ← この文書
  ↓
current candidate README.md
  ↓
manifest.json / summary.json
  ↓
commands.jsonl / staged/*.jsonl
```

実機を動かす場合は、その後に:

```text
HARDWARE_OPERATION_PROCEDURE.md
```

を読む。
