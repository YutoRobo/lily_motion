# Lily Documentation Map

更新日: 2026-08-12

この文書は `docs/` の**索引と文書責務の正本**である。

目的は、同じstatus・数値・手順を複数文書に重複させず、**どの情報をどの文書で確認すべきか**を明確にすることである。

---

## 1. 初見者の基本ルート

### プロジェクト全体を理解する

```text
../README.md
  ↓
README.md  ← この文書
  ↓
RUNTIME_ARCHITECTURE.md
  ↓
COMMAND_DATA_FORMAT.md
```

### motion生成・評価を開発する

```text
../README.md
  ↓
../README_V3_CORE.md
  ↓
RUNTIME_ARCHITECTURE.md
  ↓
COMMAND_DATA_FORMAT.md
```

### current candidateを理解する

```text
../README.md
  ↓
BASELINE.md
  ↓
../data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/README.md
  ↓
manifest.json / summary.json / quarter_stage_manifest.json
```

### 実機試験を行う

```text
../README.md
  ↓
BASELINE.md
  ↓
HARDWARE_PRETEST_STATUS.md
  ↓
HARDWARE_OPERATION_PROCEDURE.md
  ↓
Lily_8leg_Robot_Command_Reference.md
```

---

## 2. Authoritative documents

### A. Project / document entry

| 文書 | 責務 |
|---|---|
| [`../README.md`](../README.md) | project全体の入口。Lily、回転、runtime、repo構成を短く案内する |
| [`README.md`](README.md) | docs全体の地図、読む順番、正本ルール |
| [`../README_V3_CORE.md`](../README_V3_CORE.md) | motion生成・評価の開発入口 |

### B. Current status / baseline

| 文書 | 責務 |
|---|---|
| [`BASELINE.md`](BASELINE.md) | **current candidate / current validation条件の正本** |
| [`HARDWARE_PRETEST_STATUS.md`](HARDWARE_PRETEST_STATUS.md) | **今どこまで実機前確認済みかの正本** |
| [`BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md`](BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md) | 2026-08-12 pre-hardware freezeのimmutable evidence |

`BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md` は履歴証跡であり、後続のcurrent statusを追記する文書ではない。

### C. Operation

| 文書 | 責務 |
|---|---|
| [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md) | **実機試験順序、安全条件、PASS/FAIL、STOP条件の正本** |
| [`Lily_8leg_Robot_Command_Reference.md`](Lily_8leg_Robot_Command_Reference.md) | **exact command lookupの正本** |

操作手順とcommand集を分ける。

- 「次に何をするか」→ `HARDWARE_OPERATION_PROCEDURE.md`
- 「そのcommandは何か」→ `Lily_8leg_Robot_Command_Reference.md`

### D. Architecture / data contract

| 文書 | 責務 |
|---|---|
| [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md) | `/cmdForJetson`、Gazebo/real共通path、timing layer、consumer boundary |
| [`COMMAND_DATA_FORMAT.md`](COMMAND_DATA_FORMAT.md) | JSON / JSONL、24軸record、candidate package、source-vs-transport data contract |
| [`HARDWARE_LIMITS.md`](HARDWARE_LIMITS.md) | hardware joint limit / hard gate |

### E. Design evidence

| 文書 | 責務 |
|---|---|
| [`kinematics_link_length_update_0p075.md`](kinematics_link_length_update_0p075.md) | coxa 0.075 m採用に関するgeometry判断記録 |

設計判断記録はcurrent仕様の入口にはしない。current値は正本文書から参照し、判断過程を調べるときだけ読む。

---

## 3. 情報のownership rule

変化しやすい情報を複数文書にコピーしない。

| 情報 | 正本 |
|---|---|
| current candidate path | `BASELINE.md` |
| candidate SHA / frame count | `BASELINE.md` + candidate manifest |
| Gazebo / hardware verification status | `BASELINE.md`, `HARDWARE_PRETEST_STATUS.md` |
| hardware trial progression | `HARDWARE_OPERATION_PROCEDURE.md` |
| exact shell command | `Lily_8leg_Robot_Command_Reference.md` |
| transport profile / architecture | `RUNTIME_ARCHITECTURE.md` |
| JSONL field semantics | `COMMAND_DATA_FORMAT.md` |
| joint hard limit | `HARDWARE_LIMITS.md` |
| motion core responsibilities | `../README_V3_CORE.md` |

他文書でこれらに触れる場合は、**要約＋正本へのlink**を基本とする。

---

## 4. Current / historicalの境界

### Current authoritative

現行operationでは、原則として次の文書だけで判断できる状態を維持する。

```text
../README.md
docs/README.md
BASELINE.md
HARDWARE_PRETEST_STATUS.md
HARDWARE_OPERATION_PROCEDURE.md
Lily_8leg_Robot_Command_Reference.md
RUNTIME_ARCHITECTURE.md
COMMAND_DATA_FORMAT.md
HARDWARE_LIMITS.md
../README_V3_CORE.md
```

### Historical development notes

`docs/v3_0_*` は開発過程の検討履歴である。

そこに記載された以下は、current仕様を上書きしない。

- old current/provisional baseline
- old candidate path
- old geometry
- old link length
- old smoothing / resampling setting
- old Gazebo replay rate
- old joint-limit assumption
- old script name

履歴noteを読む場合は、そのnoteのversion時点の前提として解釈する。

### Archived operational documents

`archive/docs_legacy/` はstaleなoperation文書・experiment logの保管場所である。

`archive/` は**現行実機操作のentry pointではない**。

---

## 5. 文書更新ルール

### Current candidateが変わったとき

最低限更新する:

```text
BASELINE.md
HARDWARE_PRETEST_STATUS.md
candidate README / manifest / summary
```

必要に応じてoperation procedure / command referenceも更新する。

### Runtime interfaceが変わったとき

最低限更新する:

```text
RUNTIME_ARCHITECTURE.md
COMMAND_DATA_FORMAT.md   # data contract変更がある場合
HARDWARE_OPERATION_PROCEDURE.md
Lily_8leg_Robot_Command_Reference.md
```

### Hardware limitが変わったとき

正本:

```text
HARDWARE_LIMITS.md
```

他文書へ数値をコピーするのではなく、原則linkで追従する。

### Historical evidence

freeze済みbaseline evidenceは後からcurrent情報へ書き換えない。

新しいfreezeが必要なら新規recordを作成する。

---

## 6. 文書を増やす判断基準

新しいmarkdownを作る前に、既存正本へ入る内容か確認する。

新規文書が妥当なのは主に:

- immutable baseline / experiment evidenceを固定する
- 独立した設計判断を残す
- 既存文書の責務と明確に異なる仕様を定義する

場合である。

単なるstatus更新やcommand追加のために新しい文書を増やさない。

---

## 7. 迷ったとき

```text
全体像       → ../README.md
current状態  → BASELINE.md
実機手順     → HARDWARE_OPERATION_PROCEDURE.md
command      → Lily_8leg_Robot_Command_Reference.md
runtime      → RUNTIME_ARCHITECTURE.md
data          → COMMAND_DATA_FORMAT.md
motion開発   → ../README_V3_CORE.md
履歴         → v3_0_* / archive/
```
