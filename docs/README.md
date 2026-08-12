# Lily Documentation Map

更新日: 2026-08-12

この文書は `docs/` の**索引と文書責務の正本**である。

まず「何をしたいか」から文書を選ぶ。

---

## 1. 目的別の入口

| やりたいこと | 読む文書 |
|---|---|
| プロジェクト全体を知る | [`../README.md`](../README.md) |
| コマンドをそのままコピーして実行する | [`COPY_PASTE_COMMANDS.md`](COPY_PASTE_COMMANDS.md) |
| 現在のcandidate / baselineを確認する | [`CURRENT_BASELINE.md`](CURRENT_BASELINE.md) |
| 現在どこまで検証済みか確認する | [`VALIDATION_STATUS.md`](VALIDATION_STATUS.md) |
| motion生成・評価programを使う | [`MOTION_DEVELOPMENT_GUIDE.md`](MOTION_DEVELOPMENT_GUIDE.md) |
| JSONLを新しく作る | [`JSONL_CREATION_GUIDE.md`](JSONL_CREATION_GUIDE.md) |
| JSONL field / candidate package仕様を確認する | [`COMMAND_DATA_FORMAT.md`](COMMAND_DATA_FORMAT.md) |
| Gazebo / 実機のruntimeを理解する | [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md) |
| 実機試験を行う | [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md) |
| commandの意味や選び方を確認する | [`COMMAND_REFERENCE.md`](COMMAND_REFERENCE.md) |
| hardware joint limitを確認する | [`HARDWARE_LIMITS.md`](HARDWARE_LIMITS.md) |

---

## 2. 推奨読書順

### 2.1 Motion developer

```text
../README.md
  ↓
MOTION_DEVELOPMENT_GUIDE.md
  ↓
JSONL_CREATION_GUIDE.md
  ↓
COMMAND_DATA_FORMAT.md
  ↓
RUNTIME_ARCHITECTURE.md
```

### 2.2 Current candidateを理解したい

```text
../README.md
  ↓
CURRENT_BASELINE.md
  ↓
../data/reference_candidates/v3_0_44_candidate_022_wide_urdf0p075/README.md
  ↓
manifest.json / summary.json
  ↓
reports/
```

### 2.3 実機operator

```text
../README.md
  ↓
CURRENT_BASELINE.md
  ↓
VALIDATION_STATUS.md
  ↓
HARDWARE_OPERATION_PROCEDURE.md
  ↓
COPY_PASTE_COMMANDS.md
```

commandの意味や別の操作を調べる場合は `COMMAND_REFERENCE.md` を使用する。

### 2.4 コマンドだけ使いたい

```text
COPY_PASTE_COMMANDS.md
```

Gazeboと実機CANのコマンドを同じページにまとめている。

---

## 3. Current authoritative documents

### Project / motion development

1. [`../README.md`](../README.md) — project全体入口
2. [`MOTION_DEVELOPMENT_GUIDE.md`](MOTION_DEVELOPMENT_GUIDE.md) — motion programをどう使うか
3. [`JSONL_CREATION_GUIDE.md`](JSONL_CREATION_GUIDE.md) — JSONL作成の具体例
4. [`COMMAND_DATA_FORMAT.md`](COMMAND_DATA_FORMAT.md) — JSON / JSONL / candidate data contract

### Current state / execution

5. [`CURRENT_BASELINE.md`](CURRENT_BASELINE.md) — current candidate / baseline正本
6. [`VALIDATION_STATUS.md`](VALIDATION_STATUS.md) — current verification status
7. [`RUNTIME_ARCHITECTURE.md`](RUNTIME_ARCHITECTURE.md) — runtime boundary / timing architecture
8. [`HARDWARE_OPERATION_PROCEDURE.md`](HARDWARE_OPERATION_PROCEDURE.md) — 実機操作正本
9. [`COPY_PASTE_COMMANDS.md`](COPY_PASTE_COMMANDS.md) — Gazebo / CANのcopy-paste用command
10. [`COMMAND_REFERENCE.md`](COMMAND_REFERENCE.md) — commandの意味・選択・詳細索引
11. [`HARDWARE_LIMITS.md`](HARDWARE_LIMITS.md) — joint hard gate

### Decision / evidence

- [`BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md`](BASELINE_PRE_HARDWARE_GAZEBO_PASS_20260812.md) — immutable pre-hardware evidence
- [`kinematics_link_length_update_0p075.md`](kinematics_link_length_update_0p075.md) — geometry変更判断記録

---

## 4. 文書の役割分担

```text
README.md
  = project全体入口

MOTION_DEVELOPMENT_GUIDE.md
  = motion生成・評価・export toolの標準作業フロー

JSONL_CREATION_GUIDE.md
  = JSONLを実際に作る具体例 / current frozen JSONLの来歴

COMMAND_DATA_FORMAT.md
  = JSON / JSONL / candidate packageのデータ仕様

CURRENT_BASELINE.md
  = current candidate / SHA / transport / frozen stageの正本

VALIDATION_STATUS.md
  = 現在どこまで確認済みか

RUNTIME_ARCHITECTURE.md
  = source JSONLからreal/Gazeboまでのprogram boundary

HARDWARE_OPERATION_PROCEDURE.md
  = 実機をどう動かすか

COPY_PASTE_COMMANDS.md
  = Gazebo / CANのコマンドをそのままコピーする場所

COMMAND_REFERENCE.md
  = commandの意味・用途・詳細を調べる場所
```

同じ情報を複数文書で正本化しない。

---

## 5. Motion tool documentation policy

motion系programは、単なるfile一覧ではなく:

```text
目的
→ standard tool
→ input / parameter
→ output
→ validation
→ next step
```

で説明する。

標準toolの使い方は [`MOTION_DEVELOPMENT_GUIDE.md`](MOTION_DEVELOPMENT_GUIDE.md) に集約する。

JSONL生成のcommand例は [`JSONL_CREATION_GUIDE.md`](JSONL_CREATION_GUIDE.md) に集約する。

実行だけが目的の場合は [`COPY_PASTE_COMMANDS.md`](COPY_PASTE_COMMANDS.md) を使用する。

個々のhistorical `v3_0_*` noteをcurrent user guideとして使わない。

---

## 6. Historical development notes

`v3_0_*` のnoteは、そのversion時点の検討履歴である。

それらに書かれた:

- current baseline
- old link length
- old Gazebo replay rate
- old smoothing/resampling setting
- old candidate path
- old parameter recommendation

は、現行operationを上書きしない。

履歴noteは再現性・設計判断追跡のため残す。

---

## 7. Archive boundary

```text
archive/
```

は旧runner、旧experiment、stale operational docsの保管場所。

archive内のprogramや文書をcurrent standard entry pointとして使用しない。

current repeated-roll candidateのhistorical再現が必要な場合のみ、source commit / archive / historical noteを明示的に追跡する。

---

## 8. 旧文書名について

2026-08-12に、分かりにくかったcurrent文書名を整理した。

| 旧path | current正本 |
|---|---|
| `../README_V3_CORE.md` | `MOTION_DEVELOPMENT_GUIDE.md` |
| `BASELINE.md` | `CURRENT_BASELINE.md` |
| `HARDWARE_PRETEST_STATUS.md` | `VALIDATION_STATUS.md` |
| `Lily_8leg_Robot_Command_Reference.md` | `COMMAND_REFERENCE.md` |

旧pathはhistorical link互換用stubとして残すが、current文書として新規linkしない。

---

## 9. Change-control rule

current authoritative documentの役割を変更する場合:

1. `docs/README.md` の責務表を先に確認する。
2. 同じ事実を別文書へ重複コピーしない。
3. immutable evidence documentは書き換えない。
4. 旧pathを消す場合はhistorical linkへの影響を確認する。
