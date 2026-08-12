# v3_0_44_candidate_022_wide_urdf0p075

このdirectoryは、Lily回転移動の**review済みreference candidate package**である。

このREADMEはcandidate固有の構成だけを説明する。current project status、transport条件、実機試験手順は上位文書を正本とする。

- current baseline: [`../../../docs/BASELINE.md`](../../../docs/BASELINE.md)
- pretest status: [`../../../docs/HARDWARE_PRETEST_STATUS.md`](../../../docs/HARDWARE_PRETEST_STATUS.md)
- hardware procedure: [`../../../docs/HARDWARE_OPERATION_PROCEDURE.md`](../../../docs/HARDWARE_OPERATION_PROCEDURE.md)
- command reference: [`../../../docs/Lily_8leg_Robot_Command_Reference.md`](../../../docs/Lily_8leg_Robot_Command_Reference.md)
- data contract: [`../../../docs/COMMAND_DATA_FORMAT.md`](../../../docs/COMMAND_DATA_FORMAT.md)

---

## 1. Candidate identity

`candidate_022_wide` をURDF coxa 0.075 geometryに基づくpre-hardware reference candidateとしてfreezeしたpackageである。

旧candidate directoryは上書きしない。

candidate採用時のgeometry / evaluation判断は `manifest.json`、`pre_hardware_decision.md`、`reports/` に残す。

hardware full-rollのcurrent statusはこのREADMEに重複記載せず、[`../../../docs/BASELINE.md`](../../../docs/BASELINE.md) を参照する。

---

## 2. Package structure

```text
v3_0_44_candidate_022_wide_urdf0p075/
├── README.md
├── commands.jsonl
├── manifest.json
├── summary.json
├── pre_hardware_decision.md
├── reports/
└── staged/
    ├── air_entry_and_hold_only_commands.jsonl
    ├── combined_with_hold_commands.jsonl
    ├── roll_0_50_commands.jsonl
    ├── roll_50_100_commands.jsonl
    ├── roll_100_300_commands.jsonl
    ├── roll_300_end_commands.jsonl
    ├── roll_to_1of4_commands.jsonl
    ├── roll_to_2of4_commands.jsonl
    ├── roll_to_3of4_commands.jsonl
    ├── roll_to_4of4_commands.jsonl
    └── quarter_stage_manifest.json
```

---

## 3. Source trajectory

```text
commands.jsonl
```

がcandidateのfrozen source trajectoryである。

実行用derived dataを生成しても、このsourceをsilent変更しない。

identity / provenance / checksumは `manifest.json` および上位baseline文書で管理する。

---

## 4. Existing risk-oriented staged data

```text
air_entry_and_hold_only_commands.jsonl
roll_0_50_commands.jsonl
roll_50_100_commands.jsonl
roll_100_300_commands.jsonl
roll_300_end_commands.jsonl
combined_with_hold_commands.jsonl
```

`roll_0_50` 等は**semantic quarterではない**。

初回実機validationを細かく止めながら進めるためのsequential/risk-oriented splitであり、前stageの終了姿勢から次stageへ続ける用途を持つ。

正確な実機試験順序は [`../../../docs/HARDWARE_OPERATION_PROCEDURE.md`](../../../docs/HARDWARE_OPERATION_PROCEDURE.md) を参照する。

---

## 5. Semantic cumulative quarter data

`commands.jsonl` のcontiguous `roll_index` blockから生成してfreezeしたderived data:

```text
roll_to_1of4_commands.jsonl
roll_to_2of4_commands.jsonl
roll_to_3of4_commands.jsonl
roll_to_4of4_commands.jsonl
quarter_stage_manifest.json
```

意味:

```text
roll_to_1of4 = rolling-start → semantic roll 1終了
roll_to_2of4 = rolling-start → semantic roll 2終了
roll_to_3of4 = rolling-start → semantic roll 3終了
roll_to_4of4 = rolling-start → semantic roll 4終了
```

これらは**累積stage**である。

したがって、`roll_to_1of4` の直後に `roll_to_2of4` を継続実行する用途ではない。各stageは同じrolling-start postureから目的地点までを独立に確認する。

生成規則・source/output checksumは `quarter_stage_manifest.json` を正本とする。

---

## 6. Reports / evidence

```text
reports/
```

にはcandidate採用に用いたgeometry、clearance、constraint、Gazebo等の評価証跡を保持する。

このcandidateではURDF FK evaluatorをgeometry判断のsource of truthとして採用した。

古いfallback FKや旧axis conventionに基づくreportをcurrent geometry判定へ優先しない。

---

## 7. Do not silently change

このpackageをreference candidateとして扱う間は、次をsilent変更しない。

- `commands.jsonl`
- existing staged JSONL
- semantic quarter JSONL
- `quarter_stage_manifest.json`
- candidate identity / provenance fields

変更が必要なら、新candidate / new baselineとして追跡可能な形で扱う。
