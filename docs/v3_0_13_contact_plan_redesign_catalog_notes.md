# v3.0.13 Contact Plan Redesign Catalog

## 目的

v3.0.12 までの結果から、連続パラメータを小さく振るだけでは 1 回転成立に届きにくいことが分かった。v3.0.13 では、接触計画そのもの、すなわち **ConstrainedBodyRoll 中にどの脚を支持脚として残すか** を明示的な variant として増やす。

この変更の目的は、特定の variant を正解と決めることではない。接触集合の仮説を隠さず、同じ whole-roll evaluator で比較できるようにすることである。

## 追加した contact_plan_variant

既存:

- default
- next_only_roll
- six_support_roll
- front_pair_roll
- rear_pair_roll

追加:

- upper_front_pair_roll
- upper_rear_pair_roll
- lower_front_pair_roll
- lower_rear_pair_roll
- diagonal_front_roll
- diagonal_rear_roll
- four_corner_roll
- x_cross_roll
- upper_quad_roll
- lower_quad_roll

脚 ID は以下の v3 内部定義に基づく。

```text
0 TRF, 1 TRH, 2 BRF, 3 BRH, 4 TLF, 5 TLH, 6 BLF, 7 BLH
```

## 新規スクリプト

```bash
python run_v3_0_contact_plan_catalog.py \
  --output testdata/v3_0_13_contact_plan_catalog.json
```

このスクリプトは、連続パラメータをほぼ固定し、contact_plan_variant だけを比較する。

## goal-oriented sweep の更新

`run_v3_0_goal_oriented_sweep.py` の quick / broad でも、新しい接触計画を候補に含めるようにした。

quick:

```bash
python run_v3_0_goal_oriented_sweep.py \
  --mode quick \
  --output testdata/v3_0_13_goal_oriented_quick.json
```

catalog:

```bash
python run_v3_0_contact_plan_catalog.py \
  --steps-per-phase 6 \
  --filter-window 3 \
  --body-roll-pitch-deg 90 \
  --output testdata/v3_0_13_contact_plan_catalog.json
```

## 注意

v3.0.13 は成功歩容の完成版ではない。目的は、接触集合の選び方が failure signature にどう効くかを見ることである。

特に見るべき指標は以下。

- generator_ik_failure_count
- filtered_penetration_count
- filtered_near_count
- filtered_max_second_joint_deg
- filtered_max_joint_delta_deg
- filtered_max_contact_drift_m
- contact_drift_hard_violation_count

## ロードマップ上の位置づけ

- v3.0.8: whole-roll / filtered / contact-lock 評価
- v3.0.9: raw 接地点固定生成
- v3.0.11: soft contact drift 評価
- v3.0.12: roadmap 付き goal-oriented sweep
- v3.0.13: contact-plan catalog sweep

次は、catalog 結果で比較的ましな contact plan に絞り、candidate support placement と base-pose trajectory をより広く探索する。
