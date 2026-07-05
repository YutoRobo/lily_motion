# Hardware Joint Limits

## hardware_limit_v2

実機構造上の関節可動域は以下とする。

| Joint   | Meaning     | Hard limit | Notes                                                         |
| ------- | ----------- | ---------: | ------------------------------------------------------------- |
| Joint 1 | base_clause |   ±360 deg | 旧評価で用いていた ±180 deg は撤回する                                      |
| Joint 2 | thigh       |    ±95 deg | 構造上は限界を攻めると ±100 deg 程度まで可能だが、通常評価では ±95 deg を hard limit とする |
| Joint 3 | tibia       |   ±150 deg | 旧評価の ±135 deg から更新する                                          |

## Hard gate

hardware_limit_v2 における関節角 hard gate は以下とする。

* `abs(base_clause) <= 360 deg`
* `abs(thigh) <= 95 deg`
* `abs(tibia) <= 150 deg`

## Monitored margin

以下は hard gate ではなく監視指標とする。

* `abs(base_clause) > 330 deg`
* `abs(base_clause) > 340 deg`
* thigh の ±95 deg への接近度
* tibia の ±150 deg への接近度

## Historical note

過去の一部評価では base_clause の制約を ±180 deg と仮定していた。
しかし、元repoの `roll(Direction.FORWARD)` 実行確認により、元プログラムも base servo range ±380 deg を使用し、内部IK角・servo target角・publish角が ±180 deg を超え得ることが確認された。

したがって、base_clause ±180 deg violation は historical diagnostic として残すが、現行の hardware_limit_v2 hard gate には含めない。

