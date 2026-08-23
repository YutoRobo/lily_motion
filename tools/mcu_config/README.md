# Lily MCU Config Editor

Lilyの各関節MCUに対して、CAN経由でConfigパラメータを確認・変更・保存するためのPython 2用GUIツールです。

## 1. 配置

推奨配置:

```text
lily_motion/
└─ tools/
   └─ mcu_config/
      ├─ lily_mcu_config_editor.py
      └─ README.md
```

## 2. 実行環境

- Jetson / Linux
- Python 2.7
- Tkinter / ttk
- can-utils (`candump`, `cansend`)
- CANインターフェース: `can0`
- CAN bitrate: 500 kbit/s

本ツールはPython 2環境との互換性を優先し、PythonからSocketCANを直接操作せず、`candump` / `cansend` を利用してCAN通信します。

## 3. CAN接続

Jetson起動後、以下のコマンドで `can0` を設定します。

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

接続確認には、必要に応じて以下を使用します。

```bash
candump can0
```

## 4. 起動方法

Axis 11のみを対象にする場合:

```bash
python2 lily_mcu_config_editor.py --interface can0 --axes 11
```

複数軸を対象にする場合:

```bash
python2 lily_mcu_config_editor.py --interface can0 --axes 0-23
```

軸指定例:

```text
--axes 11
--axes 10-11
--axes 0,1,2,11
--axes 0-23
```

## 5. 主な機能

- HardwareConfig / SoftwareConfigのREAD
- 1パラメータ単位のWRITE
- WRITE後のMCU Echo確認
- WRITEした1パラメータのみREAD back
- HardwareConfig / SoftwareConfigの個別SAVE
- 未接続軸の応答なし表示
- HardwareConfig SAVE後の再起動要求表示

通常のパラメータ変更では、対象パラメータのみ通信するため、全パラメータを毎回読み直さずCAN負荷を抑えています。

## 6. MCU Config CAN仕様

各軸のConfig CAN IDは以下です。

```text
Request  = 0x080 | LEG_NUMBER
Response = 0x180 | LEG_NUMBER
```

Axis 11 (`LEG_NUMBER = 0x0B`) の場合:

```text
Request  = 0x08B
Response = 0x18B
```

8 byte payload:

```text
Byte 0 : Command
Byte 1 : Config Type
Byte 2 : Parameter ID
Byte 3 : Result
Byte 4-7 : Value (little endian, 32 bit)
```

Command:

```text
0x01 : READ
0x02 : WRITE
0x03 : SAVE
```

Config Type:

```text
0x01 : HardwareConfig
0x02 : SoftwareConfig
```

## 7. 操作上の注意

- READはMCU仕様上、RUN中も使用可能です。
- WRITE / SAVEは `aliment_standby` 状態で行ってください。
- SoftwareConfigのWRITEはRAMへ即時反映されます。
- SAVEしないSoftwareConfig変更は、電源再投入で保存済み値へ戻ります。
- HardwareConfigはWRITE後にSAVEし、SAVE成功後は必ずMCU電源を再投入してください。
- SAVE中に電源を切らないでください。
- 極端なPID値や機械的に危険な値は入力しないでください。
- 通常運用では自動周期更新を行わず、必要な軸・パラメータのみ更新してください。

## 8. 現在確認済みの内容

Axis 11の単軸実機で以下を確認済みです。

- Config READ
- SoftwareConfig WRITE / Echo / READ back
- SoftwareConfig SAVE / 電源再投入後の復元
- HardwareConfig WRITE / Echo / READ back
- HardwareConfig SAVE / 電源再投入後の復元
- 未接続軸との混在表示
- GUIから変更したKpが実際の制御挙動へ反映されること

24軸同時接続試験は、実機構成が揃った段階で別途実施します。
