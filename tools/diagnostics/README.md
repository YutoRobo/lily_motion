# Lily Diagnostics Tools

このdirectoryは、Lilyのmotion / command / 実機telemetryを確認する診断ツールを置く。

## MCU position debug realtime viewer

`realtime_position_debug_viewer_ui.py` は、MCUが送信する位置debug telemetryを受信専用で可視化するTkinter GUIである。

Telemetry format:

```text
CAN ID   : 0x500 | axis
DLC      : 8
byte 0-3 : internal position command [joint rad], float32 little-endian
byte 4-7 : actual position          [joint rad], float32 little-endian
```

このviewer自身はCAN frameを送信しない。内部で `candump -L` を起動して受信する。

### 1脚を確認する

例: leg-index 3 = axes 9,10,11

```bash
python2 tools/diagnostics/realtime_position_debug_viewer_ui.py \
  --interface can0 \
  --leg-index 3
```

GUI上で `Duration [s]` を入力して `START` を押す。
最初に受信した対象axis telemetry frameを `t=0` とし、指定時間後に自動停止する。

### axisを明示する

```bash
python2 tools/diagnostics/realtime_position_debug_viewer_ui.py \
  --interface can0 \
  --axes 9,10,11 \
  --duration-sec 5
```

### CSVを保存しない

```bash
python2 tools/diagnostics/realtime_position_debug_viewer_ui.py \
  --interface can0 \
  --leg-index 3 \
  --no-csv
```

CSVを有効にした場合、command / actual / tracking errorをradとdegで保存する。

主な引数は `--help` で確認する。
