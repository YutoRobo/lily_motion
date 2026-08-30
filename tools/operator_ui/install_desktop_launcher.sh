#!/usr/bin/env bash
set -u

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$THIS_DIR/../.." && pwd)"
LAUNCHER="$THIS_DIR/launch_lily_operator.sh"
GAZEBO_LAUNCHER="$THIS_DIR/launch_lily_operator_gazebo.sh"
HARDWARE_GAZEBO_LAUNCHER="$THIS_DIR/launch_lily_operator_hardware_gazebo.sh"
APP_DIR="$HOME/.local/share/applications"
APP_FILE="$APP_DIR/lily-operator.desktop"
VCAN_APP_FILE="$APP_DIR/lily-operator-vcan0.desktop"
GAZEBO_APP_FILE="$APP_DIR/lily-operator-gazebo.desktop"
HARDWARE_GAZEBO_APP_FILE="$APP_DIR/lily-operator-hardware-gazebo.desktop"

if command -v xdg-user-dir >/dev/null 2>&1; then
  DESKTOP_DIR="$(xdg-user-dir DESKTOP)"
else
  DESKTOP_DIR="$HOME/Desktop"
fi
if [ -z "$DESKTOP_DIR" ]; then
  DESKTOP_DIR="$HOME/Desktop"
fi
DESKTOP_FILE="$DESKTOP_DIR/Lily Operator.desktop"
VCAN_DESKTOP_FILE="$DESKTOP_DIR/Lily Operator (vcan0).desktop"
GAZEBO_DESKTOP_FILE="$DESKTOP_DIR/Lily Operator (Gazebo).desktop"
HARDWARE_GAZEBO_DESKTOP_FILE="$DESKTOP_DIR/Lily Operator (Hardware + Gazebo).desktop"

mkdir -p "$APP_DIR"
mkdir -p "$DESKTOP_DIR"

# Remove desktop/application entries from the abandoned CAN->Gazebo bridge
# experiment so reinstalling leaves only the supported launch modes.
rm -f \
  "$APP_DIR/lily-gazebo-sync-bridge.desktop" \
  "$APP_DIR/lily-gazebo-sync-bridge-vcan0.desktop" \
  "$DESKTOP_DIR/Lily Gazebo Sync Bridge.desktop" \
  "$DESKTOP_DIR/Lily Gazebo Sync Bridge (vcan0).desktop"

cat > "$APP_FILE" <<EOF
[Desktop Entry]
Type=Application
Name=Lily Operator
Comment=Launch Lily Operator UI on physical CAN with ROS startup checks
Exec=bash "$LAUNCHER"
Path=$ROOT
Terminal=false
Icon=applications-engineering
Categories=Development;Engineering;
StartupNotify=true
EOF

cat > "$VCAN_APP_FILE" <<EOF
[Desktop Entry]
Type=Application
Name=Lily Operator (vcan0)
Comment=Launch Lily Operator UI on virtual CAN for software testing
Exec=env LILY_CAN_CHANNEL=vcan0 bash "$LAUNCHER"
Path=$ROOT
Terminal=false
Icon=applications-development
Categories=Development;Engineering;
StartupNotify=true
EOF

cat > "$GAZEBO_APP_FILE" <<EOF
[Desktop Entry]
Type=Application
Name=Lily Operator (Gazebo)
Comment=Launch Gazebo-only Lily Motion UI without CAN StateMachine
Exec=bash "$GAZEBO_LAUNCHER"
Path=$ROOT
Terminal=false
Icon=applications-graphics
Categories=Development;Engineering;
StartupNotify=true
EOF

cat > "$HARDWARE_GAZEBO_APP_FILE" <<EOF
[Desktop Entry]
Type=Application
Name=Lily Operator (Hardware + Gazebo)
Comment=Prepare Gazebo world and controllers, then launch the unchanged physical-CAN Lily Operator
Exec=bash "$HARDWARE_GAZEBO_LAUNCHER"
Path=$ROOT
Terminal=true
Icon=applications-engineering
Categories=Development;Engineering;
StartupNotify=true
EOF

cp "$APP_FILE" "$DESKTOP_FILE"
cp "$VCAN_APP_FILE" "$VCAN_DESKTOP_FILE"
cp "$GAZEBO_APP_FILE" "$GAZEBO_DESKTOP_FILE"
cp "$HARDWARE_GAZEBO_APP_FILE" "$HARDWARE_GAZEBO_DESKTOP_FILE"
chmod +x "$DESKTOP_FILE" "$VCAN_DESKTOP_FILE" "$GAZEBO_DESKTOP_FILE" "$HARDWARE_GAZEBO_DESKTOP_FILE"

if command -v gio >/dev/null 2>&1; then
  gio set "$DESKTOP_FILE" metadata::trusted true >/dev/null 2>&1 || true
  gio set "$VCAN_DESKTOP_FILE" metadata::trusted true >/dev/null 2>&1 || true
  gio set "$GAZEBO_DESKTOP_FILE" metadata::trusted true >/dev/null 2>&1 || true
  gio set "$HARDWARE_GAZEBO_DESKTOP_FILE" metadata::trusted true >/dev/null 2>&1 || true
fi

if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database "$APP_DIR" >/dev/null 2>&1 || true
fi

printf 'Installed Lily Operator launchers:\n'
printf '  Physical CAN Desktop:        %s\n' "$DESKTOP_FILE"
printf '  Virtual CAN Desktop:         %s\n' "$VCAN_DESKTOP_FILE"
printf '  Gazebo-only Desktop:         %s\n' "$GAZEBO_DESKTOP_FILE"
printf '  Hardware + Gazebo Desktop:   %s\n' "$HARDWARE_GAZEBO_DESKTOP_FILE"
printf '  Physical CAN App menu:       %s\n' "$APP_FILE"
printf '  Virtual CAN App menu:        %s\n' "$VCAN_APP_FILE"
printf '  Gazebo-only App menu:        %s\n' "$GAZEBO_APP_FILE"
printf '  Hardware + Gazebo App menu:  %s\n' "$HARDWARE_GAZEBO_APP_FILE"
printf '\nUse "Lily Operator" for can0 / hardware only.\n'
printf 'Use "Lily Operator (vcan0)" for virtual-CAN + MCU emulator tests.\n'
printf 'Use "Lily Operator (Gazebo)" for Gazebo-only /cmdForJetson tests with CAN StateMachine OFF.\n'
printf 'Use "Lily Operator (Hardware + Gazebo)" to auto-start Gazebo world/control/interpolator, then the unchanged normal Lily Operator.\n'
printf 'If Ubuntu asks whether to trust/launch a desktop file, choose "Trust and Launch".\n'
