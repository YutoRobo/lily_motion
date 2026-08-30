#!/usr/bin/env bash
set -u

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$THIS_DIR/../.." && pwd)"
LAUNCHER="$THIS_DIR/launch_lily_operator.sh"
APP_DIR="$HOME/.local/share/applications"
APP_FILE="$APP_DIR/lily-operator.desktop"

if command -v xdg-user-dir >/dev/null 2>&1; then
  DESKTOP_DIR="$(xdg-user-dir DESKTOP)"
else
  DESKTOP_DIR="$HOME/Desktop"
fi
if [ -z "$DESKTOP_DIR" ]; then
  DESKTOP_DIR="$HOME/Desktop"
fi
DESKTOP_FILE="$DESKTOP_DIR/Lily Operator.desktop"

mkdir -p "$APP_DIR"
mkdir -p "$DESKTOP_DIR"

cat > "$APP_FILE" <<EOF
[Desktop Entry]
Type=Application
Name=Lily Operator
Comment=Launch Lily Operator UI with CAN and ROS startup checks
Exec=bash "$LAUNCHER"
Path=$ROOT
Terminal=false
Icon=applications-engineering
Categories=Development;Engineering;
StartupNotify=true
EOF

cp "$APP_FILE" "$DESKTOP_FILE"
chmod +x "$DESKTOP_FILE"

if command -v gio >/dev/null 2>&1; then
  gio set "$DESKTOP_FILE" metadata::trusted true >/dev/null 2>&1 || true
fi

if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database "$APP_DIR" >/dev/null 2>&1 || true
fi

printf 'Installed Lily Operator launcher:\n'
printf '  Desktop: %s\n' "$DESKTOP_FILE"
printf '  App menu: %s\n' "$APP_FILE"
printf '\nDouble-click "Lily Operator" on the Desktop.\n'
printf 'If Ubuntu asks whether to trust/launch the file, choose "Trust and Launch".\n'
