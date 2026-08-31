#!/bin/bash
# Installs the Arctos CAN Control Panel into the application menu.
# Safe to re-run any time (e.g. after moving arctos_ws) to refresh the entry.
set -e

GUI_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APPS_DIR="$HOME/.local/share/applications"

mkdir -p "$APPS_DIR"
chmod +x "$GUI_DIR/arctos_can_control_panel.py"

# Regenerate the .desktop file with the correct absolute path in case this
# folder was moved, rather than trusting a possibly-stale checked-in copy.
cat > "$APPS_DIR/arctos-can-control-panel.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=Arctos CAN Control Panel
Comment=Set up or safely shut down the CANable adapter for the Arctos arm
Exec=/usr/bin/python3 $GUI_DIR/arctos_can_control_panel.py
Icon=network-transmit-receive
Terminal=false
Categories=Utility;
StartupNotify=true
EOF

chmod +x "$APPS_DIR/arctos-can-control-panel.desktop"

if command -v update-desktop-database >/dev/null 2>&1; then
    update-desktop-database "$APPS_DIR" 2>/dev/null || true
fi

echo "Installed. Search for 'Arctos CAN Control Panel' in the application menu/Activities."
echo "(You can also run it directly any time: python3 $GUI_DIR/arctos_can_control_panel.py)"
