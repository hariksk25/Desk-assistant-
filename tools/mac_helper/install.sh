#!/bin/sh
# Build and install the Desk Radar Mac helper as a LaunchAgent (starts at login, restarts if it stops).
# usage: tools/mac_helper/install.sh [device-host]      (default deskradar6.local)
# remove: tools/mac_helper/uninstall.sh
# First start: macOS asks to allow Calendar access for "DeskRadarHelper" (click Allow). Later, the first
# volume / mute / now-playing use asks for Automation access (System Events / Music / Spotify).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
HOST="${1:-deskradar6.local}"
DEST="$HOME/Library/Application Support/DeskRadar"
AGENT="$HOME/Library/LaunchAgents/local.deskradar.helper.plist"
mkdir -p "$DEST"
swiftc -O "$HERE/DeskRadarHelper.swift" -o "$DEST/DeskRadarHelper" \
  -Xlinker -sectcreate -Xlinker __TEXT -Xlinker __info_plist -Xlinker "$HERE/Info.plist"
codesign --force --sign - --identifier local.deskradar.helper "$DEST/DeskRadarHelper"
cat > "$AGENT" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>Label</key><string>local.deskradar.helper</string>
  <key>ProgramArguments</key><array><string>$DEST/DeskRadarHelper</string></array>
  <key>EnvironmentVariables</key><dict><key>DESKRADAR_HOST</key><string>$HOST</string></dict>
  <key>RunAtLoad</key><true/>
  <key>KeepAlive</key><true/>
  <key>StandardErrorPath</key><string>$DEST/helper.log</string>
  <key>ProcessType</key><string>Interactive</string>
</dict>
</plist>
EOF
launchctl bootout "gui/$(id -u)" "$AGENT" 2>/dev/null || true
launchctl bootstrap "gui/$(id -u)" "$AGENT"
echo "installed: $DEST/DeskRadarHelper (log: $DEST/helper.log)"
