#!/bin/sh
# Stop and remove the Desk Radar Mac helper LaunchAgent and binary.
AGENT="$HOME/Library/LaunchAgents/local.deskradar.helper.plist"
launchctl bootout "gui/$(id -u)" "$AGENT" 2>/dev/null || true
rm -f "$AGENT"
rm -rf "$HOME/Library/Application Support/DeskRadar"
echo "removed"
