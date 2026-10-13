#!/bin/sh
# Copy a local folder under sd_card/deskradar/ onto the v6 device's microSD over WiFi.
# usage: tools/sd_upload.sh sd_card/deskradar/logos [host]
# The device only accepts paths under /deskradar/ (the card also holds the owner's phone files).
set -e
DIR="${1:?usage: tools/sd_upload.sh sd_card/deskradar/<folder> [host]}"
HOST="${2:-deskradar6.local}"
case "$DIR" in sd_card/deskradar/*) ;; *) echo "folder must be under sd_card/deskradar/"; exit 1 ;; esac
REMOTE="/${DIR#sd_card/}"
ok=0; bad=0
for f in "$DIR"/*; do
  [ -f "$f" ] || continue
  if curl -4 -s -m 60 -F "f=@$f" "http://$HOST/sdput?path=$REMOTE/$(basename "$f")" | grep -q "^ok"; then ok=$((ok+1)); else bad=$((bad+1)); echo "failed: $f"; fi
done
echo "$ok uploaded, $bad failed -> $REMOTE"
