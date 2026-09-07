#!/usr/bin/env bash
# deploy.sh — push this dashboard to the always-on background service.
#
# WHY A COPY EXISTS. The dashboard runs as a macOS LaunchAgent
# (com.beesdr.esp32c6-dashboard) so it is up whenever you log in. launchd-spawned
# processes get no TCC grant for ~/Documents, so a plist pointing straight at this
# folder dies on every start with:
#
#   can't open file '.../dashboard/serve.py': [Errno 1] Operation not permitted
#
# ~/Library/Application Support is not TCC-protected, so the service runs from a
# copy there. This directory stays the source of truth; run this after editing.
#
#   dashboard/deploy.sh            # sync + restart the service
#   dashboard/deploy.sh --status   # just report what is running
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME="$HOME/Library/Application Support/BEESDR/ESP32C6-dashboard"
LABEL="com.beesdr.esp32c6-dashboard"
PLIST="$HOME/Library/LaunchAgents/$LABEL.plist"
PORT=8760

status() {
  if launchctl print "gui/$(id -u)/$LABEL" >/dev/null 2>&1; then
    echo "service : loaded"
  else
    echo "service : NOT loaded"
  fi
  if curl -fsS -o /dev/null "http://127.0.0.1:$PORT/"; then
    echo "http    : responding on http://127.0.0.1:$PORT/"
  else
    echo "http    : no response on $PORT — check ~/Library/Logs/beesdr-esp32c6-dashboard.error.log"
  fi
}

if [ "${1:-}" = "--status" ]; then status; exit 0; fi

if [ ! -f "$PLIST" ]; then
  echo "LaunchAgent not installed: $PLIST" >&2
  echo "Run the server in the foreground instead:  dashboard/dashboard.sh" >&2
  exit 1
fi

mkdir -p "$RUNTIME"
cp "$HERE/serve.py" "$HERE/index.html" "$HERE/dashboard.sh" "$RUNTIME"/
chmod +x "$RUNTIME/dashboard.sh"
echo "synced -> $RUNTIME"

launchctl bootout "gui/$(id -u)/$LABEL" 2>/dev/null || true
sleep 1
launchctl bootstrap "gui/$(id -u)" "$PLIST"
sleep 2
status
