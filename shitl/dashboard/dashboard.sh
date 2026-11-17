#!/usr/bin/env bash
# dashboard.sh — run the ESP32-SHITL-Tx control dashboard on demand.
# Pure stdlib; runs on the system python3, no venv/pip needed. Nothing is
# installed and nothing starts at login: the dashboard is up only while you ask
# for it.
#
#   dashboard/dashboard.sh start      # background, http://127.0.0.1:8761/
#   dashboard/dashboard.sh stop
#   dashboard/dashboard.sh status
#   dashboard/dashboard.sh            # foreground (Ctrl-C to stop)
#
# Anything after the command is passed to serve.py:
#   dashboard/dashboard.sh start --port 9000
#   dashboard/dashboard.sh --host 0.0.0.0     # reachable from the LAN (no auth!)
#
# It talks to the boards over the network only and never holds a serial port.
# It also listens on UDP 8762 for the boards' once-a-second announcements.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN="${TMPDIR:-/tmp}"
PIDFILE="${RUN%/}/esp32shitltx-dashboard.pid"
LOG="${RUN%/}/esp32shitltx-dashboard.log"

running() { [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; }

case "${1:-}" in
  start)
    shift
    if running; then echo "already running (pid $(cat "$PIDFILE"))"; exit 0; fi
    nohup /usr/bin/python3 -u "$HERE/serve.py" "$@" >"$LOG" 2>&1 &
    echo $! >"$PIDFILE"
    sleep 1
    if ! running; then
      echo "dashboard failed to start:" >&2; cat "$LOG" >&2; rm -f "$PIDFILE"; exit 1
    fi
    grep -m1 -o 'http://[^ ]*' "$LOG" || true
    echo "running (pid $(cat "$PIDFILE")), log: $LOG"
    ;;
  stop)
    if running; then
      kill "$(cat "$PIDFILE")"; echo "stopped"
    else
      echo "not running"
    fi
    rm -f "$PIDFILE"
    ;;
  status)
    if running; then
      echo "running (pid $(cat "$PIDFILE")), log: $LOG"
    else
      echo "not running"; exit 1
    fi
    ;;
  *)
    exec /usr/bin/python3 "$HERE/serve.py" "$@"
    ;;
esac
