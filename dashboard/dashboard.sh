#!/usr/bin/env bash
# dashboard.sh — start the ESP32-C6 WiFi TX control dashboard.
# Pure stdlib; runs on the system python3, no venv/pip needed. Serves on the LAN
# by default so you can also open it from a phone/laptop on the same WiFi.
#
#   dashboard/dashboard.sh            # 0.0.0.0:8760
#   dashboard/dashboard.sh --host 127.0.0.1 --port 9000
#
# Do NOT run tools/esp32c6_ctl.py at the same time — one process per serial port.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$HERE/serve.py" "$@"
