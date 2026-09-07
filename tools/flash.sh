#!/usr/bin/env bash
# flash.sh — build once, flash one or many ESP32-C6 boards with the same image.
# Activates the shared ESP-IDF (~/esp/esp-idf) itself via tools/idf_env.sh, so no
# export.sh dance is needed first. Run from anywhere; paths resolve from this file.
#
#   tools/flash.sh                       # flash every detected /dev/cu.usbmodem*
#   tools/flash.sh /dev/cu.usbmodem101   # flash one specific port
#   tools/flash.sh --monitor /dev/cu.usbmodem101   # flash then open monitor
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FW="$HERE/../firmware"

# shellcheck source=idf_env.sh
. "$HERE/idf_env.sh"

MONITOR=0
PORTS=()
for a in "$@"; do
  case "$a" in
    --monitor) MONITOR=1 ;;
    *) PORTS+=("$a") ;;
  esac
done

if [ "${#PORTS[@]}" -eq 0 ]; then
  # autodetect
  for p in /dev/cu.usbmodem* /dev/cu.usbserial* /dev/cu.wchusbserial*; do
    [ -e "$p" ] || continue
    case "$p" in *Bluetooth*|*debug-console*) continue ;; esac
    PORTS+=("$p")
  done
fi

if [ "${#PORTS[@]}" -eq 0 ]; then
  echo "No ESP32-C6 serial ports detected. Plug a board in." >&2
  exit 1
fi

echo "Building once ..."
( cd "$FW" && idf.py set-target esp32c6 >/dev/null && idf.py build )

for p in "${PORTS[@]}"; do
  echo "== Flashing $p =="
  ( cd "$FW" && idf.py -p "$p" flash )
done

if [ "$MONITOR" -eq 1 ]; then
  ( cd "$FW" && idf.py -p "${PORTS[0]}" monitor )
fi
