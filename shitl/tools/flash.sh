#!/usr/bin/env bash
# flash.sh — build ESP32-SHITL-Tx and flash it over USB (esp32c6 or esp32s3).
# Activates the shared ESP-IDF (~/esp/esp-idf) itself via the repo's
# tools/idf_env.sh, which this app shares with ESP32-Tx.
# Run from anywhere; paths resolve from this file.
#
#   shitl/tools/flash.sh /dev/cu.usbmodemXXXX          # build + flash one board
#   shitl/tools/flash.sh --build                       # build only (for an OTA push)
#   shitl/tools/flash.sh --target esp32s3 PORT         # skip chip detection
#   shitl/tools/flash.sh --monitor PORT                # flash, then open the monitor
#
# A port is required: every bench ESP32 looks alike on USB, and flashing the
# wrong one takes it away from another app. USB flashing is only needed once per
# board (it writes the bootloader and the two-slot partition table); after that
# use `shitl/tools/shitltx_ctl.py --host <ip> ota` or the dashboard's Update button.
#
# esp32c6 builds in firmware/build; any other target in firmware/build-<target>.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FW="$HERE/../firmware"

# shellcheck source=../../tools/idf_env.sh
. "$HERE/../../tools/idf_env.sh"

MONITOR=0
BUILD_ONLY=0
TARGET=""
PORT=""
while [ $# -gt 0 ]; do
  case "$1" in
    --monitor) MONITOR=1 ;;
    --build) BUILD_ONLY=1 ;;
    --target) TARGET="$2"; shift ;;
    --target=*) TARGET="${1#--target=}" ;;
    *) PORT="$1" ;;
  esac
  shift
done

idf_args() {
  if [ "$1" = esp32c6 ]; then
    echo "-D IDF_TARGET=esp32c6"
  else
    echo "-B build-$1 -D SDKCONFIG=sdkconfig.$1 -D IDF_TARGET=$1"
  fi
}

if [ "$BUILD_ONLY" -eq 1 ]; then
  TARGET="${TARGET:-esp32c6}"
elif [ -z "$PORT" ]; then
  echo "usage: shitl/tools/flash.sh [--target esp32c6|esp32s3] [--monitor] PORT   |   shitl/tools/flash.sh --build" >&2
  echo "serial ports present:" >&2
  ls /dev/cu.usbmodem* /dev/cu.usbserial* /dev/cu.wchusbserial* 2>/dev/null >&2 || echo "  (none)" >&2
  exit 1
elif [ -z "$TARGET" ]; then
  TARGET="$(esptool.py -p "$PORT" chip_id 2>/dev/null \
    | grep -oE 'ESP32-[A-Z][0-9]+' | head -1 | tr 'A-Z' 'a-z' | tr -d '-' || true)"
fi

case "$TARGET" in
  esp32c6|esp32s3) ;;
  *) echo "$PORT: unsupported or undetected chip '${TARGET:-?}'." >&2
     echo "Is another program holding the port (lsof $PORT)? Otherwise pass --target." >&2
     exit 1 ;;
esac

# sdkconfig is generated from sdkconfig.defaults* once and then wins over it, so
# an edit to the defaults would be silently ignored. Drop it when they are newer.
sc="$FW/sdkconfig"; [ "$TARGET" = esp32c6 ] || sc="$FW/sdkconfig.$TARGET"
for d in "$FW/sdkconfig.defaults" "$FW/sdkconfig.defaults.$TARGET" "$FW/partitions.csv"; do
  if [ -f "$sc" ] && [ -f "$d" ] && [ "$d" -nt "$sc" ]; then
    echo "$(basename "$d") changed - regenerating $(basename "$sc")"
    rm -f "$sc"
  fi
done

echo "Building for $TARGET ..."
# shellcheck disable=SC2046
( cd "$FW" && idf.py $(idf_args "$TARGET") build )
[ "$BUILD_ONLY" -eq 1 ] && exit 0

echo "== Flashing $PORT ($TARGET) =="
# shellcheck disable=SC2046
( cd "$FW" && idf.py $(idf_args "$TARGET") -p "$PORT" flash )

if [ "$MONITOR" -eq 1 ]; then
  # shellcheck disable=SC2046
  ( cd "$FW" && idf.py $(idf_args "$TARGET") -p "$PORT" monitor )
fi
