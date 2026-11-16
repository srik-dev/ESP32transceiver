#!/usr/bin/env bash
# flash.sh — build and flash one or many ESP32 boards (esp32c6 or esp32s3).
# Activates the shared ESP-IDF (~/esp/esp-idf) itself via tools/idf_env.sh, so no
# export.sh dance is needed first. Run from anywhere; paths resolve from this file.
#
# The chip type is read off each port (esptool chip_id), so a mixed bag of C6 and
# S3 boards can be flashed in one go; each target is built once.
#
#   tools/flash.sh                       # flash every detected serial port
#   tools/flash.sh /dev/cu.usbmodem101   # flash one specific port
#   tools/flash.sh --monitor /dev/cu.usbmodem101   # flash then open monitor
#   tools/flash.sh --target esp32s3 PORT # skip detection, force the target
#   tools/flash.sh ch340:0-1.2           # a driverless CH340K board (see below)
#
# Boards on a CH340 bridge macOS has no driver for have no /dev/cu.* entry. They
# are found and flashed through tools/ch340_usb.py (libusb) under a ch340:... name.
#
# esp32c6 builds in firmware/build (sdkconfig); any other target in
# firmware/build-<target> (sdkconfig.<target>), so targets never clobber each other.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FW="$HERE/../firmware"

# shellcheck source=idf_env.sh
. "$HERE/idf_env.sh"

MONITOR=0
TARGET=""
PORTS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --monitor) MONITOR=1 ;;
    --target) TARGET="$2"; shift ;;
    --target=*) TARGET="${1#--target=}" ;;
    *) PORTS+=("$1") ;;
  esac
  shift
done

if [ "${#PORTS[@]}" -eq 0 ]; then
  # autodetect
  for p in /dev/cu.usbmodem* /dev/cu.usbserial* /dev/cu.wchusbserial* \
           /dev/cu.SLAB_USBtoUART*; do
    [ -e "$p" ] || continue
    case "$p" in *Bluetooth*|*debug-console*) continue ;; esac
    PORTS+=("$p")
  done
  while read -r p; do
    [ -n "$p" ] && PORTS+=("$p")
  done < <(python "$HERE/ch340_usb.py" list 2>/dev/null)
fi

if [ "${#PORTS[@]}" -eq 0 ]; then
  echo "No ESP32 serial ports detected. Plug a board in." >&2
  exit 1
fi

# idf.py arguments selecting the build tree for a target.
idf_args() {
  if [ "$1" = esp32c6 ]; then
    echo "-D IDF_TARGET=esp32c6"
  else
    echo "-B build-$1 -D SDKCONFIG=sdkconfig.$1 -D IDF_TARGET=$1"
  fi
}

# "ESP32-S3" as printed by esptool -> "esp32s3"
detect_target() {
  { case "$1" in
      ch340:*) python "$HERE/ch340_usb.py" esptool "$1" chip_id ;;
      *)       esptool.py -p "$1" chip_id ;;
    esac; } 2>/dev/null \
    | grep -oE 'ESP32-[A-Z][0-9]+' | head -1 | tr 'A-Z' 'a-z' | tr -d '-' || true
}

TARGETS=()
BUILT=" "
for p in "${PORTS[@]}"; do
  t="${TARGET:-$(detect_target "$p")}"
  case "$t" in
    esp32c6|esp32s3) ;;
    *) echo "$p: unsupported or undetected chip '${t:-?}'." >&2
       echo "Is another program holding the port (lsof $p)? Otherwise pass --target." >&2
       exit 1 ;;
  esac
  TARGETS+=("$t")
  case "$BUILT" in *" $t "*) continue ;; esac
  # sdkconfig is generated from sdkconfig.defaults* once and then wins over it,
  # so an edit to the defaults (e.g. enabling BLE) would be silently ignored.
  # It is disposable: drop it when any defaults file is newer.
  sc="$FW/sdkconfig"; [ "$t" = esp32c6 ] || sc="$FW/sdkconfig.$t"
  for d in "$FW/sdkconfig.defaults" "$FW/sdkconfig.defaults.$t"; do
    if [ -f "$sc" ] && [ -f "$d" ] && [ "$d" -nt "$sc" ]; then
      echo "$(basename "$d") changed - regenerating $(basename "$sc")"
      rm -f "$sc"
    fi
  done
  echo "Building for $t ..."
  # shellcheck disable=SC2046
  ( cd "$FW" && idf.py $(idf_args "$t") build )
  BUILT="$BUILT$t "
done

i=0
for p in "${PORTS[@]}"; do
  t="${TARGETS[$i]}"; i=$((i + 1))
  echo "== Flashing $p ($t) =="
  case "$p" in
    ch340:*)
      bdir="$FW/build"; [ "$t" = esp32c6 ] || bdir="$FW/build-$t"
      ( cd "$bdir" && python "$HERE/ch340_usb.py" esptool "$p" --chip "$t" -b 460800 \
          --after hard_reset write_flash "@flash_args" ) ;;
    *)
      # shellcheck disable=SC2046
      ( cd "$FW" && idf.py $(idf_args "$t") -p "$p" flash ) ;;
  esac
done

if [ "$MONITOR" -eq 1 ] && [ "${PORTS[0]#ch340:}" != "${PORTS[0]}" ]; then
  echo "--monitor is not available on ch340: ports; boot log:"
  python "$HERE/ch340_usb.py" reset "${PORTS[0]}"
elif [ "$MONITOR" -eq 1 ]; then
  # shellcheck disable=SC2046
  ( cd "$FW" && idf.py $(idf_args "${TARGETS[0]}") -p "${PORTS[0]}" monitor )
fi
