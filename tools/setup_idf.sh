#!/usr/bin/env bash
# setup_idf.sh — one-time ESP-IDF install, SHARED by every ESP project on this Mac.
# Already done: ~/esp/esp-idf is at v5.5.5 with the esp32c6 toolchain in
# ~/.espressif. This is the reference procedure / fallback, ~2 GB. Do not vendor a
# second copy per project — point IDF_DIR at the shared one instead.
set -euo pipefail

IDF_BRANCH="${IDF_BRANCH:-v5.5.5}"
IDF_DIR="${IDF_DIR:-$HOME/esp/esp-idf}"

if [ -d "$IDF_DIR/.git" ]; then
  echo "ESP-IDF already present at $IDF_DIR"
else
  echo "Cloning ESP-IDF $IDF_BRANCH into $IDF_DIR ..."
  mkdir -p "$(dirname "$IDF_DIR")"
  git clone --recursive -j8 --branch "$IDF_BRANCH" --depth 1 \
    https://github.com/espressif/esp-idf.git "$IDF_DIR"
fi

echo "Installing the esp32c6 toolchain ..."
"$IDF_DIR/install.sh" esp32c6

cat <<MSG

Done. Activate the environment in each new shell before building:

    . "$IDF_DIR/export.sh"

Then, from this project's firmware/ directory:

    idf.py set-target esp32c6
    idf.py build
MSG
