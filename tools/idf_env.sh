# idf_env.sh — activate the SHARED ESP-IDF install. SOURCE this, don't run it.
#
#   . tools/idf_env.sh
#
# One IDF install is shared by every ESP project on this Mac; nothing is vendored
# per-project. Resolution order, first hit wins:
#
#   1. IDF_PATH already exported  — a shell that ran export.sh; reused as-is
#   2. $IDF_DIR                   — explicit override
#   3. ~/esp/esp-idf              — the shared default (currently v5.5.5)
#
# The toolchain itself lives in IDF_TOOLS_PATH (~/.espressif by default) and is
# shared the same way — export.sh puts it on PATH.

if [ -n "${IDF_PATH:-}" ] && command -v idf.py >/dev/null 2>&1; then
  :                                          # already active in this shell
else
  _idf_dir="${IDF_DIR:-$HOME/esp/esp-idf}"
  if [ ! -f "$_idf_dir/export.sh" ]; then
    echo "ESP-IDF not found at $_idf_dir" >&2
    echo "Install it with tools/setup_idf.sh, or point IDF_DIR at an existing copy." >&2
    unset _idf_dir
    return 1 2>/dev/null || exit 1
  fi
  # IDF's export.sh trips over `set -u` and returns non-zero on some paths;
  # relax both while sourcing, then restore.
  _idf_had_u=0; case "$-" in *u*) _idf_had_u=1 ;; esac
  _idf_had_e=0; case "$-" in *e*) _idf_had_e=1 ;; esac
  set +u +e
  . "$_idf_dir/export.sh" >/dev/null
  _idf_rc=$?
  [ "$_idf_had_u" = 1 ] && set -u
  [ "$_idf_had_e" = 1 ] && set -e
  if [ "$_idf_rc" -ne 0 ] || ! command -v idf.py >/dev/null 2>&1; then
    echo "ESP-IDF export.sh failed (rc=$_idf_rc) for $_idf_dir" >&2
    unset _idf_dir _idf_rc _idf_had_u _idf_had_e
    return 1 2>/dev/null || exit 1
  fi
  unset _idf_dir _idf_rc _idf_had_u _idf_had_e
fi
