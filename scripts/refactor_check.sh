#!/bin/sh
# Bit-exact refactoring gate: render a fixed scene/flag matrix to PPM and
# byte-compare every image against a stored baseline. Any refactor that claims
# to preserve output must keep every case byte-identical (cmp, not PSNR).
#
# Usage:
#   scripts/refactor_check.sh baseline   # (re)generate the baseline images
#   scripts/refactor_check.sh [check]    # render again and byte-compare
#
# The baseline is generated once from the pre-refactor commit into
# outputs/refactor_baseline/ (gitignored) and kept across refactor stages.
# Because "check" re-renders from scratch, running "baseline" followed by
# "check" on the same commit also proves run-to-run determinism.
#
# Cases whose LFS-backed .inc geometry is not materialized are skipped with a
# warning so the script still works on checkouts without Git LFS (e.g. CI).
set -eu

CLI=${UMBREON_CLI:-build/umbreon_cli}
BASE_DIR=${REFACTOR_BASELINE_DIR:-outputs/refactor_baseline}
MODE=${1:-check}

# Small output and a pinned thread count: threads must match between baseline
# and check runs (the renderer is deterministic per thread-count, not across).
COMMON="-W 256 -H 256 --threads 4"

case "$MODE" in
  baseline) OUT_DIR=$BASE_DIR ;;
  check)    OUT_DIR=outputs/refactor_check ;;
  *) echo "usage: $0 [baseline|check]" >&2; exit 2 ;;
esac

if [ ! -x "$CLI" ]; then
  echo "error: $CLI not found or not executable (run 'task build' first," \
       "or set UMBREON_CLI)" >&2
  exit 1
fi
mkdir -p "$OUT_DIR"

PASSED=0
FAILED=0
SKIPPED=0

is_lfs_pointer() {
  head -c 60 "$1" 2>/dev/null | grep -q "git-lfs"
}

# render_case <name> <scene.pov> [extra flags...]
# The scene's geometry include is assumed to be <scene>.inc (true for every
# scene in the matrix below).
render_case() {
  name=$1
  scene=$2
  shift 2
  inc="${scene%.pov}.inc"
  if [ ! -f "$scene" ] || [ ! -f "$inc" ] || is_lfs_pointer "$inc"; then
    echo "SKIP $name: $inc missing or an unmaterialized LFS pointer"
    SKIPPED=$((SKIPPED + 1))
    return 0
  fi
  out="$OUT_DIR/$name.ppm"
  # shellcheck disable=SC2086  # COMMON is intentionally word-split
  if ! "$CLI" "$scene" -o "$out" $COMMON "$@" >"$OUT_DIR/$name.log" 2>&1; then
    echo "FAIL $name: render error (see $OUT_DIR/$name.log)"
    FAILED=$((FAILED + 1))
    return 0
  fi
  if [ "$MODE" = "baseline" ]; then
    echo "BASE $name: $out"
    PASSED=$((PASSED + 1))
    return 0
  fi
  ref="$BASE_DIR/$name.ppm"
  if [ ! -f "$ref" ]; then
    echo "SKIP $name: no baseline image ($ref); run '$0 baseline' first"
    SKIPPED=$((SKIPPED + 1))
  elif cmp -s "$ref" "$out"; then
    echo "OK   $name"
    PASSED=$((PASSED + 1))
  else
    echo "FAIL $name: output differs from baseline ($ref vs $out)"
    FAILED=$((FAILED + 1))
  fi
}

# --- scene/flag matrix -------------------------------------------------------
# Each case pins one code path that the refactor must keep byte-identical.
render_case default data/test1.pov
render_case aa      data/test1.pov --aa adaptive
render_case ao      data/test1.pov --ao-samples 16 --ao-res out --ao-ld on
render_case gi      data/1ab0_scene1.pov --gi on
render_case pt1     data/1ab0_scene1.pov --integrator pt1 --quality draft --seed 1
render_case pt2     data/1ab0_scene1.pov --integrator pt2 --seed 1
render_case pt2em   data/emissive_test.pov --integrator pt2 --seed 1
render_case edges   data/edges/edge_ribbon1.pov --edges on
render_case transp  data/1ab0_scene5_transp.pov --alpha _52_53=0.5
render_case objedge data/test1.pov --obj-edges on

echo "----"
echo "$MODE: $PASSED ok, $FAILED failed, $SKIPPED skipped"
[ "$FAILED" -eq 0 ] || exit 1
