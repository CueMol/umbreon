#!/bin/sh
# A/B comparison of the two diffuse-GI integrators (irradiance cache vs pt1
# path-traced gather): render the same scene with the same camera and settings,
# save both images + timing into outputs/compare_<scene>_<date>/, and print
# the PSNR/SSIM between them and the wall-time ratio.
#
# Usage: scripts/compare_integrators.sh [scene.pov] [extra umbreon_cli flags...]
#   scene defaults to data/1ab0_scene6_densurf.pov (the largest mesh scene).
set -eu

SCENE=${1:-data/1ab0_scene6_densurf.pov}
[ $# -gt 0 ] && shift
CLI=${UMBREON_CLI:-build/umbreon_cli}

if [ ! -x "$CLI" ]; then
  echo "error: $CLI not found or not executable (run 'task build' first," \
       "or set UMBREON_CLI)" >&2
  exit 1
fi
if [ ! -f "$SCENE" ]; then
  echo "error: scene '$SCENE' not found" >&2
  exit 1
fi

BASE=$(basename "$SCENE" .pov)
OUT="outputs/compare_${BASE}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"

# Identical scene/camera/settings for both integrators; --supersample 1 so the
# pt1 half-res gather means half of the output resolution (see pt1_design.md).
COMMON="--gi on --supersample 1 -W 1920 -H 1080 --seed 0"

echo "== cache integrator: $SCENE"
T0=$(date +%s)
# shellcheck disable=SC2086  # COMMON is intentionally word-split
"$CLI" "$SCENE" -o "$OUT/cache.ppm" --integrator cache $COMMON "$@" \
  | tee "$OUT/log_cache.txt"
T1=$(date +%s)
T_CACHE=$((T1 - T0))

echo "== pt1 integrator: $SCENE"
T0=$(date +%s)
# shellcheck disable=SC2086
"$CLI" "$SCENE" -o "$OUT/pt1.ppm" --integrator pt1 $COMMON "$@" \
  | tee "$OUT/log_pt1.txt"
T1=$(date +%s)
T_PT1=$((T1 - T0))
[ -f outputs/timing.json ] && mv outputs/timing.json "$OUT/timing_pt1.json"

echo "== pt2 integrator: $SCENE"
T0=$(date +%s)
# shellcheck disable=SC2086
"$CLI" "$SCENE" -o "$OUT/pt2.ppm" --integrator pt2 $COMMON "$@" \
  | tee "$OUT/log_pt2.txt"
T1=$(date +%s)
T_PT2=$((T1 - T0))
[ -f outputs/timing.json ] && mv outputs/timing.json "$OUT/timing_pt2.json"

# PNGs for viewing (the PPMs stay for the linear-space metric).
"$CLI" --convert "$OUT/cache.ppm" "$OUT/cache.png"
"$CLI" --convert "$OUT/pt1.ppm" "$OUT/pt1.png"
"$CLI" --convert "$OUT/pt2.ppm" "$OUT/pt2.png"

echo "== metrics"
{
  printf "cache vs pt1: "
  "$CLI" --compare "$OUT/cache.ppm" "$OUT/pt1.ppm"
  printf "pt1 vs pt2:   "
  "$CLI" --compare "$OUT/pt1.ppm" "$OUT/pt2.ppm"
} | tee "$OUT/metrics.txt"
echo "cache: ${T_CACHE}s   pt1: ${T_PT1}s   pt2: ${T_PT2}s" | tee -a "$OUT/metrics.txt"
awk -v c="$T_CACHE" -v p="$T_PT1" -v q="$T_PT2" 'BEGIN {
  if (p > 0) printf "time ratio (cache/pt1): %.2f\n", c / p;
  else print "time ratio (cache/pt1): n/a (pt1 < 1s)";
  if (p > 0 && q > 0) printf "time ratio (pt2/pt1): %.2f\n", q / p;
}' | tee -a "$OUT/metrics.txt"

echo "results in $OUT/"
