#!/bin/bash
# A/B harness for the group-alpha multipass reuse (--blend-reuse).
# Renders each benchmark scene twice (--blend-reuse off / on), md5-compares
# the PNGs (bit-exactness) and reports wall times (speedup).
#
# Usage: scripts/blend_ab.sh [scene_dir1 ...]
#   Default scene dirs: ~/tmp/20260703_gi_transp (transp1/transp2, alpha
#   _41_42=0.5) and ~/tmp/20260703_transp_opt (transp3, 3 groups 0.2/0.3/0.4).
# Requires build/umbreon_cli (task build).
set -u

CLI="$(dirname "$0")/../build/umbreon_cli"
[ -x "$CLI" ] || { echo "error: $CLI not found (run 'task build')" >&2; exit 1; }

OUTDIR="${TMPDIR:-/tmp}/blend_ab_$$"
mkdir -p "$OUTDIR"
trap 'rm -rf "$OUTDIR"' EXIT

PASS=0
FAIL=0

# run <name> <pov> <extra args...>
# Renders off/on, compares md5, prints times.
run() {
  local name="$1" pov="$2"; shift 2
  local out_off="$OUTDIR/${name}_off.png" out_on="$OUTDIR/${name}_on.png"
  local t0 t1 t2
  t0=$(date +%s.%N)
  "$CLI" "$pov" "$@" --blend-reuse off -o "$out_off" >/dev/null 2>&1 \
    || { echo "FAIL $name (render off failed)"; FAIL=$((FAIL+1)); return; }
  t1=$(date +%s.%N)
  "$CLI" "$pov" "$@" --blend-reuse on -o "$out_on" >/dev/null 2>&1 \
    || { echo "FAIL $name (render on failed)"; FAIL=$((FAIL+1)); return; }
  t2=$(date +%s.%N)
  local m_off m_on
  m_off=$(md5 -q "$out_off" 2>/dev/null || md5sum "$out_off" | cut -d' ' -f1)
  m_on=$(md5 -q "$out_on" 2>/dev/null || md5sum "$out_on" | cut -d' ' -f1)
  local d_off d_on speedup
  d_off=$(echo "$t1 $t0" | awk '{printf "%.2f", $1-$2}')
  d_on=$(echo "$t2 $t1" | awk '{printf "%.2f", $1-$2}')
  speedup=$(echo "$d_off $d_on" | awk '{ if ($2 > 0) printf "%.2fx", $1/$2; else printf "n/a" }')
  if [ "$m_off" = "$m_on" ]; then
    echo "ok   $name  off=${d_off}s on=${d_on}s speedup=${speedup}"
    PASS=$((PASS+1))
  else
    echo "FAIL $name  MD5 MISMATCH (off=$m_off on=$m_on)"
    FAIL=$((FAIL+1))
  fi
}

# --- scene set B/D: transp1 (opaque ball-stick + transparent ribbon),
# transp2 (single blend group = whole scene) ---
D1=~/tmp/20260703_gi_transp
if [ -f "$D1/transp1.pov" ]; then
  for f in transp1 transp2; do
    run "${f}_rt"    "$D1/$f.pov" --alpha _41_42=0.5
    run "${f}_edges" "$D1/$f.pov" --alpha _41_42=0.5 --edges on
    run "${f}_pt1"   "$D1/$f.pov" --quality high --integrator pt1 --alpha _41_42=0.5
  done
else
  echo "skip: $D1 not found"
fi

# --- scene E: transp3 (3 sections, mixed mesh/ball-stick, alphas 0.2/0.3/0.4) ---
D2=~/tmp/20260703_transp_opt
if [ -f "$D2/transp3.pov" ]; then
  A3="--alpha _33_34=0.2 --alpha _33_37=0.3 --alpha _48_49=0.4"
  run "transp3_rt"     "$D2/transp3.pov" $A3
  run "transp3_rt_1k"  "$D2/transp3.pov" $A3 -W 1024 -H 1024
  run "transp3_edges"  "$D2/transp3.pov" $A3 --edges on
  run "transp3_pt1"    "$D2/transp3.pov" $A3 --quality high --integrator pt1
else
  echo "skip: $D2 not found"
fi

echo "----"
echo "blend_ab: $PASS ok, $FAIL failed"
[ "$FAIL" -eq 0 ]
