#!/bin/bash
#
# Generate a CueMol-equivalent semi-transparent reference image for ONE
# transparent section, to validate umbreon's single-pass transparency against
# the POV-Ray + blendpng pipeline.
#
# CueMol's layered transparency renders two opaque layers and weighted-adds them
# (this is blendpng, NOT depth-order "over"; it is order-independent):
#   layer0 = scene with the transparent section HIDDEN  (the opaque "floor")
#   layer1 = scene with everything shown opaquely
#   ref    = layer0*(1-alpha) + layer1*alpha            (blendpng, sRGB 8-bit)
# For a single transparent section this is exactly umbreon's result, except the
# blend space: blendpng mixes in sRGB 8-bit, umbreon mixes in linear. That space
# difference (plus POV adaptive AA vs umbreon box supersample) is the expected
# residual when comparing; the compositing model itself is identical.
#
# Usage:  ./run_transp_ref.sh <scene_basename> <section_id> <alpha> [width]
# Example:./run_transp_ref.sh 1ab0_scene2 _34_37 0.5 300
#
#   <scene_basename>  a .pov in this directory (without the .pov extension)
#   <section_id>      a "_show_<id>" section; pass the <id> (e.g. _34_37)
#   <alpha>           opacity of that section (0..1)
#   [width]           square image size (default 300)
#
# Output: <scene>_<section>_a<alpha>_ref.png  (the CueMol-equivalent reference)
# Then compare against umbreon, e.g.:
#   ../build/umbreon_cli <scene>.pov --alpha <section>=<alpha> -o umbreon.png

set -eu

SCENE="${1:?scene basename}"
SECT="${2:?section id (e.g. _34_37)}"
ALPHA="${3:?alpha 0..1}"
W="${4:-300}"

HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

# Standard CueMol POV declares (match umbreon_cli's defaults).
DECL=(Declare=_stereo=0 Declare=_iod=0.03 Declare=_perspective=0
      Declare=_shadow=0 Declare=_light_inten=1.3 Declare=_flash_frac=0.6
      Declare=_amb_frac=0)
# NB: no File_Gamma here. POV's default output gamma (with the scene's
# assumed_gamma 2.2) reproduces the committed docs/match references bit-for-bit;
# passing File_Gamma=1 would write linear-encoded PNGs that do NOT match.
AA=(-D "+W$W" "+H$W" +FN8 Quality=11 Antialias=On Antialias_Depth=3
    Antialias_Threshold=0.1 Jitter=Off)

L0="${SCENE}${SECT}_a${ALPHA}_layer0.png"
L1="${SCENE}${SECT}_a${ALPHA}_layer1.png"
REF="${SCENE}${SECT}_a${ALPHA}_ref.png"

echo "== layer0 (section ${SECT} hidden = opaque floor) =="
povray "Input_File_Name=${SCENE}.pov" "Output_File_Name=${L0}" \
       "${DECL[@]}" "Declare=_show${SECT}=0" "${AA[@]}"

echo "== layer1 (all sections shown opaquely) =="
povray "Input_File_Name=${SCENE}.pov" "Output_File_Name=${L1}" \
       "${DECL[@]}" "${AA[@]}"

echo "== blendpng-equivalent composite: ref = layer0*(1-a) + layer1*a (sRGB) =="
"${PYTHON:-python3}" - "$L0" "$L1" "$ALPHA" "$REF" <<'PY'
import sys
from PIL import Image
import numpy as np
l0 = np.asarray(Image.open(sys.argv[1]).convert("RGB"), dtype=np.float64)
l1 = np.asarray(Image.open(sys.argv[2]).convert("RGB"), dtype=np.float64)
a = float(sys.argv[3])
out = l0 * (1.0 - a) + l1 * a          # blendpng mixes in 8-bit sRGB space
out = np.clip(out + 0.5, 0, 255).astype(np.uint8)
Image.fromarray(out, "RGB").save(sys.argv[4])
print("wrote", sys.argv[4])
PY

echo "Reference: $HERE/$REF"
echo "Compare with: ../build/umbreon_cli ${SCENE}.pov --alpha ${SECT}=${ALPHA} -W $W -H $W -o umbreon.png"
