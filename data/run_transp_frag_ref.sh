#!/bin/bash
#
# Fragment-alpha reference: render a scene whose .inc carries intrinsic per-color
# transparency (4-component "rgbt" colors) and compare umbreon against POV-Ray.
#
# Unlike GROUP alpha (run_transp_ref.sh, which renders opaque layers and blends),
# FRAGMENT alpha is POV's native transmit: a single POV render of the scene IS
# the reference. umbreon reproduces it with front-to-back "over" compositing
# (every transparent surface, no dedup). The only expected residual is POV
# adaptive AA vs umbreon box supersample (and linear- vs gamma-space blending).
#
# Usage:  ./run_transp_frag_ref.sh <scene_basename> [width]
# Example:./run_transp_frag_ref.sh 1ab0_scene5_transp 300
#
# Output: <scene>_frag_pov.png (reference) and <scene>_frag_umb.png (umbreon).
# Set PYTHON=<venv>/bin/python to also print PSNR/SSIM.

set -eu

SCENE="${1:?scene basename (e.g. 1ab0_scene5_transp)}"
W="${2:-300}"
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

POV="${SCENE}_frag_pov.png"
UMB="${SCENE}_frag_umb.png"

# NB: no File_Gamma (POV's default output gamma matches umbreon, per the
# committed docs/match references). Declares match umbreon_cli's defaults.
echo "== POV native-transmit reference =="
povray "Input_File_Name=${SCENE}.pov" "Output_File_Name=${POV}" \
  Declare=_stereo=0 Declare=_iod=0.03 Declare=_perspective=0 Declare=_shadow=0 \
  Declare=_light_inten=1.3 Declare=_flash_frac=0.6 Declare=_amb_frac=0 \
  -D "+W$W" "+H$W" +FN8 Quality=11 Antialias=On Antialias_Depth=3 \
  Antialias_Threshold=0.1 Jitter=Off

echo "== umbreon (front-to-back over) =="
../build/umbreon_cli "${SCENE}.pov" -W "$W" -H "$W" -o "${UMB}"

if [ -n "${PYTHON:-}" ]; then
  echo "== compare =="
  "${PYTHON}" - "$POV" "$UMB" <<'PY'
import sys, numpy as np
from PIL import Image
from skimage.metrics import structural_similarity as ssim
a=np.asarray(Image.open(sys.argv[1]).convert("RGB"),np.float64)
b=np.asarray(Image.open(sys.argv[2]).convert("RGB"),np.float64)
mse=np.mean((a-b)**2); psnr=99 if mse==0 else 10*np.log10(255**2/mse)
print(f"umbreon vs POV native transmit:  PSNR {psnr:.2f} dB  SSIM {ssim(a,b,channel_axis=2,data_range=255):.4f}")
PY
fi
