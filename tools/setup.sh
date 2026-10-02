#!/usr/bin/env bash
# One-time environment setup on Ubuntu 22.04/24.04 (CPU-only cloud box).
# - Blender 5.0.1 as a Python module (pip wheel, Python 3.11)
# - Mesa EGL (llvmpipe) so Workbench/EEVEE can create a GL context headless
# - ffmpeg for encoding, Inter font (bundled in assets/fonts)
set -euo pipefail
cd "$(dirname "$0")/.."
SUDO=""; [ "$(id -u)" -ne 0 ] && SUDO="sudo"
$SUDO apt-get update -qq
DEBIAN_FRONTEND=noninteractive $SUDO apt-get install -y -qq \
  libegl1 libegl-mesa0 libgl1-mesa-dri libgl1 libgbm1 libxkbcommon0 libxi6 libxxf86vm1 \
  libxfixes3 libxrender1 libsm6 ffmpeg
python3 --version | grep -q "3.11" || { echo "bpy 5.0.1 needs Python 3.11"; exit 1; }
python3 -m pip install -r requirements.txt
# HDRIs (CC0, Poly Haven) are committed in assets/hdri; re-download if missing
for n in studio_small_09 studio_small_03 photo_studio_loft_hall; do
  f="assets/hdri/${n}_2k.hdr"
  [ -s "$f" ] || curl -sSfL -o "$f" "https://dl.polyhaven.org/file/ph-assets/HDRIs/hdr/2k/${n}_2k.hdr"
done
export EGL_PLATFORM=surfaceless
python3 - <<'PY'
import bpy; print("bpy", bpy.app.version_string, "OK")
PY
python3 tools/test_kinematics.py | tail -1
