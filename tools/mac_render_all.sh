#!/usr/bin/env bash
# Render the whole film at 1920x1080 on an Apple-Silicon Mac (Cycles on the Metal GPU),
# then stitch video/final.mp4 (<= 95 MB, fits GitHub) and video/final_hq.mp4 (CRF 18 master).
#
#   git pull                         # get the latest scenes first
#   tools/mac_render_all.sh          # render + stitch (resumable: just run it again)
#   tools/mac_render_all.sh --push   # ... and commit + push the videos when done
#   tools/mac_render_all.sh --fresh  # delete earlier final frames first (normally not needed:
#                                    #  frames at a different resolution are re-rendered anyway)
#
#   CARVIZ_SCENES="s03 s05" tools/mac_render_all.sh   # only some scenes (stitch needs all eight)
#
# Keep the Mac on power with the lid open; the script keeps it awake (caffeinate).
# Progress: tail -f out/render_<scene>_final_shard0.log  (one line every 10 frames, with ETA)
set -euo pipefail
SELF="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
cd "$(dirname "$SELF")/.."

SCENES="${CARVIZ_SCENES:-s01 s02 s03 s04 s05 s06 s07 s08}"
DEVICE="${CARVIZ_DEVICE:-metal}"
PUSH=0; FRESH=0
for arg in ${1+"$@"}; do
  case "$arg" in
    --push) PUSH=1 ;;
    --fresh) FRESH=1 ;;
    *) echo "unknown option: $arg"; exit 2 ;;
  esac
done

# keep the Mac awake for the whole run
if [ -z "${CARVIZ_CAFFEINATED:-}" ] && command -v caffeinate >/dev/null 2>&1; then
  export CARVIZ_CAFFEINATED=1
  exec caffeinate -is "$SELF" ${1+"$@"}
fi

mkdir -p out video
LOG=out/mac_render_all.log
exec > >(tee -a "$LOG") 2>&1
echo "=== $(date '+%F %T') mac_render_all start (commit $(git rev-parse --short HEAD 2>/dev/null || echo '?'))"

# ---- Python 3.11 with bpy -------------------------------------------------------------
if [ -x .venv/bin/python ]; then PY=.venv/bin/python
elif command -v python3.11 >/dev/null 2>&1; then PY=python3.11
else PY=python3; fi
"$PY" - <<'PY' || { echo "Python 3.11 with bpy 5.0.1 is needed: python3.11 -m venv .venv && .venv/bin/pip install -r requirements.txt"; exit 1; }
import sys
assert sys.version_info[:2] == (3, 11), f"Python {sys.version.split()[0]} (bpy 5.0.1 needs 3.11)"
import bpy
print("bpy", bpy.app.version_string, "on Python", sys.version.split()[0])
PY
command -v ffmpeg >/dev/null 2>&1 || { echo "ffmpeg missing: brew install ffmpeg"; exit 1; }

# ---- up to date? ---------------------------------------------------------------------
if git fetch -q origin 2>/dev/null; then
  BR=$(git rev-parse --abbrev-ref HEAD)
  if [ -n "$(git rev-list HEAD..origin/"$BR" 2>/dev/null)" ]; then
    echo "WARNING: origin/$BR has newer commits - run 'git pull' first (Ctrl-C now to abort)."
    sleep 20
  fi
fi

if [ "$FRESH" = 1 ]; then
  echo "--fresh: removing earlier final frames"
  rm -rf out/s0?/final
fi

# ---- benchmark one heavy frame; choose the denoiser prefilter ---------------------------
# ACCURATE removes speckles on the see-through shells; if it cannot run on the GPU it is
# much slower, and FAST keeps the overnight time budget (tools/bench.py decides).
if [ -z "${CARVIZ_PREFILTER:-}" ]; then
  BENCH=$("$PY" tools/bench.py s08 577 --device "$DEVICE" 2>&1 | tee -a "$LOG" | grep '^BENCH ' || true)
  echo "$BENCH"
  case "$BENCH" in
    *choose=FAST*) export CARVIZ_PREFILTER=FAST ;;
    *choose=ACCURATE*) export CARVIZ_PREFILTER=ACCURATE ;;
    *) echo "benchmark failed - see $LOG"; exit 1 ;;
  esac
fi
echo "denoiser prefilter: $CARVIZ_PREFILTER"

# ---- render all scenes (resumable) -----------------------------------------------------
T0=$(date +%s)
# shellcheck disable=SC2086
"$PY" tools/render_finals.py $SCENES --device "$DEVICE" --shards 2 --threads 0 --nice 0

# ---- check + stitch --------------------------------------------------------------------
MISSING=""
for s in s01 s02 s03 s04 s05 s06 s07 s08; do
  [ -s "video/${s}_final.mp4" ] || MISSING="$MISSING $s"
done
if [ -n "$MISSING" ]; then
  echo "Scenes not finished:$MISSING - run tools/mac_render_all.sh again to resume (see out/render_finals logs)."
  exit 1
fi
"$PY" tools/stitch.py --hq

echo
echo "=== $(date '+%F %T') done in $(( ($(date +%s) - T0) / 60 )) min"
for f in video/s0?_final.mp4 video/final.mp4 video/final_hq.mp4; do
  printf "%-24s %8s  %s\n" "$f" "$(du -h "$f" | cut -f1)" \
    "$(ffprobe -v error -show_entries stream=width,height,nb_frames -select_streams v:0 -of csv=p=0 "$f")"
done

if [ "$PUSH" = 1 ]; then
  git add video/s0?_final.mp4 video/final.mp4
  git commit -m "Final 1080p videos (Cycles on Apple M-series GPU via Metal): s01-s08 and final.mp4" || true
  git pull --rebase origin "$(git rev-parse --abbrev-ref HEAD)" && git push origin HEAD \
    || echo "Push failed - run: git pull --rebase && git push"
else
  echo "To publish: git add video/s0?_final.mp4 video/final.mp4 && git commit -m 'Final 1080p videos' && git pull --rebase && git push"
fi
echo "video/final_hq.mp4 is the full-quality master (too big for GitHub; share it directly)."
