#!/usr/bin/env python3
"""Stitch every scene's composited frames into video/final.mp4 (H.264).

Encodes straight from the PNG frames (one pass, frame-exact timing), and
muxes narration.srt as a soft (toggleable) subtitle track.  Also re-encodes
each scene video unless --final-only.

    python3 tools/stitch.py                  # quality 'final'
    python3 tools/stitch.py --quality draft --every 4   # quick full-length check

Size: GitHub refuses files > 100 MB, so by default final.mp4 is encoded with a
2-pass target bitrate that keeps it under ~95 MB; --crf N uses constant quality
instead.
"""
import argparse
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from carviz import spec as S, timeline as TL  # noqa: E402


def frame_list(quality, every):
    items = []
    for sc in TL.SCENES:
        comp = os.path.join(ROOT, "out", sc.id, quality, "comp")
        for f in range(1, sc.frames + 1, every):
            p = os.path.join(comp, f"{f:05d}.png")
            items.append((sc.id, f, p))
    return items


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quality", default="final")
    ap.add_argument("--every", type=int, default=1)
    ap.add_argument("--crf", type=int, default=None)
    ap.add_argument("--max-mb", type=float, default=95.0)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    items = frame_list(a.quality, a.every)
    missing = [i for i in items if not os.path.exists(i[2])]
    if missing:
        sc = sorted({m[0] for m in missing})
        raise SystemExit(f"{len(missing)} frames missing (scenes {sc}); first: {missing[0][2]}")
    vdir = os.path.join(ROOT, "video")
    os.makedirs(vdir, exist_ok=True)
    lst = os.path.join(ROOT, "out", f"final_{a.quality}_frames.txt")
    dur = a.every / S.FPS
    with open(lst, "w") as fh:
        for _, _, p in items:
            fh.write(f"file '{p}'\nduration {dur:.6f}\n")
        fh.write(f"file '{items[-1][2]}'\n")
    out = a.out or os.path.join(vdir, "final.mp4" if a.quality == "final" else f"final_{a.quality}.mp4")
    srt = os.path.join(ROOT, "narration.srt")
    total = TL.TOTAL
    base = ["ffmpeg", "-loglevel", "error", "-y", "-f", "concat", "-safe", "0", "-i", lst]
    vf = ["-vf", f"fps={S.FPS},format=yuv420p"]
    tmp = out + ".tmp.mp4"
    if a.crf is not None:
        cmd = base + ["-i", srt] + vf + ["-c:v", "libx264", "-preset", "slow", "-crf", str(a.crf),
                                          "-profile:v", "high", "-c:s", "mov_text",
                                          "-metadata:s:s:0", "language=eng", "-movflags", "+faststart", tmp]
        subprocess.run(cmd, check=True)
    else:
        kbps = int(a.max_mb * 8 * 1024 / total * 0.97)
        print(f"2-pass at {kbps} kb/s for {total:.1f} s")
        log = os.path.join(ROOT, "out", "x264pass")
        subprocess.run(base + vf + ["-c:v", "libx264", "-preset", "slow", "-b:v", f"{kbps}k", "-pass", "1",
                                    "-passlogfile", log, "-an", "-f", "mp4", os.devnull], check=True)
        subprocess.run(base + ["-i", srt] + vf + ["-c:v", "libx264", "-preset", "slow", "-b:v", f"{kbps}k",
                                                  "-pass", "2", "-passlogfile", log, "-profile:v", "high",
                                                  "-c:s", "mov_text", "-metadata:s:s:0", "language=eng",
                                                  "-movflags", "+faststart", tmp], check=True)
    os.replace(tmp, out)
    size = os.path.getsize(out) / 1e6
    print(f"wrote {out} ({size:.1f} MB, {len(items)} frames)")


if __name__ == "__main__":
    main()
