#!/usr/bin/env python3
"""Contact sheet of a rendered scene for review (frames sampled every N seconds,
tiled with scene/global timestamps and the current beat id).

    python3 tools/contact_sheet.py s02 --quality preview --step 2 --cols 6
    -> out/s02/preview/contact_s02_preview.png (one or more pages)
"""
import argparse
import glob
import os
import sys

from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from carviz import spec as S, timeline as TL  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scene")
    ap.add_argument("--quality", default="preview")
    ap.add_argument("--step", type=float, default=2.0, help="seconds between samples")
    ap.add_argument("--cols", type=int, default=6)
    ap.add_argument("--rows", type=int, default=6, help="rows per page")
    ap.add_argument("--width", type=int, default=320, help="thumbnail width")
    ap.add_argument("--src", default="comp", choices=["comp", "raw"])
    ap.add_argument("--t0", type=float, default=0.0)
    ap.add_argument("--t1", type=float, default=None)
    a = ap.parse_args()
    sc = TL.scene(a.scene)
    d = os.path.join(ROOT, "out", a.scene, a.quality, a.src)
    have = sorted(int(os.path.basename(p)[:5]) for p in glob.glob(os.path.join(d, "[0-9]*.png")))
    if not have:
        raise SystemExit(f"no frames in {d}")
    t1 = sc.dur if a.t1 is None else a.t1
    picks, t = [], a.t0
    while t <= t1 + 1e-6:
        want = int(round(t * S.FPS)) + 1
        best = min(have, key=lambda f: abs(f - want))
        picks.append(best)
        t += a.step
    font = ImageFont.truetype(os.path.join(ROOT, "assets/fonts/Inter-SemiBold.otf"), 13)
    im0 = Image.open(os.path.join(d, f"{have[0]:05d}.png"))
    w = a.width
    h = int(w * im0.height / im0.width)
    per = a.cols * a.rows
    outs = []
    for page in range(0, len(picks), per):
        chunk = picks[page:page + per]
        rows = (len(chunk) + a.cols - 1) // a.cols
        sheet = Image.new("RGB", (a.cols * w, rows * (h + 18)), (20, 20, 20))
        dr = ImageDraw.Draw(sheet)
        for k, f in enumerate(chunk):
            im = Image.open(os.path.join(d, f"{f:05d}.png")).convert("RGB").resize((w, h), Image.LANCZOS)
            x, y = (k % a.cols) * w, (k // a.cols) * (h + 18)
            sheet.paste(im, (x, y + 18))
            ts = (f - 1) / S.FPS
            beat = next((b.id for b in sc.beats if b.start <= ts < b.end), sc.beats[-1].id)
            g = sc.start + ts
            dr.text((x + 4, y + 2), f"f{f} {ts:5.1f}s  [{int(g // 60)}:{g % 60:04.1f}] {beat}", fill=(230, 230, 230),
                    font=font)
        out = os.path.join(ROOT, "out", a.scene, a.quality, f"contact_{a.scene}_{a.quality}_{page // per + 1}.png")
        sheet.save(out)
        outs.append(out)
    print("\n".join(outs))


if __name__ == "__main__":
    main()
