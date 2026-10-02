#!/usr/bin/env python3
"""Overlay demo / visual test: draws every HUD widget and several label styles
onto synthetic backgrounds at 1280x720 and 640x360.

    python3 tools/overlay_demo.py [--out DIR] [--guides] [--bench] [--seq]

--guides  also writes copies with the margins, the subtitle band (bottom-centre
          18 %) and a sample two-line subtitle drawn in, to check the layout.
--bench   times render_overlay/compose_frame on a busy 1280x720 frame (1 core).
--seq     exercises compose_sequence (JSON in, resumable, parallel) on a short
          animated sequence and checks resume/atomic behaviour.
"""
import argparse
import json
import math
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFilter, ImageFont  # noqa: E402

from carviz import overlay as ov  # noqa: E402

DEFAULT_OUT = "/tmp/claude-0/-home-user-internet-programming/c84c1512-5ccc-5790-b044-4c62d4f91067/scratchpad/overlay"


# ---------------------------------------------------------------------------
# synthetic backgrounds
# ---------------------------------------------------------------------------

def _radial(W, H, c_in, c_out, cx=0.5, cy=0.45, power=1.4):
    x = (np.arange(W) / W - cx) * (W / H)
    y = np.arange(H) / H - cy
    d = np.sqrt(x[None, :] ** 2 + y[:, None] ** 2) / 0.95
    t = np.clip(d, 0, 1) ** power
    c_in, c_out = np.array(c_in, float), np.array(c_out, float)
    img = c_in[None, None, :] * (1 - t[..., None]) + c_out[None, None, :] * t[..., None]
    return img


def _gear(draw, cx, cy, r, teeth, fill, s):
    pts = []
    for k in range(teeth * 4):
        a = 2 * math.pi * k / (teeth * 4)
        rr = r if (k % 4) in (0, 1) else r * 0.88
        pts.append((cx + rr * math.cos(a), cy + rr * math.sin(a)))
    draw.polygon([(x * s, y * s) for x, y in pts], fill=fill)


def background(kind, W, H):
    """kind: 'dark' (studio with grey metal shapes), 'light', 'warm' (mid)."""
    s = 2  # supersample the shapes
    if kind == "dark":
        base = _radial(W, H, (46, 49, 55), (7, 8, 10))
        cols = [(118, 122, 128), (86, 90, 96), (150, 154, 160), (64, 67, 72)]
    elif kind == "light":
        base = _radial(W, H, (236, 238, 241), (168, 172, 179))
        cols = [(70, 74, 80), (120, 124, 131), (40, 43, 48), (150, 154, 160)]
    else:
        base = _radial(W, H, (92, 84, 74), (20, 18, 16))
        cols = [(170, 160, 150), (120, 112, 104), (210, 200, 190), (60, 56, 52)]
    im = Image.fromarray(base.clip(0, 255).astype("uint8"), "RGB").resize((W * s, H * s), Image.BILINEAR)
    d = ImageDraw.Draw(im)
    u = H / 100.0
    # floor shadow
    sh = Image.new("L", im.size, 0)
    ImageDraw.Draw(sh).ellipse([0.22 * W * s, 0.70 * H * s, 0.80 * W * s, 0.86 * H * s], fill=150)
    sh = sh.filter(ImageFilter.GaussianBlur(4 * u * s))
    im.paste((0, 0, 0), (0, 0), sh.point(lambda v: int(v * 0.6)))
    # an engine-ish block, shafts and gears
    d.rounded_rectangle([0.30 * W * s, 0.30 * H * s, 0.52 * W * s, 0.70 * H * s], radius=int(2 * u * s), fill=cols[1])
    d.rectangle([0.32 * W * s, 0.34 * H * s, 0.50 * W * s, 0.40 * H * s], fill=cols[3])
    d.rectangle([0.50 * W * s, 0.47 * H * s, 0.86 * W * s, 0.52 * H * s], fill=cols[0])
    _gear(d, 0.60 * W, 0.50 * H, 13 * u, 26, cols[2], s)
    _gear(d, 0.60 * W, 0.50 * H, 4 * u, 12, cols[3], s)
    _gear(d, 0.72 * W, 0.62 * H, 9 * u, 18, cols[0], s)
    d.ellipse([0.405 * W * s - 6 * u * s, 0.48 * H * s - 6 * u * s, 0.405 * W * s + 6 * u * s, 0.48 * H * s + 6 * u * s],
              fill=cols[2])
    im = im.filter(ImageFilter.GaussianBlur(0.6 * s)).resize((W, H), Image.LANCZOS)
    rng = np.random.default_rng(1)
    a = np.asarray(im).astype(np.int16) + rng.integers(-3, 4, size=(H, W, 1))
    return Image.fromarray(a.clip(0, 255).astype("uint8"), "RGB")


# ---------------------------------------------------------------------------
# scenarios (label + hud lists exactly as they appear in labels.json / hud.json)
# ---------------------------------------------------------------------------

def scenarios():
    S = {}
    S["shift"] = dict(  # s05-like: everything a gear-shift shot shows
        labels=[
            dict(id="sleeve", text="Synchro sleeve", ax=0.43, ay=0.36, dx=-0.07, dy=-0.07, style="part"),
            dict(id="blocker", text="Blocker ring", ax=0.60, ay=0.40, dx=0.08, dy=-0.06, style="emph"),
            dict(id="dogs", text="Dog teeth", ax=0.66, ay=0.62, dx=0.07, dy=0.06, style="part", occluded=True),
            dict(id="gear2", text="2nd gear", ax=0.40, ay=0.60, dx=-0.08, dy=0.05, style="dim"),
        ],
        hud=[
            dict(type="section_title", number=5, title="A gear shift"),
            dict(type="step_card", number=3, text="Synchronize"),
            dict(type="readouts", rows=[["Engine", "1210 rpm"], ["2nd gear", "1446 rpm", True],
                                        ["Output shaft", "861 rpm"]]),
            dict(type="slowmo", factor=150),
            dict(type="status", text="Synchronizing", kind="warn"),
            dict(type="gear", value="N"),
            dict(type="pedal", value=1.0),
            dict(type="hpattern", x=-1.0, y=-0.45),
        ])
    S["recap"] = dict(  # s08-like: the full cluster, real time
        labels=[
            dict(id="engine", text="Engine", ax=0.40, ay=0.45, dx=-0.07, dy=-0.07, style="emph"),
            dict(id="gearbox", text="Gearbox", ax=0.58, ay=0.50, dx=0.06, dy=-0.08),
        ],
        hud=[
            dict(type="slowmo", factor=1),
            dict(type="status", text="Clutch engaged", kind="ok"),
            dict(type="gear", value=2),
            dict(type="speed", value_kmh=31.6),
            dict(type="pedal", value=0.0),
            dict(type="rpm", value=2340, label="Engine"),
            dict(type="hpattern", x=-1.0, y=-1.0),
        ])
    S["engine"] = dict(  # s02-like
        labels=[
            dict(id="intake", text="Intake valve", ax=0.36, ay=0.36, dx=-0.08, dy=-0.05),
            dict(id="plug", text="Spark plug", ax=0.405, ay=0.30, dx=0.0, dy=-0.09),
            dict(id="piston", text="Piston", ax=0.40, ay=0.55, dx=-0.09, dy=0.0),
            dict(id="crank", text="Crankshaft", ax=0.46, ay=0.68, dx=0.07, dy=0.06),
        ],
        hud=[
            dict(type="section_title", number=2, title="The engine"),
            dict(type="stroke_strip", active=1, progress=0.62, cylinder=1),
            dict(type="slowmo", factor=198.4),
            dict(type="rpm", value=850, label="Engine"),
            dict(type="readouts", title="Valvetrain", rows=[["Crank", "850 rpm"], ["Camshafts", "425 rpm"]]),
        ])
    S["firing"] = dict(
        labels=[dict(id="c1", text="1", ax=0.33, ay=0.40, dx=0.0, dy=-0.08),
                dict(id="c2", text="2", ax=0.40, ay=0.40, dx=0.0, dy=-0.08),
                dict(id="c3", text="3", ax=0.47, ay=0.40, dx=0.0, dy=-0.08, style="emph"),
                dict(id="c4", text="4", ax=0.54, ay=0.40, dx=0.0, dy=-0.08)],
        hud=[dict(type="firing_ticker", active=1), dict(type="slowmo", factor=70.3),
             dict(type="rpm", value=850)])
    S["gearbox"] = dict(  # s04-like ratios + reverse
        labels=[dict(id="idler", text="Reverse idler", ax=0.72, ay=0.62, dx=-0.08, dy=0.07, style="emph"),
                dict(id="out", text="Output shaft", ax=0.80, ay=0.495, dx=0.0, dy=-0.10)],
        hud=[dict(type="ratio_card", ratio_text="3.48 : 1", caption="1st gear"),
             dict(type="readouts", rows=[["Input shaft", "1500 rpm"], ["Output shaft", "431 rpm"]]),
             dict(type="slowmo", factor=12),
             dict(type="status", text="Disengaged", kind="info"),
             dict(type="gear", value="R"), dict(type="rpm", value=6900, label="Engine"),
             dict(type="hpattern", x=1.0, y=-1.0)])
    S["clutch"] = dict(  # s03-like take-off: slipping clutch, pedal part-way, kind inferred from text
        labels=[dict(id="fw", text="Flywheel", ax=0.405, ay=0.48, dx=-0.08, dy=-0.07),
                dict(id="disc", text="Friction disc", ax=0.60, ay=0.42, dx=0.06, dy=-0.09, style="emph"),
                dict(id="pp", text="Pressure plate", ax=0.72, ay=0.62, dx=0.07, dy=0.06),
                dict(id="ds", text="Diaphragm spring", ax=0.47, ay=0.60, dx=-0.08, dy=0.08, occluded=True)],
        hud=[dict(type="section_title", number=3, title="The clutch"),
             dict(type="readouts", rows=[["Engine", "1480 rpm"], ["Disc (input shaft)", "620 rpm"]]),
             dict(type="slowmo", factor=1 / 12.0),
             dict(type="status", text="SLIPPING"),
             dict(type="gear", value="1"), dict(type="speed", value_kmh=104.6),
             dict(type="pedal", value=0.36)])
    S["title"] = dict(labels=[], hud=[dict(type="title_card", title="How a Manual Car Works",
                                           subtitle="Inside a front-engine, rear-wheel-drive car")])
    S["fade"] = dict(  # partial alphas + fade over everything
        labels=[dict(id="fw", text="Flywheel", ax=0.60, ay=0.50, dx=0.08, dy=-0.06, alpha=0.5)],
        hud=[dict(type="section_title", number=3, title="The clutch", alpha=0.5),
             dict(type="step_card", number=1, text="Clutch in", total=5),
             dict(type="slowmo", factor=2.5, alpha=0.7),
             dict(type="fade", color="black", alpha=0.35)])
    S["edge"] = dict(  # label clamping: near edges / anchors in the subtitle band / off-frame
        labels=[dict(id="a", text="Rear wheel", ax=0.97, ay=0.50, dx=0.07, dy=-0.06),
                dict(id="b", text="Driveshaft", ax=0.50, ay=0.92, dx=0.06, dy=0.05),
                dict(id="c", text="Differential", ax=0.06, ay=0.10, dx=-0.07, dy=-0.06),
                dict(id="d", text="Propeller\nshaft", ax=0.30, ay=0.62, dx=0.0, dy=0.0),
                dict(id="e", text="Off frame", ax=1.01, ay=0.4, dx=-0.07, dy=0.0)],
        hud=[])
    return S


def draw_guides(im):
    """Margins, subtitle band and a sample subtitle (for layout checks only)."""
    W, H = im.size
    u = H / 100
    ov_ = Image.new("RGBA", im.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(ov_)
    m = ov.MARGIN_U * u
    d.rectangle([m, m, W - m, H - m], outline=(0, 255, 255, 120), width=1)
    d.rectangle([0.15 * W, ov.SAFE_Y * H, 0.85 * W, H - 1], fill=(255, 0, 255, 40), outline=(255, 0, 255, 140))
    f = ImageFont.truetype(os.path.join(ov.FONT_DIR, "Inter-Medium.otf"), 4.6 * u)
    for i, line in enumerate(["and out through two driveshafts to the", "wheels."]):
        y = 0.875 * H + i * 5.6 * u
        d.text((W / 2, y), line, font=f, fill=(255, 255, 255, 255), anchor="ms", stroke_width=max(1, int(0.3 * u)),
               stroke_fill=(0, 0, 0, 255))
    base = im.convert("RGBA")
    base.alpha_composite(ov_)
    return base.convert("RGB")


def run_demo(out, guides=False, bg_path=None):
    os.makedirs(out, exist_ok=True)
    S = scenarios()
    bgs = {"shift": "dark", "recap": "light", "engine": "dark", "firing": "warm", "gearbox": "light", "clutch": "light",
           "title": "warm", "fade": "dark", "edge": "light"}
    written = []
    for (W, H) in ((1280, 720), (640, 360)):
        cache = {}
        for name, sc in S.items():
            kinds = [bgs[name]] + (["light"] if name == "shift" else []) + (["dark"] if name == "recap" else [])
            for kind in kinds:
                if bg_path:  # a real render as the background for every scenario
                    kind = "real"
                if kind not in cache:
                    cache[kind] = (Image.open(bg_path).convert("RGB").resize((W, H), Image.LANCZOS)
                                   if kind == "real" else background(kind, W, H))
                im = ov.render_overlay(cache[kind].copy(), sc["labels"], sc["hud"])
                p = os.path.join(out, f"{name}_{kind}_{W}.png")
                im.save(p)
                written.append(p)
                if guides:
                    gp = os.path.join(out, f"guides_{name}_{kind}_{W}.png")
                    draw_guides(im).save(gp)
                    written.append(gp)
    # one contact sheet per resolution (3 columns) for a quick review
    for W in (1280, 640):
        tiles = [Image.open(f) for f in written if f.endswith(f"_{W}.png") and "/guides_" not in f]
        if tiles:
            cols, tw, th = 3, 640, 360
            rows = (len(tiles) + cols - 1) // cols
            sheet = Image.new("RGB", (cols * tw, rows * th), (0, 0, 0))
            for i, im in enumerate(tiles):
                sheet.paste(im.resize((tw, th), Image.LANCZOS), ((i % cols) * tw, (i // cols) * th))
            p = os.path.join(out, f"sheet_{W}.png")
            sheet.save(p)
            written.append(p)
    return written


def bench(out, n=40):
    """Single-core timing for a busy 1280x720 frame (labels + 8 widgets)."""
    os.makedirs(out, exist_ok=True)
    S = scenarios()
    bg = background("dark", 1280, 720)
    raw = os.path.join(out, "bench_raw.png")
    bg.save(raw, compress_level=1)
    labels = S["shift"]["labels"]
    hud = S["shift"]["hud"] + [dict(type="rpm", value=3000), dict(type="speed", value_kmh=24)]
    # warm caches (fonts, shadows) as a worker would after its first frame
    ov.compose_frame(raw, os.path.join(out, "bench_out.png"), labels, hud)
    t_draw = []
    t_full = []
    for k in range(n):
        lab = [dict(L, ax=L["ax"] + 0.001 * k) for L in labels]
        h2 = [dict(w, value=3000 + 7 * k) if w["type"] == "rpm" else w for w in hud]
        im = bg.copy()
        t0 = time.perf_counter()
        ov.render_overlay(im, lab, h2)
        t_draw.append(time.perf_counter() - t0)
        t0 = time.perf_counter()
        ov.compose_frame(raw, os.path.join(out, "bench_out.png"), lab, h2)
        t_full.append(time.perf_counter() - t0)
    res = dict(draw_ms=round(1000 * float(np.median(t_draw)), 1),
               compose_frame_ms=round(1000 * float(np.median(t_full)), 1),
               out_kb=round(os.path.getsize(os.path.join(out, "bench_out.png")) / 1024, 1))
    print("[bench] 1280x720, 4 labels + 10 widgets:", res)
    return res


def seq_test(out, n=24, workers=2):
    """compose_sequence end to end: JSON -> frames, then resume (skip all)."""
    d = os.path.join(out, "seq")
    if os.path.isdir(d):
        shutil.rmtree(d)
    raw, comp = os.path.join(d, "raw"), os.path.join(d, "comp")
    os.makedirs(raw)
    bg = background("dark", 640, 360)
    lab, hud = {}, {}
    for f in range(1, n + 1):
        bg.save(os.path.join(raw, f"{f:05d}.png"), compress_level=1)
        t = (f - 1) / (n - 1)
        lab[str(f)] = [dict(id="p", text="Piston", ax=0.40 + 0.05 * math.sin(6 * t), ay=0.5, dx=-0.08, dy=-0.06,
                            alpha=min(1.0, f / 6), style="part", occluded=f > n * 0.7)]
        hud[str(f)] = [dict(type="rpm", alpha=1.0, value=850 + 2000 * t),
                       dict(type="stroke_strip", alpha=1.0, active=int(4 * t) % 4, progress=(4 * t) % 1.0),
                       dict(type="hpattern", alpha=1.0, x=-1.0, y=-1 + 2 * t)]
    lp, hp = os.path.join(d, "labels.json"), os.path.join(d, "hud.json")
    json.dump(dict(fps=24, width=640, height=360, frames=lab), open(lp, "w"))
    json.dump(dict(fps=24, frames=hud), open(hp, "w"))
    r1 = ov.compose_sequence(raw, comp, lp, hp, workers=workers)
    os.remove(os.path.join(comp, "00005.png"))
    r2 = ov.compose_sequence(raw, comp, lp, hp, workers=workers)
    leftovers = [x for x in os.listdir(comp) if x.startswith(".tmp_")]
    ok = r1["done"] == n and r2["done"] == 1 and r2["skipped"] == n - 1 and not leftovers
    print(f"[seq] first {r1}, resume {r2}, tmp leftovers {leftovers} -> {'OK' if ok else 'FAIL'}")
    # contact strip of a few frames
    sel = [1, 6, 12, 18, 24]
    sheet = Image.new("RGB", (640 * len(sel) // 2, 180))
    for i, f in enumerate(sel):
        sheet.paste(Image.open(os.path.join(comp, f"{f:05d}.png")).resize((320, 180)), (320 * i, 0))
    sheet.save(os.path.join(out, "seq_strip.png"))
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--guides", action="store_true")
    ap.add_argument("--bench", action="store_true")
    ap.add_argument("--seq", action="store_true")
    ap.add_argument("--bg", default=None, help="use this image as the background of every scenario")
    a = ap.parse_args()
    files = run_demo(a.out, guides=a.guides, bg_path=a.bg)
    print(f"wrote {len(files)} images to {a.out}")
    if a.bench:
        bench(a.out)
    if a.seq:
        seq_test(a.out)


if __name__ == "__main__":
    main()
