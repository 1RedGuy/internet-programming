"""2D overlay compositor: part labels (leader lines) + HUD widgets drawn with
Pillow onto rendered frames.  No bpy.

Everything is sized relative to the frame height (unit ``u = H / 100``), so a
640x360 preview and a 1280x720 final look identical.  Shapes are drawn 2x
supersampled and box-filtered down (anti-aliased lines, arcs and rounded
corners); text is drawn at 1x with FreeType anti-aliasing and sub-pixel
positioning (crisper than downsampled text at small sizes; letter-spaced caps
captions are composed 4x supersampled for even spacing).  Each widget is
drawn into its own small tile, so the cost scales with the overlay area, not
the frame.

Input contract
==============
labels.json  (written by ``carviz.labels.Labels.write``)::

    {"fps": 24, "width": W, "height": H,
     "frames": {"<frame>": [label, ...], ...}}

    label = {"id": "piston",          # stable id (unused for drawing)
             "text": "Piston",        # may contain "\\n" for two lines
             "ax": 0.43, "ay": 0.52,  # anchor, normalised frame coords, origin top-left
             "dx": 0.08, "dy": -0.06, # label offset from the anchor (fraction of W, of H),
                                      # fixed per label so labels never jitter
             "alpha": 1.0,            # fade 0..1                       (default 1)
             "style": "part",         # "part" | "emph" | "dim"         (default "part")
             "occluded": false}       # anchor hidden behind geometry   (default false)

  Placement: if |dx*W| >= 2u the label sits beside the anchor: the plate's
  near vertical edge is at x = ax*W + dx*W (left edge when dx > 0, right edge
  when dx < 0), vertically centred on y = ay*H + dy*H.  Otherwise the label
  sits above (dy < 0) / below (dy > 0) the anchor, centred on x.  dx = dy = 0
  centres the plate on the anchor (no leader).  Plates are clamped inside the
  frame (2.5u margin) and never enter the subtitle band (plate bottom <=
  0.81 H; the anchor dot itself may lie lower).  The leader is computed from
  the final plate: anchor beside the plate -> diagonal to a short horizontal
  elbow into the facing side; anchor above/below -> straight to the facing
  edge.  Anchors outside the frame fade the label out; ``occluded`` draws it
  at 62 % alpha, grey text, dashed leader and a hollow dot (hidden-line
  convention).  Plates blur what is behind them slightly (frosted glass) so
  text stays legible on busy renders.
  Styles: "part" white text on a dark plate; "emph" accent dot/leader/border
  (the part in the power path); "dim" smaller-contrast secondary label.

hud.json  (written by ``carviz.labels.Hud.write``)::

    {"fps": 24, "frames": {"<frame>": [widget, ...], ...}}
    widget = {"type": "<type>", "alpha": 1.0, ...fields...,
              "pos": [x, y],      # optional override, normalised frame coords
              "anchor": "tl"}     # optional: which point of the widget box `pos`
                                  # refers to: tl tc tr ml c mr bl bc br (default tl;
                                  # title_card default c)

  Widgets are drawn in list order (later on top), after the labels; ``fade``
  is always applied last, over everything.

Widget types (fields; defaults in brackets)
-------------------------------------------
  title_card     title, subtitle[""]           large centred title, soft backdrop
  section_title  number, title                 "2  The engine" chapter heading
  gear           value  "N"|"1".."5"|"R"|int (0/None = N, -1 = R)   big digital gear indicator
  rpm            value, max[7000], redline[6800], label["Engine"]   arc tachometer
  speed          value_kmh (or value)          numeric km/h
  pedal          value 0..1 (1 = floored), label["Clutch pedal"],
                 bite[[spec.CLUTCH_BITE_LO, spec.CLUTCH_BITE_HI]] (null = hide)
  hpattern       x -1..1 (plane), y -1..1 (+1 = top row 1/3/5)   5-speed gate + lever dot
  slowmo         factor                        "SLOW MOTION x150" / "REAL TIME" (factor < 1.05);
                 a rate < 0.95 (Track.slowmo, e.g. 1/150) is inverted automatically;
                 shown as x2.5 (< 10), x63 (< 100), x150/x200 (nearest 10 above 100)
  status         text, kind ok|warn|info   badge with coloured dot (green / accent / blue);
                 kind defaults from the text: SLIPPING/SYNCHRONIZING -> warn,
                 DISENGAGED/ENGINE OFF -> info, ENGAGED/LOCKED -> ok
  readouts       rows [[label, value(, highlight)], ...], title[None]
                 value: number or string; "3000 rpm" -> number + small grey unit;
                 a truthy 3rd element draws the value in the accent colour
  stroke_strip   active 0..3 (INTAKE COMPRESSION POWER EXHAUST, = kin.stroke_of()[0]) or the
                 stroke name, progress 0..1, cylinder[1]
  firing_ticker  active 0..3 = index into spec.FIRING_ORDER (1 3 4 2);
                 or cylinder 1..4; None/-1 = nothing highlighted
  step_card      number, text, total[None] (shows "STEP n / total" when given)
  ratio_card     ratio_text, caption[""]
  fade           color["black"] ("black"|"white"|"#rrggbb"|[r,g,b]), alpha

Default layout (16:9, u = H/100, 5u margins)::

    +---------------------------------------------------------------------+
    | section_title       step_card | ratio_card |          slowmo badge  |
    | readouts            stroke_strip | firing_ticker        status badge |
    |                     (top-centre slot, one at a time)   speed | gear  |
    |                                                       clutch pedal  |
    |                         title_card (centre)                         |
    | hpattern                                                 rpm dial   |
    | (bottom-left, ends at 0.78 H)              (bottom-right, 0.78 H)   |
    |          . . . subtitles: bottom-centre 18 %, kept clear . . .       |
    +---------------------------------------------------------------------+

  step_card, ratio_card, stroke_strip and firing_ticker share the top-centre
  slot (they are never needed together; use ``pos`` if they are).

API
===
  render_overlay(im, labels, hud) -> PIL.Image      draw onto an RGB image (in place)
  compose_frame(raw_path, out_path, labels, hud, size=None)
  compose_sequence(raw_dir, out_dir, labels_json, hud_json, frames=None,
                   workers=2, overwrite=False) -> dict
  CLI: python3 -m carviz.overlay <raw_dir> <out_dir> <labels.json> <hud.json>
                   [--frames 1-240|1,5,9] [--workers 2] [--overwrite]
"""
from __future__ import annotations

import argparse
import json
import math
import multiprocessing as mp
import os
import re
import sys
import time
from functools import lru_cache

from PIL import Image, ImageDraw, ImageFilter, ImageFont, features as _pil_features

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FONT_DIR = os.path.join(ROOT, "assets", "fonts")

try:  # spec has no bpy dependency
    from . import spec as _S
    _BITE = (float(_S.CLUTCH_BITE_LO), float(_S.CLUTCH_BITE_HI))
    _FIRING = tuple(_S.FIRING_ORDER)
except Exception:  # pragma: no cover - allows standalone use of this file
    _BITE = (0.22, 0.50)
    _FIRING = (1, 3, 4, 2)

# ---------------------------------------------------------------------------
# Style
# ---------------------------------------------------------------------------
SS = 2                          # supersampling factor for shapes
ACCENT = (255, 138, 31)         # warm accent = 3D power-path glow
ACCENT_INK = (30, 17, 4)        # dark text on accent fills
WHITE = (247, 248, 250)
TEXT2 = (214, 219, 225)         # secondary text
CAPTION = (156, 164, 175)       # small-caps captions
MUTED = (112, 119, 130)         # inactive items
PLATE = (13, 15, 19)
PLATE_A = 0.72
FROST_U = 0.9                   # backdrop blur radius under plates (u); 0 = off
BORDER = (255, 255, 255, 30)    # hairline plate border
TRACK = (255, 255, 255, 36)     # bar tracks, gate slots
RED = (236, 76, 70)
GREEN = (74, 208, 140)
BLUE = (102, 178, 255)

MARGIN_U = 5.0                  # HUD margin (all sides), in u
LABEL_MARGIN_U = 2.5            # labels keep this far from the frame edge
SAFE_Y = 0.82                   # subtitle band starts here (fraction of H)
BOTTOM_U = 78.0                 # bottom edge of the bottom-corner widgets (u)
RADIUS_U = 1.1                  # plate corner radius
SHADOW_A = 0.34
SHADOW_BLUR_U = 0.9
SHADOW_DY_U = 0.35

_HAS_RAQM = _pil_features.check("raqm")
TNUM = ("tnum",) if _HAS_RAQM else None

_WEIGHTS = {"regular": "Inter-Regular.otf", "medium": "Inter-Medium.otf",
            "semibold": "Inter-SemiBold.otf", "bold": "Inter-Bold.otf"}


@lru_cache(maxsize=512)
def _font(weight, px):
    path = os.path.join(FONT_DIR, _WEIGHTS[weight])
    return ImageFont.truetype(path, max(4.0, px))


@lru_cache(maxsize=64)
def _font_file(path, px):
    return ImageFont.truetype(path, px)


@lru_cache(maxsize=512)
def _cap(font):
    """Cap height (px) of a font object."""
    return -font.getbbox("H", anchor="ls")[1]


@lru_cache(maxsize=256)
def _alpha_lut(a8):
    return [int(v * a8 / 255.0 + 0.5) for v in range(256)]


def _clamp(x, lo=0.0, hi=1.0):
    return lo if x < lo else hi if x > hi else x


def _smooth(e0, e1, x):
    t = _clamp((x - e0) / (e1 - e0))
    return t * t * (3 - 2 * t)


def _mix(c0, c1, t):
    return tuple(int(round(a + (b - a) * t)) for a, b in zip(c0, c1))


def _rgba(c, a=1.0):
    return (c[0], c[1], c[2], int(round(255 * a)))


def _color(c):
    if isinstance(c, (list, tuple)):
        return tuple(int(v) for v in c[:3])
    c = str(c).strip().lower()
    named = {"black": (0, 0, 0), "white": (255, 255, 255), "accent": ACCENT}
    if c in named:
        return named[c]
    if c.startswith("#") and len(c) == 7:
        return tuple(int(c[i:i + 2], 16) for i in (1, 3, 5))
    return (0, 0, 0)


# ---------------------------------------------------------------------------
# Frame context and drawing tiles
# ---------------------------------------------------------------------------
_ANCHORS = {"tl": (0, 0), "tc": (.5, 0), "tr": (1, 0), "ml": (0, .5), "c": (.5, .5), "mr": (1, .5),
            "bl": (0, 1), "bc": (.5, 1), "br": (1, 1)}


class _Ctx:
    """Per-frame drawing context: the RGB frame and the layout unit."""

    def __init__(self, im):
        self.im = im
        self.W, self.H = im.size
        self.u = self.H / 100.0

    def font(self, weight, size_u):
        return _font(weight, round(size_u * self.u * 4) / 4.0)

    def place(self, wdg, w, h, x, y, anchor="tl"):
        """Top-left of a w x h box whose `anchor` point is at (x, y) px.  A
        widget's "pos" (normalised) + optional "anchor" override the default."""
        if wdg.get("pos") is not None:
            px, py = wdg["pos"][:2]
            x, y = float(px) * self.W, float(py) * self.H
            anchor = wdg.get("anchor") or ("c" if wdg.get("type") == "title_card" else "tl")
        a = _ANCHORS.get(anchor, (0, 0))
        return x - a[0] * w, y - a[1] * h


@lru_cache(maxsize=256)
def _shadow_img(w2, h2, r2, blur2, a8):
    """Blurred black rounded-rect shadow (RGBA, SS resolution) with a margin
    of 3*blur2 around the w2 x h2 box."""
    m = int(math.ceil(3 * blur2)) + 1
    mask = Image.new("L", (w2 + 2 * m, h2 + 2 * m), 0)
    ImageDraw.Draw(mask).rounded_rectangle([m, m, m + w2 - 1, m + h2 - 1], radius=r2, fill=a8)
    if blur2 > 0:
        mask = mask.filter(ImageFilter.GaussianBlur(blur2))
    img = Image.new("RGBA", mask.size, (0, 0, 0, 0))
    img.putalpha(mask)
    return img, m


@lru_cache(maxsize=4096)
def _tw(font, s, tracking=0.0, features=None):
    """Advance width of a string (px) incl. tracking (features: tuple)."""
    if not s:
        return 0.0
    if tracking and len(s) > 1:  # same metrics as the 4x tracked-text renderer
        f4 = _font_file(font.path, font.size * 4)
        return (sum(f4.getlength(ch) for ch in s) + tracking * 4 * (len(s) - 1)) / 4.0
    if features and _HAS_RAQM:
        return font.getlength(s, features=list(features))
    return font.getlength(s)


@lru_cache(maxsize=4096)
def _text_mask(s, font, anchor, fx, fy, features, tracking):
    """Rendered 1x coverage mask of a string (cached).  Returns (mask core,
    (dx, dy)) where dx,dy is the mask origin relative to the integer anchor
    position; fx, fy = sub-pixel start offset (quantised by the caller)."""
    if tracking and len(s) > 1:
        # Letter-spaced caps: per-glyph placement at 1x shows uneven gaps from
        # hinting at small sizes, so compose the glyphs 4x supersampled and
        # box-reduce (even spacing).  Vertical anchor is always the baseline.
        k = 4
        f4 = _font_file(font.path, font.size * k)
        adv = [f4.getlength(ch) for ch in s]
        total = (sum(adv) + tracking * k * (len(s) - 1)) / k          # 1x px
        asc, desc = f4.getmetrics()
        hx = {"l": 0.0, "m": total / 2.0, "r": total}.get(anchor[0], 0.0)
        left = fx - hx                       # text start relative to the integer anchor x
        li = math.floor(left) - 2
        ti = math.floor(fy - asc / k) - 2
        x4, b4 = (left - li) * k, (fy - ti) * k
        w4 = int(math.ceil((x4 + total * k + 2 * k) / k)) * k
        h4 = int(math.ceil((b4 + desc + 2 * k) / k)) * k
        img = Image.new("L", (w4, h4), 0)
        d = ImageDraw.Draw(img)
        x = x4
        for ch, a in zip(s, adv):
            d.text((x, b4), ch, font=f4, fill=255, anchor="ls")
            x += a + tracking * k
        return img.reduce(k).im, (li, ti)
    kw = {"features": list(features)} if features and _HAS_RAQM else {}
    mask, off = font.getmask2(s, "L", anchor=anchor, start=(fx, fy), **kw)
    return mask, off


def _q(v):
    """Split a coordinate into integer part and a sub-pixel offset quantised
    to 1/4 px (keeps the text-mask cache small)."""
    i = math.floor(v)
    f = round((v - i) * 4) / 4.0
    if f >= 1.0:
        i, f = i + 1, 0.0
    return int(i), f


class _Tile:
    """A small RGBA surface covering the frame-pixel box (x0,y0)-(x1,y1) plus
    `pad`.  Shapes are drawn at SS x; text is queued and drawn at 1x after the
    box-filter reduction.  All coordinates are frame pixels (floats)."""

    def __init__(self, ctx, x0, y0, x1, y1, pad=None):
        self.ctx = ctx
        pad = 3.6 * ctx.u if pad is None else pad
        self.ox = int(math.floor(x0 - pad))
        self.oy = int(math.floor(y0 - pad))
        self.w = max(1, int(math.ceil(x1 + pad)) - self.ox)
        self.h = max(1, int(math.ceil(y1 + pad)) - self.oy)
        self.im = Image.new("RGBA", (self.w * SS, self.h * SS), (0, 0, 0, 0))
        self.dr = ImageDraw.Draw(self.im)
        self.texts = []
        self.frost = []
        self.dirty = False

    # coordinate transforms (frame px -> tile SS px)
    def X(self, x):
        return (x - self.ox) * SS

    def Y(self, y):
        return (y - self.oy) * SS

    def _shape(self, colors, bbox, fn):
        """Run fn(draw, ox, oy) (coords offset by -ox,-oy).  ImageDraw
        overwrites RGBA pixels, so a translucent colour over existing pixels
        is drawn on a bbox-sized layer and alpha-composited instead."""
        if self.dirty and any(c is not None and len(c) == 4 and c[3] < 255 for c in colors):
            x0 = max(0, int(math.floor(bbox[0])) - 2)
            y0 = max(0, int(math.floor(bbox[1])) - 2)
            x1 = min(self.im.width, int(math.ceil(bbox[2])) + 3)
            y1 = min(self.im.height, int(math.ceil(bbox[3])) + 3)
            if x1 > x0 and y1 > y0:
                layer = Image.new("RGBA", (x1 - x0, y1 - y0), (0, 0, 0, 0))
                fn(ImageDraw.Draw(layer), x0, y0)
                self.im.alpha_composite(layer, dest=(x0, y0))
        else:
            fn(self.dr, 0, 0)
        self.dirty = True

    def _paste_rgba(self, src, X, Y):
        """alpha_composite src at integer SS coords, cropping at tile edges."""
        sx0, sy0 = max(0, -X), max(0, -Y)
        sx1, sy1 = min(src.width, self.im.width - X), min(src.height, self.im.height - Y)
        if sx1 <= sx0 or sy1 <= sy0:
            return
        if (sx0, sy0, sx1, sy1) != (0, 0, src.width, src.height):
            src = src.crop((sx0, sy0, sx1, sy1))
        self.im.alpha_composite(src, dest=(X + sx0, Y + sy0))
        self.dirty = True

    # ---- shapes ---------------------------------------------------------
    def _box(self, x, y, w, h):
        X0, Y0 = round(self.X(x)), round(self.Y(y))
        X1, Y1 = round(self.X(x + w)), round(self.Y(y + h))
        return [X0, Y0, max(X0, X1 - 1), max(Y0, Y1 - 1)]

    def rrect(self, x, y, w, h, r, fill=None, outline=None, ow=0.0):
        b = self._box(x, y, w, h)
        R = max(0, int(round(min(r * SS, (b[2] - b[0] + 1) / 2.0, (b[3] - b[1] + 1) / 2.0))))
        width = max(1, int(round(ow * SS))) if outline is not None else 0

        def fn(dr, ox, oy):
            dr.rounded_rectangle([b[0] - ox, b[1] - oy, b[2] - ox, b[3] - oy], radius=R, fill=fill,
                                 outline=outline, width=width)
        self._shape([fill, outline], b, fn)

    def circle(self, cx, cy, r, fill=None, outline=None, ow=0.0):
        b = [round(self.X(cx - r)), round(self.Y(cy - r)), round(self.X(cx + r)) - 1, round(self.Y(cy + r)) - 1]
        width = max(1, int(round(ow * SS))) if outline is not None else 0

        def fn(dr, ox, oy):
            dr.ellipse([b[0] - ox, b[1] - oy, b[2] - ox, b[3] - oy], fill=fill, outline=outline, width=width)
        self._shape([fill, outline], b, fn)

    def line(self, pts, width, fill, caps=True):
        self.lines([pts], width, fill, caps)

    def lines(self, polylines, width, fill, caps=True):
        """Several polylines in one layer (same colour/width)."""
        W = max(1, int(round(width * SS)))
        P = [[(self.X(x), self.Y(y)) for x, y in pl] for pl in polylines if len(pl) >= 2]
        if not P:
            return
        xs = [p[0] for pl in P for p in pl]
        ys = [p[1] for pl in P for p in pl]
        bb = [min(xs) - W, min(ys) - W, max(xs) + W, max(ys) + W]
        r = W / 2.0

        def fn(dr, ox, oy):
            for pl in P:
                q = [(x - ox, y - oy) for x, y in pl]
                dr.line(q, fill=fill, width=W, joint="curve" if len(q) > 2 else None)
                if caps and W > 2:
                    for X, Y in (q[0], q[-1]):
                        dr.ellipse([X - r, Y - r, X + r - 1, Y + r - 1], fill=fill)
        self._shape([fill], bb, fn)

    def dashed(self, pts, width, fill, dash, gap):
        """Polyline drawn as dashes (lengths in frame px)."""
        segs = []
        period = max(1e-3, dash + gap)
        d0 = 0.0  # distance along the polyline at the segment start
        for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
            L = math.hypot(x1 - x0, y1 - y0)
            if L < 1e-6:
                continue
            ux, uy = (x1 - x0) / L, (y1 - y0) / L
            k = math.floor(d0 / period)
            while k * period < d0 + L:
                a = max(k * period, d0) - d0
                b = min(k * period + dash, d0 + L) - d0
                if b > a:
                    segs.append([(x0 + ux * a, y0 + uy * a), (x0 + ux * b, y0 + uy * b)])
                k += 1
            d0 += L
        self.lines(segs, width, fill, caps=False)

    def arc(self, cx, cy, r, a0, a1, width, fill, caps=True):
        """Arc centred on radius r, angles in degrees clockwise from +x (screen)."""
        if a1 - a0 <= 0.05:
            return
        W = max(1, int(round(width * SS)))
        R = r * SS + W / 2.0
        CX, CY = self.X(cx), self.Y(cy)
        rr = W / 2.0
        ends = [(CX + r * SS * math.cos(math.radians(a)) - 0.5, CY + r * SS * math.sin(math.radians(a)) - 0.5)
                for a in (a0, a1)]

        def fn(dr, ox, oy):
            dr.arc([CX - R - ox, CY - R - oy, CX + R - 1 - ox, CY + R - 1 - oy], a0, a1, fill=fill, width=W)
            if caps:
                for X, Y in ends:
                    dr.ellipse([X - rr - ox, Y - rr - oy, X + rr - ox, Y + rr - oy], fill=fill)
        self._shape([fill], [CX - R, CY - R, CX + R, CY + R], fn)

    def polygon(self, pts, fill):
        P = [(self.X(x), self.Y(y)) for x, y in pts]
        bb = [min(p[0] for p in P), min(p[1] for p in P), max(p[0] for p in P), max(p[1] for p in P)]

        def fn(dr, ox, oy):
            dr.polygon([(x - ox, y - oy) for x, y in P], fill=fill)
        self._shape([fill], bb, fn)

    def shadow(self, x, y, w, h, r, dy=None, blur=None, alpha=SHADOW_A):
        u = self.ctx.u
        dy = SHADOW_DY_U * u if dy is None else dy
        blur = SHADOW_BLUR_U * u if blur is None else blur
        img, m = _shadow_img(int(round(w * SS)), int(round(h * SS)), int(round(r * SS)),
                             int(round(blur * SS)), int(round(255 * alpha)))
        self._paste_rgba(img, int(round(self.X(x))) - m, int(round(self.Y(y + dy))) - m)

    def plate(self, x, y, w, h, r=None, alpha=PLATE_A, shadow=True, border=True, color=PLATE, frost=True):
        """Translucent dark plate: soft shadow, frosted (blurred) backdrop,
        hairline border."""
        u = self.ctx.u
        r = RADIUS_U * u if r is None else r
        if frost and FROST_U > 0:
            self.frost.append((x, y, w, h, r))
        if shadow:
            self.shadow(x, y, w, h, r)
        # the plate overwrites the shadow inside its own area: consistent plate tone
        b = self._box(x, y, w, h)
        R = int(round(min(r * SS, (b[2] - b[0] + 1) / 2.0, (b[3] - b[1] + 1) / 2.0)))
        self.dr.rounded_rectangle(b, radius=R, fill=_rgba(color, alpha))
        self.dirty = True
        if border:
            self.rrect(x, y, w, h, r, outline=BORDER if border is True else border, ow=max(0.5, 0.11 * u))

    # ---- text (queued, drawn at 1x) --------------------------------------
    def text(self, x, y, s, font, fill, anchor="ls", features=None, tracking=0.0):
        """Draw `s` with its anchor point at (x, y) (baseline anchors 'ls',
        'ms', 'rs' recommended).  `tracking` = extra px between characters."""
        if s:
            feats = tuple(features) if (features and _HAS_RAQM and not tracking) else None
            self.texts.append((x, y, s, font, fill, anchor, feats, float(tracking or 0.0)))

    def commit(self, alpha=1.0):
        if alpha <= 0.002:
            return
        im = self.im.reduce(SS) if SS > 1 else self.im
        for x, y, w, h, r in self.frost:
            _frost(self.ctx, x, y, w, h, r, alpha)
        if self.texts:
            d = ImageDraw.Draw(im)
            for x, y, s, font, fill, anchor, feats, tr in self.texts:
                ix, fx = _q(x - self.ox)
                iy, fy = _q(y - self.oy)
                mask, (dx, dy) = _text_mask(s, font, anchor, fx, fy, feats, tr)
                ink = d._getink(_rgba(fill) if len(fill) == 3 else fill)[0]
                d.draw.draw_bitmap((ix + dx, iy + dy), mask, ink)
        if alpha < 0.998:
            a = im.getchannel("A").point(_alpha_lut(int(round(alpha * 255))))
            self.ctx.im.paste(im, (self.ox, self.oy), a)
        else:
            self.ctx.im.paste(im, (self.ox, self.oy), im)


@lru_cache(maxsize=256)
def _rrect_mask(w, h, r):
    """Anti-aliased rounded-rect coverage mask (L, 1x), cached by size."""
    m = Image.new("L", (w * SS, h * SS), 0)
    ImageDraw.Draw(m).rounded_rectangle([0, 0, w * SS - 1, h * SS - 1], radius=r * SS, fill=255)
    return m.reduce(SS) if SS > 1 else m


def _frost(ctx, x, y, w, h, r, alpha):
    """Blur the frame under a plate ("frosted glass"): keeps text legible on
    busy backgrounds.  Done in place on the frame before the tile is pasted."""
    W, H = ctx.W, ctx.H
    X0, Y0 = int(round(x)), int(round(y))
    X1, Y1 = int(round(x + w)), int(round(y + h))
    if X1 <= 0 or Y1 <= 0 or X0 >= W or Y0 >= H or X1 - X0 < 2 or Y1 - Y0 < 2:
        return
    rad = FROST_U * ctx.u
    b = int(math.ceil(2 * rad)) + 1
    cx0, cy0 = max(0, X0 - b), max(0, Y0 - b)
    cx1, cy1 = min(W, X1 + b), min(H, Y1 + b)
    reg = ctx.im.crop((cx0, cy0, cx1, cy1)).filter(ImageFilter.GaussianBlur(rad))
    reg = reg.crop((X0 - cx0, Y0 - cy0, X1 - cx0, Y1 - cy0))
    mask = _rrect_mask(X1 - X0, Y1 - Y0, int(round(min(r, (X1 - X0) / 2, (Y1 - Y0) / 2))))
    if alpha < 0.998:
        mask = mask.point(_alpha_lut(int(round(alpha * 255))))
    ctx.im.paste(reg, (X0, Y0), mask)


def _caps(ctx, size_u=2.05, weight="semibold"):
    """Small-caps caption font + tracking (px)."""
    f = ctx.font(weight, size_u)
    return f, 0.07 * size_u * ctx.u


# ---------------------------------------------------------------------------
# Labels
# ---------------------------------------------------------------------------
LABEL_FONT_U = 2.65
LABEL_H_U = 4.5
LABEL_PAD_U = 1.2
OCCLUDED_ALPHA = 0.62   # occluded labels: dimmer, dashed leader, hollow dot


def _draw_label(ctx, L):
    u, W, H = ctx.u, ctx.W, ctx.H
    text = str(L.get("text", ""))
    if not text:
        return
    style = L.get("style", "part") or "part"
    occl = bool(L.get("occluded", False))
    ax, ay = float(L.get("ax", 0.5)), float(L.get("ay", 0.5))
    dx, dy = float(L.get("dx", 0.0) or 0.0), float(L.get("dy", 0.0) or 0.0)
    if not all(math.isfinite(v) for v in (ax, ay, dx, dy)):
        return
    alpha = _clamp(float(L.get("alpha", 1.0) if L.get("alpha") is not None else 1.0))
    # fade out as the anchor leaves the frame
    edge = min(ax, 1 - ax, ay, 1 - ay)
    alpha *= _smooth(-0.02, 0.0, edge) if edge < 0 else 1.0
    if occl:
        alpha *= OCCLUDED_ALPHA
    if alpha <= 0.004:
        return
    dim = style == "dim"
    emph = style == "emph"

    lines = text.split("\n")
    f = ctx.font("semibold" if not dim else "medium", LABEL_FONT_U * (0.92 if dim else 1.0))
    cap = _cap(f)
    lh = 1.45 * f.size
    tw = max(_tw(f, s) for s in lines)
    pad = LABEL_PAD_U * u
    bw = tw + 2 * pad
    bh = LABEL_H_U * u * (0.92 if dim else 1.0) + (len(lines) - 1) * lh

    A = (ax * W, ay * H)
    C = (A[0] + dx * W, A[1] + dy * H)
    side = abs(dx * W) >= 2.0 * u
    if side:
        sgn = 1.0 if dx > 0 else -1.0
        bx = C[0] if sgn > 0 else C[0] - bw
        by = C[1] - bh / 2
    elif abs(dy * H) > 0.5 * u:
        sgn = 1.0 if dy > 0 else -1.0
        bx = C[0] - bw / 2
        by = C[1] if sgn > 0 else C[1] - bh
    else:
        bx, by = C[0] - bw / 2, C[1] - bh / 2
    # keep the plate in frame and out of the subtitle band
    m = LABEL_MARGIN_U * u
    bx = _clamp(bx, m, W - m - bw)
    by = _clamp(by, m, SAFE_Y * H - 1.0 * u - bh)

    # leader geometry (from the final, clamped plate): attach to the plate side
    # facing the anchor -- left/right edge with a short horizontal elbow, or
    # top/bottom edge with a straight leader when the anchor is above/below.
    lead = []
    if side or abs(dy * H) > 0.5 * u:
        if A[0] < bx or A[0] > bx + bw:
            sgn = 1.0 if A[0] < bx else -1.0
            ex = bx if sgn > 0 else bx + bw
            ey = by + bh / 2
            elbow = min(2.6 * u, abs(ex - A[0]) * 0.45)
            K = (ex - sgn * elbow, ey)
            lead = [A, K, (ex, ey)] if math.hypot(K[0] - A[0], K[1] - A[1]) > 0.6 * u else [A, (ex, ey)]
        elif A[1] < by or A[1] > by + bh:
            ex = _clamp(A[0], bx + 1.2 * u, bx + bw - 1.2 * u)
            ey = by if A[1] < by else by + bh
            lead = [A, (ex, ey)]

    xs = [bx, bx + bw] + [p[0] for p in lead]
    ys = [by, by + bh] + [p[1] for p in lead]
    t = _Tile(ctx, min(xs), min(ys), max(xs), max(ys))

    accent_line = emph
    lw = max(1.0, 0.2 * u)
    line_c = _rgba(ACCENT) if accent_line else _rgba(WHITE, 0.92 if not dim else 0.7)
    # leader with a soft dark under-stroke for contrast on light backgrounds
    if lead:
        if occl:
            t.dashed(lead, lw + 1.6, (0, 0, 0, 70), 1.1 * u, 0.7 * u)
            t.dashed(lead, lw, line_c, 1.1 * u, 0.7 * u)
        else:
            t.line(lead, lw + 1.6, (0, 0, 0, 70))
            t.line(lead, lw, line_c)
    # plate
    t.plate(bx, by, bw, bh, r=0.95 * u, alpha=PLATE_A if not dim else 0.6, shadow=not dim,
            border=_rgba(ACCENT, 0.9) if emph else True)
    # anchor dot (drawn over the leader)
    if lead:
        dr = 0.62 * u
        dot_c = ACCENT if emph else WHITE
        if occl:
            t.circle(A[0], A[1], dr + 0.22 * u, fill=(0, 0, 0, 90))
            t.circle(A[0], A[1], dr, outline=_rgba(dot_c), ow=max(1.0, 0.22 * u))
        else:
            t.circle(A[0], A[1], dr + 0.26 * u, fill=(0, 0, 0, 110))
            t.circle(A[0], A[1], dr, fill=_rgba(dot_c))
    # text (cap-centred)
    tc = TEXT2 if (dim or occl) else WHITE
    y0 = by + bh / 2 - (len(lines) - 1) * lh / 2 + cap / 2
    for i, s in enumerate(lines):
        t.text(bx + pad, y0 + i * lh, s, f, tc, anchor="ls")
    t.commit(alpha * (0.85 if dim else 1.0))


# ---------------------------------------------------------------------------
# HUD widgets
# ---------------------------------------------------------------------------


def _w_title_card(ctx, w, alpha):
    u, W, H = ctx.u, ctx.W, ctx.H
    title = str(w.get("title", ""))
    sub = str(w.get("subtitle", "") or "")
    ft = ctx.font("bold", 8.4)
    fs = ctx.font("medium", 3.3)
    tw = _tw(ft, title)
    sw = _tw(fs, sub)
    capt = _cap(ft)
    gap_rule = 2.6 * u
    block_h = capt + (gap_rule * 2 + 0.36 * u + _cap(fs) if sub else 0)
    cx, cy = ctx.place(w, 0, 0, 0.5 * W, 0.40 * H, "c")
    rise = (1.0 - alpha) * 1.2 * u          # subtle drift while fading
    top = cy - block_h / 2 + rise
    # soft elliptical backdrop for legibility on any background
    bd = _backdrop(W, H, int(round(max(tw, sw) + 24 * u)), int(round(block_h + 26 * u)))
    bx, by = int(round(cx - bd.width / 2)), int(round(cy - bd.height / 2 + rise))
    a = bd.getchannel("A")
    if alpha < 0.998:
        a = a.point(_alpha_lut(int(round(alpha * 255))))
    ctx.im.paste(bd, (bx, by), a)
    # title with a soft shadow
    base = top + capt
    sh = _text_shadow(title, ft.path, ft.size, int(round(0.6 * u)))
    shimg, (ox, oy) = sh
    sa = shimg.point(_alpha_lut(int(round(alpha * 255 * 0.55))))
    ctx.im.paste((0, 0, 0), (int(round(cx - tw / 2 + ox)), int(round(base + oy + 0.25 * u))), sa)
    t = _Tile(ctx, cx - tw / 2 - u, top - u, cx + tw / 2 + u, top + block_h + u, pad=u)
    t.text(cx, base, title, ft, WHITE, anchor="ms")
    if sub:
        ry = base + gap_rule
        t.rrect(cx - 3.2 * u, ry, 6.4 * u, 0.36 * u, 0.18 * u, fill=_rgba(ACCENT))
        t.text(cx, ry + 0.36 * u + gap_rule + _cap(fs), sub, fs, TEXT2, anchor="ms")
    t.commit(alpha)


@lru_cache(maxsize=16)
def _backdrop(W, H, bw, bh):
    """Soft dark elliptical vignette (RGBA) of size bw x bh."""
    import numpy as np
    x = (np.arange(bw) - (bw - 1) / 2) / (bw / 2)
    y = (np.arange(bh) - (bh - 1) / 2) / (bh / 2)
    d2 = x[None, :] ** 2 + y[:, None] ** 2
    a = 0.42 * np.clip(1 - d2, 0, 1) ** 1.6
    img = Image.new("RGBA", (bw, bh), (6, 7, 9, 0))
    img.putalpha(Image.fromarray((a * 255).astype("uint8"), "L"))
    return img


@lru_cache(maxsize=32)
def _text_shadow(text, font_path, size, blur):
    f = ImageFont.truetype(font_path, size)
    l, t, r, b = f.getbbox(text, anchor="ls")
    m = 3 * blur + 2
    mask = Image.new("L", (int(r - l) + 2 * m, int(b - t) + 2 * m), 0)
    ImageDraw.Draw(mask).text((m - l, m - t), text, font=f, fill=255, anchor="ls")
    if blur > 0:
        mask = mask.filter(ImageFilter.GaussianBlur(blur))
    return mask, (l - m, t - m)


def _w_section_title(ctx, w, alpha):
    u = ctx.u
    num = str(w.get("number", "")).strip()
    title = str(w.get("title", ""))
    f = ctx.font("semibold", 3.0)
    cap = _cap(f)
    h = 5.4 * u
    pad = 1.7 * u
    nw = _tw(f, num)
    gap = 1.25 * u
    tw = _tw(f, title)
    bw = pad + (nw + 2 * gap + 0.16 * u if num else 0) + tw + pad * 1.05
    x, y = ctx.place(w, bw, h, MARGIN_U * u, MARGIN_U * u, "tl")
    t = _Tile(ctx, x, y, x + bw, y + h)
    t.plate(x, y, bw, h)
    base = y + h / 2 + cap / 2
    cx = x + pad
    if num:
        t.text(cx, base, num, f, ACCENT)
        cx += nw + gap
        t.rrect(cx, y + h / 2 - 1.3 * u, 0.16 * u, 2.6 * u, 0.08 * u, fill=(255, 255, 255, 70))
        cx += 0.16 * u + gap
    t.text(cx, base, title, f, WHITE)
    t.commit(alpha)


def _gear_str(v):
    if v is None:
        return "N"
    if isinstance(v, float) and v == int(v):
        v = int(v)
    s = str(v).strip().upper()
    if s in ("-1", "REV", "REVERSE"):
        return "R"
    return "N" if s in ("0", "", "NONE", "N") else s


def _w_gear(ctx, w, alpha):
    u = ctx.u
    s = _gear_str(w.get("value"))
    bw, h = 12.0 * u, 13.0 * u
    x, y = ctx.place(w, bw, h, ctx.W - MARGIN_U * u, 17.6 * u, "tr")
    t = _Tile(ctx, x, y, x + bw, y + h)
    t.plate(x, y, bw, h)
    fc, tr = _caps(ctx)
    t.text(x + bw / 2, y + 3.7 * u, "GEAR", fc, CAPTION, anchor="ms", tracking=tr)
    fd = ctx.font("semibold", 8.6)
    engaged = s != "N"
    t.text(x + bw / 2, y + h - 2.15 * u, s, fd, ACCENT if engaged else WHITE, anchor="ms")
    t.commit(alpha)


def _w_speed(ctx, w, alpha):
    u = ctx.u
    v = w.get("value_kmh", w.get("value", 0)) or 0
    s = str(int(round(abs(float(v)))))
    bw, h = 16.5 * u, 13.0 * u
    x, y = ctx.place(w, bw, h, ctx.W - MARGIN_U * u - 13.5 * u, 17.6 * u, "tr")
    t = _Tile(ctx, x, y, x + bw, y + h)
    t.plate(x, y, bw, h)
    fc, tr = _caps(ctx)
    t.text(x + bw / 2, y + 3.7 * u, "SPEED", fc, CAPTION, anchor="ms", tracking=tr)
    fv = ctx.font("semibold", 6.0)
    fu = ctx.font("medium", 2.3)
    # number right-aligned at a fixed x so digits never shift sideways
    unit = "km/h"
    uw = _tw(fu, unit)
    gap = 0.7 * u
    room = bw - 2 * 1.2 * u - gap - uw
    if _tw(fv, s, 0.0, TNUM) > room:  # 3+ digits: shrink to fit the plate
        fv = ctx.font("semibold", 6.0 * 0.97 * room / _tw(fv, s, 0.0, TNUM))
    nw2 = _tw(fv, "00", 0.0, TNUM)
    xr = x + bw / 2 + (nw2 + gap + uw) / 2 - uw - gap
    xr = max(xr, x + 1.2 * u + _tw(fv, s, 0.0, TNUM))
    base = y + h - 2.15 * u
    t.text(xr, base, s, fv, WHITE, anchor="rs", features=TNUM)
    t.text(xr + gap, base, unit, fu, CAPTION, anchor="ls")
    t.commit(alpha)


def _w_pedal(ctx, w, alpha):
    u = ctx.u
    v = _clamp(float(w.get("value", 0.0) or 0.0))
    label = str(w.get("label", "Clutch pedal")).upper()
    bw, h = 30.0 * u, 8.6 * u
    x, y = ctx.place(w, bw, h, ctx.W - MARGIN_U * u, 32.2 * u, "tr")
    t = _Tile(ctx, x, y, x + bw, y + h)
    t.plate(x, y, bw, h)
    pad = 1.7 * u
    fc, tr = _caps(ctx)
    t.text(x + pad, y + 3.75 * u, label, fc, CAPTION, tracking=tr)
    fv = ctx.font("semibold", 2.35)
    vs = "UP" if v < 0.015 else "DOWN" if v > 0.985 else f"{int(round(v * 100))}%"
    t.text(x + bw - pad, y + 3.75 * u, vs, fv, WHITE, anchor="rs", features=TNUM)
    bx, by, bwid, bh = x + pad, y + 5.2 * u, bw - 2 * pad, 1.15 * u
    t.rrect(bx, by, bwid, bh, bh / 2, fill=TRACK)
    if v > 0.002:
        t.rrect(bx, by, max(bh, bwid * v), bh, bh / 2, fill=_rgba(WHITE))
    bite = w.get("bite", _BITE)
    if bite:  # clutch bite range (pedal travel over which the clutch slips)
        lo, hi = float(bite[0]), float(bite[1])
        mh = max(1.0, 0.28 * u)
        t.rrect(bx + bwid * lo, by + bh + 0.55 * u, bwid * (hi - lo), mh, mh / 2, fill=_rgba(ACCENT, 0.75))
    t.commit(alpha)


def _w_rpm(ctx, w, alpha):
    u = ctx.u
    v = max(0.0, float(w.get("value", 0) or 0))
    vmax = float(w.get("max", 7000) or 7000)
    red = float(w.get("redline", 6800) or vmax)
    label = str(w.get("label", "Engine") or "").upper()
    bw, h = 26.0 * u, 22.0 * u
    x, y = ctx.place(w, bw, h, ctx.W - MARGIN_U * u, BOTTOM_U * u, "br")
    t = _Tile(ctx, x, y, x + bw, y + h)
    t.plate(x, y, bw, h)
    cx, cy = x + bw / 2, y + 10.5 * u
    r = 8.3 * u
    th = 1.0 * u
    a0, sweep = 135.0, 270.0
    ang = lambda rpm: a0 + sweep * _clamp(rpm / vmax)  # noqa: E731
    t.arc(cx, cy, r, a0, a0 + sweep, th, TRACK)
    if red < vmax:
        t.arc(cx, cy, r, ang(red), a0 + sweep, th, _rgba(RED, 0.55), caps=False)
    if v > 1:
        t.arc(cx, cy, r, a0, ang(min(v, red)), th, _rgba(ACCENT))
        if v > red:
            t.arc(cx, cy, r, ang(red), ang(v), th, _rgba(RED), caps=False)
    # 1000-rpm ticks inside the arc
    ticks = []
    r0, r1 = r - th / 2 - 1.25 * u, r - th / 2 - 0.55 * u
    for k in range(0, int(vmax // 1000) + 1):
        a = math.radians(ang(k * 1000))
        ticks.append([(cx + r0 * math.cos(a), cy + r0 * math.sin(a)), (cx + r1 * math.cos(a), cy + r1 * math.sin(a))])
    t.lines(ticks, max(1.0, 0.2 * u), (255, 255, 255, 120), caps=False)
    # value
    fv = ctx.font("semibold", 3.9)
    t.text(cx, cy + _cap(fv) / 2 - 0.3 * u, str(int(round(v))), fv, WHITE, anchor="ms", features=TNUM)
    fc, tr = _caps(ctx, 1.9)
    t.text(cx, cy + _cap(fv) / 2 + 2.45 * u, "RPM", fc, CAPTION, anchor="ms", tracking=tr)
    fl, trl = _caps(ctx)
    t.text(cx, y + h - 1.9 * u, label, fl, TEXT2, anchor="ms", tracking=trl)
    t.commit(alpha)


def _w_hpattern(ctx, w, alpha):
    u = ctx.u
    lx = _clamp(float(w.get("x", 0.0) or 0.0), -1.2, 1.2)
    ly = _clamp(float(w.get("y", 0.0) or 0.0), -1.2, 1.2)
    bw, h = 26.0 * u, 22.0 * u
    x, y = ctx.place(w, bw, h, MARGIN_U * u, BOTTOM_U * u, "bl")
    t = _Tile(ctx, x, y, x + bw, y + h)
    t.plate(x, y, bw, h)
    cx, cy = x + bw / 2, y + h / 2
    sx, sy = 6.8 * u, 5.2 * u
    sw = 1.25 * u
    slot = TRACK
    t.line([(cx - sx, cy), (cx + sx, cy)], sw, slot)
    for k in (-1, 0, 1):
        t.line([(cx + k * sx, cy - sy), (cx + k * sx, cy + sy)], sw, slot)
    # engaged-gate highlight (smooth: grows as the lever reaches the gate end)
    planes = {-1: (1, 2), 0: (3, 4), 1: (5, "R")}
    sel = {}
    for k, (gt, gb) in planes.items():
        near = 1.0 - _clamp(abs(lx - k) * 2.5)
        if near <= 0:
            continue
        s_top = near * _smooth(0.35, 0.92, ly)
        s_bot = near * _smooth(0.35, 0.92, -ly)
        if s_top > 0.01:
            sel[gt] = s_top
            t.line([(cx + k * sx, cy), (cx + k * sx, cy - sy)], sw, _rgba(ACCENT, 0.85 * s_top))
        if s_bot > 0.01:
            sel[gb] = s_bot
            t.line([(cx + k * sx, cy), (cx + k * sx, cy + sy)], sw, _rgba(ACCENT, 0.85 * s_bot))
    fn = ctx.font("semibold", 2.6)
    capn = _cap(fn)
    for k, (gt, gb) in planes.items():
        for g, yy in ((gt, cy - sy - 1.7 * u), (gb, cy + sy + 1.7 * u + capn)):
            s = sel.get(g, 0.0)
            col = _mix(CAPTION, ACCENT, s)
            t.text(cx + k * sx, yy, str(g), fn, col, anchor="ms")
    # lever knob
    px, py = cx + lx * sx, cy - ly * sy
    t.circle(px, py, 1.55 * u, fill=(0, 0, 0, 90))
    t.circle(px, py, 1.2 * u, fill=_rgba(WHITE))
    t.circle(px, py, 0.5 * u, fill=_rgba(_mix(WHITE, ACCENT, max(sel.values()) if sel else 0.0)))
    t.commit(alpha)


def _slowmo_text(factor):
    f = float(factor or 1.0)
    if f < 1.0 and f > 0:      # given as a rate (e.g. 1/150): invert
        f = 1.0 / f if f < 0.95 else 1.0
    if f < 1.05:
        return None
    if f < 9.95:
        s = f"{f:.1f}".rstrip("0").rstrip(".")
    elif f < 100:
        s = str(int(round(f)))
    else:
        s = str(int(round(f / 10.0) * 10))
    return s


def _badge(ctx, w, alpha, parts, default_xy, dot=None):
    """Pill badge: parts = [(text, font, color, tracking), ...] drawn in a row."""
    u = ctx.u
    h = 4.6 * u
    pad = 1.6 * u
    gap = 0.75 * u
    widths = [_tw(f, s, tr, TNUM) for s, f, c, tr in parts]
    dot_w = (0.75 * u * 2 + 1.0 * u) if dot else 0.0
    bw = pad * 2 + dot_w + sum(widths) + gap * (len(parts) - 1)
    x, y = ctx.place(w, bw, h, default_xy[0], default_xy[1], "tr")
    t = _Tile(ctx, x, y, x + bw, y + h)
    t.plate(x, y, bw, h, r=h / 2)
    cx = x + pad
    cyc = y + h / 2
    if dot:
        t.circle(cx + 0.75 * u, cyc, 0.75 * u, fill=_rgba(dot))
        cx += dot_w
    for (s, f, c, tr), wd in zip(parts, widths):
        t.text(cx, cyc + _cap(f) / 2, s, f, c, tracking=tr, features=TNUM if not tr else None)
        cx += wd + gap
    t.commit(alpha)


def _w_slowmo(ctx, w, alpha):
    u = ctx.u
    s = _slowmo_text(w.get("factor", 1.0))
    fc, tr = _caps(ctx, 2.1)
    xy = (ctx.W - MARGIN_U * u, MARGIN_U * u)
    if s is None:
        _badge(ctx, w, alpha, [("REAL TIME", fc, WHITE, tr)], xy, dot=WHITE)
    else:
        fv = ctx.font("semibold", 2.65)
        _badge(ctx, w, alpha, [("SLOW MOTION", fc, CAPTION, tr), ("×" + s, fv, WHITE, 0.0)], xy)


def _w_status(ctx, w, alpha):
    u = ctx.u
    kind = w.get("kind") or _status_kind(w.get("text", ""))
    kind = str(kind).lower()
    col = {"ok": GREEN, "warn": ACCENT, "info": BLUE}.get(kind, BLUE)
    fc, tr = _caps(ctx, 2.1)
    text = str(w.get("text", "")).upper()
    _badge(ctx, w, alpha, [(text, fc, WHITE, tr)], (ctx.W - MARGIN_U * u, (MARGIN_U + 5.8) * u), dot=col)


def _status_kind(text):
    t = str(text).upper()
    if "SLIP" in t or "SYNC" in t or "WARN" in t:
        return "warn"
    if "DISENGAGED" in t or "OFF" in t or "NEUTRAL" in t:
        return "info"
    if "ENGAGED" in t or "LOCKED" in t or t == "OK":
        return "ok"
    return "info"


_NUM_RE = re.compile(r"^\s*([-+−]?\d[\d,.]*)\s*(.*)$")


def _split_value(v):
    if isinstance(v, bool) or v is None:
        return (str(v) if v is not None else "–"), ""
    if isinstance(v, int):
        return str(v), ""
    if isinstance(v, float):
        if abs(v) >= 100 or abs(v - round(v)) < 1e-9:
            return str(int(round(v))), ""
        return (f"{v:.2f}" if abs(v) < 10 else f"{v:.1f}"), ""
    m = _NUM_RE.match(str(v))
    if m:
        return m.group(1), m.group(2)
    return str(v), ""


def _w_readouts(ctx, w, alpha):
    u = ctx.u
    rows = [r for r in (w.get("rows") or []) if r]
    title = w.get("title")
    if not rows and not title:
        return
    fl = ctx.font("medium", 2.45)
    fn = ctx.font("semibold", 2.75)
    fu = ctx.font("medium", 2.2)
    fc, tr = _caps(ctx)
    pad_x, pad_t, pad_b = 1.8 * u, 1.1 * u, 1.1 * u
    row_h = 4.0 * u
    title_h = 3.6 * u if title else 0.0
    parsed = []
    for r in rows:
        lab = str(r[0]) if len(r) > 0 else ""
        num, unit = _split_value(r[1] if len(r) > 1 else "")
        hi = bool(r[2]) if len(r) > 2 else False
        parsed.append((lab, num, unit, hi))
    labw = max([_tw(fl, p[0]) for p in parsed] + [0.0])
    numw = max([_tw(fn, p[1], features=TNUM) for p in parsed] + [_tw(fn, "0000", features=TNUM)])
    unitw = max([_tw(fu, p[2]) for p in parsed] + [0.0])
    ugap = 0.6 * u if unitw > 0 else 0.0
    inner = labw + 3.4 * u + numw + ugap + unitw
    if title:
        inner = max(inner, _tw(fc, str(title).upper(), tr))
    bw = inner + 2 * pad_x
    h = pad_t + title_h + row_h * len(parsed) + pad_b
    x, y = ctx.place(w, bw, h, MARGIN_U * u, 11.8 * u, "tl")
    t = _Tile(ctx, x, y, x + bw, y + h)
    t.plate(x, y, bw, h)
    yy = y + pad_t
    if title:
        t.text(x + pad_x, yy + title_h * 0.5 + _cap(fc) / 2, str(title).upper(), fc, CAPTION, tracking=tr)
        yy += title_h
    xnum = x + pad_x + labw + 3.4 * u + numw
    for i, (lab, num, unit, hi) in enumerate(parsed):
        if i > 0 or title:
            t.rrect(x + pad_x, yy - 0.06 * u, bw - 2 * pad_x, max(0.5, 0.1 * u), 0, fill=(255, 255, 255, 22))
        base = yy + row_h / 2 + _cap(fn) / 2
        t.text(x + pad_x, base, lab, fl, TEXT2)
        t.text(xnum, base, num, fn, ACCENT if hi else WHITE, anchor="rs", features=TNUM)
        if unit:
            t.text(xnum + ugap, base, unit, fu, CAPTION)
        yy += row_h
    t.commit(alpha)


_STROKES = ("INTAKE", "COMPRESSION", "POWER", "EXHAUST")


def _w_stroke_strip(ctx, w, alpha):
    u = ctx.u
    act = w.get("active", 0)
    if isinstance(act, str):
        act = _STROKES.index(act.upper()) if act.upper() in _STROKES else -1
    act = -1 if act is None else int(act)
    prog = _clamp(float(w.get("progress", 0.0) or 0.0))
    cyl = w.get("cylinder", 1)
    fc, tr = _caps(ctx, 2.0)
    fb, trb = _caps(ctx, 2.1)
    box_w = max(_tw(fb, s, trb) for s in _STROKES) + 4.4 * u
    box_h = 5.6 * u
    gap = 0.7 * u
    pad = 1.2 * u
    head = 3.5 * u
    bw = 4 * box_w + 3 * gap + 2 * pad
    h = pad + head + box_h + pad
    x, y = ctx.place(w, bw, h, ctx.W / 2, MARGIN_U * u, "tc")
    t = _Tile(ctx, x, y, x + bw, y + h)
    t.plate(x, y, bw, h)
    t.text(x + pad + 0.3 * u, y + pad + 2.3 * u, f"CYLINDER {cyl}" if cyl is not None else "FOUR-STROKE CYCLE",
           fc, CAPTION, tracking=tr)
    if cyl is not None:
        t.text(x + bw - pad - 0.3 * u, y + pad + 2.3 * u, "FOUR-STROKE CYCLE", fc, MUTED, anchor="rs", tracking=tr)
    by = y + pad + head
    for i, s in enumerate(_STROKES):
        bx = x + pad + i * (box_w + gap)
        on = i == act
        if on:
            t.rrect(bx, by, box_w, box_h, 0.7 * u, fill=_rgba(ACCENT, 0.2))
            t.rrect(bx, by, box_w, box_h, 0.7 * u, outline=_rgba(ACCENT, 0.95), ow=max(1.0, 0.16 * u))
            ph = 0.5 * u
            inset = 0.75 * u
            pw = (box_w - 2 * inset) * prog
            t.rrect(bx + inset, by + box_h - inset - ph, box_w - 2 * inset, ph, ph / 2, fill=(255, 255, 255, 40))
            if pw > 0.2 * u:
                t.rrect(bx + inset, by + box_h - inset - ph, max(ph, pw), ph, ph / 2, fill=_rgba(ACCENT))
        else:
            t.rrect(bx, by, box_w, box_h, 0.7 * u, fill=(255, 255, 255, 13))
        t.text(bx + box_w / 2, by + box_h / 2 + _cap(fb) / 2 - (0.45 * u if on else 0.0), s, fb,
               WHITE if on else MUTED, anchor="ms", tracking=trb)
    t.commit(alpha)


def _w_firing_ticker(ctx, w, alpha):
    u = ctx.u
    order = tuple(w.get("order", _FIRING))
    if w.get("cylinder") is not None:
        act = order.index(int(w["cylinder"])) if int(w["cylinder"]) in order else -1
    else:
        a = w.get("active", -1)
        act = -1 if a is None else int(a)
    fc, tr = _caps(ctx, 2.1)
    fn = ctx.font("semibold", 3.0)
    cap_s = "FIRING ORDER"
    cw = _tw(fc, cap_s, tr)
    h = 6.0 * u
    pad = 1.7 * u
    r = 1.95 * u
    step = 2 * r + 2.6 * u
    bw = pad + cw + 2.2 * u + 4 * (2 * r) + 3 * (step - 2 * r) + pad * 0.9
    x, y = ctx.place(w, bw, h, ctx.W / 2, MARGIN_U * u, "tc")
    t = _Tile(ctx, x, y, x + bw, y + h)
    t.plate(x, y, bw, h, r=h / 2)
    cyc = y + h / 2
    t.text(x + pad, cyc + _cap(fc) / 2, cap_s, fc, CAPTION, tracking=tr)
    x0 = x + pad + cw + 2.2 * u + r
    for i, c in enumerate(order):
        cx = x0 + i * step
        on = i == act
        if on:
            t.circle(cx, cyc, r, fill=_rgba(ACCENT))
        t.text(cx, cyc + _cap(fn) / 2, str(c), fn, ACCENT_INK if on else TEXT2, anchor="ms")
        if i < len(order) - 1:
            mx = cx + step / 2
            k = 0.45 * u
            t.line([(mx - k * 0.6, cyc - k), (mx + k * 0.6, cyc), (mx - k * 0.6, cyc + k)],
                   max(1.0, 0.2 * u), _rgba(MUTED), caps=False)
    t.commit(alpha)


def _w_step_card(ctx, w, alpha):
    u = ctx.u
    num = str(w.get("number", ""))
    text = str(w.get("text", ""))
    total = w.get("total")
    ft = ctx.font("semibold", 3.5)
    fnum = ctx.font("bold", 3.0)
    fc, tr = _caps(ctx, 1.95)
    sub = f"STEP {num} / {total}" if total else None
    h = (8.8 if sub else 7.6) * u
    pad = 1.5 * u
    r = 2.35 * u
    tw = _tw(ft, text)
    if sub:
        tw = max(tw, _tw(fc, sub, tr))
    bw = pad + 2 * r + 1.8 * u + tw + pad * 1.4
    x, y = ctx.place(w, bw, h, ctx.W / 2, MARGIN_U * u, "tc")
    t = _Tile(ctx, x, y, x + bw, y + h)
    t.plate(x, y, bw, h)
    cx, cyc = x + pad + r, y + h / 2
    t.circle(cx, cyc, r, fill=_rgba(ACCENT))
    t.text(cx, cyc + _cap(fnum) / 2, num, fnum, ACCENT_INK, anchor="ms")
    tx = cx + r + 1.8 * u
    if sub:
        t.text(tx, y + 3.2 * u, sub, fc, CAPTION, tracking=tr)
        t.text(tx, y + h - 1.75 * u, text, ft, WHITE)
    else:
        t.text(tx, cyc + _cap(ft) / 2, text, ft, WHITE)
    t.commit(alpha)


def _w_ratio_card(ctx, w, alpha):
    u = ctx.u
    rt = str(w.get("ratio_text", ""))
    capt = str(w.get("caption", "") or "")
    fr = ctx.font("semibold", 5.4)
    fc = ctx.font("medium", 2.5)
    rw = _tw(fr, rt, features=TNUM)
    cw = _tw(fc, capt)
    pad_x = 2.4 * u
    h = (12.9 if capt else 8.6) * u
    bw = max(rw, cw) + 2 * pad_x
    x, y = ctx.place(w, bw, h, ctx.W / 2, MARGIN_U * u, "tc")
    t = _Tile(ctx, x, y, x + bw, y + h)
    t.plate(x, y, bw, h)
    base = y + 2.1 * u + _cap(fr)
    t.text(x + bw / 2, base, rt, fr, WHITE, anchor="ms", features=TNUM)
    if capt:
        t.rrect(x + bw / 2 - 2.2 * u, base + 1.7 * u, 4.4 * u, 0.3 * u, 0.15 * u, fill=_rgba(ACCENT))
        t.text(x + bw / 2, base + 1.7 * u + 0.3 * u + 1.6 * u + _cap(fc), capt, fc, TEXT2, anchor="ms")
    t.commit(alpha)


@lru_cache(maxsize=8)
def _solid(size, color):
    return Image.new("RGB", size, color)


def _w_fade(ctx, w, alpha):
    if alpha <= 0.002:
        return
    col = _color(w.get("color", "black"))
    if alpha >= 0.998:
        ctx.im.paste(col, (0, 0, ctx.W, ctx.H))
        return
    blended = Image.blend(ctx.im, _solid(ctx.im.size, col), alpha)
    ctx.im.paste(blended)


WIDGETS = {
    "title_card": _w_title_card,
    "section_title": _w_section_title,
    "gear": _w_gear,
    "rpm": _w_rpm,
    "speed": _w_speed,
    "pedal": _w_pedal,
    "hpattern": _w_hpattern,
    "slowmo": _w_slowmo,
    "status": _w_status,
    "readouts": _w_readouts,
    "stroke_strip": _w_stroke_strip,
    "firing_ticker": _w_firing_ticker,
    "step_card": _w_step_card,
    "ratio_card": _w_ratio_card,
    "fade": _w_fade,
}
_warned = set()


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def render_overlay(im, labels=(), hud=()):
    """Draw labels then HUD widgets (then fades) onto `im` (RGB, modified in
    place, also returned)."""
    if im.mode != "RGB":
        im = im.convert("RGB")
    ctx = _Ctx(im)
    for L in labels or ():
        try:
            _draw_label(ctx, L)
        except Exception as e:  # never lose a frame over one bad label
            print(f"[overlay] label {L.get('id')!r} failed: {e}", file=sys.stderr)
    fades = []
    for wdg in hud or ():
        typ = wdg.get("type")
        fn = WIDGETS.get(typ)
        if fn is None:
            if typ not in _warned:
                _warned.add(typ)
                print(f"[overlay] unknown widget type {typ!r} ignored", file=sys.stderr)
            continue
        a = _clamp(float(wdg.get("alpha", 1.0) if wdg.get("alpha") is not None else 1.0))
        if typ == "fade":
            fades.append((wdg, a))
            continue
        if a <= 0.002:
            continue
        try:
            fn(ctx, wdg, a)
        except Exception as e:
            print(f"[overlay] widget {typ!r} failed: {e}", file=sys.stderr)
    for wdg, a in fades:
        _w_fade(ctx, wdg, a)
    return ctx.im


PNG_COMPRESS = 1


def _save_atomic(im, out_path):
    d = os.path.dirname(os.path.abspath(out_path))
    os.makedirs(d, exist_ok=True)
    tmp = os.path.join(d, f".tmp_{os.path.basename(out_path)}.{os.getpid()}.png")
    im.save(tmp, format="PNG", compress_level=PNG_COMPRESS)
    os.replace(tmp, out_path)


def compose_frame(raw_path, out_path, labels, hud, size=None):
    """Composite one frame: raw PNG (or PIL image) -> out_path (RGB PNG,
    written atomically).  `size` = (W, H) resizes the raw frame first."""
    if isinstance(raw_path, Image.Image):
        im = raw_path.convert("RGB") if raw_path.mode != "RGB" else raw_path.copy()
    else:
        with Image.open(raw_path) as src:
            im = src.convert("RGB")
    if size is not None and tuple(size) != im.size:
        im = im.resize(tuple(int(v) for v in size), Image.LANCZOS)
    im = render_overlay(im, labels or [], hud or [])
    _save_atomic(im, out_path)
    return out_path


def _load_json(x):
    if isinstance(x, dict):
        return x
    if x is None:
        return {"frames": {}}
    with open(x) as fh:
        return json.load(fh)


def _job(args):
    raw, out, labels, hud = args
    t0 = time.time()
    compose_frame(raw, out, labels, hud)
    return out, time.time() - t0


def _clean_stale_tmp(out_dir):
    for name in os.listdir(out_dir):
        m = re.match(r"^\.tmp_.*\.(\d+)\.png$", name)
        if not m:
            continue
        pid = int(m.group(1))
        try:
            os.kill(pid, 0)
            alive = True
        except ProcessLookupError:
            alive = False
        except PermissionError:
            alive = True
        if not alive:
            try:
                os.remove(os.path.join(out_dir, name))
            except OSError:
                pass


def _mp_context():
    """'fork' where available: works from unguarded caller scripts (no
    __main__ re-import) and is safe from a process that imported bpy because
    the children only run Pillow code.  CARVIZ_MP_START overrides."""
    method = os.environ.get("CARVIZ_MP_START")
    if not method:
        method = "fork" if "fork" in mp.get_all_start_methods() else "spawn"
    return mp.get_context(method)


def compose_sequence(raw_dir, out_dir, labels_json, hud_json, frames=None, workers=2, overwrite=False,
                     name="{:05d}.png", verbose=True):
    """Composite many frames: raw_dir/00001.png -> out_dir/00001.png.

    labels_json / hud_json: paths or already-loaded dicts.  frames: list of
    frame numbers (default: every numbered PNG in raw_dir).  Resumable: an
    existing non-empty output is skipped unless overwrite=True; outputs are
    written to a temp file then renamed.  workers > 1 uses a process pool
    (see _mp_context).
    Returns {"done", "skipped", "missing", "seconds", "per_frame"}."""
    t0 = time.time()
    os.makedirs(out_dir, exist_ok=True)
    _clean_stale_tmp(out_dir)
    lab = _load_json(labels_json).get("frames", {})
    hud = _load_json(hud_json).get("frames", {})
    if frames is None:
        frames = sorted(int(m.group(1)) for m in (re.match(r"^(\d+)\.png$", n) for n in os.listdir(raw_dir)) if m)
    jobs, skipped, missing = [], 0, 0
    for f in frames:
        f = int(f)
        raw = os.path.join(raw_dir, name.format(f))
        out = os.path.join(out_dir, name.format(f))
        if not os.path.exists(raw):
            missing += 1
            continue
        if not overwrite and os.path.exists(out) and os.path.getsize(out) > 0:
            skipped += 1
            continue
        jobs.append((raw, out, lab.get(str(f), []), hud.get(str(f), [])))
    n = len(jobs)
    tsum = 0.0
    if n:
        workers = max(1, min(int(workers or 1), n))
        step = max(1, n // 10)
        if workers == 1:
            it = map(_job, jobs)
            pool = None
        else:
            pool = _mp_context().Pool(workers)
            it = pool.imap_unordered(_job, jobs, chunksize=max(1, min(8, n // (workers * 4) or 1)))
        try:
            for k, (out, dt) in enumerate(it):
                tsum += dt
                if verbose and ((k + 1) % step == 0 or k + 1 == n):
                    print(f"[overlay] {k + 1}/{n} frames ({time.time() - t0:.1f}s)", flush=True)
        finally:
            if pool is not None:
                pool.close()
                pool.join()
    res = dict(done=n, skipped=skipped, missing=missing, seconds=round(time.time() - t0, 2),
               per_frame=round(tsum / n, 4) if n else 0.0)
    if verbose:
        print(f"[overlay] done {n}, skipped {skipped}, missing raw {missing}, {res['seconds']}s "
              f"({res['per_frame']}s/frame/worker)", flush=True)
    return res


def _parse_frames(s):
    if not s:
        return None
    out = []
    for part in s.split(","):
        part = part.strip()
        if "-" in part:
            a, b = part.split("-", 1)
            out.extend(range(int(a), int(b) + 1))
        elif part:
            out.append(int(part))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python3 -m carviz.overlay", description=__doc__.split("\n")[0])
    ap.add_argument("raw_dir")
    ap.add_argument("out_dir")
    ap.add_argument("labels_json")
    ap.add_argument("hud_json")
    ap.add_argument("--frames", default=None, help="e.g. 1-240 or 1,5,9")
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--overwrite", action="store_true")
    a = ap.parse_args(argv)
    compose_sequence(a.raw_dir, a.out_dir, a.labels_json, a.hud_json, frames=_parse_frames(a.frames),
                     workers=a.workers, overwrite=a.overwrite)


if __name__ == "__main__":
    main()
