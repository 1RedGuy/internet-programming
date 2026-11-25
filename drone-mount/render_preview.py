"""Renders preview PNGs of the mount (simple numpy z-buffer, no GPU needed)."""
import math
import pathlib
import sys
import json

import cadquery as cq
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import mavic4pro_mount as m


def mesh(wp, tol=0.08):
    v, t = wp.val().tessellate(tol, 0.2)
    return np.array([(p.x, p.y, p.z) for p in v]), np.array(t)


def render(items, eye_dir, up=(0, 0, 1), size=(1500, 1050), margin=0.07, ss=2, bg=(250, 250, 248)):
    """items: list of (verts, tris, rgb). Orthographic view looking along -eye_dir."""
    W, H = size[0] * ss, size[1] * ss
    f = np.array(eye_dir, float); f /= np.linalg.norm(f)
    r = np.cross(up, f); r /= np.linalg.norm(r)
    u = np.cross(f, r)
    allv = np.vstack([v for v, _, _ in items])
    px, py = allv @ r, allv @ u
    sc = min(W * (1 - 2 * margin) / (px.max() - px.min()), H * (1 - 2 * margin) / (py.max() - py.min()))
    ox = W / 2 - sc * (px.max() + px.min()) / 2
    oy = H / 2 + sc * (py.max() + py.min()) / 2
    zbuf = np.full((H, W), -1e9)
    img = np.empty((H, W, 3), float); img[:] = bg
    idb = np.zeros((H, W), int)
    nrm = np.zeros((H, W, 3))
    light = np.array([0.35, -0.45, 0.82]); light /= np.linalg.norm(light)
    for k, (v, tris, col) in enumerate(items, 1):
        sx, sy, sz = ox + sc * (v @ r), oy - sc * (v @ u), v @ f
        col = np.array(col, float)
        for a, b, c in tris:
            n = np.cross(v[b] - v[a], v[c] - v[a]); ln = np.linalg.norm(n)
            if ln < 1e-12:
                continue
            n /= ln
            if n @ f < 0:
                n = -n
            x0, x1 = int(max(min(sx[a], sx[b], sx[c]), 0)), int(min(max(sx[a], sx[b], sx[c]) + 1, W - 1))
            y0, y1 = int(max(min(sy[a], sy[b], sy[c]), 0)), int(min(max(sy[a], sy[b], sy[c]) + 1, H - 1))
            if x1 < x0 or y1 < y0:
                continue
            X, Y = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
            den = (sy[b] - sy[c]) * (sx[a] - sx[c]) + (sx[c] - sx[b]) * (sy[a] - sy[c])
            if abs(den) < 1e-9:
                continue
            w0 = ((sy[b] - sy[c]) * (X - sx[c]) + (sx[c] - sx[b]) * (Y - sy[c])) / den
            w1 = ((sy[c] - sy[a]) * (X - sx[c]) + (sx[a] - sx[c]) * (Y - sy[c])) / den
            w2 = 1 - w0 - w1
            Z = w0 * sz[a] + w1 * sz[b] + w2 * sz[c]
            sub = zbuf[y0:y1 + 1, x0:x1 + 1]
            mk = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6) & (Z > sub)
            if not mk.any():
                continue
            sub[mk] = Z[mk]
            shade = 0.42 + 0.58 * max(0.0, float(n @ light))
            img[y0:y1 + 1, x0:x1 + 1][mk] = col * shade
            idb[y0:y1 + 1, x0:x1 + 1][mk] = k
            nrm[y0:y1 + 1, x0:x1 + 1][mk] = n
    # outlines: part boundaries and sharp creases
    e = np.zeros((H, W), bool)
    for ax in (0, 1):
        did = np.diff(idb, axis=ax) != 0
        dn = (np.abs(np.diff(nrm, axis=ax)).sum(axis=2) > 0.55)
        dz = np.abs(np.diff(zbuf, axis=ax)) > 2.5
        ee = did | dn | (dz & (np.diff(idb, axis=ax) == 0) & (np.take(idb, range(1, idb.shape[ax]), axis=ax) > 0))
        if ax == 0:
            e[:-1, :] |= ee; e[1:, :] |= ee
        else:
            e[:, :-1] |= ee; e[:, 1:] |= ee
    img[e] = img[e] * 0.25
    im = Image.fromarray(img.clip(0, 255).astype(np.uint8)).resize(size, Image.LANCZOS)
    to_px = lambda p: ((ox + sc * (np.array(p) @ r)) / ss, (oy - sc * (np.array(p) @ u)) / ss)
    return im, to_px


def font(sz):
    for f in ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
              "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf"):
        if pathlib.Path(f).exists():
            return ImageFont.truetype(f, sz)
    return ImageFont.load_default()


def label(im, to_px, pt, text, off=(60, -60), fnt=None):
    dr = ImageDraw.Draw(im)
    x, y = to_px(pt)
    tx, ty = x + off[0], y + off[1]
    dr.line([(x, y), (tx, ty)], fill=(30, 30, 30), width=2)
    dr.ellipse([x - 4, y - 4, x + 4, y + 4], fill=(30, 30, 30))
    fnt = fnt or font(22)
    bb = dr.textbbox((0, 0), text, font=fnt)
    w, h = bb[2] - bb[0], bb[3] - bb[1]
    ax = tx if off[0] >= 0 else tx - w
    ay = ty - h - 8 if off[1] < 0 else ty + 4
    dr.rectangle([ax - 5, ay - 4, ax + w + 5, ay + h + 8], fill=(255, 255, 255), outline=(60, 60, 60))
    dr.text((ax, ay), text, fill=(20, 20, 20), font=fnt)


# calibration of the FCC photos (ref/): pixel of the centre top camera, mm per pixel, nose = up
TOP = dict(file="ref/fcc_top_view.png", cx=612.5, cy=401.0, mm=0.533)
BOT = dict(file="ref/fcc_bottom_view.png", cx=592.0, cy=340.0, mm=0.524)   # cy +/- 20 px uncertain


def overlay(d, out):
    """Draw the mount footprint on the real top / bottom photos."""
    here = pathlib.Path(__file__).parent
    sx, L = d["sensor_x"], d["saddle_len"]
    tiles = []
    for cal, title, bottom in ((TOP, "TOP  (FCC external photo 1)", False), (BOT, "BOTTOM  (FCC external photo 7)", True)):
        im = Image.open(here / cal["file"]).convert("RGB")
        k = 3
        box = (int(cal["cx"] - 150), int(cal["cy"] - 215), int(cal["cx"] + 150), int(cal["cy"] + 215))
        im = im.crop(box).resize(((box[2] - box[0]) * k, (box[3] - box[1]) * k), Image.LANCZOS)
        dr = ImageDraw.Draw(im, "RGBA")
        def P(x, y):      # mount coords (x forward, y left) -> pixel
            return ((cal["cx"] - y / cal["mm"] - box[0]) * k, (cal["cy"] - (x - sx) / cal["mm"] - box[1]) * k)
        def rect(x0, x1, y0, y1, fill, outline, w=4):
            a, b = P(x1, y1), P(x0, y0)
            dr.rectangle([min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])], fill=fill, outline=outline, width=w)
        bx0, bx1 = d["beam_x"] - d["beam_w"] / 2, d["beam_x"] + d["beam_w"] / 2
        f = font(26)
        if not bottom:
            rect(L / 2 - d["tray_l"], L / 2, -d["tray_w"] / 2, d["tray_w"] / 2, (40, 160, 170, 70), (20, 110, 120, 255))
            rect(-L / 2, L / 2, -d["deck_w"] / 2, d["deck_w"] / 2, (240, 120, 30, 90), (200, 80, 0, 255))
            rect(bx0, bx1, -d["ear_out"], d["ear_out"], (240, 120, 30, 90), (200, 80, 0, 255))
            for sy in (1, -1):
                c = P(d["beam_x"], sy * d["ear_y"]); dr.ellipse([c[0] - 9, c[1] - 9, c[0] + 9, c[1] + 9], fill=(255, 255, 255, 255), outline=(0, 0, 0, 255))
            for txt, pt, col in (("tray", (L / 2 - d["tray_l"] + 4, d["tray_w"] / 2 - 2), (10, 80, 90)),
                                 ("saddle", (-4, 16), (150, 50, 0)),
                                 ("clamp screws", (bx1 + 9, d["ear_out"] + 2), (150, 50, 0))):
                q = P(*pt); dr.text(q, txt, fill=col + (255,), font=f)
        else:
            rect(bx0, bx1, -d["ear_out"], d["ear_out"], (60, 60, 70, 120), (255, 220, 0, 255))
            q = P(bx0 - 3, d["ear_out"]); dr.text(q, "clamp bar under the belly", fill=(255, 220, 0, 255), font=f)
            q = P(bx1 + 26, 30); dr.text(q, "+/- 10 mm along the body", fill=(255, 220, 0, 255), font=font(20))
        dr.rectangle([0, 0, im.width, 44], fill=(0, 0, 0, 170)); dr.text((12, 8), title, fill=(255, 255, 255, 255), font=f)
        tiles.append(im)
    sheet = Image.new("RGB", (tiles[0].width + tiles[1].width + 20, tiles[0].height), "white")
    sheet.paste(tiles[0], (0, 0)); sheet.paste(tiles[1], (tiles[0].width + 20, 0))
    sheet.save(out)


# inside profile of a third-party AirTag clip that is reported to fit the Mavic 4 Pro
# (measured from its STL): it grips the lower hull from below
CLIP = dict(in_hw=25.2, flat_hw=17.8, chamfer=8.8, hook_z=41.9, tip_z=47.0, tip_hw=20.6, out_hw=34.0, beam=10.8)


def section_check(d, parts, out):
    """Cross-section at the clamp: assumed fuselage, the third-party clip, and this mount."""
    k, W, H = 9.0, 1500, 1320
    cy, cz = W / 2, 215
    to = lambda y, z: (cy + y * k, cz - z * k)
    im = Image.new("RGB", (W, H), (250, 250, 248)); dr = ImageDraw.Draw(im, "RGBA")
    f, fs = font(24), font(20)
    for g in range(-70, 71, 10):
        dr.line([to(g, 22), to(g, -108)], fill=(225, 228, 235, 255))
    for g in range(-100, 21, 10):
        dr.line([to(-75, g), to(75, g)], fill=(225, 228, 235, 255))
    # fuselage (same construction as body_proxy)
    hw, rf, bw, h = d["body_w"] / 2, d["roof_flat"] / 2, d["belly_w"] / 2, d["body_h"]
    rs, led = hw - rf, -h + d["hull_h"]
    pts = [(rf, 0.0)] + [(rf + rs * math.sin(math.radians(a)), -rs + rs * math.cos(math.radians(a))) for a in range(15, 91, 15)]
    pts += [(hw, led + 4), (hw - 4, led), (bw, led), (bw, -h + 8.8), (bw - 7.4, -h)]
    pts = pts + [(-y, z) for y, z in reversed(pts)]
    dr.polygon([to(*q) for q in pts], fill=(196, 198, 205, 255), outline=(90, 90, 100, 255))
    # third-party clip, from below
    c = CLIP; zb = -h
    half = [(c["flat_hw"], zb), (c["in_hw"], zb + c["chamfer"]), (c["in_hw"], zb + c["hook_z"]), (c["tip_hw"], zb + c["tip_z"]),
            (c["tip_hw"] + 8.8, zb + c["tip_z"]), (c["out_hw"], zb + c["hook_z"] - 0.2), (c["out_hw"], zb + c["chamfer"]),
            (c["flat_hw"] + 1.5, zb - c["beam"]), (0, zb - c["beam"])]
    clip = half + [(-y, z) for y, z in reversed(half)]
    dr.line([to(*q) for q in clip] + [to(*clip[0])], fill=(20, 150, 60, 255), width=4)
    # this mount: slice the real solids at the beam
    slab = cq.Workplane("XY").box(0.4, 400, 400).translate((d["beam_x"], 0, 0))
    for name, col in (("saddle", (240, 120, 30, 215)), ("yoke", (60, 62, 72, 215)), ("tray", (40, 130, 140, 215))):
        v, t = mesh(parts[name].intersect(slab), 0.05)
        for a, b_, c_ in t:
            dr.polygon([to(v[a][1], v[a][2]), to(v[b_][1], v[b_][2]), to(v[c_][1], v[c_][2])], fill=col)

    def dim(p0, p1, text, col=(150, 10, 10), dy=-34, dx=0):
        a, b = to(*p0), to(*p1)
        dr.line([a, b], fill=col + (255,), width=3)
        for q in (a, b):
            dr.ellipse([q[0] - 5, q[1] - 5, q[0] + 5, q[1] + 5], fill=col + (255,))
        mx, my = (a[0] + b[0]) / 2 + dx, (a[1] + b[1]) / 2 + dy
        bb = dr.textbbox((0, 0), text, font=fs)
        dr.rectangle([mx - bb[2] / 2 - 6, my - 4, mx + bb[2] / 2 + 6, my + bb[3] + 6], fill=(255, 255, 255, 235))
        dr.text((mx - bb[2] / 2, my), text, fill=col + (255,), font=fs)

    dim((-bw, -h + 24), (bw, -h + 24), f"lower hull {2 * bw:g} mm  =  inside of the AirTag clip (photos: 51)", (15, 120, 45))
    dim((-hw, -24), (hw, -24), f"waist ~{2 * hw:g} mm  (FCC photos; ring model is 73 inside)")
    dim((0, 0), (0, -h), f"height {h:g} mm  (82-88, least certain)", dx=-230, dy=40)
    dim((hw, -40), (d["yi"], -40), f"{d['side_clear']:g} mm air", dx=92, dy=-12)
    dr.text((30, 22), "Section at the clamp, looking forward", fill=(20, 20, 20, 255), font=f)
    y0 = H - 150
    for i, (col, txt) in enumerate((((196, 198, 205), "fuselage as reconstructed (FCC photos + the uploaded models)"),
                                    ((20, 150, 60), "third-party AirTag clip for the Mavic 4 Pro, real STL outline"),
                                    ((240, 120, 30), "this mount: saddle + beam (orange), U-clamp (dark), tray (teal)"))):
        dr.rectangle([30, y0 + i * 40, 62, y0 + i * 40 + 24], fill=col + (255,), outline=(60, 60, 60, 255))
        dr.text((76, y0 + i * 40), txt, fill=(20, 20, 20, 255), font=fs)
    im.save(out)


if __name__ == "__main__":
    override = json.loads(pathlib.Path(sys.argv[1]).read_text()) if len(sys.argv) > 1 else {}
    d, parts = m.build(override)
    out = pathlib.Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    overlay(d, out / "preview_on_photos.png")
    section_check(d, parts, out / "preview_section_check.png")

    ORANGE, TEAL, GREY, DARK, BLUE, STEEL, BLACK, WHITE = ((240, 120, 30), (40, 130, 140), (200, 202, 208), (70, 72, 80),
                                                           (150, 185, 235), (175, 180, 190), (25, 25, 28), (235, 235, 230))
    body = m.body_proxy(d)
    cams, vents = m.body_marks(d)
    L = d["saddle_len"]
    box_h = 30.0
    zt = d["tray_z0"] + d["tray_t"]
    payload = (cq.Workplane("XY").box(d["tray_l"] - 4, 56, box_h)
               .translate((L / 2 - d["tray_l"] / 2, 0, zt + box_h / 2)).edges().fillet(3))

    sad, tr, yk = mesh(parts["saddle"]), mesh(parts["tray"]), mesh(parts["yoke"])
    bod, pl, scr = mesh(body, 0.15), mesh(payload, 0.15), mesh(m.screws(d), 0.05)
    cm, vt = mesh(cams, 0.1), mesh(vents, 0.1)
    base = [(*bod, GREY), (*cm, BLACK), (*vt, WHITE)]
    ey, sx = d["ear_y"], d["sensor_x"]

    # 1) assembled, 3/4 rear view
    items = base + [(*yk, DARK), (*sad, ORANGE), (*tr, TEAL), (*pl, BLUE), (*scr, STEEL)]
    im, px = render(items, eye_dir=(-0.75, -0.9, 0.62))
    label(im, px, (d["beam_x"], -d["ear_out"] + 4, -8), "saddle + cross beam", (-60, -150))
    label(im, px, (-L / 2 - 4, -d["tray_w"] / 2 + 6, zt), "tray (slides in from the rear)", (-260, -60))
    label(im, px, (d["beam_x"], -d["yi"] - 3, -55), "U-clamp, clear of the sides", (40, 150))
    label(im, px, (L / 2 - d["tray_l"] / 2, 0, zt + box_h), "your payload (example box)", (120, -70))
    label(im, px, (sx, 0, 2), "centre top camera - stays uncovered", (150, -110))
    label(im, px, (sx - 76, 0, 1), "exhaust grille - stays uncovered", (-60, -330))
    label(im, px, (sx - 30, -d["body_w"] / 2, -21), "side air vent", (-60, 190))
    im.save(out / "preview_assembly.png")

    # 2) exploded
    lift, drop = 24, 26
    tr_x = (tr[0] + np.array([-34, 0, lift]), tr[1])
    yk_x = (yk[0] + np.array([0, 0, -drop]), yk[1])
    scr_x = (scr[0] + np.array([0, 0, 16]), scr[1])
    items = base + [(*yk_x, DARK), (*sad, ORANGE), (*tr_x, TEAL), (*scr_x, STEEL)]
    im, px = render(items, eye_dir=(-0.8, -0.75, 0.75))
    label(im, px, (-L / 2 - 34 - 5, -d["lock_y"], d["lock_z"] + lift), "2 x M3x12 tray lock", (-200, 40))
    label(im, px, (L / 2 - 4, d["rail_w"] / 2 + 2, d["deck_top"]), "dovetail channel", (170, -90))
    label(im, px, (d["beam_x"], -ey, d["ear_top"] + 20), "2 x M3x20 clamp screws", (-330, -30))
    label(im, px, (d["beam_x"], -d["yi"] - 3, -55 - drop), "clamp comes up from below", (60, 120))
    label(im, px, (d["beam_x"], -ey, d["flange_top"] - drop), "captive nut under the flange", (-340, 70))
    im.save(out / "preview_exploded.png")

    # 3) front view with dimensions (payload hidden)
    items = [(*bod, GREY), (*yk, DARK), (*sad, ORANGE), (*tr, TEAL), (*scr, STEEL)]
    im, px = render(items, eye_dir=(1, 0, 0), size=(1500, 1250), margin=0.17)
    dr = ImageDraw.Draw(im); fnt = font(22)

    def dim(p0, p1, text, side=1):
        a, b = px(p0), px(p1)
        dr.line([a, b], fill=(180, 20, 20), width=3)
        for q in (a, b):
            dr.ellipse([q[0] - 5, q[1] - 5, q[0] + 5, q[1] + 5], fill=(180, 20, 20))
        mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
        bb = dr.textbbox((0, 0), text, font=fnt)
        x0 = mx + 10 if side > 0 else mx - 18 - bb[2]
        dr.rectangle([x0 - 4, my - 18, x0 + bb[2] + 6, my + 16], fill=(255, 255, 255))
        dr.text((x0, my - 14), text, fill=(150, 10, 10), font=fnt)

    bw, bh, X = d["body_w"], d["body_h"], 200
    dim((X, -bw / 2, -bh * 0.62), (X, bw / 2, -bh * 0.62), f"waist ~{bw:g}")
    dim((X, -10, 0), (X, -10, -bh), f"height ~{bh:g}", side=-1)
    dim((X, d["ear_out"] + 6, 0), (X, d["ear_out"] + 6, zt), f"{zt:.1f} up")
    dim((X, -d["ear_out"], d["yoke_bot"] - 9), (X, d["ear_out"], d["yoke_bot"] - 9), f"overall {2 * d['ear_out']:.0f} wide")
    dim((X, d["yi"] + 8, -bh), (X, d["yi"] + 8, d["yoke_bot"]), f"{-d['yoke_bot'] - bh:.1f} below belly")
    dim((X, bw / 2, -bh * 0.38), (X, d["yi"], -bh * 0.38), f"{d['side_clear']:g} air gap")
    im.save(out / "preview_front.png")
    print("ok")
