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


# calibration of the FCC photos (ref/): pixel of the centre top camera, mm per pixel at fuselage height, nose = up
TOP = dict(file="ref/fcc_top_view.png", cx=612.5, cy=401.0, mm=0.533)
BOT = dict(file="ref/fcc_bottom_view.png", cx=592.0, cy=340.0, mm=0.50)    # cy +/- 20 px uncertain


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
            rect(-L / 2, L / 2, -d["deck_w"] / 2, d["deck_w"] / 2, (240, 120, 30, 90), (200, 80, 0, 255))
            rect(bx0, bx1, -d["ear_out"], d["ear_out"], (240, 120, 30, 90), (200, 80, 0, 255))
            for sy in (1, -1):
                c = P(d["beam_x"], sy * d["ear_y"]); dr.ellipse([c[0] - 9, c[1] - 9, c[0] + 9, c[1] + 9], fill=(255, 255, 255, 255), outline=(0, 0, 0, 255))
            for txt, pt, col in (("saddle", (-4, 16), (150, 50, 0)), ("clamp screws", (bx1 + 9, d["ear_out"] + 2), (150, 50, 0))):
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


def section(d, parts, out):
    """Cross-section at the clamp with the rubber strips and the measured numbers."""
    k, W, H = 9.0, 1500, 1330
    cy, cz = W / 2, 215
    to = lambda y, z: (cy + y * k, cz - z * k)
    im = Image.new("RGB", (W, H), (250, 250, 248)); dr = ImageDraw.Draw(im, "RGBA")
    f, fs = font(24), font(20)
    for g in range(-70, 71, 10):
        dr.line([to(g, 22), to(g, -108)], fill=(225, 228, 235, 255))
    for g in range(-100, 21, 10):
        dr.line([to(-75, g), to(75, g)], fill=(225, 228, 235, 255))
    hw, rf, bf, h = d["body_w"] / 2, d["roof_flat"] / 2, d["belly_flat"] / 2, d["body_h"]
    rs, rb = hw - rf, hw - bf
    pts = [(rf + rs * math.sin(math.radians(a)), -rs + rs * math.cos(math.radians(a))) for a in range(0, 91, 10)]
    pts += [(bf + rb * math.cos(math.radians(a)), -h + rb - rb * math.sin(math.radians(a))) for a in range(0, 91, 10)]
    pts = pts + [(-y, z) for y, z in reversed(pts)]
    dr.polygon([to(*q) for q in pts], fill=(196, 198, 205, 255), outline=(90, 90, 100, 255))
    for sy in (1, -1):                                   # side vents
        a, b = to(sy * (hw - 1.2), -15), to(sy * (hw + 0.4), -26)
        dr.rectangle([min(a[0], b[0]), a[1], max(a[0], b[0]), b[1]], fill=(40, 40, 45, 255))
    slab = cq.Workplane("XY").box(0.4, 400, 400).translate((d["beam_x"] + 4.5, 0, 0))    # beside the air slot
    for name, col in (("saddle", (240, 120, 30, 220)), ("yoke", (60, 62, 72, 220))):
        v, t = mesh(parts[name].intersect(slab), 0.05)
        for a, b_, c_ in t:
            dr.polygon([to(v[a][1], v[a][2]), to(v[b_][1], v[b_][2]), to(v[c_][1], v[c_][2])], fill=col)
    rub = (245, 240, 215, 255)
    def pad(y0, y1, z0, z1):
        a, b = to(y0, z1), to(y1, z0)
        dr.rectangle([min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])], fill=rub, outline=(120, 110, 60, 255), width=2)
    for sy in (1, -1):
        pad(sy * 13.5, sy * 22.5, 0, d["pad_top"])
        pad(sy * hw, sy * d["yk"], -h + 22, -h + 31)
    pad(-17, 17, d["yoke_in_bot"], -h)

    def dim(p0, p1, text, col=(150, 10, 10), dy=-34, dx=0):
        a, b = to(*p0), to(*p1)
        dr.line([a, b], fill=col + (255,), width=3)
        for q in (a, b):
            dr.ellipse([q[0] - 5, q[1] - 5, q[0] + 5, q[1] + 5], fill=col + (255,))
        mx, my = (a[0] + b[0]) / 2 + dx, (a[1] + b[1]) / 2 + dy
        bb = dr.textbbox((0, 0), text, font=fs)
        dr.rectangle([mx - bb[2] / 2 - 6, my - 4, mx + bb[2] / 2 + 6, my + bb[3] + 6], fill=(255, 255, 255, 235))
        dr.text((mx - bb[2] / 2, my), text, fill=col + (255,), font=fs)

    dim((-hw, -48), (hw, -48), f"body {2 * hw:g} mm  (measured 68-71)")
    dim((-12, 0), (-12, -h), f"height {h:g} mm  (80 +/- 4)", dx=-150, dy=130)
    dim((hw, -20.5), (d["ya"], -20.5), f"{d['arm_clear']:g} mm air at the vent", dx=150, dy=-12)
    dim((hw, -h + 26.5), (d["yk"], -h + 26.5), f"{d['side_gap']:g} mm, rubber", dx=112, dy=-12)
    dim((d["ear_out"] + 4, d["ear_bot"]), (d["ear_out"] + 4, d["flange_top"]), f"{d['clamp_gap']:g} mm travel", dx=95, dy=-12)
    dr.text((30, 22), "Section at the clamp, looking forward", fill=(20, 20, 20, 255), font=f)
    y0 = H - 140
    for i, (col, txt) in enumerate((((196, 198, 205), "fuselage as measured from the FCC photos (dark marks = side air vents)"),
                                    ((240, 120, 30), "saddle with its two arms (orange) and U-clamp (dark), screws at the ears"),
                                    ((245, 240, 215), "rubber seal strips - the only things that touch the drone"))):
        dr.rectangle([30, y0 + i * 40, 62, y0 + i * 40 + 24], fill=col + (255,), outline=(60, 60, 60, 255))
        dr.text((76, y0 + i * 40), txt, fill=(20, 20, 20, 255), font=fs)
    im.save(out)


# motor axes measured from the top photo (mm, relative to the centre top camera; x forward, y left), symmetrised
MOTORS = dict(front=(111.0, 185.5), rear=(-165.0, 163.0))
PROP_R = 133.5


def prop_plan(d, out):
    """Top view: propeller discs, fuselage, clamp and the zone a payload box may use."""
    k, W, H = 2.6, 1500, 1180
    ox, oy = W / 2, H / 2 - 40
    to = lambda x, y: (ox - y * k, oy - x * k)             # nose up, left of the drone on the left
    im = Image.new("RGB", (W, H), (250, 250, 248)); dr = ImageDraw.Draw(im, "RGBA")
    f, fs = font(24), font(20)
    for (mx, my) in MOTORS.values():
        for s in (1, -1):
            c = to(mx, s * my); r = PROP_R * k
            dr.ellipse([c[0] - r, c[1] - r, c[0] + r, c[1] + r], fill=(120, 150, 220, 60), outline=(60, 90, 170, 255), width=3)
            dr.ellipse([c[0] - 15 * k, c[1] - 15 * k, c[0] + 15 * k, c[1] + 15 * k], fill=(70, 72, 80, 255))
    hw = d["body_w"] / 2
    body = [(108, -30), (108, 30), (75, 43), (40, 37), (0, hw), (-34.5, hw), (-47, 45.5), (-90, 45.5), (-99, 30), (-99, -30),
            (-90, -45.5), (-47, -45.5), (-34.5, -hw), (0, -hw), (40, -37), (75, -43)]
    dr.polygon([to(*q) for q in body], fill=(200, 202, 208, 255), outline=(90, 90, 100, 255))
    for (cx_, cy_, rr) in ((0, 0, 7.6), (73, 28, 7.5), (73, -28, 7.5)):
        c = to(cx_, cy_); dr.ellipse([c[0] - rr * k, c[1] - rr * k, c[0] + rr * k, c[1] + rr * k], fill=(25, 25, 28, 255))
    a, b = to(-71, 25), to(-80, -25); dr.rectangle([a[0], a[1], b[0], b[1]], fill=(235, 235, 230, 255), outline=(90, 90, 100, 255))
    sg, L = d["sensor_gap"], d["saddle_len"]
    def rect(x0, x1, hy, fill, outline, w=4):
        a, b = to(x1, hy), to(x0, -hy); dr.rectangle([a[0], a[1], b[0], b[1]], fill=fill, outline=outline, width=w)
    rect(-sg - 60, -sg, 42, (40, 160, 170, 70), (20, 110, 120, 255))                      # payload zone 84 x 60
    rect(-sg - L, -sg, d["deck_w"] / 2, (240, 120, 30, 110), (200, 80, 0, 255))
    bx = -sg - L / 2 + d["beam_x"]
    rect(bx - d["beam_w"] / 2, bx + d["beam_w"] / 2, d["ear_out"], (240, 120, 30, 110), (200, 80, 0, 255))
    # clearance from the rear corner of the payload zone to the rear disc
    corner = (-sg - 60, 42.0); mx, my = MOTORS["rear"]
    v = (corner[0] - mx, corner[1] - my); n = math.hypot(*v); edge = (mx + v[0] / n * PROP_R, my + v[1] / n * PROP_R)
    dr.line([to(*corner), to(*edge)], fill=(190, 20, 20, 255), width=4)
    dr.text((to(*edge)[0] - 250, to(*edge)[1] + 6), f"about {n - PROP_R:.0f} mm", fill=(170, 10, 10, 255), font=f)
    dr.text((30, 22), "Top view: propeller discs (267 mm) and the mount", fill=(20, 20, 20, 255), font=f)
    q = to(125, 0); dr.text((q[0] - 28, q[1] - 30), "nose", fill=(60, 60, 60, 255), font=fs)
    y0 = H - 150
    for i, (col, txt) in enumerate((((120, 150, 220), "propeller discs, positions measured from the FCC top photo (+/- 5-8 mm)"),
                                    ((240, 120, 30), "saddle and the cross beam with the clamp screws"),
                                    ((40, 160, 170), "zone for the payload box: 84 x 60 mm; its rear corners are the closest point to the propellers"))):
        dr.rectangle([30, y0 + i * 40, 62, y0 + i * 40 + 24], fill=col + (255,), outline=(60, 60, 60, 255))
        dr.text((76, y0 + i * 40), txt, fill=(20, 20, 20, 255), font=fs)
    im.save(out)


if __name__ == "__main__":
    override = json.loads(pathlib.Path(sys.argv[1]).read_text()) if len(sys.argv) > 1 else {}
    d, parts = m.build(override)
    out = pathlib.Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    overlay(d, out / "preview_on_photos.png")
    section(d, parts, out / "preview_section.png")
    prop_plan(d, out / "preview_propellers.png")

    ORANGE, GREY, DARK, STEEL, BLACK, WHITE, BRASS = ((240, 120, 30), (200, 202, 208), (70, 72, 80), (175, 180, 190),
                                                      (25, 25, 28), (235, 235, 230), (205, 165, 60))
    body = m.body_proxy(d)
    cams, vents = m.body_marks(d)
    L = d["saddle_len"]
    sad, yk = mesh(parts["saddle"]), mesh(parts["yoke"])
    bod, scr = mesh(body, 0.15), mesh(m.screws(d), 0.05)
    cm, vt = mesh(cams, 0.1), mesh(vents, 0.1)
    base = [(*bod, GREY), (*cm, BLACK), (*vt, WHITE)]
    ey, sx = d["ear_y"], d["sensor_x"]
    ins = None
    for sy in (1, -1):                       # inserts, drawn as brass sleeves (preview only)
        zi = d["flange_top"] - d["flange_t"]
        a_ = m._cyl_z(d["beam_x"], sy * ey, zi - 0.4, zi + 4.0, 2.4)
        b_ = cq.Workplane("YZ").circle(2.4).extrude(4.0).translate((-L / 2 - 0.4, sy * d["lock_y"], d["lock_z"]))
        ins = a_.union(b_) if ins is None else ins.union(a_).union(b_)
    ins = mesh(ins, 0.05)

    # 1) assembled, 3/4 rear view
    items = base + [(*yk, DARK), (*sad, ORANGE), (*scr, STEEL), (*ins, BRASS)]
    im, px = render(items, eye_dir=(-0.75, -0.9, 0.62))
    label(im, px, (d["beam_x"], -d["ya"] - 5, -19), "arm with an air slot in front of the vent", (150, -40))
    label(im, px, (d["beam_x"], -d["yk"] - 4, -65), "U-clamp, rubber-lined", (60, 170))
    label(im, px, (d["beam_x"], -d["yk"] - d["upright_t"] - 6, d["yoke_bot"] + 17), "tie-down eyelet (one per side)", (-380, 110))
    label(im, px, (sx, 0, 2), "centre top camera - stays uncovered", (150, -110))
    label(im, px, (sx - 76, 0, 1), "exhaust grille - stays uncovered", (-30, 130))
    label(im, px, (sx - 38, -d["body_w"] / 2, -21), "side air vent", (-60, 230))
    im.save(out / "preview_assembly.png")

    # 2) exploded
    drop = 26
    yk_x = (yk[0] + np.array([0, 0, -drop]), yk[1])
    scr_x = (scr[0] + np.array([0, 0, 22]), scr[1])
    items = base + [(*yk_x, DARK), (*sad, ORANGE), (*scr_x, STEEL)]
    im, px = render(items, eye_dir=(-0.8, -0.75, 0.75))
    label(im, px, (d["beam_x"], -ey, d["ear_bot"] + 30), "2 x M3x20 clamp screws", (-330, -30))
    label(im, px, (L / 2 - 4, d["rail_w"] / 2 + 2, d["deck_top"]), "dovetail channel for the payload rail", (120, -90))
    label(im, px, (d["beam_x"], -d["yk"] - 4, -65 - drop), "clamp comes up from below", (60, 110))
    label(im, px, (d["beam_x"], -ey, d["flange_top"] - d["flange_t"] - drop), "insert pressed in from BELOW the flange", (-300, 80))
    im.save(out / "preview_exploded.png")

    # 3) the test ring as it prints
    g = parts["ring_gauge"].rotate((0, 0, 0), (0, 1, 0), -90)
    v, t = mesh(g, 0.05)
    im, px = render([(v, t, (240, 150, 60))], eye_dir=(0.25, -0.35, 1), up=(0, 1, 0), size=(1400, 1000))
    ImageDraw.Draw(im).text((30, 24), "ring_gauge_print.stl - two flat pieces, 3 mm thick, as they lie on the print bed",
                            fill=(20, 20, 20), font=font(26))
    im.save(out / "preview_ring_gauge.png")

    # 4) the stepped feeler
    v, t_ = mesh(m.gap_stair(), 0.03)
    im, px = render([(v, t_, (240, 150, 60))], eye_dir=(-0.5, -1, 0.9), size=(1400, 760))
    ImageDraw.Draw(im).text((30, 24), "gap_stair_print.stl - steps of 2 to 16 mm, number engraved on every step",
                            fill=(20, 20, 20), font=font(26))
    im.save(out / "preview_gap_stair.png")
    print("ok")
