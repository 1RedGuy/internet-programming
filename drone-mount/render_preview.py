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


if __name__ == "__main__":
    override = json.loads(pathlib.Path(sys.argv[1]).read_text()) if len(sys.argv) > 1 else {}
    d, parts = m.build(override)
    out = pathlib.Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)

    ORANGE, TEAL, GREY, DARK, BLUE, STEEL = ((240, 120, 30), (40, 130, 140), (205, 205, 205),
                                             (70, 72, 80), (150, 185, 235), (175, 180, 190))
    body = m.body_proxy(d)
    L = d["saddle_len"]
    box_h = 30.0
    zt = d["tray_z0"] + d["tray_t"]
    payload = (cq.Workplane("XY").box(70, 60, box_h)
               .translate((L / 2 + d["tray_front"] - d["tray_l"] / 2, 0, zt + box_h / 2)).edges().fillet(3))

    sad, tr = mesh(parts["saddle"]), mesh(parts["tray"])
    y1, y2 = mesh(parts["yoke"]), mesh(parts["yoke2"])
    bod, pl, scr = mesh(body, 0.15), mesh(payload, 0.15), mesh(m.screws(d), 0.05)
    ey, zear = d["ear_y"], -d["skirt_drop"] + d["ear_t"]

    # 1) assembled, 3/4 rear view
    items = [(*bod, GREY), (*y1, DARK), (*y2, DARK), (*sad, ORANGE), (*tr, TEAL), (*pl, BLUE), (*scr, STEEL)]
    im, px = render(items, eye_dir=(-0.75, -0.9, 0.62))
    label(im, px, (L / 2 - 4, -d["deck_w"] / 2, d["deck_z"] - 2), "saddle", (150, -40))
    label(im, px, (-L / 2 + 5, -d["tray_w"] / 2, zt - 1), "tray (slides on from the rear)", (-260, -90))
    label(im, px, (d["yoke_x"], -d["wo"] / 2, -45), "2 x U-clamp under the belly", (60, 130))
    label(im, px, (-d["yoke_x"], -ey, zear + 3), "4 x M3x16 + nut", (-170, 150))
    label(im, px, (L / 2 - d["tray_l"] / 2, 0, zt + box_h), "your payload (example box)", (120, -70))
    label(im, px, (70, 0, 0), "nose  ->", (40, -50))
    im.save(out / "preview_assembly.png")

    # 2) exploded
    lift, drop = 26, 30
    tr_x = (tr[0] + np.array([-38, 0, lift]), tr[1])
    y1_x, y2_x = (y1[0] + np.array([0, 0, -drop]), y1[1]), (y2[0] + np.array([0, 0, -drop]), y2[1])
    scr_x = (scr[0] + np.array([0, 0, 14]), scr[1])
    items = [(*bod, GREY), (*y1_x, DARK), (*y2_x, DARK), (*sad, ORANGE), (*tr_x, TEAL), (*scr_x, STEEL)]
    im, px = render(items, eye_dir=(-0.8, -0.75, 0.75))
    label(im, px, (-L / 2 - 43, 0, d["lock_z"] + lift), "M3x12 tray lock screw", (-150, 90))
    label(im, px, (L / 2 - 6, d["rail_w"] / 2 - 2, d["rail_top_z"]), "dovetail rail", (170, -110))
    label(im, px, (L / 2 - 10 - 38, d["tray_w"] / 2 - 9, zt + lift), "M3 holes + slots for the payload", (60, -70))
    label(im, px, (d["yoke_x"], -d["wo"] / 2, -50 - drop), "clamp comes up from below", (90, 110))
    label(im, px, (-d["yoke_x"], -ey, d["yoke_top_z"] - drop), "captive nut under the flange", (-330, 60))
    im.save(out / "preview_exploded.png")

    # 3) front view with dimensions (payload hidden)
    items = [(*bod, GREY), (*y1, DARK), (*sad, ORANGE), (*tr, TEAL), (*scr, STEEL)]
    im, px = render(items, eye_dir=(1, 0, 0), size=(1500, 1150), margin=0.17)
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

    bw, bh = d["body_w"], d["body_h"]
    dim((0, -bw / 2, -bh * 0.72), (0, bw / 2, -bh * 0.72), f"body_w = {bw:g}  (MEASURE)")
    dim((0, -14, 0), (0, -14, -bh), f"body_h = {bh:g}  (MEASURE)", side=-1)
    dim((0, d["wo"] / 2 + d["ear_out"] + 6, 0), (0, d["wo"] / 2 + d["ear_out"] + 6, zt), f"{zt:.1f} mm up")
    dim((0, -d["wo"] / 2 - d["ear_out"], -bh - 16), (0, d["wo"] / 2 + d["ear_out"], -bh - 16),
        f"overall {d['wo'] + 2 * d['ear_out']:.0f} wide")
    dim((0, d["wo"] / 2 + 4, -bh), (0, d["wo"] / 2 + 4, d["yoke_bot_z"]),
        f"{-d['yoke_bot_z'] - bh:.1f} below belly", side=1)
    im.save(out / "preview_front.png")
    print("ok")
