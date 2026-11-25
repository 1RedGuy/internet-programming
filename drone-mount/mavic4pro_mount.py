"""
Parametric payload mount for DJI Mavic 4 Pro  (CadQuery)  - v3

Geometry is based on the FCC filing photos of the aircraft (FCC ID
SS3-L3AB2410, external + internal photos with rulers), scaled with DJI's
published unfolded size 328.7 x 390.5 mm.  Expect about +/-4 mm on the
fuselage numbers - print the ring gauge first.

What the photos show (positions measured from the centre top camera):
  * the fuselage shell is ~201 mm long, the centre top camera sits in the middle
  * the "waist" behind that camera is ~70 mm wide and 82-88 mm high, the roof
    is nearly flat over ~50 mm
  * 34 mm behind the camera the rear-arm fairings start (93 mm wide),
    66-80 mm behind it is the exhaust grille
  * on both sides, ~10-48 mm behind the camera and ~15-25 mm below the roof,
    are mesh air vents
  * the belly is a ~51 mm wide metal heat sink; lights and cameras occupy its
    front half, the downward camera is roughly under the centre top camera

Parts
  saddle     - short pad on the roof between the centre top camera and the
               exhaust grille, with one cross beam
  yoke       - one U-clamp that goes round the waist and is pulled up against
               the cross beam with two M3 screws into heat-set inserts.  It
               touches the belly through rubber; the side gap is a parameter
               that still has to be set from a measurement on the real drone
  tray       - payload plate, slides into the saddle's dovetail from the rear,
               locked with M3 screws
  ring_gauge - 3 mm slice of saddle beam + yoke to test the fit

Coordinates: X = nose, Y = left, Z = up.  Origin = roof surface under the
middle of the saddle.  Units: mm.

Usage:
  python mavic4pro_mount.py                 # default parameters
  python mavic4pro_mount.py my_params.json  # override any key of P
"""
import json
import math
import pathlib
import sys

import cadquery as cq

P = dict(
    # --- fuselage at the clamp station (from photos, verify with the gauge) ---
    body_w=70.0,        # width of the waist
    body_h=85.0,        # roof to underside of the belly (82-88: least certain number)
    roof_flat=50.0,     # nearly flat width of the roof
    belly_w=51.0,       # width of the metal plate under the belly (FCC photo)
    side_clear=7.0,     # gap between fuselage sides and the yoke (to be set from a real measurement)
    sensor_gap=12.0,    # saddle front edge to the CENTRE of the top camera
    # --- saddle -------------------------------------------------------------
    pad_top=3.0,        # soft pad between saddle and roof
    pad_belly=3.0,      # soft pad between yoke and belly
    saddle_len=50.0,
    deck_w=58.0,
    deck_t=7.0,
    beam_x=13.0,        # cross beam / yoke position (+ = towards the nose)
    beam_w=14.0,        # width of beam and yoke along the fuselage
    # --- clamp ----------------------------------------------------------------
    ear_drop=12.0,      # underside of the screw ears below the roof
    ear_len=14.0,       # ear length sideways
    clamp_gap=6.0,      # tightening travel between ear and yoke flange
    flange_t=8.0,
    upright_t=4.0,
    bar_t=4.0,          # yoke thickness under the belly
    yoke_rb=14.0,       # inner corner radius of the yoke
    # --- dovetail (channel in the saddle, rail under the tray) ----------------
    rail_w=30.0,        # channel width at the deck surface
    rail_h=4.5,
    rail_angle=60.0,
    fit_clear=0.25,     # per flank; raise to 0.35 if the tray is tight
    # --- tray -----------------------------------------------------------------
    tray_w=84.0,
    tray_l=60.0,
    tray_t=2.6,
    endwall_t=5.0,
    lock_y=23.5,        # lock screws left/right of the channel
    hole_d=3.4,         # M3 clearance
    insert_d=3.7,       # hole for M3 heat-set inserts with 4.0-4.2 mm outer diameter
    insert_depth=5.0,   # pocket depth (inserts up to 5 mm long)
)


def derived(p):
    d = dict(p)
    d["deck_top"] = p["pad_top"] + p["deck_t"]
    d["chan_bot"] = d["deck_top"] - p["rail_h"]
    d["chan_bot_hw"] = p["rail_w"] / 2 + p["rail_h"] / math.tan(math.radians(p["rail_angle"]))
    d["yi"] = p["body_w"] / 2 + p["side_clear"]          # inner half width of the yoke
    d["ear_y"] = d["yi"] + p["ear_len"] / 2              # clamp screw axis
    d["ear_out"] = d["yi"] + p["ear_len"]
    d["ear_top"] = -p["ear_drop"] + 7.0
    d["flange_top"] = -p["ear_drop"] - p["clamp_gap"]
    d["yoke_in_bot"] = -p["body_h"] - p["pad_belly"]
    d["yoke_bot"] = d["yoke_in_bot"] - p["bar_t"]
    d["tray_z0"] = d["deck_top"] + 0.15
    d["lock_z"] = d["deck_top"] - 3.5
    d["sensor_x"] = p["saddle_len"] / 2 + p["sensor_gap"]
    assert d["chan_bot"] - p["pad_top"] >= 2.0, "deck too thin under the dovetail channel"
    assert p["lock_y"] - 3.0 > d["chan_bot_hw"] + 1.0 and p["lock_y"] + 3.0 < p["deck_w"] / 2, "lock_y"
    assert abs(p["beam_x"]) + p["beam_w"] / 2 <= p["saddle_len"] / 2, "beam outside the saddle"
    assert 2 * d["yi"] - 2 * p["yoke_rb"] >= p["belly_w"], "yoke corners would touch the belly plate"
    assert p["flange_t"] - p["insert_depth"] >= 2.5, "flange too thin behind the insert"
    assert d["yi"] >= p["deck_w"] / 2 + 8, "fuselage too narrow for this deck_w"
    return d


def _prism_x(pts_yz, x0, x1):
    return (cq.Workplane("YZ").polyline(pts_yz).close().extrude(x1 - x0)
            .translate((x0, 0, 0)))


def _box(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0)
            .translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)))


def _cyl_z(x, y, z0, z1, r):
    return cq.Workplane("XY").circle(r).extrude(z1 - z0).translate((x, y, z0))


# --------------------------------------------------------------------------
def _beam(d, x0, x1):
    """Cross beam with the two screw ears (no holes). All slopes are 45 deg so the
    saddle prints upside down without supports."""
    hw, pad, top = d["deck_w"] / 2, d["pad_top"], d["deck_top"]
    yi, yo, eb, et = d["yi"], d["ear_out"], -d["ear_drop"], d["ear_top"]
    out = None
    for s in (+1, -1):
        y_knee = yi - (pad - et)        # underside: level at pad height, then 45 deg down to the ear
        y_top = yo - (top - et)         # top side: 45 deg from the ear up to the deck surface
        pts = [(s * (hw - 2), pad), (s * y_knee, pad), (s * yi, et), (s * yi, eb), (s * yo, eb),
               (s * yo, et), (s * y_top, top), (s * (hw - 2), top)]
        b = _prism_x(pts, x0, x1)
        out = b if out is None else out.union(b)
    return out


def saddle_blank(d, with_channel=True):
    L, hw, pad, top = d["saddle_len"], d["deck_w"] / 2, d["pad_top"], d["deck_top"]
    s = _box(-L / 2, L / 2, -hw, hw, pad, top)
    # relieve every edge of the pad face so printed plastic stays away from the roof
    c = 1.5
    for sx in (+1, -1):
        s = s.cut(_box(-c, c, -hw - 1, hw + 1, -c, c).rotate((0, 0, 0), (0, 1, 0), 45)
                  .translate((sx * L / 2, 0, pad)))
    for sy in (+1, -1):
        s = s.cut(_box(-L / 2 - 1, L / 2 + 1, -c, c, -c, c).rotate((0, 0, 0), (1, 0, 0), 45)
                  .translate((0, sy * hw, pad)))
    s = s.union(_beam(d, d["beam_x"] - d["beam_w"] / 2, d["beam_x"] + d["beam_w"] / 2))
    if with_channel:
        a, b = d["rail_w"] / 2, d["chan_bot_hw"]
        s = s.cut(_prism_x([(-a, top + 0.5), (a, top + 0.5), (a, top), (b, d["chan_bot"]),
                            (-b, d["chan_bot"]), (-a, top)], -L / 2 - 1, L / 2 + 1))
    return s


def saddle(d):
    L = d["saddle_len"]
    s = saddle_blank(d)
    eb = -d["ear_drop"]
    xr = -L / 2
    for sy in (+1, -1):
        # clamp screws: through hole + flat seat for the screw head
        s = s.cut(_cyl_z(d["beam_x"], sy * d["ear_y"], eb - 1, d["deck_top"] + 1, d["hole_d"] / 2))
        s = s.cut(_cyl_z(d["beam_x"], sy * d["ear_y"], eb + 5.0, d["deck_top"] + 1, 3.3))
        # lock screws for the future payload rail: M3 heat-set inserts pressed in from the rear face
        for dia, depth in ((d["insert_d"], d["insert_depth"]), (d["insert_d"] + 0.8, 0.6), (3.2, 10.5)):
            s = s.cut(cq.Workplane("YZ").circle(dia / 2).extrude(depth + 0.1)
                      .translate((xr - 0.1, sy * d["lock_y"], d["lock_z"])))
    return s


def yoke(d, x0=None, x1=None, holes=True):
    """U-clamp round the waist: a plain U, the same width top to bottom.
    Print lying on its side (profile on the bed)."""
    if x0 is None:
        x0, x1 = d["beam_x"] - d["beam_w"] / 2, d["beam_x"] + d["beam_w"] / 2
    yu, t = d["yi"], d["upright_t"]
    zt, zib, zb = d["flange_top"], d["yoke_in_bot"], d["yoke_bot"]
    outer = _box(x0, x1, -yu - t, yu + t, zb, zt).edges("|X").edges("<Z").fillet(d["yoke_rb"] + t)
    inner = _box(x0 - 1, x1 + 1, -yu, yu, zib, zt + 5).edges("|X").edges("<Z").fillet(d["yoke_rb"])
    y = outer.cut(inner)
    ft = d["flange_t"]
    for sy in (+1, -1):
        ya, yb = sorted((sy * (yu + t - 0.5), sy * d["ear_out"]))
        y = y.union(_box(x0, x1, ya, yb, zt - ft, zt))
        yo = sy * (yu + t)
        y = y.union(_prism_x([(yo, zt - ft + 0.1), (yo + sy * 3.5, zt - ft + 0.1), (yo, zt - ft - 3.5)], x0, x1))
    try:                                   # break every edge - friendlier to hands and to the drone
        y = y.edges().chamfer(0.4)
    except Exception:
        pass
    if holes:
        xc = (x0 + x1) / 2
        for sy in (+1, -1):
            # screw passes through a plastic shoulder, the insert is pressed in from BELOW,
            # so the screw pulls it into the part instead of out of it
            y = y.cut(_cyl_z(xc, sy * d["ear_y"], zt - ft - 1, zt + 1, d["hole_d"] / 2))
            y = y.cut(_cyl_z(xc, sy * d["ear_y"], zt - ft - 8.0, zt - ft + d["insert_depth"], d["insert_d"] / 2))
            y = y.cut(_cyl_z(xc, sy * d["ear_y"], zt - ft - 8.0, zt - ft + 0.6, d["insert_d"] / 2 + 0.4))
    return y


def tray(d):
    L, hw = d["saddle_len"], d["deck_w"] / 2
    xf, xb = L / 2, L / 2 - d["tray_l"]
    z0, zt = d["tray_z0"], d["tray_z0"] + d["tray_t"]
    wall_x1 = -L / 2 - 0.3
    wall_x0 = wall_x1 - d["endwall_t"]
    assert xb <= wall_x0 + 1e-6, "tray_l too short: tray must reach behind the saddle"

    t = _box(xb, xf, -d["tray_w"] / 2, d["tray_w"] / 2, z0, zt).edges("|Z").fillet(6.0)

    # male dovetail under the plate (two flanks, hollow middle)
    c = d["fit_clear"] / math.sin(math.radians(d["rail_angle"]))
    k = 1 / math.tan(math.radians(d["rail_angle"]))
    zr = d["chan_bot"] + 0.4
    hr, ht = d["rail_w"] / 2 - c, d["rail_w"] / 2 + (d["deck_top"] - zr) * k - c
    rail = _prism_x([(-hr, z0 + 0.01), (hr, z0 + 0.01), (ht, zr), (-ht, zr)], wall_x1, xf)
    rail = rail.cut(_box(wall_x1 + 6, xf + 1, -(hr - 7.0), hr - 7.0, zr - 1, z0))
    # stop / lock wall behind the saddle
    wall = _box(wall_x0, wall_x1, -hw, hw, d["lock_z"] - 3.5, z0 + 0.01)
    t = t.union(rail).union(wall)
    for sy in (+1, -1):
        t = t.cut(cq.Workplane("YZ").circle(d["hole_d"] / 2).extrude(d["endwall_t"] + 2)
                  .translate((wall_x0 - 1, sy * d["lock_y"], d["lock_z"])))

    # payload fixing.  The cross beam leaves no room under the plate, so holes and
    # slots sit behind it: per side one M3 hole, one strap slot, one M3 hole.
    x_free = d["beam_x"] - d["beam_w"] / 2 - 2.0           # rearward of this the underside is open
    y_hole = d["tray_w"] / 2 - 7.0
    y_slot = d["tray_w"] / 2 - 3.5 - 1.6
    x_h1 = x_free - 3.0
    x_s = x_h1 - 5.0 - 11.0
    x_h2 = x_s - 11.0 - 5.0
    assert x_h2 > xb + 3.5, "tray_l too short for the hole / slot / hole pattern"
    for sy in (+1, -1):
        for x in (x_h1, x_h2):
            t = t.cut(_cyl_z(x, sy * y_hole, z0 - 1, zt + 1, d["hole_d"] / 2))
        t = t.cut(_box(x_s - 11, x_s + 11, sy * y_slot - 1.6, sy * y_slot + 1.6, z0 - 1, zt + 1))
    return t


GAUGE_H = 80.0            # the gauge yoke is made for this height, so its gap = clamp_gap + (real height - 80)
YOKE_SIZES = (82.0, 85.0, 88.0)


def ring_gauge(d):
    x0, x1 = d["beam_x"] - 1.5, d["beam_x"] + 1.5
    top = saddle_blank(d, with_channel=False).intersect(_box(x0, x1, -200, 200, -200, 200))
    dg = derived({k: v for k, v in d.items() if k in P} | dict(body_h=GAUGE_H))
    return top.union(yoke(dg, x0, x1, holes=False))


# ----------------------------------------------------------------------------
# Preview helpers (not printed)
def body_proxy(d):
    """Approximate fuselage built from the photo measurements - preview only."""
    sx = d["sensor_x"]
    hw, rf, bw, h = d["body_w"] / 2, d["roof_flat"] / 2, d["belly_w"] / 2, d["body_h"]
    rs = hw - rf
    pts = [(rf, 0.0)]
    for a in range(15, 91, 15):
        pts.append((rf + rs * math.sin(math.radians(a)), -rs + rs * math.cos(math.radians(a))))
    pts += [(hw, -h + 22), (bw, -h + 6), (bw, -h)]        # lower corners: shape NOT verified
    pts = pts + [(-y, z) for y, z in reversed(pts)]
    body = _prism_x(pts, sx - 101, sx + 100)
    for s in (+1, -1):                                   # rear-arm fairings
        hip = (cq.Workplane("XY")
               .polyline([(sx - 96, s * 46.5), (sx - 47, s * 46.5), (sx - 34, s * (hw - 1)), (sx - 96, s * (hw - 1))])
               .close().extrude(34).translate((0, 0, -58)))
        body = body.union(hip)
    return body


def body_marks(d):
    sx, hw, h = d["sensor_x"], d["body_w"] / 2, d["body_h"]
    cams = _cyl_z(sx, 0, -0.5, 2.0, 7.8)
    for s in (+1, -1):
        cams = cams.union(_cyl_z(sx + 68, s * 26, -0.5, 2.0, 7.5))
        cams = cams.union(_cyl_z(sx + 60, s * 17, -h - 1.5, -h + 0.5, 7.5))
    cams = cams.union(_cyl_z(sx, 0, -h - 1.5, -h + 0.5, 8.0))              # downward camera
    cams = cams.union(_cyl_z(sx + 13, 0, -h - 1.0, -h + 0.5, 4.0))         # auxiliary light
    vents = _box(sx - 81, sx - 72, -24, 24, -1.0, 1.0)                     # exhaust grille
    for s in (+1, -1):
        y0, y1 = sorted((s * (hw - 0.5), s * (hw + 0.6)))
        vents = vents.union(_box(sx - 48, sx - 10, y0, y1, -25, -17))      # side mesh vents
    return cams, vents


def screws(d):
    out = None
    for sy in (+1, -1):
        zh = -d["ear_drop"] + 5.0
        s = (_cyl_z(d["beam_x"], sy * d["ear_y"], zh - 20, zh, 1.5)
             .union(_cyl_z(d["beam_x"], sy * d["ear_y"], zh, zh + 3, 2.75)))
        out = s if out is None else out.union(s)
    return out


def volume_cm3(wp):
    return wp.val().Volume() / 1000.0


def build(p=None):
    d = derived({**P, **(p or {})})
    return d, dict(saddle=saddle(d), yoke=yoke(d), tray=tray(d), ring_gauge=ring_gauge(d))


if __name__ == "__main__":
    override = json.loads(pathlib.Path(sys.argv[1]).read_text()) if len(sys.argv) > 1 else {}
    d, parts = build(override)
    out = pathlib.Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    for old in out.glob("*"):
        if old.suffix in (".step", ".stl"):
            old.unlink()

    flip = lambda wp: wp.rotate((0, 0, 0), (1, 0, 0), 180)       # top face down on the bed
    on_side = lambda wp: wp.rotate((0, 0, 0), (0, 1, 0), -90)    # profile on the bed
    printable = dict(saddle=flip(parts["saddle"]), yoke=on_side(parts["yoke"]),
                     tray=flip(parts["tray"]), ring_gauge=on_side(parts["ring_gauge"]))
    for name, wp in printable.items():
        if name in ("yoke", "tray"):      # the payload rail / box comes later
            continue
        cq.exporters.export(parts[name], str(out / f"{name}.step"))
        cq.exporters.export(wp, str(out / f"{name}_print.stl"), tolerance=0.02, angularTolerance=0.1)
    for hgt in sorted(set(YOKE_SIZES) | {d["body_h"]}):
        dy = derived({k: v for k, v in d.items() if k in P} | dict(body_h=hgt))
        y = yoke(dy)
        cq.exporters.export(y, str(out / f"yoke_H{hgt:g}.step"))
        cq.exporters.export(on_side(y), str(out / f"yoke_H{hgt:g}_print.stl"), tolerance=0.02, angularTolerance=0.1)

    asm = cq.Assembly()
    asm.add(parts["saddle"], name="saddle", color=cq.Color(0.95, 0.45, 0.1))
    asm.add(parts["yoke"], name="yoke", color=cq.Color(0.2, 0.2, 0.22))
    asm.add(body_proxy(d), name="fuselage_APPROX_from_photos", color=cq.Color(0.8, 0.8, 0.8))
    asm.export(str(out / "assembly.step"))

    print(f"tray top {d['tray_z0'] + d['tray_t']:.1f} mm above the roof, "
          f"yoke {-d['yoke_bot'] - d['body_h']:.1f} mm below the belly, "
          f"overall width {2 * d['ear_out']:.0f} mm, yoke opening {2 * d['yi']:.0f} x {-d['yoke_in_bot'] + d['flange_top']:.0f}")
    tot = [0.0, 0.0]
    for name in ("saddle", "yoke"):
        v = volume_cm3(parts[name]); bb = parts[name].val().BoundingBox()
        tot[0] += v * 1.27 * 0.8; tot[1] += v * 1.27
        print(f"{name:7s} {v:5.1f} cm3 -> about {v * 1.27 * 0.8:.0f}-{v * 1.27:.0f} g in PETG  "
              f"bbox {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f}")
    print(f"total about {tot[0]:.0f}-{tot[1]:.0f} g (+ 2 screws and 4 inserts ~3 g)")
