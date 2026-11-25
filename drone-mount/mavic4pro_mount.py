"""
Parametric payload mount for DJI Mavic 4 Pro  (CadQuery)  - v5

Where the numbers come from
---------------------------
FCC filing photos of the aircraft (FCC ID SS3-L3AB2410) with a steel ruler on
the table.  The top photo is strongly oblique, so a single scale is wrong; the
measurements below use the table homography from ~65 ruler ticks plus the four
propellers as known lengths (267 mm diameter) to get the height scaling.
Checks of that model: the height scaling agrees left/right within 1.5 %, the
blade pivot spacing comes out 19.5-21.4 mm on all four propellers, and the
front motor span comes out 361 mm (published overall width 390.5 mm minus the
motor diameter).  See measure/top_view_measure.py.

Measured (positions are behind the CENTRE top camera, along the fuselage):
  waist width                      68-71 mm   (-> body_w = 70)
  roof free strip                  camera .. 66 mm (start of the exhaust housing)
  rear-arm fairings (91 mm wide)   start at 34-35 mm
  side mesh vents                  about 8 .. 42 mm, 15-26 mm below the roof
  fuselage shell                   108 mm ahead of the camera, 99 mm behind it
  height at the waist              88-91 mm from the owner's test ring on the real drone
                                   (the oblique side photos alone had suggested 80 +/- 4)

The owner tried a printed ring (84 mm opening) on the real drone: the top
piece sat well and there was a gap of "almost a finger" on both sides, which
matches 7 mm per side for a 70 mm body.

Parts
  saddle     - pad on the roof behind the centre camera, with a cross beam whose
               two arms reach down past the side vents.  The arms stand 7 mm off
               the body and are slotted where they pass the vents.
  yoke       - U-clamp under the belly, pulled up against the arms with two M3
               screws into heat-set inserts.  It leaves 4 mm per side for a
               rubber seal, so it sits snug on the sides.
  ring_gauge - 3 mm slice of beam + yoke to test the fit
  gap_stair  - stepped feeler (2-16 mm) to read gaps without a caliper

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
    # --- fuselage at the clamp station ----------------------------------------
    body_w=70.0,        # width of the waist (photogrammetry: 68-71)
    body_h=90.0,        # roof to belly at the waist (owner's test ring: 88-91)
    roof_flat=50.0,     # nearly flat width of the roof
    belly_flat=48.0,    # flat width of the belly (the metal plate)
    sensor_gap=12.0,    # saddle front edge to the CENTRE of the top camera
    vent_top=10.0,      # the side vents lie between these depths below the roof
    vent_bot=28.0,      #   (measured 15-26, with margin)
    # --- fits -----------------------------------------------------------------
    pad_top=3.0,        # rubber between saddle and roof
    pad_belly=3.0,      # rubber between yoke and belly
    arm_clear=7.0,      # air between the body sides and the saddle arms (vents!)
    side_gap=4.0,       # gap between the body sides and the yoke, filled with rubber
    # --- saddle -------------------------------------------------------------
    saddle_len=50.0,
    deck_w=58.0,
    deck_t=7.0,
    beam_x=15.0,        # cross beam / yoke position (+ = towards the nose)
    beam_w=14.0,        # width of beam, arms and yoke along the fuselage
    arm_t=5.0,          # arm thickness
    vent_slot=6.0,      # width of the air slot in each arm
    # --- clamp ----------------------------------------------------------------
    ear_drop=34.0,      # underside of the screw ears below the roof (below the vents)
    ear_len=14.0,
    clamp_gap=7.0,      # tightening travel between ear and yoke flange
    flange_t=8.0,
    upright_t=4.0,
    bar_t=4.0,          # yoke thickness under the belly
    yoke_rb=8.0,        # inner corner radius of the yoke (small, so it cannot ride on the body's lower corners)
    eye_d=4.6,          # hole of the two tie-down eyelets on the sides of the yoke
    eye_out=10.0,       # how far the eyelets stick out sideways
    # --- dovetail channel for the future payload rail -------------------------
    rail_w=30.0,
    rail_h=4.5,
    rail_angle=60.0,
    lock_y=23.5,
    # --- fasteners ------------------------------------------------------------
    hole_d=3.4,         # M3 clearance
    insert_d=3.7,       # hole for M3 heat-set inserts with 4.0-4.2 mm outer diameter
    insert_depth=5.0,
)

GAUGE_H = 76.0            # the gauge yoke is made for this height: its gap = clamp_gap + (real height - 76)
YOKE_SIZES = (85.0, 88.0, 90.0, 92.0)


def derived(p):
    d = dict(p)
    hw = p["body_w"] / 2
    d["deck_top"] = p["pad_top"] + p["deck_t"]
    d["chan_bot"] = d["deck_top"] - p["rail_h"]
    d["chan_bot_hw"] = p["rail_w"] / 2 + p["rail_h"] / math.tan(math.radians(p["rail_angle"]))
    d["ya"] = hw + p["arm_clear"]                 # inner face of the saddle arms
    d["yk"] = hw + p["side_gap"]                  # inner face of the yoke uprights
    d["ear_y"] = d["yk"] + p["upright_t"] + 6.5   # clamp screw axis
    d["ear_out"] = d["ear_y"] + p["ear_len"] / 2
    d["ear_bot"] = -p["ear_drop"]
    d["flange_top"] = d["ear_bot"] - p["clamp_gap"]
    d["yoke_in_bot"] = -p["body_h"] - p["pad_belly"]
    d["yoke_bot"] = d["yoke_in_bot"] - p["bar_t"]
    d["lock_z"] = d["deck_top"] - 3.5
    d["sensor_x"] = p["saddle_len"] / 2 + p["sensor_gap"]
    assert d["chan_bot"] - p["pad_top"] >= 2.0, "deck too thin under the dovetail channel"
    assert p["lock_y"] - 3.0 > d["chan_bot_hw"] + 1.0 and p["lock_y"] + 3.0 < p["deck_w"] / 2, "lock_y"
    assert abs(p["beam_x"]) + p["beam_w"] / 2 <= p["saddle_len"] / 2, "beam outside the saddle"
    assert d["ya"] >= p["deck_w"] / 2 + 6, "fuselage too narrow for this deck_w"
    assert p["ear_drop"] >= p["vent_bot"] + 5, "ears must sit below the vents"
    assert d["ear_y"] - 3.3 >= d["ya"] + p["arm_t"] - 1.2, "screw head would cut too far into the arm"
    assert 2 * d["yk"] - 2 * p["yoke_rb"] >= p["belly_flat"], "yoke corners would touch the belly"
    assert p["flange_t"] - p["insert_depth"] >= 2.5, "flange too thin behind the insert"
    assert d["flange_top"] - p["flange_t"] - 10 > d["yoke_in_bot"] + p["yoke_rb"], "fuselage too low for this ear_drop"
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
    """Cross beam with two arms that hang down past the vents and end in screw ears.
    Every sloped face is 45 deg, so the saddle prints upside down without supports."""
    hw, pad, top = d["deck_w"] / 2, d["pad_top"], d["deck_top"]
    ya, ta, yo, eb = d["ya"], d["arm_t"], d["ear_out"], d["ear_bot"]
    zk = -5.0                                  # where the inner 45 deg run reaches the arm
    c = ta * math.sqrt(2)                      # keeps the wall thickness through the bend
    out = None
    for s in (+1, -1):
        y_knee = ya - (pad - zk)
        pts = [(hw - 2, pad), (y_knee, pad), (ya, zk), (ya, eb), (yo, eb), (yo, eb + 7.0),
               (ya + ta, eb + 7.0 + (yo - ya - ta)), (ya + ta, zk - ta + c), (ya + zk + c - top, top), (hw - 2, top)]
        b = _prism_x([(s * y, z) for y, z in pts], x0, x1)
        out = b if out is None else out.union(b)
    return out


def saddle_blank(d, with_channel=True):
    L, hw, pad, top = d["saddle_len"], d["deck_w"] / 2, d["pad_top"], d["deck_top"]
    s = _box(-L / 2, L / 2, -hw, hw, pad, top)
    c = 1.5                                    # relieve every edge of the pad face
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
    eb, xr = d["ear_bot"], -L / 2
    for sy in (+1, -1):
        # clamp screws: through hole + flat seat for the screw head (reached from above)
        s = s.cut(_cyl_z(d["beam_x"], sy * d["ear_y"], eb - 1, d["deck_top"] + 1, d["hole_d"] / 2))
        s = s.cut(_cyl_z(d["beam_x"], sy * d["ear_y"], eb + 5.0, d["deck_top"] + 1, 3.3))
        # air slot through each arm where it passes the side vent
        ya, yb = sorted((sy * (d["ya"] - 1), sy * (d["ya"] + d["arm_t"] + 1)))
        s = s.cut(_box(d["beam_x"] - d["vent_slot"] / 2, d["beam_x"] + d["vent_slot"] / 2, ya, yb,
                       -d["vent_bot"], -d["vent_top"]))
        # M3 heat-set inserts for the future payload rail, pressed in from the rear face
        for dia, depth in ((d["insert_d"], d["insert_depth"]), (d["insert_d"] + 0.8, 0.6), (3.2, 10.5)):
            s = s.cut(cq.Workplane("YZ").circle(dia / 2).extrude(depth + 0.1)
                      .translate((xr - 0.1, sy * d["lock_y"], d["lock_z"])))
    return s


def yoke(d, x0=None, x1=None, holes=True):
    """U-clamp under the belly, a plain U. Print lying on its side (profile on the bed)."""
    if x0 is None:
        x0, x1 = d["beam_x"] - d["beam_w"] / 2, d["beam_x"] + d["beam_w"] / 2
    yk, t = d["yk"], d["upright_t"]
    zt, zib, zb = d["flange_top"], d["yoke_in_bot"], d["yoke_bot"]
    outer = _box(x0, x1, -yk - t, yk + t, zb, zt).edges("|X").edges("<Z").fillet(d["yoke_rb"] + t)
    inner = _box(x0 - 1, x1 + 1, -yk, yk, zib, zt + 5).edges("|X").edges("<Z").fillet(d["yoke_rb"])
    y = outer.cut(inner)
    ft = d["flange_t"]
    for sy in (+1, -1):
        ya, yb = sorted((sy * (yk + t - 0.5), sy * d["ear_out"]))
        y = y.union(_box(x0, x1, ya, yb, zt - ft, zt))
        yo = sy * (yk + t)
        y = y.union(_prism_x([(yo, zt - ft + 0.1), (yo + sy * 3.5, zt - ft + 0.1), (yo, zt - ft - 3.5)], x0, x1))
        # tie-down eyelet: on the side, above the bottom of the yoke, so it costs no ground clearance
        ez0 = zb + d["yoke_rb"] + t
        ea, eb_ = sorted((sy * (yk + t - 0.5), sy * (yk + t + d["eye_out"])))
        y = y.union(_box(x0, x1, ea, eb_, ez0, ez0 + 9.5))
    try:                                   # break every edge - friendlier to hands and to the drone
        y = y.edges().chamfer(0.4)
    except Exception:
        pass
    if holes:
        xc = (x0 + x1) / 2
        for sy in (+1, -1):
            # the screw passes a plastic shoulder; the insert is pressed in from BELOW,
            # so the screw pulls it into the part instead of out of it
            y = y.cut(_cyl_z(xc, sy * d["ear_y"], zt - ft - 1, zt + 1, d["hole_d"] / 2))
            y = y.cut(_cyl_z(xc, sy * d["ear_y"], zt - ft - 8.0, zt - ft + d["insert_depth"], d["insert_d"] / 2))
            y = y.cut(_cyl_z(xc, sy * d["ear_y"], zt - ft - 8.0, zt - ft + 0.6, d["insert_d"] / 2 + 0.4))
            ez = zb + d["yoke_rb"] + t + 4.75
            ey_ = sy * (yk + t + d["eye_out"] / 2 + 0.3)
            y = y.cut(cq.Workplane("YZ").circle(d["eye_d"] / 2).extrude(x1 - x0 + 2).translate((x0 - 1, ey_, ez)))
            for xe in (x0, x1):            # soften the hole edges so a cord does not chafe
                y = y.cut(cq.Workplane("YZ").circle(d["eye_d"] / 2 + 0.6).extrude(1.2).translate((xe - 0.6, ey_, ez)))
        # two notches in the middle of the belly bar: a cord tied round the bar stays centred
        for xa, xb in ((x0 - 1, x0 + 1.5), (x1 - 1.5, x1 + 1)):
            y = y.cut(_box(xa, xb, -2.5, 2.5, zb - 1, zib + 1))
    return y


def ring_gauge(d):
    x0, x1 = d["beam_x"] - 1.5, d["beam_x"] + 1.5
    top = saddle_blank(d, with_channel=False).intersect(_box(x0, x1, -200, 200, -200, 200))
    dg = derived({k: v for k, v in d.items() if k in P} | dict(body_h=GAUGE_H))
    return top.union(yoke(dg, x0, x1, holes=False))


def gap_stair(steps=(2, 4, 6, 8, 10, 12, 14, 16), run=10.0, width=14.0):
    """Stepped feeler: push it into a gap until it stops, read the number on the last step that went in."""
    out = None
    for i, h in enumerate(steps):
        b = _box(i * run, (i + 1) * run, 0, width, 0, h)
        try:
            txt = (cq.Workplane("XY").workplane(offset=h - 0.6).center(i * run + run / 2, width / 2)
                   .text(str(h), 6.0, 0.7, halign="center", valign="center"))
            b = b.cut(txt)
        except Exception:
            pass
        out = b if out is None else out.union(b)
    return out


# ----------------------------------------------------------------------------
# Preview helpers (not printed)
def body_proxy(d):
    """Fuselage built from the photo measurements - preview and clash check only."""
    sx = d["sensor_x"]
    hw, rf, bf, h = d["body_w"] / 2, d["roof_flat"] / 2, d["belly_flat"] / 2, d["body_h"]
    rs, rb = hw - rf, hw - bf
    pts = [(rf + rs * math.sin(math.radians(a)), -rs + rs * math.cos(math.radians(a))) for a in range(0, 91, 15)]
    pts += [(bf + rb * math.cos(math.radians(a)), -h + rb - rb * math.sin(math.radians(a))) for a in range(0, 91, 15)]
    pts = pts + [(-y, z) for y, z in reversed(pts)]
    body = _prism_x(pts, sx - 99, sx + 108)
    for s in (+1, -1):                                   # rear-arm fairings, 91 mm across
        hip = (cq.Workplane("XY")
               .polyline([(sx - 95, s * 45.5), (sx - 47, s * 45.5), (sx - 34.5, s * (hw - 1)), (sx - 95, s * (hw - 1))])
               .close().extrude(30).translate((0, 0, -60)))
        body = body.union(hip)
    return body


def body_marks(d):
    sx, hw, h = d["sensor_x"], d["body_w"] / 2, d["body_h"]
    cams = _cyl_z(sx, 0, -0.5, 2.0, 7.6)
    for s in (+1, -1):
        cams = cams.union(_cyl_z(sx + 73, s * 28, -0.5, 2.0, 7.5))
        cams = cams.union(_cyl_z(sx + 62, s * 17, -h - 1.5, -h + 0.5, 7.5))
    cams = cams.union(_cyl_z(sx, 0, -h - 1.5, -h + 0.5, 8.0))              # downward camera (position +/- 10 mm)
    cams = cams.union(_cyl_z(sx + 13, 0, -h - 1.0, -h + 0.5, 4.0))         # auxiliary light
    vents = _box(sx - 80, sx - 71, -25, 25, -1.0, 1.0)                     # exhaust grille
    for s in (+1, -1):
        y0, y1 = sorted((s * (hw - 0.5), s * (hw + 0.6)))
        vents = vents.union(_box(sx - 42, sx - 8, y0, y1, -26, -15))       # side mesh vents
    return cams, vents


def screws(d):
    out = None
    for sy in (+1, -1):
        zh = d["ear_bot"] + 5.0
        s = (_cyl_z(d["beam_x"], sy * d["ear_y"], zh - 20, zh, 1.5)
             .union(_cyl_z(d["beam_x"], sy * d["ear_y"], zh, zh + 3, 2.75)))
        out = s if out is None else out.union(s)
    return out


def volume_cm3(wp):
    return wp.val().Volume() / 1000.0


def build(p=None):
    d = derived({**P, **(p or {})})
    return d, dict(saddle=saddle(d), yoke=yoke(d), ring_gauge=ring_gauge(d))


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
    cq.exporters.export(parts["saddle"], str(out / "saddle.step"))
    cq.exporters.export(flip(parts["saddle"]), str(out / "saddle_print.stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(parts["ring_gauge"], str(out / "ring_gauge.step"))
    cq.exporters.export(on_side(parts["ring_gauge"]), str(out / "ring_gauge_print.stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(gap_stair(), str(out / "gap_stair_print.stl"), tolerance=0.02, angularTolerance=0.1)
    # one mesh of the assembled mount, only for looking at it in a 3D viewer
    cq.exporters.export(parts["saddle"].union(parts["yoke"], clean=False), str(out / "mount_assembled_VIEW_ONLY.stl"),
                        tolerance=0.03, angularTolerance=0.15)
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

    print(f"yoke opening {2 * d['yk']:.0f} wide (body {d['body_w']:g} + 2 x {d['side_gap']:g} rubber), "
          f"arms {2 * d['ya']:.0f} apart, overall width {2 * d['ear_out']:.0f} mm, "
          f"yoke {-d['yoke_bot'] - d['body_h']:.1f} mm below the belly")
    tot = [0.0, 0.0]
    for name in ("saddle", "yoke"):
        v = volume_cm3(parts[name]); bb = parts[name].val().BoundingBox()
        tot[0] += v * 1.27 * 0.8; tot[1] += v * 1.27
        print(f"{name:7s} {v:5.1f} cm3 -> about {v * 1.27 * 0.8:.0f}-{v * 1.27:.0f} g in PETG  "
              f"bbox {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f}")
    print(f"total about {tot[0]:.0f}-{tot[1]:.0f} g (+ 2 screws and 4 inserts ~3 g)")
