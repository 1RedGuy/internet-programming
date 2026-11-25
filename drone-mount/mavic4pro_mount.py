"""
Parametric payload mount for DJI Mavic 4 Pro  (CadQuery)  - v2, screw clamp

Parts
  saddle     - sits on top of the fuselage
  yoke (x2)  - U-shaped clamp that comes up from under the belly and is pulled
               against the saddle with two M3 screws (no straps needed)
  tray       - universal payload plate, slides onto the saddle's dovetail rail
               from the rear and is locked with one M3 screw
  fit_gauge  - 3 mm slices of saddle + yoke; print first to check the fit

Nothing printed touches the drone: every inner surface is offset by `pad_t`
for a soft liner, and the inner edges at the ends are relieved further.

Coordinates: X = nose of the drone, Y = left, Z = up, Z=0 = top of the fuselage.
Units: mm.

Usage:
  python mavic4pro_mount.py                 # default parameters
  python mavic4pro_mount.py my_params.json  # override any key of P
"""
import json
import math
import pathlib
import sys

import cadquery as cq

# --------------------------------------------------------------------------
# Parameters.  The first block MUST be checked against the real drone.
# --------------------------------------------------------------------------
P = dict(
    # --- drone fuselage at the mounting station (MEASURE THESE) -------------
    body_w=80.0,        # fuselage width where the mount sits
    body_h=66.0,        # fuselage height (top surface to belly) at that place
    body_rc=20.0,       # radius of the top "shoulders" of the fuselage
    body_rb=14.0,       # radius of the bottom corners of the fuselage
    # --- saddle -------------------------------------------------------------
    skirt_drop=22.0,    # how far the saddle skirts reach down the sides
    pad_t=2.0,          # gap for the soft liner (foam / felt), everywhere
    wall_t=2.6,         # shell thickness
    saddle_len=68.0,    # length along the fuselage
    deck_t=3.6,         # deck thickness over the fuselage crown
    deck_w=54.0,        # width of the flat deck
    edge_relief=1.0,    # extra clearance of the inner edges at both ends
    # --- clamp ----------------------------------------------------------------
    yoke_x=20.0,        # yokes sit at x = +/- yoke_x
    yoke_w=12.0,        # width of a yoke (along the fuselage)
    bar_t=3.5,          # thickness of the yoke under the belly
    ear_out=9.0,        # how far the screw ears stick out sideways
    ear_t=5.0,          # saddle ear thickness
    flange_t=6.0,       # yoke flange thickness (holds the captive nut)
    clamp_gap=3.0,      # gap between ear and flange = tightening travel
    # --- dovetail interface ---------------------------------------------------
    rail_w=30.0,        # dovetail width at its root
    rail_h=4.5,         # dovetail height
    rail_angle=60.0,    # flank angle from horizontal
    fit_clear=0.25,     # clearance per flank (raise to 0.35 if the tray is tight)
    # --- tray -----------------------------------------------------------------
    tray_w=84.0,
    tray_l=80.0,
    tray_t=2.6,
    tray_front=0.0,     # tray front edge relative to saddle front edge (+ = forward)
    gib_w=7.0,          # width of the tray's dovetail jaws
    endwall_t=5.0,      # tray stop / lock wall behind the saddle
    hole_d=3.4,         # M3 clearance
)


def derived(p):
    d = dict(p)
    d["deck_z"] = p["pad_t"] + p["deck_t"]
    d["rail_top_w"] = p["rail_w"] + 2 * p["rail_h"] / math.tan(math.radians(p["rail_angle"]))
    d["rail_top_z"] = d["deck_z"] + p["rail_h"]
    d["tray_z0"] = d["rail_top_z"] + 0.15
    d["lock_z"] = d["deck_z"] + 1.0          # axis of the M3 lock screw
    d["wi"] = p["body_w"] + 2 * p["pad_t"]   # inner width of saddle / yoke
    d["wo"] = d["wi"] + 2 * p["wall_t"]      # outer width
    d["ear_y"] = d["wo"] / 2 + p["ear_out"] / 2          # clamp screw axis
    d["yoke_top_z"] = -p["skirt_drop"] - p["clamp_gap"]
    d["yoke_bot_z"] = -p["body_h"] - p["pad_t"] - p["bar_t"]
    assert p["skirt_drop"] > p["body_rc"], "skirt_drop must be larger than body_rc"
    assert p["deck_w"] <= d["wi"], "deck wider than the saddle"
    assert d["lock_z"] - 3.3 >= p["pad_t"] + p["edge_relief"], "deck_t too thin for the lock nut"
    assert p["yoke_x"] + p["yoke_w"] / 2 <= p["saddle_len"] / 2, "yokes outside the saddle"
    assert p["body_h"] - p["skirt_drop"] - p["clamp_gap"] > p["body_rb"] + p["flange_t"] + 4, \
        "fuselage too low for this skirt_drop"
    return d


def _prism_x(pts_yz, x0, x1):
    """Extrude a YZ polygon from x0 to x1."""
    return (cq.Workplane("YZ").polyline(pts_yz).close().extrude(x1 - x0)
            .translate((x0, 0, 0)))


def _box(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0)
            .translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)))


# --------------------------------------------------------------------------
def body_proxy(d, length=190.0, x_shift=-10.0):
    """Schematic fuselage - ONLY for previews / clash checks, not the real shape."""
    b = _box(x_shift - length / 2, x_shift + length / 2, -d["body_w"] / 2, d["body_w"] / 2, -d["body_h"], 0)
    b = b.edges("|X").edges(">Z").fillet(d["body_rc"])
    b = b.edges("|X").edges("<Z").fillet(d["body_rb"])
    return b


def _top_profile(x0, x1, width, top_z, radius, height=90.0):
    """Block with rounded top shoulders (fuselage-top shape, offset)."""
    return (_box(x0, x1, -width / 2, width / 2, top_z - height, top_z)
            .edges("|X").edges(">Z").fillet(radius))


def saddle_blank(d):
    L, pad, wall = d["saddle_len"], d["pad_t"], d["wall_t"]
    h_out = pad + wall + d["skirt_drop"]
    outer = _top_profile(-L / 2, L / 2, d["wo"], pad + wall, d["body_rc"] + pad + wall, h_out)
    deck = _box(-L / 2, L / 2, -d["deck_w"] / 2, d["deck_w"] / 2, pad, d["deck_z"])
    rb, rt = d["rail_w"] / 2, d["rail_top_w"] / 2
    z0, z1 = d["deck_z"], d["rail_top_z"]
    rail = _prism_x([(-rb, z0 - 0.5), (rb, z0 - 0.5), (rb, z0), (rt, z1), (-rt, z1), (-rb, z0)],
                    -L / 2, L / 2)
    inner = _top_profile(-L / 2 - 1, L / 2 + 1, d["wi"], pad, d["body_rc"] + pad)
    s = outer.union(deck).union(rail).cut(inner)
    # nothing sharp at the lower rim
    s = s.faces("<Z").edges("|X").chamfer(0.8)
    # relieve the inner edges at both ends so a printed edge can never reach the drone
    r = d["edge_relief"]
    for sx in (+1, -1):
        xa, xb = sorted((sx * (L / 2 - 2.0), sx * (L / 2 + 1.0)))
        s = s.cut(_top_profile(xa, xb, d["wi"] + 2 * r, pad + r, d["body_rc"] + pad + r))
    return s


def saddle(d):
    L = d["saddle_len"]
    s = saddle_blank(d)

    # four screw ears; the 45 deg ramp on the rear side makes them printable
    # without supports when the saddle stands on its rear face
    zb = -d["skirt_drop"]
    y0, y1 = d["wo"] / 2 - 1.0, d["wo"] / 2 + d["ear_out"]
    w = d["yoke_w"]
    clip = _box(-L / 2, L / 2, -200, 200, -200, 200)
    for xc in (+d["yoke_x"], -d["yoke_x"]):
        for sy in (+1, -1):
            pts = [(xc + w / 2, sy * y0), (xc + w / 2, sy * y1), (xc - w / 2, sy * y1),
                   (xc - w / 2 - (y1 - y0), sy * y0)]
            ear = (cq.Workplane("XY").polyline(pts).close().extrude(d["ear_t"])
                   .translate((0, 0, zb)).intersect(clip))
            s = s.union(ear)
            s = s.cut(cq.Workplane("XY").circle(d["hole_d"] / 2).extrude(d["ear_t"] + 2)
                      .translate((xc, sy * d["ear_y"], zb - 1)))

    # tray lock: M3 screw along X from the rear, nut dropped into a slot in the rail
    xr = -L / 2
    hole = (cq.Workplane("YZ").circle(3.3 / 2).extrude(10.5)
            .translate((xr - 0.1, 0, d["lock_z"])))
    nut = _box(xr + 3.5, xr + 7.8, -2.9, 2.9, d["lock_z"] - 3.3, d["rail_top_z"] + 0.1)
    # weight saving: the middle of the rail is not needed, only its two flanks
    hw = d["rail_w"] / 2 - 7.0
    groove = _box(xr + 13.0, L / 2 + 1, -hw, hw, d["deck_z"], d["rail_top_z"] + 0.1)
    return s.cut(hole).cut(nut).cut(groove)


def yoke(d, xc=0.0, holes=True):
    """U-clamp that goes under the belly. Print lying on its side (profile on the bed)."""
    w, pad, wall = d["yoke_w"], d["pad_t"], d["wall_t"]
    x0, x1 = xc - w / 2, xc + w / 2
    zt, zb = d["yoke_top_z"], d["yoke_bot_z"]
    outer = (_box(x0, x1, -d["wo"] / 2, d["wo"] / 2, zb, zt)
             .edges("|X").edges("<Z").fillet(d["body_rb"] + pad + wall))
    inner = (_box(x0 - 1, x1 + 1, -d["wi"] / 2, d["wi"] / 2, zb + d["bar_t"], zt + 5)
             .edges("|X").edges("<Z").fillet(d["body_rb"] + pad))
    y = outer.cut(inner)
    ft = d["flange_t"]
    for sy in (+1, -1):
        ya, yb = sorted((sy * (d["wo"] / 2 - 0.5), sy * (d["wo"] / 2 + d["ear_out"])))
        y = y.union(_box(x0, x1, ya, yb, zt - ft, zt))
        yo = sy * d["wo"] / 2
        y = y.union(_prism_x([(yo, zt - ft + 0.1), (yo + sy * 3.5, zt - ft + 0.1), (yo, zt - ft - 3.5)], x0, x1))
    try:                                   # break every edge - friendlier to hands and to the drone
        y = y.edges().chamfer(0.4)
    except Exception:
        pass
    for sy in ((+1, -1) if holes else ()):
        y = y.cut(cq.Workplane("XY").circle(d["hole_d"] / 2).extrude(ft + 2)
                  .translate((xc, sy * d["ear_y"], zt - ft - 1)))
        y = y.cut(cq.Workplane("XY").polygon(6, 6.6).extrude(8.0 + 3.0)      # captive M3 nut
                  .translate((xc, sy * d["ear_y"], zt - ft - 8.0)))
    return y


def tray(d):
    L = d["saddle_len"]
    xf = L / 2 + d["tray_front"]             # tray front edge
    xb = xf - d["tray_l"]                    # tray rear edge
    z0, zt = d["tray_z0"], d["tray_z0"] + d["tray_t"]
    xs_rear = -L / 2                         # saddle rear face
    wall_x0 = xs_rear - 0.3 - d["endwall_t"]
    assert xb <= wall_x0 + 1e-6, "tray_l too short: tray must reach behind the saddle"

    plate = _box(xb, xf, -d["tray_w"] / 2, d["tray_w"] / 2, z0, zt).edges("|Z").fillet(6.0)

    # dovetail jaws + end wall, one block, then the channel is cut out of it
    c = d["fit_clear"] / math.sin(math.radians(d["rail_angle"]))
    gib_out = d["rail_top_w"] / 2 + c + d["gib_w"]
    jaws = _box(wall_x0, xf, -gib_out, gib_out, d["deck_z"] + 0.3, z0 + 0.01)
    # end wall reaches lower so the lock screw has material around it
    wall = _box(wall_x0, wall_x0 + d["endwall_t"], -gib_out, gib_out, d["lock_z"] - 4.0, z0 + 0.01)

    k = 1 / math.tan(math.radians(d["rail_angle"]))
    za, zb2 = d["deck_z"] - 1.0, z0 + 0.01
    ha = d["rail_w"] / 2 + (za - d["deck_z"]) * k + c
    hb = d["rail_w"] / 2 + (zb2 - d["deck_z"]) * k + c
    channel = _prism_x([(-ha, za), (ha, za), (hb, zb2), (-hb, zb2)], xs_rear - 0.3, xf + 1)

    t = plate.union(jaws).union(wall).cut(channel)

    # lock screw hole through the end wall
    t = t.cut(cq.Workplane("YZ").circle(d["hole_d"] / 2).extrude(d["endwall_t"] + 2)
              .translate((wall_x0 - 1, 0, d["lock_z"])))

    # payload fixing: M3 holes + slots (zip tie / strap / elastic) beside the jaws
    y_hole = gib_out + 2.0 + 3.0
    y_slot = d["tray_w"] / 2 - 3.5 - 1.6
    assert y_slot - 1.6 > y_hole + d["hole_d"] / 2 + 1.0, "tray_w too small for holes + slots"
    n = int((d["tray_l"] - 20) // 20) + 1
    for i in range(n):
        x = xf - 10 - 20 * i
        for sy in (+1, -1):
            t = t.cut(cq.Workplane("XY").circle(d["hole_d"] / 2).extrude(d["tray_t"] + 2)
                      .translate((x, sy * y_hole, z0 - 1)))
    for xc in (xf - d["tray_l"] * 0.25, xf - d["tray_l"] * 0.75):
        for sy in (+1, -1):
            t = t.cut(_box(xc - 11, xc + 11, sy * y_slot - 1.6, sy * y_slot + 1.6, z0 - 1, zt + 1))
    # weight-saving windows over the rail
    for xa in (xf - 34.0, xf - 66.0):
        if xa > wall_x0 + d["endwall_t"] + 2:
            t = t.cut(_box(xa, xa + 26.0, -11, 11, z0 - 1, zt + 1).edges("|Z").fillet(4.0))
    return t


def fit_gauge(d):
    slab = _box(-1.5, 1.5, -200, 200, -200, 200)
    dd = dict(d, yoke_w=3.0)
    return saddle_blank(d).intersect(slab).union(yoke(dd, 0.0, holes=False))


def screws(d):
    """Preview only."""
    out = None
    zh = -d["skirt_drop"] + d["ear_t"]
    for xc in (+d["yoke_x"], -d["yoke_x"]):
        for sy in (+1, -1):
            s = (cq.Workplane("XY").circle(1.5).extrude(16).translate((xc, sy * d["ear_y"], zh - 16))
                 .union(cq.Workplane("XY").circle(2.75).extrude(3).translate((xc, sy * d["ear_y"], zh))))
            out = s if out is None else out.union(s)
    return out


def volume_cm3(wp):
    return wp.val().Volume() / 1000.0


def build(p=None):
    d = derived({**P, **(p or {})})
    parts = dict(saddle=saddle(d), yoke=yoke(d, d["yoke_x"]), yoke2=yoke(d, -d["yoke_x"]),
                 tray=tray(d), fit_gauge=fit_gauge(d))
    return d, parts


if __name__ == "__main__":
    override = json.loads(pathlib.Path(sys.argv[1]).read_text()) if len(sys.argv) > 1 else {}
    d, parts = build(override)
    out = pathlib.Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    for old in out.glob("*"):
        if old.suffix in (".step", ".stl"):
            old.unlink()

    # print orientations: saddle / yoke / gauge stand on their rear (X) face,
    # tray lies on its top face
    on_end = lambda wp: wp.rotate((0, 0, 0), (0, 1, 0), -90)
    printable = dict(
        saddle=on_end(parts["saddle"]),
        yoke=on_end(yoke(d, 0.0)),
        tray=parts["tray"].rotate((0, 0, 0), (1, 0, 0), 180),
        fit_gauge=on_end(parts["fit_gauge"]),
    )
    for name, wp in printable.items():
        src = parts[name] if name != "yoke" else yoke(d, 0.0)
        cq.exporters.export(src, str(out / f"{name}.step"))
        fn = f"{name}_print_x2.stl" if name == "yoke" else f"{name}_print.stl"
        cq.exporters.export(wp, str(out / fn), tolerance=0.02, angularTolerance=0.1)

    asm = cq.Assembly()
    asm.add(parts["saddle"], name="saddle", color=cq.Color(0.95, 0.45, 0.1))
    asm.add(parts["yoke"], name="yoke_front", color=cq.Color(0.2, 0.2, 0.22))
    asm.add(parts["yoke2"], name="yoke_rear", color=cq.Color(0.2, 0.2, 0.22))
    asm.add(parts["tray"], name="tray", color=cq.Color(0.1, 0.45, 0.5))
    asm.add(body_proxy(d), name="fuselage_proxy_NOT_REAL_SHAPE", color=cq.Color(0.8, 0.8, 0.8))
    asm.export(str(out / "assembly.step"))

    print(f"tray top above fuselage = {d['tray_z0'] + d['tray_t']:.2f} mm, "
          f"clamp reaches {-d['yoke_bot_z'] - d['body_h']:.1f} mm below the belly, "
          f"overall width {d['wo'] + 2 * d['ear_out']:.1f} mm")
    total = [0.0, 0.0]
    for name, n in (("saddle", 1), ("yoke", 2), ("tray", 1)):
        v = volume_cm3(parts[name]) * n
        bb = parts[name].val().BoundingBox()
        total[0] += v * 1.27 * 0.8; total[1] += v * 1.27
        print(f"{name:7s} x{n}  {v:5.1f} cm3 -> about {v * 1.27 * 0.8:.0f}-{v * 1.27:.0f} g in PETG  "
              f"bbox {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f}")
    print(f"total about {total[0]:.0f}-{total[1]:.0f} g (+ 5 screws and nuts ~5 g)")
