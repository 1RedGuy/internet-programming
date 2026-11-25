"""
Parametric payload mount for DJI Mavic 4 Pro  (CadQuery)

Parts
  saddle     - sits on top of the fuselage, held by two hook-and-loop straps that
               pass through tunnels in the saddle and wrap fully around the body
  tray       - universal payload plate, slides onto the saddle's dovetail rail
               from the rear and is locked with one M3 screw
  fit_gauge  - 3 mm slice of the saddle; print first to check the fit on the drone

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
    body_w=80.0,        # fuselage width where the saddle sits
    body_rc=20.0,       # radius of the top "shoulders" of the fuselage
    skirt_drop=22.0,    # how far the saddle skirts reach down the sides
    # --- saddle -------------------------------------------------------------
    pad_t=2.0,          # gap for self-adhesive EVA / neoprene pad
    wall_t=2.6,         # shell thickness
    saddle_len=72.0,    # length along the fuselage
    web_end=10.0,       # solid material in front of / behind the strap tunnels
    strap_w=20.0,       # strap width (tunnel is strap_w + 2)
    strap_t=3.0,        # tunnel height
    floor_t=1.4,        # material under the strap tunnel
    roof_t=2.2,         # material over the strap tunnel
    deck_w=54.0,        # width of the flat deck
    # --- dovetail interface ---------------------------------------------------
    rail_w=30.0,        # dovetail width at its root
    rail_h=4.5,         # dovetail height
    rail_angle=60.0,    # flank angle from horizontal
    fit_clear=0.25,     # clearance per flank (raise to 0.35 if the tray is tight)
    # --- tray -----------------------------------------------------------------
    tray_w=84.0,
    tray_l=80.0,
    tray_t=3.0,
    tray_front=0.0,     # tray front edge relative to saddle front edge (+ = forward)
    gib_w=7.0,          # width of the tray's dovetail jaws
    endwall_t=5.0,      # tray stop / lock wall behind the saddle
    hole_d=3.4,         # M3 clearance
)


def derived(p):
    d = dict(p)
    d["deck_z"] = p["pad_t"] + p["floor_t"] + p["strap_t"] + p["roof_t"]
    d["rail_top_w"] = p["rail_w"] + 2 * p["rail_h"] / math.tan(math.radians(p["rail_angle"]))
    d["rail_top_z"] = d["deck_z"] + p["rail_h"]
    d["tray_z0"] = d["rail_top_z"] + 0.15
    d["tunnel_len"] = p["strap_w"] + 2.0
    d["mid_web"] = p["saddle_len"] - 2 * p["web_end"] - 2 * d["tunnel_len"]
    d["tunnel_x"] = p["saddle_len"] / 2 - p["web_end"] - d["tunnel_len"] / 2
    d["lock_z"] = d["deck_z"] + 1.0          # axis of the M3 lock screw
    assert d["mid_web"] >= 5, "saddle_len too short for two straps"
    assert p["body_rc"] + p["pad_t"] + p["wall_t"] < p["skirt_drop"] + p["pad_t"] + p["wall_t"], \
        "skirt_drop must be larger than body_rc"
    assert p["deck_w"] <= p["body_w"] + 2 * p["pad_t"], "deck wider than the saddle"
    return d


def _prism_x(pts_yz, x0, x1):
    """Extrude a YZ polygon from x0 to x1."""
    return (cq.Workplane("YZ").polyline(pts_yz).close().extrude(x1 - x0)
            .translate((x0, 0, 0)))


# --------------------------------------------------------------------------
def body_proxy(d, length=190.0, height=68.0, x_shift=-10.0):
    """Schematic fuselage - ONLY for previews / clash checks, not the real shape."""
    b = (cq.Workplane("XY").box(length, d["body_w"], height)
         .translate((x_shift, 0, -height / 2)))
    b = b.edges("|X").fillet(d["body_rc"])
    return b


def saddle_blank(d):
    L, pad, wall = d["saddle_len"], d["pad_t"], d["wall_t"]
    wi = d["body_w"] + 2 * pad
    wo = wi + 2 * wall
    h_out = pad + wall + d["skirt_drop"]

    outer = (cq.Workplane("XY").box(L, wo, h_out)
             .translate((0, 0, pad + wall - h_out / 2))
             .edges("|X").edges(">Z").fillet(d["body_rc"] + pad + wall))
    deck = (cq.Workplane("XY").box(L, d["deck_w"], d["deck_z"] - pad)
            .translate((0, 0, (d["deck_z"] + pad) / 2)))
    rb, rt = d["rail_w"] / 2, d["rail_top_w"] / 2
    z0, z1 = d["deck_z"], d["rail_top_z"]
    rail = _prism_x([(-rb, z0 - 0.5), (rb, z0 - 0.5), (rb, z0), (rt, z1), (-rt, z1), (-rb, z0)],
                    -L / 2, L / 2)

    h_in = 80.0
    inner = (cq.Workplane("XY").box(L + 2, wi, h_in)
             .translate((0, 0, pad - h_in / 2))
             .edges("|X").edges(">Z").fillet(d["body_rc"] + pad))
    s = outer.union(deck).union(rail).cut(inner)
    # soften the lower rim of the skirts (nothing sharp next to the drone / fingers)
    s = s.faces("<Z").edges("|X").chamfer(0.8)
    return s


def saddle(d):
    L = d["saddle_len"]
    s = saddle_blank(d)

    # strap tunnels (strap goes through the saddle and all the way round the body)
    zc = d["pad_t"] + d["floor_t"] + d["strap_t"] / 2
    for sx in (+1, -1):
        t = (cq.Workplane("XY").box(d["tunnel_len"], d["deck_w"] + 20, d["strap_t"])
             .translate((sx * d["tunnel_x"], 0, zc)))
        s = s.cut(t)

    # lock: M3 screw along X from the rear, nut dropped into a slot in the rail
    xr = -L / 2
    hole = (cq.Workplane("YZ").circle(3.3 / 2).extrude(d["web_end"] + 0.5)
            .translate((xr - 0.1, 0, d["lock_z"])))
    nut = (cq.Workplane("XY").box(4.3, 5.8, d["rail_top_z"] - (d["lock_z"] - 3.3) + 0.1)
           .translate((xr + 3.5 + 4.3 / 2, 0,
                       (d["rail_top_z"] + 0.1 + d["lock_z"] - 3.3) / 2)))
    s = s.cut(hole).cut(nut)
    return s


def tray(d):
    L = d["saddle_len"]
    xf = L / 2 + d["tray_front"]             # tray front edge
    xb = xf - d["tray_l"]                    # tray rear edge
    z0, zt = d["tray_z0"], d["tray_z0"] + d["tray_t"]
    xs_rear = -L / 2                         # saddle rear face
    wall_x0 = xs_rear - 0.3 - d["endwall_t"]
    assert xb <= wall_x0 + 1e-6, "tray_l too short: tray must reach behind the saddle"

    plate = (cq.Workplane("XY").box(d["tray_l"], d["tray_w"], d["tray_t"])
             .translate(((xf + xb) / 2, 0, (z0 + zt) / 2))
             .edges("|Z").fillet(6.0))

    # dovetail jaws + end wall, one block, then the channel is cut out of it
    c = d["fit_clear"] / math.sin(math.radians(d["rail_angle"]))
    gib_out = d["rail_top_w"] / 2 + c + d["gib_w"]
    zb = d["deck_z"] + 0.3
    jaws = (cq.Workplane("XY").box(xf - wall_x0, 2 * gib_out, z0 - zb + 0.01)
            .translate(((xf + wall_x0) / 2, 0, (z0 + zb) / 2)))
    # end wall reaches lower so the lock screw has material around it
    wall_zb = d["lock_z"] - 4.0
    wall = (cq.Workplane("XY").box(d["endwall_t"], 2 * gib_out, z0 - wall_zb + 0.01)
            .translate((wall_x0 + d["endwall_t"] / 2, 0, (z0 + wall_zb) / 2)))

    k = 1 / math.tan(math.radians(d["rail_angle"]))
    za, zb2 = d["deck_z"] - 1.0, z0 + 0.01
    ha = d["rail_w"] / 2 + (za - d["deck_z"]) * k + c
    hb = d["rail_w"] / 2 + (zb2 - d["deck_z"]) * k + c
    channel = _prism_x([(-ha, za), (ha, za), (hb, zb2), (-hb, zb2)], xs_rear - 0.3, xf + 1)

    t = plate.union(jaws).union(wall).cut(channel)

    # lock screw hole through the end wall
    t = t.cut(cq.Workplane("YZ").circle(d["hole_d"] / 2).extrude(d["endwall_t"] + 2)
              .translate((wall_x0 - 1, 0, d["lock_z"])))

    # payload fixing: M3 holes + strap slots in the free zones beside the jaws
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
            t = t.cut(cq.Workplane("XY").box(d["strap_w"] + 2, 3.2, d["tray_t"] + 2)
                      .translate((xc, sy * y_slot, (z0 + zt) / 2)))
    return t


def fit_gauge(d):
    slab = cq.Workplane("XY").box(3.0, 400, 400)
    return saddle_blank(d).intersect(slab)


def straps(d, body_h=68.0):
    """Preview only: the two straps as closed bands around body + saddle."""
    out = None
    zc = d["pad_t"] + d["floor_t"] + d["strap_t"] / 2
    for sx in (+1, -1):
        w = d["body_w"] + 2 * (d["pad_t"] + d["wall_t"]) + 3
        h = body_h + zc + 1.5
        o = (cq.Workplane("XY").box(d["strap_w"], w, h).translate((sx * d["tunnel_x"], 0, zc + 0.75 - h / 2))
             .edges("|X").fillet(d["body_rc"] + 3))
        i = (cq.Workplane("XY").box(d["strap_w"] + 2, w - 3, h - 3).translate((sx * d["tunnel_x"], 0, zc + 0.75 - h / 2))
             .edges("|X").fillet(d["body_rc"] + 1.5))
        b = o.cut(i)
        out = b if out is None else out.union(b)
    return out


def volume_cm3(wp):
    return wp.val().Volume() / 1000.0


def build(p=None):
    d = derived({**P, **(p or {})})
    return d, dict(saddle=saddle(d), tray=tray(d), fit_gauge=fit_gauge(d))


if __name__ == "__main__":
    override = json.loads(pathlib.Path(sys.argv[1]).read_text()) if len(sys.argv) > 1 else {}
    d, parts = build(override)
    out = pathlib.Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)

    # print orientations: saddle stands on its front face, tray lies on its top face
    printable = dict(
        saddle=parts["saddle"].rotate((0, 0, 0), (0, 1, 0), -90),
        tray=parts["tray"].rotate((0, 0, 0), (1, 0, 0), 180),
        fit_gauge=parts["fit_gauge"].rotate((0, 0, 0), (0, 1, 0), -90),
    )
    for name, wp in parts.items():
        cq.exporters.export(wp, str(out / f"{name}.step"))
        cq.exporters.export(printable[name], str(out / f"{name}_print.stl"),
                            tolerance=0.02, angularTolerance=0.1)

    asm = cq.Assembly()
    asm.add(parts["saddle"], name="saddle", color=cq.Color(0.95, 0.45, 0.1))
    asm.add(parts["tray"], name="tray", color=cq.Color(0.1, 0.45, 0.5))
    asm.add(body_proxy(d), name="fuselage_proxy_NOT_REAL_SHAPE", color=cq.Color(0.8, 0.8, 0.8))
    asm.save(str(out / "assembly.step"))

    print(f"deck_z={d['deck_z']:.2f}  tray top above fuselage={d['tray_z0'] + d['tray_t']:.2f} mm")
    for name in ("saddle", "tray"):
        v = volume_cm3(parts[name])
        bb = parts[name].val().BoundingBox()
        print(f"{name:7s} solid {v:5.1f} cm3 -> about {v * 1.27 * 0.8:.0f}-{v * 1.27:.0f} g in PETG  "
              f"bbox {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f}")
