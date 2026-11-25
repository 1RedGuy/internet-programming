"""
Payload box for the Mavic 4 Pro mount (CadQuery).

It slides into the dovetail channel of the saddle from the rear and is locked with
two M3 screws that go through open slots in its rear foot into the saddle's inserts.
It prints upright, as modelled, WITHOUT supports: the rails, the side pockets, the
skirts and the rear foot all end on one plane, and the floor between them is bridged.

Inside (default): 52 x 81 mm floor, 26 mm high, plus two lower side pockets.
Planned content: LiPo 50 x 34 x 8, two stacked XIAO boards 25 x 18 x 18, spare room.

Coordinates as in mavic4pro_mount.py (X = nose, Z = up from the roof).
"""
import json
import math
import pathlib
import sys

import cadquery as cq

import mavic4pro_mount as m

B = dict(
    box_w=84.0,         # outer width
    wall=1.2,
    floor=1.6,
    inner_h=26.0,       # clear height above the main floor
    lid_t=1.4,
    rear_chamfer=10.0,  # cut rear corners: they are the closest points to the rear propellers
    front_chamfer=3.0,
    floor_gap=0.6,      # air between the box floor and the saddle deck (bridges may sag a little)
    rail_fit=0.35,      # clearance per dovetail flank; untested with your printer, so on the loose side
    foot_t=5.3,         # rear foot that stops against the saddle and takes the lock screws
    pod_from=30.0,      # the side pockets start this far from the centreline (the deck is 29)
    skirt_from=39.5,    # in front, beside the cross beam, only this outer strip may hang down
    boss_r=4.3,
)


def _poly(pts, z0, z1):
    return cq.Workplane("XY").polyline(pts).close().extrude(z1 - z0).translate((0, 0, z0))


def _outline(d, b, inset=0.0):
    """Plan outline of the box, optionally moved inwards (approximate offset of a convex polygon)."""
    xf = d["saddle_len"] / 2 - inset
    xr = -d["saddle_len"] / 2 - 0.3 - b["foot_t"] + inset
    hw = b["box_w"] / 2 - inset
    cr = b["rear_chamfer"] - inset * (2 - math.sqrt(2))
    cf = max(b["front_chamfer"] - inset * (2 - math.sqrt(2)), 0.2)
    return [(xf, hw - cf), (xf - cf, hw), (xr + cr, hw), (xr, hw - cr), (xr, -hw + cr), (xr + cr, -hw),
            (xf - cf, -hw), (xf, -hw + cf)]


def levels(d, b):
    zb = d["chan_bot"] + 0.4                 # everything that touches the print bed ends here
    zf0 = d["deck_top"] + b["floor_gap"]     # underside of the main floor
    zf1 = zf0 + b["floor"]
    return zb, zf0, zf1, zf1 + b["inner_h"]


def boss_xy(d, b):
    xf = d["saddle_len"] / 2
    xr = -d["saddle_len"] / 2 - 0.3 - b["foot_t"]
    hw = b["box_w"] / 2
    return [(xf - 5.4, hw - 5.4), (xf - 5.4, -hw + 5.4), (xr + 9.0, hw - 7.4), (xr + 9.0, -hw + 7.4)]


def box(d, b):
    L = d["saddle_len"]
    xf, xs = L / 2, -L / 2 - 0.3                      # box front, front face of the rear foot
    xr = xs - b["foot_t"]
    hw, t = b["box_w"] / 2, b["wall"]
    zb, zf0, zf1, zt = levels(d, b)
    beam0 = d["beam_x"] - d["beam_w"] / 2 - 2.0      # the cross beam starts here: pockets must end before it

    s = _poly(_outline(d, b), zb, zt)
    inner = _outline(d, b, t)
    s = s.cut(_poly(inner, zf1, zt + 1))                                             # main cavity
    # room for the saddle under the floor
    s = s.cut(m._box(xs, xf + 1, -b["pod_from"], b["pod_from"], zb - 1, zf0))        # over the deck
    s = s.cut(m._box(beam0, xf + 1, -b["skirt_from"], b["skirt_from"], zb - 1, zf0)) # over the cross beam
    # side pockets (lower floor) behind the cross beam
    for sy in (+1, -1):
        ya, yb = sorted((sy * (b["pod_from"] + t), sy * (hw + 1)))
        pod = _poly(inner, zb + b["floor"], zf1 + 0.1).intersect(m._box(xr - 1, beam0 - t, ya, yb, zb, zf1 + 1))
        s = s.cut(pod)
    # dovetail rails: two flanks, hollow in the middle
    c = b["rail_fit"] / math.sin(math.radians(d["rail_angle"]))
    k = 1 / math.tan(math.radians(d["rail_angle"]))
    hr = d["rail_w"] / 2 - c
    ht = d["rail_w"] / 2 + (d["deck_top"] - zb) * k - c
    rail = m._prism_x([(-hr, zf0 + 0.05), (hr, zf0 + 0.05), (hr, d["deck_top"]), (ht, zb), (-ht, zb), (-hr, d["deck_top"])], xs, xf)
    rail = rail.cut(m._box(xs - 1, xf + 1, -(hr - 7.0), hr - 7.0, zb - 1, zf0))
    s = s.union(rail)
    # lock screws: slots open at the bottom, so the foot can end on the print bed
    for sy in (+1, -1):
        s = s.cut(cq.Workplane("YZ").circle(d["hole_d"] / 2).extrude(b["foot_t"] + 2).translate((xr - 1, sy * d["lock_y"], d["lock_z"])))
        s = s.cut(m._box(xr - 1, xs + 1, sy * d["lock_y"] - d["hole_d"] / 2, sy * d["lock_y"] + d["hole_d"] / 2, zb - 1, d["lock_z"]))
    # lid bosses with holes for M3 heat-set inserts
    for i, (x, y) in enumerate(boss_xy(d, b)):
        z0 = zf1 - 0.1 if i < 2 else zb + b["floor"] - 0.1
        s = s.union(cq.Workplane("XY").circle(b["boss_r"]).extrude(zt - z0).translate((x, y, z0)))
    s = s.intersect(_poly(_outline(d, b), zb - 1, zt + 1))                           # nothing may stick out of the outline
    for (x, y) in boss_xy(d, b):
        s = s.cut(m._cyl_z(x, y, zt - 7.0, zt + 1, d["insert_d"] / 2))
        s = s.cut(m._cyl_z(x, y, zt - 0.6, zt + 1, d["insert_d"] / 2 + 0.4))
    # cable notches in the rim (antenna leads) and a service window in the rear wall (USB-C)
    for sy in (+1, -1):
        s = s.cut(m._box(-4, 0, sy * hw - 3, sy * hw + 3, zt - 4.0, zt + 1))
    s = s.cut(m._box(xr - 1, xr + t + 1, 6.0, 20.0, zf1 + 1.5, zf1 + 9.5))
    return s


def lid(d, b):
    zb, zf0, zf1, zt = levels(d, b)
    l = _poly(_outline(d, b), zt, zt + b["lid_t"])
    for (x, y) in boss_xy(d, b):
        l = l.cut(m._cyl_z(x, y, zt - 1, zt + b["lid_t"] + 1, d["hole_d"] / 2))
    return l


def rail_test(d, b):
    """12 mm slice of the box bottom: prints in minutes, checks that the dovetail slides in the saddle."""
    zb, zf0, zf1, zt = levels(d, b)
    return box(d, b).intersect(m._box(-14, -2, -b["pod_from"] + 0.5, b["pod_from"] - 0.5, zb - 1, zf1 + 0.01))


def contents(d, b):
    """Preview only: battery and the two stacked boards."""
    zb, zf0, zf1, zt = levels(d, b)
    xr = -d["saddle_len"] / 2 - 0.3 - b["foot_t"] + b["wall"]
    bat = m._box(xr + 1, xr + 51, -30, 4, zf1, zf1 + 8)            # LiPo 50 x 34 x 8, lengthwise
    brd = m._box(xr + 3, xr + 28, 8, 26, zf1, zf1 + 18)            # two stacked XIAO boards 25 x 18 x 18
    return bat, brd


if __name__ == "__main__":
    override = json.loads(pathlib.Path(sys.argv[1]).read_text()) if len(sys.argv) > 1 else {}
    d = m.derived({**m.P, **{k: v for k, v in override.items() if k in m.P}})
    b = {**B, **{k: v for k, v in override.items() if k in B}}
    out = pathlib.Path(__file__).parent / "out"
    bx, ld, rt = box(d, b), lid(d, b), rail_test(d, b)
    zb, zf0, zf1, zt = levels(d, b)
    drop = lambda wp: wp.translate((0, 0, -wp.val().BoundingBox().zmin))
    cq.exporters.export(bx, str(out / "box.step"))
    cq.exporters.export(ld, str(out / "box_lid.step"))
    cq.exporters.export(drop(bx), str(out / "box_print.stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(drop(ld), str(out / "box_lid_print.stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(drop(rt), str(out / "box_rail_test_print.stl"), tolerance=0.02, angularTolerance=0.1)
    L = d["saddle_len"]
    print(f"outer {L + 0.3 + b['foot_t']:.1f} x {b['box_w']:g} mm, top of the lid {zt + b['lid_t']:.1f} mm above the roof")
    print(f"inside: floor {L + 0.3 + b['foot_t'] - 2 * b['wall']:.1f} x {b['box_w'] - 2 * b['wall']:.1f} mm, {b['inner_h']:g} mm high; "
          f"side pockets {zf1 - zb - b['floor']:.1f} mm deeper")
    for name, wp in (("box", bx), ("lid", ld)):
        v = wp.val().Volume() / 1000
        print(f"{name}: {v:.1f} cm3 -> about {v * 1.27 * 0.85:.0f}-{v * 1.27:.0f} g in PETG, valid {wp.val().isValid()}, solids {wp.solids().size()}")
