"""Car body + interior (prefix ``body_``): a compact 4-door RWD saloon.

    from carviz.assemblies import body
    B = body.build({"detail": "high"})          # ~0.48 M triangles, ~10-15 s ('low': 0.27 M, ~4 s)
    B.drive(track, {"exterior_opacity": arr})   # optional per-group fades (no moving parts)

Everything is in car coordinates (spec.py): ``B.root`` (Empty ``body_root``) sits
at the car origin (ground point under the front-axle centre) with no rotation and
every part is its direct child with an identity transform, so a part-local point
IS a car-frame point.  The car's world motion is applied by scenes to the car
root (carviz.assemblies.car), never here.

Construction
------------
The painted skin is a Catmull-Clark control cage in cube topology (rings along
the car + front/rear face caps, creases on the shoulder line, cowl, deck and
hood/boot edges), subdivided and APPLIED at build time (level 4 'high', 3
'low').  One Manifold boolean then cuts every opening (windows, wheel arches,
lamps, grille, intake, fog lamps, cowl, mirror sails, B-pillar) and every
4.5 mm panel gap (hood, doors, boot lid, bumpers, fuel flap, plate recess) out
of the closed surface; panel gaps are box-section ribbons swept along curves
ray-projected onto the surface.  Every cut edge gets a rolled flange (2 mm
chamfer + return: 10 mm gaps, 18 mm window frames, 40 mm arch lips).  Glass,
lamp lenses and black inserts are the matching pieces of the uncut surface
(INTERSECT), so they fit their openings exactly.

Parts (``B.parts`` key -> object ``body_<key>``; materials)
---------------------------------------------------------
exterior   shell (car_paint: body, doors, hood, boot lid, roof, fenders)
           bumpers (car_paint: front + rear bumper covers, split by real gaps)
           glass (glass: windscreen, 4 door glasses, rear screen; 4 mm inside the frames)
           trim (plastic_black window surrounds, front lip, rear diffuser, cowl panel,
                 grille backing, intake mesh, wipers; body_gloss_black B-pillars, mirror
                 sails, screen frit bands, roof antenna; chrome grille surround + twin
                 exhaust tips; grey grille slats)
           lights_front (glass lens, chrome projector bowls + bezel, black housing,
                 body_led DRL light guide, fog lamps)  - emission OFF
           lights_rear (body_lens_red lens, red housing, chrome reflector, light guide)
           mirrors (car_paint housing, chrome glass, plastic_black base)
           handles (car_paint pulls on plastic_black gaskets)
           cabin_trim (door cards with armrests, headliner (body_headliner), parcel shelf:
                 trim fixed to the shell, so it fades with the exterior group)
interior   interior (2 front seats + rear bench in body_fabric, dashboard with binnacle,
                 display and vents, centre console with the gear-lever opening)
           steering_wheel (wheel + column + shroud at spec.STEERING_WHEEL_CENTRE,
                 column 24 deg below horizontal, ends at the dash support bracket)
           pedals (brake + throttle on the pedal-box axis at spec.BRAKE_PEDAL_X /
                 THROTTLE_PEDAL_X; static. The clutch pedal is the clutch assembly's)
underbody  underbody (paint_black: floor pan + transmission tunnel with the gear-lever
                 hole, firewall at spec.Y_FIREWALL with tunnel opening + clutch-pushrod
                 hole (40 x 64 mm at spec.MASTER_CYL_POS), toe boards, scuttle, rear seat
                 pan, boot floor over the axle, rear panel, 4 wheelhouse liners)
xray       xray_edges (optional, opts['xray_edges']=True: camera-only emissive feature
                 lines - windows, lamps, arches, shoulder + sill lines; starts hidden
                 with cv_opacity 0)
``body_`` local materials (not in the shared table) are built here through
carviz.materials.presentation_group(), so they honour cv_opacity / cv_glow too.

Anchors (object, local offset = car-frame point on the surface)
    hood, windscreen, roof, front_bumper, door_left, cabin, firewall, tunnel

Opts
    detail       'high' (default, subdivision level 4) | 'low' (level 3, ~3x faster)
    xray_edges   False (default) | True
    collection   collection name (default 'body')

Presentation keys for ``drive`` (all optional per-frame arrays, 0..1)
    exterior_opacity, interior_opacity, underbody_opacity   fade whole groups
    glass_opacity          glass alone (overrides exterior for the glass)
    xray_edges_opacity     the feature lines (needs opts['xray_edges'])
    explode                lifts exterior parts by +1.3 m and interior by +0.65 m
The body has no moving parts (doors closed); ``drive`` bakes only these fades.

meta
    groups {'exterior','interior','underbody'(,'xray')}: [objects]; group_names;
    arch (radius, centre_z, y); console_hole / tunnel_hole (half-width, y front, y rear);
    steering_column_angle_deg; triangles {part: n}; triangles_total; build_time;
    build_log (timings, island counts).

Typical values used (not in spec.py; 2.0 L front-engine RWD compact saloon):
    wheel-arch opening radius 0.350 m about (axle, z 0.300) -> 34 mm over the tyre at
    rest; wheelhouse liners radius 0.40 m (room for +-60 mm travel and ~32 deg lock);
    hood 0.87-0.95 m over the engine (cam cover top ~0.80); cowl 0.957 m at y -0.64;
    windscreen rake 62 deg from vertical; roof 1.42 m at y -1.95; belt 0.976-1.012 m;
    sill bottom ~0.185 m; floor top spec.Z_FLOOR; tunnel 0.51 x 0.615 m at the
    bellhousing, 0.27 x 0.45 m at the propshaft; rear H-point (y -2.22, z 0.50); boot
    floor 0.565 m from y -2.56 (diff underneath); front seat cushion 0.50 x 0.50 m.
"""
from __future__ import annotations

import math
import time

import bmesh
import bpy
import numpy as np
from mathutils import Vector
from mathutils import geometry as mgeo
from mathutils.bvhtree import BVHTree

from .. import spec as S

# ---------------------------------------------------------------------------
# Shell cage (Catmull-Clark control net, cube topology)
# ---------------------------------------------------------------------------
# Half ring columns c = 0..17 (x >= 0), from the bottom centre-line to the
# top centre-line:  0..6 underside (6 = sill bottom-outer corner),
# 6..11 body side (7 rocker, 8 lower door, 9 upper door, 10 shoulder line,
# 11 top corner = belt outer), 11..17 top (12 glass base / fender top,
# 13 mid greenhouse, 14 roof rail / A-, C-pillar line, 15..16 roof, 17 top
# centre-line).  Rows r = 0 (front-face boundary) .. 14 (rear-face boundary);
# the front and rear faces are 12 x 5 quad caps.
NC = 17            # last column index
NU = 6             # underside / top edge count (cap width per side)
NS = 5             # side edge count (cap height)


def _lerp(a, b, t):
    return a + (b - a) * t


def _row_points(y, xs, z_low, zb, z_sh, top):
    """Half-ring control points (18, 3) for a body row.

    xs: max half-width (shoulder), z_low: sill bottom-outer corner height,
    zb: underside height, z_sh: shoulder-line height, top: dict describing
    the upper part (keys: kind, ...)."""
    P = np.zeros((NC + 1, 3))
    P[:, 1] = y
    # underside
    for c, x in enumerate((0.0, 0.16, 0.32, 0.47, 0.60, xs - 0.12)):
        P[c] = (x, y, zb + (0.006 if c == 5 else 0.0))
    P[6] = (xs - 0.070, y, z_low)
    P[7] = (xs - 0.024, y, z_low + 0.10)
    P[8] = (xs + 0.000, y, _lerp(z_low + 0.10, z_sh, 0.40))
    P[9] = (xs + 0.004, y, _lerp(z_low + 0.10, z_sh, 0.75))
    P[10] = (xs, y, z_sh)
    t = top
    P[11] = (xs - t.get("corner_in", 0.035), y, z_sh + t.get("corner_up", 0.035))
    for c in range(12, NC + 1):
        x, z = t["pts"][c - 12]
        P[c] = (x, y + t.get("dy", [0] * 6)[c - 12], z)
    return P


def _hood_top(z_c, xs, edge_drop=0.028):
    """Top for hood / deck rows: gently crowned surface."""
    xe = xs - 0.10
    xsamp = (xe, xe - 0.10, 0.50, 0.34, 0.17, 0.0)
    pts = []
    for x in xsamp:
        u = x / max(xe, 1e-6)
        pts.append((x, z_c - edge_drop * u ** 2))
    return dict(kind="hood", pts=pts)


def _hood_top_front(z_c, xs):
    """Top for the front-fender rows: crisp fender crown, the hood sitting
    slightly lower between the fenders (shut-line valley), crowned centre."""
    pts = [(xs - 0.080, z_c + 0.006), (xs - 0.150, z_c - 0.006), (0.50, z_c - 0.001),
           (0.34, z_c + 0.006), (0.17, z_c + 0.012), (0.0, z_c + 0.014)]
    return dict(kind="hood", pts=pts, corner_in=0.028, corner_up=0.028)


def _gh_top(xg, zg, xr, zr, z_roof):
    """Top for greenhouse rows: glass base (xg, zg), roof rail (xr, zr),
    roof centre z_roof."""
    xm = _lerp(xg, xr, 0.5) + 0.012
    zm = _lerp(zg, zr, 0.5)
    pts = [(xg, zg), (xm, zm), (xr, zr), (xr - 0.13, zr + 0.55 * (z_roof - zr)),
           (0.30, z_roof - 0.12 * (z_roof - zr)), (0.0, z_roof)]
    return dict(kind="gh", pts=pts, corner_in=0.045, corner_up=0.0)


def shell_cage():
    """Return (verts (n,3), quads (m,4), crease dict {(i,j): w})."""
    rows = []
    # --- row 0: front-face boundary ring (hood leading edge, bumper corners, lip)
    r0 = np.array([
        (0.000, 0.800, 0.268), (0.160, 0.798, 0.268), (0.320, 0.788, 0.270), (0.470, 0.765, 0.274),
        (0.600, 0.728, 0.280), (0.700, 0.675, 0.288), (0.765, 0.615, 0.300),
        (0.786, 0.605, 0.390), (0.794, 0.606, 0.500), (0.793, 0.606, 0.590), (0.782, 0.602, 0.665),
        (0.752, 0.600, 0.718),
        (0.690, 0.683, 0.728), (0.590, 0.738, 0.731), (0.470, 0.772, 0.736), (0.330, 0.795, 0.740),
        (0.165, 0.808, 0.743), (0.000, 0.812, 0.744)])
    rows.append(r0)
    # --- body rows: y, xs, z_low, zb, z_sh, top
    defs = []
    # front fender rows: wedge (hood falls ~11 deg toward the nose), crisp fender crowns
    defs.append((0.50, 0.858, 0.285, 0.272, 0.775, _hood_top_front(0.800, 0.858)))
    defs.append((0.28, 0.879, 0.235, 0.210, 0.815, _hood_top_front(0.850, 0.879)))
    defs.append((0.00, 0.881, 0.200, 0.180, 0.845, _hood_top_front(0.890, 0.881)))
    defs.append((-0.34, 0.879, 0.190, 0.170, 0.866, _hood_top_front(0.926, 0.879)))
    # cowl (windscreen base): greenhouse collapsed onto the cowl
    cowl = dict(kind="cowl", pts=[(0.795, 0.952), (0.760, 0.956), (0.715, 0.960), (0.52, 0.958),
                                  (0.27, 0.957), (0.0, 0.957)], corner_in=0.045, corner_up=0.03)
    defs.append((-0.64, 0.872, 0.185, 0.165, 0.876, cowl))
    # mid windscreen
    defs.append((-1.04, 0.869, 0.183, 0.163, 0.880,
                 _gh_top(0.800, 0.965, 0.668, 1.150, 1.185)))
    # header (roof front)
    defs.append((-1.46, 0.868, 0.183, 0.163, 0.885,
                 _gh_top(0.800, 0.972, 0.628, 1.345, 1.392)))
    # B-pillar / roof peak
    defs.append((-1.95, 0.869, 0.185, 0.163, 0.890,
                 _gh_top(0.800, 0.982, 0.640, 1.362, 1.422)))
    # rear door / C-pillar start
    defs.append((-2.40, 0.877, 0.190, 0.165, 0.896,
                 _gh_top(0.800, 0.995, 0.632, 1.350, 1.410)))
    # rear screen header
    defs.append((-2.68, 0.880, 0.205, 0.175, 0.900,
                 _gh_top(0.795, 1.006, 0.605, 1.320, 1.372)))
    # mid rear screen (C-pillar descending)
    defs.append((-2.92, 0.876, 0.225, 0.200, 0.902,
                 _gh_top(0.785, 1.018, 0.640, 1.175, 1.215)))
    # deck front (rear screen base)
    deck0 = dict(kind="deck", pts=[(0.770, 1.030), (0.735, 1.040), (0.690, 1.048), (0.50, 1.055),
                                   (0.27, 1.058), (0.0, 1.059)], corner_in=0.045, corner_up=0.035)
    defs.append((-3.16, 0.862, 0.265, 0.245, 0.900, deck0))
    deck1 = _hood_top(1.050, 0.840, edge_drop=0.02)
    defs.append((-3.33, 0.840, 0.305, 0.285, 0.895, deck1))
    for (y, xs, zl, zb, zsh, top) in defs:
        rows.append(_row_points(y, xs, zl, zb, zsh, top))
    # --- rear-face boundary ring
    r14 = np.array([
        (0.000, -3.470, 0.330), (0.160, -3.469, 0.330), (0.320, -3.462, 0.332), (0.470, -3.446, 0.336),
        (0.600, -3.420, 0.340), (0.690, -3.388, 0.346), (0.752, -3.355, 0.360),
        (0.780, -3.390, 0.450), (0.790, -3.400, 0.580), (0.790, -3.402, 0.720), (0.786, -3.398, 0.860),
        (0.765, -3.392, 0.975),
        (0.700, -3.452, 1.010), (0.600, -3.476, 1.020), (0.480, -3.492, 1.026), (0.340, -3.502, 1.030),
        (0.170, -3.508, 1.032), (0.000, -3.510, 1.033)])
    rows.append(r14)

    V = []
    ring_idx = []

    def add(p):
        V.append(tuple(float(v) for v in p))
        return len(V) - 1

    for R in rows:
        idx = [None] * (2 * NC)
        for c in range(NC + 1):
            idx[c] = add(R[c])
        for c in range(1, NC):
            x, y, z = R[c]
            idx[2 * NC - c] = add((-x, y, z))
        ring_idx.append(idx)
    Q = []
    nk = 2 * NC
    for r in range(len(rows) - 1):
        a, b = ring_idx[r], ring_idx[r + 1]
        for k in range(nk):
            k1 = (k + 1) % nk
            Q.append((a[k], a[k1], b[k1], b[k]))

    def ring_k(c, side):
        if c in (0, NC):
            return c
        return c if side > 0 else 2 * NC - c

    def cap(ring, centre_y, setback, z_levels, bulge_y):
        """Front/rear face cap: grid a in [-NU..NU], b in [0..NS]."""
        G = {}
        for a in range(-NU, NU + 1):
            s = 1 if a >= 0 else -1
            G[(a, 0)] = ring[ring_k(abs(a), s)]
            G[(a, NS)] = ring[ring_k(NC - abs(a), s)]
        for b in range(NS + 1):
            G[(NU, b)] = ring[ring_k(NU + b, 1)]
            G[(-NU, b)] = ring[ring_k(NU + b, -1)]
        for b in range(1, NS):
            zb = z_levels[b - 1]
            for a in range(0, NU):
                # x from blending bottom/top/side boundary x
                xb = V[G[(a, 0)]][0]
                xt = V[G[(a, NS)]][0]
                xside = V[G[(NU, b)]][0]
                tb = b / NS
                x = _lerp(xb, xt, tb)
                x = x * (0.92 + 0.08 * (xside / max(V[G[(NU, 0)]][0], 1e-6)))
                yv = centre_y(zb) + bulge_y - setback(x)
                pR = add((x, yv, zb))
                G[(a, b)] = pR
                if a > 0:
                    G[(-a, b)] = add((-x, yv, zb))
        for b in range(NS):
            for a in range(-NU, NU):
                Q.append((G[(a, b)], G[(a + 1, b)], G[(a + 1, b + 1)], G[(a, b + 1)]))
        return G

    def front_y(z):
        return float(np.interp(z, [0.27, 0.36, 0.47, 0.57, 0.655, 0.744],
                               [0.80, 0.860, 0.877, 0.873, 0.857, 0.814]))

    def front_set(x):
        return float(np.interp(x, [0, 0.15, 0.30, 0.45, 0.58, 0.70, 0.80],
                               [0, 0.006, 0.022, 0.050, 0.092, 0.150, 0.230]))

    Gf = cap(ring_idx[0], front_y, front_set, (0.36, 0.47, 0.57, 0.655), 0.0)

    def rear_y(z):
        return float(np.interp(z, [0.33, 0.45, 0.60, 0.76, 0.90, 1.03],
                               [-3.47, -3.570, -3.583, -3.572, -3.558, -3.51]))

    def rear_set(x):
        return -float(np.interp(x, [0, 0.15, 0.30, 0.45, 0.58, 0.70, 0.80],
                                [0, 0.005, 0.018, 0.042, 0.080, 0.130, 0.200]))

    Gr = cap(ring_idx[-1], rear_y, rear_set, (0.45, 0.60, 0.76, 0.90), 0.0)

    # ---- creases
    crease = {}

    def col_crease(c, r0_, r1_, w):
        for r in range(r0_, r1_):
            for s in (1, -1):
                k = ring_k(c, s)
                crease[(ring_idx[r][k], ring_idx[r + 1][k])] = w

    def row_crease(r, c0, c1, w):
        for c in range(c0, c1):
            for s in (1, -1):
                crease[(ring_idx[r][ring_k(c, s)], ring_idx[r][ring_k(c + 1, s)])] = w

    col_crease(10, 0, len(rows) - 1, 0.85)      # shoulder line
    row_crease(5, 12, NC, 1.0)                    # cowl / windscreen base
    row_crease(12, 12, NC, 0.9)                   # rear screen base
    row_crease(0, 11, NC, 0.90)                   # hood leading edge
    row_crease(len(rows) - 1, 11, NC, 0.85)       # boot lid trailing edge
    col_crease(11, 0, 5, 0.75)                    # front fender shoulder (crisp)
    col_crease(12, 0, 5, 0.35)                    # front fender crown
    col_crease(11, 11, len(rows) - 1, 0.45)       # rear fender top edge (boot side)
    col_crease(7, 1, len(rows) - 2, 0.40)         # sill / rocker crease
    meta = dict(rows=len(rows), ring_idx=ring_idx, cap_front=Gf, cap_rear=Gr)
    return np.array(V), np.array(Q, dtype=np.int64), crease, meta


# ---------------------------------------------------------------------------
# Low-level mesh helpers (numpy <-> bpy, subdivision, booleans)
# ---------------------------------------------------------------------------

def _mesh_from_vf(name, V, F):
    """bpy mesh from vertices (n,3) and a list of faces (any sizes)."""
    me = bpy.data.meshes.new(name)
    me.from_pydata([tuple(map(float, v)) for v in V], [], [tuple(int(i) for i in f) for f in F])
    me.update()
    return me


def _recalc_outward(me):
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    bm.to_mesh(me)
    bm.free()
    # make total signed volume positive (closed meshes)
    me.calc_loop_triangles()
    co = np.zeros(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    tri = np.zeros(len(me.loop_triangles) * 3, dtype=np.int64)
    me.loop_triangles.foreach_get("vertices", tri)
    tri = tri.reshape(-1, 3)
    if len(tri):
        a, b, c = co[tri[:, 0]], co[tri[:, 1]], co[tri[:, 2]]
        vol = float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum()) / 6.0
        if vol < 0:
            me.flip_normals()
    return me


def _eval_modifiers(me, mods):
    """Apply a list of (type, {prop: value}) modifiers to mesh `me` and
    return a NEW mesh (the input mesh is left untouched)."""
    ob = bpy.data.objects.new("_body_tmp", me)
    bpy.context.scene.collection.objects.link(ob)
    try:
        for typ, props in mods:
            m = ob.modifiers.new(typ.lower(), typ)
            for k, v in props.items():
                setattr(m, k, v)
        dg = bpy.context.evaluated_depsgraph_get()
        dg.update()
        out = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    finally:
        bpy.data.objects.remove(ob)
    return out


def _subdivide(me, levels):
    return _eval_modifiers(me, [("SUBSURF", dict(levels=levels, render_levels=levels, use_creases=True,
                                                 quality=3, boundary_smooth="ALL"))])


def _face_attr(me, name, values=None, dtype="INT"):
    a = me.attributes.get(name)
    if a is None:
        a = me.attributes.new(name, dtype, "FACE")
    if values is not None:
        a.data.foreach_set("value", np.asarray(values))
    return a


def _get_face_attr(me, name, dtype=np.int32):
    a = me.attributes.get(name)
    out = np.zeros(len(me.polygons), dtype=dtype)
    if a is not None:
        a.data.foreach_get("value", out)
    return out


def _boolean(me, cutter_meshes, operation="DIFFERENCE", solver="MANIFOLD"):
    """Manifold boolean of `me` with the union of closed cutter meshes.

    Returns a NEW mesh whose face int attribute 'cvb_orig' is 1 on faces from
    `me` and 0 on faces created from the cutters."""
    tgt = me.copy()
    _face_attr(tgt, "cvb_orig", np.ones(len(tgt.polygons), dtype=np.int32))
    col = bpy.data.collections.new("_body_cutters")
    bpy.context.scene.collection.children.link(col)
    obs = []
    for i, cm in enumerate(cutter_meshes):
        o = bpy.data.objects.new(f"_body_cut{i}", cm)
        col.objects.link(o)
        obs.append(o)
    ob = bpy.data.objects.new("_body_booltgt", tgt)
    bpy.context.scene.collection.objects.link(ob)
    try:
        m = ob.modifiers.new("b", "BOOLEAN")
        m.operation = operation
        m.operand_type = "COLLECTION"
        m.collection = col
        try:
            m.solver = solver
        except TypeError:
            m.solver = "EXACT"
        dg = bpy.context.evaluated_depsgraph_get()
        dg.update()
        out = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    finally:
        bpy.data.objects.remove(ob)
        bpy.data.meshes.remove(tgt)
        for o in obs:
            bpy.data.objects.remove(o)
        bpy.data.collections.remove(col)
    return out


def _delete_faces(me, mask):
    """Delete faces where mask is True (returns me)."""
    mask = np.asarray(mask, dtype=bool)
    if not mask.any():
        return me
    bm = bmesh.new()
    bm.from_mesh(me)
    bm.faces.ensure_lookup_table()
    kill = [bm.faces[i] for i in np.nonzero(mask)[0]]
    bmesh.ops.delete(bm, geom=kill, context="FACES")
    bm.to_mesh(me)
    bm.free()
    me.update()
    return me


def _face_centres_normals(me):
    n = len(me.polygons)
    c = np.zeros(n * 3)
    nn = np.zeros(n * 3)
    me.polygons.foreach_get("center", c)
    me.polygons.foreach_get("normal", nn)
    return c.reshape(-1, 3), nn.reshape(-1, 3)


def _verts(me):
    co = np.zeros(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    return co.reshape(-1, 3)


def _bvh_of(me):
    co = _verts(me)
    polys = [tuple(p.vertices) for p in me.polygons]
    return BVHTree.FromPolygons([Vector(v) for v in co], polys, epsilon=0.0)


# ---------------------------------------------------------------------------
# Closed cutter / insert shapes (numpy geometry -> bpy meshes)
# ---------------------------------------------------------------------------

def _round_poly(pts, radius, n=6):
    """Round every corner of a closed 2D polygon (fillet radius, n segs)."""
    P = np.asarray(pts, dtype=float)
    out = []
    m = len(P)
    for i in range(m):
        a, b, c = P[i - 1], P[i], P[(i + 1) % m]
        d0, d1 = b - a, c - b
        l0, l1 = np.hypot(*d0), np.hypot(*d1)
        if l0 < 1e-9 or l1 < 1e-9:
            continue
        d0, d1 = d0 / l0, d1 / l1
        cosang = float(np.clip(np.dot(d0, d1), -1, 1))
        th = math.acos(cosang)
        r = radius if np.ndim(radius) == 0 else radius[i]
        if th < 1e-3 or r <= 0:
            out.append(b)
            continue
        t = min(r * math.tan(th / 2), 0.45 * l0, 0.45 * l1)
        R = t / math.tan(th / 2)
        p0, p1 = b - d0 * t, b + d1 * t
        cross = d0[0] * d1[1] - d0[1] * d1[0]
        nrm = np.array([-d0[1], d0[0]]) * (1 if cross > 0 else -1)
        cen = p0 + nrm * R
        a0 = math.atan2(*(p0 - cen)[::-1])
        a1 = math.atan2(*(p1 - cen)[::-1])
        da = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
        for k in range(n + 1):
            ang = a0 + da * k / n
            out.append(cen + R * np.array([math.cos(ang), math.sin(ang)]))
    return np.array(out)


def _densify(pts, step, closed=True):
    P = np.asarray(pts, dtype=float)
    seq = np.vstack([P, P[:1]]) if closed else P
    out = []
    for i in range(len(seq) - 1):
        a, b = seq[i], seq[i + 1]
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
        for k in range(n):
            out.append(a + (b - a) * k / n)
    if not closed:
        out.append(seq[-1])
    return np.array(out)


def _prism(poly_uv, O, U, V, N, d0, d1, name="_prism"):
    """Closed prism: 2D polygon in the plane (O; U, V) extruded along N from
    -d0 to +d1.  Returns a bpy mesh (outward normals)."""
    P = np.asarray(poly_uv, dtype=float)
    if 0.5 * np.sum(P[:, 0] * np.roll(P[:, 1], -1) - np.roll(P[:, 0], -1) * P[:, 1]) < 0:
        P = P[::-1]
    O, U, V, N = (np.asarray(v, dtype=float) for v in (O, U, V, N))
    base = O + P[:, :1] * U + P[:, 1:2] * V
    lo = base - N * d0
    hi = base + N * d1
    n = len(P)
    Vt = np.vstack([lo, hi])
    F = []
    for i in range(n):
        j = (i + 1) % n
        F.append((i, j, n + j, n + i))
    tris = mgeo.tessellate_polygon([[Vector((p[0], p[1], 0.0)) for p in P]])
    for t in tris:
        F.append((t[2], t[1], t[0]))
        F.append((n + t[0], n + t[1], n + t[2]))
    me = _mesh_from_vf(name, Vt, F)
    return _recalc_outward(me)


def _cyl_x(yc, zc, r, x0, x1, n=96, name="_cyl"):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    poly = np.stack([yc + r * np.cos(a), zc + r * np.sin(a)], axis=1)
    return _prism(poly, (x0, 0, 0), (0, 1, 0), (0, 0, 1), (1, 0, 0), 0.0, x1 - x0, name)


def _ribbon(P, Nrm, width, depth, closed=False, name="_ribbon", extend=0.0):
    """Closed box-section tube swept along a 3D curve lying on a surface.

    P (n,3) points, Nrm (n,3) surface normals.  Cross-section: +-width/2 in
    the surface tangent plane (perpendicular to the curve), +-depth along the
    normal.  Open curves are extended by `extend` at both ends and capped."""
    P = np.asarray(P, dtype=float).copy()
    Nn = np.asarray(Nrm, dtype=float).copy()
    if not closed and extend > 0:
        t0 = P[0] - P[1]
        t0 /= np.linalg.norm(t0)
        t1 = P[-1] - P[-2]
        t1 /= np.linalg.norm(t1)
        P = np.vstack([P[:1] + t0 * extend, P, P[-1:] + t1 * extend])
        Nn = np.vstack([Nn[:1], Nn, Nn[-1:]])
    n = len(P)
    if closed:
        T = np.roll(P, -1, 0) - np.roll(P, 1, 0)
    else:
        T = np.zeros_like(P)
        T[1:-1] = P[2:] - P[:-2]
        T[0] = P[1] - P[0]
        T[-1] = P[-1] - P[-2]
    T /= np.linalg.norm(T, axis=1)[:, None]
    Nn = Nn - T * np.einsum("ij,ij->i", Nn, T)[:, None]
    Nn /= np.linalg.norm(Nn, axis=1)[:, None]
    B = np.cross(Nn, T)
    corners = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
    Vt = []
    for i in range(n):
        for (s, t) in corners:
            Vt.append(P[i] + s * 0.5 * width * B[i] + t * depth * Nn[i])
    F = []
    m = n if closed else n - 1
    for i in range(m):
        j = (i + 1) % n
        for k in range(4):
            k1 = (k + 1) % 4
            F.append((4 * i + k, 4 * i + k1, 4 * j + k1, 4 * j + k))
    if not closed:
        F.append((3, 2, 1, 0))
        b = 4 * (n - 1)
        F.append((b, b + 1, b + 2, b + 3))
    me = _mesh_from_vf(name, np.array(Vt), F)
    return _recalc_outward(me)


def _project(bvh, pts2d, view, smooth=2):
    """Project 2D points onto the surface by ray casting.

    view: 'x+' (pts are (y,z), ray from +X), 'x-' (from -X), 'z' (pts (x,y),
    from above), 'y+' (pts (x,z), from the front), 'y-' (from the rear).
    Returns (P, N) arrays for the points that hit."""
    P, N = [], []
    for p in pts2d:
        if view == "x+":
            o, d = Vector((3.0, p[0], p[1])), Vector((-1, 0, 0))
        elif view == "x-":
            o, d = Vector((-3.0, p[0], p[1])), Vector((1, 0, 0))
        elif view == "z":
            o, d = Vector((p[0], p[1], 4.0)), Vector((0, 0, -1))
        elif view == "y+":
            o, d = Vector((p[0], 6.0, p[1])), Vector((0, -1, 0))
        elif view == "y-":
            o, d = Vector((p[0], -8.0, p[1])), Vector((0, 1, 0))
        elif view[0] == "polar":
            # p = (phi_deg, z): ray from the vertical axis through (cx, cy),
            # heading phi measured from +Y toward +X (sign of cx mirrors it)
            cx, cy = view[1]
            ph = math.radians(p[0])
            sx = 1.0 if cx >= 0 else -1.0
            d = Vector((sx * math.sin(ph), math.cos(ph), 0.0))
            o = Vector((cx, cy, p[1])) + d * 3.0
            d = -d
        else:
            o, d = Vector(view[0](p)), Vector(view[1])
        hit = bvh.ray_cast(o, d)
        if hit[0] is None:
            continue
        P.append(np.array(hit[0]))
        N.append(np.array(hit[1]))
    if not P:
        return np.zeros((0, 3)), np.zeros((0, 3))
    P, N = np.array(P), np.array(N)
    for _ in range(smooth):
        if len(N) > 2:
            N[1:-1] = (N[:-2] + 2 * N[1:-1] + N[2:]) / 4
    N /= np.linalg.norm(N, axis=1)[:, None]
    return P, N


# ---------------------------------------------------------------------------
# Shell design: openings (cut through), inserts (pieces of the uncut surface
# kept as glass / lenses / trim), panel gaps (surface-following grooves)
# ---------------------------------------------------------------------------
ARCH_R = 0.350           # wheel-arch opening radius (tyre 0.3115 -> ~34 mm gap at rest)
ARCH_ZC = 0.300          # arch centre height (wheel centre 0.305)
ARCH_LIP = 0.025         # arch lip return depth (keeps 7 mm to a bumped tyre's sidewall)
GAP_W = 0.0045           # panel gap width
BELT_Y = (-0.83, -2.78)  # side DLO belt ends
B_PILLAR = (-1.928, -2.006)  # B-pillar band (y at the belt)


def _belt_z(y):
    return float(np.interp(y, [-2.78, -0.83], [1.012, 0.976]))


def _dlo_polys():
    """Side daylight-opening polygons (y, z) for the right side: front door
    glass, rear door glass, B-pillar applique (between them)."""
    lean = 0.012          # B-pillar leans forward toward the top
    zt = 1.312
    bf, br = B_PILLAR
    yA = -0.83
    a_pt = (-1.485, 1.287)        # top-front corner (parallel to the A-pillar)
    front = [(yA, _belt_z(yA)), (bf, _belt_z(bf)), (bf + lean, zt), (-1.70, zt), (-1.56, 1.305), a_pt]
    front = _round_poly(front, [0.012, 0.010, 0.010, 0.0, 0.10, 0.06], 6)
    rear = [(br, _belt_z(br)), (-2.78, 1.030), (-2.62, 1.150), (-2.46, 1.262), (-2.30, 1.300),
            (-2.15, zt), (br + lean, zt)]
    rear = _round_poly(rear, [0.010, 0.02, 0.10, 0.10, 0.10, 0.03, 0.010], 6)
    bp = [(bf - 0.0015, _belt_z(bf) - 0.0), (br + 0.0015, _belt_z(br)), (br + lean + 0.0015, zt),
          (bf + lean - 0.0015, zt)]
    return front, rear, bp


def _screen_frames():
    """(O, U, V, N) frames of the windscreen and rear-screen planes."""
    wO = np.array([0.0, -0.640, 0.957])
    wV = np.array([0.0, -0.820, 0.433])
    wV /= np.linalg.norm(wV)
    wU = np.array([1.0, 0.0, 0.0])
    wN = -np.cross(wU, wV)
    rO = np.array([0.0, -3.160, 1.057])
    rV = np.array([0.0, 0.480, 0.315])
    rV /= np.linalg.norm(rV)
    rU = np.array([1.0, 0.0, 0.0])
    rN = np.cross(rU, rV)
    return (wO, wU, wV, wN), (rO, rU, rV, rN)


def _windscreen_poly():
    pts = [(-0.70, 0.012), (0.70, 0.012), (0.592, 0.858), (0.0, 0.874), (-0.592, 0.858)]
    return _round_poly(pts, [0.05, 0.05, 0.07, 0.0, 0.07], 8)


def _rearscreen_poly():
    pts = [(-0.615, 0.035), (0.0, 0.028), (0.615, 0.035), (0.535, 0.528), (-0.535, 0.528)]
    return _round_poly(pts, [0.05, 0.0, 0.05, 0.07, 0.07], 8)


def _plane_poly(points3d, N):
    """Project 3D points along N onto a plane through their centroid:
    returns (poly_uv, O, U, V)."""
    P = np.asarray(points3d, dtype=float)
    N = np.asarray(N, dtype=float)
    N = N / np.linalg.norm(N)
    O = P.mean(axis=0)
    ref = np.array([0.0, 0.0, 1.0]) if abs(N[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    U = np.cross(ref, N)
    U /= np.linalg.norm(U)
    V = np.cross(N, U)
    d = P - O
    return np.stack([d @ U, d @ V], axis=1), O, U, V


def _outline3d(bvh, pieces):
    """Concatenate projected outline pieces [(view, pts2d), ...] -> (n,3)."""
    out = []
    for view, pts in pieces:
        P, _ = _project(bvh, pts, view, smooth=0)
        out.extend(list(P))
    return np.array(out)


def _mirror_x(P):
    P = np.array(P, dtype=float)
    P[:, 0] *= -1
    return P


LAMP_X_IN = 0.357        # headlamp inner end (front view)
LAMP_Z_BOT = 0.666       # headlamp lower edge (front view)
GRILLE_POLY = [(-0.268, 0.514), (0.268, 0.514), (0.330, 0.703), (-0.330, 0.703)]   # front view (x, z)
INTAKE_POLY = [(-0.520, 0.335), (0.520, 0.335), (0.468, 0.458), (-0.468, 0.458)]
CORNER_VENT = [(0.560, 0.345), (0.700, 0.362), (0.684, 0.468), (0.512, 0.468)]   # right side, front view


def _hood_gap_z(x):
    """Height of the hood front shut line on the nose (front view)."""
    return 0.724 - 0.014 * (x / 0.8) ** 2


def _lamp_outlines(bvh):
    """3D outlines of the right headlight and right tail light (on the shell)."""
    zt = lambda x: _hood_gap_z(x) + 0.005          # noqa: E731  lamp top just above the hood gap
    head = _outline3d(bvh, [
        ("y+", [(LAMP_X_IN, LAMP_Z_BOT), (LAMP_X_IN, zt(LAMP_X_IN))]),
        ("y+", [(x, zt(x)) for x in (0.45, 0.55, 0.65, 0.73)]),
        ("x+", [(0.580, 0.716), (0.540, 0.711), (0.505, 0.700)]),
        ("x+", [(0.520, 0.686), (0.565, 0.672)]),
        ("y+", [(0.73, LAMP_Z_BOT), (0.60, LAMP_Z_BOT), (0.47, LAMP_Z_BOT)]),
    ])
    tail = _outline3d(bvh, [
        ("y-", [(0.386, 0.806), (0.384, 0.958)]),
        ("y-", [(0.52, 0.975), (0.66, 0.985), (0.74, 0.975)]),
        ("x+", [(-3.37, 0.968), (-3.30, 0.958), (-3.235, 0.925)]),
        ("x+", [(-3.25, 0.875), (-3.33, 0.828)]),
        ("y-", [(0.74, 0.806), (0.60, 0.800), (0.48, 0.802)]),
    ])
    return head, tail


def _gap_curves():
    """Panel-gap polylines: list of (view, pts2d, closed)."""
    g = []
    for sgn, view in ((1, "x+"), (-1, "x-")):
        # front door leading edge (from inside the DLO / sail down to the sill)
        g.append((view, [(-0.700, 1.02), (-0.640, 0.86), (-0.620, 0.62), (-0.630, 0.40),
                         (-0.6385, 0.2895)], False))
        # door bottoms along the rocker
        g.append((view, [(-0.6385, 0.2875), (-1.40, 0.284), (-2.00, 0.286), (-2.2285, 0.288)], False))
        # B-pillar line (front door rear / rear door front)
        g.append((view, [(-1.967, 1.02), (-1.972, 0.70), (-1.978, 0.40), (-1.980, 0.2885)], False))
        # rear door rear edge with the dog-leg around the rear arch
        g.append((view, [(-2.80, 1.06), (-2.765, 0.86), (-2.70, 0.735), (-2.55, 0.690), (-2.40, 0.625),
                         (-2.29, 0.520), (-2.235, 0.400), (-2.2285, 0.2905)], False))
        # front bumper upper edge: along the headlamp's lower edge, around the corner, into the arch
        zb = LAMP_Z_BOT - 0.0005
        g.append(("y+", [(sgn * LAMP_X_IN, zb), (sgn * 0.55, zb), (sgn * 0.70, zb), (sgn * 0.78, zb)], False))
        g.append((("polar", (sgn * 0.30, 0.25)),
                  [(40.0, zb), (50, zb), (58, zb - 0.002), (66, zb - 0.010), (74, zb - 0.024), (82, 0.625),
                   (90, 0.605), (97, 0.580), (104, 0.552), (112, 0.52), (122, 0.48)], False))
        # bumper / lamp junction at the lamp's inner end (closes the bumper's top edge)
        g.append(("y+", [(sgn * (LAMP_X_IN + 0.0012), LAMP_Z_BOT - 0.008),
                         (sgn * (LAMP_X_IN + 0.0012), _hood_gap_z(LAMP_X_IN) + 0.007)], False))
        # rear bumper upper edge around the corner: under the tail light -> rear arch
        g.append((("polar", (sgn * 0.30, -3.10)),
                  [(180.0, 0.772), (155, 0.772), (135, 0.772), (118, 0.770), (105, 0.762), (95, 0.738),
                   (88, 0.70), (82, 0.64), (76, 0.58), (70, 0.53), (64, 0.50), (58, 0.47)], False))
        # black rear diffuser / lower valance: rear arch -> around the corner
        g.append((("polar", (sgn * 0.30, -3.10)),
                  [(180.0, 0.415), (150, 0.415), (125, 0.418), (105, 0.42), (90, 0.42), (75, 0.42),
                   (62, 0.42), (52, 0.42)], False))
        # black front lip: front arch -> around the corner
        g.append((("polar", (sgn * 0.30, 0.25)),
                  [(0.0, 0.305), (30, 0.305), (55, 0.307), (75, 0.31), (90, 0.31), (105, 0.31),
                   (120, 0.31)], False))
    # fuel filler flap (right rear quarter)
    fl = _round_poly([(-2.885, 0.800), (-3.035, 0.800), (-3.035, 0.915), (-2.885, 0.915)], 0.025, 5)
    g.append(("x+", list(map(tuple, fl)), True))
    # hood shut lines (top view) and hood front edge (front view)
    for s in (1, -1):
        g.append(("z", [(s * 0.728, -0.60), (s * 0.732, -0.20), (s * 0.730, 0.20), (s * 0.716, 0.45),
                        (s * 0.700, 0.58), (s * 0.688, 0.66), (s * 0.682, 0.688)], False))
    g.append(("y+", [(x, _hood_gap_z(x)) for x in np.linspace(-0.80, 0.80, 17)], False))
    # boot lid: side lines (top view) + rear-face outline (rear view)
    for s in (1, -1):
        g.append(("z", [(s * 0.640, -3.17), (s * 0.650, -3.30), (s * 0.655, -3.42), (s * 0.650, -3.50)],
                  False))
    for s in (1, -1):
        g.append(("y-", [(s * 0.66, 1.03), (s * 0.62, 1.012), (s * 0.372, 0.975), (s * 0.370, 0.900),
                         (s * 0.372, 0.7695)], False))
    # rear bumper top edge (rear view)
    g.append(("y-", [(-0.31, 0.772), (0.0, 0.772), (0.31, 0.772)], False))
    g.append(("y-", [(-0.31, 0.415), (0.0, 0.415), (0.31, 0.415)], False))
    g.append(("y+", [(-0.31, 0.305), (0.0, 0.305), (0.31, 0.305)], False))
    # licence-plate recess outline on the boot lid
    pl = _round_poly([(-0.262, 0.842), (0.262, 0.842), (0.262, 0.952), (-0.262, 0.952)], 0.012, 4)
    g.append(("y-", list(map(tuple, pl)), True))
    return g


def _openings(bvh):
    """Closed cutter meshes for everything cut through the shell, plus the
    insert prisms (same shapes) keyed by insert kind."""
    cut, ins = [], {}
    front, rear, bpil = _dlo_polys()
    for sgn in (1, -1):
        O, N = (sgn * 0.30, 0, 0), (sgn, 0, 0)
        U = (0, 1, 0) if sgn > 0 else (0, -1, 0)
        for poly, kind in ((front, "glass_side"), (rear, "glass_side"), (bpil, "bpillar")):
            P = np.array(poly)
            uv = P.copy() if sgn > 0 else np.stack([-P[:, 0], P[:, 1]], axis=1)
            me = _prism(uv, O, U, (0, 0, 1), N, 0.0, 1.0)
            cut.append(me.copy())
            ins.setdefault(kind, []).append(me)
    (wO, wU, wV, wN), (rO, rU, rV, rN) = _screen_frames()
    me = _prism(_windscreen_poly(), wO, wU, wV, wN, 0.30, 0.30)
    cut.append(me.copy())
    ins.setdefault("glass_wind", []).append(me)
    me = _prism(_rearscreen_poly(), rO, rU, rV, rN, 0.30, 0.30)
    cut.append(me.copy())
    ins.setdefault("glass_rear", []).append(me)
    # mirror sail (black triangle ahead of the front door glass, carries the mirror)
    yA, zA = -0.83, _belt_z(-0.83)
    d = np.array([-0.655, 0.311])
    d /= np.linalg.norm(d)
    nrm2 = np.array([-d[1], d[0]])            # points forward/up, away from the glass
    p1 = np.array([yA, zA]) + nrm2 * 0.003 + np.array([0.003, 0.0])
    p3 = p1 + d * 0.17
    p2 = np.array([-0.672, zA - 0.006])
    sail = _round_poly([p1, p2, p3], [0.004, 0.02, 0.01], 3)
    for sgn in (1, -1):
        O, N = (sgn * 0.30, 0, 0), (sgn, 0, 0)
        U = (0, 1, 0) if sgn > 0 else (0, -1, 0)
        uv = sail.copy() if sgn > 0 else np.stack([-sail[:, 0], sail[:, 1]], axis=1)
        me = _prism(uv, O, U, (0, 0, 1), N, 0.0, 1.0)
        cut.append(me.copy())
        ins.setdefault("sail", []).append(me)
    # cowl panel (black plastic between hood and windscreen; covers the screen base)
    cowl = _round_poly([(-0.725, -0.588), (0.725, -0.588), (0.700, -0.675), (-0.700, -0.675)], 0.02, 4)
    me = _prism(cowl, (0, 0, 0.80), (1, 0, 0), (0, 1, 0), (0, 0, 1), 0.0, 0.40)
    cut.append(me.copy())
    ins.setdefault("cowl", []).append(me)
    # wheel arches
    for y in (S.Y_FRONT_AXLE, S.Y_REAR_AXLE):
        for sgn in (1, -1):
            x0, x1 = (0.45, 1.40) if sgn > 0 else (-1.40, -0.45)
            cut.append(_cyl_x(y, ARCH_ZC, ARCH_R, x0, x1))
    # lamps (projected along a diagonal so the corner wrap is cut cleanly)
    head, tail = _lamp_outlines(bvh)
    for sgn in (1, -1):
        for P3, Nd, kind in ((head, (0.45, 1.0, 0.0), "lens_front"), (tail, (0.55, -1.0, 0.0), "lens_rear")):
            P = P3 if sgn > 0 else _mirror_x(P3)
            Nv = np.array(Nd) * np.array([sgn, 1, 1])
            uv, O, U, V = _plane_poly(P, Nv)
            uv = _round_poly(uv, 0.012, 4)
            Nn = Nv / np.linalg.norm(Nv)
            me = _prism(uv, O, U, V, Nn, 0.35, 0.35)
            cut.append(me.copy())
            ins.setdefault(kind, []).append(me)
    # corner vents in the bumper corners (black ducts carrying the fog lamps),
    # projected along the corner's diagonal so the wrap is cut cleanly
    for sgn in (1, -1):
        P3 = _outline3d(bvh, [("y+", [(sgn * x, z) for (x, z) in CORNER_VENT])])
        Nv = np.array([sgn * 0.40, 1.0, 0.0])
        uv, O, U, V = _plane_poly(P3, Nv)
        uv = _round_poly(uv, 0.014, 4)
        me = _prism(uv, O, U, V, Nv / np.linalg.norm(Nv), 0.35, 0.35)
        cut.append(me.copy())
        ins.setdefault("vent", []).append(me)
    # grille + lower intake (front view, along -Y)
    grille = _round_poly(GRILLE_POLY, [0.040, 0.040, 0.022, 0.022], 6)
    intake = _round_poly(INTAKE_POLY, 0.035, 6)
    for poly, kind in ((grille, "grille"), (intake, "intake")):
        me = _prism(poly, (0, 0.40, 0), (1, 0, 0), (0, 0, 1), (0, 1, 0), 0.0, 0.80)
        cut.append(me.copy())
        ins.setdefault(kind, []).append(me)
    return cut, ins


def _gap_cutters(bvh):
    out = []
    for view, pts, closed in _gap_curves():
        polar = isinstance(view, tuple) and view[0] == "polar"
        if polar:   # (deg, z): ~8 mm of arc per step at the corner radius
            P2 = _densify(np.asarray(pts) * np.array([1.0, 100.0]), 1.0, closed=closed) / np.array([1.0, 100.0])
        else:
            P2 = _densify(pts, 0.008, closed=closed)
        P, N = _project(bvh, P2, view, smooth=3)
        if len(P) < 3:
            continue
        out.append(_ribbon(P, N, GAP_W, 0.012, closed=closed, extend=0.0 if closed else 0.004))
    return out


# ---------------------------------------------------------------------------
# Shell construction
# ---------------------------------------------------------------------------

def _cage_mesh():
    V, Q, crease, meta = shell_cage()
    me = _mesh_from_vf("body_cage", V, Q)
    _recalc_outward(me)
    ek = {tuple(sorted(e.vertices)): e.index for e in me.edges}
    cr = np.zeros(len(me.edges), dtype=np.float32)
    for (a, b), w in crease.items():
        i = ek.get(tuple(sorted((a, b))))
        if i is not None:
            cr[i] = w
    at = me.attributes.new("crease_edge", "FLOAT", "EDGE")
    at.data.foreach_set("value", cr)
    return me


def _offset_mesh(me, dist):
    """Move every vertex along its normal by dist (in place)."""
    bm = bmesh.new()
    bm.from_mesh(me)
    bm.normal_update()
    for v in bm.verts:
        v.co += v.normal * dist
    bm.to_mesh(me)
    bm.free()
    me.update()
    return me


def _flange(me, depth_fn, chamfer=0.0016, skip_fn=None, dir_fn=None):
    """Turn every boundary edge loop of an open shell inward (panel-edge
    return), with a small 45-degree chamfer that catches a highlight.

    depth_fn(co, normal) -> flange depth (m); skip_fn(co, normal) -> True to
    leave that boundary edge alone (e.g. the hidden underside cut)."""
    bm = bmesh.new()
    bm.from_mesh(me)
    bm.normal_update()
    bnd = [e for e in bm.edges if e.is_boundary]
    if skip_fn is not None:
        bnd = [e for e in bnd if not skip_fn(np.array((e.verts[0].co + e.verts[1].co) / 2),
                                             np.array(e.link_faces[0].normal))]
    if not bnd:
        bm.free()
        return me
    vset = {v for e in bnd for v in e.verts}
    dout = {v: Vector((0, 0, 0)) for v in vset}
    for e in bnd:
        f = e.link_faces[0]
        t = e.verts[1].co - e.verts[0].co
        d = t.cross(f.normal)
        if d.length < 1e-12:
            continue
        d.normalize()
        mid = (e.verts[0].co + e.verts[1].co) / 2
        if d.dot(mid - f.calc_center_median()) < 0:
            d = -d
        for v in e.verts:
            dout[v] += d
    nrm = {}
    for v in vset:
        n = Vector((0, 0, 0))
        for f in v.link_faces:
            n += f.normal
        nrm[v] = n.normalized() if n.length > 1e-12 else Vector((0, 0, 1))
        if dout[v].length > 1e-12:
            dout[v].normalize()
    depth = {v: float(depth_fn(np.array(v.co), np.array(nrm[v]))) for v in vset}
    inward = {v: -nrm[v] for v in vset}
    if dir_fn is not None:
        for v in vset:
            d = dir_fn(np.array(v.co), np.array(nrm[v]))
            if d is not None:
                inward[v] = Vector(d).normalized()

    def _extrude(edges, move):
        res = bmesh.ops.extrude_edge_only(bm, edges=edges)
        newv = [g for g in res["geom"] if isinstance(g, bmesh.types.BMVert)]
        newe = [g for g in res["geom"] if isinstance(g, bmesh.types.BMEdge)]
        src = {}
        for v in newv:
            for e in v.link_edges:
                o = e.other_vert(v)
                if o in move:
                    src[v] = o
                    break
        return newv, newe, src

    move1 = {v: v for v in vset}
    nv1, ne1, src1 = _extrude(bnd, move1)
    root = {}
    for v in nv1:
        s = src1.get(v)
        if s is None:
            continue
        root[v] = s
        v.co = s.co - nrm[s] * chamfer + dout[s] * (0.6 * chamfer)
    b2 = [e for e in ne1 if e.is_boundary]
    nv2, ne2, src2 = _extrude(b2, {v: v for v in nv1})
    for v in nv2:
        s = src2.get(v)
        if s is None or s not in root:
            continue
        r = root[s]
        v.co = s.co + inward[r] * max(depth[r] - chamfer, 0.001) + dout[r] * (0.25 * chamfer)
    bm.to_mesh(me)
    bm.free()
    me.update()
    return me


def _face_islands(me):
    """Connected components of faces (sharing edges).  Returns labels array."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    nl = len(me.loops)
    le = np.zeros(nl, dtype=np.int64)
    me.loops.foreach_get("edge_index", le)
    ls = np.zeros(len(me.polygons), dtype=np.int64)
    lt = np.zeros(len(me.polygons), dtype=np.int64)
    me.polygons.foreach_get("loop_start", ls)
    me.polygons.foreach_get("loop_total", lt)
    lf = np.repeat(np.arange(len(me.polygons)), lt)
    order = np.argsort(le, kind="stable")
    e_sorted, f_sorted = le[order], lf[order]
    same = e_sorted[1:] == e_sorted[:-1]
    a, b = f_sorted[:-1][same], f_sorted[1:][same]
    n = len(me.polygons)
    g = coo_matrix((np.ones(len(a)), (a, b)), shape=(n, n))
    _, lab = connected_components(g, directed=False)
    return lab


def build_shell_meshes(detail="high", log=None):
    """Return dict of bpy meshes: 'shell' (face attr 'body_part': 0 shell,
    1 front bumper, 2 rear bumper, 3 rear diffuser, 4 front lip), 'closed'
    (the uncut subdivided surface) and the inserts keyed by kind."""
    t0 = time.time()
    levels = 4 if detail == "high" else 3
    cage = _cage_mesh()
    closed = _subdivide(cage, levels)
    bpy.data.meshes.remove(cage)
    bvh = _bvh_of(closed)
    cut, ins = _openings(bvh)
    gaps = _gap_cutters(bvh)
    t1 = time.time()
    shell = _boolean(closed, cut + gaps, "DIFFERENCE")
    orig = _get_face_attr(shell, "cvb_orig")
    if not (orig == 0).any() or len(shell.polygons) < 0.5 * len(closed.polygons):
        # Manifold solver refused the input (should not happen: everything is closed)
        bpy.data.meshes.remove(shell)
        shell = _boolean(closed, cut + gaps, "DIFFERENCE", solver="EXACT")
        orig = _get_face_attr(shell, "cvb_orig")
    if not (orig == 0).any():
        raise RuntimeError("body shell boolean produced no cuts")
    _delete_faces(shell, orig == 0)
    # remove the flat underside between the sills (the floor pan is a separate part)
    c, n = _face_centres_normals(shell)
    under = (n[:, 2] < -0.45) & (c[:, 2] < 0.30) & (np.abs(c[:, 0]) < 0.765) & (c[:, 1] < 0.50) & (c[:, 1] > -3.30)
    _delete_faces(shell, under)
    t2 = time.time()

    def depth_fn(co, nrm):
        x, y, z = co
        dy_f, dy_r = y - S.Y_FRONT_AXLE, y - S.Y_REAR_AXLE
        for dy in (dy_f, dy_r):
            if abs(x) > 0.5 and math.hypot(dy, z - ARCH_ZC) < ARCH_R + 0.01:
                return ARCH_LIP               # wheel-arch lip
        if z > 0.95 and abs(x) < 0.85:
            return 0.018                  # window frames
        return 0.010                      # panel gaps, lamp and grille openings

    def skip_fn(co, nrm):
        return co[2] < 0.30 and nrm[2] < -0.3 and abs(co[0]) < 0.80

    def dir_fn(co, nrm):
        # arch lips return straight inward (constant radius about the axle) so a
        # bumped / steered tyre never meets them; everything else follows -normal
        x, y, z = co
        for ya in (S.Y_FRONT_AXLE, S.Y_REAR_AXLE):
            if abs(x) > 0.5 and math.hypot(y - ya, z - ARCH_ZC) < ARCH_R + 0.01:
                return (-math.copysign(1.0, x), 0.0, 0.0)
        return None

    _flange(shell, depth_fn, skip_fn=skip_fn, dir_fn=dir_fn)
    lab = _face_islands(shell)
    c, _ = _face_centres_normals(shell)
    part = np.zeros(len(shell.polygons), dtype=np.int32)
    for k in np.unique(lab):
        sel = lab == k
        cy = c[sel, 1]
        cz = c[sel, 2]
        if sel.sum() < 400:
            # small islands (fuel flap, plate mounts): follow the panel around them
            if cy.mean() > 0.45 and cz.mean() < 0.62:
                part[sel] = 1
            continue
        if cy.mean() > 0.40 and cz.mean() < 0.32 and cz.max() < 0.36:
            part[sel] = 4                     # black front lip
        elif cy.mean() < -3.0 and cz.mean() < 0.40 and cz.max() < 0.45:
            part[sel] = 3                     # black rear diffuser / valance
        elif cy.min() > 0.12 and cy.mean() > 0.45:
            part[sel] = 1
        elif cy.max() < -2.55 and cy.mean() < -3.0:
            part[sel] = 2
    _face_attr(shell, "body_part", part)
    t3 = time.time()
    inserts = {}
    for kind, mes in ins.items():
        bm = bmesh.new()
        for m in mes:      # one INTERSECT per prism (a collection operand would intersect them all)
            piece = _boolean(closed, [m], "INTERSECT")
            orig = _get_face_attr(piece, "cvb_orig")
            _delete_faces(piece, orig == 0)
            bm.from_mesh(piece)
            bpy.data.meshes.remove(piece)
            bpy.data.meshes.remove(m)
        out = bpy.data.meshes.new(f"body_ins_{kind}")
        bm.to_mesh(out)
        bm.free()
        inserts[kind] = out
    for m in cut + gaps:
        bpy.data.meshes.remove(m)
    t4 = time.time()
    if log is not None:
        log.update(subd=t1 - t0, diff=t2 - t1, flange=t3 - t2, inserts=t4 - t3,
                   islands=int(len(np.unique(lab))), front_bumper=int((part == 1).sum()),
                   rear_bumper=int((part == 2).sum()), diffuser=int((part == 3).sum()),
                   front_lip=int((part == 4).sum()))
    return dict(shell=shell, closed=closed, **inserts)


# ---------------------------------------------------------------------------
# Materials (shared library first; local look-dev materials for things the
# shared table does not have - lamp lenses, upholstery, headliner, LED guides)
# ---------------------------------------------------------------------------
_LOCAL_VERSION = 6


def _mat(name):
    """Shared material (carviz.materials.get) with a placeholder fallback."""
    try:
        from .. import materials as _m
        return _m.get(name)
    except Exception:
        pass
    try:
        from .. import meshutil as _mu
        return _mu.get_material(name)
    except Exception:
        m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
        return m


def _presentation_wrap(m, shader_out, tint=(1, 1, 1)):
    nt = m.node_tree
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    try:
        from .. import materials as _m
        grp = nt.nodes.new("ShaderNodeGroup")
        grp.node_tree = _m.presentation_group()
        nt.links.new(shader_out, grp.inputs["Shader"])
        try:
            grp.inputs["Glow Tint"].default_value = (*tint, 1.0)
        except Exception:
            pass
        nt.links.new(grp.outputs["Shader"], out.inputs["Surface"])
    except Exception:
        # local fallback: same opacity logic as the shared CV_Presentation group
        att = nt.nodes.new("ShaderNodeAttribute")
        att.attribute_type = "OBJECT"
        att.attribute_name = "cv_opacity"
        mixf = nt.nodes.new("ShaderNodeMath")
        mixf.operation = "MULTIPLY_ADD"     # 1 + alpha*(fac-1)
        sub = nt.nodes.new("ShaderNodeMath")
        sub.operation = "SUBTRACT"
        nt.links.new(att.outputs["Fac"], sub.inputs[0])
        sub.inputs[1].default_value = 1.0
        nt.links.new(sub.outputs[0], mixf.inputs[0])
        nt.links.new(att.outputs["Alpha"], mixf.inputs[1])
        mixf.inputs[2].default_value = 1.0
        tr = nt.nodes.new("ShaderNodeBsdfTransparent")
        mx = nt.nodes.new("ShaderNodeMixShader")
        nt.links.new(mixf.outputs[0], mx.inputs[0])
        nt.links.new(tr.outputs[0], mx.inputs[1])
        nt.links.new(shader_out, mx.inputs[2])
        nt.links.new(mx.outputs[0], out.inputs["Surface"])


def _local_mat(name):
    m = bpy.data.materials.get(name)
    if m is not None and m.get("cv_body_version") == _LOCAL_VERSION and m.node_tree is not None:
        return m
    if m is None:
        m = bpy.data.materials.new(name)
    try:
        m.use_nodes = True
    except Exception:
        pass
    nt = m.node_tree
    nt.nodes.clear()

    def principled(base, rough, metallic=0.0, sheen=0.0, coat=0.0, spec=0.5):
        p = nt.nodes.new("ShaderNodeBsdfPrincipled")
        p.inputs["Base Color"].default_value = (*base, 1.0)
        p.inputs["Roughness"].default_value = rough
        p.inputs["Metallic"].default_value = metallic
        p.inputs["Specular IOR Level"].default_value = spec
        if sheen:
            p.inputs["Sheen Weight"].default_value = sheen
            p.inputs["Sheen Roughness"].default_value = 0.6
        if coat:
            p.inputs["Coat Weight"].default_value = coat
            p.inputs["Coat Roughness"].default_value = 0.05
        return p.outputs[0]

    def thin_glass(tint):
        fres = nt.nodes.new("ShaderNodeFresnel")
        fres.inputs["IOR"].default_value = 1.5
        tr = nt.nodes.new("ShaderNodeBsdfTransparent")
        tr.inputs["Color"].default_value = (*tint, 1.0)
        gl = nt.nodes.new("ShaderNodeBsdfGlossy")
        gl.inputs["Roughness"].default_value = 0.03
        lp = nt.nodes.new("ShaderNodeLightPath")
        inv = nt.nodes.new("ShaderNodeMath")
        inv.operation = "SUBTRACT"
        inv.inputs[0].default_value = 1.0
        nt.links.new(lp.outputs["Is Shadow Ray"], inv.inputs[1])
        mul = nt.nodes.new("ShaderNodeMath")
        mul.operation = "MULTIPLY"
        nt.links.new(fres.outputs[0], mul.inputs[0])
        nt.links.new(inv.outputs[0], mul.inputs[1])
        mx = nt.nodes.new("ShaderNodeMixShader")
        nt.links.new(mul.outputs[0], mx.inputs[0])
        nt.links.new(tr.outputs[0], mx.inputs[1])
        nt.links.new(gl.outputs[0], mx.inputs[2])
        return mx.outputs[0]

    def emission(col, strength):
        em = nt.nodes.new("ShaderNodeEmission")
        em.inputs["Color"].default_value = (*col, 1.0)
        em.inputs["Strength"].default_value = strength
        tr = nt.nodes.new("ShaderNodeBsdfTransparent")
        lp = nt.nodes.new("ShaderNodeLightPath")      # lines are invisible to everything but the camera
        mx = nt.nodes.new("ShaderNodeMixShader")
        nt.links.new(lp.outputs["Is Camera Ray"], mx.inputs[0])
        nt.links.new(tr.outputs[0], mx.inputs[1])
        nt.links.new(em.outputs[0], mx.inputs[2])
        return mx.outputs[0]

    spec = {
        # name: (builder, viewport rgba, tint)
        "body_lens_red": (lambda: thin_glass((0.85, 0.035, 0.025)), (0.55, 0.03, 0.02, 0.6), (1, 0.2, 0.1)),
        "body_lens_clear": (lambda: thin_glass((0.92, 0.93, 0.95)), (0.8, 0.82, 0.85, 0.3), (1, 1, 1)),
        "body_lamp_red": (lambda: principled((0.30, 0.012, 0.010), 0.25, coat=0.5), (0.3, 0.02, 0.02, 1),
                          (1, 0.2, 0.1)),
        "body_led": (lambda: principled((0.80, 0.80, 0.78), 0.35), (0.85, 0.85, 0.85, 1), (1, 1, 1)),
        "body_fabric": (lambda: principled((0.030, 0.030, 0.033), 0.92, sheen=0.25, spec=0.2),
                        (0.05, 0.05, 0.055, 1), (1, 1, 1)),
        "body_headliner": (lambda: principled((0.30, 0.29, 0.27), 0.95, sheen=0.3, spec=0.2),
                           (0.4, 0.39, 0.37, 1), (1, 1, 1)),
        "body_gloss_black": (lambda: principled((0.006, 0.006, 0.007), 0.08, coat=1.0), (0.02, 0.02, 0.02, 1),
                             (1, 1, 1)),
        "body_grey_trim": (lambda: principled((0.10, 0.10, 0.105), 0.45), (0.12, 0.12, 0.13, 1), (1, 1, 1)),
        "body_lamp_silver": (lambda: principled((0.62, 0.63, 0.65), 0.22, metallic=1.0), (0.6, 0.6, 0.62, 1),
                             (1, 1, 1)),
        "body_xray_line": (lambda: emission((0.70, 0.82, 1.0), 1.6), (0.7, 0.82, 1.0, 1), (0.7, 0.82, 1.0)),
    }[name]
    sh = spec[0]()
    _presentation_wrap(m, sh, spec[2])
    m.diffuse_color = spec[1]
    try:
        m.surface_render_method = "BLENDED" if spec[1][3] < 1 else "DITHERED"
        m.cycles.emission_sampling = "NONE"
    except Exception:
        pass
    m["cv_body_version"] = _LOCAL_VERSION
    return m


def _get_mat(name):
    return _local_mat(name) if name.startswith("body_") else _mat(name)


# ---------------------------------------------------------------------------
# Generic geometry helpers for the detail parts
# ---------------------------------------------------------------------------

def _boundary_loops(me):
    """Ordered boundary loops of a mesh: list of (P (n,3), N (n,3)) using
    vertex normals.  Loops shorter than 4 vertices are dropped."""
    bm = bmesh.new()
    bm.from_mesh(me)
    bm.normal_update()
    adj = {}
    for e in bm.edges:
        if e.is_boundary:
            a, b = e.verts
            adj.setdefault(a, []).append(b)
            adj.setdefault(b, []).append(a)
    loops, seen = [], set()
    for start in list(adj):
        if start in seen:
            continue
        loop, prev, cur = [start], None, start
        seen.add(start)
        while True:
            nxt = [n for n in adj[cur] if n is not prev and n not in seen]
            if not nxt:
                break
            prev, cur = cur, nxt[0]
            loop.append(cur)
            seen.add(cur)
        if len(loop) >= 4:
            P = np.array([tuple(v.co) for v in loop])
            N = np.array([tuple(v.normal) for v in loop])
            loops.append((P, N))
    bm.free()
    return loops


def _resample_loop(P, N, step):
    """Resample a closed loop evenly (arc length), carrying normals."""
    Q = np.vstack([P, P[:1]])
    NN = np.vstack([N, N[:1]])
    seg = np.linalg.norm(np.diff(Q, axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    L = s[-1]
    n = max(8, int(L / step))
    t = np.linspace(0, L, n, endpoint=False)
    Pn = np.stack([np.interp(t, s, Q[:, k]) for k in range(3)], axis=1)
    Nn = np.stack([np.interp(t, s, NN[:, k]) for k in range(3)], axis=1)
    Nn /= np.linalg.norm(Nn, axis=1)[:, None]
    return Pn, Nn


def _smooth_loop(A, it=2):
    for _ in range(it):
        A = (np.roll(A, 1, 0) + 2 * A + np.roll(A, -1, 0)) / 4
    return A


def _loop_frames(P, N, centre=None):
    """Tangent T, normal N, outward-in-surface B for a closed loop."""
    T = np.roll(P, -1, 0) - np.roll(P, 1, 0)
    T /= np.linalg.norm(T, axis=1)[:, None]
    N = N - T * np.einsum("ij,ij->i", N, T)[:, None]
    N /= np.linalg.norm(N, axis=1)[:, None]
    B = np.cross(T, N)
    c = P.mean(axis=0) if centre is None else centre
    if np.mean(np.einsum("ij,ij->i", B, P - c)) < 0:
        B = -B
    return T, N, B


def _sweep(P, N, profile_bn, closed=True, centre=None, B=None):
    """Sweep a closed 2D profile (b, n) along a loop P with surface normals N.
    b points away from the loop's enclosed region (outward in the surface)."""
    if B is None:
        _, N, B = _loop_frames(P, N, centre)
    prof = np.asarray(profile_bn, dtype=float)
    m, n = len(prof), len(P)
    V = (P[:, None, :] + prof[None, :, 0:1] * B[:, None, :] + prof[None, :, 1:2] * N[:, None, :]).reshape(-1, 3)
    F = []
    rng = n if closed else n - 1
    for i in range(rng):
        j = (i + 1) % n
        for k in range(m):
            k1 = (k + 1) % m
            F.append((i * m + k, i * m + k1, j * m + k1, j * m + k))
    if not closed:
        F.append(tuple(range(m))[::-1])
        F.append(tuple((n - 1) * m + k for k in range(m)))
    return V, F


def _rounded_rect_profile(b0, b1, n0, n1, r, seg=3):
    """Closed profile polygon (rounded rectangle) in the (b, n) plane."""
    r = min(r, 0.49 * (b1 - b0), 0.49 * (n1 - n0))
    pts = []
    for (cb, cn, a0) in ((b1 - r, n1 - r, 0.0), (b0 + r, n1 - r, 90.0), (b0 + r, n0 + r, 180.0),
                         (b1 - r, n0 + r, 270.0)):
        for k in range(seg + 1):
            a = math.radians(a0 + 90.0 * k / seg)
            pts.append((cb + r * math.cos(a), cn + r * math.sin(a)))
    return pts


class _Acc:
    """Accumulate (V, F, material-slot) pieces into one mesh object."""

    def __init__(self):
        self.V, self.F, self.M = [], [], []
        self.nv = 0
        self.sharp = []

    def add(self, V, F, mat=0):
        V = np.asarray(V, dtype=float).reshape(-1, 3)
        self.V.append(V)
        for f in F:
            self.F.append(tuple(int(i) + self.nv for i in f))
            self.M.append(mat)
        self.nv += len(V)

    def add_mesh(self, me, mat=0, matrix=None):
        co = _verts(me)
        if matrix is not None:
            M4 = np.array(matrix)
            co = co @ M4[:3, :3].T + M4[:3, 3]
        self.add(co, [tuple(p.vertices) for p in me.polygons], mat)

    def empty(self):
        return self.nv == 0

    def to_object(self, name, mats, col, smooth_angle=40.0, fix=False):
        V = np.vstack(self.V) if self.V else np.zeros((0, 3))
        me = _mesh_from_vf(name, V, self.F)
        me.polygons.foreach_set("material_index", np.array(self.M, dtype=np.int32))
        if fix:
            bm = bmesh.new()
            bm.from_mesh(me)
            bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
            bm.to_mesh(me)
            bm.free()
        for m in mats:
            me.materials.append(_get_mat(m))
        ob = bpy.data.objects.new(name, me)
        col.objects.link(ob)
        _smooth(ob, smooth_angle)
        return ob


def _smooth(ob, angle_deg=40.0):
    me = ob.data
    me.shade_smooth()
    try:
        me.set_sharp_from_angle(angle=math.radians(angle_deg))
    except Exception:
        pass


def _box_mesh(size, radius=0.0, seg=2, center=(0, 0, 0)):
    """Rounded box (bmesh bevel) -> (V, F)."""
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bmesh.ops.scale(bm, vec=Vector(size), verts=bm.verts[:])
    r = min(radius, 0.49 * min(size))
    if r > 0:
        bmesh.ops.bevel(bm, geom=bm.edges[:] + bm.verts[:], offset=r, offset_type="OFFSET", segments=seg,
                        profile=0.5, affect="EDGES", clamp_overlap=True)
    bmesh.ops.translate(bm, vec=Vector(center), verts=bm.verts[:])
    V = np.array([tuple(v.co) for v in bm.verts])
    F = [tuple(v.index for v in f.verts) for f in bm.faces]
    bm.free()
    return V, F


def _subd_box(size, levels=2, center=(0, 0, 0), taper=None):
    """Smooth rounded block: a cube cage (HALF-extents `size`, cage
    coordinates in [-0.5, 0.5] passed to `taper`) subdivided -> (V, F)."""
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bmesh.ops.subdivide_edges(bm, edges=bm.edges[:], cuts=1, use_grid_fill=True)
    for v in bm.verts:
        x, y, z = v.co
        if taper is not None:
            x, y, z = taper(x, y, z)
        v.co = Vector((2 * x * size[0], 2 * y * size[1], 2 * z * size[2]))
    me = bpy.data.meshes.new("_sbox")
    bm.to_mesh(me)
    bm.free()
    sub = _subdivide(me, levels)
    bpy.data.meshes.remove(me)
    V = _verts(sub) + np.asarray(center)
    F = [tuple(p.vertices) for p in sub.polygons]
    bpy.data.meshes.remove(sub)
    return V, F


def _lathe_vf(profile_rz, seg=32, axis=(0, 0, 1), origin=(0, 0, 0), up=None):
    """Revolve (r, h) profile about `axis` through origin -> (V, F)."""
    ax = np.asarray(axis, dtype=float)
    ax /= np.linalg.norm(ax)
    ref = np.array([1.0, 0, 0]) if abs(ax[0]) < 0.9 else np.array([0, 1.0, 0])
    if up is not None:
        ref = np.asarray(up, dtype=float)
    e1 = np.cross(ax, ref)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(ax, e1)
    prof = np.asarray(profile_rz, dtype=float)
    a = np.linspace(0, 2 * math.pi, seg, endpoint=False)
    V = []
    for r, h in prof:
        for t in a:
            V.append(np.asarray(origin) + ax * h + r * (math.cos(t) * e1 + math.sin(t) * e2))
    F = []
    m = len(prof)
    for i in range(m - 1):
        for k in range(seg):
            k1 = (k + 1) % seg
            F.append((i * seg + k, i * seg + k1, (i + 1) * seg + k1, (i + 1) * seg + k))
    if prof[0][0] > 1e-9:
        F.append(tuple(range(seg))[::-1])
    if prof[-1][0] > 1e-9:
        F.append(tuple((m - 1) * seg + k for k in range(seg)))
    return np.array(V), F


def _tube_vf(points, radius, seg=10, closed=False):
    """Circular tube along a polyline -> (V, F) (capped if open)."""
    P = np.asarray(points, dtype=float)
    n = len(P)
    T = np.zeros_like(P)
    if closed:
        T = np.roll(P, -1, 0) - np.roll(P, 1, 0)
    else:
        T[1:-1] = P[2:] - P[:-2]
        T[0] = P[1] - P[0]
        T[-1] = P[-1] - P[-2]
    T /= np.linalg.norm(T, axis=1)[:, None]
    ref = np.array([0, 0, 1.0]) if abs(T[0][2]) < 0.9 else np.array([1.0, 0, 0])
    N0 = np.cross(T[0], ref)
    N0 /= np.linalg.norm(N0)
    Ns = [N0]
    for i in range(1, n):
        Nn = Ns[-1] - T[i] * np.dot(Ns[-1], T[i])
        Nn /= np.linalg.norm(Nn)
        Ns.append(Nn)
    Ns = np.array(Ns)
    Bs = np.cross(T, Ns)
    a = np.linspace(0, 2 * math.pi, seg, endpoint=False)
    prof = [(radius * math.cos(t), radius * math.sin(t)) for t in a]
    V = (P[:, None, :] + np.array(prof)[None, :, 0:1] * Ns[:, None, :]
         + np.array(prof)[None, :, 1:2] * Bs[:, None, :]).reshape(-1, 3)
    F = []
    rng = n if closed else n - 1
    for i in range(rng):
        j = (i + 1) % n
        for k in range(seg):
            k1 = (k + 1) % seg
            F.append((i * seg + k, i * seg + k1, j * seg + k1, j * seg + k))
    if not closed:
        F.append(tuple(range(seg))[::-1])
        F.append(tuple((n - 1) * seg + k for k in range(seg)))
    return V, F


def _frame_matrix(origin, x_axis, y_axis):
    """4x4 matrix with given origin and (orthonormalised) X, Y axes."""
    x = np.asarray(x_axis, float)
    x /= np.linalg.norm(x)
    y = np.asarray(y_axis, float)
    y = y - x * np.dot(x, y)
    y /= np.linalg.norm(y)
    z = np.cross(x, y)
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = x, y, z, origin
    return M


def _xform(V, M):
    V = np.asarray(V, float)
    return V @ M[:3, :3].T + M[:3, 3]


# ---------------------------------------------------------------------------
# Exterior detail parts
# ---------------------------------------------------------------------------

def _islands_of(me):
    """Split a mesh into connected pieces -> list of new meshes."""
    lab = _face_islands(me)
    out = []
    for k in np.unique(lab):
        sel = np.nonzero(lab == k)[0]
        bm = bmesh.new()
        bm.from_mesh(me)
        bm.faces.ensure_lookup_table()
        keep = set(int(i) for i in sel)
        kill = [f for f in bm.faces if f.index not in keep]
        bmesh.ops.delete(bm, geom=kill, context="FACES")
        m = bpy.data.meshes.new("_island")
        bm.to_mesh(m)
        bm.free()
        out.append(m)
    return out


def _window_frames(acc, glass_mes, mat):
    """Black rubber/plastic surround swept along every glass pane edge."""
    prof = _rounded_rect_profile(-0.015, 0.0045, -0.005, 0.0062, 0.0022, 2)
    for me in glass_mes:
        for P, N in _boundary_loops(me):
            P, N = _resample_loop(P, N, 0.006)
            N = _smooth_loop(N, 3)
            N /= np.linalg.norm(N, axis=1)[:, None]
            V, F = _sweep(P, N, prof)
            acc.add(V, F, mat)


def _frit_bands(acc, screen_mes, mat, width=0.055):
    """Black ceramic frit band printed on the inside of the windscreen and
    rear screen along their edges (sits 1.5 mm behind the glass)."""
    for me in screen_mes:
        for P, N in _boundary_loops(me):
            P, N = _resample_loop(P, N, 0.008)
            N = _smooth_loop(N, 3)
            N /= np.linalg.norm(N, axis=1)[:, None]
            V, F = _sweep(P, N, [(-width, -0.0015), (0.0, -0.0015), (0.0, -0.0025), (-width, -0.0025)])
            acc.add(V, F, mat)


def _lamp_axes(P3, outward):
    A = np.asarray(outward, float)
    A /= np.linalg.norm(A)
    V = np.array([0.0, 0.0, 1.0])
    U = np.cross(V, A)
    U /= np.linalg.norm(U)
    V = np.cross(A, U)
    return A, U, V


def _lamp_unit(acc_lens, acc_body, lens_me, outward, kind, mats):
    """Lens (piece of the shell surface), housing walls + back, reflectors."""
    m_lens, m_house, m_chrome, m_led, m_inner = mats
    _offset_mesh(lens_me, -0.0015)
    acc_lens.add_mesh(lens_me, m_lens)
    loops = _boundary_loops(lens_me)
    if not loops:
        return
    P, N = max(loops, key=lambda l: len(l[0]))
    P, N = _resample_loop(P, N, 0.008)
    A, U, V = _lamp_axes(P, outward)
    c = P.mean(axis=0)
    depth = 0.11 if kind == "front" else 0.075
    # housing wall: from just outside the lens edge, straight back along -A
    _, Nn, B = _loop_frames(P, N)
    rings = [P + B * 0.004 - A * 0.002, P + B * 0.002 - A * 0.03, P - B * 0.004 - A * depth]
    m = len(P)
    Vh = np.vstack(rings)
    F = []
    for r in range(len(rings) - 1):
        for i in range(m):
            j = (i + 1) % m
            F.append((r * m + i, r * m + j, (r + 1) * m + j, (r + 1) * m + i))
    back = rings[-1]
    uv = np.stack([(back - c) @ U, (back - c) @ V], axis=1)
    tris = mgeo.tessellate_polygon([[Vector((p[0], p[1], 0.0)) for p in uv]])
    base = (len(rings) - 1) * m
    for t in tris:
        F.append((base + t[0], base + t[1], base + t[2]))
    acc_body.add(Vh, F, m_inner if kind == "rear" else m_house)
    # extents in lamp coordinates
    pu, pv = (P - c) @ U, (P - c) @ V
    u0, u1, v0, v1 = pu.min(), pu.max(), pv.min(), pv.max()

    def surf_depth(u, v):
        # lens depth along A at lamp coords (u, v): nearest loop point (approx.)
        k = np.argmin((pu - u) ** 2 + (pv - v) ** 2)
        return float((P[k] - c) @ A)

    if kind == "front":
        # two compact projector modules (chrome bowl, clear condenser lens, black
        # bezel ring) in a silver reflector housing - they read through the clear lens
        span = u1 - u0
        vv = 0.5 * (v0 + v1) - 0.002
        rad = 0.42 * (v1 - v0)
        for frac in (0.20, 0.42):
            u = u0 + frac * span if outward[0] > 0 else u1 - frac * span
            a0 = surf_depth(u, vv) - 0.030
            org = c + U * u + V * vv + A * a0
            bowl = [(0.0, -0.040), (rad * 0.55, -0.036), (rad * 0.9, -0.022), (rad * 1.10, 0.0),
                    (rad * 1.14, 0.003)]
            Vb, Fb = _lathe_vf(bowl, 28, axis=A, origin=org)
            acc_body.add(Vb, Fb, m_chrome)
            Vl, Fl = _lathe_vf([(0.0, 0.014), (rad * 0.55, 0.012), (rad * 0.66, 0.007), (rad * 0.66, 0.0),
                                (0.0, 0.0)], 24, axis=A, origin=org)
            acc_lens.add(Vl, Fl, m_lens)
            Vr, Fr = _lathe_vf([(rad * 0.66, 0.009), (rad * 0.86, 0.008), (rad * 0.86, -0.004),
                                (rad * 0.66, -0.004)], 24, axis=A, origin=org)
            acc_body.add(Vr, Fr, m_inner)
        # small chrome reflector bowls in the outer (wrapped) part of the lamp
        for frac in (0.66, 0.80):
            u = u0 + frac * span if outward[0] > 0 else u1 - frac * span
            a0 = surf_depth(u, vv) - 0.022
            org = c + U * u + V * vv + A * a0
            r2 = 0.75 * rad
            Vb, Fb = _lathe_vf([(0.0, -0.022), (r2 * 0.6, -0.019), (r2, -0.008), (r2 * 1.1, 0.0)], 20,
                               axis=A, origin=org)
            acc_body.add(Vb, Fb, m_chrome)
        # LED daytime-running light guide along the upper edge of the lens
        sel = pv > (v0 + 0.62 * (v1 - v0))
        idx = np.nonzero(sel)[0]
        if len(idx) > 4:
            # longest run of consecutive indices (loop order)
            runs, cur = [], [idx[0]]
            for a, b in zip(idx[:-1], idx[1:]):
                if b == a + 1:
                    cur.append(b)
                else:
                    runs.append(cur)
                    cur = [b]
            runs.append(cur)
            if len(runs) > 1 and runs[0][0] == 0 and runs[-1][-1] == len(P) - 1:
                runs[0] = runs[-1] + runs[0]
                runs.pop()
            run = max(runs, key=len)
            Q = P[run] - B[run] * 0.010 - A * 0.022
            if len(Q) > 3:
                Vt, Ft = _tube_vf(Q, 0.0042, 10)
                acc_body.add(Vt, Ft, m_led)
        # dark inner bezel just inside the lens edge (frames the lamp, like real units)
        Vz, Fz = _sweep(P - A * 0.012, N, [(-0.007, -0.001), (-0.002, 0.0), (-0.002, -0.004), (-0.007, -0.005)],
                        B=B)
        acc_body.add(Vz, Fz, m_inner)
    else:
        # rear: chrome reflector strip + LED light guide following the outline
        Q = P - B * 0.012 - A * 0.018
        Vt, Ft = _tube_vf(np.vstack([Q, Q[:1]]), 0.004, 8)
        acc_body.add(Vt, Ft, m_led)
        # horizontal chrome reflector bar across the lower third of the lamp
        sel = (pv > v0 + 0.25 * (v1 - v0)) & (pv < v0 + 0.45 * (v1 - v0))
        if sel.sum() > 2:
            us = np.linspace(u0 + 0.12 * (u1 - u0), u1 - 0.12 * (u1 - u0), 16)
            vv = v0 + 0.36 * (v1 - v0)
            Q = np.array([c + U * u + V * vv + A * (surf_depth(u, vv) - 0.028) for u in us])
            Vt, Ft = _tube_vf(Q, 0.006, 10)
            acc_body.add(Vt, Ft, m_chrome)


def _exhaust_tips(acc, mat):
    """Twin round tail-pipe tips under the right of the rear valance (the
    exhaust manifold is on the right, +X)."""
    for x in (0.425, 0.515):
        prof = [(0.030, -3.30), (0.034, -3.47), (0.0365, -3.52), (0.0365, -3.548), (0.0335, -3.548),
                (0.0315, -3.53), (0.0315, -3.36)]
        V, F = _lathe_vf([(r, -y) for (r, y) in prof], 28, axis=(0, -1, 0), origin=(x, 0.0, 0.335))
        acc.add(V, F, mat)


def _antenna(acc, bvh, mat):
    """Shark-fin roof antenna near the rear of the roof."""
    h = bvh.ray_cast(Vector((0.0, -2.50, 3.0)), Vector((0, 0, -1)))
    if h[0] is None:
        return
    p = np.array(h[0])

    def taper(x, y, z):
        # fin: tall at the rear, swept to a point at the front, narrow at the top
        u = y + 0.5                      # 0 front .. 1 rear
        return x * (1.0 - 0.6 * (z + 0.5)), y, (z + 0.5) * (0.25 + 0.75 * u) - 0.5

    V, F = _subd_box((0.024, 0.075, 0.032), 2, (0, 0, 0), taper)
    V = V + p + np.array([0.0, 0.0, 0.030])
    acc.add(V, F, mat)


def _corner_vents(acc_trim, acc_lamp, bvh, vent_me, trim_mats, lamp_mats):
    """Black corner vents (duct + horizontal bar) with a round fog lamp set
    back inside each one."""
    m_black, m_gloss = trim_mats
    m_lens, m_house, m_chrome = lamp_mats
    for piece in _islands_of(vent_me):
        c = _verts(piece).mean(axis=0)
        sgn = 1.0 if c[0] > 0 else -1.0
        ax = np.array([sgn * 0.40, 1.0, 0.0])
        ax /= np.linalg.norm(ax)
        for P, N in _boundary_loops(piece):
            P, N = _resample_loop(P, N, 0.007)
            _, _, B = _loop_frames(P, N)
            P0 = P - B * 0.0025
            rings = [P0, P0 - ax * 0.05]
            m = len(P0)
            V = np.vstack(rings)
            F = [(i, (i + 1) % m, m + (i + 1) % m, m + i) for i in range(m)]
            uvb = np.stack([(rings[-1] - c) @ np.cross([0, 0, 1.0], ax), (rings[-1] - c)[:, 2]], axis=1)
            tris = mgeo.tessellate_polygon([[Vector((q[0], q[1], 0.0)) for q in uvb]])
            F += [(m + t[0], m + t[1], m + t[2]) for t in tris]
            acc_trim.add(V, F, m_black)
            # black surround lip
            Vs, Fs = _sweep(P, _smooth_loop(N, 3), _rounded_rect_profile(-0.008, 0.002, -0.012, 0.002, 0.002, 2),
                            B=B)
            acc_trim.add(Vs, Fs, m_black)
        # fog lamp: chrome bowl + clear lens + black housing, recessed 18 mm in the vent
        hit = bvh.ray_cast(Vector(c + ax * 0.5), Vector(-ax))
        base = (np.array(hit[0]) if hit[0] is not None else c) - ax * 0.018 + np.array([0, 0, -0.004])
        r = 0.027
        Vh, Fh = _lathe_vf([(0.0, -0.030), (r * 1.30, -0.030), (r * 1.30, 0.004), (r * 1.05, 0.006)], 24,
                           axis=ax, origin=base)
        acc_lamp.add(Vh, Fh, m_house)
        Vb, Fb = _lathe_vf([(0.0, -0.026), (r * 0.5, -0.024), (r * 0.85, -0.012), (r, 0.003)], 24, axis=ax,
                           origin=base)
        acc_lamp.add(Vb, Fb, m_chrome)
        Vl, Fl = _lathe_vf([(0.0, 0.008), (r * 0.7, 0.007), (r * 1.02, 0.004), (r * 1.02, 0.0), (0.0, 0.0)], 24,
                           axis=ax, origin=base)
        acc_lamp.add(Vl, Fl, m_lens)
        bpy.data.meshes.remove(piece)
    bpy.data.meshes.remove(vent_me)


def _surface_point(bvh, view, p2):
    P, N = _project(bvh, [p2], view, smooth=0)
    if len(P) == 0:
        return None, None
    return P[0], N[0]


def _mirror(acc, bvh, sgn, mats):
    """Door mirror on the sail: painted housing (rounded front, flat back
    carrying the glass), black base/stalk.  Typical size 0.20 x 0.12 m."""
    m_paint, m_glass, m_base = mats
    p, n = _surface_point(bvh, "x+" if sgn > 0 else "x-", (-0.760, 1.002))
    if p is None:
        return
    hx, hy, hz = 0.126, 0.047, 0.064
    cx = p[0] + sgn * (0.032 + hx)
    cy, cz = -0.790, 1.048

    def taper(x, y, z):
        xo = x * sgn                          # -0.5 inner end .. +0.5 outer end
        y2 = y - 0.10 * max(0.0, y) ** 1.0 * (1 + xo)      # nose swept back toward the outer end
        z2 = z * (1.0 - 0.22 * max(0.0, y)) - 0.06 * max(0.0, xo) * (z < 0)
        return x, y2 - 0.04 * (xo + 0.5), z2

    V, F = _subd_box((hx, hy, hz), 2, (0, 0, 0), taper)
    yaw = math.radians(-5.0 * sgn)
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    V = V @ Rz.T + np.array([cx, cy, cz])
    acc.add(V, F, m_paint)
    back = float(np.min(V[:, 1]))
    gV, gF = _box_mesh((0.170, 0.004, 0.096), 0.018, 3, (0, 0, 0.002))
    gV = gV @ Rz.T + np.array([cx, back + 0.0015 + 0.004 * 0, cz])
    acc.add(gV, gF, m_glass)
    # black base on the sail + stalk into the housing
    x0 = p[0] - sgn * 0.006
    x1 = cx - sgn * (hx * 0.6)
    bV, bF = _subd_box((abs(x1 - x0) / 2 + 0.01, 0.040, 0.020), 2, ((x0 + x1) / 2, cy + 0.005, cz - 0.028))
    acc.add(bV, bF, m_base)


def _handle(acc, bvh, sgn, y, z, mats):
    m_paint, m_gasket = mats
    p, n = _surface_point(bvh, "x+" if sgn > 0 else "x-", (y, z))
    if p is None:
        return
    t = np.array([0.0, 1.0, 0.0])
    t = t - n * np.dot(t, n)
    M = _frame_matrix(p, t, np.cross(n, t))     # x along the door, y up-ish, z = surface normal
    gV, gF = _box_mesh((0.150, 0.034, 0.006), 0.012, 3, (0, 0, 0.001))
    acc.add(_xform(gV, M), gF, m_gasket)
    hV, hF = _subd_box((0.064, 0.0125, 0.010), 2, (0, 0, 0.012),
                       lambda a, b, c: (a, b, c * (1.0 - 0.3 * abs(a))))
    acc.add(_xform(hV, M), hF, m_paint)


def _wipers(acc, bvh, mat):
    (wO, wU, wV, wN), _ = _screen_frames()
    for x0, length in ((-0.33, 0.56), (0.17, 0.50)):
        pts = []
        for k in range(12):
            u = x0 + length * k / 11
            v = 0.045 + 0.02 * (k / 11)
            q = wO + wU * u + wV * v
            hit = bvh.ray_cast(Vector(q + wN * 0.5), Vector(-wN))
            if hit[0] is not None:
                pts.append(np.array(hit[0]) + wN * 0.010)
        if len(pts) > 3:
            P = np.array(pts)
            V, F = _tube_vf(P, 0.0055, 8)
            acc.add(V, F, mat)
            # arm from the pivot (on the cowl) to the blade
            piv = wO + wU * (x0 + 0.03) + wV * (-0.045)
            hit = bvh.ray_cast(Vector(piv + np.array([0, 0, 0.5])), Vector((0, 0, -1)))
            if hit[0] is not None:
                p0 = np.array(hit[0]) + np.array([0, 0, 0.012])
                V2, F2 = _tube_vf(np.array([p0, P[len(P) // 2] + wN * 0.006]), 0.005, 8)
                acc.add(V2, F2, mat)
                V3, F3 = _lathe_vf([(0.0, 0.0), (0.016, 0.0), (0.016, 0.014), (0.0, 0.016)], 16,
                                   origin=p0 - np.array([0, 0, 0.010]))
                acc.add(V3, F3, mat)


def _duct(acc, opening_me, depth, mat):
    """Dark duct behind a front opening: walls straight back (-Y) from just
    inside the opening edge, closed by a back panel, so the opening reads as
    deep and black at any distance."""
    for P, N in _boundary_loops(opening_me):
        P, N = _resample_loop(P, N, 0.008)
        _, _, B = _loop_frames(P, N)
        P0 = P - B * 0.0025
        rings = [P0, P0 - np.array([0.0, depth * 0.5, 0.0]), P0 - np.array([0.0, depth, 0.0])]
        m = len(P0)
        V = np.vstack(rings)
        F = [(r * m + i, r * m + (i + 1) % m, (r + 1) * m + (i + 1) % m, (r + 1) * m + i)
             for r in range(len(rings) - 1) for i in range(m)]
        back = rings[-1]
        tris = mgeo.tessellate_polygon([[Vector((p[0], p[2], 0.0)) for p in back]])
        base = (len(rings) - 1) * m
        F += [(base + t[0], base + t[1], base + t[2]) for t in tris]
        acc.add(V, F, mat)


def _slats(acc, bvh, poly, zs, setback, mat, thick=0.009, deep=0.024):
    """Horizontal bars across a symmetric trapezoid opening (front view poly),
    following the curved nose in plan, `setback` behind the outer surface."""
    (xb, zb), (xt, zt) = poly[1], poly[2]
    for zk in zs:
        hw = float(np.interp(zk, [zb, zt], [xb, xt])) - 0.010
        pts = []
        for x in np.linspace(-hw, hw, 28):
            hit = bvh.ray_cast(Vector((x, 3.0, zk)), Vector((0, -1, 0)))
            if hit[0] is not None:
                pts.append(np.array(hit[0]) - np.array([0.0, setback, 0.0]))
        if len(pts) < 4:
            continue
        P = np.array(pts)
        n = len(P)
        N = np.tile([0.0, 0.0, 1.0], (n, 1))
        B = np.tile([0.0, -1.0, 0.0], (n, 1))
        prof = _rounded_rect_profile(-deep / 2, deep / 2, -thick / 2, thick / 2, 0.0025, 2)
        V, F = _sweep(P, N, prof, closed=False, B=B)
        acc.add(V, F, mat)


def _grille(acc, bvh, grille_me, intake_me, mats):
    """Upper trapezoid grille (thin chrome surround, gloss-black slats, deep
    black duct) and the wide lower intake (black frame, slats, duct)."""
    m_black, m_chrome, m_grey, m_gloss = mats
    for P, N in _boundary_loops(grille_me):
        P, N = _resample_loop(P, N, 0.006)
        N = _smooth_loop(N, 3)
        N /= np.linalg.norm(N, axis=1)[:, None]
        V, F = _sweep(P, N, _rounded_rect_profile(-0.009, 0.0035, -0.006, 0.0045, 0.0025, 2))
        acc.add(V, F, m_chrome)
    _duct(acc, grille_me, 0.110, m_black)
    _slats(acc, bvh, GRILLE_POLY, np.linspace(0.548, 0.676, 5), 0.020, m_gloss)
    # lower intake
    _duct(acc, intake_me, 0.070, m_black)
    _slats(acc, bvh, INTAKE_POLY, (0.377, 0.416), 0.022, m_gloss, thick=0.008, deep=0.020)
    for P, N in _boundary_loops(intake_me):
        P, N = _resample_loop(P, N, 0.008)
        N = _smooth_loop(N, 3)
        N /= np.linalg.norm(N, axis=1)[:, None]
        V, F = _sweep(P, N, _rounded_rect_profile(-0.010, 0.002, -0.020, 0.002, 0.002, 2))
        acc.add(V, F, m_black)


# ---------------------------------------------------------------------------
# Underbody: floor pan + transmission tunnel, firewall/toe board, rear seat
# pan and kick-up over the differential, boot floor, wheelhouse liners.
# Typical values (not in spec): sill inner face x = +-0.765, floor top
# z = spec.Z_FLOOR, tunnel 0.48 m wide x 0.60 m high at the bellhousing,
# 0.26 m x 0.45 m at the propshaft; rear seat pan z 0.30; boot floor z 0.56.
# ---------------------------------------------------------------------------
X_SILL_IN = 0.765
SEAT_PAN = (-2.17, -2.405, 0.305)          # rear seat pan: y front, y rear, z
BOOT_FLOOR_Y0, BOOT_FLOOR_Z = -2.56, 0.565
TUNNEL_HOLE = (0.075, -0.885, -1.070)      # lever hole in the tunnel top: half-width, y front, y rear


def _tunnel_params(y):
    """(half-width at the floor, top height) of the transmission tunnel."""
    ys = [-0.40, -0.47, -0.62, -1.15, -1.40, -1.90, -2.30, -2.46]
    w = [0.255, 0.255, 0.225, 0.205, 0.165, 0.140, 0.135, 0.135]
    h = [0.615, 0.615, 0.600, 0.585, 0.525, 0.475, 0.455, 0.450]
    return float(np.interp(-y, [-v for v in ys], w)), float(np.interp(-y, [-v for v in ys], h))


def _floor_section(y, zf, wf, wt, ht, n_flat=4, r=0.035):
    """Cross-section (x, z) across the floor with a tunnel (left to right)."""
    pts = []
    # left flat part
    for k in range(n_flat):
        x = -wf + (wf - wt - r) * k / n_flat
        pts.append((x, zf))
    ht = max(ht, zf + 0.05)
    top_w = max(0.04, wt - 0.06)
    # tunnel: base fillet, wall, top corner, top
    left = [(-wt - r, zf), (-wt + 0.006, zf + 0.020), (-wt + 0.020, zf + 0.06),
            (-top_w - 0.02, ht - 0.035), (-top_w + 0.010, ht - 0.006), (-top_w * 0.45, ht)]
    pts += left[1:]
    pts += [(0.0, ht + 0.004)]
    right = [(-x, z) for (x, z) in reversed(left[1:])]
    pts += right
    for k in range(n_flat):
        x = wt + r + (wf - wt - r) * (k + 1) / n_flat
        pts.append((x, zf))
    return pts


def _underbody(col, detail):
    acc = _Acc()
    # ---- cabin floor + tunnel (lofted along y)
    zf = S.Z_FLOOR
    ys = list(np.linspace(-0.585, -2.10, 26))
    secs = []
    for y in ys:
        wt, ht = _tunnel_params(y)
        secs.append([(x, y, z) for (x, z) in _floor_section(y, zf, X_SILL_IN, wt, ht)])
    # heel kick + rear seat pan (raised floor, narrower between the wheelhouses)
    for y, z, wf in ((-2.135, 0.255, 0.765), (-2.17, 0.300, 0.74), (-2.255, 0.305, 0.66),
                     (-2.285, 0.305, 0.585), (SEAT_PAN[1], SEAT_PAN[2], 0.585)):
        wt, ht = _tunnel_params(y)
        secs.append([(x, y, zz) for (x, zz) in _floor_section(y, z, wf, wt, ht)])
    K = len(secs[0])
    V = np.array([p for sec in secs for p in sec])
    F = []
    for i in range(len(secs) - 1):
        for k in range(K - 1):
            F.append((i * K + k, i * K + k + 1, (i + 1) * K + k + 1, (i + 1) * K + k))
    me = _mesh_from_vf("_floor", V, F)
    # lever hole in the tunnel top (the gear lever passes through it)
    hw, yf, yr = TUNNEL_HOLE
    # open surface: bisect along the hole outline, then delete the faces inside it
    bm = bmesh.new()
    bm.from_mesh(me)
    for co, no in (((hw, 0, 0), (1, 0, 0)), ((-hw, 0, 0), (1, 0, 0)), ((0, yf, 0), (0, 1, 0)),
                   ((0, yr, 0), (0, 1, 0))):
        bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], plane_co=Vector(co),
                               plane_no=Vector(no))
    kill = [f for f in bm.faces if abs(f.calc_center_median().x) < hw and yr < f.calc_center_median().y < yf
            and f.calc_center_median().z > 0.45]
    bmesh.ops.delete(bm, geom=kill, context="FACES")
    bm.to_mesh(me)
    bm.free()
    acc.add_mesh(me, 0)
    bpy.data.meshes.remove(me)
    # boot floor over the axle: starts behind the rear backrest (no kick-up wall
    # in front of the differential, which sits in the open space under it)
    yk1, zb = BOOT_FLOOR_Y0, BOOT_FLOOR_Z
    bw = 0.555
    Vb = [(-bw, yk1, zb), (bw, yk1, zb), (bw, -3.36, zb), (-bw, -3.36, zb)]
    acc.add(Vb, [(0, 1, 2, 3)], 0)
    # rear panel (closes the boot floor at the back)
    acc.add([(-0.70, -3.36, zb), (0.70, -3.36, zb), (0.70, -3.36, 0.80), (-0.70, -3.36, 0.80)], [(0, 1, 2, 3)], 0)
    # ---- firewall: vertical bulkhead + toe board + plenum under the cowl, with a
    # tunnel-shaped opening, steering-column and clutch-pushrod holes
    yfw = S.Y_FIREWALL
    wt, ht = _tunnel_params(yfw)
    outline = [(-0.72, 0.32), (-wt, 0.32), (-wt, ht - 0.05), (-wt + 0.05, ht), (wt - 0.05, ht), (wt, ht - 0.05),
               (wt, 0.32), (0.72, 0.32), (0.72, 0.86), (-0.72, 0.86)]
    outline = _round_poly(outline, [0.0, 0.02, 0.05, 0.03, 0.03, 0.05, 0.02, 0.0, 0.06, 0.06], 4)
    holes = []
    for (hx, hz, hr) in ((S.MASTER_CYL_POS[0], S.MASTER_CYL_POS[2], 0.032),):
        a = np.linspace(0, 2 * math.pi, 20, endpoint=False)
        holes.append(np.stack([hx + hr * np.cos(a), hz + hr * np.sin(a)], axis=1))
    loops = [outline] + holes
    vecs = [[Vector((p[0], p[1], 0.0)) for p in lp] for lp in loops]
    tris = mgeo.tessellate_polygon(vecs)
    flat = np.vstack(loops)
    Vf = [(p[0], yfw, p[1]) for p in flat]
    acc.add(Vf, [tuple(t) for t in tris], 0)
    # toe board: from the firewall foot down/back to the floor
    acc.add([(-0.72, yfw, 0.32), (-wt, yfw, 0.32), (-wt - 0.035, -0.585, zf), (-X_SILL_IN, -0.585, zf)],
            [(0, 1, 2, 3)], 0)
    acc.add([(wt, yfw, 0.32), (0.72, yfw, 0.32), (X_SILL_IN, -0.585, zf), (wt + 0.035, -0.585, zf)],
            [(0, 1, 2, 3)], 0)
    # tunnel roof between the firewall and the floor start
    secF = _floor_section(yfw, zf, X_SILL_IN, wt, ht)
    i0 = [i for i, (x, z) in enumerate(secF) if abs(x) <= wt + 1e-6]
    a_, b_ = i0[0], i0[-1]
    Vt = [(x, yfw, z) for (x, z) in secF[a_:b_ + 1]] + [(x, -0.585, z) for (x, z) in secF[a_:b_ + 1]]
    m = b_ - a_ + 1
    acc.add(Vt, [(k, k + 1, m + k + 1, m + k) for k in range(m - 1)], 0)
    # plenum / scuttle under the cowl panel
    acc.add([(-0.72, yfw, 0.86), (0.72, yfw, 0.86), (0.70, -0.64, 0.92), (-0.70, -0.64, 0.92)], [(0, 1, 2, 3)], 0)
    # ---- wheelhouse liners: partial cylinders about each wheel axis with a
    # rolled outer edge that meets the shell's arch lip
    for y_ax, x_in in ((S.Y_FRONT_AXLE, 0.660), (S.Y_REAR_AXLE, 0.585)):
        prof = [(x_in, 0.400), (0.80, 0.400), (0.845, 0.392), (0.862, 0.372), (0.866, 0.356)]
        ang = np.radians(np.linspace(-14, 194, 40))
        for sgn in (1, -1):
            Vw, Fw = [], []
            for (x, r) in prof:
                for a in ang:
                    Vw.append((sgn * x, y_ax + r * math.cos(a), S.WHEEL_CENTER_Z + r * math.sin(a)))
            na = len(ang)
            for i in range(len(prof) - 1):
                for k in range(na - 1):
                    Fw.append((i * na + k, i * na + k + 1, (i + 1) * na + k + 1, (i + 1) * na + k))
            acc.add(Vw, Fw, 0)
    ob = acc.to_object("body_underbody", ["paint_black"], col, smooth_angle=45.0)
    return ob


# steering column geometry (typical: 24 deg below horizontal; it ends at the
# dash support bracket above/behind the pedal-box pivot)
COLUMN_ANGLE = math.radians(24.0)
_cd = np.array([0.0, math.cos(COLUMN_ANGLE), -math.sin(COLUMN_ANGLE)])    # wheel -> forward/down
STEERING_FIREWALL = tuple(np.array(S.STEERING_WHEEL_CENTRE)
                          + _cd * ((S.Y_FIREWALL - S.STEERING_WHEEL_CENTRE[1]) / _cd[1]))


# ---------------------------------------------------------------------------
# Interior.  Typical values (not in spec): front seat cushion 0.50 x 0.50 m,
# backrest 0.62 m reclined 22 deg, rear bench H-point (+-0.37, -2.25, 0.46),
# backrest 26 deg; dashboard top z 0.955; steering wheel 370 mm, column 24 deg;
# console top z 0.64-0.66 with the lever opening around spec.SHIFT_KNOB_REST.
# ---------------------------------------------------------------------------
CONSOLE_HOLE = (0.060, -0.905, -1.055)     # console lever opening: half-width, y front, y rear
REAR_H_POINT = (-2.22, 0.50)                # rear bench H-point (y, z) (typical, not in spec)


def _rot_x(deg):
    a = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def _placed(VF, R=None, t=(0, 0, 0)):
    V, F = VF
    V = np.asarray(V, float)
    if R is not None:
        V = V @ np.asarray(R).T
    return V + np.asarray(t), F


def _seat(acc, x, yH, zH, mats, recline=22.0, width=0.50, rear=False, base_dy=0.17, thick=0.065,
          depth=0.25, base_dz=0.12):
    m_fab, m_base = mats
    # cushion (top ~0.09 below the H-point), tilted front-up
    cz = zH - 0.15
    yc = yH + 0.09 - (0.25 - depth)
    if not rear:
        acc.add(*_placed(_subd_box((width / 2, depth, 0.060), 2), _rot_x(10), (x, yc, cz)), m_fab)
    else:
        # outer cushions on the seat pan + a raised centre section over the tunnel hump
        for s_ in (-1, 1):
            acc.add(*_placed(_subd_box((0.215, depth, 0.060), 2), _rot_x(10), (x + s_ * 0.375, yc, cz)), m_fab)
        wt, ht = _tunnel_params(yc)
        acc.add(*_placed(_subd_box((0.165, depth * 0.96, 0.035), 2), _rot_x(10), (x, yc, ht + 0.042)), m_fab)
    if not rear:
        for s in (-1, 1):   # side bolsters
            acc.add(*_placed(_subd_box((0.045, 0.22, 0.055), 2), _rot_x(10),
                             (x + s * (width / 2 - 0.035), yH + 0.08, cz + 0.045)), m_fab)
        acc.add(*_placed(_box_mesh((width - 0.10, 0.42, 0.07), 0.015, 2), None, (x, yH + 0.06, cz - 0.085)), m_base)
        for s in (-1, 1):   # seat rails
            acc.add(*_placed(_box_mesh((0.03, 0.50, 0.02), 0.004, 1), None,
                             (x + s * 0.17, yH + 0.05, S.Z_FLOOR + 0.012)), m_base)
    # backrest: lower edge just behind the H-point, reclined
    R = _rot_x(recline)
    up = R @ np.array([0, 0, 1.0])
    back = R @ np.array([0, -1.0, 0])
    L = 0.60 if not rear else 0.52
    base = np.array([x, yH - base_dy, zH - base_dz])
    ctr = base + up * (L / 2) + back * thick
    acc.add(*_placed(_subd_box((width / 2 - 0.01, thick, L / 2), 2,
                               taper=lambda a, b, c: (a * (1 - 0.10 * max(c, 0)), b, c)), R, ctr), m_fab)
    if not rear:
        for s in (-1, 1):
            acc.add(*_placed(_subd_box((0.04, 0.07, L / 2 - 0.06), 2), R,
                             ctr + np.array([s * (width / 2 - 0.04), 0, 0]) - back * 0.04 - up * 0.04), m_fab)
    # headrest on two posts
    top = base + up * L + back * 0.05
    hr = top + up * 0.13
    acc.add(*_placed(_subd_box((0.13 if not rear else 0.11, 0.05, 0.085), 2), R, hr), m_fab)
    for s in (-1, 1):
        acc.add(*_tube_vf(np.array([top - up * 0.02 + np.array([s * 0.07, 0, 0]),
                                    hr - up * 0.04 + np.array([s * 0.07, 0, 0])]), 0.0055, 8), m_base)


def _dashboard(acc, mats):
    m_dash, m_screen, m_trim = mats
    # cross-section (y, z) of the dash, lofted across the car and smoothed
    sec = [(-0.660, 0.905), (-0.760, 0.948), (-0.900, 0.958), (-0.975, 0.945), (-1.000, 0.905),
           (-0.985, 0.820), (-0.940, 0.720), (-0.880, 0.640), (-0.800, 0.620), (-0.700, 0.660)]
    xs = np.linspace(-0.735, 0.735, 13)
    rings = []
    for x in xs:
        bulge = 0.025 * math.exp(-((x + 0.37) / 0.18) ** 2)       # deeper in front of the driver
        ring = []
        for (y, z) in sec:
            yy = y - bulge * (1.0 if y < -0.85 else 0.0)
            # taper the ends toward the doors
            e = max(0.0, abs(x) - 0.62) / 0.115
            ring.append((x, yy + e * 0.05 * (1 if y < -0.8 else 0), z - e * 0.02))
        rings.append(ring)
    K = len(sec)
    V = np.array([p for r in rings for p in r])
    F = []
    for i in range(len(rings) - 1):
        for k in range(K):
            k1 = (k + 1) % K
            F.append((i * K + k, i * K + k1, (i + 1) * K + k1, (i + 1) * K + k))
    F.append(tuple(range(K))[::-1])
    F.append(tuple((len(rings) - 1) * K + k for k in range(K)))
    me = _mesh_from_vf("_dash", V, F)
    _recalc_outward(me)
    sub = _subdivide(me, 2)
    acc.add_mesh(sub, m_dash)
    bpy.data.meshes.remove(me)
    bpy.data.meshes.remove(sub)
    # instrument binnacle hood (driver side)
    acc.add(*_placed(_subd_box((0.17, 0.085, 0.045), 2, taper=lambda a, b, c: (a, b, c * (1 - 0.3 * max(-b, 0)))),
                     _rot_x(-8), (S.X_DRIVER, -0.905, 0.968)), m_dash)
    # instrument panel face (gloss black) under the hood
    acc.add(*_placed(_box_mesh((0.28, 0.006, 0.075), 0.01, 2), _rot_x(-15), (S.X_DRIVER, -0.965, 0.935)), m_screen)
    # central display standing on the dash top
    acc.add(*_placed(_box_mesh((0.27, 0.014, 0.115), 0.012, 3), _rot_x(-12), (0.0, -0.955, 1.01)), m_screen)
    # centre stack between dash face and console
    acc.add(*_placed(_subd_box((0.130, 0.06, 0.16), 2), _rot_x(18), (0.0, -0.975, 0.745)), m_dash)
    # air vents (dark rectangles with a trim surround)
    for vx in (-0.62, -0.12, 0.12, 0.62):
        acc.add(*_placed(_box_mesh((0.11 if abs(vx) > 0.3 else 0.07, 0.010, 0.040), 0.008, 2), _rot_x(-5),
                         (vx, -0.996, 0.885)), m_trim)


def _console(acc, mats):
    m_body, m_trim = mats
    hw, yf, yr = CONSOLE_HOLE
    outer = _round_poly([(-0.120, -1.92), (0.120, -1.92), (0.125, -0.95), (0.110, -0.86), (-0.110, -0.86),
                         (-0.125, -0.95)], [0.05, 0.05, 0.02, 0.03, 0.03, 0.02], 4)
    inner = _round_poly([(-hw, yr), (hw, yr), (hw, yf), (-hw, yf)], 0.02, 4)[::-1]

    def ztop(y):
        return float(np.interp(-y, [0.86, 0.95, 1.10, 1.45, 1.55, 1.92], [0.700, 0.660, 0.650, 0.640, 0.680, 0.690]))

    def zbot(x, y):
        wt, ht = _tunnel_params(y)
        return ht - 0.012 - 0.03 * (abs(x) / 0.125)

    V, F = [], []
    loops = []
    for poly in (outer, inner):
        n = len(poly)
        b0 = len(V)
        for (x, y) in poly:
            V.append((x, y, zbot(x, y)))
        t0 = len(V)
        for (x, y) in poly:
            V.append((x, y, ztop(y)))
        for i in range(n):
            j = (i + 1) % n
            F.append((b0 + i, b0 + j, t0 + j, t0 + i))
        loops.append((b0, t0, n))
    vecs = [[Vector((p[0], p[1], 0.0)) for p in poly] for poly in (outer, inner)]
    tris = mgeo.tessellate_polygon(vecs)
    no = len(outer)
    for t in tris:
        idx = []
        for k in t:
            idx.append(loops[0][1] + k if k < no else loops[1][1] + (k - no))
        F.append(tuple(idx))
    for t in tris:
        idx = []
        for k in t:
            idx.append(loops[0][0] + k if k < no else loops[1][0] + (k - no))
        F.append(tuple(idx[::-1]))
    me = _mesh_from_vf("_console", np.array(V), F)
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    bmesh.ops.bevel(bm, geom=[e for e in bm.edges if e.calc_face_angle(0) > 0.8], offset=0.008, segments=2,
                    affect="EDGES", clamp_overlap=True, profile=0.5)
    bm.to_mesh(me)
    bm.free()
    acc.add_mesh(me, m_body)
    bpy.data.meshes.remove(me)
    # shifter bezel ring around the opening
    P = np.array([(x, y, ztop(y) + 0.002) for (x, y) in inner[::-1]])
    Nn = np.tile([0, 0, 1.0], (len(P), 1))
    Vb, Fb = _sweep(P, Nn, _rounded_rect_profile(-0.004, 0.016, -0.004, 0.004, 0.002, 2))
    acc.add(Vb, Fb, m_trim)
    # armrest lid
    acc.add(*_placed(_subd_box((0.115, 0.17, 0.025), 2), None, (0.0, -1.74, ztop(-1.74) + 0.015)), m_body)


def _steering(acc, mats):
    m_rim, m_spoke, m_col = mats
    c = np.array(S.STEERING_WHEEL_CENTRE, float)
    ax = -_cd                                   # wheel axis, toward the driver
    up = np.array([0, 0, 1.0]) - ax * ax[2]
    up /= np.linalg.norm(up)
    side = np.cross(up, ax)
    R = np.stack([side, up, ax], axis=1)        # local x=side, y=up, z=axis
    # rim: torus, slightly oval cross-section
    Rr, rr = 0.185, 0.0155
    n_a, n_b = 72, 12
    V, F = [], []
    for i in range(n_a):
        a = 2 * math.pi * i / n_a
        ca, sa = math.cos(a), math.sin(a)
        for j in range(n_b):
            b = 2 * math.pi * j / n_b
            rad = Rr + rr * 1.15 * math.cos(b)
            V.append((rad * ca, rad * sa, rr * math.sin(b)))
    for i in range(n_a):
        for j in range(n_b):
            i1, j1 = (i + 1) % n_a, (j + 1) % n_b
            F.append((i * n_b + j, i1 * n_b + j, i1 * n_b + j1, i * n_b + j1))
    acc.add(*_placed((np.array(V), F), R, c), m_rim)
    # hub / airbag cover
    hub = _subd_box((0.075, 0.065, 0.030), 2, taper=lambda a, b, cc: (a, b, cc * (1 - 0.3 * abs(a))))
    acc.add(*_placed(hub, R, c - ax * 0.020), m_rim)
    # spokes at 3, 9 and 6 o'clock
    for ang in (0.0, math.pi, -math.pi / 2):
        d = np.array([math.cos(ang), math.sin(ang), 0.0])
        p0 = d * 0.055 + np.array([0, 0, -0.025])
        p1 = d * (Rr - 0.004) + np.array([0, 0, -0.004])
        Vs, Fs = _tube_vf(np.array([p0, (p0 + p1) / 2, p1]), 0.011, 10)
        Vs[:, 2] *= 1.0
        acc.add(*_placed((Vs, Fs), R, c), m_spoke)
    # column shroud and column (ends at the dash support bracket)
    p_wheel = c - ax * 0.05
    p_shroud = c - ax * 0.25
    acc.add(*_tube_vf(np.array([p_wheel, p_shroud]), 0.036, 16), m_col)
    p_end = c + _cd * ((-0.70 - c[1]) / _cd[1])
    acc.add(*_tube_vf(np.array([p_shroud, p_end]), 0.022, 12), m_col)
    acc.add(*_placed(_box_mesh((0.10, 0.05, 0.08), 0.01, 2), None, p_end + np.array([0, 0.0, 0.03])), m_col)


def _pedals(acc, mats):
    """Brake + throttle (the clutch pedal belongs to the clutch assembly);
    hanging pedals on the pedal-box axis, static (not driven by the Track)."""
    m_arm, m_pad = mats
    pv = np.array(S.CLUTCH_PEDAL_PIVOT, float)
    for x, arm, ang, pad_w, pad_h, z_off in ((S.BRAKE_PEDAL_X, S.PEDAL_ARM, S.PEDAL_REST_ANGLE, 0.085, 0.060, 0.0),
                                            (S.THROTTLE_PEDAL_X, 0.29, 0.42, 0.048, 0.110, -0.02)):
        piv = np.array([x, pv[1], pv[2] + z_off])
        d = np.array([0.0, -math.sin(ang), -math.cos(ang)])
        pad_c = piv + d * arm
        # pivot boss
        acc.add(*_lathe_vf([(0.0, -0.02), (0.016, -0.02), (0.016, 0.02), (0.0, 0.02)], 16, axis=(1, 0, 0),
                           origin=piv), m_arm)
        # arm (flat bar, curved slightly)
        mid = piv + d * (arm * 0.55) + np.array([0, -0.012, 0])
        Vt, Ft = _tube_vf(np.array([piv, mid, pad_c + d * 0.01]), 0.008, 8)
        acc.add(Vt, Ft, m_arm)
        # pad faces the driver: normal = (0, -cos, +sin) (perpendicular to the arm)
        nrm = np.array([0.0, -math.cos(ang), math.sin(ang)])
        M = _frame_matrix(pad_c + nrm * 0.008, (1, 0, 0), np.cross(nrm, (1, 0, 0)))
        Vp, Fp = _box_mesh((pad_w, pad_h, 0.014), 0.005, 2)
        acc.add(_xform(Vp, M), Fp, m_pad)


def _door_cards(acc, mats):
    m_card, m_fab = mats
    for sgn in (1, -1):
        for (y0, y1, bottom) in ((-0.72, -1.94, None), (-2.01, -2.70, "rear")):
            ys = np.linspace(y0, y1, 14)
            top, bot = [], []
            for y in ys:
                zt = _belt_z(y) - 0.012
                zb = 0.30
                if bottom == "rear":
                    dy = y - S.Y_REAR_AXLE
                    if abs(dy) < 0.43:
                        zb = max(zb, S.WHEEL_CENTER_Z + math.sqrt(max(0.43 ** 2 - dy ** 2, 0)) + 0.01)
                top.append((y, zt))
                bot.append((y, zb))
            poly = bot + top[::-1]
            n = len(poly)
            x_in, x_out = sgn * 0.770, sgn * 0.800
            V = [(x_in, y, z) for (y, z) in poly] + [(x_out, y, z) for (y, z) in poly]
            F = [tuple(range(n))[::-1], tuple(range(n, 2 * n))]
            for i in range(n):
                j = (i + 1) % n
                F.append((i, j, n + j, n + i))
            me = _mesh_from_vf("_card", np.array(V), F)
            _recalc_outward(me)
            acc.add_mesh(me, m_card)
            bpy.data.meshes.remove(me)
            # armrest + fabric insert
            ya, yb = (y0 - 0.25, y1 + 0.12) if bottom is None else (y0 - 0.06, -2.40)
            za = 0.66 if bottom is None else 0.745
            acc.add(*_placed(_subd_box((0.030, abs(yb - ya) / 2, 0.022), 2), None,
                             (sgn * 0.748, (ya + yb) / 2, za)), m_card)
            acc.add(*_placed(_box_mesh((0.006, abs(yb - ya), 0.16), 0.004, 1), None,
                             (sgn * 0.767, (ya + yb) / 2, 0.80)), m_fab)


def _headliner(acc, closed_me, mat):
    """Roof lining: the inside of the roof panel (shell surface offset in)."""
    me = closed_me.copy()
    c, n = _face_centres_normals(me)
    keep = (c[:, 2] > 1.26) & (np.abs(c[:, 0]) < 0.66) & (c[:, 1] < -1.40) & (c[:, 1] > -2.70) & (n[:, 2] > 0.5)
    _delete_faces(me, ~keep)
    _offset_mesh(me, -0.030)
    me.flip_normals()
    acc.add_mesh(me, mat)
    bpy.data.meshes.remove(me)


def _interior(col, closed_me, detail):
    seats = _Acc()
    mats = ["body_fabric", "plastic_black", "body_gloss_black", "body_grey_trim", "body_headliner", "rubber",
            "paint_black"]
    FAB, PB, GB, GT, HL, RB, PK = range(7)
    yH, zH = S.H_POINT[1], S.H_POINT[2]
    for x in (S.X_DRIVER, -S.X_DRIVER):
        _seat(seats, x, yH, zH, (FAB, PB))
    # rear bench (one cushion + one backrest, 3 headrests)
    yR, zR = REAR_H_POINT
    _seat(seats, 0.0, yR, zR, (FAB, PB), recline=26.0, width=1.18, rear=True, base_dy=0.11, thick=0.055,
          depth=0.215, base_dz=0.06)
    for x in (-0.38, 0.38):
        R = _rot_x(26.0)
        up = R @ np.array([0, 0, 1.0])
        top = np.array([x, yR - 0.11, zR - 0.06]) + up * 0.52 + R @ np.array([0, -0.05, 0])
        seats.add(*_placed(_subd_box((0.11, 0.045, 0.07), 2), R, top + up * 0.10), FAB)
    _dashboard(seats, (PB, GB, GT))
    _console(seats, (PB, GT))
    interior = seats.to_object("body_interior", mats, col, smooth_angle=50.0)
    # trim fixed to the body shell (fades with the exterior): door cards,
    # headliner, parcel shelf
    ct = _Acc()
    _door_cards(ct, (0, 1))
    _headliner(ct, closed_me, 2)
    ct.add(*_placed(_box_mesh((1.30, 0.56, 0.012), 0.004, 1), _rot_x(8), (0.0, -2.86, 0.955)), 0)
    cabin_trim = ct.to_object("body_cabin_trim", ["plastic_black", "body_fabric", "body_headliner"], col,
                              smooth_angle=50.0)
    st = _Acc()
    _steering(st, (0, 1, 0))
    steering = st.to_object("body_steering_wheel", ["plastic_black", "body_grey_trim"], col, smooth_angle=50.0)
    pd = _Acc()
    _pedals(pd, (0, 1))
    pedals = pd.to_object("body_pedals", ["paint_black", "rubber"], col, smooth_angle=40.0)
    return interior, steering, pedals, cabin_trim


# ---------------------------------------------------------------------------
# Optional x-ray feature lines: thin camera-only emissive tubes along the
# lines that define the car's shape (window openings, lamps, wheel arches,
# shoulder and sill lines, nose/tail outline).  Hidden (cv_opacity 0) until
# a scene fades them in (presentation 'xray_edges_opacity').
# ---------------------------------------------------------------------------

def _xray_lines(col, bvh, loops):
    acc = _Acc()
    r = 0.0016
    for P in loops:
        if len(P) > 3:
            V, F = _tube_vf(np.vstack([P, P[:1]]), r, 6)
            acc.add(V, F, 0)
    # wheel-arch lips
    for y in (S.Y_FRONT_AXLE, S.Y_REAR_AXLE):
        for sgn in (1, -1):
            pts = []
            for a in np.radians(np.linspace(-12, 192, 70)):
                hp = bvh.ray_cast(Vector((sgn * 2.0, y + (ARCH_R + 0.004) * math.cos(a),
                                          ARCH_ZC + (ARCH_R + 0.004) * math.sin(a))), Vector((-sgn, 0, 0)))
                if hp[0] is not None:
                    pts.append(np.array(hp[0]))
            if len(pts) > 3:
                acc.add(*_tube_vf(np.array(pts), r, 6), 0)
    # shoulder line, sill line (side views) and the nose/tail outline (polar)
    for sgn, view in ((1, "x+"), (-1, "x-")):
        for zf in (lambda y: float(np.interp(-y, [-0.62, 0.0, 0.64, 1.46, 2.40, 2.92, 3.30],
                                             [0.792, 0.856, 0.873, 0.882, 0.893, 0.898, 0.893])),
                   lambda y: 0.245):
            ys = [y for y in np.linspace(0.62, -3.40, 260)
                  if not (abs(y - S.Y_FRONT_AXLE) < ARCH_R + 0.02 or abs(y - S.Y_REAR_AXLE) < ARCH_R + 0.02)]
            run = []
            for y in ys:
                P, _ = _project(bvh, [(y, zf(y))], view, smooth=0)
                if len(P) and (not run or np.linalg.norm(P[0] - run[-1]) < 0.06):
                    run.append(P[0])
                elif len(P):
                    if len(run) > 3:
                        acc.add(*_tube_vf(np.array(run), r, 6), 0)
                    run = [P[0]]
            if len(run) > 3:
                acc.add(*_tube_vf(np.array(run), r, 6), 0)
    if acc.empty():
        return None
    ob = acc.to_object("body_xray_edges", ["body_xray_line"], col, smooth_angle=80.0)
    return ob


# ---------------------------------------------------------------------------
# Assembly
# ---------------------------------------------------------------------------
GROUPS = {
    "exterior": ("shell", "bumpers", "trim", "lights_front", "lights_rear", "mirrors", "handles", "glass",
                 "cabin_trim"),
    "interior": ("interior", "steering_wheel", "pedals"),
    "underbody": ("underbody",),
}


def _submesh(me, keep_mask, name):
    out = me.copy()
    out.name = name
    _delete_faces(out, ~np.asarray(keep_mask, bool))
    return out


def _tri_count(ob):
    me = ob.data
    me.calc_loop_triangles()
    return len(me.loop_triangles)


def build(opts=None):
    """Build the car body + interior.  See the module docstring."""
    from .. import rig
    opts = dict(opts or {})
    detail = opts.get("detail", "high")
    t0 = time.time()
    col = rig.collection(opts.get("collection", "body"))
    root = rig.empty("body_root", (0.0, 0.0, 0.0), col=col, size=0.4)
    log = {}
    M = build_shell_meshes(detail, log=log)
    closed = M.pop("closed")
    bvh = _bvh_of(closed)
    parts = {}

    def finish(ob, key, smooth=None):
        if smooth is not None:
            _smooth(ob, smooth)
        ob.parent = root
        rig.set_presentation(ob, 1.0, 0.0)
        parts[key] = ob
        return ob

    # ---- shell + bumpers (painted)
    sh = M.pop("shell")
    part = _get_face_attr(sh, "body_part")
    for key, mask in (("shell", part == 0), ("bumpers", (part == 1) | (part == 2))):
        me = _submesh(sh, mask, f"body_{key}")
        for a in ("cvb_orig", "body_part", "crease_edge"):
            if me.attributes.get(a) is not None:
                me.attributes.remove(me.attributes[a])
        me.materials.clear()
        me.materials.append(_get_mat("car_paint"))
        me.polygons.foreach_set("material_index", np.zeros(len(me.polygons), dtype=np.int32))
        ob = bpy.data.objects.new(f"body_{key}", me)
        col.objects.link(ob)
        finish(ob, key, 40.0)
    black_lower = _submesh(sh, part >= 3, "_black_lower")     # front lip + rear diffuser
    bpy.data.meshes.remove(sh)
    # ---- glass
    g = _Acc()
    panes, screens = [], []
    for k in ("glass_wind", "glass_side", "glass_rear"):
        me = M.pop(k)
        _offset_mesh(me, -0.004)
        g.add_mesh(me, 0)
        panes.append(me)
        if k != "glass_side":
            screens.append(me)
    finish(g.to_object("body_glass", ["glass"], col, smooth_angle=60.0), "glass")
    # ---- trim: window surrounds, B-pillar appliques, mirror sails, cowl, grille, intake, wipers
    tr = _Acc()
    TB, TG, TC, TR = 0, 1, 2, 3          # plastic_black, gloss black, chrome, grey
    xray_loops = [_resample_loop(P, N, 0.01)[0] for me in panes for (P, N) in _boundary_loops(me)]
    for k in ("lens_front", "lens_rear", "grille", "intake"):
        xray_loops += [_resample_loop(P, N, 0.01)[0] for (P, N) in _boundary_loops(M[k])]
    _window_frames(tr, panes, TB)
    _frit_bands(tr, screens, TG)
    for me in panes:
        bpy.data.meshes.remove(me)
    for k, dist, mat in (("bpillar", 0.0012, TG), ("sail", 0.0008, TG), ("cowl", -0.0015, TB)):
        me = M.pop(k)
        _offset_mesh(me, dist)
        tr.add_mesh(me, mat)
        bpy.data.meshes.remove(me)
    gr, it = M.pop("grille"), M.pop("intake")
    _grille(tr, bvh, gr, it, (TB, TC, TR, TG))
    bpy.data.meshes.remove(gr)
    bpy.data.meshes.remove(it)
    _wipers(tr, bvh, TB)
    vent_lamps = _Acc()
    _corner_vents(tr, vent_lamps, bvh, M.pop("vent"), (TB, TG), (0, 4, 2))
    tr.add_mesh(black_lower, TB)
    bpy.data.meshes.remove(black_lower)
    _exhaust_tips(tr, TC)
    _antenna(tr, bvh, TG)
    finish(tr.to_object("body_trim", ["plastic_black", "body_gloss_black", "chrome", "body_grey_trim"], col,
                        smooth_angle=45.0), "trim")
    # ---- lamps
    for key, kind, out_dir, mats in (
            ("lights_front", "front", (0.45, 1.0, 0.0), ["body_lens_clear", "body_lamp_silver", "chrome", "body_led",
                                                         "plastic_black"]),
            ("lights_rear", "rear", (0.55, -1.0, 0.0), ["body_lens_red", "plastic_black", "chrome", "body_lens_red",
                                                        "body_lamp_red"])):
        lens_all = M.pop(f"lens_{kind}")
        acc = _Acc()
        for piece in _islands_of(lens_all):
            sgn = 1.0 if _verts(piece)[:, 0].mean() > 0 else -1.0
            od = (sgn * out_dir[0], out_dir[1], out_dir[2])
            _lamp_unit(acc, acc, piece, od, kind, (0, 1, 2, 3, 4))
            bpy.data.meshes.remove(piece)
        bpy.data.meshes.remove(lens_all)
        if kind == "front" and not vent_lamps.empty():
            acc.add(np.vstack(vent_lamps.V), [tuple(int(i) for i in f) for f in vent_lamps.F], 0)
            # (material slots: re-map below)
            for k in range(len(vent_lamps.F)):
                acc.M[-len(vent_lamps.F) + k] = vent_lamps.M[k]
        finish(acc.to_object(f"body_{key}", mats, col, smooth_angle=50.0), key)
    # ---- mirrors + handles
    mi = _Acc()
    for sgn in (1, -1):
        _mirror(mi, bvh, sgn, (0, 1, 2))
    finish(mi.to_object("body_mirrors", ["car_paint", "chrome", "plastic_black"], col, smooth_angle=50.0), "mirrors")
    hd = _Acc()
    for sgn in (1, -1):
        for (y, z) in ((-1.70, 0.842), (-2.555, 0.852)):
            _handle(hd, bvh, sgn, y, z, (0, 1))
    finish(hd.to_object("body_handles", ["car_paint", "plastic_black"], col, smooth_angle=50.0), "handles")
    for k in list(M):
        bpy.data.meshes.remove(M.pop(k))
    # ---- optional x-ray feature lines (hidden until faded in)
    if opts.get("xray_edges", False):
        xo = _xray_lines(col, bvh, xray_loops)
        if xo is not None:
            finish(xo, "xray_edges")
            xo["cv_opacity"] = 0.0
            xo.hide_render = True
    # ---- underbody + interior
    finish(_underbody(col, detail), "underbody")
    interior, steering, pedals, cabin_trim = _interior(col, closed, detail)
    finish(cabin_trim, "cabin_trim")
    finish(interior, "interior")
    finish(steering, "steering_wheel")
    finish(pedals, "pedals")
    # ---- anchors (local offsets = car coordinates: every part sits at the root origin)
    def hit(o, d):
        h = bvh.ray_cast(Vector(o), Vector(d))
        return tuple(h[0]) if h[0] is not None else tuple(o)

    wO, wU, wV, wN = _screen_frames()[0]
    ws = np.array(wO) + wV * 0.45
    wt, ht = _tunnel_params(-1.50)
    anchors = {
        "hood": (parts["shell"], hit((0.0, 0.05, 3.0), (0, 0, -1))),
        "windscreen": (parts["glass"], tuple(np.array(hit(tuple(ws + wN * 0.5), tuple(-wN))) - wN * 0.004)),
        "roof": (parts["shell"], hit((0.0, -1.95, 3.0), (0, 0, -1))),
        "front_bumper": (parts["bumpers"], hit((0.0, 3.0, 0.485), (0, -1, 0))),
        "door_left": (parts["shell"], hit((-3.0, -1.30, 0.70), (1, 0, 0))),
        "cabin": (parts["interior"], (0.0, -1.60, 0.85)),
        "firewall": (parts["underbody"], (0.40, S.Y_FIREWALL, 0.72)),
        "tunnel": (parts["underbody"], (0.0, -1.50, ht + 0.004)),
    }
    bpy.data.meshes.remove(closed)
    groups = {g: [parts[k] for k in keys] for g, keys in GROUPS.items()}
    if "xray_edges" in parts:
        groups["xray"] = [parts["xray_edges"]]
    tris = {k: _tri_count(o) for k, o in parts.items()}
    explode = {k: (0.0, 0.0, 1.3) for k in GROUPS["exterior"]}
    explode.update({k: (0.0, 0.0, 0.65) for k in GROUPS["interior"]})
    meta = dict(
        groups=groups,
        group_names={g: list(keys) for g, keys in GROUPS.items()},
        arch=dict(radius=ARCH_R, centre_z=ARCH_ZC, y=(S.Y_FRONT_AXLE, S.Y_REAR_AXLE)),
        console_hole=CONSOLE_HOLE, tunnel_hole=TUNNEL_HOLE,
        steering_column_angle_deg=math.degrees(COLUMN_ANGLE),
        build_log=log, triangles=tris, triangles_total=int(sum(tris.values())),
        build_time=time.time() - t0, detail=detail,
    )
    asm = rig.Assembly(name="body", root=root, parts=parts, anchors=anchors, explode=explode, meta=meta,
                       _driver=_drive)
    return asm


def _drive(asm, track, presentation):
    """The body has no moving parts.  Optional per-frame presentation arrays
    fade whole groups: 'exterior_opacity', 'interior_opacity',
    'underbody_opacity', 'glass_opacity' (glass only, overrides exterior)."""
    from .. import rig
    frames = track.frames
    for g in ("exterior", "interior", "underbody"):
        arr = presentation.get(f"{g}_opacity")
        if arr is not None:
            objs = asm.meta["groups"][g]
            if g == "exterior" and presentation.get("glass_opacity") is not None:
                objs = [o for o in objs if o is not asm.parts["glass"]]
            rig.bake_fade(objs, frames, np.asarray(arr, float))
    if presentation.get("glass_opacity") is not None:
        rig.bake_fade([asm.parts["glass"]], frames, np.asarray(presentation["glass_opacity"], float))
    if presentation.get("xray_edges_opacity") is not None and "xray_edges" in asm.parts:
        rig.bake_fade([asm.parts["xray_edges"]], frames, np.asarray(presentation["xray_edges_opacity"], float))
