#!/usr/bin/env python3
"""Look-dev test scenes for carviz.materials + carviz.lighting.

    python3 tools/lookdev.py                       # all scenes, Cycles 640x360 16 spp + OIDN
    python3 tools/lookdev.py --only dark_mech,light_car --samples 16
    python3 tools/lookdev.py --workbench           # same scenes in Workbench (viewport colours)
    python3 tools/lookdev.py --verify              # numeric cv_opacity test (1 / 0 / 0.25 / missing)

Scenes
  dark_balls   every material on a shader ball (dark studio), names overlaid
  dark_close   close-up of dark_mech (gases, section paint, glow gear, brass, steel)
  dark_macro   macro of clutch facing, spring, synchro ring, shaft
  dark_mech    mechanical vignette: 4-cylinder cut block (section_cut faces,
               pistons, the four gas materials), gear pair (one with cv_glow=1),
               shaft, bearing, synchro ring, spring, clutch disc, case, spark
               plug, brake line, and a car-paint panel at cv_opacity=0.25
  light_car    car proxy (paint, glass, tyres, rims, chrome) on the light cyclorama
  light_xray   same car with the body at cv_opacity 0.2 over a proxy drivetrain
               (gearbox glowing), i.e. the s01 "inside" look
  road         car proxy on the road preset, camera alongside (lights follow)

Output: <scratch>/lookdev/<scene>.png (+ _wb.png for Workbench); render times
are printed and written to times.json.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
os.environ.setdefault("EGL_PLATFORM", "surfaceless")

import bpy  # noqa: E402  (bpy before bmesh)
import bmesh  # noqa: E402
from mathutils import Vector  # noqa: E402

from carviz import lighting, materials  # noqa: E402

DEFAULT_OUT = os.path.join(os.environ.get("CV_SCRATCH", "/tmp"), "lookdev")


# ---------------------------------------------------------------------------
# Scene helpers
# ---------------------------------------------------------------------------

def reset():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.render.fps = 24
    return sc


def link(ob, col=None):
    (col or bpy.context.scene.collection).objects.link(ob)
    return ob


def mesh_obj(name, bm, mat, loc=(0, 0, 0), rot=(0, 0, 0), smooth_angle=35.0, parent=None):
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    if isinstance(mat, (list, tuple)):
        for m in mat:
            me.materials.append(materials.get(m))
    else:
        me.materials.append(materials.get(mat))
    for p in me.polygons:
        p.use_smooth = True
    me.set_sharp_from_angle(angle=math.radians(smooth_angle))
    ob = bpy.data.objects.new(name, me)
    ob.location = loc
    ob.rotation_euler = rot
    link(ob)
    if parent is not None:
        ob.parent = parent
    materials.ensure_props(ob)
    return ob


def bevel(ob, width=0.0008, segments=2, angle=35.0):
    m = ob.modifiers.new("bevel", "BEVEL")
    m.width = width
    m.segments = segments
    m.limit_method = "ANGLE"
    m.angle_limit = math.radians(angle)
    m.harden_normals = False
    return ob


def lathe(name, prof, mat, segs=64, loc=(0, 0, 0), rot=(0, 0, 0), parent=None, smooth_angle=35.0):
    """Revolve (r, y) points about local +Y (closed profile = list order)."""
    bm = bmesh.new()
    rings = []
    for r, y in prof:
        if r < 1e-6:
            rings.append([bm.verts.new((0.0, y, 0.0))] * segs)
            continue
        rings.append([bm.verts.new((r * math.cos(2 * math.pi * k / segs), y,
                                    r * math.sin(2 * math.pi * k / segs))) for k in range(segs)])
    for i in range(len(rings) - 1):
        a, b = rings[i], rings[i + 1]
        for k in range(segs):
            k1 = (k + 1) % segs
            vs = [a[k], a[k1], b[k1], b[k]]
            uniq = []
            for v in vs:
                if v not in uniq:
                    uniq.append(v)
            if len(uniq) >= 3:
                try:
                    bm.faces.new(uniq)
                except ValueError:
                    pass
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-7)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return mesh_obj(name, bm, mat, loc, rot, smooth_angle, parent)


def ring_prof(r0, r1, w, y=0.0, ch=0.0):
    """Closed rectangular annulus profile (inner r0, outer r1, width w along Y)."""
    h = w / 2
    return [(r0, y - h + ch), (r0, y + h - ch), (r0 + ch, y + h), (r1 - ch, y + h), (r1, y + h - ch),
            (r1, y - h + ch), (r1 - ch, y - h), (r0 + ch, y - h), (r0, y - h + ch)]


def gear(name, teeth, r_tip, r_root, width, bore, mat, loc=(0, 0, 0), rot=(0, 0, 0), helix=0.0):
    """Simple spur/helical-looking gear, axis local +Y (tooth 0 on +X)."""
    n = teeth * 8
    pitch = 2 * math.pi / teeth

    def r_at(t):
        ph = ((t / pitch) % 1.0)
        # trapezoid tooth: root 0-.15, flank .15-.35, tip .35-.65, flank .65-.85, root
        if ph < 0.15 or ph >= 0.85:
            return r_root
        if ph < 0.35:
            return r_root + (r_tip - r_root) * (ph - 0.15) / 0.2
        if ph < 0.65:
            return r_tip
        return r_tip - (r_tip - r_root) * (ph - 0.65) / 0.2

    bm = bmesh.new()
    layers = []
    for y in (-width / 2, width / 2):
        twist = helix * y
        outer = [bm.verts.new((r_at(t + 0.5 * pitch) * math.cos(t + twist), y,
                               r_at(t + 0.5 * pitch) * math.sin(t + twist)))
                 for t in (2 * math.pi * k / n for k in range(n))]
        inner = [bm.verts.new((bore * math.cos(t), y, bore * math.sin(t)))
                 for t in (2 * math.pi * k / n for k in range(n))]
        layers.append((outer, inner))
    (o0, i0), (o1, i1) = layers
    for k in range(n):
        k1 = (k + 1) % n
        bm.faces.new((o0[k], o0[k1], i0[k1], i0[k]))
        bm.faces.new((o1[k], i1[k], i1[k1], o1[k1]))
        bm.faces.new((o0[k], o1[k], o1[k1], o0[k1]))
        bm.faces.new((i0[k], i0[k1], i1[k1], i1[k]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    ob = mesh_obj(name, bm, mat, loc, rot, smooth_angle=30.0)
    return bevel(ob, width=min(0.0008, width * 0.05))


def box(name, size, mat, loc=(0, 0, 0), rot=(0, 0, 0), bev=0.0, segs=3):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bmesh.ops.scale(bm, vec=Vector(size), verts=bm.verts)
    if bev > 0:
        bmesh.ops.bevel(bm, geom=list(bm.edges) + list(bm.verts), offset=bev, segments=segs,
                        affect="EDGES", profile=0.5)
    return mesh_obj(name, bm, mat, loc, rot)


def cylinder(name, r, length, mat, loc=(0, 0, 0), rot=(0, 0, 0), segs=48):
    """Cylinder along local Y."""
    return lathe(name, [(0, -length / 2), (r, -length / 2), (r, length / 2), (0, length / 2)], mat, segs,
                 loc, rot)


def sphere(name, r, mat, loc):
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=64, v_segments=32, radius=r)
    return mesh_obj(name, bm, mat, loc, smooth_angle=180.0)


def spring(name, r, wire, pitch, turns, mat, loc, rot=(0, 0, 0)):
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions = "3D"
    cu.bevel_depth = wire
    cu.bevel_resolution = 4
    sp = cu.splines.new("POLY")
    n = int(turns * 24)
    sp.points.add(n)
    for i in range(n + 1):
        t = i / 24 * 2 * math.pi
        sp.points[i].co = (r * math.cos(t), pitch * t / (2 * math.pi) - pitch * turns / 2, r * math.sin(t), 1)
    cu.materials.append(materials.get(mat))
    ob = bpy.data.objects.new(name, cu)
    ob.location = loc
    ob.rotation_euler = rot
    link(ob)
    materials.ensure_props(ob)
    return ob


def apply_mods(ob, smooth_angle=35.0):
    """Bake modifiers (booleans) into the mesh and re-smooth by angle."""
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    old = ob.data
    ob.modifiers.clear()
    ob.data = me
    bpy.data.meshes.remove(old)
    for p in me.polygons:
        p.use_smooth = True
    me.set_sharp_from_angle(angle=math.radians(smooth_angle))
    return ob


def boolean(ob, cutter, op="DIFFERENCE"):
    m = ob.modifiers.new("bool", "BOOLEAN")
    m.operation = op
    m.solver = "EXACT"
    m.material_mode = "TRANSFER"
    m.object = cutter
    cutter.hide_render = True
    cutter.hide_viewport = True
    return m


def camera(eye, target, lens=50.0, fstop=None, name="cam"):
    cd = bpy.data.cameras.new(name)
    cd.lens = lens
    cd.sensor_width = 36.0
    cd.clip_start = 0.02
    cd.clip_end = 2000.0
    ob = bpy.data.objects.new(name, cd)
    ob.location = eye
    ob.rotation_euler = (Vector(target) - Vector(eye)).to_track_quat("-Z", "Y").to_euler()
    link(ob)
    bpy.context.scene.camera = ob
    if fstop:
        lighting.camera_dof(ob, (Vector(target) - Vector(eye)).length, fstop=fstop)
    return ob


def orbit_eye(center, az, el, dist):
    a, e = math.radians(az), math.radians(el)
    c = Vector(center)
    return c + Vector((math.sin(a) * math.cos(e), -math.cos(a) * math.cos(e), math.sin(e))) * dist


# ---------------------------------------------------------------------------
# Scenes
# ---------------------------------------------------------------------------

BALL_MATS = [n for n in materials.all_names() if n not in ("floor", "backdrop")]


def scene_dark_balls():
    sc = reset()
    cols = 9
    rows = math.ceil(len(BALL_MATS) / cols)
    r = 0.075
    dx, dy = 0.235, 0.36
    balls = []
    for i, m in enumerate(BALL_MATS):
        row, colm = divmod(i, cols)
        x = (colm - (cols - 1) / 2) * dx
        y = (rows - 1 - row) * dy - 0.25
        z = r + row * 0.0
        balls.append((m, sphere(f"ball_{m}", r, m, (x, y, z))))
    center = (0, 0.0, 0.12)
    st = lighting.setup_studio("dark", center=center, size=1.0, key_azimuth=215.0)
    eye = orbit_eye((0, 0.0, 0.0), 180.0, 40.0, 3.2)
    camera(eye, (0, 0.11, 0.0), lens=50, fstop=11.0)
    return sc, dict(labels=[(m, ob.location + Vector((0, 0, -r * 1.05))) for m, ob in balls], studio=st)


def scene_dark_mech():
    sc = reset()
    # --- 4-cylinder cut block (cut plane x = 0, block occupies x < 0) ------------
    pitch, bore_r = 0.094, 0.043
    ys = [1.5 * pitch - i * pitch for i in range(4)]
    blk = box("block", (0.26, 0.42, 0.24), "cast_iron", loc=(0.0, 0.0, 0.12), bev=0.008)
    head = box("head", (0.24, 0.42, 0.10), "cast_aluminium", loc=(0.0, 0.0, 0.29), bev=0.006)
    # bores (honed: machined steel look) and the half-space cut (section_cut)
    bores = []
    for i, y in enumerate(ys):
        bores.append(cylinder(f"bore{i}", bore_r, 0.6, "cast_iron", loc=(0, y, 0.2),
                              rot=(math.pi / 2, 0, 0)))
    bm = bmesh.new()
    for b in bores:
        b_me = b.data.copy()
        b_me.transform(b.matrix_basis)
        bm.from_mesh(b_me)
        bpy.data.meshes.remove(b_me)
    bore_u = mesh_obj("bores_u", bm, "cast_iron")
    for b in bores:
        bpy.data.objects.remove(b, do_unlink=True)
    half = box("halfspace", (0.6, 0.8, 0.8), "section_cut", loc=(0.3, 0.0, 0.2))
    for ob in (blk,):
        boolean(ob, bore_u)
        boolean(ob, half)
        apply_mods(ob)
    boolean(head, half)
    apply_mods(head)
    bpy.data.objects.remove(bore_u, do_unlink=True)
    bpy.data.objects.remove(half, do_unlink=True)
    # pistons + gas volumes (crowns at different heights: intake/comp/power/exhaust)
    crown = [0.10, 0.215, 0.20, 0.13]
    gas = ["gas_intake", "gas_compressed", "gas_burning", "gas_exhaust"]
    deck = 0.24
    for i, y in enumerate(ys):
        pz = crown[i]
        lathe(f"piston{i}", [(0, -0.06), (bore_r - 0.0005, -0.06), (bore_r - 0.0005, 0.0), (0, 0.0)],
              "machined_aluminium", loc=(-0.0, y, pz), rot=(-math.pi / 2, 0, 0))
        h = deck + 0.03 - pz
        lathe(f"gas{i}", [(0, 0.0), (bore_r - 0.001, 0.0), (bore_r - 0.001, h), (0, h)], gas[i],
              loc=(-0.0, y, pz + 0.0005), rot=(-math.pi / 2, 0, 0))
    # spark plug on top of cylinder 3
    plug = [(0.0, 0.0), (0.006, 0.0), (0.006, 0.02), (0.009, 0.022), (0.009, 0.075), (0.006, 0.09), (0.0, 0.09)]
    lathe("plug_ins", plug, "ceramic", loc=(-0.03, ys[2], 0.34), rot=(-math.pi / 2, 0, 0))
    lathe("plug_hex", ring_prof(0.0, 0.011, 0.012, ch=0.001), "steel_machined", loc=(-0.03, ys[2], 0.345),
          rot=(-math.pi / 2, 0, 0), segs=6, smooth_angle=10.0)
    lathe("plug_tip", ring_prof(0.0, 0.0025, 0.012), "copper", loc=(-0.03, ys[2], 0.432),
          rot=(-math.pi / 2, 0, 0))
    # --- gearbox bits on the right --------------------------------------------
    g1 = gear("gear_big", 37, 0.046, 0.0405, 0.022, 0.012, "steel_machined", loc=(0.30, 0.05, 0.10),
              helix=0.0)
    g2 = gear("gear_small", 24, 0.031, 0.0255, 0.022, 0.010, "steel_machined", loc=(0.30, 0.05, 0.10 + 0.0717),
              rot=(0, math.pi / 24, 0))
    g2["cv_glow"] = 1.0
    cylinder("shaft", 0.012, 0.36, "steel_ground", loc=(0.30, 0.05, 0.10))
    lathe("synchro_ring", ring_prof(0.030, 0.038, 0.010, ch=0.001), "brass", loc=(0.30, -0.02, 0.10))
    lathe("bearing_outer", ring_prof(0.026, 0.034, 0.012, ch=0.0008), "steel_ground", loc=(0.30, 0.17, 0.10))
    spring("spring", 0.018, 0.0028, 0.012, 7, "steel_dark", loc=(0.20, -0.20, 0.022), rot=(0, 0, 0.4))
    # clutch disc standing on edge
    lathe("clutch_face", ring_prof(0.075, 0.114, 0.0084, ch=0.0005), "friction", loc=(0.40, -0.18, 0.115),
          rot=(0, 0, 0.6))
    lathe("clutch_hub", ring_prof(0.012, 0.074, 0.003), "steel_machined", loc=(0.40, -0.18, 0.115), rot=(0, 0, 0.6))
    # forged crank-like web + machined journal
    box("crank_web", (0.06, 0.02, 0.11), "steel_forged", loc=(0.15, 0.28, 0.055), bev=0.008)
    cylinder("journal", 0.024, 0.03, "steel_ground", loc=(0.15, 0.255, 0.03))
    # brake line: copper tube with fluid core (cut in half lengthwise)
    lathe("brake_tube", ring_prof(0.0035, 0.0048, 0.30), "copper", loc=(0.12, -0.05, 0.006),
          rot=(0, 0, math.pi / 2 + 0.3))
    lathe("brake_fluid", [(0, -0.15), (0.0034, -0.15), (0.0034, 0.15), (0, 0.15)], "brake_fluid",
          loc=(0.12, -0.05, 0.016), rot=(0, 0, math.pi / 2 + 0.3))
    # rubber boot, black plastic knob, black bracket, aluminium case piece with machined face
    boot = [(0.0, 0.0)] + [(0.02 + (0.008 if k % 2 else 0.0), 0.008 * k) for k in range(8)] + [(0.0, 0.056)]
    lathe("boot", boot, "rubber", loc=(-0.20, -0.30, 0.0), rot=(-math.pi / 2, 0, 0))
    sphere("knob", 0.022, "plastic_black", (-0.10, -0.32, 0.022))
    box("bracket", (0.08, 0.04, 0.006), "paint_black", loc=(0.05, -0.30, 0.003), bev=0.002)
    lathe("case", ring_prof(0.07, 0.085, 0.10, ch=0.003), "cast_aluminium", loc=(0.30, 0.30, 0.09), rot=(0, 0, 0))
    lathe("case_flange", ring_prof(0.07, 0.10, 0.012, ch=0.001), "machined_aluminium", loc=(0.30, 0.244, 0.09))
    # translucent car-paint panel at cv_opacity 0.25 in front-left
    pan = box("panel", (0.004, 0.36, 0.26), "car_paint", loc=(0.22, -0.02, 0.24), bev=0.0015)
    sd = pan.modifiers.new("bend", "SIMPLE_DEFORM")
    sd.deform_method = "BEND"
    sd.angle = math.radians(25)
    sd.deform_axis = "Z"
    pan["cv_opacity"] = 0.25
    pan.location = (0.26, -0.33, 0.16)
    pan.rotation_euler = (0, 0, 0.9)
    st = lighting.setup_studio("dark", center=(0.12, 0.0, 0.14), size=0.6, key_azimuth=170.0)
    camera(orbit_eye((0.12, -0.02, 0.10), 118.0, 27.0, 1.35), (0.12, -0.02, 0.10), lens=45, fstop=6.0)
    return sc, dict(studio=st)


def car_proxy(body_opacity=1.0, glow_gearbox=0.0, drivetrain=False):
    """Crude car stand-in (correct package: wheels, body extents) for paint/glass/tyre look-dev."""
    root = bpy.data.objects.new("car_root", None)
    link(root)
    body = box("body", (1.76, 4.45, 0.66), "car_paint", loc=(0, -1.355, 0.63), bev=0.16, segs=5)
    cutters = []
    for y in (0.0, -2.62):
        cutters.append(cylinder(f"arch{y}", 0.37, 2.4, "car_paint", loc=(0, y, 0.305), rot=(0, 0, math.pi / 2)))
    bm = bmesh.new()
    for c in cutters:
        m2 = c.data.copy()
        m2.transform(c.matrix_basis)
        bm.from_mesh(m2)
        bpy.data.meshes.remove(m2)
    arches = mesh_obj("arches", bm, "car_paint")
    for c in cutters:
        bpy.data.objects.remove(c, do_unlink=True)
    boolean(body, arches)
    apply_mods(body)
    bpy.data.objects.remove(arches, do_unlink=True)
    cabin = box("cabin", (1.42, 2.25, 0.50), "glass", loc=(0, -1.62, 1.15), bev=0.14, segs=5)
    roof = box("roof", (1.30, 1.45, 0.05), "car_paint", loc=(0, -1.75, 1.395), bev=0.02)
    trim = box("trim", (1.775, 2.6, 0.025), "chrome", loc=(0, -1.30, 0.80), bev=0.008)
    grille = box("grille", (0.9, 0.04, 0.16), "plastic_black", loc=(0, 0.86, 0.55), bev=0.02)
    parts = [body, cabin, roof, trim, grille]
    tyre = [(0.2032, -0.09), (0.25, -0.1025), (0.295, -0.1), (0.3115, -0.075), (0.3115, 0.075),
            (0.295, 0.1), (0.25, 0.1025), (0.2032, 0.09)]
    for y in (0.0, -2.62):
        for side in (-1, 1):
            x = side * 0.735
            rot = (0, 0, -math.pi / 2)
            t = lathe(f"tyre{y}{side}", tyre + [tyre[0]], "tire_rubber", loc=(x, y, 0.305), rot=rot, segs=72)
            rim = [(0.0, side * 0.06), (0.06, side * 0.06), (0.17, side * 0.075), (0.2032, side * 0.085),
                   (0.2032, -side * 0.08), (0.19, -side * 0.08), (0.18, side * 0.06), (0.06, side * 0.045),
                   (0.0, side * 0.045)]
            rr = lathe(f"rim{y}{side}", rim, "rim_alloy", loc=(x, y, 0.305), rot=rot, segs=72)
            parts += [t, rr]
    if drivetrain:
        parts += [box("dt_engine", (0.45, 0.55, 0.50), "cast_iron", loc=(0, -0.10, 0.50), bev=0.03),
                  box("dt_head", (0.40, 0.55, 0.12), "cast_aluminium", loc=(0, -0.10, 0.80), bev=0.02),
                  lathe("dt_gearbox", [(0, -0.47), (0.17, -0.47), (0.12, -0.80), (0.09, -1.12), (0, -1.12)],
                        "cast_aluminium", loc=(0, 0, 0.36)),
                  cylinder("dt_prop", 0.033, 1.28, "steel_machined", loc=(0, -1.76, 0.335)),
                  sphere("dt_diff", 0.14, "cast_aluminium", (0, -2.62, 0.305)),
                  cylinder("dt_shafts", 0.0125, 1.30, "steel_machined", loc=(0, -2.62, 0.305),
                           rot=(0, 0, math.pi / 2))]
        for p in parts[-6:]:
            if p.name.startswith("dt_gearbox"):
                p["cv_glow"] = glow_gearbox
    for p in parts:
        p.parent = root
        if p.name in ("body", "roof", "trim", "grille"):
            p["cv_opacity"] = body_opacity
        if p.name == "cabin":
            p["cv_opacity"] = min(1.0, body_opacity * 1.2)
    return root, parts


def scene_light_car():
    sc = reset()
    root, parts = car_proxy()
    st = lighting.setup_studio("light")
    for m, y in (("car_paint", -0.4), ("chrome", -1.0), ("rim_alloy", -1.6), ("glass", -2.2),
                 ("tire_rubber", -2.8)):
        sphere(f"ball_{m}", 0.22, m, (-1.9, y, 0.22))
    eye = orbit_eye(lighting.CAR_CENTER, 228.0, 9.0, 9.5)
    camera(eye, (0.3, -0.9, 0.55), lens=40)
    return sc, dict(studio=st)


def scene_light_xray():
    sc = reset()
    root, parts = car_proxy(body_opacity=0.2, glow_gearbox=0.8, drivetrain=True)
    st = lighting.setup_studio("light")
    eye = orbit_eye(lighting.CAR_CENTER, 235.0, 24.0, 8.0)
    camera(eye, (0.0, -1.2, 0.45), lens=40)
    return sc, dict(studio=st)


def scene_road():
    sc = reset()
    root, parts = car_proxy()
    root.location = (0.0, 60.0, 0.0)            # car well down the road; lights follow it
    st = lighting.setup_studio("road", follow=root)
    bpy.context.view_layer.update()
    c = root.location
    eye = (c.x + 3.6, c.y + 3.2, 1.05)
    camera(eye, (c.x, c.y - 1.7, 0.55), lens=32)
    return sc, dict(studio=st)


def scene_dark_close():
    """Close-up of the same vignette: cast texture, section paint, friction, glow."""
    sc, info = scene_dark_mech()
    bpy.data.objects.remove(sc.camera, do_unlink=True)
    camera(orbit_eye((0.17, -0.05, 0.12), 128.0, 22.0, 0.62), (0.17, -0.05, 0.12), lens=50, fstop=8.0)
    return sc, info


def scene_dark_macro():
    """Macro on the clutch facing, spring and the cast-iron block corner."""
    sc, info = scene_dark_mech()
    bpy.data.objects.remove(sc.camera, do_unlink=True)
    camera(orbit_eye((0.36, -0.12, 0.10), 150.0, 18.0, 0.42), (0.36, -0.12, 0.10), lens=50, fstop=11.0)
    return sc, info


SCENES = {
    "dark_balls": scene_dark_balls,
    "dark_macro": scene_dark_macro,
    "dark_mech": scene_dark_mech,
    "dark_close": scene_dark_close,
    "light_car": scene_light_car,
    "light_xray": scene_light_xray,
    "road": scene_road,
}


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------

def cycles_settings(sc, samples=16, res=(640, 360)):
    sc.render.engine = "CYCLES"
    r = sc.render
    r.resolution_x, r.resolution_y = res
    r.resolution_percentage = 100
    r.image_settings.file_format = "PNG"
    r.image_settings.color_mode = "RGB"
    cy = sc.cycles
    cy.device = "CPU"
    cy.samples = samples
    cy.use_adaptive_sampling = True
    cy.adaptive_threshold = 0.05
    cy.use_denoising = True
    cy.denoiser = "OPENIMAGEDENOISE"
    cy.denoising_input_passes = "RGB_ALBEDO_NORMAL"
    cy.denoising_prefilter = "FAST"
    cy.max_bounces = 6
    cy.diffuse_bounces = 2
    cy.glossy_bounces = 3
    cy.transmission_bounces = 4
    cy.volume_bounces = 0
    cy.transparent_max_bounces = 12
    cy.caustics_reflective = False
    cy.caustics_refractive = False
    cy.blur_glossy = 1.0
    cy.sample_clamp_indirect = 4.0
    cy.use_light_tree = True
    cy.seed = 7
    lighting.setup_color_management(sc)


def workbench_settings(sc, res=(640, 360)):
    sc.render.engine = "BLENDER_WORKBENCH"
    r = sc.render
    r.resolution_x, r.resolution_y = res
    r.resolution_percentage = 100
    sh = sc.display.shading
    sh.light = "STUDIO"
    sh.color_type = "MATERIAL"
    sh.show_cavity = True
    sh.cavity_type = "BOTH"
    sh.show_shadows = True
    sh.shadow_intensity = 0.4
    sh.show_specular_highlight = True
    sc.display.render_aa = "8"
    sc.view_settings.view_transform = "Standard"
    sc.view_settings.look = "None"


def render(sc, path):
    sc.render.filepath = path
    t0 = time.time()
    bpy.ops.render.render(write_still=True)
    return time.time() - t0


def overlay_labels(path, sc, labels):
    from bpy_extras.object_utils import world_to_camera_view
    from PIL import Image, ImageDraw, ImageFont
    im = Image.open(path).convert("RGB")
    d = ImageDraw.Draw(im)
    try:
        font = ImageFont.truetype(os.path.join(ROOT, "assets", "fonts", "Inter-Medium.otf"), 10)
    except Exception:
        font = ImageFont.load_default()
    W, H = im.size
    for name, p in labels:
        co = world_to_camera_view(sc, sc.camera, Vector(p))
        x, y = co.x * W, (1 - co.y) * H
        tw = d.textlength(name, font=font)
        d.text((x - tw / 2, y + 2), name, fill=(235, 235, 235), font=font)
    im.save(path)


def verify_opacity(out_dir):
    """Numeric check: pixel(background A) vs pixel(background B) for cubes with
    cv_opacity 1 / 0 / 0.25 / missing.  Measured opacity = 1 - dPix/dBg."""
    import numpy as np
    res = {}
    sc = reset()
    cycles_settings(sc, samples=16, res=(250, 50))
    sc.cycles.use_denoising = False
    sc.cycles.use_adaptive_sampling = False
    sc.view_settings.view_transform = "Standard"
    sc.view_settings.look = "None"
    w = bpy.data.worlds.new("w")
    w.use_nodes = True
    w.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.0
    sc.world = w
    bgm = bpy.data.materials.new("bg")
    nt = bgm.node_tree
    nt.nodes.clear()
    em = nt.nodes.new("ShaderNodeEmission")
    o = nt.nodes.new("ShaderNodeOutputMaterial")
    nt.links.new(em.outputs[0], o.inputs[0])
    bm = bmesh.new()
    bmesh.ops.create_grid(bm, x_segments=1, y_segments=1, size=5)
    me = bpy.data.meshes.new("bg")
    bm.to_mesh(me)
    me.materials.append(bgm)
    bg = link(bpy.data.objects.new("bg", me))
    bg.location = (0, 0, -2)
    # single-sided quads (exact), plus a closed cube at 0.25 (two surfaces:
    # expected 1 - 0.75^2 = 0.4375)
    cases = [("1.0", 1.0), ("0.0", 0.0), ("0.25", 0.25), ("missing", None), ("cube_0.25", 0.25)]
    for i, (lab, val) in enumerate(cases):
        if lab.startswith("cube"):
            ob = box(f"c{i}", (0.6, 0.6, 0.6), "section_cut", loc=(-2.0 + i, 0, 0))
        else:
            bmq = bmesh.new()
            bmesh.ops.create_grid(bmq, x_segments=1, y_segments=1, size=0.3)
            ob = mesh_obj(f"q{i}", bmq, "section_cut", loc=(-2.0 + i, 0, 0))
        del ob["cv_opacity"]
        del ob["cv_glow"]
        if val is not None:
            ob["cv_opacity"] = val
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sun.data.energy = 3.0
    link(sun)
    cd = bpy.data.cameras.new("c")
    cd.type = "ORTHO"
    cd.ortho_scale = 5.0
    cam = link(bpy.data.objects.new("c", cd))
    cam.location = (0, 0, 5)
    sc.camera = cam
    pix = []
    for col in ((1.0, 0.0, 0.0, 1.0), (0.0, 1.0, 0.0, 1.0)):
        em.inputs["Color"].default_value = col
        p = os.path.join(out_dir, "verify_tmp.exr")
        sc.render.image_settings.file_format = "OPEN_EXR"
        sc.render.filepath = p
        bpy.ops.render.render(write_still=True)
        img = bpy.data.images.load(p)
        a = np.array(img.pixels[:], dtype=np.float32).reshape(50, 250, 4)[..., :3]
        bpy.data.images.remove(img)
        pix.append(a)
    d_pix = pix[1] - pix[0]
    d_bg = np.array([-1.0, 1.0, 0.0])
    for i, (lab, val) in enumerate(cases):
        cx = int((i + 0.5) * 50)
        patch = d_pix[20:30, cx - 5:cx + 5].reshape(-1, 3)
        frac = float(np.mean(patch[:, 1] / d_bg[1]))          # fraction of background seen (green ch.)
        res[lab] = round(1.0 - frac, 4)
    os.remove(os.path.join(out_dir, "verify_tmp.exr"))
    return res


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--only", default="")
    ap.add_argument("--samples", type=int, default=16)
    ap.add_argument("--workbench", action="store_true")
    ap.add_argument("--verify", action="store_true")
    ap.add_argument("--threads", type=int, default=0)
    ap.add_argument("--save-blend", action="store_true")
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    if a.verify:
        res = verify_opacity(a.out)
        print("[verify] measured opacity:", res)
        ok = (abs(res["1.0"] - 1) < 0.02 and abs(res["0.0"]) < 0.02 and abs(res["0.25"] - 0.25) < 0.03
              and abs(res["missing"] - 1) < 0.02 and abs(res["cube_0.25"] - 0.4375) < 0.03)
        print("[verify]", "PASS" if ok else "FAIL")
        return 0 if ok else 1
    names = [n for n in (a.only.split(",") if a.only else SCENES) if n]
    times = {}
    for n in names:
        sc, info = SCENES[n]()
        if a.threads:
            sc.render.threads_mode = "FIXED"
            sc.render.threads = a.threads
        if a.workbench:
            workbench_settings(sc)
            path = os.path.join(a.out, f"{n}_wb.png")
        else:
            cycles_settings(sc, a.samples)
            path = os.path.join(a.out, f"{n}.png")
        if a.save_blend:
            bpy.ops.wm.save_as_mainfile(filepath=os.path.join(a.out, f"{n}.blend"))
        dt = render(sc, path)
        if info.get("labels"):
            overlay_labels(path, sc, info["labels"])
        times[n] = round(dt, 2)
        print(f"[lookdev] {n}: {dt:.1f} s -> {path}", flush=True)
    with open(os.path.join(a.out, "times_wb.json" if a.workbench else "times.json"), "w") as fh:
        json.dump(times, fh, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
