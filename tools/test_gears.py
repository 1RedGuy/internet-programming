#!/usr/bin/env python3
"""Self-test of carviz.gears / carviz.meshutil.

1. Mesh checks (shapely): every gear pair of the car is checked for
   interpenetration (max intersection area must be ~0) and separation
   (min/max gap must stay small) over a full tooth pitch of rotation and, for
   helical/spiral gears, across the face width.  Fails loudly (exit 1).
2. Mesh builds: every builder runs; meshes must be manifold, outward
   (positive volume) and within the polygon budget; build time is reported.
3. Test renders (Cycles, 640x360, <= 16 spp) of the helical pair, the
   reverse idler trio, the final drive, the differential and the small
   parts (splines, dog ring, sleeve, sprocket, ring gear).

usage: python3 tools/test_gears.py [--no-render] [--out DIR] [--samples N]
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402

from carviz import gears as G  # noqa: E402
from carviz import spec as S  # noqa: E402

DEG = math.pi / 180

# thresholds
AREA_TOL = 1e-12          # m^2  (any overlap is reported; 1e-12 m^2 = 1 um^2)


def check_table():
    rows = []
    mn = S.GEAR_NORMAL_MODULE
    hx = S.GEAR_HELIX
    pa = S.GEAR_PRESSURE_ANGLE
    width = 0.018
    pairs = [("input/cs headset", S.Z_INPUT, S.Z_CS_DRIVEN)]
    for g, (zc, zm) in S.GEAR_PAIRS.items():
        pairs.append((f"gear {g} (cs/main)", zc, zm))
    for label, za, zb in pairs:
        t = time.time()
        r = G.check_helical_pair(za, zb, mn, hx, width, pressure_angle=pa, handA="right",
                                 centre_distance=S.GEARBOX_CENTRE_DISTANCE, n_samples=32, n_slices=5)
        rows.append((label, f"{za}/{zb}", "helical m2.25 25deg R/L", r, time.time() - t, mn))
    m = S.REV_MODULE
    for label, za, zb in (("reverse cs/idler", S.Z_REV_CS, S.Z_REV_IDLER),
                          ("reverse idler/out", S.Z_REV_IDLER, S.Z_REV_OUT)):
        t = time.time()
        r = G.check_helical_pair(za, zb, m, 0.0, 0.015, pressure_angle=20 * DEG, n_samples=48, n_slices=1)
        rows.append((label, f"{za}/{zb}", "spur m2.5", r, time.time() - t, m))
    mo = S.RING_PITCH_DIAMETER / S.Z_RING
    for label, sp in (("final drive straight", 0.0), ("final drive spiral 35", 35 * DEG)):
        t = time.time()
        r = G.check_bevel_pair(S.Z_PINION_TEETH, S.Z_RING, mo, spiral=sp, handA="left", n_samples=32,
                               n_slices=5 if sp else 3)
        rows.append((label, f"{S.Z_PINION_TEETH}/{S.Z_RING}", f"bevel m{mo*1e3:.2f}", r, time.time() - t, mo))
    t = time.time()
    r = G.check_bevel_pair(S.Z_SPIDER, S.Z_SIDE_GEAR, DIFF_MODULE, pressure_angle=22.5 * DEG, n_samples=32)
    rows.append(("diff spider/side", f"{S.Z_SPIDER}/{S.Z_SIDE_GEAR}", f"bevel m{DIFF_MODULE*1e3:.1f} 22.5deg", r,
                 time.time() - t, DIFF_MODULE))
    # spline / dog-tooth fits (static, part locked on its mate): no overlap, small clearance
    from shapely.geometry import Polygon
    for label, n, rmin, rmaj in (("clutch hub spline", S.CLUTCH_DISC_SPLINE_TEETH, 0.0215 / 2, 0.0254 / 2),
                                 ("synchro hub/sleeve/dog", S.DOG_TEETH, 0.0300, 0.0335)):
        t = time.time()
        ext = G.spline_profile(n, rmin, rmaj)["xy"]
        intl = G.spline_profile(n, rmin, rmaj, internal=True)["xy"]
        pe = Polygon(ext).buffer(0)
        big = Polygon([(rmaj * 3 * math.cos(a), rmaj * 3 * math.sin(a))
                       for a in np.linspace(0, 2 * math.pi, 64, endpoint=False)])
        pi_ = big.difference(Polygon(intl)).buffer(0)
        area = pe.intersection(pi_).area
        gap = pe.distance(pi_) if area == 0 else 0.0
        rows.append((label, f"{n}/{n}", "spline fit (static)", dict(max_area=area, min_gap=gap, max_gap=gap),
                     time.time() - t, 2 * (rmaj - rmin)))
    # negative controls: the checker must catch a wrong phase / wrong hand
    neg = []
    pa_ = G.profile_polygon(26, mn, hx)
    pb_ = G.profile_polygon(35, mn, hx)
    r = G.check_mesh_2d(pa_, pb_, S.GEARBOX_CENTRE_DISTANCE, 0.0, 26 / 35, zA=26, zB=35,
                        phaseB=G.mesh_phase(26, 35, 0.0, 0.0) + math.pi / 35)
    neg.append(("26/35 phase off half pitch", r))
    # same hand on both gears must interfere away from the mid plane
    m_t, _ = G.transverse(mn, hx, pa)
    rpa, rpb = G.pitch_radius(26, m_t), G.pitch_radius(35, m_t)
    worst = 0.0
    for y in (-width / 2, width / 2):
        oa = float(G.helix_psi_offset(y, rpa, hx, "right"))
        ob = float(G.helix_psi_offset(y, rpb, hx, "right"))
        rr = G.check_mesh_2d(pa_, pb_, S.GEARBOX_CENTRE_DISTANCE, oa, 26 / 35, zA=26, zB=35,
                             phaseB=G.mesh_phase(26, 35, 0.0, 0.0) + ob, n_samples=16)
        worst = max(worst, rr["max_area"])
    neg.append(("26/35 same hand", dict(max_area=worst, min_gap=0.0, max_gap=0.0)))
    r = G.check_bevel_pair(10, 41, mo, spiral=35 * DEG, handA="left", handB="left", n_samples=16)
    neg.append(("10/41 spiral same hand", r))
    return rows, neg


DIFF_MODULE = 4.0e-3       # differential bevel module (not in spec; assembly choice)


def print_table(rows, neg):
    print()
    print(f"{'pair':24s} {'teeth':7s} {'type':26s} {'max area mm2':>12s} {'min gap mm':>10s} {'max gap mm':>10s}"
          f" {'time s':>6s}  result")
    print("-" * 112)
    fails = 0
    for label, teeth, typ, r, dt, mod in rows:
        # no overlap; always some clearance; never separated by more than 0.06 module
        # (= the designed backlash 0.05*m/2 per flank plus Tredgold/sampling slack)
        ok = r["max_area"] <= AREA_TOL and r["min_gap"] > 0 and r["max_gap"] < 0.06 * mod
        fails += 0 if ok else 1
        print(f"{label:24s} {teeth:7s} {typ:26s} {r['max_area']*1e6:12.3e} {r['min_gap']*1e3:10.4f} "
              f"{r['max_gap']*1e3:10.4f} {dt:6.2f}  {'OK' if ok else 'FAIL'}")
    print("negative controls (must detect interference):")
    for label, r in neg:
        ok = r["max_area"] > AREA_TOL
        fails += 0 if ok else 1
        print(f"  {label:34s} max area {r['max_area']*1e6:10.4f} mm2  {'detected' if ok else 'NOT DETECTED - FAIL'}")
    return fails


# ---------------------------------------------------------------------------
# Mesh builds + renders (bpy)
# ---------------------------------------------------------------------------


def build_all(collection=None):
    import bpy  # noqa: F401
    from carviz import meshutil as MU
    out = {}
    mn, hx = S.GEAR_NORMAL_MODULE, S.GEAR_HELIX
    mo = S.RING_PITCH_DIAMETER / S.Z_RING
    specs = [
        ("input26", lambda: G.helical_gear("input26", S.Z_INPUT, mn, hx, "right", 0.020, bore=0.028,
                                           hub=(0.045, 0.030, -0.004))),
        ("cs35", lambda: G.helical_gear("cs35", S.Z_CS_DRIVEN, mn, hx, "left", 0.020, bore=0.030,
                                        hub=(0.050, 0.026), web=0.012)),
        ("cs17", lambda: G.helical_gear("cs17", 17, mn, hx, "left", 0.022, bore=0.026)),
        ("main44", lambda: G.helical_gear("main44", 44, mn, hx, "right", 0.020, bore=0.040,
                                          hub=(0.060, 0.030), web=0.011)),
        ("main44_low", lambda: G.helical_gear("main44_low", 44, mn, hx, "right", 0.020, bore=0.040,
                                              hub=(0.060, 0.030), web=0.011, detail="low")),
        ("rev15", lambda: G.spur_gear("rev15", S.Z_REV_CS, S.REV_MODULE, 0.016, bore=0.022)),
        ("idler22", lambda: G.spur_gear("idler22", S.Z_REV_IDLER, S.REV_MODULE, 0.016, bore=0.018,
                                        hub=(0.030, 0.022))),
        ("rev38", lambda: G.spur_gear("rev38", S.Z_REV_OUT, S.REV_MODULE, 0.016, bore=0.040,
                                      hub=(0.058, 0.024), web=0.010)),
        ("pinion10", lambda: G.bevel_gear("pinion10", S.Z_PINION_TEETH, S.Z_RING, mo, spiral=35 * DEG,
                                          hand="left", back_hub=(0.036, 0.040))),
        ("ring41", lambda: G.bevel_gear("ring41", S.Z_RING, S.Z_PINION_TEETH, mo, spiral=35 * DEG,
                                        hand="right", bore=0.105)),
        ("pinion10_straight", lambda: G.bevel_gear("pinion10_straight", S.Z_PINION_TEETH, S.Z_RING, mo,
                                                   back_hub=(0.036, 0.040))),
        ("spider10", lambda: G.bevel_gear("spider10", S.Z_SPIDER, S.Z_SIDE_GEAR, DIFF_MODULE,
                                          pressure_angle=22.5 * DEG, bore=0.016, spherical_back=True)),
        ("side16", lambda: G.bevel_gear("side16", S.Z_SIDE_GEAR, S.Z_SPIDER, DIFF_MODULE,
                                        pressure_angle=22.5 * DEG, bore=0.024, spherical_back=True,
                                        back_hub=(0.040, 0.012))),
        ("ring132", lambda: G.ring_gear("ring132", S.FLYWHEEL_RING_TEETH)),
        ("sprocket21", lambda: G.sprocket("sprocket21", S.CRANK_SPROCKET_TEETH, rows=2, bore=0.025)),
        ("sprocket42", lambda: G.sprocket("sprocket42", S.CAM_SPROCKET_TEETH, rows=2, bore=0.030,
                                          hub=(0.050, 0.030))),
    ]
    extra = [
        ("shaft_spl23", lambda: G.external_splines("shaft_spl23", S.CLUTCH_DISC_SPLINE_TEETH, 0.0254, 0.0215,
                                                   0.040)),
        ("hub_spl23", lambda: G.internal_splines("hub_spl23", S.CLUTCH_DISC_SPLINE_TEETH, 0.0254, 0.0215,
                                                 0.020, outer_d=0.040)),
        ("dog32", lambda: G.dog_ring("dog32", S.DOG_TEETH, 0.0300, 0.0335, 0.006, facing="+Y")),
        ("sleeve32", lambda: G.sleeve_internal_teeth("sleeve32", S.DOG_TEETH, 0.0300, 0.0335, 0.020,
                                                     r_outer=0.043)),
        ("hub32", lambda: G.external_splines("hub32", S.DOG_TEETH, 0.067, 0.060, 0.016, bore=0.036)),
    ]
    for name, fn in specs + extra:
        t = time.time()
        ob = fn()
        dt = time.time() - t
        st = MU.mesh_stats(ob)
        out[name] = (ob, st, dt)
    return out


def report_builds(built):
    print()
    print(f"{'part':18s} {'verts':>7s} {'tris':>7s} {'non-manifold':>12s} {'volume cm3':>11s} {'build ms':>9s}")
    fails = 0
    for name, (_ob, st, dt) in built.items():
        ok = st["non_manifold_edges"] == 0 and st["volume"] > 0 and st["tris"] <= 60000 and dt < 1.0
        fails += 0 if ok else 1
        print(f"{name:18s} {st['verts']:7d} {st['tris']:7d} {st['non_manifold_edges']:12d} "
              f"{st['volume']*1e6:11.3f} {dt*1000:9.1f}  {'OK' if ok else 'FAIL'}")
    return fails


def _tri_world(ob, M):
    """World-space vertex list + polygon list of a mesh object under matrix M."""
    me = ob.data
    co = np.zeros(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    Mn = np.array(M)
    W = co @ Mn[:3, :3].T + Mn[:3, 3]
    return W, [tuple(p.vertices) for p in me.polygons]


def mesh_overlap_checks(n_samples=12):
    """3D check on the BUILT meshes: rotate meshing pairs through one tooth
    pitch (at kin phases) and count intersecting triangle pairs (BVH)."""
    import bpy
    from mathutils import Matrix
    from mathutils.bvhtree import BVHTree
    from carviz import kin as K
    bpy.ops.wm.read_factory_settings(use_empty=True)
    mn, hx = S.GEAR_NORMAL_MODULE, S.GEAR_HELIX
    C = S.GEARBOX_CENTRE_DISTANCE
    ph = K.gb_phase_table()

    def spin(loc, phase):
        return Matrix.Translation(loc) @ Matrix.Rotation(-phase, 4, "Y")

    def pair_check(label, A, B, MA_fn, MB_fn, zA, sweep_sign=1.0):
        t = time.time()
        PA, FA = _tri_world(A, Matrix.Identity(4))
        PB, FB = _tri_world(B, Matrix.Identity(4))
        worst = 0
        for i in range(n_samples):
            d = sweep_sign * (2 * math.pi / zA) * i / n_samples
            WA, _ = _tri_world(A, MA_fn(d))
            WB, _ = _tri_world(B, MB_fn(d))
            ta = BVHTree.FromPolygons([tuple(v) for v in WA], FA, epsilon=0.0)
            tb = BVHTree.FromPolygons([tuple(v) for v in WB], FB, epsilon=0.0)
            worst = max(worst, len(ta.overlap(tb)))
        del PA, PB
        return (label, worst, time.time() - t)

    res = []
    # gearbox helical pairs at kin phases (input/main gears RH, countershaft gears LH)
    a = G.helical_gear("t_in", S.Z_INPUT, mn, hx, "right", 0.022, bore=0.026)
    b = G.helical_gear("t_cs", S.Z_CS_DRIVEN, mn, hx, "left", 0.022, bore=0.030)
    k = -S.Z_INPUT / S.Z_CS_DRIVEN
    res.append(pair_check("headset 26/35 (mesh)", a, b,
                          lambda d: spin((0, 0, 0), ph["input_gear"][0] + d),
                          lambda d: spin((0, 0, -C), ph["cs_drive"][0] + k * d), S.Z_INPUT))
    zc, zm = S.GEAR_PAIRS[1]
    a = G.helical_gear("t_c1", zc, mn, hx, "left", 0.022, bore=0.026)
    b = G.helical_gear("t_m1", zm, mn, hx, "right", 0.022, bore=0.040, hub=(0.06, 0.03), web=0.011)
    res.append(pair_check("gear 1 17/44 (mesh)", a, b,
                          lambda d: spin((0, 0, -C), ph["cs_1"][0] + d),
                          lambda d: spin((0, 0, 0), ph["gear_1"][0] - d * zc / zm), zc))
    ix, iz = K.reverse_idler_centre()
    g15 = G.spur_gear("t_r15", S.Z_REV_CS, S.REV_MODULE, 0.016, bore=0.022)
    g22 = G.spur_gear("t_i22", S.Z_REV_IDLER, S.REV_MODULE, 0.016, bore=0.018)
    g38 = G.spur_gear("t_r38", S.Z_REV_OUT, S.REV_MODULE, 0.016, bore=0.040)
    ki = -S.Z_REV_CS / S.Z_REV_IDLER
    ko = ki * -S.Z_REV_IDLER / S.Z_REV_OUT
    res.append(pair_check("reverse 15/22 (mesh)", g15, g22,
                          lambda d: spin((0, 0, -C), ph["cs_R"][0] + d),
                          lambda d: spin((ix, 0, iz), ph["idler"][0] + ki * d), S.Z_REV_CS))
    res.append(pair_check("reverse 22/38 (mesh)", g22, g38,
                          lambda d: spin((ix, 0, iz), ph["idler"][0] + ki * d),
                          lambda d: spin((0, 0, 0), ph["gear_R"][0] + ko * d), S.Z_REV_CS))
    mo = S.RING_PITCH_DIAMETER / S.Z_RING
    F = G.bevel_pair_frames(S.Z_PINION_TEETH, S.Z_RING, mo)
    pin = G.bevel_gear("t_pin", S.Z_PINION_TEETH, S.Z_RING, mo, spiral=35 * DEG, hand="left")
    ring = G.bevel_gear("t_ring", S.Z_RING, S.Z_PINION_TEETH, mo, spiral=35 * DEG, hand="right", bore=0.105)
    phB = G.bevel_mesh_phase(S.Z_PINION_TEETH, S.Z_RING, 0.0, F["dirA"], F["dirB"], F["sense"])
    MA, MB = Matrix(F["MA"].tolist()), Matrix(F["MB"].tolist())
    r = S.Z_PINION_TEETH / S.Z_RING
    res.append(pair_check("final drive 10/41 spiral (mesh)", pin, ring,
                          lambda d: MA @ Matrix.Rotation(-d, 4, "Y"),
                          lambda d: MB @ Matrix.Rotation(-(phB + F["sense"] * d * r), 4, "Y"), S.Z_PINION_TEETH))
    F2 = G.bevel_pair_frames(S.Z_SPIDER, S.Z_SIDE_GEAR, DIFF_MODULE)
    sp = G.bevel_gear("t_sp", S.Z_SPIDER, S.Z_SIDE_GEAR, DIFF_MODULE, pressure_angle=22.5 * DEG, bore=0.016,
                      spherical_back=True)
    sd = G.bevel_gear("t_sd", S.Z_SIDE_GEAR, S.Z_SPIDER, DIFF_MODULE, pressure_angle=22.5 * DEG, bore=0.024,
                      spherical_back=True, back_hub=(0.040, 0.014))
    phB2 = G.bevel_mesh_phase(S.Z_SPIDER, S.Z_SIDE_GEAR, 0.0, F2["dirA"], F2["dirB"], F2["sense"])
    MA2, MB2 = Matrix(F2["MA"].tolist()), Matrix(F2["MB"].tolist())
    r2 = S.Z_SPIDER / S.Z_SIDE_GEAR
    res.append(pair_check("diff 10/16 (mesh)", sp, sd,
                          lambda d: MA2 @ Matrix.Rotation(-d, 4, "Y"),
                          lambda d: MB2 @ Matrix.Rotation(-(phB2 + F2["sense"] * d * r2), 4, "Y"), S.Z_SPIDER))
    # synchro: sleeve on hub, and sleeve engaged over the dog ring (aligned)
    r_in, r_out = 0.0300, 0.0335
    hub = G.external_splines("t_hub", S.DOG_TEETH, 2 * r_out, 2 * r_in, 0.016, bore=0.036)
    slv = G.sleeve_internal_teeth("t_slv", S.DOG_TEETH, r_in, r_out, 0.020, r_outer=0.044)
    dog = G.dog_ring("t_dog", S.DOG_TEETH, r_in, r_out, 0.008, facing="+Y")
    res.append(pair_check("sleeve on hub (mesh)", hub, slv, lambda d: spin((0, 0, 0), d),
                          lambda d: spin((0, 0.003, 0), d), S.DOG_TEETH))
    res.append(pair_check("sleeve over dogs (mesh)", dog, slv, lambda d: spin((0, 0, 0), d),
                          lambda d: spin((0, 0.010, 0), d), S.DOG_TEETH))
    # negative control: dogs misaligned by half a pitch must collide
    res.append(pair_check("NEG dogs half-pitch off", dog, slv, lambda d: spin((0, 0, 0), d),
                          lambda d: spin((0, 0.010, 0), d + math.pi / S.DOG_TEETH), S.DOG_TEETH))
    return res


def print_mesh_checks(res):
    print()
    print(f"{'3D mesh check (BVH)':34s} {'max overlapping tri pairs':>26s} {'time s':>7s}")
    fails = 0
    for label, worst, dt in res:
        neg = label.startswith("NEG")
        ok = (worst > 0) if neg else (worst == 0)
        fails += 0 if ok else 1
        print(f"{label:34s} {worst:26d} {dt:7.2f}  {'OK' if ok else 'FAIL'}")
    return fails


def _setup_render(out_dir, samples):
    import bpy
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = samples
    sc.cycles.use_denoising = True
    try:
        sc.cycles.denoiser = "OPENIMAGEDENOISE"
    except Exception:
        pass
    sc.render.resolution_x, sc.render.resolution_y = 640, 360
    sc.render.resolution_percentage = 100
    sc.render.threads_mode = "FIXED"
    sc.render.threads = 2
    sc.view_settings.view_transform = "AgX"
    world = bpy.data.worlds.new("w")
    sc.world = world
    world.use_nodes = True
    nt = world.node_tree
    bg = nt.nodes.get("Background")
    hdr = os.path.join(ROOT, "assets", "hdri", "studio_small_03_2k.hdr")
    if os.path.exists(hdr):
        env = nt.nodes.new("ShaderNodeTexEnvironment")
        env.image = bpy.data.images.load(hdr)
        nt.links.new(env.outputs["Color"], bg.inputs["Color"])
        bg.inputs["Strength"].default_value = 0.35
    else:
        bg.inputs["Color"].default_value = (0.3, 0.3, 0.32, 1)
    mat = bpy.data.materials.new("test_steel")
    mat.use_nodes = True
    b = mat.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = (0.62, 0.62, 0.64, 1)
    b.inputs["Metallic"].default_value = 1.0
    b.inputs["Roughness"].default_value = 0.28
    mat.diffuse_color = (0.6, 0.6, 0.62, 1)
    sec = bpy.data.materials.new("section_cut")
    sec.use_nodes = True
    b = sec.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = (0.8, 0.15, 0.08, 1)
    b.inputs["Roughness"].default_value = 0.5
    floor = bpy.data.materials.new("test_floor")
    floor.use_nodes = True
    b = floor.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = (0.18, 0.18, 0.2, 1)
    b.inputs["Roughness"].default_value = 0.6
    return mat, floor


def _key_light(loc, target, energy=30.0, size=0.3):
    import bpy
    from mathutils import Vector
    ld = bpy.data.lights.new("key", "AREA")
    ld.energy = energy
    ld.size = size
    lo = bpy.data.objects.new("key", ld)
    bpy.context.scene.collection.objects.link(lo)
    lo.location = loc
    lo.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    return lo


def _camera(loc, target, lens=50.0):
    import bpy
    from mathutils import Vector
    cd = bpy.data.cameras.new("cam")
    cd.lens = lens
    cd.clip_start = 0.005
    co = bpy.data.objects.new("cam", cd)
    bpy.context.scene.collection.objects.link(co)
    co.location = loc
    co.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.camera = co
    return co


def _place_spin(ob, loc, phase):
    ob.rotation_mode = "XYZ"
    ob.location = loc
    ob.rotation_euler = (0.0, -phase, 0.0)


def _clear_scene():
    import bpy
    for ob in list(bpy.data.objects):
        bpy.data.objects.remove(ob)


def render_tests(out_dir, samples=16, only=None):
    import bpy
    from mathutils import Matrix
    from carviz import kin as K
    from carviz import meshutil as MU
    os.makedirs(out_dir, exist_ok=True)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    mat, floor_mat = _setup_render(out_dir, samples)
    mn, hx = S.GEAR_NORMAL_MODULE, S.GEAR_HELIX
    C = S.GEARBOX_CENTRE_DISTANCE
    files = []

    def floor(z):
        f = MU.extrude_polygon("floor", [(-2, -2), (2, -2), (2, 2), (-2, 2)], -0.001, 0.0, smooth_angle=None)
        f.data.transform(Matrix.Rotation(math.pi / 2, 4, "X"))
        f.location = (0, 0, z)
        MU.assign_material(f, floor_mat)

    def shoot(name, cam_loc, target, lens, key_loc, energy=30):
        _camera(cam_loc, target, lens)
        _key_light(key_loc, target, energy)
        _key_light((-key_loc[0] * 0.8, key_loc[1] * 0.5, key_loc[2] * 0.6), target, energy * 0.3, 0.6)
        p = os.path.join(out_dir, name + ".png")
        if only and not any(o in name for o in only):
            return
        bpy.context.scene.render.filepath = p
        t = time.time()
        bpy.ops.render.render(write_still=True)
        print(f"rendered {p} in {time.time() - t:.1f}s")
        files.append(p)

    # 1. helical headset pair (input 26 over countershaft 35), kin phases
    ph = K.gb_phase_table()
    a = G.helical_gear("input26", S.Z_INPUT, mn, hx, "right", 0.022, bore=0.026, hub=(0.042, 0.036, -0.006),
                       material=mat)
    b = G.helical_gear("cs35", S.Z_CS_DRIVEN, mn, hx, "left", 0.022, bore=0.030, hub=(0.048, 0.030), web=0.012,
                       material=mat)
    shaft = MU.cylinder("shaft", 0.013, -0.06, 0.06, 48, chamfer=0.001, material=mat)
    cshaft = MU.cylinder("cshaft", 0.015, -0.06, 0.06, 48, chamfer=0.001, material=mat)
    _place_spin(a, (0, 0, 0), ph["input_gear"][0])
    _place_spin(shaft, (0, 0, 0), 0)
    _place_spin(b, (0, 0, -C), ph["cs_drive"][0])
    _place_spin(cshaft, (0, 0, -C), 0)
    floor(-C - 0.06)
    shoot("gears_helical_pair", (0.17, 0.16, -0.005), (0.0, 0.0, -0.036), 60, (0.3, 0.4, 0.4))

    # 2. reverse idler trio (spur, m 2.5) at kin positions
    _clear_scene()
    ix, iz = K.reverse_idler_centre()
    g15 = G.spur_gear("rev15", S.Z_REV_CS, S.REV_MODULE, 0.016, bore=0.022, material=mat)
    g22 = G.spur_gear("idler22", S.Z_REV_IDLER, S.REV_MODULE, 0.016, bore=0.018, hub=(0.030, 0.024), material=mat)
    g38 = G.spur_gear("rev38", S.Z_REV_OUT, S.REV_MODULE, 0.016, bore=0.040, hub=(0.058, 0.026), web=0.010,
                      material=mat)
    _place_spin(g15, (0, 0, -C), ph["cs_R"][0])
    _place_spin(g22, (ix, 0, iz), ph["idler"][0])
    _place_spin(g38, (0, 0, 0), ph["gear_R"][0])
    floor(-C - 0.05)
    shoot("gears_reverse_trio", (0.05, 0.30, -0.02), (0.02, 0.0, -0.035), 50, (0.3, 0.5, 0.5))

    # 3. final drive: spiral bevel pinion 10 (LH) + ring 41 (RH), common apex at origin
    _clear_scene()
    mo = S.RING_PITCH_DIAMETER / S.Z_RING
    F = G.bevel_pair_frames(S.Z_PINION_TEETH, S.Z_RING, mo)
    pin = G.bevel_gear("pinion10", S.Z_PINION_TEETH, S.Z_RING, mo, spiral=35 * DEG, hand="left",
                       back_hub=(0.036, 0.045), material=mat)
    ring = G.bevel_gear("ring41", S.Z_RING, S.Z_PINION_TEETH, mo, spiral=35 * DEG, hand="right", bore=0.105,
                        material=mat)
    phA = 0.0
    phB = G.bevel_mesh_phase(S.Z_PINION_TEETH, S.Z_RING, phA, F["dirA"], F["dirB"], F["sense"])
    pin.matrix_world = Matrix(F["MA"].tolist()) @ Matrix.Rotation(-phA, 4, "Y")
    ring.matrix_world = Matrix(F["MB"].tolist()) @ Matrix.Rotation(-phB, 4, "Y")
    floor(-0.11)
    shoot("gears_final_drive", (-0.16, -0.25, 0.14), (0.03, -0.04, 0.0), 45, (-0.3, -0.2, 0.6))

    # 4. differential: 2 spiders (straight bevel 10T, on the cross pin = world Y) and
    #    2 side gears (16T, on the axle axis = world X); common apex at the origin.
    #    spider B = spider A turned pi about X; side 2 = side 1 turned pi about Z
    #    (valid because 16 and 10 are even, see gears.bevel_pair_frames).
    _clear_scene()
    F2 = G.bevel_pair_frames(S.Z_SPIDER, S.Z_SIDE_GEAR, DIFF_MODULE)
    kw = dict(pressure_angle=22.5 * DEG, spherical_back=True, material=mat)
    phA = 0.0
    phB = G.bevel_mesh_phase(S.Z_SPIDER, S.Z_SIDE_GEAR, phA, F2["dirA"], F2["dirB"], F2["sense"])
    MA, MB = Matrix(F2["MA"].tolist()), Matrix(F2["MB"].tolist())
    Rx, Rz = Matrix.Rotation(math.pi, 4, "X"), Matrix.Rotation(math.pi, 4, "Z")
    for k, (Mp, Ms) in enumerate(((MA, MB), (Rx @ MA, Rz @ MB))):
        sp = G.bevel_gear(f"spider{k}", S.Z_SPIDER, S.Z_SIDE_GEAR, DIFF_MODULE, bore=0.016, **kw)
        sd = G.bevel_gear(f"side{k}", S.Z_SIDE_GEAR, S.Z_SPIDER, DIFF_MODULE, bore=0.024,
                          back_hub=(0.040, 0.014), **kw)
        sp.matrix_world = Mp @ Matrix.Rotation(-phA, 4, "Y")
        sd.matrix_world = Ms @ Matrix.Rotation(-phB, 4, "Y")
    pin = MU.cylinder("crosspin", 0.0079, -0.075, 0.075, 32, chamfer=0.0008, material=mat)
    floor(-0.06)
    shoot("gears_differential", (0.07, -0.10, 0.15), (0.0, 0.0, -0.005), 45, (0.2, -0.3, 0.5))

    # 5. small parts: synchro hub + sleeve (quarter cut) + dog ring; clutch-hub
    #    splines (half cut); duplex timing sprocket
    _clear_scene()
    r_in, r_out = 0.0300, 0.0335
    hub = G.external_splines("hub32", S.DOG_TEETH, 2 * r_out, 2 * r_in, 0.016, bore=0.036, material=mat)
    slv = G.sleeve_internal_teeth("sleeve32", S.DOG_TEETH, r_in, r_out, 0.020, r_outer=0.044, material=mat)
    dog = G.dog_ring("dog32", S.DOG_TEETH, r_in, r_out, 0.008, facing="+Y", material=mat)
    for ob, y in ((hub, 0.0), (slv, 0.0), (dog, -0.020)):
        _place_spin(ob, (0.0, y, 0.0), 0.0)
    MU.cut_quarter(slv, "+X", "+Z", space="LOCAL")
    MU.cut_quarter(hub, "+X", "+Z", space="LOCAL")
    shaft = G.external_splines("shaft23", S.CLUTCH_DISC_SPLINE_TEETH, 0.0254, 0.0215, 0.06, material=mat)
    hsp = G.internal_splines("hub23", S.CLUTCH_DISC_SPLINE_TEETH, 0.0254, 0.0215, 0.016, outer_d=0.040,
                             material=mat)
    MU.cut_half(hsp, "+Z", space="LOCAL")
    _place_spin(shaft, (-0.085, 0.0, 0.0), 0.0)
    _place_spin(hsp, (-0.085, 0.0, 0.0), 0.0)
    spr = G.sprocket("sprocket21", S.CRANK_SPROCKET_TEETH, rows=2, bore=0.025, material=mat)
    _place_spin(spr, (0.095, 0.0, 0.0), 0.0)
    for ob in (hub, slv, hsp):
        MU.assign_material(ob, "section_cut", faces=[p.index for p in ob.data.polygons
                                                     if ob.data.materials[p.material_index] is not None
                                                     and ob.data.materials[p.material_index].name.startswith("section_cut")])
    floor(-0.05)
    shoot("gears_small_parts", (0.03, 0.17, 0.20), (0.0, -0.005, 0.0), 40, (0.3, 0.3, 0.6))
    return files


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--out", default=os.path.join(ROOT, "out", "test_gears"))
    ap.add_argument("--samples", type=int, default=16)
    ap.add_argument("--only", nargs="*", help="render only shots whose name contains one of these")
    ap.add_argument("--skip-checks", action="store_true")
    args = ap.parse_args()
    fails = 0
    if not args.skip_checks:
        rows, neg = check_table()
        fails = print_table(rows, neg)
        import bpy
        bpy.ops.wm.read_factory_settings(use_empty=True)
        built = build_all()
        fails += report_builds(built)
        fails += print_mesh_checks(mesh_overlap_checks())
    if fails:
        print(f"\n*** {fails} FAILURE(S) ***")
        sys.exit(1)
    print("\nall gear checks passed")
    if not args.no_render:
        render_tests(args.out, args.samples, args.only)


if __name__ == "__main__":
    main()
