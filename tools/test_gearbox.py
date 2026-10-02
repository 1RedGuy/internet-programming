#!/usr/bin/env python3
"""Gearbox assembly verification.  Run: python3 tools/test_gearbox.py [--detail high|low]
                                          [--no-render] [--no-collide] [--out DIR]

1. Build alone (low + high, every cutaway variant): build time, triangles, part checks.
2. Drive with realistic state.Programs and check numerically:
   * read-back: baked rotations/translations = Track/kin values (shafts, every gear,
     countershaft, idler, sleeves, rails, lever), rotation directions and speed ratios
     (output/input = 1/ratio in every gear, countershaft opposite, reverse backwards)
   * gear meshes: gears.check_mesh_2d / check_helical_pair at the kin phases (2D) and BVH
     overlap of every meshing pair at >= 24 sampled frames
   * synchro: sleeve vs hub / blocker rings / dog rings / fork, struts, blocker rings vs
     cones, at >= 24 frames covering neutral, contact, blocking, through and engaged, plus
     the analytic roof-clearance (blocker index and dog alignment vs sleeve depth) at EVERY
     frame; dogs aligned when engaged
   * selector: rail = fork = sleeve travel, finger tip inside the selected rail's slot,
     lever knob travel ~ spec gate, detent balls on the rails, interlock (one rail moves)
   * housings: no moving part touches the case / web / tail housing at any sampled frame
   * exploded 1-2 synchro: which parts the exploded parts pass through (meta['explode_hide'])
3. Renders (Cycles 640x360, <= 16 spp, OIDN, studio 'dark'): overall, half cutaway,
   quarter cutaway, synchro close-up, exploded 1-2 synchro, lever/tower, plus a 6-frame
   Workbench motion strip of the 1->2 shift.
"""
import argparse
import math
import os
import sys
import time

os.environ.setdefault("EGL_PLATFORM", "surfaceless")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Vector  # noqa: E402
from mathutils.bvhtree import BVHTree  # noqa: E402

from carviz import gears as G  # noqa: E402
from carviz import kin, state  # noqa: E402
from carviz import spec as S  # noqa: E402
from carviz.assemblies import gearbox as GB  # noqa: E402

FAILS = []
MM = 1e-3


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        FAILS.append(msg)


def fresh():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def tris(objs):
    n = 0
    for ob in objs:
        if ob.type == "MESH":
            ob.data.calc_loop_triangles()
            n += len(ob.data.loop_triangles)
    return n


# ---------------------------------------------------------------------------
# Programs
# ---------------------------------------------------------------------------

def shift_program():
    """1st -> 2nd at 3000 rpm (like s05, but 1/60 so the gears visibly move)."""
    P = state.Program(None, duration=17.0)
    P.slowmo.key(0, 1 / 60, "step")
    v1 = S.road_speed_kmh(3000, 1)
    P.speed_kmh.key(0, v1, "step")
    P.start_in_gear(1)
    P.throttle_rpm.key(0, 3000, "step"); P.throttle_rpm.key(1.0, 3000, "linear"); P.throttle_rpm.key(1.2, 800, "step")
    P.throttle_rpm.key(13.0, 800, "linear"); P.throttle_rpm.key(13.5, 1850, "ease")
    P.pedal.key(0, 0, "step"); P.pedal.key(1.0, 0, "linear"); P.pedal.key(2.0, 1, "ease")
    P.pedal.key(13.0, 1, "linear"); P.pedal.key(16.0, 0, "ease")
    P.disengage(1, 2.5, 1.6)
    P.engage(2, 4.8, travel=1.6, hold=3.0, through=2.4, seat=1.0)
    return P.run()


def cruise_program(gear, rpm=1500.0, dur=3.0, slow=1 / 40):
    P = state.Program(None, duration=dur)
    P.slowmo.key(0, slow, "step")
    v = S.road_speed_kmh(rpm, gear)          # negative in reverse
    P.speed_kmh.key(0, v, "step")
    P.start_in_gear(gear)
    P.throttle_rpm.key(0, rpm, "step")
    return P.run()


def linkage_program():
    """Car stationary, engine idling, clutch pressed: 1, 3, 4, 5, R in turn (each with a
    synchro event that stops the input side), lever across the gate in neutral."""
    P = state.Program(None, duration=30.0)
    P.slowmo.key(0, 1 / 30, "step")
    P.pedal.key(0, 1, "step")
    t = 0.5
    seq = [(1, -1), (3, 0), (4, 0), (5, 1), ("R", 1)]
    plane = 0
    for g, pl in seq:
        if pl != plane:
            t = P.select_plane(t, pl, 0.8) + 0.2
            plane = pl
        t = P.engage(g, t, travel=0.6, hold=1.0, through=0.6, seat=0.3) + 0.6
        if g != "R":
            t = P.disengage(g, t, 0.6) + 0.3
    return P.run()


# ---------------------------------------------------------------------------
# Collision helpers (static BVHs cached)
# ---------------------------------------------------------------------------

def bvh_world(ob, dg):
    ev = ob.evaluated_get(dg)
    me = ev.to_mesh()
    mw = ev.matrix_world
    co = np.zeros(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    M = np.array(mw)
    co = co @ M[:3, :3].T + M[:3, 3]
    polys = [tuple(p.vertices) for p in me.polygons]
    ev.to_mesh_clear()
    return BVHTree.FromPolygons([Vector(v) for v in co], polys, epsilon=0.0)


def collide_pairs(pairs, frames, static=()):
    """pairs: [(name_a, obj_a, name_b, obj_b)]; returns {(a, b): [(frame, n), ...]}."""
    sc = bpy.context.scene
    cache_static = {}
    bad = {}
    for f in frames:
        sc.frame_set(int(f))
        dg = bpy.context.evaluated_depsgraph_get()
        cache = {}

        def get(name, ob):
            if name in static:
                if name not in cache_static:
                    cache_static[name] = bvh_world(ob, dg)
                return cache_static[name]
            if name not in cache:
                cache[name] = bvh_world(ob, dg)
            return cache[name]
        for na, oa, nb, ob in pairs:
            n = len(get(na, oa).overlap(get(nb, ob)))
            if n:
                bad.setdefault((na, nb), []).append((int(f), n))
    return bad


def meshes_under(ob):
    """ob + its mesh children (rigidly attached: gear + dogs + cone) as separate entries."""
    return [ob] + [c for c in ob.children_recursive if c.type == "MESH"]


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------

def readback(A, T, label):
    P = A.parts
    sc = bpy.context.scene
    errs = {}
    for f in (1, T.n // 2, T.n):
        sc.frame_set(f)
        i = f - 1

        def e(key, val):
            errs[key] = max(errs.get(key, 0.0), abs(val))
        e("input_shaft", P["input_shaft"].rotation_euler[1] + T.theta_in[i])
        e("output_shaft", P["output_shaft"].rotation_euler[1] + T.theta_out[i])
        for nm in ("input_gear", "cs_drive", "cs_1", "cs_2", "cs_3", "cs_5", "cs_R", "idler", "gear_1", "gear_2",
                   "gear_3", "gear_5", "gear_R"):
            e(nm, P[nm].rotation_euler[1] + kin.gb_angle(nm, T.theta_in[i], T.theta_out[i]))
        e("countershaft", P["countershaft"].rotation_euler[1] + T.theta_cs[i])
        for k in ("12", "34", "5R"):
            d = T[f"sleeve_{k}"][i] * S.SLEEVE_TRAVEL
            e(f"sleeve_{k}", P[f"sleeve_{k}"].parent.location.y - d)
            e(f"rail_{k}", P[f"rail_{k}"].location.y - d)
            e(f"hub_{k}", P[f"hub_{k}"].rotation_euler[1] + T.theta_out[i])
    worst = max(errs.values())
    check(worst < 2e-5, f"{label}: baked rotations/translations = Track/kin (max err {worst:.1e}; "
                        f"{len(errs)} channels)")
    return errs


def speed_ratios(A, T, gear):
    P = A.parts
    sc = bpy.context.scene
    sc.frame_set(1)
    a0 = {k: -P[k].rotation_euler[1] for k in ("input_shaft", "output_shaft", "countershaft", "idler")}
    g0 = {g: -P[f"gear_{g}"].rotation_euler[1] for g in (1, 2, 3, 5, "R")}
    sc.frame_set(T.n)
    a1 = {k: -P[k].rotation_euler[1] for k in a0}
    g1 = {g: -P[f"gear_{g}"].rotation_euler[1] for g in g0}
    d_in = a1["input_shaft"] - a0["input_shaft"]
    d_out = a1["output_shaft"] - a0["output_shaft"]
    d_cs = a1["countershaft"] - a0["countershaft"]
    ratio = d_in / d_out
    check(abs(ratio - S.GEAR_RATIOS[gear]) < 1e-4 * abs(S.GEAR_RATIOS[gear]) + 1e-6,
          f"gear {gear}: baked input/output turns = {ratio:+.4f} (spec {S.GEAR_RATIOS[gear]:+.4f}), "
          f"out {S.rad_s_to_rpm(T.w_out[0]):.0f} rpm, in {S.rad_s_to_rpm(T.w_in[0]):.0f} rpm")
    check(d_cs * d_in < 0 and abs(d_cs / d_in + S.Z_INPUT / S.Z_CS_DRIVEN) < 1e-6,
          f"gear {gear}: countershaft turns opposite to the input at 26/35 ({d_cs / d_in:+.4f})")
    if gear in (1, 2, 3, 5, "R"):
        dg = g1[gear] - g0[gear]
        check(abs(dg - d_out) < 1e-6 * max(1, abs(d_out)), f"gear {gear}: engaged gear turns with the output shaft")
    if gear == "R":
        check(d_out < 0 < d_in, "reverse: output shaft turns backwards while the input turns forwards")
    else:
        check(d_out > 0 and d_in > 0, f"gear {gear}: output turns forwards (CW-F)")
    for g in (1, 2, 3, 5, "R"):
        dg = g1[g] - g0[g]
        k = kin.gb_phase_table()[f"gear_{g}"][1]
        assert abs(dg - k * d_in) < 1e-6 * max(1.0, abs(dg)), g


def mesh2d_checks():
    print("2D mesh checks at the kin phases (gears.check_mesh_2d, every transverse slice):")
    PT = kin.gb_phase_table()
    C = S.GEARBOX_CENTRE_DISTANCE
    m, hx = S.GEAR_NORMAL_MODULE, S.GEAR_HELIX
    pairs = [("input_gear", "cs_drive", S.Z_INPUT, S.Z_CS_DRIVEN, (0.0, 0.0), (0.0, -C), FACE("input"))]
    for g, (zc, zm) in S.GEAR_PAIRS.items():
        pairs.append((f"cs_{g}", f"gear_{g}", zc, zm, (0.0, -C), (0.0, 0.0), FACE(g)))
    for (a, b, za, zb, ca, cb, w) in pairs:
        handA = "left" if a.startswith("cs") else "right"
        handB = "right" if handA == "left" else "left"
        dirn = math.atan2(cb[1] - ca[1], cb[0] - ca[0])
        m_t, at = G.transverse(m, hx, S.GEAR_PRESSURE_ANGLE)
        pa = G.involute_profile(za, m_t, at, m_n=m, detail="high")
        pb = G.involute_profile(zb, m_t, at, m_n=m, detail="high")
        worst = dict(max_area=0.0, min_gap=1e9)
        rpa, rpb = G.pitch_radius(za, m_t), G.pitch_radius(zb, m_t)
        for y in np.linspace(-w / 2, w / 2, 5):
            oa = float(G.helix_psi_offset(y, rpa, hx, handA))
            ob = float(G.helix_psi_offset(y, rpb, hx, handB))
            r = G.check_mesh_2d(pa, pb, C, phaseA=PT[a][0] + oa, ratio=za / zb, phaseB=PT[b][0] + ob,
                                dir_angle=dirn, zA=za, zB=zb, n_samples=24)
            worst["max_area"] = max(worst["max_area"], r["max_area"])
            worst["min_gap"] = min(worst["min_gap"], r["min_gap"])
        check(worst["max_area"] < 1e-12 and worst["min_gap"] > 0,
              f"{a} / {b}: overlap {worst['max_area']:.1e} m^2, min gap {worst['min_gap'] * 1e6:.1f} um")
    # reverse train (spur, module 2.5, profile shifted)
    I = kin.reverse_idler_centre()
    cs = (0.0, -C)
    rm = S.REV_MODULE
    for (a, b, za, zb, ca, cb, xa, xb) in (("cs_R", "idler", S.Z_REV_CS, S.Z_REV_IDLER, cs, I, 0.15, -0.15),
                                           ("idler", "gear_R", S.Z_REV_IDLER, S.Z_REV_OUT, I, (0.0, 0.0), -0.15, 0.15)):
        pa = G.involute_profile(za, rm, 20 * math.pi / 180, x_shift=xa, m_n=rm, detail="high")
        pb = G.involute_profile(zb, rm, 20 * math.pi / 180, x_shift=xb, m_n=rm, detail="high")
        dirn = math.atan2(cb[1] - ca[1], cb[0] - ca[0])
        r = G.check_mesh_2d(pa, pb, math.dist(ca, cb), phaseA=PT[a][0], ratio=za / zb, phaseB=PT[b][0],
                            dir_angle=dirn, zA=za, zB=zb, n_samples=48)
        check(r["max_area"] < 1e-12 and r["min_gap"] > 0,
              f"{a} / {b} (x {xa:+.2f}/{xb:+.2f}): overlap {r['max_area']:.1e}, min gap {r['min_gap'] * 1e6:.1f} um")


def FACE(g):
    return GB.FACE[g] * MM


def analytic_synchro(T, label):
    """Roof clearance at every frame: blocker index (as baked) and the gear's dog
    misalignment vs the sleeve's depth past their ridges."""
    worst_b, worst_d, eng_mis = 1e9, 1e9, 0.0
    for k in ("12", "34", "5R"):
        s = np.asarray(T[f"sleeve_{k}"])
        bl = np.asarray(T[f"blocker_{k}"])
        for sigma, g in GB.SYNCHRO_SIDES[k].items():
            x = np.maximum(sigma * s, 0.0) * GB.TRAVEL
            off = GB.blocker_offset(bl, s, sigma)
            ridge = GB.U_BLK_RIDGE_C - GB.BLK_GAP + GB.blocker_axial(x)
            depth = GB.SLV_HALF + x - ridge
            for i in range(T.n):
                if depth[i] > 0 and abs(off[i]) > 0:
                    margin = GB.contact_depth(off[i]) - depth[i]
                    worst_b = min(worst_b, margin)
            mis = kin.dog_misalignment(g, np.asarray(T.theta_in), np.asarray(T.theta_out))
            dd = GB.SLV_HALF + x - GB.U_DOG_RIDGE
            for i in range(T.n):
                if dd[i] > 0:
                    margin = GB.contact_depth(mis[i]) - dd[i]
                    worst_d = min(worst_d, margin)
                if abs(s[i]) >= S.SYNC_ENGAGED and np.sign(s[i]) == sigma:
                    eng_mis = max(eng_mis, abs(mis[i]))
    if worst_b < 1e9:
        check(worst_b >= -1e-4, f"{label}: sleeve roofs never cut into an indexed blocker ring "
                                f"(min roof clearance {worst_b * 1000:.1f} um)")
    if worst_d < 1e9:
        check(worst_d >= -1e-4, f"{label}: sleeve meets the dog teeth only when aligned "
                                f"(min roof clearance {worst_d * 1000:.1f} um)")
    check(eng_mis < 1e-6, f"{label}: dogs exactly aligned whenever a sleeve is engaged (max {math.degrees(eng_mis):.1e} deg)")


def selector_checks(A, T, label):
    P = A.parts
    sc = bpy.context.scene
    worst_x, worst_y, worst_z, knob_y, knob_x = 0.0, 0.0, 0.0, 0.0, 0.0
    lever = P["lever"]
    lf = GB.LEVER["L_f"]
    frames = np.linspace(1, T.n, 60).astype(int)
    for f in frames:
        sc.frame_set(int(f))
        i = f - 1
        tip = lever.matrix_world @ Vector((0.0, 0.0, -lf))
        root = A.root.matrix_world.inverted() @ tip
        plane = T.lever_x[i]
        # selected rail = nearest plane
        k = {-1: "12", 0: "34", 1: "5R"}[int(round(plane))]
        d = T[f"sleeve_{k}"][i] * S.SLEEVE_TRAVEL
        slot_x = -plane * GB.SLOT_PITCH * MM
        worst_x = max(worst_x, abs(root.x - slot_x))
        worst_y = max(worst_y, abs((root.y - S.Y_SHIFT_LEVER) - d))
        worst_z = max(worst_z, abs(root.z - GB.TIP_Z * MM))
        knob = P["knob"].matrix_world @ Vector(GB.LEVER["knob_vec"])
        kn = A.root.matrix_world.inverted() @ knob
        knob_y = max(knob_y, abs(kn.y - (S.SHIFT_KNOB_REST[1])))
        knob_x = max(knob_x, abs(kn.x))
    check(worst_y < 2e-6, f"{label}: finger tip moves exactly with the selected rail (max err {worst_y * 1e6:.2f} um)")
    check(worst_x < 0.3e-3, f"{label}: finger tip inside the selected slot laterally (max off-centre "
                            f"{worst_x * 1e3:.2f} mm, clearance to neighbour plates "
                            f"{GB.SLOT_PITCH - 2.0 - GB.TIP_R:.2f} mm)")
    check(worst_z < 1.0e-3, f"{label}: finger tip stays at slot height (max {worst_z * 1e3:.2f} mm rise)")
    check(abs(knob_y - S.LEVER_THROW) < 0.004 or knob_y < 1e-4,
          f"{label}: knob fore/aft travel {knob_y * 1e3:.1f} mm (spec LEVER_THROW {S.LEVER_THROW * 1e3:.0f} mm)")
    if knob_x > 1e-4:
        check(abs(knob_x - S.LEVER_GATE_SPACING) < 0.003,
              f"{label}: knob lateral gate travel {knob_x * 1e3:.1f} mm (spec {S.LEVER_GATE_SPACING * 1e3:.0f} mm)")
    # direction: lever forward = rail rearward (sample the most-engaged frame)
    for k in ("12", "34", "5R"):
        s = np.asarray(T[f"sleeve_{k}"])
        if np.max(np.abs(s)) < 0.9:
            continue
        i = int(np.argmax(np.abs(s)))
        sc.frame_set(i + 1)
        knob = A.root.matrix_world.inverted() @ (P["knob"].matrix_world @ Vector(GB.LEVER["knob_vec"]))
        dy = knob.y - S.SHIFT_KNOB_REST[1]
        check(np.sign(dy) == -np.sign(s[i]), f"{label}: rail {k} at {s[i]:+.2f}: knob moves "
                                             f"{'forward' if dy > 0 else 'back'} {abs(dy) * 1e3:.1f} mm (class-1 lever)")


def gate_check(A, T):
    """Lever position vs spec.SHIFT_GATE at each engaged gear."""
    sc = bpy.context.scene
    P = A.parts
    seen = {}
    for i in range(T.n):
        g = T.gear[i]
        if g == "N" or g in seen:
            continue
        gg = g if g == "R" else int(g)
        if any(abs(T[f"sleeve_{k}"][i]) > 0.97 for k in ("12", "34", "5R")):
            sc.frame_set(i + 1)
            kn = A.root.matrix_world.inverted() @ (P["knob"].matrix_world @ Vector(GB.LEVER["knob_vec"]))
            gx = kn.x / S.LEVER_GATE_SPACING
            gy = (kn.y - S.SHIFT_KNOB_REST[1]) / S.LEVER_THROW
            seen[g] = (gx, gy)
            ex, ey = S.SHIFT_GATE[gg]
            check(abs(gx - ex) < 0.15 and abs(gy - ey) < 0.15,
                  f"H-pattern: gear {g}: knob at ({gx:+.2f}, {gy:+.2f}) gate units, spec {S.SHIFT_GATE[gg]}")


def collision_suite(A, T, label, frames, housings=True):
    P = A.parts
    pairs = []

    def add(a, b):
        if a in P and b in P:
            pairs.append((a, P[a], b, P[b]))
    add("input_gear", "cs_drive")
    for g in (1, 2, 3, 5):
        add(f"cs_{g}", f"gear_{g}")
    add("cs_R", "idler")
    add("idler", "gear_R")
    add("idler", "gear_1")
    add("idler", "cs_1")
    add("idler", "gear_5")
    add("gear_3", "gear_2")
    add("gear_R", "gear_1")
    for k in ("12", "34", "5R"):
        gs = GB.SYNCHRO_SIDES[k]
        sl = f"sleeve_{k}"
        add(sl, f"hub_{k}")
        add(sl, f"fork_{k}")
        for g in gs.values():
            add(sl, f"blocker_{g}")
            add(sl, f"dogs_{g}")
            add(f"blocker_{g}", f"cone_{g}")
            add(f"blocker_{g}", f"dogs_{g}")
            add(f"blocker_{g}", f"hub_{k}")
            add(f"blocker_{g}", "input_gear" if g == 4 else f"gear_{g}")
            add(sl, "input_gear" if g == 4 else f"gear_{g}")
            for i in range(3):
                add(f"strut_{k}_{i}", f"blocker_{g}")
        for i in range(3):
            add(f"strut_{k}_{i}", sl)
            add(f"strut_{k}_{i}", f"hub_{k}")
        for k2 in ("12", "34", "5R"):
            if k2 != k:
                add(f"fork_{k}", f"rail_{k2}")
                add(f"fork_{k}", f"fork_{k2}")
                add(f"head_{k}", f"head_{k2}")
                add(f"head_{k}", f"rail_{k2}")
        add("lever", f"head_{k}")
        add(f"detent_ball_{k}", f"rail_{k}")
    add("input_shaft", "output_shaft")
    add("output_shaft", "output_flange")
    add("output_shaft", "washers")
    for k in ("12", "34", "5R"):
        add("output_shaft", f"hub_{k}")
    static = set()
    if housings:
        hs = [h for h in ("case", "case_web", "tail_housing") if h in P]
        movers = ["input_gear", "cs_drive", "cs_1", "cs_2", "cs_3", "cs_5", "cs_R", "idler", "gear_1", "gear_2",
                  "gear_3", "gear_5", "gear_R", "sleeve_12", "sleeve_34", "sleeve_5R", "fork_12", "fork_34",
                  "fork_5R", "rail_12", "rail_34", "rail_5R", "head_12", "head_34", "head_5R", "lever",
                  "output_shaft", "input_shaft", "countershaft", "output_flange", "blocker_1", "blocker_2",
                  "blocker_3", "blocker_4", "blocker_5", "blocker_R", "dogs_1", "dogs_2", "dogs_3", "dogs_4",
                  "dogs_5", "dogs_R", "hub_12", "hub_34", "hub_5R", "washers", "detent_ball_12", "detent_ball_34",
                  "detent_ball_5R", "boot", "knob"]
        for h in hs:
            static.add(h)
            for m in movers:
                add(h, m)
        add("idler_shaft", "idler")
        add("idler_shaft", "gear_R")
    t = time.time()
    bad = collide_pairs(pairs, frames, static=static)
    print(f"  ({len(pairs)} pairs x {len(frames)} frames in {time.time() - t:.0f} s)")
    if bad:
        for (a, b), lst in sorted(bad.items()):
            print(f"     overlap {a} x {b}: frames {[f for f, n in lst][:12]} (max {max(n for f, n in lst)} tri pairs)")
    check(not bad, f"{label}: no interpenetration in {len(pairs)} pairs at {len(frames)} frames")
    return bad


def sample_frames(T, n=26):
    """Frames spread over the program + every synchro phase boundary."""
    fr = set(np.linspace(1, T.n, n).astype(int).tolist())
    for k in ("12", "34", "5R"):
        s = np.abs(np.asarray(T[f"sleeve_{k}"]))
        for thr in (0.05, S.SYNC_CONTACT, S.SYNC_BLOCK - 0.01, S.SYNC_BLOCK + 0.003, 0.55, 0.62,
                    S.SYNC_THROUGH - 0.01, S.SYNC_THROUGH + 0.02, 0.85, 0.99):
            idx = np.nonzero(np.diff(np.sign(s - thr)))[0]
            for i in idx[:4]:
                fr.add(int(i) + 1)
                fr.add(int(i) + 2)
    return sorted(f for f in fr if 1 <= f <= T.n)


# ---------------------------------------------------------------------------
# Renders
# ---------------------------------------------------------------------------

def setup_cycles(samples):
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = samples
    sc.cycles.use_adaptive_sampling = True
    sc.cycles.use_denoising = True
    try:
        sc.cycles.denoiser = "OPENIMAGEDENOISE"
    except Exception:
        pass
    sc.cycles.max_bounces = 6
    sc.cycles.glossy_bounces = 3
    sc.cycles.transparent_max_bounces = 8
    sc.render.resolution_x, sc.render.resolution_y = 640, 360
    sc.render.resolution_percentage = 100
    sc.render.film_transparent = False


def camera(name="cam"):
    sc = bpy.context.scene
    cd = bpy.data.cameras.new(name)
    cam = bpy.data.objects.new(name, cd)
    sc.collection.objects.link(cam)
    sc.camera = cam
    cd.clip_start = 0.005
    return cam


def aim(cam, eye, target, lens, fstop=None, focus=None):
    cam.location = eye
    d = Vector(target) - Vector(eye)
    cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
    cam.data.lens = lens
    if fstop:
        cam.data.dof.use_dof = True
        cam.data.dof.aperture_fstop = fstop
        cam.data.dof.focus_distance = focus or d.length
    else:
        cam.data.dof.use_dof = False


def show_variant(A, variant, removed_opacity=0.0):
    """Hide/show housings for a cutaway variant ('none' whole, else kept pieces)."""
    P = A.parts
    for name, ob in P.items():
        base = name.split("__")[0]
        if base in ("case", "case_web", "tail_housing", "oil"):
            if "__" not in name:
                vis = variant == "none"
            else:
                v = name.split("__")[1].rsplit("_", 1)[0]
                kind = name.rsplit("_", 1)[1]
                vis = v == variant and (kind == "kept" or removed_opacity > 0)
                if kind == "removed":
                    ob["cv_opacity"] = removed_opacity
            ob.hide_render = not vis
            ob.hide_viewport = not vis


def render_suite(out, samples):
    from carviz import lighting
    fresh()
    A = GB.build({"cutaway": ["none", "half", "quarter"], "detail": "high", "sections": ["synchro_12"], "oil": True})
    T = shift_program()
    sc = bpy.context.scene
    sc.frame_start, sc.frame_end = 1, T.n
    ex = np.zeros(T.n)
    A.drive(T, {"explode": ex})
    lighting.setup_color_management(sc)
    Zc = S.Z_CRANK
    yc = -0.80
    yh = A.meta["synchro"]["per_synchro"]["12"]["y_hub"]
    lighting.setup_studio("dark", center=(0.0, yc, Zc - 0.02), size=0.75, floor_z=0.0, key_azimuth=230.0)
    setup_cycles(samples)
    cam = camera()
    files = []

    def sections(on):
        for n, sn in A.meta["sections"].items():
            A.parts[n].hide_render = on
            A.parts[sn].hide_render = not on

    def shoot(name, eye, target, lens, frame, variant, fstop=None, sec=False):
        show_variant(A, variant)
        sections(sec)
        sc.frame_set(frame)
        aim(cam, eye, target, lens, fstop)
        path = os.path.join(out, f"gearbox_{name}.png")
        sc.render.filepath = path
        t = time.time()
        bpy.ops.render.render(write_still=True)
        print(f"  render {name}: {time.time() - t:.1f} s")
        files.append(path)

    i_block = int(np.argmax(np.asarray(T.syncing_12) > 0)) + 40
    i_eng2 = T.n - 30
    th = np.asarray(T.theta_out)
    cand = [i for i in range(T.n) if T.syncing_12[i] > 0]
    f_sec = 1 + min(cand, key=lambda i: abs(((th[i] + math.pi) % (2 * math.pi)) - math.pi - 0.35))
    shoot("overall", (-0.95, -0.05, Zc + 0.55), (0.0, -0.80, Zc - 0.03), 32, 1, "none")
    shoot("half", (-1.05, -0.62, Zc + 0.32), (0.0, -0.78, Zc - 0.03), 34, 1, "half")
    shoot("quarter", (-0.62, -0.30, Zc + 0.48), (0.0, -0.72, Zc - 0.01), 32, 1, "quarter")
    shoot("synchro_close", (-0.20, yh + 0.075, Zc + 0.035), (-0.030, yh + 0.016, Zc + 0.012), 50, i_block, "half",
          fstop=11.0)
    shoot("synchro_section", (-0.17, yh + 0.035, Zc + 0.13), (0.0, yh + 0.012, Zc + 0.0), 45, f_sec, "half",
          sec=True)
    # exploded 1-2 synchro (hide the neighbours listed in meta)
    exv = np.ones(T.n)
    A.bake_explode(T, {"explode": exv})
    hide = set(A.meta["explode_hide"])
    for n in hide:
        A.parts[n].hide_render = True
    shoot("exploded", (-0.34, -0.50, Zc + 0.16), (0.0, -0.64, Zc - 0.005), 40, i_block, "half", fstop=8.0)
    for n in hide:
        A.parts[n].hide_render = False
    A.bake_explode(T, {"explode": ex})
    shoot("lever", (-0.42, -0.76, Zc + 0.30), (0.0, -0.95, Zc + 0.11), 40, i_eng2, "half")
    return A, T, files


def workbench_strip(out):
    fresh()
    A = GB.build({"cutaway": ["half"], "detail": "low"})
    T = shift_program()
    sc = bpy.context.scene
    sc.frame_start, sc.frame_end = 1, T.n
    A.drive(T, {})
    show_variant(A, "half")
    sc.render.engine = "BLENDER_WORKBENCH"
    sc.display.shading.light = "STUDIO"
    sc.display.shading.color_type = "MATERIAL"
    sc.display.shading.show_cavity = True
    sc.render.resolution_x, sc.render.resolution_y = 640, 360
    cam = camera("wbcam")
    Zc = S.Z_CRANK
    yh = A.meta["synchro"]["per_synchro"]["12"]["y_hub"]
    aim(cam, (-0.20, yh + 0.004, Zc + 0.05), (0.0, yh + 0.004, Zc + 0.008), 50)
    s = np.asarray(T.sleeve_12)
    want = [1, int(np.argmax(np.abs(s) < 0.02)) + 3]
    for thr in (S.SYNC_CONTACT, S.SYNC_BLOCK, 0.60, S.SYNC_THROUGH + 0.05, 0.999):
        idx = np.nonzero((s[1:] >= thr) & (s[:-1] < thr))[0]
        if len(idx):
            want.append(int(idx[0]) + 2)
    files = []
    for f in want[:8]:
        sc.frame_set(f)
        p = os.path.join(out, f"gearbox_strip_{f:04d}.png")
        sc.render.filepath = p
        bpy.ops.render.render(write_still=True)
        files.append(p)
    try:
        from PIL import Image
        ims = [Image.open(p) for p in files]
        w, h = ims[0].size
        sheet = Image.new("RGB", (w * 3, h * ((len(ims) + 2) // 3)), (0, 0, 0))
        for i, im in enumerate(ims):
            sheet.paste(im, ((i % 3) * w, (i // 3) * h))
        sp = os.path.join(out, "gearbox_strip.png")
        sheet.save(sp)
        files.append(sp)
    except Exception as e:  # pragma: no cover
        print("contact sheet failed:", e)
    return files, [(f, round(float(s[f - 1]), 3)) for f in want[:8]]


# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--detail", default="high")
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--no-collide", action="store_true")
    ap.add_argument("--only-render", action="store_true")
    ap.add_argument("--samples", type=int, default=16)
    ap.add_argument("--out", default=os.path.join(ROOT, "out", "test_gearbox"))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    if not args.only_render:
        print("build:")
        for det in ("low", args.detail):
            fresh()
            t = time.time()
            A = GB.build({"cutaway": ["none", "half", "quarter"], "detail": det})
            bt = time.time() - t
            whole = [ob for k, ob in A.parts.items() if "__" not in k and ob.type == "MESH"]
            print(f"  detail={det}: build {bt:.1f} s, {len(A.parts)} parts, tris all variants "
                  f"{tris(A.parts.values()):,}, whole gearbox (no cut pieces) {tris(whole):,}")
            print(f"  timings {A.meta['timings']}")
            check(bt < 60.0, f"build time {bt:.1f} s < 60 s ({det})")
            check(tris(A.parts.values()) < 1_500_000, f"triangles < 1.5 M with every variant ({det})")
        for name in ("input_shaft", "countershaft", "output_shaft", "gear_1", "gear_2", "gear_3", "gear_5", "gear_R",
                     "idler", "hub_12", "sleeve_12", "blocker_2", "blocker_1", "dogs_2", "dogs_1", "cone_2", "fork_12",
                     "rail_12", "rail_34", "rail_5R", "lever", "knob", "case", "needle_bearing_2"):
            check(name in A.anchors, f"anchor {name}")
        for g in (1, 2, 3, 4, 5, "R", "N"):
            check(g in A.meta["power_path"] and all(n in A.parts for n in A.meta["power_path"][g]),
                  f"power_path[{g}] = {A.meta['power_path'][g]}")
        y = A.meta["y"]
        print(f"  main case length {A.meta['case_length'] * 1000:.0f} mm (front {S.Y_GEARBOX_FRONT}, rear face "
              f"{y['case_end']:.3f}); flange face {y['flange_face']:.3f} = spec {S.Y_GEARBOX_REAR}")
        check(abs(y["flange_face"] - S.Y_GEARBOX_REAR) < 1e-9, "output flange face at Y_GEARBOX_REAR")
        sy = A.meta["synchro"]
        print(f"  synchro: hub->gear face {sy['hub_to_gear_face'] * 1e3:.2f} mm, blocker ridge on cone "
              f"{sy['blocker_ridge_on_cone'] * 1e3:.2f} mm, dog ridge {sy['dog_ridge'] * 1e3:.2f} mm, "
              f"sleeve reaches the dogs at x = {sy['sleeve_x_at_dog_ridge'] * 1e3:.2f} mm "
              f"(SYNC_THROUGH = {S.SYNC_THROUGH * S.SLEEVE_TRAVEL * 1e3:.2f} mm)")
        # sleeve directions vs spec.SYNCHROS
        for k, (rear, front) in S.SYNCHROS.items():
            yh = A.meta["synchro"]["per_synchro"][k]["y_hub"]
            yf = A.meta["gear_y"][str(front)]
            yr = A.meta["gear_y"][str(rear)]
            check(yf > yh > yr, f"synchro {k}: gear {front} in front (+Y) of the hub, gear {rear} behind "
                                f"({yf:.3f} > {yh:.3f} > {yr:.3f})")
        mesh2d_checks()

        print("drive: 1st -> 2nd shift (1/60 slow motion):")
        fresh()
        A = GB.build({"cutaway": ["none"], "detail": args.detail})
        T = shift_program()
        check(not T.violations, f"shift program integrates cleanly {T.violations[:2]}")
        sc = bpy.context.scene
        sc.frame_start, sc.frame_end = 1, T.n
        t = time.time()
        A.drive(T, {"explode": np.zeros(T.n)})
        print(f"  drive (bake {T.n} frames): {time.time() - t:.1f} s")
        readback(A, T, "shift")
        analytic_synchro(T, "shift")
        sc_ = GB.synchro_clearance(T)
        check(sc_["blocker_mm"] >= -1e-4 and sc_["dogs_mm"] >= -1e-4 and sc_["engaged_misalignment_deg"] < 1e-6,
              f"gearbox.synchro_clearance helper agrees: {sc_}")
        selector_checks(A, T, "shift")
        i = T.idx(7.0)
        check(T.syncing_12[i] > 0 and abs(T.blocker_12[i]) > 0, "blocker ring indexed while syncing")
        sc.frame_set(i + 1)
        b2 = A.parts["blocker_2"]
        rel = -b2.rotation_euler[1] - T.theta_out[i]
        check(abs(abs(rel) - S.BLOCKER_INDEX * 2 * math.pi / S.DOG_TEETH) < 1e-6,
              f"blocker_2 turned {math.degrees(rel):+.3f} deg from the hub while blocking "
              f"(= BLOCKER_INDEX {math.degrees(S.BLOCKER_INDEX * 2 * math.pi / S.DOG_TEETH):.3f} deg)")
        ax = b2.parent.location.y
        check(abs(ax - GB.BLK_GAP * MM) < 1e-9, f"blocker_2 pressed onto the cone while blocking (axial {ax * 1e3:.2f} mm)")
        if not args.no_collide:
            fr = sample_frames(T)
            print(f"  collision frames ({len(fr)}): {fr}")
            collision_suite(A, T, "shift", fr)

        print("drive: cruise in each gear (speed read-back):")
        for g in (1, 2, 3, 4, 5, "R"):
            T = cruise_program(g)
            check(not T.violations, f"cruise {g} integrates cleanly {T.violations[:1]}")
            sc.frame_start, sc.frame_end = 1, T.n
            A.drive(T, {})
            speed_ratios(A, T, g)
            readback(A, T, f"cruise {g}")
            analytic_synchro(T, f"cruise {g}")
            if g in (4, "R") and not args.no_collide:
                collision_suite(A, T, f"cruise {g}", list(np.linspace(1, T.n, 24).astype(int)), housings=False)

        print("drive: linkage demonstration (car stationary, clutch pressed: 1, 3, 4, 5, R):")
        T = linkage_program()
        check(not T.violations, f"linkage program integrates cleanly {T.violations[:2]}")
        sc.frame_start, sc.frame_end = 1, T.n
        A.drive(T, {})
        readback(A, T, "linkage")
        analytic_synchro(T, "linkage")
        selector_checks(A, T, "linkage")
        gate_check(A, T)
        moving = np.stack([np.abs(np.asarray(T[f"sleeve_{k}"])) > 0.02 for k in ("12", "34", "5R")])
        check(int(np.max(moving.sum(axis=0))) <= 1, "interlock: never more than one rail out of neutral")
        if not args.no_collide:
            fr = sample_frames(T, 30)
            print(f"  collision frames ({len(fr)})")
            collision_suite(A, T, "linkage", fr)

        print("exploded 1-2 synchro:")
        T = shift_program()
        grp = set(A.meta["explode_group"])
        moved = set()
        for carrier_name in A.explode:
            ob = A.parts[carrier_name]
            moved |= {c.name[len(GB.PREFIX):] for c in ob.children_recursive if c.type == "MESH"}
        movers = sorted(n for n in moved if n in A.parts)
        others = [n for n, o in A.parts.items() if o.type == "MESH" and n not in moved and "__" not in n]
        pairs = [(a, A.parts[a], b, A.parts[b]) for a in movers for b in others]
        hit = set()
        for f_ex in (0.2, 0.4, 0.6, 0.8, 1.0):
            A.drive(T, {"explode": np.full(T.n, f_ex)})
            bad = collide_pairs(pairs, [1], static=())
            hit |= {b for (a, b) in bad}
        hit = sorted(hit)
        print(f"  moving parts: {movers}")
        print(f"  parts they pass through (explode 0.2..1): {hit}")
        hidden = set(A.meta["explode_hide"])
        check(set(hit) <= hidden, f"meta['explode_hide'] covers them (missing {sorted(set(hit) - hidden)})")
        A.drive(T, {"explode": np.ones(T.n)})
        inner = [(a, A.parts[a], b, A.parts[b]) for a in sorted(grp) for b in sorted(grp)
                 if a < b and a in A.parts and b in A.parts]
        bad = collide_pairs(inner, [1, T.n // 2])
        fam = lambda n: n.split("_")[-1] if n.split("_")[0] in ("gear", "dogs", "cone") else n  # noqa: E731
        bad = {k: v for k, v in bad.items() if fam(k[0]) != fam(k[1])}
        check(not bad, f"exploded group parts clear of each other {sorted(bad)}")
        A.drive(T, {"explode": np.zeros(T.n)})

    if not args.no_render:
        print("renders:")
        A, T, files = render_suite(args.out, args.samples)
        wb, info = workbench_strip(args.out)
        print("  strip frames (frame, sleeve_12):", info)
        for f in files + wb:
            print("  ", f)

    if FAILS:
        print(f"\n{len(FAILS)} FAILURES")
        for f in FAILS:
            print("   -", f)
        sys.exit(1)
    print("\nall gearbox checks passed")


if __name__ == "__main__":
    main()
