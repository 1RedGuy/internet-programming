#!/usr/bin/env python3
"""Clutch assembly verification.  Run: python3 tools/test_clutch.py [--detail high|low]
                                         [--no-render] [--out DIR] [--frames-checked N]

1. Build the clutch alone (cutaways 'none' + 'half', hydraulic cut, live section) and
   report build time and triangles (high and low detail).
2. Drive it with test Programs and check numerically:
   A) neutral, idle 850 rpm slowed 40x: engaged -> pedal to the floor -> held -> slow
      release (slip) -> engaged again (several crank turns, a full pedal cycle);
   B) pull-away in 1st (pedal down, select 1st, slip, lock) - validity + read-backs.
   * collisions (carviz.collide) between every moving part and its neighbours /
     housings at >= 24 frames spread over the pedal cycle and >= 2 crank turns
     (stand-in flywheel with the engine's cover-bolt holes, stand-in input shaft
     with the mating 23T splines)
   * baked rotations / translations / shape-key values read back against the Track
     (disc = input-shaft abs angle, cover/plate/spring/race = theta_e, plate lift,
     disc float, bearing, fork angle, slave, master, pedal) and spin directions
   * clamp: pedal up -> facings within 0.1 mm of flywheel and plate; pedal down ->
     clearance per face = (lift - cushion travel)/2
   * light contact (0 < gap < 0.1 mm) at every sampled frame: finger tips vs bearing
     nose, diaphragm rim vs plate ridge, fork vs bearing pads, fork vs slave pushrod
   * pedal pushrod tip vs master piston (free play at rest, closed after it)
   * hydraulics: slave = 1.655 x bearing, master/slave = area ratio, fork lever
   * spline clearance (2D) between the disc hub and the mating input-shaft spline
   * static clearances of the hydraulic line, slave, pedal box vs the bellhousing
3. Renders (Cycles 640x360, 16 spp, OIDN, studio 'dark') of overall / cutaway engaged /
   cutaway released / exploded / pedal box / slave+fork / live section, and a 6-frame
   Workbench motion strip, into --out.
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

from carviz import collide, kin, state  # noqa: E402
from carviz import gears as G  # noqa: E402
from carviz import meshutil as MU  # noqa: E402
from carviz import spec as S  # noqa: E402
from carviz.assemblies import clutch as CL  # noqa: E402

FAILS = []
MM = 1e-3


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        FAILS.append(msg)


def tris(objs):
    n = 0
    for ob in objs:
        if ob.type == "MESH":
            ob.data.calc_loop_triangles()
            n += len(ob.data.loop_triangles)
    return n


def fresh():
    bpy.ops.wm.read_factory_settings(use_empty=True)


# ---------------------------------------------------------------------------
# stand-ins (tests only): flywheel (engine profile + cover-bolt holes), input shaft
# ---------------------------------------------------------------------------

def standin_flywheel():
    yf0, yf1 = S.Y_FLYWHEEL_FRONT, S.Y_FLYWHEEL_FACE
    prof = [(0.0105, yf0 - 0.0015), (0.0425, yf0 - 0.0015), (0.0445, yf0 - 0.003), (0.128, yf0 - 0.003),
            (0.132, yf0 - 0.0005), (0.1368, yf0 - 0.0005), (0.1368, yf0 - 0.0124), (0.1395, yf0 - 0.0128),
            (0.1395, yf1 + 0.001), (0.1385, yf1), (0.1205, yf1), (0.1195, yf1 + 0.0012), (0.1175, yf1 + 0.0012),
            (0.1165, yf1), (0.0745, yf1), (0.072, yf1 + 0.008), (0.040, yf1 + 0.008), (0.040, yf1 + 0.0065),
            (0.0105, yf1 + 0.0065)]
    prof = [(r, y - CL.ROOT_LOC[1]) for r, y in prof]
    fw = MU.lathe("test_flywheel", prof, 128, closed=True, caps=False, material="cast_iron")
    MU.assign_material(fw, "steel_machined", faces=[])
    cut = []
    for k in range(6):
        a = 2 * math.pi * (k + 0.25) / 6
        c = MU.cylinder(f"fwh{k}", 0.0038, -0.0001, 0.014, segments=12)
        c.location = (0.1295 * math.cos(a), yf1 - CL.ROOT_LOC[1] - 0.001, 0.1295 * math.sin(a))
        c.data.transform(c.matrix_basis)
        c.matrix_basis.identity()
        cut.append(c)
    CL._COL = bpy.data.collections.get("clutch")
    CL._bool(fw, cut)
    fw.location = tuple(CL.ROOT_LOC)
    # flywheel bolt heads in the recess (engine: 8 x M10 at r 31 mm, heads to y_face + 0.3 mm)
    heads = []
    for k in range(8):
        a = 2 * math.pi * (k + 0.5) / 8
        h = MU.cylinder(f"fwb{k}", 0.0072, yf1 + 0.0003 - CL.ROOT_LOC[1], yf1 + 0.0065 - CL.ROOT_LOC[1],
                        segments=6, material="steel_dark")
        h.data.transform(__import__("mathutils").Matrix.Translation((0.031 * math.cos(a), 0, 0.031 * math.sin(a))))
        heads.append(h)
    hb = MU.join("test_flywheel_bolts", heads)
    hb.location = tuple(CL.ROOT_LOC)
    return fw, hb


def standin_shaft():
    sp = CL.SPLINE
    sh = G.external_splines("test_input_spline", sp["n"], sp["d_major"], sp["d_minor"], 0.034,
                            flank_angle=sp["flank_angle"], fill=sp["fill"], bore=0.008)
    sh.location = tuple(CL.ROOT_LOC + np.array([0.0, CL.HUB_YC - 0.004, 0.0]))
    body = MU.cylinder("test_input_body", 0.0150, -0.130, -0.025, segments=48, material="steel_machined")
    body.location = tuple(CL.ROOT_LOC)
    return sh, body


# ---------------------------------------------------------------------------
# programs
# ---------------------------------------------------------------------------

def program_release(duration=7.0, slow=1.0 / 40.0):
    """s03-like: engaged idle, press, hold, slow release (slip), engaged (neutral)."""
    P = state.Program(None, duration=duration)
    P.slowmo.key(0, slow, "step")
    P.throttle_rpm.key(0, S.IDLE_RPM, "step")
    P.pedal.key(0.0, 0.0, "step")
    P.pedal.key(0.8, 0.0, "linear")
    P.pedal.key(2.3, 1.0, "ease")
    P.pedal.key(3.2, 1.0, "linear")
    P.pedal.key(6.2, 0.0, "ease")
    P.w_in0_rpm = S.IDLE_RPM
    return P.run()


def program_pullaway():
    P = state.Program(None, duration=9.0)
    P.slowmo.key(0, 1.0 / 30.0, "step")
    P.slowmo.key(3.2, 1.0 / 30.0, "linear")
    P.slowmo.key(3.6, 0.5, "ease")
    P.throttle_rpm.key(0, S.IDLE_RPM, "step")
    P.throttle_rpm.key(5.0, S.IDLE_RPM, "linear")
    P.throttle_rpm.key(5.6, 1500, "ease")
    P.pedal.key(0.0, 0.0, "step")
    P.pedal.key(0.6, 0.0, "linear")
    P.pedal.key(2.0, 1.0, "ease")
    P.select_plane(3.2, -1, dur=0.4)
    t_in = P.engage(1, 3.7)
    P.pedal.key(t_in + 0.3, 1.0, "linear")
    P.pedal.key(8.4, 0.0, "ease")
    P.speed_kmh.key(0.0, 0.0, "step")
    P.speed_kmh.key(6.1, 0.0, "linear")
    P.speed_kmh.key(8.6, 9.5, "ease")
    return P.run()


def sample_frames(T, n):
    return sorted(set(np.linspace(1, T.n, n).round().astype(int).tolist()))


# ---------------------------------------------------------------------------
# geometry helpers
# ---------------------------------------------------------------------------

def world_verts(ob, dg):
    ev = ob.evaluated_get(dg)
    me = ev.to_mesh()
    co = np.zeros(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    M = np.array(ev.matrix_world)
    ev.to_mesh_clear()
    co = co.reshape(-1, 3)
    return co @ M[:3, :3].T + M[:3, 3]


def bvh_of(ob, dg):
    ev = ob.evaluated_get(dg)
    me = ev.to_mesh()
    mw = ev.matrix_world
    vs = [mw @ v.co for v in me.vertices]
    ps = [tuple(p.vertices) for p in me.polygons]
    ev.to_mesh_clear()
    return BVHTree.FromPolygons(vs, ps, epsilon=0.0)


def min_gap(verts, bvh):
    best = 1e9
    for v in verts:
        hit = bvh.find_nearest(Vector(tuple(v)))
        if hit[0] is not None:
            best = min(best, hit[3])
    return best


def ang_err(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


# ---------------------------------------------------------------------------

def build_stats():
    print("== 1. build ==")
    out = {}
    for detail in ("high", "low"):
        fresh()
        t = time.time()
        A = CL.build({"cutaway": ["none", "half", "half_px"], "detail": detail, "hydraulic_cut": True,
                      "section_rotating": True})
        dt = time.time() - t
        vis = [A.parts[k] for k in A.parts if "__" not in k and not k.startswith("fluid") and A.parts[k].type == "MESH"
               and k != "section_cutter"]
        n_vis = tris(vis)
        n_all = tris(A.meshes())
        out[detail] = (dt, n_vis, n_all)
        print(f"  detail={detail}: build {dt:.1f} s, {n_vis:,} tris (assembled view), {n_all:,} incl. cutaway pieces")
        check(dt < 60.0, f"build time {dt:.1f} s < 60 s ({detail})")
        lim = 1.5e6 if detail == "high" else 4.0e5
        check(n_all < lim, f"triangles {n_all:,} < {lim:,.0f} ({detail})")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--out", default=os.path.join(ROOT, "out", "test_clutch"))
    ap.add_argument("--frames-checked", type=int, default=30)
    ap.add_argument("--skip-stats", action="store_true")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    if not args.skip_stats:
        build_stats()

    # ---------------------------------------------------------------- design numbers
    print("== 2a. design (pure geometry) ==")
    D = CL.D
    print(f"  diaphragm: fulcrum r {CL.R_F*1e3:.1f} mm, rim r {CL.R_RIM*1e3:.1f}, finger contact r "
          f"{D['r_c']*1e3:.2f}, nose r {D['R_N']*1e3:.2f}, release rotation {math.degrees(D['alpha']):.2f} deg")
    print(f"  lever (geometric) {(CL.R_F - D['r_c']) / (CL.R_RIM - CL.R_F):.3f}  (kin while lifting: "
          f"{CL.FINGER_REL / S.PRESSURE_PLATE_LIFT:.3f})")
    pp = np.linspace(0, 1, 201)
    g = kin.clutch_geometry(pp)
    check(np.allclose(g["slave"], CL.FORK_RATIO * g["bearing"], atol=1e-12), "slave = 1.655 x bearing (kin)")
    gam = CL.fork_angle(g["bearing"])
    check(np.max(np.abs(CL.slave_from_fork(gam) - g["slave"])) < 1e-9,
          "fork geometry: slave end travel = kin slave at every pedal position (exact lever)")
    print(f"  fork: pivot x {CL.X_PIVOT:.4f} (spec {S.RELEASE_FORK_PIVOT[0]}), slave x {CL.X_SLAVE:.4f} "
          f"(spec {S.SLAVE_CYL_POS[0]}), arms {CL.FORK_CX*1e3:.1f} : {CL.FORK_RATIO*CL.FORK_CX*1e3:.1f} mm, "
          f"max angle {math.degrees(gam.max()):.2f} deg")
    ar = (S.MASTER_CYL_BORE / S.SLAVE_CYL_BORE) ** 2
    check(np.allclose(g["slave"], g["master"] * ar), "master x A_m = slave x A_s (incompressible)")
    lo, hi = D["nose_gap_range"]
    check(0.0 < lo and hi < 0.1 * MM, f"nose gap (analytic, all pedal) {lo*1e3:.3f}..{hi*1e3:.3f} mm")
    check(abs(D["rim_gap_full"] - CL.GAP) < 1e-7, "rim stays on the ridge at full lift (analytic)")

    # ---------------------------------------------------------------- drive
    print("== 2b. drive (program A: neutral, press, slow release) ==")
    fresh()
    A = CL.build({"cutaway": ["none", "half", "half_px"], "detail": "high", "section_rotating": True,
                  "section_side": +1, "hydraulic_cut": True})
    fw, fwb = standin_flywheel()
    sh, sb = standin_shaft()
    T = program_release()
    v = T.validate(strict=False)
    check(not v, f"program A valid ({len(v)} violations)")
    print("  " + T.summary(1.0).replace("\n", "\n  "))
    t0 = time.time()
    A.drive(T, {"variant": "none"})
    print(f"  drive baked in {time.time() - t0:.2f} s for {T.n} frames")
    P = A.parts
    sc = bpy.context.scene
    # spin the stand-ins like the engine / gearbox would
    from carviz import rig
    rig.bake_spin(fw, T.frames, T.theta_e)
    rig.bake_spin(fwb, T.frames, T.theta_e)
    rig.bake_spin(sh, T.frames, T.theta_in)

    # read-backs ---------------------------------------------------------------
    print("  -- read-back vs Track")
    fr_chk = sample_frames(T, 9)
    worst = dict(disc=0, ee=0, lift=0, float_=0, bear=0, fork=0, slave=0, master=0, pedal=0, keys=0)
    for f in fr_chk:
        i = f - 1
        sc.frame_set(f)
        th_in_abs = T.theta_in[i] + CL.DISC_PHASE
        worst["disc"] = max(worst["disc"], ang_err(-P["disc"].rotation_euler[1], th_in_abs))
        for k in ("pressure_plate", "diaphragm_spring", "cover", "fulcrum", "straps", "bearing_race"):
            worst["ee"] = max(worst["ee"], ang_err(-P[k].rotation_euler[1], T.theta_e[i]))
        worst["lift"] = max(worst["lift"], abs(P["pressure_plate"].location[1] - (0 - T.clutch_plate_lift[i])))
        worst["float_"] = max(worst["float_"], abs(P["disc"].location[1] + T.clutch_plate_lift[i] / 2))
        worst["bear"] = max(worst["bear"], abs(P["release_bearing"].location[1] - A.meta["rest"]["release_bearing"][1]
                                               - T.clutch_bearing[i]))
        worst["fork"] = max(worst["fork"], abs(math.sin(P["fork"].rotation_euler[2]) * CL.FORK_CX - T.clutch_bearing[i]))
        worst["slave"] = max(worst["slave"], abs(A.meta["rest"]["slave_pushrod"][1] - P["slave_pushrod"].location[1]
                                                 - T.clutch_slave[i]))
        worst["master"] = max(worst["master"], abs(P["master_piston"].location[1] - A.meta["rest"]["master_piston"][1]
                                                   - T.clutch_master[i]))
        worst["pedal"] = max(worst["pedal"], abs(P["pedal"].rotation_euler[0] - T.clutch_pedal_angle[i]))
        kb = P["diaphragm_spring"].data.shape_keys.key_blocks
        worst["keys"] = max(worst["keys"], abs(kb["release"].value - T.clutch_plate_lift[i] / S.PRESSURE_PLATE_LIFT),
                            abs(kb["bend"].value - min(T.clutch_finger[i] / CL.F0, 1.0)))
    check(worst["disc"] < 1e-5, f"disc angle = theta_in (= gearbox input shaft) (max err {worst['disc']:.2e} rad)")
    check(worst["ee"] < 1e-5, f"cover/plate/spring/fulcrum/straps/race = theta_e (max err {worst['ee']:.2e} rad)")
    check(worst["lift"] < 1e-7, f"plate location = -plate_lift (err {worst['lift']:.1e} m)")
    check(worst["float_"] < 1e-7, f"disc floats by -plate_lift/2 (err {worst['float_']:.1e} m)")
    check(worst["bear"] < 1e-7, f"release bearing = clutch_bearing (err {worst['bear']:.1e} m)")
    check(worst["fork"] < 1e-7, f"fork angle: cx*sin(angle) = bearing (err {worst['fork']:.1e} m)")
    check(worst["slave"] < 1e-7, f"slave pushrod = clutch_slave (err {worst['slave']:.1e} m)")
    check(worst["master"] < 1e-7, f"master piston = clutch_master (err {worst['master']:.1e} m)")
    check(worst["pedal"] < 1e-6, f"pedal rotation = clutch_pedal_angle (err {worst['pedal']:.1e} rad)")
    check(worst["keys"] < 1e-5, f"diaphragm shape keys from the Track (err {worst['keys']:.1e})")
    # spin direction / speed: CW seen from the front = rotation_euler[1] decreasing
    i1, i2 = 5, 25
    sc.frame_set(i1 + 1)
    a1 = P["cover"].rotation_euler[1]
    sc.frame_set(i2 + 1)
    a2 = P["cover"].rotation_euler[1]
    w_meas = -(a2 - a1) / ((i2 - i1) / T.fps) / T.slowmo[i1]
    check(a2 < a1 and abs(w_meas - T.w_e[i1]) / T.w_e[i1] < 0.02,
          f"cover turns CW-F at engine speed: {w_meas * 60 / 2 / math.pi:.0f} rpm vs {T.rpm_e[i1]:.0f}")
    # fork direction: bearing end forward, slave end rearward
    sc.frame_set(int(np.argmax(T.pedal)) + 1)
    check(P["fork"].rotation_euler[2] > 0, "fork turns +Z: bearing end forward (+Y), slave end rearward")

    # clamp / clearances ----------------------------------------------------------
    print("  -- clamp and contact invariants")
    dg = bpy.context.evaluated_depsgraph_get()
    frames = sample_frames(T, max(args.frames_checked, 24))
    i_up = [f for f in frames if T.pedal[f - 1] < 1e-6]
    i_dn = [f for f in frames if T.pedal[f - 1] > 0.999]
    fy = S.Y_FLYWHEEL_FACE

    def facing_gaps(f):
        sc.frame_set(f)
        dg_ = bpy.context.evaluated_depsgraph_get()
        dv = world_verts(P["disc"], dg_)
        r = np.hypot(dv[:, 0] - CL.ROOT_LOC[0], dv[:, 2] - CL.ROOT_LOC[2])
        m = (r > CL.FACING_RI - 0.001) & (r < CL.FACING_RO + 0.001)
        pv = world_verts(P["pressure_plate"], dg_)
        rp = np.hypot(pv[:, 0] - CL.ROOT_LOC[0], pv[:, 2] - CL.ROOT_LOC[2])
        mp = (rp > CL.PP_RI - 0.001) & (rp < CL.PP_RO + 0.001)
        return fy - dv[m, 1].max(), dv[m, 1].min() - pv[mp, 1].max()

    g_up = [facing_gaps(f) for f in i_up]
    g_dn = [facing_gaps(f) for f in i_dn]
    if g_up:
        gu = np.array(g_up)
        check(gu.min() >= 0 and gu.max() < 0.1 * MM,
              f"pedal up: facing-flywheel {gu[:,0].min()*1e3:.3f}..{gu[:,0].max()*1e3:.3f} mm, facing-plate "
              f"{gu[:,1].min()*1e3:.3f}..{gu[:,1].max()*1e3:.3f} mm (clamped, < 0.1)")
    if g_dn:
        gd = np.array(g_dn)
        expect = (S.PRESSURE_PLATE_LIFT - CL.CUSHION_TRAVEL) / 2
        check(np.all(np.abs(gd - expect - CL.GAP) < 0.03 * MM),
              f"pedal down: clearance {gd[:,0].mean()*1e3:.3f} / {gd[:,1].mean()*1e3:.3f} mm per face "
              f"(expect (1.8-0.65)/2 = {expect*1e3:.3f} mm)")
    # light contacts
    cont = {"finger tips - bearing nose": ("bearing_race", "diaphragm_spring"),
            "diaphragm rim - plate ridge": ("pressure_plate", "diaphragm_spring"),
            "fork - bearing pads": ("fork", "release_bearing"),
            "fork - slave pushrod": ("slave_pushrod", "fork")}
    res = {k: [] for k in cont}
    for f in frames:
        sc.frame_set(f)
        dg_ = bpy.context.evaluated_depsgraph_get()
        for k, (a, b) in cont.items():
            va = world_verts(P[a], dg_)
            bb = bvh_of(P[b], dg_)
            # only vertices near the contact (closest 300 to b's bbox) for speed
            bvb = world_verts(P[b], dg_)
            lo_, hi_ = bvb.min(0) - 0.004, bvb.max(0) + 0.004
            sel = np.all((va > lo_) & (va < hi_), axis=1)
            res[k].append(min_gap(va[sel], bb) if np.any(sel) else 1.0)
    for k, vals in res.items():
        vals = np.array(vals)
        check(vals.min() > 0.0 and vals.max() < 0.12 * MM,
              f"light contact {k}: gap {vals.min()*1e3:.3f}..{vals.max()*1e3:.3f} mm over {len(vals)} frames")
    # pushrod tip vs master piston
    _, _, tip = CL.pushrod_pose(T.clutch_pedal_angle)
    gap = CL.Y_MPISTON0 + T.clutch_master - tip
    rest = gap[T.pedal < 1e-6]
    work = gap[T.pedal > S.CLUTCH_FREE_PLAY + 0.005]
    check(np.allclose(rest, CL.FREE_PLAY_GAP, atol=1e-6) and work.min() > 0 and work.max() < 0.25 * MM,
          f"pushrod-piston: free play {rest.mean()*1e3:.2f} mm at rest, {work.min()*1e3:.3f}..{work.max()*1e3:.3f} mm "
          f"after the free play (pedal arc vs linear kin)")

    # collisions --------------------------------------------------------------------
    print("  -- collisions")
    MOV = ["disc", "damper_springs", "pressure_plate", "straps", "diaphragm_spring", "fulcrum", "cover",
           "release_bearing", "bearing_race", "fork", "slave_pushrod", "master_piston", "pedal", "pushrod"]
    pairs_n = [
        ("disc", "test_flywheel"), ("disc", "test_flywheel_bolts"), ("disc", "pressure_plate"),
        ("disc", "test_input_spline"), ("disc", "diaphragm_spring"), ("disc", "cover"), ("disc", "straps"),
        ("disc", "damper_springs"), ("damper_springs", "test_flywheel"), ("damper_springs", "test_flywheel_bolts"),
        ("damper_springs", "pressure_plate"), ("pressure_plate", "diaphragm_spring"), ("pressure_plate", "cover"),
        ("pressure_plate", "fulcrum"), ("pressure_plate", "straps"), ("pressure_plate", "test_flywheel"),
        ("straps", "cover"), ("straps", "diaphragm_spring"), ("straps", "fulcrum"),
        ("diaphragm_spring", "fulcrum"), ("diaphragm_spring", "cover"), ("diaphragm_spring", "bearing_race"),
        ("diaphragm_spring", "release_bearing"), ("diaphragm_spring", "guide_tube"), ("fulcrum", "cover"),
        ("cover", "test_flywheel"), ("cover", "bellhousing"), ("cover", "fork"), ("cover", "release_bearing"),
        ("pressure_plate", "bellhousing"), ("straps", "bellhousing"), ("test_flywheel", "bellhousing"),
        ("release_bearing", "guide_tube"), ("release_bearing", "fork"), ("release_bearing", "ball_stud"),
        ("release_bearing", "bellhousing"), ("release_bearing", "test_input_body"),
        ("bearing_race", "guide_tube"), ("bearing_race", "release_bearing"), ("bearing_race", "fork"),
        ("bearing_race", "test_input_body"), ("fork", "ball_stud"), ("fork", "bellhousing"), ("fork", "guide_tube"),
        ("fork", "slave_pushrod"), ("fork", "slave_cylinder"), ("fork", "bellhousing_bolts"),
        ("slave_pushrod", "slave_cylinder"), ("slave_pushrod", "bellhousing"),
        ("master_piston", "master_cylinder"), ("master_piston", "pushrod"), ("pushrod", "master_cylinder"),
        ("pushrod", "pedal_box"), ("pushrod", "pedal"), ("pedal", "pedal_box"),
        ("guide_tube", "test_input_body"), ("test_input_spline", "guide_tube"),
    ]
    objs = dict(P)
    for o in (fw, fwb, sh, sb):
        objs[o.name] = o
    pairs = [(objs[a], objs[b]) for a, b in pairs_n]
    t0 = time.time()
    bad = collide.check_pairs(pairs, frames)
    print(f"  {len(pairs)} pairs x {len(frames)} frames in {time.time() - t0:.1f} s")
    by = {}
    for f, a, b, n in bad:
        by.setdefault((a, b), []).append((f, n))
    for (a, b), lst in by.items():
        print(f"    overlap {a} x {b}: frames {[f for f, _ in lst][:8]} ({max(n for _, n in lst)} face pairs)")
    check(not bad, f"no interpenetration at {len(frames)} frames (pedal cycle + {T.theta_e[-1]/2/math.pi:.1f} crank turns)")
    # static clearances (frame 1)
    static = [("slave_cylinder", "bellhousing"), ("hose_bracket", "slave_cylinder"), ("pedal_box", "master_cylinder"),
              ("master_cylinder", "line_02"), ("line_fittings", "bellhousing"), ("guide_tube", "ball_stud"),
              ("slave_cylinder", "line_08")]
    static += [(f"line_{i:02d}", "bellhousing") for i in range(CL.N_LINE)]
    static += [(f"line_{i:02d}", "pedal_box") for i in range(CL.N_LINE)]
    bad_s = collide.check_pairs([(objs[a], objs[b]) for a, b in static], [1])
    for f, a, b, n in bad_s:
        print(f"    static overlap {a} x {b} ({n})")
    check(not bad_s, f"static parts clear ({len(static)} pairs)")

    # spline clearance (2D) -------------------------------------------------------
    sp = CL.SPLINE
    ext = G.spline_profile(sp["n"], sp["d_minor"] / 2, sp["d_major"] / 2, flank_angle=sp["flank_angle"],
                           fill=sp["fill"])
    inn = G.spline_profile(sp["n"], sp["d_minor"] / 2, sp["d_major"] / 2, internal=True,
                           flank_angle=sp["flank_angle"], fill=sp["fill"])
    from shapely.geometry import Polygon, Point
    shaft = Polygon(ext["xy"])
    hub = Point(0, 0).buffer(0.03, 256).difference(Polygon(inn["xy"]))
    d = shaft.distance(hub)
    check(d > 0 and d < 0.2 * MM and not shaft.intersects(hub),
          f"disc hub splines slide on the 23T input splines with {d*1e3:.3f} mm clearance (2D)")

    # program B: pull-away ---------------------------------------------------------
    print("== 2c. program B: select 1st, slipping pull-away ==")
    TB = program_pullaway()
    vb = TB.validate(strict=False)
    check(not vb, f"program B valid ({len(vb)} violations: {vb[:2]})")
    print("  " + TB.summary(1.0).replace("\n", "\n  "))
    A.drive(TB, {"variant": "none"})
    i_stop = int(np.argmin(np.abs(TB.t - 5.0)))
    sc.frame_set(i_stop + 1)
    check(abs(TB.rpm_in[i_stop]) < 1.0 and TB.rpm_e[i_stop] > 800,
          f"in 1st at rest, pedal down: disc {TB.rpm_in[i_stop]:.0f} rpm, cover {TB.rpm_e[i_stop]:.0f} rpm")
    e_d = e_c = 0.0
    for f in sample_frames(TB, 12):
        sc.frame_set(f)
        # keyframes are float32: allow 2e-7 relative
        e_d = max(e_d, ang_err(-P["disc"].rotation_euler[1], TB.theta_in[f - 1] + CL.DISC_PHASE)
                  / (1e-5 + 2e-7 * abs(TB.theta_in[f - 1])))
        e_c = max(e_c, ang_err(-P["cover"].rotation_euler[1], TB.theta_e[f - 1]) / (1e-5 + 2e-7 * abs(TB.theta_e[f - 1])))
    check(e_d < 1 and e_c < 1, f"program B read-back: disc follows the input shaft, cover the engine "
          f"(err/tol {e_d:.2f}, {e_c:.2f}; angles up to {TB.theta_e[-1]:.0f} rad, float32 keys)")
    lock = np.nonzero(TB.status == "ENGAGED")[0]
    lock = lock[lock > i_stop]
    if len(lock):
        print(f"  clutch locks at t={TB.t[lock[0]]:.2f} s video, {TB.rpm_e[lock[0]]:.0f} rpm")

    if not args.no_render:
        A.drive(T, {"variant": "none"})
        render_all(A, T, args.out, fw, fwb, sh, sb)
    print("\nSUMMARY: " + ("ALL CHECKS PASSED" if not FAILS else f"{len(FAILS)} FAILED:\n  " + "\n  ".join(FAILS)))
    return 0 if not FAILS else 1


VIEWS = {
    # name: (variant, eye, target, lens, pedal-frame selector, extra presentation)
    "overall": ("none", (-0.95, 0.30, 0.95), (-0.24, -0.43, 0.46), 30, "up", {}),
    "cutaway_engaged": ("half", (-0.40, -0.66, 0.60), (-0.02, -0.37, 0.37), 42, "up", {}),
    "cutaway_released": ("half", (-0.36, -0.55, 0.47), (-0.03, -0.38, 0.36), 50, "down", {}),
    "exploded": ("none", (0.95, -0.40, 0.62), (0.0, -0.62, 0.36), 32, "up", {"explode": 1.0, "housing": 0.0}),
    "disc_detail": ("none", (-0.36, -0.22, 0.56), (0.0, -0.50, 0.36), 45, "up", {"explode": 1.0, "housing": 0.0}),
    "pedal_box": ("none", (-0.95, -0.40, 0.78), (-0.48, -0.52, 0.58), 34, "down", {}),
    "slave_fork": ("half", (-0.40, -0.47, 0.50), (-0.12, -0.40, 0.37), 48, "down", {}),
    "section": ("half_px", (0.62, -0.30, 0.50), (0.0, -0.37, 0.36), 40, "up", {"section": 1.0}),
}


def render_all(A, T, out, fw, fwb, sh, sb):
    from carviz import lighting, rig
    sc = bpy.context.scene
    st = lighting.setup_studio("dark", center=(-0.18, -0.42, 0.45), size=0.6)
    lighting.setup_color_management(sc)
    cd = bpy.data.cameras.new("test_cam")
    cam = bpy.data.objects.new("test_cam", cd)
    sc.collection.objects.link(cam)
    sc.camera = cam
    cd.clip_start = 0.005
    r = sc.render
    r.resolution_x, r.resolution_y = 640, 360
    f_up = int(np.argmin(T.pedal)) + 1
    f_dn = int(np.argmax(T.pedal)) + 1
    for name, (variant, eye, tgt, lens, sel, extra) in VIEWS.items():
        pres = {"variant": variant}
        for k, v in extra.items():
            pres[k] = np.full(T.n, v)
        A.drive(T, pres)
        hide_fw = 1.0 if name == "disc_detail" else 0.0
        for o in (fw, fwb):
            rig.bake_channel(o, "hide_render", -1, T.frames, np.full(T.n, hide_fw), "CONSTANT")
        for o in (fw, fwb):
            m = o.modifiers.get("sec")
            if name == "section" and m is None:
                m = o.modifiers.new("sec", "BOOLEAN")
                m.object = A.parts["section_cutter"]
                m.solver = "MANIFOLD"
                m.material_mode = "TRANSFER"
            if m is not None:
                m.show_render = name == "section"
        if "explode" in extra:
            rig.bake_channel(fw, "location", 1, T.frames, np.full(T.n, CL.ROOT_LOC[1] - 0.10))
            rig.bake_channel(fwb, "location", 1, T.frames, np.full(T.n, CL.ROOT_LOC[1] - 0.10))
            for o in (sh, sb):
                rig.bake_channel(o, "location", 1, T.frames, np.full(T.n, o.location[1] - 0.56))
        else:
            for o, y0 in ((fw, CL.ROOT_LOC[1]), (fwb, CL.ROOT_LOC[1])):
                rig.bake_channel(o, "location", 1, T.frames, np.full(T.n, y0))
            rig.bake_channel(sh, "location", 1, T.frames, np.full(T.n, CL.ROOT_LOC[1] + CL.HUB_YC - 0.004))
            rig.bake_channel(sb, "location", 1, T.frames, np.full(T.n, CL.ROOT_LOC[1]))
        sc.frame_set(f_up if sel == "up" else f_dn)
        r.engine = "CYCLES"
        cy = sc.cycles
        cy.device = "CPU"
        cy.samples = 16
        cy.use_adaptive_sampling = True
        cy.adaptive_threshold = 0.05
        cy.use_denoising = True
        cy.denoiser = "OPENIMAGEDENOISE"
        cy.transparent_max_bounces = 12
        cam.location = eye
        cam.rotation_euler = (Vector(tgt) - Vector(eye)).to_track_quat("-Z", "Y").to_euler()
        cam_az = math.degrees(math.atan2(eye[0] - tgt[0], -(eye[1] - tgt[1])))
        st["rig"].rotation_euler = (0.0, 0.0, math.radians(cam_az + 50.0 - 225.0))
        cd.lens = lens
        r.filepath = os.path.join(out, f"clutch_{name}.png")
        t = time.time()
        bpy.ops.render.render(write_still=True)
        print(f"  rendered {name} ({time.time() - t:.1f} s) -> {r.filepath}")
    # Workbench motion strip: half cut, the pedal going down (bearing, fork, fingers, plate)
    A.drive(T, {"variant": "half", "section": np.zeros(T.n)})
    r.engine = "BLENDER_WORKBENCH"
    sh_ = sc.display.shading
    sh_.light = "STUDIO"
    sh_.color_type = "MATERIAL"
    sh_.show_cavity = True
    sc.display.render_aa = "8"
    eye, tgt = (-0.36, -0.55, 0.47), (-0.03, -0.38, 0.36)
    cam.location = eye
    cam.rotation_euler = (Vector(tgt) - Vector(eye)).to_track_quat("-Z", "Y").to_euler()
    cd.lens = 50
    f0 = int(np.nonzero(T.pedal > 0.001)[0][0])
    f1 = int(np.argmax(T.pedal)) + 1
    for k, f in enumerate(np.linspace(f0, f1, 6).astype(int)):
        sc.frame_set(int(f))
        r.filepath = os.path.join(out, f"clutch_strip_{k}.png")
        bpy.ops.render.render(write_still=True)
    print(f"  workbench strip -> {out}/clutch_strip_[0-5].png")


if __name__ == "__main__":
    sys.exit(main())
