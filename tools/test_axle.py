#!/usr/bin/env python3
"""Propshaft + final drive + differential verification.

Run: python3 tools/test_axle.py [--detail high|low] [--no-render] [--out DIR]

1. Build the axle alone (both cutaway variants) at low and high detail: build time,
   triangle counts.
2. Drive it with a test Program: 15 km/h in 1st at 1/20 slow motion going straight, then
   a left turn eased in to R = 5 m (curvature 0.2) at 10 km/h and 1/8 slow motion.
   Checks (numbers printed):
   * read-back of the baked keys vs the Track: pinion = theta_out, case = theta_case,
     pinion/case rate = 4.10, case = mean of the side gears, spiders still relative to
     the case when straight and = -1.6 * (theta_RL - theta_case) in the turn, tube =
     kin.hooke(theta_out, beta), both flange yokes = theta_out (pinion follows the gearbox
     output exactly);
   * Hooke joints: every cross arm parallel to the axis of the yoke ears that hold it and
     perpendicular to the shaft of the other yoke, at every sampled frame;
   * final drive direction: the material points of pinion and ring at the pitch point
     have the same velocity (rolling contact), and that point (front of the ring) moves
     DOWN while the car moves forward (ring on the -X side, FACTS FD-04); the stub flange
     tops move forward (+Y): wheels roll forward in a forward gear;
   * collisions (BVH) of every moving pair (gear meshes, gears vs case / washers / pin,
     case + ring vs housing, bearings, U-joint crosses vs yokes ...) at >= 24 frames over
     full pinion turns going straight and >= 24 frames in the turn;
   * negative controls: spiders baked with the opposite spin sign and the pinion half a
     tooth out of phase MUST collide (proves the checker is sensitive);
   * exploded view (factor 1): exploded parts do not overlap each other.
3. Renders (Cycles 640x360, 16 spp, OIDN, studio 'dark') of overall / diff exterior /
   half cutaway / exploded / Hooke joint views and a 6-frame Workbench strip of the turn
   (half cutaway, top view) into --out.
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

from carviz import collide, kin, state  # noqa: E402
from carviz import spec as S  # noqa: E402
from carviz.assemblies import axle as AX  # noqa: E402

FAILS = []


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


def test_program():
    """0-5 s straight 15 km/h in 1st (1/20 slow); 5-7 s ease into a left R = 5 m turn at
    10 km/h (slowmo -> 1/8); 7-16 s steady turn."""
    P = state.Program(None, duration=16.0)
    P.start_in_gear(1)
    P.slowmo.key(0.0, 1 / 20, "step").key(5.0, 1 / 20, "linear").key(7.0, 1 / 8, "ease")
    P.speed_kmh.key(0.0, 15.0, "step").key(5.0, 15.0, "linear").key(7.0, 10.0, "ease")
    P.curvature.key(0.0, 0.0, "step").key(5.0, 0.0, "linear").key(7.0, 0.2, "ease")
    P.throttle_rpm.key(0.0, S.engine_rpm_at(15, 1), "step").key(5.0, S.engine_rpm_at(15, 1), "linear")
    P.throttle_rpm.key(7.0, S.engine_rpm_at(10, 1), "ease")
    return P.run()


def readback(ob, frames, index=1):
    sc = bpy.context.scene
    out = []
    for f in frames:
        sc.frame_set(int(f))
        out.append(ob.rotation_euler[index])
    return np.array(out)


def angle_between(a, b, sign_free=True):
    """Angle (deg) via atan2(|a x b|, a.b): exact near 0 even with float32 matrices."""
    a, b = Vector(a).normalized(), Vector(b).normalized()
    d = a.dot(b)
    if sign_free:
        d = abs(d)
    return math.degrees(math.atan2(a.cross(b).length, d))


def frames_over(T, arr, lo, hi, n):
    """n frames between video times lo..hi spread uniformly in the quantity arr."""
    i0, i1 = T.idx(lo), T.idx(hi)
    a = np.asarray(arr)[i0:i1 + 1]
    targets = np.linspace(a[0], a[-1], n, endpoint=False)
    out = sorted({int(i0 + np.argmin(np.abs(a - t))) + 1 for t in targets})
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--detail", default="high")
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--out", default=os.path.join(ROOT, "out", "test_axle"))
    ap.add_argument("--views", default=None, help="comma list of views to render (default all + strip)")
    ap.add_argument("--render-only", action="store_true", help="skip the checks (look-dev iterations)")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    if args.render_only:
        fresh()
        A = AX.build({"cutaways": ["none", "half"], "detail": args.detail})
        T = test_program()
        bpy.context.scene.frame_start, bpy.context.scene.frame_end = 1, T.n
        render_all(A, T, args.out, args.views)
        return

    # ------------------------------------------------------------------ build
    print("build:")
    for det in ("low", args.detail):
        fresh()
        t = time.time()
        A = AX.build({"cutaways": ["none", "half"], "detail": det})
        bt = time.time() - t
        allm = [o for o in A.parts.values() if o.type == "MESH"]
        whole = [o for k, o in A.parts.items() if "__" not in k and o.type == "MESH"]
        print(f"  detail={det}: build {bt:.1f} s, {len(A.parts)} parts, tris all variants {tris(allm):,}, "
              f"whole assembly {tris(whole):,}")
        check(bt < 60.0, f"build time {bt:.1f} s < 60 s ({det})")
        check(tris(allm) < 1_500_000, f"triangles < 1.5 M with every variant ({det})")
        bad = [o.name for o in allm if not o.name.startswith(AX.PREFIX)]
        check(not bad and all(o.name in A.root.users_collection[0].objects for o in A.objects()),
              f"every object prefixed {AX.PREFIX} and in collection 'axle' {bad[:3]}")
        nomat = [o.name for o in allm if len(o.data.materials) == 0 or any(m is None for m in o.data.materials)]
        noprop = [o.name for o in allm if "cv_opacity" not in o or "cv_glow" not in o]
        check(not nomat and not noprop, f"materials + presentation props on every mesh {nomat[:3]} {noprop[:3]}")
    fresh()
    A = AX.build({"cutaways": ["none", "half"], "detail": args.detail})
    P = A.parts
    print(f"  propshaft joint angle beta = {A.meta['prop']['beta_deg']:.3f} deg (joint centres "
          f"{A.meta['prop']['J1']} / {A.meta['prop']['J2']}), shaft {A.meta['prop']['length']:.3f} m")
    root_w = A.root.matrix_world

    # ------------------------------------------------------------------ geometry facts
    print("geometry:")
    sc = bpy.context.scene
    sc.frame_set(1)
    dg = bpy.context.evaluated_depsgraph_get()
    ring_c = root_w.inverted() @ P["ring_gear"].matrix_world.translation
    check(ring_c.x < -0.02 and abs(ring_c.y) < 1e-9 and abs(ring_c.z) < 1e-9,
          f"ring gear on the LEFT (-X) of the pinion axis, on the axle line: centre x = {ring_c.x * 1e3:.1f} mm")
    pin_ax = (root_w.inverted() @ P["pinion"].matrix_world).col[1].xyz
    check(abs(pin_ax.x) < 1e-9 and abs(pin_ax.z) < 1e-9 and abs((root_w.inverted() @ P["pinion"].matrix_world).translation.z)
          < 1e-9 and abs((root_w.inverted() @ P["pinion"].matrix_world).translation.x) < 1e-9,
          "pinion axis along Y through the ring axis (no hypoid offset)")
    fl = P["prop_flange_front"]
    me = fl.data
    co = np.array([fl.matrix_world @ v.co for v in me.vertices])
    check(abs(co[:, 1].max() - S.Y_GEARBOX_REAR) < 1e-4,
          f"front flange yoke face at the gearbox flange: y = {co[:, 1].max():.4f} (spec {S.Y_GEARBOX_REAR})")
    co = np.array([P["prop_flange_rear"].matrix_world @ v.co for v in P["prop_flange_rear"].data.vertices
                   if (P["prop_flange_rear"].matrix_world @ v.co).length > 0])
    y_rear_face = min(c[1] for c in co if math.hypot(c[0], c[2] - S.Z_PINION) > 0.045)
    check(abs(y_rear_face - S.Y_PINION_FLANGE) < 1e-4,
          f"rear flange yoke face on the pinion flange: y = {y_rear_face:.4f} (spec {S.Y_PINION_FLANGE})")
    st = [o for k, o in P.items() if k.startswith("stub_")]
    xs = [max(abs((o.matrix_world @ v.co).x) for v in o.data.vertices) for o in st]
    check(all(abs(x - S.X_DIFF_OUTPUT) < 1e-4 for x in xs),
          f"output stub flanges end at x = +-X_DIFF_OUTPUT ({xs[0]:.4f}, {xs[1]:.4f})")
    xm = max(abs((o.matrix_world @ v.co).x) for o in A.meshes() if not o.name.startswith(AX.PREFIX + "stub")
             and not o.name.startswith(AX.PREFIX + "cover") and "bush" not in o.name for v in o.data.vertices)
    print(f"  widest part other than stubs/cover ears: |x| = {xm:.3f} m; cover ears |x| <= "
          f"{max(abs((P['bushings'].matrix_world @ v.co).x) for v in P['bushings'].data.vertices):.3f} m")
    print(f"  diff case: inner sphere R {A.meta['diff']['case_inner_R'] * 1e3:.2f} mm, outer "
          f"{A.meta['diff']['case_outer_R'] * 1e3:.2f} mm; gear backs {A.meta['diff']['sphere_back'][0] * 1e3:.2f} / "
          f"{A.meta['diff']['sphere_back'][1] * 1e3:.2f} mm")

    # ------------------------------------------------------------------ drive
    T = test_program()
    check(not T.violations, f"test program integrates cleanly {T.violations[:2]}")
    sc.frame_start, sc.frame_end = 1, T.n
    A.drive(T, {"variant": "none"})
    th_out, th_case = np.asarray(T.theta_out), np.asarray(T.theta_case)
    RL, RR = np.asarray(T.theta_RL), np.asarray(T.theta_RR)
    i_s = T.idx(5.0)
    i_t = T.idx(7.0)
    print(f"  program: {T.n} frames; straight: pinion {(th_out[i_s] - th_out[0]) / kin.TAU:.2f} turns, case "
          f"{(th_case[i_s] - th_case[0]) / kin.TAU:.2f}; turn: case {(th_case[-1] - th_case[i_t]) / kin.TAU:.2f} turns,"
          f" spider {abs(kin.spider_spin(RL[-1], RR[-1]) - kin.spider_spin(RL[i_t], RR[i_t])) / kin.TAU:.2f} turns")
    print(f"  rpm (physical) straight: out {T.rpm_out[2]:.1f}, case {T.rpm_case[2]:.1f}, L/R {T.rpm_RL[2]:.1f}/"
          f"{T.rpm_RR[2]:.1f}; turn: case {T.rpm_case[-1]:.1f}, inner L {T.rpm_RL[-1]:.1f}, outer R {T.rpm_RR[-1]:.1f}"
          f", spider {S.Z_SIDE_GEAR / S.Z_SPIDER * (T.rpm_RR[-1] - T.rpm_RL[-1]) / 2:.1f}")

    # ------------------------------------------------------------------ read-back
    print("read-back vs Track:")
    fr = list(range(1, T.n + 1, 3))
    idx = np.array(fr) - 1
    pin = -readback(P["pinion"], fr)
    case = -readback(P["case_left"], fr)
    case_r = -readback(P["case_right"], fr)
    ring = -readback(P["ring_gear"], fr) - AX.PH_RING
    left = -readback(P["side_gear_left"], fr) - AX.PH_SIDE
    right = -(-readback(P["side_gear_right"], fr) - AX.PH_SIDE)
    stub_l = -readback(P["stub_left"], fr) - AX.PH_SIDE
    sp1 = -readback(P["spider_1"], fr)
    sp2 = -readback(P["spider_2"], fr)
    tube = -readback(P["prop_tube"], fr)
    ffront = -readback(P["prop_flange_front"], fr)
    frear = -readback(P["prop_flange_rear"], fr)
    tol = 2e-4
    e = lambda a, b: float(np.max(np.abs(a - b)))  # noqa: E731
    check(e(pin, th_out[idx]) < tol, f"pinion = theta_out (max err {e(pin, th_out[idx]):.1e} rad)")
    check(e(case, th_case[idx]) < tol and e(case_r, th_case[idx]) < tol and e(ring, th_case[idx]) < tol,
          f"case halves + ring = theta_case (max err {max(e(case, th_case[idx]), e(ring, th_case[idx])):.1e} rad)")
    d_pin = pin[-1] - pin[0]
    d_case = case[-1] - case[0]
    check(abs(d_pin / d_case - S.FINAL_DRIVE) < 1e-4, f"pinion / case rate = {d_pin / d_case:.5f} (4.10)")
    check(e(left, RL[idx]) < tol and e(right, RR[idx]) < tol and e(stub_l, RL[idx]) < tol,
          f"side gears + stubs = theta_RL / theta_RR (max err {max(e(left, RL[idx]), e(right, RR[idx])):.1e})")
    check(e(case, 0.5 * (left + right)) < tol, f"case = mean of the side gears (max err "
                                                f"{e(case, 0.5 * (left + right)):.1e} rad)")
    k_s = idx <= i_s
    k_t = idx >= i_t
    check(float(np.max(np.abs(sp1[k_s]))) < 1e-6 and float(np.max(np.abs(sp2[k_s]))) < 1e-6,
          f"straight: spiders still relative to the case (max {float(np.max(np.abs(sp1[k_s]))):.1e} rad)")
    want = -S.Z_SIDE_GEAR / S.Z_SPIDER * (RL[idx] - th_case[idx])
    check(e(sp1, want) < tol and e(sp2, want) < tol and float(np.max(np.abs(sp1[k_t]))) > 0.5,
          f"turn: spiders spin = -1.6 x (theta_RL - theta_case) (max err {e(sp1, want):.1e}, reached "
          f"{float(np.max(np.abs(sp1))):.2f} rad)")
    check(e(tube, kin.hooke(th_out[idx], AX.PROP_BETA)) < tol, "propshaft tube = kin.hooke(theta_out, beta)")
    check(e(ffront, th_out[idx]) < tol and e(frear, th_out[idx]) < tol,
          "gearbox-side and pinion-side flange yokes both = theta_out (pinion follows the gearbox exactly)")
    dev = float(np.max(np.abs(kin.hooke(th_out, AX.PROP_BETA) - th_out)))
    print(f"  tube angle deviation from the flanges: max {math.degrees(dev):.4f} deg (beta^2/4 = "
          f"{math.degrees(AX.PROP_BETA) ** 2 / 4 * math.pi / 180:.4f} deg)")

    # ------------------------------------------------------------------ Hooke joint geometry
    print("Hooke joints:")
    worst = dict(f_in=0.0, f_out=0.0, f_perp=0.0, r_in=0.0, r_out=0.0, r_perp=0.0)
    for f in frames_over(T, th_out, 0.0, 5.0, 36):
        sc.frame_set(f)
        Mf, Mc, Ms = (P[k].matrix_world for k in ("prop_flange_front", "prop_cross_front", "prop_slip_yoke"))
        worst["f_in"] = max(worst["f_in"], angle_between(Mf.col[2].xyz, Mc.col[2].xyz))
        worst["f_out"] = max(worst["f_out"], angle_between(Mc.col[0].xyz, Ms.col[0].xyz))
        worst["f_perp"] = max(worst["f_perp"], abs(90.0 - angle_between(Mc.col[0].xyz, Ms.col[1].xyz, False)))
        Mr, Mc2, Mt = (P[k].matrix_world for k in ("prop_flange_rear", "prop_cross_rear", "prop_tube_yoke"))
        worst["r_in"] = max(worst["r_in"], angle_between(Mr.col[2].xyz, Mc2.col[2].xyz))
        worst["r_out"] = max(worst["r_out"], angle_between(Mc2.col[0].xyz, Mt.col[0].xyz))
        worst["r_perp"] = max(worst["r_perp"], abs(90.0 - angle_between(Mc2.col[0].xyz, Mt.col[1].xyz, False)))
    check(max(worst.values()) < 1e-3, "every cross arm stays in its yoke ears (36 frames): " +
          ", ".join(f"{k} {v:.1e} deg" for k, v in worst.items()))

    # ------------------------------------------------------------------ final drive direction
    print("final drive direction:")
    dlt = AX._FD["deltaA"]
    Rm = AX._FD["apexA"] / math.cos(dlt) - 0.5 * P["ring_gear"]["bevel_face_width"] if "bevel_face_width" in \
        P["ring_gear"] else 0.0815
    pp_local = Vector((-Rm * math.sin(dlt), Rm * math.cos(dlt), 0.0))
    errs, vz, vstub = [], [], []
    top = Vector((-0.144, 0.0, 0.040))
    for f in frames_over(T, th_out, 0.2, 5.0, 8):
        # central difference over +-0.25 frame: the chord of a rotation through symmetric
        # positions is exactly tangent, so pinion and ring (different axes) compare exactly
        sc.frame_set(f)
        Pw = root_w @ pp_local
        St = root_w @ top
        lp = P["pinion"].matrix_world.inverted() @ Pw
        lr = P["ring_gear"].matrix_world.inverted() @ Pw
        ls = P["stub_left"].matrix_world.inverted() @ St
        sc.frame_set(f, subframe=0.25)
        p1, r1, s1 = (P[k].matrix_world @ l for k, l in (("pinion", lp), ("ring_gear", lr), ("stub_left", ls)))
        sc.frame_set(f - 1, subframe=0.75)
        p0, r0, s0 = (P[k].matrix_world @ l for k, l in (("pinion", lp), ("ring_gear", lr), ("stub_left", ls)))
        vp, vr, vs = p1 - p0, r1 - r0, s1 - s0
        errs.append((vp - vr).length / vp.length)
        vz.append(vp.normalized().z)
        vstub.append(vs.normalized().y)
    check(max(errs) < 2e-3, f"pinion and ring pitch-point velocities equal (rolling contact): rel err "
                            f"{max(errs):.1e}")
    check(max(vz) < -0.99, f"pitch point at the front of the ring moves DOWN with the car going forward "
                           f"(v_z/|v| = {max(vz):.3f}); ring on -X gives forward rolling (FD-04)")
    check(min(vstub) > 0.99, f"top of the output-stub flanges moves forward (+Y): wheels roll forward "
                             f"({min(vstub):.3f})")

    # ------------------------------------------------------------------ collisions
    print("collisions:")
    pairs = []

    def add(a, b):
        if a in P and b in P:
            pairs.append((P[a], P[b]))
    add("pinion", "ring_gear")
    for k in ("spider_1", "spider_2"):
        for s in ("side_gear_left", "side_gear_right", "case_left", "case_right", "cross_pin"):
            add(k, s)
        add(k, "spider_washer_" + k[-1])
    for s in ("left", "right"):
        for c in ("case_left", "case_right", "side_washer_" + s):
            add("side_gear_" + s, c)
        for c in ("case_left", "case_right"):
            add("side_washer_" + s, c)
            add("stub_" + s, c)
        add("stub_" + s, "housing")
        add("stub_" + s, "seal_" + s)
        add("carrier_rollers_" + s, "carrier_cup_" + s)
        add("carrier_rollers_" + s, "carrier_cone_" + s)
        add("carrier_cone_" + s, "carrier_cup_" + s)
    for m in ("ring_gear", "ring_bolts", "case_left", "case_right"):
        for h in ("housing", "cover", "pinion"):
            if not (m == "ring_gear" and h == "pinion"):
                add(m, h)
    for h in ("housing", "pinion_seal", "pinion_cup_head", "pinion_cup_tail"):
        add("pinion", h)
        add("companion_flange", h)
    for b in ("head", "tail"):
        add(f"pinion_rollers_{b}", f"pinion_cup_{b}")
        add(f"pinion_rollers_{b}", f"pinion_cone_{b}")
        add(f"pinion_cone_{b}", f"pinion_cup_{b}")
    for a, b in (("prop_flange_front", "prop_cross_front"), ("prop_cross_front", "prop_slip_yoke"),
                 ("prop_flange_front", "prop_slip_yoke"), ("prop_flange_rear", "prop_cross_rear"),
                 ("prop_cross_rear", "prop_tube_yoke"), ("prop_flange_rear", "prop_tube_yoke"),
                 ("prop_flange_rear", "housing"), ("prop_cross_rear", "companion_flange"),
                 ("prop_tube_yoke", "companion_flange")):
        add(a, b)
    print(f"  {len(pairs)} moving pairs")
    f_st = frames_over(T, th_out, 0.0, 5.0, 26)
    f_tu = frames_over(T, kin.spider_spin(RL, RR), 7.0, 16.0, 26)
    t0 = time.time()
    bad = collide.check_pairs(pairs, f_st)
    check(not bad, f"straight (15 km/h, {len(f_st)} frames over {(th_out[i_s] - th_out[0]) / kin.TAU:.1f} pinion "
                   f"turns): no overlaps {bad[:4]}")
    bad = collide.check_pairs(pairs, f_tu)
    check(not bad, f"R = 5 m left turn ({len(f_tu)} frames over one spider tooth+): no overlaps {bad[:4]}")
    print(f"  ({time.time() - t0:.1f} s)")
    gaps = {}
    sc.frame_set(f_tu[3])
    dg = bpy.context.evaluated_depsgraph_get()
    for a, b in (("pinion", "ring_gear"), ("spider_1", "side_gear_left"), ("spider_2", "side_gear_right"),
                 ("ring_gear", "housing"), ("case_left", "pinion"), ("ring_gear", "cover"), ("case_right", "housing")):
        gaps[f"{a}/{b}"] = collide.min_distance(P[a], P[b], samples=3000, dg=dg) * 1e3
    print("  min clearances (mm): " + ", ".join(f"{k} {v:.2f}" for k, v in gaps.items()))
    # negative controls
    fr_all = T.frames
    sp_wrong = -AX.SPIDER_SIGN * kin.spider_spin(RL, RR)
    from carviz import rig
    rig.bake_spin(P["spider_1"], fr_all, sp_wrong)
    rig.bake_spin(P["spider_2"], fr_all, sp_wrong)
    bad = collide.check_pairs([(P["spider_1"], P["side_gear_left"]), (P["spider_1"], P["side_gear_right"]),
                               (P["spider_2"], P["side_gear_left"]), (P["spider_2"], P["side_gear_right"])], f_tu)
    check(len(bad) > 0, f"negative control: spiders with the OPPOSITE spin sign collide in the turn "
                        f"({len(bad)} overlaps)")
    rig.bake_spin(P["pinion"], fr_all, th_out + math.pi / S.Z_PINION_TEETH)
    bad = collide.check_pairs([(P["pinion"], P["ring_gear"])], f_st[::4])
    check(len(bad) > 0, f"negative control: pinion half a tooth out of phase collides ({len(bad)} overlaps)")
    A.drive(T, {"variant": "none"})

    # ------------------------------------------------------------------ anchors, variants
    print("anchors / presentation:")
    sc.frame_set(1)
    dg = bpy.context.evaluated_depsgraph_get()
    far = []
    for name, (ob, off) in A.anchors.items():
        # anchors of spinning parts sit on the spin axis (labels must not orbit): accept a
        # point inside the part's bounding box or within 12 mm of its surface
        w = A.anchor_world(name)
        meshes = [ob] if ob.type == "MESH" else [c for c in ob.children_recursive if c.type == "MESH"]
        best = 1.0
        for m in meshes:
            V = np.array([m.matrix_world @ v.co for v in m.data.vertices])
            lo, hi = V.min(0) - 0.002, V.max(0) + 0.002
            if np.all(np.asarray(w) >= lo) and np.all(np.asarray(w) <= hi):
                best = 0.0
                break
            hit = collide._bvh(m, dg).find_nearest(w)
            if hit[0] is not None:
                best = min(best, hit[3])
        if best > 0.012:
            far.append((name, round(best * 1e3, 1)))
    need = {"propshaft", "ujoint_front", "ujoint_rear", "pinion", "ring_gear", "diff_case", "spider_gear",
            "side_gear_left", "side_gear_right", "cross_pin", "diff_housing"}
    check(need <= set(A.anchors) and not far, f"{len(A.anchors)} anchors incl. the brief's 11, each on its part "
                                              f"{far}")
    pg = A.meta["power_groups"]
    check(all(k in P for k in pg["prop"] + pg["diff"]) and A.meta["power_path"] == pg["prop"] + pg["diff"],
          f"power_path / power_groups name existing parts ({len(pg['prop'])} prop, {len(pg['diff'])} diff)")
    var = ["none"] * (T.n // 2) + ["half"] * (T.n - T.n // 2)
    rem = np.linspace(1.0, 0.0, T.n)
    A.drive(T, {"variant": var, "removed": rem})
    fa, fb = 5, T.n - 5
    sc.frame_set(fa)
    h0, k0, r0 = P["housing"].hide_render, P["housing__half_kept"].hide_render, P["housing__half_removed"].hide_render
    sc.frame_set(fb)
    h1, k1, r1 = P["housing"].hide_render, P["housing__half_kept"].hide_render, P["housing__half_removed"].hide_render
    sc.frame_set(T.n // 2 + 10)
    op = P["housing__half_removed"]["cv_opacity"]
    check((h0, k0, r0) == (False, True, True) and (h1, k1, r1) == (True, False, True) and abs(op - rem[T.n // 2 + 9])
          < 1e-3, f"per-frame variant switch none->half + 'removed' fade baked (opacity {op:.2f})")
    A.drive(T, {"variant": "none"})

    # ------------------------------------------------------------------ explode
    print("explode:")
    A.drive(T, {"variant": "half", "explode": np.ones(T.n)})
    ex_keys = ["case_left", "case_right", "ring_gear", "ring_bolts", "cross_pin", "spider_1", "spider_2",
               "side_gear_left", "side_gear_right", "stub_left", "stub_right", "spider_washer_1", "spider_washer_2",
               "side_washer_left", "side_washer_right", "carrier_cone_left", "carrier_cone_right",
               "carrier_rollers_left", "carrier_rollers_right"]
    ex_pairs = [(P[a], P[b]) for i, a in enumerate(ex_keys) for b in ex_keys[i + 1:]
                if not ({a, b} <= {"ring_gear", "ring_bolts"} or {a, b} <= {"case_left", "carrier_cone_left"}
                        or {a, b} <= {"case_right", "carrier_cone_right"}
                        or {a, b} <= {"carrier_cone_left", "carrier_rollers_left"}
                        or {a, b} <= {"carrier_cone_right", "carrier_rollers_right"}
                        or {a, b} <= {"spider_1", "spider_washer_1"} or {a, b} <= {"spider_2", "spider_washer_2"}
                        or {a, b} <= {"side_gear_left", "side_washer_left"}
                        or {a, b} <= {"side_gear_right", "side_washer_right"}
                        or {a, b} <= {"ring_bolts", "case_left"})]
    bad = collide.check_pairs(ex_pairs, [1, T.n // 2, T.n])
    check(not bad, f"exploded diff (factor 1): {len(ex_pairs)} part pairs free of each other {bad[:4]}")
    A.drive(T, {"variant": "none"})
    sc.frame_set(T.n)
    moved = [k for k, loc in A.meta["_rest_loc"].items() if (P[k].location - Vector(loc)).length > 1e-9]
    check(not moved, f"drive() without 'explode' returns every explode carrier to rest {moved[:3]}")

    if not args.no_render:
        render_all(A, T, args.out, args.views)

    if FAILS:
        print(f"\n{len(FAILS)} FAILURES")
        for f in FAILS:
            print("  -", f)
        sys.exit(1)
    print("\nall axle checks passed")


# ----------------------------------------------------------------------------
# renders
# ----------------------------------------------------------------------------

VIEWS = {
    # name: (variant, explode, eye, target, lens, frame, hide groups)
    "overall": ("none", 0, (-1.20, -0.70, 1.00), (0.0, -1.98, 0.33), 35, 1, ()),
    "diff": ("none", 0, (-0.55, -3.22, 0.72), (0.0, -2.62, 0.31), 40, 1, ()),
    "half": ("half", 0, (0.42, -3.00, 0.95), (-0.02, -2.56, 0.30), 40, 1, ()),
    "half_front": ("half", 0, (0.36, -2.18, 0.82), (-0.01, -2.56, 0.30), 42, 1, ()),
    "explode": ("half", 1, (0.16, -1.86, 0.98), (-0.02, -2.62, 0.35), 30, 1, ("housing", "pinion", "prop")),
    "nose": ("none", 0, (-0.34, -2.10, 0.50), (0.0, -2.37, 0.31), 45, 1, ()),
    "window": ("half", 0, (0.16, -2.90, 0.78), (0.0, -2.62, 0.32), 55, 56, ()),
    "ujoint": ("none", 0, (0.34, -0.93, 0.50), (0.0, -1.19, 0.355), 50, 1, ()),
}


def _hide(A, groups):
    P = A.parts
    keys = set()
    for g in groups:
        if g == "prop":
            keys |= {k for k in P if k.startswith("prop_")}
        elif g == "housing":
            for k in A.meta["groups"]["housing"]:
                keys |= {k, k + "__half_kept", k + "__half_removed"}
        elif g == "pinion":
            keys |= set(A.meta["groups"]["pinion"])
    for k in keys:
        ob = P.get(k)
        if ob is None:
            continue
        if ob.animation_data is not None:
            for path in ("hide_render", "hide_viewport"):
                try:
                    ob.keyframe_delete(path)
                except Exception:
                    pass
        ob.hide_render = True


def _unhide(A):
    for ob in A.objects():
        ob.hide_render = False


def render_all(A, T, out, views=None):
    from carviz import lighting
    sc = bpy.context.scene
    st = lighting.setup_studio("dark", center=(0.0, -2.3, 0.33), size=0.8)
    rig_ = st["rig"]
    lighting.setup_color_management(sc)
    cd = bpy.data.cameras.new("test_cam")
    cam = bpy.data.objects.new("test_cam", cd)
    sc.collection.objects.link(cam)
    sc.camera = cam
    cd.clip_start = 0.01
    r = sc.render
    r.resolution_x, r.resolution_y = 640, 360
    print("renders:")
    want = set(views.split(",")) if views else None
    for name, (variant, ex, eye, tgt, lens, frame, hide) in VIEWS.items():
        if want is not None and name not in want:
            continue
        _unhide(A)
        A.drive(T, {"variant": variant, "explode": np.full(T.n, float(ex))})
        _hide(A, hide)
        sc.frame_set(frame)
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
        rig_.location = tgt
        rig_.rotation_euler = (0.0, 0.0, math.radians(cam_az + 50.0 - 225.0))
        cd.lens = lens
        r.filepath = os.path.join(out, f"axle_{name}.png")
        t = time.time()
        bpy.ops.render.render(write_still=True)
        print(f"  rendered {name} ({time.time() - t:.1f} s) -> {r.filepath}")
    if want is not None and "strip" not in want:
        return
    # Workbench motion strip: half cutaway, top view, through the turn (spiders spin)
    _unhide(A)
    A.drive(T, {"variant": "half", "explode": np.zeros(T.n)})
    _hide(A, ("prop",))
    r.engine = "BLENDER_WORKBENCH"
    sh = sc.display.shading
    sh.light = "STUDIO"
    sh.color_type = "MATERIAL"
    sh.show_cavity = True
    sc.display.render_aa = "8"
    eye, tgt = (0.0, -2.60, 0.95), (0.0, -2.60, 0.30)
    cam.location = eye
    cam.rotation_euler = (Vector(tgt) - Vector(eye)).to_track_quat("-Z", "Y").to_euler()
    cd.lens = 45
    for k, f in enumerate(np.linspace(T.frame_at(7.0), T.n, 6).astype(int)):
        sc.frame_set(int(f))
        r.filepath = os.path.join(out, f"axle_strip_{k}.png")
        bpy.ops.render.render(write_still=True)
    print(f"  workbench strip -> {out}/axle_strip_[0-5].png")


if __name__ == "__main__":
    main()
