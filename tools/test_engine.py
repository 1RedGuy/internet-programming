#!/usr/bin/env python3
"""Engine assembly verification.  Run: python3 tools/test_engine.py [--detail high|low]
                                         [--no-render] [--out DIR] [--frames-checked N]

1. Build the engine alone (all cutaway variants) and report build time and triangles
   (high and low detail).
2. Drive it with an idle Program (850 rpm, slowed so one full 720-deg cycle spans the
   clip) and check numerically:
   * collisions (carviz.collide) between every moving part and its housings /
     neighbours at >= 24 frames spread over a full cycle
   * baked rotation / translation read back against kin (crank = theta, cam = theta/2,
     pistons = kin.piston_height, rods = kin.slider_crank, chain travel, sprockets)
   * timing chain: every roller on a sprocket arc sits in a tooth gap (radial and
     angular error), i.e. chain speed = sprocket pitch-line speed at every frame
   * valve-to-piston clearance (min over the cycle, incl. overlap TDC), valve heads seat
     flush when closed, cam lobe rides on its bucket (gap = lash) incl. peak lift,
     cam turns once per two crank turns
   * combustion: spark peaks at SPARK_ADVANCE before TDC, firing order 1-3-4-2, gas
     volume bottom follows the piston crown
   * flat-tappet lift law vs kin.valve_lift: same events, same peak lift
3. Renders (Cycles 640x360, 16 spp, OIDN, studio 'dark') of overall / long / cyl1 /
   front / flywheel-exploded views and a 6-frame Workbench motion strip into --out.
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
from carviz import spec as S  # noqa: E402
from carviz.assemblies import engine as E  # noqa: E402

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


def idle_program(duration=5.0, rpm=S.IDLE_RPM, slow=1.0 / 30.0):
    P = state.Program(None, duration=duration)
    P.slowmo.key(0, slow, "step")
    P.throttle_rpm.key(0, rpm, "step")
    P.align_engine(0.0, 0.0)
    return P.run()


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
    return collide._bvh(ob, dg)


def nearest_gap(verts, bvh):
    best = 1e9
    for v in verts:
        hit = bvh.find_nearest(Vector(v))
        if hit[0] is not None:
            best = min(best, hit[3])
    return best


def sample_frames(T, n):
    """Frames spread uniformly over the first full 720-deg engine cycle."""
    th = T.theta_e - T.theta_e[0]
    targets = np.linspace(0.0, 4 * math.pi, n, endpoint=False)
    out = sorted({int(np.argmin(np.abs(th - t))) + 1 for t in targets})
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--detail", default="high")
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--out", default=os.path.join(ROOT, "out", "test_engine"))
    ap.add_argument("--frames-checked", type=int, default=28)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    print("build:")
    for det in ("low", args.detail):
        fresh()
        t = time.time()
        A = E.build({"cutaways": ["none", "long", "cyl1", "front"], "detail": det})
        bt = time.time() - t
        whole = [ob for k, ob in A.parts.items() if "__" not in k and ob.type == "MESH"]
        print(f"  detail={det}: build {bt:.1f} s, {len(A.parts)} parts, tris all variants {tris(A.parts.values()):,}"
              f", whole engine {tris(whole):,}")
        check(bt < 60.0, f"build time {bt:.1f} s < 60 s ({det})")
        check(tris(A.parts.values()) < 1_500_000, f"triangles < 1.5 M with every variant ({det})")
    fresh()
    A = E.build({"cutaways": ["none", "long", "cyl1", "front"], "detail": args.detail})
    P = A.parts
    T = idle_program()
    check(not T.violations, f"idle program integrates cleanly {T.violations[:2]}")
    sc = bpy.context.scene
    sc.frame_start, sc.frame_end = 1, T.n
    A.drive(T, {"variant": "none"})
    th = np.asarray(T.theta_e)
    cyc = (th[-1] - th[0]) / (4 * math.pi)
    print(f"  program: {T.n} frames, crank turns {(th[-1] - th[0]) / (2 * math.pi):.2f} ({cyc:.2f} engine cycles)")
    check(cyc >= 1.0, "test covers a full 720-deg cycle")

    # ------------------------------------------------------------------ read-back
    print("motion read-back vs kin:")
    root_z = A.root.location.z
    errs = dict(crank=0.0, cam=0.0, piston=0.0, rod=0.0, valve=0.0, sprocket=0.0, chain=0.0)
    chs = A.meta["chain"]
    for f in (1, T.n // 3, T.n // 2, T.n):
        sc.frame_set(f)
        t = th[f - 1]
        errs["crank"] = max(errs["crank"], abs(P["crankshaft"].rotation_euler[1] + t),
                            abs(P["flywheel"].rotation_euler[1] + t))
        errs["cam"] = max(errs["cam"], abs(P["cam_intake"].rotation_euler[1] + kin.cam_angle(t)),
                          abs(P["cam_exhaust"].rotation_euler[1] + kin.cam_angle(t)))
        errs["sprocket"] = max(errs["sprocket"],
                               abs(P["crank_sprocket"].rotation_euler[1] + t + chs["phases"]["crank"]),
                               abs(P["cam_sprocket_intake"].rotation_euler[1] + t / 2 + chs["phases"]["intake"]))
        for c in range(1, 5):
            errs["piston"] = max(errs["piston"], abs(P[f"piston{c}"].location.z - kin.piston_height(t, c)))
            _, _, cx, cz, rt = kin.slider_crank(t, c)
            ob = P[f"conrod{c}"]
            errs["rod"] = max(errs["rod"], abs(ob.location.x - cx), abs(ob.location.z - cz),
                              abs(ob.rotation_euler[1] + rt))
            for kind in ("intake", "exhaust"):
                v = P[f"valve_{c}{kind[0]}0"]
                errs["valve"] = max(errs["valve"], abs(v.location.y + E.valve_lift_ft(t, c, kind)))
        errs["chain"] = max(errs["chain"], abs(P["timing_chain"].location.x -
                                               np.mod(E.chain_travel(t), 2 * S.CHAIN_PITCH)))
    for k, v in errs.items():
        check(v < 2e-5, f"{k}: baked value = kin value (max err {v:.2e})")
    sc.frame_set(1)
    a0 = -P["crankshaft"].rotation_euler[1]
    c0 = -P["cam_intake"].rotation_euler[1]
    sc.frame_set(T.n)
    a1 = -P["crankshaft"].rotation_euler[1]
    c1 = -P["cam_intake"].rotation_euler[1]
    check(a1 > a0, "crank angle increases (clockwise seen from the front, ARCHITECTURE sec. 2)")
    check(abs((c1 - c0) / (a1 - a0) - 0.5) < 1e-6, f"cam turns once per two crank turns (ratio {(c1 - c0) / (a1 - a0):.6f})")
    w_e = (a1 - a0) / ((T.n - 1) / T.fps)
    check(abs(w_e - T.w_e[0] * T.slowmo[0]) / w_e < 0.01,
          f"crank speed {w_e:.3f} rad/s video = {S.rad_s_to_rpm(T.w_e[0]):.0f} rpm x slow-mo")

    # ------------------------------------------------------------------ lift law
    print("valve lift law (flat-tappet cam) vs kin.valve_lift:")
    tt = np.radians(np.linspace(0, 720, 14401))
    for kind in ("intake", "exhaust"):
        a = E.valve_lift_ft(tt, 1, kind)
        b = kin.valve_lift(tt, 1, kind)
        oa = np.degrees(tt[a > 1e-9][[0, -1]])
        ob_ = np.degrees(tt[b > 1e-9][[0, -1]])
        check(np.all(np.abs(oa - ob_) < 0.2) and abs(a.max() - b.max()) < 1e-6,
              f"{kind}: opens {oa[0]:.1f} closes {oa[1]:.1f} deg (kin {ob_[0]:.1f}/{ob_[1]:.1f}), "
              f"peak {a.max() * 1e3:.2f} mm; max shape difference {np.abs(a - b).max() * 1e3:.2f} mm")
        bs = np.linspace(-1.3, 1.3, 20001)
        h = E.cam_support(bs, kind)
        exc = np.abs(np.gradient(h, bs)).max()
        check(exc < E.BUCKET_R[kind] - 0.5e-3,
              f"{kind}: cam/bucket contact stays {exc * 1e3:.1f} mm from the bucket centre < radius "
              f"{E.BUCKET_R[kind] * 1e3:.1f} mm")
    # cam profile curvature (convex, positive nose radius)
    g = E.cam_geometry("intake")
    check(g["rn"] > 0 and g["rf"] > g["Rb"], f"three-arc lobe: nose R {g['rn'] * 1e3:.1f} mm, flank R {g['rf'] * 1e3:.1f} mm")

    # ------------------------------------------------------------------ collisions
    frames = sample_frames(T, args.frames_checked)
    print(f"collisions at {len(frames)} frames over one 720-deg cycle:")
    pairs = []

    def add(a, b):
        if a in P and b in P:
            pairs.append((P[a], P[b]))
    for c in range(1, 5):
        for h in ("block", "head", "head_gasket", "crankshaft", "spark_plug%d" % c):
            add(f"piston{c}", h)
        for h in ("block", "crankshaft", f"piston{c}", "main_caps", "main_bolts", "oil_pan"):
            add(f"conrod{c}", h)
        for kind in ("i", "e"):
            for i in (0, 1):
                v, s = f"valve_{c}{kind}{i}", f"spring_{c}{kind}{i}"
                for h in ("head", "valve_seats", "valve_guides", f"piston{c}", f"cam_{'intake' if kind == 'i' else 'exhaust'}",
                          s, "cam_caps", "spark_plug%d" % c):
                    add(v, h)
                for h in ("head", "valve_guides"):
                    add(s, h)
    for h in ("block", "main_caps", "main_shells", "oil_pan", "timing_cover", "crank_sprocket"):
        add("crankshaft", h)
    for cam in ("cam_intake", "cam_exhaust"):
        for h in ("head", "cam_caps", "cam_cover", "timing_cover"):
            add(cam, h)
    for h in ("crank_sprocket", "cam_sprocket_intake", "cam_sprocket_exhaust", "chain_guide", "tensioner_arm",
              "tensioner", "block", "head", "timing_cover", "crankshaft", "cam_intake", "cam_exhaust", "damper"):
        add("timing_chain", h)
    add("cam_sprocket_intake", "cam_sprocket_exhaust")
    for sp in ("cam_sprocket_intake", "cam_sprocket_exhaust", "crank_sprocket"):
        for h in ("head", "block", "timing_cover", "chain_guide", "tensioner_arm", "tensioner"):
            add(sp, h)
    for h in ("timing_cover", "block", "accessory_belt", "water_pump"):
        add("damper", h)
    for pul in ("wp_pulley", "belt_idler", "alt_pulley"):
        for h in ("accessory_belt", "timing_cover", "water_pump", "idler_arm", "alternator", "block", "damper",
                  "intake_manifold", "oil_pan"):
            add(pul, h)
    for h in ("block", "oil_pan"):
        add("flywheel", h)
        add("ring_gear", h)
    print(f"  {len(pairs)} pairs")
    t = time.time()
    bad = collide.check_pairs(pairs, frames)
    print(f"  checked in {time.time() - t:.1f} s")
    seen = {}
    for f, a, b, n in bad:
        seen.setdefault((a, b), []).append((f, n))
    for (a, b), lst in sorted(seen.items()):
        print(f"    overlap {a} x {b}: frames {[x[0] for x in lst][:8]} (max {max(x[1] for x in lst)} face pairs)")
    check(not bad, f"no interference between moving parts and housings/neighbours ({len(seen)} pairs overlap)")

    # ------------------------------------------------------------------ chain on sprockets
    print("timing chain engagement:")
    ch = P["timing_chain"]
    lay = A.meta["chain"]["roller_layout"]
    nv, r0a, r0b, r1a, r1b = lay
    N = A.meta["chain"]["n_links"]
    spro = [("crank_sprocket", (0.0, 0.0), S.CRANK_SPROCKET_TEETH, E._r_eff(S.CRANK_SPROCKET_TEETH)),
            ("cam_sprocket_intake", (-E.X_CAM, E.Z_CAM), S.CAM_SPROCKET_TEETH, E._r_eff(S.CAM_SPROCKET_TEETH)),
            ("cam_sprocket_exhaust", (E.X_CAM, E.Z_CAM), S.CAM_SPROCKET_TEETH, E._r_eff(S.CAM_SPROCKET_TEETH))]
    worst_ang, worst_rad, n_eng = 0.0, 0.0, 0
    pitch_err = 0.0
    for f in frames:
        sc.frame_set(f)
        dg = bpy.context.evaluated_depsgraph_get()
        V = world_verts(ch, dg)
        V[:, 2] -= root_z
        cen = []
        for m in range(N // 2):
            b = m * nv
            cen.append(V[b + r0a:b + r0b].mean(axis=0))
            cen.append(V[b + r1a:b + r1b].mean(axis=0))
        cen = np.array(cen)
        d = np.linalg.norm(np.diff(np.vstack([cen, cen[:1]]), axis=0), axis=1)
        pitch_err = max(pitch_err, float(np.max(np.abs(d - S.CHAIN_PITCH))))
        for name, cc, z, R in spro:
            ang = -P[name].rotation_euler[1]
            dx, dz = cen[:, 0] - cc[0], cen[:, 2] - cc[1]
            r = np.hypot(dx, dz)
            on = np.abs(r - R) < 0.00015
            tau = 2 * math.pi / z
            psi = np.arctan2(dz[on], dx[on])
            e = (psi - ang - tau / 2 + tau / 2) % tau - tau / 2
            if np.any(on):
                n_eng += int(on.sum())
                worst_ang = max(worst_ang, float(np.max(np.abs(e)) * R))
                worst_rad = max(worst_rad, float(np.max(np.abs(r[on] - R))))
    check(n_eng > 30 * len(frames) and worst_ang < 0.06e-3,
          f"{n_eng} roller-on-sprocket samples: rollers centred in tooth gaps (max {worst_ang * 1e3:.3f} mm off "
          f"gap centre, radial {worst_rad * 1e3:.3f} mm)")
    check(pitch_err < 0.08e-3, f"roller spacing along the deformed chain = pitch (max err {pitch_err * 1e3:.3f} mm)")

    # ------------------------------------------------------------------ valvetrain invariants
    print("valvetrain:")
    gaps, seat_gap, min_vp = [], 1e9, 1e9
    pk_frames = {}
    for c in (1, 3):
        for kind in ("intake", "exhaust"):
            lift = E.valve_lift_ft(th, c, kind)
            pk_frames[(c, kind)] = int(np.argmax(lift)) + 1
    for f in frames + list(pk_frames.values()):
        sc.frame_set(f)
        dg = bpy.context.evaluated_depsgraph_get()
        for c in (1, 3):
            for kind in ("intake", "exhaust"):
                vob = P[f"valve_{c}{kind[0]}0"]
                cam = P[f"cam_{kind}"]
                F, a, C = E.valve_frame(c, kind, 0)
                C = C + np.array([0.0, 0.0, root_z])
                u = -a
                V = world_verts(cam, dg)
                sel = np.abs(V[:, 1] - F[1]) < E.CAM_LOBE_W / 2 - 0.0003
                ext = float(np.max((V[sel] - C) @ u))
                lift = -vob.location.y
                gaps.append(E.CAM_BASE_R + lift - ext)
    check(min(gaps) > 0.0 and max(gaps) < E.CAM_LASH + 0.04e-3,
          f"cam lobe rides on its bucket at every sampled frame incl. peak lift: gap {min(gaps) * 1e3:.3f}.."
          f"{max(gaps) * 1e3:.3f} mm (lash {E.CAM_LASH * 1e3:.2f} mm)")
    # seating: frames with zero lift
    for c in (1, 2):
        lift = E.valve_lift_ft(th, c, "intake")
        f = int(np.argmin(lift + (np.arange(len(lift)) < 2) * 1.0)) + 1
        sc.frame_set(f)
        dg = bpy.context.evaluated_depsgraph_get()
        v = P[f"valve_{c}i0"]
        check(abs(v.location.y) < 1e-9, f"cyl {c} intake valve closed at frame {f}: face flush with the chamber roof")
        seat_gap = min(seat_gap, collide.min_distance(v, P["valve_seats"], samples=4000, dg=dg))
    check(0.0 < seat_gap < 0.1e-3, f"closed valve face sits on its seat insert (gap {seat_gap * 1e3:.3f} mm)")
    # valve-to-piston clearance around both TDCs of every cylinder (fine frames)
    rev_s = 60.0 / S.IDLE_RPM
    T2 = idle_program(duration=30.0, slow=2 * rev_s / 30.0)          # 720 deg in 720 frames (1 deg/frame)
    A.drive(T2, {"variant": "none"})
    th2 = np.asarray(T2.theta_e)
    fr2 = []
    for c in range(1, 5):
        for tdc in (360.0, 0.0):
            cy = kin.cycle_angle_deg(th2, c)
            d = (cy - tdc + 360.0) % 720.0 - 360.0
            fr2 += [int(i) + 1 for i in np.nonzero(np.abs(d) < 40.0)[0][::2]]
    fr2 = sorted(set(fr2))
    for f in fr2:
        sc.frame_set(f)
        dg = bpy.context.evaluated_depsgraph_get()
        for c in range(1, 5):
            pb = bvh_of(P[f"piston{c}"], dg)
            for kind in ("i", "e"):
                for i in (0, 1):
                    vob = P[f"valve_{c}{kind}{i}"]
                    me = vob.data
                    co = np.zeros(len(me.vertices) * 3)
                    me.vertices.foreach_get("co", co)
                    co = co.reshape(-1, 3)
                    V = world_verts(vob, dg)[co[:, 1] < 0.004]
                    min_vp = min(min_vp, nearest_gap(V, pb))
    check(min_vp > 0.5e-3, f"valve-to-piston clearance around TDC (overlap + firing), {len(fr2)} frames: "
          f"min {min_vp * 1e3:.2f} mm (valve reliefs in the crown)")
    bad2 = collide.check_pairs([(P[f"piston{c}"], P[f"valve_{c}{k}{i}"]) for c in range(1, 5) for k in "ie"
                                for i in (0, 1)], fr2[::3])
    check(not bad2, f"no valve/piston overlap in {len(fr2[::3])} near-TDC frames")
    A.drive(T, {"variant": "none"})
    # ------------------------------------------------------------------ combustion
    print("combustion:")
    peaks = {}
    for c in range(1, 5):
        sp = P[f"spark{c}"]
        vals = []
        for f in range(1, T.n + 1):
            sc.frame_set(f)
            vals.append(sp["cv_opacity"])
        vals = np.array(vals)
        i = int(np.argmax(vals))
        cy = float(kin.cycle_angle_deg(th[i], c))
        peaks[c] = th[i]
        dev = (cy - (720 - S.SPARK_ADVANCE_DEG) + 360) % 720 - 360
        step = math.degrees(th[1] - th[0])
        check(vals.max() > 0.3 and abs(dev) <= step, f"cyl {c}: spark flash peaks at cycle {cy:.1f} deg "
              f"(target {720 - S.SPARK_ADVANCE_DEG:.0f} +- {step:.1f})")
    order = tuple(sorted(peaks, key=lambda c: (math.degrees(peaks[c] - peaks[1]) % 720.0)))
    check(order == S.FIRING_ORDER, f"sparks occur in firing order {order}")
    worst, n_g = 0.0, 0
    for f in frames[::3]:
        sc.frame_set(f)
        dg = bpy.context.evaluated_depsgraph_get()
        for c in range(1, 5):
            for kind in E.GAS_KINDS:
                g = P[f"gas_{c}_{kind}"]
                if g.hide_viewport:
                    continue
                V = world_verts(g, dg)
                crown = P[f"piston{c}"].matrix_world.translation.z + S.COMPRESSION_HEIGHT
                worst = max(worst, abs(V[:, 2].min() - crown - 0.0002))
                n_g += 1
    check(n_g > 10 and worst < 1e-5, f"gas volume floor follows the piston crown ({n_g} samples, err {worst * 1e3:.4f} mm)")

    # ------------------------------------------------------------------ geometry facts
    print("geometry:")
    import shapely  # noqa: F401
    vc = 0.0
    xs = np.linspace(-E.BORE_R, E.BORE_R, 2001)
    for x in xs:
        h = float(E.roof_z(x) - E.ZH)
        vc += h * 2 * math.sqrt(max(E.BORE_R ** 2 - x * x, 0.0)) * (xs[1] - xs[0])
    d, ra = E.CROWN_DISH, 0.032
    vdish = math.pi * d * (3 * ra * ra + d * d) / 6
    vclear = vc + math.pi / 4 * (S.BORE + 1.6e-3) ** 2 * E.GASKET_T + math.pi / 4 * S.BORE ** 2 * 0.8e-3 + vdish
    vs = math.pi / 4 * S.BORE ** 2 * S.STROKE
    print(f"  compression ratio from the modelled chamber + crown dish (reliefs ignored): {(vs + vclear) / vclear:.1f}"
          f" (spec {S.COMPRESSION_RATIO})")
    print(f"  cam centre spacing {E.CAM_SPACING * 1e3:.2f} mm (spec {S.CAM_CENTRE_SPACING * 1e3:.0f}; 42T tip dia "
          f"{2 * E.R_TIP_CAM * 1e3:.1f} mm), chain {A.meta['chain']['n_links']} links, shoe push "
          f"{A.meta['chain']['push'] * 1e3:.1f} mm")
    top = (E.Z_CAM + E.COVER_H + S.Z_CRANK)
    print(f"  cam cover top z = {top:.3f} m, timing cover top z = "
          f"{max(v.co.z for v in P['timing_cover'].data.vertices) + S.Z_CRANK:.3f} m, sump bottom "
          f"{min(v.co.z for v in P['oil_pan'].data.vertices) + S.Z_CRANK:.3f} m")
    xs_all = [world_verts(P[k], bpy.context.evaluated_depsgraph_get())[:, 0] for k in ("intake_manifold", "exhaust_manifold")]
    print(f"  width: intake side x = {xs_all[0].min():.3f} m, exhaust side x = {xs_all[1].max():.3f} m")

    if not args.no_render:
        render_all(A, T, args.out)

    if FAILS:
        print(f"\n{len(FAILS)} FAILURES")
        for f in FAILS:
            print("  -", f)
        sys.exit(1)
    print("\nall engine checks passed")


VIEWS = {
    "overall": ("none", (-1.55, 1.25, 1.25), (0.0, -0.08, 0.50), 50, 1),
    "overall_r": ("none", (1.55, 1.15, 1.15), (0.0, -0.08, 0.50), 50, 1),
    "long": ("long", (-1.75, -0.05, 0.62), (0.0, -0.06, 0.52), 50, 1),
    "cyl1": ("cyl1", (0.0, 1.45, 0.62), (0.0, 0.06, 0.55), 50, 1),
    "cyl1_head": ("cyl1", (0.0, 0.62, 0.68), (0.0, 0.06, 0.66), 50, 30),
    "front": ("front", (-0.55, 1.25, 1.30), (0.0, 0.10, 0.52), 50, 1),
    "flywheel": ("none", (0.95, -1.30, 0.60), (0.0, -0.33, 0.36), 50, 1),
}


def render_all(A, T, out):
    from carviz import lighting
    sc = bpy.context.scene
    st = lighting.setup_studio("dark", center=(0.0, -0.06, 0.48), size=0.7)
    rig_ = st["rig"]
    lighting.setup_color_management(sc)
    cd = bpy.data.cameras.new("test_cam")
    cam = bpy.data.objects.new("test_cam", cd)
    sc.collection.objects.link(cam)
    sc.camera = cam
    r = sc.render
    r.resolution_x, r.resolution_y = 640, 360
    for name, (variant, eye, tgt, lens, frame) in VIEWS.items():
        pres = {"variant": variant}
        if name == "flywheel":
            pres["explode"] = np.full(T.n, 1.0)
        A.drive(T, pres)
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
        # key light ~50 deg off the camera azimuth (camera.py convention: az 0 = -Y, 90 = +X)
        cam_az = math.degrees(math.atan2(eye[0] - tgt[0], -(eye[1] - tgt[1])))
        rig_.rotation_euler = (0.0, 0.0, math.radians(cam_az + 50.0 - 225.0))
        cd.lens = lens
        cd.clip_start = 0.01
        r.filepath = os.path.join(out, f"engine_{name}.png")
        t = time.time()
        bpy.ops.render.render(write_still=True)
        print(f"  rendered {name} ({time.time() - t:.1f} s) -> {r.filepath}")
    # Workbench motion strip (6 frames, long cut)
    A.drive(T, {"variant": "long"})
    r.engine = "BLENDER_WORKBENCH"
    sh = sc.display.shading
    sh.light = "STUDIO"
    sh.color_type = "MATERIAL"
    sh.show_cavity = True
    sc.display.render_aa = "8"
    eye, tgt = (-1.55, -0.05, 0.62), (0.0, -0.06, 0.52)
    cam.location = eye
    cam.rotation_euler = (Vector(tgt) - Vector(eye)).to_track_quat("-Z", "Y").to_euler()
    for k, f in enumerate(np.linspace(1, T.n, 6).astype(int)):
        sc.frame_set(int(f))
        r.filepath = os.path.join(out, f"engine_strip_{k}.png")
        bpy.ops.render.render(write_still=True)
    print(f"  workbench strip -> {out}/engine_strip_[0-5].png")


if __name__ == "__main__":
    main()
