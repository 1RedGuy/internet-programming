#!/usr/bin/env python3
"""Wheels / driveshafts / brakes / suspension verification.

    python3 tools/test_wheels.py [--detail high|low] [--no-render] [--out DIR] [--views a,b,...]

1. Build the assembly alone (high and low detail, all cutaway variants) and report build
   time and triangle counts.
2. Drive it with a test Program: 10 km/h at 1/10 slow motion; 0-4 s the RR wheel goes
   through one full +-60 mm suspension cycle (RL in opposite phase, fronts +-40 mm); 4.5-10 s
   the car enters a left turn of curvature 1/5 (Ackermann steer 31.6 / 24.6 deg).  Checks:
   * baked spin read back from the f-curves vs the Track (wheels, hubs, discs, shaft, tulip,
     spider, cage, races): rotation_euler[1] == -theta at every sampled frame
   * wheel speeds in the turn vs kin.wheel_speeds / front_wheel_kinematics
   * driveshaft: joint-centre spacing constant (= 0.505 m), spider centre on the diff-output
     axis, plunge >= 0 on bump AND droop (printed), inner joint angle == outer joint angle
   * Rzeppa balls (evaluated world positions): in the bisecting plane, on both offset groove
     centre lines (|P-A| = |P-B| = R_g), in the outer and inner groove meridian planes, inside
     their cage windows
   * tripod rollers inside their tracks (radial slot, side clearance), axial travel printed
   * front: actual wheel heading == steer_FL/FR, tie-rod length constant, rack consistency
   * springs: moving coil end follows its seat
   * tyre contact: lowest tread point vs ground at rest / bump / full lock
   * collisions (carviz.collide BVH overlap) at 24 frames over the suspension cycle and 12 frames
     of the turn, for every moving pair that must not touch
3. Cycles 640x360 16 spp renders (studio 'dark') of the rear corner, CV-joint cutaway, tripod
   cutaway, wheel, front corner at full lock, exploded corner, chassis overview, and a
   6-frame Workbench motion strip, into --out.
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
from carviz.assemblies import wheels as W  # noqa: E402

FAILS = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        FAILS.append(msg)


def fresh():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def test_program():
    P = state.Program(None, duration=10.0)
    P.slowmo.key(0, 0.1, "step")
    P.speed_kmh.key(0, 10.0, "step")
    ts = np.linspace(0.0, 4.0, 17)
    for t in ts:
        P.susp["RR"].key(t, 0.06 * math.sin(2 * math.pi * t / 4.0), "cubic")
        P.susp["RL"].key(t, -0.06 * math.sin(2 * math.pi * t / 4.0), "cubic")
        P.susp["FR"].key(t, 0.04 * math.sin(2 * math.pi * t / 4.0), "cubic")
        P.susp["FL"].key(t, -0.04 * math.sin(2 * math.pi * t / 4.0), "cubic")
    P.curvature.key(0.0, 0.0, "step").key(4.5, 0.0, "linear").key(7.0, 0.2, "ease").key(10.0, 0.2, "linear")
    return P.run()


def fc_value(ob, path, index, frame):
    ad = ob.animation_data
    if ad is None or ad.action is None:
        return None
    try:
        for fc in ad.action.fcurves:
            if fc.data_path == path and fc.array_index == index:
                return fc.evaluate(frame)
    except AttributeError:
        pass
    for layer in ad.action.layers:
        for strip in layer.strips:
            for cb in strip.channelbags:
                for fc in cb.fcurves:
                    if fc.data_path == path and fc.array_index == index:
                        return fc.evaluate(frame)
    return None


def world_verts(ob, dg):
    ev = ob.evaluated_get(dg)
    me = ev.to_mesh()
    co = np.zeros(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    M = np.array(ev.matrix_world)
    ev.to_mesh_clear()
    co = co.reshape(-1, 3)
    return co @ M[:3, :3].T + M[:3, 3]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--detail", default="high")
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--no-collide", action="store_true")
    ap.add_argument("--out", default=os.path.join(ROOT, "out", "test_wheels"))
    ap.add_argument("--views", default=None)
    ap.add_argument("--inner-joint", default="flange", choices=["flange", "spec"])
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    print("build:")
    for det in ("low", args.detail):
        fresh()
        t = time.time()
        A = W.build({"detail": det, "cutaways": ["cv_cut", "tripod_cut"], "inner_joint": args.inner_joint})
        bt = time.time() - t
        whole = sum(n for k, n in A.meta["triangles"].items() if "__" not in k)
        print(f"  detail={det}: {bt:.1f} s, {A.meta['triangles_total']:,} triangles incl. cut pieces "
              f"({whole:,} without), {len(A.parts)} parts, log {A.meta['build_log']}")
        check(bt < 60.0, f"build time {bt:.1f} s < 60 s ({det})")
        check(A.meta["triangles_total"] < 1.5e6, f"triangles {A.meta['triangles_total']:,} < 1.5 M ({det})")
    P = A.parts
    T = test_program()
    check(not T.violations, f"test program valid ({len(T.violations)} violations)")
    t = time.time()
    A.drive(T, {})
    print(f"  drive: {time.time() - t:.2f} s for {T.n} frames")
    sc = bpy.context.scene
    fr = T.frames
    n = T.n

    # ------------------------------------------------------------------ spin read-back
    print("spin (baked f-curves vs Track):")
    worst = 0.0
    names = []
    for c in ("RL", "RR", "FL", "FR"):
        names += [(f"wheel_{c}", c), (f"tire_{c}", c), (f"hub_{c}", c), (f"brake_disc_{c}", c)]
    for c in ("RL", "RR"):
        names += [(f"{p}_{c}", c) for p in ("shaft", "tulip", "spider", "rollers", "cage", "inner_race", "outer_race")]
    samp = np.linspace(1, n, 13).astype(int)
    for nm, c in names:
        th = T["theta_" + c]
        for f in samp:
            v = fc_value(P[nm], "rotation_euler", 1, f)
            worst = max(worst, abs(v + th[f - 1]))
    check(worst < 1e-4, f"{len(names)} spinning parts: |rotation_euler[1] + theta| max {worst:.2e} rad")
    i1, i2 = int(np.searchsorted(T.t, 8.0)), int(np.searchsorted(T.t, 9.0))
    dRL, dRR = T.theta_RL[i2] - T.theta_RL[i1], T.theta_RR[i2] - T.theta_RR[i1]
    vl, vr = kin.wheel_speeds(1.0, 0.2)
    check(abs(dRL / dRR - vl / vr) < 1e-3, f"turn: rear wheel angle ratio L/R {dRL / dRR:.4f} == kin {vl / vr:.4f} "
                                           f"(inner 0.852x, outer 1.148x case)")
    dFL, dFR = T.theta_FL[i2] - T.theta_FL[i1], T.theta_FR[i2] - T.theta_FR[i1]
    _, _, fvl, fvr = kin.front_wheel_kinematics(1.0, 0.2)
    check(abs(dFL / dFR - fvl / fvr) < 1e-3, f"turn: front wheel angle ratio L/R {dFL / dFR:.4f} == kin {fvl / fvr:.4f}")
    rpm_shaft = (T.theta_RR[i2] - T.theta_RR[i1]) / (T.t_sim[i2] - T.t_sim[i1]) * 60 / (2 * math.pi)
    print(f"  shaft RR = wheel RR = {rpm_shaft:.1f} rpm (physical) in the turn (by construction: same theta)")

    # ------------------------------------------------------------------ driveshaft geometry
    print("driveshafts:")
    DS = A.meta["driveshaft"]
    L_sh, x_ij = DS["L"], DS["x_inner"]
    p_exp = L_sh - math.sqrt(L_sh ** 2 - S.SUSPENSION_TRAVEL ** 2)
    print(f"  inner joint '{DS['inner_joint']}': tripod centre |x| = {x_ij * 1000:.1f} mm, Rzeppa centre {DS['x_outer'] * 1000:.1f} mm,"
          f" spacing L = {L_sh * 1000:.1f} mm; tulip face on the diff output flange at |x| = "
          f"{A.meta['diff_interface']['flange_face_x'] * 1000:.1f} mm")
    cyc = [int(f) for f in np.linspace(1, int(4.0 * S.FPS) + 1, 24, endpoint=False)]
    errL = errAx = errAng = 0.0
    plunges = {}
    for c in ("RL", "RR"):
        sx = 1.0 if c == "RR" else -1.0
        pl = []
        for f in cyc:
            sc.frame_set(f)
            Sw = W._nm(f"shaftpiv_{c}")
            Sm = bpy.data.objects[Sw].matrix_world
            Om = bpy.data.objects[W._nm(f"irpiv_{c}")].matrix_world
            Sp, Op = Sm.translation, Om.translation
            errL = max(errL, abs((Op - Sp).length - L_sh))
            errAx = max(errAx, abs(Sp.y - S.Y_DIFF), abs(Sp.z - S.Z_DIFF))
            u = (Sm.to_3x3() @ Vector((0, 1, 0))).normalized()
            wa = (bpy.data.objects[W._nm(f"hubpiv_{c}")].matrix_world.to_3x3() @ Vector((0, 1, 0))).normalized()
            a_in = math.degrees(u.angle(Vector((1, 0, 0))))
            a_out = math.degrees(u.angle(wa))
            errAng = max(errAng, abs(a_in - a_out))
            pl.append((T["susp_" + c][f - 1], abs(Sp.x) - x_ij, a_in))
        plunges[c] = pl
        pa = np.array(pl)
        i_hi, i_lo = int(np.argmax(pa[:, 0])), int(np.argmin(pa[:, 0]))
        print(f"  {c}: plunge at s={pa[i_hi, 0] * 1000:+.1f} mm: {pa[i_hi, 1] * 1000:.2f} mm; at s={pa[i_lo, 0] * 1000:+.1f} mm: "
              f"{pa[i_lo, 1] * 1000:.2f} mm; min {pa[:, 1].min() * 1000:.3f} mm; max joint angle {pa[:, 2].max():.2f} deg")
        check(pa[:, 1].min() > -1e-6, f"{c}: plunge >= 0 on bump and droop (outboard only, CVJ-06)")
        check(abs(pa[i_hi, 1] - p_exp) < 2e-4 and abs(pa[i_lo, 1] - p_exp) < 2e-4,
              f"{c}: plunge {p_exp * 1000:.2f} mm (= L - sqrt(L^2 - s^2)) at +-60 mm")
    check(errL < 1e-5, f"joint-centre spacing constant: max |L - {L_sh:.3f}| = {errL * 1000:.4f} mm")
    check(errAx < 1e-6, f"spider centre on the diff-output axis: max off-axis {errAx * 1000:.4f} mm")
    check(errAng < 1e-3, f"inner joint angle == outer joint angle (Z arrangement): max diff {errAng:.5f} deg")

    # ------------------------------------------------------------------ Rzeppa balls
    print("Rzeppa balls:")
    dg = None
    e_plane = e_A = e_B = e_mo = e_mi = 0.0
    win_ax = win_t = 0.0
    for c in ("RL", "RR"):
        sx = 1.0 if c == "RR" else -1.0
        bell = P[f"outer_race_{c}"]
        ir = P[f"inner_race_{c}"]
        cage = P[f"cage_{c}"]
        for f in cyc:
            sc.frame_set(f)
            Om = bpy.data.objects[W._nm(f"irpiv_{c}")].matrix_world
            O = Om.translation.copy()
            s_out = (Om.to_3x3() @ Vector((0, 1, 0))).normalized() * sx
            w_out = Vector((sx, 0, 0))
            nb = (s_out + w_out).normalized()
            A_ = O - w_out * W.RZ_OFFSET
            B_ = O + s_out * W.RZ_OFFSET
            Mb = bell.matrix_world.inverted()
            Mi = ir.matrix_world.inverted()
            Mc = cage.matrix_world.inverted()
            for k in range(S.RZEPPA_BALLS):
                Pb = P[f"ball_{c}_{k}"].matrix_world.translation.copy()
                e_plane = max(e_plane, abs(nb.dot(Pb - O)))
                e_A = max(e_A, abs((Pb - A_).length - W.RZ_RG))
                e_B = max(e_B, abs((Pb - B_).length - W.RZ_RG))
                psi = k * 2 * math.pi / S.RZEPPA_BALLS
                for M_, key in ((Mb, "o"), (Mi, "i")):
                    lp = M_ @ Pb
                    d = math.atan2(lp.z, lp.x) - psi
                    d = (d + math.pi) % (2 * math.pi) - math.pi
                    dev = abs(d) * math.hypot(lp.x, lp.z)
                    if key == "o":
                        e_mo = max(e_mo, dev)
                    else:
                        e_mi = max(e_mi, dev)
                lp = Mc @ Pb
                d = math.atan2(lp.z, lp.x) - psi
                d = (d + math.pi) % (2 * math.pi) - math.pi
                win_ax = max(win_ax, abs(lp.y))
                win_t = max(win_t, abs(d) * math.hypot(lp.x, lp.z))
    check(e_plane < 1e-5, f"ball centres in the bisecting plane: max {e_plane * 1000:.4f} mm (24 frames x 12 balls)")
    check(e_A < 1e-5 and e_B < 1e-5, f"on both offset groove centre lines: |PA|-Rg max {e_A * 1000:.4f} mm, "
                                     f"|PB|-Rg max {e_B * 1000:.4f} mm")
    check(e_mo < 1e-5 and e_mi < 1e-5, f"in the outer / inner groove meridian planes: max {e_mo * 1000:.4f} / "
                                       f"{e_mi * 1000:.4f} mm (outer race and inner race turn by the same theta)")
    check(win_ax < W.RZ_WIN_HALF - W.RZ_BALL and win_t + W.RZ_BALL < W.RZ_WIN_T,
          f"balls stay in their cage windows: axial offset {win_ax * 1000:.3f} mm (play "
          f"{(W.RZ_WIN_HALF - W.RZ_BALL) * 1000:.2f}), circumferential {win_t * 1000:.3f} mm (play "
          f"{(W.RZ_WIN_T - W.RZ_BALL) * 1000:.2f})")

    # ------------------------------------------------------------------ tripod rollers
    print("tripod rollers:")
    e_side = e_rad = 0.0
    ax_rng = [1e9, -1e9]
    for c in ("RL", "RR"):
        tul = P[f"tulip_{c}"]
        spd = P[f"spider_{c}"]
        for f in cyc:
            sc.frame_set(f)
            Mt = tul.matrix_world.inverted()
            for k, a in enumerate(W._tripod_dirs()):
                e = Vector((math.cos(a), 0.0, math.sin(a)))
                pw = spd.matrix_world @ (e * W.TP_RT)
                lp = Mt @ pw
                t_dir = Vector((math.cos(a), 0, math.sin(a)))
                n_dir = Vector((-math.sin(a), 0, math.cos(a)))
                rad = Vector((lp.x, 0, lp.z)).dot(t_dir)
                e_side = max(e_side, abs(Vector((lp.x, 0, lp.z)).dot(n_dir)))
                e_rad = max(e_rad, abs(rad - W.TP_RT))
                ax_rng = [min(ax_rng[0], lp.y * (1 if c == "RR" else -1)), max(ax_rng[1], lp.y * (1 if c == "RR" else -1))]
    check(e_side < W.TP_TRACK - W.TP_RR, f"rollers stay centred between the track walls: max side offset "
                                         f"{e_side * 1000:.3f} mm (clearance {(W.TP_TRACK - W.TP_RR) * 1000:.2f} mm)")
    check(e_rad < W.TP_SLOT, f"rollers within the radial slot: max {e_rad * 1000:.3f} mm (slot +-{W.TP_SLOT * 1000:.1f} mm)")
    print(f"  roller axial travel in the tracks (outboard +): {ax_rng[0] * 1000:+.2f} .. {ax_rng[1] * 1000:+.2f} mm "
          f"(+-r sin(alpha) per turn plus plunge)")

    # ------------------------------------------------------------------ front: steering
    print("front steering / suspension:")
    turn = [int(f) for f in np.linspace(int(4.5 * S.FPS), n, 12)]
    e_head = e_tie = e_rack = 0.0
    for f in sorted(set(cyc[::2] + turn)):
        sc.frame_set(f)
        xin = {}
        for c in ("FL", "FR"):
            sx = 1.0 if c == "FR" else -1.0
            hp = bpy.data.objects[W._nm(f"hubpiv_{c}")]
            ax = hp.matrix_world.to_3x3() @ Vector((0, 1, 0))
            head = math.atan2(ax.y, ax.x)
            e_head = max(e_head, abs(head - T["steer_" + c][f - 1]))
            g = W._front_geom(sx)
            L0 = (g["tie_out"] - g["tie_in"]).length
            Mtr = bpy.data.objects[W._nm(f"tie_rod_frame_{c}")].matrix_world
            po, pi_ = Mtr @ g["tie_out"], Mtr @ g["tie_in"]
            kn = bpy.data.objects[W._nm(f"knuckle_frame_{c}")].matrix_world
            e_tie = max(e_tie, abs((po - pi_).length - L0), (po - kn @ g["tie_out"]).length)
            xin[c] = pi_.x
        e_rack = max(e_rack, abs((xin["FR"] - xin["FL"]) - 2 * W.F_TIE_IN0.x))
    check(e_head < math.radians(0.05), f"front wheel heading == steer_FL/FR: max error {math.degrees(e_head):.4f} deg "
                                       f"(full lock {math.degrees(T.steer_FL.max()):.1f} / {math.degrees(T.steer_FR.max()):.1f} deg)")
    check(e_tie < 1e-5, f"tie rods: constant length and attached to the steering arm (max {e_tie * 1000:.4f} mm)")
    print(f"  implied rack-bar length change (Ackermann of the steering-arm geometry vs ideal): "
          f"{e_rack * 1000:.1f} mm (hidden inside the rack housing)")

    # ------------------------------------------------------------------ springs follow their seats
    print("springs:")
    e_sp = 0.0
    dg = bpy.context.evaluated_depsgraph_get()
    for f in (cyc[0], cyc[6], cyc[18]):
        sc.frame_set(f)
        dg = bpy.context.evaluated_depsgraph_get()
        for c in ("RL", "RR"):
            v = world_verts(P[f"spring_{c}"], dg)
            zb = v[:, 2].min()
            seat = W.R_SPRING[0] + T["susp_" + c][f - 1]
            e_sp = max(e_sp, abs(zb - seat))
    check(e_sp < 5e-4, f"rear coil-over: spring end on its moving seat within {e_sp * 1000:.3f} mm (bump/droop keys)")

    # ------------------------------------------------------------------ tyre / ground
    print("tyre contact:")
    lows = {}
    for lab, f in (("rest", 1), ("RR bump", cyc[6]), ("full lock", n)):
        sc.frame_set(f)
        dg = bpy.context.evaluated_depsgraph_get()
        lows[lab] = {c: world_verts(P[f"tire_{c}"], dg)[:, 2].min() for c in ("RR", "FL", "FR")}
        print(f"  lowest tread point at {lab}: " + ", ".join(f"{c} {z * 1000:+.2f} mm" for c, z in lows[lab].items()))
    check(abs(lows["rest"]["RR"] - (S.WHEEL_CENTER_Z - S.TYRE_MESH_RADIUS)) < 3e-4,
          "tyre at rest sinks 6.5 mm into the ground plane (TYRE_MESH_RADIUS)")
    check(min(lows["full lock"].values()) > -0.012,
          f"front tyres at full lock stay within the squash allowance (KPI drop): {min(lows['full lock'].values()) * 1000:.1f} mm")

    # ------------------------------------------------------------------ collisions
    if not args.no_collide:
        print("collisions (BVH overlaps):")
        pairs = []

        def add(a, b):
            if a in P and b in P:
                pairs.append((P[a], P[b]))

        for c in ("RL", "RR"):
            for k in range(S.RZEPPA_BALLS):
                for o in ("outer_race", "inner_race", "cage"):
                    add(f"ball_{c}_{k}", f"{o}_{c}")
            for a, b in (("cage", "outer_race"), ("cage", "inner_race"), ("inner_race", "outer_race"),
                         ("shaft", "outer_race"), ("shaft", "cage"), ("boot_outer", "outer_race"), ("boot_outer", "cage"),
                         ("boot_outer", "inner_race"), ("boot_outer", "shaft"), ("rollers", "tulip"), ("spider", "tulip"),
                         ("shaft", "tulip"), ("rollers", "spider"), ("boot_inner", "tulip"), ("boot_inner", "shaft"),
                         ("boot_inner", "spider"), ("boot_inner", "rollers"), ("clamp_os", "boot_outer"),
                         ("clamp_is", "boot_inner"),
                         ("tire", "upright"), ("tire", "damper"), ("tire", "spring"), ("tire", "damper_rod"),
                         ("tire", "upper_link"), ("tire", "toe_link"), ("tire", "lower_arm"), ("tire", "caliper"),
                         ("wheel", "caliper"), ("wheel", "caliper_carrier"), ("wheel", "pads"), ("wheel", "backing_plate"),
                         ("wheel", "upright"), ("wheel", "upper_link"), ("wheel", "toe_link"), ("wheel", "lower_arm"),
                         ("wheel", "outer_race"), ("wheel", "boot_outer"),
                         ("brake_disc", "caliper"), ("brake_disc", "caliper_carrier"), ("brake_disc", "pads"),
                         ("brake_disc", "backing_plate"), ("brake_disc", "upright"),
                         ("hub", "upright"), ("hub", "bearing"), ("hub", "backing_plate"), ("outer_race", "upright"),
                         ("outer_race", "bearing"), ("outer_race", "backing_plate"),
                         ("upright", "boot_outer"), ("upright", "shaft"), ("lower_arm", "boot_outer"),
                         ("lower_arm", "shaft"), ("lower_arm", "boot_inner"), ("upper_link", "shaft"),
                         ("upper_link", "boot_outer"), ("toe_link", "shaft"), ("toe_link", "boot_inner"),
                         ("toe_link", "boot_outer"), ("spring", "shaft"), ("spring", "boot_outer"), ("damper", "shaft"),
                         ("damper", "boot_outer"), ("spring", "upright"), ("spring", "damper"), ("damper", "lower_arm"),
                         ("caliper", "upright"), ("pads", "upright"), ("caliper", "backing_plate"),
                         ("caliper_carrier", "backing_plate"), ("upright", "backing_plate"), ("upright", "bearing"),
                         ("caliper", "caliper_carrier"), ("caliper", "pads"), ("pads", "caliper_carrier"),
                         ("pads", "backing_plate"), ("caliper_carrier", "upright"), ("damper", "upright"),
                         ("upright", "upper_link"), ("upright", "toe_link"), ("upright", "lower_arm"),
                         ("lower_arm", "toe_link"), ("lower_arm", "upper_link")):
                add(f"{a}_{c}", f"{b}_{c}")
            for b in ("shaft", "tulip", "boot_inner", "boot_outer", "spring", "damper_rod", "upper_link", "toe_link",
                      "lower_arm"):
                add("subframe_rear", f"{b}_{c}")
        sus = [f for f in cyc]
        t = time.time()
        bad = collide.check_pairs(pairs, sus)
        print(f"  rear: {len(pairs)} pairs x {len(sus)} frames over the full suspension cycle ({time.time() - t:.1f} s)")
        for b_ in bad[:20]:
            print("   ", b_)
        check(not bad, f"rear: no interpenetration ({len(bad)} overlaps)")
        pairs = []
        for c in ("FL", "FR"):
            for a, b in (("tire", "knuckle"), ("tire", "strut"), ("tire", "spring"), ("tire", "strut_top"),
                         ("tire", "lower_arm"), ("tire", "tie_rod"), ("tire", "rack_boot"), ("tire", "caliper"),
                         ("wheel", "caliper"), ("wheel", "caliper_carrier"), ("wheel", "pads"), ("wheel", "backing_plate"),
                         ("wheel", "knuckle"), ("wheel", "strut"), ("wheel", "tie_rod"), ("wheel", "lower_arm"),
                         ("brake_disc", "caliper"), ("brake_disc", "caliper_carrier"), ("brake_disc", "pads"),
                         ("brake_disc", "backing_plate"), ("brake_disc", "knuckle"), ("hub", "knuckle"),
                         ("hub", "bearing"), ("spring", "strut"), ("spring", "strut_top"), ("spring", "knuckle"),
                         ("strut", "strut_top"), ("lower_arm", "tie_rod"), ("caliper", "knuckle"),
                         ("caliper", "backing_plate"), ("knuckle", "tie_rod"), ("knuckle", "lower_arm"),
                         ("strut", "lower_arm"), ("rack_boot", "tie_rod")):
                add(f"{a}_{c}", f"{b}_{c}")
            for b in ("tire", "wheel", "lower_arm", "tie_rod"):
                add("subframe_front", f"{b}_{c}")
                add("rack", f"{b}_{c}")
        for c in ("FL", "FR"):
            for a_, b_ in (("knuckle", "backing_plate"), ("knuckle", "bearing"), ("caliper", "caliper_carrier"),
                           ("caliper", "pads"), ("pads", "caliper_carrier"), ("pads", "backing_plate"),
                           ("caliper_carrier", "knuckle")):
                add(f"{a_}_{c}", f"{b_}_{c}")
        fl = sorted(set(cyc[::2] + turn))
        t = time.time()
        bad = collide.check_pairs(pairs, fl)
        print(f"  front: {len(pairs)} pairs x {len(fl)} frames (suspension cycle + turn to full lock) "
              f"({time.time() - t:.1f} s)")
        for b_ in bad[:20]:
            print("   ", b_)
        check(not bad, f"front: no interpenetration ({len(bad)} overlaps)")

    # ------------------------------------------------------------------ renders
    if not args.no_render:
        render_all(A, T, args.out, args.views)
    print()
    print("FAILURES:" if FAILS else "ALL CHECKS PASSED")
    for f in FAILS:
        print("  -", f)
    return 1 if FAILS else 0


VIEWS = {
    # name: (presentation, eye, target, lens, frame, hide-prefixes, studio (centre, size))
    "rear_corner": ({}, (0.12, -3.45, 0.62), (0.48, -2.64, 0.31), 30, 25, (), None),
    "joint": ({}, (0.42, -2.98, 0.44), (0.62, -2.62, 0.32), 45, 25, ("spring_RR", "damper"), None),
    "cv_cut": ({"cutaway": "cv_cut"}, (0.48, -2.86, 0.47), (0.655, -2.62, 0.322), 55, 25,
               ("wheel_RR", "tire_RR", "brake_disc_RR", "caliper", "pads_RR", "backing_plate_RR", "spring_RR",
                "damper", "upper_link_RR", "toe_link_RR"), None),
    "tripod_cut": ({"cutaway": "tripod_cut"}, (0.06, -2.78, 0.43), (0.17, -2.62, 0.305), 50, 25,
                   ("subframe_rear",), None),
    "caliper": ({}, (0.98, -3.05, 0.52), (0.74, -2.72, 0.36), 40, 1, ("wheel_RR", "tire_RR"), None),
    "wheel": ({}, (1.65, -3.30, 0.55), (0.74, -2.62, 0.30), 42, 1, (), None),
    "front_lock": ({}, (0.05, 1.05, 0.95), (0.55, 0.0, 0.25), 30, 240, (), ((0.5, 0.0, 0.3), 0.9)),
    "strut": ({}, (0.15, -0.55, 0.95), (0.55, 0.0, 0.55), 35, 1, (), ((0.5, 0.0, 0.4), 0.9)),
    "exploded": ({"explode": 1.0}, (1.55, -3.55, 0.85), (0.70, -2.62, 0.30), 32, 1, (), None),
    "chassis": ({}, (2.9, 1.3, 2.3), (0.0, -1.30, 0.22), 28, 200, (), ((0.0, -1.31, 0.3), 1.8)),
}


def render_all(A, T, out, views=None):
    from carviz import lighting
    sc = bpy.context.scene
    studio_key = [None]
    st = {}

    def studio(spec):
        spec = spec or ((0.5, -2.62, 0.30), 0.9)
        if studio_key[0] != spec:
            lighting.clear_studio()
            st.clear()
            st.update(lighting.setup_studio("dark", center=spec[0], size=spec[1]))
            studio_key[0] = spec
        return st

    lighting.setup_color_management(sc)
    cd = bpy.data.cameras.new("test_cam")
    cam = bpy.data.objects.new("test_cam", cd)
    sc.collection.objects.link(cam)
    sc.camera = cam
    r = sc.render
    r.resolution_x, r.resolution_y = 640, 360
    want = views.split(",") if views else list(VIEWS) + ["strip"]
    for name, (pres, eye, tgt, lens, frame, hide, stu) in VIEWS.items():
        if name not in want:
            continue
        studio(stu)
        A.drive(T, dict(pres))
        hidden = []
        for k, ob in A.parts.items():
            if any(k.startswith(h) for h in hide):
                if not ob.hide_render:
                    ob.hide_render = True
                    hidden.append(ob)
                if ob.animation_data:
                    for path in ("hide_render", "hide_viewport"):
                        try:
                            for fc in list(ob.animation_data.action.fcurves):
                                if fc.data_path == path:
                                    ob.animation_data.action.fcurves.remove(fc)
                        except Exception:
                            pass
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
        st["rig"].location = tgt if stu is None else stu[0]
        st["rig"].rotation_euler = (0.0, 0.0, math.radians(cam_az + 50.0 - 225.0))
        cd.lens = lens
        cd.clip_start = 0.01
        r.filepath = os.path.join(out, f"wheels_{name}.png")
        t = time.time()
        bpy.ops.render.render(write_still=True)
        print(f"  rendered {name} ({time.time() - t:.1f} s) -> {r.filepath}")
        for ob in hidden:
            ob.hide_render = False
            ob.hide_viewport = False
        A.drive(T, {})
    if "strip" in want:
        studio(None)
        A.drive(T, {})
        r.engine = "BLENDER_WORKBENCH"
        sh = sc.display.shading
        sh.light = "STUDIO"
        sh.color_type = "MATERIAL"
        sh.show_cavity = True
        sc.display.render_aa = "8"
        eye, tgt = (0.15, -3.30, 0.48), (0.50, -2.62, 0.32)
        cam.location = eye
        cam.rotation_euler = (Vector(tgt) - Vector(eye)).to_track_quat("-Z", "Y").to_euler()
        cd.lens = 32
        for k, f in enumerate(np.linspace(1, int(4.0 * S.FPS), 6).astype(int)):
            sc.frame_set(int(f))
            r.filepath = os.path.join(out, f"wheels_strip_{k}.png")
            bpy.ops.render.render(write_still=True)
        print(f"  workbench strip -> {out}/wheels_strip_[0-5].png")


if __name__ == "__main__":
    sys.exit(main())
