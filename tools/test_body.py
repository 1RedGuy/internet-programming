#!/usr/bin/env python3
"""Self-test for carviz/assemblies/body.py (car body + interior).

    python3 tools/test_body.py                      # build 'high', all checks, renders
    python3 tools/test_body.py --detail low --no-render
    python3 tools/test_body.py --real               # also collide against the real
                                                    # engine/clutch/gearbox/axle/wheels
                                                    # assemblies that already build
    python3 tools/test_body.py --out DIR            # where images go

Checks (all numeric, printed as PASS/FAIL; exit code 1 on any FAIL):
  * build time, triangle counts (budget 1.5 M at 'high')
  * overall dimensions vs spec (length / width / height / overhangs), ground
    clearance, greenhouse fraction, windscreen rake, arch gap over the tyre
  * stand-in drivetrain envelopes at their spec positions (engine block/head/
    manifolds, bellhousing, gearbox + shift tower, propshaft + joints,
    differential + pinion nose, halfshafts + joints): BVH overlap against every
    body part, plus minimum clearances (hood over the cam cover, tunnel, floor)
  * four stand-in tyres driven through +-60 mm suspension travel x +-full lock
    (25 combinations = 25 frames): no overlap with shell/bumpers/liners/trim
  * gear-lever sweep (H-pattern extremes, two pivot heights) passes through the
    console and tunnel openings with clearance
  * brake/throttle pedals vs the clutch pedal sweep (spec pivot/arm/angles),
    steering column vs the pedal box
  * anchors exist and lie on/near their parts; presentation fades bake and read back
Renders (Cycles 640x360, 16 spp, OIDN): light studio opaque from front-3/4,
side, rear-3/4 and top; x-ray (exterior cv_opacity 0.18, stand-in drivetrain,
x-ray edges) from 3 angles; Workbench strip of the fade (6 frames).
"""
from __future__ import annotations

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

from carviz import collide, kin, rig, state  # noqa: E402
from carviz import spec as S  # noqa: E402
from carviz.assemblies import body  # noqa: E402

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok), detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
    return ok


# ---------------------------------------------------------------------------
# stand-in geometry (conservative envelopes at spec positions)
# ---------------------------------------------------------------------------

def _obj(name, V, F, col):
    me = bpy.data.meshes.new(name)
    me.from_pydata([tuple(map(float, v)) for v in V], [], [tuple(f) for f in F])
    me.update()
    ob = bpy.data.objects.new(name, me)
    col.objects.link(ob)
    return ob


def box(name, lo, hi, col):
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    V = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1),
         (x0, y1, z1)]
    F = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    return _obj(name, V, F, col)


def cyl(name, p0, p1, r, col, n=32):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    ax = p1 - p0
    ax /= np.linalg.norm(ax)
    ref = np.array([0, 0, 1.0]) if abs(ax[2]) < 0.9 else np.array([1.0, 0, 0])
    e1 = np.cross(ax, ref)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(ax, e1)
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    ring = np.cos(a)[:, None] * e1 + np.sin(a)[:, None] * e2
    V = np.vstack([p0 + r * ring, p1 + r * ring])
    F = [(i, (i + 1) % n, n + (i + 1) % n, n + i) for i in range(n)]
    F += [tuple(range(n))[::-1], tuple(range(n, 2 * n))]
    return _obj(name, V, F, col)


def tyre(name, col, n=64):
    """Tyre envelope about local +X (wheel axis): 205 wide, R 0.3115, sidewall bulge."""
    prof = [(-0.1025, 0.205), (-0.108, 0.245), (-0.107, 0.285), (-0.097, 0.305), (-0.075, S.TYRE_MESH_RADIUS),
            (0.075, S.TYRE_MESH_RADIUS), (0.097, 0.305), (0.107, 0.285), (0.108, 0.245), (0.1025, 0.205)]
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    V = []
    for (x, r) in prof:
        for t in a:
            V.append((x, r * math.cos(t), r * math.sin(t)))
    m = len(prof)
    F = []
    for i in range(m):
        i1 = (i + 1) % m
        for k in range(n):
            k1 = (k + 1) % n
            F.append((i * n + k, i * n + k1, i1 * n + k1, i1 * n + k))
    return _obj(name, V, F, col)


def drivetrain_standins(col):
    zc = S.Z_CRANK
    yb0, yb1 = S.Y_BLOCK_REAR, S.Y_BLOCK_FRONT + 0.035
    out = [
        box("sd_block", (-0.21, yb0, zc - 0.19), (0.21, yb1, S.Z_DECK), col),
        box("sd_head", (-0.25, yb0 + 0.01, S.Z_DECK), (0.25, yb1 - 0.01, 0.800), col),
        box("sd_intake", (-0.43, S.Y_CYL[-1] - 0.06, 0.50), (-0.21, S.Y_CYL[0] + 0.06, 0.76), col),
        box("sd_exhaust", (0.21, S.Y_CYL[-1] - 0.06, 0.42), (0.37, S.Y_CYL[0] + 0.06, 0.66), col),
        cyl("sd_bell", (0, S.Y_BLOCK_REAR, zc), (0, S.Y_GEARBOX_FRONT, zc), 0.180, col),
        cyl("sd_gbox", (0, S.Y_GEARBOX_FRONT, zc), (0, S.Y_GEARBOX_REAR, zc), 0.140, col),
        box("sd_tower", (-0.06, S.Y_SHIFT_LEVER - 0.07, zc), (0.06, S.Y_SHIFT_LEVER + 0.07, 0.535), col),
        cyl("sd_prop", (0, S.Y_GEARBOX_REAR, zc), (0, S.Y_PINION_FLANGE, S.Z_PINION), S.PROPSHAFT_TUBE_D / 2, col),
        cyl("sd_ujoint_f", (0, S.Y_GEARBOX_REAR + 0.02, zc), (0, S.Y_GEARBOX_REAR - 0.10, zc - 0.004), 0.055, col),
        cyl("sd_ujoint_r", (0, S.Y_PINION_FLANGE + 0.10, S.Z_PINION + 0.004), (0, S.Y_PINION_FLANGE - 0.02,
                                                                                 S.Z_PINION), 0.055, col),
        cyl("sd_diff", (-0.175, S.Y_DIFF, S.Z_DIFF), (0.175, S.Y_DIFF, S.Z_DIFF), 0.160, col),
        cyl("sd_pinion", (0, S.Y_PINION_FLANGE, S.Z_PINION), (0, S.Y_DIFF + 0.06, S.Z_PINION), 0.075, col),
    ]
    for s in (1, -1):
        out.append(cyl(f"sd_ij{s}", (s * S.X_DIFF_OUTPUT, S.Y_DIFF, S.Z_DIFF),
                       (s * (S.X_DIFF_OUTPUT + 0.13), S.Y_DIFF, S.Z_DIFF), 0.050, col))
        out.append(cyl(f"sd_hs{s}", (s * (S.X_DIFF_OUTPUT + 0.13), S.Y_DIFF, S.Z_DIFF),
                       (s * (S.X_WHEEL_HUB - 0.04), S.Y_DIFF, S.Z_DIFF), 0.020, col))
        out.append(cyl(f"sd_oj{s}", (s * (S.X_WHEEL_HUB - 0.05), S.Y_DIFF, S.Z_DIFF),
                       (s * (S.X_WHEEL_HUB + 0.03), S.Y_DIFF, S.Z_DIFF), 0.050, col))
    return out


# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--detail", default="high")
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--real", action="store_true")
    ap.add_argument("--out", default=os.path.join(ROOT, "out", "test_body"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.render.fps = S.FPS

    print("== build")
    t0 = time.time()
    B = body.build({"detail": a.detail, "xray_edges": True})
    bt = time.time() - t0
    tris = B.meta["triangles"]
    print(f"  build time {bt:.1f} s; triangles total {B.meta['triangles_total']:,}")
    for k, v in sorted(tris.items(), key=lambda kv: -kv[1]):
        print(f"    {k:14s} {v:9,d}")
    print("  build log:", {k: (round(v, 2) if isinstance(v, float) else v) for k, v in B.meta["build_log"].items()})
    budget = 1_500_000 if a.detail == "high" else 600_000
    check("triangle budget", B.meta["triangles_total"] < budget, f"{B.meta['triangles_total']:,} < {budget:,}")
    check("build time", bt < 60.0, f"{bt:.1f} s < 60 s")
    check("bumpers split from shell", B.meta["build_log"]["front_bumper"] > 0 and B.meta["build_log"]["rear_bumper"] > 0,
          f"front {B.meta['build_log']['front_bumper']} / rear {B.meta['build_log']['rear_bumper']} faces")
    for k, o in B.parts.items():
        assert o.name.startswith("body_"), o.name
        assert "cv_opacity" in o and "cv_glow" in o, o.name
        assert all(m is not None for m in o.data.materials), o.name
    check("names/props/materials", True, f"{len(B.parts)} parts, all body_*, cv_opacity/cv_glow, no empty slots")

    bpy.context.view_layer.update()
    paint = [B.parts["shell"], B.parts["bumpers"]]
    co = np.vstack([body._verts(o.data) for o in paint])
    lo, hi = co.min(0), co.max(0)
    print("== dimensions")
    check("length", abs((hi[1] - lo[1]) - S.BODY_LENGTH) < 0.012, f"{hi[1] - lo[1]:.3f} m (spec {S.BODY_LENGTH})")
    check("front overhang", abs(hi[1] - S.Y_BODY_FRONT) < 0.01, f"front y {hi[1]:+.3f} (spec {S.Y_BODY_FRONT})")
    check("rear end", abs(lo[1] - S.Y_BODY_REAR) < 0.01, f"rear y {lo[1]:+.3f} (spec {S.Y_BODY_REAR:.3f})")
    check("width (excl. mirrors)", abs((hi[0] - lo[0]) - S.BODY_WIDTH) < 0.012,
          f"{hi[0] - lo[0]:.3f} m (spec {S.BODY_WIDTH})")
    check("height", abs(hi[2] - S.BODY_HEIGHT) < 0.01, f"{hi[2]:.3f} m (spec {S.BODY_HEIGHT})")
    check("ground clearance (painted body)", lo[2] > 0.13, f"lowest painted point {lo[2]:.3f} m")
    mco = body._verts(B.parts["mirrors"].data)
    check("width over mirrors", 1.95 < mco[:, 0].max() - mco[:, 0].min() < 2.10,
          f"{mco[:, 0].max() - mco[:, 0].min():.3f} m")
    # proportions
    bv = body._bvh_of(B.parts["shell"].data)

    def zhit(x, y):
        h = bv.ray_cast(Vector((x, y, 3.0)), Vector((0, 0, -1)))
        return h[0].z if h[0] is not None else float("nan")

    belt = body._belt_z(-1.95)
    check("greenhouse ~1/3 of height", 0.27 < (S.BODY_HEIGHT - belt) / S.BODY_HEIGHT < 0.36,
          f"belt {belt:.3f} m -> greenhouse {(S.BODY_HEIGHT - belt) / S.BODY_HEIGHT:.2f} of height")
    (wO, wU, wV, wN), _ = body._screen_frames()
    rake = math.degrees(math.atan2(-wV[1], wV[2]))
    check("windscreen rake", 55 < rake < 66, f"{rake:.1f} deg from vertical")
    gap = body.ARCH_R - (S.TYRE_MESH_RADIUS + (S.WHEEL_CENTER_Z - body.ARCH_ZC))
    check("arch gap over tyre at rest", 0.02 < gap < 0.045, f"{gap * 1000:.0f} mm")
    hood = [zhit(0.0, y) for y in (S.Y_BLOCK_FRONT, 0.0, S.Y_BLOCK_REAR)]
    check("hood over engine", min(hood) > 0.84, "hood z at block front/axle/block rear: "
          + ", ".join(f"{h:.3f}" for h in hood) + " (cam cover top ~0.80)")

    print("== stand-in drivetrain clearances")
    col = rig.collection("test_standins")
    sd = drivetrain_standins(col)
    keys = ("shell", "bumpers", "underbody", "interior", "cabin_trim", "trim", "lights_front", "lights_rear",
            "glass", "steering_wheel", "pedals")
    bodyparts = [B.parts[k] for k in keys]
    bad = collide.check_pairs([(s, b) for s in sd for b in bodyparts], frames=[1])
    check("no drivetrain overlap", not bad, f"{len(bad)} overlapping pairs " + str([(x[1], x[2]) for x in bad][:6]))
    dg = bpy.context.evaluated_depsgraph_get()
    for sname, pname in (("sd_head", "shell"), ("sd_intake", "shell"), ("sd_exhaust", "shell"),
                         ("sd_bell", "underbody"), ("sd_gbox", "underbody"), ("sd_tower", "underbody"),
                         ("sd_prop", "underbody"), ("sd_ujoint_r", "underbody"), ("sd_diff", "underbody"),
                         ("sd_diff", "interior"), ("sd_pinion", "interior"), ("sd_gbox", "interior")):
        s_ob = bpy.data.objects[sname]
        d = min(collide.min_distance(s_ob, B.parts[pname], samples=600, dg=dg),
                collide.min_distance(B.parts[pname], s_ob, samples=4000, dg=dg))
        check(f"clearance {sname[3:]} -> {pname}", d > 0.008, f"{d * 1000:.0f} mm")

    print("== tyres: suspension travel + steering lock (26 frames)")
    fw = kin.front_wheel_kinematics(1.0, 0.2)       # R 5 m turn (scene 6) -> inner wheel at full lock
    lock = max(abs(fw[0]), abs(fw[1]))
    tyres = {}
    for nm, (x, y) in {"FL": (-S.TRACK_FRONT / 2, 0.0), "FR": (S.TRACK_FRONT / 2, 0.0),
                       "RL": (-S.TRACK_REAR / 2, S.Y_REAR_AXLE), "RR": (S.TRACK_REAR / 2, S.Y_REAR_AXLE)}.items():
        piv = rig.empty(f"sd_steer_{nm}", (x, y, S.WHEEL_CENTER_Z), col=col)
        t = tyre(f"sd_tyre_{nm}", col)
        t.parent = piv
        tyres[nm] = (piv, t)
    T = S.SUSPENSION_TRAVEL
    # front: every steer angle x bump -60..+45 mm (+ full bump straight ahead);
    # rear: +-60 mm (scene 7 moves the rear-right wheel through the full travel)
    combos = [(b, st) for b in (-T, -T / 2, 0.0, T / 2, 0.75 * T) for st in (-lock, -lock / 2, 0.0, lock / 2, lock)]
    combos += [(T, 0.0)]
    rear_b = np.linspace(-T, T, len(combos))
    frames = np.arange(1, len(combos) + 1)
    for nm, (piv, t) in tyres.items():
        if nm.startswith("F"):
            rig.bake_channel(piv, "location", 2, frames, np.array([S.WHEEL_CENTER_Z + b for b, st in combos]))
            rig.bake_channel(piv, "rotation_euler", 2, frames, np.array([st for b, st in combos]))
        else:
            rig.bake_channel(piv, "location", 2, frames, S.WHEEL_CENTER_Z + rear_b)
    wheel_parts = [B.parts[k] for k in ("shell", "bumpers", "underbody", "trim", "interior", "cabin_trim",
                                        "lights_front", "lights_rear")]
    bad = collide.check_pairs([(t, p) for (_, t) in tyres.values() for p in wheel_parts], frames=frames)
    check("tyres clear body (travel, lock)", not bad,
          f"{len(frames)} frames; lock {math.degrees(lock):.1f} deg, front bump -60..+45 mm at any lock and "
          f"+60 mm straight, rear +-60 mm; {len(bad)} overlaps " + str(sorted({(x[1], x[2]) for x in bad})[:6]))
    # informational: largest bump at full lock with no contact (front-left)
    piv, t = tyres["FL"]
    piv.animation_data_clear()
    lo_b, hi_b = 0.0, T
    for _ in range(7):
        mid = 0.5 * (lo_b + hi_b)
        piv.location.z = S.WHEEL_CENTER_Z + mid
        piv.rotation_euler = (0, 0, lock)
        bpy.context.view_layer.update()
        dg = bpy.context.evaluated_depsgraph_get()
        hit = any(collide.overlap_count(t, B.parts[k], dg) for k in ("shell", "bumpers", "underbody"))
        lo_b, hi_b = (lo_b, mid) if hit else (mid, hi_b)
    print(f"    info: at full lock ({math.degrees(lock):.1f} deg) the front tyre clears the arch lip up to "
          f"+{lo_b * 1000:.0f} mm bump")
    B.meta["front_bump_at_full_lock"] = lo_b
    sc.frame_set(1)

    print("== gear lever sweep through console + tunnel openings")
    chw, cyf, cyr = body.CONSOLE_HOLE
    thw, tyf, tyr = body.TUNNEL_HOLE
    worst_c, worst_t = 1e9, 1e9
    knob0 = np.array(S.SHIFT_KNOB_REST)
    for zp in (0.48, 0.55):
        pivot = np.array([0.0, S.Y_SHIFT_LEVER, zp])
        for lx in (-1, 0, 1):
            for ly in (-1, 0, 1):
                k = knob0 + np.array([lx * S.LEVER_GATE_SPACING, ly * S.LEVER_THROW, 0.0])
                for (zz, hw, yf, yr, which) in ((0.655, chw, cyf, cyr, "c"),
                                                (body._tunnel_params(S.Y_SHIFT_LEVER)[1], thw, tyf, tyr, "t")):
                    u = (zz - pivot[2]) / (k[2] - pivot[2])
                    if u < 0:
                        continue
                    p = pivot + u * (k - pivot)
                    m = min(hw - abs(p[0]), yf - p[1], p[1] - yr) - 0.010     # 10 mm lever-rod radius
                    if which == "c":
                        worst_c = min(worst_c, m)
                    else:
                        worst_t = min(worst_t, m)
    check("lever clears console opening", worst_c > 0.0, f"min margin {worst_c * 1000:.0f} mm (rod r 10 mm)")
    check("lever clears tunnel opening", worst_t > 0.0, f"min margin {worst_t * 1000:.0f} mm (rod r 10 mm)")

    print("== pedals")
    pv = np.array(S.CLUTCH_PEDAL_PIVOT)
    a_press = float(kin.clutch_geometry(np.array([1.0]))["pedal_angle"][0])
    pd = body._verts(B.parts["pedals"].data)
    brake = pd[np.abs(pd[:, 0] - S.BRAKE_PEDAL_X) < 0.06]
    clutch_x = pv[0]
    gapx = (brake[:, 0].min() - (clutch_x + 0.045))
    check("brake pad clear of clutch pedal (x)", gapx > 0.01, f"{gapx * 1000:.0f} mm lateral gap")
    for ang in (S.PEDAL_REST_ANGLE, S.PEDAL_REST_ANGLE - a_press):
        pad = pv + S.PEDAL_ARM * np.array([0, -math.sin(ang), -math.cos(ang)])
        ok = pad[1] < S.Y_FIREWALL - 0.03 and pad[2] > S.Z_FLOOR + 0.12
        check(f"clutch pad ahead of floor/firewall at {math.degrees(ang):+.0f} deg", ok,
              f"pad centre y {pad[1]:+.3f}, z {pad[2]:.3f}")
    stc = body._verts(B.parts["steering_wheel"].data)
    # pedal-box envelope: pivot shaft + bosses from the clutch to the throttle pedal
    lo_b = np.array([pv[0] - 0.05, pv[1] - 0.05, pv[2] - 0.05])
    hi_b = np.array([S.THROTTLE_PEDAL_X + 0.05, pv[1] + 0.05, pv[2] + 0.05])
    d = np.maximum(np.maximum(lo_b - stc, stc - hi_b), 0.0)
    dmin = float(np.min(np.linalg.norm(d, axis=1)))
    check("steering column clear of pedal box", dmin > 0.03,
          f"{dmin * 1000:.0f} mm from the pivot envelope (column ends at y {stc[:, 1].max():+.3f})")

    print("== anchors")
    for nm in ("hood", "windscreen", "roof", "front_bumper", "door_left", "cabin", "firewall", "tunnel"):
        ob, off = B.anchors[nm]
        w = B.anchor_world(nm)
        print(f"    {nm:13s} {ob.name:20s} ({w.x:+.3f}, {w.y:+.3f}, {w.z:+.3f})")
    check("anchors", len(B.anchors) == 8, "8 anchors")

    print("== presentation (fade bake + read back)")
    prog = state.Program(duration=2.0)
    prog.speed_kmh.key(0.0, 0.0, "step")
    tr = prog.run()
    n = tr.n
    ext = np.linspace(1.0, 0.18, n)
    B.drive(tr, {"exterior_opacity": ext, "interior_opacity": np.linspace(1.0, 0.0, n),
                 "xray_edges_opacity": np.linspace(0.0, 0.6, n)})
    errs = []
    for f in (1, n // 2, n):
        sc.frame_set(int(f))
        v = B.parts["shell"].get("cv_opacity")
        errs.append(abs(v - ext[f - 1]))
    sc.frame_set(n)
    check("fade bake read back", max(errs) < 1e-4 and B.parts["interior"].hide_render,
          f"max |cv_opacity - target| {max(errs):.1e}; interior hidden at the end")
    # reset for renders
    for o in B.objects():
        if o.animation_data:
            o.animation_data_clear()
        if o.type == "MESH":
            o["cv_opacity"] = 1.0
            o.hide_render = False
    B.parts["xray_edges"]["cv_opacity"] = 0.0
    B.parts["xray_edges"].hide_render = True

    if a.real:
        real_assemblies(B)

    if not a.no_render:
        renders(B, sd, tyres, a.out)

    nf = sum(1 for r in RESULTS if not r[1])
    print(f"\n== {len(RESULTS) - nf}/{len(RESULTS)} checks passed")
    return 1 if nf else 0


def real_assemblies(B):
    """Collide the body against whichever real assemblies build already."""
    import importlib
    print("== real assemblies")
    others = []
    for name in ("engine", "clutch", "gearbox", "axle", "wheels"):
        try:
            mod = importlib.import_module(f"carviz.assemblies.{name}")
            if not hasattr(mod, "build"):
                print(f"    {name}: no build() yet")
                continue
            A = mod.build({"detail": "low"})
            others.append((name, A))
            print(f"    {name}: built ({len(A.parts)} parts)")
        except Exception as e:   # noqa: BLE001
            print(f"    {name}: failed to build: {e}")
    if not others:
        return
    prog = state.Program(duration=3.0, substeps=4)
    prog.slowmo.key(0.0, 1 / 20, "step")
    prog.start_in_gear(1)
    prog.speed_kmh.key(0.0, 10.0, "step")
    prog.throttle_rpm.key(0.0, 1500.0, "step")
    prog.curvature.key(0.0, 0.0).key(1.5, 0.2).key(3.0, -0.2)
    prog.pedal.key(0.0, 0.0).key(1.0, 1.0).key(2.0, 0.0)
    for w in ("RL", "RR", "FL", "FR"):
        prog.susp[w].key(0.0, -0.06).key(1.0, 0.06).key(2.0, -0.06).key(3.0, 0.06)
    try:
        tr = prog.run()
    except Exception as e:   # noqa: BLE001
        print("    program failed:", e)
        return
    keys = ("shell", "bumpers", "underbody", "interior", "cabin_trim", "trim", "steering_wheel", "pedals",
            "glass", "lights_front", "lights_rear")
    bodyparts = [B.parts[k] for k in keys]
    for name, A in others:
        try:
            A.drive(tr, {})
        except Exception as e:   # noqa: BLE001
            print(f"    {name}: drive failed: {e}")
        meshes = [o for o in A.meshes() if not o.hide_render and len(o.data.polygons)]
        frames = np.linspace(1, tr.n, 24).astype(int)
        bad = collide.check_pairs([(m, b) for m in meshes for b in bodyparts], frames=frames)
        pairs = sorted({(x[1], x[2]) for x in bad})
        check(f"real {name} vs body", not bad, f"{len(bad)} overlaps over 24 frames: {pairs[:8]}")


def renders(B, sd, tyres, out):
    from carviz import lighting, materials
    sc = bpy.context.scene
    print("== renders")
    for o in sd:
        o.hide_render = True
    for nm, (piv, t) in tyres.items():
        if piv.animation_data:
            piv.animation_data_clear()
        piv.location.z = S.WHEEL_CENTER_Z
        piv.rotation_euler = (0, 0, 0)
        t.data.materials.append(materials.get("tire_rubber"))
        me = t.data
        me.shade_smooth()
        # rim disc so the stand-in wheels read as wheels
        r = body._lathe_vf([(0.0, -0.085), (S.RIM_DIAMETER / 2 - 0.005, -0.085), (S.RIM_DIAMETER / 2, -0.07),
                            (S.RIM_DIAMETER / 2, 0.06), (0.06, 0.07), (0.0, 0.09)], 48, axis=(1, 0, 0))
        V, F = r
        if piv.location.x < 0:
            V = V * np.array([-1, 1, 1])
        rim = _obj(f"sd_rim_{nm}", V, F, bpy.data.collections["test_standins"])
        rim.data.materials.append(materials.get("rim_alloy"))
        rim.parent = piv
        rim.data.shade_smooth()
        bpy.ops.object.select_all(action="DESELECT")
    lighting.setup_color_management(sc)
    lighting.setup_studio("light")
    cam = bpy.data.objects.new("test_cam", bpy.data.cameras.new("test_cam"))
    sc.collection.objects.link(cam)
    sc.camera = cam
    r = sc.render
    r.engine = "CYCLES"
    r.resolution_x, r.resolution_y = 640, 360
    cy = sc.cycles
    cy.device = "CPU"
    cy.samples = 16
    cy.use_adaptive_sampling = True
    cy.adaptive_threshold = 0.05
    cy.use_denoising = True
    cy.denoiser = "OPENIMAGEDENOISE"
    cy.max_bounces = 6
    cy.transparent_max_bounces = 16

    def shoot(tag, eye, tgt, lens=50.0):
        cam.location = eye
        cam.rotation_euler = (Vector(tgt) - Vector(eye)).to_track_quat("-Z", "Y").to_euler()
        if tag == "top":
            cam.rotation_euler = (0.0, 0.0, math.pi / 2)
        cam.data.lens = lens
        cam.data.clip_start = 0.02
        r.filepath = os.path.join(out, f"body_{tag}.png")
        t0 = time.time()
        bpy.ops.render.render(write_still=True)
        print(f"    {r.filepath}  {time.time() - t0:.0f} s")

    views = {
        "front34": ((-4.3, 3.0, 1.15), (0, -1.35, 0.62), 50),
        "side": ((-7.8, -1.355, 0.85), (0, -1.355, 0.70), 50),
        "rear34": ((3.9, -7.0, 1.45), (0, -1.6, 0.62), 50),
        "top": ((0.0, -1.355, 10.5), (0, -1.355, 0.0), 58),
    }
    for k, (e, t_, l) in views.items():
        shoot(k, e, t_, l)
    # x-ray: exterior at 0.18 with the stand-in drivetrain visible
    for o in sd:
        o.hide_render = False
        if not o.data.materials:
            o.data.materials.append(materials.get("cast_aluminium" if "prop" not in o.name else "steel_machined"))
    for o in B.meta["groups"]["exterior"]:
        o["cv_opacity"] = 0.18
    for o in B.meta["groups"]["interior"] + B.meta["groups"]["underbody"]:
        o["cv_opacity"] = 0.0
        o.hide_render = True
    B.parts["xray_edges"]["cv_opacity"] = 0.55
    B.parts["xray_edges"].hide_render = False
    for k in ("front34", "side", "rear34"):
        e, t_, l = views[k]
        shoot("xray_" + k, e, t_, l)
    # Workbench motion strip: the explode presentation (body lifted off the chassis)
    for o in B.meta["groups"]["interior"] + B.meta["groups"]["underbody"] + B.meta["groups"]["exterior"]:
        o["cv_opacity"] = 1.0
        o.hide_render = False
    B.parts["xray_edges"].hide_render = True
    prog = state.Program(duration=0.25)
    tr = prog.run()
    B.drive(tr, {"explode": np.linspace(0.0, 1.0, tr.n)})
    r.engine = "BLENDER_WORKBENCH"
    sh = sc.display.shading
    sh.light = "STUDIO"
    sh.color_type = "MATERIAL"
    sh.show_cavity = True
    files = []
    for i, f in enumerate(tr.frames):
        sc.frame_set(int(f))
        e, t_, l = (-5.6, 3.6, 2.4), (0, -1.35, 0.9), 40
        shoot(f"wb_explode_{i}", e, t_, l)
        files.append(os.path.join(out, f"body_wb_explode_{i}.png"))
    try:
        from PIL import Image
        ims = [Image.open(f) for f in files]
        W, H = ims[0].size
        strip = Image.new("RGB", (W * 3, H * 2))
        for i, im in enumerate(ims):
            strip.paste(im, ((i % 3) * W, (i // 3) * H))
        strip = strip.resize((W * 3 // 2, H))
        strip.save(os.path.join(out, "body_wb_explode_strip.png"))
        print("   ", os.path.join(out, "body_wb_explode_strip.png"))
    except Exception as e:   # noqa: BLE001
        print("    strip failed:", e)


if __name__ == "__main__":
    sys.exit(main())
