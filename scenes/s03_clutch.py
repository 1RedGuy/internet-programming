"""s03 The clutch (63.5 s): parts, splines, engaged, hydraulic release, slipping take-off.

Dark studio; engine (exterior only) + flywheel, the clutch assembly (pack, release
system, half-cut bellhousing, hydraulics, pedal box), the gearbox input shaft (+ input
gear and bearing) and a patch of the body's firewall.

Drivetrain state (carviz.state; everything below is read from the Track)
----------------------------------------------------------------------------
  0   - 27.6  parts / splines / engaged: idle 850 rpm, neutral, clutch engaged, car
              stationary.  Slow motion x230 (flywheel 132T ring gear in shot needs >= x223,
              FACTS PRS-04): 0.34 teeth/frame.
  27.6 - 29.3 ease to x56 while the camera is on the input shaft (ring gear hidden; the
              visibility test includes occlusion by the flywheel).
  30.5 - 44.6 release: one continuous press (pedal 0 -> 1 over 14.1 s of video = 0.25 s
              real); capacity reaches 0 at p = 0.5, the free disc coasts (TAU_DRAG): 850 ->
              760 rpm.  x56 keeps the fingers (18: 0.19 pitch/frame), facing rivets (24:
              0.25), hub splines (23: 0.24), input gear (26T: 0.27) and its dog ring (32:
              0.34) below 0.35; the ring gear (1.39 teeth/frame) is motion-blurred
              (shutter 0.75 -> s*p = 1.04).
  46.5        hard camera cut; slow motion steps to x8 (shutter 0.95).  1st gear is
              selected: the synchroniser stops disc + input shaft in ~0.1 s (car stationary).
  49  - 63.5  take-off: the pedal comes up to the bite, the clutch then passes exactly the
              torque the prescribed take-off needs (pedal = inverse capacity of the required
              torque), the engine is held at ~1190 rpm, the disc speeds up, slip crosses
              zero and the clutch locks at 60.9 s (83 % of the beat, 1210 rpm, 9.7 km/h);
              bite -> lock = 1.19 s real.
"""
from __future__ import annotations

import math

import bpy
import numpy as np
from mathutils import Vector

from carviz import camera as CAM
from carviz import kin, lighting, rig, scenebase, state, timeline
from carviz import materials as MAT
from carviz import meshutil as MU
from carviz import spec as S
from carviz.assemblies import body as BODY
from carviz.assemblies import clutch as CL
from carviz.assemblies import engine as ENG
from carviz.assemblies import gearbox as GBX
from carviz.labels import Hud, Labels, fmt_rpm

SCENE_ID = "s03"
SC = timeline.scene(SCENE_ID)
BT = {b.id: b for b in SC.beats}
FPS = S.FPS
DUR = SC.dur
TAU = 2.0 * math.pi
MM = 1e-3


def bstart(b):
    return BT[b].start


def bend(b):
    return BT[b].end


def wt(b, word, occ=1):
    return BT[b].word_time(word, occ)


def _curve(keys, default=0.0):
    c = state.Curve(default)
    for k in keys:
        c.key(*k)
    return c


# ===========================================================================
# Slow motion, shutter, timing plan
# ===========================================================================
F_IDLE = 230.0           # ring gear 132T at 850 rpm: >= 223 (0.34 teeth/frame)
F_REL = 56.0             # fingers 0.19, rivets 0.25, splines 0.24, input-gear dogs (32) 0.34 pitch/frame
F_SLIP = 8.0             # take-off ~1.2 s real; everything fast is motion-blurred
T_RAMP = (27.6, 29.3)    # x230 -> x56, camera on the hub (ring gear out of frame)
T_CUT = 46.5             # hard cut; x56 -> x8
SHUT_REL = 0.75          # ring gear 1.39 teeth/frame at x56 -> s*p = 1.04
SHUT_SLIP = 0.95         # < 1 so the camera cut never falls inside a shutter interval

# warm presentation glow (cv_glow; materials.CV_Presentation is strong on dark iron / friction)
GLOW_PART = 0.03         # a part glows briefly when it is named
GLOW_PATH = 0.022        # engaged: the power path
GLOW_TORQUE = 0.018      # take-off: power path, scaled by the clutch torque actually passed

# explode (parts beat): staggered assembly, front to back
ASM_T0, ASM_DUR, ASM_STAGGER = 8.95, 1.25, 0.11
OFF_FLY = -0.10          # engine.explode['flywheel']
OFF_SHAFT = -0.47        # input shaft travels with the release bearing group
SWEEP = (10.9, 12.5)     # section cutter sweeps from outside the parts to the axis plane
HOUSING_IN = (10.1, 11.4)
HYD_IN = (30.6, 31.5)    # pedal box, hydraulics, firewall fade in (camera clear of the line)

# release: one continuous press while the camera follows the chain
PEDAL_RELEASE = [(30.5, 0.0, "step"), (31.6, 0.10, "cubic"), (33.6, 0.42, "cubic"), (35.6, 0.62, "cubic"),
                 (38.2, 0.72, "cubic"), (40.6, 0.80, "cubic"), (42.8, 0.91, "cubic"), (44.6, 1.0, "cubic")]
PULSE = (31.6, 35.1)     # line_pulse travels master -> slave ("fluid flows ... to the slave cylinder")

# gear selection after the cut (clutch floored, car stationary)
T_SELECT = 46.75
T_ENGAGE = 47.1
ENGAGE_KW = dict(travel=0.3, hold=0.8, through=0.25, seat=0.15)

# take-off (video times)
T_PEDAL_UP = 49.0        # pedal leaves the floor
T_BITE = 51.4            # pedal reaches the bite (capacity starts to rise)
T_LOCK_TARGET = 60.8
V_LOCK_KMH = 9.6
M_EFF = 1450.0           # kg: car + driver + rotating-inertia equivalent (illustrative)
F_ROLL = 160.0           # N:  rolling resistance (Crr 0.012)
ETA = 0.92               # driveline efficiency in 1st
THR_SLIP = 1420.0        # driver's throttle target while slipping
THR_POST = 1640.0        # ... and at the end of the scene (engine ~1500 rpm, pulling in 1st)


def slowmo_curve():
    c = state.Curve(1.0 / F_IDLE)
    c.key(0.0, 1.0 / F_IDLE, "step")
    c.key(T_RAMP[0], 1.0 / F_IDLE, "linear")
    c.key(T_RAMP[1], 1.0 / F_REL, "ease")
    c.key(T_CUT - 1e-3, 1.0 / F_REL, "linear")
    c.key(T_CUT, 1.0 / F_SLIP, "step")
    c.key(DUR, 1.0 / F_SLIP, "linear")
    return c


def shutter_curve(t):
    c = _curve([(0.0, 0.0, "step"), (T_RAMP[0] + 0.4, 0.0, "linear"), (T_RAMP[1], SHUT_REL, "ease"),
                (T_CUT - 1e-3, SHUT_REL, "linear"), (T_CUT, SHUT_SLIP, "step")])
    return c(t)


# ---------------------------------------------------------------------------
# take-off: prescribed road speed and the pedal that passes exactly the torque it needs
# ---------------------------------------------------------------------------

def _sim_time(slow, t):
    """sim time at video times t (trapezoid, fine grid)."""
    tt = np.linspace(0.0, DUR, int(DUR * FPS * 16) + 1)
    s = np.maximum(slow(tt), 0.0)
    cum = np.concatenate([[0.0], np.cumsum(0.5 * (s[1:] + s[:-1]) * np.diff(tt))])
    return np.interp(t, tt, cum)


def takeoff_profile():
    """Video-time arrays for the slip beat: t, v_kmh, needed capacity, a (m/s^2), sim time
    since the bite, sim time of the target lock.

    Acceleration (sim time) ramps in smoothly from the bite and eases toward a gentle
    cruise acceleration after the lock; the pedal during the slip is the inverse of
    kin.clutch_capacity at the torque the car needs (CLU-10/11)."""
    slow = slowmo_curve()
    t = np.arange(T_BITE, DUR + 1e-9, 1.0 / (FPS * 4))
    tau = _sim_time(slow, t) - _sim_time(slow, np.array([T_BITE]))[0]
    tau_lock = _sim_time(slow, np.array([T_LOCK_TARGET]))[0] - _sim_time(slow, np.array([T_BITE]))[0]
    # acceleration shape: smooth rise over 0.35 s, plateau, ease to 55 % after the lock
    a_shape = kin.smoothstep(0.0, 0.35, tau) * (1.0 - 0.45 * kin.smoothstep(tau_lock - 0.15, tau_lock + 0.35, tau))
    dtau = np.gradient(tau)
    v_shape = np.cumsum(a_shape * dtau) - a_shape[0] * dtau[0]
    k = V_LOCK_KMH / 3.6 / float(np.interp(tau_lock, tau, v_shape))
    a = a_shape * k                                   # m/s^2 (sim)
    v = v_shape * k * 3.6                             # km/h
    g = S.GEAR_RATIOS[1] * S.FINAL_DRIVE
    t_need = (M_EFF * a + F_ROLL * kin.smoothstep(0.0, 0.4, v)) * S.ROLLING_RADIUS / (g * ETA)
    # rolling resistance only once the car moves; add the input-side inertia
    alpha_in = a / S.ROLLING_RADIUS * g
    t_need = t_need + state.I_INPUT * alpha_in
    cap = np.clip(t_need / state.T_CLUTCH_MAX, 0.0, 1.0)
    return t, v, cap, a, tau, tau_lock


def _pedal_for_capacity(cap):
    """Inverse of kin.clutch_capacity on the bite range (monotone)."""
    p = np.linspace(S.CLUTCH_BITE_LO, S.CLUTCH_BITE_HI, 2001)
    c = kin.clutch_capacity(p)            # decreasing in p
    return np.interp(cap, c[::-1], p[::-1])


def make_program():
    P = state.Program(SCENE_ID)
    P.slowmo = slowmo_curve()
    # ---- throttle: idle, then the driver holds ~1400 rpm for the take-off
    P.throttle_rpm.key(0.0, S.IDLE_RPM, "step")
    P.throttle_rpm.key(T_BITE - 1.6, S.IDLE_RPM, "linear")
    P.throttle_rpm.key(T_BITE + 0.4, THR_SLIP, "ease")
    # after the lock the engine pulls the car: keep the target ~150 rpm above engine speed
    P.throttle_rpm.key(T_LOCK_TARGET, THR_SLIP, "linear")
    P.throttle_rpm.key(DUR, THR_POST, "cubic")
    # ---- pedal: release (one press), held, then the take-off
    for (tk, v, m) in PEDAL_RELEASE:
        P.pedal.key(tk, v, m)
    P.pedal.key(46.0, 1.0, "linear")
    P.pedal.key(T_PEDAL_UP, 1.0, "linear")
    t, v, cap, a, tau, tau_lock = takeoff_profile()
    p_slip = _pedal_for_capacity(cap)
    p_bite = float(_pedal_for_capacity(np.array([0.004]))[0])
    P.pedal.key(T_BITE, p_bite, "ease")
    # during the slip: pedal = torque-matched; after the lock: up to the top
    i_lock = int(np.searchsorted(t, T_LOCK_TARGET + 0.25))
    step = 3
    for i in range(1, i_lock, step):
        P.pedal.key(float(t[i]), float(p_slip[i]), "linear")
    t_l = float(t[i_lock])
    P.pedal.key(t_l + 1.6, 0.0, "ease")
    # ---- road speed (rear axle), from rest at the bite
    P.speed_kmh.key(0.0, 0.0, "step")
    P.speed_kmh.key(T_BITE, 0.0, "linear")
    for i in range(1, len(t), 2):
        P.speed_kmh.key(float(t[i]), float(v[i]), "linear")
    # ---- first gear after the cut (pedal on the floor, car stationary)
    P.select_plane(T_SELECT, -1, dur=0.3)
    P.engage(1, T_ENGAGE, **ENGAGE_KW)
    return P


# ===========================================================================
# Camera: (time, target, azimuth deg, radius, height, lens, f-stop, mode)
# azimuth as camera.orbit: 0 = behind (-Y), -90 = left (-X), 180 = front
# ===========================================================================
POSES = [
    # parts: exploded stack from the left-rear, slow drift
    (0.0, (0.0, -0.67, 0.38), -54.0, 1.08, 0.42, 40.0, 6.3, "step"),
    (8.7, (0.0, -0.63, 0.38), -66.0, 1.00, 0.36, 40.0, 6.3, "cubic"),
    # assembly: drift in toward the clutch
    (10.9, (0.0, -0.40, 0.38), -88.0, 0.80, 0.20, 45.0, 6.3, "cubic"),
    # splines: section sweep, then push in on the hub
    (13.2, (0.0, -0.342, 0.368), -98.0, 0.25, 0.05, 50.0, 8.0, "cubic"),
    (17.9, (0.0, -0.345, 0.368), -102.0, 0.24, 0.05, 50.0, 8.0, "cubic"),
    # engaged: the whole half-section
    (20.6, (0.0, -0.39, 0.385), -96.0, 0.80, 0.27, 45.0, 6.3, "cubic"),
    (25.2, (0.0, -0.39, 0.385), -88.0, 0.78, 0.26, 45.0, 6.3, "cubic"),
    # "...the input shaft turns with the engine": the shaft behind the clutch (ring gear out)
    (27.3, (0.0, -0.445, 0.372), -97.0, 0.37, 0.08, 55.0, 8.0, "cubic"),
    (29.4, (0.0, -0.445, 0.372), -100.0, 0.36, 0.08, 55.0, 8.0, "cubic"),
    # release: pull back to the whole chain (front-left: pedal ... clutch)
    (31.6, (-0.22, -0.45, 0.50), -125.0, 1.00, 0.35, 35.0, 6.3, "cubic"),
    (35.0, (-0.22, -0.45, 0.50), -119.0, 0.98, 0.34, 35.0, 6.3, "cubic"),
    # fork + bearing from above (engine side of the firewall, above the hydraulic line)
    (37.5, (-0.08, -0.395, 0.37), -96.0, 0.30, 0.36, 38.0, 6.3, "cubic"),
    (40.0, (-0.07, -0.395, 0.37), -99.0, 0.29, 0.35, 38.0, 6.3, "cubic"),
    # fingers (front-left, ahead of the slave hose)
    (41.7, (0.0, -0.362, 0.398), -103.0, 0.25, 0.06, 55.0, 8.0, "cubic"),
    # macro on the facings / pressure plate (1.8 mm lift, ~0.6 mm per face clearance)
    (43.2, (0.0, -0.343, 0.452), -96.0, 0.135, 0.012, 60.0, 8.0, "cubic"),
    (46.5 - 1e-3, (0.0, -0.343, 0.452), -97.5, 0.13, 0.012, 60.0, 8.0, "cubic"),
    # --- cut: the clutch section (1st gear selected, the disc stops) ...
    (46.5, (0.0, -0.335, 0.41), -108.0, 0.42, 0.10, 45.0, 6.3, "step"),
    (48.5, (0.0, -0.335, 0.41), -112.0, 0.41, 0.10, 45.0, 6.3, "cubic"),
    # ... pull back for the pedal coming up ...
    (50.4, (-0.20, -0.43, 0.46), -121.0, 0.86, 0.30, 38.0, 6.3, "cubic"),
    (51.5, (-0.20, -0.43, 0.46), -119.0, 0.85, 0.30, 38.0, 6.3, "cubic"),
    # ... and in on the section for the slip (disc hub, damper springs, facings)
    (54.0, (0.0, -0.336, 0.405), -106.0, 0.33, 0.08, 45.0, 8.0, "cubic"),
    (59.6, (0.0, -0.336, 0.405), -100.0, 0.32, 0.08, 45.0, 8.0, "cubic"),
    # locks: ease back
    (DUR, (0.0, -0.345, 0.41), -108.0, 0.48, 0.12, 45.0, 6.3, "cubic"),
]
CUTS = [T_CUT]
CAM_SMOOTH = 0.3
KEY_OFFSET = 50.0         # key light azimuth relative to the camera (deg)


def _gauss(x, sigma_frames):
    r = int(3 * sigma_frames)
    if r < 1:
        return x
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma_frames) ** 2)
    k /= k.sum()
    xp = np.r_[np.full(r, x[0]), x, np.full(r, x[-1])]
    return np.convolve(xp, k, mode="valid")


def camera_samples(t):
    """Per-frame eye, target, lens, f-stop, azimuth (deg); Gaussian-smoothed within each
    shot (never across a cut)."""
    t = np.asarray(t, float)
    bounds = [0.0] + CUTS + [DUR + 1.0]
    keys = ("tx", "ty", "tz", "az", "r", "h", "lens", "f")
    out = {k: np.zeros(len(t)) for k in keys}
    for s0, s1 in zip(bounds[:-1], bounds[1:]):
        sel = (t >= s0 - 1e-9) & (t < s1 - 1e-9)
        if not np.any(sel):
            continue
        cs = {k: state.Curve() for k in keys}
        first = True
        for (tk, T, az, r, h, lens, f, mode) in POSES:
            if not (s0 - 1e-9 <= tk < s1 - 1e-9):
                continue
            for k, v in zip(keys, (T[0], T[1], T[2], az, r, h, lens, f)):
                cs[k].key(tk, v, "step" if first else mode)
            first = False
        for k in keys:
            out[k][sel] = _gauss(cs[k](t[sel]), CAM_SMOOTH * FPS)
    T = np.stack([out["tx"], out["ty"], out["tz"]], 1)
    az = np.radians(out["az"])
    eye = T + np.stack([out["r"] * np.sin(az), -out["r"] * np.cos(az), out["h"]], 1)
    return eye, T, out["lens"], out["f"], out["az"]


def points_in_frame(pts_fn, eye, tgt, lens, sensor=36.0, aspect=16 / 9, margin=1.06, occluder=None):
    """Per frame: does any of pts_fn(i) (N,3) project inside the frame (frustum) and,
    if occluder(i, P) (P: (..., 3) points -> bool inside a solid) is given, is not
    hidden behind that solid (ray sampled at 24 points)."""
    out = np.zeros(len(eye), bool)
    up = np.array([0.0, 0.0, 1.0])
    s = np.linspace(0.04, 0.96, 24)
    for i in range(len(eye)):
        pts = pts_fn(i)
        f = tgt[i] - eye[i]
        f /= np.linalg.norm(f)
        rgt = np.cross(f, up)
        rgt /= np.linalg.norm(rgt)
        u = np.cross(rgt, f)
        d = pts - eye[i]
        z = d @ f
        x = d @ rgt
        y = d @ u
        tx = 0.5 * sensor / lens[i] * margin
        ty = tx / aspect
        ok = (z > 0.02) & (np.abs(x) < tx * z) & (np.abs(y) < ty * z)
        if occluder is not None and np.any(ok):
            q = pts[ok]
            ray = eye[i][None, None, :] + s[None, :, None] * (q - eye[i])[:, None, :]
            hidden = np.any(occluder(i, ray), axis=1)
            ok_idx = np.nonzero(ok)[0]
            ok[ok_idx[hidden]] = False
        out[i] = bool(np.any(ok))
    return out


# ===========================================================================
# Build helpers
# ===========================================================================

def _ease_window(t, t0, t1):
    return kin.smoothstep(t0, t1, t)


def _remove(objs):
    for ob in objs:
        if ob is None:
            continue
        try:
            for c in list(ob.children):
                c.parent = None
            me = ob.data if ob.type == "MESH" else None
            bpy.data.objects.remove(ob, do_unlink=True)
            if me is not None and me.users == 0:
                bpy.data.meshes.remove(me)
        except ReferenceError:
            pass


ENGINE_KEEP = ("block", "head", "head_gasket", "cam_cover", "coils", "oil_pan", "timing_cover", "intake_manifold",
               "fuel_rail", "exhaust_manifold", "oil_filter", "alternator", "water_pump", "idler_arm",
               "accessory_belt", "wp_pulley", "belt_idler", "alt_pulley", "damper", "flywheel", "ring_gear")
GEARBOX_KEEP = ("input_shaft", "input_gear", "dogs_4", "cone_4", "brg_input_inner", "brg_input_outer",
                "brg_input_rolling")
HYDRAULICS = ("slave_cylinder", "slave_pushrod", "line_fittings", "hose_bracket", "master_cylinder",
              "master_piston", "pedal", "pushrod", "return_spring", "pedal_box")


def _firewall_patch(B, x=(-0.70, -0.28), z=(0.42, 0.90), y=(-0.47, -0.43)):
    """A piece of the body's firewall around the clutch master cylinder / pedal box
    (the underbody surface clipped to a box with bisect planes)."""
    import bmesh
    ub = B.parts["underbody"]
    bm = bmesh.new()
    bm.from_mesh(ub.data)
    bm.transform(ub.matrix_world)
    for axis, (lo, hi) in ((0, x), (1, y), (2, z)):
        no = [0.0, 0.0, 0.0]
        no[axis] = 1.0
        co = [0.0, 0.0, 0.0]
        co[axis] = lo
        geom = bm.verts[:] + bm.edges[:] + bm.faces[:]
        bmesh.ops.bisect_plane(bm, geom=geom, dist=1e-6, plane_co=co, plane_no=no, clear_inner=True)
        co[axis] = hi
        geom = bm.verts[:] + bm.edges[:] + bm.faces[:]
        bmesh.ops.bisect_plane(bm, geom=geom, dist=1e-6, plane_co=co, plane_no=no, clear_outer=True)
    me = bpy.data.meshes.new("s03_firewall")
    bm.to_mesh(me)
    bm.free()
    for m in ub.data.materials:
        me.materials.append(m)
    ob = bpy.data.objects.new("s03_firewall", me)
    bpy.context.scene.collection.objects.link(ob)
    rig.set_presentation(ob, 1.0, 0.0)
    return ob


def _add_section(ob, cutter):
    m = ob.modifiers.new("s03_section", "BOOLEAN")
    m.operation = "DIFFERENCE"
    m.object = cutter
    m.solver = "MANIFOLD"
    m.material_mode = "TRANSFER"
    return m


def _bake_mod_toggle(ob, mname, frames, on):
    on = np.asarray(on, float)
    rig.bake_channel(ob, f'modifiers["{mname}"].show_render', -1, frames, on, "CONSTANT")
    rig.bake_channel(ob, f'modifiers["{mname}"].show_viewport', -1, frames, on, "CONSTANT")
    ob.modifiers[mname].show_render = bool(on[0])
    ob.modifiers[mname].show_viewport = bool(on[0])


def _bake_compressed(ob, path, frames, vals, interp="LINEAR"):
    v = np.asarray(vals, dtype=float)
    keep = np.ones(len(v), bool)
    if len(v) > 2:
        if interp == "CONSTANT":
            keep[1:] = v[1:] != v[:-1]
        else:
            same_prev = np.r_[False, v[1:] == v[:-1]]
            same_next = np.r_[v[:-1] == v[1:], False]
            keep = ~(same_prev & same_next)
    rig.bake_channel(ob, path, -1, np.asarray(frames)[keep], v[keep], interp)


def _fade(objs, frames, op, glow=None):
    op = np.round(np.clip(op, 0.0, 1.0), 4)
    for o in objs:
        if o is None or o.type != "MESH":
            continue
        if "cv_opacity" not in o:
            o["cv_opacity"] = 1.0
        _bake_compressed(o, '["cv_opacity"]', frames, op)
        hide = (op < 0.02).astype(float)
        _bake_compressed(o, "hide_render", frames, hide, "CONSTANT")
        _bake_compressed(o, "hide_viewport", frames, hide, "CONSTANT")
        if glow is not None:
            _bake_compressed(o, '["cv_glow"]', frames, glow)


def _glow(objs, frames, g):
    g = np.round(np.clip(g, 0.0, 1.0), 4)
    for o in objs:
        if o is None or o.type != "MESH":
            continue
        if "cv_glow" not in o:
            o["cv_glow"] = 0.0
        _bake_compressed(o, '["cv_glow"]', frames, g)


def _smooth_series(x, sigma_frames):
    return _gauss(np.asarray(x, float), sigma_frames)


def _pulse(t, t_on, rise=0.5, hold=1.0, fall=1.2, peak=GLOW_PART, rest=0.0):
    up = kin.smoothstep(t_on, t_on + rise, t)
    dn = kin.smoothstep(t_on + rise + hold, t_on + rise + hold + fall, t)
    return up * (peak - (peak - rest) * dn)


# ===========================================================================
# build
# ===========================================================================

def build(quality: str) -> scenebase.SceneBuild:
    detail = "low" if quality == "preview" else "high"
    sc = scenebase.new_scene(SCENE_ID)
    studio = lighting.setup_studio("dark", center=(-0.10, -0.46, 0.42), size=0.7)
    lighting.setup_color_management(sc)

    # ---------------- drivetrain state ----------------------------------
    P = make_program()
    track = P.run()
    t = track.t
    n = track.n
    fr = track.frames

    # ---------------- assemblies ------------------------------------------
    E = ENG.build({"cutaways": ["none"], "detail": detail, "gas": False})
    C = CL.build({"cutaway": ["half"], "detail": detail, "section_rotating": True, "section_side": -1})
    GB = GBX.build({"cutaway": "none", "detail": detail})
    B = BODY.build({"detail": "low"})
    firewall = _firewall_patch(B)
    _remove([o for o in B.objects() if o is not B.root and o.type == "MESH"])

    # ---------------- presentation arrays ---------------------------------
    def ex_part(k):
        t0 = ASM_T0 + k * ASM_STAGGER
        return 1.0 - _ease_window(t, t0, t0 + ASM_DUR)
    ex = {"flywheel": ex_part(0), "disc": ex_part(1), "pressure_plate": ex_part(2), "diaphragm_spring": ex_part(3),
          "cover": ex_part(4), "release_bearing": ex_part(5), "shaft": ex_part(5)}
    housing = _ease_window(t, *HOUSING_IN)
    section_on = (t >= SWEEP[0] - 0.05).astype(float)
    hyd = _ease_window(t, *HYD_IN)
    pulse = np.where((t >= PULSE[0] - 0.3) & (t <= PULSE[1] + 0.4),
                     (t - PULSE[0]) / (PULSE[1] - PULSE[0]), -1.0)

    E.drive(track, {"explode": ex["flywheel"]})
    C.drive(track, {"variant": "half", "housing": housing, "section": section_on, "line_pulse": pulse,
                    "line_pulse_width": 0.13})
    GB.drive(track, {})
    # staggered explode: re-bake each clutch carrier with its own factor
    for key, off in C.explode.items():
        car = C.parts[C.meta["explode_carriers"][key]]
        rig.bake_channel(car, "location", 1, fr, off[1] * ex[key])

    # engine: keep the exterior + flywheel; gearbox: keep the input shaft group
    _remove([o for k, o in E.parts.items() if k not in ENGINE_KEEP and o.type == "MESH"])
    gb_keep = [GB.parts[k] for k in GEARBOX_KEEP]
    shaft_ex = rig.empty("s03_shaft_ex", parent=GB.root, size=0.02)
    bpy.context.view_layer.update()          # parent_keep needs evaluated world matrices
    for ob in (GB.parts["ex_input_shaft"], GB.parts["ex_input_gear"], GB.parts["brg_input_inner"],
               GB.parts["brg_input_outer"], GB.parts["brg_input_rolling"]):
        rig.parent_keep(ob, shaft_ex)
    _remove([o for k, o in GB.parts.items() if o.type == "MESH" and k not in GEARBOX_KEEP])
    rig.bake_channel(shaft_ex, "location", 1, fr, OFF_SHAFT * ex["shaft"])

    # section: animated cutter (presentation) + the same cut on the flywheel and ring gear
    cutter = C.parts["section_cutter"]
    sweep = _ease_window(t, *SWEEP)
    cut_x = -0.175 * (1.0 - sweep)
    rig.bake_channel(cutter, "location", 0, fr, cut_x)
    fly_mods = []
    for k in ("flywheel", "ring_gear"):
        ob = E.parts[k]
        _add_section(ob, cutter)
        _bake_mod_toggle(ob, "s03_section", fr, section_on)
        fly_mods.append(ob)

    # hydraulics / pedal box / firewall: only for the release + take-off
    hyd_objs = [C.parts[k] for k in HYDRAULICS] + [C.parts[k] for k in C.meta["hydraulic_segments"]]
    _fade(hyd_objs, fr, hyd)
    if quality == "preview":
        firewall.hide_render = True
    else:
        _fade([firewall], fr, 0.22 * hyd)

    # ---------------- glow -------------------------------------------------
    gl = {}
    # parts beat: each part glows briefly as it is named
    gl["flywheel"] = _pulse(t, wt("parts", "flywheel") - 0.1)
    gl["disc"] = _pulse(t, wt("parts", "friction") - 0.1)
    gl["plate"] = _pulse(t, wt("parts", "pressure") - 0.1)
    gl["spring"] = _pulse(t, wt("parts", "diaphragm") - 0.1)
    # engaged: the power path lights up as the narration follows it
    t_sp, t_disc = wt("engaged", "spring"), wt("engaged", "disc")
    t_fly, t_in = wt("engaged", "flywheel"), wt("engaged", "input")
    hold_end = bend("engaged") - 0.4
    def path(t_on):
        return kin.smoothstep(t_on, t_on + 0.6, t) * (1.0 - kin.smoothstep(hold_end, hold_end + 1.2, t)) * GLOW_PATH
    gl["spring"] = np.maximum(gl["spring"], path(t_sp))
    gl["plate"] = np.maximum(gl["plate"], path(t_sp + 0.4))
    gl["disc"] = np.maximum(gl["disc"], path(t_disc))
    gl["flywheel"] = np.maximum(gl["flywheel"], path(t_fly))
    gl["shaft"] = path(t_in)
    # take-off: the power path glows with the torque the clutch actually passes (Track)
    tq = np.clip(np.abs(track.clutch_torque) / 100.0, 0.0, 1.0) * kin.smoothstep(T_BITE, T_BITE + 0.3, t)
    tq = _smooth_series(tq, 6) * GLOW_TORQUE * (1.0 - kin.smoothstep(DUR - 0.8, DUR, t))
    for k in ("flywheel", "spring", "plate", "disc", "shaft"):
        gl[k] = np.maximum(gl[k], tq)
    _glow([E.parts["flywheel"], E.parts["ring_gear"]], fr, gl["flywheel"])
    _glow([C.parts["disc"], C.parts["damper_springs"]], fr, gl["disc"])
    _glow([C.parts["pressure_plate"], C.parts["straps"]], fr, gl["plate"])
    _glow([C.parts["diaphragm_spring"], C.parts["fulcrum"], C.parts["cover"]], fr, gl["spring"])
    _glow(gb_keep[:4], fr, gl["shaft"])

    # ---------------- camera + lights + motion blur -----------------------
    eye, tgt, lens, fstop, az = camera_samples(t)
    CP = CAM.CameraPath(name="cam_s03", lens=45.0, fstop=5.6, clip=(0.01, 60.0))
    for i in range(n):
        CP.key(t[i], eye=tuple(eye[i]), target=tuple(tgt[i]), lens=float(lens[i]), fstop=float(fstop[i]),
               mode="linear")
    cam = CP.bake(fr, FPS)
    # keep the cut out of every shutter interval: hold the old pose until just before
    # the half-frame boundary, the new one from just after it
    _hold_cut_keys(cam, track.frame_at(T_CUT))
    rig.bake_channel(studio["rig"], "rotation_euler", 2, fr,
                     np.radians(np.unwrap(az, period=360.0) + KEY_OFFSET - 225.0))
    shut = shutter_curve(t)
    rig.bake_channel(sc, "render.motion_blur_shutter", -1, fr, np.maximum(shut, 0.02))

    # ---------------- validation ----------------------------------------
    th_e, th_in = track.theta_e, track.theta_in
    slow = track.slowmo

    def ring_pts(i):
        ang = np.linspace(0, TAU, 48, endpoint=False)
        y0 = S.Y_FLYWHEEL_FRONT + OFF_FLY * ex["flywheel"][i]
        pts = [np.stack([rr * np.cos(ang), np.full_like(ang, y), S.Z_CRANK + rr * np.sin(ang)], 1)
               for y in (y0, y0 - 0.012) for rr in (0.150, 0.137)]
        pts = np.concatenate(pts)
        if section_on[i] > 0.5:          # the half x < cutter plane is cut away
            pts = pts[pts[:, 0] >= cut_x[i] - 1e-6]
        return pts
    def flywheel_solid(i, Pq):
        """the flywheel disc (solid to r 0.135) occludes what is behind it; with the
        section on only its x >= cutter half exists; it moves with the explode."""
        dy = OFF_FLY * ex["flywheel"][i]
        x, y, zz = Pq[..., 0], Pq[..., 1] - dy, Pq[..., 2] - S.Z_CRANK
        inside = (x * x + zz * zz <= 0.135 ** 2) & (y <= S.Y_FLYWHEEL_FRONT - 0.003) & (y >= S.Y_FLYWHEEL_FACE + 0.002)
        if section_on[i] > 0.5:
            inside &= x >= cut_x[i]
        return inside
    ring_vis = points_in_frame(ring_pts, eye, tgt, lens, occluder=flywheel_solid)
    ring_vis = ring_vis | np.r_[ring_vis[1:], False] | np.r_[False, ring_vis[:-1]]

    def blurred(ang, pitch):
        """frames whose features are fully smeared (shutter x pitch/frame >= 1)."""
        p = np.abs(np.diff(ang, prepend=ang[0])) / pitch
        return (shut * p) >= 1.0

    def mask(vis, ang, pitch):
        return vis & ~blurred(ang, pitch)

    allv = np.ones(n, bool)
    blur_on = shut >= 0.9           # long-shutter take-off frames (relaxed, brief section 3)
    gear_ang = track.gb("input_gear")
    alias = {
        "flywheel ring gear 132T": (th_e, TAU / 132, mask(ring_vis, th_e, TAU / 132)),
        "diaphragm fingers 18": (th_e, TAU / 18, mask(~blur_on, th_e, TAU / 18)),
        "cover / flywheel bolts 6": (th_e, TAU / 6, mask(~blur_on, th_e, TAU / 6)),
        "crank bolts 8": (th_e, TAU / 8, mask(~blur_on, th_e, TAU / 8)),
        "disc facing rivets 24": (th_in, TAU / 24, mask(~blur_on, th_in, TAU / 24)),
        "disc hub / input splines 23": (th_in, TAU / 23, mask(~blur_on, th_in, TAU / 23)),
        "disc damper springs 6": (th_in, TAU / 6, mask(~blur_on, th_in, TAU / 6)),
        "input gear 26T": (gear_ang, TAU / 26, mask(~blur_on, gear_ang, TAU / 26)),
        "input gear dog ring 32": (gear_ang, TAU / 32, mask(~blur_on, gear_ang, TAU / 32)),
    }
    track.validate(aliasing=alias)

    # ---------------- labels --------------------------------------------
    L = Labels()
    A = C.anchors
    t_asm_end = ASM_T0 + 5 * ASM_STAGGER + ASM_DUR
    L.add("engine", "Engine", (E.parts["block"], (-0.10, -0.24, 0.10)), wt("parts", "engine"),
          wt("parts", "clutch") + 0.6, offset=(-0.07, -0.08), style="dim", occlusion=False)
    L.add("gearbox", "Gearbox side", (GB.parts["input_gear"], (0.0, 0.0, 0.036)), wt("parts", "gearbox"),
          wt("parts", "clutch") + 0.6, offset=(0.06, -0.08), style="dim", occlusion=False)
    def fly_anchor(z_up, x=0.0):
        """flywheel rim point that follows the explode but not the spin"""
        def f(frame):
            i = int(np.clip(frame - 1, 0, n - 1))
            return (x, S.Y_FLYWHEEL_FRONT - 0.015 + OFF_FLY * ex["flywheel"][i], S.Z_CRANK + z_up)
        return f
    L.add("flywheel", "Flywheel", fly_anchor(-0.139), wt("parts", "flywheel"),
          t_asm_end - 0.3, offset=(-0.05, 0.10), occlusion=False)
    L.add("disc", "Friction disc", A["disc"], wt("parts", "friction"), t_asm_end - 0.3, offset=(-0.04, -0.12),
          occlusion=False)
    L.add("plate", "Pressure plate", A["pressure_plate"], wt("parts", "pressure"), t_asm_end - 0.3,
          offset=(0.05, -0.12), occlusion=False)
    L.add("spring", "Diaphragm spring", A["diaphragm_spring"], wt("parts", "diaphragm"), t_asm_end - 0.3,
          offset=(0.07, -0.05), occlusion=False)
    # splines
    L.add("splines", "Splines", (C.parts["disc_ex"], (-0.004, CL.HUB_YC - 0.006, 0.0125)), wt("splines", "splined"),
          bend("splines") - 0.2, offset=(-0.08, -0.10), occlusion=False)
    L.add("input_shaft", "Input shaft", (shaft_ex, (-0.0115, -0.386, 0.004)),
          wt("splines", "input shaft"), bend("splines") - 0.2, offset=(0.07, 0.10), occlusion=False)
    # release chain
    L.add("pedal", "Clutch pedal", A["pedal"], wt("release", "pedal"), 35.4, offset=(-0.06, 0.08), occlusion=False)
    L.add("master", "Master cylinder", A["master_cylinder"], wt("release", "master"), 35.4, offset=(-0.04, -0.10),
          occlusion=False)
    L.add("slave", "Slave cylinder", A["slave_cylinder"], wt("release", "slave"), 37.0, offset=(0.05, 0.10),
          occlusion=False)
    L.add("fork", "Release fork", A["fork"], wt("release", "fork"), 40.2, offset=(-0.06, 0.10), occlusion=False)
    L.add("bearing", "Release bearing", A["release_bearing"], wt("release", "bearing"), 40.2, offset=(0.06, -0.10),
          occlusion=False)
    L.add("fingers", "Fingers", A["fingers"], wt("release", "fingers"), 42.0, offset=(-0.08, -0.06),
          occlusion=False)
    plate_face = (C.parts["plate_ex"], (0.0, CL._ly(CL.Y_PF) - 0.008, 0.100))
    L.add("pplate", "Pressure plate", plate_face, wt("release", "pressure"), T_CUT - 0.1,
          offset=(0.08, -0.08), occlusion=False)
    L.add("disc2", "Disc", A["disc"], wt("release", "disc") - 0.2, T_CUT - 0.1, offset=(-0.08, 0.06),
          occlusion=False)
    # slip
    L.add("disc3", "Friction disc", A["disc"], wt("slip", "disc") + 0.6, 58.8, offset=(-0.05, -0.12), occlusion=False)
    L.add("fly3", "Flywheel", fly_anchor(0.09), wt("slip", "flywheel"), 58.8,
          offset=(-0.06, 0.15), occlusion=False)
    L.add("plate3", "Pressure plate", A["pressure_plate"], wt("slip", "pressure"), 58.8, offset=(0.09, -0.06),
          occlusion=False)

    # ---------------- HUD -----------------------------------------------
    H = Hud(track)
    H.add("fade", 0.0, 0.8, fade=0.0, color="black", alpha=lambda i: float(max(0.0, 1.0 - t[i] / 0.6)))
    H.add("fade", DUR - 0.6, DUR + 1.0, fade=0.0, color="black",
          alpha=lambda i: float(min(1.0, max(0.0, (t[i] - (DUR - 0.55)) / 0.5))))
    H.add("section_title", 0.4, 4.4, number=3, title="The clutch")
    H.add("slowmo", 0.5, DUR, factor=lambda i: float(track.slowmo[i]))
    t_hud = bstart("engaged")
    H.add("status", t_hud, DUR, text=lambda i: str(track.status[i]))
    H.add("gear", t_hud, DUR, value=lambda i: str(track.gear[i]))
    H.add("pedal", t_hud, DUR, value=lambda i: round(float(track.pedal[i]), 3))
    H.add("speed", T_CUT, DUR, value_kmh=lambda i: round(float(track.v_kmh[i]), 1))
    lift = track.clutch_plate_lift

    def rows(i):
        r = [["Engine", f"{fmt_rpm(track.rpm_e[i])} rpm"],
             ["Disc + input shaft", f"{fmt_rpm(track.rpm_in[i])} rpm"]]
        if 30.6 <= t[i] < T_CUT:
            r.append(["Plate lift", f"{lift[i] * 1000:.1f} mm"])
        return r
    H.add("readouts", t_hud, DUR, rows=rows)

    # live section booleans only in the render depsgraph: scene.frame_set (labels, baking)
    # then never re-evaluates them (labels use no occlusion rays)
    for ob in bpy.data.objects:
        for md in ob.modifiers:
            if md.type == "BOOLEAN" and md.name in ("clu_section", "s03_section"):
                rig.bake_channel(ob, f'modifiers["{md.name}"].show_viewport', -1, fr[:1], np.zeros(1), "CONSTANT")
                md.show_viewport = False
    if quality == "preview":       # Workbench shadow volumes are slow on llvmpipe (motion/framing only)
        for ob in bpy.data.objects:
            if ob.type == "MESH":
                ob.display.show_shadows = False
    sb = scenebase.SceneBuild(SCENE_ID, track, cam, L, H, motion_blur=True, shutter=SHUT_REL,
                              preview_hide=(firewall,))
    sb.extra.update(engine=E, clutch=C, gearbox=GB, studio=studio, ring_visible=ring_vis, shutter=shut,
                    explode=ex, program=P)
    return sb


def _hold_cut_keys(cam, f_cut):
    """Insert hold keys either side of the half-frame before a camera cut so the
    motion-blur shutter (centred, < 1 frame) never interpolates across the cut."""
    targets = []
    for ob in (cam, cam.data):
        ad = ob.animation_data
        if ad is None or ad.action is None:
            continue
        act = ad.action
        fcs = []
        try:
            for layer in act.layers:
                for strip in layer.strips:
                    for cb in strip.channelbags:
                        fcs += list(cb.fcurves)
        except AttributeError:
            fcs = list(getattr(act, "fcurves", []))
        targets += fcs
    for fc in targets:
        old = fc.evaluate(f_cut - 1)
        new = fc.evaluate(f_cut)
        fc.keyframe_points.insert(f_cut - 0.52, old, options={"FAST"})
        fc.keyframe_points.insert(f_cut - 0.48, new, options={"FAST"})
        for kp in fc.keyframe_points:
            kp.interpolation = "LINEAR"
        fc.update()
