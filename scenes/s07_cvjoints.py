"""s07 Driveshafts and CV joints (41.5 s): why the joints, Rzeppa outer joint, tripod plunge, the car moves.

Road studio (continuity with the s08 recap), whole car on the car lane; the camera rides with
the car around the rear-right corner, then pulls out and lets the car drive away.  The body is
hidden for the close-ups (it would clip the tyre at full bump) and fades back in for `moves`.

Drivetrain state (one Program, no cuts)
  * 1st gear, clutch engaged, straight line at 10 km/h (engine 1245 rpm, wheels 87 rpm);
    slow motion x12 (tyre tread 64 pitches: x11.05 is the limit at 10 km/h).
  * Body bounce (heave): every wheel centre moves by the same s(t) relative to the body
    (+-0.06 m = spec SUSPENSION_TRAVEL) while the tyres stay on the road; the car root is
    lowered by s (z = -s), and the camera rides with the body, so the wheel moves up and
    down while the differential stays put in the picture.  s(t): at rest, a half-cosine rise
    to full bump at 5.9 s ("moves up"), then a 9 s video period (0.75 s real time = 1.33 Hz,
    the body-bounce frequency of a car) - droop 10.4 / 19.4 / 28.4 s, bump 14.9 / 23.9 /
    32.9 s - and a half-cosine settle to ride height by 35.3 s, before `moves`.
    The driveshaft geometry (shaft angle, Rzeppa cage in the bisecting plane, tripod plunge
    p = L - sqrt(L^2 - s^2) >= 0 on bump AND droop, 3.78 mm at +-60 mm) is the wheels
    assembly's exact kinematics.
  * moves: slow motion ramps x12 -> real time (35.3-38.6 s, log-linear), then the car pulls
    away 10 -> 20 km/h in 1st (38.2-41.5 s); Cycles motion blur (shutter keyed: 0.25 frame in
    slow motion, 0.5 in real time).

Sections: the joints spin at 87 rpm (x12: 52 deg/s), so a cut made in the part's own frame
(the assembly's 'cv_cut' / 'tripod_cut' pieces) would turn away from the camera every 7 s.
These shots therefore use a section plane FIXED in the car (like the clutch's live section):
live Manifold booleans (~0.05 s/frame) on the bell, boot and clamps of the outer joint (cutter
rides with the RR corner) and on the tulip, boot and clamps of the inner joint (cutter fixed
to the car), removing the rear (-Y) half.  The cutter plane sweeps in from behind the part
(section "wipes" open) and back out to close it.  Balls, cage, inner race, spider and
rollers stay whole, as on a real cutaway.  Section faces get the section_cut material.

The RR coil-over (between a camera behind the car and the outer joint) is faded out while
the joints are shown.  The plunge is shown at true scale (no magnification) with a live
"Plunge x.x mm" readout next to the diff-to-hub distance (tulip centre on the diff to the
Rzeppa centre on the hub, sqrt(L^2 + s^2): 478.0 -> 481.7 mm at +-60 mm; the shaft between the
joint centres is rigid, so the spider slides out p = L - sqrt(L^2 - s^2), 3.78 mm); the joint
angle (max 7.2 deg) is read out too.
"""
from __future__ import annotations

import math

import bpy
import numpy as np
from mathutils import Vector

from carviz import camera as CAM
from carviz import lighting, materials, rig, scenebase, state, timeline
from carviz import meshutil as MU
from carviz import spec as S
from carviz.assemblies import car as CAR
from carviz.labels import Hud, Labels, fmt_rpm

SCENE_ID = "s07"
SC = timeline.scene(SCENE_ID)
BT = {b.id: b for b in SC.beats}
FPS = S.FPS
DUR = SC.dur
TAU = 2.0 * math.pi


def bstart(b):
    return BT[b].start


def bend(b):
    return BT[b].end


def wt(b, word, occ=1):
    return BT[b].word_time(word, occ)


def _ss(e0, e1, x):
    u = np.clip((np.asarray(x, dtype=float) - e0) / (e1 - e0), 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


# ---------------------------------------------------------------------------
# Drivetrain plan
# ---------------------------------------------------------------------------
GEAR = 1
V_KMH = 10.0                      # cruise (engine 1245 rpm in 1st)
V_END = 20.0                      # pull-away target in `moves`
SLOW = 1.0 / 12.0                 # tyre tread (64 pitches) needs >= x11.05 at 10 km/h
T_RAMP = (35.3, 38.6)             # slow motion -> real time
T_ACCEL = (38.2, 41.5)            # 10 -> 20 km/h
SUSP_PERIOD = 9.0                 # video s: 0.75 s real time at x12 (1.33 Hz, a natural ride motion)
SUSP_RISE = (4.0, 5.9)            # from rest up to the first bump ("moves up")
SUSP_LAST = 32.9                  # last bump peak (plunge beat) ...
SUSP_SETTLE = 35.3                # ... then the wheel settles back to ride height (tyre on the road)


def susp_rr(t):
    """RR wheel-centre offset (m): rest, a half-cosine rise to full bump at 5.9 s, a 9 s
    (video) sine through bump/droop (droop 10.4, 19.4, 28.4 s; bump 14.9, 23.9, 32.9 s),
    then a half-cosine settle to ride height by 35.3 s.  C1 everywhere."""
    t = np.asarray(t, dtype=float)
    A = S.SUSPENSION_TRAVEL
    t0, t1 = SUSP_RISE
    rise = A * 0.5 * (1.0 - np.cos(np.pi * np.clip((t - t0) / (t1 - t0), 0.0, 1.0)))
    osc = A * np.cos(TAU * (t - t1) / SUSP_PERIOD)
    settle = A * 0.5 * (1.0 + np.cos(np.pi * np.clip((t - SUSP_LAST) / (SUSP_SETTLE - SUSP_LAST), 0.0, 1.0)))
    return np.where(t < t1, rise, np.where(t < SUSP_LAST, osc, settle))


def slowmo_curve():
    c = state.Curve(SLOW)
    c.key(0.0, SLOW, "step")
    c.key(T_RAMP[0], SLOW, "linear")
    n = 40
    for k in range(1, n + 1):
        u = k / n
        w = u * u * (3 - 2 * u)
        c.key(T_RAMP[0] + (T_RAMP[1] - T_RAMP[0]) * u, math.exp(math.log(SLOW) * (1.0 - w)), "linear")
    return c


def build_program():
    P = state.Program(SCENE_ID)
    P.slowmo = slowmo_curve()
    P.start_in_gear(GEAR)
    P.speed_kmh.key(0.0, V_KMH, "step").key(T_ACCEL[0], V_KMH, "linear").key(T_ACCEL[1], V_END, "ease")
    rpm0, rpm1 = S.engine_rpm_at(V_KMH, GEAR), S.engine_rpm_at(V_END, GEAR)
    # the driver holds a little throttle while cruising, more while pulling away
    P.throttle_rpm.key(0.0, rpm0 + 100.0, "step").key(T_ACCEL[0] - 0.3, rpm0 + 100.0, "linear")
    P.throttle_rpm.key(T_ACCEL[0] + 0.3, rpm0 + 450.0, "ease").key(T_ACCEL[1] - 0.6, rpm1 + 300.0, "linear")
    P.throttle_rpm.key(T_ACCEL[1], rpm1 + 100.0, "ease")
    # body bounce (heave): every wheel moves by the same offset relative to the body while the
    # tyres stay on the road; the car root is lowered by that offset (build(): car root z = -s)
    for t in np.arange(0.0, DUR + 0.05, 0.1):
        for w in ("FL", "FR", "RL", "RR"):
            P.susp[w].key(float(t), float(susp_rr(t)), "cubic")
    return P


# ---------------------------------------------------------------------------
# Camera plan, CAR frame (the camera rides with the car until it lets the car go)
# (time, eye, target, lens, f-stop)
# ---------------------------------------------------------------------------
X_OJ = S.X_WHEEL_HUB              # Rzeppa centre |x|
Y_RA = S.Y_REAR_AXLE
ZW = S.WHEEL_CENTER_Z

POSES = [
    # why: wide on the rear axle from behind-right, then a slow push to the whole RR shaft
    (0.0, (0.40, -4.65, 0.88), (0.30, -2.52, 0.30), 32.0, 8.0),
    (1.2, (0.41, -4.57, 0.86), (0.31, -2.53, 0.30), 32.0, 8.0),
    (5.0, (0.46, -4.05, 0.66), (0.40, -2.60, 0.28), 35.0, 8.0),
    (10.0, (0.48, -3.95, 0.62), (0.41, -2.60, 0.28), 35.0, 8.0),
    # rzeppa: in to the outer joint (section opens 11.2-12.4); rides with the wheel
    (12.2, (0.50, -2.96, 0.45), (0.650, -2.62, 0.312), 50.0, 8.0),
    (19.0, (0.52, -2.955, 0.44), (0.652, -2.62, 0.312), 50.0, 8.0),
    (26.2, (0.54, -2.96, 0.43), (0.652, -2.62, 0.312), 50.0, 8.0),
    # plunge: track along the shaft to the tripod (section opens 27.2-28.3), seen from behind
    (28.2, (0.28, -3.03, 0.40), (0.195, -2.62, 0.322), 44.0, 8.0),
    (33.3, (0.27, -3.04, 0.395), (0.193, -2.62, 0.322), 46.0, 8.0),
    # moves: pull out past the wheel to the tyre on the road, then wide; the car drives off
    (35.4, (1.40, -4.30, 0.38), (0.74, -2.62, 0.22), 35.0, 8.0),
    (37.0, (1.95, -6.40, 0.90), (0.50, -2.30, 0.45), 35.0, 8.0),
    (39.0, (2.60, -8.40, 1.35), (0.05, -1.60, 0.60), 40.0, 8.0),
    (DUR, (2.70, -8.80, 1.45), (0.0, -1.40, 0.62), 55.0, 8.0),
]
T_DETACH = (38.8, 41.0)          # the camera's carrier slows from the car's speed ...
DETACH_KEEP = 0.25               # ... to this fraction of it (the car pulls away from the camera)
CORNER_FOLLOW = ((10.4, 12.0), (26.4, 28.0))   # blend in / out of riding with the RR wheel
CAM_SMOOTH = 0.25                # s, Gaussian low-pass on the keyed path


def _gauss(x, sigma_frames):
    r = int(3 * sigma_frames)
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma_frames) ** 2)
    k /= k.sum()
    xp = np.r_[np.full(r, x[0]), x, np.full(r, x[-1])]
    return np.convolve(xp, k, mode="valid")


def camera_car(t):
    """Per-frame eye/target (car frame), lens, f-stop."""
    cs = {k: state.Curve() for k in ("ex", "ey", "ez", "tx", "ty", "tz", "lens", "f")}
    for i, (tk, e, g, lens, f) in enumerate(POSES):
        mode = "step" if i == 0 else "cubic"
        for k, v in zip(("ex", "ey", "ez", "tx", "ty", "tz", "lens", "f"), (*e, *g, lens, f)):
            cs[k].key(tk, v, mode)
    sig = CAM_SMOOTH * FPS
    p = {k: _gauss(c(t), sig) for k, c in cs.items()}
    eye = np.stack([p["ex"], p["ey"], p["ez"]], 1)
    tgt = np.stack([p["tx"], p["ty"], p["tz"]], 1)
    return eye, tgt, p["lens"], p["f"]


def camera_world(track):
    """World eye/target: car-frame path + car position; after T_DETACH the camera's
    carrier decelerates to a stop while the target stays on the car."""
    t = track.t
    eye_c, tgt_c, lens, fstop = camera_car(t)
    # outer-joint close-up: the camera rides with the RR wheel (corner frame = body frame + s
    # vertically), so the joint stays framed and the shaft visibly swings about it
    wc = (_ss(*CORNER_FOLLOW[0], t) - _ss(*CORNER_FOLLOW[1], t)) * track.susp_RR
    eye_c = eye_c + np.stack([0 * wc, 0 * wc, wc], 1)
    tgt_c = tgt_c + np.stack([0 * wc, 0 * wc, wc], 1)
    assert np.max(np.abs(track.car_heading)) < 1e-9, "s07 drives straight"
    pos = np.stack([track.car_x, track.car_y, -track.susp_RR], 1)       # car root (heave: z = -s)
    g = 1.0 - (1.0 - DETACH_KEEP) * _ss(*T_DETACH, t)
    carrier = np.zeros_like(pos)
    carrier[0] = pos[0]
    gm = 0.5 * (g[1:] + g[:-1])
    carrier[1:] = pos[0] + np.cumsum((pos[1:] - pos[:-1]) * gm[:, None], axis=0)
    return carrier + eye_c, pos + tgt_c, lens, fstop, eye_c, tgt_c, pos


def box_visible(lo, hi, eye, tgt, lens, margin=1.3, sensor=36.0, aspect=16 / 9, grid=4):
    """Per frame: does the axis-aligned box lo..hi (car frame) reach into the (enlarged)
    view frustum?  eye/tgt (n,3) car frame, lens (n,).  Sampled box points + camera-in-box."""
    g = [np.linspace(lo[k], hi[k], grid) for k in range(3)]
    pts = np.array(np.meshgrid(*g, indexing="ij")).reshape(3, -1).T          # (m,3)
    f = tgt - eye
    f /= np.linalg.norm(f, axis=1)[:, None]
    r = np.cross(f, np.array([0.0, 0.0, 1.0]))
    r /= np.linalg.norm(r, axis=1)[:, None]
    u = np.cross(r, f)
    d = pts[None, :, :] - eye[:, None, :]                                     # (n,m,3)
    z = np.einsum("nmk,nk->nm", d, f)
    x = np.einsum("nmk,nk->nm", d, r)
    y = np.einsum("nmk,nk->nm", d, u)
    tx = (0.5 * sensor / lens * margin)[:, None]
    ty = tx / aspect
    vis = np.any((z > 0.02) & (np.abs(x) < tx * z) & (np.abs(y) < ty * z), axis=1)
    inside = np.all((eye >= lo) & (eye <= hi), axis=1)
    return vis | inside


# ---------------------------------------------------------------------------
# Presentation schedule
# ---------------------------------------------------------------------------
OUTER_OPEN = (11.2, 12.4)
OUTER_CLOSE = (33.7, 34.7)
INNER_OPEN = (27.2, 28.3)
INNER_CLOSE = (33.7, 34.7)
SWEEP_FROM = -0.075               # cutter plane start/end offset behind the joint centre (m)
COIL_OUT = (1.2, 2.3)             # RR coil-over fades away (it hides the outer joint)
COIL_IN = (34.6, 35.6)
BODY_IN = (36.0, 38.8)
SHUTTER = (0.25, 0.5)
GAUGE_IN = (28.3, 28.9)           # plunge gauge ticks fade in / out
GAUGE_OUT = (33.1, 33.6)
GAUGE_Z = 0.0605                  # tick centre above the tulip axis (tulip OD 97 mm)
GAUGE_SIZE = (0.0012, 0.0012, 0.013)             # motion-blur shutter (frames): slow motion / real time


def _sweep(t, opn, cls):
    """Cutter plane offset (m along +Y from the joint centre): SWEEP_FROM = closed
    (plane behind the part), 0 = section through the joint centre."""
    return SWEEP_FROM * (1.0 - (_ss(*opn, t) - _ss(*cls, t)))


def _fcurve_find(idb, path, index=0):
    ad = idb.animation_data
    if ad is None or ad.action is None:
        return None
    from bpy_extras import anim_utils
    cb = anim_utils.action_get_channelbag_for_slot(ad.action, ad.action_slot)
    return None if cb is None else cb.fcurves.find(path, index=index)


def _hide_or(ob, frames, hide):
    """OR an extra per-frame hide mask into whatever hide_render keys the object has."""
    hide = np.asarray(hide, bool).copy()
    fc = _fcurve_find(ob, "hide_render")
    if fc is not None:
        hide |= np.array([fc.evaluate(float(f)) > 0.5 for f in frames])
    elif ob.hide_render:
        hide[:] = True
    v = hide.astype(float)
    keep = np.r_[True, v[1:] != v[:-1]]
    fr = np.asarray(frames)[keep]
    rig.bake_channel(ob, "hide_render", -1, fr, v[keep], "CONSTANT")
    rig.bake_channel(ob, "hide_viewport", -1, fr, v[keep], "CONSTANT")


def _section(name, parent, center, targets, offsets, frames, active):
    """Live fixed-plane section: Manifold DIFFERENCE with a half-space box behind the
    plane y = center.y + offset (parent frame); cut faces get section_cut."""
    size = 0.4
    cut = MU.rounded_box(name, (size, size, size), radius=0.0, center=(0.0, -0.5 * size, 0.0),
                         material="section_cut", smooth_angle=None)
    cut.data.shade_flat()
    cut.parent = parent
    cut.location = center
    cut.hide_render = True
    cut.hide_viewport = True
    cut.display_type = "WIRE"
    rig.bake_channel(cut, "location", 1, frames, center[1] + np.asarray(offsets))
    on = np.asarray(active, float)
    keep = np.r_[True, on[1:] != on[:-1]]
    for ob in targets:
        m = ob.modifiers.new("s07_section", "BOOLEAN")
        m.operation = "DIFFERENCE"
        m.object = cut
        m.solver = "MANIFOLD"
        m.material_mode = "TRANSFER"
        for p in ("show_render", "show_viewport"):
            rig.bake_channel(ob, f'modifiers["{m.name}"].{p}', -1, np.asarray(frames)[keep], on[keep], "CONSTANT")
        m.show_render = m.show_viewport = bool(on[0])
    return cut


def _gauge_material(name, rgb, strength):
    """Emissive mark (honours cv_opacity through the shared presentation group)."""
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    em = nt.nodes.new("ShaderNodeEmission")
    em.inputs["Color"].default_value = (*rgb, 1.0)
    em.inputs["Strength"].default_value = strength
    grp = nt.nodes.new("ShaderNodeGroup")
    grp.node_tree = materials.presentation_group()
    nt.links.new(em.outputs[0], grp.inputs["Shader"])
    nt.links.new(grp.outputs["Shader"], out.inputs["Surface"])
    m.diffuse_color = (*rgb, 1.0)
    return m


def _tick(name, parent, loc, size, mat):
    ob = MU.rounded_box(name, size, radius=0.0, material=mat, smooth_angle=None)
    ob.parent = parent
    ob.location = loc
    for a in ("visible_diffuse", "visible_glossy", "visible_transmission", "visible_volume_scatter",
              "visible_shadow"):
        setattr(ob, a, False)                       # camera-only drawing aid
    ob.display.show_shadows = False
    materials.ensure_props(ob)
    return ob


# ---------------------------------------------------------------------------
# build
# ---------------------------------------------------------------------------

def build(quality: str) -> scenebase.SceneBuild:
    sc = scenebase.new_scene(SCENE_ID)
    det = "low" if quality == "preview" else "high"
    C = CAR.build({"engine": {"cutaways": ["none"], "detail": det},
                   "clutch": {"cutaway": ["none"], "detail": det},
                   "gearbox": {"cutaway": "none", "detail": det},
                   "axle": {"cutaways": ["none"], "detail": det},
                   "wheels": {"detail": det},
                   "body": {"detail": det}})
    W = C.sub["wheels"]
    A = C.sub.get("axle")
    studio = lighting.setup_studio("road", follow=C.root, key_azimuth=30.0)
    lighting.setup_color_management(sc)

    # ---------------- drivetrain state ----------------------------------
    P = build_program()
    track = P.run()
    t, n, fr = track.t, track.n, track.frames

    # ---------------- presentation arrays -------------------------------
    body_op = _ss(*BODY_IN, t)
    pres = {"wheels": {}, "body": {"exterior_opacity": body_op, "interior_opacity": body_op,
                                   "underbody_opacity": body_op, "glass_opacity": body_op}}
    C.drive(track, pres)
    rig.bake_channel(C.root, "location", 2, fr, -track.susp_RR)   # body heave: tyres stay on the road
    if quality == "preview":
        # Workbench on llvmpipe: shadow volumes of ~1.5 M triangles cost ~40 s/frame
        for ob in bpy.data.objects:
            if ob.type == "MESH":
                ob.display.show_shadows = False

    # coil-over RR out of the way of the outer joint
    coil = [o for o in W.meta["groups"]["coilover"] if o.name.endswith("_RR")]
    coil_op = 1.0 - _ss(*COIL_OUT, t) + _ss(*COIL_IN, t)
    rig.bake_fade(coil, fr, coil_op)

    # live sections (fixed planes, normal -Y: the rear half is removed)
    WP = W.parts
    F = W.meta["frames"]
    x_ij = W.meta["driveshaft"]["x_inner"]
    off_o = _sweep(t, OUTER_OPEN, OUTER_CLOSE)
    off_i = _sweep(t, INNER_OPEN, INNER_CLOSE)
    act_o = off_o > SWEEP_FROM + 1e-6
    act_i = off_i > SWEEP_FROM + 1e-6
    act_o = act_o | np.r_[act_o[1:], False] | np.r_[False, act_o[:-1]]
    act_i = act_i | np.r_[act_i[1:], False] | np.r_[False, act_i[:-1]]
    cut_o = _section("s07_cut_outer", F["corner_RR"], (X_OJ - S.TRACK_REAR / 2.0, 0.0, 0.0),
                     [WP[k] for k in ("outer_race_RR", "clamp_ob_RR", "boot_outer_RR", "clamp_os_RR")],
                     off_o, fr, act_o)
    cut_i = _section("s07_cut_inner", W.root, (x_ij, Y_RA, ZW),
                     [WP[k] for k in ("tulip_RR", "clamp_ib_RR", "boot_inner_RR", "clamp_is_RR")],
                     off_i, fr, act_i)

    # plunge gauge (drawing aid, true scale): a white tick at the spider centre's ride-height
    # position and an orange tick that follows the spider centre (x_ij + p), just above the
    # tulip in the section plane; their gap IS the plunge
    p_rr = W.meta["kinematics"]["rear_kin"](track["susp_RR"], 1, x_ij)["p"]
    z_tk = ZW + GAUGE_Z
    m_rest = _gauge_material("s07_gauge_rest", (0.85, 0.87, 0.90), 2.5)
    m_now = _gauge_material("s07_gauge_now", (1.0, 0.32, 0.02), 4.0)
    tick_rest = _tick("s07_tick_rest", W.root, (x_ij, Y_RA - 0.004, z_tk), GAUGE_SIZE, m_rest)
    tick_car = rig.empty("s07_tick_carrier", loc=(x_ij, Y_RA - 0.005, z_tk), parent=W.root, size=0.01)
    rig.bake_channel(tick_car, "location", 0, fr, x_ij + p_rr)
    tick_now = _tick("s07_tick_now", tick_car, (0.0, 0.0, 0.0), GAUGE_SIZE, m_now)
    gauge_op = _ss(*GAUGE_IN, t) * (1.0 - _ss(*GAUGE_OUT, t))
    rig.bake_fade([tick_rest, tick_now], fr, gauge_op)

    # ---------------- camera ---------------------------------------------
    eye, tgt, lens, fstop, eye_c, tgt_c, pos = camera_world(track)
    CP = CAM.CameraPath(name="cam_s07", lens=35.0, fstop=8.0, clip=(0.01, 400.0))
    for i in range(n):
        CP.key(t[i], eye=tuple(eye[i]), target=tuple(tgt[i]), lens=float(lens[i]), fstop=float(fstop[i]),
               mode="linear")
    cam = CP.bake(fr, FPS)

    # render-time economy: hide engine / clutch / gearbox / body / front corners while
    # they are well outside the (enlarged) frustum.  Camera relative to the car frame:
    eye_rel = eye - pos
    sc.frame_set(1)
    inv = C.root.matrix_world.inverted()
    econ = []
    for name in ("engine", "clutch", "gearbox", "body"):
        if name in C.sub:
            econ += C.sub[name].meshes()
    econ += [o for k, o in WP.items() if o.type == "MESH" and (k.endswith(("_FL", "_FR")) or
                                                             k in ("rack", "subframe_front"))]
    tgt_rel = tgt - pos
    cache = {}
    for ob in econ:
        pts = np.array([tuple(inv @ ob.matrix_world @ Vector(c)) for c in ob.bound_box])
        lo, hi = pts.min(0) - 0.1, pts.max(0) + 0.1
        key = (tuple(np.round(lo, 2)), tuple(np.round(hi, 2)))
        vis = cache.get(key)
        if vis is None:
            vis = box_visible(lo, hi, eye_rel, tgt_rel, lens)
            for _ in range(3):
                vis = vis | np.r_[vis[1:], False] | np.r_[False, vis[:-1]]
            cache[key] = vis
        _hide_or(ob, fr, ~vis)

    # motion blur shutter (keyed; the pipeline enables motion blur for the scene)
    mb = _ss(T_RAMP[0], T_RAMP[0] + 1.2, t)
    rig.bake_channel(sc, "render.motion_blur_shutter", -1, fr, SHUTTER[0] + (SHUTTER[1] - SHUTTER[0]) * mb)

    # ---------------- validation ----------------------------------------
    no_mb = t < T_RAMP[0] - 0.05            # frames where blur is not trusted to hide strobing
    th = {c: track["theta_" + c] for c in ("RL", "RR", "FL", "FR")}
    aliasing = {}
    for c in ("RL", "RR", "FL", "FR"):
        aliasing[f"tyre tread {c} (64)"] = (th[c], TAU / 64, no_mb)
        aliasing[f"wheel spokes {c} (5)"] = (th[c], TAU / 5, no_mb)
        aliasing[f"brake disc vanes {c} (36)"] = (th[c], TAU / 36, no_mb)
    aliasing["Rzeppa balls / cage / races RR (6)"] = (th["RR"], TAU / 6, no_mb)
    aliasing["tripod rollers RR (3)"] = (th["RR"], TAU / 3, no_mb)
    aliasing["shaft splines RR (27)"] = (th["RR"], TAU / 27, no_mb)
    aliasing["ring gear (41)"] = (track.theta_case, TAU / S.Z_RING, no_mb)
    aliasing["pinion (10)"] = (track.theta_out, TAU / S.Z_PINION_TEETH, no_mb)
    aliasing["propshaft yokes (2)"] = (track.theta_out, math.pi, no_mb)
    track.validate(aliasing=aliasing)

    # ---------------- labels --------------------------------------------
    L = Labels()
    WA = W.anchors
    t_ds, t_joint, t_end = wt("why", "driveshaft"), wt("why", "joint"), wt("why", "each end")
    t_diff = wt("why", "differential")
    L.add("driveshaft", "Driveshaft", WA["driveshaft_right"], t_ds, t_diff - 0.2, offset=(0.0, 0.11))
    L.add("outer_joint", "Outer joint", WA["outer_joint_right"], t_joint, bend("why") - 0.5, offset=(0.05, -0.12))
    L.add("inner_joint", "Inner joint", WA["inner_joint_right"], t_end, bend("why") - 0.5, offset=(0.0, -0.13))
    if A is not None and "diff_housing" in A.anchors:
        L.add("diff", "Differential", A.anchors["diff_housing"], t_diff, bend("why") - 0.5, offset=(-0.04, 0.12))
    # rzeppa: anchors in the (tilting, non-spinning) cage / inner-race pivots and the corner
    # frame, on the side facing the camera; the spinning parts turn past them
    corner, cgp, irp = F["corner_RR"], F["cagepiv_RR"], F["irpiv_RR"]
    ox = X_OJ - S.TRACK_REAR / 2.0
    ds = W.meta["driveshaft"]
    rb, cg_o = ds["ball_pcr"], ds["cage"][1]

    def facing(r, el_deg, axial):          # pivot-local point (local X = rear, Y = outboard, Z = up)
        el = math.radians(el_deg)
        return (r * math.cos(el), axial, r * math.sin(el))
    t_rz_end = bend("rzeppa") - 0.6
    balls = [WP[f"ball_RR_{k}"] for k in range(S.RZEPPA_BALLS)]
    L.add("balls", "Balls", (cgp, facing(rb, 22.0, 0.0)), wt("rzeppa", "balls"), t_rz_end, offset=(-0.10, -0.12),
          ignore=tuple(balls) + (WP["cage_RR"],))          # the anchor is on the ball track
    # inner race: its inboard face, seen through the cage's inboard opening around the shaft
    L.add("inner_race", "Inner race", (irp, facing(0.019, 8.0, -0.0112)), wt("rzeppa", "inner"), t_rz_end,
          offset=(-0.12, 0.10))
    L.add("outer_race", "Outer race", (corner, (ox + 0.004, -0.004, ds["outer_race"] + 0.006)),
          wt("rzeppa", "outer", 2), t_rz_end, offset=(0.07, -0.11))
    L.add("cage", "Cage", (cgp, facing(cg_o, -20.0, -0.0105)), wt("rzeppa", "cage"), t_rz_end, offset=(0.09, 0.11))
    # plunge
    # on the tulip's section face (the wall always exists at r = 45 mm), lower left
    L.add("inner_joint2", "Inner joint", (W.root, (x_ij - 0.008, Y_RA - 0.0015, ZW - 0.045)), wt("plunge", "inner"),
          bend("plunge") - 0.3, offset=(-0.09, 0.05))
    L.add("plunge", "Plunge", (tick_now, (0.0, 0.0, 0.5 * GAUGE_SIZE[2])), wt("plunge", "slide"),
          bend("plunge") - 0.3, offset=(0.07, -0.07), style="emph", occlusion=False)
    L.add("rest", "At rest", (tick_rest, (0.0, 0.0, 0.5 * GAUGE_SIZE[2])), GAUGE_IN[1], bend("plunge") - 0.3,
          offset=(-0.07, -0.07), occlusion=False)

    # ---------------- HUD -----------------------------------------------
    H = Hud(track)
    H.add("fade", 0.0, 0.8, fade=0.0, color="black", alpha=lambda i: float(max(0.0, 1.0 - t[i] / 0.6)))
    H.add("fade", DUR - 0.6, DUR + 1.0, fade=0.0, color="black",
          alpha=lambda i: float(min(1.0, max(0.0, (t[i] - (DUR - 0.55)) / 0.5))))
    H.add("section_title", 0.4, 4.4, number=7, title="Driveshafts & CV joints")
    shaft_key = W.meta["spin"]["shaft_RR"]           # 'theta_RR': the shaft turns with the wheel
    rpm_shaft = track["rpm_" + shaft_key.split("_", 1)[1]]
    rpm_wheel = track["rpm_RR"]
    t_hi0, t_hi1 = wt("rzeppa", "so"), bend("rzeppa") - 0.4

    def rows(i):
        hi = bool(t_hi0 <= t[i] <= t_hi1)
        return [["Driveshaft", f"{fmt_rpm(rpm_shaft[i])} rpm", hi], ["Wheel", f"{fmt_rpm(rpm_wheel[i])} rpm", hi]]
    H.add("readouts", 1.0, DUR, rows=rows)
    RX = 0.05 * 9.0 / 16.0                # HUD margin (5 u, u = H/100) as a fraction of W
    kin_rr = W.meta["kinematics"]["rear_kin"](track["susp_RR"], 1, x_ij)
    ang = np.degrees(kin_rr["alpha"])
    plunge_mm = kin_rr["p"] * 1000.0
    H.add("readouts", wt("why", "wheel"), bend("rzeppa") - 0.5, pos=(RX, 0.235),
          rows=lambda i: [["Joint angle", f"{ang[i]:+.1f}°"]])
    hub_mm = np.sqrt(kin_rr["L"] ** 2 + track["susp_RR"] ** 2) * 1000.0     # tulip centre (on the diff) -> O
    H.add("readouts", wt("plunge", "inner"), bend("plunge") - 0.2, pos=(RX, 0.235),
          rows=lambda i: [["Diff to hub", f"{hub_mm[i]:.1f} mm"], ["Plunge", f"{plunge_mm[i]:.1f} mm"]])
    H.add("slowmo", 0.6, DUR, factor=lambda i: float(1.0 / track.slowmo[i]))

    sb = scenebase.SceneBuild(SCENE_ID, track, cam, L, H, motion_blur=True, shutter=SHUTTER[1],
                              preview_hide=())
    sb.extra.update(car=C, studio=studio, cutters=(cut_o, cut_i))
    return sb
