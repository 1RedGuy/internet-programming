"""s08 Recap (32 s): the whole car drives through 1st, 2nd and 3rd in REAL TIME, then the power path.

Road studio (continuity with s07), full car on the car lane with an x-ray body (faint shell +
feature lines; interior / underbody fainter still) so the drivetrain can be seen.  The housings
(engine, bellhousing, gearbox case, diff housing) stay opaque; everything that is inside them
and can never be seen (crank, pistons, valvetrain, clutch pack, gear train, ring/pinion,
differential ...) is hidden from the render.  What turns in shot: propshaft and its Hooke
joints, gearbox output flange, pinion companion flange, driveshafts, CV joints, hubs, discs,
wheels, the accessory-drive pulleys; the clutch pedal and the gear lever move as programmed.
The camera rides with the car (parented to the car root); the lights follow the car; the road
(world-space texture, joints every 15 m) streams past; Cycles motion blur (shutter 0.5 frame).

Drivetrain state: ONE real-time Program (slowmo 1, no cuts).  The road speed is prescribed
from a smooth acceleration plan; each clutch engagement is planned so that it is consistent
with the state's Coulomb clutch model: the clutch torque is what the car's acceleration needs
((m_eff a + rolling + aero) r / overall ratio), the engine follows a smooth trajectory that
meets the disc speed (slip -> 0 with a slightly negative slope, so the lock-up holds), and the
pedal (inverse of kin.clutch_capacity) and the driver's throttle target (governor) are solved
from that.  The integrator then reproduces the plan (Track.validate: no violations).

  together  0-4    stationary, engine idling 850 rpm, clutch already down, neutral (the free
                   input shaft is still running down); the lever crosses to the 1-2 plane and
                   1st is engaged (the synchro stops the input shaft) -> "stationary in 1st".
  first     4-10   pedal up through the bite zone; the engine is revved to ~1350, sags to
                   ~970 as the clutch takes up, and the clutch LOCKS at 6.25 s (8.6 km/h,
                   ~1070 rpm; slip 4.6-6.25 s); 1st gear to 3000 rpm = 24.1 km/h at ~10.1 s.
  second    10-17  clutch in 10.05, throttle eased (engine falls 3000 -> ~1950), out of 1st
                   10.45, 2nd engaged with a real-time synchro event (10.85-11.5), clutch out
                   11.85-12.85: slip, lock at ~12.6 s at 1787 rpm (same road speed); 2nd
                   gear to 3000 rpm = 40.5 km/h at ~17.0 s.
  third     17-22  clutch in 16.95, out of 2nd 17.28, across the gate 17.52, 3rd engaged
                   17.76-18.36, clutch out: lock at ~19.1 s at 2011 rpm; gentle acceleration.
  summary   22-32  cruise in 3rd (49.1 km/h, ~2440 rpm); the camera pulls back and up; the
                   power path glows stage by stage as it is named, then as one chain; fade to
                   black over the last 1.5 s.
"""
from __future__ import annotations

import math

import bpy
import numpy as np
from mathutils import Vector

from carviz import camera as CAM
from carviz import lighting, materials, rig, scenebase, state, timeline
from carviz import spec as S
from carviz.assemblies import car as CAR
from carviz.labels import Hud, Labels, fmt_rpm
from carviz.state import I_ENGINE, K_GOVERNOR, T_CLUTCH_MAX, T_ENGINE_MAX, T_ENGINE_MIN

SCENE_ID = "s08"
SC = timeline.scene(SCENE_ID)
BT = {b.id: b for b in SC.beats}
FPS = S.FPS
DUR = SC.dur
TAU = 2.0 * math.pi
RPM = 60.0 / TAU


def bstart(b):
    return BT[b].start


def bend(b):
    return BT[b].end


def wt(b, word, occ=1):
    return BT[b].word_time(word, occ)


def _ss(e0, e1, x):
    u = np.clip((np.asarray(x, dtype=float) - e0) / (e1 - e0), 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


# ===========================================================================
# Drivetrain plan (real time)
# ===========================================================================
M_EFF = 1400.0 * 1.08            # kg, car + rotating inertia (illustrative, FACTS SFT-07 class)
DT = 0.005                       # planning grid (s)

BITE1, LOCK1 = 4.62, 6.25        # take-off: clutch starts to transmit / locks
CIN1 = 10.05                     # 1->2: clutch in
OUT1, ENG2 = 10.45, 10.85        # sleeve out of 1st / start of the 2nd-gear synchro stroke
BITE2, LOCK2 = 12.10, 12.60
CIN2 = 16.95                     # 2->3
OUT2, SEL3, ENG3 = 17.28, 17.52, 17.76
BITE3, LOCK3 = 18.65, 19.10
REV_TAKEOFF = 1350.0             # rpm the driver holds as the clutch starts to bite
HOVER = {2: 1950.0, 3: 2150.0}   # engine speed held (throttle eased) while the clutch is in


def f_res(v):
    """Rolling (0.012 g) + aero (CdA 0.66 m^2) resistance, N (FACTS SFT-07 values)."""
    v = np.asarray(v, dtype=float)
    return 0.012 * 1400.0 * 9.81 * np.tanh(v / 0.3) + 0.5 * 1.2 * 0.66 * v * v


def _piecewise(keys, t):
    t = np.asarray(t, dtype=float)
    out = np.full_like(t, keys[-1][1])
    out[t < keys[0][0]] = keys[0][1]
    for (t0, a0), (t1, a1) in zip(keys, keys[1:]):
        m = (t >= t0) & (t < t1)
        out[m] = a0 + (a1 - a0) * _ss(t0, t1, t[m])
    return out


def _accel_keys(h1, h2):
    """Car acceleration plan (m/s^2): brisk take-off while the clutch slips, steady in 1st and
    2nd (h1, h2 solved so 3000 rpm is reached exactly at clutch-in), zero while the clutch is
    in (same road speed through each shift), gentle in 3rd, then a cruise."""
    return [(0.0, 0.0), (BITE1, 0.0), (5.35, 1.9), (LOCK1, 1.9), (6.85, h1), (9.75, h1), (CIN1 + 0.1, 0.0),
            (LOCK2 - 0.05, 0.0), (13.25, h2), (16.45, h2), (CIN2 + 0.03, 0.0),
            (LOCK3 - 0.05, 0.0), (19.8, 0.8), (21.0, 0.8), (23.8, 0.0), (DUR + 10.0, 0.0)]


def _speed(h1, h2):
    t = np.arange(0.0, DUR + 1e-9, DT)
    a = _piecewise(_accel_keys(h1, h2), t)
    v = np.concatenate([[0.0], np.cumsum(0.5 * (a[1:] + a[:-1]) * DT)])
    return t, a, v


def solve_speed():
    """(t, a, v m/s, h1, h2): v = 3000 rpm in 1st at clutch-in 1, in 2nd at clutch-in 2."""
    v1 = S.road_speed_kmh(3000, 1) / 3.6
    v2 = S.road_speed_kmh(3000, 2) / 3.6
    h1, h2 = 1.3, 1.2
    for _ in range(60):
        t, a, v = _speed(h1, h2)
        e1 = v[int(round((CIN1 + 0.1) / DT))] - v1
        e2 = v[int(round((CIN2 + 0.03) / DT))] - v2
        h1 -= e1 / 2.6
        h2 -= e2 / 3.3
        if abs(e1) < 1e-7 and abs(e2) < 1e-7:
            break
    return t, a, v, h1, h2


def cap_inverse(c):
    """Pedal position giving clutch capacity fraction c (inverse of kin.clutch_capacity)."""
    s = 1.0 - np.clip(c, 0.0, 1.0)
    u = np.linspace(0.0, 1.0, 20001)
    return S.CLUTCH_BITE_LO + (S.CLUTCH_BITE_HI - S.CLUTCH_BITE_LO) * np.interp(s, u * u * (3 - 2 * u), u)


def n_in_of(v, g):
    """Input-shaft rpm at road speed v (m/s) in gear g."""
    return np.asarray(v) / S.ROLLING_RADIUS * S.FINAL_DRIVE * S.GEAR_RATIOS[g] * RPM


def plan():
    t, a, v, h1, h2 = solve_speed()
    slips = {}

    def slip_phase(g, tb, tl, n0, eps=0.25):
        """Engine trajectory n_e = n_in + slip (slip from n0 - n_in to 0, ending with a slightly
        negative slope, so at the crossing the clutch holds with margin), clutch torque =
        what the car's acceleration needs (raised if the engine would need more than its
        closed-throttle drag to follow), engine torque = Tc + I_e * alpha_e."""
        m = (t >= tb - 1e-9) & (t <= tl + 1e-9)
        u = (t[m] - tb) / (tl - tb)
        nin = n_in_of(v[m], g)
        h = (1 - eps) * (1 - u * u * (3 - 2 * u)) + eps * (1 - u)
        ne = nin + (n0 - nin[0]) * h
        al = np.gradient(ne / RPM, DT)
        r = S.GEAR_RATIOS[g] * S.FINAL_DRIVE
        tc = (M_EFF * a[m] + f_res(v[m])) * S.ROLLING_RADIUS / r
        # a clutch that is visibly slipping carries some torque (>= 4 % of its capacity)
        tc = np.maximum(tc, 0.04 * T_CLUTCH_MAX * _ss(0.0, 0.25, u))
        te = tc + I_ENGINE * al
        lo = T_ENGINE_MIN + 8.0
        tc = np.where(te < lo, tc + (lo - te), tc)
        te = tc + I_ENGINE * al
        return dict(idx=np.nonzero(m)[0], ne=ne, tc=tc, te=te)

    slips[1] = slip_phase(1, BITE1, LOCK1, REV_TAKEOFF)
    slips[2] = slip_phase(2, BITE2, LOCK2, HOVER[2])
    slips[3] = slip_phase(3, BITE3, LOCK3, HOVER[3])
    return dict(t=t, a=a, v=v, h1=h1, h2=h2, slips=slips)


def build_program():
    pl = plan()
    t, a, v, sl = pl["t"], pl["a"], pl["v"], pl["slips"]
    P = state.Program(SCENE_ID)
    P.slowmo.key(0.0, 1.0, "step")                       # REAL TIME throughout
    for k in range(0, len(t), 10):                       # 0.05 s keys (linear) of the solved speed
        P.speed_kmh.key(float(t[k]), float(v[k] * 3.6), "linear")
    P.speed_kmh.key(float(t[-1]), float(v[-1] * 3.6), "linear")

    # together: clutch already pressed, neutral (input shaft still coasting down from idle);
    # lever across to the 1-2 plane, 1st engaged: the synchro stops the input shaft.
    P.w_in0_rpm = 520.0
    P.lever_plane.key(0.0, 0.0, "step")
    P.select_plane(1.05, -1, 0.35)
    P.engage(1, 1.45, travel=0.2, hold=0.35, through=0.12, seat=0.1)

    # clutch pedal
    pd = P.pedal
    pd.key(0.0, 1.0, "step").key(4.0, 1.0, "linear")

    def slip_pedal(g, tb):
        s = sl[g]
        p = cap_inverse(s["tc"] / T_CLUTCH_MAX)
        pd.key(tb, float(p[0]), "ease")                 # free travel down to the bite point
        for j in range(8, len(p), 8):
            pd.key(float(t[s["idx"][j]]), float(p[j]), "linear")
        return float(t[s["idx"][-1]])

    tl = slip_pedal(1, BITE1)
    pd.key(tl + 0.4, 0.0, "ease")
    pd.key(CIN1, 0.0, "linear").key(CIN1 + 0.35, 1.0, "ease")
    pd.key(BITE2 - 0.25, 1.0, "linear")
    tl = slip_pedal(2, BITE2)
    pd.key(tl + 0.3, 0.0, "ease")
    pd.key(CIN2, 0.0, "linear").key(CIN2 + 0.3, 1.0, "ease")
    pd.key(BITE3 - 0.22, 1.0, "linear")
    tl = slip_pedal(3, BITE3)
    pd.key(tl + 0.3, 0.0, "ease")

    # shifts (sleeves move only with the pedal on the floor)
    P.disengage(1, OUT1, 0.25)
    P.engage(2, ENG2, travel=0.18, hold=0.28, through=0.10, seat=0.08)
    P.disengage(2, OUT2, 0.22)
    P.select_plane(SEL3, 0, 0.22)
    P.engage(3, ENG3, travel=0.16, hold=0.26, through=0.10, seat=0.08)

    # driver's throttle = engine-speed target of the governor (state.K_GOVERNOR)
    th = P.throttle_rpm
    th.key(0.0, S.IDLE_RPM, "step").key(4.0, S.IDLE_RPM, "linear").key(BITE1 - 0.05, REV_TAKEOFF, "ease")

    def thr_of(ne_rpm, te):
        return ne_rpm + np.clip(te, T_ENGINE_MIN, T_ENGINE_MAX) / K_GOVERNOR * RPM

    def slip_throttle(g):
        s = sl[g]
        w = thr_of(s["ne"], s["te"])
        for j in range(0, len(w), 8):
            th.key(float(t[s["idx"][j]]), float(w[j]), "linear")
        return float(t[s["idx"][-1]])

    def locked_throttle(t0, t1, g):
        r = S.GEAR_RATIOS[g] * S.FINAL_DRIVE
        for tt in np.arange(t0, t1 + 1e-9, 0.05):
            j = min(int(round(tt / DT)), len(t) - 1)
            te = (M_EFF * a[j] + f_res(v[j])) * S.ROLLING_RADIUS / r + I_ENGINE * r * a[j] / S.ROLLING_RADIUS
            th.key(float(tt), float(thr_of(n_in_of(v[j], g), te)), "linear")

    tl = slip_throttle(1)
    locked_throttle(tl + 0.05, CIN1 + 0.02, 1)
    th.key(CIN1 + 0.18, HOVER[2], "ease").key(BITE2, HOVER[2], "linear")
    tl = slip_throttle(2)
    locked_throttle(tl + 0.05, CIN2 + 0.02, 2)
    th.key(CIN2 + 0.18, HOVER[3], "ease").key(BITE3, HOVER[3], "linear")
    tl = slip_throttle(3)
    locked_throttle(tl + 0.05, DUR, 3)
    return P, pl


# ===========================================================================
# Camera (CAR frame: the camera rides with the car root)
# (time, eye, target, lens, f-stop)
# ===========================================================================
POSES = [
    # together: 3/4 front-left from above the roof line, slow drift toward the side
    (0.0, (-3.96, +2.96, 2.19), (+0.18, -1.17, 0.33), 32.0, 5.6),
    (3.6, (-5.13, +2.12, 2.57), (+0.07, -1.29, 0.34), 32.0, 5.6),
    # first: tracking alongside as the car pulls away (front-left, then side-on)
    (7.4, (-6.23, +0.14, 2.90), (+0.02, -1.46, 0.39), 32.0, 5.6),
    # second: side-on, high, for the shift (whole drivetrain, pedal and lever in the cabin)
    (10.4, (-6.85, -0.62, 3.15), (+0.04, -1.46, 0.42), 34.0, 5.6),
    (13.4, (-6.93, -1.30, 3.17), (+0.06, -1.44, 0.44), 34.0, 5.6),
    # keeps accelerating: swing toward the rear
    (16.4, (-5.07, -4.91, 2.44), (-0.00, -1.67, 0.39), 32.0, 5.6),
    # third: rear-left 3/4 (propshaft, axle, driveshafts, rear wheels)
    (19.4, (-3.12, -5.95, 1.86), (+0.18, -1.92, 0.38), 32.0, 5.6),
    (21.6, (-3.43, -5.83, 2.19), (+0.16, -1.86, 0.34), 32.0, 5.6),
    # summary: pull back and up to a high side view of the whole power path, keep rising
    (24.8, (-5.87, -2.17, 4.50), (+0.04, -1.42, 0.37), 32.0, 5.6),
    (DUR, (-7.41, -1.11, 6.29), (+0.03, -1.49, 0.35), 32.0, 5.6),
]
CAM_SMOOTH = 0.35                # s, Gaussian low-pass on the keyed path


def _gauss(x, sigma_frames):
    r = int(3 * sigma_frames)
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma_frames) ** 2)
    k /= k.sum()
    xp = np.r_[np.full(r, x[0]), x, np.full(r, x[-1])]
    return np.convolve(xp, k, mode="valid")


def camera_car(t):
    cs = {k: state.Curve() for k in ("ex", "ey", "ez", "tx", "ty", "tz", "lens", "f")}
    for i, (tk, e, g, lens, f) in enumerate(POSES):
        mode = "step" if i == 0 else "cubic"
        for k, val in zip(("ex", "ey", "ez", "tx", "ty", "tz", "lens", "f"), (*e, *g, lens, f)):
            cs[k].key(tk, val, mode)
    sig = CAM_SMOOTH * FPS
    p = {k: _gauss(c(t), sig) for k, c in cs.items()}
    eye = np.stack([p["ex"], p["ey"], p["ez"]], 1)
    tgt = np.stack([p["tx"], p["ty"], p["tz"]], 1)
    return eye, tgt, p["lens"], p["f"]


# ===========================================================================
# Presentation
# ===========================================================================
XRAY = dict(exterior=0.12, glass=0.05, interior=0.06, underbody=0.05, edges=0.55)
FADE_IN = 0.5                    # from black (s07 ends on black)
FADE_OUT = 1.5                   # to black over the last 1.5 s
HUD_SHIFT_OUT = (22.4, 23.2)     # pedal / H-pattern / status leave once the shifting is done
HUD_OUT = (29.2, 30.0)           # remaining HUD leaves before the final fade
GLOW_ON = 0.085                  # glow level as a stage is named ...
GLOW_HOLD = 0.048                # ... and while the whole chain stays lit
SHUTTER = 0.5

# Never visible (inside opaque housings): hidden from every render.
HIDE = {
    "engine": ("crankshaft", "flywheel", "ring_gear", "crank_sprocket", "conrod1", "conrod2", "conrod3",
               "conrod4", "piston1", "piston2", "piston3", "piston4", "cam_intake", "cam_exhaust",
               "cam_sprocket_intake", "cam_sprocket_exhaust", "timing_chain", "chain_guide",
               "tensioner_arm", "tensioner", "main_caps", "main_bolts", "main_shells", "valve_guides",
               "valve_seats", "cam_caps", "spark_plug1", "spark_plug2", "spark_plug3", "spark_plug4"),
    "engine_prefix": ("valve_", "spring_", "gas_", "spark"),
    "clutch": ("disc", "damper_springs", "pressure_plate", "straps", "diaphragm_spring", "fulcrum", "cover",
               "release_bearing", "bearing_race", "guide_tube", "ball_stud", "master_piston"),
    "gearbox_keep": ("case", "tail_housing", "output_flange", "lever", "knob", "boot"),
    "axle": ("pinion", "pinion_spacer", "pinion_cone_head", "pinion_cone_tail", "pinion_rollers_head",
             "pinion_rollers_tail", "pinion_cup_head", "pinion_cup_tail", "pinion_seal", "ring_gear",
             "ring_bolts", "case_left", "case_right", "cross_pin", "spider_1", "spider_2", "spider_washer_1",
             "spider_washer_2", "side_gear_left", "side_gear_right", "side_washer_left", "side_washer_right",
             "carrier_cone_left", "carrier_cone_right", "carrier_rollers_left", "carrier_rollers_right",
             "carrier_cup_left", "carrier_cup_right"),
}


def _meshes_of(objs):
    out = []
    for ob in objs:
        if ob is None:
            continue
        for o in [ob] + list(ob.children_recursive):
            if o.type == "MESH":
                out.append(o)
    return list(dict.fromkeys(out))


def _clear_vis_keys(ob):
    """Remove baked hide_render / hide_viewport keys (they would override a static hide)."""
    ad = ob.animation_data
    if ad is None or ad.action is None:
        return
    from bpy_extras import anim_utils
    cb = anim_utils.action_get_channelbag_for_slot(ad.action, ad.action_slot)
    if cb is None:
        return
    for fc in list(cb.fcurves):
        if fc.data_path in ("hide_render", "hide_viewport"):
            cb.fcurves.remove(fc)


def _hide(objs):
    for o in objs:
        _clear_vis_keys(o)
        o.hide_render = True
        o.hide_viewport = True


def _pulse_hold(t, t_on, rise=0.25):
    """Glow envelope for one stage: up to GLOW_ON at its word, settling to GLOW_HOLD."""
    up = _ss(t_on - 0.05, t_on + rise, t)
    settle = _ss(t_on + 0.6, t_on + 1.6, t)
    return up * (GLOW_ON + (GLOW_HOLD - GLOW_ON) * settle)


# ===========================================================================
# build
# ===========================================================================

def build(quality: str) -> scenebase.SceneBuild:
    sc = scenebase.new_scene(SCENE_ID)
    preview = quality == "preview"
    det = "low"
    C = CAR.build({"engine": {"cutaways": ["none"], "detail": det, "gas": False},
                   "clutch": {"cutaway": ["none"], "detail": det},
                   "gearbox": {"cutaway": "none", "detail": det},
                   "axle": {"cutaways": ["none"], "detail": det},
                   "wheels": {"detail": det},
                   "body": {"detail": det, "xray_edges": True}})
    E, CL, G, A, W, B = (C.sub[k] for k in ("engine", "clutch", "gearbox", "axle", "wheels", "body"))
    studio = lighting.setup_studio("road", follow=C.root, follow_heading=True)
    lighting.setup_color_management(sc)

    # ---------------- drivetrain state ----------------------------------
    P, pl = build_program()
    track = P.run()
    t, n, fr = track.t, track.n, track.frames

    # ---------------- drive (all mechanical motion + the car's world motion from the Track)
    body_pres = {"xray_edges_opacity": np.full(n, XRAY["edges"])}
    if not preview:
        body_pres.update(exterior_opacity=np.full(n, XRAY["exterior"]), glass_opacity=np.full(n, XRAY["glass"]),
                         interior_opacity=np.full(n, XRAY["interior"]),
                         underbody_opacity=np.full(n, XRAY["underbody"]))
    C.drive(track, {"engine": {"variant": "none"}, "clutch": {"variant": "none"},
                    "axle": {"variant": "none"}, "body": body_pres})
    ghost = [o for o in B.meshes() if o is not B.parts.get("xray_edges")]
    if preview:
        # Workbench ignores cv_opacity: hide the ghosted body (the feature lines outline the car)
        _hide(ghost)
    else:
        # door cards / headliner / parcel shelf would fog the view into the car
        rig.bake_fade([B.parts["cabin_trim"]], fr, np.zeros(n))

    # ---------------- render economy: what is inside opaque housings is never seen -------
    hide = [E.parts.get(k) for k in HIDE["engine"]]
    hide += [o for k, o in E.parts.items() if k.startswith(HIDE["engine_prefix"])]
    hide += [CL.parts.get(k) for k in HIDE["clutch"]]
    hide += [o for k, o in G.parts.items() if k not in HIDE["gearbox_keep"]]
    hide += [A.parts.get(k) for k in HIDE["axle"]]
    keep_vis = set()
    for k in HIDE["gearbox_keep"]:
        keep_vis.update(o.name for o in _meshes_of([G.parts[k]]))
    hidden = [o for o in _meshes_of(hide) if o.name not in keep_vis]
    _hide(hidden)
    for ob in bpy.data.objects:
        if ob.type == "MESH":
            ob.cycles.use_deform_motion = False          # nothing deforms visibly at these speeds
            if preview:
                ob.display.show_shadows = False          # Workbench shadow volumes are very slow
    for k in ("ground", "preview_marks"):
        if studio.get(k) is not None:
            studio[k].cycles.use_motion_blur = False     # static road: only the camera moves

    # ---------------- power-path glow (summary) ---------------------------
    ts = {k: wt("summary", w) for k, w in (("engine", "Engine"), ("clutch", "clutch"), ("gearbox", "gearbox"),
                                          ("prop", "propeller"), ("final", "final"),
                                          ("diff", "differential"), ("shafts", "driveshafts"),
                                          ("chain", "one"), ("wheels", "turning"))}
    EP, CP_, GP, AP, WP = E.parts, CL.parts, G.parts, A.parts, W.parts
    stages = {
        "engine": [EP[k] for k in ("block", "head", "cam_cover", "timing_cover", "oil_pan", "damper",
                                   "head_gasket") if k in EP],
        "clutch": [CP_[k] for k in ("bellhousing", "bellhousing_bolts", "fork", "slave_cylinder") if k in CP_],
        "gearbox": [GP[k] for k in ("case", "tail_housing", "output_flange")],
        "prop": [o for k, o in AP.items() if k.startswith("prop_")],
        "final": [AP[k] for k in ("companion_flange", "housing", "plugs") if k in AP],
        "diff": [AP[k] for k in ("cover", "housing_bolts", "stub_left", "stub_right", "seal_left", "seal_right")
                 if k in AP],
        "shafts": W.meta["groups"]["shafts"],
        # "... turning fuel into motion": the chain ends at the driven wheels
        "wheels": [o for o in W.meta["groups"]["rear_wheels"] if not o.name.startswith("whl_tire_")],
    }
    glow = {}
    for key, objs in stages.items():
        env = _pulse_hold(t, ts[key])
        for o in _meshes_of(objs):
            if o.hide_render or o.name.startswith("whl_tire_"):     # glowing rubber reads as copper
                continue
            glow[o.name] = np.maximum(glow.get(o.name, np.zeros(n)), env)
    # "one chain of gears and shafts": the whole path brightens once together
    chain = _ss(ts["chain"] - 0.1, ts["chain"] + 0.5, t) * (1.0 - _ss(ts["chain"] + 1.6, ts["chain"] + 2.8, t))
    for name in glow:
        glow[name] = glow[name] + (GLOW_ON - GLOW_HOLD) * chain * (glow[name] > 0)
    for name, g in glow.items():
        ob = bpy.data.objects[name]
        materials.ensure_props(ob)
        rig.bake_prop(ob, "cv_glow", fr, np.clip(g, 0.0, 0.1))

    # ---------------- camera ---------------------------------------------
    eye, tgt, lens, fstop = camera_car(t)
    CPATH = CAM.CameraPath(name="cam_s08", lens=30.0, fstop=5.6, clip=(0.05, 600.0))
    for i in range(n):
        CPATH.key(t[i], eye=tuple(eye[i]), target=tuple(tgt[i]), lens=float(lens[i]), fstop=float(fstop[i]),
                  mode="linear")
    cam = CPATH.bake(fr, FPS, parent=C.root)

    # ---------------- validation ------------------------------------------
    # Real time with Cycles motion blur on EVERY frame (FACTS PRS-05): the brief allows the
    # aliasing check to be relaxed for motion-blurred frames, so the masks below are empty;
    # the per-frame pitch steps are still computed and printed for the record.
    th = {c: track["theta_" + c] for c in ("RL", "RR", "FL", "FR")}
    no_mb = np.zeros(n, bool)
    feats = {
        "wheel spokes (5)": (th["RL"], TAU / 5),
        "tyre tread (64)": (th["RL"], TAU / 64),
        "brake disc vanes (36)": (th["RL"], TAU / 36),
        "propshaft yokes / output flange bolts (4)": (track.theta_out, TAU / 4),
        "Rzeppa / tripod (6 / 3)": (th["RL"], TAU / 6),
        "crank damper / pulleys (engine)": (track.theta_e, TAU / 6),
    }
    aliasing = {k: (ang, pitch, no_mb) for k, (ang, pitch) in feats.items()}
    track.validate(aliasing=aliasing)
    report = {k: float(np.max(np.abs(np.diff(ang))) / pitch) for k, (ang, pitch) in feats.items()}
    print("[s08] max pitch/frame (motion-blurred, real time):",
          ", ".join(f"{k}: {v:.2f}" for k, v in report.items()))

    # ---------------- labels (summary: each stage named as it glows) ------
    L = Labels()
    body_objs = tuple(B.meshes())

    def at(ob, world):
        """Anchor on a static part at a car-frame point (converted to the part's local frame)."""
        sc.frame_set(1)
        loc = ob.matrix_world.inverted() @ (C.root.matrix_world @ Vector(world))
        return (ob, tuple(loc))

    t_out_all = HUD_OUT[0] + 0.4
    life = 1.9
    lab = [
        ("engine", "Engine", at(EP["cam_cover"], (-0.06, -0.06, 0.784)), (-0.05, -0.10)),
        ("clutch", "Clutch", at(CP_["bellhousing"], (-0.17, -0.40, 0.47)), (-0.03, 0.12)),
        ("gearbox", "Gearbox", at(GP["case"], (-0.06, -0.66, 0.49)), (0.02, -0.12)),
        ("prop", "Propeller shaft", A.anchors["propshaft"], (0.0, 0.11)),
        ("final", "Final drive", at(AP["housing"], (0.0, -2.47, 0.395)), (-0.02, -0.12)),
        ("diff", "Differential", A.anchors["diff_housing"], (0.06, 0.10)),
        ("shafts", "Driveshafts", W.anchors["driveshaft_left"], (0.0, 0.14)),
    ]
    # rays to the clutch / driveshaft anchors pass the pedal box / rear suspension: those parts
    # belong to the named system, so they do not count as occluders
    own = {"clutch": tuple(CL.meshes()), "shafts": tuple(W.meshes()), "final": tuple(A.meshes()),
           "diff": tuple(A.meshes())}
    for key, text, anchor, off in lab:
        t0 = ts[key]
        L.add(key, text, anchor, t0, min(t0 + life, t_out_all), fade=0.3, offset=off, style="emph",
              ignore=body_objs + own.get(key, ()))

    # ---------------- HUD -------------------------------------------------
    H = Hud(track)
    H.add("fade", 0.0, FADE_IN + 0.2, fade=0.0, color="black",
          alpha=lambda i: float(max(0.0, 1.0 - t[i] / FADE_IN)))
    H.add("fade", DUR - FADE_OUT - 0.1, DUR + 1.0, fade=0.0, color="black",
          alpha=lambda i: float(_ss(DUR - FADE_OUT, DUR - 1.0 / FPS, t[i])))
    H.add("section_title", 0.5, 4.3, number=8, title="Recap")
    H.add("slowmo", 0.6, HUD_OUT[1], factor=lambda i: float(1.0 / max(track.slowmo[i], 1e-6)))
    H.add("status", 0.6, HUD_SHIFT_OUT[1], text=lambda i: str(track.status[i]))
    H.add("gear", 0.6, HUD_OUT[1], value=lambda i: str(track.gear[i]))
    H.add("speed", 0.6, HUD_OUT[1], value_kmh=lambda i: float(track.v_kmh[i]))
    H.add("pedal", 0.6, HUD_SHIFT_OUT[1], value=lambda i: float(track.pedal[i]))
    H.add("hpattern", 0.6, HUD_SHIFT_OUT[1], x=lambda i: float(track.lever_x[i]),
          y=lambda i: float(track.lever_y[i]))
    H.add("rpm", 0.6, HUD_OUT[1], value=lambda i: fmt_rpm(track.rpm_e[i]), label="Engine")

    sb = scenebase.SceneBuild(SCENE_ID, track, cam, L, H, motion_blur=True, shutter=SHUTTER,
                              preview_hide=())
    sb.extra.update(car=C, studio=studio, plan=pl)
    return sb
