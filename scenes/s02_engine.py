"""s02 The engine (82.5 s): inline-4, crank, four-stroke cycle, valvetrain, firing order, flywheel.

Dark studio, engine (+ flywheel) only.  The engine idles at 850 rpm in neutral
with the clutch engaged and the car stationary; only the slow-motion factor
changes:

  inline4, crank, valvetrain, flywheel   x230    (ring gear 132T at 0.34 tooth/frame)
  intake .. exhaust                      x198.3  (pi / (7 s * omega_idle): each 7.0 s
                                                  stroke beat is exactly 180 deg)
  firing                                 x85     (all four firings 1-3-4-2, 3.0 s apart;
                                                  ring gear motion-blurred into a band)

The valvetrain factor is solved (bisection on the same sub-step integral the
state integrator uses) so cylinder 1 reaches its firing TDC exactly at
T_FIRE1, 1.5 s into the firing beat, just after the camera has settled (firings
at 63.0 / 66.0 / 69.0 / 72.0 s in the order 1-3-4-2).
`P.align_engine(intake start, 360)` puts cylinder 1 at TDC starting its intake
stroke exactly when the "intake" beat starts.

The flywheel and its 132T ring gear stay visible for the whole scene.  The ring
(>= x223 needed without blur, FACTS PRS-04) is de-strobed with PER-OBJECT Cycles
motion blur: only eng_flywheel and eng_ring_gear have ob.cycles.use_motion_blur
(everything else, camera included, is unblurred, so render cost is unchanged
elsewhere).  The scene shutter is keyed per frame from the ring's tooth pitch p
moved per frame: 1/p while p >= 0.5 (the smear is exactly one tooth pitch: a
uniform band, nothing to step backwards) blending to 1.0 frame below p = 0.4
(the smear joins consecutive positions; motion reads forward).  The ring's
aliasing mask keeps only frames where it is in frame AND that blur does not
hide strobing (none, by construction).

Cutaway changes never pop: `_bake_variants` cross-fades between the engine's
cut variants per part ('direct': fade only what differs; 'via': close the
engine up, swap the piece sets while it is whole, then open the new cut).
"""
from __future__ import annotations

import math

import numpy as np

from carviz import camera as CAM
from carviz import kin, lighting, rig, scenebase, state, timeline
from carviz import spec as S
from carviz.assemblies import engine as ENG
from carviz.labels import Hud, Labels, fmt_rpm

SCENE_ID = "s02"
SC = timeline.scene(SCENE_ID)
BT = {b.id: b for b in SC.beats}
FPS = S.FPS
DUR = SC.dur


def bstart(b):
    return BT[b].start


def bend(b):
    return BT[b].end


def wt(b, word, occ=1):
    return BT[b].word_time(word, occ)


# ---------------------------------------------------------------------------
# Slow motion plan
# ---------------------------------------------------------------------------
OMEGA = S.rpm_to_rad_s(S.IDLE_RPM)
SLOW_STROKE = math.pi / (7.0 * OMEGA)          # 1/198.3: 180 deg per 7.0 s beat
SLOW_RING = 1.0 / 230.0                        # ring gear 132T in shot (>= 1/223)
SLOW_FIRE = 1.0 / 85.0                         # firing beat: 180 deg per 3.0 s
T_INTAKE = bstart("intake")                    # 21.0
T_VALVE = bstart("valvetrain")                 # 49.0
T_FIRING = bstart("firing")                    # 61.5
T_FLYB = bstart("flywheel")                    # 74.0
T_FIRE1 = T_FIRING + 1.5                       # cyl 1 firing TDC (camera settled at ~62.5)

# slow-motion ramps
SLOW_RAMPS = dict(to_stroke=(17.6, T_INTAKE - 0.6), to_valve=(T_VALVE, T_VALVE + 1.5),
                  to_fire=(T_FIRING - 0.5, T_FIRING + 1.0), to_ring=(T_FIRE1 + 9.6, T_FLYB + 0.6))


def slowmo_curve(k_valve):
    c = state.Curve(SLOW_RING)
    c.key(0.0, SLOW_RING, "step")
    c.key(SLOW_RAMPS["to_stroke"][0], SLOW_RING, "linear")
    c.key(SLOW_RAMPS["to_stroke"][1], SLOW_STROKE, "ease")
    c.key(SLOW_RAMPS["to_valve"][0], SLOW_STROKE, "linear")       # constant over all four stroke beats
    c.key(SLOW_RAMPS["to_valve"][1], k_valve, "ease")
    c.key(SLOW_RAMPS["to_fire"][0], k_valve, "linear")
    c.key(SLOW_RAMPS["to_fire"][1], SLOW_FIRE, "ease")
    c.key(SLOW_RAMPS["to_ring"][0], SLOW_FIRE, "linear")
    c.key(SLOW_RAMPS["to_ring"][1], SLOW_RING, "ease")
    return c


def _crank_advance(curve, t0, t1, substeps=8):
    """Crank angle (rad) turned between video times t0 and t1 at idle, integrated
    exactly like state._integrate (mid-point of every sub-step)."""
    h = 1.0 / (FPS * substeps)
    i0, i1 = int(round(t0 / h)), int(round(t1 / h))
    tm = (np.arange(i0, i1) + 0.5) * h
    return OMEGA * float(np.sum(np.maximum(curve(tm), 0.0))) * h


def solve_valve_slowmo():
    """Valvetrain factor so that cyl 1 is at firing TDC at T_FIRE1 (it starts the
    valvetrain beat at TDC beginning intake, so 360 deg must pass)."""
    lo, hi = 1.0 / 400.0, 1.0 / 120.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        a = math.degrees(_crank_advance(slowmo_curve(mid), T_VALVE, T_FIRE1))
        if a < 360.0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


# highlight glow (materials.CV_Presentation: emission ~2.9 x rim at 0.7 x cv_glow, so small
# values already read as a clear warm highlight on dark forged steel)
GLOW_PEAK = 0.07          # connecting rods
GLOW_PEAK_CRANK = 0.045   # the crankshaft is large and reads stronger
GLOW_HOLD = 0.025

# ---------------------------------------------------------------------------
# Cutaway schedule
# ---------------------------------------------------------------------------
V0 = "none"
VARIANT_STEPS = [            # (variant, t0, t1, kind)
    ("long", 0.7, 3.0, "direct"),
    ("cyl1", 15.3, 18.7, "via"),
    ("front", 49.0, 51.0, "direct"),
    ("long", 59.9, 62.2, "via"),
    ("none", 74.2, 76.2, "direct"),
]


def _smooth(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * (3.0 - 2.0 * u)


_REGION = {}


def _region(asm, P, V):
    key = (id(asm), P, V)
    r = _REGION.get(key)
    if r is None:
        r = _REGION[key] = _region_calc(asm, P, V)
    return r


def _region_calc(asm, P, V):
    pcs = asm.meta["cutaway_pieces"].get(V)
    if pcs is None:
        return "all"
    if P in pcs["replaces"]:
        return "kept" if f"{P}__{V}_kept" in asm.parts else "none"
    if P in pcs["removed"]:
        return "none"
    return "all"


def _to_all(P, r, V, w):
    """Objects/opacity while part P goes from region r (variant V) to whole (w 0..1)."""
    if r == "all":
        return {P: 1.0}
    if r == "kept":
        return {f"{P}__{V}_kept": 1.0, f"{P}__{V}_removed": w}
    return {P: w}


def _from_all(P, r, V, w):
    """Part P going from whole to region r of variant V (w 0..1)."""
    if r == "all":
        return {P: 1.0}
    if r == "kept":
        return {f"{P}__{V}_kept": 1.0, f"{P}__{V}_removed": 1.0 - w}
    return {P: 1.0 - w}


def _part_state(asm, P, t):
    """dict object-key -> opacity for base part P at time t."""
    cur = V0
    for (V, t0, t1, kind) in VARIANT_STEPS:
        if t < t0:
            break
        if t <= t1:
            u = (t - t0) / (t1 - t0)
            rA, rB = _region(asm, P, cur), _region(asm, P, V)
            two = kind == "via" or (rA == "kept" and rB == "kept")
            if two:
                if u < 0.5:
                    return _to_all(P, rA, cur, _smooth(2 * u))
                return _from_all(P, rB, V, _smooth(2 * u - 1))
            s = _smooth(u)
            if rA == "all" and rB == "all":
                return {P: 1.0}
            if rA == "none" and rB == "none":
                return {}
            if rB == "all":
                return _to_all(P, rA, cur, s)
            if rA == "all":
                return _from_all(P, rB, V, s)
            if rA == "kept":                    # kept -> none
                return {f"{P}__{cur}_kept": 1.0 - s}
            return {f"{P}__{V}_kept": s}        # none -> kept
        cur = V
    r = _region(asm, P, cur)
    if r == "all":
        return {P: 1.0}
    if r == "kept":
        return {f"{P}__{cur}_kept": 1.0}
    return {}


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


def _bake_variants(asm, track, extra_opacity):
    """Per-frame opacity + visibility of every non-gas engine object.
    extra_opacity: {part key: per-frame multiplier} (flywheel fades)."""
    n = track.n
    t = track.t
    base = [k for k in asm.parts if "__" not in k and not k.startswith(("gas_", "spark"))
            and k not in ("ring_gear",)]
    keys = [k for k in asm.parts if not k.startswith(("gas_", "spark"))]
    op = {k: np.zeros(n) for k in keys}
    for i in range(n):
        for P in base:
            for k, v in _part_state(asm, P, t[i]).items():
                if k in op:
                    op[k][i] = v
    op["ring_gear"] = op["flywheel"].copy()
    for k, mul in extra_opacity.items():
        op[k] = op[k] * mul
    for k, a in op.items():
        ob = asm.parts[k]
        if ob.type != "MESH":
            continue
        a = np.clip(a, 0.0, 1.0)
        a = np.round(a, 4)
        hide = (a < 0.02).astype(float)
        _bake_compressed(ob, "hide_render", track.frames, hide, "CONSTANT")
        _bake_compressed(ob, "hide_viewport", track.frames, hide, "CONSTANT")
        if np.any((a > 0.02) & (a < 0.999)):
            if "cv_opacity" not in ob:
                ob["cv_opacity"] = 1.0
            _bake_compressed(ob, '["cv_opacity"]', track.frames, np.where(a < 0.02, 0.0, a))
        else:
            ob["cv_opacity"] = 1.0
    return op


def _dominant_variant(t):
    """Variant whose gas set is shown (switch at the middle of each transition)."""
    cur = V0
    for (V, t0, t1, kind) in VARIANT_STEPS:
        if t >= 0.5 * (t0 + t1):
            cur = V
    return cur


# ---------------------------------------------------------------------------
# Camera plan: (time, target, azimuth deg, radius, eye height above target, lens, f-stop, mode)
# azimuth as camera.orbit: 0 = behind (-Y), -90 = left (-X), +-180 = front (+Y)
# ---------------------------------------------------------------------------
POSES = [
    (0.0, (0.0, -0.07, 0.50), -142.0, 1.95, 0.62, 45.0, 5.6, "step"),
    (7.6, (0.0, -0.08, 0.48), -101.0, 1.66, 0.14, 45.0, 5.6, "cubic"),
    (14.4, (0.0, -0.09, 0.40), -112.0, 1.12, -0.10, 50.0, 5.6, "cubic"),
    # cycle: swing wide around the front-left corner while the engine closes up, then
    # push in on the cylinder-1 section
    (16.9, (0.0, -0.02, 0.48), -150.0, 2.15, 0.30, 50.0, 5.6, "cubic"),
    (19.8, (0.0, 0.06, 0.555), -180.0, 1.52, 0.07, 50.0, 5.6, "cubic"),
    (21.0, (0.0, 0.06, 0.555), -179.6, 1.51, 0.07, 50.0, 5.6, "cubic"),
    (27.6, (-0.012, 0.06, 0.552), -176.5, 1.48, 0.06, 50.0, 5.6, "cubic"),
    (34.6, (-0.004, 0.06, 0.570), -179.0, 1.36, 0.06, 50.0, 5.6, "cubic"),
    (41.6, (0.0, 0.06, 0.558), -180.0, 1.46, 0.07, 50.0, 5.6, "cubic"),
    (48.8, (0.012, 0.06, 0.555), -183.5, 1.50, 0.06, 50.0, 5.6, "cubic"),
    # valvetrain: rise to look down on the cams, then settle on the chain drive
    (52.4, (0.0, -0.03, 0.64), -148.0, 1.30, 0.98, 50.0, 5.6, "cubic"),
    (53.6, (0.0, -0.02, 0.635), -150.0, 1.31, 0.95, 50.0, 5.6, "cubic"),
    (56.4, (-0.01, 0.10, 0.54), -160.0, 1.66, 0.38, 50.0, 5.6, "cubic"),
    (59.6, (-0.01, 0.12, 0.54), -163.0, 1.58, 0.36, 50.0, 5.6, "cubic"),
    # firing: swing wide to the left side of the lengthwise section
    (61.1, (0.0, -0.03, 0.57), -122.0, 1.85, 0.32, 50.0, 6.3, "cubic"),
    (62.6, (0.0, -0.08, 0.585), -90.0, 1.30, 0.10, 50.0, 6.3, "cubic"),
    (73.4, (0.0, -0.08, 0.585), -95.0, 1.26, 0.10, 50.0, 6.3, "cubic"),
    # flywheel: around the back (engine closing up), ending on the friction face
    (76.2, (0.0, -0.18, 0.47), -50.0, 1.95, 0.36, 50.0, 5.6, "cubic"),
    (78.8, (0.0, -0.28, 0.42), -16.0, 1.50, 0.24, 50.0, 5.6, "cubic"),
    (DUR, (0.0, -0.33, 0.40), 10.0, 1.16, 0.30, 50.0, 5.6, "cubic"),
]


# key-light azimuth relative to the camera azimuth (deg; the studio rig turns with the camera)
KEY_OFFSET = [(0.0, 50.0, "step"), (48.6, 50.0, "linear"), (50.6, 75.0, "ease"), (60.2, 75.0, "linear"),
              (62.2, 50.0, "ease"), (76.0, 50.0, "linear"), (79.5, 75.0, "ease")]


def _pose_curves():
    cs = {k: state.Curve() for k in ("tx", "ty", "tz", "az", "r", "h", "lens", "f")}
    for (t, T, az, r, h, lens, f, mode) in POSES:
        for k, v in zip(("tx", "ty", "tz", "az", "r", "h", "lens", "f"), (T[0], T[1], T[2], az, r, h, lens, f)):
            cs[k].key(t, v, mode)
    return cs


CAM_SMOOTH = 0.3          # s: Gaussian low-pass of the pose parameters (softens starts/stops)


def _gauss(x, sigma_frames):
    r = int(3 * sigma_frames)
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma_frames) ** 2)
    k /= k.sum()
    xp = np.r_[np.full(r, x[0]), x, np.full(r, x[-1])]
    return np.convolve(xp, k, mode="valid")


def camera_samples(t):
    """Per-frame (eye, target, lens, f-stop, azimuth deg) for frame times t (uniform)."""
    cs = _pose_curves()
    sig = CAM_SMOOTH * FPS
    p = {k: _gauss(c(t), sig) for k, c in cs.items()}
    T = np.stack([p["tx"], p["ty"], p["tz"]], 1)
    az = np.radians(p["az"])
    r, h = p["r"], p["h"]
    eye = T + np.stack([r * np.sin(az), -r * np.cos(az), h], 1)
    return eye, T, p["lens"], p["f"], np.degrees(az)


def ring_in_frame(eye, tgt, lens, sensor=36.0, aspect=16 / 9, margin=1.06):
    """Per-frame: does any point of the 132T starter ring project inside the frame?
    (frustum only: occlusion is not credited)."""
    ang = np.linspace(0, 2 * np.pi, 72, endpoint=False)
    pts = []
    for y in (S.Y_FLYWHEEL_FRONT, S.Y_FLYWHEEL_FRONT - 0.013):
        for rr in (0.151, 0.136):
            pts.append(np.stack([rr * np.cos(ang), np.full_like(ang, y), S.Z_CRANK + rr * np.sin(ang)], 1))
    pts = np.concatenate(pts)
    out = np.zeros(len(eye), bool)
    up = np.array([0.0, 0.0, 1.0])
    for i in range(len(eye)):
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
        out[i] = bool(np.any(ok))
    return out


def ring_pitch_per_frame(theta):
    """Starter-ring teeth passing per frame (centred difference of the crank angle)."""
    d = np.abs(np.diff(theta)) / (2 * math.pi / S.FLYWHEEL_RING_TEETH)
    return 0.5 * (np.r_[d[0], d] + np.r_[d, d[-1]])


def shutter_for_pitch(p):
    """Shutter (frames) for the ring's motion blur: one full tooth pitch of smear while the
    ring moves >= 0.5 pitch/frame, easing back to a 1-frame shutter below 0.4 pitch/frame."""
    p = np.asarray(p, dtype=float)
    full = 1.0 / np.maximum(p, 1e-6)
    w = kin.smoothstep(0.4, 0.5, p)
    return np.clip(1.0 + (np.minimum(full, 2.0) - 1.0) * w, 1.0, 2.0) * (p < 1.0) + 1.0 * (p >= 1.0)


# ---------------------------------------------------------------------------
# build
# ---------------------------------------------------------------------------

def _fade_curve(t, keys):
    c = state.Curve()
    for k in keys:
        c.key(*k)
    return c(t)


def build(quality: str) -> scenebase.SceneBuild:
    sc = scenebase.new_scene(SCENE_ID)
    studio = lighting.setup_studio("dark", center=(0.0, -0.06, 0.48), size=0.7)
    lighting.setup_color_management(sc)
    E = ENG.build({"cutaways": ["none", "long", "cyl1", "front"],
                   "detail": "low" if quality == "preview" else "high"})
    # Workbench previews: shadow volumes x 8 AA samples cost ~5 s/frame on llvmpipe with
    # this many triangles; previews are for motion/framing (Cycles ignores this flag)
    for ob in E.objects():
        if ob.type == "MESH":
            ob.display.show_shadows = False

    # ---------------- drivetrain state ----------------------------------
    P = state.Program(SCENE_ID)
    k_valve = solve_valve_slowmo()
    P.slowmo = slowmo_curve(k_valve)
    P.throttle_rpm.key(0.0, S.IDLE_RPM, "step")
    P.align_engine(T_INTAKE, 360.0)
    track = P.run()
    th = track.theta_e
    t = track.t
    n = track.n

    # ---------------- presentation arrays -------------------------------
    dom = [_dominant_variant(x) for x in t]
    gas_long = np.array([0.7 if (d == "long" and x < 20) else (1.0 if d == "long" else 0.0)
                         for d, x in zip(dom, t)])
    gas_cyl1 = np.array([1.0 if d == "cyl1" else 0.0 for d in dom])
    # cylinder 1's gas fades as the front of the block closes over it (cyl1 -> front)
    v_front = [s for s in VARIANT_STEPS if s[0] == "front"][0]
    gas_cyl1 = np.where((t >= v_front[1]) & (t <= v_front[2] + 0.5),
                        gas_cyl1 * (1.0 - kin.smoothstep(v_front[1], v_front[1] + 1.2, t)), gas_cyl1)
    gas_cyl1 = np.where(t > v_front[2] + 0.5, 0.0, gas_cyl1)
    v_last = VARIANT_STEPS[-1]
    gas_long = gas_long * np.where(t > v_last[1], 1.0 - kin.smoothstep(v_last[1], v_last[1] + 1.4, t), 1.0)
    E.drive(track, {"gas_sets": {"full": np.zeros(n), "long": gas_long, "cyl1": gas_cyl1}})
    op = _bake_variants(E, track, {})

    # ---------------- per-object motion blur: flywheel + ring gear only -------
    p_ring = ring_pitch_per_frame(th)
    shutter = shutter_for_pitch(p_ring)
    rig.bake_channel(sc, "render.motion_blur_shutter", -1, track.frames, shutter)

    # glow: connecting rods, then crankshaft (crank beat)
    t_rod, t_crank = wt("crank", "Connecting"), wt("crank", "crankshaft")
    g_rod = _fade_curve(t, [(0.0, 0.0, "step"), (t_rod, 0.0, "linear"), (t_rod + 0.6, GLOW_PEAK, "ease"),
                            (t_crank, GLOW_PEAK, "linear"), (t_crank + 0.6, GLOW_HOLD, "ease"),
                            (bend("crank") - 0.6, GLOW_HOLD, "linear"), (bend("crank") + 0.6, 0.0, "ease")])
    g_crank = _fade_curve(t, [(0.0, 0.0, "step"), (t_crank, 0.0, "linear"), (t_crank + 0.6, GLOW_PEAK_CRANK, "ease"),
                              (bend("crank") - 1.2, GLOW_PEAK_CRANK, "linear"), (bend("crank") + 0.6, 0.0, "ease")])
    for c in range(1, 5):
        rig.bake_prop(E.parts[f"conrod{c}"], "cv_glow", track.frames, g_rod)
    rig.bake_prop(E.parts["crankshaft"], "cv_glow", track.frames, g_crank)

    # ---------------- camera + lights -----------------------------------
    eye, tgt, lens, fstop, az = camera_samples(t)
    C = CAM.CameraPath(name="cam_s02", lens=50.0, fstop=5.6, clip=(0.02, 60.0))
    for i in range(n):
        C.key(t[i], eye=tuple(eye[i]), target=tuple(tgt[i]), lens=float(lens[i]), fstop=float(fstop[i]),
              mode="linear")
    cam = C.bake(track.frames, FPS)
    # key light follows the camera (key ~50 deg to the camera's right, as in the engine look-dev)
    key_off = _fade_curve(t, KEY_OFFSET)
    rig.bake_channel(studio["rig"], "rotation_euler", 2, track.frames,
                     np.radians(np.unwrap(az, period=360.0) + key_off - 225.0))
    # Cycles motion blur only on the flywheel + ring gear (set after every object exists:
    # the camera's own flag also controls camera-motion blur, which must stay off)
    blurred = {E.parts["flywheel"].name, E.parts["ring_gear"].name}
    for ob in sc.objects:
        ob.cycles.use_motion_blur = ob.name in blurred

    # ---------------- validation ----------------------------------------
    ring_vis = ring_in_frame(eye, tgt, lens)
    ring_vis = ring_vis | np.r_[ring_vis[1:], False] | np.r_[False, ring_vis[:-1]]
    # The ring gear is motion-blurred in every frame (per-object blur, keyed shutter).  Frames
    # where that blur hides strobing are excluded from its aliasing check: the smear covers
    # >= 0.95 of a tooth pitch (uniform band) or the ring moves <= 0.5 pitch/frame (the smear
    # joins consecutive positions, so the motion cannot read backwards).  Every other frame
    # in which the ring is in frame is still checked against the 0.35 pitch/frame limit.
    blur_hides = (shutter * p_ring >= 0.95) | (p_ring <= 0.5)
    ring_check = ring_vis & ~blur_hides
    aliasing = {
        "flywheel ring gear 132T (unless motion-blurred)": (th, 2 * math.pi / S.FLYWHEEL_RING_TEETH, ring_check),
        "crank sprocket 21T": (th, 2 * math.pi / S.CRANK_SPROCKET_TEETH, None),
        "cam sprockets 42T": (kin.cam_angle(th), 2 * math.pi / S.CAM_SPROCKET_TEETH, None),
        "timing chain (pitch)": (kin.chain_travel(th), S.CHAIN_PITCH, None),
        "flywheel bolt holes 6": (th, 2 * math.pi / 6, None),
    }
    track.validate(aliasing=aliasing)

    # ---------------- labels --------------------------------------------
    L = Labels()

    def pieces(*keys):
        return tuple(E.parts[k] for k in keys if k in E.parts)

    t_cyl, t_pis = wt("inline4", "cylinders"), wt("inline4", "piston")
    for c in range(1, 5):
        L.add(f"cyl{c}", str(c), E.anchors[f"cyl{c}"], t_cyl + 0.12 * (c - 1), t_pis - 0.1, offset=(0.0, -0.13),
              style="dim", occlusion=False)
    L.add("piston", "Piston", (E.parts["piston2"], (-0.03, 0.0, 0.012)), t_pis, wt("crank", "crankshaft") - 0.3,
          offset=(-0.06, -0.10))
    L.add("conrod", "Connecting rod", (E.parts["conrod4"], (0.0, 0.0, 0.42 * S.CONROD_LENGTH)), t_rod,
          bend("crank") - 0.2, offset=(0.08, -0.04), occlusion=False)
    L.add("crankshaft", "Crankshaft", E.anchors["crankshaft"], t_crank, bend("crank") - 0.2, offset=(-0.07, 0.08))
    # strokes (cylinder 1 cross-section, seen from the front: intake side on the right)
    L.add("intake_valve", "Intake valve", E.anchors["intake_valve1"], wt("intake", "intake valves"),
          wt("compression", "squeezes"), offset=(0.10, -0.06), ignore=pieces("valve_1i0__cyl1_kept"))
    L.add("spark_plug", "Spark plug", E.anchors["spark_plug1"], bend("compression") - 3.0,
          wt("power", "hot") + 0.4, offset=(0.09, -0.07), occlusion=False)
    L.add("exhaust_valve", "Exhaust valve", E.anchors["exhaust_valve1"], wt("exhaust", "exhaust valves"),
          bend("exhaust") - 0.3, offset=(-0.10, -0.06), ignore=pieces("valve_1e0__cyl1_kept"))
    # valvetrain
    t_cams = wt("valvetrain", "Camshafts")
    t_chain = wt("valvetrain", "timing chain")
    t_cs, t_half = wt("valvetrain", "crankshaft"), wt("valvetrain", "half")
    # seen from the front-left: intake side (-X) on screen right, exhaust side (+X) on screen left
    L.add("cam_in", "Intake camshaft", E.anchors["cam_intake"], t_cams + 0.6, t_cs - 0.2, offset=(0.09, -0.05),
          ignore=pieces("cam_intake__cyl1_kept", "cam_intake__cyl1_removed"))
    L.add("cam_ex", "Exhaust camshaft", E.anchors["cam_exhaust"], t_cams + 0.6, t_cs - 0.2, offset=(-0.09, -0.05),
          ignore=pieces("cam_exhaust__cyl1_kept", "cam_exhaust__cyl1_removed"))
    t_vend = VARIANT_STEPS[3][1] - 0.15
    L.add("chain", "Timing chain", (E.root, (ENG.X_CAM + 0.045, E.meta["y"]["chain"], 0.5 * ENG.Z_CAM)), t_chain,
          t_vend, offset=(-0.09, 0.0), occlusion=False)
    L.add("crank_spr", "Crank sprocket 21T", (E.root, (-0.026, E.meta["y"]["chain"] + 0.004, 0.0)), t_cs, t_vend,
          offset=(0.09, 0.0),
          occlusion=False)
    L.add("cam_spr", "Cam sprocket 42T", E.anchors["cam_sprocket"], t_half, t_vend, offset=(0.09, -0.04),
          occlusion=False)
    # firing: cylinder numbers again
    for c in range(1, 5):
        L.add(f"fcyl{c}", str(c), (E.root, (0.0, S.Y_CYL[c - 1], ENG.Z_APEX + 0.02)), T_FIRING + 1.0,
              bend("firing") - 0.4, offset=(0.0, -0.12), style="dim", occlusion=False)
    # flywheel
    L.add("flywheel", "Flywheel", E.anchors["flywheel_rim"], wt("flywheel", "flywheel"), DUR - 0.3,
          offset=(-0.09, -0.07), occlusion=False)
    L.add("face", "Friction face", (E.root, (0.075, S.Y_FLYWHEEL_FACE, -0.045)), wt("flywheel", "face"), DUR - 0.3,
          offset=(0.10, 0.02), occlusion=False)

    # ---------------- HUD -----------------------------------------------
    H = Hud(track)
    H.add("fade", 0.0, 0.8, fade=0.0, color="black", alpha=lambda i: float(max(0.0, 1.0 - t[i] / 0.7)))
    H.add("fade", DUR - 0.6, DUR + 1.0, fade=0.0, color="black",
          alpha=lambda i: float(min(1.0, max(0.0, (t[i] - (DUR - 0.55)) / 0.5))))
    H.add("section_title", 0.4, 4.4, number=2, title="The engine")
    H.add("rpm", 0.6, DUR, value=lambda i: fmt_rpm(track.rpm_e[i]), label="Engine")
    H.add("slowmo", 0.6, DUR, factor=lambda i: int(round(1.0 / track.slowmo[i])))
    strokes = [kin.stroke_of(x, 1) for x in th]
    H.add("stroke_strip", bstart("cycle") + 3.6, bend("exhaust") + 0.2, active=lambda i: strokes[i][0],
          progress=lambda i: round(strokes[i][2], 4), cylinder=1)
    H.add("readouts", t_cams + 0.6, VARIANT_STEPS[3][1] + 0.4,
          rows=lambda i: [["Crank", f"{fmt_rpm(track.rpm_e[i])} rpm"],
                          ["Camshafts", f"{fmt_rpm(track.rpm_e[i] * S.CAM_SPEED_RATIO)} rpm"]])

    def firing_cyl(i):
        # cylinder whose power stroke is under way (from its spark, 15 deg BTDC)
        best, ph = None, 1e9
        for c in S.FIRING_ORDER:
            phi = (float(kin.cycle_angle_deg(th[i], c)) + S.SPARK_ADVANCE_DEG) % 720.0
            if phi < ph:
                best, ph = c, phi
        return best
    H.add("firing_ticker", T_FIRING + 0.5, bend("firing") - 0.3, cylinder=firing_cyl)

    sb = scenebase.SceneBuild(SCENE_ID, track, cam, L, H, motion_blur=True, shutter=1.0, preview_hide=())
    sb.extra.update(engine=E, k_valve=k_valve, ring_visible=ring_vis, ring_pitch=p_ring, shutter=shutter,
                    opacity=op, studio=studio)
    return sb
