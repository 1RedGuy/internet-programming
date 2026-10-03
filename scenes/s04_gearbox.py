"""s04 The gearbox (91 s): three shafts, constant mesh, synchronisers, linkage, ratios, reverse.

Dark studio; the gearbox (half cutaway: the -X half of the case, web and tail housing
fades away in the first beat) with the clutch in its half-cut bellhousing at the front as
context.  The camera works on the -X (cut) side except for the reverse shot, which looks
at the reverse train from the +X side with the housings removed (the idler sits on +X,
behind the countershaft/reverse gears as seen from -X).

Presentation: the 1-2 synchro is exploded (assembly explode carriers) with the parts it
passes through (meta['explode_hide']), the selector and the countershaft faded out; for
the lock and linkage beats the output-side 1-2 synchro parts (hub, sleeve, struts,
blocker rings; stationary because the car is) are swapped for their quarter-section
copies so the sleeve can be seen sliding over the blocker ring and the dog teeth.
Warm glow (cv_glow): each shaft / synchro part as it is named, the selected rail + fork
+ sleeve in the linkage beat, and meta['power_path'][gear] in the driving shots.

Drivetrain program (state.Program; video time):
  shafts, neutral   idle 850 rpm, neutral, clutch engaged, car stationary: input side
                    turns, output shaft + hubs + sleeves stand still.  x72 slow motion
                    (5th gear's 32 dogs at 1043 rpm need >= x66, FACTS PRS-04).
  synchro           clutch pressed at the start (pedal 0 -> 1); the input side coasts
                    (oil drag).  1-2 synchroniser exploded and reassembled.
  lock              lever across to the 1-2 plane, then engage(1): blocker ring indexes,
                    the synchro stops the input side (car stationary), sleeve passes the
                    blocker ring and slides over 1st gear's dog teeth.
  linkage           clutch still pressed: 1 -> N -> across to 5-R -> 5 -> N -> across
                    -> 1 (rails, forks, sleeves follow the lever).
  ratios            hard cuts (P.cut) to driving shots at 1500 rpm, clutch engaged:
                    1st (12.1 km/h), 4th (42.1 km/h), 5th (51.6 km/h); x130 (the 32 dog
                    teeth of 5th gear at 1841 rpm need >= x117).
  reverse           cut: reverse engaged, reversing at 950 rpm (-7.8 km/h), x80 (5th
                    gear's dogs at 1166 rpm need >= x74).
Cut times sit half a frame before a frame boundary, so the first frame of every new shot
already shows the settled state (speeds, gear) and the new camera.
"""
from __future__ import annotations

import math

import numpy as np

from carviz import camera as CAM
from carviz import kin, lighting, rig, scenebase, state, timeline
from carviz import spec as S
from carviz.assemblies import clutch as CLU
from carviz.assemblies import gearbox as GBX
from carviz.labels import Hud, Labels, fmt_rpm

SCENE_ID = "s04"
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


def cut_time(t):
    """Cut instant half a frame before the frame nearest t (see module docstring)."""
    return (round(t * FPS) - 0.5) / FPS


# ---------------------------------------------------------------------------
# Timing
# ---------------------------------------------------------------------------
T_SYN = bstart("synchro")                      # 29.5
T_LOCK = bstart("lock")                        # 45.5
T_LINK = bstart("linkage")                     # 53.0
T_RAT = cut_time(bstart("ratios"))             # 63.48: cut to 1st-gear driving
T_4TH = cut_time(wt("ratios", "Fourth") - 0.3)  # cut to 4th
T_5TH = cut_time(wt("ratios", "Fifth") - 0.3)   # cut to 5th
T_REV = cut_time(bstart("reverse"))            # cut to reverse
SHOTS = [0.0, T_RAT, T_4TH, T_5TH, T_REV, DUR + 1.0]

# slow motion (sim s per video s)
SLOW_IDLE = 1.0 / 72.0
SLOW_DRIVE = 1.0 / 130.0
SLOW_REV = 1.0 / 80.0
DRIVE_RPM = 1500.0
REV_RPM = 950.0

# clutch pedal (synchro beat) and the shift program
PEDAL_DOWN = (T_SYN + 0.15, T_SYN + 1.35)
LOCK_SELECT = (T_LOCK + 0.10, 0.45)            # lever across to the 1-2 plane (t0, dur)
LOCK_ENGAGE = dict(t0=T_LOCK + 0.65, travel=0.6, hold=1.4, through=0.9, seat=0.5)

# presentation timing
CASE_FADE = (0.5, 3.0)                         # removed half of the case/bellhousing fades out
EXPLODE = (31.6, 33.6, 43.3, 44.9)             # explode out / hold / back in
HIDE_FADE = (30.4, 31.5, 45.0, 45.9)           # parts the explode passes through
SEL_FADE = (30.4, 31.5, 52.7, 53.6)            # rails, forks, shift heads, detents
SEC_SYN_IN = (45.0, 45.9)                      # 1-2 synchro (output side) section copies (lock, linkage)

GLOW_PEAK = 0.06
GLOW_HOLD = 0.02
GLOW_PATH = 0.05
GLOW_FREE = 0.04
GLOW_IDLER = 0.10
GHOST = 0.0                                    # housings in the reverse shot (seen from +X): removed


# ---------------------------------------------------------------------------
# Drivetrain program
# ---------------------------------------------------------------------------

def make_program():
    P = state.Program(SCENE_ID)
    # slow motion (held until the next step key)
    P.slowmo.key(0.0, SLOW_IDLE, "step")
    P.slowmo.key(T_RAT, SLOW_DRIVE, "step")
    P.slowmo.key(T_REV, SLOW_REV, "step")
    # road speed: stationary, then the driving shots
    P.speed_kmh.key(0.0, 0.0, "step")
    P.speed_kmh.key(T_RAT, S.road_speed_kmh(DRIVE_RPM, 1), "step")
    P.speed_kmh.key(T_4TH, S.road_speed_kmh(DRIVE_RPM, 4), "step")
    P.speed_kmh.key(T_5TH, S.road_speed_kmh(DRIVE_RPM, 5), "step")
    P.speed_kmh.key(T_REV, S.road_speed_kmh(REV_RPM, "R"), "step")
    P.throttle_rpm.key(0.0, S.IDLE_RPM, "step")
    P.throttle_rpm.key(T_RAT, DRIVE_RPM, "step")
    P.throttle_rpm.key(T_REV, REV_RPM, "step")
    # clutch: pressed at the start of the synchro beat, released (engaged) at the cut
    P.pedal.key(0.0, 0.0, "step")
    P.pedal.key(PEDAL_DOWN[0], 0.0, "linear")
    P.pedal.key(PEDAL_DOWN[1], 1.0, "ease")
    P.pedal.key(T_RAT, 0.0, "step")
    # ---- lock: lever across to the 1-2 plane, engage 1st (the synchro stops the input side)
    P.select_plane(LOCK_SELECT[0], -1, LOCK_SELECT[1])
    e = LOCK_ENGAGE
    t_seated = P.engage(1, e["t0"], travel=e["travel"], hold=e["hold"], through=e["through"], seat=e["seat"])
    assert t_seated < T_LINK
    # ---- linkage: 1 -> N, across to 5-R, 5, N, across, 1
    P.disengage(1, 54.75, 0.6)
    P.select_plane(56.55, 0, 0.55)
    P.select_plane(57.6, 1, 0.55)
    P.engage(5, 59.35, travel=0.35, hold=0.2, through=0.3, seat=0.25)
    P.disengage(5, 60.85, 0.4)
    P.select_plane(61.45, -1, 0.75)
    t1 = P.engage(1, 62.35, travel=0.3, hold=0.15, through=0.25, seat=0.15)
    assert t1 < T_RAT
    # ---- hard cuts between the driving shots (sleeves + lever jump)
    P.sleeve["12"].key(T_4TH, 0.0, "step")
    P.sleeve["34"].key(0.0, 0.0, "step")
    P.sleeve["34"].key(T_4TH, 1.0, "step")
    P.sleeve["34"].key(T_5TH, 0.0, "step")
    P.sleeve["5R"].key(T_5TH, -1.0, "step")
    P.sleeve["5R"].key(T_REV, 1.0, "step")
    P.lever_plane.key(T_4TH, 0.0, "step")
    P.lever_plane.key(T_5TH, 1.0, "step")
    for tc in (T_RAT, T_4TH, T_5TH, T_REV):
        P.cut(tc)
    return P


def shot_of(t):
    """Index of the shot (0 = stationary demo, 1 = 1st, 2 = 4th, 3 = 5th, 4 = reverse)."""
    return int(np.searchsorted(np.array(SHOTS[1:-1]), t, side="right"))


# ---------------------------------------------------------------------------
# Curves / helpers
# ---------------------------------------------------------------------------

def curve(t, keys, default=0.0):
    c = state.Curve(default)
    for k in keys:
        c.key(*k)
    return c(t)


def pulse(t, t_on, t_off, peak=GLOW_PEAK, rise=0.5, fall=0.8, hold=0.0):
    """0 -> peak at t_on (+rise), held, -> hold level at t_off (+fall)."""
    return curve(t, [(0.0, 0.0, "step"), (t_on, 0.0, "linear"), (t_on + rise, peak, "ease"),
                     (t_off, peak, "linear"), (t_off + fall, hold, "ease")])


def window(t, t0, t1, a=0.4, b=0.4):
    """Smooth 0..1..0 window (fade in over a after t0, out over b before t1)."""
    u = np.clip((t - t0) / max(a, 1e-6), 0, 1)
    v = np.clip((t1 - t) / max(b, 1e-6), 0, 1)
    return np.minimum(u * u * (3 - 2 * u), v * v * (3 - 2 * v))


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


class Presentation:
    """Per-object opacity / glow arrays, baked once at the end."""

    def __init__(self, track, hide_threshold=0.02):
        self.n = track.n
        self.frames = track.frames
        self.op = {}
        self.glow = {}
        self.thr = hide_threshold

    def opacity(self, ob):
        if ob.name not in self.op:
            self.op[ob.name] = [ob, np.ones(self.n)]
        return self.op[ob.name][1]

    def mul(self, ob, arr):
        a = self.opacity(ob)
        a *= np.asarray(arr, float)

    def set(self, ob, arr):
        self.opacity(ob)
        self.op[ob.name][1] = np.asarray(arr, float).copy() * np.ones(self.n)

    def add_glow(self, ob, arr):
        if ob.name not in self.glow:
            self.glow[ob.name] = [ob, np.zeros(self.n)]
        self.glow[ob.name][1] = np.maximum(self.glow[ob.name][1], np.asarray(arr, float))

    def bake(self):
        fr = self.frames
        for name, (ob, a) in self.op.items():
            if ob.type != "MESH":
                continue
            a = np.round(np.clip(a, 0.0, 1.0), 4)
            hide = (a < self.thr).astype(float)
            _bake_compressed(ob, "hide_render", fr, hide, "CONSTANT")
            _bake_compressed(ob, "hide_viewport", fr, hide, "CONSTANT")
            if "cv_opacity" not in ob:
                ob["cv_opacity"] = 1.0
            if np.any((a >= self.thr) & (a < 0.999)):
                _bake_compressed(ob, '["cv_opacity"]', fr, np.where(hide > 0, 0.0, a))
            else:
                ob["cv_opacity"] = 1.0
        for name, (ob, g) in self.glow.items():
            if ob.type != "MESH" or not np.any(g > 1e-4):
                continue
            if "cv_glow" not in ob:
                ob["cv_glow"] = 0.0
            _bake_compressed(ob, '["cv_glow"]', fr, np.round(g, 4))


# ---------------------------------------------------------------------------
# Camera plan: (time, target, azimuth deg, radius, eye height above target, lens, f-stop)
# azimuth as camera.orbit: 0 = behind (-Y), -90 = left (-X, the cut side), +90 = right
# Shots are separated by the hard cuts; each shot is smoothed on its own.
# ---------------------------------------------------------------------------
POSES = [
    # --- shafts: establishing 3/4 front-left, case opens, front half, then the whole length
    (0.0, (0.0, -0.72, 0.53), -130.0, 2.35, 0.74, 50.0, 5.6),
    (1.8, (0.0, -0.70, 0.51), -126.0, 2.20, 0.68, 50.0, 5.6),
    (4.2, (0.0, -0.53, 0.34), -112.0, 1.15, 0.30, 50.0, 5.6),
    (6.6, (0.0, -0.55, 0.34), -106.0, 1.12, 0.26, 50.0, 5.6),
    (9.2, (0.0, -0.71, 0.35), -92.0, 1.75, 0.30, 50.0, 5.6),
    (12.4, (0.0, -0.73, 0.35), -88.0, 1.65, 0.26, 50.0, 5.6),
    # --- neutral: in at the rear, track forward along the pairs, 2nd gear's bearing, pull back
    (14.4, (0.0, -0.75, 0.315), -84.0, 0.93, 0.12, 50.0, 6.3),
    (18.6, (0.0, -0.63, 0.315), -95.0, 0.91, 0.12, 50.0, 6.3),
    (21.4, (0.0, -0.612, 0.33), -102.0, 0.74, 0.14, 50.0, 7.1),
    (23.9, (0.0, -0.614, 0.33), -99.0, 0.74, 0.14, 50.0, 7.1),
    (26.0, (0.0, -0.78, 0.35), -88.0, 1.25, 0.20, 50.0, 6.3),
    (28.8, (0.0, -0.77, 0.35), -90.0, 1.22, 0.20, 50.0, 6.3),
    # --- synchro: push in on the 1-2 synchro, exploded view
    (31.4, (0.0, -0.634, 0.362), -114.0, 0.58, 0.17, 50.0, 8.0),
    (43.0, (0.0, -0.634, 0.362), -108.0, 0.56, 0.16, 50.0, 8.0),
    # --- lock: close on the sleeve, blocker ring and 1st gear's dogs
    (45.7, (0.0, -0.648, 0.388), -92.0, 0.32, 0.19, 50.0, 8.0),
    (51.6, (0.0, -0.648, 0.388), -88.0, 0.31, 0.19, 50.0, 8.0),
    # --- linkage: crane up to a high rear-left view: lever, finger, rails, forks, sleeves
    (52.2, (0.0, -0.648, 0.388), -88.0, 0.31, 0.19, 50.0, 8.0),
    (54.3, (0.0, -0.78, 0.55), -55.0, 1.04, 0.94, 50.0, 6.3),
    (63.3, (0.0, -0.775, 0.55), -61.0, 1.02, 0.90, 50.0, 6.3),
    # --- ratios, 1st: headset + 1st pair (cut)
    (T_RAT, (0.0, -0.62, 0.33), -97.0, 0.98, 0.10, 50.0, 6.3),
    (T_4TH - 0.01, (0.0, -0.625, 0.33), -92.0, 0.92, 0.10, 50.0, 6.3),
    # --- 4th: input gear dogs + 3-4 synchro
    (T_4TH, (0.0, -0.56, 0.35), -104.0, 0.80, 0.12, 50.0, 7.1),
    (T_5TH - 0.01, (0.0, -0.56, 0.35), -100.0, 0.76, 0.12, 50.0, 7.1),
    # --- 5th: whole train from the left-rear
    (T_5TH, (0.0, -0.70, 0.34), -79.0, 1.10, 0.14, 50.0, 6.3),
    (T_REV - 0.01, (0.0, -0.70, 0.34), -83.0, 1.06, 0.14, 50.0, 6.3),
    # --- reverse: +X side (the idler is on +X), housings removed; close on the reverse train,
    #     then back to include the output shaft + flange turning backwards
    (T_REV, (0.02, -0.75, 0.32), 97.0, 0.64, 0.06, 50.0, 7.1),
    (86.4, (0.02, -0.75, 0.32), 95.0, 0.63, 0.06, 50.0, 7.1),
    (88.2, (0.02, -0.86, 0.34), 92.0, 0.86, 0.05, 50.0, 7.1),
    (DUR, (0.02, -0.86, 0.34), 90.0, 0.83, 0.05, 50.0, 7.1),
]
KEY_OFFSET = 50.0           # key light azimuth relative to the camera (deg)
CAM_SMOOTH = 0.35           # s, Gaussian low-pass of the pose parameters (within a shot)


def _gauss(x, sigma_frames):
    r = int(3 * sigma_frames)
    if r < 1 or len(x) < 2:
        return x
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma_frames) ** 2)
    k /= k.sum()
    xp = np.r_[np.full(r, x[0]), x, np.full(r, x[-1])]
    return np.convolve(xp, k, mode="valid")


def camera_samples(t):
    keys = ("tx", "ty", "tz", "az", "r", "h", "lens", "f")
    out = {k: np.zeros(len(t)) for k in keys}
    shot = np.array([shot_of(x) for x in t])
    for s in range(len(SHOTS) - 1):
        sel = shot == s
        if not np.any(sel):
            continue
        ps = [p for p in POSES if shot_of(p[0]) == s]
        cs = {k: state.Curve() for k in keys}
        for j, (tp, T, az, r, h, lens, f) in enumerate(ps):
            mode = "step" if j == 0 else "cubic"
            for k, v in zip(keys, (T[0], T[1], T[2], az, r, h, lens, f)):
                cs[k].key(tp, v, mode)
        for k in keys:
            out[k][sel] = _gauss(cs[k](t[sel]), CAM_SMOOTH * FPS)
    T = np.stack([out["tx"], out["ty"], out["tz"]], 1)
    az = np.radians(out["az"])
    eye = T + np.stack([out["r"] * np.sin(az), -out["r"] * np.cos(az), out["h"]], 1)
    return eye, T, out["lens"], out["f"], out["az"]


# ---------------------------------------------------------------------------
# build
# ---------------------------------------------------------------------------

SELECTOR = tuple(f"{p}_{k}" for k in ("12", "34", "5R")
                 for p in ("rail", "fork", "head", "detent_ball", "detent_spring"))
SYNCHRO_HIDE = ("countershaft", "cs_5") + tuple(f"brg_cs_{b}_{r}" for b in ("front", "rear")
                                               for r in ("inner", "outer", "rolling"))
CLUTCH_SHOW = ("disc", "damper_springs", "pressure_plate", "straps", "diaphragm_spring", "fulcrum", "cover",
               "release_bearing", "bearing_race", "fork", "ball_stud", "guide_tube", "slave_cylinder",
               "slave_pushrod")


def build(quality: str) -> scenebase.SceneBuild:
    sc = scenebase.new_scene(SCENE_ID)
    studio = lighting.setup_studio("dark", center=(0.0, -0.70, 0.40), size=0.7)
    lighting.setup_color_management(sc)
    detail = "low" if quality == "preview" else "high"
    G = GBX.build({"cutaway": "half", "detail": detail, "sections": ["synchro_12"]})
    C = CLU.build({"cutaway": ["half"], "detail": detail})
    GP, CP = G.parts, C.parts
    for asm in (G, C):
        for ob in asm.objects():
            if ob.type == "MESH":
                ob.display.show_shadows = False

    # ---------------- drivetrain state ----------------------------------
    P = make_program()
    track = P.run()
    t = track.t
    n = track.n
    fr = track.frames
    shot = np.array([shot_of(x) for x in t])

    # ---------------- motion --------------------------------------------
    explode = curve(t, [(0.0, 0.0, "step"), (EXPLODE[0], 0.0, "linear"), (EXPLODE[1], 1.0, "ease"),
                        (EXPLODE[2], 1.0, "linear"), (EXPLODE[3], 0.0, "ease")])
    G.drive(track, {"explode": explode})
    # (1 - s)^2: a faded closed shell still hides 1 - (1 - a)^2 of what is behind it, so a
    # plain smoothstep would read as a late, quick dissolve
    case_rm = (1.0 - kin.smoothstep(CASE_FADE[0], CASE_FADE[1], t)) ** 2
    C.drive(track, {"variant": "half", "removed": case_rm})

    # ---------------- presentation (opacity, glow) ----------------------
    pres = Presentation(track, hide_threshold=0.5 if quality == "preview" else 0.02)
    rev = (shot == 4).astype(float)
    # housings: removed half fades out; the kept half goes for the reverse shot (seen from +X)
    for v in G.meta["cutaway_pieces"]["half"]["kept"]:
        pres.set(GP[v], 1.0 - rev * (1.0 - GHOST))
    for v in G.meta["cutaway_pieces"]["half"]["removed"]:
        pres.set(GP[v], case_rm)
    # clutch context: clutch pack + half bellhousing; hydraulics/pedal never shown
    for k, ob in CP.items():
        if ob.type != "MESH" or k.startswith("bellhousing"):
            continue
        if k in ("slave_cylinder", "slave_pushrod"):
            # bright, nearest the camera and without its hose: dropped for the driving shots
            pres.set(ob, (t < T_RAT).astype(float))
        elif k in CLUTCH_SHOW:
            pres.set(ob, np.ones(n))
        else:
            ob.hide_render = True
            ob.hide_viewport = True
    # parts the exploded 1-2 synchro passes through
    hide_op = curve(t, [(0.0, 1.0, "step"), (HIDE_FADE[0], 1.0, "linear"), (HIDE_FADE[1], 0.0, "ease"),
                        (HIDE_FADE[2], 0.0, "linear"), (HIDE_FADE[3], 1.0, "ease")])
    for k in G.meta["explode_hide"]:
        if k in GP and GP[k].type == "MESH" and k not in SELECTOR:
            pres.mul(GP[k], hide_op)
    # the bare countershaft goes with its gears
    for k in SYNCHRO_HIDE:
        if k in GP and k not in G.meta["explode_hide"]:
            pres.mul(GP[k], hide_op)
    # the selector (rails over the close-ups; the 1-2 fork in front of the sectioned sleeve)
    # leaves for the synchro and lock beats and is (re)introduced with the linkage
    sel_op = curve(t, [(0.0, 1.0, "step"), (SEL_FADE[0], 1.0, "linear"), (SEL_FADE[1], 0.0, "ease"),
                       (SEL_FADE[2], 0.0, "linear"), (SEL_FADE[3], 1.0, "ease")])
    for k in SELECTOR:
        if k in GP and GP[k].type == "MESH":
            pres.mul(GP[k], sel_op)
    # quarter-section copies of the output-side 1-2 synchro parts (lock + linkage; the car is
    # stationary, so they do not turn and the notch stays facing the camera)
    stationary = (shot == 0).astype(float)
    sec_sets = [
        (("hub_12", "sleeve_12", "strut_12_0", "strut_12_1", "strut_12_2", "blocker_1", "blocker_2"), SEC_SYN_IN),
    ]
    for keys, (a, b) in sec_sets:
        w_whole = curve(t, [(0.0, 1.0, "step"), (a, 1.0, "linear"), (b, 0.0, "ease")])
        w_whole = np.where(stationary > 0, w_whole, 1.0)
        on_sec = ((t >= a) & (stationary > 0)).astype(float)
        for k in keys:
            pres.mul(GP[k], w_whole)
            sk = G.meta["sections"][k]
            pres.set(GP[sk], on_sec)

    def grp(*keys):
        out = []
        for k in keys:
            if k in GP and GP[k].type == "MESH":
                out.append(GP[k])
                sk = G.meta["sections"].get(k)
                if sk:
                    out.append(GP[sk])
        return out

    def glow(objs, arr):
        for ob in objs:
            pres.add_glow(ob, arr)

    # shafts beat: each shaft lights up as it is named
    t_in, t_cs, t_out = wt("shafts", "input"), wt("shafts", "countershaft"), wt("shafts", "output")
    t_line = wt("shafts", "line")
    g_in = curve(t, [(0.0, 0.0, "step"), (t_in, 0.0, "linear"), (t_in + 0.5, GLOW_PEAK, "ease"),
                     (t_cs, GLOW_PEAK, "linear"), (t_cs + 0.8, GLOW_HOLD, "ease"), (t_line - 0.3, GLOW_HOLD, "linear"),
                     (t_line + 0.3, GLOW_PEAK, "ease"), (bend("shafts") - 0.8, GLOW_PEAK, "linear"),
                     (bend("shafts") + 0.4, 0.0, "ease")])
    glow(grp("input_shaft", "input_gear", "dogs_4", "cone_4"), g_in)
    g_cs = curve(t, [(0.0, 0.0, "step"), (t_cs, 0.0, "linear"), (t_cs + 0.5, GLOW_PEAK, "ease"),
                     (t_out, GLOW_PEAK, "linear"), (t_out + 0.8, GLOW_HOLD, "ease"),
                     (bend("shafts") - 0.8, GLOW_HOLD, "linear"), (bend("shafts") + 0.4, 0.0, "ease")])
    glow(grp("countershaft", "cs_drive", "cs_3", "cs_2", "cs_1", "cs_R", "cs_5"), g_cs)
    g_out = curve(t, [(0.0, 0.0, "step"), (t_out, 0.0, "linear"), (t_out + 0.5, GLOW_PEAK, "ease"),
                      (bend("shafts") - 0.8, GLOW_PEAK, "linear"), (bend("shafts") + 0.4, 0.0, "ease")])
    glow(grp("output_shaft", "output_flange", "washers"), g_out)
    # neutral: the free-spinning output gears
    t_free = wt("neutral", "spin")
    g_free = pulse(t, t_free - 0.2, wt("neutral", "Until") + 0.3, peak=GLOW_FREE, fall=0.9)
    for g in (1, 2, 3, 5, "R"):
        glow(grp(f"gear_{g}", f"dogs_{g}", f"cone_{g}"), g_free)
    # synchro: each part as it is named
    named = [("hub", ("hub_12", "strut_12_0", "strut_12_1", "strut_12_2"), wt("synchro", "hub"), 2.6),
             ("sleeve", ("sleeve_12",), wt("synchro", "sleeve"), 2.4),
             ("dogs", ("dogs_1",), wt("synchro", "dog"), 2.2),
             ("cone", ("cone_1",), wt("synchro", "cone"), 1.8),
             ("blocker", ("blocker_1",), wt("synchro", "brass"), 2.6)]
    for _nm, keys, t0, d in named:
        glow(grp(*keys), pulse(t, t0, t0 + d, fall=0.7))
    # lock: the gear is locked to the shaft
    t_lockw = wt("lock", "locked")
    glow(grp("sleeve_12"), pulse(t, wt("lock", "Slide") - 0.3, T_LINK - 0.6, fall=0.6))
    glow(grp("dogs_1", "hub_12", "gear_1"), pulse(t, t_lockw - 0.6, T_LINK - 0.6, fall=0.6))
    # linkage: the selected rail, its fork and sleeve
    lx = np.asarray(track.lever_x, float)
    in_link = window(t, T_LINK + 0.3, T_RAT, 0.6, 0.3)
    for k, plane in (("12", -1), ("34", 0), ("5R", 1)):
        sel = np.clip(1.0 - np.abs(lx - plane) * 2.5, 0.0, 1.0)
        glow(grp(f"rail_{k}", f"fork_{k}", f"head_{k}", f"sleeve_{k}"), GLOW_PEAK * sel * in_link)
    # ratios / reverse: power path of the engaged gear
    for s_idx, g in ((1, 1), (2, 4), (3, 5), (4, "R")):
        t0 = SHOTS[s_idx]
        on = ((t >= t0) & (t < SHOTS[s_idx + 1])).astype(float)
        ramp = np.clip((t - t0) / 0.5, 0.0, 1.0)
        ramp = ramp * ramp * (3 - 2 * ramp)
        glow(grp(*G.meta["power_path"][g]), GLOW_PATH * on * (0.55 + 0.45 * ramp))
    t_idl = wt("reverse", "idler")
    glow(grp("idler"), pulse(t, t_idl - 0.2, DUR, peak=GLOW_IDLER))
    pres.bake()

    # ---------------- camera + lights -----------------------------------
    eye, tgt, lens, fstop, az = camera_samples(t)
    CPATH = CAM.CameraPath(name="cam_s04", lens=50.0, fstop=5.6, clip=(0.02, 60.0))
    for i in range(n):
        CPATH.key(t[i], eye=tuple(eye[i]), target=tuple(tgt[i]), lens=float(lens[i]), fstop=float(fstop[i]),
                  mode="linear")
    cam = CPATH.bake(fr, FPS)
    rig.bake_channel(studio["rig"], "rotation_euler", 2, fr, np.radians(az + KEY_OFFSET - 225.0))

    # ---------------- validation ----------------------------------------
    aliasing = aliasing_dict(track, explode)
    track.validate(aliasing=aliasing)
    clear = GBX.synchro_clearance(track)
    assert clear["blocker_mm"] >= -1e-6 and clear["dogs_mm"] >= -1e-6, clear

    # ---------------- labels / HUD --------------------------------------
    L = make_labels(G, C)
    H = make_hud(track)
    preview_hide = ()
    sb = scenebase.SceneBuild(SCENE_ID, track, cam, L, H, motion_blur=False, preview_hide=preview_hide)
    sb.extra.update(gearbox=G, clutch=C, studio=studio, explode=explode, clearance=clear, aliasing=aliasing)
    return sb


# ---------------------------------------------------------------------------
# Aliasing (FACTS PRS-04): every visible toothed / featured part, pitch per frame < 0.35
# ---------------------------------------------------------------------------

def aliasing_dict(track, explode):
    th_in = np.asarray(track.theta_in, float)
    th_out = np.asarray(track.theta_out, float)
    th_e = np.asarray(track.theta_e, float)
    A = {}
    A["input shaft 23 clutch splines"] = (th_in, TAU / S.CLUTCH_DISC_SPLINE_TEETH, None)
    A["input gear 26T"] = (track.gb("input_gear"), TAU / S.Z_INPUT, None)
    A["input gear 4th dogs 32"] = (track.gb("input_gear"), TAU / S.DOG_TEETH, None)
    A["countershaft 35T"] = (track.gb("cs_drive"), TAU / S.Z_CS_DRIVEN, None)
    for g, (zc, zm) in S.GEAR_PAIRS.items():
        A[f"countershaft {zc}T ({g})"] = (track.gb(f"cs_{g}"), TAU / zc, None)
        A[f"gear {g} {zm}T"] = (track.gb(f"gear_{g}"), TAU / zm, None)
        A[f"gear {g} dogs 32"] = (track.gb(f"gear_{g}"), TAU / S.DOG_TEETH, None)
    A["countershaft 15T (R)"] = (track.gb("cs_R"), TAU / S.Z_REV_CS, None)
    A["idler 22T"] = (track.gb("idler"), TAU / S.Z_REV_IDLER, None)
    A["reverse gear 38T"] = (track.gb("gear_R"), TAU / S.Z_REV_OUT, None)
    A["reverse gear dogs 32"] = (track.gb("gear_R"), TAU / S.DOG_TEETH, None)
    A["hubs / sleeves 32"] = (th_out, TAU / S.DOG_TEETH, None)
    A["output flange 4 bolts"] = (th_out, TAU / 4, None)
    # needle cages (36 rollers): only 1st/2nd are ever exposed (exploded view; 2nd's section)
    vis_n = np.asarray(explode) > 0.02
    for g in (1, 2):
        cage = (th_out * GBX.R_JOURNAL + track.gb(f"gear_{g}") * GBX.R_GB) / (GBX.R_JOURNAL + GBX.R_GB)
        A[f"needle cage {g} (36)"] = (cage, TAU / 36, vis_n)
    # clutch (context)
    A["diaphragm fingers 18"] = (th_e, TAU / S.DIAPHRAGM_FINGERS, None)
    A["clutch disc damper springs 6"] = (th_in, TAU / S.CLUTCH_DAMPER_SPRINGS, None)
    # a hard cut has no temporal continuity: skip the frame pair across each cut
    for k, item in list(A.items()):
        m = np.ones(track.n, bool) if item[2] is None else np.asarray(item[2], bool).copy()
        for tc in SHOTS[1:-1]:
            i = int(round(tc * FPS + 0.5))
            if 0 <= i < track.n:
                m[i] = False
        A[k] = (item[0], item[1], m)
    return A


# ---------------------------------------------------------------------------
# Labels
# ---------------------------------------------------------------------------

def make_labels(G, C):
    GP = G.parts
    A = G.anchors
    root = G.root
    L = Labels()
    end_sh = bend("shafts") - 0.35
    L.add("input_shaft", "Input shaft", A["input_shaft"], wt("shafts", "input"), end_sh, offset=(-0.06, -0.12),
          occlusion=False)
    L.add("countershaft", "Countershaft", A["countershaft"], wt("shafts", "countershaft"), end_sh,
          offset=(0.05, 0.10), occlusion=False)
    out_anchor = (root, (0.0, -1.03, 0.0165))
    L.add("output_shaft", "Output shaft", out_anchor, wt("shafts", "output"), end_sh, offset=(-0.05, -0.12),
          occlusion=False)
    # neutral
    L.add("free_gears", "Free-spinning gears", A["gear_2"], wt("neutral", "spin") - 0.1, wt("neutral", "Until") + 0.6,
          offset=(0.06, -0.10), occlusion=False)
    L.add("output_shaft2", "Output shaft", out_anchor, wt("neutral", "locked"), bend("neutral") - 0.6,
          offset=(-0.05, -0.12), occlusion=False)
    # synchro (exploded 1-2 synchroniser)
    L.add("hub", "Hub", A["hub_12"], wt("synchro", "hub"), wt("synchro", "dog") - 0.4, offset=(0.05, -0.15),
          occlusion=False)
    L.add("sleeve", "Sleeve", A["sleeve_12"], wt("synchro", "sleeve"), wt("synchro", "cone") + 0.4,
          offset=(-0.06, -0.10), occlusion=False)
    L.add("dogs", "Dog teeth", A["dogs_1"], wt("synchro", "dog"), bend("synchro") - 0.9, offset=(-0.05, -0.13),
          occlusion=False)
    L.add("cone", "Cone", A["cone_1"], wt("synchro", "cone"), bend("synchro") - 0.9, offset=(0.07, 0.11),
          occlusion=False)
    L.add("blocker", "Blocker ring", A["blocker_1"], wt("synchro", "brass"), bend("synchro") - 0.9,
          offset=(-0.07, 0.11), occlusion=False)
    # lock
    L.add("sleeve_l", "Sleeve", A["sleeve_12"], wt("lock", "sleeve"), bend("lock") - 0.6, offset=(-0.07, -0.10),
          occlusion=False)
    L.add("dogs_l", "Dog teeth", A["dogs_1"], wt("lock", "dog"), bend("lock") - 0.6, offset=(0.08, 0.10),
          occlusion=False)
    L.add("gear1_l", "1st gear", A["gear_1"], wt("lock", "gear") - 0.2, bend("lock") - 0.6, offset=(0.08, -0.04),
          occlusion=False, style="dim")
    # linkage
    L.add("fork", "Fork", A["fork_12"], wt("linkage", "Forks"), T_RAT - 1.0, offset=(-0.08, 0.05), occlusion=False)
    rail_a = (GP["rail_5R"], (GBX.RAIL_X["5R"] * 1e-3, -0.575, (GBX.RAIL_Z + GBX.RAIL_R) * 1e-3))
    L.add("rail", "Shift rail", rail_a, wt("linkage", "rails"), T_RAT - 1.0, offset=(-0.06, -0.08),
          occlusion=False)
    L.add("lever", "Lever", A["lever"], wt("linkage", "lever") + 0.4, T_RAT - 0.4, offset=(-0.07, 0.0),
          occlusion=False)
    # ratios: 1st (anchors on the camera-facing (-X) side of the countershaft gears)
    e1 = T_4TH - 0.35
    L.add("ig", "Input gear 26T", A["input_gear"], wt("ratios", "input"), e1, offset=(0.0, -0.12), occlusion=False)
    L.add("csd", "35T", (GP["ex_cs_drive"], (-0.044, 0.0, 0.0)), wt("ratios", "countershaft"), e1,
          offset=(-0.07, 0.04), occlusion=False)
    L.add("cs1", "17T", (GP["ex_cs_1"], (-0.022, 0.0, 0.0)), wt("ratios", "small"), e1, offset=(0.06, 0.05),
          occlusion=False)
    L.add("g1", "Output gear 44T", A["gear_1"], wt("ratios", "large"), e1, offset=(0.04, -0.08), occlusion=False)
    # 4th
    e4 = T_5TH - 0.3
    L.add("dogs4", "Input gear dog teeth", A["dogs_4"], T_4TH + 0.5, e4, offset=(-0.07, -0.06), occlusion=False)
    L.add("sl34", "Sleeve", A["sleeve_34"], T_4TH + 0.9, e4, offset=(0.06, -0.10), occlusion=False)
    # 5th
    e5 = T_REV - 0.3
    L.add("cs5", "38T", (GP["ex_cs_5"], (-0.047, 0.0, 0.0)), T_5TH + 0.4, e5, offset=(0.07, 0.03), occlusion=False)
    L.add("g5", "23T", A["gear_5"], T_5TH + 0.4, e5, offset=(0.05, -0.10), occlusion=False)
    # reverse
    L.add("idler", "Idler gear", (GP["ex_idler"], (0.009, 0.0, 0.0)), wt("reverse", "idler"), DUR - 0.5,
          offset=(-0.09, 0.05), occlusion=False)
    L.add("out_r", "Output shaft", (root, (0.0, -1.05, 0.0165)), wt("reverse", "output"), DUR - 0.5,
          offset=(0.04, -0.10), occlusion=False)
    return L


# ---------------------------------------------------------------------------
# HUD
# ---------------------------------------------------------------------------

def make_hud(track):
    t = track.t
    H = Hud(track)

    def a_in(t0, d=0.4):
        return lambda i: float(np.clip((t[i] - t0) / d, 0.0, 1.0))

    H.add("fade", 0.0, 0.8, fade=0.0, color="black", alpha=lambda i: float(max(0.0, 1.0 - t[i] / 0.6)))
    H.add("fade", DUR - 0.6, DUR + 1.0, fade=0.0, color="black",
          alpha=lambda i: float(min(1.0, max(0.0, (t[i] - (DUR - 0.55)) / 0.5))))
    H.add("section_title", 0.4, 4.4, number=4, title="The gearbox")
    H.add("slowmo", 0.6, DUR, factor=lambda i: int(round(1.0 / track.slowmo[i])))
    H.add("status", 0.6, DUR, text=lambda i: f"CLUTCH {track.status[i]}")
    H.add("gear", 0.6, DUR, value=lambda i: str(track.gear[i]))
    H.add("rpm", 0.6, DUR, value=lambda i: fmt_rpm(track.rpm_e[i]), label="Engine")
    H.add("hpattern", bstart("neutral"), DUR, x=lambda i: round(float(track.lever_x[i]), 4),
          y=lambda i: round(float(track.lever_y[i]), 4))
    # clutch pedal while it is pressed (synchro .. linkage)
    H.add("pedal", T_SYN - 0.6, T_RAT, fade=0.0, value=lambda i: round(float(track.pedal[i]), 3),
          alpha=a_in(T_SYN - 0.6))

    def rpm_s(x, signed=False):
        v = fmt_rpm(abs(x))
        if signed and x < -0.5:
            return f"−{v} rpm"
        return f"{v} rpm"

    hi_out0 = (wt("neutral", "locked"), bend("neutral"))
    hi_cs = (wt("ratios", "turns"), wt("ratios", "and"))
    hi_ratio = (wt("ratios", "engine"), T_4TH)

    def rows(i):
        x = t[i]
        hi_o = hi_out0[0] <= x < hi_out0[1] or hi_ratio[0] <= x < hi_ratio[1] or x >= T_REV
        hi_i = hi_ratio[0] <= x < hi_ratio[1]
        hi_c = hi_cs[0] <= x < hi_cs[1]
        return [["Input shaft", rpm_s(track.rpm_in[i]), bool(hi_i)],
                ["Countershaft", rpm_s(track.rpm_cs[i]), bool(hi_c)],
                ["Output shaft", rpm_s(track.rpm_out[i], signed=True), bool(hi_o)]]
    H.add("readouts", 4.6, DUR, rows=rows)
    # ratios / reverse
    cards = [(T_RAT, T_4TH, 1, "1st gear"), (T_4TH, T_5TH, 4, "4th gear · direct"),
             (T_5TH, T_REV, 5, "5th gear · overdrive"), (T_REV, DUR, "R", "Reverse")]
    for t0, t1, g, cap in cards:
        H.add("ratio_card", t0, t1, fade=0.0, ratio_text=f"{abs(S.GEAR_RATIOS[g]):.2f} : 1", caption=cap,
              alpha=a_in(t0 + 0.15, 0.35))
    H.add("speed", T_RAT, DUR, fade=0.0, value_kmh=lambda i: round(float(track.v_kmh[i]), 1),
          alpha=a_in(T_RAT + 0.15, 0.35))
    return H
