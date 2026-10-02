"""s05 A gear shift, step by step (50 s): 1st -> 2nd at 3000 rpm, slowed 150x.

Dark studio, gearbox only (half cutaway: the -X half of the case, web and tail housing
removed; the camera works from the left).  The film's heart: the viewer sees the 1-2
sleeve leave 1st, the brass blocker ring meet 2nd gear's cone, 2nd gear's dog-tooth ring
slow down until it turns in step with the blocker ring (= output shaft), and the sleeve
slide through the blocker ring onto 2nd's dog teeth, while the HUD reads the live Track.

State (FACTS SFT-01..08, PRS-03; slowmo 1/150 constant, so the real shift takes ~0.27 s):
  intro       in 1st, clutch engaged, engine 3000 rpm (24.1 km/h, output 861 rpm)
  clutch_in   pedal down 7.35 -> 9.3 s, throttle closed (target 800) as it goes down
  neutral     1-2 sleeve 1st -> centre (13.55 -> 15.85 s)
  sync        sleeve to the blocking position (cone contact 19.5 s, block 20.2 s);
              the cone brings the input side (2nd gear, countershaft, input shaft, disc)
              from ~1420 to the output shaft's ~858 rpm by 30.85 s (state.py: cosine
              blend from cone contact to the end of the hold)
  engage      the sleeve turns the blocker ring back and passes through (31 -> 36.15 s),
              then slides over 2nd's dog teeth (seated 37.9 s)
  clutch_out  pedal up 40.55 -> 41.75 s, throttle target 1850: the clutch slips the engine
              down from ~2590 rpm to the disc's ~1777 rpm and locks (~47.3 s)
  Road speed coasts down at 0.13 m/s^2 (FACTS SFT-07) while no power flows: 24.12 ->
  24.00 km/h, so the engine lands at ~1777 rpm rather than 1787.

Aliasing (FACTS PRS-04): at 150x the input gear 26T / countershaft drive gear 35T need
rpm_in <= 2908, the input gear's 4th-gear dogs (32) rpm_in <= 2362, the 5th pair (23T/38T)
rpm_in <= 2678 and 5th's dog ring rpm_in <= 1925.  Before the synchroniser has slowed the
input side those parts are kept out of frame; the validator gets per-frame visibility masks
(frustum test of each part's bounding cylinder, no occlusion credit).
"""
from __future__ import annotations

import math

import numpy as np

from carviz import camera as CAM
from carviz import kin, lighting, rig, scenebase, state, timeline
from carviz import spec as S
from carviz.assemblies import gearbox as GB
from carviz.labels import Hud, Labels, fmt_rpm

SCENE_ID = "s05"
SC = timeline.scene(SCENE_ID)
BT = {b.id: b for b in SC.beats}
FPS = S.FPS
DUR = SC.dur
SLOW = 1.0 / 150.0
TAU = 2.0 * math.pi


def bstart(b):
    return BT[b].start


def bend(b):
    return BT[b].end


def wt(b, word, occ=1):
    return BT[b].word_time(word, occ)


# ---------------------------------------------------------------------------
# Drivetrain program (video time, s)
# ---------------------------------------------------------------------------
V1 = S.road_speed_kmh(3000, 1)                       # 24.12 km/h
T_PEDAL_DOWN = (7.35, 9.3)
T_THROTTLE_OFF = (7.45, 7.85)
T_OUT_OF_FIRST = (13.55, 2.3)                        # disengage(1, t0, dur)
T_SLEEVE_GO = 18.45                                  # sleeve leaves neutral toward 2nd
T_BLOCK = 20.2                                       # sleeve chamfers on the blocker ring
T_SYNCED = 30.85                                     # speeds matched (end of the hold)
T_THROUGH = 36.15                                    # sleeve past the blocker (gear 2 counts)
T_SEATED = 37.9
T_PEDAL_UP = (40.55, 41.75)
T_THROTTLE_ON = (40.6, 41.4)
COAST = 0.13 * 3.6                                   # km/h per real second (FACTS SFT-07)


def program():
    P = state.Program(SCENE_ID)
    P.slowmo.key(0.0, SLOW, "step")
    P.start_in_gear(1)
    # throttle: 3000 rpm in 1st, closed as the clutch goes down, re-applied at clutch-out
    P.throttle_rpm.key(0.0, 3000.0, "step")
    P.throttle_rpm.key(T_THROTTLE_OFF[0], 3000.0, "linear")
    P.throttle_rpm.key(T_THROTTLE_OFF[1], 800.0, "ease")
    P.throttle_rpm.key(T_THROTTLE_ON[0], 800.0, "linear")
    P.throttle_rpm.key(T_THROTTLE_ON[1], 1850.0, "ease")
    # clutch pedal
    P.pedal.key(0.0, 0.0, "step")
    P.pedal.key(T_PEDAL_DOWN[0], 0.0, "linear")
    P.pedal.key(T_PEDAL_DOWN[1], 1.0, "ease")
    P.pedal.key(T_PEDAL_UP[0], 1.0, "linear")
    P.pedal.key(T_PEDAL_UP[1], 0.0, "ease")
    # 1 -> N
    P.disengage(1, *T_OUT_OF_FIRST)
    # N -> 2 with a long synchronising hold (see module docstring)
    c = P.sleeve["12"]
    c.key(T_SLEEVE_GO, 0.0, "linear")
    c.key(T_BLOCK, S.SYNC_BLOCK, "ease")
    c.key(T_SYNCED, S.SYNC_BLOCK + 0.0040, "linear")
    c.key(T_SYNCED + 0.1, S.SYNC_BLOCK + 0.0045, "linear")
    c.key(T_THROUGH, S.SYNC_THROUGH, "cubic")
    c.key(T_SEATED, 1.0, "cubic")
    c.key(DUR, 1.0, "linear")
    # road speed: constant under power, coasting while the clutch is out (no drive)
    t_coast0, t_coast1 = 8.4, 47.3
    P.speed_kmh.key(0.0, V1, "step")
    P.speed_kmh.key(t_coast0, V1, "linear")
    P.speed_kmh.key(t_coast1, V1 - COAST * SLOW * (t_coast1 - t_coast0), "linear")
    P.speed_kmh.key(DUR, V1 - COAST * SLOW * (t_coast1 - t_coast0), "linear")
    return P


# ---------------------------------------------------------------------------
# Camera plan: (time, target, azimuth deg, radius, eye height above target, lens, f-stop, mode)
# azimuth as camera.orbit: 0 = behind (-Y), -90 = left (-X), -180 = front (+Y)
# ---------------------------------------------------------------------------
Y_HUB = -0.64124
Y_G2 = -0.60633
Y_G1 = -0.67765
ZC = S.Z_CRANK
Y_BAND2 = Y_HUB + 0.0175        # blocker ring / dog teeth of 2nd
Y_BAND1 = Y_HUB - 0.0175

POSES = [
    # intro: medium three-quarter view of the 1-2 synchro between 2nd and 1st
    (0.0, (0.0, -0.636, ZC - 0.012), -102.0, 0.40, 0.19, 45.0, 8.0, "step"),
    (7.0, (0.0, -0.640, ZC - 0.004), -95.0, 0.33, 0.15, 45.0, 8.0, "cubic"),
    # clutch_in -> neutral: drift in on the sleeve sitting on 1st gear's dog teeth
    (12.4, (0.0, Y_BAND1 + 0.004, ZC + 0.004), -82.0, 0.215, 0.085, 50.0, 11.0, "cubic"),
    (16.6, (0.0, Y_BAND1 + 0.010, ZC + 0.004), -86.0, 0.200, 0.080, 50.0, 11.0, "cubic"),
    # sync: close on 2nd gear's side of the synchro (blocker ring, dog teeth, gear)
    (19.0, (0.0, Y_BAND2 + 0.002, ZC + 0.002), -98.0, 0.180, 0.070, 50.0, 13.0, "cubic"),
    (23.3, (0.0, Y_BAND2 + 0.001, ZC + 0.002), -95.0, 0.175, 0.068, 50.0, 13.0, "cubic"),
    # countershaft / input shaft: pull back and look forward along the train
    (26.6, (0.0, -0.585, ZC - 0.030), -117.0, 0.43, 0.15, 40.0, 8.0, "cubic"),
    (29.4, (0.0, -0.590, ZC - 0.028), -112.0, 0.42, 0.15, 40.0, 8.0, "cubic"),
    # engage: back in on the band
    (32.6, (0.0, Y_BAND2 + 0.001, ZC + 0.002), -96.0, 0.165, 0.065, 50.0, 13.0, "cubic"),
    (38.6, (0.0, Y_BAND2 + 0.000, ZC + 0.002), -93.0, 0.170, 0.066, 50.0, 13.0, "cubic"),
    # clutch out: pull back to the whole gear train, power path in 2nd
    (44.5, (0.0, -0.640, ZC - 0.020), -78.0, 0.52, 0.22, 40.0, 8.0, "cubic"),
    (DUR, (0.0, -0.650, ZC - 0.020), -72.0, 0.58, 0.25, 40.0, 8.0, "cubic"),
]
CAM_SMOOTH = 0.35          # s: Gaussian low-pass of the pose parameters
KEY_OFFSET = -40.0         # key light azimuth relative to the camera azimuth (deg)


def _pose_curves():
    cs = {k: state.Curve() for k in ("tx", "ty", "tz", "az", "r", "h", "lens", "f")}
    for (t, T, az, r, h, lens, f, mode) in POSES:
        for k, v in zip(("tx", "ty", "tz", "az", "r", "h", "lens", "f"), (T[0], T[1], T[2], az, r, h, lens, f)):
            cs[k].key(t, v, mode)
    return cs


def _gauss(x, sigma_frames):
    r = int(3 * sigma_frames)
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma_frames) ** 2)
    k /= k.sum()
    xp = np.r_[np.full(r, x[0]), x, np.full(r, x[-1])]
    return np.convolve(xp, k, mode="valid")


def camera_samples(t):
    cs = _pose_curves()
    sig = CAM_SMOOTH * FPS
    p = {k: _gauss(c(t), sig) for k, c in cs.items()}
    T = np.stack([p["tx"], p["ty"], p["tz"]], 1)
    az = np.radians(p["az"])
    r, h = p["r"], p["h"]
    eye = T + np.stack([r * np.sin(az), -r * np.cos(az), h], 1)
    return eye, T, p["lens"], p["f"], np.degrees(az)


# ---------------------------------------------------------------------------
# Visibility (frustum only, no occlusion credit) for the aliasing masks
# ---------------------------------------------------------------------------

def part_points(ob, n_ang=48):
    """World points on the bounding cylinder (about local Y) of a spinning part, at rest."""
    from mathutils import Vector
    me = ob.data
    co = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    r = float(np.max(np.hypot(co[:, 0], co[:, 2])))
    y0, y1 = float(co[:, 1].min()), float(co[:, 1].max())
    a = np.linspace(0, TAU, n_ang, endpoint=False)
    pts = []
    for y in (y0, 0.5 * (y0 + y1), y1):
        for rr in (r, 0.75 * r):
            pts += [(rr * math.cos(x), y, rr * math.sin(x)) for x in a]
    M = ob.matrix_world
    return np.array([tuple(M @ Vector(p)) for p in pts])


def in_frame(pts, eye, tgt, lens, sensor=36.0, aspect=16 / 9, margin=1.04):
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
        ok = (z > 0.005) & (np.abs(x) < tx * z) & (np.abs(y) < ty * z)
        out[i] = bool(np.any(ok))
    return out


def _dilate(m, k=2):
    out = m.copy()
    for s in range(1, k + 1):
        out[s:] |= m[:-s]
        out[:-s] |= m[s:]
    return out


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _curve(t, keys):
    c = state.Curve()
    for k in keys:
        c.key(*k)
    return c(t)


def _smooth_arr(x, sigma_s):
    return _gauss(np.asarray(x, float), max(1e-3, sigma_s * FPS))


GLOW_PATH = 0.05           # power-path glow (warm emission; materials.CV_Presentation)
GLOW_FRICTION = 0.14       # blocker ring / cone while they slip


def build(quality: str) -> scenebase.SceneBuild:
    sc = scenebase.new_scene(SCENE_ID)
    studio = lighting.setup_studio("dark", center=(0.0, -0.64, ZC - 0.02), size=0.5)
    lighting.setup_color_management(sc)
    G = GB.build({"cutaway": ["half"], "detail": "low" if quality == "preview" else "high"})
    parts = G.parts
    for name, ob in parts.items():
        if name.endswith("__half_removed"):
            ob.hide_render = True
            ob.hide_viewport = True
    for ob in G.objects():
        if ob.type == "MESH":
            ob.display.show_shadows = False

    # ---------------- drivetrain state ----------------------------------
    P = program()
    track = P.run()
    t = track.t
    n = track.n
    fr = track.frames
    G.drive(track, {})

    # ---------------- glow ----------------------------------------------
    gear = np.asarray(track.gear)
    cap = np.asarray(track.capacity)
    g1 = _smooth_arr(np.where(gear == "1", cap, 0.0) * GLOW_PATH, 0.25)
    g2 = _smooth_arr(np.where(gear == "2", cap, 0.0) * GLOW_PATH, 0.25)
    path1, path2 = set(G.meta["power_path"][1]), set(G.meta["power_path"][2])
    for name in sorted(path1 | path2):
        ob = parts[name]
        val = np.maximum(g1 if name in path1 else 0.0, g2 if name in path2 else 0.0)
        rig.bake_prop(ob, "cv_glow", fr, np.round(val, 5))
    slip = np.abs(np.asarray(track.rpm_gear_2) - np.asarray(track.rpm_out))
    sync = np.asarray(track.syncing_12) > 0
    i_c = int(np.argmax(sync))
    slip0 = max(slip[i_c], 1.0)
    fr_glow = np.where(sync, np.clip(slip / slip0, 0.0, 1.0) ** 0.5, 0.0)
    fr_glow = fr_glow * kin.smoothstep(t[i_c], t[i_c] + 0.5, t)
    fr_glow = _smooth_arr(fr_glow, 0.2) * GLOW_FRICTION
    for name in ("blocker_2", "cone_2"):
        rig.bake_prop(parts[name], "cv_glow", fr, np.round(fr_glow, 5))

    # ---------------- camera + lights -----------------------------------
    eye, tgt, lens, fstop, az = camera_samples(t)
    C = CAM.CameraPath(name="cam_s05", lens=50.0, fstop=8.0, clip=(0.01, 40.0))
    for i in range(n):
        C.key(t[i], eye=tuple(eye[i]), target=tuple(tgt[i]), lens=float(lens[i]), fstop=float(fstop[i]),
              mode="linear")
    cam = C.bake(fr, FPS)
    rig.bake_channel(studio["rig"], "rotation_euler", 2, fr,
                     np.radians(np.unwrap(az, period=360.0) + KEY_OFFSET - 225.0))

    # ---------------- validation ----------------------------------------
    vis = {}
    for name in ("input_gear", "dogs_4", "cs_drive", "gear_5", "dogs_5", "cs_5", "input_shaft"):
        vis[name] = _dilate(in_frame(part_points(parts[name]), eye, tgt, lens))
    th_in = track.theta_in
    th_out = track.theta_out
    p32 = TAU / S.DOG_TEETH
    aliasing = {
        "input gear 26T": (track.gb("input_gear"), TAU / S.Z_INPUT, vis["input_gear"]),
        "input gear dogs (4th) 32": (track.gb("input_gear"), p32, vis["dogs_4"]),
        "countershaft drive gear 35T": (track.gb("cs_drive"), TAU / S.Z_CS_DRIVEN, vis["cs_drive"]),
        "input shaft splines 23T": (th_in, TAU / S.CLUTCH_DISC_SPLINE_TEETH, vis["input_shaft"]),
        "5th gear 23T": (track.gb("gear_5"), TAU / S.GEAR_PAIRS[5][1], vis["gear_5"]),
        "5th dog ring 32": (track.gb("gear_5"), p32, vis["dogs_5"]),
        "countershaft 5th 38T": (track.gb("cs_5"), TAU / S.GEAR_PAIRS[5][0], vis["cs_5"]),
        "3rd gear 31T": (track.gb("gear_3"), TAU / S.GEAR_PAIRS[3][1], None),
        "3rd dog ring 32": (track.gb("gear_3"), p32, None),
        "countershaft 3rd 30T": (track.gb("cs_3"), TAU / S.GEAR_PAIRS[3][0], None),
        "2nd gear 37T": (track.gb("gear_2"), TAU / S.GEAR_PAIRS[2][1], None),
        "2nd dog ring 32": (track.gb("gear_2"), p32, None),
        "countershaft 2nd 24T": (track.gb("cs_2"), TAU / S.GEAR_PAIRS[2][0], None),
        "1st gear 44T": (track.gb("gear_1"), TAU / S.GEAR_PAIRS[1][1], None),
        "1st dog ring 32": (track.gb("gear_1"), p32, None),
        "countershaft 1st 17T": (track.gb("cs_1"), TAU / S.GEAR_PAIRS[1][0], None),
        "reverse countershaft 15T": (track.gb("cs_R"), TAU / S.Z_REV_CS, None),
        "reverse idler 22T": (track.gb("idler"), TAU / S.Z_REV_IDLER, None),
        "reverse gear 38T": (track.gb("gear_R"), TAU / S.Z_REV_OUT, None),
        "sleeves / hubs / blocker rings 32": (th_out, p32, None),
    }
    track.validate(aliasing=aliasing)
    clr = GB.synchro_clearance(track)
    assert clr["blocker_mm"] >= 0.0 and clr["dogs_mm"] >= 0.0 and clr["engaged_misalignment_deg"] < 0.01, clr

    # ---------------- labels --------------------------------------------
    A = G.anchors
    L = Labels()
    L.add("g1_intro", "1st gear", A["gear_1"], wt("intro", "first"), bend("intro") - 0.3, offset=(0.07, -0.10))
    L.add("g2_intro", "2nd gear", A["gear_2"], wt("intro", "second"), bend("intro") - 0.3, offset=(-0.07, -0.10))
    L.add("sleeve_n", "Sleeve", A["sleeve_12"], wt("neutral", "sleeve"), bend("neutral") - 0.4,
          offset=(0.06, -0.12))
    L.add("blocker_s", "Blocker ring", A["blocker_2"], wt("sync", "blocker"), 23.2, offset=(0.06, -0.12))
    L.add("g2_s", "2nd gear", A["gear_2"], wt("sync", "second"), 23.2, offset=(-0.07, -0.10))
    L.add("cs", "Countershaft", A["countershaft"], wt("sync", "countershaft"), 28.6, offset=(0.06, 0.08))
    L.add("input", "Input shaft", A["input_gear"], wt("sync", "input"), 28.6, offset=(-0.06, -0.10))
    L.add("output", "Output shaft", A["output_shaft"], wt("sync", "output"), 31.6, offset=(0.06, -0.10))
    L.add("blocker_e", "Blocker ring", A["blocker_2"], wt("engage", "blocker"), 37.3, offset=(0.06, -0.12))
    L.add("sleeve_e", "Sleeve", A["sleeve_12"], wt("engage", "sleeve"), 39.6, offset=(0.08, -0.04))
    L.add("dogs_e", "Dog teeth", A["dogs_2"], wt("engage", "dog teeth"), 40.3, offset=(-0.07, 0.10))
    L.add("g2_out", "2nd gear", A["gear_2"], wt("clutch_out", "taller"), DUR - 0.5, offset=(-0.06, -0.10),
          style="emph")

    # ---------------- HUD -----------------------------------------------
    H = Hud(track)
    H.add("fade", 0.0, 0.8, fade=0.0, color="black", alpha=lambda i: float(max(0.0, 1.0 - t[i] / 0.6)))
    H.add("fade", DUR - 0.6, DUR + 1.0, fade=0.0, color="black",
          alpha=lambda i: float(min(1.0, max(0.0, (t[i] - (DUR - 0.55)) / 0.5))))
    H.add("section_title", 0.4, 4.4, number=5, title="A gear shift")
    rpm_e, rpm_g2, rpm_out = track.rpm_e, track.rpm_gear_2, track.rpm_out
    syncing = np.asarray(track.syncing_12) > 0
    matched = np.abs(np.asarray(rpm_g2) - np.asarray(rpm_out)) < 1.0

    def rows(i):
        hi2 = bool(syncing[i])
        hio = bool(syncing[i] and matched[i])
        return [["Engine", f"{fmt_rpm(rpm_e[i])} rpm"],
                ["2nd gear", f"{fmt_rpm(rpm_g2[i])} rpm", hi2],
                ["Output shaft", f"{fmt_rpm(rpm_out[i])} rpm", hio]]
    H.add("readouts", 0.8, DUR, rows=rows)
    steps = [("clutch_in", 1, "Clutch in"), ("neutral", 2, "Out of first"), ("sync", 3, "Synchronise"),
             ("engage", 4, "Engage second"), ("clutch_out", 5, "Clutch out")]
    for b, k, text in steps:
        H.add("step_card", bstart(b) + 0.1, bend(b) - 0.05 if b != "clutch_out" else DUR, fade=0.3,
              number=k, text=text, total=5)
    H.add("slowmo", 0.8, DUR, factor=lambda i: int(round(1.0 / track.slowmo[i])))
    status = np.asarray(track.status)
    H.add("status", 0.8, DUR, text=lambda i: "SYNCHRONIZING" if syncing[i] else str(status[i]))
    H.add("speed", 0.8, DUR, value_kmh=lambda i: round(float(track.v_kmh[i]), 2))
    H.add("gear", 0.8, DUR, value=lambda i: str(gear[i]))
    H.add("pedal", 0.8, DUR, value=lambda i: round(float(track.pedal[i]), 4))
    H.add("hpattern", 0.8, DUR, x=lambda i: round(float(track.lever_x[i]), 4),
          y=lambda i: round(float(track.lever_y[i]), 4))

    sb = scenebase.SceneBuild(SCENE_ID, track, cam, L, H, motion_blur=False, preview_hide=())
    sb.extra.update(gearbox=G, studio=studio, visibility=vis, clearance=clr)
    return sb
