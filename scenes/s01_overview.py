"""s01 Overview (32 s): the whole car, then the power path from the engine to the wheels.

Light studio, the full car (carviz.assemblies.car) parked at the origin.  This is the opening
of the film.

Drivetrain state: car parked, ENGINE OFF (engine_on 0), neutral, clutch pedal up, all speeds
0 for the whole scene - nothing turns (the Track still drives every assembly, so every part
sits exactly at its Track pose).

Picture (beats from carviz.timeline; every glow/label time is a narration word time):
  car     0-7    opaque car, 3/4 front-left, slow orbit at eye height (1.30 m, 50 mm);
                 fade up from black, title card 0.6-5.6 s.
  inside  7-14   the orbit flows into a push-in that comes round to the car's left side,
                 toward the front-left door window (50 -> 26 mm), while the paint fades to an x-ray shell (exterior 1 -> 0.15
                 from 8 to 12 s, black trim and lamps to half that until the camera is inside,
                 then 0 over 13.4-14.0 s, mirrors to a quarter;
                 glass, the cabin (seats, dash, wheel, headliner, door cards, pedal boxes)
                 and the floor/tunnel/firewall -> 0; feature lines fade in).  At 13.3 s the
                 camera passes INTO the car through the front-left side-window opening (the
                 glass is gone by 10.8 s, so crossing that plane shows no veil pop) and ends
                 near the driver's head, looking forward/down at the engine and clutch
                 through the faded dash and firewall.
  path    14-29  the camera travels rearward INSIDE the cabin (z 1.13 -> 1.25 m, x -0.5 ->
                 -0.1): forward/down at the engine, down the tunnel at the gearbox and
                 propeller shaft, then over the rear seat down at the differential,
                 driveshafts and both rear wheels (28 -> 21 mm).  Each stage glows
                 (warm pulse, then a dimmer steady glow) and gets its label at its spoken
                 word: Engine, Clutch, Gearbox, Propeller shaft, Differential, Driveshafts,
                 Rear wheels.
  follow  29-32  body and chassis fade to 0 over 29.0-30.1 s, the wheels, suspension and
                 driveshafts a little earlier (28.9-29.6 s) (only the engine + flywheel
                 stay, the hand-off to s02); the camera turns forward, the look-at leading
                 along the drivetrain so the engine is framed by ~30.0 s, and, once the shell is
                 gone (hidden from 29.9 s), flies forward and out through the invisible
                 dash/windscreen to a close, high 3/4 rear-left view of the engine (flywheel
                 side); the glow dies away and the picture fades to black over the last 0.4 s.
  The camera never crosses a visible surface (the window opening is the only way in), and
  keeps >= 0.14 m from every visible surface (window frame at the crossing; >= 0.15 m inside
  the cabin) - checked numerically (BVH nearest-surface + segment ray casts) per frame.

Power-path glow: each stage uses the assembly's meta['power_path'] / ['power_groups'] parts
(Car.power_path()); because the engine, clutch, gearbox and differential internals are inside
opaque housings in this overview, the housings of those stages glow with them (block/head/
covers, bellhousing, gearbox case/tail, axle housing/cover) - otherwise the glow would be
invisible.
"""
from __future__ import annotations

import math

import bpy
import numpy as np
from scipy.interpolate import CubicSpline

from carviz import camera as CAM
from carviz import lighting, rig, scenebase, state, timeline
from carviz import spec as S
from carviz.assemblies import car as CAR
from carviz.labels import Hud, Labels

SCENE_ID = "s01"
SC = timeline.scene(SCENE_ID)
BT = {b.id: b for b in SC.beats}
FPS = S.FPS
DUR = SC.dur
N_FRAMES = SC.frames
T_LAST = (N_FRAMES - 1) / FPS          # time of the last frame (31.958 s)


def bstart(b):
    return BT[b].start


def bend(b):
    return BT[b].end


def wt(b, word, occ=1):
    return BT[b].word_time(word, occ)


def ss(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def ramp(t, t0, t1, v0=0.0, v1=1.0):
    """Smoothstep from v0 (t <= t0) to v1 (t >= t1)."""
    return v0 + (v1 - v0) * ss((np.asarray(t, float) - t0) / max(t1 - t0, 1e-9))


# ---------------------------------------------------------------------------
# Timing (seconds, scene time)
# ---------------------------------------------------------------------------
# narration word times (the picture follows the words)
T_ENGINE = wt("path", "engine")              # 15.96
T_CLUTCH = wt("path", "clutch")              # 18.11
T_GEARBOX = wt("path", "gearbox")            # 18.96
T_PROP = wt("path", "propeller shaft")       # 20.25
T_DIFF = wt("path", "differential")          # 21.96
T_SHAFTS = wt("path", "driveshafts")         # 25.82
T_WHEELS = wt("path", "wheels")              # 27.11

# body fades (inside beat)
X_RAY = 0.15                                  # ghosted shell opacity (closed shell: ~0.25 coverage)
EXT_FADE = (8.0, 12.0)                        # exterior 1 -> X_RAY
GLASS_FADE = (8.0, 10.8)                      # glass 1 -> 0
CABIN_FADE = (8.4, 11.2)                      # seats/dash/headliner/door cards 1 -> 0
FLOOR_FADE = (8.6, 11.6)                      # floor pan / tunnel / firewall 1 -> 0
EDGES_FADE = (9.2, 12.4)                      # x-ray feature lines 0 -> EDGES
EDGES = 0.55
TRIM_REL = 0.5                                # trim + lamps ghost at X_RAY * TRIM_REL
MIRROR_REL = 0.25                             # door mirrors ghost at X_RAY * MIRROR_REL
TRIM_OUT = (13.4, 14.0)                       # trim + lamps -> 0 once the camera is inside
# follow beat: everything but the engine fades away
OUT_FADE = (29.0, 30.1)
WHEELS_OUT = (28.9, 29.6)                     # wheels/suspension/driveshafts go first (cut ~29.48 s)
GLOW_OUT = (29.2, 31.2)
BLACK = (T_LAST - 0.4, T_LAST)                # fade to black ends exactly on the last frame

# glow (materials: emission 2 x cv_glow, base dimmed 0.7 x cv_glow; steep on dark metal)
GLOW_PEAK = 0.10
GLOW_HOLD = 0.04
GLOW_RISE = 0.30
GLOW_DECAY = 1.6
# per-material scale: emission over near-black rubber reads far stronger than over metal
GLOW_SCALE = {"tire_rubber": 0.30, "rubber": 0.55, "rim_alloy": 0.8}
# after the last word: one ripple of light runs engine -> wheels (stage k peaks at
# SWEEP_T0 + k * SWEEP_DT), the power path read as a whole before the dive
SWEEP_T0 = 28.45
SWEEP_DT = 0.14
SWEEP_AMP = 0.0                               # off: from inside the cabin the ripple is not visible end to end
SWEEP_SIGMA = 0.17


# labels: (id, text, assembly, anchor name | world point, t_in (= word time), t_out, plate offset (dx, dy))
LABELS = [
    ("engine", "Engine", "engine", "cam_cover", T_ENGINE, 18.8, (-0.10, -0.10)),
    ("clutch", "Clutch", "clutch", "bellhousing", T_CLUTCH, 20.3, (0.09, -0.12)),
    ("gearbox", "Gearbox", "gearbox", "case", T_GEARBOX, 20.7, (0.10, -0.06)),
    ("prop", "Propeller shaft", "axle", "propshaft", T_PROP, 22.0, (0.08, 0.10)),
    ("diff", "Differential", "axle", "diff_housing", T_DIFF, 28.4, (-0.10, -0.12)),
    ("shafts", "Driveshafts", "wheels", "driveshaft_left", T_SHAFTS, 28.2, (0.0, 0.13)),
    ("wheels", "Rear wheels", "wheels", (-0.65, -2.62, 0.50), T_WHEELS, 28.2, (0.0, -0.15)),   # inner sidewall (seen from inside)
]


# ---------------------------------------------------------------------------
# Drivetrain state
# ---------------------------------------------------------------------------
def build_program():
    P = state.Program(SCENE_ID)
    P.engine_on.key(0.0, 0.0, "step")         # engine OFF for the whole scene
    P.throttle_rpm.key(0.0, 0.0, "step")
    P.speed_kmh.key(0.0, 0.0, "step")         # parked
    P.pedal.key(0.0, 0.0, "step")             # clutch pedal up (engaged), neutral
    return P


# ---------------------------------------------------------------------------
# Camera: a smooth C2 path through key poses (eye, target) + eased lens / f-stop
# ---------------------------------------------------------------------------
ORBIT_C = (0.0, -1.20, 0.63)                   # orbit centre / look-at during `car`
ORBIT_R = 7.4
ORBIT_Z = 1.30                                 # eye height


def orbit_eye(az_deg, r=ORBIT_R, z=ORBIT_Z, c=ORBIT_C):
    a = math.radians(az_deg)
    return (c[0] + r * math.sin(a), c[1] - r * math.cos(a), z)


# inside: the orbit flows into a push-in that comes round to the car's left side and passes
# through the front-left door-window opening at WIN (the glass is hidden by then); the camera
# keeps its eye on the engine/clutch, slowing to ~0.5 m/s at the window.
WIN = (-0.73, -1.60, 1.14)
APPROACH = [
    (8.1, (-4.75, 3.85, 1.29), (0.0, -1.10, 0.58)),
    (9.0, (-4.55, 2.60, 1.27), (0.0, -0.95, 0.55)),
    (10.0, (-3.85, 1.15, 1.23), (0.0, -0.75, 0.53)),
    (11.0, (-2.85, -0.05, 1.20), (0.0, -0.35, 0.53)),
    (12.0, (-1.80, -0.92, 1.17), (0.0, -0.05, 0.54)),
    (12.7, (-1.15, -1.33, 1.15), (0.0, 0.08, 0.55)),
    (13.3, WIN, (0.0, 0.05, 0.55)),                        # through the window opening
    (13.9, (-0.55, -1.62, 1.135), (0.0, -0.08, 0.54)),
]
# inside the cabin, left of the tunnel (x ~ -0.5: >= 0.19 m from the side glass, >= 0.17 m
# below the roof): (t, eye, look-at); the look-at glides along the drivetrain
INSIDE = [
    (14.6, (-0.48, -1.56, 1.13), (0.0, -0.22, 0.53)),
    (15.9, (-0.47, -1.52, 1.14), (0.0, -0.14, 0.56)),      # engine (15.96)
    (17.6, (-0.48, -1.74, 1.15), (0.0, -0.42, 0.48)),      # clutch (18.11)
    (19.0, (-0.49, -1.84, 1.16), (0.02, -0.80, 0.43)),     # gearbox (18.96)
    (20.4, (-0.50, -1.96, 1.17), (0.04, -1.45, 0.37)),     # propeller shaft (20.25)
    (21.9, (-0.45, -2.02, 1.20), (0.05, -2.30, 0.34)),     # differential (21.96)
    (23.5, (-0.30, -2.08, 1.22), (0.0, -2.58, 0.33)),      # over the rear seat, centring
    (25.6, (-0.15, -2.10, 1.24), (-0.04, -2.62, 0.33)),    # driveshafts (25.82)
    (27.5, (-0.10, -2.11, 1.25), (-0.05, -2.62, 0.33)),    # wheels (27.11): both in view
]
# follow: the look-at runs back along the drivetrain to the engine while the body fades;
# once the shell is hidden (29.9 s) the camera leaves through the dash/windscreen.  The
# look-at leads the eye forward so the engine is in frame by ~30.0 s (not empty floor);
# the 29.5 / 30.6 s targets are paired so the natural spline before 28.4 s is unchanged.
FOLLOW = [
    (28.4, (-0.40, -2.08, 1.21), (0.02, -2.55, 0.34)),    # slide left, still on the axle
    (29.5, (-0.47, -1.95, 1.17), (0.03, -1.70, 0.38)),    # look-at passes beside, not below
    (30.6, (-0.52, -1.45, 1.15), (0.0, -0.35, 0.51)),
    (31.3, (-0.66, -1.00, 1.16), (0.0, -0.15, 0.55)),
    (31.96, (-0.83, -0.65, 1.20), (0.0, -0.07, 0.56)),    # 3/4 rear-left, high: hand-off
]


def camera_keys():
    """(t, eye, target) key poses.  camera.py azimuth: 180 = front, 270 = left."""
    keys = []
    # car: slow orbit at eye height
    for k in range(8):
        t = k * 1.0
        az = 204.0 + 2.0 * t
        keys.append((t, orbit_eye(az), ORBIT_C))
    keys += APPROACH + INSIDE + FOLLOW
    return keys


LENS_KEYS = [(0.0, 50.0), (8.5, 50.0), (12.8, 26.0), (14.6, 26.0), (16.2, 28.0), (17.6, 26.0), (21.9, 22.0), (24.5, 21.0),
             (28.4, 21.0), (30.6, 24.0), (31.96, 32.0)]
FSTOP_KEYS = [(0.0, 5.6), (8.0, 5.6), (13.0, 6.3), (31.96, 6.3)]


def build_camera(track):
    keys = camera_keys()
    T = np.array([k[0] for k in keys])
    E = np.array([k[1] for k in keys], float)
    G = np.array([k[2] for k in keys], float)
    t = track.t
    # natural spline ends; the scene opens already in motion (no ease-in from rest: the
    # picture fades up from black)
    se = CubicSpline(T, E, axis=0, bc_type="natural")
    sg = CubicSpline(T, G, axis=0, bc_type="natural")
    eye, tgt = se(t), sg(t)
    lens = state.Curve(50.0)
    for tt, v in LENS_KEYS:
        lens.key(tt, v, "ease")
    fst = state.Curve(5.6)
    for tt, v in FSTOP_KEYS:
        fst.key(tt, v, "ease")
    cam = CAM.CameraPath(name="s01_cam", lens=50.0, fstop=5.6)
    L, F = lens(t), fst(t)
    for i in range(len(t)):
        cam.key(float(t[i]), eye=tuple(eye[i]), target=tuple(tgt[i]), lens=float(L[i]),
                fstop=float(F[i]), mode="linear")
    return cam, eye, tgt


# ---------------------------------------------------------------------------
# What the camera can see.  Engine off, every housing closed: the internals (crank train,
# valvetrain, timing drive, clutch pack, gear trains, final drive, CV-joint balls / cages /
# tripods) are never visible in this scene, so they are hidden (render cost and Workbench
# preview time).  Everything outside the housings stays, plus the flywheel, which the engine
# keeps when the bellhousing fades away for the hand-off to s02.
# ---------------------------------------------------------------------------
VISIBLE = {
    "engine": {"block", "head", "head_gasket", "cam_cover", "timing_cover", "oil_pan", "coils",
               "intake_manifold", "fuel_rail", "exhaust_manifold", "oil_filter", "water_pump",
               "wp_pulley", "alternator", "alt_pulley", "idler_arm", "belt_idler", "accessory_belt",
               "damper", "flywheel"},          # flywheel (+ ring gear): seen once the bellhousing fades
    "clutch": {"bellhousing", "bellhousing_bolts", "fork", "slave_cylinder", "slave_pushrod",
               "hose_bracket", "line_fittings", "master_cylinder", "pedal", "pedal_box", "pushrod",
               "return_spring"} | {f"line_{i:02d}" for i in range(12)},
    "gearbox": {"case", "case_web", "tail_housing", "output_flange", "lever", "knob", "boot"},
    "axle": {"prop_flange_front", "prop_cross_front", "prop_slip_yoke", "prop_boot", "prop_tube",
             "prop_weights", "prop_tube_yoke", "prop_cross_rear", "prop_flange_rear", "housing",
             "cover", "housing_bolts", "plugs", "bushings", "bushing_sleeves", "seal_left",
             "seal_right", "pinion_seal", "companion_flange", "stub_left", "stub_right"},
}
HIDDEN_WHEELS = ("ball_", "cage_", "inner_race_", "spider_", "rollers_")


def force_hide(ob):
    """Hidden for the whole scene, even if an assembly keyed its visibility."""
    ob.hide_render = ob.hide_viewport = True
    ad = ob.animation_data
    if ad is not None and ad.action is not None:
        for path in ("hide_render", "hide_viewport"):
            rig.bake_channel(ob, path, -1, [1.0], [1.0], "CONSTANT")


def hide_internals(C):
    n = 0
    for key, keep in VISIBLE.items():
        a = C.sub[key]
        keep_obj = set()
        for k in keep:
            ob = a.parts.get(k)
            if ob is not None:
                keep_obj.add(ob.name)
                keep_obj.update(c.name for c in ob.children_recursive)
        for ob in a.meshes():
            if ob.name not in keep_obj:
                force_hide(ob)
                n += 1
    for k, ob in C.sub["wheels"].parts.items():
        if ob.type == "MESH" and k.startswith(HIDDEN_WHEELS):
            force_hide(ob)
            n += 1
    return n


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def objs_of(asm, names):
    """Mesh objects (with mesh children) of an assembly's part names / objects."""
    out = []
    for n in names:
        ob = asm.parts.get(n) if isinstance(n, str) else n
        if ob is None:
            continue
        for o in [ob] + list(ob.children_recursive):
            if o.type == "MESH":
                out.append(o)
    return list(dict.fromkeys(out))


def visible_meshes(objs):
    return [o for o in objs if o.type == "MESH" and not o.hide_render]


def glow_curve(t, t_word, peak=GLOW_PEAK, hold=GLOW_HOLD, lead=0.06):
    """Warm pulse landing on the word, settling to a steady dimmer glow."""
    t = np.asarray(t, float)
    t0 = t_word - lead
    up = ss((t - t0) / GLOW_RISE)
    down = ss((t - (t0 + GLOW_RISE)) / GLOW_DECAY)
    return up * (peak + (hold - peak) * down)


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
def build(quality: str) -> scenebase.SceneBuild:
    preview = quality == "preview"
    sc = scenebase.new_scene(SCENE_ID)

    # ---------------- state ----------------------------------------------------
    P = build_program()
    track = P.run()
    t, fr = track.t, track.frames

    # ---------------- assemblies ------------------------------------------------
    hi = "high"
    lo = "low"
    opts = {
        "engine": {"cutaways": ["none"], "detail": lo, "gas": False},
        "clutch": {"cutaway": ["none"], "detail": lo},
        "gearbox": {"cutaway": "none", "detail": lo},
        "axle": {"cutaways": ["none"], "detail": lo},
        "wheels": {"detail": lo if preview else hi},
        "body": {"detail": hi if not preview else lo, "xray_edges": True},
    }
    C = CAR.build(opts)
    E, CL, G, A, W, B = (C.sub[k] for k in ("engine", "clutch", "gearbox", "axle", "wheels", "body"))

    # aliasing: nothing turns (engine off, parked); the fastest visible features are listed
    # anyway so the validator proves it (all steps are 0).
    tread = 2 * math.pi / 64
    aliasing = {
        "tyre tread RL (64 pitches)": (track.theta_RL, tread, None),
        "tyre tread RR (64 pitches)": (track.theta_RR, tread, None),
        "tyre tread FL (64 pitches)": (track.theta_FL, tread, None),
        "propshaft U-joint (4 arms)": (track.theta_out, 2 * math.pi / 4, None),
        "crank pulley / damper": (track.theta_e, 2 * math.pi / 6, None),
    }
    track.validate(aliasing=aliasing)

    # ---------------- studio ------------------------------------------------------
    lighting.setup_studio("light")
    lighting.setup_color_management(sc)

    # ---------------- presentation arrays ---------------------------------------
    ext = ramp(t, *EXT_FADE, 1.0, X_RAY) * ramp(t, *OUT_FADE, 1.0, 0.0)
    glass = ramp(t, *GLASS_FADE, 1.0, 0.0)
    cabin = ramp(t, *CABIN_FADE, 1.0, 0.0)
    floor = ramp(t, *FLOOR_FADE, 1.0, 0.0)
    edges = ramp(t, *EDGES_FADE, 0.0, EDGES) * ramp(t, *OUT_FADE, 1.0, 0.0)
    chassis = ramp(t, *OUT_FADE, 1.0, 0.0)

    C.drive(track, {
        "engine": {"variant": "none"},
        "clutch": {"variant": "none"},
        "axle": {"variant": "none"},
        "body": {"exterior_opacity": ext, "glass_opacity": glass, "interior_opacity": cabin,
                 "underbody_opacity": floor, "xray_edges_opacity": edges},
    })
    n_hidden = hide_internals(C)
    print(f"[s01] hidden internals: {n_hidden} meshes")
    sc.frame_set(1)

    # cabin trim (door cards, headliner, parcel shelf) belongs to the exterior group but
    # would double the shell's veil from above: it leaves with the cabin
    rig.bake_fade([B.parts["cabin_trim"]], fr, cabin)
    # black plastic trim (grille / intake ducts, window surrounds) and the lamp internals
    # (chrome bowls, reflectors) are many glossy layers: ghost them fainter than the paint,
    # and drop them once the camera is inside (seen from the cabin, the stacked cowl / wiper /
    # headlamp ghosts made a dark mottled band along the top of the frame)
    ext_trim = (ramp(t, *EXT_FADE, 1.0, X_RAY * TRIM_REL) * ramp(t, *OUT_FADE, 1.0, 0.0)
                * ramp(t, *TRIM_OUT, 1.0, 0.0))
    rig.bake_fade([B.parts["trim"], B.parts["lights_front"], B.parts["lights_rear"]], fr, ext_trim)
    # the left door mirror sits between the travelling camera and the engine/gearbox (a large
    # out-of-focus ghost): mirrors ghost fainter still
    ext_mirror = ramp(t, *EXT_FADE, 1.0, X_RAY * MIRROR_REL) * ramp(t, *OUT_FADE, 1.0, 0.0)
    rig.bake_fade([B.parts["mirrors"]], fr, ext_mirror)
    # the clutch pedal box is in the footwell: it leaves with the body's pedals
    pedal_objs = visible_meshes(objs_of(CL, CL.meta["groups"]["pedal_box"]))
    rig.bake_fade(pedal_objs, fr, cabin)
    # follow: chassis (everything but the engine) fades away for the hand-off to s02; the
    # wheels assembly goes first: the front wheels sit next to the engine, which the camera
    # frames before the chassis cut (29.9 s)
    keep = set(pedal_objs)
    wheels_out = ramp(t, *WHEELS_OUT, 1.0, 0.0)
    # (hidden below 8 %: a 2-5 % ghost of the black suspension/tyres renders as dark speckle
    #  at low sample counts while the camera moves)
    for asm in (CL, G, A, W):
        objs = [o for o in visible_meshes(asm.meshes()) if o not in keep]
        rig.bake_fade(objs, fr, wheels_out if asm is W else chassis, threshold=0.08)

    # ---------------- power-path glow ------------------------------------------
    stages = dict(C.power_path())        # label -> objects (assembly meta power paths)
    extra = {
        "Engine": visible_meshes(E.meshes()),
        "Clutch": objs_of(CL, list(CL.meta["groups"]["housing"]) + list(CL.meta["groups"]["release"])),
        "Gearbox": objs_of(G, list(G.meta["groups"]["housing"]) + ["output_flange"]),
        "Differential": objs_of(A, A.meta["groups"]["housing"]),
    }
    words = {"Engine": T_ENGINE, "Clutch": T_CLUTCH, "Gearbox": T_GEARBOX, "Propeller shaft": T_PROP,
             "Differential": T_DIFF, "Driveshafts": T_SHAFTS, "Rear wheels": T_WHEELS}
    glow = {}
    glow_out = ramp(t, *GLOW_OUT, 1.0, 0.0)

    def add(obj, arr):
        g = glow.get(obj.name)
        glow[obj.name] = (obj, arr if g is None else np.maximum(g[1], arr))

    def centre(ob):
        from mathutils import Vector
        pts = [ob.matrix_world @ Vector(c) for c in ob.bound_box]
        return sum(pts, Vector()) / 8.0

    def scale_of(ob):
        mats = [m.material.name for m in ob.material_slots if m.material is not None]
        return min([GLOW_SCALE.get(m, 1.0) for m in mats] or [1.0])

    for k, (label, tw) in enumerate(words.items()):
        objs = [o for o in dict.fromkeys(list(stages.get(label, [])) + extra.get(label, []))
                if not o.hide_render]
        sweep = SWEEP_AMP * np.exp(-0.5 * ((t - (SWEEP_T0 + k * SWEEP_DT)) / SWEEP_SIGMA) ** 2)
        for ob in objs:
            dt = 0.0
            c = centre(ob)
            if label == "Propeller shaft":          # sweep front -> rear along the shaft
                dt = 0.55 * float(np.clip((S.Y_GEARBOX_REAR - c.y) / (S.Y_GEARBOX_REAR - S.Y_PINION_FLANGE), 0, 1))
            elif label == "Driveshafts":            # inner joint -> outer joint
                dt = 0.45 * float(np.clip((abs(c.x) - 0.17) / (0.66 - 0.17), 0, 1))
            add(ob, (glow_curve(t, tw + dt) + sweep) * scale_of(ob) * glow_out)
    for name, (ob, arr) in glow.items():
        rig.bake_prop(ob, "cv_glow", fr, arr)

    # ---------------- preview (Workbench ignores opacity): hide ghosted shells ---------------
    if preview:
        for ob in B.meta["groups"]["exterior"]:
            rig.bake_visibility(ob, fr, ext if ob is not B.parts["glass"] else glass, threshold=0.5)
        for ob in bpy.context.scene.objects:
            if ob.type == "MESH":
                ob.display.show_shadows = False

    # ---------------- camera -------------------------------------------------------
    cam, eye, tgt = build_camera(track)
    cam_ob = cam.bake(fr, FPS)

    # ---------------- labels -------------------------------------------------------
    body_objs = [o for o in B.meshes()]
    L = Labels()
    for lid, text, asm, an, t_in, t_out, off in LABELS:
        anchor = an if isinstance(an, tuple) else C.sub[asm].anchors[an]   # tuple = world point
        L.add(lid, text, anchor, t_in - 0.05, t_out, fade=0.35, offset=off,
              style="emph", occlusion=False, ignore=body_objs)

    # ---------------- HUD ----------------------------------------------------------
    H = Hud(track)
    H.add("fade", 0.0, 1.2, fade=0.0, color="black", alpha=lambda i: float(1.0 - ss(t[i] / 0.9)))
    H.add("title_card", 0.6, 5.6, fade=0.7, title="How a Manual Car Works",
          subtitle="Inside a front-engine, rear-wheel-drive car", pos=[0.5, 0.125])
    H.add("fade", BLACK[0], DUR + 1.0, fade=0.0, color="black",
          alpha=lambda i: float(ss((t[i] - BLACK[0]) / (BLACK[1] - BLACK[0]))))

    # camera motion blur: the fly-in (1.6 m/s) and the turns inside the cabin pan up to ~35 deg/s;
    # nothing else moves (engine off), so it costs ~10 % render time
    sb = scenebase.SceneBuild(SCENE_ID, track, cam_ob, L, H, motion_blur=True, shutter=0.5, preview_hide=())
    sb.extra.update(car=C, eye=eye, target=tgt)
    return sb
