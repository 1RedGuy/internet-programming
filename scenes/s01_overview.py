"""s01 Overview (32 s): the whole car, then the power path from the engine to the wheels.

Light studio, the full car (carviz.assemblies.car) parked at the origin.  This is the opening
of the film.

Drivetrain state: car parked, ENGINE OFF (engine_on 0), neutral, clutch pedal up, all speeds
0 for the whole scene - nothing turns (the Track still drives every assembly, so every part
sits exactly at its Track pose).

Picture (beats from carviz.timeline; every glow/label time is a narration word time):
  car     0-7    opaque car, 3/4 front-left, slow orbit at eye height (1.30 m, 50 mm);
                 fade up from black, title card 0.6-5.6 s.
  inside  7-14   the orbit flows into a push-in over the front-left wing (50 -> 30 mm); the
                 paint fades to an x-ray shell (exterior 1 -> 0.15 from 8 to 12 s, black trim
                 and lamps to half that, glass and the cabin (seats, dash, headliner, door
                 cards, pedals) -> 0, floor/tunnel/firewall -> 0 so the drivetrain under them
                 reads; feature lines fade in).  The camera ends high above-left of the engine
                 bay with the whole drivetrain revealed through the faint shell.
  path    14-29  high 3/4 from the left travelling front -> rear along the drivetrain; each
                 stage glows (warm pulse, then a dimmer steady glow) and gets its label at
                 its spoken word: Engine, Clutch, Gearbox, Propeller shaft, Differential,
                 Driveshafts, Rear wheels.
  follow  29-32  the camera rises and dives to a 3/4 front-left view of the engine; body and
                 chassis fade to 0 (only the engine stays, the hand-off to s02), the glow dies
                 away and the picture fades to black over the last 0.4 s.

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
# follow beat: everything but the engine fades away
OUT_FADE = (29.35, 30.9)
GLOW_OUT = (29.3, 31.3)
BLACK = (T_LAST - 0.4, T_LAST)                # fade to black ends exactly on the last frame

# glow (materials: emission 2 x cv_glow, base dimmed 0.7 x cv_glow; steep on dark metal)
GLOW_PEAK = 0.10
GLOW_HOLD = 0.04
GLOW_RISE = 0.30
GLOW_DECAY = 1.6
# per-material scale: emission over near-black rubber reads far stronger than over metal
GLOW_SCALE = {"tire_rubber": 0.40, "rubber": 0.55, "rim_alloy": 0.8}
# after the last word: one ripple of light runs engine -> wheels (stage k peaks at
# SWEEP_T0 + k * SWEEP_DT), the power path read as a whole before the dive
SWEEP_T0 = 28.45
SWEEP_DT = 0.14
SWEEP_AMP = 0.035
SWEEP_SIGMA = 0.17


# labels: (id, text, assembly, anchor, t_in (= word time), t_out, offset (dx, dy) of the plate)
LABELS = [
    ("engine", "Engine", "engine", "cam_cover", T_ENGINE, 20.2, (-0.10, -0.10)),
    ("clutch", "Clutch", "clutch", "bellhousing", T_CLUTCH, 21.3, (0.09, -0.12)),
    ("gearbox", "Gearbox", "gearbox", "case", T_GEARBOX, 21.9, (0.10, -0.06)),
    ("prop", "Propeller shaft", "axle", "propshaft", T_PROP, 25.2, (0.08, 0.10)),
    ("diff", "Differential", "axle", "diff_housing", T_DIFF, 28.9, (0.10, -0.12)),
    ("shafts", "Driveshafts", "wheels", "driveshaft_left", T_SHAFTS, 28.9, (-0.11, -0.09)),
    ("wheels", "Rear wheels", "wheels", "tire_RL", T_WHEELS, 28.9, (-0.12, 0.06)),
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


def arc_eye(tgt, az_deg, dist, elev_deg):
    """Eye at azimuth/elevation/distance from a target (camera.py azimuth convention)."""
    a, e = math.radians(az_deg), math.radians(elev_deg)
    h = dist * math.cos(e)
    return (tgt[0] + h * math.sin(a), tgt[1] - h * math.cos(a), tgt[2] + dist * math.sin(e))


# path beat: (t, target, azimuth, distance, elevation) - an arcing track along the drivetrain
PATH = [
    (15.7, (0.0, -0.22, 0.52), 236.0, 2.75, 44.0),     # engine (15.96)
    (17.4, (0.0, -0.42, 0.48), 244.0, 2.80, 43.0),
    (18.9, (0.0, -0.72, 0.44), 252.0, 2.85, 42.0),     # clutch (18.11), gearbox (18.96)
    (20.5, (0.0, -1.28, 0.40), 262.0, 2.90, 40.0),     # propeller shaft (20.25)
    (22.2, (-0.10, -1.98, 0.37), 274.0, 2.95, 41.0),   # differential (21.96)
    (24.2, (-0.20, -2.42, 0.36), 285.0, 3.00, 42.0),
    (26.0, (-0.28, -2.58, 0.36), 293.0, 3.05, 43.0),   # driveshafts (25.82)
    (27.9, (-0.30, -2.62, 0.36), 300.0, 3.15, 45.0),   # wheels (27.11)
]


def camera_keys():
    """(t, eye, target) key poses.  camera.py azimuth: 180 = front, 270 = left."""
    keys = []
    # car: slow orbit at eye height
    for k in range(8):
        t = k * 1.0
        az = 204.0 + 2.0 * t
        keys.append((t, orbit_eye(az), ORBIT_C))
    # inside: push in over the front-left wing, rising; whole drivetrain revealed at 14 s
    keys += [
        (9.0, (-4.30, 3.50, 1.38), (0.0, -1.05, 0.53)),
        (10.5, (-3.70, 2.70, 1.52), (0.0, -0.88, 0.50)),
        (12.0, (-2.95, 1.95, 1.75), (0.0, -0.82, 0.47)),
        (13.2, (-2.35, 1.45, 1.98), (0.0, -0.88, 0.44)),
        (14.2, (-2.05, 1.15, 2.10), (0.0, -0.92, 0.42)),
    ]
    # path: high 3/4 from the left, travelling front -> rear along the drivetrain
    for t, tg, az, d, el in PATH:
        keys.append((t, arc_eye(tg, az, d, el), tg))
    # follow: rise and swing forward, then dive to a 3/4 front-left view of the engine
    keys += [
        (29.4, (-2.95, -2.20, 2.62), (0.0, -1.35, 0.42)),
        (30.7, (-2.25, -0.25, 1.92), (0.0, -0.38, 0.50)),
        (31.96, (-1.05, 0.72, 1.12), (0.0, -0.06, 0.58)),
    ]
    return keys


LENS_KEYS = [(0.0, 50.0), (9.0, 50.0), (13.6, 30.0), (14.4, 30.0), (15.7, 35.0), (27.6, 35.0),
             (29.4, 30.0), (31.96, 40.0)]
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
    # (chrome bowls, reflectors) are many glossy layers: ghost them fainter than the paint
    ext_trim = ramp(t, *EXT_FADE, 1.0, X_RAY * TRIM_REL) * ramp(t, *OUT_FADE, 1.0, 0.0)
    rig.bake_fade([B.parts["trim"], B.parts["lights_front"], B.parts["lights_rear"]], fr, ext_trim)
    # the left door mirror sits between the travelling camera and the engine/gearbox (a large
    # out-of-focus ghost): mirrors ghost fainter still
    ext_mirror = ramp(t, *EXT_FADE, 1.0, X_RAY * MIRROR_REL) * ramp(t, *OUT_FADE, 1.0, 0.0)
    rig.bake_fade([B.parts["mirrors"]], fr, ext_mirror)
    # the clutch pedal box is in the footwell: it leaves with the body's pedals
    pedal_objs = visible_meshes(objs_of(CL, CL.meta["groups"]["pedal_box"]))
    rig.bake_fade(pedal_objs, fr, cabin)
    # follow: chassis (everything but the engine) fades away for the hand-off to s02
    keep = set(pedal_objs)
    for asm in (CL, G, A, W):
        objs = [o for o in visible_meshes(asm.meshes()) if o not in keep]
        rig.bake_fade(objs, fr, chassis)

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
        L.add(lid, text, C.sub[asm].anchors[an], t_in - 0.05, t_out, fade=0.35, offset=off,
              style="emph", occlusion=False, ignore=body_objs)

    # ---------------- HUD ----------------------------------------------------------
    H = Hud(track)
    H.add("fade", 0.0, 1.2, fade=0.0, color="black", alpha=lambda i: float(1.0 - ss(t[i] / 0.9)))
    H.add("title_card", 0.6, 5.6, fade=0.7, title="How a Manual Car Works",
          subtitle="Inside a front-engine, rear-wheel-drive car", pos=[0.5, 0.125])
    H.add("fade", BLACK[0], DUR + 1.0, fade=0.0, color="black",
          alpha=lambda i: float(ss((t[i] - BLACK[0]) / (BLACK[1] - BLACK[0]))))

    sb = scenebase.SceneBuild(SCENE_ID, track, cam_ob, L, H, motion_blur=False, preview_hide=())
    sb.extra.update(car=C, eye=eye, target=tgt)
    return sb
