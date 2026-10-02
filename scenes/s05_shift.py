"""s05 A gear shift, step by step (50 s): 1st -> 2nd at 3000 rpm, slowed 150x.

Dark studio, gearbox only.  The case is the 'half' cutaway (-X half removed; the camera
works from the left) and the 1-2 synchroniser is shown in a WORLD-FIXED quarter section:
the hub, sleeve, struts, both blocker rings, both gear cones and dog rings and the 1-2
fork are cut by a static box (x < 0, z > axis, between 3rd gear and the web) with
per-frame Boolean modifiers (Manifold solver, ~0.2 s/frame), so the parts turn inside a
cut that always faces the camera.  The upper half then reads like the textbook synchro
profile (sleeve / blocker ring on the cone / dog teeth; cut faces in shades of section
red per part so neighbours do not merge: sleeve red, hub darker, blocker rings orange,
gear cone + dog ring maroon), and
the lower half keeps the whole parts, where the brass blocker teeth (output speed) and
2nd gear's steel dog teeth (gear speed) run side by side: the viewer sees them slip,
slow and lock in step.  (gearbox opts['sections'] cuts in each part's LOCAL frame, so
its notch turns with the parts and faces the camera only ~1/4 of the time.)

State (FACTS SFT-01..08, PRS-03; slowmo 1/150 constant, the real shift takes ~0.27 s):
  intro       in 1st, clutch engaged, engine 3000 rpm (24.1 km/h, output 861 rpm)
  clutch_in   pedal down 7.35 -> 9.3 s, throttle closed (target 800) as it goes down
  neutral     1-2 sleeve 1st -> centre (13.55 -> 15.85 s)
  sync        sleeve to the blocking position (cone contact ~19.5 s, block 20.2 s);
              the cone brings the input side (2nd gear, countershaft, input shaft, disc)
              from ~1420 to the output shaft's ~858 rpm by T_SYNCED (state.py: cosine
              blend from cone contact to the end of the hold)
  engage      the sleeve turns the blocker ring back and passes through (-> T_THROUGH),
              then slides over 2nd's dog teeth (seated T_SEATED)
  clutch_out  pedal up 40.55 -> 41.75 s, throttle target 1850: the clutch slips the engine
              down from ~2590 rpm to the disc's ~1778 rpm and locks (~47.4 s)
  Road speed coasts down at 0.13 m/s^2 (FACTS SFT-07) while no power flows: 24.12 ->
  24.00 km/h, so the engine lands at ~1778 rpm rather than 1787.

Aliasing (FACTS PRS-04): at 150x the input gear 26T / countershaft drive gear 35T need
rpm_in <= 2908, the input gear's 4th-gear dogs (32) rpm_in <= 2362, the 5th pair (23T/38T)
rpm_in <= 2678 and 5th's dog ring rpm_in <= 1925.  Until the synchroniser has slowed the
input side those parts are kept out of frame; the validator gets per-frame visibility
masks (frustum test of each part's bounding cylinder, no occlusion credit).
"""
from __future__ import annotations

import math

import bpy  # noqa: I001  (bpy before bmesh)
import bmesh
import numpy as np

from carviz import camera as CAM
from carviz import kin, lighting, materials, rig, scenebase, state, timeline
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
T_THROUGH = 36.20                                    # sleeve past the blocker (gear 2 counts);
#   36.20 keeps every rendered frame clear of a ~5 um interpolation error in
#   gearbox.blocker_offset (see the report) -- synchro_clearance >= 0 is asserted below
T_SEATED = 37.9
T_PEDAL_UP = (40.55, 41.75)
T_THROTTLE_ON = (40.6, 41.4)
T_COAST = (8.4, 47.3)                                # no drive: clutch out .. locked again
COAST = 0.13 * 3.6                                   # km/h lost per real second (SFT-07)


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
    # road speed: constant under power, coasting while no power flows
    v2 = V1 - COAST * SLOW * (T_COAST[1] - T_COAST[0])
    P.speed_kmh.key(0.0, V1, "step")
    P.speed_kmh.key(T_COAST[0], V1, "linear")
    P.speed_kmh.key(T_COAST[1], v2, "linear")
    P.speed_kmh.key(DUR, v2, "linear")
    return P


# ---------------------------------------------------------------------------
# Section (world-fixed quarter cut through the 1-2 synchroniser)
# ---------------------------------------------------------------------------
ZC = S.Z_CRANK
_U = GB.U                               # gearbox axial layout (mm behind the case front)
Y_HUB = GB.yu(_U["hub_12"])                                   # 1-2 hub centre, car Y (-0.6412)
Y_G2 = GB.yu(0.5 * (_U["gear_2"][0] + _U["gear_2"][1]))       # 2nd gear centre (-0.6063)
Y_G1 = GB.yu(0.5 * (_U["gear_1"][0] + _U["gear_1"][1]))       # 1st gear centre (-0.6777)
Y_INPUT_GEAR = GB.yu(0.5 * (_U["input_gear"][0] + _U["input_gear"][1]))
# cut box spans from the 1st-gear/web gap to the 3rd/2nd-gear gap (-0.6937 .. -0.5913)
SECTION_Y = (GB.yu(0.5 * (_U["gear_1"][1] + _U["web"][0])), GB.yu(0.5 * (_U["gear_3"][1] + _U["gear_2"][0])))
SECTION_PARTS = ("hub_12", "sleeve_12", "strut_12_0", "strut_12_1", "strut_12_2", "blocker_1", "blocker_2",
                 "cone_1", "cone_2", "dogs_1", "dogs_2", "fork_12")


# Shade of each part's cut faces (all in the section-red family, but adjacent parts must
# not merge into one red blob: sleeve red, hub/struts darker, blocker rings orange (brass),
# 2nd/1st gear's cone and dog ring deep maroon, fork mid red).  Index into SECTION_SHADES.
SECTION_SHADE_OF = {"sleeve_12": 0, "hub_12": 1, "strut_12_0": 1, "strut_12_1": 1, "strut_12_2": 1,
                    "blocker_1": 2, "blocker_2": 2, "cone_1": 3, "cone_2": 3, "dogs_1": 3, "dogs_2": 3,
                    "fork_12": 4}
SECTION_SHADES = (materials.SECTION_RED, (0.17, 0.016, 0.012), (0.50, 0.13, 0.022), (0.085, 0.008, 0.010),
                  (0.26, 0.05, 0.04))


def section_material():
    """section_cut variant whose colour comes from the cut object's 's05_sec' property."""
    base = materials.get("section_cut")
    m = base.copy()
    m.name = "s05_section_cut"
    nt = m.node_tree
    bsdf = next(n for n in nt.nodes if n.type == "BSDF_PRINCIPLED")
    attr = nt.nodes.new("ShaderNodeAttribute")
    attr.attribute_type = "OBJECT"
    attr.attribute_name = "s05_sec"
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.interpolation = "CONSTANT"
    els = ramp.color_ramp.elements
    n = len(SECTION_SHADES)
    while len(els) < n:
        els.new(0.5)
    for k, col in enumerate(SECTION_SHADES):
        els[k].position = k / n
        els[k].color = (*col, 1.0)
    nt.links.new(attr.outputs["Fac"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], bsdf.inputs["Base Color"])
    return m


def add_section(G):
    """Static cutter box + Boolean DIFFERENCE modifiers (evaluated per frame)."""
    y0, y1 = SECTION_Y
    me = bpy.data.meshes.new("s05_section_cutter")
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bm.to_mesh(me)
    bm.free()
    me.materials.append(section_material())
    cutter = bpy.data.objects.new("s05_section_cutter", me)
    rig.link(cutter, G.root.users_collection[0])
    cutter.scale = (0.30, y1 - y0, 0.30)
    cutter.location = (-0.15, 0.5 * (y0 + y1), ZC + 0.15)
    cutter.hide_render = True
    cutter.display_type = "WIRE"
    n = len(SECTION_SHADES)
    for name in SECTION_PARTS:
        ob = G.parts[name]
        ob["s05_sec"] = (SECTION_SHADE_OF[name] + 0.5) / n
        md = ob.modifiers.new("s05_section", "BOOLEAN")
        md.operation = "DIFFERENCE"
        md.object = cutter
        md.solver = "MANIFOLD"
        md.material_mode = "TRANSFER"          # cut faces get the cutter's section material
    return cutter


# ---------------------------------------------------------------------------
# Camera plan: (time, target, azimuth deg, distance, elevation deg, lens, f-stop, mode)
# azimuth as camera.orbit: 0 = behind (-Y), -90 = left (-X), -180 = front (+Y)
# ---------------------------------------------------------------------------
Y_B1 = Y_HUB - 0.009            # 1st-gear side of the synchro
Y_B2 = Y_HUB + 0.010            # 2nd-gear side
Z_PROF = ZC + 0.013             # profile view: the cut profile in the upper middle of the frame

POSES = [
    # intro: medium shot of 2nd gear, the sectioned 1-2 synchro, 1st gear, countershaft
    # (the input gear and 5th must stay out of frame until the synchro has slowed them)
    (0.0, (0.0, -0.6425, ZC - 0.008), -97.0, 0.276, 21.0, 50.0, 8.0, "step"),
    (6.6, (0.0, -0.644, ZC - 0.004), -95.0, 0.262, 17.0, 50.0, 9.0, "cubic"),
    # clutch_in: push in to the synchro profile (seen from just below the axis: the cut
    # profile above, the whole lower half with its teeth rows below)
    (11.8, (0.0, Y_B1, Z_PROF), -88.0, 0.192, -3.0, 50.0, 20.0, "cubic"),
    # neutral: the sleeve slides off 1st gear's dog teeth
    (16.4, (0.0, Y_B1 + 0.002, Z_PROF), -89.0, 0.188, -3.0, 50.0, 20.0, "cubic"),
    # sync: 2nd gear's side (blocker ring on the cone, dog teeth)
    (19.0, (0.0, Y_B2, Z_PROF), -93.0, 0.174, -3.0, 50.0, 20.0, "cubic"),
    (22.6, (0.0, Y_B2 - 0.001, Z_PROF), -94.0, 0.170, -3.0, 50.0, 20.0, "cubic"),
    # countershaft / input shaft: pull back (countershaft), then swing to the rear-left
    # and look forward along the gear train (the input gear enters frame once allowed)
    (24.4, (0.0, -0.637, ZC - 0.062), -90.0, 0.246, -7.0, 50.0, 18.0, "cubic"),
    (25.55, (0.0, -0.636, ZC - 0.062), -89.0, 0.248, -6.0, 50.0, 18.0, "cubic"),
    (27.4, (0.0, -0.585, ZC - 0.056), -62.0, 0.427, 19.0, 42.0, 10.0, "cubic"),
    (29.3, (0.0, -0.587, ZC - 0.054), -64.0, 0.418, 19.0, 42.0, 10.0, "cubic"),
    # engage: closer on the upper profile: sleeve, blocker ring, 2nd gear's dog teeth
    (32.6, (0.0, Y_HUB + 0.012, ZC + 0.022), -93.0, 0.146, -2.0, 50.0, 22.0, "cubic"),
    (38.8, (0.0, Y_HUB + 0.011, ZC + 0.022), -91.0, 0.152, -2.0, 50.0, 22.0, "cubic"),
    # clutch out: pull back to the whole gear train, power path in 2nd
    (44.6, (0.0, -0.640, ZC - 0.020), -78.0, 0.565, 23.0, 40.0, 11.0, "cubic"),
    (DUR, (0.0, -0.650, ZC - 0.020), -72.0, 0.625, 24.0, 40.0, 11.0, "cubic"),
]
CAM_SMOOTH = 0.35          # s: Gaussian low-pass of the pose parameters
KEY_OFFSET = -40.0         # key light azimuth relative to the camera azimuth (deg)
FOCUS_NEAR = 0.016         # m: profile shots focus slightly in front of the cut plane


def _pose_curves():
    keys = ("tx", "ty", "tz", "az", "d", "el", "lens", "f")
    cs = {k: state.Curve() for k in keys}
    for (t, T, az, d, el, lens, f, mode) in POSES:
        for k, v in zip(keys, (T[0], T[1], T[2], az, d, el, lens, f)):
            cs[k].key(t, v, mode)
    return cs


def _gauss(x, sigma_frames):
    r = int(3 * sigma_frames)
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma_frames) ** 2)
    k /= k.sum()
    xp = np.r_[np.full(r, x[0]), x, np.full(r, x[-1])]
    return np.convolve(xp, k, mode="valid")


def camera_samples(t):
    """Per-frame (eye, target, lens, f-stop, azimuth deg, distance)."""
    cs = _pose_curves()
    sig = CAM_SMOOTH * FPS
    p = {k: _gauss(c(t), sig) for k, c in cs.items()}
    T = np.stack([p["tx"], p["ty"], p["tz"]], 1)
    az = np.radians(p["az"])
    el = np.radians(p["el"])
    d = p["d"]
    r, h = d * np.cos(el), d * np.sin(el)
    eye = T + np.stack([r * np.sin(az), -r * np.cos(az), h], 1)
    return eye, T, p["lens"], p["f"], np.degrees(az), d


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


def _smooth_arr(x, sigma_s):
    return _gauss(np.asarray(x, float), max(1e-3, sigma_s * FPS))


GLOW_PATH = 0.015          # power-path glow (warm emission; materials.CV_Presentation)
GLOW_FRICTION = 0.045      # blocker ring + cone of 2nd while they slip


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
    cutter = add_section(G)

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
    # power path while torque can flow (clutch capacity, in gear)
    g1 = _smooth_arr(np.where(gear == "1", cap, 0.0) * GLOW_PATH, 0.25)
    g2 = _smooth_arr(np.where(gear == "2", cap, 0.0) * GLOW_PATH, 0.25)
    path1, path2 = set(G.meta["power_path"][1]), set(G.meta["power_path"][2])
    for name in sorted(path1 | path2):
        val = np.maximum(g1 if name in path1 else 0.0, g2 if name in path2 else 0.0)
        rig.bake_prop(parts[name], "cv_glow", fr, np.round(val, 5))
    # friction: blocker ring + 2nd gear's cone while they slip (fades with the slip speed)
    slip = np.abs(np.asarray(track.rpm_gear_2) - np.asarray(track.rpm_out))
    sync = np.asarray(track.syncing_12) > 0
    i_c = int(np.argmax(sync))
    slip0 = max(slip[i_c], 1.0)
    fg = np.where(sync, np.clip(slip / slip0, 0.0, 1.0) ** 0.5, 0.0)
    fg = _smooth_arr(fg * kin.smoothstep(t[i_c], t[i_c] + 0.5, t), 0.2) * GLOW_FRICTION
    for name in ("blocker_2", "cone_2"):
        rig.bake_prop(parts[name], "cv_glow", fr, np.round(fg, 5))

    # ---------------- camera + lights -----------------------------------
    eye, tgt, lens, fstop, az, dist = camera_samples(t)
    C = CAM.CameraPath(name="cam_s05", lens=50.0, fstop=8.0, clip=(0.01, 40.0))
    for i in range(n):
        C.key(t[i], eye=tuple(eye[i]), target=tuple(tgt[i]), lens=float(lens[i]), fstop=float(fstop[i]),
              mode="linear")
    # close (profile) shots focus a little in front of the cut plane so the whole teeth
    # rows of the lower half are sharp too
    near = np.clip((0.30 - dist) / 0.08, 0.0, 1.0)
    for i in range(n):
        C.focus_offset.key(t[i], -FOCUS_NEAR * float(near[i]), "linear")
    cam = C.bake(fr, FPS)
    rig.bake_channel(studio["rig"], "rotation_euler", 2, fr,
                     np.radians(np.unwrap(az, period=360.0) + KEY_OFFSET - 225.0))

    # ---------------- validation ----------------------------------------
    vis = {}
    for name in ("input_gear", "dogs_4", "cs_drive", "gear_5", "dogs_5", "cs_5"):
        vis[name] = _dilate(in_frame(part_points(parts[name]), eye, tgt, lens))
    th_in = track.theta_in
    th_out = track.theta_out
    p32 = TAU / S.DOG_TEETH
    aliasing = {
        "input gear 26T": (track.gb("input_gear"), TAU / S.Z_INPUT, vis["input_gear"]),
        "input gear dogs (4th) 32": (track.gb("input_gear"), p32, vis["dogs_4"]),
        "countershaft drive gear 35T": (track.gb("cs_drive"), TAU / S.Z_CS_DRIVEN, vis["cs_drive"]),
        "input shaft splines 23T": (th_in, TAU / S.CLUTCH_DISC_SPLINE_TEETH, None),
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
    # Anchors sit on surfaces the camera sees: the near flanks of the (whole) gears, and
    # the world-fixed section plane x = 0 (0.5 mm in front of it) for the cut synchro
    # parts.  The section faces are visible by construction, so those labels skip the
    # occlusion ray test (the anchor lies on the cut face itself).
    A = G.anchors
    R = G.root
    sl_sleeve, sl_blk2 = A["sleeve_12"][0], A["blocker_2"][0]
    an = {
        "g2": (R, (-0.0459, Y_G2, 0.010)),              # 2nd gear 37T (tip r 48 mm), near flank
        "g1": (R, (-0.0545, Y_G1, 0.014)),              # 1st gear 44T (tip r 57 mm), near flank
        "sleeve": (sl_sleeve, (-0.0005, 0.0, 0.036)),   # sleeve section (moves with it)
        "blocker2": (sl_blk2, (-0.0005, 0.0035, 0.0275)),
        "cone2": (R, (-0.0005, Y_HUB + 0.0130, 0.0240)),
        "dogs2": (R, (-0.0005, Y_HUB + 0.0198, 0.0317)),
        "cs": (R, (-0.0142, Y_HUB, -S.GEARBOX_CENTRE_DISTANCE)),   # bare countershaft, cs_2..cs_1
        "input": (R, (-0.033, Y_INPUT_GEAR, 0.008)),   # input gear (on the input shaft), near flank
        "output": (R, (-0.0125, Y_HUB - 0.002, 0.013)),  # output-shaft splines inside the cut hub
    }
    L = Labels()
    L.add("g2_intro", "2nd gear", an["g2"], wt("intro", "second"), bend("intro") - 0.4, offset=(-0.07, -0.10),
          ignore=(cutter,))
    L.add("g1_intro", "1st gear", an["g1"], wt("intro", "first"), bend("intro") - 0.4, offset=(0.05, 0.10),
          ignore=(cutter,))
    L.add("sleeve_n", "Sleeve", an["sleeve"], wt("neutral", "sleeve"), bend("neutral") - 0.3, offset=(0.12, 0.02),
          occlusion=False)
    L.add("blocker_s", "Blocker ring", an["blocker2"], wt("sync", "blocker"), 22.9, offset=(-0.12, -0.05),
          occlusion=False)
    L.add("cone_s", "Cone", an["cone2"], wt("sync", "cone"), 22.9, offset=(-0.12, 0.08), occlusion=False)
    L.add("g2_s", "2nd gear", an["g2"], wt("sync", "second"), 22.9, offset=(-0.07, -0.08), ignore=(cutter,))
    L.add("cs", "Countershaft", an["cs"], wt("sync", "countershaft"), 28.9, offset=(0.08, 0.0), occlusion=False)
    L.add("input", "Input shaft", an["input"], wt("sync", "input"), 28.9, offset=(0.05, 0.10), occlusion=False)
    L.add("output", "Output shaft", an["output"], wt("sync", "output"), 31.6, offset=(0.04, 0.12),
          occlusion=False)
    L.add("blocker_e", "Blocker ring", an["blocker2"], wt("engage", "blocker"), 37.2, offset=(-0.12, -0.05),
          occlusion=False)
    L.add("sleeve_e", "Sleeve", an["sleeve"], wt("engage", "sleeve"), 39.8, offset=(0.12, 0.02), occlusion=False)
    L.add("dogs_e", "Dog teeth", an["dogs2"], wt("engage", "dog teeth"), 40.3, offset=(-0.13, 0.10),
          occlusion=False)
    L.add("g2_out", "2nd gear", an["g2"], wt("clutch_out", "taller"), DUR - 0.5, offset=(-0.06, -0.10),
          style="emph", occlusion=False)

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
    sb.extra.update(gearbox=G, studio=studio, visibility=vis, clearance=clr, cutter=cutter)
    return sb
