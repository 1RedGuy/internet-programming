"""s06 Final drive and differential (59 s): propshaft, ring and pinion, open diff straight and in a turn.

Dark studio (light rig rides with the car), gearbox tail + propshaft + rear axle, rear
corners (driveshafts, wheels, suspension) and, for the turn, the x-ray body.

Drivetrain state (one continuous Program, 1st gear, clutch engaged, car moving in the world;
the camera and the studio lights ride with the car root):

  prop .. ringpinion   15 km/h straight, x20 slow motion (ring/pinion need >= x10.6 at
                       15 km/h, FACTS O6; tyre tread 64 pitches needs >= x16.6)
  diffparts            time freezes (slowmo eases to 0 over 1 s, "PAUSED" badge) so the
                       exploded differential does not orbit; the freeze instant is chosen
                       (initial wheel phase `wheel0`, solved) so the cross-pin stands exactly
                       vertical: pin and spiders explode straight up/down, case halves, side
                       gears and stubs sideways along the axle.  Time resumes at the end.
  straight             15 km/h straight, x20 (spiders still on their pin, wheels equal)
  turn                 road speed held at 15 km/h while the curvature eases to 1/5 m^-1 (left,
                       R = 5 m): the outer (right) wheel speeds up 130.5 -> 149.8 rpm, the
                       inner slows to 111.1 rpm, the case stays exactly 130.5 rpm (FACTS
                       DIF-07).  x20 while the tyres are in frame (outer tread 0.33 pitch/
                       frame), then x12 for the close-up on the spiders (ring/pinion 0.31
                       pitch/frame; tyres out of frame).

Every rotation comes from the Track (axle/wheels/gearbox drive()); only presentation
(fades, explode, glow, camera, labels, HUD, floor guide lines) is keyed here.

Presentation
  * The car root is keyed from the Track for the whole scene (the car really drives ~10 m);
    camera (CameraPath parented to the car root) and the dark-studio light rig (Child Of
    the car root, key kept ~50 deg right of the camera) ride with it.  The cyclorama is
    world-fixed, centred on the path and enlarged; its floor gets a scene-local jointed
    concrete material (1.5 m slabs, world space) so the motion and the turn read.
  * Axle housing: whole until 6.8 s, then the 'half' cut (removed half fades out); the kept
    half fades out for the exploded view and back in after it.  Case halves ghosted
    (cv_opacity 0.15) from 32 s so the spiders show inside.  Rear corners hidden while the
    differential is exploded / in the straight close-up, back for the turn.  Body: faint
    x-ray shell (0.08) + feature lines, only for the wide turn shot (hidden in previews).
  * Turn: blue guide lines on the floor = the rear-wheel contact paths of the whole turn
    (extended to 90 deg of heading): the outer path is visibly longer.
  * Aliasing is validated for pinion/ring/ring bolts/side gears/spiders/U-joints/bearing
    rollers always, and for the tyre tread (64) and brake-disc vents (36) whenever a rear
    tyre is inside the camera frustum (frustum test, no occlusion credit).
"""
from __future__ import annotations

import math

import bpy
import numpy as np

from carviz import camera as CAM
from carviz import kin, lighting, materials, rig, scenebase, state, timeline
from carviz import spec as S
from carviz.assemblies import axle as AXL
from carviz.assemblies import body as BODY
from carviz.assemblies import wheels as WHL
from carviz.labels import Hud, Labels, fmt_rpm

try:  # the gearbox is only context for the opening shot (tail housing + output flange)
    from carviz.assemblies import gearbox as GBX
except Exception as _e:  # pragma: no cover
    print("[s06] gearbox assembly unavailable, building without it:", _e)
    GBX = None

SCENE_ID = "s06"
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


# ---------------------------------------------------------------------------
# Drivetrain plan
# ---------------------------------------------------------------------------
GEAR = 1
V_CRUISE = 15.0                     # km/h, held for the whole scene (case rpm constant in the turn)
KAPPA = 0.2                         # 1/m, left turn (R = 5 m)
SLOW_CRUISE = 1.0 / 20.0
SLOW_CLOSE = 1.0 / 12.0             # turn, close-up on the spiders (tyres out of frame)
T_FREEZE = (20.55, 21.55)           # slowmo -> 0
T_THAW = (31.45, 32.45)             # 0 -> SLOW_CRUISE
T_TURN = (41.0, 44.6)               # curvature ease-in (road speed held at V_CRUISE)
T_CLOSE = (48.9, 50.0)              # slowmo SLOW_CRUISE -> SLOW_CLOSE (after the tyres leave frame)
EXPLODE = (24.35, 26.05, 30.15, 31.75)   # out start, out end, back start, back end


def program(wheel0=0.0, pose0=(0.0, 0.0, 0.0)):
    P = state.Program(SCENE_ID)
    P.start_in_gear(GEAR)
    P.wheel0 = float(wheel0)
    P.car_pose0 = pose0
    s = P.slowmo
    s.key(0.0, SLOW_CRUISE, "step")
    s.key(T_FREEZE[0], SLOW_CRUISE, "linear")
    s.key(T_FREEZE[1], 0.0, "ease")
    s.key(T_THAW[0], 0.0, "linear")
    s.key(T_THAW[1], SLOW_CRUISE, "ease")
    s.key(T_CLOSE[0], SLOW_CRUISE, "linear")
    s.key(T_CLOSE[1], SLOW_CLOSE, "ease")
    P.speed_kmh.key(0.0, V_CRUISE, "step")
    P.curvature.key(0.0, 0.0, "step").key(T_TURN[0], 0.0, "linear").key(T_TURN[1], KAPPA, "ease")
    P.throttle_rpm.key(0.0, S.engine_rpm_at(V_CRUISE, GEAR), "step")
    return P


def solve_track():
    """Run the program twice: the second run shifts the initial wheel phase so that the
    differential case is at angle 0 (mod 2 pi) while time is frozen -> cross-pin vertical,
    +Z (spider_2 / pin explode direction) pointing up."""
    tr = program().run()
    i = tr.idx(0.5 * (T_FREEZE[1] + T_THAW[0]))
    w0 = (-float(tr.theta_case[i])) % TAU
    tr = program(wheel0=w0).run()
    return tr, w0


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------

def curve(t, keys, default=0.0):
    c = state.Curve(default)
    for k in keys:
        c.key(*k)
    return c(t)


def pulse(t, t_on, t_off, peak, hold=0.0, rise=0.6, fall=0.6):
    """0 -> peak at t_on (rise), -> hold shortly after, -> 0 at t_off (fall)."""
    ks = [(0.0, 0.0, "step"), (t_on, 0.0, "linear"), (t_on + rise, peak, "ease")]
    if hold > 0:
        ks += [(t_on + rise + 1.2, peak, "linear"), (t_on + rise + 2.0, hold, "ease"),
               (t_off - fall, hold, "linear")]
    else:
        ks += [(t_off - fall, peak, "linear")]
    ks += [(t_off, 0.0, "ease")]
    return curve(t, ks)


def _compress(frames, vals, interp):
    v = np.asarray(vals, dtype=float)
    keep = np.ones(len(v), bool)
    if len(v) > 2:
        if interp == "CONSTANT":
            keep[1:] = v[1:] != v[:-1]
        else:
            same_prev = np.r_[False, v[1:] == v[:-1]]
            same_next = np.r_[v[:-1] == v[1:], False]
            keep = ~(same_prev & same_next)
    return np.asarray(frames, dtype=float)[keep], v[keep]


def bake_vis(ob, frames, op, glow=None):
    """cv_opacity (compressed LINEAR keys) + hide_render/hide_viewport (CONSTANT keys)."""
    if ob is None or ob.type != "MESH":
        return
    a = np.round(np.clip(np.asarray(op, dtype=float), 0.0, 1.0), 4)
    hide = (a < 0.02).astype(float)
    for path in ("hide_render", "hide_viewport"):
        f, v = _compress(frames, hide, "CONSTANT")
        rig.bake_channel(ob, path, -1, f, v, "CONSTANT")
    if "cv_opacity" not in ob:
        ob["cv_opacity"] = 1.0
    f, v = _compress(frames, np.where(a < 0.02, 0.0, a), "LINEAR")
    rig.bake_channel(ob, '["cv_opacity"]', -1, f, v)
    if glow is not None:
        bake_glow(ob, frames, glow)


def bake_glow(ob, frames, g):
    if ob is None or ob.type != "MESH":
        return
    if "cv_glow" not in ob:
        ob["cv_glow"] = 0.0
    f, v = _compress(frames, np.round(np.asarray(g, dtype=float), 5), "LINEAR")
    rig.bake_channel(ob, '["cv_glow"]', -1, f, v)


def meshes_of(objs):
    out = []
    for o in objs:
        if o is None:
            continue
        for x in [o] + list(o.children_recursive):
            if x.type == "MESH" and x not in out:
                out.append(x)
    return out


# ---------------------------------------------------------------------------
# Studio floor (polished concrete with saw-cut joints: world space, so the car's
# motion reads against it) + Workbench-only guide lines for previews
# ---------------------------------------------------------------------------
SLAB = 1.5          # m
CYC_R = 12.0        # cyclorama radius (the lights use the close-up studio size)
JOINT_W = 0.006


def _floor_material():
    m = bpy.data.materials.get("s06_floor")
    if m is not None:
        return m
    m = bpy.data.materials.new("s06_floor")
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    N, L = nt.nodes, nt.links
    out = N.new("ShaderNodeOutputMaterial")
    bsdf = N.new("ShaderNodeBsdfPrincipled")
    L.new(bsdf.outputs[0], out.inputs[0])
    geo = N.new("ShaderNodeNewGeometry")
    sep = N.new("ShaderNodeSeparateXYZ")
    L.new(geo.outputs["Position"], sep.inputs[0])

    def math_(op, a, b=None, clamp=False):
        n = N.new("ShaderNodeMath")
        n.operation = op
        n.use_clamp = clamp
        for k, v in enumerate((a, b)):
            if v is None:
                continue
            if isinstance(v, (int, float)):
                n.inputs[k].default_value = float(v)
            else:
                L.new(v, n.inputs[k])
        return n.outputs[0]

    def joint(c):
        u = math_("FRACT", math_("DIVIDE", c, SLAB))
        d = math_("MULTIPLY", math_("MINIMUM", u, math_("SUBTRACT", 1.0, u)), SLAB)   # m to the joint
        mr = N.new("ShaderNodeMapRange")
        mr.interpolation_type = "SMOOTHSTEP"
        L.new(d, mr.inputs["Value"])
        mr.inputs["From Min"].default_value = JOINT_W * 0.5 + 0.0025
        mr.inputs["From Max"].default_value = JOINT_W * 0.5
        return mr.outputs["Result"]

    jm = math_("MAXIMUM", joint(sep.outputs["X"]), joint(sep.outputs["Y"]))
    n1 = N.new("ShaderNodeTexNoise")
    n1.inputs["Scale"].default_value = 0.45
    n1.inputs["Detail"].default_value = 4.0
    L.new(geo.outputs["Position"], n1.inputs["Vector"])
    n2 = N.new("ShaderNodeTexNoise")
    n2.inputs["Scale"].default_value = 9.0
    n2.inputs["Detail"].default_value = 3.0
    L.new(geo.outputs["Position"], n2.inputs["Vector"])
    ramp = N.new("ShaderNodeMapRange")
    L.new(n1.outputs["Fac"], ramp.inputs["Value"])
    ramp.inputs["From Min"].default_value = 0.35
    ramp.inputs["From Max"].default_value = 0.65
    mix_slab = N.new("ShaderNodeMix")
    mix_slab.data_type = "RGBA"
    L.new(ramp.outputs["Result"], mix_slab.inputs["Factor"])
    mix_slab.inputs[6].default_value = (0.020, 0.020, 0.021, 1.0)
    mix_slab.inputs[7].default_value = (0.033, 0.032, 0.033, 1.0)
    fine = N.new("ShaderNodeMix")
    fine.data_type = "RGBA"
    fine.blend_type = "MULTIPLY"
    fm = N.new("ShaderNodeMapRange")
    L.new(n2.outputs["Fac"], fm.inputs["Value"])
    fm.inputs["From Min"].default_value = 0.3
    fm.inputs["From Max"].default_value = 0.7
    fm.inputs["To Min"].default_value = 0.85
    fm.inputs["To Max"].default_value = 1.12
    comb = N.new("ShaderNodeCombineXYZ")
    for k in range(3):
        L.new(fm.outputs["Result"], comb.inputs[k])
    fine.inputs["Factor"].default_value = 1.0
    L.new(mix_slab.outputs[2], fine.inputs[6])
    L.new(comb.outputs[0], fine.inputs[7])
    mix_j = N.new("ShaderNodeMix")
    mix_j.data_type = "RGBA"
    L.new(jm, mix_j.inputs["Factor"])
    L.new(fine.outputs[2], mix_j.inputs[6])
    mix_j.inputs[7].default_value = (0.005, 0.005, 0.006, 1.0)
    L.new(mix_j.outputs[2], bsdf.inputs["Base Color"])
    rough = math_("ADD", math_("MULTIPLY", ramp.outputs["Result"], 0.16), 0.40)
    rmix = N.new("ShaderNodeMix")
    rmix.data_type = "FLOAT"
    L.new(jm, rmix.inputs["Factor"])
    L.new(rough, rmix.inputs[2])
    rmix.inputs[3].default_value = 0.9
    L.new(rmix.outputs[0], bsdf.inputs["Roughness"])
    bsdf.inputs["Specular IOR Level"].default_value = 0.45
    m.diffuse_color = (0.035, 0.035, 0.038, 1.0)
    m.roughness = 0.4
    return m


def _preview_grid(center, half, col):
    """Workbench-only floor joints (previews cannot show the shader's joints)."""
    import bmesh
    bm = bmesh.new()
    cx, cy = center
    w = 0.02
    n0, n1 = int(math.floor((cx - half) / SLAB)), int(math.ceil((cx + half) / SLAB))
    m0, m1 = int(math.floor((cy - half) / SLAB)), int(math.ceil((cy + half) / SLAB))
    for k in range(n0, n1 + 1):
        x = k * SLAB
        vs = [bm.verts.new(p) for p in ((x - w, cy - half, 0.001), (x + w, cy - half, 0.001),
                                         (x + w, cy + half, 0.001), (x - w, cy + half, 0.001))]
        bm.faces.new(vs)
    for k in range(m0, m1 + 1):
        y = k * SLAB
        vs = [bm.verts.new(p) for p in ((cx - half, y - w, 0.0012), (cx + half, y - w, 0.0012),
                                         (cx + half, y + w, 0.0012), (cx - half, y + w, 0.0012))]
        bm.faces.new(vs)
    me = bpy.data.meshes.new("s06_preview_grid")
    bm.to_mesh(me)
    bm.free()
    mat = bpy.data.materials.new("s06_preview_grid")
    mat.diffuse_color = (0.16, 0.16, 0.17, 1.0)
    me.materials.append(mat)
    ob = bpy.data.objects.new("s06_preview_grid", me)
    ob.display.show_shadows = False
    rig.link(ob, col)
    return ob


# ---------------------------------------------------------------------------
# Wheel paths drawn on the floor for the turn (rear-wheel contact paths of the whole
# planned turn, extended at constant curvature to 90 deg of heading change)
# ---------------------------------------------------------------------------
PATH_W = 0.026


def _wheel_paths(track, i0, extra_deg=90.0):
    """World (x, y) polylines of the RL / RR contact points from frame i0 on, extended at
    KAPPA until the heading has changed by extra_deg from frame i0."""
    hx = -np.sin(track.car_heading)
    hy = np.cos(track.car_heading)
    rx = track.car_x - hx * S.WHEELBASE
    ry = track.car_y - hy * S.WHEELBASE
    pts_c = np.stack([rx, ry], 1)[i0:]
    hd = track.car_heading[i0:]
    # extend
    h_end = float(track.car_heading[i0]) + math.radians(extra_deg)
    cx, cy, h = float(pts_c[-1, 0]), float(pts_c[-1, 1]), float(hd[-1])
    ext_c, ext_h = [], []
    ds = 0.05
    while h < h_end:
        cx += -math.sin(h) * ds
        cy += math.cos(h) * ds
        h += KAPPA * ds
        ext_c.append((cx, cy))
        ext_h.append(h)
    if ext_c:
        pts_c = np.vstack([pts_c, np.array(ext_c)])
        hd = np.r_[hd, np.array(ext_h)]
    r = np.stack([np.cos(hd), np.sin(hd)], 1)
    half = S.TRACK_REAR / 2
    left = pts_c - r * half
    right = pts_c + r * half
    # thin out (keep ~2 cm spacing)
    def thin(p):
        keep = [0]
        for k in range(1, len(p)):
            if np.linalg.norm(p[k] - p[keep[-1]]) > 0.02:
                keep.append(k)
        return p[keep]
    return thin(left), thin(right)


def _ribbon(name, pts, width, z, mat, col):
    import bmesh
    bm = bmesh.new()
    p = np.asarray(pts)
    d = np.gradient(p, axis=0)
    d /= np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-9)
    nrm = np.stack([-d[:, 1], d[:, 0]], 1)
    L = [bm.verts.new((*(p[k] - nrm[k] * width / 2), z)) for k in range(len(p))]
    R = [bm.verts.new((*(p[k] + nrm[k] * width / 2), z)) for k in range(len(p))]
    for k in range(len(p) - 1):
        bm.faces.new((L[k], R[k], R[k + 1], L[k + 1]))
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    me.materials.append(mat)
    ob = bpy.data.objects.new(name, me)
    ob.visible_shadow = False
    ob.display.show_shadows = False
    materials.ensure_props(ob)
    rig.link(ob, col)
    return ob


def _path_material():
    m = bpy.data.materials.get("s06_path")
    if m is not None:
        return m
    m = bpy.data.materials.new("s06_path")
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    em = nt.nodes.new("ShaderNodeEmission")
    em.inputs["Color"].default_value = (0.10, 0.42, 1.0, 1.0)
    em.inputs["Strength"].default_value = 1.4
    tr = nt.nodes.new("ShaderNodeBsdfTransparent")
    at = nt.nodes.new("ShaderNodeAttribute")
    at.attribute_type = "OBJECT"
    at.attribute_name = "cv_opacity"
    mix = nt.nodes.new("ShaderNodeMixShader")
    nt.links.new(at.outputs["Fac"], mix.inputs[0])
    nt.links.new(tr.outputs[0], mix.inputs[1])
    nt.links.new(em.outputs[0], mix.inputs[2])
    nt.links.new(mix.outputs[0], out.inputs[0])
    m.diffuse_color = (0.10, 0.42, 1.0, 1.0)
    return m


# ---------------------------------------------------------------------------
# Camera plan (car frame: the camera rides with the car root).
# (time, target, azimuth deg, radius, height above target, lens, f-stop, mode)
# azimuth as camera.orbit: 0 = behind (-Y), +90 = right (+X), +-180 = front, -90 = left
# ---------------------------------------------------------------------------
YD, ZD = S.Y_DIFF, S.Z_DIFF
POSES = [
    # prop: from the gearbox tail along the spinning propshaft to the rear axle
    (0.0, (0.0, -1.16, 0.36), -142.0, 0.78, 0.30, 40.0, 5.6, "step"),
    (2.4, (0.0, -1.62, 0.345), -108.0, 0.82, 0.20, 40.0, 5.6, "cubic"),
    (4.6, (0.0, -2.28, 0.32), -122.0, 1.00, 0.38, 40.0, 5.6, "cubic"),
    (6.3, (0.0, -2.52, 0.31), -134.0, 0.98, 0.48, 40.0, 5.6, "cubic"),
    # ringpinion: in on the pinion / ring mesh (housing cut away), then up to the top view
    # (azimuths continue past -180 so the camera swings round the front, not the back)
    (8.6, (-0.02, -2.53, 0.355), -210.0, 0.52, 0.32, 45.0, 5.6, "cubic"),
    (12.5, (-0.02, -2.54, 0.36), -200.0, 0.49, 0.28, 45.0, 5.6, "cubic"),
    (15.3, (-0.02, -2.55, 0.36), -194.0, 0.47, 0.26, 45.0, 5.6, "cubic"),
    (18.2, (0.0, -2.52, 0.32), -182.0, 0.14, 1.02, 40.0, 6.3, "cubic"),
    (19.4, (0.0, -2.52, 0.32), -178.0, 0.15, 1.00, 40.0, 6.3, "cubic"),
    # diffparts: ring back face (bolts) from the left-rear, then wider for the exploded view
    (22.0, (-0.03, YD, ZD + 0.02), -64.0, 0.56, 0.42, 45.0, 5.6, "cubic"),
    (24.1, (-0.03, YD, ZD + 0.03), -58.0, 0.60, 0.42, 45.0, 5.6, "cubic"),
    # (pulled back so pin top .. lower spider fit above the subtitle band)
    (26.3, (-0.03, YD, ZD + 0.035), -36.0, 0.96, 0.40, 40.0, 6.3, "cubic"),
    (30.0, (-0.03, YD, ZD + 0.035), -30.0, 0.96, 0.40, 40.0, 6.3, "cubic"),
    # straight: close on the (ghosted) case from the right-rear, above the cut housing
    (32.6, (0.0, YD + 0.01, ZD + 0.01), 26.0, 0.42, 0.46, 45.0, 5.6, "cubic"),
    (37.8, (0.0, YD + 0.01, ZD + 0.01), 14.0, 0.40, 0.48, 45.0, 5.6, "cubic"),
    # pull out and up: steep high view over the rear axle for the turn (engine bay out of frame)
    (41.0, (0.0, -2.45, 0.30), 0.0, 0.85, 2.25, 30.0, 8.0, "cubic"),
    (46.9, (0.0, -2.45, 0.30), 3.0, 0.82, 2.20, 30.0, 8.0, "cubic"),
    # push in on the spiders (steep: the tyres stay out of frame at x8)
    (49.3, (0.0, YD + 0.01, ZD + 0.01), 12.0, 0.25, 0.56, 40.0, 5.6, "cubic"),
    (55.0, (0.0, YD + 0.01, ZD + 0.01), 19.0, 0.25, 0.56, 40.0, 5.6, "cubic"),
    (DUR, (0.0, YD + 0.01, ZD + 0.01), 22.0, 0.25, 0.57, 40.0, 5.6, "cubic"),
]
KEY_OFFSET = 50.0          # key light azimuth relative to the camera (deg, to the camera's right)
CAM_SMOOTH = 0.3           # s Gaussian smoothing of the pose parameters


def _gauss(x, sigma_frames):
    r = int(3 * sigma_frames)
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma_frames) ** 2)
    k /= k.sum()
    xp = np.r_[np.full(r, x[0]), x, np.full(r, x[-1])]
    return np.convolve(xp, k, mode="valid")


def camera_samples(t):
    keys = ("tx", "ty", "tz", "az", "r", "h", "lens", "f")
    cs = {k: state.Curve() for k in keys}
    for (tt, T, az, r, h, lens, f, mode) in POSES:
        for k, v in zip(keys, (T[0], T[1], T[2], az, r, h, lens, f)):
            cs[k].key(tt, v, mode)
    p = {k: _gauss(c(t), CAM_SMOOTH * FPS) for k, c in cs.items()}
    T = np.stack([p["tx"], p["ty"], p["tz"]], 1)
    az = np.radians(p["az"])
    eye = T + np.stack([p["r"] * np.sin(az), -p["r"] * np.cos(az), p["h"]], 1)
    return eye, T, p["lens"], p["f"], np.degrees(az)


def in_frame(eye, tgt, lens, pts, sensor=36.0, aspect=16 / 9, margin=1.04):
    """Per frame: does any of pts (car frame, (k,3)) project inside the frame?"""
    out = np.zeros(len(eye), bool)
    up = np.array([0.0, 0.0, 1.0])
    for i in range(len(eye)):
        f = tgt[i] - eye[i]
        f /= np.linalg.norm(f)
        rgt = np.cross(f, up)
        rgt /= max(np.linalg.norm(rgt), 1e-9)
        u = np.cross(rgt, f)
        d = pts - eye[i]
        z = d @ f
        tx = 0.5 * sensor / lens[i] * margin
        ty = tx / aspect
        ok = (z > 0.02) & (np.abs(d @ rgt) < tx * z) & (np.abs(d @ u) < ty * z)
        out[i] = bool(np.any(ok))
    return out


def tyre_points(side):
    """Sample points on a rear tyre (car frame) for frustum tests."""
    sx = -1.0 if side == "RL" else 1.0
    xc = sx * S.TRACK_REAR / 2
    ang = np.linspace(0, TAU, 48, endpoint=False)
    pts = []
    for dx in (-0.10, 0.0, 0.10):
        for rr in (0.31, 0.24):
            pts.append(np.stack([np.full_like(ang, xc + dx), S.Y_REAR_AXLE + rr * np.cos(ang),
                                 S.WHEEL_CENTER_Z + rr * np.sin(ang)], 1))
    return np.concatenate(pts)


def _rollers_n(d, D, B):
    """Roller count of axle._taper_bearing (same formula)."""
    rm = 0.25 * (d + D)
    ar, g = math.radians(13.0), math.radians(2.0)
    rho_m = rm / math.sin(ar) * math.sin(g)
    return int(TAU * rm / (2.0 * rho_m * 1.16))


# ---------------------------------------------------------------------------
# build
# ---------------------------------------------------------------------------

def build(quality: str) -> scenebase.SceneBuild:
    sc = scenebase.new_scene(SCENE_ID)
    preview = quality == "preview"
    detail = "low" if preview else "high"

    # ---------------- drivetrain state ----------------------------------
    track, wheel0 = solve_track()
    n, t, fr = track.n, track.t, track.frames

    # ---------------- car root + assemblies ------------------------------
    col = rig.collection("car")
    root = rig.empty("car_root", col=col, size=0.3)
    root.rotation_mode = "XYZ"
    rig.bake_channel(root, "location", 0, fr, track.car_x)
    rig.bake_channel(root, "location", 1, fr, track.car_y)
    rig.bake_channel(root, "rotation_euler", 2, fr, track.car_heading)

    A = AXL.build({"cutaways": ["none", "half"], "detail": detail})
    A.root.parent = root
    W = WHL.build({"detail": detail, "corners": ["RL", "RR"], "suspension": True})
    W.root.parent = root
    B = None
    if not preview:          # x-ray shell: Workbench cannot show it (opacity is ignored)
        B = BODY.build({"detail": detail, "xray_edges": True})
        B.root.parent = root
    G = None
    if GBX is not None:
        try:
            G = GBX.build({"cutaway": "none", "detail": detail})
            G.root.parent = root
        except Exception as e:  # pragma: no cover
            print("[s06] gearbox build failed, continuing without it:", e)
            G = None

    # ---------------- studio ---------------------------------------------
    # lights ride with the car (Child Of the car root); the cyclorama is world-fixed and
    # centred on the car's path, its floor re-surfaced with jointed concrete
    studio = lighting.setup_studio("dark", center=(0.0, -2.3, 0.33), size=0.9, follow=root,
                                   follow_heading=True)
    lighting.setup_color_management(sc)
    path_xy = np.stack([track.car_x, track.car_y], 1)
    hx, hy = -np.sin(track.car_heading), np.cos(track.car_heading)
    ctr = path_xy - np.stack([hx, hy], 1) * 1.6            # car centre (world) per frame
    mid = 0.5 * (ctr.min(0) + ctr.max(0))
    reach = float(np.max(np.linalg.norm(ctr - mid, axis=1))) + 3.0
    cyc_r = max(CYC_R, reach / 0.7)                       # floor disc = 0.7 R (lighting._cyclorama)
    for ob in (studio["floor"], studio["backdrop"]):     # world-fixed, centred on the path
        ob.location = (float(mid[0]), float(mid[1]), 0.0)
        ob.scale = (cyc_r, cyc_r, cyc_r)
    studio["floor"].data.materials[0] = _floor_material()
    if preview:
        _preview_grid((float(mid[0]), float(mid[1])), 10.0, studio["collection"])

    # ---------------- drive (all mechanical motion from the Track) -------
    ex = curve(t, [(0.0, 0.0, "step"), (EXPLODE[0], 0.0, "linear"), (EXPLODE[1], 1.0, "ease"),
                   (EXPLODE[2], 1.0, "linear"), (EXPLODE[3], 0.0, "ease")])
    var = ["none" if x < 6.8 else "half" for x in t]
    A.drive(track, {"variant": var, "explode": ex, "removed": 1.0})
    W.drive(track, {})
    if G is not None:
        G.drive(track, {})
        keep = {"case", "case_web", "tail_housing", "output_flange", "output_shaft", "lever", "knob", "boot"}
        for k, ob in G.parts.items():
            if ob.type == "MESH" and k not in keep:
                ob.hide_render = ob.hide_viewport = True

    # ---------------- presentation: axle housing / case / wheels / body --
    PA = A.parts
    pc = A.meta["cutaway_pieces"]["half"]
    housing_out = curve(t, [(0.0, 1.0, "step"), (23.0, 1.0, "linear"), (24.2, 0.0, "ease"),
                            (31.55, 0.0, "linear"), (32.5, 1.0, "ease")])
    removed_op = curve(t, [(0.0, 1.0, "step"), (6.85, 1.0, "linear"), (8.4, 0.0, "ease")])
    half = np.array([v == "half" for v in var])
    for k in pc["replaces"]:
        bake_vis(PA.get(k), fr, np.where(half, 0.0, 1.0))
    for k in pc["kept"]:
        bake_vis(PA.get(k), fr, np.where(half, housing_out, 0.0))
    for k in pc["removed"]:
        bake_vis(PA.get(k), fr, np.where(half, removed_op * housing_out, 0.0))
    # ghosted case after reassembly (spiders visible inside)
    CASE_GHOST = 0.15
    case_op = curve(t, [(0.0, 1.0, "step"), (31.9, 1.0, "linear"), (33.0, CASE_GHOST, "ease")])
    for k in ("case_left", "case_right"):
        bake_vis(PA[k], fr, case_op)

    # rear corners: hidden while the differential is exploded and in the straight close-up
    wheel_op = curve(t, [(0.0, 1.0, "step"), (22.6, 1.0, "linear"), (23.6, 0.0, "ease"),
                         (38.4, 0.0, "linear"), (40.0, 1.0, "ease")])
    for ob in W.meshes():
        bake_vis(ob, fr, wheel_op)
    # x-ray body for the turn (faint shell + feature lines)
    XRAY = 0.08
    body_in = curve(t, [(0.0, 0.0, "step"), (38.8, 0.0, "linear"), (40.8, 1.0, "ease"),
                        (46.8, 1.0, "linear"), (48.2, 0.0, "ease")])
    if B is not None:
        B.drive(track, {"exterior_opacity": XRAY * body_in, "glass_opacity": 0.0 * body_in,
                        "interior_opacity": 0.0 * body_in, "underbody_opacity": 0.0 * body_in,
                        "xray_edges_opacity": 0.6 * body_in})
        # headliner / door cards / parcel shelf would fog the view from above
        rig.bake_fade([B.parts["cabin_trim"]], fr, np.zeros(n))
    if preview:   # Workbench shadow volumes are very slow on llvmpipe with this many triangles
        for ob in bpy.context.scene.objects:
            if ob.type == "MESH":
                ob.display.show_shadows = False

    # floor guide lines: rear-wheel paths of the turn
    i_turn = track.idx(T_TURN[0])
    pl, pr = _wheel_paths(track, i_turn)
    pm = _path_material()
    path_op = curve(t, [(0.0, 0.0, "step"), (wt("turn", "outer") - 0.6, 0.0, "linear"),
                        (wt("turn", "outer") + 0.6, 0.85, "ease"), (47.0, 0.85, "linear"), (48.4, 0.0, "ease")])
    path_obs = []
    for nm, pts in (("s06_path_inner", pl), ("s06_path_outer", pr)):
        ob = _ribbon(nm, pts, PATH_W, 0.0015, pm, studio["collection"])
        bake_vis(ob, fr, path_op)
        path_obs.append(ob)

    # ---------------- glow --------------------------------------------------
    GP = 0.022          # warm highlight (materials: emission 2 x cv_glow): subtle on bright steel
    glow = {}

    def add_glow(keys, arr):
        for m in meshes_of([PA.get(k) for k in keys]):
            glow[m.name] = np.maximum(glow.get(m.name, np.zeros(n)), arr)

    t_prop = wt("prop", "propeller")
    GD = 0.4 * GP       # dark parts (painted tube, cast case): the emission dominates their base colour
    add_glow([k for k in PA if k.startswith("prop_")], pulse(t, t_prop, 4.2, GD))
    add_glow(["pinion", "companion_flange"], pulse(t, wt("ringpinion", "pinion"), 11.6, GP))
    add_glow(["ring_gear"], pulse(t, wt("ringpinion", "ring"), 13.2, GP))
    # "turning the drive through a right angle": pinion -> ring -> both output stubs
    add_glow(["pinion", "companion_flange", "ring_gear", "stub_left", "stub_right"],
             pulse(t, wt("ringpinion", "turning"), 18.8, GP))
    add_glow(["ring_gear"], pulse(t, wt("diffparts", "bolted"), 24.0, GP))
    add_glow(["case_left", "case_right"], pulse(t, wt("diffparts", "differential"), 25.6, GD))
    add_glow(["spider_1", "spider_2"], pulse(t, wt("diffparts", "spider"), 28.0, GP))
    add_glow(["side_gear_left", "side_gear_right"], pulse(t, wt("diffparts", "side"), 29.7, GP))
    add_glow(["stub_left", "stub_right"], pulse(t, wt("diffparts", "splined"), 31.2, GP))
    t_one = wt("straight", "Everything")
    add_glow(["ring_gear", "spider_1", "spider_2", "side_gear_left", "side_gear_right", "cross_pin", "stub_left",
              "stub_right"], pulse(t, t_one, t_one + 2.6, GP))
    add_glow(["case_left", "case_right"], pulse(t, t_one, t_one + 2.6, GD))
    add_glow(["spider_1", "spider_2"], pulse(t, 47.8, 53.6, GP))
    add_glow(["case_left", "case_right"], pulse(t, wt("turn", "case"), 57.8, 0.03, 0.02))   # ghosted, dark
    by_name = {o.name: o for o in A.meshes()}
    for name, g in glow.items():
        bake_glow(by_name.get(name) or bpy.data.objects[name], fr, np.clip(g, 0.0, 0.1))

    # ---------------- camera + lights -------------------------------------
    eye, tgt, lens, fstop, az = camera_samples(t)
    C = CAM.CameraPath(name="cam_s06", lens=40.0, fstop=5.6, clip=(0.02, 80.0))
    for i in range(n):
        C.key(t[i], eye=tuple(eye[i]), target=tuple(tgt[i]), lens=float(lens[i]), fstop=float(fstop[i]),
              mode="linear")
    cam = C.bake(fr, FPS, parent=root)
    rig.bake_channel(studio["rig"], "rotation_euler", 2, fr,
                     np.radians(np.unwrap(az, period=360.0) + KEY_OFFSET - 225.0))

    # ---------------- validation --------------------------------------------
    vis = {}
    for c in ("RL", "RR"):
        v = in_frame(eye, tgt, lens, tyre_points(c)) & (wheel_op > 0.02)
        vis[c] = v | np.r_[v[1:], False] | np.r_[False, v[:-1]]
    sp_rel = kin.spider_spin(track.theta_RL, track.theta_RR)
    cg = A.meta["cage"]
    n_ph = _rollers_n(*AXL.PB_HEAD)
    n_cb = _rollers_n(*AXL.CB)
    aliasing = {
        "pinion 10T": (track.theta_out, TAU / S.Z_PINION_TEETH, None),
        "ring gear 41T": (track.theta_case, TAU / S.Z_RING, None),
        "ring bolts 10": (track.theta_case, TAU / AXL.N_RING_BOLTS, None),
        "side gear L 16T": (track.theta_RL, TAU / S.Z_SIDE_GEAR, None),
        "side gear R 16T": (track.theta_RR, TAU / S.Z_SIDE_GEAR, None),
        "spiders 10T (on pin)": (sp_rel, TAU / S.Z_SPIDER, None),
        "U-joint crosses (4 arms)": (track.theta_out, TAU / 4, None),
        "pinion bearing rollers": (track.theta_out * cg["pinion_head"], TAU / n_ph, None),
        "carrier bearing rollers": (track.theta_case * cg["carrier"], TAU / n_cb, None),
        "tyre tread RL (64)": (track.theta_RL, TAU / 64, vis["RL"]),
        "tyre tread RR (64)": (track.theta_RR, TAU / 64, vis["RR"]),
        "brake disc vents RL (36)": (track.theta_RL, TAU / 36, vis["RL"]),
        "brake disc vents RR (36)": (track.theta_RR, TAU / 36, vis["RR"]),
    }
    track.validate(aliasing=aliasing)          # raises on any violation

    # ---------------- labels -------------------------------------------------
    L = Labels()
    AN = A.anchors
    body_objs = tuple(B.meshes()) if B is not None else ()
    ghost = (PA["case_left"], PA["case_right"])
    # propshaft label: the point of the shaft axis nearest the camera's line of sight (it
    # slides smoothly along the shaft as the camera travels, so it never leaves the frame)
    J1 = np.array(A.meta["prop"]["J1"])
    J2 = np.array(A.meta["prop"]["J2"])

    def prop_anchor(f):
        i = int(np.clip(f - 1, 0, n - 1))
        e, g = eye[i], tgt[i]
        d = (g - e) / np.linalg.norm(g - e)
        u = J2 - J1
        w0 = J1 - e
        a, b, c = u @ u, u @ d, d @ d
        dd, ee = u @ w0, d @ w0
        s_ = np.clip((b * ee - c * dd) / max(a * c - b * b, 1e-12), 0.05, 0.95)
        p = J1 + s_ * u
        h = track.car_heading[i]
        ch, sh = math.cos(h), math.sin(h)
        return (track.car_x[i] + ch * p[0] - sh * p[1], track.car_y[i] + sh * p[0] + ch * p[1], p[2])
    L.add("prop", "Propeller shaft", prop_anchor, t_prop, 6.1, offset=(0.06, -0.12), occlusion=False)
    L.add("ujoint", "Universal joint", AN["ujoint_front"], 0.7, 1.9, offset=(-0.06, -0.13), style="dim")
    L.add("rear_axle", "Rear axle", AN["diff_housing"], wt("prop", "rear"), 6.4, offset=(0.07, -0.10),
          occlusion=False)
    L.add("pinion", "Pinion 10T", AN["pinion"], wt("ringpinion", "pinion"), 19.6, offset=(0.08, -0.08),
          occlusion=False)
    L.add("ring", "Ring gear 41T", AN["ring_gear"], wt("ringpinion", "ring"), 15.9, offset=(-0.12, 0.03))
    L.add("ring_top", "Ring gear 41T", AN["ring_gear"], 17.3, 19.6, offset=(-0.12, 0.03))
    L.add("ring2", "Ring gear", AN["ring_gear"], wt("diffparts", "ring"), 23.7, offset=(-0.07, -0.08))
    # left (flange) half, outboard of the ring: the side the camera sees in this beat
    L.add("case", "Differential case", (PA["x_case_left"], (0.0, -0.072, 0.040)), wt("diffparts", "differential"),
          25.3, offset=(0.08, -0.08), occlusion=False)
    L.add("spiders", "Spider gears", AN["spider_gear"], wt("diffparts", "spider"), 30.4, offset=(0.08, -0.06))
    L.add("sides", "Side gears", AN["side_gear_right"], wt("diffparts", "side"), 30.4, offset=(0.07, 0.07))
    L.add("stub", "Driveshaft stub", AN["stub_right"], wt("diffparts", "driveshaft"), 31.4, offset=(0.05, 0.10))
    centre = (A.parts["diff_pivot"], (0.0, 0.0, 0.0))
    L.add("spiders_s", "Spider gears", centre, wt("straight", "spider"), 38.2, offset=(-0.10, -0.12),
          ignore=ghost)
    L.add("outer", "Outer wheel", W.anchors["tire_RR"], wt("turn", "outer"), 47.4, offset=(0.06, -0.08),
          ignore=body_objs)
    L.add("inner", "Inner wheel", W.anchors["tire_RL"], wt("turn", "inner"), 47.4, offset=(-0.06, -0.08),
          ignore=body_objs)
    L.add("spiders_t", "Spider gears", centre, 48.6, 53.6, offset=(-0.10, -0.12), ignore=ghost + body_objs)
    L.add("case_t", "Differential case", AN["diff_case"], wt("turn", "case"), DUR - 0.5, offset=(0.09, -0.08),
          ignore=ghost + body_objs)

    # ---------------- HUD ------------------------------------------------------
    H = Hud(track)
    H.add("fade", 0.0, 0.8, fade=0.0, color="black", alpha=lambda i: float(max(0.0, 1.0 - t[i] / 0.6)))
    H.add("fade", DUR - 0.6, DUR + 1.0, fade=0.0, color="black",
          alpha=lambda i: float(min(1.0, max(0.0, (t[i] - (DUR - 0.55)) / 0.5))))
    H.add("section_title", 0.4, 4.4, number=6, title="Final drive & differential")
    H.add("slowmo", 0.6, T_FREEZE[0] + 0.1, factor=lambda i: int(round(1.0 / max(track.slowmo[i], 1e-3))))
    H.add("status", T_FREEZE[0] + 0.5, T_THAW[0] + 0.2, text="PAUSED", kind="info",
          pos=[1.0 - 0.05 * 9 / 16, 0.05], anchor="tr")
    H.add("slowmo", T_THAW[1] - 0.3, DUR, factor=lambda i: int(round(1.0 / max(track.slowmo[i], 1e-3))))

    def rpm(arr, i):
        return f"{fmt_rpm(arr[i])} rpm"

    H.add("readouts", 0.8, bend("prop") + 0.2, rows=lambda i: [["Propshaft", rpm(track.rpm_out, i)]])
    H.add("readouts", bend("prop") + 0.2, bend("ringpinion") + 0.1,
          rows=lambda i: [["Propshaft", rpm(track.rpm_out, i)], ["Ring gear", rpm(track.rpm_case, i)]])
    H.add("ratio_card", wt("ringpinion", "final"), bend("ringpinion") - 0.1, ratio_text="4.10 : 1",
          caption="Final drive  41 / 10")
    spider_rpm = np.abs(track.rpm_RR - track.rpm_RL) / 2.0 * S.Z_SIDE_GEAR / S.Z_SPIDER

    t_match0, t_match1 = wt("straight", "both"), bend("straight")
    t_outer = wt("turn", "outer")
    t_sp0, t_sp1 = wt("straight", "spider"), wt("straight", "Everything")
    t_sp2, t_sp3 = wt("turn", "spider"), wt("turn", "letting")
    t_up, t_slow = wt("turn", "speed"), wt("turn", "slows")
    t_case = wt("turn", "case")

    def on(i, a, b):
        return bool(a <= t[i] <= b)

    def rows_axle(i):
        return [["Left wheel", rpm(track.rpm_RL, i), on(i, t_match0, t_match1) or on(i, t_slow, t_case)],
                ["Right wheel", rpm(track.rpm_RR, i), on(i, t_match0, t_match1) or on(i, t_outer, t_outer + 3.0)
                 or on(i, t_up, t_case)],
                ["Diff case", rpm(track.rpm_case, i), on(i, t_case, DUR)],
                ["Spiders on pin", rpm(spider_rpm, i), on(i, t_sp0, t_sp1) or on(i, t_sp2, t_sp3 + 2.0)]]
    H.add("readouts", bstart("straight") + 0.3, DUR, rows=rows_axle, title="Rear axle")

    sb = scenebase.SceneBuild(SCENE_ID, track, cam, L, H, motion_blur=False,
                              preview_hide=())
    sb.extra.update(axle=A, wheels=W, body=B, gearbox=G, studio=studio, wheel0=wheel0, tyre_visible=vis,
                    car_root=root)
    return sb
