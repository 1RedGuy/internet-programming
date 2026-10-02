"""Driveshafts + CV joints, hubs, brakes, wheels/tyres and suspension (prefix ``whl_``).

    from carviz.assemblies import wheels
    W = wheels.build({"detail": "high", "cutaways": ["cv_cut", "tripod_cut"]})
    W.drive(track, {"cutaway": "cv_cut", "boot_opacity": 1.0})

Frame / origin
--------------
``W.root`` (Empty ``whl_root``) sits at the car origin (ground point under the front-axle
centre) with no rotation, so root-local = car frame.  Corners: ``RL``, ``RR`` (rear, driven),
``FL``, ``FR`` (front, steered).  Right-side geometry is built once; the left side is its
mirror image (x -> -x for corner parts, local y -> -y for spinning parts), so every
spinning part still has local +Y = world +X and is baked with ``rotation_euler[1] = -theta``
(ARCHITECTURE.md section 2: + = forward rolling = right-handed about -X).  All objects live
in the collection ``wheels`` (opts['collection']).

Diff interface (axle assembly)
------------------------------
opts['inner_joint'] = 'flange' (default): the tripod housing's base face is bolted to the
axle's 6-bolt output flange whose outer face is at |x| = X_DIFF_OUTPUT (axl_stub_*), with a
0.1 mm gap and a 19 mm centring spigot in the flange's 20 mm recess.  The tripod centre is
then at |x| = 0.1771 and the joint-centre spacing is 0.478 m (plunge 3.78 mm and joint
angle 7.2 deg at +-60 mm).  'spec': tripod centre at |x| = X_DIFF_OUTPUT as the spec.py
comment / FACTS CVJ-01 say (spacing 0.505 m, plunge 3.58 mm, 6.8 deg); the tulip face is
then at |x| = 0.123 and overlaps the axle's output flange unless the axle ends its stubs
there.  meta['diff_interface'] publishes the face position.

Motion (all from the Track)
---------------------------
* Wheels, tyres, hubs, discs: ``theta_<c>`` about the wheel axis (transverse pivots).
* Rear (RL/RR): upright, hub, brake and outer CV joint move straight up/down by
  ``susp_<c>`` (wheel axis stays parallel to the diff outputs, FACTS CVJ-02).  Driveshaft:
  rigid, length L (above) between the tripod centre S and the Rzeppa centre O; O follows the
  hub; S slides along the diff-output axis by the plunge p = L - sqrt(L^2 - s^2) >= 0
  (outboard on bump AND droop, twice per bounce, CVJ-06); shaft angle alpha = asin(s/L).
  Equal joint angles (Z arrangement), so tulip, spider, rollers, shaft, inner race, cage and
  outer race all turn by ``theta_<c>`` (CVJ-07: shaft rpm = wheel rpm = side-gear rpm).
  Exact CV kinematics: shaft frame = (rotation about world Y by alpha) o wheel frame (the
  mirror symmetry of a CV joint through its bisecting plane); the cage is tilted by alpha/2
  (it lies in the bisecting plane); every ball centre is computed per frame as the
  intersection of its outer-race groove (Birfield offset-centre arc, track offset
  ``RZ_OFFSET`` toward the mouth; inner-race grooves offset the other way) with the
  bisecting plane, so it lies on BOTH groove centre lines and in the bisecting plane at
  every frame (tools/test_wheels.py checks it numerically).  Tripod: spider centre on the
  tulip axis (its ~0.1 mm third-order orbit is ignored); the spherical rollers ride on the
  trunnions and slide axially in the tracks (+-r sin(alpha) per turn plus the plunge).
  Rollers are bodies of revolution about their trunnions, so their roll is not baked.
* Rear links (upper camber link, front toe link, lower wishbone): inner pivots on the
  subframe, outer joints on the upright.  The Track asks for a purely vertical wheel path,
  which rigid links cannot give exactly (they would swing the wheel on arcs), so each link
  is aimed at its outer joint every frame and stretched along its length (<= 2.5 % at
  +-60 mm, invisible; documented simplification).  Coil-over (damper + spring) on an
  upright bracket behind the axle: vertical; the spring is rebuilt by shape keys ``bump`` /
  ``droop`` (value = +-s / SUSPENSION_TRAVEL; end coils stay closed, the wire stays round).
* Front (FL/FR): MacPherson strut with exact rigid kinematics: the L-shaped lower arm turns
  about its bush axis by gamma(s) (solved so the wheel centre rises by ``susp_<c>``); the
  steering axis runs from the top mount T (fixed; kingpin inclination ``KPI`` = 14 deg, no
  caster) to the lower ball joint B; the knuckle turns about it by the angle that gives the
  wheel the Track's ``steer_<c>`` as its actual heading, and slides along it as the strut
  compresses (spring shape keys ``bump`` / ``droop``, value = compression / 75 mm).  Tie
  rods keep their length: the rack-end ball joint is solved on the rack axis every frame
  (rack bar hidden in its housing; bellows scale with the rack travel).  The KPI lowers the
  wheel centre ~3.5 mm at full lock (a real car's body rises instead).
* Boots are axisymmetric, so they are not spun (invisible); they bend with the joint angle
  through shape keys ``bend`` (alpha > 0) / ``bend_neg`` (alpha < 0), value = |alpha| /
  BOOT_BEND_REF, and the inner boot also stretches with ``plunge`` (= p / BOOT_PLUNGE_REF).
  Their clamps turn with the shaft / bell / tulip (the crimp ear shows the rotation).

Parts (``W.parts`` key -> object ``whl_<key>``; c in RL, RR, FL, FR; r in RL, RR; f in FL, FR)
--------------------------------------------------------------------------------------------
spinning:  tire_<c>, wheel_<c> (alloy + 5 lug nuts + centre cap + valve), hub_<c> (flange,
           studs, bearing inner race, axle nut (rear) / dust cap (front)), brake_disc_<c>,
           outer_race_<r> (Rzeppa bell + stub axle), cage_<r>, inner_race_<r>,
           ball_<r>_0..5 (located per frame), shaft_<r> (bar + splined ends), spider_<r>,
           rollers_<r> (3 spherical rollers + needle bearings), tulip_<r> (tripod housing),
           clamp_ob_<r>, clamp_os_<r>, clamp_ib_<r>, clamp_is_<r> (boot clamps: outer /
           inner boot, big / small end)
boots:     boot_outer_<r>, boot_inner_<r>
corner:    caliper_<c> (painted cast housing, guide pins + boots, bleed nipple, hose stub),
           caliper_carrier_<c>, pads_<c>, backing_plate_<c> (dust shield), bearing_<c>
           (outer ring + seals)
rear:      upright_<r>, upper_link_<r>, toe_link_<r>, lower_arm_<r>, damper_<r> (body + lower
           spring seat), damper_rod_<r> (rod, bump stop, upper seat, top mount), spring_<r>,
           subframe_rear
front:     knuckle_<f>, strut_<f> (tube + lower seat), strut_top_<f> (rod, bump stop, upper
           seat, top bearing), top_mount_<f> (body plate), spring_<f>, lower_arm_<f>,
           tie_rod_<f> (tie rod + both ball joints), rack_joint_<f> (inner ball-joint socket),
           rack_boot_<f>, rack (housing + pinion housing), subframe_front
cutaway pieces (opts['cutaways']): ``<part>__<variant>_kept`` / ``_removed`` for
   'cv_cut':     outer_race_<r>, clamp_ob_<r>, boot_outer_<r>
   'tripod_cut': tulip_<r>, clamp_ib_<r>, boot_inner_<r>
   The spinning pieces are cut in their own frame (the opening turns with the part, as on
   a motorised training cutaway: pick slowmo so it stays in view); the boot pieces are cut
   in a fixed plane.  At theta = 0 every cut removes the half on the ``cut_normal`` side
   (default world -Y = rear, for s07's views from behind).
Motion frames (Empties) are in meta['frames']: corner_<r> (rear corner, keyed z),
hubpiv_<c>, bellpiv_<r>, irpiv_<r> / cagepiv_<r> (inner race / cage frames at O),
shaftpiv_<r> (shaft + spider frame at S), tulippiv_<r>, knuckle_frame_<f>,
strut_top_frame_<f>, lower_arm_frame_<c>, upper_link_frame_<r>, toe_link_frame_<r>,
tie_rod_frame_<f>, rack_joint_frame_<f>, rack_boot_frame_<f>.

Anchors (object, local offset; all follow their part)
   driveshaft_left / driveshaft_right (mid-shaft, top surface), inner_joint_left/right,
   outer_joint_left/right, balls_right, cage_right, inner_race_right, outer_race_right,
   tripod_rollers_right, plunge_right (spider centre), boot_outer_right, boot_inner_right,
   wheel_<c>, tire_<c>, brake_disc_<c>, hub_<c>, caliper_<c>, upright_<r>, spring_<r>,
   damper_<r>, upper_link_<r>, toe_link_<r>, lower_arm_<c>, strut_<f>, knuckle_<f>,
   tie_rod_<f>

Opts
   detail       'high' (default) | 'low'
   cutaways     list of 'cv_cut' | 'tripod_cut' (default [])
   cut_normal   world direction of the removed half at theta = 0 (default (0, -1, 0))
   inner_joint  'flange' (default) | 'spec'   (see "Diff interface")
   corners      subset of ('RL', 'RR', 'FL', 'FR') (default all)
   suspension   build links / springs / subframes / steering (default True)
   collection   collection name (default 'wheels')

Presentation keys for ``drive`` (all optional)
   explode       0..1, scalar or per-frame array (Assembly.bake_explode)
   boot_opacity  0..1 cv_opacity (+ visibility) of boots and their clamps
   cutaway       'none' | 'cv_cut' | 'tripod_cut' | 'both' (scalar or per-frame list): bakes
                 CONSTANT visibility of the whole parts vs their kept / removed pieces
                 (without the key, cut pieces stay hidden)
   removed       0..1 cv_opacity of the removed cutaway pieces while a cut is shown
                 (default 0 = hidden): fade/keep the removed half
   Scenes fade other groups themselves with rig.bake_fade(meta['groups'][name], ...), e.g.
   'coilover' (it stands between a camera behind the car and the outer joint).

explode (offsets in each part's parent frame, factor 1): wheel_<c>, tire_<c> 0.30 m
   outboard; brake_disc_<c> 0.16 m outboard; caliper_<c>, caliper_carrier_<c>, pads_<c>
   0.12 m radially out from the disc (up/back).

meta
   power_path     [tulip, spider, rollers, shaft, inner race, balls, cage, outer race, hub,
                   disc, wheel, tyre] for RL then RR
   power_groups   {'shafts': [both driveshafts incl. joints, balls, boots, clamps],
                   'rear_wheels': [rear hubs, discs, wheels, tyres]}
   groups         name -> [objects]; group_names name -> [part keys]: wheels, rear_wheels,
                  front_wheels, wheel_<c>, hubs, brakes, brakes_<c>, shafts,
                  driveshaft_<r>, balls_<r>, boots, suspension, suspension_<c>, coilover,
                  steering, subframes
   cutaway_pieces {variant: {'kept': [...], 'removed': [...], 'replaces': [...]}}
   frames, spin (part -> Track key), corners, driveshaft (L, x_inner, inner_joint,
   x_outer, ball / roller / race radii, track offset), diff_interface, kinematics
   (rear_kin, ball_centres, front_kin), dims, cut_normal, triangles, triangles_total,
   build_time, build_log

Typical values used (not in spec.py; compact RWD saloon class)
   Wheel 16 x 7J ET40, 5 x 120 PCD, 72.6 mm centre bore, 5 twin-spoke cast alloy, M14 acorn
   nuts.  Tyre 205/55 R16 (64 directional tread pitches, 4 circumferential grooves 7.5 mm
   deep, V-shaped shoulder grooves, notches and sipes, rim protector).  Ventilated discs
   300 x 20 mm (36 vanes, machined friction faces), single-piston (38 mm) floating calipers
   behind the axle line, 1.2 mm dust shields.  Hub bearing double-row unit 80 mm OD.
   Rzeppa (Birfield) joint: 6 x 17 mm balls on a 61 mm pitch circle, track offset 3.5 mm,
   bell 91 mm OD; tripod: 3 x 29 mm spherical rollers on 17 mm trunnions (needle bearings)
   on a 54 mm circle, tulip 97 mm OD; 25 mm bar with 27-tooth splined ends.  Rear: upper
   camber link, front toe link (with adjuster), lower wishbone with ball joint, coil-over
   (spring 107 mm OD, 11 mm wire, 5 active coils) on an upright bracket behind the axle,
   subframe with side members, cross tubes and an upper-link tower.  Front: MacPherson
   strut (45 mm tube, 150 mm spring), KPI 14 deg, scrub ~18 mm, L-shaped lower arm, rack
   ahead of the axle under the sump's shallow front part (steering-arm geometry is within
   12 mm of ideal Ackermann rack travel at R = 5 m; the difference is hidden in the rack
   housing).  Calipers: silver-grey painted (local material ``whl_caliper_paint`` on the
   shared CV_Presentation group).
"""
from __future__ import annotations

import math
import time

import bpy  # noqa: I001  (bpy before bmesh)
import bmesh
import numpy as np
from mathutils import Matrix, Vector

from .. import meshutil as MU
from .. import rig
from .. import spec as S

try:
    from .. import materials as MAT
except Exception:  # pragma: no cover
    MAT = None
try:
    from .. import gears as G
except Exception:  # pragma: no cover
    G = None

PREFIX = "whl_"
MM = 1e-3
DEG = math.pi / 180.0
TAU = 2.0 * math.pi
PI = math.pi

# ===========================================================================
# Layout (right side; left = mirror)
# ===========================================================================
XW_R = S.TRACK_REAR / 2.0          # rear wheel centre plane x (0.740)
XW_F = S.TRACK_FRONT / 2.0         # front (0.735)
ZW = S.WHEEL_CENTER_Z
Y_RA = S.Y_REAR_AXLE
Y_FA = S.Y_FRONT_AXLE
CORNERS = ("RL", "RR", "FL", "FR")


def _side(c):
    return -1.0 if c.endswith("L") else 1.0


def _is_rear(c):
    return c.startswith("R")


def _w0(c):
    sx = _side(c)
    return Vector((sx * (XW_R if _is_rear(c) else XW_F), Y_RA if _is_rear(c) else Y_FA, ZW))


# ---- wheel / tyre ---------------------------------------------------------
TYRE_R = S.TYRE_MESH_RADIUS        # 0.3115 modelled tread radius
R_SEAT = S.RIM_DIAMETER / 2.0      # 0.2032 bead seat radius
RIM_HW = 0.5 * 7.0 * 25.4 * MM     # 7J: 88.9 mm from the centre plane to each flange face
ET = 0.040                          # wheel offset: mounting face 40 mm outboard of the centre plane
PCD_R = 0.060                      # 5 x 120
N_LUGS = 5
CB_R = 0.0363                      # centre bore 72.6 mm
PAD_FACE = 0.062                   # front face of the wheel's centre pad (local y)
TREAD_D = 0.0075                   # groove depth

# ---- hub / disc / caliper (local axial coordinate a = outboard offset from the centre plane)
HUB_FLANGE = (0.024, 0.034)        # hub flange back / face
HAT = (0.034, 0.040)               # disc hat flange (clamped between hub and wheel)
DISC_RO, DISC_RI = 0.150, 0.096    # disc outer / inner radius of the friction ring
DISC_A = (-0.016, 0.004)           # friction ring inboard / outboard face (20 mm)
DISC_VENT = (-0.0095, -0.0025)     # vent gap
SPIGOT_R = 0.0360
BRG_A = (-0.052, -0.018)           # hub bearing axial extent
BRG_R = (0.0262, 0.040)            # bearing bore (hub barrel) / OD
SHIELD_A = (-0.0310, -0.0298)      # dust shield (backing plate)
PAD_GAP = 0.0002                   # pad <-> disc running clearance
PAD_FRIC = 0.010
PAD_BACK = 0.005
CAL_PHI = 12.0 * DEG               # caliper centre: 12 deg above the horizontal, behind the axle

# ---- driveshaft (rear) ------------------------------------------------------
TP_FACE = -0.027                   # tulip base face (bolted to the diff output flange), from the joint centre
X_OJ = S.X_WHEEL_HUB               # Rzeppa (outer joint) centre
X_IJ_SPEC = S.X_DIFF_OUTPUT        # 'spec': tripod centre at X_DIFF_OUTPUT (spec.py comment, FACTS CVJ-01)
X_IJ_FLANGE = S.X_DIFF_OUTPUT - TP_FACE + 0.0001   # 'flange' (default): tulip face 0.1 mm off the axle's output flange
X_IJ = X_IJ_FLANGE                 # default tripod (inner joint) centre at rest
SHAFT_L = X_OJ - X_IJ              # default joint-centre spacing (0.478 m; 0.505 m with 'spec')
SHAFT_R = S.HALFSHAFT_D / 2.0
OJ_A = X_OJ - XW_R                 # -0.085: outer joint centre relative to the wheel centre plane
# Rzeppa (Birfield) joint
RZ_RB = 0.0305                     # ball-centre radius (zero angle)
RZ_BALL = 0.0085                   # ball radius (17 mm)
RZ_OFFSET = 0.0035                 # track offset of the groove centres (each side of the centre)
RZ_RG = math.hypot(RZ_RB, RZ_OFFSET)   # groove centre-line arc radius
RZ_GROOVE = RZ_BALL + 0.00015      # groove (tube) radius: 0.15 mm radial clearance
RZ_IR_SPH = 0.0275                 # inner race outer sphere
RZ_CAGE_I, RZ_CAGE_O = 0.0279, 0.0331   # cage spherical shell
RZ_OR_SPH = 0.0335                 # outer race inner sphere
RZ_OR_OUT = 0.0455                 # bell outer radius
RZ_IR_W = 0.011                    # inner race half width
RZ_CAGE_W = 0.0125                 # cage half width
RZ_WIN_HALF = RZ_BALL + 0.0001     # cage window half-width (axial)
RZ_WIN_T = 0.0102                  # cage window half-length (circumferential, at the ball circle)
RZ_MOUTH = -0.028                  # bell mouth (local axial, inboard is negative on the right)
RZ_BASE = 0.022                    # bell closed end (inner face) ... outer base at +0.030
# Tripod joint
TP_RT = 0.027                      # roller-centre radius
TP_RR = 0.0145                     # roller spherical outer radius (29 mm)
TP_RW = 0.0065                     # roller half width (along the trunnion)
TP_TRUN = 0.0085                   # trunnion radius
TP_RBORE = 0.0100                  # roller bore (needles 1.5 mm between)
TP_TRACK = TP_RR + 0.0002          # track side-wall radius
TP_SLOT = 0.0012                   # radial freedom of a roller in its track (each way)
TP_HUB_R = 0.0190                  # spider hub radius
TP_CAV_R = 0.0215                  # central cavity radius
TP_OUT = 0.0485                    # tulip outer radius
TP_FLOOR = -0.0205                 # cavity floor (local, from the joint centre at rest)
TP_MOUTH = 0.030
TP_SPIGOT = (0.0095, 0.0030)       # centring spigot (radius, length) into the output flange's 20 mm recess

BOOT_BEND_REF = 0.125              # rad (shape keys 'bend' / 'bend_neg' at value 1)
BOOT_PLUNGE_REF = 0.006            # m   (shape key 'plunge' at value 1)

# ---- rear suspension hardpoints (car frame, right side) -------------------
R_UPPER_IN = Vector((0.330, -2.540, 0.470))
R_UPPER_OUT = Vector((0.600, -2.600, 0.480))
R_TOE_IN = Vector((0.320, -2.470, 0.240))
R_TOE_OUT = Vector((0.650, -2.470, 0.245))
R_LOW_IN_F = Vector((0.300, -2.470, 0.150))
R_LOW_IN_R = Vector((0.300, -2.770, 0.150))
R_LOW_OUT = Vector((0.660, -2.620, 0.145))
R_DAMP_LOW = Vector((0.555, -2.790, 0.215))   # damper lower eye (on the upright bracket)
R_DAMP_TOP = Vector((0.555, -2.790, 0.552))   # top mount (under the boot floor)
R_SPRING = (0.315, 0.528)                      # spring seats z (lower on the damper body, upper)

# ---- front suspension (car frame, right side) ------------------------------
KPI = 14.0 * DEG
F_B = Vector((0.680, 0.000, 0.150))            # lower ball joint
F_AXIS = Vector((-math.sin(KPI), 0.0, math.cos(KPI)))   # steering / strut axis (up)
F_L0 = 0.720                                   # B -> T
F_T = F_B + F_AXIS * F_L0                      # top mount
F_ARM_F = Vector((0.330, 0.030, 0.165))        # lower arm front bush
F_ARM_R = Vector((0.360, -0.300, 0.165))       # lower arm rear bush
F_TIE_OUT = Vector((0.680, 0.125, 0.215))      # outer tie-rod ball joint (on the steering arm)
F_RACK_Y, F_RACK_Z = 0.135, 0.205              # rack axis
F_RACK_HALF = 0.200                            # rack housing half length
F_TIE_IN0 = Vector((0.335, F_RACK_Y, F_RACK_Z))  # inner ball joint at rest
F_STRUT_L = (0.225, 0.495)                     # strut tube along the axis (from B)
F_SPRING_L = (0.497, 0.690)                    # spring seats along the axis (lower, upper)


# ===========================================================================
# Small helpers
# ===========================================================================
_COL = None


def _nm(key):
    return PREFIX + key


def _local_materials():
    """Materials not in the shared table, built on the shared CV_Presentation group."""
    name = "whl_caliper_paint"
    m = bpy.data.materials.get(name)
    if m is not None and m.get("cv_version") == 1:
        return m
    if m is None:
        m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    bsdf = nt.nodes.new("ShaderNodeBsdfPrincipled")
    # silver-grey painted cast iron (zinc-flake / silver caliper paint), light clear coat
    bsdf.inputs["Base Color"].default_value = (0.36, 0.37, 0.38, 1.0)
    bsdf.inputs["Metallic"].default_value = 0.65
    bsdf.inputs["Roughness"].default_value = 0.42
    try:
        bsdf.inputs["Coat Weight"].default_value = 0.35
        bsdf.inputs["Coat Roughness"].default_value = 0.12
    except Exception:
        pass
    shader = bsdf.outputs["BSDF"]
    try:
        grp = nt.nodes.new("ShaderNodeGroup")
        grp.node_tree = MAT.presentation_group()
        nt.links.new(shader, grp.inputs["Shader"])
        grp.inputs["Glow Tint"].default_value = (1.0, 1.0, 1.0, 1.0)
        shader = grp.outputs["Shader"]
    except Exception:
        pass
    nt.links.new(shader, out.inputs["Surface"])
    m.diffuse_color = (0.42, 0.43, 0.44, 1.0)
    m.metallic = 0.6
    m.roughness = 0.4
    m["cv_version"] = 1
    m["cv_material"] = name
    return m


def _mat(name):
    if name.startswith("whl_"):
        return _local_materials()
    if MAT is not None:
        try:
            return MAT.get(name)
        except KeyError:
            pass
    return MU.get_material(name)


def _link(ob):
    if _COL is not None and ob.name not in _COL.objects:
        for c in list(ob.users_collection):
            c.objects.unlink(ob)
        _COL.objects.link(ob)
    return ob


def _mb_obj(mb, key, mats, smooth=35.0, fix=True):
    ob = mb.to_object(_nm(key), _COL, smooth_angle=smooth, fix_normals=fix)
    for m in mats:
        ob.data.materials.append(_mat(m))
    return ob


def _lathe(key, prof, seg, mat, closed=True, phase=0.0, smooth=35.0, caps=True):
    ob = MU.lathe(_nm(key), prof, segments=seg, closed=closed, caps=caps, collection=_COL,
                  smooth_angle=smooth, phase=phase)
    MU.assign_material(ob, mat)
    return ob


def _ring(key, r_in, r_out, y0, y1, seg, mat, ch=0.0004):
    """Annulus (hollow cylinder) along local Y with small chamfers."""
    c = min(ch, 0.3 * (r_out - r_in), 0.3 * (y1 - y0))
    prof = [(r_in + c, y0), (r_out - c, y0), (r_out, y0 + c), (r_out, y1 - c), (r_out - c, y1),
            (r_in + c, y1), (r_in, y1 - c), (r_in, y0 + c)]
    return _lathe(key, prof, seg, mat, closed=True)


def _disc(key, r, y0, y1, seg, mat, ch=0.0004):
    c = min(ch, 0.3 * r, 0.3 * (y1 - y0))
    prof = [(0.0, y0), (r - c, y0), (r, y0 + c), (r, y1 - c), (r - c, y1), (0.0, y1)]
    return _lathe(key, prof, seg, mat, closed=False)


def _xf(ob, M):
    ob.data.transform(M)
    ob.data.update()
    return ob


def _mirror_mesh(me, axis):
    """Mirror mesh data along local axis (0=x, 1=y) keeping outward normals."""
    s = [1.0, 1.0, 1.0]
    s[axis] = -1.0
    me.transform(Matrix.Diagonal((s[0], s[1], s[2], 1.0)), shape_keys=True)
    me.flip_normals()
    me.update()
    return me


def _copy_obj(src, key, mirror_axis=None):
    me = src.data.copy()
    me.name = _nm(key)
    if mirror_axis is not None:
        _mirror_mesh(me, mirror_axis)
    ob = bpy.data.objects.new(_nm(key), me)
    _COL.objects.link(ob)
    for k in src.keys():
        if k not in ("cv_opacity", "cv_glow"):
            ob[k] = src[k]
    return ob


def _bool(target, operands, op="DIFFERENCE"):
    """Manifold boolean (applied); operands deleted.  Operand materials transfer."""
    ops = [o for o in operands if o is not None]
    if not ops:
        return target
    tmp = bpy.data.collections.new("whl_bool_tmp")
    bpy.context.scene.collection.children.link(tmp)
    for o in ops:
        for c in list(o.users_collection):
            c.objects.unlink(o)
        tmp.objects.link(o)
    if not target.users_collection:
        _COL.objects.link(target)
    mod = target.modifiers.new("whl_bool", "BOOLEAN")
    mod.operation = op
    mod.operand_type = "COLLECTION"
    mod.collection = tmp
    mod.material_mode = "TRANSFER"
    me = None
    for solver in ("MANIFOLD", "EXACT"):
        try:
            mod.solver = solver
        except TypeError:
            continue
        dg = bpy.context.evaluated_depsgraph_get()
        dg.update()
        cand = bpy.data.meshes.new_from_object(target.evaluated_get(dg))
        if len(cand.polygons) > 0:
            me = cand
            break
        bpy.data.meshes.remove(cand)
    target.modifiers.remove(mod)
    if me is not None:
        old = target.data
        target.data = me
        if old.users == 0:
            bpy.data.meshes.remove(old)
    for o in ops:
        d = o.data
        bpy.data.objects.remove(o)
        if d is not None and d.users == 0:
            bpy.data.meshes.remove(d)
    bpy.data.collections.remove(tmp)
    return target


def _join(key, objs, smooth=None):
    objs = [o for o in objs if o is not None]
    ob = MU.join(_nm(key) + "_tmpjoin", objs, collection=_COL, smooth_angle=smooth)
    ob.name = _nm(key)
    ob.data.name = _nm(key)
    return ob


def _toX(ob):
    """Rotate mesh data so local +Y -> +X (round parts built along Y, used along X)."""
    return _xf(ob, Matrix.Rotation(-PI / 2, 4, "Z"))


def _place(ob, loc=(0, 0, 0), rot=None):
    M = Matrix.Translation(Vector(loc))
    if rot is not None:
        M = M @ rot
    return _xf(ob, M)


def _frame(z_axis, x_hint=(0.0, 0.0, 1.0)):
    """3x3 rotation whose local +Y = y_axis; returns Matrix with columns X, Y, Z."""
    y = Vector(z_axis).normalized()
    xh = Vector(x_hint)
    x = xh - y * xh.dot(y)
    if x.length < 1e-6:
        xh = Vector((1.0, 0.0, 0.0)) if abs(y.x) < 0.9 else Vector((0.0, 0.0, 1.0))
        x = xh - y * xh.dot(y)
    x.normalize()
    z = x.cross(y)
    return Matrix((x, y, z)).transposed()


def _align_y(p0, p1, x_hint=(0.0, 0.0, 1.0)):
    """4x4: local origin at p0, local +Y toward p1."""
    p0, p1 = Vector(p0), Vector(p1)
    R = _frame(p1 - p0, x_hint).to_4x4()
    return Matrix.Translation(p0) @ R


def _beam(key, pts, radii, mat, seg=14, flat=1.0, x_hint=(0.0, 0.0, 1.0), caps=True):
    """Lofted round/elliptic beam through points with per-point radii (closed)."""
    P = [Vector(p) for p in pts]
    n = len(P)
    rs = list(radii) if np.ndim(radii) else [radii] * n
    mb = MU.MeshBuilder()
    rings = []
    a = np.arange(seg) * TAU / seg
    for i in range(n):
        t = (P[min(i + 1, n - 1)] - P[max(i - 1, 0)]).normalized()
        R = _frame(t, x_hint)
        xv, zv = R.col[0], R.col[2]
        pts_i = [P[i] + xv * (rs[i] * math.cos(ai)) + zv * (rs[i] * flat * math.sin(ai)) for ai in a]
        rings.append(mb.verts([tuple(p) for p in pts_i]))
    for i in range(n - 1):
        mb.bridge(rings[i], rings[i + 1])
    if caps:
        c0 = mb.verts([tuple(P[0])])[0]
        c1 = mb.verts([tuple(P[-1])])[0]
        mb.fan(c0, rings[0][::-1])
        mb.fan(c1, rings[-1])
    return _mb_obj(mb, key, [mat], smooth=50.0)


def _sphere(key, r, seg=24, rings=12, mat="steel_ground", center=(0, 0, 0)):
    prof = [(0.0, -r)] + [(r * math.sin(PI * k / rings), -r * math.cos(PI * k / rings)) for k in range(1, rings)] \
        + [(0.0, r)]
    ob = _lathe(key, prof, seg, mat, closed=False, smooth=80.0)
    if any(center):
        _place(ob, center)
    return ob


def _box(key, size, center=(0, 0, 0), r=0.002, mat="paint_black", seg=2):
    ob = MU.rounded_box(_nm(key), size, radius=r, segments=seg, center=center, collection=_COL,
                        smooth_angle=35.0)
    MU.assign_material(ob, mat)
    return ob


def _cyl_between(key, p0, p1, r, mat, seg=16, ch=0.0005, x_hint=(0, 0, 1)):
    """Solid cylinder from p0 to p1."""
    L = (Vector(p1) - Vector(p0)).length
    ob = MU.cylinder(_nm(key), r, 0.0, L, segments=seg, chamfer=ch, collection=_COL, smooth_angle=35.0)
    MU.assign_material(ob, mat)
    return _xf(ob, _align_y(p0, p1, x_hint))


def _chaikin(poly, it=2, closed=True):
    P = np.asarray(poly, dtype=float)
    for _ in range(it):
        Q = []
        n = len(P)
        rng = range(n) if closed else range(n - 1)
        if not closed:
            Q.append(P[0])
        for i in rng:
            a, b = P[i], P[(i + 1) % n]
            Q.append(0.75 * a + 0.25 * b)
            Q.append(0.25 * a + 0.75 * b)
        if not closed:
            Q.append(P[-1])
        P = np.array(Q)
    return P


def _loft(key, sections, mat, closed_caps=True, smooth=40.0):
    """sections: list of (M,3) arrays (same M), lofted + capped."""
    mb = MU.MeshBuilder()
    rings = [mb.verts(np.asarray(s)) for s in sections]
    for a, b in zip(rings[:-1], rings[1:]):
        mb.bridge(a, b)
    if closed_caps:
        for rg, flip in ((rings[0], True), (rings[-1], False)):
            c = mb.verts([np.asarray(sections[0 if flip else -1]).mean(axis=0)])[0]
            mb.fan(c, rg[::-1] if flip else rg)
    return _mb_obj(mb, key, [mat], smooth=smooth)


def _seg(n, detail):
    return max(12, int(n if detail == "high" else n * 0.5) // 4 * 4)


def _helix_vf(r_mean, wire, z0, z1, coils, seg_wire=10, per_turn=48, end_flat=0.75):
    """Coil-spring surface (closed): centre-line helix along local +Y from y=z0 to y=z1,
    ground/closed end coils (pitch = wire) and uniform active coils.  Returns (V, F)."""
    turns = coils + 2 * end_flat
    n = int(turns * per_turn) + 1
    t = np.linspace(0.0, turns, n)
    L = z1 - z0
    act = max(coils, 1e-3)
    p_act = (L - wire - 2 * end_flat * wire) / act
    y = np.where(t < end_flat, wire * t,
                 np.where(t < end_flat + act, wire * end_flat + p_act * (t - end_flat),
                          wire * end_flat + p_act * act + wire * (t - end_flat - act)))
    ys = z0 + wire / 2 + y
    ang = TAU * t
    C = np.stack([r_mean * np.cos(ang), ys, r_mean * np.sin(ang)], axis=1)
    a = np.arange(seg_wire) * TAU / seg_wire
    V = []
    for i in range(n):
        tang = C[min(i + 1, n - 1)] - C[max(i - 1, 0)]
        tang /= np.linalg.norm(tang)
        radial = np.array([math.cos(ang[i]), 0.0, math.sin(ang[i])])
        nrm = radial - tang * radial.dot(tang)
        nrm /= np.linalg.norm(nrm)
        b = np.cross(tang, nrm)
        V.append(C[i] + wire / 2 * (np.cos(a)[:, None] * nrm + np.sin(a)[:, None] * b))
    V = np.concatenate(V + [C[:1], C[-1:]], axis=0)
    F = []
    m = seg_wire
    for i in range(n - 1):
        for k in range(m):
            F.append((i * m + k, i * m + (k + 1) % m, (i + 1) * m + (k + 1) % m, (i + 1) * m + k))
    c0, c1 = n * m, n * m + 1
    for k in range(m):
        F.append((c0, (k + 1) % m, k))
        F.append((c1, (n - 1) * m + k, (n - 1) * m + (k + 1) % m))
    return V, F


def _spring(key, r_mean, wire, y_fixed, y_moving, coils, travel_bump, travel_droop, mat="steel_dark",
            detail="high", M=None):
    """Spring between a fixed seat (y_fixed) and a moving seat (y_moving) along local Y, with
    shape keys 'bump' (moving end travels |travel_bump| toward the fixed end) and 'droop'
    (away by travel_droop), rebuilt helices (end coils stay closed, wire stays round)."""
    sw, pt = (10, 48) if detail == "high" else (8, 28)
    lo, hi = min(y_fixed, y_moving), max(y_fixed, y_moving)
    sgn = 1.0 if y_moving < y_fixed else -1.0        # +1: moving end is the low end
    V, F = _helix_vf(r_mean, wire, lo, hi, coils, sw, pt)
    mb = MU.MeshBuilder()
    mb.verts(V)
    mb.faces(np.array([f for f in F if len(f) == 4], dtype=np.int64))
    mb.faces(np.array([f for f in F if len(f) == 3], dtype=np.int64))
    ob = _mb_obj(mb, key, [mat], smooth=60.0, fix=False)
    # orientation check (outward normals) - fix if needed, then re-read order is unchanged
    if MU.signed_volume(ob.data) < 0:
        ob.data.flip_normals()
    keys = []
    for name, d in (("bump", travel_bump), ("droop", -travel_droop)):
        if sgn > 0:
            Vk, _ = _helix_vf(r_mean, wire, lo + d, hi, coils, sw, pt)
        else:
            Vk, _ = _helix_vf(r_mean, wire, lo, hi - d, coils, sw, pt)
        keys.append((name, Vk))
    for name, Vk in keys:
        _add_shape_key(ob, name, Vk)
    if M is not None:
        ob.data.transform(M, shape_keys=True)
        ob.data.update()
    return ob


def _add_shape_key(ob, name, new_co, slider_min=0.0):
    if ob.data.shape_keys is None:
        ob.shape_key_add(name="Basis", from_mix=False)
    kb = ob.shape_key_add(name=name, from_mix=False)
    kb.data.foreach_set("co", np.asarray(new_co, dtype=np.float32).ravel())
    kb.slider_min = slider_min
    kb.slider_max = 1.0
    kb.value = 0.0
    return kb


def _co(ob):
    me = ob.data
    co = np.zeros(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    return co.reshape(-1, 3)


def _tris(objs):
    n = 0
    for ob in objs:
        if ob is not None and ob.type == "MESH":
            ob.data.calc_loop_triangles()
            n += len(ob.data.loop_triangles)
    return n


# ===========================================================================
# Tyre 205/55 R16  (local frame: +Y = axis, outboard of the right wheel; symmetric)
# ===========================================================================
_CROWN_DX = 0.084
_CROWN_DROP = 0.0040
_SH_R = 0.018
_TREAD_A_END = 20.0 * DEG         # tread grid ends 20 deg down the shoulder arc
_SIDEWALL = [(0.1028, 0.2810), (0.1050, 0.2730), (0.1063, 0.2650), (0.1068, 0.2580), (0.1064, 0.2510),
             (0.1053, 0.2450), (0.1057, 0.2432), (0.1056, 0.2414), (0.1040, 0.2398), (0.1018, 0.2350),
             (0.1002, 0.2320), (0.1012, 0.2300), (0.1019, 0.2276), (0.1011, 0.2253), (0.0986, 0.2240),
             (0.0950, 0.2234), (0.0921, 0.2225), (0.0897, 0.2204), (0.0885, 0.2180), (0.0885, 0.2060),
             (0.0869, 0.2036), (0.0700, 0.2035), (0.0686, 0.2048), (0.0670, 0.2085), (0.0600, 0.2100)]


def _crown_r(dx):
    return TYRE_R - _CROWN_DROP * (np.asarray(dx) / 0.085) ** 2


def _shoulder():
    x1 = _CROWN_DX
    r1 = float(_crown_r(x1))
    slope = -2 * _CROWN_DROP * x1 / 0.085 ** 2
    n = np.array([-slope, 1.0])
    n /= np.linalg.norm(n)
    c = np.array([x1, r1]) - _SH_R * n
    return c, math.atan2(n[1], n[0])


def _tread_curve():
    """Dense (dx, r) curve from the crown centre over the shoulder to _TREAD_A_END + arclength."""
    c, a1 = _shoulder()
    xs = np.linspace(0.0, _CROWN_DX, 300)
    P1 = np.stack([xs, _crown_r(xs)], 1)
    angs = np.linspace(a1, _TREAD_A_END, 200)[1:]
    P2 = c[None, :] + _SH_R * np.stack([np.cos(angs), np.sin(angs)], 1)
    P = np.concatenate([P1, P2])
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    return P, s


def tyre_profile_outer():
    """Full outer half-profile (dx >= 0) from the crown centre to the bottom (for checks)."""
    P, _ = _tread_curve()
    c, _ = _shoulder()
    arc = [tuple(c + _SH_R * np.array([math.cos(a), math.sin(a)])) for a in (12 * DEG, 4 * DEG, -4 * DEG)]
    return [tuple(p) for p in P] + arc + list(_SIDEWALL) + [(0.0, 0.2100)]


def _stitch(mb, ia, aa, ib, ab):
    """Triangle strip between two closed rings (index arrays + angles, increasing order)."""
    ia, ib = np.asarray(ia), np.asarray(ib)
    a0 = aa[0]
    au = (np.asarray(aa) - a0) % TAU
    bu = (np.asarray(ab) - a0) % TAU
    kb = int(np.argmin(bu))
    ib = np.roll(ib, -kb)
    bu = np.roll(bu, -kb)
    na, nb = len(ia), len(ib)
    A = np.concatenate([au, [TAU]])
    B = np.concatenate([bu, [bu[0] + TAU]])
    i = j = 0
    tris = []
    while i < na or j < nb:
        if j >= nb or (i < na and A[i + 1] <= B[j + 1]):
            tris.append((ia[i % na], ia[(i + 1) % na], ib[j % nb]))
            i += 1
        else:
            tris.append((ia[i % na], ib[(j + 1) % nb], ib[j % nb]))
            j += 1
    mb.faces(np.array(tris, dtype=np.int64))


def _tyre(key, detail):
    high = detail == "high"
    NP = 64 if high else 44
    U = [0.00, 0.12, 0.16, 0.25, 0.28, 0.40, 0.50, 0.53, 0.55, 0.585, 0.70] if high \
        else [0.00, 0.12, 0.16, 0.40, 0.70]
    nu = len(U)
    NU = NP * nu
    K_SHEAR = 1.6                                   # rad/m: V-shaped (directional) pattern
    P, sarr = _tread_curve()
    s_g = float(sarr[-1])
    s_p1 = float(sarr[299])                         # end of the crown
    c, a1 = _shoulder()

    def s_at_angle(a):
        p = c + _SH_R * np.array([math.cos(a), math.sin(a)])
        k = int(np.argmin(np.linalg.norm(P - p, axis=1)))
        return float(sarr[k])

    if high:
        s_pos = [0.0, 0.004, 0.0105, 0.0175, 0.0265, 0.030, 0.038, 0.047, 0.051, 0.059, 0.063,
                 0.0715, 0.080, s_p1] + [s_at_angle(a * DEG) for a in (62, 41)] + [s_g]
    else:
        s_pos = [0.0, 0.0105, 0.0175, 0.0265, 0.038, 0.051, 0.059, 0.0715, s_p1] + \
                [s_at_angle(a * DEG) for a in (58, 36)] + [s_g]
    s_pos = sorted(set(round(v, 7) for v in s_pos))
    s_lines = [-v for v in reversed(s_pos[1:])] + s_pos
    J = len(s_lines)

    def base(sv):
        a = abs(sv)
        x = float(np.interp(a, sarr, P[:, 0]))
        r = float(np.interp(a, sarr, P[:, 1]))
        k = int(np.clip(np.searchsorted(sarr, a), 1, len(sarr) - 1))
        t = P[k] - P[k - 1]
        t /= np.linalg.norm(t)
        n_in = np.array([t[1], -t[0]])
        if sv < 0:
            x, n_in = -x, np.array([-n_in[0], n_in[1]])
        return x, r, n_in

    bases = [base(sv) for sv in s_lines]
    taper = max(s_g - s_p1, 1e-6)
    depth_fn = {
        0: lambda s: 0.0,                                                   # tread surface
        1: lambda s: TREAD_D,                                               # circumferential groove
        2: lambda s: TREAD_D * float(np.clip((s_g - abs(s)) / taper, 0, 1)) ** 0.8,   # shoulder groove
        3: lambda s: 0.0050,                                                # notch
        4: lambda s: 0.0040,                                                # sipe
    }

    def level(sc, uc):
        a = abs(sc)
        if 0.0175 < a < 0.0265 or 0.051 < a < 0.059:
            return 1
        if a >= 0.059:
            if uc < 0.16:
                return 2
            if high and 0.55 <= uc < 0.585 and 0.063 < a < 0.080:
                return 4
            return 0
        if a >= 0.0265:
            if uc < 0.12 and a > 0.038:
                return 3
            if high and 0.50 <= uc < 0.53 and 0.030 < a < 0.047:
                return 4
            return 0
        if high and 0.25 <= uc < 0.28 and a > 0.004:
            return 4
        return 0

    def psi(i, sv):
        return TAU * (i // nu + U[i % nu]) / NP + K_SHEAR * abs(sv)

    lev = np.zeros((NU, J - 1), dtype=np.int8)
    for i in range(NU):
        u0 = U[i % nu]
        u1 = U[(i + 1) % nu] if (i + 1) % nu else 1.0
        uc = 0.5 * (u0 + u1)
        for j in range(J - 1):
            lev[i, j] = level(0.5 * (s_lines[j] + s_lines[j + 1]), uc)

    mb = MU.MeshBuilder()
    vid = {}
    pts = []

    def V(i, j, L):
        d = depth_fn[int(L)](s_lines[j])
        k = (i, j, round(d, 7))
        idx = vid.get(k)
        if idx is None:
            x, r, n = bases[j]
            dx, rr = x + n[0] * d, r + n[1] * d
            a = psi(i, s_lines[j])
            idx = len(pts)
            pts.append((rr * math.cos(a), dx, rr * math.sin(a)))
            vid[k] = idx
        return idx

    quads, ngons = [], []
    for i in range(NU):
        i1 = (i + 1) % NU
        for j in range(J - 1):
            L = lev[i, j]
            quads.append([V(i, j, L), V(i1, j, L), V(i1, j + 1, L), V(i, j + 1, L)])
    # all depths present at every grid location (for T-junction-free walls)
    depths_at = {}
    for (i, j, d), idx in vid.items():
        depths_at.setdefault((i, j), []).append((d, idx))
    for k in depths_at:
        depths_at[k].sort()

    def column(i, j, d_from, d_to):
        """vertex indices at (i, j) from depth d_from to d_to inclusive (in that order)."""
        lo, hi = min(d_from, d_to), max(d_from, d_to)
        col = [idx for (d, idx) in depths_at[(i, j)] if lo - 1e-9 <= round(d, 7) <= hi + 1e-9]
        return col if d_from <= d_to else col[::-1]

    def wall(a, b, La, Lb):
        (ia, ja), (ib, jb) = a, b
        da1, db1 = depth_fn[int(La)](s_lines[ja]), depth_fn[int(La)](s_lines[jb])
        da2, db2 = depth_fn[int(Lb)](s_lines[ja]), depth_fn[int(Lb)](s_lines[jb])
        poly = [V(ia, ja, La)] + column(ib, jb, round(db1, 7), round(db2, 7)) + \
            column(ia, ja, round(da2, 7), round(da1, 7))[:-1]
        out = []
        for v in poly:
            if not out or out[-1] != v:
                out.append(v)
        if len(out) > 1 and out[0] == out[-1]:
            out.pop()
        if len(out) == 4:
            quads.append(out)
        elif len(out) >= 3:
            ngons.append(out)

    for i in range(NU):
        i1 = (i + 1) % NU
        for j in range(J - 1):
            L = lev[i, j]
            L2 = lev[i1, j]
            if L2 != L:
                wall((i1, j), (i1, j + 1), L, L2)
            if j + 1 < J - 1:
                L3 = lev[i, j + 1]
                if L3 != L:
                    wall((i, j + 1), (i1, j + 1), L, L3)
    ring_hi = np.array([V(i, J - 1, 0) for i in range(NU)])
    ring_lo = np.array([V(i, 0, 0) for i in range(NU)])
    mb.verts(np.array(pts))
    mb.faces(np.array(quads, dtype=np.int64))
    for f in ngons:
        mb.face(f)
    ang_hi = np.array([psi(i, s_lines[-1]) for i in range(NU)])
    # sidewalls + bottom: one lathe from the +s_g boundary around the bottom to the -s_g one
    arc = [tuple(c + _SH_R * np.array([math.cos(a), math.sin(a)])) for a in (12 * DEG, 4 * DEG, -4 * DEG)]
    half = arc + list(_SIDEWALL)
    prof = [(r, x) for (x, r) in half] + [(0.2100, 0.0)] + [(r, -x) for (x, r) in reversed(half)]
    nsw = 192 if high else 120
    phase = float(ang_hi[0])
    rings = MU._revolve_into(mb, prof, nsw, closed=False, phase=phase)
    ang_sw = phase + np.arange(nsw) * TAU / nsw
    _stitch(mb, ring_hi, ang_hi, rings[0], ang_sw)
    _stitch(mb, ring_lo, ang_hi, rings[-1], ang_sw)
    ob = mb.to_object(_nm(key), _COL, smooth_angle=32.0, fix_normals=True)
    ob.data.materials.append(_mat("tire_rubber"))
    return ob


# ===========================================================================
# Wheel: 16 x 7J ET40 cast alloy, 5 twin spokes, acorn nuts, centre cap, valve
# ===========================================================================

def _arc_pts(cx, cr, rad, a0, a1, n):
    return [(cx + rad * math.cos(a), cr + rad * math.sin(a)) for a in np.linspace(a0, a1, n)]


def rim_section():
    """Closed (dx, r) cross-section of the rim (shapely-rounded)."""
    from shapely.geometry import Polygon
    pts = [(0.1000, 0.1960)]
    pts += _arc_pts(0.0944, 0.2160, 0.0056, 0.0, PI, 10)
    pts += [(0.0888, 0.2052), (0.0870, 0.2032), (0.0700, 0.2032), (0.0665, 0.2052), (0.0630, 0.2035),
            (0.0560, 0.1880), (0.0500, 0.1855), (0.0060, 0.1855), (-0.0150, 0.1990), (-0.0600, 0.2030),
            (-0.0630, 0.2035), (-0.0665, 0.2052), (-0.0700, 0.2032), (-0.0870, 0.2032), (-0.0888, 0.2052)]
    pts += _arc_pts(-0.0944, 0.2160, 0.0056, 0.0, PI, 10)
    pts += [(-0.1000, 0.2095), (-0.0960, 0.2030), (-0.0880, 0.1990), (-0.0600, 0.1985), (-0.0150, 0.1945),
            (0.0030, 0.1810), (0.0500, 0.1810), (0.0590, 0.1840), (0.0700, 0.1880), (0.0780, 0.1900),
            (0.0900, 0.1910)]
    poly = Polygon(pts)
    if not poly.is_valid:
        poly = poly.buffer(0)
    r = 0.0005
    poly = poly.buffer(r, quad_segs=2).buffer(-2 * r, quad_segs=2).buffer(r, quad_segs=2)
    poly = poly.simplify(2e-5)
    return np.array(poly.exterior.coords)[:-1]


def _spoke_sections(phi_h, phi_r, n_st, detail):
    rs = np.linspace(0.066, 0.197, n_st)
    secs = []
    for r in rs:
        t = (r - 0.066) / (0.197 - 0.066)
        phi = phi_h + (phi_r - phi_h) * t
        w = 0.0265 + 0.0055 * t
        wf = w - 0.0050
        vf = 0.0612 if r <= 0.084 else 0.0612 + 0.0373 * ((r - 0.084) / 0.113) ** 1.5
        vb = max(vf - (0.0200 - 0.0040 * t), 0.0418)
        poly = [(-w / 2, vb), (w / 2, vb), (wf / 2, vf - 0.0028), (0.28 * wf, vf - 0.0006), (0.0, vf),
                (-0.28 * wf, vf - 0.0006), (-wf / 2, vf - 0.0028)]
        q = _chaikin(poly, 2 if detail == "high" else 1)
        ca, sa = math.cos(phi), math.sin(phi)
        X = r * ca - q[:, 0] * sa
        Z = r * sa + q[:, 0] * ca
        secs.append(np.stack([X, q[:, 1], Z], 1))
    return secs


def _circle(c, r, n):
    a = np.arange(n) * TAU / n
    return np.stack([c[0] + r * np.cos(a), c[1] + r * np.sin(a)], 1)


def _lug_angles():
    return [PI / 2 + k * TAU / N_LUGS + PI / N_LUGS for k in range(N_LUGS)]


def _wheel(key, detail):
    high = detail == "high"
    parts = []
    sec = rim_section()
    rim = _lathe(key + "_rim", [(r, x) for (x, r) in sec], 160 if high else 96, "rim_alloy", closed=True,
                 smooth=30.0)
    parts.append(rim)
    # centre pad with centre bore and 5 lug pockets
    nc = 96 if high else 48
    holes = [_circle((0.0, 0.0), CB_R, 64 if high else 32)]
    for a in _lug_angles():
        holes.append(_circle((PCD_R * math.cos(a), PCD_R * math.sin(a)), 0.0135, 24 if high else 16))
    pad = MU.extrude_polygon(_nm(key + "_pad"), _circle((0, 0), 0.0838, nc), ET, PAD_FACE, holes=holes,
                             chamfer=0.0016, collection=_COL, smooth_angle=30.0)
    MU.assign_material(pad, "rim_alloy")
    parts.append(pad)
    # conical nut seats at the pocket bottoms
    seat_prof = [(0.0074, 0.0400), (0.0137, 0.0400), (0.0137, 0.0478), (0.0125, 0.0478), (0.0090, 0.0417),
                 (0.0074, 0.0417)]
    for k, a in enumerate(_lug_angles()):
        ob = _lathe(f"{key}_seat{k}", seat_prof, 24 if high else 16, "rim_alloy", closed=True)
        _place(ob, (PCD_R * math.cos(a), 0.0, PCD_R * math.sin(a)))
        parts.append(ob)
    # twin spokes
    n_st = 13 if high else 8
    for k in range(5):
        pk = PI / 2 + k * TAU / 5
        for j in (-1, 1):
            secs = _spoke_sections(pk + j * 5.0 * DEG, pk + j * 11.5 * DEG, n_st, detail)
            parts.append(_loft(f"{key}_spoke{k}{j}", secs, "rim_alloy", smooth=38.0))
    # valve stem (between two spoke pairs, through the outboard well wall)
    va = _lug_angles()[0]
    e = Vector((math.cos(va), 0.0, math.sin(va)))
    p0 = e * 0.1870 + Vector((0.0, 0.0560, 0.0))
    p1 = e * 0.1735 + Vector((0.0, 0.0790, 0.0))
    stem = _cyl_between(key + "_valve", p0, p1, 0.0050, "rubber", seg=12, ch=0.0008)
    capv = _cyl_between(key + "_valvecap", p1, p1 + (p1 - p0).normalized() * 0.010, 0.0042, "plastic_black",
                        seg=12, ch=0.0008)
    parts += [stem, capv]
    # acorn lug nuts (M14, 19 mm hex)
    for k, a in enumerate(_lug_angles()):
        cone = _lathe(f"{key}_nc{k}", [(0.0, 0.0419), (0.0088, 0.0419), (0.0123, 0.0480), (0.0, 0.0480)],
                      24 if high else 12, "chrome", closed=False)
        hexa = MU.extrude_polygon(_nm(f"{key}_nh{k}"), _circle((0, 0), 0.0095 / math.cos(PI / 6), 6), 0.0479,
                                  0.0612, chamfer=0.0008, collection=_COL, smooth_angle=30.0)
        MU.assign_material(hexa, "chrome")
        dome = _lathe(f"{key}_nd{k}", [(0.0, 0.0605)] + [(0.0089 * math.cos(t), 0.0610 + 0.0080 * math.sin(t))
                                                          for t in np.linspace(0, PI / 2, 7)],
                      24 if high else 12, "chrome", closed=False)
        for ob in (cone, hexa, dome):
            _place(ob, (PCD_R * math.cos(a), 0.0, PCD_R * math.sin(a)))
            parts.append(ob)
    # centre cap
    cap = _lathe(key + "_cap", [(0.0, 0.0560), (0.0350, 0.0560), (0.0356, 0.0600), (0.0356, 0.0640),
                                (0.0335, 0.0660), (0.0200, 0.0668), (0.0, 0.0670)], 64 if high else 32,
                 "plastic_black", closed=False)
    emb = _lathe(key + "_emb", [(0.0, 0.0667), (0.0150, 0.0667), (0.0165, 0.06705), (0.0, 0.06725)],
                 48 if high else 24, "rim_alloy", closed=False)
    parts += [cap, emb]
    return _join(key, parts)


# ===========================================================================
# Hub, bearing, disc
# ===========================================================================

def _hub(key, detail, rear=True):
    high = detail == "high"
    seg = 96 if high else 48
    prof = [(0.0128, -0.0540), (0.0250, -0.0540), (0.0262, -0.0528), (0.0262, 0.0200), (0.0300, 0.0240),
            (0.0660, 0.0240), (0.0680, 0.0255), (0.0680, 0.0325), (0.0665, 0.0340), (SPIGOT_R, 0.0340),
            (SPIGOT_R, 0.0450), (SPIGOT_R - 0.0010, 0.0460), (0.0280, 0.0460), (0.0280, 0.0420),
            (0.0128, 0.0420)]
    if not rear:
        prof = [(0.0, -0.0540), (0.0250, -0.0540)] + prof[2:13] + [(0.0280, 0.0420), (0.0, 0.0420)]
        body = _lathe(key + "_b", prof, seg, "steel_machined", closed=False)
    else:
        body = _lathe(key + "_b", prof, seg, "steel_machined", closed=True)
    parts = [body]
    race = _ring(key + "_race", BRG_R[0] - 0.0002, 0.0292, BRG_A[0], BRG_A[1], seg, "steel_ground")
    parts.append(race)
    for k, a in enumerate(_lug_angles()):
        c = (PCD_R * math.cos(a), 0.0, PCD_R * math.sin(a))
        st = MU.cylinder(_nm(f"{key}_st{k}"), 0.0068, 0.0190, 0.0600, segments=16 if high else 10, chamfer=0.0008,
                         collection=_COL)
        hd = MU.cylinder(_nm(f"{key}_sh{k}"), 0.0098, 0.0185, 0.0240, segments=16 if high else 10, chamfer=0.0006,
                         collection=_COL)
        for ob in (st, hd):
            MU.assign_material(ob, "steel_machined")
            _place(ob, c)
            parts.append(ob)
    if rear:
        nut = MU.extrude_polygon(_nm(key + "_nut"), _circle((0, 0), 0.0160 / math.cos(PI / 6), 6), 0.0445, 0.0555,
                                 chamfer=0.0010, collection=_COL, smooth_angle=30.0)
        MU.assign_material(nut, "steel_dark")
        wsh = _ring(key + "_wsh", 0.0128, 0.0235, 0.0420, 0.0445, 48 if high else 24, "steel_dark")
        parts += [nut, wsh]
    else:
        capd = _lathe(key + "_dc", [(0.0, 0.0420), (0.0272, 0.0420), (0.0272, 0.0530), (0.0255, 0.0560),
                                    (0.0, 0.0565)], 48 if high else 24, "steel_machined", closed=False)
        parts.append(capd)
    return _join(key, parts)


def _brake_disc(key, detail):
    high = detail == "high"
    seg = 144 if high else 72
    v0, v1 = DISC_VENT
    a0, a1 = DISC_A
    prof = [(0.1490, v1), (0.1500, v1 + 0.0008), (0.1500, a1 - 0.0008), (0.1492, a1), (0.1000, a1),
            (0.0965, a1 + 0.0009), (0.0890, a1 + 0.0018), (0.0822, a1 + 0.0055), (0.0800, a1 + 0.0100),
            (0.0800, HAT[1] - 0.0018), (0.0782, HAT[1]), (0.0372, HAT[1]), (0.0362, HAT[1] - 0.0010),
            (0.0362, HAT[0] + 0.0006), (0.0370, HAT[0]), (0.0740, HAT[0]), (0.0740, a1 + 0.0030),
            (0.0752, a1 - 0.0020), (0.0800, v1), (0.0955, v1)]
    hat = _lathe(key + "_hat", prof, seg, "cast_iron", closed=True, smooth=30.0)
    cut = []
    for k, a in enumerate(_lug_angles()):
        cy = MU.cylinder(_nm(f"{key}_hole{k}"), 0.0076, HAT[0] - 0.003, HAT[1] + 0.003, segments=16, collection=_COL)
        _place(cy, (PCD_R * math.cos(a), 0.0, PCD_R * math.sin(a)))
        cut.append(cy)
    _bool(hat, cut)
    inner = _lathe(key + "_inb", [(0.0960, a0 + 0.0008), (0.0968, a0), (0.1492, a0), (0.1500, a0 + 0.0008),
                                  (0.1500, v0 - 0.0008), (0.1490, v0), (0.0965, v0), (0.0960, v0 - 0.0006)],
                   seg, "cast_iron", closed=True, smooth=30.0)
    parts = [hat, inner]
    nv = 36 if high else 24
    vane_poly = [(0.0975, -0.00225), (0.1485, -0.00225), (0.1485, 0.00225), (0.0975, 0.00225)]
    for k in range(nv):
        vb = MU.extrude_polygon(_nm(f"{key}_v{k}"), vane_poly, v0 - 0.0002, v1 + 0.0002, collection=_COL,
                                smooth_angle=30.0)
        MU.assign_material(vb, "cast_iron")
        _xf(vb, Matrix.Rotation(-(k + 0.5) * TAU / nv, 4, "Y"))
        parts.append(vb)
    ob = _join(key, parts)
    # machined friction faces -> steel_machined
    me = ob.data
    slot = MU.assign_material(ob, "steel_machined", faces=[])
    n = len(me.polygons)
    cen = np.zeros(n * 3)
    nor = np.zeros(n * 3)
    me.polygons.foreach_get("center", cen)
    me.polygons.foreach_get("normal", nor)
    cen, nor = cen.reshape(-1, 3), nor.reshape(-1, 3)
    rr = np.hypot(cen[:, 0], cen[:, 2])
    sel = (np.abs(nor[:, 1]) > 0.99) & (rr > 0.0995) & (rr < 0.1493) & \
        ((np.abs(cen[:, 1] - a0) < 1e-5) | (np.abs(cen[:, 1] - a1) < 1e-5))
    mi = np.zeros(n, dtype=np.int32)
    me.polygons.foreach_get("material_index", mi)
    mi[sel] = slot
    me.polygons.foreach_set("material_index", mi)
    return ob


# ===========================================================================
# Driveshaft: Rzeppa (Birfield) outer joint
#   bell / inner race / cage frames: origin = joint centre O, local +Y = world +X
#   (outboard on the right side; the left side is mirrored in local y)
# ===========================================================================

def _groove_cutters(key, centre_y, phi0, phi1, detail, phase=0.0):
    """6 tubes of radius RZ_GROOVE along arcs of radius RZ_RG centred at (0, centre_y, 0)."""
    high = detail == "high"
    out = []
    n = int(abs(phi1 - phi0) / (2.0 * DEG)) + 2
    for k in range(S.RZEPPA_BALLS):
        psi = phase + k * TAU / S.RZEPPA_BALLS
        e = np.array([math.cos(psi), 0.0, math.sin(psi)])
        ph = np.linspace(phi0, phi1, n)
        pts = [tuple(np.array([0.0, centre_y, 0.0]) + RZ_RG * (math.cos(f) * e + math.sin(f) * np.array([0, 1.0, 0])))
               for f in ph]
        out.append(MU.tube_along(_nm(f"{key}_g{k}"), pts, RZ_GROOVE, segments=24 if high else 14, bend_radius=0.0,
                                 caps=True, collection=_COL, smooth_angle=60.0))
    return out


def _sphere_arc(R, y0, y1, n):
    ys = np.linspace(y0, y1, n)
    return [(math.sqrt(max(R * R - y * y, 0.0)), y) for y in ys]


def _bell(key, detail):
    """Outer race (bell) + stub axle.  Track centre offset toward the mouth (-Y)."""
    high = detail == "high"
    seg = 96 if high else 48
    na = 24 if high else 12
    y_fl = -0.022
    prof = [(0.0, RZ_BASE), (math.sqrt(RZ_OR_SPH ** 2 - RZ_BASE ** 2) - 0.0012, RZ_BASE)]
    prof += _sphere_arc(RZ_OR_SPH, RZ_BASE - 0.0010, y_fl, na)
    prof += [(0.0285, RZ_MOUTH), (0.0410, RZ_MOUTH), (0.0420, RZ_MOUTH + 0.0008), (0.0420, -0.0255),
             (0.0400, -0.0245), (0.0400, -0.0165), (0.0425, -0.0150), (0.0450, -0.0125), (RZ_OR_OUT, -0.0110),
             (RZ_OR_OUT, 0.0140), (0.0447, 0.0190), (0.0425, 0.0235), (0.0390, 0.0270), (0.0340, 0.0293),
             (0.0300, 0.0300), (0.0160, 0.0300), (0.0135, 0.0315), (SHAFT_R, 0.0335), (SHAFT_R, 0.1360),
             (SHAFT_R - 0.0010, 0.1395), (0.0, 0.1395)]
    ob = _lathe(key, prof, seg, "steel_machined", closed=False, smooth=40.0)
    _bool(ob, _groove_cutters(key, -RZ_OFFSET, -64 * DEG, 42 * DEG, detail))
    return ob


def _inner_race(key, detail):
    high = detail == "high"
    seg = 72 if high else 40
    w = RZ_IR_W
    re = math.sqrt(RZ_IR_SPH ** 2 - w * w)
    prof = [(0.0127 + 0.0005, -w), (re - 0.0004, -w)] + _sphere_arc(RZ_IR_SPH, -w + 0.0004, w - 0.0004,
                                                                     18 if high else 9) + \
        [(re - 0.0004, w), (0.0132, w), (0.0127, w - 0.0005), (0.0127, -w + 0.0005)]
    ob = _lathe(key, prof, seg, "steel_machined", closed=True, smooth=40.0)
    _bool(ob, _groove_cutters(key, +RZ_OFFSET, -56 * DEG, 40 * DEG, detail))
    return ob


def _cage(key, detail):
    high = detail == "high"
    seg = 96 if high else 48
    w = RZ_CAGE_W
    ri, ro = RZ_CAGE_I, RZ_CAGE_O
    outer = _sphere_arc(ro, -w, w, 14 if high else 7)
    inner = _sphere_arc(ri, w, -w, 14 if high else 7)
    ob = _lathe(key, outer + inner, seg, "steel_machined", closed=True, smooth=40.0)
    cut = []
    for k in range(S.RZEPPA_BALLS):
        psi = k * TAU / S.RZEPPA_BALLS
        bx = _box(f"{key}_w{k}", (0.016, 2 * RZ_WIN_HALF, 2 * RZ_WIN_T), r=0.0012, mat="steel_machined", seg=2)
        _xf(bx, Matrix.Translation((RZ_RB * math.cos(psi), 0.0, RZ_RB * math.sin(psi))) @
            Matrix.Rotation(-psi, 4, "Y"))
        cut.append(bx)
    _bool(ob, cut)
    return ob


# ===========================================================================
# Driveshaft: bar, tripod spider + rollers, tulip
#   shaft / spider frame: origin = spider centre S, local +Y = world +X (toward O on the right)
#   tulip frame: origin = S0 (rest spider centre), local +Y = world +X (mouth outboard on the right)
# ===========================================================================

def _shaft(key, detail, L=None):
    high = detail == "high"
    L = SHAFT_L if L is None else L
    seg = 32 if high else 20
    prof = [(0.0, -0.013), (0.0108, -0.013), (0.0112, -0.0125), (0.0112, 0.0235), (SHAFT_R, 0.0262),
            (SHAFT_R, L - 0.0282), (0.0112, L - 0.0255), (0.0112, L + 0.0115), (0.0108, L + 0.012), (0.0, L + 0.012)]
    parts = [_lathe(key + "_bar", prof, seg, "steel_machined", closed=False, smooth=40.0)]
    if G is not None:
        for nm, yc, ln in (("_spa", 0.0055, 0.035), ("_spb", L - 0.0080, 0.038)):
            sp = G.external_splines(_nm(key + nm), 27, 0.0251, 0.0228, ln, collection=_COL,
                                    detail="high" if high else "low")
            _place(sp, (0.0, yc, 0.0))
            parts.append(sp)
    return _join(key, parts)


def _tripod_dirs(phase=PI / 2):
    return [phase + k * TAU / S.TRIPOD_ROLLERS for k in range(S.TRIPOD_ROLLERS)]


def _spider(key, detail):
    high = detail == "high"
    parts = [_ring(key + "_hub", 0.0127, TP_HUB_R, -0.010, 0.010, 48 if high else 24, "steel_machined", ch=0.0008)]
    for k, a in enumerate(_tripod_dirs()):
        e = Vector((math.cos(a), 0.0, math.sin(a)))
        tr = _cyl_between(f"{key}_t{k}", e * 0.0160, e * 0.0350, TP_TRUN, "steel_ground", seg=24 if high else 14,
                          ch=0.0006, x_hint=(0, 1, 0))
        ws = _cyl_between(f"{key}_c{k}", e * 0.0337, e * 0.0349, TP_TRUN + 0.0014, "steel_dark", seg=24 if high else 14,
                          ch=0.0003, x_hint=(0, 1, 0))
        parts += [tr, ws]
    clip = _ring(key + "_clip", 0.0106, 0.0140, -0.0123, -0.0108, 32 if high else 16, "steel_dark", ch=0.0002)
    parts.append(clip)
    return _join(key, parts)


def _rollers(key, detail):
    high = detail == "high"
    seg = 40 if high else 20
    w = TP_RW
    re = math.sqrt(TP_RR ** 2 - w * w)
    prof = [(TP_RBORE + 0.0004, -w), (re - 0.0004, -w)] + _sphere_arc(TP_RR, -w + 0.0004, w - 0.0004, 12 if high else 6) + \
        [(re - 0.0004, w), (TP_RBORE + 0.0004, w), (TP_RBORE, w - 0.0004), (TP_RBORE, -w + 0.0004)]
    parts = []
    nn = 18 if high else 0
    for k, a in enumerate(_tripod_dirs()):
        e = Vector((math.cos(a), 0.0, math.sin(a)))
        M = Matrix.Translation(e * TP_RT) @ _frame(e, (0.0, 1.0, 0.0)).to_4x4()
        rl = _lathe(f"{key}_{k}", prof, seg, "steel_ground", closed=True, smooth=40.0)
        _xf(rl, M)
        parts.append(rl)
        for m in range(nn):
            b = m * TAU / nn
            nd = MU.cylinder(_nm(f"{key}_n{k}_{m}"), 0.00070, -w + 0.0006, w - 0.0006, segments=6, chamfer=0.0002,
                             collection=_COL, smooth_angle=60.0)
            MU.assign_material(nd, "steel_ground")
            _xf(nd, M @ Matrix.Translation((0.00925 * math.cos(b), 0.0, 0.00925 * math.sin(b))))
            parts.append(nd)
    return _join(key, parts)


def tripod_cavity():
    """shapely polygon of the tulip cavity (local XZ)."""
    from shapely.geometry import LineString, Point
    from shapely.ops import unary_union
    shapes = [Point(0.0, 0.0).buffer(TP_CAV_R, quad_segs=16)]
    for a in _tripod_dirs():
        e = np.array([math.cos(a), math.sin(a)])
        shapes.append(LineString([tuple(e * (TP_RT - TP_SLOT)), tuple(e * (TP_RT + TP_SLOT))]).buffer(TP_TRACK,
                                                                                                   quad_segs=16))
    return unary_union(shapes)


def _tulip(key, detail):
    high = detail == "high"
    seg = 96 if high else 48
    f0 = TP_FACE
    sr, sl = TP_SPIGOT
    prof = [(0.0, f0 - sl), (sr - 0.0006, f0 - sl), (sr, f0 - sl + 0.0006), (sr, f0), (0.0440, f0),
            (0.0466, f0 + 0.0008), (TP_OUT, f0 + 0.0030), (TP_OUT, f0 + 0.0072), (TP_OUT - 0.0010, f0 + 0.0082),
            (TP_OUT - 0.0010, f0 + 0.0102), (TP_OUT, f0 + 0.0112), (TP_OUT, 0.0150), (0.0460, 0.0165),
            (0.0460, 0.0255), (0.0478, 0.0268), (TP_OUT, 0.0285), (TP_OUT - 0.0012, TP_MOUTH), (0.0, TP_MOUTH)]
    ob = _lathe(key, prof, seg, "steel_machined", closed=False, smooth=40.0)
    cav = tripod_cavity().simplify(1e-5)
    xy = np.array(cav.exterior.coords)[:-1]
    cut = MU.extrude_polygon(_nm(key + "_cav"), xy, TP_FLOOR, TP_MOUTH + 0.010, chamfer=0.0, collection=_COL)
    _bool(ob, [cut])
    return ob


# ===========================================================================
# Boots (rubber bellows) and clamps.  Profiles in zeta = distance from the big end.
# ===========================================================================

def _boot_curves(big_seat, Lb, r_lip, z_lip, z_f0, z_f1, n_f, pk, vl, z_s0, z_s1, small_seat, t=0.0016,
                 t_neck=0.0030, n=96, housing=None):
    zs = np.linspace(0.0, z_s1, n)
    zs = np.unique(np.concatenate([zs, np.linspace(0.0, z_lip + 0.004, 18)]))
    n = len(zs)
    Ro = np.zeros(n)
    for i, z in enumerate(zs):
        if z <= Lb:
            Ro[i] = big_seat + t_neck
        elif z <= z_lip:
            u = (z - Lb) / (z_lip - Lb)
            Ro[i] = big_seat + t_neck + (r_lip - big_seat - t_neck) * (u * u * (3 - 2 * u))
        elif z <= z_f0:
            Ro[i] = r_lip + (pk[0] - r_lip) * (z - z_lip) / max(z_f0 - z_lip, 1e-9)
        elif z <= z_f1:
            u = (z - z_f0) / (z_f1 - z_f0)
            p_ = pk[0] + (pk[1] - pk[0]) * u
            v_ = vl[0] + (vl[1] - vl[0]) * u
            Ro[i] = v_ + (p_ - v_) * (0.5 + 0.5 * math.cos(TAU * n_f * u))
        elif z <= z_s0:
            u = (z - z_f1) / (z_s0 - z_f1)
            Ro[i] = pk[1] + (small_seat + t_neck - pk[1]) * (u * u * (3 - 2 * u))
        else:
            Ro[i] = small_seat + t_neck
    slope = np.gradient(Ro, zs)
    g = t * np.minimum(np.sqrt(1 + slope ** 2), 2.2)
    Ri = Ro - g
    if housing is not None:
        hz, hr = np.asarray(housing, float).T
        dz = np.linspace(-0.0015, 0.0015, 7)
        clear = np.max([np.interp(zs + d, hz, hr, left=0.0, right=0.0) for d in dz], axis=0)
        Ri = np.maximum(Ri, np.where(clear > 0, clear + 0.0003, 0.0))
        Ro = np.maximum(Ro, Ri + 0.0014)
    Ri = np.where(zs <= Lb - 0.0015, big_seat + 0.0002, Ri)
    Ro = np.where(zs <= Lb - 0.0015, big_seat + t_neck, Ro)
    Ri = np.where(zs >= z_s0 - 0.0005, small_seat + 0.0003, Ri)
    return zs, Ro, Ri


def _boot(key, zs, Ro, Ri, y_of, detail):
    """Lathe a boot from curves; y_of(zeta) maps to local y."""
    high = detail == "high"
    step = 1 if high else 2
    idx = list(range(0, len(zs), step))
    if idx[-1] != len(zs) - 1:
        idx.append(len(zs) - 1)
    outer = [(Ro[i], y_of(zs[i])) for i in idx]
    inner = [(Ri[i], y_of(zs[i])) for i in reversed(idx)]
    ob = _lathe(key, outer + inner, 56 if high else 32, "rubber", closed=True, smooth=55.0)
    return ob


def _clamp(key, r_in, y0, y1, ear_dir=PI / 2, detail="high"):
    high = detail == "high"
    band = _ring(key + "_band", r_in, r_in + 0.0008, min(y0, y1), max(y0, y1), 64 if high else 32,
                 "steel_machined", ch=0.0002)
    e = Vector((math.cos(ear_dir), 0.0, math.sin(ear_dir)))
    ym = 0.5 * (y0 + y1)
    ear = _box(key + "_ear", (0.0080, abs(y1 - y0) * 0.9, 0.0030), center=(0, 0, 0), r=0.0010,
               mat="steel_machined", seg=2)
    _xf(ear, Matrix.Translation(e * (r_in + 0.0018) + Vector((0, ym, 0))) @ Matrix.Rotation(-(ear_dir - PI / 2), 4, "Y"))
    return _join(key, [band, ear])


def _boot_weight(zeta, z0, z1):
    u = np.clip((np.asarray(zeta) - z0) / (z1 - z0), 0.0, 1.0)
    return u * u * (3 - 2 * u)


# outer (Rzeppa) boot: zeta = -(y + 0.0170) on the right (big end on the bell groove)
OB_Z0 = 0.0170
OB_ARGS = dict(big_seat=0.0400, Lb=0.0075, r_lip=0.0470, z_lip=0.0115, z_f0=0.0150, z_f1=0.0720, n_f=4.5,
               pk=(0.0470, 0.0255), vl=(0.0395, 0.0195), z_s0=0.0800, z_s1=0.0950, small_seat=SHAFT_R + 0.0003,
               housing=[(-0.003, 0.0425), (-0.0005, 0.0400), (0.0075, 0.0400), (0.0085, 0.0420), (0.0102, 0.0420),
                        (0.0110, 0.0410), (0.0111, 0.0)])
# inner (tripod) boot: zeta = y - 0.0165 on the right (big end on the tulip groove)
IB_Z0 = 0.0165
IB_ARGS = dict(big_seat=0.0460, Lb=0.0090, r_lip=0.0530, z_lip=0.0140, z_f0=0.0180, z_f1=0.0800, n_f=4.5,
               pk=(0.0530, 0.0275), vl=(0.0455, 0.0210), z_s0=0.0890, z_s1=0.1040, small_seat=SHAFT_R + 0.0003,
               housing=[(-0.003, 0.0485), (0.0, 0.0460), (0.0090, 0.0460), (0.0103, 0.0478), (0.0120, 0.0485),
                        (0.0135, 0.0473), (0.0136, 0.0)])


# ===========================================================================
# Brakes and bearing (built in the transverse-local convention: +Y = wheel axis
# outboard, local X = rear, local Z = up; corner parts are turned into the corner
# frame with _toX)
# ===========================================================================

def _cal_frame():
    """Caliper frame (transverse-local): X' = radial toward the caliper, Y' = axis, Z' = tangential."""
    e = Vector((math.cos(CAL_PHI), 0.0, math.sin(CAL_PHI)))
    t = Vector((-math.sin(CAL_PHI), 0.0, math.cos(CAL_PHI)))
    return Matrix((e, Vector((0.0, 1.0, 0.0)), t)).transposed().to_4x4()


def _sector(r0, r1, b0, n=10, rc=0.004):
    """Rounded annular-sector outline (x = radial, z = tangential) about the caliper axis."""
    pts = []
    for b in np.linspace(-b0, b0, n):
        pts.append((r1 * math.cos(b), r1 * math.sin(b)))
    for b in np.linspace(b0, -b0, n):
        pts.append((r0 * math.cos(b), r0 * math.sin(b)))
    from shapely.geometry import Polygon
    p = Polygon(pts).buffer(-rc, quad_segs=3).buffer(rc, quad_segs=3)
    return np.array(p.exterior.coords)[:-1]


def _brakes(key_cal, key_car, key_pad, detail):
    """(caliper, carrier, pads) in transverse-local coordinates."""
    high = detail == "high"
    C = _cal_frame()
    a_in = DISC_A[0] - PAD_GAP                      # inner pad friction face
    a_out = DISC_A[1] + PAD_GAP
    b_in = a_in - PAD_FRIC                          # inner pad backing plate face
    b_out = a_out + PAD_FRIC
    # ---- pads
    fr = _sector(0.105, 0.147, 0.044 / 0.126, 12 if high else 6, 0.006)
    bk = _sector(0.103, 0.149, 0.050 / 0.126, 12 if high else 6, 0.004)
    ear = np.array([(0.118, -0.0600), (0.134, -0.0600), (0.134, 0.0600), (0.118, 0.0600)])
    pads = []
    for nm, f0, f1, k0, k1 in (("i", b_in, a_in, b_in - PAD_BACK, b_in), ("o", a_out, b_out, b_out, b_out + PAD_BACK)):
        f = MU.extrude_polygon(_nm(f"{key_pad}_f{nm}"), fr, f0, f1, chamfer=0.0007, collection=_COL)
        MU.assign_material(f, "friction")
        b = MU.extrude_polygon(_nm(f"{key_pad}_b{nm}"), bk, k0, k1, chamfer=0.0004, collection=_COL)
        MU.assign_material(b, "steel_dark")
        e2 = MU.extrude_polygon(_nm(f"{key_pad}_e{nm}"), ear, k0, k1, chamfer=0.0004, collection=_COL)
        MU.assign_material(e2, "steel_dark")
        pads += [f, b, e2]
    pad_ob = _join(key_pad, pads)
    _xf(pad_ob, C)
    # ---- caliper housing (floating, single 38 mm piston): one cast body (boolean union)
    bk_in = b_in - PAD_BACK                         # inner backing back face
    bk_out = b_out + PAD_BACK
    pb = _box(key_cal + "_pb", (0.064, bk_in - 0.0005 - (-0.0580), 0.084),
              center=(0.135, 0.5 * (bk_in - 0.0005 - 0.0580), 0.0), r=0.013, mat="whl_caliper_paint", seg=4)
    pist = _lathe(key_cal + "_cyl", [(0.0, -0.0665), (0.0230, -0.0665), (0.0275, -0.0625), (0.0285, -0.0560),
                                     (0.0285, -0.0400), (0.0, -0.0400)], 48 if high else 24, "whl_caliper_paint",
                  closed=False)
    _place(pist, (0.128, 0.0, 0.0))
    br = _box(key_cal + "_br", (0.0175, -0.0560 + (bk_out + 0.0125) + 0.060 - 0.004, 0.066),
              center=(0.1612, 0.5 * (-0.0560 + bk_out + 0.0125), 0.0), r=0.0075, mat="whl_caliper_paint", seg=3)
    fgs = []
    for z0, z1 in ((-0.035, -0.008), (0.008, 0.035)):
        fgs.append(_box(key_cal + f"_fg{z0:+.2f}", (0.056, 0.0125, z1 - z0),
                        center=(0.142, bk_out + 0.0005 + 0.00625, 0.5 * (z0 + z1)), r=0.0055,
                        mat="whl_caliper_paint", seg=3))
    ears = [_box(key_cal + f"_ear{sz}", (0.022, 0.018, 0.016), center=(0.160, -0.0450, sz * 0.050), r=0.0045,
                 mat="whl_caliper_paint", seg=2) for sz in (-1, 1)]
    _bool(pb, [pist, br] + fgs + ears, "UNION")
    MU.smooth_by_angle(pb, 40.0)
    body = [pb]
    # guide-pin ears, pins (bolt heads) and boots
    for sz in (-1, 1):
        hd = MU.extrude_polygon(_nm(f"{key_cal}_pin{sz}"), _circle((0.160, sz * 0.050), 0.0075 / math.cos(PI / 6), 6),
                                -0.0640, -0.0540, chamfer=0.0007, collection=_COL)
        MU.assign_material(hd, "steel_dark")
        bt = _lathe(f"{key_cal}_pb{sz}", [(0.0, -0.0362), (0.0080, -0.0362), (0.0090, -0.0343), (0.0080, -0.0325),
                                          (0.0, -0.0325)], 24 if high else 12, "rubber", closed=False)
        _place(bt, (0.160, 0.0, sz * 0.050))
        body += [hd, bt]
    nip = _cyl_between(key_cal + "_nip", (0.164, -0.0500, 0.020), (0.180, -0.0540, 0.026), 0.0042, "steel_machined",
                       seg=12, ch=0.0006)
    nipc = _cyl_between(key_cal + "_nipc", (0.180, -0.0540, 0.026), (0.186, -0.0555, 0.028), 0.0048, "rubber", seg=12,
                        ch=0.0008)
    banjo = _cyl_between(key_cal + "_bj", (0.150, -0.0620, -0.020), (0.150, -0.0720, -0.020), 0.0080, "steel_machined",
                         seg=16, ch=0.0008)
    hose = MU.tube_along(_nm(key_cal + "_hose"), [(0.150, -0.0720, -0.020), (0.156, -0.0950, -0.026),
                                                   (0.175, -0.1150, -0.030)], 0.0050, segments=12, bend_radius=0.02,
                         collection=_COL)
    MU.assign_material(hose, "rubber")
    body += [nip, nipc, banjo, hose]
    cal = _join(key_cal, body)
    _xf(cal, C)
    # ---- carrier (bolted to the upright, abutments over the disc edge)
    car = []
    for sz in (-1, 1):
        car.append(_box(f"{key_car}_leg{sz}", (0.060, 0.012, 0.012), center=(0.128, -0.0400, sz * 0.0675), r=0.003,
                        mat="cast_iron", seg=2))
        car.append(_box(f"{key_car}_arm{sz}", (0.014, bk_out - bk_in + 0.006 + 0.012, 0.012),
                        center=(0.1595, 0.5 * (bk_in - 0.012 + bk_out + 0.006), sz * 0.0675), r=0.003, mat="cast_iron",
                        seg=2))
        bolt = MU.extrude_polygon(_nm(f"{key_car}_b{sz}"), _circle((0.112, sz * 0.0675), 0.0085 / math.cos(PI / 6), 6),
                                  -0.0660, -0.0570, chamfer=0.0007, collection=_COL)
        MU.assign_material(bolt, "steel_dark")
        car.append(bolt)
    carrier = _join(key_car, car)
    _xf(carrier, C)
    return cal, carrier, pad_ob


def _partial_lathe(key, prof, a0, a1, nseg, mat):
    """Revolve a closed (r, y) profile over [a0, a1] (rad) with flat end caps."""
    mb = MU.MeshBuilder()
    prof = np.asarray(prof)
    rings = []
    for a in np.linspace(a0, a1, nseg + 1):
        rings.append(mb.verts(np.stack([prof[:, 0] * math.cos(a), prof[:, 1], prof[:, 0] * math.sin(a)], 1)))
    for r0, r1 in zip(rings[:-1], rings[1:]):
        mb.bridge(r0, r1)
    mb.face(list(rings[0]))
    mb.face(list(rings[-1]))
    return _mb_obj(mb, key, [mat], smooth=40.0)


def _shield(key, detail):
    """Pressed-steel dust shield (backing plate) with the caliper cut-out."""
    high = detail == "high"
    s0, s1 = SHIELD_A
    inner = _ring(key + "_in", 0.0505, 0.0965, s0, s1, 72 if high else 36, "steel_dark", ch=0.0002)
    gap = 42.0 * DEG
    prof = [(0.0955, s0), (0.1500, s0), (0.1520, s0 + 0.0010), (0.1525, s0 + 0.0070), (0.1513, s0 + 0.0070),
            (0.1508, s1 + 0.0002), (0.1495, s1), (0.0955, s1)]
    outer = _partial_lathe(key + "_out", prof, CAL_PHI + gap, CAL_PHI + TAU - gap, 64 if high else 32, "steel_dark")
    return _join(key, [inner, outer])


def _bearing(key, detail):
    high = detail == "high"
    seg = 72 if high else 36
    a0, a1 = BRG_A
    o = _ring(key + "_o", 0.0336, BRG_R[1], a0, a1, seg, "steel_ground", ch=0.0005)
    s1 = _ring(key + "_s1", 0.0294, 0.0336, a0, a0 + 0.0025, seg, "rubber", ch=0.0002)
    s2 = _ring(key + "_s2", 0.0294, 0.0336, a1 - 0.0025, a1, seg, "rubber", ch=0.0002)
    return _join(key, [o, s1, s2])


def _housing(key, detail):
    """Bearing housing of an upright / knuckle (transverse-local)."""
    high = detail == "high"
    prof = [(0.0402, -0.0160), (0.0485, -0.0160), (0.0495, -0.0170), (0.0495, SHIELD_A[0] - 0.0002),
            (0.0590, SHIELD_A[0] - 0.0002), (0.0610, SHIELD_A[0] - 0.0025), (0.0610, -0.0515), (0.0580, -0.0545),
            (0.0402, -0.0545)]
    return _lathe(key, prof, 72 if high else 36, "cast_aluminium", closed=True, smooth=40.0)


def _cal_ear_points():
    """Carrier bolt positions (transverse-local) on the upright side."""
    C = _cal_frame()
    return [C @ Vector((0.112, -0.0480, sz * 0.0675)) for sz in (-1, 1)]


def _tl_to_corner(v):
    """transverse-local vector -> corner (car-axis) offset: local X -> -Y, Y -> +X, Z -> Z."""
    return Vector((v.y, -v.x, v.z))


# ===========================================================================
# Rear suspension (car-frame coordinates; corner parts relative to the wheel centre)
# ===========================================================================

def _fork(key, p, axis, gap, r=0.016, th=0.008, toward=None, mat="cast_aluminium"):
    """Two clevis ears around a joint at p (bolt along axis), opened by gap; plus the bolt."""
    ax = Vector(axis).normalized()
    out = []
    for sgn in (-1, 1):
        c = Vector(p) + ax * sgn * (gap / 2 + th / 2)
        ear = MU.cylinder(_nm(f"{key}_e{sgn}"), r, -th / 2, th / 2, segments=24, chamfer=0.0012, collection=_COL)
        MU.assign_material(ear, mat)
        _xf(ear, Matrix.Translation(c) @ _frame(ax).to_4x4())
        out.append(ear)
        if toward is not None:
            out.append(_beam(f"{key}_w{sgn}", [c, c + (Vector(toward) - Vector(p)) * 0.55], [th * 0.5, th * 0.5], mat,
                             seg=12, flat=r * 0.85 / (th * 0.5), x_hint=tuple(ax)))
    bolt = _cyl_between(key + "_bolt", Vector(p) - ax * (gap / 2 + th + 0.004), Vector(p) + ax * (gap / 2 + th + 0.004),
                        0.0052, "steel_dark", seg=12, ch=0.0006)
    out.append(bolt)
    for sgn in (-1, 1):
        hx = MU.extrude_polygon(_nm(f"{key}_h{sgn}"), _circle((0, 0), 0.0095 / math.cos(PI / 6), 6), -0.003, 0.003,
                                chamfer=0.0006, collection=_COL)
        MU.assign_material(hx, "steel_dark")
        _xf(hx, Matrix.Translation(Vector(p) + ax * sgn * (gap / 2 + th + 0.003)) @ _frame(ax).to_4x4())
        out.append(hx)
    return out


def _eye(key, p, axis, r_out=0.019, half=0.018, r_in=0.0125, mat="paint_black"):
    """Bushing eye: steel sleeve + rubber bush (+ inner sleeve) on axis through p."""
    ax = Vector(axis).normalized()
    M = Matrix.Translation(Vector(p)) @ _frame(ax).to_4x4()
    sl = _ring(key + "_sl", r_in, r_out, -half, half, 32, mat, ch=0.001)
    rb = _ring(key + "_rb", 0.0085, r_in + 0.0003, -half + 0.002, half - 0.002, 24, "rubber", ch=0.0005)
    for ob in (sl, rb):
        _xf(ob, M)
    return [sl, rb]


def _upright_rear(key, detail):
    W = _w0("RR")
    U, L, T, D = R_UPPER_OUT - W, R_LOW_OUT - W, R_TOE_OUT - W, R_DAMP_LOW - W
    parts = [_toX(_housing(key + "_h", detail))]
    m = "cast_aluminium"
    parts.append(_beam(key + "_up", [(-0.044, 0.0, 0.058), (-0.076, 0.008, 0.106), (-0.125, 0.018, 0.158),
                                     U + Vector((0.012, 0.0, -0.034))], [0.0110, 0.015, 0.014, 0.013], m, seg=16,
                       flat=0.72, x_hint=(0, 1, 0)))
    parts += _fork(key + "_fu", U, (0, 1, 0), 0.038, r=0.017, th=0.008, toward=U + Vector((0.022, 0, -0.062)), mat=m)
    parts.append(_beam(key + "_lo", [(-0.044, 0.0, -0.058), (-0.058, 0.0, -0.105), L + Vector((0.0, 0.0, 0.030))],
                       [0.0110, 0.017, 0.016], m, seg=16, flat=0.75, x_hint=(0, 1, 0)))
    parts.append(MU.cylinder(_nm(key + "_lb"), 0.0175, 0.0, 0.026, segments=24, chamfer=0.0015, collection=_COL))
    MU.assign_material(parts[-1], m)
    _xf(parts[-1], Matrix.Translation(L + Vector((0, 0, 0.0190))) @ _frame((0, 0, 1)).to_4x4())
    parts.append(_beam(key + "_to", [(-0.044, 0.044, -0.038), (-0.064, 0.100, -0.050), T + Vector((0.034, -0.024, 0.0))],
                       [0.0105, 0.014, 0.013], m, seg=14, flat=0.8, x_hint=(0, 0, 1)))
    parts += _fork(key + "_ft", T, (0, 0, 1), 0.038, r=0.016, th=0.008, toward=T + Vector((0.068, -0.048, 0)), mat=m)
    parts.append(_beam(key + "_dm", [(-0.044, -0.044, -0.038), (-0.090, -0.100, -0.064), (-0.140, -0.145, -0.082),
                                     D + Vector((0.036, 0.0, 0.0))], [0.0105, 0.015, 0.014, 0.013], m, seg=14, flat=0.8,
                       x_hint=(0, 0, 1)))
    parts += _fork(key + "_fd", D, (0, 1, 0), 0.034, r=0.017, th=0.008, toward=D + Vector((0.066, 0, 0.0)), mat=m)
    # caliper-carrier ears
    for k, pe in enumerate(_cal_ear_points()):
        c = _tl_to_corner(pe)
        ear = MU.cylinder(_nm(f"{key}_ce{k}"), 0.0120, -0.0565, -0.0462, segments=20, chamfer=0.0012, collection=_COL)
        MU.assign_material(ear, m)
        _toX(ear)
        _place(ear, (0.0, c.y, c.z))
        parts.append(ear)
        parts.append(_beam(f"{key}_cw{k}", [(-0.0515, 0.5 * c.y, 0.5 * c.z), (-0.0515, c.y * 0.93, c.z * 0.93)],
                           [0.0050, 0.0050], m, seg=12, flat=2.0, x_hint=(1, 0, 0)))
    return _join(key, parts)


def _susp_link(key, p_in, p_out, r, axis_in, axis_out, adjuster=False):
    """Suspension link (world coords at rest) with bushing eyes at both ends."""
    p_in, p_out = Vector(p_in), Vector(p_out)
    d = (p_out - p_in).normalized()
    parts = _eye(key + "_ei", p_in, axis_in) + _eye(key + "_eo", p_out, axis_out, r_out=0.0175, half=0.015)
    parts.append(_beam(key + "_t", [p_in + d * 0.016, p_in + d * 0.040, p_out - d * 0.040, p_out - d * 0.015],
                       [r * 0.95, r, r, r * 0.95], "paint_black", seg=14, flat=0.9))
    if adjuster:
        mid = 0.5 * (p_in + p_out)
        hx = MU.extrude_polygon(_nm(key + "_adj"), _circle((0, 0), 0.0110 / math.cos(PI / 6), 6), -0.030, 0.030,
                                chamfer=0.0012, collection=_COL)
        MU.assign_material(hx, "steel_dark")
        _xf(hx, Matrix.Translation(mid) @ _frame(d).to_4x4())
        nut = MU.extrude_polygon(_nm(key + "_ln"), _circle((0, 0), 0.0105 / math.cos(PI / 6), 6), -0.004, 0.004,
                                 chamfer=0.0008, collection=_COL)
        MU.assign_material(nut, "steel_dark")
        _xf(nut, Matrix.Translation(mid - d * 0.036) @ _frame(d).to_4x4())
        parts += [hx, nut]
    return _join(key, parts)


def _wishbone_rear(key):
    pf, pr, po = R_LOW_IN_F, R_LOW_IN_R, R_LOW_OUT
    parts = _eye(key + "_ef", pf, (0, 1, 0), r_out=0.022, half=0.020) + _eye(key + "_er", pr, (0, 1, 0), r_out=0.022,
                                                                               half=0.020)
    for nm, a in (("f", pf), ("r", pr)):
        d = (po - a).normalized()
        parts.append(_beam(f"{key}_{nm}", [a + d * 0.020, a + d * 0.06, po - d * 0.07, po - d * 0.020],
                           [0.0145, 0.016, 0.016, 0.017], "paint_black", seg=14, flat=0.6, x_hint=(0, 0, 1)))
    # stamped web plate between the legs near the ball joint
    poly = [(po.x - 0.012, -po.y), (po.x - 0.150, -(po.y + 0.060)), (po.x - 0.150, -(po.y - 0.060))]
    plate = MU.extrude_polygon(_nm(key + "_pl"), poly, -0.0025, 0.0025, chamfer=0.0008, collection=_COL)
    MU.assign_material(plate, "paint_black")
    _xf(plate, Matrix.Translation((0.0, 0.0, po.z + 0.004)) @ Matrix.Rotation(PI / 2, 4, "X"))
    parts.append(plate)
    # ball joint housing + boot (stud goes up into the upright boss)
    bj = MU.cylinder(_nm(key + "_bj"), 0.0210, -0.020, 0.008, segments=28, chamfer=0.002, collection=_COL)
    MU.assign_material(bj, "steel_dark")
    _place(bj, po, Matrix.Rotation(PI / 2, 4, "X"))
    boot = _lathe(key + "_bb", [(0.0, 0.0075), (0.0160, 0.0075), (0.0150, 0.0110), (0.0125, 0.0145), (0.0, 0.0145)],
                  24, "rubber", closed=False)
    _place(boot, po, Matrix.Rotation(PI / 2, 4, "X"))
    parts += [bj, boot]
    return _join(key, parts)


def _coilover_lower(key, detail):
    """Damper body (moves with the upright; corner coords)."""
    D = R_DAMP_LOW - _w0("RR")
    high = detail == "high"
    parts = _eye(key + "_e", D, (0, 1, 0), r_out=0.0155, half=0.0150, mat="paint_black")
    body = _lathe(key + "_b", [(0.0, 0.0195), (0.0200, 0.0195), (0.0250, 0.0245), (0.0250, 0.2370),
                               (0.0235, 0.2400), (0.0100, 0.2400), (0.0095, 0.2380), (0.0095, 0.0450), (0.0, 0.0450)],
                  40 if high else 24, "paint_black", closed=False)
    seat0 = R_SPRING[0] - R_DAMP_LOW.z
    seat = _lathe(key + "_s", [(0.0252, seat0 - 0.014), (0.0300, seat0 - 0.014), (0.0560, seat0 - 0.004),
                               (0.0620, seat0 - 0.002), (0.0620, seat0), (0.0252, seat0)], 48 if high else 24,
                  "steel_dark", closed=True)
    for ob in (body, seat):
        _place(ob, D, Matrix.Rotation(PI / 2, 4, "X"))        # local +Y -> +Z
    return _join(key, parts + [body, seat])


def _coilover_top(key, detail):
    """Rod, bump stop, upper seat and top mount (static, car frame)."""
    high = detail == "high"
    zt = R_DAMP_TOP.z
    x, y = R_DAMP_TOP.x, R_DAMP_TOP.y
    rod = _lathe(key + "_rod", [(0.0, 0.330), (0.0085, 0.330), (0.0085, zt - 0.004), (0.0, zt - 0.004)], 24,
                 "chrome", closed=False)
    bump = _lathe(key + "_bs", [(0.0086, 0.5170), (0.0130, 0.5170), (0.0160, 0.5210), (0.0165, R_SPRING[1] - 0.001),
                                (0.0086, R_SPRING[1] - 0.001)], 24, "rubber", closed=True)
    useat = _lathe(key + "_us", [(0.0090, R_SPRING[1]), (0.0620, R_SPRING[1]), (0.0620, R_SPRING[1] + 0.003),
                                 (0.0560, R_SPRING[1] + 0.006), (0.0090, R_SPRING[1] + 0.006)], 48 if high else 24,
                   "steel_dark", closed=True)
    mount = _lathe(key + "_tm", [(0.0090, zt - 0.016), (0.0380, zt - 0.016), (0.0420, zt - 0.010),
                                 (0.0420, zt - 0.004), (0.0090, zt - 0.004)], 40 if high else 20, "rubber", closed=True)
    plate = _lathe(key + "_pl", [(0.0, zt - 0.004), (0.0500, zt - 0.004), (0.0500, zt + 0.0005), (0.0, zt + 0.0005)],
                   48 if high else 24, "steel_dark", closed=False)
    parts = []
    for ob in (rod, bump, useat, mount, plate):
        _place(ob, (x, y, 0.0), Matrix.Rotation(PI / 2, 4, "X"))
        parts.append(ob)
    return _join(key, parts)


def _subframe_rear(key):
    """Rear subframe (static): side members, cross tubes, link brackets (both sides)."""
    half = []
    m = "paint_black"
    half.append(_box(key + "_sm", (0.040, 0.420, 0.050), center=(0.235, -2.620, 0.165), r=0.008, mat=m, seg=2))
    # wishbone bush brackets (clevis plates), toe link bracket, upper link tower
    for p in (R_LOW_IN_F, R_LOW_IN_R):
        for sgn in (-1, 1):
            half.append(_box(f"{key}_lb{p.y:.2f}{sgn}", (0.090, 0.006, 0.060), center=(p.x - 0.022, p.y + sgn * 0.027, p.z + 0.004),
                             r=0.0025, mat=m, seg=2))
        half.append(_cyl_between(f"{key}_lbb{p.y:.2f}", p - Vector((0, 0.040, 0)), p + Vector((0, 0.040, 0)), 0.0052,
                                 "steel_dark", seg=12))
    for sgn in (-1, 1):
        half.append(_box(f"{key}_tb{sgn}", (0.095, 0.050, 0.006), center=(R_TOE_IN.x - 0.030, R_TOE_IN.y, R_TOE_IN.z + sgn * 0.0265),
                         r=0.0025, mat=m, seg=2))
    half.append(_box(key + "_tbp", (0.040, 0.050, 0.075), center=(0.235, R_TOE_IN.y, 0.210), r=0.006, mat=m, seg=2))
    half.append(_cyl_between(key + "_tbb", R_TOE_IN - Vector((0, 0, 0.034)), R_TOE_IN + Vector((0, 0, 0.034)), 0.0052,
                             "steel_dark", seg=12))
    # upper link tower (in front of the driveshaft)
    half.append(_beam(key + "_tw", [(0.235, R_UPPER_IN.y, 0.180), (0.255, R_UPPER_IN.y, 0.330),
                                    (0.290, R_UPPER_IN.y, R_UPPER_IN.z)], [0.024, 0.021, 0.019], m, seg=14, flat=0.8,
                      x_hint=(0, 1, 0)))
    for sgn in (-1, 1):
        half.append(_box(f"{key}_ub{sgn}", (0.075, 0.006, 0.050),
                         center=(R_UPPER_IN.x - 0.025, R_UPPER_IN.y + sgn * 0.024, R_UPPER_IN.z), r=0.0025, mat=m, seg=2))
    half.append(_cyl_between(key + "_ubb", R_UPPER_IN - Vector((0, 0.034, 0)), R_UPPER_IN + Vector((0, 0.034, 0)),
                             0.0052, "steel_dark", seg=12))
    right = _join(key + "_R", half)
    left = _copy_obj(right, key + "_L", mirror_axis=0)
    cross = [_cyl_between(key + "_cf", (-0.240, -2.430, 0.165), (0.240, -2.430, 0.165), 0.024, m, seg=20),
             _cyl_between(key + "_cr", (-0.240, -2.810, 0.165), (0.240, -2.810, 0.165), 0.024, m, seg=20)]
    return _join(key, [right, left] + cross)


# ===========================================================================
# Front suspension + steering (car frame, right side)
# ===========================================================================

def _knuckle_front(key, detail):
    W = _w0("FR")
    m = "cast_aluminium"
    parts = [_place(_toX(_housing(key + "_h", detail)), W)]
    clamp_c0 = F_B + F_AXIS * F_STRUT_L[0]
    clamp_c1 = F_B + F_AXIS * (F_STRUT_L[0] + 0.075)
    clamp = _lathe(key + "_cl", [(0.0226, 0.0), (0.0330, 0.0), (0.0345, 0.0020), (0.0345, 0.0730), (0.0330, 0.0750),
                                 (0.0226, 0.0750)], 40, m, closed=True)
    _xf(clamp, _align_y(clamp_c0, clamp_c1, (1, 0, 0)))
    parts.append(clamp)
    for k, l in enumerate((0.020, 0.055)):
        c = clamp_c0 + F_AXIS * l + Vector((0.0, -0.040, 0.0))
        parts.append(_box(f"{key}_pe{k}", (0.022, 0.030, 0.014), center=tuple(c), r=0.003, mat=m, seg=2))
        parts.append(_cyl_between(f"{key}_pb{k}", c - Vector((0.020, 0, 0)), c + Vector((0.020, 0, 0)), 0.0055,
                                  "steel_dark", seg=12))
    parts.append(_beam(key + "_web", [W + Vector((-0.044, 0.0, 0.058)), W + Vector((-0.070, 0.0, 0.072)),
                                      clamp_c0 + F_AXIS * 0.030 + Vector((0.020, 0.0, 0.0))], [0.0110, 0.020, 0.018], m,
                       seg=16, flat=0.6, x_hint=(0, 1, 0)))
    boss = F_B + Vector((0.0, 0.0, 0.0195))
    parts.append(_beam(key + "_lw", [W + Vector((-0.044, 0.0, -0.058)), W + Vector((-0.050, 0.0, -0.100)),
                                     boss + Vector((0.0, 0.0, 0.020))], [0.0110, 0.018, 0.017], m, seg=16, flat=0.7,
                       x_hint=(0, 1, 0)))
    bc = MU.cylinder(_nm(key + "_bo"), 0.0175, 0.0, 0.026, segments=24, chamfer=0.0015, collection=_COL)
    MU.assign_material(bc, m)
    _xf(bc, Matrix.Translation(boss) @ _frame((0, 0, 1)).to_4x4())
    parts.append(bc)
    tb = F_TIE_OUT + Vector((0.0, 0.0, 0.0215))
    parts.append(_beam(key + "_sa", [W + Vector((-0.044, 0.050, -0.032)), W + Vector((-0.068, 0.095, -0.055)),
                                     tb + Vector((0.004, -0.010, 0.018))], [0.0105, 0.015, 0.014], m, seg=14, flat=0.7,
                       x_hint=(0, 0, 1)))
    sb = MU.cylinder(_nm(key + "_sb"), 0.0150, 0.0, 0.024, segments=24, chamfer=0.0015, collection=_COL)
    MU.assign_material(sb, m)
    _xf(sb, Matrix.Translation(tb) @ _frame((0, 0, 1)).to_4x4())
    parts.append(sb)
    for k, pe in enumerate(_cal_ear_points()):
        c = _tl_to_corner(pe)
        ear = MU.cylinder(_nm(f"{key}_ce{k}"), 0.0120, -0.0565, -0.0462, segments=20, chamfer=0.0012, collection=_COL)
        MU.assign_material(ear, m)
        _toX(ear)
        _place(ear, (W.x, W.y + c.y, W.z + c.z))
        parts.append(ear)
        parts.append(_beam(f"{key}_cw{k}", [W + Vector((-0.0515, 0.5 * c.y, 0.5 * c.z)),
                                            W + Vector((-0.0515, c.y * 0.93, c.z * 0.93))], [0.0050, 0.0050], m, seg=12,
                           flat=2.0, x_hint=(1, 0, 0)))
    return _join(key, parts)


def _axis_M():
    """4x4: local origin at B, local +Y along the strut / steering axis."""
    return _align_y(F_B, F_T, (1.0, 0.0, 0.0))


def _strut_tube(key, detail):
    high = detail == "high"
    l0, l1 = F_STRUT_L
    tube = _lathe(key + "_t", [(0.0, l0 - 0.010), (0.0205, l0 - 0.010), (0.0225, l0 - 0.006), (0.0225, l1 - 0.004),
                               (0.0210, l1), (0.0120, l1), (0.0118, l1 - 0.002), (0.0118, 0.255), (0.0, 0.255)],
                  40 if high else 24, "paint_black", closed=False)
    seat = _lathe(key + "_s", [(0.0226, l1 - 0.016), (0.0300, l1 - 0.016), (0.0700, l1 - 0.006), (0.0800, l1 - 0.002),
                               (0.0800, F_SPRING_L[0]), (0.0226, F_SPRING_L[0])], 56 if high else 28, "steel_dark",
                  closed=True)
    ob = _join(key, [tube, seat])
    return _xf(ob, _axis_M())


def _strut_top(key, detail):
    high = detail == "high"
    lt = F_L0
    rod = _lathe(key + "_r", [(0.0, 0.330), (0.0110, 0.330), (0.0110, lt - 0.002), (0.0, lt - 0.002)], 24, "chrome",
                 closed=False)
    bump = _lathe(key + "_bs", [(0.0112, 0.600), (0.0170, 0.600), (0.0220, 0.610), (0.0240, F_SPRING_L[1] - 0.004),
                                (0.0112, F_SPRING_L[1] - 0.004)], 24, "rubber", closed=True)
    us = _lathe(key + "_us", [(0.0112, F_SPRING_L[1]), (0.0800, F_SPRING_L[1]), (0.0800, F_SPRING_L[1] + 0.003),
                              (0.0700, F_SPRING_L[1] + 0.008), (0.0112, F_SPRING_L[1] + 0.008)], 56 if high else 28,
                "steel_dark", closed=True)
    brg = _lathe(key + "_b", [(0.0112, 0.6990), (0.0520, 0.6990), (0.0520, 0.7105), (0.0112, 0.7105)],
                 48 if high else 24, "steel_dark", closed=True)
    nut = MU.extrude_polygon(_nm(key + "_n"), _circle((0, 0), 0.0110 / math.cos(PI / 6), 6), lt + 0.012, lt + 0.022,
                             chamfer=0.0008, collection=_COL)
    MU.assign_material(nut, "steel_dark")
    ob = _join(key, [rod, bump, us, brg, nut])
    return _xf(ob, _axis_M())


def _top_mount(key, detail):
    high = detail == "high"
    lt = F_L0
    rb = _lathe(key + "_r", [(0.0115, 0.7105), (0.0700, 0.7105), (0.0700, lt + 0.002), (0.0115, lt + 0.002)],
                48 if high else 24, "rubber", closed=True)
    pl = _lathe(key + "_p", [(0.0115, lt + 0.002), (0.0780, lt + 0.002), (0.0780, lt + 0.0055), (0.0115, lt + 0.0055)],
                48 if high else 24, "steel_dark", closed=True)
    studs = []
    for k in range(3):
        a = k * TAU / 3 + PI / 2
        st = MU.cylinder(_nm(f"{key}_st{k}"), 0.0040, lt + 0.0055, lt + 0.020, segments=10, collection=_COL)
        MU.assign_material(st, "steel_dark")
        _place(st, (0.060 * math.cos(a), 0.0, 0.060 * math.sin(a)))
        studs.append(st)
    ob = _join(key, [rb, pl] + studs)
    return _xf(ob, _axis_M())


def _arm_front(key):
    pf, pr, b = F_ARM_F, F_ARM_R, F_B
    u = (pr - pf).normalized()
    parts = _eye(key + "_ef", pf, u, r_out=0.022, half=0.020) + _eye(key + "_er", pr, u, r_out=0.030, half=0.024,
                                                                       r_in=0.016)
    d = (b - pf).normalized()
    parts.append(_beam(key + "_f", [pf + d * 0.020, pf + d * 0.080, b - d * 0.090, b - d * 0.022],
                       [0.016, 0.017, 0.018, 0.019], "paint_black", seg=14, flat=0.62, x_hint=(0, 0, 1)))
    j = pf + (b - pf) * 0.72
    parts.append(_beam(key + "_r", [pr + (j - pr).normalized() * 0.028, j], [0.016, 0.015], "paint_black", seg=14,
                       flat=0.62, x_hint=(0, 0, 1)))
    bj = MU.cylinder(_nm(key + "_bj"), 0.0215, -0.020, 0.010, segments=28, chamfer=0.002, collection=_COL)
    MU.assign_material(bj, "steel_dark")
    _place(bj, b, Matrix.Rotation(PI / 2, 4, "X"))
    boot = _lathe(key + "_bb", [(0.0, 0.0095), (0.0160, 0.0095), (0.0148, 0.0125), (0.0120, 0.0148), (0.0, 0.0148)],
                  24, "rubber", closed=False)
    _place(boot, b, Matrix.Rotation(PI / 2, 4, "X"))
    parts += [bj, boot]
    return _join(key, parts)


def _tie_rod(key):
    po, pi = F_TIE_OUT, F_TIE_IN0
    d = (po - pi).normalized()
    parts = []
    hs = _sphere(key + "_os", 0.0165, 20, 10, "steel_dark", center=tuple(po))
    boot = _lathe(key + "_ob", [(0.0, 0.0120), (0.0130, 0.0120), (0.0120, 0.0150), (0.0100, 0.0168), (0.0, 0.0168)],
                  20, "rubber", closed=False)
    _place(boot, po, Matrix.Rotation(PI / 2, 4, "X"))
    arm = _beam(key + "_oa", [po - d * 0.008, po - d * 0.050], [0.0120, 0.0095], "steel_dark", seg=12)
    rod = _cyl_between(key + "_r", po - d * 0.046, pi + d * 0.010, 0.0075, "steel_dark", seg=14)
    nut = MU.extrude_polygon(_nm(key + "_n"), _circle((0, 0), 0.0110 / math.cos(PI / 6), 6), -0.004, 0.004,
                             chamfer=0.0008, collection=_COL)
    MU.assign_material(nut, "steel_dark")
    _xf(nut, Matrix.Translation(po - d * 0.062) @ _frame(d).to_4x4())
    ins = _sphere(key + "_is", 0.0130, 16, 8, "steel_dark", center=tuple(pi))
    parts += [hs, boot, arm, rod, nut, ins]
    return _join(key, parts)


RACK_SOCKET = (0.022, 0.004)      # inner-joint socket: from x_in - 0.022 to x_in - 0.004


def _rack_joint(key):
    pi = F_TIE_IN0
    return _cyl_between(key, (pi.x - RACK_SOCKET[0], pi.y, pi.z), (pi.x - RACK_SOCKET[1], pi.y, pi.z), 0.0175,
                        "steel_dark", seg=24, ch=0.0015, x_hint=(0, 0, 1))


def _rack_boot(key, detail):
    """Bellows along local +X from 0 (housing end) to its rest length (scaled per frame)."""
    L = F_TIE_IN0.x - RACK_SOCKET[0] - F_RACK_HALF
    n = 90
    zs = np.linspace(0.0, L, n)
    Ro = []
    for z in zs:
        u = z / L
        if u < 0.08:
            Ro.append(0.0260)
        elif u > 0.90:
            Ro.append(0.0185)
        else:
            v = (u - 0.08) / 0.82
            pk = 0.0330 - 0.0050 * v
            vl = 0.0230 - 0.0020 * v
            Ro.append(vl + (pk - vl) * (0.5 - 0.5 * math.cos(TAU * 7 * v)))
    Ro = np.array(Ro)
    Ri = Ro - 0.0015
    Ri = np.where(zs < 0.08 * L, 0.0242, Ri)
    Ri = np.where(zs > 0.90 * L, 0.0177, Ri)
    outer = [(Ro[i], zs[i]) for i in range(n)]
    inner = [(Ri[i], zs[i]) for i in reversed(range(n))]
    ob = _lathe(key, outer + inner, 32 if detail == "high" else 20, "rubber", closed=True, smooth=55.0)
    return _toX(ob)


def _rack_and_subframe(key_rack, key_sf, detail):
    m = "paint_black"
    y, z = F_RACK_Y, F_RACK_Z
    h = F_RACK_HALF
    rk = [_cyl_between(key_rack + "_h", (-h, y, z), (h, y, z), 0.0240, "cast_aluminium", seg=32, ch=0.002,
                       x_hint=(0, 0, 1))]
    for sgn in (-1, 1):
        rk.append(_cyl_between(f"{key_rack}_c{sgn}", (sgn * (h - 0.020), y, z), (sgn * h, y, z), 0.0275,
                               "cast_aluminium", seg=32, ch=0.002, x_hint=(0, 0, 1)))
    # pinion housing (driver's side) and its input shaft stub toward the column
    pc = Vector((-0.170, y - 0.005, z + 0.010))
    rk.append(_cyl_between(key_rack + "_p", pc - Vector((0, -0.030, 0.035)), pc + Vector((0, -0.030, 0.045)), 0.0270,
                           "cast_aluminium", seg=28, ch=0.002))
    rk.append(_cyl_between(key_rack + "_ps", pc + Vector((0, -0.030, 0.045)), pc + Vector((0, -0.055, 0.095)), 0.0090,
                           "steel_machined", seg=16, ch=0.001))
    rack = _join(key_rack, rk)
    sf = [_box(key_sf + "_cm", (0.560, 0.060, 0.040), center=(0.0, 0.040, 0.192), r=0.008, mat=m, seg=2)]
    for sgn in (-1, 1):
        pf = Vector((sgn * F_ARM_F.x, F_ARM_F.y, F_ARM_F.z))
        pr = Vector((sgn * F_ARM_R.x, F_ARM_R.y, F_ARM_R.z))
        u = (pr - pf).normalized()
        for nm_, pc, hl, zz in (("f", pf, 0.020, 0.050), ("r", pr, 0.024, 0.066)):
            for k in (-1, 1):
                c = pc + u * k * (hl + 0.007)
                sf.append(_box(f"{key_sf}_{nm_}b{sgn}{k}", (0.078, 0.006, zz), center=(c.x - sgn * 0.034, c.y, c.z + 0.012),
                               r=0.002, mat=m, seg=2))
            sf.append(_cyl_between(f"{key_sf}_{nm_}bb{sgn}", pc - u * (hl + 0.016), pc + u * (hl + 0.016), 0.0060,
                                   "steel_dark", seg=12))
        sf.append(_beam(f"{key_sf}_lg{sgn}", [(sgn * 0.270, 0.020, 0.195), (sgn * 0.300, -0.150, 0.200),
                                             (sgn * (F_ARM_R.x - 0.045), F_ARM_R.y, 0.205)], [0.022, 0.020, 0.020], m,
                          seg=14, flat=0.7, x_hint=(0, 0, 1)))
        # rack clamps
        sf.append(_box(f"{key_sf}_rc{sgn}", (0.040, F_RACK_Y - 0.040, 0.030),
                       center=(sgn * 0.150, 0.5 * (0.070 + F_RACK_Y), 0.205), r=0.004, mat=m, seg=2))
        sf.append(_lathe(f"{key_sf}_rcl{sgn}", [(0.0242, -0.016), (0.0300, -0.016), (0.0300, 0.016), (0.0242, 0.016)],
                         32, m, closed=True))
        _toX(sf[-1])
        _place(sf[-1], (sgn * 0.150, F_RACK_Y, F_RACK_Z))
    return rack, _join(key_sf, sf)


# ===========================================================================
# Kinematics (pure numpy / mathutils, also used by tools/test_wheels.py)
# ===========================================================================

def rear_kin(s, side, x_ij=None):
    """Driveshaft geometry for wheel-centre offsets s (array) on side (+1 right, -1 left);
    x_ij = |x| of the tripod centre at rest (default X_IJ; the built assembly's value is
    meta['driveshaft']['x_inner']).  Returns dict: p (plunge >= 0), alpha (elevation of the
    shaft's +X-ward direction), S (spider centre, n x 3), O (Rzeppa centre, n x 3), L."""
    s = np.atleast_1d(np.asarray(s, dtype=float))
    x_ij = X_IJ if x_ij is None else x_ij
    L = X_OJ - x_ij
    p = L - np.sqrt(L * L - s * s)
    n = len(s)
    S_ = np.stack([side * (x_ij + p), np.full(n, Y_RA), np.full(n, ZW)], 1)
    O_ = np.stack([np.full(n, side * X_OJ), np.full(n, Y_RA), ZW + s], 1)
    d = (O_ - S_) * side
    alpha = np.arctan2(d[:, 2], d[:, 0])
    return dict(p=p, alpha=alpha, S=S_, O=O_, L=L)


def ball_centres_from(theta, s, side, x_ij=None):
    """(n, 6, 3) Rzeppa ball centres (car frame) for wheel angle theta and wheel offset s."""
    theta = np.atleast_1d(np.asarray(theta, dtype=float))
    k = rear_kin(s, side, x_ij)
    O_, S_ = k["O"], k["S"]
    n = len(theta)
    w_out = np.tile(np.array([side, 0.0, 0.0]), (n, 1))
    s_out = (O_ - S_)
    s_out /= np.linalg.norm(s_out, axis=1)[:, None]
    nb = s_out + w_out
    nb /= np.linalg.norm(nb, axis=1)[:, None]
    out = np.zeros((n, S.RZEPPA_BALLS, 3))
    for kb in range(S.RZEPPA_BALLS):
        ps = kb * TAU / S.RZEPPA_BALLS + theta
        g = np.stack([np.zeros(n), -np.cos(ps), np.sin(ps)], 1)
        m = np.cross(w_out, g)
        d = np.cross(m, nb)
        d /= np.linalg.norm(d, axis=1)[:, None]
        d *= np.sign(np.einsum("ij,ij->i", d, g))[:, None]
        dw = np.einsum("ij,ij->i", d, w_out)
        rho = -RZ_OFFSET * dw + np.sqrt((RZ_OFFSET * dw) ** 2 + RZ_RB ** 2)
        out[:, kb] = O_ + rho[:, None] * d
    return out


def ball_centres(track, c, x_ij=None):
    return ball_centres_from(track["theta_" + c], track["susp_" + c], _side(c), x_ij)


def _front_geom(side):
    m = Vector((side, 1.0, 1.0))
    mul = lambda v: Vector((v.x * m.x, v.y, v.z))       # noqa: E731
    return dict(B=mul(F_B), T=mul(F_T), a0=mul(F_AXIS).normalized(), pf=mul(F_ARM_F), pr=mul(F_ARM_R),
                tie_out=mul(F_TIE_OUT), tie_in=mul(F_TIE_IN0), W=_w0("FR" if side > 0 else "FL"))


def _front_mats(g, gamma, delta):
    """(knuckle 4x4, strut-top 4x4, lower-arm 4x4, dl) for arm angle gamma and steer delta."""
    u = (g["pr"] - g["pf"]).normalized()
    Ma = Matrix.Translation(g["pf"]) @ Matrix.Rotation(gamma, 4, u) @ Matrix.Translation(-g["pf"])
    Bp = Ma @ g["B"]
    T, a0 = g["T"], g["a0"]
    ap = (T - Bp).normalized()
    lp = (T - Bp).length
    dl = F_L0 - lp
    Rt = a0.rotation_difference(ap).to_matrix().to_4x4()
    Rs = Matrix.Rotation(delta, 4, a0)
    Mt = Matrix.Translation(T) @ Rt @ Rs @ Matrix.Translation(-T)
    Mk = Matrix.Translation(T) @ Rt @ Matrix.Translation(a0 * dl) @ Rs @ Matrix.Translation(-T)
    return Mk, Mt, Ma, dl


def front_gamma_table(side):
    g = _front_geom(side)
    gs = np.linspace(-0.45, 0.45, 721)
    zs = np.array([(_front_mats(g, gm, 0.0)[0] @ g["W"]).z - ZW for gm in gs])
    order = np.argsort(zs)
    return zs[order], gs[order]


def front_heading(M):
    """Heading (rad, + = left) of a wheel whose knuckle transform is M (axis +X at rest)."""
    ax = M.to_3x3() @ Vector((1.0, 0.0, 0.0))
    return math.atan2(ax.y, ax.x)


def front_steer_table(side):
    """(heading, kingpin rotation) table: the knuckle turns about the inclined steering axis by
    the angle that gives the wheel the Track's steer angle as its actual heading."""
    g = _front_geom(side)
    ds = np.linspace(-0.75, 0.75, 601)
    hs = np.array([front_heading(_front_mats(g, 0.0, d)[0]) for d in ds])
    return hs, ds


def front_kin(susp, steer, side):
    """Per-frame matrices for a front corner: dict(knuckle, strut_top, arm, dl, tie_in, tie_out)."""
    g = _front_geom(side)
    zt, gt = front_gamma_table(side)
    gam = np.interp(np.asarray(susp, float), zt, gt)
    hs, ds = front_steer_table(side)
    steer = np.interp(np.asarray(steer, float), hs, ds)
    L_tie = (g["tie_out"] - g["tie_in"]).length
    out = dict(knuckle=[], strut_top=[], arm=[], dl=[], tie_in=[], tie_out=[], gamma=gam)
    for gm, dt in zip(gam, np.asarray(steer, float)):
        Mk, Mt, Ma, dl = _front_mats(g, float(gm), float(dt))
        po = Mk @ g["tie_out"]
        dy, dz = F_RACK_Y - po.y, F_RACK_Z - po.z
        xi = po.x - side * math.sqrt(max(L_tie ** 2 - dy * dy - dz * dz, 0.0))
        out["knuckle"].append(Mk)
        out["strut_top"].append(Mt)
        out["arm"].append(Ma)
        out["dl"].append(dl)
        out["tie_out"].append(po)
        out["tie_in"].append(Vector((xi, F_RACK_Y, F_RACK_Z)))
    out["dl"] = np.array(out["dl"])
    return out


# ===========================================================================
# Baking helpers
# ===========================================================================

def _bake_mats(ob, frames, mats, scale=False):
    """Bake location + XYZ Euler (+ scale) from a list of 4x4 matrices (parent space)."""
    n = len(frames)
    loc = np.zeros((n, 3))
    rot = np.zeros((n, 3))
    scl = np.ones((n, 3))
    prev = None
    for i, M in enumerate(mats):
        l, q, sc = M.decompose()
        e = q.to_euler("XYZ", prev) if prev is not None else q.to_euler("XYZ")
        prev = e
        loc[i] = l
        rot[i] = e
        scl[i] = sc
    for ax in range(3):
        rig.bake_channel(ob, "location", ax, frames, loc[:, ax])
        rig.bake_channel(ob, "rotation_euler", ax, frames, rot[:, ax])
        if scale:
            rig.bake_channel(ob, "scale", ax, frames, scl[:, ax])


def _bake_key(ob, name, frames, values):
    key = ob.data.shape_keys
    if key is None or name not in key.key_blocks:
        return
    rig.bake_channel(key, f'key_blocks["{name}"].value', -1, frames, np.asarray(values, dtype=float))
    key.key_blocks[name].value = float(np.asarray(values)[0])


def _arr(v, n, default):
    if v is None:
        return np.full(n, default, dtype=float)
    a = np.asarray(v, dtype=float)
    return np.full(n, float(a)) if a.ndim == 0 else a


# ===========================================================================
# build
# ===========================================================================

def _boot_keys(ob, side, outer):
    co = _co(ob)
    a = OB_ARGS if outer else IB_ARGS
    z0 = OB_Z0 if outer else IB_Z0
    zeta = np.abs(co[:, 1]) - z0
    w = _boot_weight(zeta, a["Lb"] + 0.002, a["z_s0"])
    for name, sgn in (("bend", 1.0), ("bend_neg", -1.0)):
        th = sgn * w * BOOT_BEND_REF
        c, s_ = np.cos(th), np.sin(th)
        bend = co.copy()
        bend[:, 1] = co[:, 1] * c - co[:, 2] * s_
        bend[:, 2] = co[:, 1] * s_ + co[:, 2] * c
        _add_shape_key(ob, name, bend)
    if not outer:
        pl = co.copy()
        pl[:, 1] += w * BOOT_PLUNGE_REF * side
        _add_shape_key(ob, "plunge", pl)


def build(opts=None):
    global _COL
    t_start = time.time()
    opts = dict(opts or {})
    detail = opts.get("detail", "high")
    cuts = [c for c in (opts.get("cutaways") or []) if c in ("cv_cut", "tripod_cut")]
    corners = tuple(opts.get("corners", CORNERS))
    with_susp = bool(opts.get("suspension", True))
    ij_mode = opts.get("inner_joint", "flange")
    x_ij = X_IJ_SPEC if ij_mode == "spec" else X_IJ_FLANGE
    L_sh = X_OJ - x_ij
    nw = Vector(opts.get("cut_normal", (0.0, -1.0, 0.0)))
    n_loc = Vector((-nw.y, 0.0, nw.z))
    if n_loc.length < 1e-6:
        n_loc = Vector((1.0, 0.0, 0.0))
    n_loc.normalize()
    _COL = rig.collection(opts.get("collection", "wheels"))
    _local_materials()
    root = rig.empty(_nm("root"), col=_COL, size=0.25)
    parts, anchors, explode, frames_ = {}, {}, {}, {}
    groups = {}
    cut_pieces = {v: {"kept": [], "removed": [], "replaces": []} for v in cuts}
    spin = {}           # part key -> track theta key
    log = {}

    def add(key, ob, parent, grp=None):
        ob.name = _nm(key)
        if ob.data is not None:
            ob.data.name = _nm(key)
        _link(ob)
        ob.parent = parent
        if MAT is not None and ob.type == "MESH":
            MAT.ensure_props(ob)
        parts[key] = ob
        if grp:
            for gname in ([grp] if isinstance(grp, str) else grp):
                groups.setdefault(gname, []).append(key)
        return ob

    def empty(key, loc=(0, 0, 0), rot=(0, 0, 0), parent=None, size=0.04):
        e = rig.empty(_nm(key), loc=loc, rot=rot, parent=parent or root, col=_COL, size=size)
        frames_[key] = e
        return e

    def inst(tpl, key, parent, side, grp=None, mirror_axis=1):
        """Instance of a right-side template mesh (shared data; left side mirrored copy)."""
        if side > 0:
            me = tpl.data
        else:
            me = mirrored.get((tpl.name, mirror_axis))
            if me is None:
                me = tpl.data.copy()
                _mirror_mesh(me, mirror_axis)
                mirrored[(tpl.name, mirror_axis)] = me
        ob = bpy.data.objects.new(_nm(key), me)
        _COL.objects.link(ob)
        return add(key, ob, parent, grp)

    mirrored = {}
    t0 = time.time()
    # ------------------------------------------------------------------ templates
    T = {}
    T["tire"] = _tyre("tpl_tire", detail)
    T["wheel"] = _wheel("tpl_wheel", detail)
    T["hub_r"] = _hub("tpl_hub_r", detail, rear=True)
    T["hub_f"] = _hub("tpl_hub_f", detail, rear=False)
    T["disc"] = _brake_disc("tpl_disc", detail)
    cal, car, pads = _brakes("tpl_cal", "tpl_car", "tpl_pads", detail)
    T["caliper"], T["carrier"], T["pads"] = (_toX(cal), _toX(car), _toX(pads))
    T["shield"] = _toX(_shield("tpl_shield", detail))
    T["bearing"] = _toX(_bearing("tpl_bearing", detail))
    log["templates_wheel"] = time.time() - t0
    rear_corners = [c for c in corners if _is_rear(c)]
    front_corners = [c for c in corners if not _is_rear(c)]
    t0 = time.time()
    if rear_corners:
        T["bell"] = _bell("tpl_bell", detail)
        T["iring"] = _inner_race("tpl_iring", detail)
        T["cage"] = _cage("tpl_cage", detail)
        T["ball"] = _sphere("tpl_ball", RZ_BALL, 24 if detail == "high" else 16, 12 if detail == "high" else 8,
                            "steel_ground")
        T["shaft"] = _shaft("tpl_shaft", detail, L_sh)
        T["spider"] = _spider("tpl_spider", detail)
        T["rollers"] = _rollers("tpl_rollers", detail)
        T["tulip"] = _tulip("tpl_tulip", detail)
        zs, Ro, Ri = _boot_curves(**OB_ARGS)
        T["boot_o"] = _boot("tpl_boot_o", zs, Ro, Ri, lambda z: -(OB_Z0 + z), detail)
        zs2, Ro2, Ri2 = _boot_curves(**IB_ARGS)
        T["boot_i"] = _boot("tpl_boot_i", zs2, Ro2, Ri2, lambda z: IB_Z0 + z, detail)
        oa, ia = OB_ARGS, IB_ARGS
        zc_o, zc_i = 0.5 * (oa["z_s0"] + oa["z_s1"]), 0.5 * (ia["z_s0"] + ia["z_s1"])
        T["clamp_ob"] = _clamp("tpl_clamp_ob", oa["big_seat"] + 0.0032, -(OB_Z0 + 0.0008), -(OB_Z0 + 0.0048),
                               detail=detail)
        T["clamp_os"] = _clamp("tpl_clamp_os", oa["small_seat"] + 0.0032, L_sh - (OB_Z0 + zc_o - 0.003),
                               L_sh - (OB_Z0 + zc_o + 0.003), detail=detail)
        T["clamp_ib"] = _clamp("tpl_clamp_ib", ia["big_seat"] + 0.0032, IB_Z0 + 0.0012, IB_Z0 + 0.0062, detail=detail)
        T["clamp_is"] = _clamp("tpl_clamp_is", ia["small_seat"] + 0.0032, IB_Z0 + zc_i - 0.003, IB_Z0 + zc_i + 0.003,
                               detail=detail)
        if with_susp:
            T["upright"] = _upright_rear("tpl_upright", detail)
            T["damper"] = _coilover_lower("tpl_damper", detail)
    log["templates_driveshaft"] = time.time() - t0
    t0 = time.time()
    # ------------------------------------------------------------------ corners
    for c in corners:
        sx = _side(c)
        W = _w0(c)
        rear = _is_rear(c)
        if rear:
            corner = empty(f"corner_{c}", loc=W)
            hp = rig.transverse_pivot(_nm(f"hubpiv_{c}"), parent=corner, col=_COL)
        else:
            corner = empty(f"knuckle_frame_{c}")
            hp = rig.transverse_pivot(_nm(f"hubpiv_{c}"), loc=W, parent=corner, col=_COL)
        frames_[f"hubpiv_{c}"] = hp
        wg = [f"wheel_{c}", "wheels"] + (["rear_wheels"] if rear else ["front_wheels"])
        inst(T["tire"], f"tire_{c}", hp, 1, wg)
        inst(T["wheel"], f"wheel_{c}", hp, sx, wg)
        inst(T["hub_r" if rear else "hub_f"], f"hub_{c}", hp, sx, wg + ["hubs"])
        inst(T["disc"], f"brake_disc_{c}", hp, sx, wg + ["brakes"])
        for k in ("tire", "wheel", "hub", "brake_disc"):
            spin[f"{k}_{c}"] = "theta_" + c
        out = Vector((0.0, sx, 0.0))
        explode[f"wheel_{c}"] = tuple(out * 0.30)
        explode[f"tire_{c}"] = tuple(out * 0.30)
        explode[f"brake_disc_{c}"] = tuple(out * 0.16)
        # static corner parts (corner frame: rear = at W; front = knuckle frame at the car origin)
        cp = corner
        cg = ["brakes", f"brakes_{c}"]
        e_c = Vector((0.0, -math.cos(CAL_PHI), math.sin(CAL_PHI)))
        for nm_, tk in (("caliper", "caliper"), ("caliper_carrier", "carrier"), ("pads", "pads"),
                        ("backing_plate", "shield"), ("bearing", "bearing")):
            ob = inst(T[tk], f"{nm_}_{c}", cp, sx, cg, mirror_axis=0)
            if not rear:
                ob.location = W
            if nm_ in ("caliper", "caliper_carrier", "pads"):
                explode[f"{nm_}_{c}"] = tuple(e_c * 0.12)
        anchors[f"wheel_{c}"] = (hp, (0.0, sx * 0.095, 0.165))
        anchors[f"tire_{c}"] = (hp, (0.0, sx * 0.104, 0.262))
        anchors[f"brake_disc_{c}"] = (hp, (-0.10, sx * DISC_A[0], 0.10))
        anchors[f"hub_{c}"] = (hp, (0.0, sx * 0.034, 0.055))
        anchors[f"caliper_{c}"] = (corner, tuple((Vector((0.0, 0.0, 0.0)) if rear else W) +
                                                 Vector((sx * -0.050, 0.0, 0.0)) + e_c * 0.168))
        if rear:
            if with_susp:
                inst(T["upright"], f"upright_{c}", corner, sx, ["suspension", f"suspension_{c}"], mirror_axis=0)
                inst(T["damper"], f"damper_{c}", corner, sx, ["suspension", f"suspension_{c}", "coilover"], mirror_axis=0)
                anchors[f"upright_{c}"] = (corner, (sx * -0.100, 0.0, 0.110))
            # ---------------- driveshaft
            dg = [f"driveshaft_{c}", "shafts"]
            bp = rig.transverse_pivot(_nm(f"bellpiv_{c}"), loc=(sx * OJ_A, 0.0, 0.0), parent=corner, col=_COL)
            frames_[f"bellpiv_{c}"] = bp
            inst(T["bell"], f"outer_race_{c}", bp, sx, dg)
            inst(T["clamp_ob"], f"clamp_ob_{c}", bp, sx, dg + ["boots"])
            bo = inst(T["boot_o"], f"boot_outer_{c}", bp, sx, dg + ["boots"])
            bo.data = bo.data.copy()
            bo.data.name = _nm(f"boot_outer_{c}")
            _boot_keys(bo, sx, True)
            spin[f"outer_race_{c}"] = spin[f"clamp_ob_{c}"] = "theta_" + c
            irp = empty(f"irpiv_{c}", loc=(sx * X_OJ, Y_RA, ZW), rot=(0.0, 0.0, -PI / 2))
            inst(T["iring"], f"inner_race_{c}", irp, sx, dg)
            cgp = empty(f"cagepiv_{c}", loc=(sx * X_OJ, Y_RA, ZW), rot=(0.0, 0.0, -PI / 2))
            inst(T["cage"], f"cage_{c}", cgp, sx, dg)
            spin[f"inner_race_{c}"] = spin[f"cage_{c}"] = "theta_" + c
            for k in range(S.RZEPPA_BALLS):
                inst(T["ball"], f"ball_{c}_{k}", root, 1, dg + [f"balls_{c}"])
            shp = empty(f"shaftpiv_{c}", loc=(sx * x_ij, Y_RA, ZW), rot=(0.0, 0.0, -PI / 2))
            for nm_, tk in (("shaft", "shaft"), ("spider", "spider"), ("rollers", "rollers"), ("clamp_os", "clamp_os"),
                            ("clamp_is", "clamp_is")):
                g_ = dg + (["boots"] if nm_.startswith("clamp") else [])
                inst(T[tk], f"{nm_}_{c}", shp, sx, g_)
                spin[f"{nm_}_{c}"] = "theta_" + c
            tp = rig.transverse_pivot(_nm(f"tulippiv_{c}"), loc=(sx * x_ij, Y_RA, ZW), parent=root, col=_COL)
            frames_[f"tulippiv_{c}"] = tp
            inst(T["tulip"], f"tulip_{c}", tp, sx, dg)
            inst(T["clamp_ib"], f"clamp_ib_{c}", tp, sx, dg + ["boots"])
            bi = inst(T["boot_i"], f"boot_inner_{c}", tp, sx, dg + ["boots"])
            bi.data = bi.data.copy()
            bi.data.name = _nm(f"boot_inner_{c}")
            _boot_keys(bi, sx, False)
            spin[f"tulip_{c}"] = spin[f"clamp_ib_{c}"] = "theta_" + c
            nm_side = "right" if sx > 0 else "left"
            anchors[f"driveshaft_{nm_side}"] = (shp, (0.0, sx * 0.5 * L_sh, SHAFT_R + 0.001))
            anchors[f"inner_joint_{nm_side}"] = (tp, (0.0, sx * 0.002, TP_OUT + 0.001))
            anchors[f"outer_joint_{nm_side}"] = (bp, (0.0, 0.0, RZ_OR_OUT + 0.001))
            if sx > 0:
                anchors["balls_right"] = (irp, (0.0, 0.0, RZ_RB))
                anchors["cage_right"] = (cgp, (0.0, 0.0, RZ_CAGE_O))
                anchors["inner_race_right"] = (irp, (0.0, 0.0, RZ_IR_SPH - 0.002))
                anchors["outer_race_right"] = (bp, (0.0, -0.005, RZ_OR_OUT))
                anchors["tripod_rollers_right"] = (shp, (0.0, 0.0, TP_RT + TP_RR * 0.6))
                anchors["plunge_right"] = (shp, (0.0, 0.0, 0.0))
                anchors["boot_outer_right"] = (bp, (0.0, -0.060, 0.040))
                anchors["boot_inner_right"] = (tp, (0.0, 0.060, 0.046))
            # cutaway pieces
            for var, plist in (("cv_cut", (("outer_race", bp, True), ("clamp_ob", bp, True), ("boot_outer", bp, False))),
                               ("tripod_cut", (("tulip", tp, True), ("clamp_ib", tp, True), ("boot_inner", tp, False)))):
                if var not in cuts:
                    continue
                for nm_, par, spins in plist:
                    src = parts[f"{nm_}_{c}"]
                    for piece, sg in (("kept", 1.0), ("removed", -1.0)):
                        me = src.data.copy()
                        ob = bpy.data.objects.new(_nm(f"{nm_}_{c}__{var}_{piece}"), me)
                        _COL.objects.link(ob)
                        if ob.data.shape_keys is not None:
                            ob.shape_key_clear()
                        MU.cut_and_apply(ob, plane=((0.0, 0.0, 0.0), tuple(n_loc * sg)), space="LOCAL")
                        if nm_.startswith("boot"):
                            _boot_keys(ob, sx, nm_ == "boot_outer")
                        key = f"{nm_}_{c}__{var}_{piece}"
                        add(key, ob, par, dg + (["boots"] if nm_.startswith(("boot", "clamp")) else []))
                        cut_pieces[var][piece].append(key)
                        if spins:
                            spin[key] = "theta_" + c
                    cut_pieces[var]["replaces"].append(f"{nm_}_{c}")
            # ---------------- suspension links + coil-over (static parts at the car frame)
            if with_susp:
                mx = (lambda v: Vector((sx * v.x, v.y, v.z)))
                sg_ = ["suspension", f"suspension_{c}"]
                for nm_, pin, pout, r_, ain, aout, adj in (
                        ("upper_link", R_UPPER_IN, R_UPPER_OUT, 0.0105, (0, 1, 0), (0, 1, 0), False),
                        ("toe_link", R_TOE_IN, R_TOE_OUT, 0.0085, (0, 0, 1), (0, 0, 1), True)):
                    F0 = _align_y(mx(pin), mx(pout), (0.0, 0.0, 1.0))
                    le = empty(f"{nm_}_frame_{c}", loc=F0.to_translation(), rot=F0.to_euler("XYZ"))
                    ob = _susp_link(f"tmp_{nm_}_{c}", mx(pin), mx(pout), r_, ain, aout, adjuster=adj)
                    _xf(ob, F0.inverted())
                    add(f"{nm_}_{c}", ob, le, sg_)
                    anchors[f"{nm_}_{c}"] = (le, (0.0, 0.5 * (pout - pin).length, 0.012))
                pf, pr, po = mx(R_LOW_IN_F), mx(R_LOW_IN_R), mx(R_LOW_OUT)
                u = (pr - pf).normalized()
                A = pf + u * (u.dot(po - pf))
                yv = (po - A).normalized()
                R0 = Matrix((u, yv, u.cross(yv))).transposed().to_4x4()
                F0 = Matrix.Translation(A) @ R0
                we = empty(f"lower_arm_frame_{c}", loc=A, rot=R0.to_euler("XYZ"))
                wb = _wishbone_rear(f"tmp_wb_{c}")
                if sx < 0:
                    _mirror_mesh(wb.data, 0)
                _xf(wb, F0.inverted())
                add(f"lower_arm_{c}", wb, we, sg_)
                anchors[f"lower_arm_{c}"] = (we, (0.0, 0.5 * (po - A).length, 0.020))
                top = _coilover_top(f"tmp_top_{c}", detail)
                if sx < 0:
                    _mirror_mesh(top.data, 0)
                add(f"damper_rod_{c}", top, root, sg_ + ["coilover"])
                rm, wire = 0.048, 0.011
                Msp = Matrix.Translation((sx * R_DAMP_TOP.x, R_DAMP_TOP.y, 0.0)) @ Matrix.Rotation(PI / 2, 4, "X")
                spr = _spring(f"tmp_spring_{c}", rm, wire, R_SPRING[1], R_SPRING[0], 5.0, S.SUSPENSION_TRAVEL,
                              S.SUSPENSION_TRAVEL, detail=detail, M=Msp)
                add(f"spring_{c}", spr, root, sg_ + ["coilover"])
                anchors[f"spring_{c}"] = (root, (sx * (R_DAMP_TOP.x + rm), R_DAMP_TOP.y, 0.42))
                anchors[f"damper_{c}"] = (corner, (sx * (R_DAMP_LOW.x - XW_R - 0.026), R_DAMP_LOW.y - Y_RA, 0.10))
        else:
            # ---------------- front: knuckle frame children in car coordinates
            fg = ["suspension", f"suspension_{c}"]
            kn = _knuckle_front(f"tmp_kn_{c}", detail)
            st = _strut_tube(f"tmp_st_{c}", detail)
            if sx < 0:
                _mirror_mesh(kn.data, 0)
                _mirror_mesh(st.data, 0)
            add(f"knuckle_{c}", kn, corner, fg)
            add(f"strut_{c}", st, corner, fg)
            stp = empty(f"strut_top_frame_{c}")
            top = _strut_top(f"tmp_tp_{c}", detail)
            tm = _top_mount(f"tmp_tm_{c}", detail)
            Ms = _axis_M()
            spr = _spring(f"tmp_fs_{c}", 0.069, 0.0125, F_SPRING_L[1], F_SPRING_L[0], 3.5, 0.075, 0.075, detail=detail,
                          M=Ms)
            for ob in (top, tm, spr):
                if sx < 0:
                    _mirror_mesh(ob.data, 0)
            add(f"strut_top_{c}", top, stp, fg)
            add(f"spring_{c}", spr, stp, fg)
            add(f"top_mount_{c}", tm, root, fg)
            ae = empty(f"lower_arm_frame_{c}")
            arm = _arm_front(f"tmp_arm_{c}")
            tre = empty(f"tie_rod_frame_{c}")
            tr = _tie_rod(f"tmp_tr_{c}")
            rje = empty(f"rack_joint_frame_{c}")
            rj = _rack_joint(f"tmp_rj_{c}")
            rbe = empty(f"rack_boot_frame_{c}", loc=(sx * F_RACK_HALF, F_RACK_Y, F_RACK_Z))
            rb = _rack_boot(f"tmp_rb_{c}", detail)
            for ob in (arm, tr, rj, rb):
                if sx < 0:
                    _mirror_mesh(ob.data, 0)
            add(f"lower_arm_{c}", arm, ae, fg)
            add(f"tie_rod_{c}", tr, tre, fg + ["steering"])
            add(f"rack_joint_{c}", rj, rje, fg + ["steering"])
            add(f"rack_boot_{c}", rb, rbe, fg + ["steering"])
            g = _front_geom(sx)
            anchors[f"strut_{c}"] = (corner, tuple(g["B"] + g["a0"] * 0.40 + Vector((sx * 0.024, 0, 0))))
            anchors[f"knuckle_{c}"] = (corner, tuple(W + Vector((-sx * 0.060, 0.0, 0.070))))
            anchors[f"tie_rod_{c}"] = (tre, tuple(0.5 * (g["tie_out"] + g["tie_in"]) + Vector((0, 0, 0.009))))
            anchors[f"lower_arm_{c}"] = (ae, tuple(0.5 * (g["pf"] + g["B"]) + Vector((0, 0, 0.012))))
    if with_susp:
        if any(_is_rear(c) for c in corners):
            add("subframe_rear", _subframe_rear("tmp_sfr"), root, ["suspension", "subframes"])
        if any(not _is_rear(c) for c in corners):
            rk, sf = _rack_and_subframe("tmp_rack", "tmp_sff", detail)
            add("rack", rk, root, ["suspension", "steering"])
            add("subframe_front", sf, root, ["suspension", "subframes"])
    # remove templates (and mesh data left without users)
    for tpl in T.values():
        me = tpl.data
        bpy.data.objects.remove(tpl)
        if me.users == 0:
            bpy.data.meshes.remove(me)
    for me in [m for m in bpy.data.meshes if m.users == 0 and m.name.startswith(PREFIX)]:
        bpy.data.meshes.remove(me)
    log["corners"] = time.time() - t0
    meta = dict(
        power_path=[], power_groups={}, groups={k: [parts[n] for n in v] for k, v in groups.items()},
        group_names=groups, cutaway_pieces=cut_pieces, frames=frames_, spin=spin, corners=corners,
        driveshaft=dict(L=L_sh, x_inner=x_ij, inner_joint=ij_mode, x_outer=X_OJ, ball_r=RZ_BALL, ball_pcr=RZ_RB,
                        track_offset=RZ_OFFSET,
                        groove_r=RZ_GROOVE, roller_r=TP_RR, roller_pcr=TP_RT, cage=(RZ_CAGE_I, RZ_CAGE_O),
                        inner_race=RZ_IR_SPH, outer_race=RZ_OR_SPH),
        diff_interface=dict(flange_face_x=x_ij + TP_FACE, tulip_r=TP_OUT, joint_centre_x=x_ij,
                            spigot=TP_SPIGOT, mode=ij_mode),
        kinematics=dict(rear_kin=rear_kin, ball_centres=ball_centres, front_kin=front_kin),
        dims=dict(tyre_r=TYRE_R, rim_seat_r=R_SEAT, et=ET, pcd=2 * PCD_R, disc=(2 * DISC_RO, DISC_A),
                  kpi_deg=KPI / DEG, front_axis=(tuple(F_B), tuple(F_T))),
        cut_normal=tuple(nw), build_log=log, detail=detail,
    )
    pp = []
    for c in ("RL", "RR"):
        if c not in corners:
            continue
        pp += [f"tulip_{c}", f"spider_{c}", f"rollers_{c}", f"shaft_{c}", f"inner_race_{c}"] + \
              [f"ball_{c}_{k}" for k in range(S.RZEPPA_BALLS)] + [f"cage_{c}", f"outer_race_{c}", f"hub_{c}",
                                                                   f"brake_disc_{c}", f"wheel_{c}", f"tire_{c}"]
    meta["power_path"] = pp
    meta["power_groups"] = {
        "shafts": [k for k in groups.get("shafts", []) if "__" not in k],
        "rear_wheels": [k for c in ("RL", "RR") if c in corners for k in
                        (f"hub_{c}", f"brake_disc_{c}", f"wheel_{c}", f"tire_{c}")],
    }
    asm = rig.Assembly(name="wheels", root=root, parts=parts, anchors=anchors, explode=explode, meta=meta,
                       _driver=_drive)
    meta["triangles"] = {k: _tris([o]) for k, o in parts.items()}
    meta["triangles_total"] = int(sum(meta["triangles"].values()))
    meta["build_time"] = time.time() - t_start
    return asm


# ===========================================================================
# drive
# ===========================================================================

def _drive(asm, track, pres):
    fr = track.frames
    n = track.n
    if "explode" in pres and np.ndim(pres["explode"]) == 0:
        pres["explode"] = np.full(n, float(pres["explode"]))      # Assembly.bake_explode needs an array
    P = asm.parts
    F = asm.meta["frames"]
    corners = asm.meta["corners"]
    # ---- spinning parts
    for key, tk in asm.meta["spin"].items():
        if key in P:
            rig.bake_spin(P[key], fr, track[tk])
    for c in corners:
        sx = _side(c)
        s = np.asarray(track["susp_" + c], dtype=float)
        if _is_rear(c):
            W = _w0(c)
            rig.bake_channel(F[f"corner_{c}"], "location", 2, fr, W.z + s)
            x_ij = asm.meta["driveshaft"]["x_inner"]
            k = rear_kin(s, sx, x_ij)
            if f"shaftpiv_{c}" in F:
                shp = F[f"shaftpiv_{c}"]
                rig.bake_channel(shp, "location", 0, fr, k["S"][:, 0])
                rig.bake_channel(shp, "rotation_euler", 0, fr, k["alpha"])
                for nm_, fac in (("irpiv", 1.0), ("cagepiv", 0.5)):
                    e = F[f"{nm_}_{c}"]
                    rig.bake_channel(e, "location", 2, fr, k["O"][:, 2])
                    rig.bake_channel(e, "rotation_euler", 0, fr, k["alpha"] * fac)
                bc = ball_centres(track, c, x_ij)
                for kb in range(S.RZEPPA_BALLS):
                    ob = P.get(f"ball_{c}_{kb}")
                    if ob is not None:
                        rig.bake_loc(ob, fr, bc[:, kb])
                bpos = np.clip(k["alpha"] / BOOT_BEND_REF, 0.0, None)
                bneg = np.clip(-k["alpha"] / BOOT_BEND_REF, 0.0, None)
                for key in [x for x in P if x.startswith((f"boot_outer_{c}", f"boot_inner_{c}"))]:
                    _bake_key(P[key], "bend", fr, bpos)
                    _bake_key(P[key], "bend_neg", fr, bneg)
                    if key.startswith("boot_inner"):
                        _bake_key(P[key], "plunge", fr, k["p"] / BOOT_PLUNGE_REF)
            if f"spring_{c}" in P:
                _bake_key(P[f"spring_{c}"], "bump", fr, np.clip(s / S.SUSPENSION_TRAVEL, 0, 1))
                _bake_key(P[f"spring_{c}"], "droop", fr, np.clip(-s / S.SUSPENSION_TRAVEL, 0, 1))
            for nm_, pin, pout in (("upper_link", R_UPPER_IN, R_UPPER_OUT), ("toe_link", R_TOE_IN, R_TOE_OUT)):
                e = F.get(f"{nm_}_frame_{c}")
                if e is None:
                    continue
                pi_ = Vector((sx * pin.x, pin.y, pin.z))
                po0 = Vector((sx * pout.x, pout.y, pout.z))
                F0 = _align_y(pi_, po0, (0.0, 0.0, 1.0))
                R0 = F0.to_3x3()
                d0 = (po0 - pi_)
                mats = []
                for si in s:
                    d = po0 + Vector((0.0, 0.0, float(si))) - pi_
                    R = d0.normalized().rotation_difference(d.normalized()).to_matrix() @ R0
                    mats.append(Matrix.Translation(pi_) @ R.to_4x4() @ Matrix.Diagonal((1.0, d.length / d0.length, 1.0, 1.0)))
                _bake_mats(e, fr, mats, scale=True)
            e = F.get(f"lower_arm_frame_{c}")
            if e is not None:
                pf = Vector((sx * R_LOW_IN_F.x, R_LOW_IN_F.y, R_LOW_IN_F.z))
                pr = Vector((sx * R_LOW_IN_R.x, R_LOW_IN_R.y, R_LOW_IN_R.z))
                po = Vector((sx * R_LOW_OUT.x, R_LOW_OUT.y, R_LOW_OUT.z))
                u = (pr - pf).normalized()
                A = pf + u * (u.dot(po - pf))
                v0 = po - A
                R0 = Matrix((u, v0.normalized(), u.cross(v0.normalized()))).transposed()
                mats = []
                for si in s:
                    v = v0 + Vector((0.0, 0.0, float(si)))
                    v = v - u * u.dot(v)
                    ang = math.atan2(u.dot(v0.cross(v)), v0.dot(v))
                    R = Matrix.Rotation(ang, 3, u) @ R0
                    mats.append(Matrix.Translation(A) @ R.to_4x4() @ Matrix.Diagonal((1.0, v.length / v0.length, 1.0, 1.0)))
                _bake_mats(e, fr, mats, scale=True)
        else:
            steer = np.asarray(track["steer_" + c], dtype=float)
            fk = front_kin(s, steer, sx)
            _bake_mats(F[f"knuckle_frame_{c}"], fr, fk["knuckle"])
            if f"strut_top_frame_{c}" in F:
                _bake_mats(F[f"strut_top_frame_{c}"], fr, fk["strut_top"])
                _bake_mats(F[f"lower_arm_frame_{c}"], fr, fk["arm"])
                _bake_key(P[f"spring_{c}"], "bump", fr, np.clip(fk["dl"] / 0.075, 0, 1))
                _bake_key(P[f"spring_{c}"], "droop", fr, np.clip(-fk["dl"] / 0.075, 0, 1))
                g = _front_geom(sx)
                d0 = (g["tie_out"] - g["tie_in"]).normalized()
                mats, rj, rb = [], [], []
                x_h = sx * F_RACK_HALF
                x_se0 = g["tie_in"].x - sx * RACK_SOCKET[0]
                for po, pi_ in zip(fk["tie_out"], fk["tie_in"]):
                    d = (po - pi_).normalized()
                    R = d0.rotation_difference(d).to_matrix().to_4x4()
                    mats.append(Matrix.Translation(pi_) @ R @ Matrix.Translation(-g["tie_in"]))
                    rj.append(pi_.x - g["tie_in"].x)
                    rb.append(((pi_.x - sx * RACK_SOCKET[0]) - x_h) / (x_se0 - x_h))
                _bake_mats(F[f"tie_rod_frame_{c}"], fr, mats)
                rig.bake_channel(F[f"rack_joint_frame_{c}"], "location", 0, fr, np.array(rj))
                rig.bake_channel(F[f"rack_boot_frame_{c}"], "scale", 0, fr, np.array(rb))
    # ---- presentation: cutaway visibility, boot opacity, removed-piece opacity
    vis = {}
    opa = {}
    variant = pres.get("cutaway")
    cp = asm.meta["cutaway_pieces"]
    if variant is not None:
        vlist = [variant] * n if isinstance(variant, str) else list(variant)
        rem = _arr(pres.get("removed"), n, 0.0)
        for var, d in cp.items():
            on = np.array([(v == var or v == "both") for v in vlist])
            for key in d["kept"]:
                vis[key] = on
            for key in d["removed"]:
                vis[key] = on & (rem > 0.02)
                opa[key] = rem
            for key in d["replaces"]:
                vis[key] = vis.get(key, np.ones(n, bool)) & ~on
        for var, d in cp.items():
            for key in d["kept"] + d["removed"]:
                vis.setdefault(key, np.zeros(n, bool))
    else:
        for var, d in cp.items():
            for key in d["kept"] + d["removed"]:
                vis[key] = np.zeros(n, bool)
            for key in d["replaces"]:
                vis[key] = np.ones(n, bool)
    if pres.get("boot_opacity") is not None:
        bo = _arr(pres.get("boot_opacity"), n, 1.0)
        for key in asm.meta["group_names"].get("boots", []):
            opa[key] = opa.get(key, np.ones(n)) * bo
            vis[key] = vis.get(key, np.ones(n, bool)) & (bo > 0.02)
    for key in set(vis) | set(opa):
        ob = P.get(key)
        if ob is None:
            continue
        if key in opa:
            rig.bake_prop(ob, "cv_opacity", fr, opa[key])
        v = vis.get(key, np.ones(n, bool))
        hide = (~np.asarray(v, bool)).astype(float)
        rig.bake_channel(ob, "hide_render", -1, fr, hide, "CONSTANT")
        rig.bake_channel(ob, "hide_viewport", -1, fr, hide, "CONSTANT")
