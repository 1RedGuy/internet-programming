"""Propeller shaft + final drive + open differential (IRS centre section).

    from carviz.assemblies import axle
    A = axle.build({"cutaways": ["none", "half"], "detail": "high"})
    A.drive(track, {"variant": "half", "explode": e})   # bakes every moving part from the Track

Frame / origin
--------------
``A.root`` (Empty ``axl_root``) sits at the differential centre (0, Y_DIFF, Z_DIFF) with
no rotation; every child uses ROOT-LOCAL coordinates (car axes, origin at the diff centre =
common apex of the ring/pinion and of the differential bevels).  The propshaft parts lie
in front of it (root-local y up to +1.50 = the gearbox output flange face).

Kinematics (all from the Track)
-------------------------------
* gearbox-side flange yoke, pinion (+ companion flange, nut, bearing cones, spacer), rear
  flange yoke: ``theta_out`` (longitudinal convention, spin about car -Y).
* propshaft tube (+ slip yoke, boot, welded tube yoke, balance weights):
  ``kin.hooke(theta_out, beta)`` about the inclined tube axis.  beta = angle of each joint
  (Z-arrangement: gearbox output and pinion parallel, both tube yokes in phase, so the
  second joint exactly cancels the first and the pinion turns with theta_out).
* crosses (spiders) of both Hooke joints: carried round by their flange yoke (theta_out)
  and rocked about the flange-yoke arm by phi = atan(-tan(beta) * sin(theta_out)) - the
  exact solution that keeps the other arm perpendicular to the tube axis, i.e. in the tube
  yoke.  Both crosses stay parallel (Z-arrangement).
* differential case halves, ring gear (+ bolts), cross-pin, spiders' carriers:
  ``theta_case`` (transverse convention; ring gear abs angle = theta_case + PH_RING).
  pinion/ring: pinion = 4.10 x case exactly (theta_out = FINAL_DRIVE * theta_case).
* side gears + output stubs: left ``PH_SIDE + theta_RL``, right ``PH_SIDE - theta_RR``
  (right gear's local +Y points at the apex = car -X, so its scalar is negated).
* spiders (pinion gears of the differential): orbit with the case and spin about the
  cross-pin by ``SPIDER_SIGN * kin.spider_spin(theta_RL, theta_RR)`` (SPIDER_SIGN = -1, derived
  from the placement below and proven by tools/test_axle.py with collision checks in a
  R = 5 m turn).  Going straight they are still relative to the case.
* taper-roller bearing roller sets (cage): cone speed x r_cone / (r_cone + r_cup).

Final-drive geometry (FACTS FD-04, note O4): the ring gear sits on the LEFT (-X) of the
pinion axis with its teeth facing +X and the pinion meshing at the FRONT of the ring.  The
vector check v = w x r at the pitch point (pinion CW seen from the front: its +X/-X sides
move up/down; a forward-rolling ring's front edge moves DOWN) shows that only the -X side
gives forward wheel rotation in forward gears (the axle brief said +X; that would drive
the car backwards).  Geometrically: the final-drive pair is ``gears.bevel_pair_frames``
rotated 180 deg about Z, the pinion's local +Y (toward the apex) = car -Y (its scalar
is therefore negated inside its frame: the pinion object itself is car-aligned and spins
with +theta_out, the gear mesh is rotated 180 deg about Z in the object data) and the
ring's local +Y = car +X (transverse convention, no negation).  The pinion axis passes
through the ring axis (no hypoid offset, FD-05).

Parts (``A.parts`` keys; object names ``axl_<key>``)
-----------------------------------------------------
propshaft: prop_flange_front (gearbox-side flange yoke + 4 bolts), prop_cross_front
  (cross + 4 needle cups, seals, grease nipple), prop_slip_yoke (slip yoke: ears + splined
  sleeve), prop_boot (rubber boot + 2 clamps over the slip spline), prop_tube (65 mm tube,
  swaged front stub, weld beads), prop_weights (3 balance weights), prop_tube_yoke (welded
  rear tube yoke), prop_cross_rear, prop_flange_rear (pinion-side flange yoke + bolts);
  Empties prop_front_frame / prop_rear_frame (joint centres), prop_tube_frame (tube axis),
  prop_spin_front / prop_spin_rear (cross carriers).
pinion assembly: pinion (10T spiral bevel LH + shaft with splines; car-aligned frame at the
  companion-flange face), companion_flange (+ 4 nuts + pinion nut), pinion_spacer
  (collapsible spacer), pinion_cone_head / pinion_cone_tail (bearing cones, children of the
  pinion), pinion_rollers_head / _tail (roller sets, cage speed), pinion_cup_head / _tail,
  pinion_seal.
differential: ring_gear (41T spiral bevel RH, tapped bolt holes; child: ring_bolts),
  case_left (flange half, carries the ring) / case_right (plain half + 4 case bolts + the
  cross-pin retaining bolt) - a two-piece case split on the cross-pin plane with two
  windows, cross_pin, spider_1 (lower at rest) / spider_2 (upper), spider_washer_1/2,
  side_gear_left / side_gear_right (16T, splined bore), side_washer_left/right,
  stub_left / stub_right (splined output stubs with 6-bolt flanges, outer face at
  x = -/+ X_DIFF_OUTPUT), carrier_cone_left/right (children of the case halves),
  carrier_rollers_left/right, carrier_cup_left/right; Empties diff_pivot (transverse
  pivot), diff_cluster (case-angle frame of pin + spiders).
housing: housing (cast-iron centre section: pinion nose with ribs and mounting pad, side
  bearing bosses with bolted side covers, cover flange, drain boss, breather), cover
  (finned cast-aluminium rear cover with two mounting ears), housing_bolts (cover + side
  cover + mount-pad bolts), plugs (drain plug, filler plug, breather cap), bushings
  (rubber mount bushes) + bushing_sleeves, seal_left / seal_right (output seals).
cutaway pieces: ``<part>__half_kept`` / ``<part>__half_removed`` for housing, cover,
  housing_bolts, plugs, bushings, bushing_sleeves, seals and bearing cups.
explode carriers (Empties, in ``parts`` and ``explode``):
  x_case_left / x_case_right (D frame: +Y = car +X), x_ring (in x_case_left: ring slides
  off outboard, -X), x_side_left / x_side_right, x_stub_left / x_stub_right (D frame, along
  the axle), x_cross_pin, x_spider_1 / x_spider_2 (case frame K: +Z = cross-pin axis, so
  they explode along the pin and keep orbiting with the case).

Opts
----
``cutaways``  list of 'none' | 'half' (default both).  'half' = everything above the
              horizontal plane through the axle/pinion centre line removed from the static
              housing parts (housing, cover, cups, seals, bolts, plugs, bushings) - ring,
              pinion (shaft + bearings in the nose), case and gears are seen from above /
              above-behind.  Moving parts are never cut.  If 'none' is not requested the
              whole housing objects are not kept.  ``cutaway`` (str) is accepted as alias.
``detail``    'high' (default) | 'low'
``prop``      build the propeller shaft (default True)
``collection`` collection name (default 'axle')

Anchors
-------
propshaft, ujoint_front, ujoint_rear, slip_yoke, pinion, ring_gear, diff_case, spider_gear,
side_gear_left, side_gear_right, cross_pin, stub_left, stub_right, diff_housing, cover,
pinion_bearing.

explode (factor 1, in each carrier's parent frame - see above)
--------------------------------------------------------------
x_case_left (0,-0.16,0), x_case_right (0,+0.16,0), x_ring (0,-0.07,0) relative to the left
half, x_side_left (0,-0.095,0), x_side_right (0,+0.095,0), x_stub_left (0,-0.30,0),
x_stub_right (0,+0.30,0), x_spider_1 (0,0,-0.09), x_spider_2 (0,0,+0.09),
x_cross_pin (0,0,+0.22).  The case halves part along the pin plane, so pin, spiders and
side gears come free; the spiders/pin explode along the (rotating) pin axis - hide the
housing and pinion (or hold the car still) while exploded.

meta
----
``power_path`` (propshaft -> pinion -> ring -> case -> pin/spiders -> side gears -> stubs),
``power_groups`` {'prop': [...], 'diff': [...]}, ``groups`` (prop, pinion, final_drive,
differential, case, bearings, housing, stubs), ``cutaway_pieces['half']`` = {'kept',
'removed', 'replaces'}, ``variants``, ``prop`` (J1, J2, beta, beta_deg, length),
``final_drive`` (module, spiral_deg, ring_phase, apex distances, ring_side '-X'),
``diff`` (module, pressure angle, side_phase, spider_sign, case radii), ``cage`` (roller-set
speed ratios), ``output_flange`` (x, od, pcd, holes: the wheels assembly's inner joint
bolts to the stub flange face at |x| = X_DIFF_OUTPUT), ``dims``, ``housing_info``,
``build_time``.  ``variant_objects(asm, 'none'|'half')`` -> (show, fade, hide) lists.

presentation keys (all optional)
--------------------------------
``explode``  0..1 per frame (Assembly.bake_explode on the x_* carriers; a drive() without
             it puts the carriers back at rest)
``variant``  'none' | 'half' or a per-frame list: hide_render/hide_viewport of the whole
             housing objects vs their cutaway pieces (CONSTANT keys)
``removed``  0..1 cv_opacity of the removed pieces while 'half' is shown (default 0 =
             hidden); fade the removed half away with it.

Dimensions not in spec (typical 2.0 L RWD saloon, BMW E36/E46 "medium case" class)
--------------------------------------------------------------------------------
Hooke joints: 14 mm trunnions, 23 mm needle cups on a 74.4 mm span, flange OD 100 mm with
4 x M8 on a 80 mm PCD, joint centre 44 mm from each flange face; tube 65 x 1.8 mm.
Final drive module 4.634 mm (spec PD 190 mm), spiral 35 deg (pinion LH, ring RH), face
width 32.6 mm.  Differential: straight bevels, module 4.0 mm, 22.5 deg pressure angle,
16 mm cross-pin, spherical backs on a common sphere with 0.8 mm thrust washers.
Pinion bearings 40x80x19 + 30x62x17 taper rollers (back-to-back), carrier bearings
50x80x20 taper rollers, M10 ring-gear bolts on a 134 mm PCD, output stubs with 24-spline
26 mm ends and 94 mm six-bolt flanges.  Housing cast iron (7.5 mm walls), finned
aluminium cover (joint 60 mm behind the axle line) with two rubber-bushed mounting lugs
(bushes at x = -0.128 / +0.118 m), mounting pad with 2 bolts on the pinion nose.
Envelope: |x| <= 0.150 (stub flanges, lugs), r <= 0.14 m about the axle, pinion nose
r <= 0.067 m - inside the body agent's clearance cylinders.

Geometry notes
--------------
* Hooke joints: Z-arrangement, joint centres 44 mm inside each flange face, so the joint
  angle is beta = atan(55 / 1192) = 2.64 deg (FACTS PRP-02 quotes 2.46 deg using the
  flange-to-flange length); tube deviation from the flanges max beta^2/4 = 0.03 deg.
* _fix_bevel_caps() re-triangulates the heel/toe end faces of every bevel gear (shared
  gears.bevel_gear triangulates them in the axial projection, which bridges tooth gaps
  for large cone angles - see the function); flanks are untouched.
* Booleans are applied at build time with the Manifold solver (n-gons triangulated after
  each so BVH-based checks see the true surface); cutaway pieces are Manifold box cuts.
"""
from __future__ import annotations

import math
import time

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

from .. import gears as G
from .. import kin, materials, rig
from .. import meshutil as MU
from .. import spec as S

PREFIX = "axl_"
DEG = math.pi / 180.0
TAU = 2.0 * math.pi

ROOT_LOC = (0.0, S.Y_DIFF, S.Z_DIFF)

# ---------------------------------------------------------------------------
# Final drive (spiral bevel, no hypoid offset) - placement from gears.bevel_pair_frames
# ---------------------------------------------------------------------------
FD_MODULE = S.RING_PITCH_DIAMETER / S.Z_RING
FD_SPIRAL = 35.0 * DEG
_FD = G.bevel_pair_frames(S.Z_PINION_TEETH, S.Z_RING, FD_MODULE)
APEX_P = float(_FD["apexA"])        # pinion heel pitch plane -> apex (95.0 mm)
APEX_R = float(_FD["apexB"])        # ring heel pitch plane -> apex (23.2 mm)
# pinion local angle = -theta_out  ->  ring local angle = PH_RING + theta_case
PH_RING = float(G.bevel_mesh_phase(S.Z_PINION_TEETH, S.Z_RING, 0.0, _FD["dirA"], _FD["dirB"], _FD["sense"]))
RING_BORE = 0.105
RING_BOLT_R = 0.067
N_RING_BOLTS = 10

# ---------------------------------------------------------------------------
# Differential bevels.  Pair frame (bevel_pair_frames: A = spider on pair +Y, B = side gear
# on pair +X) mapped into the case frame K (= D-local at case angle 0: +Y = car +X axle,
# +Z = cross-pin, +X = car -Y) by P: pair X -> K Y, pair Y -> K Z, pair Z -> K X.
# spider_2 = spider_1 turned pi about pair X, side_left = side_right turned pi about pair Z.
# ---------------------------------------------------------------------------
DIFF_MODULE = 4.0e-3
DIFF_PA = 22.5 * DEG
_DF = G.bevel_pair_frames(S.Z_SPIDER, S.Z_SIDE_GEAR, DIFF_MODULE)
APEX_SP = float(_DF["apexA"])       # 32.0 mm
APEX_SD = float(_DF["apexB"])       # 20.0 mm
PH_SIDE = float(G.bevel_mesh_phase(S.Z_SPIDER, S.Z_SIDE_GEAR, 0.0, _DF["dirA"], _DF["dirB"], _DF["sense"]))
SPIDER_SIGN = -1.0
_P = np.array([[0.0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
_RX = np.diag([1.0, -1.0, -1.0, 1.0])
_RZ = np.diag([-1.0, -1.0, 1.0, 1.0])
M_SPIDER = (_P @ _DF["MA"], _P @ _RX @ _DF["MA"])
M_SIDE_R = _P @ _DF["MB"]
M_SIDE_L = _P @ _RZ @ _DF["MB"]
SPLINE_N, SPLINE_DMAJ, SPLINE_DMIN = 24, 0.026, 0.023          # side gear / stub splines
PIN_R = 0.0079
PIN_BORE_R = 0.00795
SPIDER_BORE = 0.016
SIDE_BORE = 0.028
BOLT_BOSS_R = 0.051          # case-half bolts (4 x M8, parallel to the axle)
LOCK_Z = 0.047               # cross-pin retaining bolt position along the pin

# ---------------------------------------------------------------------------
# Propeller shaft (root-local coordinates)
# ---------------------------------------------------------------------------
JOFF = 0.044                                        # flange face -> joint centre
J1 = np.array([0.0, S.Y_GEARBOX_REAR - S.Y_DIFF - JOFF, S.Z_CRANK - S.Z_DIFF])
J2 = np.array([0.0, S.Y_PINION_FLANGE - S.Y_DIFF + JOFF, S.Z_PINION - S.Z_DIFF])
PROP_BETA = math.atan2(J1[2] - J2[2], J1[1] - J2[1])   # joint angle (both joints)
PROP_L = float(np.linalg.norm(J1 - J2))
TUBE_R = S.PROPSHAFT_TUBE_D / 2
UJ_CAP_R, UJ_BORE_R = 0.0115, 0.0117
UJ_EAR_IN, UJ_EAR_OUT, UJ_CAP_OUT = 0.0225, 0.0370, 0.0372
UJ_TAB_R = 0.0150
UJ_BODY = 0.0135
UJ_TRUNNION_R = 0.0075
UJ_FLANGE_Y0, UJ_FLANGE_T, UJ_FLANGE_R, UJ_BOLT_R = 0.034, 0.010, 0.050, 0.040

# ---------------------------------------------------------------------------
# Pinion stack (root-local y), carrier bearings (|x|), housing
# ---------------------------------------------------------------------------
Y_PIN_OBJ = S.Y_PINION_FLANGE - S.Y_DIFF          # pinion object origin = flange face (0.220)
PB_HEAD = (0.040, 0.080, 0.019)                   # d, D, B
PB_TAIL = (0.030, 0.062, 0.017)
Y_PB_HEAD = 0.105                                 # head bearing large end (toward the head)
Y_PB_TAIL = 0.165                                 # tail bearing small end
Y_FL_HUB0, Y_FL_DISC0 = 0.182, 0.208
Y_NOSE_FRONT = 0.195
CB = (0.050, 0.080, 0.020)
X_CB = 0.067                                      # carrier bearing small end |x|
X_OUT = S.X_DIFF_OUTPUT
Y_JOINT = -0.060                                  # cover joint plane (root-local y)
FLANGE_T = 0.008

# Pumpkin (revolved about the axle X axis): outer and cavity profiles (x, r)
OUTER_X = [(-0.0800, 0.0500), (-0.0740, 0.0800), (-0.0650, 0.0950), (-0.0540, 0.1040), (-0.0430, 0.1080),
           (-0.0100, 0.1080), (0.0030, 0.1030), (0.0170, 0.0900), (0.0330, 0.0790), (0.0480, 0.0715),
           (0.0610, 0.0630), (0.0700, 0.0540), (0.0760, 0.0480)]
CAVITY_X = [(-0.0665, 0.0420), (-0.0640, 0.0840), (-0.0560, 0.0920), (-0.0470, 0.1005), (-0.0100, 0.1005),
            (0.0000, 0.0960), (0.0140, 0.0820), (0.0300, 0.0720), (0.0450, 0.0640), (0.0560, 0.0560),
            (0.0640, 0.0440), (0.0665, 0.0420)]

EXPLODE = {
    "x_case_left": (0.0, -0.16, 0.0), "x_case_right": (0.0, 0.16, 0.0), "x_ring": (0.0, -0.07, 0.0),
    "x_side_left": (0.0, -0.095, 0.0), "x_side_right": (0.0, 0.095, 0.0),
    "x_stub_left": (0.0, -0.30, 0.0), "x_stub_right": (0.0, 0.30, 0.0),
    "x_spider_1": (0.0, 0.0, -0.09), "x_spider_2": (0.0, 0.0, 0.09), "x_cross_pin": (0.0, 0.0, 0.22),
}

CUT_PLANE = ((0.0, 0.0, 0.0), (0.0, 0.0, 1.0))     # 'half': remove root-local z > 0


def profile_r(prof, x):
    """Linear interpolation of an (x, r) profile."""
    xs = np.array([p[0] for p in prof])
    rs = np.array([p[1] for p in prof])
    return float(np.interp(x, xs, rs))


# ===========================================================================
# Build context + small helpers
# ===========================================================================

class _Ctx:
    col = None
    detail = "high"
    parts = None


_C = _Ctx()


def _nm(key):
    return PREFIX + key


def _hi():
    return _C.detail != "low"


def _seg(r, minimum=16):
    base = 96 if _hi() else 40
    n = int(base * max(0.4, min(2.0, r / 0.04)))
    return max(minimum, min(160, n)) // 4 * 4


def _M(m):
    return Matrix(np.asarray(m, dtype=float).tolist())


def _T(x=0.0, y=0.0, z=0.0):
    return Matrix.Translation(Vector((x, y, z)))


def _xf(ob, M):
    ob.data.transform(M if isinstance(M, Matrix) else _M(M))
    ob.data.update()
    return ob


def _axis_rot(axis):
    """Rotation mapping local +Y onto a car axis 'X' | 'Y' | 'Z' | '-X' | '-Y' | '-Z'."""
    return {"Y": Matrix.Identity(4), "-Y": Matrix.Rotation(math.pi, 4, "Z"),
            "X": Matrix.Rotation(-math.pi / 2, 4, "Z"), "-X": Matrix.Rotation(math.pi / 2, 4, "Z"),
            "Z": Matrix.Rotation(math.pi / 2, 4, "X"), "-Z": Matrix.Rotation(-math.pi / 2, 4, "X")}[axis]


def _own(ob, key, mat=None, parent=None, smooth=None):
    """Name, link to the assembly collection, material (all faces), parent, props."""
    ob.name = _nm(key)
    if ob.data is not None:
        ob.data.name = _nm(key)
    for c in list(ob.users_collection):
        c.objects.unlink(ob)
    _C.col.objects.link(ob)
    if mat is not None and ob.type == "MESH":
        ob.data.materials.clear()
        MU.assign_material(ob, mat)
    if smooth is not None and ob.type == "MESH":
        MU.smooth_by_angle(ob, smooth)
    if parent is not None:
        ob.parent = parent
    materials.ensure_props(ob)
    _C.parts[key] = ob
    return ob


def _empty(key, parent, loc=(0, 0, 0), rot=(0, 0, 0), size=0.03):
    ob = rig.empty(_nm(key), loc=loc, rot=rot, parent=parent, col=_C.col, size=size)
    ob.rotation_mode = "XYZ"
    _C.parts[key] = ob
    return ob


def _mat(ob, name):
    ob.data.materials.clear()
    MU.assign_material(ob, name)
    return ob


def _lathe(profile, axis="Y", seg=None, closed=False, mat=None, r_for_seg=None):
    prof = [(float(r), float(y)) for r, y in profile]
    if seg is None:
        seg = _seg(r_for_seg if r_for_seg is not None else max(p[0] for p in prof))
    ob = MU.lathe("axl_tmp", prof, seg, axis="Y", caps=not closed, closed=closed, link=False)
    if axis != "Y":
        _xf(ob, _axis_rot(axis))
    if mat:
        _mat(ob, mat)
    return ob


def _cyl(r, y0, y1, axis="Y", at=(0.0, 0.0, 0.0), chamfer=0.0, seg=None, mat=None, inner=0.0):
    c = min(chamfer, 0.45 * (y1 - y0), 0.45 * (r - inner)) if chamfer else 0.0
    if inner > 0:
        prof = ([(inner, y0), (r - c, y0), (r, y0 + c), (r, y1 - c), (r - c, y1), (inner, y1)] if c else
                [(inner, y0), (r, y0), (r, y1), (inner, y1)])
        ob = _lathe(prof, "Y", seg, closed=True, r_for_seg=r)
    else:
        prof = [(0.0, y0)] + ([(r - c, y0), (r, y0 + c), (r, y1 - c), (r - c, y1)] if c else [(r, y0), (r, y1)])
        prof += [(0.0, y1)]
        ob = _lathe(prof, "Y", seg, r_for_seg=r)
    if axis != "Y":
        _xf(ob, _axis_rot(axis))
    if any(at):
        _xf(ob, _T(*at))
    if mat:
        _mat(ob, mat)
    return ob


def _prism(poly, e_u, e_v, e_n, n0, n1, origin=(0.0, 0.0, 0.0), holes=None, chamfer=0.0, mat=None):
    """Prism of a 2D polygon (u, v) in the plane spanned by e_u, e_v, extruded along e_n
    from n0 to n1 (all in the target frame)."""
    e_u, e_v, e_n = (np.asarray(a, dtype=float) for a in (e_u, e_v, e_n))
    poly = np.asarray(poly, dtype=float)
    holes = [np.asarray(h, dtype=float) for h in (holes or [])]
    if np.linalg.det(np.stack([e_u, e_n, e_v], axis=1)) < 0:
        e_v = -e_v
        poly = poly * np.array([1.0, -1.0])
        holes = [h * np.array([1.0, -1.0]) for h in holes]
    ob = MU.extrude_polygon("axl_tmp", poly, n0, n1, holes=holes or None, chamfer=chamfer, link=False)
    R = np.eye(4)
    R[:3, 0], R[:3, 1], R[:3, 2], R[:3, 3] = e_u, e_n, e_v, origin
    _xf(ob, R)
    if mat:
        _mat(ob, mat)
    return ob


def _box(size, center, radius=0.0, mat=None, segments=2):
    ob = MU.rounded_box("axl_tmp", size, radius=radius, segments=segments, center=center, link=False)
    if mat:
        _mat(ob, mat)
    return ob


def _circle(r, n, c=(0.0, 0.0), phase=0.0):
    a = phase + np.arange(n) * TAU / n
    return np.stack([c[0] + r * np.cos(a), c[1] + r * np.sin(a)], axis=1)


def _join(objs, name="axl_tmpjoin"):
    objs = [o for o in objs if o is not None]
    if len(objs) == 1:
        return objs[0]
    return MU.join(name, objs, collection=None, link=False)


def _copy(ob, name=None):
    c = ob.copy()
    c.data = ob.data.copy()
    for col in list(c.users_collection):
        col.objects.unlink(c)
    if name:
        c.name = name
    return c


def _instances(base, mats, delete_base=True):
    """Join copies of `base` (unlinked mesh object) transformed by each matrix."""
    out = []
    for M in mats:
        c = _copy(base)
        _xf(c, M)
        out.append(c)
    if delete_base:
        me = base.data
        bpy.data.objects.remove(base)
        if me.users == 0:
            bpy.data.meshes.remove(me)
    return _join(out)


def _frame(y_axis, origin, x_hint=(1.0, 0.0, 0.0)):
    """4x4 matrix whose local +Y = y_axis (unit), origin given, local X ~ x_hint."""
    y = np.asarray(y_axis, dtype=float)
    y = y / np.linalg.norm(y)
    x = np.asarray(x_hint, dtype=float)
    x = x - y * np.dot(x, y)
    if np.linalg.norm(x) < 1e-9:
        x = np.array([0.0, 0.0, 1.0]) - y * y[2]
    x /= np.linalg.norm(x)
    z = np.cross(x, y)
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = x, y, z, origin
    return _M(M)


_DEBUG_BOOL = False


def _triangulate_ngons(ob, max_sides=4):
    """Triangulate n-gons (> max_sides) - boolean results keep big planar n-gons, some
    with keyhole bridges around holes, which naive fan triangulation (e.g. BVH builders)
    gets wrong."""
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    ng = [f for f in bm.faces if len(f.verts) > max_sides]
    if ng:
        bmesh.ops.triangulate(bm, faces=ng, quad_method="BEAUTY", ngon_method="BEAUTY")
    wire = [e for e in bm.edges if not e.link_faces]
    if wire:
        bmesh.ops.delete(bm, geom=wire, context="EDGES")
    bm.to_mesh(ob.data)
    ob.data.update()
    bm.free()
    return ob


def _nonmanifold(objs):
    out = []
    for o in objs:
        bm = bmesh.new()
        bm.from_mesh(o.data)
        n = sum(1 for e in bm.edges if not e.is_manifold)
        bm.free()
        if n:
            out.append(f"{o.name}:{n} ({len(o.data.vertices)}v)")
    return ", ".join(out) or "all manifold"


def _bool(target, operands, op="DIFFERENCE", report=True):
    """Apply a Manifold boolean (operands collected, then deleted); target keeps its
    materials (operand materials transferred)."""
    ops = [o for o in operands if o is not None]
    if not ops:
        return target
    tmp = bpy.data.collections.new("axl_bool_tmp")
    bpy.context.scene.collection.children.link(tmp)
    for o in ops:
        for c in list(o.users_collection):
            c.objects.unlink(o)
        tmp.objects.link(o)
    linked_here = False
    if not target.users_collection:
        bpy.context.scene.collection.objects.link(target)
        linked_here = True
    mod = target.modifiers.new("axl_bool", "BOOLEAN")
    mod.operation = op
    mod.operand_type = "COLLECTION"
    mod.collection = tmp
    try:
        mod.material_mode = "TRANSFER"
    except Exception:
        pass
    me = None
    n0 = (len(target.data.vertices), len(target.data.polygons))
    for solver in ("MANIFOLD", "EXACT"):
        try:
            mod.solver = solver
        except TypeError:
            continue
        dg = bpy.context.evaluated_depsgraph_get()
        dg.update()
        cand = bpy.data.meshes.new_from_object(target.evaluated_get(dg))
        # Manifold refuses non-manifold input and passes the mesh through unchanged
        if len(cand.polygons) > 0 and (len(cand.vertices), len(cand.polygons)) != n0:
            me = cand
            break
        if solver == "MANIFOLD" and _DEBUG_BOOL:
            print(f"[axle] Manifold {op} skipped on {target.name} ({_nonmanifold([target] + ops)})")
        bpy.data.meshes.remove(cand)
    target.modifiers.remove(mod)
    if linked_here:
        bpy.context.scene.collection.objects.unlink(target)
    if me is not None:
        old = target.data
        target.data = me
        if old.users == 0:
            bpy.data.meshes.remove(old)
        _triangulate_ngons(target)
    elif report:
        print(f"[axle] WARNING boolean {op} failed on {target.name}")
    for o in ops:
        d = o.data
        bpy.data.objects.remove(o)
        if d is not None and d.users == 0:
            bpy.data.meshes.remove(d)
    bpy.data.collections.remove(tmp)
    return me is not None


# ---------------------------------------------------------------------------
# Standard hardware
# ---------------------------------------------------------------------------

def _hex_poly(af, phase=0.0):
    R = af / math.sqrt(3.0)
    a = phase + np.arange(6) * math.pi / 3
    return np.stack([R * np.cos(a), R * np.sin(a)], axis=1)


def _bolt(af, head_h, r_shank, length, washer=True, flange=False):
    """Hex bolt along local Y: bearing face at y = 0, head toward +Y, shank toward -Y."""
    parts = []
    y0 = 0.0
    if washer or flange:
        wr = af * (0.62 if flange else 0.52)
        wt = 0.0012 if flange else 0.0006
        parts.append(_cyl(wr, 0.0, wt, chamfer=0.0003, seg=24))
        y0 = wt
    parts.append(_prism(_hex_poly(af), (1, 0, 0), (0, 0, 1), (0, 1, 0), y0, y0 + head_h,
                        chamfer=min(0.0006, 0.12 * head_h)))
    c = 0.15 * r_shank
    parts.append(_lathe([(0.0, -length), (r_shank - c, -length), (r_shank, -length + c), (r_shank, y0 + 0.0004),
                         (0.0, y0 + 0.0004)], seg=16))
    return _join(parts)


def _nut(af, h, r_hole=0.0):
    ob = _prism(_hex_poly(af), (1, 0, 0), (0, 0, 1), (0, 1, 0), 0.0, h, chamfer=min(0.0006, 0.12 * h))
    return ob


def _bolt_circle(n, r, phase=0.0, y=0.0, axis="Y", flip=False, extra=None):
    """Matrices placing a +Y-oriented part on a bolt circle about the car axis `axis`."""
    out = []
    for k in range(n):
        a = phase + k * TAU / n
        M = _T(r * math.cos(a), y, r * math.sin(a))
        if flip:
            M = M @ Matrix.Rotation(math.pi, 4, "X")
        if extra is not None:
            M = extra @ M
        out.append(_axis_rot(axis) @ M)
    return out


# ---------------------------------------------------------------------------
# Taper roller bearing
# ---------------------------------------------------------------------------

def _taper_bearing(d, D, B, large_end=+1):
    """Cone, cup and roller set of a taper roller bearing along local Y, y in [0, B].
    large_end=+1: cone rib / large roller ends at y = B (apex toward -Y); -1: mirrored.
    Returns (cone, cup, rollers, cage_ratio)."""
    ri, ro = d / 2, D / 2
    rm = 0.5 * (ri + ro)
    ar, g = 13.0 * DEG, 2.0 * DEG
    ym = 0.5 * B
    yA = ym - rm / math.tan(ar)
    ti, to = math.tan(ar - g), math.tan(ar + g)

    def rin(y):
        return (y - yA) * ti

    def rout(y):
        return (y - yA) * to

    sM = rm / math.sin(ar)
    Lr = 0.62 * B
    s0, s1 = sM - Lr / 2, sM + Lr / 2
    rho0, rho1 = s0 * math.sin(g), s1 * math.sin(g)
    y0 = yA + s0 * math.cos(ar)
    y1 = yA + s1 * math.cos(ar)
    r0, r1 = s0 * math.sin(ar), s1 * math.sin(ar)
    clr = 0.00012
    c = min(0.0008, 0.05 * B)
    y_srib = y0 + 0.55 * rho0 * math.sin(ar) - 0.0003
    r_srib = r0 - 0.55 * rho0 * math.cos(ar)
    y_lrib = y1 + rho1 * math.sin(ar) + 0.0003
    r_lrib = r1 - 0.35 * rho1 * math.cos(ar)
    cone_prof = [(ri, 0.0), (ri, B), (r_lrib - c, B), (r_lrib, B - c), (r_lrib, y_lrib),
                 (rin(y_lrib) - clr, y_lrib), (rin(y_srib) - clr, y_srib), (r_srib, y_srib),
                 (r_srib, c), (r_srib - c, 0.0)]
    seg = _seg(ro, 32)
    cone = _lathe(cone_prof, closed=True, seg=seg, mat="steel_ground")
    yc0, yc1 = 0.03 * B, 0.985 * B
    cup_prof = [(ro, yc0 + c), (ro, yc1 - c), (ro - c, yc1), (rout(yc1) + clr, yc1), (rout(yc0) + clr, yc0),
                (ro - c, yc0)]
    cup = _lathe(cup_prof, closed=True, seg=seg, mat="steel_ground")
    # rollers
    cr = 0.12 * rho0
    rprof = [(0.0, -Lr / 2), (rho0 - cr, -Lr / 2), (rho0 + 0.5 * (rho1 - rho0) * cr / Lr, -Lr / 2 + cr),
             (rho1 - 0.5 * (rho1 - rho0) * cr / Lr, Lr / 2 - cr), (rho1 - cr, Lr / 2), (0.0, Lr / 2)]
    base = _lathe(rprof, seg=16 if _hi() else 10)
    rho_m = sM * math.sin(g)
    n = int(TAU * rm / (2.0 * rho_m * 1.16))
    mats = []
    for k in range(n):
        phi = (k + 0.5) * TAU / n
        u = np.array([math.sin(ar) * math.cos(phi), math.cos(ar), math.sin(ar) * math.sin(phi)])
        ctr = np.array([rm * math.cos(phi), ym, rm * math.sin(phi)])
        mats.append(_frame(u, ctr, x_hint=(-math.sin(phi), 0.0, math.cos(phi))))
    rollers = _instances(base, mats)
    _mat(rollers, "steel_ground")
    if large_end < 0:
        F = _T(0.0, B, 0.0) @ Matrix.Rotation(math.pi, 4, "Z")
        for ob in (cone, cup, rollers):
            _xf(ob, F)
    k_cage = rin(ym) / (rin(ym) + rout(ym))
    return cone, cup, rollers, k_cage


# ===========================================================================
# Gears
# ===========================================================================

def _ring_bolt_angles(in_ring):
    """Profile angles of the ring-gear bolts: in the case frame (k + 1/2) * 36 deg; in the
    ring's own frame shifted by -PH_RING because the ring is baked at PH_RING + theta_case."""
    a = (np.arange(N_RING_BOLTS) + 0.5) * TAU / N_RING_BOLTS
    return a - PH_RING if in_ring else a


def _fix_bevel_caps(ob):
    """Re-triangulate the heel and toe end faces of a gears.bevel_gear() mesh.

    WORKAROUND (shared-module issue): bevel_gear() fills both end faces with a CDT
    computed in the projection along the gear axis.  For large pitch-cone angles (ring
    gear 76 deg, side gears 58 deg) the end faces lie on back cones that are nearly
    parallel to the axis, the projected region degenerates and some triangles bridge
    across tooth gaps ("webs" up to ~1.5 mm inside the tooth space) - the mating pinion /
    spider tips then hit them.  Here every face whose vertices all lie on the heel
    (toe) back cone is removed and the band between the two boundary loops (tooth
    outline, blank ring) is re-filled with an angle-ordered zipper strip, which stays on
    the cone.  The tooth flanks are untouched."""
    dl = float(ob["bevel_delta"])
    ay = float(ob["bevel_apex_y"])
    Re = float(ob["bevel_R_e"])
    b = float(ob["bevel_face_width"])
    sd, cd = math.sin(dl), math.cos(dl)
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    for target in (Re, Re - b):
        def on(v):
            r = math.hypot(v.co.x, v.co.z)
            return abs(r * sd - (v.co.y - ay) * cd - target) < 4e-6
        faces = [f for f in bm.faces if all(on(v) for v in f.verts)]
        if not faces:
            continue
        bmesh.ops.delete(bm, geom=faces, context="FACES")
        bedges = [e for e in bm.edges if len(e.link_faces) == 1 and on(e.verts[0]) and on(e.verts[1])]
        adj = {}
        for e in bedges:
            a_, b_ = e.verts
            adj.setdefault(a_, []).append(b_)
            adj.setdefault(b_, []).append(a_)
        loops, seen = [], set()
        for st in adj:
            if st in seen:
                continue
            lp, prev, cur = [st], None, st
            seen.add(st)
            while True:
                nxt = [w for w in adj[cur] if w is not prev]
                if not nxt or nxt[0] is st:
                    break
                prev, cur = cur, nxt[0]
                if cur in seen:
                    break
                lp.append(cur)
                seen.add(cur)
            loops.append(lp)
        if len(loops) != 2:
            print(f"[axle] WARNING bevel cap fix: {len(loops)} loops on {ob.name}")
            continue
        rings = []
        for lp in loops:
            psi = np.array([math.atan2(v.co.z, v.co.x) for v in lp])
            if np.sum((np.diff(np.concatenate([psi, psi[:1]])) + math.pi) % TAU - math.pi) < 0:
                lp = lp[::-1]
                psi = psi[::-1]
            psi = psi % TAU
            k0 = int(np.argmin(psi))
            lp = lp[k0:] + lp[:k0]
            psi = np.unwrap(np.concatenate([psi[k0:], psi[:k0]]))
            psi = psi - TAU * math.floor(psi[0] / TAU)
            rings.append((lp + lp[:1], np.concatenate([psi, [psi[0] + TAU]])))
        (A_, pa), (B_, pb) = rings
        i = j = 0
        nA, nB = len(A_) - 1, len(B_) - 1
        while i < nA or j < nB:
            if j >= nB or (i < nA and pa[i + 1] <= pb[j + 1]):
                tri = (A_[i], A_[i + 1], B_[j])
                i += 1
            else:
                tri = (A_[i], B_[j + 1], B_[j])
                j += 1
            if len(set(tri)) == 3:
                try:
                    f = bm.faces.new(tri)
                    f.smooth = False
                except ValueError:
                    pass
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    bm.to_mesh(ob.data)
    bm.free()
    ob.data.update()
    MU.smooth_by_angle(ob, 35.0)
    return ob


def _sphere_back_radius(ob, apex_y, exclude_r):
    co = np.zeros(len(ob.data.vertices) * 3)
    ob.data.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    r = np.hypot(co[:, 0], co[:, 2])
    dist = np.sqrt(r * r + (co[:, 1] - apex_y) ** 2)
    sel = (r > exclude_r + 2e-4) & (co[:, 1] < apex_y - 0.01)
    Rb = float(dist[sel].max())
    on = sel & (np.abs(dist - Rb) < 2e-4)
    return Rb, float(r[on].max()), float(r[on].min())


def _build_gears():
    det = "high" if _hi() else "low"
    # gears.bevel_gear spiral teeth at detail 'low' are sliced too coarsely for the 35 deg
    # spiral (pinion/ring interfere at every phase) - 'medium' is the floor for the final drive
    det_fd = "high" if _hi() else "medium"
    pin = G.bevel_gear("axl_tmp_pin", S.Z_PINION_TEETH, S.Z_RING, FD_MODULE, spiral=FD_SPIRAL, hand="left",
                       detail=det_fd, link=False)
    ring = G.bevel_gear("axl_tmp_ring", S.Z_RING, S.Z_PINION_TEETH, FD_MODULE, spiral=FD_SPIRAL, hand="right",
                        bore=RING_BORE, detail=det_fd, link=False)
    _fix_bevel_caps(pin)
    _fix_bevel_caps(ring)
    co = np.zeros(len(ring.data.vertices) * 3)
    ring.data.vertices.foreach_get("co", co)
    ring_back = float(co.reshape(-1, 3)[:, 1].min())
    # tapped holes for the ring bolts in the back face
    holes = [_cyl(0.0042, ring_back - 0.001, ring_back + 0.014, at=(RING_BOLT_R * math.cos(a), 0.0,
                                                                   RING_BOLT_R * math.sin(a)), seg=16)
             for a in _ring_bolt_angles(in_ring=True)]
    _bool(ring, holes, "DIFFERENCE")
    MU.smooth_by_angle(ring, 35.0)
    sp = G.bevel_gear("axl_tmp_sp", S.Z_SPIDER, S.Z_SIDE_GEAR, DIFF_MODULE, pressure_angle=DIFF_PA,
                      bore=SPIDER_BORE, spherical_back=True, detail=det, link=False)
    sd = G.bevel_gear("axl_tmp_sd", S.Z_SIDE_GEAR, S.Z_SPIDER, DIFF_MODULE, pressure_angle=DIFF_PA,
                      bore=SIDE_BORE, spherical_back=True, back_hub=(0.040, 0.012), detail=det, link=False)
    _fix_bevel_caps(sp)
    _fix_bevel_caps(sd)
    co = np.zeros(len(sd.data.vertices) * 3)
    sd.data.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    sd_y0, sd_y1 = float(co[:, 1].min()), float(co[:, 1].max())
    spl = G.internal_splines("axl_tmp_spl", SPLINE_N, SPLINE_DMAJ, SPLINE_DMIN, sd_y1 - sd_y0 - 0.0004,
                             outer_d=SIDE_BORE + 0.004, detail=det, link=False)
    _xf(spl, _T(0.0, 0.5 * (sd_y0 + sd_y1), 0.0))
    _bool(sd, [spl], "UNION")
    MU.smooth_by_angle(sd, 35.0)
    Rb_sp, rB_sp, rS_sp = _sphere_back_radius(sp, APEX_SP, SPIDER_BORE / 2)
    Rb_sd, rB_sd, rS_sd = _sphere_back_radius(sd, APEX_SD, 0.020)
    return dict(pin=pin, ring=ring, ring_back=ring_back, sp=sp, sd=sd, sd_y=(sd_y0, sd_y1),
                Rb_sp=Rb_sp, rB_sp=rB_sp, rS_sp=rS_sp, Rb_sd=Rb_sd, rB_sd=rB_sd, rS_sd=rS_sd)


def _sphere_washer(apex_y, R0, R1, r_lo, r_hi, mat="brass"):
    """Spherical thrust washer (shell between radii R0 < R1 about the apex on local +Y),
    covering the band r_lo..r_hi measured on the inner sphere; behind the gear (-Y)."""
    a0, a1 = math.asin(min(0.999, r_lo / R0)), math.asin(min(0.999, r_hi / R0))
    n = 8
    inner = [(R0 * math.sin(a), apex_y - R0 * math.cos(a)) for a in np.linspace(a0, a1, n)]
    outer = [(R1 * math.sin(a), apex_y - R1 * math.cos(a)) for a in np.linspace(a1, a0, n)]
    return _lathe(inner + outer, closed=True, seg=_seg(r_hi, 32), mat=mat)


# ===========================================================================
# Differential case (two halves split on the cross-pin plane)
# ===========================================================================

def _build_case(R_ci, y_rb):
    R_co = R_ci + 0.0072
    L_b = R_co + 0.0048
    # the inner sphere must stay within ~0.1 mm of the true sphere at any detail: the thrust
    # washers sit 0.15 mm inside it
    n_sph = 32
    ball = _lathe([(R_co * math.sin(a), -R_co * math.cos(a)) for a in np.linspace(0.0, math.pi, n_sph + 1)],
                  seg=max(_seg(R_co, 48), 80))
    adds = []
    adds.append(_lathe([(0.0, -0.092), (0.0244, -0.092), (0.0250, -0.0914), (0.0250, -0.0560), (0.0310, -0.0500),
                        (0.0310, 0.0500), (0.0250, 0.0560), (0.0250, 0.0914), (0.0244, 0.092), (0.0, 0.092)],
                       seg=_seg(0.031, 48)))
    yf0 = y_rb - 0.012
    adds.append(_lathe([(0.0, yf0 - 0.007), (0.0310, yf0 - 0.007), (0.0460, yf0), (0.0790, yf0), (0.0800, yf0 + 0.001),
                        (0.0800, y_rb - 0.001), (0.0790, y_rb), (0.05245, y_rb), (0.05235, y_rb + 0.0072),
                        (0.0515, y_rb + 0.008), (0.0, y_rb + 0.008)], seg=_seg(0.08, 64)))
    adds.append(_cyl(0.0130, -L_b, L_b, axis="Z", chamfer=0.0008, seg=32))
    for k in range(4):
        a = (k + 0.5) * math.pi / 2
        adds.append(_cyl(0.0068, -0.028, 0.032, at=(BOLT_BOSS_R * math.sin(a), 0.0, BOLT_BOSS_R * math.cos(a)),
                         chamfer=0.0008, seg=24))
    _bool(ball, adds, "UNION")
    cut = [_lathe([(R_ci * math.sin(a), -R_ci * math.cos(a)) for a in np.linspace(0.0, math.pi, n_sph + 1)],
                  seg=max(_seg(R_ci, 48), 96)),
           _cyl(0.0203, -0.0485, 0.0485, seg=48), _cyl(0.0158, -0.10, 0.10, seg=40),
           _cyl(PIN_BORE_R, -0.08, 0.08, axis="Z", seg=32),
           _cyl(0.0031, -0.011, 0.020, at=(0.0, 0.0, LOCK_Z), seg=16)]
    for s in (1, -1):
        cut.append(_box((0.050, 0.048, 0.042), (s * 0.045, 0.0, 0.0), radius=0.006, segments=3))
    for k in range(4):
        a = (k + 0.5) * math.pi / 2
        cut.append(_cyl(0.0042, -0.023, 0.040, at=(BOLT_BOSS_R * math.sin(a), 0.0, BOLT_BOSS_R * math.cos(a)), seg=16))
    for a in _ring_bolt_angles(in_ring=False):
        cut.append(_cyl(0.0053, yf0 - 0.01, y_rb + 0.001, at=(RING_BOLT_R * math.cos(a), 0.0,
                                                               RING_BOLT_R * math.sin(a)), seg=16))
    _bool(ball, cut, "DIFFERENCE")
    _mat(ball, "cast_iron")
    left = _copy(ball)
    right = ball
    MU.cut_and_apply(left, plane=((0, 0, 0), (0, 1, 0)), space="LOCAL", section_material_name="cast_iron")
    MU.cut_and_apply(right, plane=((0, 0, 0), (0, -1, 0)), space="LOCAL", section_material_name="cast_iron")
    for ob in (left, right):
        MU.smooth_by_angle(ob, 32.0, keep_sharp=True)
    # case bolts (heads on the right half) + cross-pin retaining bolt
    b = _bolt(0.013, 0.0055, 0.0040, 0.044, flange=True)
    mats = []
    for k in range(4):
        a = (k + 0.5) * math.pi / 2
        mats.append(_T(BOLT_BOSS_R * math.sin(a), 0.032, BOLT_BOSS_R * math.cos(a)))
    bolts = _instances(b, mats)
    lock = _bolt(0.010, 0.004, 0.0030, 0.0235, washer=True)
    _xf(lock, _T(0.0, 0.0132, LOCK_Z))
    hw = _join([bolts, lock])
    _mat(hw, "steel_dark")
    right = _join([right, hw])
    return left, right, R_co, L_b


# ===========================================================================
# Pinion + bearings
# ===========================================================================

def _build_pinion(pin_gear):
    """Everything in the pinion object frame (car axes, origin at the flange face)."""
    oy = -Y_PIN_OBJ
    _xf(pin_gear, _T(0.0, APEX_P + oy, 0.0) @ Matrix.Rotation(math.pi, 4, "Z"))
    shaft = _lathe([(0.0, 0.0960 + oy), (0.0220, 0.0960 + oy), (0.0220, 0.1046 + oy), (0.0200, 0.1050 + oy),
                    (0.0200, 0.1238 + oy), (0.0188, 0.1244 + oy), (0.0188, 0.1640 + oy), (0.0150, 0.1650 + oy),
                    (0.0150, 0.1818 + oy), (0.0115, 0.1822 + oy), (0.0115, 0.2120 + oy), (0.0110, 0.2122 + oy),
                    (0.0110, 0.2188 + oy), (0.0100, 0.2195 + oy), (0.0, 0.2195 + oy)], seg=_seg(0.022, 40))
    spl = G.external_splines("axl_tmp_pspl", 24, 0.027, 0.0235, 0.0296, detail="high" if _hi() else "low",
                             link=False)
    _xf(spl, _T(0.0, 0.1970 + oy, 0.0))
    _bool(shaft, [spl], "UNION")
    pin = _join([pin_gear, shaft])
    _mat(pin, "steel_machined")
    MU.smooth_by_angle(pin, 35.0)
    # bearings
    cone_h, cup_h, rol_h, k_h = _taper_bearing(*PB_HEAD, large_end=-1)
    for ob in (cone_h, cup_h, rol_h):
        _xf(ob, _T(0.0, Y_PB_HEAD + oy, 0.0))
    cone_t, cup_t, rol_t, k_t = _taper_bearing(*PB_TAIL, large_end=+1)
    for ob in (cone_t, cup_t, rol_t):
        _xf(ob, _T(0.0, Y_PB_TAIL + oy, 0.0))
    y_sp0, y_sp1 = Y_PB_HEAD + PB_HEAD[2] + 0.0003, Y_PB_TAIL - 0.0003
    ym = 0.5 * (y_sp0 + y_sp1)
    spacer = _lathe([(0.0192, y_sp0 + oy), (0.0232, y_sp0 + oy), (0.0236, y_sp0 + 0.006 + oy),
                     (0.0252, ym - 0.004 + oy), (0.0252, ym + 0.004 + oy), (0.0236, y_sp1 - 0.006 + oy),
                     (0.0232, y_sp1 + oy), (0.0192, y_sp1 + oy)], closed=True, seg=_seg(0.025, 40), mat="steel_machined")
    # companion flange (+ yoke-bolt nuts + pinion nut)
    hub = G.internal_splines("axl_tmp_fhub", 24, 0.027, 0.0235, 0.030, outer_d=0.050,
                             detail="high" if _hi() else "low", link=False)
    _xf(hub, _T(0.0, 0.197 + oy, 0.0))
    disc = _lathe([(0.0195, Y_FL_DISC0 + oy), (0.0490, Y_FL_DISC0 + oy), (0.0500, Y_FL_DISC0 + 0.001 + oy),
                   (0.0500, 0.2190 + oy), (0.0490, 0.2200 + oy), (0.0205, 0.2200 + oy), (0.0195, 0.2190 + oy)],
                  closed=True, seg=_seg(0.05, 64))
    _bool(hub, [disc], "UNION")
    holes = [_cyl(0.0045, Y_FL_DISC0 - 0.002 + oy, 0.221 + oy, at=(UJ_BOLT_R * math.cos(a), 0.0,
                                                                   UJ_BOLT_R * math.sin(a)), seg=16)
             for a in (np.arange(4) + 0.5) * math.pi / 2]
    _bool(hub, holes, "DIFFERENCE")
    _mat(hub, "steel_machined")
    nut = _nut(0.013, 0.0065)
    _xf(nut, Matrix.Rotation(math.pi, 4, "X"))      # extends toward -Y from y=0
    nuts = _instances(nut, [_T(UJ_BOLT_R * math.cos(a), Y_FL_DISC0 + oy, UJ_BOLT_R * math.sin(a))
                            for a in (np.arange(4) + 0.5) * math.pi / 2])
    pnut = _join([_prism(_hex_poly(0.030), (1, 0, 0), (0, 0, 1), (0, 1, 0), 0.2123 + oy, 0.2193 + oy,
                         chamfer=0.0008),
                  _cyl(0.0160, 0.2120 + oy, 0.2124 + oy, seg=40)])
    hw = _join([nuts, pnut])
    _mat(hw, "steel_dark")
    flange = _join([hub, hw])
    MU.smooth_by_angle(flange, 35.0)
    seal = _lathe([(0.0251, 0.1842 + oy), (0.0262, 0.1840 + oy), (0.0349, 0.1840 + oy), (0.0349, 0.1938 + oy),
                   (0.0290, 0.1938 + oy), (0.0256, 0.1925 + oy), (0.0251, 0.1900 + oy)], closed=True,
                  seg=_seg(0.035, 48), mat="rubber")
    return dict(pinion=pin, cone_head=cone_h, cup_head=cup_h, rollers_head=rol_h, k_head=k_h,
                cone_tail=cone_t, cup_tail=cup_t, rollers_tail=rol_t, k_tail=k_t, spacer=spacer,
                flange=flange, seal=seal)


# ===========================================================================
# Housing (centre section) + cover
# ===========================================================================

def _lathe_x(prof_xr, close_axis=True, seg=None):
    """Solid of revolution about the car X axis from an (x, r) polyline."""
    pts = [(r, x) for x, r in prof_xr]
    if close_axis:
        pts = [(0.0, pts[0][1])] + pts + [(0.0, pts[-1][1])]
    return _lathe(pts, axis="X", seg=seg or _seg(0.11, 96))


def _flange_outline(grow=0.012, n=48):
    """(x, z) outline of the cover-joint flange: the pumpkin's section at the joint
    plane grown by `grow`, with rounded ends."""
    x0, x1 = OUTER_X[0][0], OUTER_X[-1][0]
    xs = np.linspace(x0, x1, n)
    zs = np.array([math.sqrt(max(0.0, (profile_r(OUTER_X, x) + grow) ** 2 - Y_JOINT ** 2)) for x in xs])
    keep = zs > 0.004
    xs, zs = xs[keep], zs[keep]
    top = list(zip(xs, zs))
    bot = list(zip(xs[::-1], -zs[::-1]))
    left = [(xs[0] - zs[0] * 0.55 * math.sin(a), zs[0] * math.cos(a)) for a in np.linspace(0.0, math.pi, 9)[1:-1]]
    right = [(xs[-1] + zs[-1] * 0.55 * math.sin(a), -zs[-1] * math.cos(a)) for a in np.linspace(0.0, math.pi, 9)[1:-1]]
    return np.array(top + right + bot + left)


def _build_housing():
    """Returns dict(housing, cover, bolts, plugs, bushings, sleeves, seal_l, seal_r, info)."""
    seg_big = _seg(0.11, 96)
    # ---------------- outer solids
    outer = _lathe_x(OUTER_X, seg=seg_big)
    _bool(outer, [_box((0.4, 0.4, 0.4), (0.0, Y_JOINT - 0.2, 0.0))], "DIFFERENCE")
    adds = []
    # pinion nose (revolved about Y)
    adds.append(_lathe([(0.0, 0.050), (0.0520, 0.050), (0.0520, 0.128), (0.0420, 0.150), (0.0420, 0.1825),
                        (0.0445, 0.1845), (0.0445, 0.1930), (0.0425, Y_NOSE_FRONT), (0.0, Y_NOSE_FRONT)],
                       seg=_seg(0.052, 64)))
    # nose ribs (4, diagonal) + mounting pad on top of the nose
    for a in (math.pi / 2, -math.pi / 2):
        e_r = np.array([math.cos(a), 0.0, math.sin(a)])
        e_t = np.array([-math.sin(a), 0.0, math.cos(a)])
        poly = [(0.040, 0.070), (0.072, 0.070), (0.064, 0.100), (0.047, 0.150), (0.040, 0.156)]
        adds.append(_prism([(u, v) for u, v in poly], e_r, (0, 1, 0), e_t, -0.0028, 0.0028, chamfer=0.0008))
    adds.append(_box((0.064, 0.044, 0.026), (0.0, 0.140, 0.046), radius=0.006, segments=2))
    # side bearing bosses + bolted side covers (both sides)
    for s in (1, -1):
        prof = [(s * 0.0560, 0.0), (s * 0.0560, 0.0540), (s * 0.0900, 0.0540), (s * 0.0900, 0.0620),
                (s * 0.0970, 0.0620), (s * 0.0970, 0.0390), (s * 0.1150, 0.0390), (s * 0.1160, 0.0380),
                (s * 0.1160, 0.0)]
        if s < 0:
            prof = prof[::-1]
        adds.append(_lathe([(r, x) for x, r in prof], axis="X", seg=_seg(0.062, 64)))
    # cover-joint flange (housing half)
    adds.append(_prism(_flange_outline(), (1, 0, 0), (0, 0, 1), (0, 1, 0), Y_JOINT, Y_JOINT + FLANGE_T,
                       chamfer=0.0012))
    # drain-plug boss (bottom) and breather boss (top)
    x_dr, y_dr = -0.022, 0.018
    z_dr = -math.sqrt(profile_r(OUTER_X, x_dr) ** 2 - y_dr ** 2)
    adds.append(_cyl(0.0125, -0.012, 0.006, axis="-Z", at=(x_dr, y_dr, z_dr), chamfer=0.0015, seg=32))
    x_br, y_br = 0.034, 0.024
    z_br = math.sqrt(profile_r(OUTER_X, x_br) ** 2 - y_br ** 2)
    adds.append(_cyl(0.0075, -0.010, 0.006, axis="Z", at=(x_br, y_br, z_br), chamfer=0.001, seg=24))
    housing = outer
    _bool(housing, adds, "UNION")
    # ---------------- cover
    cov = _lathe_x(OUTER_X, seg=seg_big)
    _bool(cov, [_box((0.4, 0.4, 0.4), (0.0, Y_JOINT + 0.2, 0.0))], "DIFFERENCE")
    cadds = [_prism(_flange_outline(), (1, 0, 0), (0, 0, 1), (0, 1, 0), Y_JOINT - FLANGE_T, Y_JOINT,
                    chamfer=0.0012)]
    # vertical cooling fins on the dome
    for xk in (-0.052, -0.034, -0.016, 0.002, 0.020):
        r0 = profile_r(OUTER_X, xk)
        y_lim = -(abs(Y_JOINT) + FLANGE_T + 0.004)
        if r0 < abs(y_lim) + 0.012:
            continue
        phi = math.acos(abs(y_lim) / (r0 + 0.006))
        n = 14
        ang = np.linspace(-phi, phi, n)
        ri, rf = r0 - 0.004, r0 + 0.0065
        pts = [(-ri * math.cos(a), ri * math.sin(a)) for a in ang]
        pts += [(-rf * math.cos(a), rf * math.sin(a)) for a in ang[::-1]]
        cadds.append(_prism(pts, (0, 1, 0), (0, 0, 1), (1, 0, 0), xk - 0.00175, xk + 0.00175, chamfer=0.0007))
    # mounting ears (lugs with vertical rubber bushes) on the upper corners
    ears = []
    for xb, poly in ((-0.128, [(-0.010, 0.090), (-0.080, 0.097), (-0.122, 0.097), (-0.122, 0.058), (-0.092, 0.054),
                               (-0.048, 0.040), (-0.010, 0.050)]),
                     (0.118, [(0.000, 0.090), (0.070, 0.097), (0.112, 0.097), (0.112, 0.058), (0.082, 0.052),
                              (0.040, 0.030), (0.000, 0.050)])):
        # cast lug: tapered arm with a gusset running down into the dome / flange
        cadds.append(_prism(poly, (1, 0, 0), (0, 0, 1), (0, 1, 0), Y_JOINT - 0.030, Y_JOINT - 0.004,
                            chamfer=0.0032))
        cadds.append(_cyl(0.0215, 0.054, 0.099, axis="Z", at=(xb, Y_JOINT - 0.017, 0.0), chamfer=0.002, seg=40))
        ears.append((xb, Y_JOINT - 0.017))
    # filler-plug boss on the back of the dome
    x_fp = 0.022
    y_fp = -profile_r(OUTER_X, x_fp)
    cadds.append(_cyl(0.0125, -0.010, 0.005, axis="-Y", at=(x_fp, y_fp, -0.012), chamfer=0.0015, seg=32))
    _bool(cov, cadds, "UNION")
    # ---------------- cavity + holes
    def cavity_set():
        c = [_lathe_x(CAVITY_X, seg=seg_big)]
        # pinion nose bores (head cup, shoulder, tail cup, seal)
        c.append(_lathe([(0.0, 0.020), (0.0400, 0.020), (0.0400, Y_PB_HEAD + PB_HEAD[2]), (0.0290, Y_PB_HEAD + PB_HEAD[2]),
                         (0.0290, Y_PB_TAIL), (0.0310, Y_PB_TAIL), (0.0310, 0.1830), (0.0350, 0.1830),
                         (0.0350, Y_NOSE_FRONT + 0.01), (0.0, Y_NOSE_FRONT + 0.01)], seg=_seg(0.04, 64)))
        for s in (1, -1):
            prof = [(s * 0.050, 0.0), (s * 0.050, 0.0400), (s * (X_CB + CB[2]), 0.0400),
                    (s * (X_CB + CB[2]), 0.0320), (s * 0.1000, 0.0320), (s * 0.1000, 0.0262),
                    (s * 0.1300, 0.0262), (s * 0.1300, 0.0)]
            if s < 0:
                prof = prof[::-1]
            c.append(_lathe([(r, x) for x, r in prof], axis="X", seg=_seg(0.04, 64)))
        return c
    hcut = cavity_set()
    # cover bolt holes (through both flanges), side-cover bolt holes, plug/breather holes, pad bolt holes
    bolt_pts = []
    for xk in (-0.062, -0.036, -0.010, 0.016, 0.040):
        zk = math.sqrt((profile_r(OUTER_X, xk) + 0.0055) ** 2 - Y_JOINT ** 2)
        bolt_pts += [(xk, zk), (xk, -zk)]
    for (xk, zk) in bolt_pts:
        hcut.append(_cyl(0.0042, -0.012, 0.012, at=(xk, Y_JOINT, zk), seg=16))
    side_bolts = []
    for s in (1, -1):
        for k in range(6):
            a = (k + 0.5) * TAU / 6
            p = (s * 0.0935, 0.0560 * math.cos(a), 0.0560 * math.sin(a))
            side_bolts.append((s, p))
            hcut.append(_cyl(0.0034, -0.0040, 0.006, axis="X" if s > 0 else "-X", at=p, seg=12))
    hcut.append(_cyl(0.0080, -0.020, 0.030, axis="-Z", at=(x_dr, y_dr, z_dr), seg=24))
    hcut.append(_cyl(0.0030, -0.030, 0.020, axis="Z", at=(x_br, y_br, z_br), seg=12))
    pad_bolts = [(-0.020, 0.140), (0.020, 0.140)]
    for (xp, yp) in pad_bolts:
        hcut.append(_cyl(0.0045, -0.012, 0.012, axis="Z", at=(xp, yp, 0.059), seg=16))
    _bool(housing, hcut, "DIFFERENCE")
    _mat(housing, "cast_iron")
    MU.smooth_by_angle(housing, 32.0, keep_sharp=True)
    ccut = cavity_set()
    for (xk, zk) in bolt_pts:
        ccut.append(_cyl(0.0045, -0.012, 0.012, at=(xk, Y_JOINT, zk), seg=16))
    for (xb, yb) in ears:
        ccut.append(_cyl(0.0182, 0.040, 0.110, axis="Z", at=(xb, yb, 0.0), seg=40))
    ccut.append(_cyl(0.0080, -0.030, 0.012, axis="-Y", at=(x_fp, y_fp, -0.012), seg=24))
    _bool(cov, ccut, "DIFFERENCE")
    _mat(cov, "cast_aluminium")
    MU.smooth_by_angle(cov, 32.0, keep_sharp=True)
    # ---------------- bolts, plugs, bushings, seals
    b = _bolt(0.013, 0.0055, 0.0040, 0.0150, flange=True)
    mats = [_T(xk, Y_JOINT - FLANGE_T, zk) @ Matrix.Rotation(math.pi, 4, "Z") for (xk, zk) in bolt_pts]
    cover_bolts = _instances(b, mats)
    b = _bolt(0.010, 0.0045, 0.0030, 0.0065, flange=True)
    mats = []
    for s, p in side_bolts:
        mats.append(_T(*p) @ _T(s * (0.0970 - 0.0935), 0.0, 0.0) @ _axis_rot("X" if s > 0 else "-X"))
    sbolts = _instances(b, mats)
    b = _bolt(0.016, 0.0065, 0.0050, 0.0160, flange=True)
    pbolts = _instances(b, [_T(xp, yp, 0.059) @ _axis_rot("Z") for (xp, yp) in pad_bolts])
    bolts = _join([cover_bolts, sbolts, pbolts])
    _mat(bolts, "steel_dark")
    # drain plug (hex socket), filler plug, breather cap
    def plug():
        head = _cyl(0.0105, 0.0, 0.0050, chamfer=0.0008, seg=32)
        sock = _prism(_hex_poly(0.008), (1, 0, 0), (0, 0, 1), (0, 1, 0), 0.0015, 0.006)
        _bool(head, [sock], "DIFFERENCE")
        stem = _cyl(0.0078, -0.012, 0.0005, seg=24)
        return _join([head, stem])
    dp = plug()
    _xf(dp, _T(x_dr, y_dr, z_dr - 0.006) @ _axis_rot("-Z"))
    fp = plug()
    _xf(fp, _T(x_fp, y_fp - 0.005, -0.012) @ _axis_rot("-Y"))
    br = _join([_cyl(0.0029, -0.010, 0.012, seg=12), _cyl(0.0060, 0.012, 0.0185, chamfer=0.0012, seg=24)])
    _xf(br, _T(x_br, y_br, z_br + 0.006) @ _axis_rot("Z"))
    plugs = _join([dp, fp, br])
    _mat(plugs, "steel_dark")
    bush = _cyl(0.0182, 0.0, 0.040, inner=0.0092, chamfer=0.0015, seg=40)
    bushes = _instances(bush, [_T(xb, yb, 0.056) for (xb, yb) in ears])
    _mat(bushes, "rubber")
    sl = _cyl(0.0091, 0.0, 0.046, inner=0.0058, chamfer=0.0006, seg=32)
    sleeves = _instances(sl, [_T(xb, yb, 0.053) for (xb, yb) in ears])
    _mat(sleeves, "steel_machined")
    seals = []
    for s in (1, -1):
        prof = [(0.0171, 0.1055), (0.0181, 0.1050), (0.0261, 0.1050), (0.0261, 0.1155), (0.0215, 0.1155),
                (0.0176, 0.1140), (0.0171, 0.1110)]
        sob = _lathe(prof, closed=True, seg=_seg(0.026, 48), mat="rubber")
        _xf(sob, _axis_rot("X" if s > 0 else "-X"))
        seals.append(sob)
    info = dict(bolt_pts=bolt_pts, ears=ears, drain=(x_dr, y_dr, z_dr), filler=(x_fp, y_fp, -0.012),
                breather=(x_br, y_br, z_br))
    return dict(housing=housing, cover=cov, bolts=bolts, plugs=plugs, bushings=bushes, sleeves=sleeves,
                seal_r=seals[0], seal_l=seals[1], info=info)


# ===========================================================================
# Propeller shaft
# ===========================================================================

def _ear_tabs(ear_axis, side, y_base, y_tip_r=UJ_TAB_R, w_base=0.032):
    """Two yoke ears holding a cross arm along `ear_axis` ('X'|'Z').  Built for 'Z'
    (plates perpendicular to Z) and turned about Y for 'X'.  The tabs run from the bore
    (y = 0) to y = side*y_base."""
    parts = []
    for s in (1, -1):
        n = 12
        arc = [(y_tip_r * math.cos(a), -side * y_tip_r * math.sin(a)) for a in np.linspace(0.0, math.pi, n)]
        # outline in (x, y): semicircle on the far side, straight sides to the base
        poly = [(w_base / 2, side * y_base)] + arc[:] + [(-w_base / 2, side * y_base)]
        hole = _circle(UJ_BORE_R, 32)
        ob = _prism(poly, (1, 0, 0), (0, 1, 0), (0, 0, 1), min(s * UJ_EAR_IN, s * UJ_EAR_OUT),
                    max(s * UJ_EAR_IN, s * UJ_EAR_OUT), holes=[hole], chamfer=0.0012)
        parts.append(ob)
    ears = _join(parts)
    if ear_axis == "X":
        _xf(ears, Matrix.Rotation(math.pi / 2, 4, "Y"))
    return ears


def _flange_yoke(side, extra_len=0.0):
    """Flange yoke: ears along Z, flange disc on the side*Y side (face at side*(Y0+T))."""
    y0, y1 = UJ_FLANGE_Y0, UJ_FLANGE_Y0 + UJ_FLANGE_T
    ears = _ear_tabs("Z", side, y0 + 0.002)
    prof = [(0.0, 0.020), (0.0200, 0.020), (0.0215, 0.0215), (0.0240, y0 - 0.002), (UJ_FLANGE_R - 0.004, y0),
            (UJ_FLANGE_R - 0.001, y0), (UJ_FLANGE_R, y0 + 0.001), (UJ_FLANGE_R, y1 - 0.001),
            (UJ_FLANGE_R - 0.001, y1), (0.0, y1)]
    if side < 0:
        prof = [(r, -y) for r, y in prof]
    disc = _lathe(prof, seg=_seg(UJ_FLANGE_R, 64))
    _bool(disc, [ears], "UNION")
    holes = [_cyl(0.0045, -0.003, UJ_FLANGE_T + 0.003, at=(UJ_BOLT_R * math.cos(a), side * y0, UJ_BOLT_R * math.sin(a)),
                  seg=16) for a in (np.arange(4) + 0.5) * math.pi / 2]
    if side < 0:
        for h in holes:
            _xf(h, _T(0.0, -UJ_FLANGE_T, 0.0))
    _bool(disc, holes, "DIFFERENCE")
    _mat(disc, "steel_forged")
    # bolts: heads on the yoke side of the flange, shank through (and beyond: extra_len)
    b = _bolt(0.013, 0.0055, 0.0040, UJ_FLANGE_T + extra_len, flange=True)
    mats = []
    for a in (np.arange(4) + 0.5) * math.pi / 2:
        M = _T(UJ_BOLT_R * math.cos(a), side * y0, UJ_BOLT_R * math.sin(a))
        mats.append(M if side < 0 else M @ Matrix.Rotation(math.pi, 4, "X"))
    bolts = _instances(b, mats)
    _mat(bolts, "steel_dark")
    ob = _join([disc, bolts])
    MU.smooth_by_angle(ob, 35.0)
    return ob


def _cross():
    """Cross (spider) of a Hooke joint: arms along local X and Z, needle cups, seals."""
    body = _box((2 * UJ_BODY, 0.024, 2 * UJ_BODY), (0.0, 0.0, 0.0), radius=0.004, segments=3)
    parts = [body]
    caps, seals = [], []
    for ax in ("X", "-X", "Z", "-Z"):
        parts.append(_cyl(UJ_TRUNNION_R, 0.0, UJ_EAR_IN - 0.001, axis=ax, seg=24))
        seals.append(_cyl(0.0108, UJ_EAR_IN - 0.0035, UJ_EAR_IN - 0.0008, axis=ax, chamfer=0.0008, seg=32))
        cap = _lathe([(0.0, UJ_CAP_OUT), (UJ_CAP_R - 0.0008, UJ_CAP_OUT), (UJ_CAP_R, UJ_CAP_OUT - 0.0008),
                      (UJ_CAP_R, UJ_EAR_IN - 0.0008), (0.0, UJ_EAR_IN - 0.0008)], seg=32)
        clip = _cyl(0.0134, UJ_EAR_OUT + 0.0001, UJ_EAR_OUT + 0.0011, inner=UJ_CAP_R - 0.0012, seg=32)
        c = _join([cap, clip])
        _xf(c, _axis_rot(ax))
        caps.append(c)
    nip = _join([_cyl(0.0022, 0.0, 0.006, seg=12), _prism(_hex_poly(0.006), (1, 0, 0), (0, 0, 1), (0, 1, 0),
                                                          0.002, 0.0045), _cyl(0.0018, 0.006, 0.0085, seg=12)])
    _xf(nip, _frame((1 / math.sqrt(2), 0.0, 1 / math.sqrt(2)), (0.0085, 0.0, 0.0085), x_hint=(0, 1, 0)))
    b = _join(parts)
    _mat(b, "steel_machined")
    cp = _join(caps)
    _mat(cp, "steel_ground")
    sl = _join(seals)
    _mat(sl, "rubber")
    _mat(nip, "brass")
    ob = _join([b, cp, sl, nip])
    MU.smooth_by_angle(ob, 35.0)
    return ob


def _slip_yoke():
    ears = _ear_tabs("X", -1, 0.028)
    base = _lathe([(0.0155, -0.0200), (0.0300, -0.0200), (0.0380, -0.0240), (0.0380, -0.0320), (0.0240, -0.0460),
                   (0.0230, -0.0500), (0.0230, -0.1030), (0.0215, -0.1050), (0.0155, -0.1050)], closed=True,
                  seg=_seg(0.038, 64))
    _bool(base, [ears], "UNION")
    spl = G.internal_splines("axl_tmp_sspl", 22, 0.031, 0.0275, 0.050, outer_d=0.0345,
                             detail="high" if _hi() else "low", link=False)
    _xf(spl, _T(0.0, -0.078, 0.0))
    _bool(base, [spl], "UNION")
    _mat(base, "steel_forged")
    MU.smooth_by_angle(base, 35.0)
    return base


def _boot():
    y0, y1 = -0.1045, -0.1530
    n = 5
    pts = [(0.0, y0), (0.0218, y0)]
    N = 8 * n
    for i in range(1, N):
        t = i / N
        y = y0 + (y1 - y0) * t
        rb = 0.0220 + (0.0172 - 0.0220) * t
        pts.append((rb + 0.0034 * math.sin(math.pi * n * t) ** 2, y))
    pts += [(0.0172, y1), (0.0, y1)]
    boot = _lathe(pts, seg=_seg(0.025, 48), mat="rubber")
    c1 = _cyl(0.0236, -0.1110, -0.1055, inner=0.0210, chamfer=0.0005, seg=48)
    c2 = _cyl(0.0190, -0.1525, -0.1470, inner=0.0165, chamfer=0.0005, seg=48)
    clamps = _join([c1, c2])
    _mat(clamps, "steel_machined")
    ob = _join([boot, clamps])
    MU.smooth_by_angle(ob, 40.0)
    return ob


def _tube():
    yr = -(PROP_L - 0.036)
    prof = [(0.0, -0.1400), (0.0160, -0.1400), (0.0160, -0.1555), (0.0190, -0.1620), (0.0255, -0.1700),
            (0.0310, -0.1790), (TUBE_R, -0.1840), (TUBE_R, yr), (0.0, yr)]
    tube = _lathe(prof, seg=_seg(TUBE_R, 64))
    beads = []
    for yb in (-0.1835, yr - 0.0004):
        beads.append(_lathe([(TUBE_R - 0.0004 + 0.0014 * math.sin(a), yb + 0.0022 * math.cos(a))
                             for a in np.linspace(0.0, math.pi, 7)], closed=True, seg=_seg(TUBE_R, 64)))
    tb = _join(beads)
    _mat(tube, "steel_dark")
    _mat(tb, "steel_dark")
    ob = _join([tube, tb])
    MU.smooth_by_angle(ob, 40.0)
    return ob


def _weights():
    out = []
    for (y, ang, w) in ((-0.230, 35.0, 0.022), (-0.250, 205.0, 0.016), (-(PROP_L - 0.110), 120.0, 0.020)):
        a = math.radians(ang)
        pl = _box((w, 0.016, 0.0032), (0.0, 0.0, TUBE_R + 0.0012), radius=0.0012, segments=2)
        _xf(pl, _T(0.0, y, 0.0) @ Matrix.Rotation(-a, 4, "Y"))
        out.append(pl)
    ob = _join(out)
    _mat(ob, "steel_dark")
    return ob


def _tube_yoke():
    ears = _ear_tabs("X", 1, 0.028)
    base = _lathe([(0.0, 0.0200), (0.0300, 0.0200), (0.0380, 0.0240), (0.0380, 0.0320), (0.0340, 0.0360),
                   (0.0300, 0.0362), (0.0300, 0.0500), (0.0, 0.0500)], seg=_seg(0.038, 64))
    _bool(base, [ears], "UNION")
    _mat(base, "steel_forged")
    MU.smooth_by_angle(base, 35.0)
    return base


# ===========================================================================
# build
# ===========================================================================

def build(opts=None):
    t_start = time.time()
    opts = dict(opts or {})
    cutaways = opts.get("cutaways")
    if cutaways is None:
        cutaways = [opts["cutaway"]] if "cutaway" in opts else ["none", "half"]
    cutaways = list(dict.fromkeys(cutaways))
    for v in cutaways:
        if v not in ("none", "half"):
            raise ValueError(f"axle: unknown cutaway {v!r}")
    _C.detail = opts.get("detail", "high")
    _C.col = rig.collection(opts.get("collection", "axle"))
    _C.parts = parts = {}
    with_prop = opts.get("prop", True)

    root = rig.empty(_nm("root"), loc=ROOT_LOC, col=_C.col, size=0.15)
    root.rotation_mode = "XYZ"
    D = rig.transverse_pivot(_nm("diff_pivot"), loc=(0, 0, 0), parent=root, col=_C.col)
    parts["diff_pivot"] = D

    # ------------------------------------------------------------------ gears
    g = _build_gears()
    R_ci = max(g["Rb_sp"], g["Rb_sd"]) + 0.0010
    y_rb = -APEX_R + g["ring_back"]            # D-local y (= car x) of the ring back face

    # ------------------------------------------------------------------ case + ring
    case_l, case_r, R_co, L_pin = _build_case(R_ci, y_rb)
    x_case_l = _empty("x_case_left", D, size=0.02)
    x_case_r = _empty("x_case_right", D, size=0.02)
    _own(case_l, "case_left", parent=x_case_l)
    _own(case_r, "case_right", parent=x_case_r)
    x_ring = _empty("x_ring", x_case_l, size=0.02)
    ring = _own(g["ring"], "ring_gear", "steel_machined", parent=x_ring)
    ring.location = (0.0, -APEX_R, 0.0)
    rb = _bolt(0.016, 0.0065, 0.0050, 0.012 + 0.012, flange=True)
    ring_bolts = _instances(rb, [_T(RING_BOLT_R * math.cos(a), g["ring_back"] - 0.012, RING_BOLT_R * math.sin(a))
                                 @ Matrix.Rotation(math.pi, 4, "X") for a in _ring_bolt_angles(in_ring=True)])
    _own(ring_bolts, "ring_bolts", "steel_dark", parent=ring)

    # ------------------------------------------------------------------ pin, spiders, side gears
    cluster = _empty("diff_cluster", D, size=0.02)
    x_pin = _empty("x_cross_pin", cluster, size=0.02)
    pin = _cyl(PIN_R, -(L_pin - 0.0003), L_pin - 0.0003, axis="Z", chamfer=0.0007, seg=40)
    _bool(pin, [_cyl(0.0032, -0.02, 0.02, at=(0.0, 0.0, LOCK_Z), seg=16)], "DIFFERENCE")
    _own(pin, "cross_pin", "steel_ground", parent=x_pin, smooth=35.0)
    for k in (1, 2):
        M = M_SPIDER[k - 1]
        xs = _empty(f"x_spider_{k}", cluster, loc=tuple(M[:3, 3]), rot=tuple(_M(M).to_euler("XYZ")), size=0.01)
        sp = g["sp"] if k == 1 else _copy(g["sp"])
        _own(sp, f"spider_{k}", "steel_machined", parent=xs)
        w = _sphere_washer(APEX_SP, g["Rb_sp"] + 0.00015, R_ci - 0.00015, SPIDER_BORE / 2 + 0.0008, g["rB_sp"] - 0.0003)
        _own(w, f"spider_washer_{k}", parent=xs)
    sd_y0 = g["sd_y"][0]
    for side, M, sgn in (("left", M_SIDE_L, -1), ("right", M_SIDE_R, 1)):
        loc, rot = tuple(M[:3, 3]), tuple(_M(M).to_euler("XYZ"))
        xs = _empty(f"x_side_{side}", D, loc=loc, rot=rot, size=0.01)
        sd = g["sd"] if side == "left" else _copy(g["sd"])
        _own(sd, f"side_gear_{side}", "steel_machined", parent=xs)
        w = _sphere_washer(APEX_SD, g["Rb_sd"] + 0.00015, R_ci - 0.00015, 0.0205, g["rB_sd"] - 0.0003)
        _own(w, f"side_washer_{side}", parent=xs)
        xst = _empty(f"x_stub_{side}", D, loc=loc, rot=rot, size=0.01)
        _own(_stub(sd_y0), f"stub_{side}", parent=xst)

    # ------------------------------------------------------------------ carrier bearings
    cage = {}
    for side, s in (("left", -1), ("right", 1)):
        cone, cup, rol, k = _taper_bearing(*CB, large_end=+1)
        # bearing frame y in [0, B] (large end at +y) -> D-local y = s*(X_CB + y)
        F = _T(0.0, s * X_CB, 0.0) @ (Matrix.Identity(4) if s > 0 else Matrix.Rotation(math.pi, 4, "Z"))
        for ob in (cone, cup, rol):
            _xf(ob, F)
        _own(cone, f"carrier_cone_{side}", parent=parts[f"case_{side}"])
        _own(rol, f"carrier_rollers_{side}", parent=parts[f"x_case_{side}"])
        _own(cup, f"carrier_cup_{side}", parent=D)
        cage["carrier"] = k

    # ------------------------------------------------------------------ pinion
    pp = _build_pinion(g["pin"])
    pin_ob = _own(pp["pinion"], "pinion", parent=root)
    pin_ob.location = (0.0, Y_PIN_OBJ, 0.0)
    _own(pp["flange"], "companion_flange", parent=pin_ob)
    _own(pp["spacer"], "pinion_spacer", parent=pin_ob)
    _own(pp["cone_head"], "pinion_cone_head", parent=pin_ob)
    _own(pp["cone_tail"], "pinion_cone_tail", parent=pin_ob)
    for key in ("rollers_head", "rollers_tail", "cup_head", "cup_tail", "seal"):
        ob = _own(pp[key], f"pinion_{key}", parent=root)
        ob.location = (0.0, Y_PIN_OBJ, 0.0)
    cage["pinion_head"], cage["pinion_tail"] = pp["k_head"], pp["k_tail"]

    # ------------------------------------------------------------------ housing
    hs = _build_housing()
    _own(hs["housing"], "housing", parent=root)
    _own(hs["cover"], "cover", parent=root)
    _own(hs["bolts"], "housing_bolts", parent=root, smooth=35.0)
    _own(hs["plugs"], "plugs", parent=root, smooth=35.0)
    _own(hs["bushings"], "bushings", parent=root, smooth=35.0)
    _own(hs["sleeves"], "bushing_sleeves", parent=root, smooth=35.0)
    _own(hs["seal_l"], "seal_left", parent=root)
    _own(hs["seal_r"], "seal_right", parent=root)

    # ------------------------------------------------------------------ propshaft
    if with_prop:
        ff = _empty("prop_front_frame", root, loc=tuple(J1), size=0.03)
        _own(_flange_yoke(+1), "prop_flange_front", parent=ff)
        spf = _empty("prop_spin_front", ff, size=0.02)
        _own(_cross(), "prop_cross_front", parent=spf)
        tf = _empty("prop_tube_frame", root, loc=tuple(J1), rot=(PROP_BETA, 0.0, 0.0), size=0.03)
        tube = _own(_tube(), "prop_tube", parent=tf)
        _own(_slip_yoke(), "prop_slip_yoke", parent=tube)
        _own(_boot(), "prop_boot", parent=tube)
        _own(_weights(), "prop_weights", parent=tube, smooth=30.0)
        ty = _tube_yoke()
        _xf(ty, _T(0.0, -PROP_L, 0.0))
        _own(ty, "prop_tube_yoke", parent=tube)
        rf = _empty("prop_rear_frame", root, loc=tuple(J2), size=0.03)
        _own(_flange_yoke(-1, extra_len=0.012 + 0.0085), "prop_flange_rear", parent=rf)
        spr = _empty("prop_spin_rear", rf, size=0.02)
        _own(_cross(), "prop_cross_rear", parent=spr)

    for ob in list(parts.values()):
        if ob.type == "MESH":
            materials.ensure_props(ob)

    # ------------------------------------------------------------------ cutaways
    pieces = _make_cutaways(cutaways)

    # ------------------------------------------------------------------ assembly
    asm = rig.Assembly(name="axle", root=root, parts=parts)
    asm.explode = {k: v for k, v in EXPLODE.items() if k in parts}
    asm.anchors = _anchors(parts, with_prop)
    prop_names = [k for k in ("prop_flange_front", "prop_cross_front", "prop_tube", "prop_cross_rear",
                              "prop_flange_rear") if k in parts]
    diff_names = ["companion_flange", "pinion", "ring_gear", "case_left", "case_right", "cross_pin", "spider_1",
                  "spider_2", "side_gear_left", "side_gear_right", "stub_left", "stub_right"]
    housing_keys = ["housing", "cover", "housing_bolts", "plugs", "bushings", "bushing_sleeves", "seal_left",
                    "seal_right", "pinion_cup_head", "pinion_cup_tail", "pinion_seal", "carrier_cup_left",
                    "carrier_cup_right"]
    asm.meta = dict(
        power_path=prop_names + diff_names,
        power_groups={"prop": prop_names, "diff": diff_names},
        groups=dict(
            prop=[k for k in parts if k.startswith("prop_") and parts[k].type == "MESH"],
            pinion=["pinion", "companion_flange", "pinion_spacer", "pinion_cone_head", "pinion_cone_tail",
                    "pinion_rollers_head", "pinion_rollers_tail"],
            final_drive=["pinion", "ring_gear"],
            differential=["case_left", "case_right", "cross_pin", "spider_1", "spider_2", "spider_washer_1",
                          "spider_washer_2", "side_gear_left", "side_gear_right", "side_washer_left",
                          "side_washer_right", "ring_gear", "ring_bolts"],
            case=["case_left", "case_right"],
            bearings=["pinion_cone_head", "pinion_cone_tail", "pinion_rollers_head", "pinion_rollers_tail",
                      "pinion_cup_head", "pinion_cup_tail", "carrier_cone_left", "carrier_cone_right",
                      "carrier_rollers_left", "carrier_rollers_right", "carrier_cup_left", "carrier_cup_right"],
            housing=housing_keys,
            stubs=["stub_left", "stub_right"]),
        cutaway_pieces=pieces,
        variants=list(cutaways),
        prop=dict(J1=tuple(J1 + np.array(ROOT_LOC)), J2=tuple(J2 + np.array(ROOT_LOC)), beta=PROP_BETA,
                  beta_deg=math.degrees(PROP_BETA), length=PROP_L, joint_offset=JOFF),
        final_drive=dict(module=FD_MODULE, spiral_deg=35.0, ring_phase=PH_RING, apex_pinion=APEX_P,
                         apex_ring=APEX_R, ring_side="-X", ratio=S.FINAL_DRIVE),
        diff=dict(module=DIFF_MODULE, pressure_angle_deg=22.5, side_phase=PH_SIDE, spider_sign=SPIDER_SIGN,
                  case_inner_R=R_ci, case_outer_R=R_co, sphere_back=(g["Rb_sp"], g["Rb_sd"]),
                  apex_spider=APEX_SP, apex_side=APEX_SD),
        cage=cage,
        output_flange=dict(x=X_OUT, od=0.094, pcd=0.074, holes=6),
        dims=dict(ring_back_x=y_rb, carrier_bearing_x=(X_CB, X_CB + CB[2]), cover_joint_y=Y_JOINT + S.Y_DIFF,
                  pinion_bearings_y=(Y_PB_HEAD + S.Y_DIFF, Y_PB_TAIL + S.Y_DIFF)),
        housing_info=hs["info"],
    )
    for g, keys in asm.meta["groups"].items():
        asm.meta["groups"][g] = [k for k in keys if k in parts]
    # explode rest positions recorded now (Assembly.bake_explode reads meta['_rest_loc'])
    asm.meta["_rest_loc"] = {k: tuple(parts[k].location) for k in asm.explode}
    asm._driver = _driver
    asm.meta["build_time"] = time.time() - t_start
    return asm


def _stub(sd_y0):
    """Output stub in the side-gear frame (local +Y toward the apex, outboard = -Y):
    splined end in the side gear, journal through the case and the seal, 6-bolt flange
    whose outer face is at |x| = X_DIFF_OUTPUT."""
    def ly(ax):            # |x| (car) -> local y
        return APEX_SD - ax
    spl = G.external_splines("axl_tmp_stspl", SPLINE_N, SPLINE_DMAJ, SPLINE_DMIN, 0.030,
                             detail="high" if _hi() else "low", link=False)
    _xf(spl, _T(0.0, ly(0.031), 0.0))
    body = _lathe([(0.0, ly(0.0455)), (0.0150, ly(0.0455)), (0.0150, ly(0.0935)), (0.0170, ly(0.0945)),
                   (0.0170, ly(0.1190)), (0.0215, ly(0.1210)), (0.0260, ly(0.1250)), (0.0260, ly(0.1370)),
                   (0.0460, ly(0.1380)), (0.0470, ly(0.1390)), (0.0470, ly(X_OUT - 0.001)), (0.0460, ly(X_OUT)),
                   (0.0, ly(X_OUT))], seg=_seg(0.047, 64))
    _bool(body, [spl], "UNION")
    holes = [_cyl(0.0045, ly(X_OUT) - 0.002, ly(0.1375), at=(0.037 * math.cos(a), 0.0, 0.037 * math.sin(a)), seg=16)
             for a in np.arange(6) * TAU / 6]
    holes.append(_cyl(0.0100, ly(X_OUT) - 0.002, ly(X_OUT) + 0.004, seg=32))   # centring recess
    _bool(body, holes, "DIFFERENCE")
    _mat(body, "steel_machined")
    MU.smooth_by_angle(body, 35.0)
    return body


# ===========================================================================
# Cutaway pieces
# ===========================================================================

CUT_KEYS = ("housing", "cover", "housing_bolts", "plugs", "bushings", "bushing_sleeves", "seal_left", "seal_right",
            "pinion_cup_head", "pinion_cup_tail", "pinion_seal", "carrier_cup_left", "carrier_cup_right")


def _world_verts_local(ob):
    """Vertices in ROOT-local coordinates (parent chain below the root)."""
    me = ob.data
    co = np.zeros(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    M = Matrix.Identity(4)
    chain = []
    o = ob
    while o is not None and o.name != _nm("root"):
        chain.append(o)
        o = o.parent
    for o in reversed(chain):
        M = M @ o.matrix_basis
    A = np.array(M)
    return co @ A[:3, :3].T + A[:3, 3], M


def _n_nonmanifold(ob):
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    n = sum(1 for e in bm.edges if not e.is_manifold)
    bm.free()
    return n


def _cut_piece(ob, co, no):
    """Copy of ob with the half-space on the +no side removed; the section faces get
    'section_cut'.  Manifold boolean with a box (robust on the boolean-built castings),
    falling back to meshutil.cut_and_apply (retried with micron plane offsets)."""
    no = np.asarray(no, dtype=float)
    no = no / np.linalg.norm(no)
    # a few microns off the exact plane: lathe vertices lying exactly on it leave
    # zero-length edges in the boolean result
    for eps in (1.3e-5, -1.7e-5, 3.1e-5):
        c = _copy(ob)
        c.parent = None                          # booleans work in world space: use local coords
        c.matrix_basis = Matrix.Identity(4)
        box = _box((2.0, 2.0, 1.0), (0.0, 0.0, 0.0))
        _xf(box, _frame(no, np.asarray(co) + (0.5 + eps) * no, x_hint=(1.0, 0.0, 0.0))
            @ Matrix.Rotation(-math.pi / 2, 4, "X"))
        _mat(box, "section_cut")
        if _bool(c, [box], "DIFFERENCE", report=False) and _n_nonmanifold(c) == 0:
            me = c.data
            sec = [i for i, m in enumerate(me.materials) if m is not None and m.name == "section_cut"]
            if sec:
                mi = np.zeros(len(me.polygons), dtype=np.int32)
                me.polygons.foreach_get("material_index", mi)
                flat = np.isin(mi, sec)
                sm = np.zeros(len(me.polygons), dtype=bool)
                me.polygons.foreach_get("use_smooth", sm)
                me.polygons.foreach_set("use_smooth", sm & ~flat)
            return c
        d = c.data
        bpy.data.objects.remove(c)
        bpy.data.meshes.remove(d)
    print(f"[axle] cutaway of {ob.name}: Manifold cut failed, using a plane cut")
    best = None
    for eps in (0.0, 1.3e-5, -1.7e-5, 3.1e-5, -4.3e-5):
        c = _copy(ob)
        MU.cut_and_apply(c, plane=(tuple(np.asarray(co) + eps * no), tuple(no)), space="LOCAL")
        n = _n_nonmanifold(c)
        if best is None or n < best[0]:
            if best is not None:
                d = best[1].data
                bpy.data.objects.remove(best[1])
                bpy.data.meshes.remove(d)
            best = (n, c)
        else:
            d = c.data
            bpy.data.objects.remove(c)
            bpy.data.meshes.remove(d)
        if n == 0:
            break
    return best[1]


def _make_cutaways(cutaways):
    parts = _C.parts
    pieces = {}
    if "half" not in cutaways:
        return pieces
    kept, removed, replaces = [], [], []
    for key in CUT_KEYS:
        ob = parts.get(key)
        if ob is None:
            continue
        V, M = _world_verts_local(ob)
        Minv = M.inverted()
        co_w, no_w = CUT_PLANE
        co_l = Minv @ Vector(co_w)
        no_l = (Minv.to_3x3().inverted().transposed() @ Vector(no_w)).normalized()
        up = V[:, 2] > 1e-7
        replaces.append(key)
        if not up.any():
            k = _copy(ob)
            _own(k, f"{key}__half_kept", parent=ob.parent)
            k.location, k.rotation_euler = ob.location, ob.rotation_euler
            kept.append(f"{key}__half_kept")
            continue
        if up.all():
            r = _copy(ob)
            _own(r, f"{key}__half_removed", parent=ob.parent)
            r.location, r.rotation_euler = ob.location, ob.rotation_euler
            removed.append(f"{key}__half_removed")
            continue
        k = _cut_piece(ob, co_l, no_l)
        _own(k, f"{key}__half_kept", parent=ob.parent)
        k.location, k.rotation_euler = ob.location, ob.rotation_euler
        kept.append(f"{key}__half_kept")
        r = _cut_piece(ob, co_l, -np.array(no_l))
        _own(r, f"{key}__half_removed", parent=ob.parent)
        r.location, r.rotation_euler = ob.location, ob.rotation_euler
        removed.append(f"{key}__half_removed")
    pieces["half"] = dict(kept=kept, removed=removed, replaces=replaces)
    if "none" not in cutaways:
        for key in replaces:
            ob = parts.pop(key)
            me = ob.data
            bpy.data.objects.remove(ob)
            if me.users == 0:
                bpy.data.meshes.remove(me)
    return pieces


def variant_objects(asm, variant):
    """(show, fade, hide) object lists for a cutaway variant ('none' | 'half')."""
    P = asm.parts
    pc = asm.meta.get("cutaway_pieces", {}).get("half")
    if pc is None:
        return [], [], []
    whole = [P[k] for k in pc["replaces"] if k in P]
    kept = [P[k] for k in pc["kept"] if k in P]
    rem = [P[k] for k in pc["removed"] if k in P]
    if variant == "half":
        return kept, rem, whole
    return whole, [], kept + rem


# ===========================================================================
# Anchors
# ===========================================================================

def _anchors(P, with_prop):
    a = {}
    if with_prop:
        a["propshaft"] = (P["prop_tube"], (0.0, -0.5 * PROP_L, 0.0))
        a["ujoint_front"] = (P["prop_cross_front"], (0.0, 0.0, 0.0))
        a["ujoint_rear"] = (P["prop_cross_rear"], (0.0, 0.0, 0.0))
        a["slip_yoke"] = (P["prop_slip_yoke"], (0.0, -0.075, 0.0))
    a["pinion"] = (P["pinion"], (0.0, 0.080 - Y_PIN_OBJ, 0.0))
    a["pinion_bearing"] = (P["pinion"], (0.0, Y_PB_HEAD + 0.010 - Y_PIN_OBJ, 0.0))
    a["ring_gear"] = (P["x_ring"], (0.0, -APEX_R - 0.004, 0.092))
    a["diff_case"] = (P["x_case_right"], (0.0, 0.022, 0.045))
    a["spider_gear"] = (P["x_spider_2"], (0.0, -0.004, 0.0))
    a["cross_pin"] = (P["x_cross_pin"], (0.0, 0.0, 0.045))
    a["side_gear_left"] = (P["x_side_left"], (0.0, 0.0, 0.0))
    a["side_gear_right"] = (P["x_side_right"], (0.0, 0.0, 0.0))
    a["stub_left"] = (P["x_stub_left"], (0.0, APEX_SD - 0.125, 0.0))
    a["stub_right"] = (P["x_stub_right"], (0.0, APEX_SD - 0.125, 0.0))
    hk = "housing" if "housing" in P else "housing__half_kept"
    a["diff_housing"] = (P[hk], (0.0, 0.03, 0.099))
    ck = "cover" if "cover" in P else "cover__half_kept"
    a["cover"] = (P[ck], (-0.02, -0.105, 0.0))
    return a


# ===========================================================================
# Driver
# ===========================================================================

def _per_frame(val, n, default):
    if val is None:
        return [default] * n
    if isinstance(val, str):
        return [val] * n
    v = list(val)
    if len(v) != n:
        raise ValueError(f"axle presentation array length {len(v)} != {n} frames")
    return v


def _bake_hide(ob, frames, hide):
    hide = np.asarray(hide, dtype=float)
    keep = [0] + [i for i in range(1, len(hide)) if hide[i] != hide[i - 1]]
    fr = np.asarray(frames, dtype=float)[keep]
    rig.bake_channel(ob, "hide_render", -1, fr, hide[keep], "CONSTANT")
    rig.bake_channel(ob, "hide_viewport", -1, fr, hide[keep], "CONSTANT")


def _driver(asm, track, pres):
    P = asm.parts
    fr = track.frames
    n = len(fr)
    th_out = np.asarray(track.theta_out, dtype=float)
    th_case = np.asarray(track.theta_case, dtype=float)
    RL = np.asarray(track.theta_RL, dtype=float)
    RR = np.asarray(track.theta_RR, dtype=float)
    beta = PROP_BETA
    if "prop_tube" in P:
        phi = np.arctan(-math.tan(beta) * np.sin(th_out))
        rig.bake_spin(P["prop_flange_front"], fr, th_out)
        rig.bake_spin(P["prop_spin_front"], fr, th_out)
        rig.bake_rot(P["prop_cross_front"], fr, 2, phi)
        rig.bake_spin(P["prop_tube"], fr, kin.hooke(th_out, beta))
        rig.bake_spin(P["prop_flange_rear"], fr, th_out)
        rig.bake_spin(P["prop_spin_rear"], fr, th_out)
        rig.bake_rot(P["prop_cross_rear"], fr, 2, phi)
    cg = asm.meta["cage"]
    rig.bake_spin(P["pinion"], fr, th_out)
    rig.bake_spin(P["pinion_rollers_head"], fr, th_out * cg["pinion_head"])
    rig.bake_spin(P["pinion_rollers_tail"], fr, th_out * cg["pinion_tail"])
    rig.bake_spin(P["case_left"], fr, th_case)
    rig.bake_spin(P["case_right"], fr, th_case)
    rig.bake_spin(P["ring_gear"], fr, PH_RING + th_case)
    rig.bake_spin(P["diff_cluster"], fr, th_case)
    rig.bake_spin(P["carrier_rollers_left"], fr, th_case * cg["carrier"])
    rig.bake_spin(P["carrier_rollers_right"], fr, th_case * cg["carrier"])
    sp = SPIDER_SIGN * kin.spider_spin(RL, RR)
    rig.bake_spin(P["spider_1"], fr, sp)
    rig.bake_spin(P["spider_2"], fr, sp)
    left = PH_SIDE + RL
    right = PH_SIDE - RR
    rig.bake_spin(P["side_gear_left"], fr, left)
    rig.bake_spin(P["stub_left"], fr, left)
    rig.bake_spin(P["side_gear_right"], fr, right)
    rig.bake_spin(P["stub_right"], fr, right)
    # no explode requested: put the carriers back at rest (a previous drive() may have keyed them)
    if pres.get("explode") is None:
        for k, loc in asm.meta.get("_rest_loc", {}).items():
            ob = P.get(k)
            if ob is not None:
                if ob.animation_data is not None:
                    ob.animation_data_clear()
                ob.location = loc
    # cutaway variant visibility
    pc = asm.meta.get("cutaway_pieces", {}).get("half")
    if pc is not None:
        var = _per_frame(pres.get("variant"), n, "none" if "none" in asm.meta["variants"] else "half")
        is_half = np.array([v == "half" for v in var])
        rem = pres.get("removed", 0.0)
        rem = np.full(n, float(rem)) if np.isscalar(rem) else np.asarray(rem, dtype=float)
        for k in pc["replaces"]:
            if k in P:
                _bake_hide(P[k], fr, is_half.astype(float))
        for k in pc["kept"]:
            _bake_hide(P[k], fr, (~is_half).astype(float))
        for k in pc["removed"]:
            ob = P[k]
            op = np.where(is_half, rem, 0.0)
            rig.bake_prop(ob, "cv_opacity", fr, np.where(is_half, rem, 1.0))
            _bake_hide(ob, fr, (op < 0.02).astype(float))
