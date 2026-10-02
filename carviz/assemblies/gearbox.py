"""Gearbox assembly: 5-speed, 3-shaft, constant-mesh, synchronised manual gearbox (RWD).

    from carviz.assemblies import gearbox
    GB = gearbox.build({"cutaway": ["none", "half"], "detail": "high"})
    GB.drive(track, {"explode": explode_array})

Frame / origin
--------------
``GB.root`` (Empty ``gbx_root``) sits on the main drivetrain axis at (X_CRANK, 0, Z_CRANK),
no rotation.  Children use ROOT-LOCAL coordinates: x and y exactly as the car frame, z up
from the main axis.  Input + output shafts are coaxial on the main axis (x = z = 0), the
countershaft is directly below at z = -GEARBOX_CENTRE_DISTANCE (= Z_COUNTERSHAFT), the
reverse idler at ``kin.reverse_idler_centre()`` (+X side).  Every rotating part spins about
its LOCAL +Y (ARCHITECTURE.md sec. 2); phases come from ``kin.gearbox_phases``.

Axial layout (front -> rear, ``meta['y']`` has every value in car Y)
------------------------------------------------------------------
case front face Y_GEARBOX_FRONT | input bearing | input gear 26T + 4th dog ring + cone |
3-4 synchro | 3rd 31T | 2nd 37T | 1-2 synchro | 1st 44T | intermediate web (centre
bearings) | reverse 38T spur (+ idler 22T, cs 15T) | 5-R synchro | 5th 23T | rear wall +
rear bearing | tail housing (shift housing, rail heads, selector finger, lever tower) |
output flange face at Y_GEARBOX_REAR.  Countershaft gears sit under their partners.
Sleeve directions follow spec.SYNCHROS: sleeve forward (+Y) engages 2 / 4 / R, rearward
engages 1 / 3 / 5.

Parts (``GB.parts`` keys; object names are ``gbx_<key>``)
--------------------------------------------------------
rotating (baked from the Track via kin):
  input_shaft (23T clutch splines, pilot), input_gear (26T), dogs_4, cone_4,
  countershaft, cs_drive (35T), cs_3, cs_2, cs_1, cs_R (15T spur), cs_5 (38T),
  idler (22T spur),
  output_shaft (with hub-seat splines), gear_3, gear_2, gear_1, gear_R, gear_5,
  dogs_<g>, cone_<g> for g in 1 2 3 5 R (children of gear_<g>),
  needle_<g> (cage + rollers) for g in 1 2 3 5 R, needle_pilot, needle_idler,
  hub_12 / hub_34 / hub_5R, sleeve_<k>, strut_<k>_<i> (i = 0..2, children of the hub),
  blocker_<g> (brass) for g in 1 2 3 4 5 R,
  output_flange, washers (thrust washers/spacers on the output shaft),
  brg_<b>_inner / brg_<b>_outer / brg_<b>_rolling for b in input, out_mid, out_rear,
  cs_front, cs_mid, cs_rear  (outer races are static)
translating / pivoting:
  rail_12 / rail_34 / rail_5R (rail + detent notches), fork_<k> (child of its rail, brass
  pads), head_<k> (slotted shift head, child of its rail), detent_ball_<k>,
  detent_spring_<k>, lever (ball pivot + rod + selector finger), knob (child of lever,
  engraved H-pattern), boot (rubber gaiter, shape keys 'shift' / 'select')
static:
  case (main case, ribs, flanges, top cover, plugs, bolts), case_web (intermediate web +
  idler support), tail_housing (tail, shift housing, lever tower, rear mount, seal),
  idler_shaft, spigot_bush, oil (static oil in the sump, ``brake_fluid`` material, only
  with opts['oil'])
cutaway pieces: ``<case|case_web|tail_housing>__<half|quarter>_<kept|removed>``.

Opts
----
``cutaway``  'none' | 'half' | 'quarter' or a list of them (default 'none').
   'half'    - remove the -X half through the main-axis plane (x = 0): input, counter
               and output shafts in profile.  Kept + removed (complement) pieces.
   'quarter' - remove the upper -X quadrant (x < 0 and z > 0 above the main axis).
   Only static housings are cut (cuts are at build time; moving parts stay whole).  If
   'none' is not requested the whole housings are not built (kept + removed = whole).
``detail``   'high' (default) | 'low'
``oil``      True: add the static oil volume in the sump (default False)
``collection`` collection name (default 'gearbox')

Anchors (object, local offset) - never on a spinning frame (labels do not orbit):
  input_shaft, countershaft, output_shaft, gear_1, gear_2, gear_3, gear_5, gear_R, idler,
  hub_12, sleeve_12, blocker_2, blocker_1, dogs_2, dogs_1, cone_2, fork_12, rail_12,
  rail_34, rail_5R, lever, knob, case, needle_bearing_2 (+ hub_34, sleeve_34, hub_5R,
  sleeve_5R, fork_34, fork_5R, input_gear, cs_drive, finger, output_flange)

Explode (presentation key ``explode`` 0..1, exploded 1-2 synchroniser along Y):
  sleeve_12 +30 mm, blocker_2 +55, gear_2 (+ dogs_2, cone_2) +85, fork_12 +30 (follows the
  sleeve), blocker_1 -35, gear_1 (+ dogs_1, cone_1) -65; hub_12 (+ struts) stays, the needle
  bearings stay on the shaft (so they become visible).  Neighbours the exploded parts would
  pass through are listed in ``meta['explode_hide']`` - fade them while exploded.

meta
----
``y`` (every axial position, car Y), ``power_path`` {1,2,3,4,5,'R','N': [part names]},
``groups`` (named part lists), ``cutaway_pieces`` {variant: {'kept','removed','replaces'}},
``explode_hide``, ``explode_group``, ``synchro`` (synchro dimensions / phase positions),
``lever`` (pivot, lever lengths, gate), ``input_splines`` (n, d_major, d_minor),
``input_shaft_radii``, ``case_length``, ``dims``, ``triangles``, ``build_time``.

Presentation keys: ``explode`` (0..1 per frame, or scalar).

Motion (all from the Track; see _drive)
---------------------------------------
toothed parts: ``track.gb(name)``; input shaft = theta_in (23T clutch splines in phase with
the disc hub; the 26T gear object carries the kin phase); countershaft = theta_cs;
output shaft, hubs, sleeves, flange = theta_out; blocker ring on the engaging side =
theta_out + track.blocker_<k>, limited by the sleeve-chamfer geometry (the sleeve's
chamfers turn the ring back as they pass it); blocker rings move onto their cone between
the strut-contact point and spec.SYNC_CONTACT; struts move with the sleeve up to
SYNC_CONTACT and are then pressed down by the sleeve's detent groove; sleeves, forks, rails
and shift heads translate by sleeve_<k> * SLEEVE_TRAVEL; detent balls ride the rail notches;
lever tilts from lever_x / lever_y (class-1 lever: knob forward = rail rearward) so the
selector finger sits exactly in the selected rail's slot; needle/ball cages turn at the
rolling-contact speed between their races.

Dimensions not in spec (typical 2.0 L RWD 5-speed values)
---------------------------------------------------------
face widths input 22 / 3rd 20 / 2nd 21 / 1st 24 / R 16 / 5th 19 mm (countershaft gears
equal); synchro hub 18.5 mm wide, sleeve 24 mm wide, OD 85 mm, fork groove 7 x 3.5 mm;
dog / blocker / hub teeth (32, r 30 - 33.5 mm), 120 deg chamfers; single cone, half angle
6.5 deg, mean diameter ~52 mm, blocker ring 6.5 mm long with 3 lugs in 3 hub slots
(+-2.81 deg index); 3 struts with detent humps; output shaft 37 mm under the gears with
3 x 18 mm needle rollers (gear bore 43 mm); bearings 6207 (input), 6208 (output centre and
rear), NU 30x62x16 (countershaft); countershaft 28 mm; case wall 6.5 mm; shift rails 16 mm
at 95 mm above the main axis (x +24 / 0 / -24 mm for 1-2 / 3-4 / 5-R); lever ratio
55/8.5 = 6.5 (pivot 165 mm above the main axis); reverse train profile shifted
+0.15 / -0.15 / +0.15 (15T / 22T / 38T, centre distances unchanged; FACTS REV-05).
"""
from __future__ import annotations

import math
import time

import bpy  # noqa: I001  (bpy before bmesh)
import bmesh  # noqa: F401
import numpy as np
from mathutils import Matrix, Vector

from .. import gears as G
from .. import kin
from .. import materials as MAT
from .. import meshutil as MU
from .. import rig
from .. import spec as S

PREFIX = "gbx_"
MM = 1e-3
TAU = 2.0 * math.pi
PI = math.pi
DEG = PI / 180.0

# ===========================================================================
# Layout (millimetres unless noted).  u = distance BEHIND the case front face,
# car Y = Y_GEARBOX_FRONT - u.  z, x relative to the main axis.
# ===========================================================================
Y0 = S.Y_GEARBOX_FRONT
C_MM = S.GEARBOX_CENTRE_DISTANCE / MM              # 75.72
IDLER_XZ = tuple(v / MM for v in kin.reverse_idler_centre())   # (43.8, -60.9)


def yu(u_mm):
    """car Y (m) of an axial position u (mm behind the case front face)."""
    return Y0 - u_mm * MM


# ---- synchroniser (identical for 1-2, 3-4, 5-R) --------------------------------
N_DOG = S.DOG_TEETH
R_IN, R_OUT = 30.0, 33.5            # hub splines / dog teeth / blocker teeth (external form)
CH_DEG = 120.0                      # included chamfer angle of every roof-shaped tooth end
T_CH = math.tan(math.radians(CH_DEG) / 2)
TRAVEL = S.SLEEVE_TRAVEL / MM       # 8.5
X_CONTACT = S.SYNC_CONTACT * TRAVEL
X_BLOCK = S.SYNC_BLOCK * TRAVEL
X_HOLD_END = (S.SYNC_BLOCK + 0.005) * TRAVEL     # state.Program.engage() hold end
X_THROUGH = S.SYNC_THROUGH * TRAVEL
HUB_HALF = 9.25
SLV_HALF = 12.0
SLV_R_OUT = 42.5
GROOVE_W, GROOVE_D = 7.0, 3.5       # fork groove -> floor r 39.0
SLV_RC = 0.15e-3                    # sleeve radial clearance (gears.spline_profile default)
SLV_CLR = 0.06e-3
CONE_HALF = 6.5 * DEG
TAN_CONE = math.tan(CONE_HALF)
BLK_L = 6.5                         # blocker ring axial length
BLK_TEETH = 2.1                     # teeth band (roof ridge -> back face)
BLK_R_BODY = 29.0                   # body OD under the sleeve teeth
BLK_RC0 = 25.6                      # internal cone radius at the hub-side face
BLK_GAP = 0.6                       # axial free play blocker -> cone (rest -> contact)
LUG_LEN = 2.0
LUG_R = (26.4, 28.8)
SLOT_HALF = 9.0 * DEG               # hub slot half angle
SLOT_FLOOR = 24.3
SLOT_GAPS = (0, 11, 21)             # slots centred on hub tooth gaps k + 1/2
LUG_HALF = SLOT_HALF - S.BLOCKER_INDEX * TAU / N_DOG - 0.45 * DEG
STRUT_R = (25.5, 29.7)
STRUT_HUMP_R = 30.9
STRUT_HALFW = 3.75                  # tangential half width (mm)
DOG_L = 4.5                         # dog teeth axial length
DOG_GAP = 0.40                      # blocker back face (on the cone) -> dog ridge
GEAR_GAP = 2.4                      # dog ring back -> gear tooth face
R_GB = 21.5                         # free-gear bore (needle outer race)
R_JOURNAL = 18.5                    # output shaft under the needles
R_DOG_BODY = 26.0                   # dog ring collar bore
# sleeve detent groove (cut into the internal teeth) and the strut hump
DET_GROOVE = ((29.0, 2.6), (31.4, 1.9))     # (r, half width) at the bottom and floor
HUMP = ((STRUT_R[1], 2.0), (STRUT_HUMP_R, 0.8))


def _ext_half_angle(r, n=N_DOG, r_minor=R_IN, r_major=R_OUT, flank=30 * DEG, fill=0.5):
    """Half angle (rad) of an external spline/dog tooth at radius r (mm), as built by
    gears.spline_profile (straight flanks)."""
    tau = TAU / n
    rp = 0.5 * (r_minor + r_major)
    tf = math.tan(flank)
    h_p = math.sin(fill * tau / 2) * rp
    x_p = rp * math.cos(fill * tau / 2)
    lo, hi = 0.0, r
    for _ in range(60):
        x = 0.5 * (lo + hi)
        h = h_p - (x - x_p) * tf
        if x * x + h * h > r * r:
            hi = x
        else:
            lo = x
    x = 0.5 * (lo + hi)
    return math.atan2(h_p - (x - x_p) * tf, x)


def _int_half_angle(r):
    """Half angle (rad) of a sleeve internal tooth at radius r (mm)."""
    return PI / N_DOG - _ext_half_angle(r) - (SLV_CLR / MM) / r


_RGRID = np.linspace(R_IN + SLV_RC / MM + 1e-3, R_OUT - 1e-3, 41)


def contact_depth(delta):
    """Axial depth (mm, ridge past ridge) at which the sleeve's roof first touches the
    roof of an external tooth ring (blocker / dogs) turned by `delta` rad from the
    aligned position (0 = sleeve teeth centred in the gaps).  inf = never (aligned)."""
    dphi = PI / N_DOG - abs(delta)
    best = math.inf
    for r in _RGRID:
        hs, hb = _int_half_angle(r), _ext_half_angle(r)
        a0, a1 = max(0.0, dphi - hb), min(hs, dphi)
        if a0 > a1:
            continue
        for a in (a0, a1):
            best = min(best, r * (math.sin(a) + math.sin(dphi - a)) / T_CH)
    return best


def allowed_index(depth):
    """Largest |delta| (rad) the external ring may be turned while the sleeve ridge is
    `depth` mm past its ridge without interference."""
    if depth <= 0.0:
        return PI / N_DOG
    lo, hi = 0.0, PI / N_DOG
    if contact_depth(lo) >= depth:
        pass
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if contact_depth(mid) >= depth:
            lo = mid
        else:
            hi = mid
    return lo


D_BLOCK = contact_depth(S.BLOCKER_INDEX * TAU / N_DOG)      # roof contact depth when indexed
# positions measured from the hub centre toward the engaging side (mm)
U_BLK_RIDGE_C = SLV_HALF + X_HOLD_END - D_BLOCK              # blocker roof ridge (on the cone)
U_BLK_BACK_C = U_BLK_RIDGE_C + BLK_TEETH
U_BLK_FACE_C = U_BLK_BACK_C - BLK_L                          # hub-side face (on the cone)
U_BLK_FACE_R = U_BLK_FACE_C - BLK_GAP                        # ... at rest
U_DOG_RIDGE = U_BLK_BACK_C + DOG_GAP
U_DOG_BACK = U_DOG_RIDGE + DOG_L
SP = U_DOG_BACK + GEAR_GAP                                   # hub centre -> gear tooth face
U_CONE_SMALL = U_BLK_FACE_C + 0.45
STRUT_GAP0 = X_CONTACT - BLK_GAP                             # strut free travel to the lug
U_LUG_TIP_R = U_BLK_FACE_R - LUG_LEN
STRUT_HALF = U_LUG_TIP_R - STRUT_GAP0 - 0.05


def cone_radius(u):
    """Gear cone OD (mm) at distance u (mm) from the hub centre (0.03 mm under the
    blocker's internal cone when the ring is on the cone)."""
    return BLK_RC0 + (u - U_BLK_FACE_C) * TAN_CONE - 0.03


def blocker_axial(x):
    """Blocker ring axial travel toward its cone (mm) for sleeve travel x (mm) on its side."""
    return np.clip(np.asarray(x, float) - STRUT_GAP0, 0.0, BLK_GAP)


def strut_axial(x):
    """Strut travel (mm) for sleeve travel x (mm): with the sleeve until SYNC_CONTACT."""
    return np.minimum(np.asarray(x, float), X_CONTACT)


def _groove_halfw(r):
    (r0, w0), (r1, w1) = DET_GROOVE
    return np.where(r >= r1, w1, w0 + (w1 - w0) * (np.clip(r, r0, r1) - r0) / (r1 - r0))


def _strut_depression_table():
    """Radial depression (mm) of a strut hump vs sleeve-to-strut offset (mm)."""
    ys = np.linspace(-3.0, 3.0, 601)
    (ra, wa), (rb, wb) = HUMP
    top = np.where(np.abs(ys) <= wb, rb,
                   np.where(np.abs(ys) <= wa, rb - (np.abs(ys) - wb) * (rb - ra) / (wa - wb), -1.0))
    r_tip = R_IN + SLV_RC / MM
    offs = np.linspace(0.0, 8.0, 161)
    dep = []
    for d in offs:
        # sleeve inner surface (radius) over the hump, groove shifted by d
        yy = ys - d
        inner = np.full_like(ys, r_tip)
        rr = np.linspace(r_tip, DET_GROOVE[1][0], 30)
        for r in rr:
            inner = np.where(np.abs(yy) <= _groove_halfw(np.full_like(yy, r)), np.maximum(inner, r), inner)
        need = np.max(np.where(top > 0, top - (inner - 0.12), 0.0))
        dep.append(max(0.0, need))
    return offs, np.maximum.accumulate(np.array(dep))


_DEP_X, _DEP_Y = _strut_depression_table()


def strut_depression(offset):
    return np.interp(np.abs(np.asarray(offset, float)), _DEP_X, _DEP_Y)


# ---- axial layout (u, mm) ---------------------------------------------------------
FACE = {"input": 22.0, 3: 20.0, 2: 21.0, 1: 24.0, "R": 16.0, 5: 19.0}
U = {}
U["front_wall"] = (0.0, 16.0)
U["brg_input"] = (2.0, 19.0)
U["input_gear"] = (26.0, 26.0 + FACE["input"])
U["hub_34"] = U["input_gear"][1] + SP
U["gear_3"] = (U["hub_34"] + SP, U["hub_34"] + SP + FACE[3])
U["gear_2"] = (U["gear_3"][1] + 9.0, U["gear_3"][1] + 9.0 + FACE[2])
U["hub_12"] = U["gear_2"][1] + SP
U["gear_1"] = (U["hub_12"] + SP, U["hub_12"] + SP + FACE[1])
U["web"] = (U["gear_1"][1] + 8.0, U["gear_1"][1] + 28.0)
U["brg_out_mid"] = (U["web"][0] + 1.0, U["web"][0] + 19.0)
U["gear_R"] = (U["web"][1] + 10.0, U["web"][1] + 10.0 + FACE["R"])
U["hub_5R"] = U["gear_R"][1] + SP
U["gear_5"] = (U["hub_5R"] + SP, U["hub_5R"] + SP + FACE[5])
U["rear_wall"] = (U["gear_5"][1] + 9.0, U["gear_5"][1] + 27.0)
U["brg_out_rear"] = U["rear_wall"]
U["case_end"] = U["rear_wall"][1]
U["flange_face"] = (Y0 - S.Y_GEARBOX_REAR) / MM            # 650
U["lever"] = (Y0 - S.Y_SHIFT_LEVER) / MM                   # 490
U["tail_end"] = U["flange_face"] - 26.0
U["disc"] = (Y0 - S.Y_DISC_CENTRE) / MM                    # negative: in the bellhousing
U["pilot_tip"] = (Y0 - (S.Y_CRANK_FLANGE + 0.0105)) / MM

SYNCHRO_SIDES = {"12": {+1: 2, -1: 1}, "34": {+1: 4, -1: 3}, "5R": {+1: "R", -1: 5}}
GEAR_HUB = {2: ("12", +1), 1: ("12", -1), 4: ("34", +1), 3: ("34", -1), "R": ("5R", +1), 5: ("5R", -1)}

# ---- countershaft, idler ----------------------------------------------------------
R_CS = 14.0
CS_GEARS = {"cs_drive": (S.Z_CS_DRIVEN, "input"), "cs_3": (S.GEAR_PAIRS[3][0], 3),
            "cs_2": (S.GEAR_PAIRS[2][0], 2), "cs_1": (S.GEAR_PAIRS[1][0], 1),
            "cs_R": (S.Z_REV_CS, "R"), "cs_5": (S.GEAR_PAIRS[5][0], 5)}
X_REV = {"cs_R": 0.15, "idler": -0.15, "gear_R": 0.15}      # profile shift (REV-05)
R_IDLER_SHAFT = 9.0

# ---- selector / lever ---------------------------------------------------------------
RAIL_Z = 95.0
RAIL_R = 8.0
RAIL_X = {"12": 24.0, "34": 0.0, "5R": -24.0}
PLANE = {"12": -1, "34": 0, "5R": +1}
TIP_Z = 110.0                                  # selector-finger ball centre above the main axis
TIP_R = 1.9
LEVER_RATIO = S.LEVER_THROW / S.SLEEVE_TRAVEL   # knob : finger


def _lever_geometry():
    """Pivot height, finger length L_f and knob lever so that L_knob/L_f = LEVER_RATIO."""
    kz, ky = S.SHIFT_KNOB_REST[2], S.SHIFT_KNOB_REST[1] - S.Y_SHIFT_LEVER
    z_tip = S.Z_CRANK + TIP_Z * MM
    lo, hi = 0.01, 0.2
    for _ in range(60):
        lf = 0.5 * (lo + hi)
        h = kz - (z_tip + lf)
        if math.hypot(h, ky) / lf > LEVER_RATIO:
            lo = lf
        else:
            hi = lf
    lf = 0.5 * (lo + hi)
    zp = z_tip + lf
    return dict(L_f=lf, z_pivot=zp, knob_vec=(0.0, ky, kz - zp), L_k=math.hypot(kz - zp, ky))


LEVER = _lever_geometry()
SLOT_PITCH = LEVER["L_f"] * S.LEVER_GATE_SPACING / LEVER["knob_vec"][2] / MM   # ~4.65 mm


# ===========================================================================
# Small helpers
# ===========================================================================
_COL = None
_DETAIL = "high"


def _nm(key):
    return PREFIX + key


def _seg(n):
    k = 1.0 if _DETAIL == "high" else 0.5
    return max(8, int(round(n * k / 4.0)) * 4)


def _cr(r, n):
    """Circumscribed radius for an n-gon bore so its edges never come inside r."""
    return r / math.cos(PI / n)


def _xform(obj, M):
    obj.data.transform(M)
    obj.data.update()
    return obj


def _translate(obj, v):
    return _xform(obj, Matrix.Translation(Vector(v)))


def _m(v):
    return v * MM


def _lathe(key, prof_mm, seg, material, closed=True, smooth=35.0):
    """Revolve an (r, y) profile in mm (y = local Y) -> object (metres)."""
    prof = [(r * MM, y * MM) for r, y in prof_mm]
    ob = MU.lathe(_nm(key), prof, seg, closed=closed, caps=not closed, collection=_COL,
                  material=material, smooth_angle=smooth)
    return ob


def _ring(key, r0, r1, y0, y1, seg, material, ch=0.3):
    """Annular ring (mm) with small chamfers."""
    c = min(ch, 0.3 * (r1 - r0), 0.3 * abs(y1 - y0))
    ya, yb = min(y0, y1), max(y0, y1)
    prof = [(r0 + c, ya), (r1 - c, ya), (r1, ya + c), (r1, yb - c), (r1 - c, yb), (r0 + c, yb), (r0, yb - c),
            (r0, ya + c)]
    return _lathe(key, prof, seg, material)


def _join(key, objs, smooth=None):
    objs = [o for o in objs if o is not None]
    if len(objs) == 1:
        ob = objs[0]
        ob.name = _nm(key)
        ob.data.name = _nm(key)
        return ob
    ob = MU.join(_nm(key) + "_tmpjoin", objs, collection=_COL, smooth_angle=smooth)
    ob.name = _nm(key)
    ob.data.name = _nm(key)
    return ob


def _bool(target, operands, op="DIFFERENCE"):
    """Manifold boolean (collection operand), applied; operands deleted; cutter
    surfaces keep their material (TRANSFER)."""
    ops = [o for o in operands if o is not None]
    if not ops:
        return target
    tmp = bpy.data.collections.new("gbx_bool_tmp")
    bpy.context.scene.collection.children.link(tmp)
    for o in ops:
        for c in list(o.users_collection):
            c.objects.unlink(o)
        tmp.objects.link(o)
    if not target.users_collection:
        _COL.objects.link(target)
    mod = target.modifiers.new("gbx_bool", "BOOLEAN")
    mod.operation = op
    mod.operand_type = "COLLECTION"
    mod.collection = tmp
    mod.material_mode = "TRANSFER"
    me = None
    for solver in ("MANIFOLD", "EXACT"):
        mod.solver = solver
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
    MU.smooth_by_angle(target, 32.0)
    return target


def _prism(key, outer_xz_mm, y0, y1, holes=(), chamfer=0.0, material=None, smooth=35.0):
    """Prism along local Y (mm): outer outline (x, z) with holes, chamfered outer edges
    (validated inset, like gears._safe_inset)."""
    outer = np.asarray(outer_xz_mm, float) * MM
    if MU._signed_area(outer) < 0:
        outer = outer[::-1]
    hs = []
    for h in holes:
        h = np.asarray(h, float) * MM
        hs.append(h[::-1] if MU._signed_area(h) > 0 else h)
    mb = MU.MeshBuilder()
    y0m, y1m = y0 * MM, y1 * MM
    c = chamfer * MM
    lo, hi = [], []
    if c > 0:
        ins, c = G._safe_inset(outer, c)
    if c > 0:
        r0 = mb.ring_xz(ins, y0m)
        r1 = mb.ring_xz(outer, y0m + c)
        r2 = mb.ring_xz(outer, y1m - c)
        r3 = mb.ring_xz(ins, y1m)
        mb.bridge(r0, r1)
        mb.bridge(r1, r2)
        mb.bridge(r2, r3)
        lo.append(r0)
        hi.append(r3)
    else:
        r0 = mb.ring_xz(outer, y0m)
        r1 = mb.ring_xz(outer, y1m)
        mb.bridge(r0, r1)
        lo.append(r0)
        hi.append(r1)
    for h in hs:
        a = mb.ring_xz(h, y0m)
        b = mb.ring_xz(h, y1m)
        mb.bridge(a, b)
        lo.append(a)
        hi.append(b)
    mb.cap(lo, axis=1)
    mb.cap(hi, axis=1)
    ob = mb.to_object(_nm(key), _COL, smooth_angle=smooth)
    if material is not None:
        MU.assign_material(ob, material)
    return ob


def _frame(eu, ev, ew, origin=(0, 0, 0)):
    """Matrix mapping canonical prism coords (x=u, y=w, z=v) -> eu, ew, ev (mm origin)."""
    M = Matrix.Identity(4)
    for i in range(3):
        M[i][0], M[i][1], M[i][2] = eu[i], ew[i], ev[i]
        M[i][3] = origin[i] * MM
    return M


def _prism_axis(key, poly_uv_mm, w0, w1, eu, ev, ew, origin=(0, 0, 0), holes=(), chamfer=0.0, material=None):
    """Prism of a (u, v) polygon extruded along ew from w0 to w1 (all mm)."""
    ob = _prism(key, poly_uv_mm, w0, w1, holes=holes, chamfer=chamfer, material=material)
    M = _frame(eu, ev, ew, origin)
    _xform(ob, M)
    if M.to_3x3().determinant() < 0:
        ob.data.flip_normals()
    return ob


def _box(key, x0, x1, y0, y1, z0, z1, r=0.0, material=None):
    """Rounded box (mm)."""
    return MU.rounded_box(_nm(key), ((x1 - x0) * MM, (y1 - y0) * MM, (z1 - z0) * MM), radius=r * MM, segments=2,
                          center=((x0 + x1) / 2 * MM, (y0 + y1) / 2 * MM, (z0 + z1) / 2 * MM), collection=_COL,
                          material=material)


def _cyl_along(key, p0, p1, r, seg=24, material=None, chamfer=0.0):
    """Closed cylinder from point p0 to p1 (mm)."""
    p0, p1 = np.asarray(p0, float) * MM, np.asarray(p1, float) * MM
    L = float(np.linalg.norm(p1 - p0))
    ob = MU.cylinder(_nm(key), r * MM, 0.0, L, segments=seg, chamfer=chamfer * MM, collection=_COL, material=material)
    d = Vector((p1 - p0) / L)
    q = Vector((0, 1, 0)).rotation_difference(d)
    _xform(ob, Matrix.Translation(Vector(p0)) @ q.to_matrix().to_4x4())
    return ob


def _circle(c, r, n=48, a0=0.0):
    a = a0 + np.linspace(0, TAU, n, endpoint=False)
    return np.stack([c[0] + r * np.cos(a), c[1] + r * np.sin(a)], axis=1)


def _sector(r0, r1, a0, a1, n=12):
    """Annular sector polygon (x, z) mm from angle a0 to a1 (rad, psi)."""
    a = np.linspace(a0, a1, n)
    outer = np.stack([r1 * np.cos(a), r1 * np.sin(a)], axis=1)
    inner = np.stack([r0 * np.cos(a[::-1]), r0 * np.sin(a[::-1])], axis=1)
    return np.vstack([outer, inner])


def _shp():
    from shapely.geometry import LineString, Point, Polygon, box
    from shapely.ops import unary_union
    return Point, Polygon, box, LineString, unary_union


def _poly_xy(geom, tol=0.0):
    if hasattr(geom, "geoms"):
        geom = max(geom.geoms, key=lambda g: g.area)
    if tol:
        geom = geom.simplify(tol, preserve_topology=True)
    return np.asarray(geom.exterior.coords)[:-1]


def _place(ob, parent, loc_mm=(0.0, 0.0, 0.0), rot=None):
    """Parent (identity inverse) and set the local location (mm)."""
    ob.parent = parent
    ob.matrix_parent_inverse = Matrix.Identity(4)
    ob.location = (loc_mm[0] * MM, loc_mm[1] * MM, loc_mm[2] * MM)
    if rot is not None:
        ob.rotation_euler = rot
    return ob


def _empty(key, parent, loc_mm=(0.0, 0.0, 0.0), size=0.02):
    e = rig.empty(_nm(key), col=_COL, size=size)
    return _place(e, parent, loc_mm)


def _hex_prism(key, af, y0, y1, material, chamfer=0.3):
    """Hexagon prism along Y (mm), across flats af."""
    r = af / 2 / math.cos(PI / 6)
    poly = _circle((0, 0), r, 6, a0=PI / 6)
    return _prism(key, poly, y0, y1, chamfer=chamfer, material=material)


def _bolt(key, af, head_h, shank_r, shank_len, material="steel_dark", washer=True):
    """Hex-head bolt along -Y from y=0 (head on +Y side of the joint face at y=0):
    head occupies y in [0, head_h], shank y in [-shank_len, 0].  mm."""
    head = _hex_prism(key + "_h", af, 0.0, head_h, material, chamfer=0.35)
    parts = [head]
    if washer:
        parts.append(_ring(key + "_w", shank_r + 0.2, af * 0.62, -0.0, 1.2, 24, material, ch=0.2))
        _translate(head, (0, 1.2 * MM, 0))
    sh = MU.cylinder(_nm(key + "_s"), shank_r * MM, -shank_len * MM, 0.0, segments=12, chamfer=0.3 * MM,
                     collection=_COL, material=material)
    parts.append(sh)
    return _join(key, parts)


def _rot_y(psi):
    """Matrix rotating local profile angle 0 to psi (rotation about +Y by -psi)."""
    return Matrix.Rotation(-psi, 4, "Y")
