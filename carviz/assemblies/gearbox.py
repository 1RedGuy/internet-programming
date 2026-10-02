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
explode carriers / anchors: ``ex_<part>`` Empties (non-rotating parents of the spinning
  parts; label anchors and explode offsets live on them)
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
``oil``      True: add the static oil volume in the sump (part ``oil`` + cut pieces like the
             housings, material ``brake_fluid`` at cv_opacity 0.3; default False)
``sections`` list of 'synchro_12' | 'synchro_34' | 'synchro_5R' (or True = ['synchro_12']):
             build quarter-sectioned copies ``<part>__sec`` of that synchro's hub, sleeve,
             struts, blocker rings, cones, dog rings, both gears and their needle bearings,
             cut in each part's LOCAL frame (the local quadrant x < 0, z > 0 removed - the
             notch turns with the part, like a motorised cutaway demonstrator).  Each copy is
             a CHILD of its part (inherits all baked motion) and starts hidden; a scene shows
             it and hides the whole part (hide_render is not inherited).  meta['sections']
             maps part -> section part name.
``collection`` collection name (default 'gearbox')

Anchors (object, local offset) - never on a spinning frame (labels do not orbit):
  input_shaft, countershaft, output_shaft, gear_1, gear_2, gear_3, gear_5, gear_R, idler,
  hub_12, sleeve_12, blocker_2, blocker_1, dogs_2, dogs_1, cone_2, fork_12, rail_12,
  rail_34, rail_5R, lever, knob, case, needle_bearing_2 (+ hub_34, sleeve_34, hub_5R,
  sleeve_5R, fork_34, fork_5R, input_gear, cs_drive, finger, output_flange, and
  dogs_<g> / cone_<g> / blocker_<g> for every g in 1 2 3 4 5 R)

Explode (presentation key ``explode`` 0..1, exploded 1-2 synchroniser along Y)
------------------------------------------------------------------------------
``GB.explode`` keys are explode CARRIERS (Empties ``ex_<part>``, also in ``parts``) so the
parts' own spin / slide composes with the offset: ex_sleeve_12 +30 mm, ex_blocker_2 +55,
ex_gear_2 (gear_2 + dogs_2 + cone_2) +85, ex_blocker_1 -35, ex_gear_1 (gear_1 + dogs_1 +
cone_1) -65.  hub_12 (+ struts) stays and the needle bearings stay on the output shaft, so
they become visible.  Parts the exploded parts pass through (incl. the 1-2 fork, which
stays on its rail) are listed in ``meta['explode_hide']`` - fade them while exploded;
``meta['explode_group']`` lists what an exploded close-up shows.

meta
----
``y`` (every axial position, car Y), ``power_path`` {1,2,3,4,5,'R','N': [part names]},
``groups`` (named part lists), ``cutaway_pieces`` {variant: {'kept','removed','replaces'}},
``explode_hide``, ``explode_group``, ``synchro`` (synchro dimensions / phase positions),
``lever`` (pivot, lever lengths, gate), ``input_splines`` (n, d_major, d_minor),
``input_shaft_radii``, ``case_length``, ``dims``, ``sections``, ``bearings`` (sizes, contact
radii), ``detents``, ``carriers`` / ``slides`` (Empty names), ``gear_y`` / ``cs_y`` (gear
centre planes), ``variants``, ``triangles``, ``build_time``, ``timings``.

Presentation keys: ``explode`` (0..1 per frame, or scalar).

Helpers: ``synchro_clearance(track)`` checks a scene's shift program against the synchro
geometry (roof clearances blocker / dogs >= 0, dogs aligned when engaged);
``lever_angles``, ``blocker_offset``, ``contact_depth`` are the geometric laws used below.

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
(+-2.81 deg index); 3 struts with detent humps; output shaft 37 mm under the gears on
3 mm needle rollers (gear bore 43 mm); bearings 6207 (input), 6208 (output centre and
rear), NU 30x62x16 (countershaft); countershaft 28 mm; case wall 6.5 mm; shift rails 16 mm
at 95 mm above the main axis (x +24 / 0 / -24 mm for 1-2 / 3-4 / 5-R, slotted heads 4.67 mm
apart under the finger, SEL-05/06); lever ratio 55/8.5 = 6.5 (ball pivot 167 mm above the
main axis, see meta['lever']); reverse train profile shifted +0.15 / -0.15 / +0.15
(15T / 22T / 38T, centre distances unchanged; FACTS REV-05).
Interfaces: input shaft 23T clutch splines 25.4 x 21.5 mm from Y -0.326 to -0.356 (spun
with theta_in, tooth 0 on +X), only the r 7.5 mm pilot runs forward through the flywheel
centre into the crank spigot bore (brass spigot bush); case front locating spigot r 44 mm
(3 mm proud of Y_GEARBOX_FRONT) for the bellhousing bore; output flange face at
Y_GEARBOX_REAR, OD 100 mm, 4 x M8 on PCD 80 mm at 45 deg (propshaft yoke flange).
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
# Everything forward of Y -0.3245 must be the slim pilot (r 7.5 mm): the engine flywheel is
# solid at its centre down to Y -0.3235 (bore r 10.5 mm).  The 23T splines start at Y -0.3260,
# 1.2 mm ahead of the clutch-disc hub's front end (hub 21 mm long, centred 3.5 mm behind
# Y_DISC_CENTRE).
U_SPL0 = (Y0 - (-0.3260)) / MM

SYNCHRO_SIDES = {"12": {+1: 2, -1: 1}, "34": {+1: 4, -1: 3}, "5R": {+1: "R", -1: 5}}
GEAR_HUB = {2: ("12", +1), 1: ("12", -1), 4: ("34", +1), 3: ("34", -1), "R": ("5R", +1), 5: ("5R", -1)}

# ---- countershaft, idler ----------------------------------------------------------
R_CS = 14.0
CS_GEARS = {"cs_drive": (S.Z_CS_DRIVEN, "input"), "cs_3": (S.GEAR_PAIRS[3][0], 3),
            "cs_2": (S.GEAR_PAIRS[2][0], 2), "cs_1": (S.GEAR_PAIRS[1][0], 1),
            "cs_R": (S.Z_REV_CS, "R"), "cs_5": (S.GEAR_PAIRS[5][0], 5)}
X_REV = {"cs_R": 0.15, "idler": -0.15, "gear_R": 0.15}      # profile shift (REV-05)
R_IDLER_SHAFT = 9.0
OIL_LEVEL = -C_MM + 8.0                         # static oil level (just above the countershaft axis)
OIL_OPACITY = 0.3

# ---- selector / lever ---------------------------------------------------------------
RAIL_Z = 95.0
RAIL_R = 8.0
RAIL_X = {"12": 24.0, "34": 0.0, "5R": -24.0}
RAIL_U0 = 12.5                                 # rail front end (rest), blind bore from u = 3
PLANE = {"12": -1, "34": 0, "5R": +1}
TIP_Z = 112.0                                  # selector-finger ball centre above the main axis
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


# ===========================================================================
# Rolling-element bearings
# ===========================================================================

def _sphere_into(mb, c, r, nu=12, nv=7):
    """UV sphere into a MeshBuilder (centre c in metres)."""
    c = np.asarray(c, float)
    top = mb.verts([c + (0, 0, r)])[0]
    bot = mb.verts([c - (0, 0, r)])[0]
    rings = []
    for i in range(1, nv):
        th = PI * i / nv
        a = np.arange(nu) * TAU / nu
        pts = np.stack([np.sin(th) * np.cos(a), np.sin(th) * np.sin(a), np.full(nu, np.cos(th))], axis=1) * r + c
        rings.append(mb.verts(pts))
    mb.fan(int(top), rings[0])
    for a_, b_ in zip(rings[:-1], rings[1:]):
        mb.bridge(a_, b_)
    mb.fan(int(bot), rings[-1])


def _cyl_into(mb, c, r, half_len, n=10, axis=1):
    """Closed cylinder along Y (axis=1) into a MeshBuilder; c in metres."""
    a = np.arange(n) * TAU / n
    c = np.asarray(c, float)
    lo = mb.verts(np.stack([c[0] + r * np.cos(a), np.full(n, c[1] - half_len), c[2] + r * np.sin(a)], axis=1))
    hi = mb.verts(np.stack([c[0] + r * np.cos(a), np.full(n, c[1] + half_len), c[2] + r * np.sin(a)], axis=1))
    mb.bridge(lo, hi)
    mb.face(list(lo))
    mb.face(list(hi))


def _race_profile(r_in, r_out, B, groove_r, groove_c, outer):
    """(r, y) profile (mm) of a bearing ring with a circular raceway groove."""
    c = 0.5
    h = B / 2
    gw = min(groove_r * 0.85, h - 1.0)
    surf = r_out if not outer else r_in
    arc = []
    for t in np.linspace(-1, 1, 9):
        y = t * gw
        dr = math.sqrt(max(groove_r ** 2 - y * y, 0.0))
        rr = groove_c + (dr if not outer else -dr)
        arc.append((rr, y))
    if not outer:   # inner ring: groove on the outside (r_out)
        prof = [(r_in + c, -h), (surf - c, -h), (surf, -h + c), (surf, -gw)] + [(min(surf, p[0]), p[1]) for p in arc] + \
               [(surf, gw), (surf, h - c), (surf - c, h), (r_in + c, h), (r_in, h - c), (r_in, -h + c)]
    else:           # outer ring: groove on the inside (r_in)
        prof = [(r_in + c, -h), (r_out - c, -h), (r_out, -h + c), (r_out, h - c), (r_out - c, h), (r_in + c, h),
                (r_in, h - c), (r_in, gw)] + [(max(surf, p[0]), p[1]) for p in arc[::-1]] + [(r_in, -gw)]
    return prof


def _ball_bearing(key, d, D, B, n_balls=None, seg=64):
    """Deep-groove ball bearing (mm), axis local Y, centred at y=0.
    Returns (inner, outer, rolling, r_inner_contact, r_outer_contact)."""
    db = 0.31 * (D - d)
    rp = (d + D) / 4
    n = n_balls or max(7, int(TAU * rp / (db * 1.45)))
    gr = 0.52 * db
    r_ir = rp - 0.38 * db
    r_or = rp + 0.38 * db
    inner = _lathe(key + "_inner", _race_profile(d / 2 + 0.02, r_ir, B, gr, rp, False), seg, "steel_ground")
    outer = _lathe(key + "_outer", _race_profile(r_or, D / 2 - 0.02, B, gr, rp, True), seg, "steel_ground")
    mb = MU.MeshBuilder()
    for k in range(n):
        a = TAU * (k + 0.5) / n
        _sphere_into(mb, (rp * math.cos(a) * MM, 0.0, rp * math.sin(a) * MM), db / 2 * MM - 1e-5,
                     nu=12 if _DETAIL == "high" else 8, nv=7 if _DETAIL == "high" else 5)
    balls = mb.to_object(_nm(key + "_balls"), _COL, smooth_angle=80.0)
    MU.assign_material(balls, "steel_ground")
    # pressed-steel cage: two thin wavy bands each side of the ball row
    cw = 0.3
    cage = []
    for sgn in (-1, 1):
        y = sgn * (db * 0.42)
        cage.append(_ring(key + f"_cg{sgn}", rp - 0.22 * db, rp + 0.22 * db, y - cw, y + cw, seg, "steel_dark", ch=0.05))
    rolling = _join(key + "_rolling", [balls] + cage)
    return inner, outer, rolling, rp - db / 2, rp + db / 2


def _roller_bearing(key, d, D, B, seg=64):
    """Cylindrical roller bearing (NU style: flanged outer ring), mm, axis local Y."""
    dr = 0.25 * (D - d)
    rp = (d + D) / 4
    n = max(9, int(TAU * rp / (dr * 1.35)))
    r_ir = rp - dr / 2
    r_or = rp + dr / 2
    h = B / 2
    lr = B * 0.62
    inner = _ring(key + "_inner", d / 2 + 0.02, r_ir - 0.02, -h, h, seg, "steel_ground", ch=0.5)
    fl = 0.6 * dr
    prof = [(r_or + 0.02 + 0.3, -h), (D / 2 - 0.5 - 0.02, -h), (D / 2 - 0.02, -h + 0.5), (D / 2 - 0.02, h - 0.5),
            (D / 2 - 0.5 - 0.02, h), (r_or - fl + 0.3, h), (r_or - fl, h - 0.3), (r_or - fl, lr / 2 + 0.4),
            (r_or + 0.02, lr / 2 + 0.4), (r_or + 0.02, -lr / 2 - 0.4), (r_or - fl, -lr / 2 - 0.4), (r_or - fl, -h + 0.3),
            (r_or - fl + 0.3, -h)]
    outer = _lathe(key + "_outer", prof, seg, "steel_ground")
    mb = MU.MeshBuilder()
    for k in range(n):
        a = TAU * (k + 0.5) / n
        _cyl_into(mb, (rp * math.cos(a) * MM, 0.0, rp * math.sin(a) * MM), (dr / 2 - 0.03) * MM, lr / 2 * MM,
                  n=12 if _DETAIL == "high" else 8)
    rollers = mb.to_object(_nm(key + "_rollers"), _COL, smooth_angle=50.0)
    MU.assign_material(rollers, "steel_ground")
    cage = [_ring(key + f"_cg{s}", rp - 0.3 * dr, rp + 0.3 * dr, s * (lr / 2 + 0.05), s * (lr / 2 + 0.9), seg,
                  "brass", ch=0.1) for s in (-1, 1)]
    rolling = _join(key + "_rolling", [rollers] + cage)
    return inner, outer, rolling, r_ir, r_or


def _needle(key, r_i, r_o, L, seg=48):
    """Needle-roller bearing (caged rollers only, races = shaft and gear bore), mm."""
    dn = r_o - r_i - 0.06
    rp = (r_i + r_o) / 2
    n = max(10, int(TAU * rp / (dn * 1.18)))
    lr = L - 2.4
    mb = MU.MeshBuilder()
    for k in range(n):
        a = TAU * (k + 0.5) / n
        _cyl_into(mb, (rp * math.cos(a) * MM, 0.0, rp * math.sin(a) * MM), dn / 2 * MM, lr / 2 * MM,
                  n=8 if _DETAIL == "high" else 6)
    rollers = mb.to_object(_nm(key + "_rollers"), _COL, smooth_angle=60.0)
    MU.assign_material(rollers, "steel_ground")
    # cage: end rings + window bars between the rollers (pressed steel)
    parts = [rollers]
    ri_c, ro_c = rp - 0.28 * dn, rp + 0.28 * dn
    for s in (-1, 1):
        parts.append(_ring(key + f"_cr{s}", ri_c, ro_c, s * (lr / 2 + 0.05), s * (L / 2), seg, "steel_dark", ch=0.1))
    bw = 0.5 * TAU * rp / n - dn / 2 - 0.05     # half free arc between rollers
    bw = max(0.15, min(bw, 0.6))
    mbb = MU.MeshBuilder()
    for k in range(n):
        a = TAU * k / n
        poly = _sector(ri_c, ro_c, a - bw / rp, a + bw / rp, n=2) * MM
        lo = mbb.ring_xz(poly, -lr / 2 * MM)
        hi = mbb.ring_xz(poly, lr / 2 * MM)
        mbb.bridge(lo, hi)
        mbb.face(list(lo))
        mbb.face(list(hi))
    bars = mbb.to_object(_nm(key + "_bars"), _COL, smooth_angle=30.0)
    MU.assign_material(bars, "steel_dark")
    parts.append(bars)
    return _join(key, parts), r_i, r_o


# ===========================================================================
# Gears
# ===========================================================================

def _gear_info(z, m_n, helix, x=0.0):
    m_t, at = G.transverse(m_n, helix, S.GEAR_PRESSURE_ANGLE)
    r_p = m_t * z / 2 / MM
    return dict(r_p=r_p, r_f=r_p - (1.25 - x) * m_n / MM, r_a=r_p + (1.0 + x) * m_n / MM, m=m_n / MM)


def _helical(key, z, m_n, helix, hand, width, bore_r, x=0.0, web=None, hub=None):
    return G.helical_gear(_nm(key), z, m_n, helix=helix, hand=hand, width=width * MM, bore=2 * bore_r * MM,
                          detail=_DETAIL, x_shift=x, material="steel_machined", collection=_COL,
                          web=None if web is None else web * MM,
                          hub=None if hub is None else tuple(v * MM for v in hub))


def _free_gear(g, z, m_n, helix, hand, width, sigma, x=0.0):
    """Free-running output-shaft gear with its synchro-side neck, dog ring and cone.
    Built in the gear's local frame (mid-face at y=0); the hub lies at
    y = -sigma*(SP + width/2).  Returns (gear, dogs, cone)."""
    key = f"gear_{g}"
    gi = _gear_info(z, m_n, helix, x)
    h = width / 2
    seg = _seg(96)
    small = gi["r_f"] - R_GB < 14.0
    r_rb = R_GB if small else gi["r_f"] - 2.4 * gi["m"]
    rim = _helical(key + "_rim", z, m_n, helix, hand, width, r_rb, x=x)
    parts = [rim]
    ys = -sigma        # t (toward the hub) -> local y = ys * t

    def tr(prof):      # (r, t) -> (r, y)
        return [(r, ys * t) for r, t in prof]
    t_dog_back = SP + h - U_DOG_BACK
    if not small:
        tw = 0.32 * width
        r_hub = R_GB + 8.0
        web = [(r_hub - 0.5, -tw), (r_rb + 0.6, -tw), (r_rb + 0.6, tw), (r_hub - 0.5, tw)]
        parts.append(_lathe(key + "_web", tr(web), seg, "steel_machined"))
        hub = [(R_GB + 0.5, -h - 2.0), (r_hub - 1.0, -h - 2.0), (r_hub, -h - 1.0), (r_hub, -h + 1.5),
               (r_hub + 1.2, -tw - 0.3), (r_hub + 1.2, tw + 0.3), (r_hub, h - 1.0), (r_hub, t_dog_back + 0.8),
               (R_GB, t_dog_back + 0.8), (R_GB, -h - 1.5)]
        parts.append(_lathe(key + "_hub", tr(hub), seg, "steel_machined"))
    else:
        rt = min(R_GB + 6.5, gi["r_f"] - 0.6)
        hub = [(R_GB + 0.5, -h - 2.0), (rt - 1.0, -h - 2.0), (rt, -h - 1.0), (rt, -h + 0.5),
               (R_GB, -h + 0.5), (R_GB, -h - 1.5)]
        parts.append(_lathe(key + "_thrust", tr(hub), seg, "steel_machined"))
    neck, cone_ob = _neck_cone(key, f"cone_{g}", h, sigma, gi["r_f"], R_GB)
    parts.append(neck)
    gear = _join(key, parts)
    gear["gear_z"] = int(z)
    gear["gear_r_tip"] = gi["r_a"] * MM
    gear["gear_r_root"] = gi["r_f"] * MM
    # dog ring (roof ends toward the hub)
    dogs = G.dog_ring(_nm(f"dogs_{g}"), N_DOG, R_IN * MM, R_OUT * MM, DOG_L * MM, chamfer_angle_deg=CH_DEG,
                      facing="-Y" if sigma > 0 else "+Y", r_body=R_DOG_BODY * MM, material="steel_machined",
                      collection=_COL, detail=_DETAIL)
    t_dog_c = SP + h - (U_DOG_RIDGE + DOG_L / 2)
    _translate(dogs, (0.0, ys * t_dog_c * MM, 0.0))
    return gear, dogs, cone_ob


def _neck_cone(key, cone_key, h, sigma, r_f, r_bore):
    """Neck (gear face -> dog ring, kept below the root inside the face so it never
    reaches into the mating countershaft gear) and the synchro cone, in the gear's
    local frame (mid-face y = 0, hub at y = -sigma*(SP + h))."""
    ys = -sigma
    t_dog_back = SP + h - U_DOG_BACK
    rin = min(29.0, r_f - 0.6)
    neck = [(r_bore, h - 0.6), (rin, h - 0.6), (rin, h + 0.4), (29.0, h + 1.0), (29.0, t_dog_back - 0.6),
            (29.6, t_dog_back), (29.6, t_dog_back + 0.8), (r_bore, t_dog_back + 0.8)]
    neck_ob = _lathe(key + "_neck", [(r, ys * t) for r, t in neck], _seg(96), "steel_machined")
    u_large = U_DOG_RIDGE + 1.9
    u_small = U_CONE_SMALL
    t_l, t_s = SP + h - u_large, SP + h - u_small          # t increases toward the hub
    rs, rl = cone_radius(u_small + 0.35), cone_radius(u_large)
    rb = r_bore + 0.1
    cone = [(rb, t_l), (rl, t_l), (rs, t_s - 0.35), (rs - 0.35, t_s), (rb + 0.4, t_s), (rb, t_s - 0.4)]
    cone_ob = _lathe(cone_key, [(r, ys * t) for r, t in cone], _seg(128), "steel_ground")
    return neck_ob, cone_ob


def _input_gear():
    """26T helical input gear (rim) - the input shaft carries the body."""
    gi = _gear_info(S.Z_INPUT, S.GEAR_NORMAL_MODULE, S.GEAR_HELIX)
    ob = _helical("input_gear", S.Z_INPUT, S.GEAR_NORMAL_MODULE, S.GEAR_HELIX, "right", FACE["input"], 23.0)
    ob["gear_r_tip"] = gi["r_a"] * MM
    return ob


def _cs_gear(key, z, width, helical=True, x=0.0):
    m = S.GEAR_NORMAL_MODULE if helical else S.REV_MODULE
    helix = S.GEAR_HELIX if helical else 0.0
    gi = _gear_info(z, m, helix, x)
    if gi["r_f"] - R_CS > 20.0:
        tw = 0.4 * width
        ob = _helical(key, z, m, helix, "left", width, R_CS + 0.02, x=x, web=tw,
                      hub=(2 * (R_CS + 8.0), width - 0.5))
    elif gi["r_f"] - R_CS > 3.0:
        ob = _helical(key, z, m, helix, "left", width, R_CS + 0.02, x=x)
    else:
        ob = _helical(key, z, m, helix, "left" if helical else "right", width, 0.0, x=x)
    ob["gear_r_tip"] = gi["r_a"] * MM
    return ob


# ===========================================================================
# Synchroniser parts (local frames centred on the hub centre plane)
# ===========================================================================

def _slot_angles():
    return [(k + 0.5) * TAU / N_DOG for k in SLOT_GAPS]


def _hub(k):
    """Synchro hub: 32 external splines with 3 strut slots, internal splines on the shaft."""
    Point, Polygon, box, LineString, uu = _shp()
    sp = G.spline_profile(N_DOG, R_IN, R_OUT, pts_flank=3 if _DETAIL == "high" else 1)
    outline = Polygon(sp["xy"]).buffer(0)
    cuts = [Polygon(_sector(SLOT_FLOOR, R_OUT + 5, a - SLOT_HALF, a + SLOT_HALF, n=10)) for a in _slot_angles()]
    poly = outline.difference(uu(cuts))
    outer = _poly_xy(poly)
    isp = G.spline_profile(24, 18.3, 20.0, internal=True, pts_flank=1, clearance=0.06, radial_clearance=0.15)
    hub = _prism(f"hub_{k}", outer, -HUB_HALF, HUB_HALF, holes=[isp["xy"]], chamfer=0.35, material="steel_machined")
    # machined recess on both faces between the splined bore boss and the toothed rim
    cuts = []
    for sg in (-1, 1):
        y0, y1 = sg * (HUB_HALF - 2.2), sg * (HUB_HALF + 1.0)
        cuts.append(_lathe(f"hub_{k}_rc{sg}", [(22.6, min(y0, y1)), (SLOT_FLOOR - 0.6, min(y0, y1)),
                                                (SLOT_FLOOR - 0.6, max(y0, y1)), (22.6, max(y0, y1))],
                           _seg(96), "steel_machined"))
    _bool(hub, cuts, "DIFFERENCE")
    return hub


def _sleeve(k):
    sl = G.sleeve_internal_teeth(_nm(f"sleeve_{k}"), N_DOG, R_IN * MM, R_OUT * MM, 2 * SLV_HALF * MM,
                                 r_outer=SLV_R_OUT * MM, groove=(GROOVE_W * MM, GROOVE_D * MM, 0.0),
                                 chamfer_angle_deg=CH_DEG, material="steel_machined", collection=_COL,
                                 detail=_DETAIL)
    # detent groove across the internal teeth (strut humps)
    (r0, w0), (r1, w1) = DET_GROOVE
    cut = _lathe(f"sleeve_{k}_det", [(r0 - 0.6, -w0 - 0.6), (r0, -w0), (r1, -w1), (r1, w1), (r0, w0), (r0 - 0.6, w0 + 0.6)],
                 _seg(128), "steel_machined")
    try:
        MU.cut_and_apply(sl, cutter=cut, section_material_name="steel_machined", delete_cutter=True)
    except Exception as e:  # pragma: no cover - keep a usable sleeve
        print(f"[gearbox] sleeve detent groove boolean failed: {e}")
        bpy.data.objects.remove(cut)
    MU.smooth_by_angle(sl, 35.0)
    return sl


def _strut(k, i, psi):
    """Strut (key) with detent hump; local frame = hub frame (slot at psi)."""
    (ra, wa), (rb, wb) = HUMP
    L = STRUT_HALF
    r0, r1 = STRUT_R
    prof = [(r0, -L + 0.4), (r0 + 0.4, -L), (r1 - 0.4, -L), (r1, -L + 0.4), (r1, -wa), (rb, -wb), (rb, wb), (r1, wa),
            (r1, L - 0.4), (r1 - 0.4, L), (r0 + 0.4, L), (r0, L - 0.4)]
    # polygon in (radial u, axial v), extruded tangentially
    ob = _prism_axis(f"strut_{k}_{i}", prof, -STRUT_HALFW, STRUT_HALFW, eu=(1, 0, 0), ev=(0, 1, 0), ew=(0, 0, 1),
                     chamfer=0.25, material="steel_dark")
    _xform(ob, _rot_y(psi))
    return ob


def _blocker(g, sigma):
    """Brass blocker ring; local origin on its hub-side face, local y = sigma * w
    (w = distance from the hub-side face toward its gear)."""
    seg = _seg(128)
    ys = sigma

    def tr(prof):
        return [(r, ys * w) for r, w in prof]
    # internal cone with fine oil-breaking threads (crests on the cone)
    pts = []
    n_thr = 7
    pitch = (BLK_L - 1.0) / n_thr
    w = 0.5
    pts.append((BLK_RC0 + 0.5 * TAN_CONE + 0.35, 0.0))
    while w < BLK_L - 0.5 - 1e-6:
        pts.append((BLK_RC0 + w * TAN_CONE, w))
        pts.append((BLK_RC0 + (w + pitch * 0.5) * TAN_CONE + 0.28, w + pitch * 0.5))
        w += pitch
    pts.append((BLK_RC0 + (BLK_L - 0.5) * TAN_CONE, BLK_L - 0.5))
    pts.append((BLK_RC0 + BLK_L * TAN_CONE + 0.4, BLK_L))
    kc = 1.0 / math.cos(PI / seg)
    pts = [(r * kc, w) for r, w in pts]
    body = [(BLK_R_BODY - 0.4, 0.0), (BLK_R_BODY, 0.4), (BLK_R_BODY, BLK_L - BLK_TEETH + 0.2),
            (29.6, BLK_L - BLK_TEETH + 0.6), (29.6, BLK_L)] + pts[::-1]
    # profile must be closed & consistent: body outer path (w ascending) then inner (w descending)
    body = [(BLK_R_BODY - 0.4, 0.0), (BLK_R_BODY, 0.4), (BLK_R_BODY, BLK_L - BLK_TEETH + 0.2),
            (29.6, BLK_L - BLK_TEETH + 0.6), (29.6, BLK_L)] + pts[::-1]
    ring = _lathe(f"blocker_{g}_body", tr(body), seg, "brass")
    teeth = G.dog_ring(_nm(f"blocker_{g}_teeth"), N_DOG, R_IN * MM, R_OUT * MM, BLK_TEETH * MM,
                       chamfer_angle_deg=CH_DEG, facing="-Y" if sigma > 0 else "+Y", r_body=28.6 * MM,
                       material="brass", collection=_COL, detail=_DETAIL)
    _translate(teeth, (0.0, ys * (BLK_L - BLK_TEETH / 2) * MM, 0.0))
    parts = [ring, teeth]
    for i, a in enumerate(_slot_angles()):
        poly = _sector(LUG_R[0], LUG_R[1], a - LUG_HALF, a + LUG_HALF, n=6)
        w0, w1 = -LUG_LEN, 0.6
        lug = _prism(f"blocker_{g}_lug{i}", poly, min(ys * w0, ys * w1), max(ys * w0, ys * w1), chamfer=0.25,
                     material="brass")
        parts.append(lug)
    return _join(f"blocker_{g}", parts)


# ===========================================================================
# Shafts (built in their own local frames: axis = local Y, u -> local y)
# ===========================================================================

def _yl(u, u_ref=0.0):
    """local y (mm) of axial position u for an object whose origin is at u_ref."""
    return -(u - u_ref)


def _input_shaft():
    """Input shaft: pilot, 23T clutch splines, bearing journal, gear body, hollow rear
    (pilot bore for the output shaft).  Origin at the case front face (u = 0)."""
    up, ud = U["pilot_tip"], U["disc"]
    us0, us1 = U_SPL0, ud + 22.0                    # clutch splines: aft of the flywheel's solid centre
    ug0, ug1 = U["input_gear"]
    u_end = U["hub_34"] - U_CONE_SMALL
    prof = [(0.0, up), (6.8, up), (7.5, up + 0.7), (7.5, us0 - 1.3), (8.6, us0 - 0.9), (10.6, us0 - 0.5),
            (10.6, us1 + 0.3), (12.2, us1 + 0.3), (13.0, us1 + 1.1), (13.0, -9.0), (14.0, -8.6), (15.0, -8.0),
            (15.0, -1.0), (16.0, -0.6), (17.5, -0.6), (17.5, U["brg_input"][1] + 0.5), (19.5, U["brg_input"][1] + 1.0),
            (20.5, ug0 - 2.5), (23.2, ug0 - 1.0), (23.2, ug1 + 0.8), (21.5, ug1 + 1.6), (21.5, u_end - 0.4),
            (21.1, u_end), (14.8, u_end), (14.2, u_end - 0.6), (14.2, ug0 + 10.0), (13.0, ug0 + 9.0), (0.0, ug0 + 9.0)]
    prof = [(r, _yl(u)) for r, u in prof]
    sh = _lathe("input_shaft_body", prof, _seg(64), "steel_machined", closed=False)
    spl = G.external_splines(_nm("input_splines"), S.CLUTCH_DISC_SPLINE_TEETH, 0.0254, 0.0215, (us1 - us0) * MM,
                             material="steel_machined", collection=_COL, detail=_DETAIL)
    _translate(spl, (0.0, _yl(0.5 * (us0 + us1)) * MM, 0.0))
    return _join("input_shaft", [sh, spl])


def _output_shaft():
    """Output shaft (origin at u = 0 like every main-axis part)."""
    h34, h12, h5 = U["hub_34"], U["hub_12"], U["hub_5R"]
    web, rw = U["web"], U["rear_wall"]
    pil0 = U["input_gear"][0] + 11.0
    pts = [(0.0, pil0), (10.3, pil0), (11.0, pil0 + 0.7), (11.0, h34 - HUB_HALF - 1.5), (12.5, h34 - HUB_HALF - 1.0),
           (18.0, h34 - HUB_HALF - 0.6),
           (18.0, h34 + HUB_HALF + 0.4), (R_JOURNAL, h34 + HUB_HALF + 0.9),

           (R_JOURNAL, h12 - HUB_HALF - 0.9), (18.0, h12 - HUB_HALF - 0.4), (18.0, h12 + HUB_HALF + 0.4),
           (R_JOURNAL, h12 + HUB_HALF + 0.9),
           (R_JOURNAL, web[0] - 2.0), (20.0, web[0] - 1.5), (20.0, U["brg_out_mid"][1] + 0.5), (R_JOURNAL, U["brg_out_mid"][1] + 1.2),
           (R_JOURNAL, h5 - HUB_HALF - 0.9), (18.0, h5 - HUB_HALF - 0.4), (18.0, h5 + HUB_HALF + 0.4),
           (R_JOURNAL, h5 + HUB_HALF + 0.9),
           (R_JOURNAL, rw[0] - 2.5), (20.0, rw[0] - 2.0), (20.0, rw[1] + 0.5), (17.5, rw[1] + 1.5), (16.0, rw[1] + 3.0),
           (16.0, U["tail_end"] - 16.0), (15.0, U["tail_end"] - 15.0), (15.0, U["flange_face"] - 10.8),
           (10.0, U["flange_face"] - 10.4), (10.0, U["flange_face"] - 3.2), (9.2, U["flange_face"] - 2.5),
           (0.0, U["flange_face"] - 2.5)]
    prof = [(r, _yl(u)) for r, u in pts]
    sh = _lathe("output_shaft_body", prof, _seg(64), "steel_machined", closed=False)
    parts = [sh]
    for k, hc in (("34", h34), ("12", h12), ("5R", h5)):
        spl = G.external_splines(_nm(f"oshaft_spl_{k}"), 24, 0.0400, 0.0366, (2 * HUB_HALF + 0.6) * MM,
                                 material="steel_machined", collection=_COL, detail=_DETAIL)
        _translate(spl, (0.0, _yl(hc) * MM, 0.0))
        parts.append(spl)
    return _join("output_shaft", parts)


def _countershaft():
    """Countershaft (origin at u = 0 on the countershaft axis)."""
    f, m, r = U["brg_input"], U["brg_out_mid"], U["rear_wall"]
    u0, u1 = 3.2, r[1] - 3.5
    pts = [(0.0, u0), (13.2, u0), (14.0, u0 + 0.8), (15.0, u0 + 1.0), (15.0, f[1] + 0.5), (14.0, f[1] + 1.2),
           (R_CS, m[0] - 1.6), (15.0, m[0] - 1.0), (15.0, m[1] + 1.0), (R_CS, m[1] + 1.6),
           (R_CS, r[0] - 1.0), (15.0, r[0] - 0.5), (15.0, u1 - 0.8), (14.2, u1), (0.0, u1)]
    return _lathe("countershaft", [(rr, _yl(u)) for rr, u in pts], _seg(48), "steel_machined", closed=False)


def _output_flange():
    """Companion flange (origin u = 0).  Rear face at Y_GEARBOX_REAR."""
    uf = U["flange_face"]
    pts = [(15.05, uf - 38.0), (23.5, uf - 38.0), (24.0, uf - 37.5), (24.0, uf - 12.0), (26.0, uf - 10.0),
           (49.0, uf - 10.0), (50.0, uf - 9.0), (50.0, uf - 0.8), (49.2, uf), (18.6, uf), (18.0, uf - 0.6),
           (18.0, uf - 9.6), (15.05, uf - 9.6)]
    fl = _lathe("output_flange_body", [(r, _yl(u)) for r, u in pts], _seg(96), "steel_forged")
    cut = []
    for k in range(4):
        a = TAU * (k + 0.5) / 4
        c = (40.0 * math.cos(a), 40.0 * math.sin(a))          # 4 x M8 on PCD 80 at 45 deg (axle yoke)
        cut.append(_cyl_along(f"fl_h{k}", (c[0], _yl(uf - 12.0), c[1]), (c[0], _yl(uf + 1.0), c[1]), 4.25, seg=20))
    _bool(fl, cut)
    r_n = 30.0 / 2 / math.cos(PI / 6)            # M20 nut, 30 AF, in the flange recess (r 18)
    nut = _prism("flange_nut", _circle((0, 0), r_n, 6, a0=PI / 6), 0.0, 8.0, holes=[_circle((0, 0), _cr(10.08, 32), 32)],
                 chamfer=0.8, material="steel_dark")
    _translate(nut, (0.0, _yl(uf - 1.4) * MM, 0.0))
    ob = _join("output_flange", [fl, nut])
    return ob


# ===========================================================================
# Housings (root-local mm: x, y = car Y in mm, z above the main axis)
# ===========================================================================

def _ycar(u):
    return Y0 / MM - u


def _case_sections():
    """Shapely polygons: inner cavity I and outer skin O of the main case."""
    Point, Polygon, box, LineString, uu = _shp()
    top = Polygon([(-49.0, 0.0), (49.0, 0.0), (49.0, 40.0), (44.5, 98.0), (37.0, 113.0), (-37.0, 113.0),
                   (-44.5, 98.0), (-49.0, 40.0)])
    I = uu([Point(0, 0).buffer(64.0, 64), Point(0, -C_MM).buffer(57.0, 64),
            Point(*IDLER_XZ).buffer(37.0, 48), top])
    I = I.buffer(9.0, 24).buffer(-9.0, 24)
    O = I.buffer(6.5, 24)
    return I, O


def _prism_y(key, poly, u0, u1, holes=(), chamfer=0.0, material="cast_aluminium", tol=0.25):
    """Prism of a shapely polygon / (x,z) array between axial positions u0 < u1."""
    if hasattr(poly, "exterior"):
        xz = _poly_xy(poly, tol)
        hs = [np.asarray(i.coords)[:-1] for i in poly.interiors] if not holes else list(holes)
    else:
        xz, hs = np.asarray(poly), list(holes)
    return _prism(key, xz, _ycar(u1), _ycar(u0), holes=hs, chamfer=chamfer, material=material)


def _cyl_u(key, x, z, r, u0, u1, seg=32, material="cast_aluminium"):
    return _cyl_along(key, (x, _ycar(u0), z), (x, _ycar(u1), z), r, seg=seg, material=material)


def _main_case():
    Point, Polygon, box, LineString, uu = _shp()
    I, O = _case_sections()
    ue = U["case_end"]
    fw1 = U["front_wall"][1]
    rw0 = U["rear_wall"][0]
    case = _prism_y("case", O, 0.0, ue, chamfer=1.2)
    adds = [_prism_y("case_ffl", O.buffer(4.0, 16), 0.0, 9.0, chamfer=1.0),
            _prism_y("case_rfl", O.buffer(3.0, 16), ue - 8.0, ue, chamfer=0.8)]
    # cast ribs: short vertical ribs on the lower flanks, a longitudinal "waist" rib each
    # side where the main and countershaft bores meet, a keel rib under the sump
    ring = O.buffer(3.2, 16).difference(O.buffer(-1.0, 16))
    for i, u in enumerate((70.0, 160.0, 258.0, 322.0)):
        for sgn in (-1, 1):
            win = box(25.0, -128.0, 200.0, 18.0) if sgn > 0 else box(-200.0, -128.0, -25.0, 18.0)
            band = ring.intersection(win)
            for gi, g in enumerate(getattr(band, "geoms", [band])):
                if g.area > 10:
                    adds.append(_prism_y(f"case_rib{i}{sgn}{gi}", g, u - 1.8, u + 1.8, chamfer=0.7))
    for sgn in (-1, 1):
        zw = -30.0
        xo = max(abs(x) for x, z in np.asarray(O.exterior.coords) if abs(z - zw) < 3.0 and x * sgn > 0)
        adds.append(_box(f"case_wrib{sgn}", sgn * xo - 2.5, sgn * xo + 3.2, _ycar(ue - 10.0), _ycar(10.0), zw - 2.4,
                         zw + 2.4, r=1.2, material="cast_aluminium"))
    adds.append(_box("case_brib", -3.0, 3.0, _ycar(ue - 12), _ycar(12.0), -C_MM - 63.5 - 4.0, -C_MM - 60.0, r=1.0,
                     material="cast_aluminium"))
    # bolt bosses around the front (bellhousing) flange
    ext = O.buffer(2.5)
    L = ext.exterior.length
    for i in range(14):
        p = ext.exterior.interpolate(L * i / 14)
        adds.append(_cyl_u(f"case_ffb{i}", p.x, p.y, 7.5, 0.0, 15.0, seg=24))
    for (x, z, r) in ((0.0, 0.0, 36.0), (0.0, -C_MM, 31.0)):
        adds.append(_cyl_u("case_fboss", x, z, r + 8.0, fw1 - 1.0, fw1 + 4.0, seg=_seg(64)))
    for (x, z, r) in ((0.0, 0.0, 40.0), (0.0, -C_MM, 31.0)):
        adds.append(_cyl_u("case_rboss", x, z, r + 8.0, rw0 - 4.0, rw0 + 1.0, seg=_seg(64)))
    adds.append(_cyl_u("case_fboss_out", 0.0, 0.0, 44.0, -3.0, 0.5, seg=_seg(64)))
    # detent bosses on the top of the rear wall
    for k, x in RAIL_X.items():
        adds.append(_cyl_along(f"case_detboss_{k}", (x, _ycar(ue - 9.0), 112.0), (x, _ycar(ue - 9.0), 124.0), 7.0,
                               seg=24, material="cast_aluminium"))
    _bool(case, adds, "UNION")
    cuts = [_prism_y("case_cav", I, fw1, rw0)]
    cuts.append(_cyl_u("case_b_in", 0.0, 0.0, _cr(36.05, _seg(96)), -5.0, fw1 + 6.0, seg=_seg(96)))
    cuts.append(_cyl_u("case_b_csf", 0.0, -C_MM, _cr(31.05, _seg(96)), 2.0, fw1 + 6.0, seg=_seg(96)))
    cuts.append(_cyl_u("case_b_out", 0.0, 0.0, _cr(40.05, _seg(96)), rw0 - 6.0, ue + 5.0, seg=_seg(96)))
    cuts.append(_cyl_u("case_b_csr", 0.0, -C_MM, _cr(31.05, _seg(96)), rw0 - 6.0, ue - 2.0, seg=_seg(96)))
    for k, x in RAIL_X.items():
        cuts.append(_cyl_u(f"case_b_rf{k}", x, RAIL_Z, _cr(8.1, 32), 3.0, fw1 + 6.0, seg=32))
        cuts.append(_cyl_u(f"case_b_rr{k}", x, RAIL_Z, _cr(8.1, 32), rw0 - 6.0, ue + 5.0, seg=32))
        cuts.append(_cyl_along(f"case_b_det{k}", (x, _ycar(ue - 9.0), RAIL_Z), (x, _ycar(ue - 9.0), 130.0),
                               _cr(4.15, 24), seg=24))
    _bool(case, cuts, "DIFFERENCE")
    MU.smooth_by_angle(case, 30.0)
    return case


def _case_extras():
    """Bolts, top cover, plugs, breather (separate static meshes joined into 'case')."""
    Point, Polygon, box, LineString, uu = _shp()
    I, O = _case_sections()
    ue = U["case_end"]
    parts = []
    top = 119.5
    cover = _box("top_cover", -44.0, 44.0, _ycar(ue - 22.0), _ycar(22.0), top, top + 6.0, r=2.5,
                 material="cast_aluminium")
    parts.append(cover)
    k = 0
    for u in np.linspace(30.0, ue - 30.0, 6):
        for x in (-38.0, 38.0):
            b = _bolt(f"cv_bolt{k}", 10.0, 6.0, 3.0, 4.0)
            _xform(b, Matrix.Translation(Vector((x * MM, _ycar(u) * MM, (top + 6.0) * MM))) @ Matrix.Rotation(PI / 2, 4, "X"))
            parts.append(b)
            k += 1
    # flange bolts front (toward the bellhousing: heads on the case side, +Y face at u = 9)
    ext = O.buffer(2.5)
    L = ext.exterior.length
    for i in range(14):
        p = ext.exterior.interpolate(L * i / 14)
        b = _bolt(f"ff_bolt{i}", 13.0, 7.5, 4.0, 4.0)
        _xform(b, Matrix.Translation(Vector((p.x * MM, _ycar(15.0) * MM, p.y * MM))) @ Matrix.Rotation(PI, 4, "Z"))
        parts.append(b)
    # drain plug (bottom) and fill plug (+X side at oil level), breather on the cover
    zb = -C_MM - 57.0 - 6.5
    dp = _hex_prism("drain_plug", 17.0, 0.0, 7.0, "steel_dark", chamfer=0.8)
    _xform(dp, Matrix.Translation(Vector((0.0, _ycar(205.0) * MM, (zb + 0.5) * MM))) @ Matrix.Rotation(-PI / 2, 4, "X"))
    parts.append(dp)
    zf = -48.0
    xo = max(x for x, z in np.asarray(O.exterior.coords) if abs(z - zf) < 4.0)
    fp = _hex_prism("fill_plug", 22.0, 0.0, 8.0, "steel_dark", chamfer=0.8)
    _xform(fp, Matrix.Translation(Vector(((xo - 0.5) * MM, _ycar(150.0) * MM, zf * MM))) @ Matrix.Rotation(-PI / 2, 4, "Z"))
    parts.append(fp)
    br = _lathe("breather", [(0.0, 0.0), (3.0, 0.0), (3.0, 8.0), (5.5, 8.5), (5.5, 12.0), (0.0, 12.5)], 16, "steel_dark",
                closed=False)
    _xform(br, Matrix.Translation(Vector((20.0 * MM, _ycar(200.0) * MM, (top + 5.5) * MM))) @ Matrix.Rotation(PI / 2, 4, "X"))
    parts.append(br)
    # detent plugs
    for kk, x in RAIL_X.items():
        pl = _hex_prism(f"det_plug_{kk}", 12.0, 0.0, 5.0, "steel_dark", chamfer=0.5)
        _xform(pl, Matrix.Translation(Vector((x * MM, _ycar(ue - 9.0) * MM, 124.0 * MM))) @ Matrix.Rotation(PI / 2, 4, "X"))
        parts.append(pl)
    return parts


def _web():
    """Intermediate web (centre-bearing plate) + reverse-idler support lug."""
    Point, Polygon, box, LineString, uu = _shp()
    I, O = _case_sections()
    w0, w1 = U["web"]
    web = _prism_y("case_web", I.buffer(-0.3, 16), w0, w1, chamfer=0.6)
    adds = []
    for (x, z, r) in ((0.0, 0.0, 40.0), (0.0, -C_MM, 31.0)):
        adds.append(_cyl_u("web_boss", x, z, r + 7.0, w0 - 3.0, w1 + 3.0, seg=_seg(64)))
    ix, iz = IDLER_XZ
    adds.append(_cyl_u("web_iboss", ix, iz, R_IDLER_SHAFT + 7.0, w1 - 1.0, w1 + 4.0, seg=32))
    _bool(web, adds, "UNION")
    cuts = [_cyl_u("web_b_out", 0.0, 0.0, _cr(40.05, _seg(96)), w0 - 5.0, w1 + 5.0, seg=_seg(96)),
            _cyl_u("web_b_cs", 0.0, -C_MM, _cr(31.05, _seg(96)), w0 - 5.0, w1 + 5.0, seg=_seg(96)),
            _cyl_u("web_b_idl", ix, iz, _cr(R_IDLER_SHAFT + 0.02, 32), w1 - 12.0, w1 + 6.0, seg=32)]
    for k, x in RAIL_X.items():
        cuts.append(_cyl_u(f"web_b_r{k}", x, RAIL_Z, _cr(8.3, 32), w0 - 5.0, w1 + 5.0, seg=32))
    # lightening windows between the bores
    for (x, z) in ((-40.0, -40.0), (36.0, 58.0), (-36.0, 58.0)):
        cuts.append(_cyl_u("web_win", x, z, 9.0, w0 - 5.0, w1 + 5.0, seg=24))
    _bool(web, cuts, "DIFFERENCE")
    # idler support lug from the case wall to the idler-shaft end
    gR1 = U["gear_R"][1]
    e = np.array([ix, iz]) / math.hypot(ix, iz)
    tip = np.array([ix, iz]) + e * 41.0
    lug_poly = LineString([(ix, iz), tuple(tip)]).buffer(10.0, 8).union(Point(ix, iz).buffer(14.0, 32))
    lug_poly = lug_poly.intersection(I.buffer(5.0))
    lug = _prism_y("idl_lug", lug_poly, gR1 + 3.0, gR1 + 11.0, chamfer=0.8)
    _bool(lug, [_cyl_u("idl_lug_b", ix, iz, _cr(R_IDLER_SHAFT + 0.02, 32), gR1, gR1 + 14.0, seg=32)])
    return [web, lug]


def _tail_housing():
    Point, Polygon, box, LineString, uu = _shp()
    ue, ut, ul = U["case_end"], U["tail_end"], U["lever"]
    seg = _seg(96)
    prof = [(0.0, ue), (70.0, ue), (70.0, ue + 9.0), (54.0, ue + 11.0), (52.0, ue + 18.0), (44.0, ut - 70.0),
            (37.0, ut - 18.0), (36.0, ut - 12.0), (34.0, ut - 10.0), (34.0, ut), (0.0, ut)]
    tail = _lathe("tail_housing", [(r, _ycar(u)) for r, u in prof], seg, "cast_aluminium", closed=False)
    tail.data.transform(Matrix.Identity(4))
    adds = []
    us0, us1 = ue, 540.0
    sh_sec = Polygon([(-48.0, 22.0), (48.0, 22.0), (48.0, 112.0), (40.0, 131.0), (-40.0, 131.0), (-48.0, 112.0)])
    sh_sec = sh_sec.buffer(-6.0, 16).buffer(6.0, 16)
    adds.append(_prism_y("tail_shift", sh_sec, us0, us1 - 14.0, chamfer=2.0))
    rear = Polygon([(-44.0, 22.0), (44.0, 22.0), (44.0, 104.0), (36.0, 121.0), (-36.0, 121.0), (-44.0, 104.0)])
    adds.append(_prism_y("tail_shift_r", rear.buffer(-6.0, 16).buffer(6.0, 16), us1 - 16.0, us1, chamfer=2.5))
    adds.append(_lathe("tail_tower", [(0.0, 118.0), (36.0, 118.0), (36.0, 126.0), (30.0, 133.0), (25.0, 140.0),
                                      (24.0, 146.0), (24.0, 159.0), (22.5, 160.5), (0.0, 160.5)], _seg(64),
                       "cast_aluminium", closed=False))
    _xform(adds[-1], Matrix.Translation(Vector((0.0, _ycar(ul) * MM, 0.0))) @ Matrix.Rotation(PI / 2, 4, "X"))
    adds.append(_box("tail_mount", -42.0, 42.0, _ycar(585.0), _ycar(548.0), -50.0, -26.0, r=3.0, material="cast_aluminium"))
    adds.append(_box("tail_mountrib", -6.0, 6.0, _ycar(585.0), _ycar(ue + 20.0), -46.0, -30.0, r=2.0, material="cast_aluminium"))
    for x in (-34.0, 34.0):
        adds.append(_cyl_along("tail_mbs", (x, _ycar(566.0), -50.0), (x, _ycar(566.0), -54.0), 8.0, seg=24,
                               material="cast_aluminium"))
    for i, u in enumerate((ue + 45.0, ue + 95.0)):
        rib = sh_sec.buffer(2.5, 8).difference(sh_sec.buffer(-1.0, 8))
        for gi, g in enumerate(getattr(rib, "geoms", [rib])):
            adds.append(_prism_y(f"tail_rib{i}{gi}", g, u - 1.6, u + 1.6, chamfer=0.6))
    _bool(tail, adds, "UNION")
    cprof = [(0.0, ue - 5.0), (46.0, ue - 5.0), (46.0, ue + 14.0), (40.0, ue + 24.0), (26.0, ut - 30.0), (24.2, ut - 20.0),
             (24.2, ut + 5.0), (0.0, ut + 5.0)]
    cav = _lathe("tail_cav", [(r, _ycar(u)) for r, u in cprof], seg, "cast_aluminium", closed=False)
    cuts = [cav, _box("tail_scav", -42.0, 42.0, _ycar(534.0), _ycar(ue - 5.0), 28.0, 125.0, r=4.0),
            _cyl_along("tail_tbore", (0.0, _ycar(ul), 110.0), (0.0, _ycar(ul), 170.0), 13.0, seg=48)]
    _bool(tail, cuts, "DIFFERENCE")
    # pivot socket cap + bolts on the flange
    parts = [tail]
    cap = _lathe("tower_cap", [(9.2, 160.0), (22.0, 160.0), (22.0, 163.0), (18.0, 166.0), (11.0, 168.5), (9.2, 168.0)],
                 _seg(64), "steel_dark")
    _xform(cap, Matrix.Translation(Vector((0.0, _ycar(ul) * MM, 0.0))) @ Matrix.Rotation(PI / 2, 4, "X"))
    parts.append(cap)
    for i in range(8):
        a = TAU * (i + 0.5) / 8
        b = _bolt(f"tl_bolt{i}", 13.0, 7.5, 4.0, 4.0)
        _xform(b, Matrix.Translation(Vector((62.0 * math.cos(a) * MM, _ycar(ue + 9.0) * MM, 62.0 * math.sin(a) * MM)))
               @ Matrix.Rotation(PI, 4, "Z"))
        parts.append(b)
    seal = _ring("tail_seal", 24.3, 33.0, _ycar(ut + 0.5), _ycar(ut - 9.0), seg, "rubber", ch=0.6)
    parts.append(seal)
    return parts


# ===========================================================================
# Selector: rails, forks, shift heads, detents, lever
# ===========================================================================

def _rail_profile(k):
    """(r, u) lathe profile of a rail with three V-notches at the detent."""
    u0 = RAIL_U0
    u1 = 515.0
    ud = U["case_end"] - 9.0
    pts = [(0.0, u0), (RAIL_R - 0.8, u0), (RAIL_R, u0 + 0.8)]
    for s in (-1, 0, 1):
        c = ud + s * TRAVEL
        for t in np.linspace(-1.0, 1.0, 9):
            yy = 1.7 * t
            depth = 1.3 * max(0.0, 1.0 - abs(t) ** 1.6)
            pts.append((RAIL_R - depth, c + yy))
    pts += [(RAIL_R, u1 - 0.8), (RAIL_R - 0.8, u1), (0.0, u1)]
    return pts


def _rail_r_at(k, u):
    prof = _rail_profile(k)
    r = np.array([p[0] for p in prof[1:-1]])
    uu_ = np.array([p[1] for p in prof[1:-1]])
    return np.interp(u, uu_, r)


def _rail(k):
    x = RAIL_X[k]
    prof = [(r, _ycar(u)) for r, u in _rail_profile(k)]
    ob = _lathe(f"rail_{k}", prof, 32, "steel_ground", closed=False)
    _translate(ob, (x * MM, 0.0, RAIL_Z * MM))
    return ob


def _fork(k):
    """Shift fork in its sleeve groove (root-local rest pose, mm)."""
    Point, Polygon, box, LineString, uu = _shp()
    x_r = RAIL_X[k]
    hc = U[f"hub_{k}"]
    yc = _ycar(hc)
    a0, a1 = -6.0 * DEG, PI + 6.0 * DEG
    r_lip0 = SLV_R_OUT - GROOVE_D + 0.9        # 39.9
    r_out = 50.0
    yoke = Polygon(_sector(SLV_R_OUT + 1.2, r_out, a0, a1, n=40))
    boss = Point(x_r, RAIL_Z).buffer(13.0, 32)
    if abs(x_r) < 1e-6:
        webp = box(-5.5, r_out - 6.0, 5.5, RAIL_Z - 6.0)
    else:
        s = 1.0 if x_r > 0 else -1.0
        webp = LineString([(s * 12.0, r_out - 4.0), (x_r, RAIL_Z - 4.0)]).buffer(6.0, 8)
        webp = webp.union(Polygon([(s * 2.0, r_out - 3.0), (s * 22.0, r_out - 6.0), (x_r, RAIL_Z - 8.0)]))
    body = uu([yoke, boss, webp]).buffer(2.0, 8).buffer(-2.0, 8)
    body = body.difference(Point(0, 0).buffer(SLV_R_OUT + 1.2, 96))
    hole = _circle((x_r, RAIL_Z), _cr(RAIL_R + 0.05, 32), 32)
    main = _prism(f"fork_{k}_body", _poly_xy(body, 0.05), yc - 4.5, yc + 4.5, holes=[hole], chamfer=0.7,
                  material="steel_forged")
    lip = _prism(f"fork_{k}_lip", _sector(r_lip0, SLV_R_OUT + 1.6, a0 + 0.04, a1 - 0.04, n=48), yc - 2.8, yc + 2.8,
                 chamfer=0.3, material="steel_forged")
    bos = _lathe(f"fork_{k}_boss", [(_cr(RAIL_R + 0.05, 32), -13.0), (12.0, -13.0), (13.0, -12.0), (13.0, 12.0),
                                    (12.0, 13.0), (_cr(RAIL_R + 0.05, 32), 13.0)], 32, "steel_forged")
    _translate(bos, (x_r * MM, yc * MM, RAIL_Z * MM))
    pin = _cyl_along(f"fork_{k}_pin", (x_r, yc + 6.0, RAIL_Z + 14.0), (x_r, yc + 6.0, RAIL_Z - 14.0), 2.0, seg=12,
                     material="steel_dark")
    parts = [main, lip, bos, pin]
    for i, a in enumerate((0.0, PI)):
        pad = _prism(f"fork_{k}_pad{i}", _sector(SLV_R_OUT - GROOVE_D + 0.4, SLV_R_OUT + 1.0, a - 5.5 * DEG,
                                                  a + 5.5 * DEG, n=6), yc - 3.3, yc + 3.3, chamfer=0.25, material="brass")
        parts.append(pad)
    return _join(f"fork_{k}", parts)


def _head(k):
    """Slotted shift head on rail k (root-local rest pose, mm): a 4 mm plate at
    x = -plane*SLOT_PITCH under the selector finger, carried by a collar on the rail
    (side rails: a cranked arm that passes over the 3-4 collar)."""
    Point, Polygon, box, LineString, uu = _shp()
    x_r = RAIL_X[k]
    p = PLANE[k]
    xp = -p * SLOT_PITCH
    yl = _ycar(U["lever"])
    pt = 4.0                                     # plate thickness (x)
    w_mid = TIP_R + 0.3                          # slot half width at the ball centre
    z_bot = TIP_Z - 3.4                          # V bottom (ball clears the flanks)
    z_top = TIP_Z + 5.0
    # the finger pivots +-9 deg about the ball pivot: above the ball it is offset by up to
    # TRAVEL * dz / L_f, so the slot walls flare above the ball centre
    w_top = w_mid + 1.12 * TRAVEL / (LEVER["L_f"] / MM) * (z_top - TIP_Z)
    z_lo = 98.0 if p == 0 else 106.4
    rc = 11.0
    parts = []
    col = _lathe(f"head_{k}_collar", [(_cr(RAIL_R + 0.05, 32), -10.0), (rc - 1.0, -10.0), (rc, -9.0), (rc, 9.0),
                                      (rc - 1.0, 10.0), (_cr(RAIL_R + 0.05, 32), 10.0)], 32, "steel_forged")
    _translate(col, (x_r * MM, yl * MM, RAIL_Z * MM))
    parts.append(col)
    plate = Polygon([(-12.0, z_lo), (12.0, z_lo), (12.0, z_top), (w_top + 0.8, z_top), (w_top, z_top - 0.6),
                     (w_mid, TIP_Z), (w_mid, z_bot + w_mid), (0.0, z_bot), (-w_mid, z_bot + w_mid), (-w_mid, TIP_Z),
                     (-w_top, z_top - 0.6), (-w_top - 0.8, z_top), (-12.0, z_top)]).buffer(0)
    pl = _prism_axis(f"head_{k}_plate", _poly_xy(plate), xp - pt / 2, xp + pt / 2, eu=(0, 1, 0), ev=(0, 0, 1),
                     ew=(1, 0, 0), origin=(0.0, yl, 0.0), chamfer=0.3, material="steel_forged")
    parts.append(pl)
    if p != 0:
        s = 1.0 if x_r > 0 else -1.0
        e = xp + s * pt / 2
        arm = Polygon([(e - s * 0.5, z_lo + 7.0), (e - s * 0.5, z_lo + 0.0), (s * 16.5, RAIL_Z + 5.2),
                       (x_r - s * 5.0, RAIL_Z + 4.0), (x_r + s * 6.0, RAIL_Z + 4.0), (x_r + s * 6.0, RAIL_Z + 9.0),
                       (s * 20.0, z_lo + 7.0)]).buffer(0)
        am = _prism(f"head_{k}_arm", _poly_xy(arm), yl + 4.0, yl + 11.0, chamfer=0.6, material="steel_forged")
        parts.append(am)
    return _join(f"head_{k}", parts)


def _detent(k):
    """Detent ball + spring (spring origin at its top so it can be scaled in z)."""
    x = RAIL_X[k]
    ud = U["case_end"] - 9.0
    mb = MU.MeshBuilder()
    _sphere_into(mb, (0.0, 0.0, 0.0), 3.97 * MM, nu=16, nv=10)
    ball = mb.to_object(_nm(f"detent_ball_{k}"), _COL, smooth_angle=80.0)
    MU.assign_material(ball, "steel_ground")
    # helical compression spring (coil wire 0.9 mm), length 14 from z_top downward
    pts = []
    n_c, L, rr = 6.0, 14.0, 3.0
    for t in np.linspace(0.0, 1.0, int(n_c * 16) + 1):
        a = TAU * n_c * t
        pts.append((rr * math.cos(a) * MM, 0.0, (-L * t) * MM))
    pts = [Vector((p[0], p[2] * 0 + p[1], p[2])) for p in pts]
    pts = [(rr * math.cos(TAU * n_c * t) * MM, rr * math.sin(TAU * n_c * t) * MM, -L * t * MM)
           for t in np.linspace(0.0, 1.0, int(n_c * 16) + 1)]
    spring = MU.tube_along(_nm(f"detent_spring_{k}"), pts, 0.45 * MM, segments=8, bend_radius=0.0, collection=_COL,
                           material="steel_dark")
    return ball, spring, x, ud


def _lever():
    """Gear lever: ball pivot, cranked rod to the knob, selector finger down to the
    rail heads.  Origin = pivot centre; rest pose (lever unrotated)."""
    lf = LEVER["L_f"] / MM
    kv = np.array(LEVER["knob_vec"]) / MM
    parts = []
    mb = MU.MeshBuilder()
    _sphere_into(mb, (0, 0, 0), 9.0 * MM, nu=24, nv=14)
    ball = mb.to_object(_nm("lever_ball"), _COL, smooth_angle=80.0)
    MU.assign_material(ball, "steel_ground")
    parts.append(ball)
    # finger (straight down): neck r 4.5 -> thin tip r 1.9 below the plate tops
    tip_len = lf
    fprof = [(0.0, -tip_len - TIP_R), (TIP_R * 0.6, -tip_len - TIP_R * 0.8), (TIP_R, -tip_len),
             (TIP_R, -tip_len + 9.5), (4.5, -tip_len + 13.5), (4.5, -6.0), (6.0, -4.0), (0.0, -4.0)]
    fin = _lathe("lever_finger", fprof, 24, "steel_machined", closed=False)
    _xform(fin, Matrix.Rotation(PI / 2, 4, "X"))          # local y -> z
    parts.append(fin)
    # rod to the knob (tilted rest vector), r 6.5 -> 5.5
    d = Vector(kv).normalized()
    L = Vector(kv).length
    rod = _lathe("lever_rod", [(0.0, 5.0), (7.0, 5.0), (6.5, 12.0), (5.5, 60.0), (5.5, L - 14.0), (0.0, L - 14.0)], 24,
                 "steel_dark", closed=False)
    q = Vector((0, 1, 0)).rotation_difference(d)
    _xform(rod, q.to_matrix().to_4x4())
    parts.append(rod)
    lever = _join("lever", parts)
    # knob: ovoid with an inset H-pattern on top (raised light strips)
    R = 21.0
    z_top = 17.5
    a_top = math.asin(z_top / R)
    kprof = [(0.0, -28.0), (6.8, -28.0), (7.4, -27.2), (7.6, -24.0)]
    for t in np.linspace(0.0, 1.0, 7)[1:]:          # flare from the neck into the ball
        a = -2.25 + t * (-0.9 + 2.25)
        kprof.append((7.6 + (R * math.cos(-0.9) - 7.6) * (t ** 1.6), -24.0 + (R * math.sin(-0.9) + 24.0) * t))
    for a in np.linspace(-0.9, a_top, 22)[1:]:
        kprof.append((R * math.cos(a), R * math.sin(a)))
    kprof += [(R * math.cos(a_top) - 0.8, z_top + 0.15), (0.0, z_top + 0.15)]
    knob = _lathe("knob", kprof, _seg(96), "plastic_black", closed=False)
    _xform(knob, Matrix.Rotation(PI / 2, 4, "X"))
    pat = []
    zt = z_top + 0.15
    for xx in (-6.0, 0.0, 6.0):
        pat.append(_box(f"knob_l{xx}", xx - 0.45, xx + 0.45, -6.2, 6.2, zt - 0.6, zt + 0.25, material="machined_aluminium"))
    pat.append(_box("knob_lx", -6.0, 6.0, -0.45, 0.45, zt - 0.6, zt + 0.25, material="machined_aluminium"))
    for i, (xx, yy) in enumerate(((-6.0, 7.4), (0.0, 7.4), (6.0, 7.4), (-6.0, -7.4), (0.0, -7.4), (6.0, -7.4))):
        dot = _cyl_along(f"knob_d{i}", (xx, yy, zt - 0.8), (xx, yy, zt + 0.25), 1.2, seg=16,
                         material="machined_aluminium")
        pat.append(dot)
    knob = _join("knob", [knob] + pat)
    _translate(knob, tuple(kv * MM))
    return lever, knob


def _boot():
    """Rubber gaiter between the tower cap and the lever, with shape keys 'shift' and
    'select' (unit rotations of 0.2 rad about X / Y of its upper rings about the pivot)."""
    kv = np.array(LEVER["knob_vec"]) / MM
    d = kv / np.linalg.norm(kv)
    z0 = 2.5            # base (relative to the pivot) on the tower cap
    z1 = 48.0
    n_conv = 5
    prof = []
    for i in range(n_conv * 4 + 1):
        t = i / (n_conv * 4)
        z = z0 + (z1 - z0) * t
        r_base = 19.0 + (7.2 - 19.0) * t ** 0.8
        wob = 1.6 * (1 - 0.6 * t) * (0.5 + 0.5 * math.cos(TAU * n_conv * t))
        prof.append((r_base + wob, z))
    outer = prof
    inner = [(max(r - 1.4, 6.6), z) for r, z in prof[::-1]]
    ring = outer + inner
    boot = _lathe("boot", ring, _seg(48), "rubber", closed=True)
    _xform(boot, Matrix.Rotation(PI / 2, 4, "X"))
    # lean the upper part with the lever's rest tilt (shear in y proportional to height)
    me = boot.data
    co = np.zeros(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    co[:, 1] += (co[:, 2] - z0 * MM) * (d[1] / d[2])
    me.vertices.foreach_set("co", co.ravel())
    me.update()
    boot.shape_key_add(name="Basis", from_mix=False)
    base = co.copy()
    f = np.clip((base[:, 2] - z0 * MM) / ((z1 - z0) * MM), 0.0, 1.0)
    for name, axis in (("shift", "X"), ("select", "Y")):
        sk = boot.shape_key_add(name=name, from_mix=False)
        out = np.empty_like(base)
        for i in range(len(base)):
            R = Matrix.Rotation(0.2 * f[i], 3, axis)
            out[i] = np.array(R @ Vector(base[i]))
        sk.data.foreach_set("co", out.ravel())
        sk.slider_min = -1.0
        sk.slider_max = 1.0
    return boot


# ===========================================================================
# Cutaways (static housings only)
# ===========================================================================

def _copy(ob, key):
    c = ob.copy()
    c.data = ob.data.copy()
    c.name = _nm(key)
    c.data.name = _nm(key)
    _COL.objects.link(c)
    for k in ("cv_opacity", "cv_glow"):
        if k in ob:
            c[k] = ob[k]
    return c


def _cut_variant(ob, base_key, variant, sec="section_cut"):
    """(kept, removed) cut pieces of a static housing for 'half' / 'quarter'."""
    kept = _copy(ob, f"{base_key}__{variant}_kept")
    rem = _copy(ob, f"{base_key}__{variant}_removed")
    kw = dict(space="LOCAL", section_material_name=sec)
    if variant == "half":
        MU.cut_and_apply(kept, plane=((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0)), **kw)
        MU.cut_and_apply(rem, plane=((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), **kw)
    elif variant == "quarter":
        MU.cut_and_apply(kept, wedge=[((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0)), ((0.0, 0.0, 0.0), (0.0, 0.0, 1.0))], **kw)
        MU.cut_and_apply(rem, plane=((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), **kw)
        MU.cut_and_apply(rem, plane=((0.0, 0.0, 0.0), (0.0, 0.0, -1.0)), **kw)
    else:
        raise ValueError(variant)
    return kept, rem


# ===========================================================================
# build
# ===========================================================================

def _tri_count(objs):
    n = 0
    for o in objs:
        if o.type == "MESH":
            o.data.calc_loop_triangles()
            n += len(o.data.loop_triangles)
    return n


def build(opts=None):
    global _COL, _DETAIL
    opts = dict(opts or {})
    t_start = time.time()
    _DETAIL = opts.get("detail", "high")
    cut = opts.get("cutaway", "none")
    variants = [cut] if isinstance(cut, str) else list(cut)
    for v in variants:
        if v not in ("none", "half", "quarter"):
            raise ValueError(f"gearbox: unknown cutaway {v!r}")
    _COL = rig.collection(opts.get("collection", "gearbox"))
    root = rig.empty(_nm("root"), loc=(S.X_CRANK, 0.0, S.Z_CRANK), col=_COL, size=0.1)
    parts, carriers, slides = {}, {}, {}
    timings = {}

    def T(label, t0):
        timings[label] = round(time.time() - t0, 2)

    def reg(key, ob):
        parts[key] = ob
        MAT.ensure_props(ob)
        return ob

    def carrier(key, loc_mm, parent=None):
        e = _empty(f"ex_{key}", parent or root, loc_mm)
        carriers[key] = e
        parts[f"ex_{key}"] = e
        return e

    # ------------------------------------------------------------------ main axis
    t0 = time.time()
    ex = carrier("input_shaft", (0.0, Y0 / MM, 0.0))
    reg("input_shaft", _place(_input_shaft(), ex))
    ug = U["input_gear"]
    yc_ig = _ycar(0.5 * (ug[0] + ug[1]))
    h4 = U["hub_34"]
    rim = _input_gear()
    tip = rim["gear_r_tip"]
    gi4 = _gear_info(S.Z_INPUT, S.GEAR_NORMAL_MODULE, S.GEAR_HELIX)
    neck, cone4 = _neck_cone("input_gear", "cone_4", FACE["input"] / 2, +1, gi4["r_f"], 21.5)
    ig = _join("input_gear", [rim, neck])
    ig["gear_r_tip"] = tip
    ex = carrier("input_gear", (0.0, yc_ig, 0.0))
    ig = reg("input_gear", _place(ig, ex))
    # input gear's 4th-gear dog ring + cone (rotate with the input gear)
    dogs4 = G.dog_ring(_nm("dogs_4"), N_DOG, R_IN * MM, R_OUT * MM, DOG_L * MM, chamfer_angle_deg=CH_DEG,
                       facing="-Y", r_body=R_DOG_BODY * MM, material="steel_machined", collection=_COL, detail=_DETAIL)
    _translate(dogs4, (0.0, (_ycar(h4 - U_DOG_RIDGE - DOG_L / 2) - yc_ig) * MM, 0.0))
    reg("dogs_4", _place(dogs4, ig))
    reg("cone_4", _place(cone4, ig))
    ex = carrier("output_shaft", (0.0, Y0 / MM, 0.0))
    reg("output_shaft", _place(_output_shaft(), ex))
    ex = carrier("output_flange", (0.0, Y0 / MM, 0.0))
    reg("output_flange", _place(_output_flange(), ex))
    T("shafts", t0)

    # ------------------------------------------------------------------ free gears
    t0 = time.time()
    gear_specs = {3: (S.GEAR_PAIRS[3][1], S.GEAR_NORMAL_MODULE, S.GEAR_HELIX, 0.0),
                  2: (S.GEAR_PAIRS[2][1], S.GEAR_NORMAL_MODULE, S.GEAR_HELIX, 0.0),
                  1: (S.GEAR_PAIRS[1][1], S.GEAR_NORMAL_MODULE, S.GEAR_HELIX, 0.0),
                  "R": (S.Z_REV_OUT, S.REV_MODULE, 0.0, X_REV["gear_R"]),
                  5: (S.GEAR_PAIRS[5][1], S.GEAR_NORMAL_MODULE, S.GEAR_HELIX, 0.0)}
    gear_y = {}
    for g, (z, m, hx, xs) in gear_specs.items():
        k, sigma = GEAR_HUB[g]
        u0, u1 = U[f"gear_{g}"]
        uc = 0.5 * (u0 + u1)
        gear_y[g] = _ycar(uc)
        gear, dogs, cone = _free_gear(g, z, m, hx, "right", u1 - u0, sigma, x=xs)
        ex = carrier(f"gear_{g}", (0.0, _ycar(uc), 0.0))
        reg(f"gear_{g}", _place(gear, ex))
        reg(f"dogs_{g}", _place(dogs, gear))
        reg(f"cone_{g}", _place(cone, gear))
        nd, ri, ro = _needle(f"needle_{g}", R_JOURNAL, R_GB, (u1 - u0) - 1.0)
        reg(f"needle_{g}", _place(nd, root, (0.0, _ycar(uc), 0.0)))
    gear_y[4] = yc_ig
    T("free_gears", t0)

    # ------------------------------------------------------------------ synchros
    t0 = time.time()
    synchro_meta = {}
    for k in ("34", "12", "5R"):
        yh = _ycar(U[f"hub_{k}"])
        ex = carrier(f"hub_{k}", (0.0, yh, 0.0))
        hub = reg(f"hub_{k}", _place(_hub(k), ex))
        for i, psi in enumerate(_slot_angles()):
            st = _strut(k, i, psi)
            reg(f"strut_{k}_{i}", _place(st, hub))
        exs = carrier(f"sleeve_{k}", (0.0, yh, 0.0))
        sl = _empty(f"sl_sleeve_{k}", exs)
        slides[f"sleeve_{k}"] = sl
        reg(f"sleeve_{k}", _place(_sleeve(k), sl))
        for sigma, g in SYNCHRO_SIDES[k].items():
            exb = carrier(f"blocker_{g}", (0.0, yh + sigma * U_BLK_FACE_R, 0.0))
            slb = _empty(f"sl_blocker_{g}", exb)
            slides[f"blocker_{g}"] = slb
            reg(f"blocker_{g}", _place(_blocker(g, sigma), slb))
        synchro_meta[k] = dict(y_hub=yh * MM, gears=dict(SYNCHRO_SIDES[k]))
    T("synchros", t0)

    # ------------------------------------------------------------------ countershaft
    t0 = time.time()
    ex = carrier("countershaft", (0.0, Y0 / MM, -C_MM))
    reg("countershaft", _place(_countershaft(), ex))
    cs_y = {}
    for key, (z, partner) in CS_GEARS.items():
        if partner == "input":
            u0, u1 = U["input_gear"]
        else:
            u0, u1 = U[f"gear_{partner}"]
        uc = 0.5 * (u0 + u1)
        cs_y[key] = _ycar(uc)
        helical = partner != "R"
        ob = _cs_gear(key, z, u1 - u0, helical=helical, x=X_REV["cs_R"] if key == "cs_R" else 0.0)
        ex = carrier(key, (0.0, _ycar(uc), -C_MM))
        reg(key, _place(ob, ex))
    # reverse idler on its shaft
    ix, iz = IDLER_XZ
    u0, u1 = U["gear_R"]
    uc = 0.5 * (u0 + u1)
    idl = G.helical_gear(_nm("idler"), S.Z_REV_IDLER, S.REV_MODULE, helix=0.0, width=(u1 - u0 - 2.0) * MM,
                         bore=2 * 11.5 * MM, detail=_DETAIL, x_shift=X_REV["idler"], material="steel_machined",
                         collection=_COL)
    ex = carrier("idler", (ix, _ycar(uc), iz))
    reg("idler", _place(idl, ex))
    nd, _, _ = _needle("needle_idler", R_IDLER_SHAFT, 11.5, u1 - u0 - 3.0)
    reg("needle_idler", _place(nd, root, (ix, _ycar(uc), iz)))
    wv1 = U["web"][1]
    ish = _lathe("idler_shaft", [(0.0, _ycar(wv1 - 11.5)), (R_IDLER_SHAFT - 0.5, _ycar(wv1 - 11.5)),
                                 (R_IDLER_SHAFT, _ycar(wv1 - 11.0)), (R_IDLER_SHAFT, _ycar(u1 + 11.5)),
                                 (R_IDLER_SHAFT - 0.8, _ycar(u1 + 12.5)), (0.0, _ycar(u1 + 12.5))], 32,
                 "steel_machined", closed=False)
    _translate(ish, (ix * MM, 0.0, iz * MM))
    reg("idler_shaft", _place(ish, root))
    T("countershaft", t0)

    # ------------------------------------------------------------------ bearings
    t0 = time.time()
    brg_specs = {
        "input": ("ball", (35.0, 72.0, 17.0), 0.0, 0.0, 0.5 * sum(U["brg_input"]), "in"),
        "cs_front": ("roller", (30.0, 62.0, 16.0), 0.0, -C_MM, 0.5 * sum(U["brg_input"]), "cs"),
        "out_mid": ("ball", (40.0, 80.0, 18.0), 0.0, 0.0, 0.5 * sum(U["brg_out_mid"]), "out"),
        "cs_mid": ("roller", (30.0, 62.0, 16.0), 0.0, -C_MM, 0.5 * sum(U["brg_out_mid"]), "cs"),
        "out_rear": ("ball", (40.0, 80.0, 18.0), 0.0, 0.0, 0.5 * sum(U["rear_wall"]), "out"),
        "cs_rear": ("roller", (30.0, 62.0, 16.0), 0.0, -C_MM, U["rear_wall"][0] + 8.0, "cs"),
    }
    brg_meta = {}
    for b, (kind, (d, D, B), x, z, uc, shaft) in brg_specs.items():
        fn = _ball_bearing if kind == "ball" else _roller_bearing
        inner, outer, rolling, rci, rco = fn(f"brg_{b}", d, D, B)
        for nm_, ob in (("inner", inner), ("outer", outer), ("rolling", rolling)):
            reg(f"brg_{b}_{nm_}", _place(ob, root, (x, _ycar(uc), z)))
        brg_meta[b] = dict(kind=kind, d=d, D=D, B=B, shaft=shaft, r_i=rci, r_o=rco, u=uc)
    nd, _, _ = _needle("needle_pilot", 11.0, 14.2, 19.0)
    reg("needle_pilot", _place(nd, root, (0.0, _ycar(U["input_gear"][0] + 22.0), 0.0)))
    # thrust washers / spacers on the output shaft (rotate with it)
    wsh = [_ring("washer_c", R_JOURNAL + 0.05, 25.0, _ycar(U["gear_2"][0] - 2.4), _ycar(U["gear_3"][1] + 2.4),
                 _seg(64), "steel_machined")]
    for i, (ua, ub) in enumerate(((U["gear_1"][1] + 2.4, U["web"][0] - 2.1),
                                 (U["brg_out_mid"][1] + 1.3, U["gear_R"][0] - 2.4),
                                 (U["gear_5"][1] + 2.4, U["rear_wall"][0] - 0.3))):
        wsh.append(_ring(f"washer{i}", R_JOURNAL + 0.05 if i != 2 else 20.05, 27.0, _ycar(ub), _ycar(ua), _seg(64),
                         "steel_machined"))
    reg("washers", _place(_join("washers", wsh), root, (0.0, 0.0, 0.0)))
    bush = _ring("spigot_bush", 7.6, 10.45, (S.Y_CRANK_FLANGE + 0.0115) / MM, (S.Y_CRANK_FLANGE + 0.0005) / MM, 32,
                 "brass", ch=0.3)
    reg("spigot_bush", _place(bush, root))
    T("bearings", t0)

    # ------------------------------------------------------------------ selector
    t0 = time.time()
    det_info = {}
    for k in ("12", "34", "5R"):
        rail = reg(f"rail_{k}", _place(_rail(k), root))
        exf = carrier(f"fork_{k}", (0.0, 0.0, 0.0), parent=rail)
        reg(f"fork_{k}", _place(_fork(k), exf))
        reg(f"head_{k}", _place(_head(k), rail))
        ball, spring, x, ud = _detent(k)
        reg(f"detent_ball_{k}", _place(ball, root, (x, _ycar(ud), RAIL_Z + 11.0)))
        reg(f"detent_spring_{k}", _place(spring, root, (x, _ycar(ud), 124.0)))
        det_info[k] = dict(x=x, u=ud)
    piv = _empty("lever_pivot", root, (0.0, S.Y_SHIFT_LEVER / MM, (LEVER["z_pivot"] - S.Z_CRANK) / MM))
    lever, knob = _lever()
    reg("lever", _place(lever, piv))
    reg("knob", _place(knob, lever))
    reg("boot", _place(_boot(), piv))
    T("selector", t0)

    # ------------------------------------------------------------------ housings
    t0 = time.time()
    housings = {"case": [_main_case()] + _case_extras(), "case_web": _web(), "tail_housing": _tail_housing()}
    if opts.get("oil"):
        I, O = _case_sections()
        from shapely.geometry import box as _sbox
        oil_poly = I.buffer(-0.4).intersection(_sbox(-200, -300, 200, OIL_LEVEL))
        housings["oil"] = [_prism_y("oil", oil_poly, U["front_wall"][1] + 0.4, U["rear_wall"][0] - 0.4,
                                    material="brake_fluid")]
    T("housings", t0)
    t0 = time.time()
    cut_pieces = {}
    for v in variants:
        if v == "none":
            continue
        kept, rem = [], []
        for key, comps in housings.items():
            ka, ra = [], []
            for i, ob in enumerate(comps):
                a, b = _cut_variant(ob, f"{key}_{i}", v, sec="brake_fluid" if key == "oil" else "section_cut")
                ka.append(a)
                ra.append(b)
            ka = [o for o in ka if len(o.data.polygons)] or ka[:1]
            ra = [o for o in ra if len(o.data.polygons)] or ra[:1]
            reg(f"{key}__{v}_kept", _place(_join(f"{key}__{v}_kept", ka), root))
            reg(f"{key}__{v}_removed", _place(_join(f"{key}__{v}_removed", ra), root))
            kept.append(f"{key}__{v}_kept")
            rem.append(f"{key}__{v}_removed")
        cut_pieces[v] = dict(kept=kept, removed=rem, replaces=list(housings))
    for key, comps in housings.items():
        if "none" in variants:
            reg(key, _place(_join(key, comps), root))
        else:
            for ob in comps:
                me = ob.data
                bpy.data.objects.remove(ob)
                if me.users == 0:
                    bpy.data.meshes.remove(me)
    T("cutaways", t0)
    # quarter-sectioned copies of rotating synchro parts (children: inherit every motion)
    sections = {}
    sec_opt = opts.get("sections") or []
    if sec_opt is True:
        sec_opt = ["synchro_12"]
    for grp in sec_opt:
        if grp.startswith("synchro_"):
            k = grp.split("_", 1)[1]
            gs = list(SYNCHRO_SIDES[k].values())
            names_ = [f"hub_{k}", f"sleeve_{k}"] + [f"strut_{k}_{i}" for i in range(3)]
            for g in gs:
                gk = "input_gear" if g == 4 else f"gear_{g}"
                names_ += [f"blocker_{g}", f"cone_{g}", f"dogs_{g}", gk] + ([f"needle_{g}"] if g != 4 else [])
        else:
            raise ValueError(f"gearbox: unknown sections group {grp!r}")
        for n in names_:
            if n in sections or n not in parts:
                continue
            src = parts[n]
            c = src.copy()
            c.data = src.data.copy()
            c.name = src.name + "__sec"
            c.data.name = c.name
            c.animation_data_clear()
            _COL.objects.link(c)
            MU.cut_and_apply(c, wedge=[((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0)), ((0.0, 0.0, 0.0), (0.0, 0.0, 1.0))],
                             space="LOCAL")
            c.parent = src
            c.matrix_parent_inverse = Matrix.Identity(4)
            c.location = (0.0, 0.0, 0.0)
            c.rotation_euler = (0.0, 0.0, 0.0)
            c.scale = (1.0, 1.0, 1.0)
            c.hide_render = True
            c.hide_viewport = True
            sections[n] = n + "__sec"
            reg(n + "__sec", c)

    for key in [k for k in parts if k.split("__")[0] == "oil"]:
        rig.set_presentation(parts[key], opacity=OIL_OPACITY)
    for ob in list(parts.values()):
        if ob.type == "MESH":
            MAT.ensure_props(ob)

    # ------------------------------------------------------------------ anchors
    def at(ob, x=0.0, y=0.0, z=0.0):
        return (ob, (x * MM, y * MM, z * MM))
    anchors = {
        "input_shaft": at(root, 0.0, _ycar(21.5), 20.0),
        "countershaft": at(root, 0.0, _ycar(U["hub_12"]), -C_MM + 14.5),
        "output_shaft": at(root, 0.0, _ycar(U["gear_3"][1] + 6.0), 24.6),
        "input_gear": at(carriers["input_gear"], 0.0, 0.0, 34.0),
        "cs_drive": at(carriers["cs_drive"], 0.0, 0.0, -45.0),
        "idler": at(carriers["idler"], 0.0, 0.0, -29.0),
        "case": at(root, 20.0, _ycar(250.0), 125.5),
        "needle_bearing_2": at(root, 0.0, gear_y[2], 0.5 * (R_GB + R_JOURNAL)),
        "lever": (parts["lever"], tuple(np.array(LEVER["knob_vec"]) * 0.55)),
        "knob": (parts["knob"], tuple(np.array(LEVER["knob_vec"]) + np.array((0.0, 0.0, 0.021)))),
        "finger": (parts["lever"], (0.0, 0.0, -LEVER["L_f"] + 0.012)),
        "output_flange": at(carriers["output_flange"], 0.0, -U["flange_face"] + 5.0, 50.0),
    }
    for g in (1, 2, 3, 5, "R"):
        anchors[f"gear_{g}"] = (carriers[f"gear_{g}"], (0.0, 0.0, parts[f"gear_{g}"]["gear_r_tip"]))
    for g in (1, 2, 3, 4, 5, "R"):
        k, sigma = GEAR_HUB[g]
        gc = carriers["input_gear" if g == 4 else f"gear_{g}"]
        yg = gear_y[g]
        yh = _ycar(U[f"hub_{k}"])
        anchors[f"dogs_{g}"] = (gc, (0.0, (yh + sigma * (U_DOG_RIDGE + DOG_L / 2) - yg) * MM, R_OUT * MM))
        uc = 0.5 * (U_CONE_SMALL + U_BLK_FACE_C)
        anchors[f"cone_{g}"] = (gc, (0.0, (yh + sigma * (U_CONE_SMALL + 0.5) - yg) * MM, cone_radius(uc) * MM))
        anchors[f"blocker_{g}"] = (slides[f"blocker_{g}"], (0.0, sigma * (BLK_L - 1.0) * MM, R_OUT * MM))
    for k in ("12", "34", "5R"):
        anchors[f"hub_{k}"] = (carriers[f"hub_{k}"], (0.0, 0.0, 31.5 * MM))
        anchors[f"sleeve_{k}"] = (slides[f"sleeve_{k}"], (0.0, 0.0, SLV_R_OUT * MM))
        xr = RAIL_X[k]
        anchors[f"fork_{k}"] = (carriers[f"fork_{k}"], (0.5 * xr * MM, _ycar(U[f"hub_{k}"]) * MM, 72.0 * MM))
        anchors[f"rail_{k}"] = (parts[f"rail_{k}"], (xr * MM, _ycar(0.5 * (U["hub_12"] + U["web"][0])) * MM,
                                                     (RAIL_Z + RAIL_R) * MM))

    # ------------------------------------------------------------------ meta
    def names(*ks):
        return [k for k in ks if k in parts]
    path_common = ["input_shaft", "input_gear", "cs_drive", "countershaft"]
    power_path = {}
    for g in (1, 2, 3, 5):
        k, _ = GEAR_HUB[g]
        power_path[g] = names(*path_common, f"cs_{g}", f"gear_{g}", f"dogs_{g}", f"sleeve_{k}", f"hub_{k}",
                              "output_shaft", "output_flange")
    power_path[4] = names("input_shaft", "input_gear", "dogs_4", "sleeve_34", "hub_34", "output_shaft",
                          "output_flange")
    power_path["R"] = names(*path_common, "cs_R", "idler", "gear_R", "dogs_R", "sleeve_5R", "hub_5R",
                            "output_shaft", "output_flange")
    power_path["N"] = names("input_shaft")
    groups = {
        "input": names("input_shaft", "input_gear", "dogs_4", "cone_4", "needle_pilot"),
        "countershaft": names("countershaft", *CS_GEARS),
        "reverse": names("cs_R", "idler", "gear_R", "dogs_R", "cone_R", "idler_shaft", "needle_idler"),
        "output": names("output_shaft", "output_flange", "washers"),
        "free_gears": names(*[f"{p}_{g}" for g in (1, 2, 3, 5, "R") for p in ("gear", "dogs", "cone", "needle")]),
        "selector": names(*[f"{p}_{k}" for k in ("12", "34", "5R") for p in ("rail", "fork", "head", "detent_ball",
                                                                               "detent_spring")],
                          "lever", "knob", "boot"),
        "bearings": [k for k in parts if k.startswith("brg_") or k.startswith("needle_")],
        "housing": [k for k in parts if k.split("__")[0] in ("case", "case_web", "tail_housing")],
        "oil": [k for k in parts if k.split("__")[0] == "oil"],
    }
    for k in ("12", "34", "5R"):
        gs = [SYNCHRO_SIDES[k][+1], SYNCHRO_SIDES[k][-1]]
        groups[f"synchro_{k}"] = names(f"hub_{k}", f"sleeve_{k}", *[f"strut_{k}_{i}" for i in range(3)],
                                       *[f"blocker_{g}" for g in gs], *[f"cone_{g}" for g in gs],
                                       *[f"dogs_{g}" for g in gs])
    explode = {"ex_sleeve_12": (0.0, 0.030, 0.0),
               "ex_blocker_2": (0.0, 0.055, 0.0), "ex_gear_2": (0.0, 0.085, 0.0),
               "ex_blocker_1": (0.0, -0.035, 0.0), "ex_gear_1": (0.0, -0.065, 0.0)}
    explode_group = names("hub_12", "sleeve_12", "blocker_2", "blocker_1", "gear_2", "dogs_2", "cone_2",
                          "gear_1", "dogs_1", "cone_1", "needle_2", "needle_1", "output_shaft",
                          *[f"strut_12_{i}" for i in range(3)])
    explode_hide = names(*EXPLODE_HIDE)
    meta = dict(
        y={k: (_ycar(v) * MM if not isinstance(v, tuple) else tuple(_ycar(x) * MM for x in v)) for k, v in U.items()},
        power_path=power_path, groups=groups, cutaway_pieces=cut_pieces, variants=variants,
        explode_hide=explode_hide, explode_group=explode_group,
        synchro=dict(per_synchro=synchro_meta, r_in=R_IN * MM, r_out=R_OUT * MM, hub_width=2 * HUB_HALF * MM,
                     sleeve_width=2 * SLV_HALF * MM, sleeve_od=2 * SLV_R_OUT * MM, chamfer_deg=CH_DEG,
                     cone_half_angle_deg=math.degrees(CONE_HALF), blocker_gap=BLK_GAP * MM,
                     blocker_ridge_on_cone=U_BLK_RIDGE_C * MM, dog_ridge=U_DOG_RIDGE * MM, hub_to_gear_face=SP * MM,
                     roof_contact_depth_indexed=D_BLOCK * MM,
                     sleeve_x_at_dog_ridge=(U_DOG_RIDGE - SLV_HALF) * MM, strut_free_travel=STRUT_GAP0 * MM,
                     slot_angles_deg=[math.degrees(a) for a in _slot_angles()]),
        lever=dict(pivot=(0.0, S.Y_SHIFT_LEVER, LEVER["z_pivot"]), L_finger=LEVER["L_f"], L_knob=LEVER["L_k"],
                   knob_rest=tuple(np.array((0.0, S.Y_SHIFT_LEVER, LEVER["z_pivot"])) + np.array(LEVER["knob_vec"])),
                   slot_pitch=SLOT_PITCH * MM, ratio=LEVER_RATIO, rail_x={k: v * MM for k, v in RAIL_X.items()},
                   rail_z=S.Z_CRANK + RAIL_Z * MM, gate=dict(S.SHIFT_GATE)),
        input_splines=(S.CLUTCH_DISC_SPLINE_TEETH, 0.0254, 0.0215),
        input_shaft_radii=dict(pilot=0.0075, splines_major=0.0127, plain=0.013, seal=0.015, bearing=0.0175,
                               pilot_tip_y=_ycar(U["pilot_tip"]) * MM,
                               splines_y=(_ycar(U_SPL0) * MM, _ycar(U["disc"] + 22.0) * MM)),
        case_length=U["case_end"] * MM, bearings=brg_meta, detents=det_info,
        dims=dict(face=FACE, centre_distance=S.GEARBOX_CENTRE_DISTANCE, idler=kin.reverse_idler_centre(),
                  case_section_mm=dict(main_r=64.0, cs_r=57.0, wall=6.5), oil=bool(opts.get("oil"))),
        carriers={k: v.name for k, v in carriers.items()}, slides={k: v.name for k, v in slides.items()},
        sections=sections,
        gear_y={str(k): v * MM for k, v in gear_y.items()}, cs_y={k: v * MM for k, v in cs_y.items()},
    )
    meta["triangles"] = _tri_count([o for o in parts.values() if o.type == "MESH"])
    meta["build_time"] = round(time.time() - t_start, 2)
    meta["timings"] = timings
    asm = rig.Assembly(name="gearbox", root=root, parts=parts, anchors=anchors, explode=explode, meta=meta,
                       _driver=_drive)
    return asm


# parts the exploded 1-2 synchro would pass through (verified by tools/test_gearbox.py)
EXPLODE_HIDE = ("blocker_3", "blocker_4", "blocker_R", "brg_cs_mid_inner", "brg_cs_mid_outer", "brg_cs_mid_rolling",
                "brg_out_mid_inner", "brg_out_mid_outer", "brg_out_mid_rolling", "case_web", "cone_3", "cone_4", "cone_R",
                "cs_1", "cs_2", "cs_3", "cs_R", "cs_drive", "dogs_3", "dogs_4", "dogs_R", "fork_12", "fork_34",
                "gear_3", "gear_R", "hub_34", "idler", "input_gear", "input_shaft", "sleeve_34", "strut_34_0",
                "strut_34_1", "strut_34_2", "washers", "needle_3", "needle_R", "needle_pilot", "idler_shaft",
                "needle_idler", "case_web__half_kept", "case_web__half_removed", "case_web__quarter_kept",
                "case_web__quarter_removed")


# ===========================================================================
# Driver
# ===========================================================================

_IDX_DEPTH = np.linspace(0.0, 2.6, 131)
_IDX_ALLOWED = None


def _allowed_index_v(depth):
    global _IDX_ALLOWED
    if _IDX_ALLOWED is None:
        _IDX_ALLOWED = np.array([allowed_index(d) for d in _IDX_DEPTH])
    return np.interp(np.asarray(depth, float), _IDX_DEPTH, _IDX_ALLOWED, right=_IDX_ALLOWED[-1])


def blocker_offset(track_blocker, sleeve, sigma):
    """Blocker ring angle relative to the output shaft (rad) for the ring on side sigma:
    track.blocker_<k> on the engaging side, limited by the sleeve-chamfer geometry."""
    s = np.asarray(sleeve, float)
    b = np.asarray(track_blocker, float)
    x = np.maximum(sigma * s, 0.0) * TRAVEL
    ridge = U_BLK_RIDGE_C - BLK_GAP + blocker_axial(x)
    depth = SLV_HALF + x - ridge
    lim = _allowed_index_v(depth) - 0.004 * DEG
    lim = np.maximum(lim, 0.0)
    off = np.sign(b) * np.minimum(np.abs(b), lim)
    return np.where(sigma * s > 1e-9, off, 0.0)


def synchro_clearance(track):
    """Geometric sanity check of a Track against this synchro geometry (for scene
    programs).  Returns dict(blocker_mm, dogs_mm, engaged_misalignment_deg): the minimum
    roof clearance (mm, >= 0 means no interference) between each sleeve and the
    indexed blocker ring / the dog ring it is entering, over every frame, and the
    largest dog misalignment while any sleeve is engaged."""
    worst_b, worst_d, eng = math.inf, math.inf, 0.0
    th_in, th_out = np.asarray(track.theta_in), np.asarray(track.theta_out)
    for k in ("12", "34", "5R"):
        s = np.asarray(track[f"sleeve_{k}"], float)
        bl = np.asarray(track[f"blocker_{k}"], float)
        for sigma, g in SYNCHRO_SIDES[k].items():
            x = np.maximum(sigma * s, 0.0) * TRAVEL
            off = blocker_offset(bl, s, sigma)
            depth = SLV_HALF + x - (U_BLK_RIDGE_C - BLK_GAP + blocker_axial(x))
            mis = kin.dog_misalignment(g, th_in, th_out)
            dd = SLV_HALF + x - U_DOG_RIDGE
            for i in np.nonzero((depth > 0) & (np.abs(off) > 0))[0]:
                worst_b = min(worst_b, contact_depth(off[i]) - depth[i])
            for i in np.nonzero(dd > 0)[0]:
                worst_d = min(worst_d, contact_depth(mis[i]) - dd[i])
            m = (np.abs(s) >= S.SYNC_ENGAGED) & (np.sign(s) == sigma)
            if np.any(m):
                eng = max(eng, float(np.max(np.abs(mis[m]))))
    return dict(blocker_mm=worst_b, dogs_mm=worst_d, engaged_misalignment_deg=math.degrees(eng))


def detent_ball_height(k, d_mm):
    """Ball centre above the rail axis (mm) for rail displacement d (mm, +Y)."""
    ud = U["case_end"] - 9.0
    rb = 3.97
    ys = np.linspace(-rb, rb, 81)
    d = np.asarray(d_mm, float)[..., None]
    r = _rail_r_at(k, ud + ys[None, :] + d)
    h = r + np.sqrt(np.maximum(rb * rb - ys[None, :] ** 2, 0.0))
    return np.max(h, axis=-1)


def lever_angles(lever_x, lever_y):
    """(a, b): lever tilt about X (fore/aft) and Y (select) in rad, so the finger tip
    moves the active rail by -lever_y*TRAVEL and sits in slot x = -lever_x*SLOT_PITCH."""
    lf = LEVER["L_f"] / MM
    d = -np.asarray(lever_y, float) * TRAVEL
    a = np.arcsin(np.clip(d / lf, -1, 1))
    b = np.arcsin(np.clip(np.asarray(lever_x, float) * SLOT_PITCH / (lf * np.cos(a)), -1, 1))
    return a, b


def _cage(theta_i, r_i, theta_o, r_o):
    return (np.asarray(theta_i) * r_i + np.asarray(theta_o) * r_o) / (r_i + r_o)


def _drive(asm, track, presentation):
    P = asm.parts
    fr = track.frames
    ex = presentation.get("explode") if isinstance(presentation, dict) else None
    if ex is not None and np.ndim(ex) == 0:
        presentation["explode"] = np.full(track.n, float(ex))   # Assembly.bake_explode needs per-frame values
    th_in = np.asarray(track.theta_in, float)
    th_out = np.asarray(track.theta_out, float)
    th_cs = kin.gb_angle("cs_drive", th_in) - kin.gb_phase_table()["cs_drive"][0]
    spin = rig.bake_spin
    spin(P["input_shaft"], fr, th_in)
    spin(P["input_gear"], fr, track.gb("input_gear"))
    spin(P["countershaft"], fr, th_cs)
    for key in CS_GEARS:
        spin(P[key], fr, track.gb(key))
    spin(P["idler"], fr, track.gb("idler"))
    gear_ang = {4: track.gb("input_gear")}
    for g in (1, 2, 3, 5, "R"):
        ang = track.gb(f"gear_{g}")
        gear_ang[g] = ang
        spin(P[f"gear_{g}"], fr, ang)
        spin(P[f"needle_{g}"], fr, _cage(th_out, R_JOURNAL, ang, R_GB))
    for key in ("output_shaft", "output_flange", "washers"):
        spin(P[key], fr, th_out)
    spin(P["needle_pilot"], fr, _cage(th_out, 11.0, th_in, 14.2))
    spin(P["needle_idler"], fr, _cage(0.0 * th_in, R_IDLER_SHAFT, track.gb("idler"), 11.5))
    shaft_ang = {"in": th_in, "out": th_out, "cs": th_cs}
    for b, info in asm.meta["bearings"].items():
        th = shaft_ang[info["shaft"]]
        spin(P[f"brg_{b}_inner"], fr, th)
        spin(P[f"brg_{b}_rolling"], fr, _cage(th, info["r_i"], 0.0 * th, info["r_o"]))
    # synchronisers
    for k in ("12", "34", "5R"):
        s = np.clip(np.asarray(track[f"sleeve_{k}"], float), -1, 1)
        x = np.abs(s) * TRAVEL
        spin(P[f"hub_{k}"], fr, th_out)
        spin(P[f"sleeve_{k}"], fr, th_out)
        rig.bake_channel(P[f"sleeve_{k}"].parent, "location", 1, fr, s * TRAVEL * MM)
        # struts: travel with the sleeve to SYNC_CONTACT, then pressed in by the groove
        sa = np.sign(s) * strut_axial(x)
        dep = strut_depression(x - strut_axial(x))
        for i, psi in enumerate(_slot_angles()):
            st = P[f"strut_{k}_{i}"]
            rig.bake_loc(st, fr, {0: -dep * math.cos(psi) * MM, 1: sa * MM, 2: -dep * math.sin(psi) * MM})
        bl = np.asarray(track[f"blocker_{k}"], float)
        for sigma, g in SYNCHRO_SIDES[k].items():
            xs = np.maximum(sigma * s, 0.0) * TRAVEL
            off = blocker_offset(bl, s, sigma)
            spin(P[f"blocker_{g}"], fr, th_out + off)
            rig.bake_channel(P[f"blocker_{g}"].parent, "location", 1, fr, sigma * blocker_axial(xs) * MM)
        # rail / fork / head (rail rest pose = location 0)
        d = s * TRAVEL
        rig.bake_channel(P[f"rail_{k}"], "location", 1, fr, d * MM)
        hb = detent_ball_height(k, d)
        hb = hb + 0.02
        rig.bake_channel(P[f"detent_ball_{k}"], "location", 2, fr, (RAIL_Z + hb) * MM)
        sc = (124.0 - 0.6 - (RAIL_Z + hb + 3.97)) / 14.0
        rig.bake_channel(P[f"detent_spring_{k}"], "scale", 2, fr, sc)
    # lever
    a, b = lever_angles(track.lever_x, track.lever_y)
    rig.bake_rot(P["lever"], fr, 0, a)
    rig.bake_rot(P["lever"], fr, 1, b)
    boot = P["boot"]
    if boot.data.shape_keys is not None:
        for name, val in (("shift", a / 0.2), ("select", b / 0.2)):
            _bake_shape(boot, name, fr, val)


def _bake_shape(obj, name, frames, values):
    """Bake a shape-key value per frame (on the Key datablock's action)."""
    key = obj.data.shape_keys
    if key.animation_data is None:
        key.animation_data_create()
    act = key.animation_data.action
    if act is None:
        act = bpy.data.actions.new(name=f"{obj.name}_keys")
        key.animation_data.action = act
    path = f'key_blocks["{name}"].value'
    try:
        fc = act.fcurve_ensure_for_datablock(key, path, index=0)
    except (AttributeError, TypeError):
        fc = act.fcurves.find(path, index=0) or act.fcurves.new(path, index=0)
    kp = fc.keyframe_points
    if len(kp):
        kp.clear()
    n = len(frames)
    kp.add(n)
    co = np.empty(2 * n, dtype=np.float32)
    co[0::2] = np.asarray(frames, float)
    co[1::2] = np.asarray(values, float)
    kp.foreach_set("co", co)
    kp.foreach_set("interpolation", np.full(n, 1, dtype=np.int32))
    fc.update()
    key.key_blocks[name].value = float(values[0])
