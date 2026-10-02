"""Clutch assembly: single dry plate, push-type diaphragm-spring clutch with
hydraulic release (pedal -> master -> line -> slave -> fork -> release bearing).

    from carviz.assemblies import clutch
    C = clutch.build({"cutaway": ["none", "half"], "detail": "high"})
    C.drive(track, {"explode": ex, "variant": "half"})

Frame / origin
--------------
``C.root`` (Empty ``clu_root``) sits on the crank axis at the disc centre
(X_CRANK, Y_DISC_CENTRE, Z_CRANK), no rotation.  Children use ROOT-LOCAL
coordinates (car frame axes, origin at the root).  Spinning parts have their
origin on the crank axis at the root and spin about local +Y
(``rig.bake_spin``); car-frame positions are listed in ``meta['positions']``.

Parts (``C.parts`` keys; object names ``clu_<key>``)
---------------------------------------------------
moving (all motion baked from the Track):
  disc              hub (23 internal splines, abs angle = theta_in), hub flange, disc + retainer plates,
                    stop pins, cushion segments, two friction facings (grooves,
                    rivet holes, rivets).  Spins with the input shaft, floats
                    axially by -plate_lift/2; shape key 'free' opens the cushion
                    (facings spread 0.65 mm) as the clamp is released.
  damper_springs    6 torsional damper coil springs (child of disc)
  pressure_plate    cast iron, machined friction face, fulcrum ridge, 3 strap lugs;
                    spins with theta_e, moves -Y by clutch_plate_lift
  straps            3 tangential 3-leaf drive straps + rivets; spin theta_e, shape
                    key 'lift' (S-bend, plate end follows the plate)
  diaphragm_spring  Belleville ring + 18 fingers; spins theta_e; shape keys
                    'bend' (fingers bend, rim still; value = min(finger, F0)/F0) and
                    'release' (rigid rotation about the fulcrum rings; value =
                    plate_lift / PRESSURE_PLATE_LIFT).  Tips move exactly by
                    clutch_finger, rim exactly by clutch_plate_lift.
  fulcrum           two pivot (fulcrum) wire rings + 18 shoulder rivets; spin theta_e
  cover             pressed-steel cover (3 strap pockets, side windows, rivet heads,
                    strap pedestals) + 6 bolts to the flywheel; spins theta_e
  release_bearing   non-rotating carrier (sleeve, cup, outer ring, seal, fork pads);
                    moves +Y by clutch_bearing
  bearing_race      rotating inner race with the finger-contact nose (child of
                    release_bearing); spins theta_e (constant light contact)
  fork              pressed-steel release fork (yoke, channel arm, ball cup, contact
                    bumps); rotates about a vertical axis through the ball stud by
                    asin(bearing / FORK_CX) (exact contact at both ends)
static:
  ball_stud, guide_tube, bellhousing (+ cutaway pieces), slave_cylinder,
  hose_bracket, line_fittings, master_cylinder (+ reservoir), pedal_box
moving (hydraulics / pedal):
  slave_pushrod     slave piston + pushrod; moves -Y by clutch_slave
  master_piston     master piston (inside the body); moves +Y by clutch_master
  pedal             clutch pedal (arm, pad, bushing, clevis pin); rotates about +X by
                    clutch_pedal_angle (pad moves forward)
  pushrod           clevis + rod from the pedal to the master piston; follows the
                    pedal (pinned at the clevis, tip guided on the master axis)
  return_spring     pedal return spring (aimed + stretched between its hooks)
  line_00..line_11  hydraulic line, master -> slave (steel pipe then rubber hose)
                    = meta['hydraulic_segments']
with opts['hydraulic_cut']: line_NN__cut_kept / __cut_removed (half pipes) and
  fluid_NN (brake_fluid core) for every segment.

Anchors (``C.anchors``; label points follow explode but not spin)
  disc, facing, hub_splines, damper_springs, pressure_plate, diaphragm_spring,
  fingers, cover, release_bearing, fork, slave_cylinder, master_cylinder, pedal,
  hydraulic_line, bellhousing, guide_tube, pushrod

Opts
----
``cutaway``  'none' | 'half' | 'half_px' or a list of them (default ['none']).  'half'
   removes the -X half of the BELLHOUSING (x < X_CRANK) with red section faces as a
   stepped cut that keeps the slave-cylinder mount (lug + a patch of wall) so the slave
   does not float; pieces ``bellhousing__half_kept`` / ``bellhousing__half_removed``
   (+ ``bellhousing_bolts__half_*``).  'half_px' removes the +X half instead (plane cut,
   pieces ``*__half_px_kept/_removed``): pair it with section_side=+1 for an
   unobstructed half-section of the spinning clutch seen from +X.  If 'none' is not
   requested the whole bellhousing is not built.
``detail``   'high' (default) | 'low'
``hydraulic_cut``  bool (default False): half-pipe line pieces + brake-fluid cores
``section_rotating``  bool (default False): adds LIVE Boolean modifiers (static
   half-space cutter, x < X_CRANK removed, red cut faces) to every rotating clutch
   part, so a half-section of the spinning clutch can be shown (cut stays fixed in
   space while the parts turn).  Expensive per frame (Manifold boolean on each part)
   - only for section shots; toggled by presentation['section'].
``section_side``  -1 (default: x < X_CRANK removed, like the bellhousing cut) | +1
   (x > X_CRANK removed: unobstructed by fork / slave / hydraulics on the -X side)
``race_spin``  'always' (default, FACTS CLU-12: constant light contact) | 'contact'
   (race only turns while bearing travel > 0.3 mm, else holds its angle)
``collection`` collection name (default 'clutch')

Explode (``C.explode``; offsets along -Y at factor 1, applied to explode carriers so
spin/lift compose):  disc, pressure_plate (+straps), diaphragm_spring (+fulcrum),
cover, release_bearing (+race, fork, ball stud, guide tube).  The engine's flywheel
explodes by -0.10 m; these continue the sequence at ~65-80 mm spacing.

Presentation keys (all optional; per-frame arrays or scalars)
  explode   0..1
  variant   'none' | 'half' | 'half_px' (or per-frame list): CONSTANT-keyed visibility
            of the whole bellhousing vs its cut pieces
  removed   0..1 cv_opacity of the removed cutaway piece while 'half' is shown
            (default 0 = hidden)
  housing   0..1 cv_opacity of the (visible) bellhousing + its bolts (fade it away
            for exploded views; < 0.02 hides it)
  section   0/1 enables the live section modifiers (opts['section_rotating'])
  line_pulse  position 0..1 (master -> slave) of a glow pulse on the hydraulic line
            (cv_glow on the segments, their cut halves and fluid cores);
            line_pulse_width (default 0.12)
  hydraulic_cut  0/1 (needs opts['hydraulic_cut']): show the half-pipe line pieces +
            brake-fluid cores instead of the whole line

meta
----
  hydraulic_segments, hydraulic_fluid (if built), power_path, groups, y (axial
  stations, car frame), positions (car-frame pivot/slave/master actually used vs
  spec), fork (lever data), diaphragm (geometry + contact radius), spline
  (disc hub spline n/d_major/d_minor/flank/fill), disc_angle (how the disc phase is
  derived), explode_carriers, cutaway_pieces, shape_keys, variants, build_time.

Dimensions not in spec (typical 228 mm passenger-car clutch)
------------------------------------------------------------
facings 3.5 mm on a 1.4 mm cushion layer (12 wavy segments, 0.65 mm cushion travel),
18 radial grooves, 24 rivets per facing; hub 21 mm long, 23 splines 25.4 x 21.5 mm
(30 deg flanks, = gearbox input shaft) on a 41 mm barrel; hub flange 4.5 mm; side plates 1.2 mm; damper
springs 27 mm long, 13.3 mm OD on r = 53 mm; pressure plate ID 146 mm, fulcrum ridge
r = 105.2 mm; diaphragm 2.3 mm thick, OD 217 mm, 12 deg cone, fulcrum circle
r = 90 mm, finger tips r = 20.5 mm, 2.6 mm slots with 6 mm keyholes; cover 3.5 mm
sheet; release bearing contact radius from the lever geometry (~26 mm); guide tube
43 mm OD; slave 19.05 mm bore, master 15.87 mm bore (spec); 3/16 in steel line +
10 mm rubber hose; pedal clevis 50 mm from the pivot (6:1 with the 300 mm arm).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

import bpy  # noqa: I001
import bmesh  # noqa: F401
import numpy as np
from mathutils import Matrix, Vector
from mathutils import geometry as mgeo

from .. import kin, rig
from .. import meshutil as MU
from .. import spec as S

try:  # shared modules written in parallel; guarded
    from .. import gears as G
except Exception:  # pragma: no cover
    G = None
try:
    from .. import materials as MAT
except Exception:  # pragma: no cover
    MAT = None

PREFIX = "clu_"
TAU = 2.0 * math.pi
PI = math.pi
MM = 1e-3
DEG = PI / 180.0

# ===========================================================================
# Frame
# ===========================================================================
ROOT_LOC = np.array([S.X_CRANK, S.Y_DISC_CENTRE, S.Z_CRANK], dtype=float)


def _L(p):
    """Car-frame point -> root-local."""
    return np.asarray(p, dtype=float) - ROOT_LOC


def _ly(y):
    return float(y) - ROOT_LOC[1]


GAP = 0.04 * MM          # "touching" contact clearance (keeps collision checks clean)
DISC_PHASE = 0.0         # disc abs angle = theta_in + DISC_PHASE (gearbox spins the input shaft with theta_in)

# ===========================================================================
# Axial stack (car frame y)
# ===========================================================================
Y_FF = S.Y_FLYWHEEL_FACE                       # flywheel friction face
T_DISC = S.CLUTCH_DISC_THICKNESS               # clamped disc thickness
Y_PF = Y_FF - T_DISC                           # pressure-plate friction face (rest)
PP_T = S.PRESSURE_PLATE_THICKNESS
Y_PB = Y_PF - PP_T                             # pressure-plate back face
Y_RW = S.Y_GEARBOX_FRONT + 0.012               # bellhousing rear wall, front face
Y_RW1 = S.Y_GEARBOX_FRONT                      # rear wall rear face (= gearbox joint)

# ---------------- friction disc --------------------------------------------
FACING_RO = S.CLUTCH_DISC_OD / 2
FACING_RI = S.CLUTCH_DISC_ID / 2
FACING_T = S.CLUTCH_FACING_THICKNESS - GAP
CUSH_HALF = (S.CLUTCH_DISC_THICKNESS - 2 * S.CLUTCH_FACING_THICKNESS) / 2   # 0.7 mm
CUSHION_TRAVEL = 0.65 * MM
N_CUSHION = 12
N_GROOVES = 18
N_FACING_RIVETS = 24
# = gearbox input-shaft clutch splines (gearbox.meta['input_splines'] = (23, 25.4 mm, 21.5 mm),
#   gears.external_splines defaults: 30 deg flanks, fill 0.5; tooth 0 on +X, shaft spun with theta_in)
SPLINE = dict(n=S.CLUTCH_DISC_SPLINE_TEETH, d_major=25.4 * MM, d_minor=21.5 * MM,
              flank_angle=30.0 * DEG, fill=0.5)
HUB_LEN = 21.0 * MM
HUB_YC = -3.5 * MM                     # hub centre rel. disc centre (protrudes rearward)
HUB_OD = 41.0 * MM
HUB_FL = (0.0200, 0.0700, 4.5 * MM)    # hub flange r_in, r_out, thickness
SP_T = 1.2 * MM                        # side plate thickness
SP_Y = 2.4 * MM                        # side plate inner face offset from mid-plane
SP_RI, SP_RO = 0.0300, 0.0715
DAMPER_R = 0.053
DAMPER_LEN = 27.0 * MM
DAMPER_COIL_R = 5.2 * MM
DAMPER_WIRE_R = 1.45 * MM
DAMPER_TURNS = 5.5
WIN_RAD = (0.0460, 0.0600)             # spring window radial extent
STOP_PIN_R = 0.0655

# ---------------- pressure plate -------------------------------------------
PP_RI = 0.0730
PP_RO = S.PRESSURE_PLATE_OD / 2
R_RIM = 0.1042                          # fulcrum ridge (diaphragm rim contact) radius
RIDGE_H = 3.0 * MM
RIDGE_RHO = 1.5 * MM
Y_RIDGE_C = Y_PB - RIDGE_H + RIDGE_RHO  # ridge round-top centre

# ---------------- diaphragm spring -----------------------------------------
DS_T = 2.3 * MM
DS_RO = 0.1085
DS_CONE = 12.0 * DEG
R_F = 0.0900                            # fulcrum (pivot ring) circle
R_TIP = 0.0205
R_CURL = (0.0285, 0.0360)               # tip curl: slope 0 below r0, full cone above r1
N_FING = S.DIAPHRAGM_FINGERS
SLOT_W = 2.6 * MM
R_KEY = 0.0850                          # keyhole centre radius (rivet circle)
KEY_R = 3.0 * MM
RING_RHO = 1.5 * MM                     # pivot wire ring section radius
RIVET_R = 2.2 * MM
F0 = S.RELEASE_BEARING_TRAVEL * (S.CLUTCH_BITE_LO - S.CLUTCH_FREE_PLAY) / (1 - S.CLUTCH_FREE_PLAY)  # 1.37 mm
FINGER_REL = S.RELEASE_BEARING_TRAVEL - F0                                                       # 7.63 mm

# ---------------- cover ----------------------------------------------------
CT = 3.5 * MM
COVER_RO = S.CLUTCH_COVER_OD / 2
COVER_RWO = 0.1225                      # side wall outer radius (regular)
POCKET_DR = 0.0145                      # pocket bulge
POCKET_PSI = [45.0 * DEG + k * 120.0 * DEG for k in range(3)]
POCKET_HALF = 15.0 * DEG
POCKET_BLEND = 6.0 * DEG
COVER_BOLT_R = 0.1295                   # = engine flywheel cover-bolt circle
COVER_BOLT_PSI = [(15.0 + 60.0 * k) * DEG for k in range(6)]   # = engine flywheel holes
COVER_R_OPEN = 0.0805
SIDE_WIN_PSI = [(105.0 + 120.0 * k) * DEG for k in range(3)]
STRAP_R = 0.1245
STRAP_W = 13.0 * MM
STRAP_LEAF = 0.8 * MM
STRAP_SPAN = 18.0 * DEG
LUG_PSI = [p - STRAP_SPAN / 2 for p in POCKET_PSI]
Y_STRAP_F = Y_PB - 2.6 * MM             # strap front face (= lug back face)
Y_STRAP_B = Y_STRAP_F - 3 * STRAP_LEAF  # strap back face

# ---------------- release bearing / fork / slave ---------------------------
RHO_N = 2.5 * MM                        # bearing nose section radius
RHO_I = 4.0 * MM                        # fork prong contact bump radius
RHO_O = 5.0 * MM                        # fork outer contact bump radius
FORK_CX = 0.0697                        # pivot -> bearing contact line (X)
FORK_RATIO = S.RELEASE_FORK_RATIO_EFFECTIVE
X_PIVOT = S.X_CRANK - FORK_CX
X_SLAVE = X_PIVOT - FORK_RATIO * FORK_CX
Z_FORK = S.Z_CRANK
FORK_PRONG_Z = 0.032
GT_RI, GT_RO = 0.0175, 0.0215           # guide tube
SLV_BORE = S.SLAVE_CYL_BORE
SLV_R = 0.0155

# ---------------- master / pedal -------------------------------------------
PIV = np.array(S.CLUTCH_PEDAL_PIVOT, dtype=float)
PHI_MAX = S.CLUTCH_PEDAL_TRAVEL / S.PEDAL_ARM
R_CLEVIS = S.PEDAL_ARM / S.CLUTCH_PEDAL_RATIO      # 50 mm -> 6:1
B_CLEVIS = PHI_MAX / 2                               # clevis lean at rest (perpendicular mid-stroke)
X_MASTER = S.MASTER_CYL_POS[0]
Z_MASTER = PIV[2] - R_CLEVIS                         # pushrod line 50 mm below the pivot
Y_MFLANGE = S.Y_FIREWALL + 0.2 * MM                  # master flange rear face (engine side)
Y_MMOUTH = Y_MFLANGE + 5.0 * MM                      # bore mouth
Y_MPISTON0 = Y_MMOUTH + 6.8 * MM                     # master piston rear face at rest
Y_MFRONT = Y_MFLANGE + 0.068
FREE_PLAY_GAP = S.CLUTCH_FREE_PLAY * S.CLUTCH_PEDAL_TRAVEL / S.CLUTCH_PEDAL_RATIO   # 1.87 mm
MASTER_R = 0.0135
SPRING_B = np.array([0.0, -0.046, -0.004])           # return-spring anchor on the bracket (rel pivot)
SPRING_A_S = 0.062                                   # return-spring hook on the arm (from pivot)
PEDAL_TOP_DEPTH = 0.016                              # arm section depth at its upper extension

N_LINE = 12
PIPE_R = 2.4 * MM
HOSE_R = 5.0 * MM

DETAIL = "high"
_COL = None


# ===========================================================================
# Pure-geometry design (no bpy): diaphragm lever, contact radius, nose
# ===========================================================================

def _smooth(e0, e1, x):
    t = np.clip((np.asarray(x, float) - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3 - 2 * t)


def _bisect(f, lo, hi, n=80):
    flo = f(lo)
    fhi = f(hi)
    if flo * fhi > 0:
        raise ValueError(f"no sign change on [{lo}, {hi}]: {flo}, {fhi}")
    for _ in range(n):
        mid = 0.5 * (lo + hi)
        fm = f(mid)
        if fm * flo > 0:
            lo, flo = mid, fm
        else:
            hi = mid
    return 0.5 * (lo + hi)


def _design():
    """Diaphragm mid-surface, fulcrum, release rotation, finger-contact radius and
    bearing-nose placement.  All in car-frame y, radius r."""
    tc = math.tan(DS_CONE)
    rr = np.linspace(R_TIP - 0.002, DS_RO + 0.002, 4001)
    slope = tc * _smooth(R_CURL[0], R_CURL[1], rr)
    h_c = DS_T / (2 * math.cos(DS_CONE))
    y_m_rim = Y_RIDGE_C - (RIDGE_RHO + GAP) / math.cos(DS_CONE) - h_c
    # integrate slope from R_RIM
    cum = np.concatenate([[0.0], np.cumsum(0.5 * (slope[1:] + slope[:-1]) * np.diff(rr))])
    cum_rim = np.interp(R_RIM, rr, cum)
    ym = y_m_rim + (cum - cum_rim)
    hh = DS_T / (2 * np.cos(np.arctan(slope)))

    def y_mid(r):
        return np.interp(r, rr, ym)

    def h_of(r):
        return np.interp(r, rr, hh)

    F = np.array([R_F, float(y_mid(R_F))])
    n_front = np.array([-math.sin(DS_CONE), math.cos(DS_CONE)])
    C_ridge = np.array([R_RIM, Y_RIDGE_C])

    def rot(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[c, -s], [s, c]])

    def rim_gap(a, lift):
        Cp = C_ridge - np.array([0.0, lift])
        return float((Cp - F) @ (rot(a) @ n_front) - DS_T / 2 - RIDGE_RHO)

    # release angle: rim stays on the ridge when the plate has lifted fully
    alpha = _bisect(lambda a: rim_gap(a, S.PRESSURE_PLATE_LIFT) - GAP, -0.4, 0.0)
    R = rot(alpha)

    def rear_pt(r):
        return np.array([r, float(y_mid(r) - h_of(r))])

    def dy_release(r):
        Q = rear_pt(r)
        return float((F + R @ (Q - F))[1] - Q[1])

    # finger-contact radius: rear-surface point that moves FINGER_REL under 'release'
    r_c = _bisect(lambda r: dy_release(r) - FINGER_REL, R_TIP + 0.0005, R_CURL[0] - 0.0005)

    def g_bend(r):
        r = np.asarray(r, float)
        u = np.clip((R_F - r) / (R_F - r_c), 0.0, None)
        return u ** 1.5

    # nose radius: choose R_N so the gap to the deformed tip zone stays closest to GAP
    r0 = np.linspace(R_TIP, R_CURL[0] + 0.002, 300)
    Q0 = np.stack([r0, y_mid(r0) - h_of(r0)], axis=1)
    rel = (F + (Q0 - F) @ R.T) - Q0
    pp = np.linspace(0.0, 1.0, 41)
    geo = kin.clutch_geometry(pp)
    vb = np.clip(geo["finger"] / F0, 0.0, 1.0)
    vr = geo["plate_lift"] / S.PRESSURE_PLATE_LIFT
    y_rest_c = float(y_mid(r_c) - h_of(r_c))

    def gaps(RN):
        yc0 = y_rest_c - GAP - RHO_N
        out = []
        for i in range(len(pp)):
            P = Q0.copy()
            P[:, 1] += vb[i] * F0 * g_bend(r0)
            P += vr[i] * rel
            c = np.array([RN, yc0 + geo["bearing"][i]])
            d = np.min(np.hypot(P[:, 0] - c[0], P[:, 1] - c[1])) - RHO_N
            out.append(d)
        return np.array(out)

    best = None
    for RN in np.linspace(r_c - 0.002, r_c + 0.002, 81):
        gp = gaps(RN)
        cost = np.max(np.abs(gp - GAP)) + (1.0 if gp.min() < 0.005 * MM else 0.0)
        if best is None or cost < best[0]:
            best = (cost, RN, gp)
    _, R_N, gp = best
    y_n0 = y_rest_c - GAP                 # nose front point (rest)
    return dict(y_mid=y_mid, h_of=h_of, F=F, alpha=alpha, r_c=r_c, R_N=float(R_N), y_n0=y_n0,
                g_bend=g_bend, rot=R, nose_gap_range=(float(gp.min()), float(gp.max())),
                y_m_rim=y_m_rim, n_front=n_front, rim_gap_full=rim_gap(alpha, S.PRESSURE_PLATE_LIFT))


D = _design()
Y_N0 = D["y_n0"]                         # release-bearing nose front (rest), car frame
Y_PAD = Y_N0 - 0.030                     # carrier fork-pad rear faces
Y_P = Y_PAD - RHO_I - GAP                # fork plane = ball-stud centre (car y)
Y_E0 = Y_P + RHO_O + GAP                 # slave pushrod end face at rest
Y_GT_FRONT = Y_N0 - 0.022                # guide tube front end


def _y_cover_back():
    """Front surface of the cover back wall: behind the rear pivot ring."""
    Fm = D["F"]
    n = D["n_front"]
    ring_c = Fm - (DS_T / 2 + RING_RHO + GAP) * n
    return float(ring_c[1] - RING_RHO - GAP), Fm + (DS_T / 2 + RING_RHO + GAP) * n, ring_c


Y_CBW, RING_FRONT_C, RING_REAR_C = _y_cover_back()


def fork_angle(bearing):
    """Fork rotation (rad, right-handed about +Z) for a bearing travel (m)."""
    return np.arcsin(np.clip(np.asarray(bearing, float) / FORK_CX, -1, 1))


def slave_from_fork(gamma):
    """Slave pushrod travel implied by the fork angle (equals kin slave exactly)."""
    return FORK_RATIO * FORK_CX * np.sin(gamma)


def clevis_point(phi):
    """Pedal clevis-pin position (car frame) for pedal rotation phi (array ok)."""
    phi = np.asarray(phi, float)
    b = B_CLEVIS - phi
    return np.stack([np.full_like(b, PIV[0]), PIV[1] - R_CLEVIS * np.sin(b), PIV[2] - R_CLEVIS * np.cos(b)], -1)


ROD_TIP0 = Y_MPISTON0 - FREE_PLAY_GAP
_C0 = clevis_point(0.0)
ROD_L = float(math.hypot(ROD_TIP0 - _C0[1], Z_MASTER - _C0[2]))


def pushrod_pose(phi):
    """(clevis xyz (n,3), rod angle about +X (n,), tip y (n,)) for pedal angle phi."""
    C = clevis_point(phi)
    dz = Z_MASTER - C[..., 2]
    tip_y = C[..., 1] + np.sqrt(ROD_L ** 2 - dz ** 2)
    ang = np.arctan2(dz, tip_y - C[..., 1])
    return C, ang, tip_y


def spring_hook_a(phi):
    a = S.PEDAL_REST_ANGLE - np.asarray(phi, float)
    return np.stack([np.zeros_like(a), -SPRING_A_S * np.sin(a), -SPRING_A_S * np.cos(a)], -1)


# ===========================================================================
# Small bpy helpers
# ===========================================================================

def _nm(key):
    return PREFIX + key


def _seg(n):
    k = 1.0 if DETAIL == "high" else 0.5
    return max(8, int(round(n * k / 4.0)) * 4)


def _mat(name):
    return MU.get_material(name)


def _xform(obj, M):
    obj.data.transform(M)
    if M.to_3x3().determinant() < 0:          # mirror: keep normals outward
        obj.data.flip_normals()
    obj.data.update()
    return obj


def _translate(obj, v):
    return _xform(obj, Matrix.Translation(Vector(tuple(map(float, v)))))


def _rot_psi(psi):
    """Matrix turning local +X toward +Z by psi (profile angle) about +Y."""
    return Matrix.Rotation(-psi, 4, "Y")


def _lathe(key, prof, seg, material, closed=True, smooth=35.0, phase=0.0):
    ob = MU.lathe(_nm(key), prof, seg, closed=closed, caps=not closed, collection=_COL,
                  material=material, smooth_angle=smooth, phase=phase)
    return ob


def _cyl(key, p0, p1, r, seg=24, material=None, chamfer=0.0, r_in=0.0):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    L = float(np.linalg.norm(p1 - p0))
    ob = MU.cylinder(_nm(key), r, 0.0, L, segments=seg, chamfer=chamfer, inner_radius=r_in, collection=_COL,
                     material=material)
    d = Vector(tuple((p1 - p0) / L))
    q = Vector((0, 1, 0)).rotation_difference(d)
    _xform(ob, Matrix.Translation(Vector(tuple(p0))) @ q.to_matrix().to_4x4())
    return ob


def _box(key, c, size, r=0.0, material=None, M=None):
    ob = MU.rounded_box(_nm(key), tuple(size), radius=r, segments=2, center=(0, 0, 0), collection=_COL,
                        material=material)
    T = Matrix.Translation(Vector(tuple(map(float, c))))
    _xform(ob, (M @ T) if M is not None else T)
    return ob


def _sphere(key, c, r, seg=16, material=None, hemi=None):
    """UV sphere (or hemisphere: hemi = +1/-1 keeps the +Y/-Y half) at c."""
    n = max(6, seg // 2)
    if hemi is None:
        prof = [(0.0, -r)] + [(r * math.sin(PI * i / n), -r * math.cos(PI * i / n)) for i in range(1, n)] + [(0.0, r)]
    else:
        prof = [(r * math.cos(0.5 * PI * i / (n // 2)), hemi * r * math.sin(0.5 * PI * i / (n // 2)))
                for i in range(0, n // 2)] + [(0.0, hemi * r)]
        prof = [(0.0, 0.0)] + prof
    ob = MU.lathe(_nm(key), prof, seg, closed=False, caps=True, collection=_COL, material=material)
    _translate(ob, c)
    return ob


def _bool(target, operands, op="DIFFERENCE", solver="MANIFOLD", mode="INDEX"):
    """Apply a boolean with a collection operand; operands are deleted."""
    ops = [o for o in operands if o is not None]
    if not ops:
        return target
    tmp = bpy.data.collections.new("clu_bool_tmp")
    bpy.context.scene.collection.children.link(tmp)
    for o in ops:
        for c in list(o.users_collection):
            c.objects.unlink(o)
        tmp.objects.link(o)
    unlinked = False
    if not target.users_collection:
        _COL.objects.link(target)
        unlinked = True
    mod = target.modifiers.new("clu_bool", "BOOLEAN")
    mod.operation = op
    mod.operand_type = "COLLECTION"
    mod.collection = tmp
    mod.material_mode = mode
    me = None
    for sv in ((solver, "EXACT") if solver != "EXACT" else ("EXACT",)):
        mod.solver = sv
        dg = bpy.context.evaluated_depsgraph_get()
        dg.update()
        cand = bpy.data.meshes.new_from_object(target.evaluated_get(dg))
        if len(cand.polygons) > 0:
            me = cand
            break
        bpy.data.meshes.remove(cand)
    target.modifiers.remove(mod)
    if me is not None:
        # NB: never materials.clear() here - it resets every face's material index
        old = target.data
        target.data = me
        if old.users == 0:
            bpy.data.meshes.remove(old)
    for o in ops:
        m_ = o.data
        bpy.data.objects.remove(o)
        if m_ is not None and m_.users == 0:
            bpy.data.meshes.remove(m_)
    bpy.data.collections.remove(tmp)
    if unlinked:
        _COL.objects.unlink(target)
    return target


def _join(key, objs, smooth=None):
    objs = [o for o in objs if o is not None]
    ob = MU.join(_nm(key) + "_tmpjoin", objs, collection=_COL, smooth_angle=smooth)
    ob.name = _nm(key)
    ob.data.name = _nm(key)
    return ob


def _face_mat(obj, material, pred):
    """Assign `material` to faces whose centre (x, y, z arrays) satisfies pred."""
    me = obj.data
    n = len(me.polygons)
    cen = np.zeros(n * 3)
    me.polygons.foreach_get("center", cen)
    cen = cen.reshape(-1, 3)
    sel = pred(cen[:, 0], cen[:, 1], cen[:, 2])
    MU.assign_material(obj, material, faces=np.nonzero(sel)[0])
    return obj


def _verts(obj):
    me = obj.data
    co = np.zeros(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    return co.reshape(-1, 3)


def _add_shape_key(obj, name, disp):
    """Basis + one shape key with per-vertex displacement disp (n,3)."""
    if obj.data.shape_keys is None:
        obj.shape_key_add(name="Basis", from_mix=False)
    kb = obj.shape_key_add(name=name, from_mix=False)
    co = _verts(obj) + disp
    kb.data.foreach_set("co", co.astype(np.float32).ravel())
    kb.slider_min = 0.0
    kb.slider_max = 1.0
    return kb


def _finish(obj, parent, smooth=None):
    obj.parent = parent
    if smooth is not None:
        MU.smooth_by_angle(obj, smooth, keep_sharp=True)
    rig.set_presentation(obj, 1.0, 0.0)
    return obj


def _empty(key, parent, loc=(0, 0, 0)):
    return rig.empty(_nm(key), loc=tuple(map(float, loc)), parent=parent, col=_COL, size=0.02)


def _radial_box(key, psi, r0, r1, y0, y1, half_w, rr=0.0, material=None):
    """Box from r0 to r1 (along the ray psi), y0..y1, tangential half width."""
    ob = _box(key, ((r0 + r1) / 2, (y0 + y1) / 2, 0.0), (r1 - r0, y1 - y0, 2 * half_w), r=rr, material=material)
    return _xform(ob, _rot_psi(psi))


def _sweep_poly(key, path, section, material=None, smooth=40.0, caps=True, up=(1.0, 0.0, 0.0)):
    """Sweep a closed 2D section (k,2) [(u, v) in (side, normal) units] along a 3D
    path; section scale may vary: section(s) -> (k,2) for s in [0,1]."""
    P = np.asarray(path, float)
    n = len(P)
    T = np.zeros_like(P)
    T[1:-1] = P[2:] - P[:-2]
    T[0] = P[1] - P[0]
    T[-1] = P[-1] - P[-2]
    T /= np.linalg.norm(T, axis=1)[:, None]
    upv = np.asarray(up, float)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    s /= max(s[-1], 1e-12)
    mb = MU.MeshBuilder()
    rings = []
    for i in range(n):
        Nv = upv - T[i] * np.dot(upv, T[i])
        Nv /= np.linalg.norm(Nv)
        Bv = np.cross(T[i], Nv)
        sec = section(s[i]) if callable(section) else np.asarray(section, float)
        pts = P[i] + sec[:, 0:1] * Nv + sec[:, 1:2] * Bv
        rings.append(mb.verts(pts))
    for i in range(n - 1):
        mb.bridge(rings[i], rings[i + 1])
    if caps:
        mb.face(list(rings[0]))
        mb.face(list(rings[-1]))
    ob = mb.to_object(_nm(key), _COL, smooth_angle=smooth, materials=[material] if material else None)
    return ob


def _rect_section(w, h, c=0.0, n_round=0):
    """Rectangle (w along u, h along v) with chamfer c."""
    if c <= 0:
        return np.array([(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)])
    return np.array([(-w / 2 + c, -h / 2), (w / 2 - c, -h / 2), (w / 2, -h / 2 + c), (w / 2, h / 2 - c),
                     (w / 2 - c, h / 2), (-w / 2 + c, h / 2), (-w / 2, h / 2 - c), (-w / 2, -h / 2 + c)])


def _hex_prism(key, c, axis, r, h, material="steel_dark", chamfer=0.0004, hole=0.0):
    """Hexagon prism (bolt head / nut) centred on c along `axis` (unit), height h;
    hole > 0 makes a nut with a bore of that radius."""
    a = np.linspace(0, TAU, 6, endpoint=False) + PI / 6
    poly = np.stack([r * np.cos(a), r * np.sin(a)], 1)
    holes = None
    if hole > 0:
        b = np.linspace(0, TAU, 16, endpoint=False)
        holes = [np.stack([hole * np.cos(b), hole * np.sin(b)], 1)]
    ob = MU.extrude_polygon(_nm(key), poly, -h / 2, h / 2, holes=holes, chamfer=chamfer, collection=_COL,
                            material=material, smooth_angle=30.0)
    q = Vector((0, 1, 0)).rotation_difference(Vector(tuple(axis)))
    _xform(ob, Matrix.Translation(Vector(tuple(map(float, c)))) @ q.to_matrix().to_4x4())
    return ob


def _bolt(key, head_c, axis, r_head=0.0068, h_head=0.0055, r_shank=0.004, l_shank=0.012, flange=True,
          material="steel_dark"):
    """Hex flange bolt: head centred at head_c, shank going along -axis... axis
    points from the head AWAY from the joint (head outward); shank goes opposite."""
    ax = np.asarray(axis, float) / np.linalg.norm(axis)
    hc = np.asarray(head_c, float)
    parts = [_hex_prism(key + "_h", hc, ax, r_head, h_head, material)]
    if flange:
        parts.append(_cyl(key + "_f", hc - ax * (h_head / 2), hc - ax * (h_head / 2 - 0.0012), r_head * 1.18,
                          seg=_seg(20), material=material))
    if l_shank > 0:
        parts.append(_cyl(key + "_s", hc - ax * (h_head / 2), hc - ax * (h_head / 2 + l_shank), r_shank,
                          seg=_seg(12), material=material))
    return parts


# ===========================================================================
# CDT sheet (thin curved sheet from a 2D outline) - used for the diaphragm
# ===========================================================================

def _cdt_sheet(key, loops, spacing, y_front, y_back, material, r_band=None):
    """Closed thin sheet: planform region (outer loop + hole loops, (x,z) metres)
    triangulated with interior Steiner points, mapped to 3D with y_front(r) /
    y_back(r).  Returns object."""
    import shapely
    from shapely.geometry import Polygon
    outer, holes = loops[0], loops[1:]
    poly = Polygon(outer, holes)
    assert poly.is_valid, "diaphragm outline invalid"
    allp = np.concatenate(loops)
    edges = []
    k0 = 0
    for lp in loops:
        n = len(lp)
        edges += [(k0 + i, k0 + (i + 1) % n) for i in range(n)]
        k0 += n
    rmin, rmax = r_band
    rs = np.arange(rmin + spacing / 2, rmax, spacing * 0.9)
    st = []
    for r in rs:
        m = max(12, int(TAU * r / spacing))
        a = np.arange(m) * TAU / m + (0.5 * TAU / m if (len(st) % 2) else 0.0)
        st.append(np.stack([r * np.cos(a), r * np.sin(a)], 1))
    st = np.concatenate(st)
    inner = poly.buffer(-0.6 * spacing)
    keep = shapely.contains_xy(inner, st[:, 0], st[:, 1])
    st = st[keep]
    verts = [Vector((float(x), float(z))) for x, z in np.concatenate([allp, st])]
    vo, eo, fo, ov, oe, of = mgeo.delaunay_2d_cdt(verts, edges, [], 0, 1e-9)
    V2 = np.array([(v[0], v[1]) for v in vo])
    F = np.array([f for f in fo if len(f) == 3], dtype=np.int64)
    cen = V2[F].mean(axis=1)
    F = F[shapely.contains_xy(poly, cen[:, 0], cen[:, 1])]
    # orientation: make every triangle CCW in (x, z)
    a = V2[F]
    cr = (a[:, 1, 0] - a[:, 0, 0]) * (a[:, 2, 1] - a[:, 0, 1]) - (a[:, 1, 1] - a[:, 0, 1]) * (a[:, 2, 0] - a[:, 0, 0])
    F[cr < 0] = F[cr < 0][:, [0, 2, 1]]
    inv = {}
    for oi, lst in enumerate(ov):
        for ii in lst:
            inv[ii] = oi
    r = np.hypot(V2[:, 0], V2[:, 1])
    nv = len(V2)
    mb = MU.MeshBuilder()
    fr_ = mb.verts(np.stack([V2[:, 0], y_front(r), V2[:, 1]], 1))
    bk_ = mb.verts(np.stack([V2[:, 0], y_back(r), V2[:, 1]], 1))
    mb.faces(fr_[F])
    mb.faces(bk_[F[:, [0, 2, 1]]])
    k0 = 0
    for lp in loops:
        n = len(lp)
        idx = np.array([inv[k0 + i] for i in range(n)])
        mb.bridge(fr_[idx], bk_[idx])
        k0 += n
    ob = mb.to_object(_nm(key), _COL, smooth_angle=30.0, materials=[material])
    assert len(ob.data.vertices) == 2 * nv
    return ob


# ===========================================================================
# Parts
# ===========================================================================

def _build_disc():
    """Friction disc (root-local coords: y = 0 at the disc mid-plane)."""
    seg = _seg(192)
    parts = []
    # --- facings -------------------------------------------------------------
    for side in (+1, -1):
        y_in = side * CUSH_HALF
        y_out = side * (CUSH_HALF + FACING_T)
        c = 0.35 * MM
        prof = [(FACING_RI + c, y_out), (FACING_RO - c, y_out), (FACING_RO, y_out - side * c),
                (FACING_RO, y_in + side * 0.2 * MM), (FACING_RO - 0.2 * MM, y_in),
                (FACING_RI + 0.2 * MM, y_in), (FACING_RI, y_in + side * 0.2 * MM), (FACING_RI, y_out - side * c)]
        if side < 0:
            prof = prof[::-1]
        fac = _lathe(f"facing{side:+d}", prof, seg, "friction")
        cut = []
        ph = 0.0 if side > 0 else 0.5
        for k in range(N_GROOVES):
            psi = (k + 0.25) * TAU / N_GROOVES
            ya, yb_ = y_out - side * 0.9 * MM, y_out + side * 0.002
            cut.append(_radial_box(f"gr{side}{k}", psi, FACING_RI - 0.003, FACING_RO + 0.003,
                                   min(ya, yb_), max(ya, yb_), 1.0 * MM))
        for k in range(N_FACING_RIVETS):
            psi = (k + ph) * TAU / N_FACING_RIVETS + 0.5 * TAU / N_GROOVES * 0.0
            rr = 0.0860 if k % 2 == 0 else 0.1010
            x, z = rr * math.cos(psi), rr * math.sin(psi)
            yb = y_in + side * 1.3 * MM
            cut.append(_cyl(f"rh{side}{k}", (x, yb, z), (x, y_out + side * 0.002, z), 2.6 * MM, seg=_seg(16)))
            cut.append(_cyl(f"rc{side}{k}", (x, y_out - side * 0.6 * MM, z), (x, y_out + side * 0.002, z), 3.4 * MM,
                            seg=_seg(16)))
            # rivet head sitting at the bottom of the hole
            parts.append(_sphere(f"rv{side}{k}", (x, yb - side * 0.9 * MM, z), 2.2 * MM, seg=_seg(12),
                                 material="brass"))
        _bool(fac, cut)
        parts.append(fac)
    # --- cushion segments (wavy spring steel paddles) ------------------------
    nu, nr = (24, 5) if DETAIL == "high" else (12, 3)
    t = 0.5 * MM
    amp = CUSH_HALF - t / 2 - 0.03 * MM
    for k in range(N_CUSHION):
        psi0 = k * TAU / N_CUSHION
        span = 0.80 * TAU / N_CUSHION
        mb = MU.MeshBuilder()
        u = np.linspace(-0.5, 0.5, nu + 1)
        rr = np.linspace(0.0810, FACING_RO - 1.5 * MM, nr + 1)
        grid_f, grid_b = [], []
        for r in rr:
            ps = psi0 + u * span
            yw = amp * np.sin(2 * TAU * (u + 0.5)) * _smooth(0.081, 0.088, r)
            grid_f.append(mb.verts(np.stack([r * np.cos(ps), yw + t / 2, r * np.sin(ps)], 1)))
            grid_b.append(mb.verts(np.stack([r * np.cos(ps), yw - t / 2, r * np.sin(ps)], 1)))
        for i in range(nr):
            mb.bridge(grid_f[i], grid_f[i + 1], closed=False)
            mb.bridge(grid_b[i + 1], grid_b[i], closed=False)
        # side walls at u = -0.5 / +0.5 and the radial ends
        mb.bridge(np.array([g[0] for g in grid_f]), np.array([g[0] for g in grid_b]), closed=False)
        mb.bridge(np.array([g[-1] for g in grid_b]), np.array([g[-1] for g in grid_f]), closed=False)
        mb.bridge(grid_b[0], grid_f[0], closed=False)
        mb.bridge(grid_f[-1], grid_b[-1], closed=False)
        parts.append(mb.to_object(_nm(f"cush{k}"), _COL, smooth_angle=50.0, materials=["steel_dark"]))
    # --- disc plate (front) with joggle out to the cushion ring, retainer (rear)
    c = 0.3 * MM
    fy0, fy1 = SP_Y, SP_Y + SP_T
    dp = [(SP_RI, fy0 + c), (SP_RI + c, fy0), (SP_RO - 0.0008, fy0), (0.0745, -SP_T / 2),
          (0.0820, -SP_T / 2), (0.0820, SP_T / 2), (0.0745 + 0.0012, SP_T / 2), (SP_RO + 0.0004, fy1),
          (SP_RI + c, fy1), (SP_RI, fy1 - c)]
    dplate = _lathe("dplate", dp, seg, "steel_dark")
    rp = [(SP_RI, -fy0 - c), (SP_RI, -fy1 + c), (SP_RI + c, -fy1), (SP_RO - c, -fy1), (SP_RO, -fy1 + c),
          (SP_RO, -fy0 - c), (SP_RO - c, -fy0), (SP_RI + c, -fy0)]
    rplate = _lathe("rplate", rp, seg, "steel_dark")
    r0f, r1f, tf = HUB_FL
    hf = [(r0f, -tf / 2), (r1f - c, -tf / 2), (r1f, -tf / 2 + c), (r1f, tf / 2 - c), (r1f - c, tf / 2), (r0f, tf / 2)]
    flange = _lathe("hubfl", hf, seg, "steel_machined")
    wl = DAMPER_LEN + 0.6 * MM
    for ob, ytag in ((dplate, "d"), (rplate, "r"), (flange, "f")):
        cut = []
        for k in range(S.CLUTCH_DAMPER_SPRINGS):
            psi = (30.0 + 60.0 * k) * DEG
            cut.append(_box(f"win{ytag}{k}", ((WIN_RAD[0] + WIN_RAD[1]) / 2, 0.0, 0.0),
                            (WIN_RAD[1] - WIN_RAD[0], 0.03, wl), r=1.0 * MM, M=_rot_psi(psi)))
        if ytag == "f":       # stop-pin slots in the hub flange
            for k in range(S.CLUTCH_DAMPER_SPRINGS):
                psi = 60.0 * k * DEG
                cut.append(_box(f"slot{k}", (STOP_PIN_R, 0.0, 0.0), (6.0 * MM, 0.03, 12.0 * MM), r=2.9 * MM,
                                M=_rot_psi(psi)))
        _bool(ob, cut)
        parts.append(ob)
    # pressed lips around the spring windows on both side plates
    lip_r = DAMPER_COIL_R + DAMPER_WIRE_R + 0.35 * MM + 0.6 * MM
    for k in range(S.CLUTCH_DAMPER_SPRINGS):
        psi = (30.0 + 60.0 * k) * DEG
        for side in (+1, -1):
            for edge in (-1, +1):
                arc = np.linspace(0.30, 1.05, 7) if edge > 0 else np.linspace(PI - 1.05, PI - 0.30, 7)
                sec = [(math.cos(t) * (lip_r + 0.5 * MM), math.sin(t) * (lip_r + 0.5 * MM)) for t in arc]
                sec += [(math.cos(t) * (lip_r - 0.6 * MM), math.sin(t) * (lip_r - 0.6 * MM)) for t in arc[::-1]]
                sec = np.array(sec)
                sec[:, 1] *= side
                L2 = DAMPER_LEN / 2 - 2.0 * MM
                ob = _sweep_poly(f"lip{k}{side}{edge}", [(DAMPER_R, 0.0, -L2), (DAMPER_R, 0.0, L2)], sec,
                                 material="steel_dark", smooth=40.0, up=(1.0, 0.0, 0.0))
                _xform(ob, _rot_psi(psi))
                parts.append(ob)
    # stop pins (spacer rivets through both side plates)
    for k in range(S.CLUTCH_DAMPER_SPRINGS):
        psi = 60.0 * k * DEG
        x, z = STOP_PIN_R * math.cos(psi), STOP_PIN_R * math.sin(psi)
        parts.append(_cyl(f"pin{k}", (x, -fy1 - 0.6 * MM, z), (x, fy1 + 0.6 * MM, z), 2.4 * MM, seg=_seg(16),
                          material="steel_machined", chamfer=0.4 * MM))
    # hub barrel with the internal splines
    if G is not None:
        hub = G.internal_splines(_nm("hub"), SPLINE["n"], SPLINE["d_major"], SPLINE["d_minor"], HUB_LEN,
                                 outer_d=HUB_OD, flank_angle=SPLINE["flank_angle"], fill=SPLINE["fill"],
                                 material="steel_machined", collection=_COL, detail=DETAIL)
    else:  # pragma: no cover
        hub = MU.cylinder(_nm("hub"), HUB_OD / 2, -HUB_LEN / 2, HUB_LEN / 2, inner_radius=SPLINE["d_major"] / 2,
                          collection=_COL, material="steel_machined")
    _translate(hub, (0.0, HUB_YC, 0.0))
    parts.append(hub)
    disc = _join("disc", parts)
    MU.smooth_by_angle(disc, 35.0, keep_sharp=False)
    # cushion shape key 'free': facings spread apart by CUSHION_TRAVEL
    co = _verts(disc)
    r = np.hypot(co[:, 0], co[:, 2])
    y = co[:, 1]
    dy = np.zeros_like(y)
    fac = (r >= FACING_RI - 0.1 * MM) & (np.abs(y) >= CUSH_HALF - 0.01 * MM)
    dy[fac] = np.sign(y[fac]) * CUSHION_TRAVEL / 2
    cus = (r >= 0.0815) & (np.abs(y) < CUSH_HALF - 0.01 * MM)
    dy[cus] = y[cus] / CUSH_HALF * CUSHION_TRAVEL / 2
    disp = np.zeros_like(co)
    disp[:, 1] = dy
    _add_shape_key(disc, "free", disp)
    return disc


def _build_damper_springs():
    obs = []
    nturn = DAMPER_TURNS
    n = int(nturn * (16 if DETAIL == "high" else 10))
    for k in range(S.CLUTCH_DAMPER_SPRINGS):
        psi = (30.0 + 60.0 * k) * DEG
        t = np.linspace(0, 1, n + 1)
        a = TAU * nturn * t
        zl = (t - 0.5) * (DAMPER_LEN - 2 * DAMPER_WIRE_R)
        pts = np.stack([DAMPER_R + DAMPER_COIL_R * np.cos(a), DAMPER_COIL_R * np.sin(a), zl], 1)
        ob = MU.tube_along(_nm(f"dsp{k}"), pts, DAMPER_WIRE_R, segments=_seg(10), bend_radius=0.0, caps=True,
                           collection=_COL, material="steel_dark", smooth_angle=60.0)
        _xform(ob, _rot_psi(psi))
        obs.append(ob)
    return _join("damper_springs", obs)


def _build_pressure_plate():
    seg = _seg(192)
    yf, yb = _ly(Y_PF), _ly(Y_PB)
    c = 0.8 * MM
    yr = _ly(Y_RIDGE_C)
    ridge = [(R_RIM + 0.0034, yb)]
    for th in np.linspace(0.0, -PI, 13):
        ridge.append((R_RIM + RIDGE_RHO * math.cos(th), yr + RIDGE_RHO * math.sin(th)))
    ridge += [(R_RIM - 0.0034, yb)]
    prof = [(PP_RI + c, yf), (PP_RO - c, yf), (PP_RO, yf - c), (PP_RO, yb + c), (PP_RO - c, yb)] + ridge + \
           [(PP_RI + c, yb), (PP_RI, yb + c), (PP_RI, yf - c)]
    pl = _lathe("pressure_plate", prof, seg, "cast_iron")
    parts = [pl]
    for k, psi in enumerate(LUG_PSI):
        parts.append(_radial_box(f"lug{k}", psi, PP_RO - 0.002, STRAP_R + 0.0065, _ly(Y_STRAP_F), yb + 0.010,
                                 11.0 * MM, rr=1.5 * MM, material="cast_iron"))
    # balancing notches on the OD between the lugs
    cut = [_radial_box(f"bal{k}", psi + 60 * DEG, PP_RO - 0.0025, PP_RO + 0.004, yb + 0.004, yf - 0.004,
                       7.0 * MM, rr=2.0 * MM) for k, psi in enumerate(LUG_PSI)]
    pl = _join("pressure_plate", parts)
    _bool(pl, cut)
    MU.smooth_by_angle(pl, 35.0)
    # machined friction face (+ ID / OD lands)
    _face_mat(pl, "steel_machined", lambda x, y, z: (np.abs(y - yf) < 1e-5))
    return pl


def _strap_path(psi_l):
    a = np.array([STRAP_R * math.cos(psi_l), STRAP_R * math.sin(psi_l)])
    b = np.array([STRAP_R * math.cos(psi_l + STRAP_SPAN), STRAP_R * math.sin(psi_l + STRAP_SPAN)])
    return a, b


def _build_straps():
    """3 sets of 3 tangential leaf straps; local x along the strap from cover end."""
    obs = []
    yl0 = _ly(Y_STRAP_F)
    nl = 24 if DETAIL == "high" else 12
    for k, psi_l in enumerate(LUG_PSI):
        a, b = _strap_path(psi_l)      # a = lug (plate) end, b = cover end
        d = (a - b)
        L = float(np.linalg.norm(d))
        e = d / L
        nrm = np.array([-e[1], e[0]])
        ext = 7.0 * MM
        for leaf in range(3):
            y1 = yl0 - GAP - leaf * STRAP_LEAF
            y0 = y1 - STRAP_LEAF + 0.03 * MM
            mb = MU.MeshBuilder()
            us = np.linspace(-ext, L + ext, nl + 1)
            ring = []
            for u in us:
                p = b + e * u
                ww = STRAP_W / 2
                q = [p - nrm * ww, p + nrm * ww]
                ring.append(mb.verts([(q[0][0], y0, q[0][1]), (q[1][0], y0, q[1][1]),
                                      (q[1][0], y1, q[1][1]), (q[0][0], y1, q[0][1])]))
            for i in range(nl):
                mb.bridge(ring[i], ring[i + 1])
            mb.face(list(ring[0])[::-1])
            mb.face(list(ring[-1]))
            obs.append(mb.to_object(_nm(f"strap{k}{leaf}"), _COL, smooth_angle=30.0, materials=["steel_dark"]))
        # rivet heads: at the lug end on the strap's back face, at the cover end on the front face
        yb = yl0 - GAP - 3 * STRAP_LEAF + 0.03 * MM
        obs.append(_sphere(f"srv_a{k}", (a[0], yb, a[1]), 3.2 * MM, seg=_seg(16), material="steel_machined", hemi=-1))
        obs.append(_sphere(f"srv_b{k}", (b[0], yl0 - GAP, b[1]), 3.2 * MM, seg=_seg(16), material="steel_machined",
                           hemi=+1))
    st = _join("straps", obs)
    # shape key 'lift': S-bend, the lug end follows the plate by -PRESSURE_PLATE_LIFT
    co = _verts(st)
    disp = np.zeros_like(co)
    for k, psi_l in enumerate(LUG_PSI):
        a, b = _strap_path(psi_l)
        L = float(np.linalg.norm(a - b))
        e = (a - b) / L
        rel = co[:, [0, 2]] - b
        u = np.clip((rel @ e) / L, 0.0, 1.0)
        near = np.abs(rel @ np.array([-e[1], e[0]])) < STRAP_W / 2 + 3 * MM
        near &= (rel @ e > -10 * MM) & (rel @ e < L + 10 * MM)
        # flat (riveted) over the cover pedestal and over the plate lug, S-bend between
        s = _smooth(6.5 * MM / L, 1.0 - 12.0 * MM / L, u)
        disp[near, 1] = -S.PRESSURE_PLATE_LIFT * s[near]
    _add_shape_key(st, "lift", disp)
    return st


def _diaphragm_loops():
    tau = TAU / N_FING
    n_out = 360 if DETAIL == "high" else 180
    a = np.arange(n_out) * TAU / n_out
    outer = np.stack([DS_RO * np.cos(a), DS_RO * np.sin(a)], 1)
    w2 = SLOT_W / 2
    u_t = math.sqrt(R_TIP ** 2 - w2 ** 2)
    u_m = R_KEY - math.sqrt(KEY_R ** 2 - w2 ** 2)
    n_edge = 40 if DETAIL == "high" else 20
    n_key = 16 if DETAIL == "high" else 10
    n_tip = 4
    inner = []
    for k in range(N_FING):
        ps = (k + 0.5) * tau
        cs, sn = math.cos(ps), math.sin(ps)
        ehat, vhat = np.array([cs, sn]), np.array([-sn, cs])
        # tip arc of finger k (from its lower edge to its upper edge = lower edge of slot k+0.5)
        a_lo = (k - 0.5) * tau + math.asin(w2 / R_TIP)
        a_hi = ps - math.asin(w2 / R_TIP)
        for t in np.linspace(a_lo, a_hi, n_tip + 2)[1:-1]:
            inner.append((R_TIP * math.cos(t), R_TIP * math.sin(t)))
        # lower edge of slot outward
        for u in np.linspace(u_t, u_m, n_edge):
            inner.append(tuple(ehat * u - vhat * w2))
        # keyhole arc around the far side
        ph_l = math.atan2(-w2, -(R_KEY - u_m))
        ph_u = math.atan2(w2, -(R_KEY - u_m))
        if ph_l > 0:
            ph_l -= TAU
        for ph in np.linspace(ph_l, ph_u + 0.0 if ph_u > ph_l else ph_u + TAU, n_key + 2)[1:-1]:
            p = ehat * (R_KEY + KEY_R * math.cos(ph)) + vhat * (KEY_R * math.sin(ph))
            inner.append(tuple(p))
        # upper edge of slot inward
        for u in np.linspace(u_m, u_t, n_edge):
            inner.append(tuple(ehat * u + vhat * w2))
    return [outer, np.array(inner)]


def _build_diaphragm():
    loops = _diaphragm_loops()
    ym, hf = D["y_mid"], D["h_of"]
    y0 = ROOT_LOC[1]
    sp = 1.8 * MM if DETAIL == "high" else 3.2 * MM
    ob = _cdt_sheet("diaphragm_spring", loops, sp, lambda r: ym(r) + hf(r) - y0, lambda r: ym(r) - hf(r) - y0,
                    "steel_dark", r_band=(R_TIP, DS_RO))
    # shape keys
    co = _verts(ob)
    r = np.hypot(co[:, 0], co[:, 2])
    y = co[:, 1] + y0
    # 'bend': fingers bend (rim still), contact point moves +F0
    disp = np.zeros_like(co)
    disp[:, 1] = F0 * np.where(r < R_F, D["g_bend"](r), 0.0)
    _add_shape_key(ob, "bend", disp)
    # 'release': rigid rotation about the fulcrum circle
    Fm = D["F"]
    Rm = D["rot"]
    P = np.stack([r, y], 1) - Fm
    Pn = P @ Rm.T + Fm
    scale = np.where(r > 1e-9, Pn[:, 0] / np.maximum(r, 1e-9), 1.0)
    disp = np.zeros_like(co)
    disp[:, 0] = co[:, 0] * (scale - 1)
    disp[:, 2] = co[:, 2] * (scale - 1)
    disp[:, 1] = Pn[:, 1] - y
    _add_shape_key(ob, "release", disp)
    return ob


def _build_fulcrum():
    """Pivot (fulcrum) wire rings + shoulder rivets with front heads."""
    seg = _seg(160)
    obs = []
    for nm, c in (("ring_f", RING_FRONT_C), ("ring_r", RING_REAR_C)):
        n = 10
        prof = [(c[0] + RING_RHO * math.cos(TAU * i / n), _ly(c[1]) + RING_RHO * math.sin(TAU * i / n))
                for i in range(n)]
        obs.append(_lathe(nm, prof, seg, "steel_ground", closed=True, smooth=60.0))
    tau = TAU / N_FING
    yh = RING_FRONT_C[1] + RING_RHO + GAP        # front head rear face
    for k in range(N_FING):
        ps = (k + 0.5) * tau
        x, z = R_KEY * math.cos(ps), R_KEY * math.sin(ps)
        obs.append(_cyl(f"rvs{k}", (x, _ly(Y_CBW) + GAP, z), (x, _ly(yh) + 0.0008, z), RIVET_R, seg=_seg(12),
                        material="steel_machined"))
        obs.append(_cyl(f"rvh{k}", (x, _ly(yh), z), (x, _ly(yh) + 0.9 * MM, z), 3.3 * MM, seg=_seg(16),
                        material="steel_machined", chamfer=0.4 * MM))
    return _join("fulcrum", obs)


def _pocket_b(psi):
    """0..1 pocket bulge weight for profile angle(s) psi."""
    psi = np.asarray(psi, float)
    b = np.zeros_like(psi)
    for pc in POCKET_PSI:
        d = np.abs((psi - pc + PI) % TAU - PI)
        b = np.maximum(b, 1.0 - _smooth(POCKET_HALF, POCKET_HALF + POCKET_BLEND, d))
    return b


def _cover_profile(db):
    """Closed (r, y_local) cross-section loop of the cover for a bulge db (m)."""
    yf0 = _ly(Y_FF - GAP)
    yf1 = yf0 - CT
    yb0 = _ly(Y_CBW)
    yb1 = yb0 - CT
    rwo = COVER_RWO + db
    rwi = rwo - CT
    rci = 4.0 * MM
    rco = rci + CT
    pts = [(COVER_RO, yf0 - 0.6 * MM), (COVER_RO - 0.6 * MM, yf0), (rwi + 1.2 * MM, yf0), (rwi, yf0 - 1.2 * MM)]
    for i in range(7):                                  # inner corner
        th = PI / 2 * i / 6
        pts.append((rwi - rci + rci * math.cos(th), yb0 + rci - rci * math.sin(th)))
    pts.append((COVER_R_OPEN + CT / 2, yb0))
    for i in range(1, 8):                               # rolled opening edge
        th = PI / 2 + PI * i / 8
        pts.append((COVER_R_OPEN + CT / 2 + CT / 2 * math.cos(th), (yb0 + yb1) / 2 + CT / 2 * math.sin(th)))
    pts.append((COVER_R_OPEN + CT / 2, yb1))
    for i in range(7):                                  # outer corner
        th = -PI / 2 + PI / 2 * i / 6
        pts.append((rwi - rci + rco * math.cos(th), yb0 + rci + rco * math.sin(th)))
    pts.append((rwo, yf1 - 1.6 * MM))
    pts.append((rwo + 0.6 * MM, yf1 - 0.5 * MM))
    pts.append((rwo + 1.6 * MM, yf1))
    pts.append((COVER_RO - 0.6 * MM, yf1))
    pts.append((COVER_RO, yf1 + 0.6 * MM))
    return pts


def _build_cover():
    seg = _seg(288)
    base = np.array(_cover_profile(0.0))
    bulge = np.array(_cover_profile(POCKET_DR))
    psi = np.arange(seg) * TAU / seg
    b = _pocket_b(psi)
    mb = MU.MeshBuilder()
    rings = []
    for i in range(len(base)):
        r = base[i, 0] + (bulge[i, 0] - base[i, 0]) * b
        y = np.full(seg, base[i, 1])
        rings.append(mb.verts(np.stack([r * np.cos(psi), y, r * np.sin(psi)], 1)))
    for i in range(len(rings)):
        mb.bridge(rings[i], rings[(i + 1) % len(rings)])
    cov = mb.to_object(_nm("cover"), _COL, smooth_angle=40.0, materials=["paint_black"])
    yf1 = _ly(Y_FF - GAP) - CT
    yb0 = _ly(Y_CBW)
    yb1 = yb0 - CT
    # windows: three side windows + three pocket windows (straps visible)
    cut = []
    for k, ps in enumerate(SIDE_WIN_PSI):
        cut.append(_radial_box(f"cw{k}", ps, 0.112, 0.135, yb0 + 7.0 * MM, yf1 - 4.0 * MM, 15.0 * MM, rr=4.0 * MM))
    for k, ps in enumerate(POCKET_PSI):
        cut.append(_radial_box(f"cp{k}", ps, 0.129, 0.150, yb0 + 6.0 * MM, yf1 - 3.0 * MM, 17.0 * MM, rr=4.0 * MM))
    _bool(cov, cut)
    parts = [cov]
    # strap pedestals (pressed bosses) + their rivet heads behind the wall
    for k, psi_l in enumerate(LUG_PSI):
        a, bpt = _strap_path(psi_l)
        y_top = _ly(Y_STRAP_B) - GAP
        parts.append(_cyl(f"ped{k}", (bpt[0], yb0 - 0.5 * MM, bpt[1]), (bpt[0], y_top, bpt[1]), 5.5 * MM,
                          seg=_seg(20), material="paint_black", chamfer=0.6 * MM))
        parts.append(_sphere(f"pedh{k}", (bpt[0], yb1, bpt[1]), 3.4 * MM, seg=_seg(16), material="steel_machined",
                             hemi=-1))
    # stamped stiffening ribs on the back wall (between the strap pedestals)
    for k in range(6):
        ps = (15.0 + 60.0 * k) * DEG
        rib = _radial_box(f"crib{k}", ps, 0.0935, 0.1135, yb1 - 1.6 * MM, yb1 + 0.6 * MM, 3.2 * MM, rr=1.4 * MM,
                          material="paint_black")
        parts.append(rib)
    # rear heads of the fulcrum rivets
    tau = TAU / N_FING
    for k in range(N_FING):
        ps = (k + 0.5) * tau
        parts.append(_sphere(f"rvt{k}", (R_KEY * math.cos(ps), yb1, R_KEY * math.sin(ps)), 3.3 * MM,
                             seg=_seg(14), material="steel_machined", hemi=-1))
    # cover bolts into the flywheel
    for k, ps in enumerate(COVER_BOLT_PSI):
        x, z = COVER_BOLT_R * math.cos(ps), COVER_BOLT_R * math.sin(ps)
        parts += _bolt(f"cb{k}", (x, yf1 - 0.0055 / 2, z), (0, -1, 0), r_head=6.5 * MM,
                       h_head=5.5 * MM, r_shank=3.5 * MM, l_shank=CT + 0.008)
    cv = _join("cover", parts)
    MU.smooth_by_angle(cv, 40.0, keep_sharp=True)
    return cv


def _build_release_bearing():
    seg = _seg(96)
    yn = _ly(Y_N0)
    c = 0.4 * MM
    # stationary carrier: sleeve + cup back wall + cup/outer ring
    prof = [(0.0218, yn - 0.046), (0.0252 - c, yn - 0.046), (0.0252, yn - 0.046 + c), (0.0252, yn - 0.0235),
            (0.0405 - c, yn - 0.0235), (0.0405, yn - 0.0235 + c), (0.0405, yn - 0.0045),
            (0.0400, yn - 0.0035), (0.0350, yn - 0.0035), (0.0345, yn - 0.0045), (0.0345, yn - 0.0205),
            (0.0218, yn - 0.0205)]
    car = _lathe("release_bearing", prof, seg, "steel_dark")
    _face_mat(car, "steel_ground", lambda x, y, z: (y > yn - 0.0060) & (np.hypot(x, z) > 0.0340))
    parts = [car]
    seal = [(0.0290, yn - 0.0058), (0.0346, yn - 0.0058), (0.0346, yn - 0.0042), (0.0292, yn - 0.0045)]
    parts.append(_lathe("seal", seal, seg, "rubber"))
    for sgn in (+1, -1):
        zc = sgn * (0.0245 + 0.0385) / 2
        parts.append(_box(f"pad{sgn}", (0.0, (_ly(Y_PAD) + yn - 0.0215) / 2, zc),
                          (0.016, (yn - 0.0215) - _ly(Y_PAD), 0.0140), r=1.2 * MM, material="steel_machined"))
    rb = _join("release_bearing", parts)
    MU.smooth_by_angle(rb, 35.0, keep_sharp=True)
    # rotating inner race with the finger-contact nose
    RN = D["R_N"]
    ync = yn - RHO_N
    nose = []
    for i in range(13):
        th = PI - PI * i / 12
        nose.append((RN + RHO_N * math.cos(th), ync + RHO_N * math.sin(th)))
    rprof = [(0.0228, yn - 0.016), (0.0228, ync - 0.5 * MM)]
    rprof += [(max(0.0228, p[0]), p[1]) for p in nose]
    rprof += [(0.0285, ync - 1.0 * MM), (0.0285, yn - 0.016 + 0.4 * MM), (0.0281, yn - 0.016)]
    # dedupe
    clean = []
    for p in rprof:
        if not clean or abs(p[0] - clean[-1][0]) + abs(p[1] - clean[-1][1]) > 1e-7:
            clean.append(p)
    race = _lathe("bearing_race", clean, seg, "steel_ground", smooth=50.0)
    return rb, race


def _build_guide_tube():
    seg = _seg(64)
    y0 = _ly(Y_RW)
    y1 = _ly(Y_GT_FRONT)
    c = 0.6 * MM
    prof = [(GT_RI + c, y1), (GT_RO - c, y1), (GT_RO, y1 - c), (GT_RO, y0 + 0.0055 + 1.5 * MM),
            (GT_RO + 1.5 * MM, y0 + 0.0055), (0.048 - c, y0 + 0.0055), (0.048, y0 + 0.0055 - c), (0.048, y0),
            (GT_RI, y0), (GT_RI, y1 - c)]
    gt = _lathe("guide_tube", prof, seg, "steel_machined")
    parts = [gt]
    for k in range(3):
        ps = (90.0 + 120.0 * k) * DEG
        x, z = 0.040 * math.cos(ps), 0.040 * math.sin(ps)
        parts += _bolt(f"gtb{k}", (x, y0 + 0.0055 + 2.4 * MM, z), (0, 1, 0), r_head=5.0 * MM, h_head=4.0 * MM,
                       r_shank=3.0 * MM, l_shank=0.0, flange=True)
    ob = _join("guide_tube", parts)
    MU.smooth_by_angle(ob, 35.0, keep_sharp=True)
    return ob


def _fork_outline():
    """Fork sheet planform in (x, z) relative to the pivot (pivot at 0)."""
    from shapely.geometry import Polygon, Point
    from shapely.ops import unary_union
    cx = FORK_CX
    dox = -FORK_RATIO * FORK_CX
    zp = FORK_PRONG_Z
    yoke = Polygon([(cx - 0.052, -0.041), (cx + 0.0005, -0.041), (cx + 0.0050, -0.037), (cx + 0.0050, -0.0262),
                    (cx - 0.031, -0.0262), (cx - 0.031, 0.0262), (cx + 0.0050, 0.0262), (cx + 0.0050, 0.037),
                    (cx + 0.0005, 0.041), (cx - 0.052, 0.041)])
    arm = Polygon([(cx - 0.050, -0.016), (0.0, -0.0150), (dox, -0.0115), (dox - 0.006, -0.0105),
                   (dox - 0.0105, -0.006), (dox - 0.0105, 0.006), (dox - 0.006, 0.0105), (dox, 0.0115),
                   (0.0, 0.0150), (cx - 0.050, 0.016)])
    boss = Point(0.0, 0.0).buffer(0.0165, 32)
    sh = unary_union([yoke, arm, boss]).buffer(1.5 * MM, 8).buffer(-1.5 * MM, 8)
    sh = sh.simplify(0.05 * MM)
    return np.asarray(sh.exterior.coords)[:-1], zp


def _build_fork():
    """Release fork, local coords relative to the pivot (object origin at the ball)."""
    outline, zp = _fork_outline()
    th = 3.0 * MM
    a = np.linspace(0, TAU, 32, endpoint=False)
    hole = np.stack([7.7 * MM * np.cos(a), 7.7 * MM * np.sin(a)], 1)
    plate = MU.extrude_polygon(_nm("fork_plate"), outline, -th / 2, th / 2, holes=[hole], chamfer=0.5 * MM,
                               collection=_COL, material="paint_black", smooth_angle=40.0)
    parts = [plate]
    cx = FORK_CX
    dox = -FORK_RATIO * FORK_CX
    # channel flanges bent rearward along the arm edges
    for sgn in (+1, -1):
        path = []
        for x in np.linspace(cx - 0.050, dox + 0.010, 14):
            w = float(np.interp(x, [dox, 0.0, cx - 0.050], [0.0115, 0.0150, 0.016]))
            path.append((x, -th / 2 - 4.0 * MM, sgn * (w - 1.25 * MM)))
        sec = _rect_section(8.0 * MM, 2.5 * MM, c=0.4 * MM)
        fl = _sweep_poly(f"fork_fl{sgn}", path, sec, material="paint_black", up=(0.0, 1.0, 0.0))
        parts.append(fl)
    # ball cup (dome over the front half of the ball)
    n = 10
    prof = [(7.7 * MM * math.cos(0.5 * PI * i / n), 7.7 * MM * math.sin(0.5 * PI * i / n)) for i in range(n + 1)]
    prof += [(10.6 * MM * math.cos(0.5 * PI * i / n), 10.6 * MM * math.sin(0.5 * PI * i / n)) for i in range(n, -1, -1)]
    prof = [(max(p[0], 0.0), p[1]) for p in prof]
    prof[n] = (0.0, prof[n][1])
    prof[n + 1] = (0.0, prof[n + 1][1])
    cup = MU.lathe(_nm("fork_cup"), prof, _seg(32), closed=True, caps=False, collection=_COL,
                   material="paint_black")
    parts.append(cup)
    # contact bumps (cylinders about Z, centred on the pivot plane y = 0)
    for sgn in (+1, -1):
        parts.append(_cyl(f"fork_bi{sgn}", (cx, 0.0, sgn * zp - 5.0 * MM), (cx, 0.0, sgn * zp + 5.0 * MM), RHO_I,
                          seg=_seg(24), material="steel_machined", chamfer=0.6 * MM))
    parts.append(_cyl("fork_bo", (dox, 0.0, -7.0 * MM), (dox, 0.0, 7.0 * MM), RHO_O, seg=_seg(24),
                      material="steel_machined", chamfer=0.6 * MM))
    # retaining spring clip gripping the stud neck behind the ball (two legs to the sheet)
    clip = MU.extrude_polygon(_nm("fork_clip"), _rect_section(0.022, 0.024, 2 * MM), -8.6 * MM, -7.6 * MM,
                              holes=[np.stack([5.8 * MM * np.cos(a), 5.8 * MM * np.sin(a)], 1)], collection=_COL,
                              material="steel_dark")
    parts.append(clip)
    for sgn in (+1, -1):
        parts.append(_box(f"fork_cl{sgn}", (0.0, -5.0 * MM, sgn * 0.0112), (0.016, 7.4 * MM, 0.0011), r=0.3 * MM,
                          material="steel_dark"))
    fk = _join("fork", parts)
    MU.smooth_by_angle(fk, 40.0, keep_sharp=True)
    return fk


def _build_ball_stud():
    """Ball stud (root-local)."""
    p = _L((X_PIVOT, Y_P, Z_FORK))
    parts = [_sphere("ball", p, 7.5 * MM, seg=_seg(24), material="steel_ground")]
    parts.append(_cyl("neck", (p[0], p[1] - 5.0 * MM, p[2]), (p[0], p[1] - 11.0 * MM, p[2]), 4.0 * MM,
                      seg=_seg(20), material="steel_machined"))
    parts.append(_cyl("stud", (p[0], p[1] - 10.5 * MM, p[2]), (p[0], _ly(Y_RW) + 6.0 * MM, p[2]), 5.5 * MM,
                      seg=_seg(20), material="steel_machined", chamfer=0.8 * MM))
    parts.append(_hex_prism("stud_hex", (p[0], _ly(Y_RW) + 4.0 * MM, p[2]), (0, 1, 0), 9.0 * MM, 6.0 * MM))
    ob = _join("ball_stud", parts)
    MU.smooth_by_angle(ob, 35.0, keep_sharp=True)
    return ob


BH_IN = [(-0.2920, 0.158), (-0.3130, 0.158), (-0.3175, 0.149), (-0.3680, 0.149), (-0.3860, 0.124),
         (Y_RW, 0.110)]
BH_WALL = 6.0 * MM
FLANGE_NOTCH_PSI = -20.0 * DEG          # engine exhaust downpipe passes here behind the block
FLANGE_NOTCH_HALF = 24.0 * DEG
FLANGE_BOLT_PSI = [(22.5 + 45.0 * k) * DEG for k in range(7)]   # 8th (-22.5 deg) dropped for the notch


def bh_inner_r(y):
    ys = [p[0] for p in BH_IN][::-1]
    rs = [p[1] for p in BH_IN][::-1]
    return float(np.interp(y, ys, rs))


REAR_BORE_R = 0.0455                    # rear-wall bore, sits on the gearbox case's locating spigot (r 44 mm)
REAR_MAIN_R = 0.137                     # rear flange: main circle ...
REAR_LOBE = (0.0, -S.GEARBOX_CENTRE_DISTANCE, 0.078)   # ... + countershaft lobe (x, z, r) rel. main axis


def _rear_flange_outline(n=160):
    from shapely.geometry import Point
    from shapely.ops import unary_union
    hull = unary_union([Point(0, 0).buffer(REAR_MAIN_R, 64),
                        Point(REAR_LOBE[0], REAR_LOBE[1]).buffer(REAR_LOBE[2], 48)]).convex_hull
    return np.asarray(hull.exterior.coords)[:-1]


def _rear_bolts():
    """Rear flange bolt positions (x, z rel. main axis): 6 on the outline, 11 mm in."""
    from shapely.geometry import LineString, Polygon
    poly = Polygon(_rear_flange_outline())
    out = []
    for deg in (30.0, 90.0, 150.0, 210.0, 270.0, 330.0):
        a = deg * DEG
        ray = LineString([(0, 0), (0.3 * math.cos(a), 0.3 * math.sin(a))])
        hit = ray.intersection(poly.exterior)
        pts = [hit] if hit.geom_type == "Point" else list(getattr(hit, "geoms", []))
        r = max(math.hypot(p.x, p.y) for p in pts)
        out.append(((r - 0.0115) * math.cos(a), (r - 0.0115) * math.sin(a)))
    return out


def _build_bellhousing():
    seg = _seg(192)
    y0 = _ly(S.Y_BELLHOUSING_FRONT - GAP)
    yfl = _ly(S.Y_BELLHOUSING_FRONT - 0.012)
    yr0, yr1 = _ly(Y_RW), _ly(Y_RW1 + GAP)
    c = 1.0 * MM
    # shell (closed revolved profile: front flange, bell, rear wall with bore)
    inner = [(r, _ly(y)) for (y, r) in BH_IN]
    outer = [(r + BH_WALL, _ly(y)) for (y, r) in BH_IN[1:-1]]
    prof = [(0.158, y0), (0.164, y0), (0.164, yfl - 3 * MM)]
    prof += [p for p in outer if p[1] < yfl - 3 * MM]
    prof += [(0.110 + BH_WALL + 0.004, yr0 + 0.004), (0.120, yr1 + 0.001), (REAR_BORE_R + 0.5 * MM, yr1 + 0.001),
             (REAR_BORE_R, yr1 + 0.0015), (REAR_BORE_R, yr0)]
    prof += inner[::-1][:-1]
    # sort sanity: profile is a closed loop (outer down the bell, inner back up)
    bh = _lathe("bellhousing", prof, seg, "cast_aluminium")
    adds = []
    # front flange: round except a notch clearing the engine's exhaust downpipe (+X, below the axis)
    ps = np.arange(seg * 2) * TAU / (seg * 2)
    dn = np.abs((ps - FLANGE_NOTCH_PSI + PI) % TAU - PI)
    rf = 0.188 - (0.188 - 0.1665) * (1.0 - _smooth(FLANGE_NOTCH_HALF, FLANGE_NOTCH_HALF + 8 * DEG, dn))
    outl = np.stack([rf * np.cos(ps), rf * np.sin(ps)], 1)
    a_ = np.linspace(0, TAU, seg, endpoint=False)
    fl = MU.extrude_polygon(_nm("bh_flange"), outl, yfl, y0, holes=[np.stack([0.158 * np.cos(a_), 0.158 * np.sin(a_)], 1)],
                            chamfer=1.0 * MM, collection=_COL, material="cast_aluminium")
    adds.append(fl)
    adds.append(_lathe("bh_fillet", [(0.1635, yfl + 0.001), (0.1700, yfl + 0.001), (0.1635, yfl - 0.006)], seg,
                       "cast_aluminium"))
    # ribs on the bell
    for k, ps in enumerate((50.0, 90.0, 130.0, 230.0, 270.0, 310.0)):
        ps *= DEG
        pts = []
        ys = np.linspace(S.Y_BELLHOUSING_FRONT - 0.012, Y_RW - 0.004, 12)
        for y in ys:
            pts.append((bh_inner_r(y) + BH_WALL - 1.0 * MM, _ly(y)))
        hh = [0.0105 if y > -0.37 else 0.0085 for y in ys]
        poly = [(p[0], p[1]) for p in pts] + [(p[0] + h, p[1]) for p, h in zip(pts[::-1], hh[::-1])]
        poly = np.array(poly)
        rib = MU.extrude_polygon(_nm(f"rib{k}"), poly, -2.5 * MM, 2.5 * MM, chamfer=0.8 * MM, collection=_COL,
                                 material="cast_aluminium")
        # extrude_polygon builds in XZ (x = r, z = y_local) along Y: map (x, y, z) -> (x, z, y)
        _xform(rib, Matrix(((1, 0, 0, 0), (0, 0, 1, 0), (0, 1, 0, 0), (0, 0, 0, 1))))
        _xform(rib, _rot_psi(ps))
        adds.append(rib)
    # rear flange plate: covers the gearbox case front face (main bore + countershaft lobe)
    hull = _rear_flange_outline()
    a_ = np.linspace(0, TAU, seg, endpoint=False)
    rfp = MU.extrude_polygon(_nm("bh_rear"), hull, yr1, yr0, holes=[np.stack([REAR_BORE_R * np.cos(a_),
                                                                              REAR_BORE_R * np.sin(a_)], 1)],
                             chamfer=1.0 * MM, collection=_COL, material="cast_aluminium")
    adds.append(rfp)
    # front flange bolt bosses
    for k, ps in enumerate(FLANGE_BOLT_PSI):
        x, z = 0.180 * math.cos(ps), 0.180 * math.sin(ps)
        adds.append(_cyl(f"fb{k}", (x, yfl - 0.008, z), (x, y0, z), 0.0115, seg=_seg(24), material="cast_aluminium",
                         chamfer=1.0 * MM))
    # rear flange bosses (on the flange outline)
    for k, (x, z) in enumerate(_rear_bolts()):
        adds.append(_cyl(f"rb{k}", (x, yr1, z), (x, yr0 + 0.010, z), 0.0095, seg=_seg(20), material="cast_aluminium",
                         chamfer=1.0 * MM))
    # slave cylinder lug (outside, in front of the fork window) + gusset
    xs, zs = X_SLAVE - ROOT_LOC[0], Z_FORK - ROOT_LOC[2]
    yl0, yl1 = _ly(Y_E0) + 4.0 * MM, _ly(Y_E0) + 12.0 * MM
    r_in_l = bh_inner_r(Y_E0 + 0.008)
    adds.append(_box("slug", ((xs - 0.022 + (-r_in_l - 0.002)) / 2, (yl0 + yl1) / 2, zs),
                     (abs(xs - 0.022 + r_in_l + 0.002), yl1 - yl0, 0.075), r=2.0 * MM, material="cast_aluminium"))
    for sgn in (+1, -1):
        yg = Y_E0 + 0.034
        gus = np.array([(-r_in_l - 0.002, yl1 - 0.002), (xs - 0.006, yl1 - 0.002), (xs + 0.012, yl1 + 0.006),
                        (-bh_inner_r(yg) - 0.003, _ly(yg))])
        g = MU.extrude_polygon(_nm(f"sgus{sgn}"), gus, -2.5 * MM, 2.5 * MM, chamfer=0.8 * MM, collection=_COL,
                               material="cast_aluminium")
        _xform(g, Matrix(((1, 0, 0, 0), (0, 0, 1, 0), (0, 1, 0, 0), (0, 0, 0, 1))))
        _translate(g, (0.0, 0.0, zs + sgn * 0.035))
        adds.append(g)
    # ball-stud boss on the rear wall
    pv = _L((X_PIVOT, Y_P, Z_FORK))
    adds.append(_cyl("sboss", (pv[0], yr0 - 0.002, pv[2]), (pv[0], yr0 + 0.006, pv[2]), 0.011, seg=_seg(24),
                     material="cast_aluminium", chamfer=1.0 * MM))
    _bool(bh, adds, op="UNION")
    # holes: fork window, slave lug bore + bolt holes, flange bolt holes
    cut = []
    yp = _ly(Y_P)
    cut.append(_box("fwin", (-(bh_inner_r(Y_P) + 0.003), yp - 0.0075, zs), (0.030, 0.029, 0.034), r=3.0 * MM))
    cut.append(_cyl("slh", (xs, yl0 - 0.002, zs), (xs, yl1 + 0.002, zs), 0.0125, seg=_seg(24)))
    for sgn in (+1, -1):
        cut.append(_cyl(f"slb{sgn}", (xs, yl0 - 0.002, zs + sgn * 0.024), (xs, yl1 + 0.002, zs + sgn * 0.024),
                        0.0042, seg=_seg(12)))
    for k, ps in enumerate(FLANGE_BOLT_PSI):
        x, z = 0.180 * math.cos(ps), 0.180 * math.sin(ps)
        cut.append(_cyl(f"fbh{k}", (x, yfl - 0.010, z), (x, y0 + 0.002, z), 0.0055, seg=_seg(12)))
    _bool(bh, cut)
    MU.smooth_by_angle(bh, 35.0, keep_sharp=True)
    # machined faces: front/rear joint faces
    _face_mat(bh, "machined_aluminium", lambda x, y, z: (np.abs(y - y0) < 1e-5) | (np.abs(y - yr1) < 1e-5))
    # bolts (front flange: head on the rear face pointing rearward; rear flange: head on the front face)
    bolts = []
    for k, ps in enumerate(FLANGE_BOLT_PSI):
        x, z = 0.180 * math.cos(ps), 0.180 * math.sin(ps)
        bolts += _bolt(f"bfb{k}", (x, yfl - 0.008 - 3.0 * MM - 0.05 * MM, z), (0, -1, 0), r_head=7.5 * MM, h_head=6.0 * MM,
                       r_shank=0.0, l_shank=0.0)
    for k, (x, z) in enumerate(_rear_bolts()):
        bolts += _bolt(f"brb{k}", (x, yr0 + 0.010 + 2.75 * MM + 0.05 * MM, z), (0, 1, 0), r_head=7.0 * MM, h_head=5.5 * MM,
                       r_shank=0.0, l_shank=0.0)
    return bh, _join("bellhousing_bolts", bolts)


def _keep_boxes():
    """Regions of the -X half kept by the 'half' cutaway (slave mount, hose boss)."""
    xs = X_SLAVE - ROOT_LOC[0]
    y0 = _ly(Y_E0) - 0.002
    y1 = _ly(Y_E0 + 0.040)
    return [((-0.30, -0.088), (y0, y1), (-0.046, 0.046))]


def _cutter_box(key, x, y, z):
    return MU.rounded_box(_nm(key), (x[1] - x[0], y[1] - y[0], z[1] - z[0]), radius=0.0,
                          center=((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2), collection=_COL)


def _cut_with_cutter(obj, cutter):
    """Manifold DIFFERENCE with a cutter object; faces on the cutter surface get
    section_cut (local fix of meshutil._cut_with_object, which loses material
    indices because it clears the result's material slots after assigning)."""
    from mathutils.bvhtree import BVHTree
    sec = MU.get_material("section_cut")
    tmp = cutter.copy()
    tmp.data = cutter.data.copy()
    _COL.objects.link(tmp)
    cv = [v.co.copy() for v in cutter.data.vertices]
    cf = [tuple(p.vertices) for p in cutter.data.polygons]
    _bool(obj, [tmp], op="DIFFERENCE", solver="MANIFOLD", mode="INDEX")
    me = obj.data
    if sec.name not in [m.name for m in me.materials if m]:
        me.materials.append(sec)
    si = [i for i, m in enumerate(me.materials) if m and m.name == sec.name][0]
    bvh = BVHTree.FromPolygons(cv, cf)
    n = len(me.polygons)
    mi = np.zeros(n, dtype=np.int32)
    me.polygons.foreach_get("material_index", mi)
    cen = np.zeros(n * 3)
    me.polygons.foreach_get("center", cen)
    cen = cen.reshape(-1, 3)
    tol = 2e-6
    flat = np.zeros(n, dtype=bool)
    for i in range(n):
        loc, _nrm, _idx, dist = bvh.find_nearest(Vector(tuple(cen[i])))
        if loc is not None and dist < tol:
            mi[i] = si
            flat[i] = True
    me.polygons.foreach_set("material_index", mi)
    sm = np.zeros(n, dtype=bool)
    me.polygons.foreach_get("use_smooth", sm)
    me.polygons.foreach_set("use_smooth", sm & ~flat)
    me.update()
    m_ = cutter.data
    bpy.data.objects.remove(cutter)
    if m_.users == 0:
        bpy.data.meshes.remove(m_)
    return int(flat.sum())


def _stepped_half_cut(kept, rem):
    """kept: remove x < 0 except the keep boxes; rem: the complement.  Section faces red."""
    big = ((-0.6, 0.0), (-0.6, 0.6), (-0.6, 0.6))
    ca = _cutter_box("cutA", *big)
    _bool(ca, [_cutter_box(f"kA{i}", *b) for i, b in enumerate(_keep_boxes())], op="DIFFERENCE")
    _cut_with_cutter(kept, ca)
    cb = _cutter_box("cutB", (0.0, 0.6), big[1], big[2])
    _bool(cb, [_cutter_box(f"kB{i}", *b) for i, b in enumerate(_keep_boxes())], op="UNION")
    _cut_with_cutter(rem, cb)


def _hose_bracket_geom():
    """Pipe/hose junction bracket on the firewall (body side, root-local points)."""
    J = _L(LINE_JUNCTION)
    yfw = _ly(S.Y_FIREWALL) + 0.0005
    return dict(J=J, base=np.array([J[0], yfw, J[2]]))


# ---------------- slave cylinder ---------------------------------------------
def _slave_axis():
    return np.array([X_SLAVE, 0.0, Z_FORK]) - np.array([ROOT_LOC[0], 0.0, ROOT_LOC[2]])


def _build_slave():
    ax = _slave_axis()
    ye = _ly(Y_E0)
    seg = _seg(48)
    c = 0.8 * MM
    yb0, yb1 = ye + 0.012, ye + 0.050
    rb = SLV_BORE / 2
    body = [(rb + 0.6 * MM, yb0 + GAP), (SLV_R - c, yb0 + GAP), (SLV_R, yb0 + c), (SLV_R, yb1), (SLV_R - 2 * MM, yb1 + 0.004),
            (0.009, yb1 + 0.006), (0.0, yb1 + 0.006), (0.0, ye + 0.049), (rb, ye + 0.049), (rb, yb0 + 0.6 * MM)]
    b = _lathe("slv_body", body, seg, "cast_aluminium", closed=True)
    _translate(b, (ax[0], 0.0, ax[2]))
    parts = [b]
    # mounting flange with two ears + bolts into the lug
    a = np.linspace(0, TAU, 24, endpoint=False)
    fl = MU.extrude_polygon(_nm("slv_fl"), np.array([(-0.012, -0.030), (0.012, -0.030), (0.012, 0.030),
                                                     (-0.012, 0.030)]), yb0 + GAP, yb0 + 0.007, chamfer=0.8 * MM,
                            holes=[np.stack([(rb + 0.6 * MM) * np.cos(a), (rb + 0.6 * MM) * np.sin(a)], 1)],
                            collection=_COL, material="cast_aluminium")
    _translate(fl, (ax[0], 0.0, ax[2]))
    parts.append(fl)
    for sgn in (+1, -1):
        parts += _bolt(f"slvb{sgn}", (ax[0], yb0 + 0.007 + 2.6 * MM, ax[2] + sgn * 0.024), (0, 1, 0),
                       r_head=6.0 * MM, h_head=5.0 * MM, r_shank=0.0, l_shank=0.0)
    # rubber boot through the lug bore
    boot = [(0.0045, ye + 0.0035), (0.0062, ye + 0.0035)]
    for i in range(1, 6):
        yy = ye + 0.0035 + i * (yb0 - ye - 0.0035) / 6
        boot.append((0.0095 if i % 2 else 0.0078, yy))
    boot += [(0.0098, yb0 + 0.001), (0.0045, yb0 + 0.001)]
    bt = _lathe("slv_boot", boot, seg, "rubber")
    _translate(bt, (ax[0], 0.0, ax[2]))
    parts.append(bt)
    # inlet union (copper) on top at the front + bleed nipple on the outboard side
    yu = yb1 - 0.006
    U0 = np.array([ax[0], yu, ax[2] + SLV_R - 1.0 * MM])
    parts.append(_hex_prism("slv_un", U0 + [0, 0, 0.0055], (0, 0, 1), 6.5 * MM, 7.0 * MM, material="copper"))
    parts.append(_cyl("slv_un2", U0 + [0, 0, 0.009], U0 + [0, 0, 0.016], 4.2 * MM, seg=_seg(16), material="copper"))
    dn = np.array([-0.70710678, 0.0, 0.70710678])
    N0 = np.array([ax[0] - SLV_R * 0.55, yb1 - 0.016, ax[2] + SLV_R * 0.55])
    parts.append(_hex_prism("slv_bn", N0 + dn * 0.004, dn, 4.6 * MM, 5.0 * MM, material="steel_machined"))
    parts.append(_cyl("slv_bn2", N0 + dn * 0.0065, N0 + dn * 0.014, 2.6 * MM, seg=_seg(12), material="steel_machined"))
    parts.append(_cyl("slv_bcap", N0 + dn * 0.011, N0 + dn * 0.017, 3.6 * MM, seg=_seg(12), material="rubber",
                      chamfer=0.8 * MM))
    ob = _join("slave_cylinder", parts)
    MU.smooth_by_angle(ob, 35.0, keep_sharp=True)
    return ob, U0 + np.array([0, 0, 0.016])


def _build_slave_pushrod():
    ax = _slave_axis()
    ye = _ly(Y_E0)
    seg = _seg(32)
    prof = [(0.0, ye), (0.0052, ye), (0.0056, ye + 0.0004), (0.0056, ye + 0.0020), (0.0040, ye + 0.0028),
            (0.0040, ye + 0.030), (SLV_BORE / 2 - 0.15 * MM, ye + 0.030), (SLV_BORE / 2 - 0.15 * MM, ye + 0.044),
            (0.0, ye + 0.044)]
    pr = _lathe("slave_pushrod", prof, seg, "steel_machined", closed=False)
    _translate(pr, (ax[0], 0.0, ax[2]))
    return pr


# ---------------- master cylinder ---------------------------------------------
def _build_master():
    seg = _seg(48)
    xm, zm = X_MASTER - ROOT_LOC[0], Z_MASTER - ROOT_LOC[2]
    y0, y1 = _ly(Y_MMOUTH), _ly(Y_MFRONT)
    c = 1.0 * MM
    rb = S.MASTER_CYL_BORE / 2
    body = [(rb + 0.6 * MM, y0), (MASTER_R - c, y0), (MASTER_R, y0 + c), (MASTER_R, y1 - 0.002),
            (0.0095, y1 + 0.002), (0.0075, y1 + 0.004), (0.0, y1 + 0.004), (0.0, y1 - 0.004), (rb, y1 - 0.004),
            (rb, y0 + 0.6 * MM)]
    b = _lathe("mc_body", body, seg, "cast_aluminium", closed=True)
    _translate(b, (xm, 0.0, zm))
    parts = [b]
    yf0, yf1 = _ly(Y_MFLANGE), _ly(Y_MMOUTH)
    fl = MU.extrude_polygon(_nm("mc_fl"), np.array([(-0.040, -0.009), (-0.032, -0.014), (0.032, -0.014),
                                                    (0.040, -0.009), (0.040, 0.009), (0.032, 0.014),
                                                    (-0.032, 0.014), (-0.040, 0.009)]), yf0, yf1, chamfer=0.6 * MM,
                            holes=[np.stack([(S.MASTER_CYL_BORE / 2 + 0.6 * MM) * np.cos(np.linspace(0, TAU, 32, endpoint=False)),
                                             (S.MASTER_CYL_BORE / 2 + 0.6 * MM) * np.sin(np.linspace(0, TAU, 32, endpoint=False))], 1)],
                            collection=_COL, material="cast_aluminium")
    _translate(fl, (xm, 0.0, zm))
    parts.append(fl)
    # studs through the firewall (shared with the pedal box) + nuts
    for sgn in (+1, -1):
        sx = xm + sgn * 0.032
        parts.append(_cyl(f"mcs{sgn}", (sx, yf0 - 0.012, zm), (sx, yf1 + 0.008, zm), 3.9 * MM, seg=_seg(12),
                          material="steel_dark"))
        parts.append(_hex_prism(f"mcn{sgn}", (sx, yf1 + 0.0035, zm), (0, 1, 0), 7.0 * MM, 6.5 * MM))
    # outlet tube nut at the front
    parts.append(_hex_prism("mc_out", (xm, y1 + 0.0075, zm), (0, 1, 0), 6.5 * MM, 7.0 * MM, material="steel_machined"))
    # reservoir on top
    yr = (y0 + y1) / 2 + 0.004
    parts.append(_cyl("mc_neck", (xm, yr, zm + MASTER_R - 2 * MM), (xm, yr, zm + MASTER_R + 0.010), 6.0 * MM,
                      seg=_seg(16), material="cast_aluminium"))
    parts.append(_box("mc_res", (xm, yr, zm + MASTER_R + 0.010 + 0.021), (0.032, 0.044, 0.042), r=5.0 * MM,
                      material="plastic_black"))
    cap_c = np.array([xm, yr, zm + MASTER_R + 0.010 + 0.042])
    capp = [(0.0, 0.0), (0.0150, 0.0)] + [(0.0150 if i % 2 == 0 else 0.0142, 0.0012 * i) for i in range(1, 8)] + \
           [(0.0130, 0.0095), (0.0, 0.0095)]
    cap = _lathe("mc_cap", capp, _seg(48), "plastic_black", closed=False, smooth=30.0)
    _xform(cap, Matrix.Rotation(PI / 2, 4, "X"))   # lathe Y -> Z
    _translate(cap, cap_c)
    parts.append(cap)
    ob = _join("master_cylinder", parts)
    MU.smooth_by_angle(ob, 35.0, keep_sharp=True)
    return ob, np.array([xm, y1 + 0.011, zm])


def _build_master_piston():
    xm, zm = X_MASTER - ROOT_LOC[0], Z_MASTER - ROOT_LOC[2]
    yp = _ly(Y_MPISTON0)
    rb = S.MASTER_CYL_BORE / 2 - 0.1 * MM
    prof = [(0.0, yp + 0.0015), (0.0030, yp + 0.0015), (0.0040, yp), (rb, yp), (rb, yp + 0.004),
            (rb - 1.2 * MM, yp + 0.005), (rb - 1.2 * MM, yp + 0.013), (rb, yp + 0.014), (rb, yp + 0.018),
            (0.0, yp + 0.018)]
    pi_ = _lathe("master_piston", prof, _seg(32), "machined_aluminium", closed=False)
    _translate(pi_, (xm, 0.0, zm))
    return pi_


# ---------------- pedal, pushrod, pedal box ----------------------------------
def _pedal_frame():
    a0 = S.PEDAL_REST_ANGLE
    d0 = np.array([0.0, -math.sin(a0), -math.cos(a0)])
    npad = np.array([0.0, -math.cos(a0), math.sin(a0)])
    return a0, d0, npad


def _build_pedal():
    """Clutch pedal in pedal-local coords (origin at the pivot, rest pose)."""
    a0, d0, npad = _pedal_frame()
    Cv = clevis_point(0.0) - PIV
    pad_c = d0 * S.PEDAL_ARM
    up = -d0 * 0.030
    uc = Cv / np.linalg.norm(Cv)
    path = [uc * 0.0095, Cv * 0.6, Cv]
    for s_ in np.linspace(0.32, 0.96, 9):
        b = B_CLEVIS + (a0 - B_CLEVIS) * min(1.0, (s_ - 0.1667) / 0.5)
        path.append(np.array([0.0, -math.sin(b), -math.cos(b)]) * S.PEDAL_ARM * s_
                    + np.array([0, -0.004 * math.sin(PI * s_), 0]))
    path.append(pad_c - npad * 0.006)
    path = np.array(path)

    def sec(s_):
        depth = float(np.interp(s_, [0.0, 0.10, 0.3, 1.0], [0.019, 0.022, 0.018, 0.011]))
        return _rect_section(0.0085, depth, c=1.0 * MM)

    arm = _sweep_poly("pedal_arm", path, sec, material="paint_black", up=(1.0, 0.0, 0.0))
    ext = _sweep_poly("pedal_ext", [up, -d0 * 0.0095], _rect_section(0.0085, PEDAL_TOP_DEPTH, c=1.0 * MM),
                      material="paint_black", up=(1.0, 0.0, 0.0))
    parts = [arm, ext]
    parts.append(_cyl("pedal_bush", (-0.012, 0.0, 0.0), (0.012, 0.0, 0.0), 0.0105, seg=_seg(32),
                      material="paint_black", chamfer=1.0 * MM, r_in=0.0074))
    parts.append(_cyl("pedal_bush2", (-0.0135, 0.0, 0.0), (0.0135, 0.0, 0.0), 0.0076, seg=_seg(24),
                      material="plastic_black", r_in=0.0061))
    # clevis pin + clip
    parts.append(_cyl("pedal_cpin", (Cv[0] - 0.011, Cv[1], Cv[2]), (Cv[0] + 0.011, Cv[1], Cv[2]), 3.9 * MM,
                      seg=_seg(16), material="steel_machined", chamfer=0.5 * MM))
    # return-spring hook pin
    A = spring_hook_a(0.0)
    parts.append(_cyl("pedal_hpin", (A[0] - 0.0075, A[1], A[2]), (A[0] + 0.0075, A[1], A[2]), 2.5 * MM, seg=_seg(12),
                      material="steel_machined"))
    # pad: steel backing + ribbed rubber pad
    M = np.stack([np.array([1.0, 0, 0]), np.cross(npad, [1.0, 0, 0]), npad], 1)   # cols: x, up-ish, normal
    Mm = Matrix(((M[0, 0], M[0, 1], M[0, 2], 0), (M[1, 0], M[1, 1], M[1, 2], 0), (M[2, 0], M[2, 1], M[2, 2], 0),
                 (0, 0, 0, 1)))

    def padbox(key, c_local, size, r, material):
        ob = MU.rounded_box(_nm(key), size, radius=r, segments=2, center=tuple(c_local), collection=_COL,
                            material=material)
        _xform(ob, Matrix.Translation(Vector(tuple(pad_c))) @ Mm)
        return ob

    parts.append(padbox("pad_plate", (0, 0, -0.0045), (0.068, 0.054, 0.004), 1.0 * MM, "paint_black"))
    parts.append(padbox("pad_rub", (0, 0, 0.0015), (0.074, 0.058, 0.009), 3.0 * MM, "rubber"))
    for i in range(5):
        parts.append(padbox(f"pad_rib{i}", (0, -0.020 + 0.010 * i, 0.0068), (0.064, 0.0045, 0.0026), 1.0 * MM,
                            "rubber"))
    ob = _join("pedal", parts)
    MU.smooth_by_angle(ob, 40.0, keep_sharp=True)
    return ob


def _build_pushrod():
    """Clevis + rod along local +Y from the clevis pin (origin) to the tip (y = ROD_L)."""
    parts = []
    for sgn in (+1, -1):
        cheek = MU.rounded_box(_nm(f"pr_ch{sgn}"), (0.0028, 0.024, 0.014), radius=1.0 * MM, segments=2,
                               center=(sgn * 0.0069, 0.0035, 0.0), collection=_COL, material="steel_dark")
        _bool(cheek, [_cyl(f"pr_chh{sgn}", (sgn * 0.004, 0, 0), (sgn * 0.010, 0, 0), 4.05 * MM, seg=_seg(16))])
        parts.append(cheek)
    parts.append(MU.rounded_box(_nm("pr_blk"), (0.0166, 0.010, 0.014), radius=1.5 * MM, segments=2,
                                center=(0.0, 0.0205, 0.0), collection=_COL, material="steel_dark"))
    parts.append(_cyl("pr_rod", (0, 0.024, 0), (0, ROD_L - 0.0035, 0), 4.0 * MM, seg=_seg(16),
                      material="steel_machined"))
    parts.append(_hex_prism("pr_nut", (0, 0.029, 0), (0, 1, 0), 7.0 * MM, 5.0 * MM))
    parts.append(_sphere("pr_tip", (0, ROD_L - 4.5 * MM, 0), 4.5 * MM, seg=_seg(16), material="steel_machined"))
    ob = _join("pushrod", parts)
    MU.smooth_by_angle(ob, 40.0, keep_sharp=True)
    return ob


def _build_return_spring():
    """Tension spring along local +Y from 0 to its rest length (hooks included)."""
    A = spring_hook_a(0.0)
    L0 = float(np.linalg.norm(A - SPRING_B))
    hook = 0.008
    n_t = 14
    m = n_t * (12 if DETAIL == "high" else 8)
    t = np.linspace(0, 1, m + 1)
    rc = 3.2 * MM
    yy = hook + t * (L0 - 2 * hook)
    a = TAU * n_t * t
    pts = np.stack([rc * np.cos(a), yy, rc * np.sin(a)], 1)
    pts = np.concatenate([[(0.0, 0.0, 0.0), (0.0, hook * 0.6, 0.0)], pts, [(0.0, L0 - hook * 0.6, 0.0), (0.0, L0, 0.0)]])
    ob = MU.tube_along(_nm("return_spring"), pts, 0.75 * MM, segments=_seg(8), bend_radius=0.0, caps=True,
                       collection=_COL, material="steel_dark", smooth_angle=60.0)
    return ob, L0


def _build_pedal_box():
    """Pedal-box bracket (static, root-local): side plates, firewall plate, pivot
    bolt, stop bumper, spring anchor."""
    p = _L(PIV)
    xc = p[0]
    yfw = _ly(S.Y_FIREWALL) - 0.0005
    parts = []
    zm_r = Z_MASTER - PIV[2]
    a_h = np.linspace(0, TAU, 24, endpoint=False)
    for sgn in (+1, -1):
        x0 = xc + sgn * 0.0215
        poly = np.array([(yfw - 0.004, zm_r - 0.024), (yfw - 0.004, 0.046), (p[1] - 0.054, 0.046),
                         (p[1] - 0.054, -0.014), (p[1] - 0.030, -0.022), (p[1] + 0.010, -0.024),
                         (p[1] + 0.040, zm_r - 0.024)])
        poly[:, 1] += p[2]
        hole = np.stack([(p[1] + 0.052) + 0.016 * np.cos(a_h), (p[2] + 0.004) + 0.016 * np.sin(a_h)], 1)
        # extrude in X: build in (y, z) as (x, z) then swap x<->y
        pl = MU.extrude_polygon(_nm(f"pb_side{sgn}"), poly, -1.25 * MM, 1.25 * MM, holes=[hole], chamfer=0.4 * MM,
                                collection=_COL, material="paint_black")
        _xform(pl, Matrix(((0, 1, 0, 0), (1, 0, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1))))
        _translate(pl, (x0, 0.0, 0.0))
        parts.append(pl)
    # firewall mounting plate with pushrod hole (+ nuts of the master-cylinder studs)
    zm = Z_MASTER - ROOT_LOC[2]
    a = np.linspace(0, TAU, 24, endpoint=False)
    plate = MU.extrude_polygon(_nm("pb_plate"), np.array([(-0.044, -0.026), (0.044, -0.026), (0.044, 0.098),
                                                          (-0.044, 0.098)]) + [xc, zm],
                               yfw - 0.004, yfw, holes=[np.stack([xc + 0.012 * np.cos(a), zm + 0.012 * np.sin(a)], 1)]
                               + [np.stack([xc + sg * 0.032 + 0.0042 * np.cos(a), zm + 0.0042 * np.sin(a)], 1)
                                  for sg in (-1, 1)],
                               chamfer=0.4 * MM, collection=_COL, material="paint_black")
    parts.append(plate)
    for sgn in (+1, -1):
        parts.append(_hex_prism(f"pb_sn{sgn}", (xc + sgn * 0.032, yfw - 0.004 - 3.05 * MM, zm), (0, -1, 0), 7.0 * MM,
                                6.0 * MM, hole=4.2 * MM))
    # top plate joining the side plates
    parts.append(_box("pb_top", (xc, (yfw - 0.004 + p[1] - 0.054) / 2, p[2] + 0.0465),
                      (0.0455, (yfw - 0.004) - (p[1] - 0.054), 0.0025), r=0.8 * MM, material="paint_black"))
    # pivot bolt + nut
    parts.append(_cyl("pb_pivot", (xc - 0.030, p[1], p[2]), (xc + 0.030, p[1], p[2]), 6.0 * MM, seg=_seg(16),
                      material="steel_machined"))
    parts.append(_hex_prism("pb_pbh", (xc - 0.0255, p[1], p[2]), (-1, 0, 0), 8.0 * MM, 5.5 * MM))
    parts.append(_hex_prism("pb_pbn", (xc + 0.0255, p[1], p[2]), (1, 0, 0), 8.0 * MM, 6.0 * MM))
    # pedal stop: rubber bumper touching the front face of the arm's upper extension at rest
    a0, d0, _ = _pedal_frame()
    up = p - d0 * 0.030
    T = d0
    Bv = np.array([0.0, -math.cos(a0), math.sin(a0)])          # arm section depth direction (rearward)
    pc = up + T * 0.004 - Bv * (PEDAL_TOP_DEPTH / 2)              # contact point on the front face
    parts.append(_cyl("pb_stop", pc - Bv * GAP, pc - Bv * (GAP + 0.008), 5.0 * MM, seg=_seg(16),
                      material="rubber", chamfer=1.0 * MM))
    bc = pc - Bv * (GAP + 0.0105)
    parts.append(_box("pb_stopbr", (xc, bc[1], bc[2]), (0.0455, 0.005, 0.016), r=0.6 * MM, material="paint_black",
                      M=None))
    # return-spring anchor pin
    B = p + SPRING_B
    parts.append(_cyl("pb_anchor", (xc - 0.0215, B[1], B[2]), (xc + 0.0215, B[1], B[2]), 2.5 * MM, seg=_seg(12),
                      material="steel_machined"))
    # rubber dust boot around the pushrod on the cabin side of the bracket plate
    gr = [(0.0060, -0.0125), (0.0085, -0.0125)] + \
         [(0.0105 if i % 2 else 0.0090, -0.0125 + 0.002 * i) for i in range(1, 5)] + \
         [(0.0125, -0.0035), (0.0125, -0.0005), (0.0060, -0.0005)]
    g = _lathe("pb_boot", gr, _seg(24), "rubber")
    _translate(g, (xc, yfw - 0.004, zm))
    parts.append(g)
    ob = _join("pedal_box", parts)
    MU.smooth_by_angle(ob, 35.0, keep_sharp=True)
    return ob


# ---------------- hydraulic line ---------------------------------------------
LINE_JUNCTION = (-0.245, -0.428, 0.505)     # pipe -> hose junction, body bracket on the firewall


def _line_route(master_out, slave_in):
    """Polyline (car frame) master outlet -> slave inlet and the junction index."""
    mo = np.asarray(master_out) + ROOT_LOC
    si = np.asarray(slave_in) + ROOT_LOC
    J = np.array(LINE_JUNCTION)
    pipe = [mo, mo + [0, 0.012, 0], np.array([-0.495, -0.352, 0.616]), np.array([-0.432, -0.372, 0.566]),
            np.array([-0.338, -0.410, 0.526]), np.array([-0.278, -0.425, 0.508]), J]
    hose = [J, J + [0.022, 0.006, -0.018], np.array([-0.204, -0.393, 0.452]),
            np.array([si[0] - 0.004, si[1] - 0.008, si[2] + 0.046]), si + [0, 0, 0.020], si]
    return pipe, hose


def _fillet(points, r, n=8):
    return MU._fillet_path(points, r, n)


def _resample_path(P, n):
    P = np.asarray(P, float)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    t = np.linspace(0, s[-1], n)
    return np.stack([np.interp(t, s, P[:, k]) for k in range(3)], 1), s[-1]


def _build_line(master_out, slave_in, cut=False):
    """Hydraulic line split into N_LINE segments ordered master -> slave."""
    pipe, hose = _line_route(master_out, slave_in)
    pp = _fillet(pipe, 0.022, 8)
    hp = _fillet(hose, 0.018, 10)
    pp, Lp = _resample_path(pp, 240)
    hp, Lh = _resample_path(hp, 120)
    Ltot = Lp + Lh
    n_pipe = int(round(N_LINE * Lp / Ltot))
    n_pipe = min(max(n_pipe, 2), N_LINE - 2)
    n_hose = N_LINE - n_pipe
    segs = []
    fluid = []
    pieces = {"kept": [], "removed": []}

    def split(P, k):
        s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
        edges = np.linspace(0, s[-1], k + 1)
        out = []
        for i in range(k):
            sel = (s > edges[i]) & (s < edges[i + 1])
            a = np.array([np.interp(edges[i], s, P[:, j]) for j in range(3)])
            b = np.array([np.interp(edges[i + 1], s, P[:, j]) for j in range(3)])
            out.append(np.concatenate([[a], P[sel], [b]]))
        return out

    chunks = [(c, "pipe") for c in split(pp, n_pipe)] + [(c, "hose") for c in split(hp, n_hose)]
    for i, (c, kind) in enumerate(chunks):
        cl = c - ROOT_LOC
        r = PIPE_R if kind == "pipe" else HOSE_R
        ri = 0.8 * MM if kind == "pipe" else 2.0 * MM
        mat = "steel_machined" if kind == "pipe" else "rubber"
        ob = MU.tube_along(_nm(f"line_{i:02d}"), cl, r, segments=_seg(16 if kind == "pipe" else 20), bend_radius=0.0,
                           caps=True, inner_radius=ri, collection=_COL, material=mat, smooth_angle=60.0)
        segs.append(ob)
        if cut:
            fl = MU.tube_along(_nm(f"fluid_{i:02d}"), cl, ri - 0.05 * MM, segments=_seg(12), bend_radius=0.0,
                               caps=True, collection=_COL, material="brake_fluid", smooth_angle=60.0)
            fluid.append(fl)
            for part, sgn in (("kept", 1.0), ("removed", -1.0)):
                pieces[part].append(_half_pipe(f"line_{i:02d}__cut_{part}", cl, r, ri, sgn, mat))
    return segs, fluid, pieces, n_pipe, (pp - ROOT_LOC, hp - ROOT_LOC)


def _half_pipe(key, path, r_out, r_in, sgn, material, up=(0.0, 0.0, 1.0)):
    """Half of a hollow tube swept along `path`: sgn=+1 keeps the lower half (the
    upper half, toward `up`, is cut away), sgn=-1 keeps the upper half.  Cut faces
    get section_cut."""
    P = np.asarray(path, float)
    n = len(P)
    T = np.zeros_like(P)
    T[1:-1] = P[2:] - P[:-2]
    T[0] = P[1] - P[0]
    T[-1] = P[-1] - P[-2]
    T /= np.linalg.norm(T, axis=1)[:, None]
    upv = np.asarray(up, float)
    m = 10
    th = (np.linspace(PI / 2, 3 * PI / 2, m) if sgn > 0 else np.linspace(-PI / 2, PI / 2, m))
    sec = [(r_out * math.cos(t), r_out * math.sin(t)) for t in th] + \
          [(r_in * math.cos(t), r_in * math.sin(t)) for t in th[::-1]]
    sec = np.array(sec)
    k = len(sec)
    mb = MU.MeshBuilder()
    rings = []
    for i in range(n):
        Nv = upv - T[i] * np.dot(upv, T[i])
        if np.linalg.norm(Nv) < 1e-6:
            Nv = np.array([1.0, 0.0, 0.0])
        Nv /= np.linalg.norm(Nv)
        Bv = np.cross(T[i], Nv)
        rings.append(mb.verts(P[i] + sec[:, 0:1] * Nv + sec[:, 1:2] * Bv))
    for i in range(n - 1):
        a, b = rings[i], rings[i + 1]
        for j in range(k):
            j1 = (j + 1) % k
            cutf = (j == m - 1) or (j == k - 1)
            mb.face([a[j], a[j1], b[j1], b[j]], mat=1 if cutf else 0)
    mb.face(list(rings[0]))
    mb.face(list(rings[-1]))
    return mb.to_object(_nm(key), _COL, smooth_angle=50.0, materials=[material, "section_cut"])


def _build_line_fittings(seg_paths, n_pipe, master_out, slave_in):
    """Hose ferrules, junction tube nuts (static)."""
    pp, hp = seg_paths
    parts = []
    for P, end in ((hp, 0), (hp, -1)):
        p0 = P[end]
        p1 = P[end + (3 if end == 0 else -3)]
        d = (p1 - p0) / np.linalg.norm(p1 - p0)
        parts.append(_cyl(f"ferr{end}", p0 + d * 0.002, p0 + d * 0.024, HOSE_R + 1.6 * MM, seg=_seg(20),
                          material="steel_machined", chamfer=0.6 * MM))
    # junction: two tube nuts
    J = pp[-1]
    d = (pp[-1] - pp[-4]) / np.linalg.norm(pp[-1] - pp[-4])
    parts.append(_hex_prism("jn1", J - d * 0.004, d, 6.5 * MM, 7.0 * MM, material="steel_machined"))
    return _join("line_fittings", parts)


def _build_hose_bracket():
    """Body-mounted junction bracket: flange on the firewall + arm + eye around the union."""
    hb = _hose_bracket_geom()
    J, B = hb["J"], hb["base"]
    parts = []
    parts.append(_box("hbr_base", (B[0], B[1] + 0.0012, B[2]), (0.026, 0.0024, 0.034), r=0.8 * MM,
                      material="paint_black"))
    parts.append(_hex_prism("hbr_bolt", (B[0], B[1] + 0.0024 + 2.0 * MM, B[2] + 0.010), (0, 1, 0), 5.0 * MM, 4.0 * MM))
    p0 = np.array([B[0], B[1] + 0.002, B[2] - 0.008])
    p1 = np.array([J[0], J[1] - 0.012, J[2] - 0.009])
    sec = _rect_section(0.016, 0.0024, c=0.4 * MM)
    parts.append(_sweep_poly("hbr_arm", [p0, (p0 + p1) / 2 + [0, 0, -0.004], p1], sec, material="paint_black",
                             up=(1.0, 0.0, 0.0)))
    parts.append(_box("hbr_tab", (J[0], J[1] - 0.008, J[2] - 0.009), (0.020, 0.008, 0.0024), r=0.6 * MM,
                      material="paint_black"))
    a = np.linspace(0, TAU, 24, endpoint=False)
    eye = MU.extrude_polygon(_nm("hbr_eye"), np.stack([0.0105 * np.cos(a), 0.0105 * np.sin(a)], 1), -0.0012, 0.0012,
                             holes=[np.stack([0.0072 * np.cos(a), 0.0072 * np.sin(a)], 1)], collection=_COL,
                             material="paint_black")
    _xform(eye, Matrix.Rotation(PI / 2, 4, "X"))           # plate in XY (normal Z), y-thickness -> z
    _translate(eye, (J[0], J[1] - 0.004, J[2] - 0.009))
    parts.append(eye)
    ob = _join("hose_bracket", parts)
    MU.smooth_by_angle(ob, 35.0, keep_sharp=True)
    return ob


# ===========================================================================
# Assembly
# ===========================================================================

@dataclass
class ClutchAssembly(rig.Assembly):
    """Assembly whose explode offsets act on explode CARRIERS (meta['explode_carriers'])."""

    def bake_explode(self, track, presentation):
        ex = presentation.get("explode")
        if not self.explode:
            return
        n = len(track.frames)
        ex = np.zeros(n) if ex is None else np.broadcast_to(np.asarray(ex, float), (n,)).copy()
        for key, off in self.explode.items():
            car = self.meta["explode_carriers"].get(key)
            ob = self.parts.get(car)
            if ob is None:
                continue
            for ax in range(3):
                rig.bake_channel(ob, "location", ax, track.frames, off[ax] * ex)


def _per_frame(v, n, default):
    if v is None:
        return np.full(n, float(default))
    a = np.asarray(v, dtype=float)
    return np.broadcast_to(a, (n,)).copy() if a.ndim == 0 else a


def build(opts=None) -> rig.Assembly:
    global DETAIL, _COL
    t0 = time.time()
    opts = dict(opts or {})
    DETAIL = opts.get("detail", "high")
    cutaways = opts.get("cutaway", ["none"])
    if isinstance(cutaways, str):
        cutaways = [cutaways]
    cutaways = list(cutaways)
    _COL = rig.collection(opts.get("collection", "clutch"))
    if MAT is not None:
        for m in ("cast_iron", "cast_aluminium", "steel_machined", "steel_ground", "steel_dark", "friction",
                  "rubber", "paint_black", "copper", "brass", "plastic_black", "machined_aluminium"):
            MAT.get(m)
    root = rig.empty(_nm("root"), loc=tuple(ROOT_LOC), col=_COL, size=0.08)
    parts = {}

    def carrier(key):
        e = _empty(f"{key}_ex", root)
        parts[f"{key}_ex"] = e
        return e

    # ---------------- clutch pack ---------------------------------------
    disc_ex = carrier("disc")
    disc = _finish(_build_disc(), disc_ex)
    parts["disc"] = disc
    ds = _build_damper_springs()
    _finish(ds, disc, smooth=None)
    parts["damper_springs"] = ds
    plate_ex = carrier("plate")
    parts["pressure_plate"] = _finish(_build_pressure_plate(), plate_ex)
    parts["straps"] = _finish(_build_straps(), plate_ex)
    spring_ex = carrier("spring")
    parts["diaphragm_spring"] = _finish(_build_diaphragm(), spring_ex)
    parts["fulcrum"] = _finish(_build_fulcrum(), spring_ex, smooth=None)
    cover_ex = carrier("cover")
    parts["cover"] = _finish(_build_cover(), cover_ex)
    # ---------------- release system ------------------------------------
    rel_ex = carrier("release")
    rb, race = _build_release_bearing()
    parts["release_bearing"] = _finish(rb, rel_ex)
    parts["bearing_race"] = _finish(race, rb)
    parts["guide_tube"] = _finish(_build_guide_tube(), rel_ex)
    fork = _build_fork()
    pv = _L((X_PIVOT, Y_P, Z_FORK))
    fork.location = tuple(pv)
    parts["fork"] = _finish(fork, rel_ex)
    parts["ball_stud"] = _finish(_build_ball_stud(), rel_ex)
    # ---------------- bellhousing + cutaways -------------------------------
    bh, bh_bolts = _build_bellhousing()
    pieces = {}
    variants = []
    if "half" in cutaways:
        variants.append("half")
        kept = bh.copy()
        kept.data = bh.data.copy()
        kept.name = _nm("bellhousing__half_kept")
        _COL.objects.link(kept)
        rem = bh.copy()
        rem.data = bh.data.copy()
        rem.name = _nm("bellhousing__half_removed")
        _COL.objects.link(rem)
        _stepped_half_cut(kept, rem)
        bk = bh_bolts.copy()
        bk.data = bh_bolts.data.copy()
        bk.name = _nm("bellhousing_bolts__half_kept")
        _COL.objects.link(bk)
        br = bh_bolts.copy()
        br.data = bh_bolts.data.copy()
        br.name = _nm("bellhousing_bolts__half_removed")
        _COL.objects.link(br)
        _stepped_half_cut(bk, br)
        for k_, ob in (("bellhousing__half_kept", kept), ("bellhousing__half_removed", rem),
                       ("bellhousing_bolts__half_kept", bk), ("bellhousing_bolts__half_removed", br)):
            parts[k_] = _finish(ob, root)
        pieces["half"] = dict(kept=["bellhousing__half_kept", "bellhousing_bolts__half_kept"],
                              removed=["bellhousing__half_removed", "bellhousing_bolts__half_removed"],
                              replaces=["bellhousing", "bellhousing_bolts"])
    if "half_px" in cutaways:
        variants.append("half_px")
        names = {}
        for src, base in ((bh, "bellhousing"), (bh_bolts, "bellhousing_bolts")):
            for part, nrm in (("kept", (1.0, 0.0, 0.0)), ("removed", (-1.0, 0.0, 0.0))):
                o = src.copy()
                o.data = src.data.copy()
                o.name = _nm(f"{base}__half_px_{part}")
                _COL.objects.link(o)
                MU.cut_and_apply(o, plane=((0.0, 0.0, 0.0), nrm), space="LOCAL")
                parts[f"{base}__half_px_{part}"] = _finish(o, root)
                names.setdefault(part, []).append(f"{base}__half_px_{part}")
        pieces["half_px"] = dict(kept=names["kept"], removed=names["removed"],
                                 replaces=["bellhousing", "bellhousing_bolts"])
    if "none" in cutaways or not variants:
        parts["bellhousing"] = _finish(bh, root)
        parts["bellhousing_bolts"] = _finish(bh_bolts, root)
        variants.insert(0, "none")
    else:
        for o in (bh, bh_bolts):
            me = o.data
            bpy.data.objects.remove(o)
            bpy.data.meshes.remove(me)
    # ---------------- slave, line, master, pedal ----------------------------
    slave, slave_in = _build_slave()
    parts["slave_cylinder"] = _finish(slave, root)
    parts["slave_pushrod"] = _finish(_build_slave_pushrod(), root)
    master, master_out = _build_master()
    parts["master_cylinder"] = _finish(master, root)
    parts["master_piston"] = _finish(_build_master_piston(), root)
    segs, fluid, lpieces, n_pipe, seg_paths = _build_line(master_out, slave_in, cut=bool(opts.get("hydraulic_cut")))
    seg_names = []
    for i, ob in enumerate(segs):
        k = f"line_{i:02d}"
        parts[k] = _finish(ob, root)
        seg_names.append(k)
    fluid_names = []
    for i, ob in enumerate(fluid):
        k = f"fluid_{i:02d}"
        parts[k] = _finish(ob, root)
        fluid_names.append(k)
    for part in ("kept", "removed"):
        for ob in lpieces[part]:
            k = ob.name[len(PREFIX):]
            parts[k] = _finish(ob, root)
    if lpieces["kept"]:
        pieces["hydraulic_cut"] = dict(kept=[o.name[len(PREFIX):] for o in lpieces["kept"]] + fluid_names,
                                       removed=[o.name[len(PREFIX):] for o in lpieces["removed"]],
                                       replaces=list(seg_names))
    parts["line_fittings"] = _finish(_build_line_fittings(seg_paths, n_pipe, master_out, slave_in), root)
    parts["hose_bracket"] = _finish(_build_hose_bracket(), root)
    pedal = _build_pedal()
    pedal.location = tuple(_L(PIV))
    parts["pedal"] = _finish(pedal, root)
    pr = _build_pushrod()
    C0, ang0, _ = pushrod_pose(np.array([0.0]))
    pr.location = tuple(_L(C0[0]))
    pr.rotation_euler = (float(ang0[0]), 0.0, 0.0)
    parts["pushrod"] = _finish(pr, root)
    rs, L0 = _build_return_spring()
    B = _L(PIV) + SPRING_B
    rs.location = tuple(B)
    A0 = spring_hook_a(0.0)
    rs.rotation_euler = (math.atan2(A0[2] - SPRING_B[2], A0[1] - SPRING_B[1]), 0.0, 0.0)
    parts["return_spring"] = _finish(rs, root)
    parts["pedal_box"] = _finish(_build_pedal_box(), root)
    for ob in parts.values():
        if ob.type == "MESH":
            rig.set_presentation(ob, 1.0, 0.0)

    # ---------------- section (live booleans on rotating parts) -------------
    section_mods = []
    if opts.get("section_rotating"):
        section_mods = _add_section(parts, root, int(opts.get("section_side", -1)))

    # ---------------- anchors -----------------------------------------------
    yl = _ly
    anchors = {
        "disc": (parts["disc_ex"], (0.0, 0.0, 0.098)),
        "facing": (parts["disc_ex"], (-0.070, yl(Y_FF) - 0.001, 0.070)),
        "hub_splines": (parts["disc_ex"], (0.0, HUB_YC - HUB_LEN / 2, 0.0145)),
        "damper_springs": (parts["disc_ex"], (DAMPER_R * math.cos(90 * DEG), 0.006, DAMPER_R * math.sin(90 * DEG))),
        "pressure_plate": (parts["plate_ex"], (0.0, yl(Y_PF) - 0.008, PP_RO)),
        "diaphragm_spring": (parts["spring_ex"], (0.0, yl(D["y_m_rim"]) - 0.004, 0.100)),
        "fingers": (parts["spring_ex"], (0.045 * math.cos(80 * DEG), yl(float(D["y_mid"](0.045))),
                                         0.045 * math.sin(80 * DEG))),
        "cover": (parts["cover_ex"], (0.0, yl(Y_CBW) - CT, 0.112)),
        "release_bearing": (parts["release_ex"], (0.0, yl(Y_N0) - 0.012, 0.0405)),
        "fork": (parts["fork"], (-FORK_RATIO * FORK_CX * 0.5, 0.0, 0.012)),
        "guide_tube": (parts["release_ex"], (0.0, yl(Y_GT_FRONT) - 0.030, GT_RO)),
        "slave_cylinder": (root, tuple(_slave_axis() + [0.0, yl(Y_E0) + 0.032, SLV_R])),
        "master_cylinder": (root, tuple(_L((X_MASTER, (Y_MMOUTH + Y_MFRONT) / 2, Z_MASTER + MASTER_R)))),
        "pedal": (parts["pedal"], tuple(_pedal_frame()[1] * S.PEDAL_ARM)),
        "pushrod": (root, tuple(_L((X_MASTER, (Y_MFLANGE + float(C0[0][1])) / 2, Z_MASTER + 0.005)))),
        "hydraulic_line": (root, tuple(seg_paths[0][len(seg_paths[0]) // 2])),
        "bellhousing": (root, (0.0, yl(-0.40), bh_inner_r(-0.40) + BH_WALL)),
    }
    explode = {
        "disc": (0.0, -0.165, 0.0),
        "pressure_plate": (0.0, -0.240, 0.0),
        "diaphragm_spring": (0.0, -0.310, 0.0),
        "cover": (0.0, -0.385, 0.0),
        "release_bearing": (0.0, -0.470, 0.0),
    }
    groups = {
        "clutch_pack": ["disc", "damper_springs", "pressure_plate", "straps", "diaphragm_spring", "fulcrum", "cover"],
        "driving": ["cover", "diaphragm_spring", "fulcrum", "pressure_plate", "straps"],
        "driven": ["disc", "damper_springs"],
        "release": ["release_bearing", "bearing_race", "fork", "ball_stud", "guide_tube"],
        "hydraulics": ["master_cylinder", "master_piston"] + seg_names + ["line_fittings", "hose_bracket",
                                                                          "slave_cylinder", "slave_pushrod"],
        "pedal_box": ["pedal", "pushrod", "return_spring", "pedal_box"],
        "housing": [k for k in ("bellhousing", "bellhousing_bolts") if k in parts],
    }
    pos = dict(
        fork_pivot=(X_PIVOT, Y_P, Z_FORK), fork_pivot_spec=S.RELEASE_FORK_PIVOT,
        slave_axis=(X_SLAVE, Z_FORK), slave_pushrod_face_rest_y=Y_E0, slave_spec=S.SLAVE_CYL_POS,
        master_axis=(X_MASTER, Z_MASTER), master_flange_y=Y_MFLANGE, master_spec=S.MASTER_CYL_POS,
        pedal_pivot=tuple(PIV), clevis_rest=tuple(map(float, C0[0])), line_junction=LINE_JUNCTION,
        release_bearing_face_rest_y=Y_N0, release_bearing_spec_y=S.Y_RELEASE_BEARING)
    meta = dict(
        hydraulic_segments=seg_names, hydraulic_fluid=fluid_names, n_pipe_segments=n_pipe,
        power_path=["cover", "diaphragm_spring", "fulcrum", "pressure_plate", "straps", "disc", "damper_springs"],
        groups=groups, positions=pos,
        y=dict(flywheel_face=Y_FF, disc_centre=S.Y_DISC_CENTRE, plate_face=Y_PF, plate_back=Y_PB,
               ridge_top=Y_PB - RIDGE_H, cover_back_wall=Y_CBW, cover_flange=Y_FF, finger_tips=Y_N0 + GAP,
               bearing_face=Y_N0, fork_pads=Y_PAD, fork_plane=Y_P, guide_tube_front=Y_GT_FRONT,
               bellhousing_front=S.Y_BELLHOUSING_FRONT, bellhousing_rear_wall=Y_RW, gearbox_front=Y_RW1,
               hub=(S.Y_DISC_CENTRE + HUB_YC - HUB_LEN / 2, S.Y_DISC_CENTRE + HUB_YC + HUB_LEN / 2)),
        fork=dict(cx=FORK_CX, slave_arm=FORK_RATIO * FORK_CX, ratio=FORK_RATIO, axis="vertical (+Z) through the ball",
                  angle="asin(bearing / cx)", max_angle=float(fork_angle(S.RELEASE_BEARING_TRAVEL))),
        diaphragm=dict(fulcrum_r=R_F, rim_r=R_RIM, contact_r=float(D["r_c"]), nose_r=D["R_N"],
                       release_angle=float(D["alpha"]), F0=F0, finger_release=FINGER_REL,
                       lever_ratio_geom=(R_F - D["r_c"]) / (R_RIM - R_F), nose_gap_range=D["nose_gap_range"]),
        spline=dict(SPLINE, hub_length=HUB_LEN, hub_y=meta_hub_y()),
        disc_angle="theta_in + DISC_PHASE (0): same abs angle as the gearbox input shaft (spun with theta_in, "
                   "splines tooth 0 on +X), so the hub's internal splines (gaps on k*2pi/23) mesh with it",
        explode_carriers={"disc": "disc_ex", "pressure_plate": "plate_ex", "diaphragm_spring": "spring_ex",
                          "cover": "cover_ex", "release_bearing": "release_ex"},
        cutaway_pieces=pieces, variants=variants,
        shape_keys=dict(disc=["free"], straps=["lift"], diaphragm_spring=["bend", "release"]),
        section_modifiers=[(o.name, m) for o, m in section_mods],
        race_spin=opts.get("race_spin", "always"),
        rest=dict(), build_time=None,
    )
    # rest locations of moving objects (location keys are absolute)
    for k in ("disc", "pressure_plate", "release_bearing", "slave_pushrod", "master_piston", "pedal", "pushrod",
              "return_spring", "fork"):
        meta["rest"][k] = tuple(parts[k].location)
    asm = ClutchAssembly(name="clutch", root=root, parts=parts, anchors=anchors, explode=explode, meta=meta,
                         _driver=_drive)
    _default_visibility(asm, variants[0])
    meta["build_time"] = time.time() - t0
    return asm


def meta_hub_y():
    return (S.Y_DISC_CENTRE + HUB_YC - HUB_LEN / 2, S.Y_DISC_CENTRE + HUB_YC + HUB_LEN / 2)


ROTATING = ("disc", "damper_springs", "pressure_plate", "straps", "diaphragm_spring", "fulcrum", "cover",
            "bearing_race", "release_bearing")


def _add_section(parts, root, side=-1):
    """Live half-space boolean on rotating parts: removes x < X_CRANK (side=-1, same
    side as the bellhousing 'half' cut) or x > X_CRANK (side=+1)."""
    cut = MU.rounded_box(_nm("section_cutter"), (0.6, 0.6, 0.6), radius=0.0, center=(0.3 * side, 0.0, 0.0),
                         collection=_COL, material="section_cut", smooth_angle=None)
    cut.data.shade_flat()            # transferred cut faces must be flat-shaded
    cut.parent = root
    cut.hide_render = True
    cut.hide_viewport = True
    cut.display_type = "WIRE"
    parts["section_cutter"] = cut
    out = []
    for k in ROTATING:
        ob = parts[k]
        m = ob.modifiers.new("clu_section", "BOOLEAN")
        m.operation = "DIFFERENCE"
        m.object = cut
        m.solver = "MANIFOLD"
        m.material_mode = "TRANSFER"
        m.show_viewport = False
        m.show_render = False
        out.append((ob, m.name))
    return out


def variant_objects(asm, variant):
    """{'visible': [...], 'removed': [...], 'hidden': [...]} objects for a variant."""
    P = asm.parts
    pieces = asm.meta["cutaway_pieces"]
    vis, rem, hid = [], [], []
    for v, d in pieces.items():
        if v == "hydraulic_cut":
            continue
        if v == variant:
            vis += [P[k] for k in d["kept"] if k in P]
            rem += [P[k] for k in d["removed"] if k in P]
            hid += [P[k] for k in d["replaces"] if k in P]
        else:
            hid += [P[k] for k in d["kept"] + d["removed"] if k in P]
            if variant == "none" or variant not in pieces:
                vis += [P[k] for k in d["replaces"] if k in P]
    if not pieces:
        vis += [P[k] for k in ("bellhousing", "bellhousing_bolts") if k in P]
    return dict(visible=vis, removed=rem, hidden=[h for h in hid if h not in vis])


def _default_visibility(asm, variant):
    vo = variant_objects(asm, variant)
    for ob in vo["visible"]:
        ob.hide_render = ob.hide_viewport = False
    for ob in vo["removed"] + vo["hidden"]:
        ob.hide_render = ob.hide_viewport = True
    hc = asm.meta["cutaway_pieces"].get("hydraulic_cut")
    if hc:
        for k in hc["kept"] + hc["removed"]:
            asm.parts[k].hide_render = asm.parts[k].hide_viewport = True


def _bake_const(ob, path, idx, fr, vals):
    rig.bake_channel(ob, path, idx, fr, vals, interpolation="CONSTANT")


def _drive(asm, track, pres):
    P = asm.parts
    fr = np.asarray(track.frames)
    n = len(fr)
    pres = pres or {}
    rest = asm.meta["rest"]
    th_e = np.asarray(track.theta_e, float)
    th_d = np.asarray(track.theta_in, float) + DISC_PHASE
    lift = np.asarray(track.clutch_plate_lift, float)
    finger = np.asarray(track.clutch_finger, float)
    bearing = np.asarray(track.clutch_bearing, float)
    slave = np.asarray(track.clutch_slave, float)
    master = np.asarray(track.clutch_master, float)
    phi = np.asarray(track.clutch_pedal_angle, float)
    # ---- clutch pack -----------------------------------------------------------
    rig.bake_spin(P["disc"], fr, th_d)
    rig.bake_channel(P["disc"], "location", 1, fr, rest["disc"][1] - lift / 2)
    key = P["disc"].data.shape_keys
    rig.bake_channel(key, 'key_blocks["free"].value', -1, fr, np.minimum(lift, CUSHION_TRAVEL) / CUSHION_TRAVEL)
    rig.bake_spin(P["pressure_plate"], fr, th_e)
    rig.bake_channel(P["pressure_plate"], "location", 1, fr, rest["pressure_plate"][1] - lift)
    rig.bake_spin(P["straps"], fr, th_e)
    rig.bake_channel(P["straps"].data.shape_keys, 'key_blocks["lift"].value', -1, fr, lift / S.PRESSURE_PLATE_LIFT)
    rig.bake_spin(P["diaphragm_spring"], fr, th_e)
    sk = P["diaphragm_spring"].data.shape_keys
    rig.bake_channel(sk, 'key_blocks["bend"].value', -1, fr, np.clip(finger / F0, 0.0, 1.0))
    rig.bake_channel(sk, 'key_blocks["release"].value', -1, fr, lift / S.PRESSURE_PLATE_LIFT)
    rig.bake_spin(P["fulcrum"], fr, th_e)
    rig.bake_spin(P["cover"], fr, th_e)
    # ---- release system ----------------------------------------------------------
    rig.bake_channel(P["release_bearing"], "location", 1, fr, rest["release_bearing"][1] + bearing)
    if asm.meta.get("race_spin", "always") == "contact":
        d = np.diff(th_e, prepend=th_e[0])
        race = th_e[0] + np.cumsum(np.where(bearing > 0.3 * MM, d, 0.0))
    else:
        race = th_e
    rig.bake_spin(P["bearing_race"], fr, race)
    rig.bake_channel(P["fork"], "rotation_euler", 2, fr, fork_angle(bearing))
    # ---- hydraulics + pedal --------------------------------------------------------
    rig.bake_channel(P["slave_pushrod"], "location", 1, fr, rest["slave_pushrod"][1] - slave)
    rig.bake_channel(P["master_piston"], "location", 1, fr, rest["master_piston"][1] + master)
    rig.bake_channel(P["pedal"], "rotation_euler", 0, fr, phi)
    C, ang, _ = pushrod_pose(phi)
    Cl = C - ROOT_LOC
    for ax in range(3):
        rig.bake_channel(P["pushrod"], "location", ax, fr, Cl[:, ax])
    rig.bake_channel(P["pushrod"], "rotation_euler", 0, fr, ang)
    A = spring_hook_a(phi)
    dv = A - SPRING_B
    L = np.linalg.norm(dv, axis=1)
    L0 = float(np.linalg.norm(spring_hook_a(0.0) - SPRING_B))
    rig.bake_channel(P["return_spring"], "rotation_euler", 0, fr, np.arctan2(dv[:, 2], dv[:, 1]))
    rig.bake_channel(P["return_spring"], "scale", 1, fr, L / L0)
    # ---- presentation ----------------------------------------------------------------
    var = pres.get("variant")
    if var is None:
        var = asm.meta["variants"][0]
    if True:
        names = [var] * n if isinstance(var, str) else list(var)
        rem_op = _per_frame(pres.get("removed"), n, 0.0)
        objs = {}
        states = {v: variant_objects(asm, v) for v in set(names)}
        for v, vo in states.items():
            for k in ("visible", "removed", "hidden"):
                for ob in vo[k]:
                    objs[ob.name] = ob
        for nm_, ob in objs.items():
            hide = np.zeros(n)
            op = np.ones(n)
            for i, v in enumerate(names):
                vo = states[v]
                if ob in vo["visible"]:
                    hide[i] = 0.0
                elif ob in vo["removed"]:
                    op[i] = rem_op[i]
                    hide[i] = 1.0 if rem_op[i] < 0.02 else 0.0
                else:
                    hide[i] = 1.0
            _bake_const(ob, "hide_render", -1, fr, hide)
            _bake_const(ob, "hide_viewport", -1, fr, hide)
            if np.any((op < 1.0) & (hide < 0.5)):
                rig.bake_prop(ob, "cv_opacity", fr, op)
    hop = pres.get("housing")
    if hop is not None or pres.get("removed") is None:
        op = _per_frame(hop, n, 1.0)
        vnames = [var] * n if isinstance(var, str) else list(var)
        vcache = {v: variant_objects(asm, v)["visible"] for v in set(vnames)}
        for k in ("bellhousing", "bellhousing_bolts", "bellhousing__half_kept", "bellhousing_bolts__half_kept",
                  "bellhousing__half_px_kept", "bellhousing_bolts__half_px_kept"):
            ob = P.get(k)
            if ob is None:
                continue
            vis = np.array([ob in vcache[v] for v in vnames], dtype=float)
            if not np.any(vis):
                continue
            o = vis * op
            rig.bake_prop(ob, "cv_opacity", fr, o)
            _bake_const(ob, "hide_render", -1, fr, (o < 0.02).astype(float))
            _bake_const(ob, "hide_viewport", -1, fr, (o < 0.02).astype(float))
    sec = pres.get("section")
    if sec is None and asm.meta.get("section_modifiers"):
        sec = 0.0
    if sec is not None:
        on = _per_frame(sec, n, 0.0) > 0.5
        for ob_name, mname in asm.meta.get("section_modifiers", []):
            ob = bpy.data.objects[ob_name]
            m = ob.modifiers[mname]
            rig.bake_channel(ob, f'modifiers["{mname}"].show_render', -1, fr, on.astype(float), "CONSTANT")
            rig.bake_channel(ob, f'modifiers["{mname}"].show_viewport', -1, fr, on.astype(float), "CONSTANT")
            m.show_render = bool(on[0])
            m.show_viewport = bool(on[0])
    hc = asm.meta["cutaway_pieces"].get("hydraulic_cut")
    hcut = pres.get("hydraulic_cut", 0.0)
    if hc and hcut is not None:
        on = (_per_frame(hcut, n, 0.0) > 0.5).astype(float)
        for k in asm.meta["hydraulic_segments"]:
            _bake_const(P[k], "hide_render", -1, fr, on)
            _bake_const(P[k], "hide_viewport", -1, fr, on)
        for k in hc["kept"]:
            _bake_const(P[k], "hide_render", -1, fr, 1.0 - on)
            _bake_const(P[k], "hide_viewport", -1, fr, 1.0 - on)
    pulse = pres.get("line_pulse")
    if pulse is not None:
        c = _per_frame(pulse, n, -1.0)
        w = float(pres.get("line_pulse_width", 0.12))
        segs = asm.meta["hydraulic_segments"]
        m = len(segs)
        for i, k in enumerate(segs):
            u = (i + 0.5) / m
            g = np.where(c >= 0, np.exp(-((u - c) / w) ** 2), 0.0)
            for kk in (k, f"{k}__cut_kept", f"fluid_{i:02d}"):
                if kk in P:
                    rig.bake_prop(P[kk], "cv_glow", fr, g)
