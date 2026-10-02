"""Engine assembly: 2.0 L DOHC 16-valve inline-4 (port injected, longitudinal).

    from carviz.assemblies import engine
    E = engine.build({"cutaways": ["none", "long", "cyl1", "front"], "detail": "high"})
    E.drive(track, {"variant": "long"})

Frame / origin
--------------
``E.root`` (Empty ``eng_root``) sits on the crank axis at (X_CRANK, 0, Z_CRANK), no
rotation.  Every child uses ROOT-LOCAL coordinates: x, y exactly as the car frame,
z measured up from the crank axis.  Cylinder 1 is at the FRONT (+Y), intake side = -X,
exhaust side = +X, crank turns clockwise seen from the front (kin handles signs).

Parts (``E.parts`` keys; object names are ``eng_<key>``)
-------------------------------------------------------
moving (all motion baked from the Track through carviz.kin / the helpers below):
  crankshaft, crank_sprocket, damper, flywheel, ring_gear (child of flywheel),
  conrod1..4, piston1..4 (rings + gudgeon pin included, multi-material),
  cam_intake, cam_exhaust, cam_sprocket_intake, cam_sprocket_exhaust, timing_chain,
  valve_<c><i|e><0|1>     valve + collets + retainer + bucket tappet (e.g. valve_1i0 =
                          cylinder 1, intake, front valve; 1 = rear valve of the pair)
  spring_<c><i|e><0|1>    valve spring (shape key 'open' = compression by lift)
  gas_<c>_<kind>          kind in intake|compressed|burning|exhaust (full set)
  gas_<set>_<c>_<kind>    clipped gas sets for cutaways ('long', 'cyl1')
  spark<c>                emissive flash at the plug gap
static:
  block, main_caps (5 caps), main_bolts, main_shells, head, head_gasket, valve_guides,
  valve_seats, stem_seals, cam_caps, cam_cover, coils, spark_plug1..4, timing_cover,
  oil_pan, chain_guide (tight side), tensioner_arm, tensioner, intake_manifold
  (runners + plenum + throttle body), fuel_rail (rail + injectors), exhaust_manifold,
  oil_filter
cutaway pieces: ``<part>__<variant>_kept`` / ``<part>__<variant>_removed``.

Opts
----
``cutaways``  list of 'none' | 'long' | 'cyl1' | 'front'   (default ['none'])
   'long'  - x=0 section, -X half of every housing removed (moving parts whole).
   'cyl1'  - transverse section at cylinder 1 seen from the front.  Bottom end (block,
             sump, crank, piston 1, rod 1, gasket, plug 1) cut through the cylinder-1
             axis (y = Y_CYL1); head, cam cover, cams, valves, manifolds cut through the
             FRONT valve pair axes (y = Y_CYL1 + VALVE_Y) with a 24 mm slot down to
             the cylinder axis around the spark plug (stepped section, so valves,
             springs, buckets, lobes, ports, plug, piston, rod and crank throw all
             show in section).  Moving parts that straddle a section plane get
             kept/removed cut copies (planes y=const are invariant under every engine
             motion, so the build-time cuts stay valid while they move).
   'front' - cam cover + coils removed, timing cover cut back to a 14 mm rim, crank
             damper and the belt-driven water pump/idler/belt removed (as when a real
             timing cover is taken off) so chain, all three sprockets, guides, tensioner
             and cams with their phasing are visible from the front-top.
   'none'  - the whole engine.  If 'none' is not requested the whole housings are
             not kept (only the pieces).
``detail``    'high' (default) | 'low'
``gas``       build combustion-gas volumes (default True)
``collection`` collection name (default 'engine')

meta
----
``cutaway_pieces[variant]`` = {'kept': [...], 'removed': [...], 'replaces': [...]}:
   show kept, fade/slide removed away, hide replaces (the whole objects the pieces
   stand for).  Moving cut copies are baked like their originals.
``gas_sets`` {set: {cyl: {kind: name}}}, ``power_path``, ``groups``, ``y`` (axial
positions), ``valve_frames``, ``chain`` (layout numbers), ``cam_spacing``,
``lift_law``, ``dims`` (key dimensions), ``variants``.

presentation keys (all optional, per-frame arrays or scalars)
--------------------------------------------------------------
``explode``  0..1   flywheel moves back (-Y) by 100 mm (explode carrier = its location)
``gas``      0..1   global multiplier on gas opacity (default 1)
``gas_sets`` {set: 0..1}  per gas set multiplier (default: the set matching the single
             requested cutaway, else 'full' = 1, others 0)
``spark``    0..1   multiplier on the spark flash (default 1)
``variant``  name or per-frame list of names: bakes visibility of whole objects /
             variant pieces (CONSTANT keys)
``removed``  0..1   cv_opacity of the removed pieces while a cut variant is shown
             (default 0 = hidden)

Dimensions not in spec (typical 2.0 L DOHC values)
--------------------------------------------------
main journals 56 x 22 mm, crankpins 48 x 22.4 mm, webs 24.6 mm with counterweights
R 71 mm, rod I-beam 21.5 mm wide, piston compression height from spec, 3 rings,
22 mm gudgeon pin, pent-roof chamber (apex 13 mm, roof planes at 20 deg = valve
angle), valves on a 39 mm pair spacing at +-16.5 mm from the bore axis, 6 mm stems,
bucket tappets 37/36 mm, cam base circle 36 mm, springs 40 mm installed (7 coils,
3.6 mm wire), head gasket 1.2 mm MLS, single-row 3/8" roller chain (06B: roller
6.35 mm, inner width 5.72 mm).

Deviations from shared modules (see the report / requests):
* cam centre spacing: two 42T 3/8" sprockets have a 132.7 mm tip diameter, so the
  spec's 130 mm cam spacing would make them clash; this module uses
  CAM_SPACING = max(spec, tip diameter + 2 mm) = 134.7 mm.
* chain travel: a roller chain advances exactly z*p per sprocket turn, i.e. at
  z*p/(2pi) per radian, not at the pitch radius (kin.chain_travel drifts 0.037
  link/rev); CHAIN_TRAVEL(theta) below uses the exact law.
* valve lift: kin.valve_lift (sin^2 over 240 deg) cannot be produced by a flat
  bucket tappet (needs a negative nose radius: ~3.4 mm interference).  Valves follow
  ``valve_lift_ft`` - a classic three-arc flat-tappet cam (base circle 18 mm, nose
  radius 5 mm) with the SAME opening/closing angles and peak lift as spec/kin; the
  lobes are exactly that cam, so the bucket rides on the lobe at every angle.
"""
from __future__ import annotations

import math
import time

import bpy  # noqa: I001  (bpy must be imported before bmesh)
import bmesh
import numpy as np
from mathutils import Matrix, Vector

from .. import kin
from .. import meshutil as MU
from .. import rig
from .. import spec as S

try:  # shared modules (written in parallel); guarded
    from .. import gears as G
except Exception:  # pragma: no cover
    G = None
try:
    from .. import materials as MAT
except Exception:  # pragma: no cover
    MAT = None

PREFIX = "eng_"
MM = 1e-3
DEG = math.pi / 180.0
TAU = 2.0 * math.pi
PI = math.pi

# ===========================================================================
# Geometry constants (ROOT-LOCAL: z above the crank axis)
# ===========================================================================
R_THROW = S.CRANK_THROW
ROD_L = S.CONROD_LENGTH
ZD = S.DECK_HEIGHT                 # block deck
GASKET_T = 1.2 * MM
ZH = ZD + GASKET_T                 # head deck face
YF, YR = S.Y_BLOCK_FRONT, S.Y_BLOCK_REAR
Y_CYL = list(S.Y_CYL)
PITCH = S.BORE_PITCH
Y_MAIN = [Y_CYL[0] + PITCH / 2 - i * PITCH for i in range(S.N_CYL + 1)]
BORE_R = S.BORE / 2

MAIN_R, MAIN_W = 28.0 * MM, 22.0 * MM
PIN_R, PIN_W = 24.0 * MM, 22.4 * MM            # crankpin; PIN_W = gap between web faces
WEB_IN, WEB_OUT = PIN_W / 2, PITCH / 2 - MAIN_W / 2 - 0.2 * MM   # web faces from cyl centre
CW_R = 71.0 * MM                               # counterweight radius
ROD_W = 21.5 * MM
GPIN_R = 11.0 * MM                             # gudgeon pin
COMP_H = S.COMPRESSION_HEIGHT
SKIRT_BELOW_PIN = 24.0 * MM
CROWN_DISH = 1.9 * MM                          # depth of the shallow crown dish
BORE_BOT = 87.0 * MM                           # bottom of the bores (above crank axis)

# valvetrain
VALVE_ANGLE = 20.0 * DEG
SA, CA = math.sin(VALVE_ANGLE), math.cos(VALVE_ANGLE)
TAN_A = math.tan(VALVE_ANGLE)
APEX_H = 13.0 * MM                 # pent-roof apex above the head deck face
VALVE_X = 16.5 * MM                # valve face centre offset from the bore axis (x)
VALVE_Y = 19.5 * MM                # valve pair half spacing (y)
VALVE_HEAD_R = {"intake": S.INTAKE_VALVE_HEAD_D / 2, "exhaust": S.EXHAUST_VALVE_HEAD_D / 2}
VALVE_LIFT = {"intake": S.VALVE_LIFT_INTAKE, "exhaust": S.VALVE_LIFT_EXHAUST}
STEM_R = 3.0 * MM
CAM_BASE_R = 18.0 * MM
CAM_NOSE_R = 5.0 * MM
CAM_LASH = 0.05 * MM               # running clearance cam <-> bucket (never touches)
BUCKET_R = {"intake": 18.5 * MM, "exhaust": 18.0 * MM}
BUCKET_H, BUCKET_CROWN = 26.0 * MM, 4.5 * MM
SPRING_LEN, SPRING_WIRE_R, SPRING_COIL_R, SPRING_TURNS = 40.0 * MM, 1.8 * MM, 11.4 * MM, 7
CHAIN_ROLLER_D = 6.35 * MM
CHAIN_P = S.CHAIN_PITCH
R_TIP_CAM = 0.5 * CHAIN_P * (0.6 + 1.0 / math.tan(PI / S.CAM_SPROCKET_TEETH))


def roof_z(x):
    """Height of the pent-roof chamber roof at lateral offset x (root-local)."""
    return ZH + np.maximum(0.0, APEX_H - np.abs(x) * TAN_A)


Z_ROOF_V = float(roof_z(VALVE_X))                    # valve face centre height
Z_APEX = ZH + APEX_H
CAM_JOURNAL_R, CAM_JOURNAL_W = 13.5 * MM, 16.0 * MM
CAM_LOBE_W = 14.0 * MM
CAM_SHAFT_R = 12.0 * MM
CHAIN_PUSH = 6.0 * MM              # tensioner shoe push into the slack span (nominal)
R_SHOE, R_GUIDE, E_GUIDE = 1.4, 1.4, 4.0 * MM   # shoe/guide face radii (roller line), guide push

# timing drive
Y_CHAIN = YF + 12.0 * MM
COVER_IN, COVER_T = 0.028, 0.005           # timing cover inner depth from YF, wall
Y_COVER_FRONT = YF + COVER_IN + COVER_T     # 0.163
Y_DAMPER = (Y_COVER_FRONT + 0.004, Y_COVER_FRONT + 0.034)

# head outline
HEAD_W_DECK, HEAD_W_TOP = 0.086, 0.115
X_PORT_FACE = HEAD_W_TOP
COVER_H = 0.036                             # cam cover height above Z_CAM
X_SLOT = 12.0 * MM                          # cyl1 slot half width
Y_P0 = Y_CYL[0]                             # cyl1 cut plane through the cylinder axis
Y_P1 = Y_CYL[0] + VALVE_Y                   # cyl1 cut plane through the front valve pair

GAS_ALPHA = {"intake": 0.42, "compressed": 0.50, "burning": 0.80, "exhaust": 0.50}
GAS_KINDS = ("intake", "compressed", "burning", "exhaust")


# ===========================================================================
# Pure kinematics (no bpy needed)
# ===========================================================================

def chain_travel(theta):
    """Exact roller-chain travel (m) for crank angle theta: z*p per crank turn."""
    return np.asarray(theta) * S.CRANK_SPROCKET_TEETH * CHAIN_P / TAU


def _r_eff(z):
    """Radius on which roller centres move so that p of travel = one tooth."""
    return z * CHAIN_P / TAU


# --- flat-tappet three-arc cam ---------------------------------------------

def _arccam(Rb, L, rn, ad):
    """Three-arc cam (base Rb, flank arcs, nose rn) with lift L, half duration ad
    (cam rad).  Returns (rf, F, N, beta_t): flank radius, flank centre (+ side),
    nose centre, flank/nose transition angle."""
    dn = Rb + L - rn

    def f(rf):
        lhs = -(rf - Rb) * math.cos(ad)
        rhs = (dn * dn - (rf - rn) ** 2 + (rf - Rb) ** 2) / (2 * dn)
        return lhs - rhs
    lo, hi = Rb + 1e-7, 10.0
    flo = f(lo)
    for _ in range(200):
        m = 0.5 * (lo + hi)
        fm = f(m)
        if flo * fm <= 0:
            hi = m
        else:
            lo, flo = m, fm
    rf = 0.5 * (lo + hi)
    F = -(rf - Rb) * np.array([math.cos(ad), math.sin(ad)])
    N = np.array([dn, 0.0])
    u = (N - F) / np.linalg.norm(N - F)
    return rf, F, N, abs(math.atan2(u[1], u[0]))


_CAMGEO = {}


def cam_geometry(kind):
    g = _CAMGEO.get(kind)
    if g is None:
        if kind == "intake":
            span = S.IVC_DEG - S.IVO_DEG
        else:
            span = S.EVC_DEG - S.EVO_DEG
        ad = math.radians(span / 2.0) / 2.0          # half duration in CAM radians
        rf, F, N, bt = _arccam(CAM_BASE_R, VALVE_LIFT[kind], CAM_NOSE_R, ad)
        g = dict(Rb=CAM_BASE_R, L=VALVE_LIFT[kind], rn=CAM_NOSE_R, ad=ad, rf=rf, F=F, N=N, bt=bt)
        _CAMGEO[kind] = g
    return g


def cam_support(beta, kind):
    """Support function h(beta) of the lobe (distance cam centre -> bucket face when
    the follower direction makes angle beta with the nose)."""
    g = cam_geometry(kind)
    b = np.abs(np.asarray(beta, dtype=float))
    b = np.where(b > PI, TAU - b, b)
    h_nose = g["N"][0] * np.cos(b) + g["rn"]
    h_fl = g["F"][0] * np.cos(b) + g["F"][1] * np.sin(b) + g["rf"]
    return np.where(b <= g["bt"], h_nose, np.where(b <= g["ad"], h_fl, g["Rb"]))


def valve_lift_ft(theta, cyl, kind):
    """Valve lift (m) of the flat-tappet cam for crank angle theta (same events and
    peak as kin.valve_lift; fuller curve because a bucket tappet needs it)."""
    phi = kin.cycle_angle_deg(theta, cyl)
    pk = kin.valve_peak_cycle_deg(kind)
    d = (np.asarray(phi) - pk + 360.0) % 720.0 - 360.0       # crank deg from peak
    beta = np.radians(d) / 2.0                                # cam rad from nose
    return np.maximum(cam_support(beta, kind) - CAM_BASE_R, 0.0)


def cam_outline(kind, n=200, lash=CAM_LASH):
    """Closed lobe outline (n,2) in the lobe frame (nose on +X), CCW."""
    g = cam_geometry(kind)
    Rb, rn, rf, F, N, bt, ad = (g["Rb"] - lash, g["rn"] - lash, g["rf"] - lash, g["F"], g["N"],
                                g["bt"], g["ad"])
    Fm = np.array([F[0], -F[1]])
    n_nose, n_fl = max(8, n // 6), max(8, n // 7)
    n_base = max(16, n - n_nose - 2 * n_fl)
    pts = []
    for a in np.linspace(-bt, bt, n_nose, endpoint=False):           # nose
        pts.append(N + rn * np.array([math.cos(a), math.sin(a)]))
    for a in np.linspace(bt, ad, n_fl, endpoint=False):               # + flank
        pts.append(F + rf * np.array([math.cos(a), math.sin(a)]))
    for a in np.linspace(ad, TAU - ad, n_base, endpoint=False):       # base circle
        pts.append(Rb * np.array([math.cos(a), math.sin(a)]))
    for a in np.linspace(-ad, -bt, n_fl, endpoint=False):             # - flank
        pts.append(Fm + rf * np.array([math.cos(a), math.sin(a)]))
    return np.array(pts)


# --- valve frames ------------------------------------------------------------

def valve_frame(cyl, kind, i):
    """(F, a, cam_centre): valve face centre, unit axis (toward the tip/cam), cam
    centre, all root-local.  i = 0 front valve, 1 rear valve."""
    sx = -1.0 if kind == "intake" else 1.0
    yv = Y_CYL[cyl - 1] + (VALVE_Y if i == 0 else -VALVE_Y)
    F = np.array([sx * VALVE_X, yv, Z_ROOF_V])
    a = np.array([sx * SA, 0.0, CA])
    return F, a, F + T_CAM * a


def valve_dir_psi(kind):
    """Profile angle (psi, in XZ from +X toward +Z) from the cam centre to its valve."""
    sx = -1.0 if kind == "intake" else 1.0
    return math.atan2(-CA, -sx * SA)


# --- timing chain layout -----------------------------------------------------

def _tangent(c1, r1, c2, r2):
    """Belt tangent from circle 1 to circle 2 (signed radii: + = wrapped CCW,
    - = backside, wrapped CW).  Returns (p1, p2)."""
    d = np.asarray(c2, float) - np.asarray(c1, float)
    L = float(np.hypot(*d))
    beta = math.atan2(d[1], d[0])
    g = math.acos(max(-1.0, min(1.0, (r2 - r1) / L)))
    best = None
    for sg in (1.0, -1.0):
        n = np.array([math.cos(beta + sg * g), math.sin(beta + sg * g)])
        t = np.array([n[1], -n[0]])
        p1 = np.asarray(c1) - r1 * n
        p2 = np.asarray(c2) - r2 * n
        if np.dot(p2 - p1, t) > 0:
            best = (p1, p2)
    return best


def _loop(circles):
    """Closed belt path around signed circles [(centre, r)] in order.
    Returns (segments, total_length); segment = ('arc', i, c, r, a0, sweep, s0, len)
    or ('line', p0, p1, s0, len); s measured from the start of circle 0's arc."""
    n = len(circles)
    tans = [_tangent(circles[i][0], circles[i][1], circles[(i + 1) % n][0], circles[(i + 1) % n][1])
            for i in range(n)]
    if any(t is None for t in tans):
        return [], float("inf")
    segs = []
    s = 0.0
    for i in range(n):
        c, r = circles[i]
        arr = tans[i - 1][1]          # arrival on circle i
        dep = tans[i][0]              # departure from circle i
        a0 = math.atan2(arr[1] - c[1], arr[0] - c[0])
        a1 = math.atan2(dep[1] - c[1], dep[0] - c[0])
        if r > 0:
            sweep = (a1 - a0) % TAU
        else:
            sweep = -((a0 - a1) % TAU)
        ln = abs(r) * abs(sweep)
        segs.append(("arc", i, np.asarray(c, float), r, a0, sweep, s, ln))
        s += ln
        p0, p1 = tans[i]
        ln = float(np.hypot(*(p1 - p0)))
        segs.append(("line", p0, p1, s, ln))
        s += ln
    return segs, s


def _loop_point(segs, s):
    for sg in segs:
        s0, ln = sg[-2], sg[-1]
        if s0 - 1e-12 <= s <= s0 + ln + 1e-12:
            u = (s - s0)
            if sg[0] == "arc":
                _, _, c, r, a0, sweep, _, _ = sg
                a = a0 + np.sign(sweep) * u / abs(r)
                return c + abs(r) * np.array([math.cos(a), math.sin(a)])
            p0, p1 = sg[1], sg[2]
            return p0 + (p1 - p0) * (u / max(ln, 1e-12))
    return None


def _chain_circles(x_cam, z_cam, push):
    R0, R1 = _r_eff(S.CRANK_SPROCKET_TEETH), _r_eff(S.CAM_SPROCKET_TEETH)
    C0, Ce, Ci = np.array([0.0, 0.0]), np.array([x_cam, z_cam]), np.array([-x_cam, z_cam])

    def backside(cA, rA, cB, rB, e, Rb):
        p1, p2 = _tangent(cA, rA, cB, rB)
        M = 0.5 * (p1 + p2)
        t = (p2 - p1) / np.linalg.norm(p2 - p1)
        n_out = np.array([t[1], -t[0]])            # right of travel = outside the loop
        return M + (Rb - e) * n_out

    cg = backside(Ci, R1, C0, R0, E_GUIDE, R_GUIDE)
    cs = backside(C0, R0, Ce, R1, push, R_SHOE)
    return [(C0, R0), (cs, -R_SHOE), (Ce, R1), (Ci, R1), (cg, -R_GUIDE)]


def _cam_height(x_cam):
    return Z_ROOF_V + (x_cam - VALVE_X) / SA * CA


def _solve_cam_spacing():
    """Smallest cam spacing >= sprocket tip clearance for which the chain loop (with
    the nominal tensioner push) is an exact even number of pitches."""
    x_min = max(S.CAM_CENTRE_SPACING / 2, R_TIP_CAM + 1.0 * MM)

    def length(x):
        return _loop(_chain_circles(x, _cam_height(x), CHAIN_PUSH))[1]
    n = int(math.ceil(length(x_min) / CHAIN_P - 1e-9))
    n += n % 2
    lo, hi = x_min, x_min + 0.02
    for _ in range(100):
        m = 0.5 * (lo + hi)
        if length(m) < n * CHAIN_P:
            lo = m
        else:
            hi = m
    return 0.5 * (lo + hi), n


X_CAM, CHAIN_LINKS = _solve_cam_spacing()
CAM_SPACING = 2 * X_CAM
T_CAM = (X_CAM - VALVE_X) / SA                       # valve face -> cam centre along the axis
Z_CAM = _cam_height(X_CAM)                           # cam centre height
BUCKET_TOP = T_CAM - CAM_BASE_R                      # along the valve axis (valve frame y)
VLEN = BUCKET_TOP - BUCKET_CROWN - 0.02 * MM          # valve length (face -> tip)
Y_RETAINER = VLEN - 8.0 * MM                         # spring top seat (retainer underside)
Y_SPRING_SEAT = Y_RETAINER - SPRING_LEN              # spring bottom seat (head floor)
Y_GUIDE_TOP = Y_SPRING_SEAT + 5.0 * MM
Y_GUIDE_BOT = 30.0 * MM


_CHAIN = None


def chain_layout():
    """Timing chain path (XZ, root-local) wrapping crank sprocket (21T), slack-side
    tensioner shoe, exhaust cam sprocket, intake cam sprocket, tight-side guide.
    Travel direction CCW in (x, z) (= with the sprockets' psi).  The shoe push is
    fine-tuned so the path length is exactly CHAIN_LINKS pitches (even)."""
    global _CHAIN
    if _CHAIN is not None:
        return _CHAIN
    N = CHAIN_LINKS
    lo, hi = 0.2 * MM, 15 * MM
    for _ in range(80):
        m = 0.5 * (lo + hi)
        if _loop(_chain_circles(X_CAM, Z_CAM, m))[1] < N * CHAIN_P:
            lo = m
        else:
            hi = m
    e = 0.5 * (lo + hi)
    circles = _chain_circles(X_CAM, Z_CAM, e)
    segs, L = _loop(circles)
    names = {0: ("crank", S.CRANK_SPROCKET_TEETH), 2: ("exhaust", S.CAM_SPROCKET_TEETH),
             3: ("intake", S.CAM_SPROCKET_TEETH)}
    phases = {}
    for sg in segs:
        if sg[0] == "arc" and sg[1] in names:
            nm, z = names[sg[1]]
            tau = TAU / z
            psi_a, s_a, r = sg[4], sg[6], sg[3]
            phases[nm] = float((psi_a - s_a / r - tau / 2) % tau)
    _CHAIN = dict(segments=segs, length=L, n_links=N, push=e, circles=circles, phases=phases,
                  R_crank=circles[0][1], R_cam=circles[2][1], R_shoe=R_SHOE, R_guide=R_GUIDE,
                  guide_centre=circles[4][0], shoe_centre=circles[1][0])
    return _CHAIN


def chain_points(spacing=0.4 * MM):
    """Dense closed polyline (k,2) of the chain path starting at s=0, in travel order."""
    ch = chain_layout()
    L = ch["length"]
    n = int(math.ceil(L / spacing))
    return np.array([_loop_point(ch["segments"], L * i / n) for i in range(n)])


# ===========================================================================
# Small helpers (bpy)
# ===========================================================================

_COL = None
_DETAIL = "high"


def _cr(r, n):
    """Circumscribed polygon radius: a bore with n segments whose edges never come
    inside the true radius r (shafts are inscribed, so no false overlaps)."""
    return r / math.cos(PI / n)


def _seg(n):
    """Segment count scaled by detail (multiple of 4, >= 8)."""
    k = 1.0 if _DETAIL == "high" else 0.5
    return max(8, int(round(n * k / 4.0)) * 4)


def _mat(name):
    return MU.get_material(name)


def _nm(key):
    return PREFIX + key


def _new_from_builder(mb, key, mats, smooth=35.0):
    obj = mb.to_object(_nm(key), _COL, smooth_angle=smooth, materials=mats)
    return obj


def _set_mat(obj, name):
    MU.assign_material(obj, name)
    return obj


def _xform(obj, M):
    obj.data.transform(M)
    obj.data.update()
    return obj


def _translate(obj, v):
    return _xform(obj, Matrix.Translation(Vector(v)))


def _bool(target, operands, op="DIFFERENCE"):
    """Apply a Manifold boolean (collection operand) to target; operands are
    deleted.  Operand materials are transferred (cutter surfaces keep theirs)."""
    ops = [o for o in operands if o is not None]
    if not ops:
        return target
    tmp = bpy.data.collections.new("eng_bool_tmp")
    bpy.context.scene.collection.children.link(tmp)
    for o in ops:
        for c in list(o.users_collection):
            c.objects.unlink(o)
        tmp.objects.link(o)
    if not target.users_collection:
        _COL.objects.link(target)
    mod = target.modifiers.new("eng_bool", "BOOLEAN")
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


def _join(key, objs, smooth=None):
    """MU.join + keep the exact name."""
    objs = [o for o in objs if o is not None]
    ob = MU.join(_nm(key) + "_tmpjoin", objs, collection=_COL, smooth_angle=smooth)
    ob.name = _nm(key)
    ob.data.name = _nm(key)
    return ob


def _union(key, objs):
    base = objs[0]
    _bool(base, objs[1:], "UNION")
    base.name = _nm(key)
    base.data.name = _nm(key)
    return base


def _shapely():
    from shapely.geometry import Point, Polygon
    from shapely.ops import unary_union
    return Point, Polygon, unary_union


def _poly_xy(geom, tol=0.0):
    """Exterior coords (k,2) of a shapely polygon (largest if multi)."""
    if hasattr(geom, "geoms"):
        geom = max(geom.geoms, key=lambda g: g.area)
    if tol:
        geom = geom.simplify(tol)
    return np.asarray(geom.exterior.coords)[:-1]


def _circle(c, r, n=48):
    a = np.linspace(0, TAU, n, endpoint=False)
    return np.stack([c[0] + r * np.cos(a), c[1] + r * np.sin(a)], axis=1)


def _plate_xy(key, poly_xy, z0, z1, holes=None, chamfer=0.0, material=None):
    """Prism with outline in the XY plane, extruded along Z from z0 to z1."""
    flip = lambda P: np.stack([np.asarray(P)[:, 0], -np.asarray(P)[:, 1]], axis=1)  # noqa: E731
    ob = MU.extrude_polygon(_nm(key), flip(poly_xy), z0, z1, holes=[flip(h) for h in (holes or [])],
                            chamfer=chamfer, collection=_COL, material=material)
    _xform(ob, Matrix.Rotation(PI / 2, 4, "X"))   # (x, y, z) -> (x, -z, y)
    return ob


def _cyl_along(key, p0, p1, r, seg=24, material=None, chamfer=0.0):
    """Closed cylinder from point p0 to p1 (root-local)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    L = float(np.linalg.norm(p1 - p0))
    ob = MU.cylinder(_nm(key), r, 0.0, L, segments=seg, chamfer=chamfer, collection=_COL, material=material)
    d = Vector((p1 - p0) / L)
    q = Vector((0, 1, 0)).rotation_difference(d)
    _xform(ob, Matrix.Translation(Vector(p0)) @ q.to_matrix().to_4x4())
    return ob


def _box(key, x0, x1, y0, y1, z0, z1, r=0.0, material=None):
    return MU.rounded_box(_nm(key), (x1 - x0, y1 - y0, z1 - z0), radius=r, segments=2,
                          center=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), collection=_COL,
                          material=material)


def _valve_matrix(cyl, kind, i):
    """Matrix mapping valve-frame coords (y along the axis from the face) to root-local."""
    F, a, _ = valve_frame(cyl, kind, i)
    phi = (-1.0 if kind == "intake" else 1.0) * VALVE_ANGLE
    R = Matrix.Rotation(phi, 4, "Y") @ Matrix.Rotation(PI / 2, 4, "X")
    return Matrix.Translation(Vector(F)) @ R


def _apply_vf(obj, cyl, kind, i):
    return _xform(obj, _valve_matrix(cyl, kind, i))


# ===========================================================================
# Moving parts
# ===========================================================================

def _web_outline(psi_p):
    Point, Polygon, uu = _shapely()
    P = R_THROW * np.array([math.cos(psi_p), math.sin(psi_p)])
    c0 = psi_p + PI
    arc = [(CW_R * math.cos(a), CW_R * math.sin(a)) for a in np.linspace(c0 - 1.08, c0 + 1.08, 40)]
    sector = Polygon([(0.0, 0.0)] + arc)
    g = uu([sector, Point(*P).buffer(0.030, 32), Point(0, 0).buffer(0.036, 32)]).convex_hull
    g = g.buffer(-0.007, 16).buffer(0.007, 16)
    return _poly_xy(g, tol=0.0002)


def _pin_psi(cyl):
    return PI / 2 + math.radians(S.CRANKPIN_PHASE_DEG[cyl])


def _build_crankshaft():
    seg = _seg(48)
    parts = []
    for i, ym in enumerate(Y_MAIN):
        y0 = ym - MAIN_W / 2 - 0.002
        y1 = ym + MAIN_W / 2 + 0.002
        parts.append(MU.cylinder(_nm(f"crk_main{i}"), MAIN_R, y0, y1, segments=seg, collection=_COL,
                                 material="steel_ground"))
    for c in range(1, S.N_CYL + 1):
        yc = Y_CYL[c - 1]
        psi = _pin_psi(c)
        P = R_THROW * np.array([math.cos(psi), math.sin(psi)])
        pin = MU.cylinder(_nm(f"crk_pin{c}"), PIN_R, yc - WEB_IN - 0.002, yc + WEB_IN + 0.002, segments=seg,
                          collection=_COL, material="steel_ground")
        _translate(pin, (P[0], 0.0, P[1]))
        parts.append(pin)
        out = _web_outline(psi)
        for sgn in (1, -1):
            a, b = sorted((yc + sgn * WEB_IN, yc + sgn * WEB_OUT))
            parts.append(MU.extrude_polygon(_nm(f"crk_web{c}{sgn}"), out, a, b, chamfer=0.0022,
                                            collection=_COL, material="steel_forged"))
    yfm = Y_MAIN[0] + MAIN_W / 2
    snout = [(0.0, yfm - 0.004), (0.0225, yfm - 0.004), (0.0225, YF + 0.002), (0.0195, YF + 0.0032),
             (0.0195, Y_CHAIN + 0.0085), (0.018, Y_CHAIN + 0.010), (0.018, Y_DAMPER[1] - 0.0005),
             (0.0165, Y_DAMPER[1] + 0.001), (0.0, Y_DAMPER[1] + 0.001)]
    parts.append(MU.lathe(_nm("crk_snout"), snout, seg, collection=_COL, material="steel_ground"))
    yrm = Y_MAIN[-1] - MAIN_W / 2
    yfl = S.Y_CRANK_FLANGE
    rear = [(0.0, yrm + 0.003), (MAIN_R, yrm + 0.003), (MAIN_R, YR + 0.0100), (0.042, YR + 0.0100),
            (0.042, YR + 0.0005), (0.0425, YR),
            (0.0425, yfl + 0.0006), (0.0419, yfl), (0.0105, yfl), (0.0105, yfl + 0.012), (0.0, yfl + 0.012)]
    parts.append(MU.lathe(_nm("crk_rear"), rear, _seg(64), collection=_COL, material="steel_machined"))
    crank = _union("crankshaft", parts)
    # flywheel bolt holes in the flange + oil drillings in the journals
    cut = []
    for k in range(8):
        a = TAU * (k + 0.5) / 8
        p = (0.031 * math.cos(a), 0.0, 0.031 * math.sin(a))
        cut.append(_cyl_along(f"crk_fh{k}", (p[0], yfl - 0.001, p[2]), (p[0], yfl + 0.016, p[2]), 0.0042, seg=12))
    for c in range(1, S.N_CYL + 1):
        psi = _pin_psi(c)
        P = R_THROW * np.array([math.cos(psi), 0.0, math.sin(psi)])
        P[1] = Y_CYL[c - 1]
        d = np.array([math.cos(psi + 0.6), 0.0, math.sin(psi + 0.6)])
        cut.append(_cyl_along(f"crk_oil{c}", P - 0.004 * d, P + 0.03 * d, 0.0025, seg=10))
    _bool(crank, cut)
    return crank


def _rod_outline():
    Point, Polygon, uu = _shapely()
    big = Point(0, 0).buffer(0.0345, 48)
    boss = Polygon([(-0.0405, -0.017), (0.0405, -0.017), (0.0405, 0.0125), (-0.0405, 0.0125)])
    beam = Polygon([(-0.0165, 0.02), (0.0165, 0.02), (0.0092, ROD_L - 0.016), (-0.0092, ROD_L - 0.016)])
    small = Point(0, ROD_L).buffer(0.0148, 40)
    g = uu([big, boss, beam, small]).buffer(0.007, 24).buffer(-0.007, 24)
    return _poly_xy(g, tol=0.0001)


def _build_rod(c):
    key = f"conrod{c}"
    seg = _seg(48)
    _, Polygon, _ = _shapely()
    rod = MU.extrude_polygon(_nm(key), _rod_outline(), -ROD_W / 2, ROD_W / 2, chamfer=0.0008,
                             collection=_COL, material="steel_forged")
    cut = [MU.cylinder(_nm(key + "_be"), PIN_R + 0.002, -0.02, 0.02, segments=seg, collection=_COL,
                       material="steel_machined"),
           MU.cylinder(_nm(key + "_se"), GPIN_R + 0.0012, -0.02, 0.02, segments=_seg(32), collection=_COL,
                       material="steel_machined")]
    _translate(cut[1], (0, 0, ROD_L))
    pk = Polygon([(-0.0108, 0.046), (0.0108, 0.046), (0.0052, ROD_L - 0.027), (-0.0052, ROD_L - 0.027)])
    pk = pk.buffer(-0.002, 8).buffer(0.002, 8)
    pko = _poly_xy(pk)
    for k, (a, b) in enumerate(((0.003, ROD_W / 2 + 0.002), (-ROD_W / 2 - 0.002, -0.003))):
        cut.append(MU.extrude_polygon(_nm(f"{key}_pk{k}"), pko, a, b, collection=_COL, material="steel_forged"))
    cut.append(_box(key + "_split", -0.05, 0.05, -0.02, 0.02, -0.00025, 0.00025))
    _bool(rod, cut)
    extra = []
    # bearing shells (big end) and small-end bush
    extra.append(MU.cylinder(_nm(key + "_sh"), PIN_R + 0.00195, -0.0098, 0.0098, segments=seg,
                             inner_radius=_cr(PIN_R + 0.00005, seg), collection=_COL, material="copper"))
    bush = MU.cylinder(_nm(key + "_bu"), GPIN_R + 0.00118, -ROD_W / 2 + 0.0004, ROD_W / 2 - 0.0004,
                       segments=_seg(40), inner_radius=_cr(GPIN_R + 0.00005, _seg(40)), collection=_COL, material="brass")
    _translate(bush, (0, 0, ROD_L))
    extra.append(bush)
    for sx in (-1, 1):
        hexo = _circle((0, 0), 0.0066, 6)
        h = _plate_xy(f"{key}_bolt{sx}", hexo, -0.0245, -0.0172, chamfer=0.0005, material="steel_dark")
        _translate(h, (sx * 0.031, 0, 0))
        extra.append(h)
    return _join(key, [rod] + extra)


def _relief_cutters(c, key):
    """Valve reliefs in piston c's crown (piston frame at TDC)."""
    out = []
    z_pin_tdc = R_THROW + ROD_L
    crown_tdc = z_pin_tdc + COMP_H
    d_out = 1.8 * MM
    for kind in ("intake", "exhaust"):
        r_rel = VALVE_HEAD_R[kind] + 1.5 * MM
        for i in (0, 1):
            F, a, _ = valve_frame(c, kind, i)
            delta = ((F[2] - crown_tdc) + d_out - r_rel * SA) / CA
            p0 = F - delta * a
            p1 = F + 0.05 * a
            off = np.array([0.0, Y_CYL[c - 1], z_pin_tdc])
            out.append(_cyl_along(f"{key}_rel{kind[0]}{i}", p0 - off, p1 - off, r_rel, seg=_seg(40),
                                  material="machined_aluminium"))
    return out


def _build_piston(c):
    key = f"piston{c}"
    hc = COMP_H
    R_SK, R_LD, R_G = BORE_R - 0.05 * MM, BORE_R - 0.32 * MM, 0.0390
    sb = SKIRT_BELOW_PIN
    prof = [(0.0, hc), (R_LD - 0.0006, hc), (R_LD, hc - 0.0006), (R_LD, hc - 0.005), (R_G, hc - 0.005),
            (R_G, hc - 0.0062), (R_LD, hc - 0.0062), (R_LD, hc - 0.0100), (R_G, hc - 0.0100),
            (R_G, hc - 0.0115), (R_LD, hc - 0.0115), (R_LD, hc - 0.0145), (0.0385, hc - 0.0145),
            (0.0385, hc - 0.0175), (R_SK, hc - 0.0185), (R_SK, -sb + 0.0008), (R_SK - 0.0008, -sb),
            (0.0405, -sb), (0.0405, -0.006), (0.0362, hc - 0.019), (0.0345, hc - 0.0075), (0.0, hc - 0.0075)]
    body = MU.lathe(_nm(key), prof, _seg(72), axis="Z", collection=_COL, material="machined_aluminium")
    add = []
    for sgn in (1, -1):
        a, b = sorted((sgn * 0.0112, sgn * 0.0372))
        add.append(MU.cylinder(_nm(f"{key}_boss{sgn}"), 0.0138, a, b, segments=_seg(32), collection=_COL,
                               material="machined_aluminium"))
        add.append(_box(f"{key}_strut{sgn}", -0.0085, 0.0085, a, b, 0.0, hc - 0.006, material="machined_aluminium"))
    _bool(body, add, "UNION")
    cut = [MU.cylinder(_nm(key + "_pb"), GPIN_R + 0.00002, -0.046, 0.046, segments=_seg(32), collection=_COL,
                       material="steel_machined")]
    for sgn in (1, -1):
        a, b = sorted((sgn * 0.0318, sgn * 0.05))
        cut.append(_box(f"{key}_slip{sgn}", -0.05, 0.05, a, b, -0.04, hc - 0.0195))
    cut += _relief_cutters(c, key)
    # shallow spherical dish in the crown (sets the compression ratio, see test_engine)
    d, ra = CROWN_DISH, 0.032
    Rs = (ra * ra + d * d) / (2 * d)
    prof = [(0.0, hc - d)] + [(r, hc - d + Rs - math.sqrt(Rs * Rs - r * r)) for r in np.linspace(0.004, ra, 9)]
    prof += [(ra, hc + 0.002), (0.0, hc + 0.002)]
    cut.append(MU.lathe(_nm(key + "_dish"), prof, _seg(72), axis="Z", collection=_COL, material="machined_aluminium"))
    _bool(body, cut)
    rings = []
    for k, (y0, y1, ri) in enumerate(((hc - 0.0061, hc - 0.0051, R_G + 0.0002),
                                      (hc - 0.0114, hc - 0.0101, R_G + 0.0002),
                                      (hc - 0.0174, hc - 0.0146, 0.0387))):
        r = MU.lathe(_nm(f"{key}_ring{k}"), [(ri, y0), (R_SK, y0), (R_SK, y1), (ri, y1)], _seg(72), axis="Z",
                     closed=True, caps=False, collection=_COL, material="steel_dark")
        rings.append(r)
    pin = MU.cylinder(_nm(key + "_pin"), GPIN_R, -0.0355, 0.0355, segments=_seg(32), inner_radius=0.0066,
                      chamfer=0.0005, collection=_COL, material="steel_ground")
    return _join(key, [body] + rings + [pin])


def _valve_profile(kind):
    rh = VALVE_HEAD_R[kind]
    y0, y1, r1 = 0.0029, 0.024, rh - 0.0017
    tul = [(STEM_R + (r1 - STEM_R) * ((y1 - y) / (y1 - y0)) ** 3.0, y) for y in np.linspace(y0, y1, 11)]
    prof = [(0.0, 0.0), (rh - 0.0012, 0.0), (rh, 0.0005), (rh, 0.0012)] + tul
    prof += [(STEM_R, VLEN - 0.0068), (0.0026, VLEN - 0.0062), (0.0026, VLEN - 0.0050), (STEM_R, VLEN - 0.0044),
             (STEM_R, VLEN - 0.0005), (0.0025, VLEN), (0.0, VLEN)]
    return prof


def _build_valve(c, kind, i):
    """Valve + collets + retainer + bucket, in the valve frame (y along the axis)."""
    key = f"valve_{c}{kind[0]}{i}"
    rb = BUCKET_R[kind]
    v = MU.lathe(_nm(key), _valve_profile(kind), _seg(40), collection=_COL, material="steel_ground")
    col_ = MU.lathe(_nm(key + "_co"), [(STEM_R + 0.00003, VLEN - 0.0105), (0.0047, VLEN - 0.0105),
                                       (0.0059, VLEN - 0.0016), (STEM_R + 0.00003, VLEN - 0.0016)], _seg(24),
                    closed=True, caps=False, collection=_COL, material="steel_dark")
    ret = MU.lathe(_nm(key + "_rt"), [(0.00475, Y_RETAINER), (0.0138, Y_RETAINER), (0.0138, Y_RETAINER + 0.0016),
                                      (0.0092, Y_RETAINER + 0.0030), (0.0085, VLEN - 0.0020),
                                      (0.00595, VLEN - 0.0020), (0.00475, VLEN - 0.0105)], _seg(40),
                   closed=True, caps=False, collection=_COL, material="steel_machined")
    bt = BUCKET_TOP
    bk = MU.lathe(_nm(key + "_bk"), [(0.0, bt), (rb - 0.0006, bt), (rb, bt - 0.0006), (rb, bt - BUCKET_H + 0.0005),
                                     (rb - 0.0005, bt - BUCKET_H), (rb - 0.0016, bt - BUCKET_H),
                                     (rb - 0.0016, bt - BUCKET_CROWN - 0.0012), (rb - 0.0028, bt - BUCKET_CROWN),
                                     (0.0, bt - BUCKET_CROWN)], _seg(56), collection=_COL, material="steel_machined")
    return _join(key, [v, col_, ret, bk])


def _spring_centre_z(phi):
    """Height of the spring wire centre at helix angle phi (closed end turns)."""
    d = 2 * SPRING_WIRE_R
    z0, z1 = SPRING_WIRE_R + 0.00003, SPRING_LEN - SPRING_WIRE_R - 0.00005
    t_end = TAU
    tot = SPRING_TURNS * TAU
    act = z1 - z0 - 2 * d
    phi = np.asarray(phi, dtype=float)
    return np.where(phi < t_end, z0 + d * phi / t_end,
                    np.where(phi > tot - t_end, z1 - d * (tot - phi) / t_end,
                             z0 + d + act * (phi - t_end) / (tot - 2 * t_end)))


def _build_spring(c, kind, i):
    key = f"spring_{c}{kind[0]}{i}"
    tot = SPRING_TURNS * TAU
    n = int(SPRING_TURNS * (_seg(28)))
    phi = np.linspace(0.0, tot, n)
    R = SPRING_COIL_R
    pts = np.stack([R * np.cos(phi), _spring_centre_z(phi), R * np.sin(phi)], axis=1)
    ob = MU.tube_along(_nm(key), pts, SPRING_WIRE_R, segments=_seg(10), bend_radius=0.0, caps=True,
                       collection=_COL, material="steel_dark", smooth_angle=60.0)
    return ob


def _add_spring_key(ob, kind):
    """Shape key 'open' = the spring compressed by the max lift (end turns rigid)."""
    me = ob.data
    if me.shape_keys is not None:
        return
    co = np.zeros(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    ang = np.arctan2(co[:, 2], co[:, 0]) % TAU
    tot = SPRING_TURNS * TAU
    cand = ang[:, None] + TAU * np.arange(-1, SPRING_TURNS + 1)[None, :]
    cand = np.clip(cand, 0.0, tot)
    zc = _spring_centre_z(cand)
    k = np.argmin(np.abs(zc - co[:, 1:2]), axis=1)
    phi = cand[np.arange(len(co)), k]
    g = np.clip((phi - TAU) / (tot - 2 * TAU), 0.0, 1.0)
    g = np.where(phi >= tot - TAU, 1.0, g)
    ob.shape_key_add(name="Basis", from_mix=False)
    kb = ob.shape_key_add(name="open", from_mix=False)
    new = co.copy()
    new[:, 1] -= VALVE_LIFT[kind] * g
    kb.data.foreach_set("co", new.ravel())
    kb.slider_min, kb.slider_max = 0.0, 1.0


def _build_cam(kind):
    key = "cam_" + kind
    sx = -1.0 if kind == "intake" else 1.0
    seg = _seg(40)
    y_rear = Y_MAIN[-1] - CAM_JOURNAL_W / 2 - 0.004
    prof = [(0.0, y_rear), (CAM_SHAFT_R - 0.001, y_rear), (CAM_SHAFT_R, y_rear + 0.001)]
    for ym in Y_MAIN[::-1]:
        a, b = ym - CAM_JOURNAL_W / 2, ym + CAM_JOURNAL_W / 2
        prof += [(CAM_SHAFT_R, a - 0.0015), (CAM_JOURNAL_R, a - 0.0003), (CAM_JOURNAL_R, b + 0.0003),
                 (CAM_SHAFT_R, b + 0.0015)]
    yj = Y_MAIN[0] + CAM_JOURNAL_W / 2
    prof = [p for p in prof if p[1] < yj + 0.0016]
    prof += [(0.0165, yj + 0.003), (0.0165, YF + 0.003), (0.022, YF + 0.0035), (0.022, Y_CHAIN - 0.0028),
             (0.0125, Y_CHAIN - 0.0026), (0.0125, Y_CHAIN + 0.0024), (0.0, Y_CHAIN + 0.0024)]
    shaft = MU.lathe(_nm(key), prof, seg, collection=_COL, material="cast_iron")
    lobes = []
    outline = cam_outline(kind, n=_seg(200))
    psi_dir = valve_dir_psi(kind)
    for c in range(1, S.N_CYL + 1):
        psi = kin.cam_lobe_psi(c, kind, psi_dir)
        cs, sn = math.cos(psi), math.sin(psi)
        rot = np.stack([outline[:, 0] * cs - outline[:, 1] * sn, outline[:, 0] * sn + outline[:, 1] * cs], axis=1)
        for i in (0, 1):
            yv = Y_CYL[c - 1] + (VALVE_Y if i == 0 else -VALVE_Y)
            lobes.append(MU.extrude_polygon(_nm(f"{key}_lobe{c}{i}"), rot, yv - CAM_LOBE_W / 2, yv + CAM_LOBE_W / 2,
                                            chamfer=0.0005, collection=_COL, material="steel_ground"))
    cam = _union(key, [shaft] + lobes)
    # the journals are ground too: recolour faces at journal radius
    _recolour_radial(cam, CAM_JOURNAL_R, "steel_ground", axis_xz=(0.0, 0.0), tol=0.0002)
    return cam


def _recolour_radial(ob, r, mat, axis_xz=(0.0, 0.0), tol=0.0002):
    me = ob.data
    idx = MU.assign_material(ob, mat, faces=[])
    n = len(me.polygons)
    cen = np.zeros(n * 3)
    me.polygons.foreach_get("center", cen)
    cen = cen.reshape(-1, 3)
    rr = np.hypot(cen[:, 0] - axis_xz[0], cen[:, 2] - axis_xz[1])
    sel = np.abs(rr - r * math.cos(PI / 40)) < max(tol, r * (1 - math.cos(PI / 20)))
    mi = np.zeros(n, dtype=np.int32)
    me.polygons.foreach_get("material_index", mi)
    mi[sel] = idx
    me.polygons.foreach_set("material_index", mi)
    return ob


def _sprocket_obj(key, z, bore=0.0, hub=None, holes=0, bolt=False):
    p = CHAIN_P
    r_pitch = p / (2 * math.sin(PI / z))
    rd = CHAIN_ROLLER_D + 2 * (r_pitch - _r_eff(z)) + 0.10 * MM
    if G is not None and hasattr(G, "sprocket"):
        ob = G.sprocket(_nm(key), z, pitch=p, roller_d=rd, width=0.0052, bore=bore, hub=hub,
                        detail="high" if _DETAIL == "high" else "low", material="steel_machined", collection=_COL)
    else:  # fallback: plain toothed disc
        a = np.linspace(0, TAU, z * 8, endpoint=False)
        rr = r_pitch + 0.0025 * np.cos(z * a)
        ob = MU.extrude_polygon(_nm(key), np.stack([rr * np.cos(a), rr * np.sin(a)], 1), -0.0026, 0.0026,
                                collection=_COL, material="steel_machined")
    if holes:
        cut = []
        rh = 0.5 * (hub[0] / 2 + r_pitch - 0.012) if hub else r_pitch * 0.6
        for k in range(holes):
            a = TAU * (k + 0.5) / holes
            cut.append(_cyl_along(f"{key}_h{k}", (rh * math.cos(a), -0.01, rh * math.sin(a)),
                                  (rh * math.cos(a), 0.01, rh * math.sin(a)), 0.0105, seg=_seg(28)))
        _bool(ob, cut)
    if bolt:
        parts = [ob]
        w = MU.cylinder(_nm(key + "_w"), 0.0135, 0.0026, 0.0046, segments=_seg(32), chamfer=0.0003,
                        collection=_COL, material="steel_machined")
        hx = MU.extrude_polygon(_nm(key + "_b"), _circle((0, 0), 0.0092, 6), 0.0046, 0.0118, chamfer=0.0006,
                                collection=_COL, material="steel_dark")
        parts += [w, hx]
        ob = _join(key, parts)
    _translate(ob, (0.0, Y_CHAIN, 0.0))
    return ob


def _build_flywheel():
    yf0, yf1 = S.Y_FLYWHEEL_FRONT, S.Y_FLYWHEEL_FACE
    rg_w = 0.012
    r_ring_in = 0.137
    prof = [(0.0105, yf0 - 0.0015), (0.0425, yf0 - 0.0015), (0.0445, yf0 - 0.003), (0.128, yf0 - 0.003),
            (0.132, yf0 - 0.0005), (r_ring_in - 0.0002, yf0 - 0.0005), (r_ring_in - 0.0002, yf0 - rg_w - 0.0004),
            (0.1395, yf0 - rg_w - 0.0008), (0.1395, yf1 + 0.001), (0.1385, yf1), (0.1205, yf1),
            (0.1195, yf1 + 0.0012), (0.1175, yf1 + 0.0012), (0.1165, yf1), (0.0745, yf1), (0.072, yf1 + 0.008),
            (0.040, yf1 + 0.008), (0.040, yf1 + 0.0065), (0.0105, yf1 + 0.0065)]
    fw = MU.lathe(_nm("flywheel"), prof, _seg(128), closed=True, caps=False, collection=_COL, material="cast_iron")
    # machined friction face (faces lying on the rear face plane between r 0.0745 and 0.1165)
    me = fw.data
    idx = MU.assign_material(fw, "steel_machined", faces=[])
    n = len(me.polygons)
    cen = np.zeros(n * 3)
    me.polygons.foreach_get("center", cen)
    cen = cen.reshape(-1, 3)
    rr = np.hypot(cen[:, 0], cen[:, 2])
    sel = (np.abs(cen[:, 1] - yf1) < 1e-5) & (rr > 0.074) & (rr < 0.1405)
    sel |= (np.abs(cen[:, 1] - yf0 + 0.0015) < 1e-5)
    mi = np.zeros(n, dtype=np.int32)
    me.polygons.foreach_get("material_index", mi)
    mi[sel] = idx
    me.polygons.foreach_set("material_index", mi)
    cut = []
    for k in range(6):      # clutch cover bolt holes + 2 dowels
        a = TAU * (k + 0.25) / 6
        cut.append(_cyl_along(f"fw_ch{k}", (0.1295 * math.cos(a), yf1 - 0.001, 0.1295 * math.sin(a)),
                              (0.1295 * math.cos(a), yf1 + 0.014, 0.1295 * math.sin(a)), 0.0038, seg=12))
    for k in range(8):
        a = TAU * (k + 0.5) / 8
        cut.append(_cyl_along(f"fw_bh{k}", (0.031 * math.cos(a), yf0 + 0.001, 0.031 * math.sin(a)),
                              (0.031 * math.cos(a), yf1, 0.031 * math.sin(a)), 0.0047, seg=12))
    _bool(fw, cut)
    bolts = []
    for k in range(8):
        a = TAU * (k + 0.5) / 8
        hb = MU.extrude_polygon(_nm(f"fw_bolt{k}"), _circle((0.031 * math.cos(a), 0.031 * math.sin(a)), 0.0072, 6),
                                yf1 + 0.0065 - 0.0062, yf1 + 0.0065, chamfer=0.0005, collection=_COL,
                                material="steel_dark")
        bolts.append(hb)
    fw = _join("flywheel", [fw] + bolts)
    ring = None
    if G is not None and hasattr(G, "ring_gear"):
        ring = G.ring_gear(_nm("ring_gear"), z=S.FLYWHEEL_RING_TEETH, width=rg_w, inner_d=2 * r_ring_in,
                           outer_d=S.FLYWHEEL_DIAMETER, chamfer_side="-Y",
                           detail="high" if _DETAIL == "high" else "low", material="steel_machined",
                           collection=_COL)
        _translate(ring, (0.0, yf0 - rg_w / 2 - 0.0004, 0.0))
    return fw, ring


def _build_damper():
    y0, y1 = Y_DAMPER
    prof = [(0.0182, y0), (0.030, y0), (0.031, y0 + 0.004), (0.050, y0 + 0.006), (0.0505, y0 + 0.001)]
    ribs = 6
    ry0, ry1 = y0 + 0.004, y1 - 0.004
    prof += [(0.068, y0 + 0.001), (0.0705, y0), (0.0755, y0), (0.0765, ry0)]
    pitch = (ry1 - ry0) / ribs
    for k in range(ribs):
        prof += [(0.0745, ry0 + (k + 0.5) * pitch), (0.0765, ry0 + (k + 1) * pitch)]
    prof += [(0.0755, y1), (0.0705, y1), (0.068, y1 - 0.001), (0.0505, y1 - 0.001), (0.050, y1 - 0.004),
             (0.024, y1 - 0.004), (0.0182, y1 - 0.004)]
    hub = MU.lathe(_nm("damper"), prof, _seg(96), closed=True, caps=False, collection=_COL,
                   material="cast_iron")
    rub = MU.lathe(_nm("damper_rub"), [(0.0506, y0 + 0.0015), (0.0538, y0 + 0.0015), (0.0538, y1 - 0.0025),
                                       (0.0506, y1 - 0.0025)], _seg(96), closed=True, caps=False,
                   collection=_COL, material="rubber")
    bolt_w = MU.cylinder(_nm("damper_w"), 0.024, y1 - 0.004, y1 - 0.0015, segments=_seg(40), chamfer=0.0004,
                         collection=_COL, material="steel_machined")
    bolt = MU.extrude_polygon(_nm("damper_b"), _circle((0, 0), 0.0125, 6), y1 - 0.0015, y1 + 0.008,
                              chamfer=0.0008, collection=_COL, material="steel_dark")
    return _join("damper", [hub, rub, bolt_w, bolt])


def _prism_xy_mb(mb, outline, z0, z1, mat=0):
    """Closed prism (outline in XY, extruded along Z) into MeshBuilder mb."""
    o = np.asarray(outline)
    a = mb.verts(np.stack([o[:, 0], o[:, 1], np.full(len(o), z0)], 1))
    b = mb.verts(np.stack([o[:, 0], o[:, 1], np.full(len(o), z1)], 1))
    mb.bridge(a, b, mat=mat)
    mb.face(list(a)[::-1], mat=mat)
    mb.face(list(b), mat=mat)


def _chain_template():
    """One chain pitch-pair (inner link at x 0..p, outer link at p..2p) in chain-mesh
    coords: X along the path, Y in the chain plane (toward the loop inside), Z across."""
    Point, Polygon, uu = _shapely()
    p = CHAIN_P
    mb = MU.MeshBuilder()
    nr = _seg(16)
    for x in (0.0, p):
        _prism_xy_mb(mb, _circle((x, 0.0), CHAIN_ROLLER_D / 2, nr), -0.00284, 0.00284)
        _prism_xy_mb(mb, _circle((x, 0.0), 0.00159, max(8, nr // 2)), -0.0056, 0.0056)

    def plate(x0, x1, r, waist):
        g = uu([Point(x0, 0).buffer(r, 24), Point(x1, 0).buffer(r, 24),
                Polygon([(x0, -waist), (x1, -waist), (x1, waist), (x0, waist)])])
        g = g.buffer(0.0009, 8).buffer(-0.0009, 8)
        return _poly_xy(g, tol=0.00005)
    pin_ = plate(0.0, p, 0.0041, 0.0034)
    pout = plate(p, 2 * p, 0.0040, 0.0033)
    for sg in (1, -1):
        a, b = sorted((sg * 0.00290, sg * 0.00400))
        _prism_xy_mb(mb, pin_, a, b)
        a, b = sorted((sg * 0.00405, sg * 0.00515))
        _prism_xy_mb(mb, pout, a, b)
    return mb


def _build_chain():
    ch = chain_layout()
    N = ch["n_links"]
    p = CHAIN_P
    tpl = _chain_template()
    V = tpl.coords()
    sizes, loops, _ = tpl._all_faces()
    starts = np.concatenate([[0], np.cumsum(sizes)[:-1]])
    nv = len(V)
    mb = MU.MeshBuilder()
    allV = np.concatenate([V + np.array([2 * p * m, 0.0, 0.0]) for m in range(N // 2)])
    mb.verts(allV)
    for k in range(len(sizes)):
        f = loops[starts[k]:starts[k] + sizes[k]]
        arr = np.stack([f + nv * m for m in range(N // 2)])
        mb.faces(arr) if sizes[k] in (3, 4) else [mb.face(list(r)) for r in arr]
    chain = mb.to_object(_nm("timing_chain"), _COL, smooth_angle=40.0, materials=["steel_dark"])
    nr = _seg(16)
    npn = 2 * max(8, nr // 2)
    chain["roller_layout"] = [int(nv), 0, 2 * nr, 2 * nr + npn, 2 * nr + npn + 2 * nr]
    # path curve (2D, cyclic POLY), points reversed so the Curve modifier runs with the chain
    # 3D POLY cyclic curve, CCW points: the Curve modifier then maps mesh x = arc length
    # from P[0] along the point order (2D curves get re-oriented for filling).
    P = chain_points(spacing=0.25 * MM)
    cu = bpy.data.curves.new(_nm("chain_path"), "CURVE")
    cu.dimensions = "3D"
    cu.twist_mode = "MINIMUM"
    sp = cu.splines.new("POLY")
    sp.points.add(len(P) - 1)
    co = np.zeros((len(P), 4))
    co[:, 0], co[:, 1], co[:, 3] = P[:, 0], P[:, 1], 1.0
    sp.points.foreach_set("co", co.ravel())
    sp.use_cyclic_u = True
    path = bpy.data.objects.new(_nm("chain_path"), cu)
    _COL.objects.link(path)
    path.location = (0.0, Y_CHAIN, 0.0)
    path.rotation_euler = (PI / 2, 0.0, 0.0)
    path.hide_render = True
    chain.parent = path
    mod = chain.modifiers.new("chain_path", "CURVE")
    mod.object = path
    mod.deform_axis = "POS_X"
    return chain, path


# ===========================================================================
# Combustion-chamber shaped solids (head chamber cutter and gas volumes)
# ===========================================================================
X_SQUISH = APEX_H / TAN_A          # |x| beyond which the roof is the flat squish band


def _clip_region(yc, r, clip):
    """Disk (centre (0, yc), radius r) intersected with keep-half-planes
    clip = [((px, py), (nx, ny))]: keep points with (p - pt).n <= 0."""
    Point, Polygon, _ = _shapely()
    g = Point(0.0, yc).buffer(r, 96)
    for (pt, nr) in (clip or []):
        pt, nr = np.asarray(pt, float), np.asarray(nr, float)
        t = np.array([-nr[1], nr[0]])
        big = 1.0
        hp = Polygon([pt + t * big, pt - t * big, pt - t * big - nr * big, pt + t * big - nr * big])
        g = g.intersection(hp)
    return g


def _chamber_solid(key, yc, r, z_bot, top_off=0.0, clip=None, material=None, max_seg=2.5 * MM):
    """Closed solid over a (clipped) disk: flat bottom at z_bot, top following the
    pent roof (roof_z + top_off).  Top = planar strips between the roof break lines
    (x = 0, +-X_SQUISH) so the ridge is exact; returns (object, bottom vertex indices)."""
    g = _clip_region(yc, r, clip)
    B = np.asarray(g.exterior.coords)[:-1]
    if MU._signed_area(B) < 0:
        B = B[::-1]
    breaks = [-X_SQUISH, 0.0, X_SQUISH]
    pts = []
    n = len(B)
    for k in range(n):
        p0, p1 = B[k], B[(k + 1) % n]
        pts.append(p0)
        cuts = []
        for xb in breaks:
            if (p0[0] - xb) * (p1[0] - xb) < 0:
                u = (xb - p0[0]) / (p1[0] - p0[0])
                cuts.append((u, p0 + u * (p1 - p0)))
        L = float(np.hypot(*(p1 - p0)))
        m = int(math.ceil(L / max_seg))
        for j in range(1, m):
            cuts.append((j / m, p0 + (j / m) * (p1 - p0)))
        for u, q in sorted(cuts, key=lambda t: t[0]):
            if np.hypot(*(q - pts[-1])) > 1e-7:
                pts.append(q)
    P = np.array(pts)
    for xb in breaks:          # snap break points exactly
        P[np.abs(P[:, 0] - xb) < 1e-9, 0] = xb
    zt = roof_z(P[:, 0]) + top_off
    mb = MU.MeshBuilder()
    top = mb.verts(np.stack([P[:, 0], P[:, 1], zt], 1))
    bot = mb.verts(np.stack([P[:, 0], P[:, 1], np.full(len(P), z_bot)], 1))
    mb.bridge(bot, top)
    mb.face(list(bot)[::-1])
    # top: one planar ngon per strip
    edges = [-1.0] + breaks + [1.0]
    for x0, x1 in zip(edges[:-1], edges[1:]):
        sel = [i for i in range(len(P)) if x0 - 1e-9 <= P[i, 0] <= x1 + 1e-9]
        if len(sel) < 3:
            continue
        # loop order: the selected indices form one contiguous run of the loop
        idx = np.array(sel)
        gaps = np.nonzero(np.diff(idx) > 1)[0]
        if len(gaps):
            k = gaps[0] + 1
            idx = np.concatenate([idx[k:], idx[:k]])
        mb.face([int(top[i]) for i in idx])
    ob = mb.to_object(_nm(key), _COL, smooth_angle=None, fix_normals=True,
                      materials=[material] if material else None)
    return ob, np.asarray(bot)


# ===========================================================================
# Port / runner paths
# ===========================================================================

def _port_path(c, kind, i):
    F, a, _ = valve_frame(c, kind, i)
    yc = Y_CYL[c - 1]
    sy = 1.0 if i == 0 else -1.0
    if kind == "intake":
        sx = -1.0
        pts = [F - 0.004 * a, F + 0.010 * a, F + 0.020 * a + np.array([sx * 0.010, -sy * 0.003, 0.003]),
               np.array([sx * 0.062, yc + sy * 0.0105, ZH + 0.044]),
               np.array([sx * (HEAD_W_TOP + 0.004), yc + sy * 0.0105, ZH + 0.052])]
        r = 0.0125
    else:
        sx = 1.0
        pts = [F - 0.004 * a, F + 0.008 * a, F + 0.016 * a + np.array([sx * 0.010, -sy * 0.002, 0.0]),
               np.array([sx * 0.058, yc + sy * 0.0100, ZH + 0.034]),
               np.array([sx * (HEAD_W_TOP + 0.004), yc + sy * 0.0100, ZH + 0.036])]
        r = 0.0112
    return pts, r


PLENUM = dict(x=-0.222, z=ZH - 0.015, r=0.046, y0=Y_CYL[-1] - 0.052, y1=Y_CYL[0] + 0.052)


def _runner_path(c):
    yc = Y_CYL[c - 1]
    return [(-HEAD_W_TOP - 0.0002, yc, ZH + 0.052), (-0.140, yc, ZH + 0.062), (-0.190, yc, ZH + 0.098),
            (-0.240, yc, ZH + 0.085), (-0.252, yc, ZH + 0.030), (-0.238, yc, ZH - 0.010)]


COLLECTOR = (0.172, -0.081)


def _header_path(c):
    yc = Y_CYL[c - 1]
    inlet = {1: (0.150, -0.059), 2: (0.194, -0.059), 3: (0.194, -0.103), 4: (0.150, -0.103)}[c]
    mid_y = {1: 0.040, 2: -0.040, 3: -0.120, 4: -0.200}[c]
    return [(HEAD_W_TOP + 0.0002, yc, ZH + 0.036), (0.132, yc, ZH + 0.032), (inlet[0], mid_y, ZH - 0.050),
            (inlet[0], inlet[1], 0.110), (inlet[0], inlet[1], 0.050)]


# ===========================================================================
# Housings
# ===========================================================================

def _sym(half):
    """Mirror a right-half profile [(x, z) ...] (top to bottom) into a closed polygon."""
    h = [tuple(p) for p in half]
    return np.array(h + [(-x, z) for (x, z) in reversed(h)])


BLOCK_HALF = [(0.068, ZD), (0.068, 0.150), (0.072, 0.120), (0.080, 0.092), (0.096, 0.055), (0.102, 0.020),
              (0.102, -0.062), (0.110, -0.062), (0.110, -0.070)]
BAY_HALF = [(0.058, 0.089), (0.078, 0.068), (0.090, 0.040), (0.095, 0.015), (0.095, -0.090)]
Z_PAN = -0.070
HEAD_BOLT_X = 0.068


def _build_block():
    Point, Polygon, uu = _shapely()
    blk = MU.extrude_polygon(_nm("block"), _sym(BLOCK_HALF), YR, YF, chamfer=0.0015, collection=_COL,
                             material="cast_iron")
    add = []
    for ym in Y_MAIN:
        for sx in (-1, 1):
            add.append(_cyl_along(f"blk_boss{ym:.3f}{sx}", (sx * HEAD_BOLT_X, ym, 0.098), (sx * HEAD_BOLT_X, ym, ZD),
                                  0.0105, seg=_seg(24), material="cast_iron"))
    for c in range(1, 5):      # cylinder barrel ribs on the skirt (between the bulkheads)
        for sx in (-1, 1):
            add.append(_box(f"blk_rib{c}{sx}", sx * 0.102 - 0.004, sx * 0.102 + 0.004, Y_CYL[c - 1] - 0.004,
                            Y_CYL[c - 1] + 0.004, -0.06, 0.03, r=0.0015, material="cast_iron"))
    _bool(blk, add, "UNION")
    cut = []
    for c in range(1, 5):
        cut.append(_cyl_along(f"blk_bore{c}", (0, Y_CYL[c - 1], BORE_BOT), (0, Y_CYL[c - 1], ZD + 0.002),
                              _cr(BORE_R, _seg(96)), seg=_seg(96), material="cast_iron"))
    stad = uu([Point(0.0, y).buffer(0.058, 64) for y in Y_CYL] +
              [Polygon([(-0.058, Y_CYL[-1]), (0.058, Y_CYL[-1]), (0.058, Y_CYL[0]), (-0.058, Y_CYL[0])])])
    inner = uu([Point(0.0, y).buffer(0.049, 64) for y in Y_CYL])
    jk = stad.difference(inner)
    cut.append(_plate_xy("blk_jacket", _poly_xy(jk), 0.105, ZD - 0.007,
                         holes=[np.asarray(h.coords)[:-1] for h in jk.interiors], material="cast_iron"))
    bay = _sym(BAY_HALF)
    for i in range(4):
        cut.append(MU.extrude_polygon(_nm(f"blk_bay{i}"), bay, Y_MAIN[i + 1] + 0.0105, Y_MAIN[i] - 0.0105,
                                      collection=_COL, material="cast_iron"))
    low = _sym([(0.095, 0.0), (0.095, -0.090)])
    cut.append(MU.extrude_polygon(_nm("blk_low"), low, YR + 0.006, YF - 0.006, collection=_COL, material="cast_iron"))
    cut.append(MU.cylinder(_nm("blk_mainbore"), MAIN_R + 0.002, YR + 0.004, YF - 0.004, segments=_seg(64),
                           collection=_COL, material="cast_iron"))
    cut.append(MU.cylinder(_nm("blk_fseal"), 0.0235, YF - 0.010, YF + 0.002, segments=_seg(48), collection=_COL,
                           material="cast_iron"))
    cut.append(MU.cylinder(_nm("blk_rseal"), 0.0432, YR - 0.002, YR + 0.0105, segments=_seg(64), collection=_COL,
                           material="cast_iron"))
    for ym in Y_MAIN:
        for sx in (-1, 1):
            cut.append(_cyl_along(f"blk_hb{ym:.3f}{sx}", (sx * HEAD_BOLT_X, ym, ZD - 0.085),
                                  (sx * HEAD_BOLT_X, ym, ZD + 0.002), 0.0056, seg=_seg(16)))
            cut.append(_cyl_along(f"blk_mb{ym:.3f}{sx}", (sx * 0.040, ym, -0.002), (sx * 0.040, ym, 0.055), 0.0052,
                                  seg=_seg(16)))
    for c in range(1, 5):
        for sx in (-1, 1):
            for sy in (-1, 1):
                p = (sx * 0.0535, Y_CYL[c - 1] + sy * 0.032)
                cut.append(_cyl_along(f"blk_wh{c}{sx}{sy}", (p[0], p[1], ZD - 0.009), (p[0], p[1], ZD + 0.002),
                                      0.0035, seg=_seg(12)))
    for c in range(1, 5):          # core-plug holes into the water jacket
        for sx in (-1, 1):
            cut.append(_cyl_along(f"blk_cp{c}{sx}", (sx * 0.054, Y_CYL[c - 1], 0.160), (sx * 0.070, Y_CYL[c - 1], 0.160),
                                  0.0155, seg=_seg(32)))
    _bool(blk, cut)
    plugs = []
    for c in range(1, 5):
        for sx in (-1, 1):
            pl = MU.lathe(_nm(f"blk_plug{c}{sx}"), [(0.0, -0.0010), (0.0153, -0.0010), (0.0154, 0.0012),
                                                     (0.0144, 0.0016), (0.0, 0.0006)], _seg(32), collection=_COL,
                          material="brass")
            q = Vector((0, 1, 0)).rotation_difference(Vector((sx, 0, 0)))
            _xform(pl, Matrix.Translation((sx * 0.0675, Y_CYL[c - 1], 0.160)) @ q.to_matrix().to_4x4())
            plugs.append(pl)
    ds = MU.tube_along(_nm("blk_dip"), [(-0.1022, -0.175, -0.030), (-0.112, -0.175, 0.120), (-0.126, -0.175, 0.268)],
                       0.0042, segments=_seg(12), bend_radius=0.06, caps=True, collection=_COL,
                       material="steel_machined")
    hd = MU.lathe(_nm("blk_diph"), [(0.0045, 0.0), (0.012, 0.0), (0.012, 0.012), (0.0045, 0.012)], _seg(24),
                  closed=True, caps=False, collection=_COL, material="plastic_black")
    q = Vector((0, 1, 0)).rotation_difference(Vector((1, 0, 0)))
    _xform(hd, Matrix.Translation((-0.132, -0.175, 0.282)) @ q.to_matrix().to_4x4() @ Matrix.Translation((0, -0.006, 0)))
    return _join("block", [blk] + plugs + [ds, hd])


def _face_mat(ob, mat, pred):
    me = ob.data
    n = len(me.polygons)
    if n == 0:
        return ob
    cen = np.zeros(n * 3)
    nrm = np.zeros(n * 3)
    me.polygons.foreach_get("center", cen)
    me.polygons.foreach_get("normal", nrm)
    sel = pred(cen.reshape(-1, 3), nrm.reshape(-1, 3))
    if not np.any(sel):
        return ob
    idx = MU.assign_material(ob, mat, faces=[])
    mi = np.zeros(n, dtype=np.int32)
    me.polygons.foreach_get("material_index", mi)
    mi[sel] = idx
    me.polygons.foreach_set("material_index", mi)
    return ob


def _build_main_caps():
    caps, bolts, shells = [], [], []
    for i, ym in enumerate(Y_MAIN):
        cp = _box(f"mcap{i}", -0.052, 0.052, ym - 0.0105, ym + 0.0105, -0.050, -0.0001, r=0.004, material="cast_iron")
        _bool(cp, [MU.cylinder(_nm(f"mcap_b{i}"), MAIN_R + 0.002, ym - 0.02, ym + 0.02, segments=_seg(64),
                               collection=_COL, material="cast_iron")])
        caps.append(cp)
        for sx in (-1, 1):
            h = _plate_xy(f"mbolt{i}{sx}", _circle((sx * 0.040, ym), 0.0085, 6), -0.0575, -0.0505, chamfer=0.0006,
                          material="steel_dark")
            w = _plate_xy(f"mwash{i}{sx}", _circle((sx * 0.040, ym), 0.0105, _seg(24)), -0.0505, -0.0501,
                          material="steel_machined")
            bolts += [h, w]
        shells.append(MU.cylinder(_nm(f"mshell{i}"), MAIN_R + 0.00195, ym - 0.0095, ym + 0.0095, segments=_seg(64),
                                  inner_radius=_cr(MAIN_R + 0.00005, _seg(64)), collection=_COL, material="copper"))
    return _join("main_caps", caps), _join("main_bolts", bolts), _join("main_shells", shells)


HEAD_HALF = [(HEAD_W_TOP, Z_CAM), (HEAD_W_TOP, ZH + 0.020), (HEAD_W_DECK + 0.006, ZH + 0.005),
             (HEAD_W_DECK, ZH)]


def _valley_polys(kind):
    """XZ polygons (one per cam) of the cam valley cavity above the buckets."""
    Point, Polygon, uu = _shapely()
    sx = -1.0 if kind == "intake" else 1.0
    C = np.array([sx * X_CAM, Z_CAM])
    a2 = np.array([sx * SA, CA])
    b2 = np.array([CA, -sx * SA])
    fl = [C - 0.023 * a2 + 0.031 * b2, C - 0.023 * a2 - 0.031 * b2]
    g = uu([Point(*C).buffer(0.0302, 48), Polygon([fl[0], fl[1], fl[1] + 0.03 * a2, fl[0] + 0.03 * a2]),
            Polygon([(C[0] - 0.0315, C[1]), (C[0] + 0.0315, C[1]), (C[0] + 0.0315, Z_CAM + 0.004),
                     (C[0] - 0.0315, Z_CAM + 0.004)])]).convex_hull
    # clip below the floor plane (distance 23 mm from the cam centre toward the valve)
    t = b2
    big = 1.0
    P0 = C - 0.023 * a2
    keep = Polygon([P0 + t * big, P0 - t * big, P0 - t * big + a2 * big, P0 + t * big + a2 * big])
    return _poly_xy(g.intersection(keep))


def _head_jacket():
    """Coolant jacket cavity of the head: a band above the deck minus a wall around
    every chamber, port, valve seat/guide, plug boss/well and the cam valleys."""
    reg = np.array([(-0.076, ZH + 0.0065), (0.076, ZH + 0.0065), (0.100, ZH + 0.030), (0.105, ZH + 0.072),
                    (-0.105, ZH + 0.072), (-0.100, ZH + 0.030)])
    J = MU.extrude_polygon(_nm("hd_jacket"), reg, YR + 0.010, YF - 0.012, collection=_COL, material="cast_aluminium")
    ex = []
    for c in range(1, 5):
        yc = Y_CYL[c - 1]
        ch, _ = _chamber_solid(f"hj_ch{c}", yc, BORE_R + 0.004, ZH - 0.010, top_off=0.0055)
        ex.append(ch)
        ex.append(_cyl_along(f"hj_pb{c}", (0, yc, Z_APEX - 0.010), (0, yc, Z_APEX + 0.022), 0.0125, seg=_seg(24)))
        ex.append(_cyl_along(f"hj_pw{c}", (0, yc, Z_APEX + 0.015), (0, yc, Z_CAM), 0.0175, seg=_seg(24)))
        for kind in ("intake", "exhaust"):
            rh = VALVE_HEAD_R[kind]
            for i in (0, 1):
                F, a, _ = valve_frame(c, kind, i)
                k = f"{c}{kind[0]}{i}"
                ex.append(_cyl_along(f"hj_th{k}", F - 0.003 * a, F + 0.014 * a, rh + 0.005, seg=_seg(24)))
                ex.append(_cyl_along(f"hj_gd{k}", F + 0.006 * a, F + (Y_SPRING_SEAT + 0.002) * a, 0.0095,
                                     seg=_seg(16)))
                ex.append(_cyl_along(f"hj_bk{k}", F + (Y_SPRING_SEAT - 0.004) * a, F + (BUCKET_TOP + 0.012) * a,
                                     BUCKET_R[kind] + 0.0045, seg=_seg(32)))
                pts, r = _port_path(c, kind, i)
                pts = [pts[0] - 0.004 * a] + pts[1:-1] + [pts[-1] + np.array([0.02 if kind == "exhaust" else -0.02,
                                                                              0, 0])]
                ex.append(MU.tube_along(_nm(f"hj_pt{k}"), pts, r + 0.004, segments=_seg(16), bend_radius=0.02,
                                        bend_samples=4, caps=True, collection=_COL))
    _bool(J, ex)
    return J


def _build_head():
    head = MU.extrude_polygon(_nm("head"), _sym(HEAD_HALF), YR, YF, chamfer=0.002, collection=_COL,
                              material="cast_aluminium")
    cut = []
    for c in range(1, 5):
        yc = Y_CYL[c - 1]
        ch, _ = _chamber_solid(f"hd_ch{c}", yc, BORE_R + 0.0001, ZH - 0.002, top_off=0.0002,
                               material="cast_aluminium")
        cut.append(ch)
        cut.append(_cyl_along(f"hd_plugh{c}", (0, yc, Z_APEX - 0.004), (0, yc, Z_APEX + 0.0195), 0.0072,
                              seg=_seg(24)))
        cut.append(_cyl_along(f"hd_well{c}", (0, yc, Z_APEX + 0.019), (0, yc, Z_CAM + 0.003), 0.0135,
                              seg=_seg(32), material="cast_aluminium"))
        for kind in ("intake", "exhaust"):
            rh = VALVE_HEAD_R[kind]
            for i in (0, 1):
                F, a, _ = valve_frame(c, kind, i)
                k = f"{c}{kind[0]}{i}"
                cut.append(_cyl_along(f"hd_thr{k}", F - 0.003 * a, F + 0.012 * a, rh - 0.0025, seg=_seg(32),
                                      material="cast_aluminium"))
                cut.append(_cyl_along(f"hd_seat{k}", F - 0.003 * a, F + 0.0065 * a, rh + 0.00152, seg=_seg(40),
                                      material="machined_aluminium"))
                cut.append(_cyl_along(f"hd_gd{k}", F + 0.006 * a, F + (Y_SPRING_SEAT + 0.001) * a,
                                      0.0055, seg=_seg(16)))
                cut.append(_cyl_along(f"hd_bk{k}", F + Y_SPRING_SEAT * a, F + (BUCKET_TOP + 0.012) * a,
                                      _cr(BUCKET_R[kind] + 0.00005, _seg(56)), seg=_seg(56),
                                      material="machined_aluminium"))
                pts, r = _port_path(c, kind, i)
                cut.append(MU.tube_along(_nm(f"hd_port{k}"), pts, r, segments=_seg(24), bend_radius=0.02,
                                         bend_samples=6, caps=True, collection=_COL, material="cast_aluminium"))
    for kind in ("intake", "exhaust"):
        poly = _valley_polys(kind)
        js = [YR + 0.008] + sum([[ym - CAM_JOURNAL_W / 2 - 0.0005, ym + CAM_JOURNAL_W / 2 + 0.0005]
                                 for ym in Y_MAIN[::-1]], []) + [YF - 0.007]
        for k in range(0, len(js), 2):
            cut.append(MU.extrude_polygon(_nm(f"hd_val{kind[0]}{k}"), poly, js[k], js[k + 1], collection=_COL,
                                          material="cast_aluminium"))
        sx = -1.0 if kind == "intake" else 1.0
        jb = MU.cylinder(_nm(f"hd_jb{kind[0]}"), _cr(CAM_JOURNAL_R + 0.00005, _seg(48)), YR + 0.006, Y_MAIN[0] + 0.009,
                         segments=_seg(48), collection=_COL, material="machined_aluminium")
        _translate(jb, (sx * X_CAM, 0, Z_CAM))
        fb = MU.cylinder(_nm(f"hd_fb{kind[0]}"), 0.0168, Y_MAIN[0] + 0.007, YF + 0.002, segments=_seg(40),
                         collection=_COL, material="machined_aluminium")
        _translate(fb, (sx * X_CAM, 0, Z_CAM))
        cut += [jb, fb]
    if _DETAIL == "high" or True:
        cut.append(_head_jacket())
        for c in range(1, 5):
            for sx in (-1, 1):
                for sy in (-1, 1):
                    p = (sx * 0.0535, Y_CYL[c - 1] + sy * 0.032)
                    cut.append(_cyl_along(f"hd_wh{c}{sx}{sy}", (p[0], p[1], ZH - 0.002), (p[0], p[1], ZH + 0.012),
                                          0.0035, seg=_seg(12)))
        for sx in (-1, 1):            # longitudinal rib under the cam-cover joint
            add_r = _box(f"hd_rib{sx}", min(sx * HEAD_W_TOP, sx * (HEAD_W_TOP + 0.004)) - 0.0001,
                         max(sx * HEAD_W_TOP, sx * (HEAD_W_TOP + 0.004)) + 0.0001, YR + 0.004, YF - 0.004,
                         Z_CAM - 0.016, Z_CAM - 0.008, r=0.0015, material="cast_aluminium")
            _bool(head, [add_r], "UNION")
    _bool(head, cut)
    _face_mat(head, "machined_aluminium", lambda c, n: ((n[:, 2] < -0.99) & (np.abs(c[:, 2] - ZH) < 1e-5)) |
              ((n[:, 2] > 0.99) & (np.abs(c[:, 2] - Z_CAM) < 1e-5)))
    return head


def _build_head_parts():
    guides, seats, caps = [], [], []
    for c in range(1, 5):
        for kind in ("intake", "exhaust"):
            rh = VALVE_HEAD_R[kind]
            for i in (0, 1):
                k = f"{c}{kind[0]}{i}"
                rg = _cr(STEM_R + 0.00004, _seg(24))
                g = MU.lathe(_nm(f"gd{k}"), [(rg, Y_GUIDE_BOT), (0.00547, Y_GUIDE_BOT + 0.0008),
                                             (0.00547, Y_GUIDE_TOP), (rg, Y_GUIDE_TOP)], _seg(24),
                             closed=True, caps=False, collection=_COL, material="steel_machined")
                sl = MU.lathe(_nm(f"ss{k}"), [(rg, Y_GUIDE_TOP - 0.004), (0.0066, Y_GUIDE_TOP - 0.004),
                                              (0.0066, Y_GUIDE_TOP + 0.002), (0.0045, Y_GUIDE_TOP + 0.0035),
                                              (rg, Y_GUIDE_TOP + 0.0035)], _seg(24),
                              closed=True, caps=False, collection=_COL, material="rubber")
                ws = MU.lathe(_nm(f"sw{k}"), [(0.0068, Y_SPRING_SEAT - 0.001), (0.0135, Y_SPRING_SEAT - 0.001),
                                              (0.0135, Y_SPRING_SEAT - 0.00002), (0.0068, Y_SPRING_SEAT - 0.00002)],
                              _seg(32), closed=True, caps=False, collection=_COL, material="steel_machined")
                # seat insert (45 deg face 0.05 mm proud of the valve face)
                o = 0.035 * MM
                sp = [(rh + 0.0002, 0.0), (rh + 0.0015, 0.0), (rh + 0.0015, 0.0065), (rh - 0.0025, 0.0065),
                      (rh - 0.0025, 0.0045), (rh - 0.0017 + o, 0.0029 + o), (rh + o, 0.0012 + o), (rh + 0.0002, 0.0012)]
                st = MU.lathe(_nm(f"st{k}"), sp, _seg(40), closed=True, caps=False, collection=_COL,
                              material="steel_machined")
                for ob in (g, sl, ws, st):
                    _apply_vf(ob, c, kind, i)
                guides += [g, sl, ws]
                seats.append(st)
    for kind in ("intake", "exhaust"):
        sx = -1.0 if kind == "intake" else 1.0
        for j, ym in enumerate(Y_MAIN):
            cp = _box(f"ccap{kind[0]}{j}", sx * X_CAM - 0.021, sx * X_CAM + 0.021, ym - CAM_JOURNAL_W / 2 + 0.0003,
                      ym + CAM_JOURNAL_W / 2 - 0.0003, Z_CAM + 0.0001, Z_CAM + 0.020, r=0.003,
                      material="cast_aluminium")
            jb = MU.cylinder(_nm(f"ccap_b{kind[0]}{j}"), _cr(CAM_JOURNAL_R + 0.00005, _seg(48)), ym - 0.02, ym + 0.02,
                             segments=_seg(48), collection=_COL, material="machined_aluminium")
            _translate(jb, (sx * X_CAM, 0, Z_CAM))
            _bool(cp, [jb])
            caps.append(cp)
            for bx in (-1, 1):
                x = sx * X_CAM + bx * 0.0155
                caps.append(_plate_xy(f"ccb{kind[0]}{j}{bx}", _circle((x, ym), 0.0063, 6), Z_CAM + 0.0200,
                                      Z_CAM + 0.0262, chamfer=0.0005, material="steel_dark"))
    return _join("valve_guides", guides), _join("valve_seats", seats), _join("cam_caps", caps)


def _build_gasket():
    Point, Polygon, uu = _shapely()
    out = uu([Polygon([(-0.068, YR), (0.068, YR), (0.068, YF), (-0.068, YF)])] +
             [Point(sx * HEAD_BOLT_X, ym).buffer(0.0105, 24) for ym in Y_MAIN for sx in (-1, 1)])
    holes = [_circle((0, y), BORE_R + 0.0008, _seg(96)) for y in Y_CYL]
    holes += [_circle((sx * HEAD_BOLT_X, ym), 0.0058, 16) for ym in Y_MAIN for sx in (-1, 1)]
    holes += [_circle((sx * 0.0535, Y_CYL[c] + sy * 0.032), 0.0035, 12) for c in range(4) for sx in (-1, 1)
              for sy in (-1, 1)]
    g = _plate_xy("head_gasket", _poly_xy(out), ZD + 0.00002, ZH - 0.00002, holes=holes, material="steel_dark")
    return g


def _build_cam_cover():
    half = [(HEAD_W_TOP - 0.002, Z_CAM + COVER_H - 0.004), (HEAD_W_TOP - 0.002, Z_CAM + 0.0002)]
    prof = np.array([(-0.062, Z_CAM + COVER_H), (0.062, Z_CAM + COVER_H), (0.086, Z_CAM + COVER_H - 0.006),
                     (HEAD_W_TOP - 0.002, Z_CAM + 0.016), (HEAD_W_TOP - 0.002, Z_CAM + 0.0002),
                     (-HEAD_W_TOP + 0.002, Z_CAM + 0.0002), (-HEAD_W_TOP + 0.002, Z_CAM + 0.016),
                     (-0.086, Z_CAM + COVER_H - 0.006)])
    del half
    cov = MU.extrude_polygon(_nm("cam_cover"), prof, YR + 0.001, YF - 0.0005, chamfer=0.006, collection=_COL,
                             material="cast_aluminium")
    add = []
    for c in range(1, 5):
        add.append(_cyl_along(f"cc_boss{c}", (0, Y_CYL[c - 1], Z_CAM + COVER_H - 0.010),
                              (0, Y_CYL[c - 1], Z_CAM + COVER_H + 0.004), 0.019, seg=_seg(40),
                              material="cast_aluminium"))
    add.append(_cyl_along("cc_fill", (-0.045, 0.018, Z_CAM + COVER_H - 0.008), (-0.045, 0.018, Z_CAM + COVER_H + 0.006),
                          0.022, seg=_seg(40), material="cast_aluminium"))
    for k in range(5):      # longitudinal stiffening ribs on the top
        y = Y_MAIN[k]
        add.append(_box(f"cc_rib{k}", -0.058, 0.058, y - 0.0025, y + 0.0025, Z_CAM + COVER_H - 0.004,
                        Z_CAM + COVER_H + 0.0025, r=0.001, material="cast_aluminium"))
    _bool(cov, add, "UNION")
    inner = np.array([(-0.058, Z_CAM + COVER_H - 0.0035), (0.058, Z_CAM + COVER_H - 0.0035),
                      (0.0825, Z_CAM + COVER_H - 0.0095), (HEAD_W_TOP - 0.0055, Z_CAM + 0.014),
                      (HEAD_W_TOP - 0.0055, Z_CAM - 0.002), (-HEAD_W_TOP + 0.0055, Z_CAM - 0.002),
                      (-HEAD_W_TOP + 0.0055, Z_CAM + 0.014), (-0.0825, Z_CAM + COVER_H - 0.0095)])
    cut = [MU.extrude_polygon(_nm("cc_in"), inner, YR + 0.0045, YF - 0.004, collection=_COL,
                              material="cast_aluminium")]
    for c in range(1, 5):
        cut.append(_cyl_along(f"cc_hole{c}", (0, Y_CYL[c - 1], Z_CAM), (0, Y_CYL[c - 1], Z_CAM + COVER_H + 0.006),
                              0.0125, seg=_seg(32), material="cast_aluminium"))
    cut.append(_cyl_along("cc_fh", (-0.045, 0.018, Z_CAM), (-0.045, 0.018, Z_CAM + COVER_H + 0.007), 0.016,
                          seg=_seg(32)))
    for sx in (-1, 1):          # front-wall notches around the cam noses (sealed by the timing cover)
        cut.append(_cyl_along(f"cc_notch{sx}", (sx * X_CAM, YF - 0.012, Z_CAM), (sx * X_CAM, YF + 0.002, Z_CAM),
                              0.0185, seg=_seg(32), material="cast_aluminium"))
    _bool(cov, cut)
    cap = MU.lathe(_nm("cc_cap"), [(0.0, 0.0), (0.0225, 0.0), (0.0235, 0.002), (0.0235, 0.010), (0.021, 0.013),
                                   (0.0, 0.013)], _seg(48), axis="Z", collection=_COL, material="plastic_black")
    _translate(cap, (-0.045, 0.018, Z_CAM + COVER_H + 0.0062))
    bolts = []
    for c in range(5):
        for sx in (-1, 1):
            x = sx * 0.072
            bolts.append(_plate_xy(f"cc_b{c}{sx}", _circle((x, Y_MAIN[c]), 0.0055, 6), Z_CAM + COVER_H - 0.0045,
                                   Z_CAM + COVER_H + 0.0015, chamfer=0.0004, material="steel_dark"))
    return _join("cam_cover", [cov, cap] + bolts)


def _plug_z():
    return Z_APEX


def _build_spark_plug(c):
    yc = Y_CYL[c - 1]
    z0 = _plug_z()
    th = [(0.0058, z0)]
    for k in range(15):
        zz = z0 + 0.0005 + k * 0.00122
        th += [(0.0068, zz), (0.0070, zz + 0.0004), (0.0068, zz + 0.0008), (0.0062, zz + 0.0011)]
    shell = th + [(0.0064, z0 + 0.0190), (0.0094, z0 + 0.0192), (0.0094, z0 + 0.0204), (0.0080, z0 + 0.0206),
                  (0.0080, z0 + 0.0334), (0.0072, z0 + 0.0350), (0.0058, z0 + 0.0352), (0.0058, z0)]
    sh = MU.lathe(_nm(f"sp{c}_shell"), shell, _seg(32), axis="Z", closed=True, caps=False, collection=_COL,
                  material="steel_machined")
    hx = _plate_xy(f"sp{c}_hex", _circle((0, 0), 0.0092, 6), z0 + 0.0206, z0 + 0.0326, chamfer=0.0007,
                   holes=[_circle((0, 0), 0.0059, 24)], material="steel_machined")
    ins = [(0.0, z0 + 0.003), (0.0026, z0 + 0.003), (0.0042, z0 + 0.012), (0.0057, z0 + 0.016),
           (0.0057, z0 + 0.0352)]
    zz = z0 + 0.0352
    for k in range(5):
        ins += [(0.0054, zz + 0.0035), (0.0050, zz + 0.0045), (0.0054, zz + 0.0055)]
        zz += 0.0063
    ins += [(0.0048, z0 + 0.0700), (0.0040, z0 + 0.0720), (0.0, z0 + 0.0720)]
    ins_o = MU.lathe(_nm(f"sp{c}_ins"), ins, _seg(32), axis="Z", collection=_COL, material="ceramic")
    term = MU.lathe(_nm(f"sp{c}_term"), [(0.0, z0 + 0.0718), (0.0028, z0 + 0.0718), (0.0028, z0 + 0.0760),
                                          (0.0036, z0 + 0.0765), (0.0036, z0 + 0.0790), (0.0, z0 + 0.0795)],
                    _seg(16), axis="Z", collection=_COL, material="steel_machined")
    el = MU.lathe(_nm(f"sp{c}_el"), [(0.0, z0 - 0.0025), (0.0011, z0 - 0.0025), (0.0011, z0 + 0.0032),
                                     (0.0, z0 + 0.0032)], 12, axis="Z", collection=_COL, material="copper")
    gnd = MU.tube_along(_nm(f"sp{c}_gnd"), [(0.0056, 0.0, z0 + 0.0004), (0.0056, 0.0, z0 - 0.0040),
                                            (-0.0008, 0.0, z0 - 0.0040)], 0.0008, segments=8, bend_radius=0.0012,
                        caps=True, collection=_COL, material="steel_machined")
    parts = [sh, hx, ins_o, term, el, gnd]
    for p in parts:
        _translate(p, (0.0, yc, 0.0))
    return _join(f"spark_plug{c}", parts)


def _build_coils():
    parts = []
    top = Z_CAM + COVER_H + 0.004
    for c in range(1, 5):
        yc = Y_CYL[c - 1]
        zt = _plug_z() + 0.066
        parts.append(_cyl_along(f"coil_boot{c}", (0, yc, zt), (0, yc, zt + 0.020), 0.0098, seg=_seg(24),
                                material="rubber"))
        parts.append(_cyl_along(f"coil_body{c}", (0, yc, zt + 0.0195), (0, yc, top), 0.0102, seg=_seg(24),
                                material="plastic_black"))
        parts.append(_box(f"coil_head{c}", -0.012, 0.040, yc - 0.0155, yc + 0.0155, top - 0.001, top + 0.024, r=0.004,
                          material="plastic_black"))
        parts.append(_box(f"coil_conn{c}", 0.040, 0.055, yc - 0.008, yc + 0.008, top + 0.006, top + 0.019, r=0.002,
                          material="plastic_black"))
    return _join("coils", parts)


def _guide_sector(key, centre, R0, R1, a0, a1, y0, y1, material):
    a = np.linspace(a0, a1, max(8, int(abs(a1 - a0) * R1 / 0.004)))
    outer = np.stack([centre[0] + R1 * np.cos(a), centre[1] + R1 * np.sin(a)], 1)
    inner = np.stack([centre[0] + R0 * np.cos(a[::-1]), centre[1] + R0 * np.sin(a[::-1])], 1)
    return MU.extrude_polygon(_nm(key), np.concatenate([outer, inner]), y0, y1, chamfer=0.0006, collection=_COL,
                              material=material)


def _arc_info(idx):
    for sg in chain_layout()["segments"]:
        if sg[0] == "arc" and sg[1] == idx:
            _, _, c, r, a0, sweep, _, _ = sg
            return c, abs(r), a0, a0 + sweep
    return None


def _build_chain_guides():
    """Tight-side guide (fixed) and slack-side tensioner arm + hydraulic tensioner."""
    out = {}
    pl = 0.0044          # chain plate half height + clearance (shoe face from the roller line)
    for key, idx, ext in (("chain_guide", 4, 0.030), ("tensioner_arm", 1, 0.030)):
        c, R, a_s, a_e = _arc_info(idx)
        lo, hi = sorted((a_s, a_e))
        lo -= ext / R
        hi += ext / R
        shoe = _guide_sector(key + "_shoe", c, R - pl - 0.0055, R - pl, lo, hi, Y_CHAIN - 0.0085, Y_CHAIN + 0.0085,
                             "plastic_black")
        back = _guide_sector(key + "_back", c, R - pl - 0.0135, R - pl - 0.0054, lo + 0.004, hi - 0.004,
                             Y_CHAIN - 0.0055, Y_CHAIN + 0.0055, "cast_aluminium")
        parts = [shoe, back]
        # ends: the lower end of each guide (closer to the crank)
        ends = [c + (R - pl - 0.016) * np.array([math.cos(a), math.sin(a)]) for a in (lo + 0.006, hi - 0.006)]
        lower, upper = sorted(ends, key=lambda p: p[1])
        if key == "chain_guide":
            pts = [lower, upper]
        else:
            pts = [lower]
        for j, p in enumerate(pts):
            parts.append(_cyl_along(f"{key}_boss{j}", (p[0], YF + 0.0002, p[1]), (p[0], Y_CHAIN + 0.0055, p[1]), 0.0085,
                                    seg=_seg(24), material="cast_aluminium"))
            parts.append(MU.extrude_polygon(_nm(f"{key}_bolt{j}"), _circle(p, 0.0065, 6), Y_CHAIN + 0.0055,
                                            Y_CHAIN + 0.0115, chamfer=0.0005, collection=_COL, material="steel_dark"))
        out[key] = _join(key, parts)
        if key == "tensioner_arm":
            # plunger pad on the upper end and the tensioner body behind it (outside the arm)
            a_u = hi - 0.012
            n_out = np.array([math.cos(a_u), math.sin(a_u)])      # from the shoe centre outward... (toward centre c)
            pad = c + (R - pl - 0.0135) * n_out
            d = -n_out                                             # away from the chain, toward c
            body_c = pad + 0.024 * d
            t = np.array([-d[1], d[0]])
            plunger = _cyl_along("tens_plunger", (pad[0], Y_CHAIN, pad[1]), (body_c[0], Y_CHAIN, body_c[1]), 0.0055,
                                 seg=_seg(20), material="steel_machined")
            corners = [body_c + 0.006 * d + 0.016 * t, body_c + 0.006 * d - 0.016 * t,
                       body_c + 0.030 * d - 0.016 * t, body_c + 0.030 * d + 0.016 * t]
            body = MU.extrude_polygon(_nm("tens_body"), np.array(corners), YF + 0.0002, Y_CHAIN + 0.009,
                                      chamfer=0.0015, collection=_COL, material="cast_aluminium")
            bolts = []
            for j, sgn in enumerate((-1, 1)):
                p = body_c + 0.022 * d + sgn * 0.011 * t
                bolts.append(MU.extrude_polygon(_nm(f"tens_bolt{j}"), _circle(p, 0.0055, 6), Y_CHAIN + 0.009,
                                                Y_CHAIN + 0.014, chamfer=0.0004, collection=_COL, material="steel_dark"))
            out["tensioner"] = _join("tensioner", [plunger, body] + bolts)
    return out


def _timing_cover_outline():
    Point, Polygon, uu = _shapely()
    geo = [Point(0, 0).buffer(0.050, 48), Point(X_CAM, Z_CAM).buffer(R_TIP_CAM + 0.011, 64),
           Point(-X_CAM, Z_CAM).buffer(R_TIP_CAM + 0.011, 64),
           Polygon([(-0.104, Z_PAN), (0.104, Z_PAN), (0.104, 0.0), (-0.104, 0.0)])]
    for idx in (1, 4):
        c, R, a_s, a_e = _arc_info(idx)
        for a in np.linspace(min(a_s, a_e) - 0.04, max(a_s, a_e) + 0.04, 12):
            geo.append(Point(c[0] + (R - 0.05) * math.cos(a), c[1] + (R - 0.05) * math.sin(a)).buffer(0.012))
    return uu(geo).convex_hull


def _build_timing_cover():
    o = _timing_cover_outline()
    outer = _poly_xy(o.buffer(0.004, 16), tol=0.0003)
    inner = _poly_xy(o.buffer(-0.0005, 16), tol=0.0003)
    cov = MU.extrude_polygon(_nm("timing_cover"), outer, YF + 0.0002, Y_COVER_FRONT, chamfer=0.003,
                             collection=_COL, material="cast_aluminium")
    _bool(cov, [MU.cylinder(_nm("tc_boss"), 0.036, Y_COVER_FRONT - 0.002, Y_COVER_FRONT + 0.003, segments=_seg(48),
                            chamfer=0.001, collection=_COL, material="cast_aluminium")], "UNION")
    cut = [MU.extrude_polygon(_nm("tc_in"), inner, YF - 0.001, YF + COVER_IN, collection=_COL,
                              material="cast_aluminium"),
           MU.cylinder(_nm("tc_seal"), 0.0236, YF + 0.01, Y_COVER_FRONT + 0.005, segments=_seg(48), collection=_COL,
                       material="steel_machined")]
    _bool(cov, cut)
    bolts = []
    ring = o.buffer(-0.002)
    L = ring.exterior.length
    for k in range(14):
        p = ring.exterior.interpolate(L * k / 14)
        if p.y < Z_PAN + 0.012:
            continue
        bolts.append(MU.extrude_polygon(_nm(f"tc_b{k}"), _circle((p.x, p.y), 0.0055, 6), Y_COVER_FRONT,
                                        Y_COVER_FRONT + 0.0055, chamfer=0.0004, collection=_COL, material="steel_dark"))
    seal = MU.cylinder(_nm("tc_lip"), 0.0232, Y_COVER_FRONT - 0.006, Y_COVER_FRONT + 0.0028, segments=_seg(48),
                       inner_radius=_cr(0.01810, _seg(48)), collection=_COL, material="rubber")
    return _join("timing_cover", [cov, seal] + bolts)


def _build_oil_pan():
    yf = Y_COVER_FRONT - 0.001
    flange = _plate_xy("pan", np.array([(-0.110, YR + 0.0005), (0.110, YR + 0.0005), (0.110, yf), (-0.110, yf)]),
                       Z_PAN - 0.008, Z_PAN - 0.0001, chamfer=0.0012, material="cast_aluminium")
    front = np.array([(-0.100, Z_PAN - 0.004), (0.100, Z_PAN - 0.004), (0.095, -0.118), (-0.095, -0.118)])
    rear = np.array([(-0.100, Z_PAN - 0.004), (0.100, Z_PAN - 0.004), (0.090, -0.182), (-0.090, -0.182)])
    a = MU.extrude_polygon(_nm("pan_f"), front, -0.050, yf - 0.004, chamfer=0.006, collection=_COL,
                           material="cast_aluminium")
    b = MU.extrude_polygon(_nm("pan_r"), rear, YR + 0.004, -0.030, chamfer=0.008, collection=_COL,
                           material="cast_aluminium")
    fins = [_box(f"pan_fin{k}", -0.080, 0.080, y - 0.002, y + 0.002, -0.186, -0.150, r=0.0015,
                 material="cast_aluminium") for k, y in enumerate(np.linspace(YR + 0.03, -0.06, 5))]
    _bool(flange, [a, b] + fins, "UNION")
    fi = np.array([(-0.097, -0.060), (0.097, -0.060), (0.0915, -0.1145), (-0.0915, -0.1145)])
    ri = np.array([(-0.097, -0.060), (0.097, -0.060), (0.0865, -0.1785), (-0.0865, -0.1785)])
    cut = [MU.extrude_polygon(_nm("pan_fi"), fi, -0.0465, yf - 0.0075, collection=_COL, material="cast_aluminium"),
           MU.extrude_polygon(_nm("pan_ri"), ri, YR + 0.0075, -0.0335, collection=_COL, material="cast_aluminium")]
    _bool(flange, cut)
    parts = [flange]
    parts.append(_plate_xy("pan_plug", _circle((0.0, -0.200), 0.009, 6), -0.192, -0.1855, chamfer=0.0007,
                           material="steel_dark"))
    for k, y in enumerate(np.linspace(YR + 0.012, yf - 0.010, 8)):
        for sx in (-1, 1):
            parts.append(_plate_xy(f"pan_b{k}{sx}", _circle((sx * 0.1045, y), 0.0050, 6), Z_PAN - 0.0130,
                                   Z_PAN - 0.0079, chamfer=0.0004, material="steel_dark"))
    return _join("oil_pan", parts)


def _build_intake():
    outer, inner = [], []
    PL = PLENUM
    for c in range(1, 5):
        path = _runner_path(c)
        outer.append(MU.tube_along(_nm(f"in_r{c}"), path, 0.0195, segments=_seg(28), bend_radius=0.03,
                                   bend_samples=8, caps=True, collection=_COL, material="cast_aluminium"))
        p_in = [(path[0][0] + 0.004, path[0][1], path[0][2])] + list(path[1:-1]) + \
               [(path[-1][0] + 0.004, path[-1][1], path[-1][2] - 0.012)]
        inner.append(MU.tube_along(_nm(f"in_rb{c}"), p_in, 0.0166, segments=_seg(28), bend_radius=0.03,
                                   bend_samples=8, caps=True, collection=_COL, material="cast_aluminium"))
    outer.append(_box("in_flange", -HEAD_W_TOP - 0.0095, -HEAD_W_TOP - 0.0002, Y_CYL[-1] - 0.030, Y_CYL[0] + 0.030,
                      ZH + 0.052 - 0.026, ZH + 0.052 + 0.026, r=0.003, material="cast_aluminium"))
    prof = [(0.0, PL["y0"]), (PL["r"] * 0.6, PL["y0"] + 0.002), (PL["r"] * 0.92, PL["y0"] + 0.010),
            (PL["r"], PL["y0"] + 0.022), (PL["r"], PL["y1"] - 0.010), (PL["r"] - 0.004, PL["y1"]),
            (0.0, PL["y1"])]
    pl = MU.lathe(_nm("in_plenum"), prof, _seg(56), collection=_COL, material="cast_aluminium")
    _translate(pl, (PL["x"], 0, PL["z"]))
    outer.append(pl)
    tb = MU.cylinder(_nm("in_tb"), 0.034, PL["y1"] - 0.004, PL["y1"] + 0.050, segments=_seg(48), chamfer=0.002,
                     collection=_COL, material="cast_aluminium")
    _translate(tb, (PL["x"], 0, PL["z"]))
    tbf = MU.cylinder(_nm("in_tbf"), 0.041, PL["y1"] + 0.043, PL["y1"] + 0.051, segments=_seg(48), chamfer=0.001,
                      collection=_COL, material="cast_aluminium")
    _translate(tbf, (PL["x"], 0, PL["z"]))
    outer += [tb, tbf]
    man = _union("intake_manifold", outer)
    pli = [(0.0, PL["y0"] + 0.004), (PL["r"] - 0.004, PL["y0"] + 0.024), (PL["r"] - 0.004, PL["y1"] - 0.008),
           (0.0, PL["y1"] - 0.004)]
    pin_ = MU.lathe(_nm("in_pli"), pli, _seg(56), collection=_COL, material="cast_aluminium")
    _translate(pin_, (PL["x"], 0, PL["z"]))
    tbi = MU.cylinder(_nm("in_tbi"), 0.0300, PL["y1"] - 0.012, PL["y1"] + 0.056, segments=_seg(48), collection=_COL,
                      material="machined_aluminium")
    _translate(tbi, (PL["x"], 0, PL["z"]))
    _bool(man, inner + [pin_, tbi])
    # throttle plate + spindle + position sensor
    yb = PL["y1"] + 0.028
    plate = MU.cylinder(_nm("in_tp"), 0.0296, -0.0008, 0.0008, segments=_seg(48), collection=_COL,
                        material="steel_machined")
    _xform(plate, Matrix.Translation((PL["x"], yb, PL["z"])) @ Matrix.Rotation(math.radians(8.0), 4, "X"))
    spindle = _cyl_along("in_tps", (PL["x"] - 0.036, yb, PL["z"]), (PL["x"] + 0.036, yb, PL["z"]), 0.0035, seg=12,
                         material="steel_machined")
    tps = _box("in_tpsb", PL["x"] - 0.050, PL["x"] - 0.034, yb - 0.016, yb + 0.016, PL["z"] - 0.016, PL["z"] + 0.016,
               r=0.003, material="plastic_black")
    return _join("intake_manifold", [man, plate, spindle, tps])


def _build_fuel_rail():
    parts = []
    xr, zr = -0.131, ZH + 0.090
    parts.append(_cyl_along("fr_rail", (xr, Y_CYL[-1] - 0.035, zr), (xr, Y_CYL[0] + 0.035, zr), 0.0085, seg=_seg(24),
                            material="steel_machined"))
    parts.append(_cyl_along("fr_feed", (xr, Y_CYL[0] + 0.035, zr), (xr, Y_CYL[0] + 0.060, zr), 0.0045, seg=12,
                            material="steel_machined"))
    for c in range(1, 5):
        yc = Y_CYL[c - 1]
        parts.append(_cyl_along(f"fr_inj{c}", (xr, yc, zr - 0.004), (xr + 0.003, yc, ZH + 0.074), 0.0068,
                                seg=_seg(20), material="plastic_black"))
        parts.append(_cyl_along(f"fr_tip{c}", (xr + 0.003, yc, ZH + 0.075), (xr + 0.0045, yc, ZH + 0.066), 0.0045,
                                seg=12, material="steel_machined"))
        parts.append(_box(f"fr_conn{c}", xr - 0.020, xr - 0.006, yc - 0.006, yc + 0.006, zr - 0.012, zr - 0.002,
                          r=0.0015, material="plastic_black"))
    return _join("fuel_rail", parts)


def _build_exhaust():
    outer, inner = [], []
    for c in range(1, 5):
        path = _header_path(c)
        outer.append(MU.tube_along(_nm(f"ex_t{c}"), path, 0.0185, segments=_seg(24), bend_radius=0.035,
                                   bend_samples=8, caps=True, collection=_COL, material="steel_dark"))
        p_in = [(path[0][0] - 0.004, path[0][1], path[0][2])] + list(path[1:-1]) + \
               [(path[-1][0], path[-1][1], path[-1][2] - 0.015)]
        inner.append(MU.tube_along(_nm(f"ex_tb{c}"), p_in, 0.0163, segments=_seg(24), bend_radius=0.035,
                                   bend_samples=8, caps=True, collection=_COL, material="steel_dark"))
    cx, cy = COLLECTOR
    col = MU.lathe(_nm("ex_col"), [(0.0, 0.072), (0.054, 0.072), (0.054, 0.060), (0.028, 0.010), (0.0, 0.010)],
                   _seg(48), axis="Z", collection=_COL, material="steel_dark")
    _xform(col, Matrix.Translation((cx, cy, 0.0)) @ Matrix.Diagonal((1, 1, 1, 1)))
    # lathe axis='Z' maps profile y -> z (we built heights directly)
    outer.append(col)
    dp = [(cx, cy, 0.016), (cx + 0.002, cy - 0.012, -0.030), (0.192, -0.190, -0.066), (0.200, -0.360, -0.072)]
    outer.append(MU.tube_along(_nm("ex_dp"), dp, 0.0265, segments=_seg(28), bend_radius=0.06, bend_samples=8,
                               caps=True, collection=_COL, material="steel_dark"))
    outer.append(_box("ex_flange", HEAD_W_TOP + 0.0002, HEAD_W_TOP + 0.0095, Y_CYL[-1] - 0.026, Y_CYL[0] + 0.026,
                      ZH + 0.036 - 0.022, ZH + 0.036 + 0.022, r=0.003, material="steel_dark"))
    dfl = MU.cylinder(_nm("ex_dfl"), 0.045, -0.009, 0.0, segments=_seg(40), chamfer=0.0008, collection=_COL,
                      material="steel_dark")
    _translate(dfl, (0.200, -0.351, -0.072))
    outer.append(dfl)
    man = _union("exhaust_manifold", outer)
    ci = MU.lathe(_nm("ex_ci"), [(0.0, 0.068), (0.050, 0.068), (0.0245, 0.012), (0.0, 0.012)], _seg(48), axis="Z",
                  collection=_COL, material="steel_dark")
    _translate(ci, (cx, cy, 0.0))
    dpi = [(cx, cy, 0.030)] + dp[1:-1] + [(0.200, -0.365, -0.072)]
    inner += [ci, MU.tube_along(_nm("ex_dpi"), dpi, 0.0240, segments=_seg(28), bend_radius=0.06, bend_samples=8,
                                caps=True, collection=_COL, material="steel_dark")]
    _bool(man, inner)
    studs = []
    for c in range(1, 5):
        for sy in (-1, 1):
            for sz in (-1, 1):
                p = (HEAD_W_TOP + 0.0095, Y_CYL[c - 1] + sy * 0.021, ZH + 0.036 + sz * 0.015)
                if sz > 0 and sy < 0:
                    continue
                studs.append(_cyl_along(f"ex_n{c}{sy}{sz}", p, (p[0] + 0.007, p[1], p[2]), 0.0055, seg=6,
                                        material="steel_machined"))
    return _join("exhaust_manifold", [man] + studs)


def _build_oil_filter():
    p0 = np.array([-0.101, -0.120, 0.022])
    d = np.array([-0.80, 0.0, -0.60])
    d /= np.linalg.norm(d)
    boss = _cyl_along("of_boss", p0 - 0.002 * d, p0 + 0.010 * d, 0.030, seg=_seg(40), material="cast_iron")
    base = _cyl_along("of_base", p0 + 0.010 * d, p0 + 0.016 * d, 0.0375, seg=_seg(48), material="steel_machined")
    can = MU.lathe(_nm("of_can"), [(0.0, 0.0), (0.0380, 0.0), (0.0385, 0.004), (0.0385, 0.078), (0.034, 0.086),
                                   (0.0, 0.087)], _seg(48), collection=_COL, material="paint_black")
    q = Vector((0, 1, 0)).rotation_difference(Vector(d))
    _xform(can, Matrix.Translation(Vector(p0 + 0.016 * d)) @ q.to_matrix().to_4x4())
    return _join("oil_filter", [boss, base, can])


# ===========================================================================
# Gas volumes and spark
# ===========================================================================
CROWN_TDC = R_THROW + ROD_L + COMP_H


def _build_gas(set_name, c, kind, clip=None):
    key = f"gas_{c}_{kind}" if set_name == "full" else f"gas_{set_name}_{c}_{kind}"
    zb = CROWN_TDC + 0.0002
    ob, bot = _chamber_solid(key, Y_CYL[c - 1], BORE_R - 0.0004, zb, top_off=-0.0004, clip=clip,
                             material="gas_" + kind, max_seg=4.0 * MM)
    MU.smooth_by_angle(ob, 50.0)
    me = ob.data
    ob.shape_key_add(name="Basis", from_mix=False)
    kb = ob.shape_key_add(name="stroke", from_mix=False)
    co = np.zeros(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    co[bot, 2] -= S.STROKE
    kb.data.foreach_set("co", co.ravel())
    ob.visible_shadow = False
    ob["cv_opacity"] = 0.0
    ob["cv_glow"] = 0.0
    return ob


def _spark_material():
    m = bpy.data.materials.get("eng_spark")
    if m is not None and m.node_tree is not None:
        return m
    m = bpy.data.materials.new("eng_spark")
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    lw = nt.nodes.new("ShaderNodeLayerWeight")
    lw.inputs["Blend"].default_value = 0.35
    inv = nt.nodes.new("ShaderNodeMath")
    inv.operation = "SUBTRACT"
    inv.inputs[0].default_value = 1.0
    nt.links.new(lw.outputs["Facing"], inv.inputs[1])
    em = nt.nodes.new("ShaderNodeEmission")
    em.inputs["Color"].default_value = (0.70, 0.82, 1.0, 1.0)
    em.inputs["Strength"].default_value = 60.0
    tr = nt.nodes.new("ShaderNodeBsdfTransparent")
    mix = nt.nodes.new("ShaderNodeMixShader")
    nt.links.new(inv.outputs[0], mix.inputs[0])
    nt.links.new(tr.outputs[0], mix.inputs[1])
    nt.links.new(em.outputs[0], mix.inputs[2])
    shader = mix.outputs[0]
    if MAT is not None and hasattr(MAT, "presentation_group"):
        grp = nt.nodes.new("ShaderNodeGroup")
        grp.node_tree = MAT.presentation_group()
        nt.links.new(shader, grp.inputs["Shader"])
        shader = grp.outputs["Shader"]
    nt.links.new(shader, out.inputs["Surface"])
    m.diffuse_color = (0.75, 0.85, 1.0, 0.8)
    try:
        m.cycles.emission_sampling = "AUTO"
        m.surface_render_method = "BLENDED"
    except Exception:
        pass
    return m


def _build_spark(c):
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=_seg(16), v_segments=_seg(12), radius=0.0034)
    bmesh.ops.translate(bm, vec=Vector((0.0, Y_CYL[c - 1], _plug_z() - 0.0031)), verts=bm.verts[:])
    me = bpy.data.meshes.new(_nm(f"spark{c}"))
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(_nm(f"spark{c}"), me)
    _COL.objects.link(ob)
    me.materials.append(_spark_material())
    me.shade_smooth()
    ob.visible_shadow = False
    ob["cv_opacity"] = 0.0
    ob["cv_glow"] = 0.0
    return ob


# ===========================================================================
# Cutaway variants
# ===========================================================================
R_LONG = [[((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0))]]
R_B = [[((0.0, Y_P0, 0.0), (0.0, 1.0, 0.0))]]
R_H = [[((0.0, Y_P1, 0.0), (0.0, 1.0, 0.0))],
       [((0.0, Y_P0, 0.0), (0.0, 1.0, 0.0)), ((X_SLOT, 0.0, 0.0), (-1.0, 0.0, 0.0)),
        ((-X_SLOT, 0.0, 0.0), (1.0, 0.0, 0.0))]]
R_FRONT = [[((0.0, YF + 0.014, 0.0), (0.0, 1.0, 0.0))]]
ACCESSORIES = ("accessory_belt", "water_pump", "wp_pulley", "belt_idler", "idler_arm", "alternator", "alt_pulley")
HOUSINGS_LONG = ("block", "main_caps", "main_bolts", "main_shells", "head", "head_gasket", "valve_guides",
                 "valve_seats", "cam_caps", "cam_cover", "coils", "spark_plug1", "spark_plug2", "spark_plug3",
                 "spark_plug4", "timing_cover", "oil_pan", "intake_manifold", "fuel_rail", "oil_filter",
                 "exhaust_manifold")


def _variant_spec(variant):
    if variant == "long":
        d = {k: R_LONG for k in HOUSINGS_LONG + ACCESSORIES}
        d["accessory_belt"] = "ALL"
        return d
    if variant == "cyl1":
        d = {k: R_B for k in ("block", "oil_pan", "main_caps", "main_bolts", "main_shells", "head_gasket",
                              "spark_plug1", "coils", "crankshaft", "piston1", "conrod1", "timing_cover",
                              "chain_guide", "tensioner_arm", "tensioner", "timing_chain", "crank_sprocket",
                              "damper", "cam_sprocket_intake", "cam_sprocket_exhaust", "intake_manifold", "fuel_rail",
                              "exhaust_manifold") + ACCESSORIES}
        d["alternator"] = d["alt_pulley"] = "ALL"
        d.update({k: R_H for k in ("head", "cam_cover", "cam_caps", "valve_guides", "valve_seats",
                                   "cam_intake", "cam_exhaust",
                                   "valve_1i0", "valve_1e0", "spring_1i0", "spring_1e0")})
        return d
    if variant == "front":
        d = {"timing_cover": R_FRONT, "cam_cover": "ALL", "coils": "ALL"}
        d.update({k: "ALL" for k in ("accessory_belt", "water_pump", "wp_pulley", "belt_idler", "idler_arm",
                                     "damper")})
        return d
    return {}


def _world_verts(ob):
    me = ob.data
    co = np.zeros(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    M = np.array(MU._world_matrix(ob))
    return co @ M[:3, :3].T + M[:3, 3]


def _in_region(V, R):
    m = np.ones(len(V), dtype=bool)
    for co, no in R:
        m &= (V - np.asarray(co)) @ np.asarray(no) > 1e-9
    return m


def _copy(ob, name):
    c = ob.copy()
    c.data = ob.data.copy()
    c.name = name
    c.data.name = name
    _COL.objects.link(c)
    return c


def _cut_region(ob, R):
    if len(R) == 1:
        MU.cut_and_apply(ob, plane=R[0])
    else:
        MU.cut_and_apply(ob, wedge=R)


def _merge_local(target, others):
    """Merge meshes of objects sharing target's transform into target (local space)."""
    bm = bmesh.new()
    bm.from_mesh(target.data)
    mats = [m for m in target.data.materials]
    for o in others:
        me = o.data.copy()
        remap = []
        for m in me.materials:
            if m not in mats:
                mats.append(m)
                target.data.materials.append(m)
            remap.append(mats.index(m))
        if remap:
            mi = np.zeros(len(me.polygons), dtype=np.int32)
            me.polygons.foreach_get("material_index", mi)
            me.polygons.foreach_set("material_index", np.asarray(remap, np.int32)[np.clip(mi, 0, len(remap) - 1)])
        bm.from_mesh(me)
        bpy.data.meshes.remove(me)
        d = o.data
        bpy.data.objects.remove(o)
        if d.users == 0:
            bpy.data.meshes.remove(d)
    bm.to_mesh(target.data)
    bm.free()
    target.data.update()
    return target


def _make_pieces(ob, key, variant, regions):
    """-> (kept_obj or None, removed_obj or None, status) for one part."""
    if regions == "ALL":
        return None, ob, "removed"
    V = _world_verts(ob)
    m = np.zeros(len(V), dtype=bool)
    for R in regions:
        m |= _in_region(V, R)
    if not m.any():
        return ob, None, "kept"
    if m.all():
        return None, ob, "removed"
    kept = _copy(ob, _nm(f"{key}__{variant}_kept"))
    for R in regions:
        _cut_region(kept, R)
    pieces = []
    for i, R in enumerate(regions):
        pc = _copy(ob, _nm(f"{key}__{variant}_rm{i}"))
        for co, no in R:
            MU.cut_and_apply(pc, plane=(co, tuple(-np.asarray(no))))
            if len(pc.data.polygons) == 0:
                break
        for Rj in regions[:i]:
            if len(pc.data.polygons):
                _cut_region(pc, Rj)
        if len(pc.data.polygons):
            pieces.append(pc)
        else:
            d = pc.data
            bpy.data.objects.remove(pc)
            bpy.data.meshes.remove(d)
    removed = None
    if pieces:
        removed = _merge_local(pieces[0], pieces[1:]) if len(pieces) > 1 else pieces[0]
        removed.name = _nm(f"{key}__{variant}_removed")
        removed.data.name = removed.name
    if len(kept.data.polygons) == 0:
        d = kept.data
        bpy.data.objects.remove(kept)
        bpy.data.meshes.remove(d)
        kept = None
    return kept, removed, "cut"


# ===========================================================================
# build()
# ===========================================================================

def _place_valves(parts, frames, movers, root):
    for c in range(1, 5):
        for kind in ("intake", "exhaust"):
            for i in (0, 1):
                k = f"{c}{kind[0]}{i}"
                vf = rig.empty(_nm(f"vf_{k}"), parent=root, col=_COL, size=0.01)
                M = _valve_matrix(c, kind, i)
                vf.location = M.to_translation()
                vf.rotation_euler = ((PI / 2), (-1.0 if kind == "intake" else 1.0) * VALVE_ANGLE, 0.0)
                frames[k] = vf
                v = _build_valve(c, kind, i)
                v.parent = vf
                v.location = (0.0, -float(valve_lift_ft(0.0, c, kind)), 0.0)
                parts[f"valve_{k}"] = v
                movers.append(dict(obj=v, type="valve", cyl=c, kind=kind))
                s = _build_spring(c, kind, i)
                s.parent = vf
                s.location = (0.0, Y_SPRING_SEAT, 0.0)
                parts[f"spring_{k}"] = s
                movers.append(dict(obj=s, type="spring", cyl=c, kind=kind))


def build(opts=None):
    global _COL, _DETAIL
    t0 = time.time()
    opts = dict(opts or {})
    cutaways = list(opts.get("cutaways", opts.get("cutaway", ["none"])) or ["none"])
    if isinstance(cutaways, str):
        cutaways = [cutaways]
    _DETAIL = opts.get("detail", "high")
    _COL = rig.collection(opts.get("collection", "engine"))
    root = rig.empty(_nm("root"), col=_COL, size=0.15)
    root.rotation_mode = "XYZ"
    parts, movers, frames = {}, [], {}
    ch = chain_layout()

    # ---- moving parts --------------------------------------------------------
    crank = _build_crankshaft()
    parts["crankshaft"] = crank
    movers.append(dict(obj=crank, type="spin", k=1.0, phase=0.0))
    for c in range(1, 5):
        rod = _build_rod(c)
        pz, _, cx, cz, rt = kin.slider_crank(0.0, c)
        rod.location = (float(cx), Y_CYL[c - 1], float(cz))
        rod.rotation_mode = "XYZ"
        rod.rotation_euler = (0.0, -float(rt), 0.0)
        parts[f"conrod{c}"] = rod
        movers.append(dict(obj=rod, type="rod", cyl=c))
        pis = _build_piston(c)
        pis.location = (0.0, Y_CYL[c - 1], float(pz))
        parts[f"piston{c}"] = pis
        movers.append(dict(obj=pis, type="piston", cyl=c))
    for kind in ("intake", "exhaust"):
        sx = -1.0 if kind == "intake" else 1.0
        cam = _build_cam(kind)
        cam.location = (sx * X_CAM, 0.0, Z_CAM)
        parts["cam_" + kind] = cam
        movers.append(dict(obj=cam, type="spin", k=S.CAM_SPEED_RATIO, phase=0.0))
        sp = _sprocket_obj("cam_sprocket_" + kind, S.CAM_SPROCKET_TEETH, hub=(0.050, 0.014, 0.0), holes=6, bolt=True)
        sp.location = cam.location
        parts["cam_sprocket_" + kind] = sp
        movers.append(dict(obj=sp, type="spin", k=S.CAM_SPEED_RATIO, phase=ch["phases"][kind]))
    cs = _sprocket_obj("crank_sprocket", S.CRANK_SPROCKET_TEETH, bore=0.0396, hub=(0.052, 0.014, 0.0))
    parts["crank_sprocket"] = cs
    movers.append(dict(obj=cs, type="spin", k=1.0, phase=ch["phases"]["crank"]))
    dm = _build_damper()
    parts["damper"] = dm
    movers.append(dict(obj=dm, type="spin", k=1.0, phase=0.0))
    fw, ring = _build_flywheel()
    parts["flywheel"] = fw
    movers.append(dict(obj=fw, type="spin", k=1.0, phase=0.0))
    if ring is not None:
        ring.parent = fw
        parts["ring_gear"] = ring
    chain, path = _build_chain()
    parts["timing_chain"] = chain
    movers.append(dict(obj=chain, type="chain"))
    _place_valves(parts, frames, movers, root)

    # ---- static parts ----------------------------------------------------------
    parts["block"] = _build_block()
    parts["main_caps"], parts["main_bolts"], parts["main_shells"] = _build_main_caps()
    parts["head"] = _build_head()
    parts["valve_guides"], parts["valve_seats"], parts["cam_caps"] = _build_head_parts()
    parts["head_gasket"] = _build_gasket()
    parts["cam_cover"] = _build_cam_cover()
    parts["coils"] = _build_coils()
    for c in range(1, 5):
        parts[f"spark_plug{c}"] = _build_spark_plug(c)
    parts["timing_cover"] = _build_timing_cover()
    parts["oil_pan"] = _build_oil_pan()
    parts.update(_build_chain_guides())
    if opts.get("manifolds", True):
        parts["intake_manifold"] = _build_intake()
        parts["fuel_rail"] = _build_fuel_rail()
        parts["exhaust_manifold"] = _build_exhaust()
        parts["oil_filter"] = _build_oil_filter()
    if opts.get("accessories", True):
        acc = _build_accessories()
        parts.update(acc)
        for name, key in (("wp", "wp_pulley"), ("idler", "belt_idler"), ("alt", "alt_pulley")):
            ob = acc[key]
            ob.location = (ACC[name][0][0], 0.0, ACC[name][0][1])
            movers.append(dict(obj=ob, type="spin", k=accessory_ratio(name), phase=0.0))

    for k, ob in parts.items():
        if ob.parent is None:
            ob.parent = root
    path.parent = root
    for vf in frames.values():
        vf.parent = root

    # ---- cutaway variants -------------------------------------------------------
    mover_of = {id(m["obj"]): m for m in movers}
    pieces = {}
    for v in cutaways:
        if v == "none":
            continue
        spec = _variant_spec(v)
        kept_l, rem_l, rep_l = [], [], []
        for key, regions in spec.items():
            ob = parts.get(key)
            if ob is None:
                continue
            kept, removed, status = _make_pieces(ob, key, v, regions)
            if status == "kept":
                continue
            if status == "removed":
                rem_l.append(key)
                continue
            rep_l.append(key)
            for suffix, pc in (("kept", kept), ("removed", removed)):
                if pc is None:
                    continue
                pk = f"{key}__{v}_{suffix}"
                parts[pk] = pc
                (kept_l if suffix == "kept" else rem_l).append(pk)
                m = mover_of.get(id(ob))
                if m is not None:
                    m2 = dict(m)
                    m2["obj"] = pc
                    movers.append(m2)
        pieces[v] = dict(kept=kept_l, removed=rem_l, replaces=rep_l)

    # ---- shape keys, gas, sparks ---------------------------------------------------
    for m in movers:
        if m["type"] == "spring":
            _add_spring_key(m["obj"], m["kind"])
    gas_sets = {}
    if opts.get("gas", True):
        clips = {"full": None, "long": [((0.0, 0.0), (-1.0, 0.0))], "cyl1": [((0.0, Y_P0), (0.0, 1.0))]}
        for sname in ["full"] + [v for v in ("long", "cyl1") if v in cutaways]:
            gas_sets[sname] = {}
            cyls = [1] if sname == "cyl1" else [1, 2, 3, 4]
            for c in cyls:
                gas_sets[sname][c] = {}
                for kind in GAS_KINDS:
                    g = _build_gas(sname, c, kind, clips[sname])
                    g.parent = root
                    key = g.name[len(PREFIX):]
                    parts[key] = g
                    gas_sets[sname][c][kind] = key
                    movers.append(dict(obj=g, type="gas", cyl=c, kind=kind, set=sname))
        for c in range(1, 5):
            sk = _build_spark(c)
            sk.parent = root
            parts[f"spark{c}"] = sk
            movers.append(dict(obj=sk, type="spark", cyl=c))

    # ---- presentation props ------------------------------------------------------
    for ob in list(parts.values()):
        if ob.type == "MESH":
            if "cv_opacity" not in ob:
                rig.set_presentation(ob, 1.0, 0.0)
            elif "cv_glow" not in ob:
                ob["cv_glow"] = 0.0

    # ---- drop whole housings not needed ------------------------------------------
    if "none" not in cutaways and pieces:
        drop = set.intersection(*[set(p["replaces"]) for p in pieces.values()])
        for key in drop:
            ob = parts.pop(key)
            for m in [m for m in movers if m["obj"] is ob]:
                movers.remove(m)
            for v, p in pieces.items():
                if key in p["replaces"]:
                    p["replaces"].remove(key)
            d = ob.data
            bpy.data.objects.remove(ob)
            if d is not None and d.users == 0:
                bpy.data.meshes.remove(d)

    root.location = (S.X_CRANK, 0.0, S.Z_CRANK)

    # ---- assembly ----------------------------------------------------------------------
    def pick(*names):
        for n in names:
            if n in parts:
                return parts[n]
        return root

    def anc(names, off):
        ob = pick(*names) if isinstance(names, (tuple, list)) else pick(names)
        return (ob, tuple(float(x) for x in off))
    c1 = Y_CYL[0]
    anchors = {f"cyl{c}": (root, (0.0, Y_CYL[c - 1], ZD)) for c in range(1, 5)}
    anchors.update({
        "piston1": anc(("piston1", "piston1__cyl1_kept"), (0, 0, COMP_H)),
        "conrod1": anc(("conrod1", "conrod1__cyl1_kept"), (0, 0, ROD_L * 0.5)),
        "crankshaft": anc(("crankshaft", "crankshaft__cyl1_kept"), (0, Y_MAIN[2], 0)),
        "cam_intake": anc(("cam_intake", "cam_intake__cyl1_kept"), (0, Y_MAIN[2], 0)),
        "cam_exhaust": anc(("cam_exhaust", "cam_exhaust__cyl1_kept"), (0, Y_MAIN[2], 0)),
        "intake_valve1": anc(("valve_1i0", "valve_1i0__cyl1_kept"), (0, 0.030, 0)),
        "exhaust_valve1": anc(("valve_1e0", "valve_1e0__cyl1_kept"), (0, 0.030, 0)),
        "intake_valve_head1": anc(("valve_1i0", "valve_1i0__cyl1_kept"), (0, 0.002, 0)),
        "valve_spring1": anc(("spring_1i0", "spring_1i0__cyl1_kept"), (0, SPRING_LEN / 2, 0)),
        "bucket1": anc(("valve_1i0", "valve_1i0__cyl1_kept"), (0, BUCKET_TOP - 0.010, 0)),
        "spark_plug1": (root, (0.0, c1, Z_APEX + 0.045)),
        "spark_gap1": (root, (0.0, c1, Z_APEX - 0.003)),
        "timing_chain": (root, tuple(np.array([-X_CAM - 0.045, Y_CHAIN, Z_CAM * 0.5]))),
        "crank_sprocket": (root, (0.0, Y_CHAIN, -0.033)),
        "cam_sprocket": (root, (-X_CAM, Y_CHAIN, Z_CAM + 0.062)),
        "cam_sprocket_exhaust": (root, (X_CAM, Y_CHAIN, Z_CAM + 0.062)),
        "flywheel": anc("flywheel", (0.0, S.Y_FLYWHEEL_FACE, 0.0)),
        "flywheel_rim": (root, (0.0, S.Y_FLYWHEEL_FACE + 0.015, 0.140)),
        "ring_gear": (root, (0.0, S.Y_FLYWHEEL_FRONT - 0.006, 0.150)),
        "block": (root, (-0.100, -0.080, 0.080)),
        "head": (root, (-HEAD_W_TOP, -0.080, ZH + 0.090)),
        "cam_cover": (root, (0.0, -0.08, Z_CAM + COVER_H)),
        "timing_cover": (root, (0.06, Y_COVER_FRONT, 0.15)),
        "oil_pan": (root, (0.0, -0.15, -0.15)),
        "intake_manifold": (root, (-0.20, Y_CYL[1], ZH + 0.10)),
        "throttle_body": (root, (PLENUM["x"], PLENUM["y1"] + 0.03, PLENUM["z"] + 0.035)),
        "exhaust_manifold": (root, (0.17, -0.02, 0.15)),
        "tensioner": (root, tuple(np.array([X_CAM * 0.9, Y_CHAIN, Z_CAM * 0.45]))),
        "chain_guide": (root, tuple(np.array([-X_CAM * 0.9, Y_CHAIN, Z_CAM * 0.45]))),
        "combustion_chamber1": (root, (0.0, c1, ZH + 0.006)),
        "alternator": (root, (ACC["alt"][0][0] - 0.03, Y_BELT - 0.07, ACC["alt"][0][1] + 0.055)),
        "water_pump": (root, (ACC["wp"][0][0], Y_BELT + 0.012, ACC["wp"][0][1] + 0.03)),
        "accessory_belt": (root, (0.08, Y_BELT, 0.05)),
        "intake_port1": (root, (-0.060, c1, ZH + 0.044)),
        "exhaust_port1": (root, (0.058, c1, ZH + 0.034)),
    })
    groups = {
        "bottom_end": ["crankshaft"] + [f"conrod{c}" for c in range(1, 5)] + [f"piston{c}" for c in range(1, 5)],
        "valvetrain": ["cam_intake", "cam_exhaust"] + [k for k in parts if k.startswith(("valve_", "spring_"))
                                                      and "__" not in k],
        "timing": ["timing_chain", "crank_sprocket", "cam_sprocket_intake", "cam_sprocket_exhaust", "chain_guide",
                   "tensioner_arm", "tensioner"],
        "housings": ["block", "head", "head_gasket", "cam_cover", "timing_cover", "oil_pan", "cam_caps",
                     "main_caps", "main_bolts", "main_shells", "valve_guides", "valve_seats"],
        "manifolds": ["intake_manifold", "fuel_rail", "exhaust_manifold"],
        "ignition": [f"spark_plug{c}" for c in range(1, 5)] + ["coils"],
        "flywheel": ["flywheel", "ring_gear"],
        "sparks": [f"spark{c}" for c in range(1, 5)],
        "accessories": list(ACCESSORIES),
    }
    groups = {g: [n for n in v if n in parts] for g, v in groups.items()}
    meta = dict(
        power_path=[n for n in ["crankshaft", "flywheel", "ring_gear", "damper", "crank_sprocket"] +
                    [f"conrod{c}" for c in range(1, 5)] + [f"piston{c}" for c in range(1, 5)] if n in parts],
        cutaway_pieces=pieces, variants=cutaways, gas_sets=gas_sets, groups=groups,
        y=dict(chain=Y_CHAIN, block_front=YF, block_rear=YR, cover_front=Y_COVER_FRONT, damper=Y_DAMPER,
               flywheel_front=S.Y_FLYWHEEL_FRONT, flywheel_face=S.Y_FLYWHEEL_FACE, cyl=list(Y_CYL),
               mains=list(Y_MAIN), cut_p0=Y_P0, cut_p1=Y_P1),
        valve_frames={k: (tuple(map(float, valve_frame(int(k[0]), "intake" if k[1] == "i" else "exhaust",
                                                       int(k[2]))[0])),
                          tuple(map(float, valve_frame(int(k[0]), "intake" if k[1] == "i" else "exhaust",
                                                       int(k[2]))[1]))) for k in frames},
        chain=dict(n_links=ch["n_links"], length=ch["length"], push=ch["push"], phases=ch["phases"],
                   r_crank=ch["R_crank"], r_cam=ch["R_cam"], pitch=CHAIN_P,
                   roller_layout=list(parts["timing_chain"].get("roller_layout", [])) if "timing_chain" in parts
                   else []),
        cam_spacing=CAM_SPACING, lift_law="flat-tappet three-arc cam (engine.valve_lift_ft)",
        dims=dict(z_deck=ZD, z_head=ZH, z_cam=Z_CAM, z_apex=Z_APEX, valve_len=VLEN, bucket_top=BUCKET_TOP,
                  x_cam=X_CAM, valve_x=VALVE_X, valve_y=VALVE_Y, valve_angle=VALVE_ANGLE),
        build_time=None, _movers=movers, _frames=frames, _path=path, _root_children=None,
    )
    asm = rig.Assembly(name="engine", root=root, parts=parts, anchors=anchors,
                       explode={"flywheel": (0.0, -0.10, 0.0)}, meta=meta, _driver=_drive)
    _default_visibility(asm, cutaways[0])
    meta["build_time"] = time.time() - t0
    return asm


# ===========================================================================
# Visibility helpers
# ===========================================================================

def variant_objects(asm, variant):
    """dict(visible=[objs], removed=[objs], hidden=[objs]) for showing `variant`."""
    pcs = asm.meta["cutaway_pieces"]
    all_piece_keys = set()
    for p in pcs.values():
        all_piece_keys |= set(k for k in p["kept"] + p["removed"] if "__" in k)
    vis, rem, hid = [], [], []
    cur = pcs.get(variant, dict(kept=[], removed=[], replaces=[]))
    for key, ob in asm.parts.items():
        if ob.type != "MESH":
            continue
        if key.startswith("gas_") or key.startswith("spark"):
            continue
        if key in all_piece_keys:
            if key in cur["kept"]:
                vis.append(ob)
            elif key in cur["removed"]:
                rem.append(ob)
            else:
                hid.append(ob)
        else:
            if key in cur["replaces"]:
                hid.append(ob)
            elif key in cur["removed"]:
                rem.append(ob)
            else:
                vis.append(ob)
    return dict(visible=vis, removed=rem, hidden=hid)


def _default_visibility(asm, variant):
    vo = variant_objects(asm, variant)
    for ob in vo["visible"]:
        ob.hide_render = ob.hide_viewport = False
    for ob in vo["removed"] + vo["hidden"]:
        ob.hide_render = ob.hide_viewport = True
    gs = _gas_set_for(asm, variant)
    for sname, cyls in asm.meta["gas_sets"].items():
        for c, kinds in cyls.items():
            for key in kinds.values():
                ob = asm.parts[key]
                ob.hide_render = ob.hide_viewport = (sname != gs)


def _gas_set_for(asm, variant):
    gs = asm.meta.get("gas_sets", {})
    return variant if variant in gs else "full"


def _bake_const(ob, path, index, frames, values):
    """CONSTANT keys only where the value changes."""
    values = np.asarray(values, dtype=float)
    frames = np.asarray(frames, dtype=float)
    keep = np.concatenate([[True], values[1:] != values[:-1]])
    rig.bake_channel(ob, path, index, frames[keep], values[keep], "CONSTANT")


# ===========================================================================
# drive()
# ===========================================================================

def _per_frame(v, n, default):
    if v is None:
        return np.full(n, float(default))
    a = np.asarray(v, dtype=float)
    return np.full(n, float(a)) if a.ndim == 0 else a


def _drive(asm, track, pres):
    fr = np.asarray(track.frames)
    n = len(fr)
    th = np.asarray(track.theta_e, dtype=float)
    pres = pres or {}
    # ---- variant / visibility ----------------------------------------------------
    var = pres.get("variant")
    if var is not None:
        names = [var] * n if isinstance(var, str) else list(var)
        rem_op = _per_frame(pres.get("removed"), n, 0.0)
        vis_state = {}
        for v in sorted(set(names)):
            vo = variant_objects(asm, v)
            vis_state[v] = vo
        objs = {}
        for v, vo in vis_state.items():
            for k in ("visible", "removed", "hidden"):
                for ob in vo[k]:
                    objs[ob.name] = ob
        for name, ob in objs.items():
            hide = np.zeros(n)
            op = np.ones(n)
            for i, v in enumerate(names):
                vo = vis_state[v]
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
    else:
        names = [asm.meta["variants"][0]] * n
    # ---- gas set multipliers ---------------------------------------------------------
    gsets = asm.meta.get("gas_sets", {})
    gmul = {}
    user = pres.get("gas_sets")
    for s in gsets:
        if user is not None and s in user:
            gmul[s] = _per_frame(user[s], n, 0.0)
        else:
            gmul[s] = np.array([1.0 if _gas_set_for(asm, v) == s else 0.0 for v in names])
    g_all = _per_frame(pres.get("gas"), n, 1.0)
    s_all = _per_frame(pres.get("spark"), n, 1.0)
    # ---- movers ---------------------------------------------------------------------------
    lifts = {}

    def lift(c, kind):
        k = (c, kind)
        if k not in lifts:
            lifts[k] = valve_lift_ft(th, c, kind)
        return lifts[k]
    mix = {}
    for m in asm.meta["_movers"]:
        ob, t = m["obj"], m["type"]
        if ob.name not in bpy.data.objects:
            continue
        if t == "spin":
            ob.rotation_mode = "XYZ"
            rig.bake_spin(ob, fr, m["k"] * th + m["phase"])
        elif t == "piston":
            rig.bake_channel(ob, "location", 2, fr, kin.piston_height(th, m["cyl"]))
        elif t == "rod":
            _, _, cx, cz, rt = kin.slider_crank(th, m["cyl"])
            rig.bake_channel(ob, "location", 0, fr, cx)
            rig.bake_channel(ob, "location", 2, fr, cz)
            ob.rotation_mode = "XYZ"
            rig.bake_spin(ob, fr, rt)
        elif t == "valve":
            rig.bake_channel(ob, "location", 1, fr, -lift(m["cyl"], m["kind"]))
        elif t == "spring":
            key = ob.data.shape_keys
            rig.bake_channel(key, 'key_blocks["open"].value', -1, fr, lift(m["cyl"], m["kind"]) / VALVE_LIFT[m["kind"]])
        elif t == "chain":
            rig.bake_channel(ob, "location", 0, fr, np.mod(chain_travel(th), 2 * CHAIN_P))
        elif t == "gas":
            c = m["cyl"]
            if c not in mix:
                mix[c] = dict(zip(GAS_KINDS, kin.gas_mix(th, c)))
                mix[c]["_h"] = (R_THROW + ROD_L - kin.piston_height(th, c)) / S.STROKE
            key = ob.data.shape_keys
            rig.bake_channel(key, 'key_blocks["stroke"].value', -1, fr, np.clip(mix[c]["_h"], 0.0, 1.0))
            op = GAS_ALPHA[m["kind"]] * np.asarray(mix[c][m["kind"]]) * g_all * gmul.get(m["set"], 0.0)
            op = np.clip(op, 0.0, 1.0)
            rig.bake_prop(ob, "cv_opacity", fr, op)
            rig.bake_visibility(ob, fr, op)
        elif t == "spark":
            op = np.clip(kin.spark(th, m["cyl"]) * s_all, 0.0, 1.0)
            rig.bake_prop(ob, "cv_opacity", fr, op)
            rig.bake_visibility(ob, fr, op)


# ===========================================================================
# Front accessory drive (poly-V belt: crank damper -> water pump -> idler -> alternator)
# ===========================================================================
Y_BELT = 0.5 * (Y_DAMPER[0] + Y_DAMPER[1])
BELT_E = 0.0022                    # belt pitch line above the pulley outer radius
ACC = dict(damper=((0.0, 0.0), 0.0765), wp=((0.052, 0.148), 0.050), idler=((-0.066, 0.170), 0.033),
           alt=((-0.194, 0.068), 0.031))


def accessory_ratio(name):
    """Spin factor of an accessory pulley per unit crank angle (belt drive)."""
    rd = ACC["damper"][1] + BELT_E
    r = ACC[name][1] + BELT_E
    return (-rd / r) if name == "idler" else (rd / r)


def _ribbed_pulley(key, r, w, hub_r, ribs=6, web_y=None, material="steel_machined"):
    """Poly-V pulley lathe profile centred at y=0 (width w)."""
    y0, y1 = -w / 2, w / 2
    prof = [(hub_r, y0), (r - 0.004, y0), (r - 0.0005, y0), (r, y0 + 0.002)]
    pitch = (w - 0.004) / ribs
    for k in range(ribs):
        prof += [(r - 0.002, y0 + 0.002 + (k + 0.5) * pitch), (r, y0 + 0.002 + (k + 1) * pitch)]
    prof += [(r - 0.0005, y1), (r - 0.004, y1), (r - 0.004, y0 + 0.004), (hub_r + 0.002, y0 + 0.004),
             (hub_r + 0.002, y1), (hub_r, y1)]
    return MU.lathe(_nm(key), prof, _seg(64), closed=True, caps=False, collection=_COL, material=material)


def _build_accessories():
    out = {}
    yb = Y_BELT
    # belt path (same signed-circle loop maths as the chain)
    circles = [(np.array(ACC["damper"][0]), ACC["damper"][1] + BELT_E),
               (np.array(ACC["wp"][0]), ACC["wp"][1] + BELT_E),
               (np.array(ACC["idler"][0]), -(ACC["idler"][1] + BELT_E)),
               (np.array(ACC["alt"][0]), ACC["alt"][1] + BELT_E)]
    segs, L = _loop(circles)
    n = int(L / 0.002)
    Q = np.array([_loop_point(segs, L * i / n) for i in range(n)])
    t = np.roll(Q, -1, 0) - np.roll(Q, 1, 0)
    t /= np.linalg.norm(t, axis=1)[:, None]
    nl = np.stack([-t[:, 1], t[:, 0]], 1)            # left = toward the loop inside
    mb = MU.MeshBuilder()
    sec = [(-0.0020, -0.0105), (-0.0020, 0.0105), (0.0020, 0.0105), (0.0020, -0.0105)]   # (normal, across)
    rings = []
    for (dn, dy) in sec:
        P = Q + nl * dn
        rings.append(mb.verts(np.stack([P[:, 0], np.full(n, yb + dy), P[:, 1]], 1)))
    for k in range(4):
        mb.bridge(rings[k], rings[(k + 1) % 4])
    out["accessory_belt"] = mb.to_object(_nm("accessory_belt"), _COL, smooth_angle=40.0, materials=["rubber"])
    # water pump: body on the timing cover + pulley
    (wx, wz), wr = ACC["wp"]
    body = _cyl_along("wp_body", (wx, Y_COVER_FRONT - 0.002, wz), (wx, Y_COVER_FRONT + 0.0025, wz), 0.042,
                      seg=_seg(48), material="cast_aluminium")
    snout = _cyl_along("wp_snout", (wx, Y_COVER_FRONT + 0.002, wz), (wx, yb + 0.0075, wz), 0.017, seg=_seg(32),
                       material="cast_aluminium")
    bolts = [MU.extrude_polygon(_nm(f"wp_b{k}"), _circle((wx + 0.034 * math.cos(a), wz + 0.034 * math.sin(a)), 0.0048, 6),
                                Y_COVER_FRONT + 0.0025, Y_COVER_FRONT + 0.0053, chamfer=0.0004, collection=_COL,
                                material="steel_dark") for k, a in enumerate(np.linspace(0.3, TAU + 0.3, 5)[:-1])]
    out["water_pump"] = _join("water_pump", [body, snout] + bolts)
    wpp = _ribbed_pulley("wp_pulley", wr, 0.024, 0.0185)
    disc = MU.cylinder(_nm("wp_disc"), wr - 0.004, 0.0085, 0.0115, segments=_seg(64), collection=_COL,
                       material="steel_machined")
    hubb = [MU.extrude_polygon(_nm(f"wp_pb{k}"), _circle((0.017 * math.cos(a), 0.017 * math.sin(a)), 0.0045, 6),
                               0.0115, 0.0145, chamfer=0.0003, collection=_COL, material="steel_dark")
            for k, a in enumerate(np.linspace(0, TAU, 5)[:-1])]
    hub = MU.cylinder(_nm("wp_hub"), 0.0110, 0.0115, 0.0135, segments=_seg(32), chamfer=0.0005, collection=_COL,
                      material="steel_machined")
    wp = _join("wp_pulley", [wpp, disc, hub] + hubb)
    _translate(wp, (0.0, yb, 0.0))
    out["wp_pulley"] = wp
    # idler on a tensioner arm pivoting on the timing cover
    (ix, iz), ir = ACC["idler"]
    idl = MU.lathe(_nm("belt_idler"), [(0.006, -0.013), (ir - 0.001, -0.013), (ir, -0.0115), (ir, 0.0115),
                                       (ir - 0.001, 0.013), (0.006, 0.013)], _seg(56), closed=True, caps=False,
                   collection=_COL, material="steel_machined")
    cap_ = MU.cylinder(_nm("idl_cap"), 0.013, 0.013, 0.016, segments=_seg(32), chamfer=0.001, collection=_COL,
                       material="plastic_black")
    idl = _join("belt_idler", [idl, cap_])
    _translate(idl, (0.0, yb, 0.0))
    out["belt_idler"] = idl
    piv = np.array([-0.020, 0.212])
    arm_pts = np.array([[ix, iz], piv])
    d = arm_pts[1] - arm_pts[0]
    d /= np.linalg.norm(d)
    nrm = np.array([-d[1], d[0]])
    poly = np.array([arm_pts[0] + 0.012 * nrm, arm_pts[1] + 0.010 * nrm, arm_pts[1] - 0.010 * nrm,
                     arm_pts[0] - 0.012 * nrm])
    arm = MU.extrude_polygon(_nm("idl_arm"), poly, Y_COVER_FRONT + 0.0005, yb - 0.0135, chamfer=0.0015, collection=_COL,
                             material="cast_aluminium")
    pv = _cyl_along("idl_piv", (piv[0], Y_COVER_FRONT, piv[1]), (piv[0], yb - 0.010, piv[1]), 0.012, seg=_seg(32),
                    material="cast_aluminium")
    ax = _cyl_along("idl_axle", (ix, yb - 0.0136, iz), (ix, yb + 0.0127, iz), 0.0059, seg=_seg(16),
                    material="steel_machined")
    out["idler_arm"] = _join("idler_arm", [arm, pv, ax])
    # alternator
    (ax_, az_), ar = ACC["alt"]
    ap = _ribbed_pulley("alt_pulley", ar, 0.024, 0.0085)
    nut = MU.extrude_polygon(_nm("alt_nut"), _circle((0, 0), 0.010, 6), 0.012, 0.019, chamfer=0.0007,
                             collection=_COL, material="steel_dark")
    web = MU.cylinder(_nm("alt_web"), ar - 0.004, -0.004, 0.004, segments=_seg(48), inner_radius=0.0085,
                      collection=_COL, material="steel_machined")
    fan = MU.cylinder(_nm("alt_fan"), 0.050, -0.032, -0.0145, segments=_seg(48), inner_radius=0.014,
                      collection=_COL, material="steel_dark")
    fcut = []
    for k in range(14):
        a = TAU * k / 14
        q = np.array([math.cos(a), math.sin(a)])
        poly = np.array([0.022 * q + 0.0035 * np.array([-q[1], q[0]]), 0.052 * q + 0.0055 * np.array([-q[1], q[0]]),
                         0.052 * q - 0.0055 * np.array([-q[1], q[0]]), 0.022 * q - 0.0035 * np.array([-q[1], q[0]])])
        fcut.append(MU.extrude_polygon(_nm(f"alt_fc{k}"), poly, -0.030, -0.0165, collection=_COL))
    _bool(fan, fcut)
    shaft = MU.cylinder(_nm("alt_shaft"), 0.0085, -0.034, 0.0125, segments=_seg(24), collection=_COL,
                        material="steel_machined")
    alt_rot = _join("alt_pulley", [ap, nut, web, fan, shaft])
    _translate(alt_rot, (0.0, yb, 0.0))
    out["alt_pulley"] = alt_rot
    fr = MU.lathe(_nm("alt_front"), [(0.0, yb - 0.110), (0.058, yb - 0.110), (0.061, yb - 0.104), (0.061, yb - 0.050),
                                     (0.054, yb - 0.040), (0.016, yb - 0.036), (0.0, yb - 0.036)], _seg(64),
                  collection=_COL, material="cast_aluminium")
    slots = []
    for k in range(10):
        a = TAU * (k + 0.5) / 10
        slots.append(_cyl_along(f"alt_sl{k}", (0.050 * math.cos(a), yb - 0.060, 0.050 * math.sin(a)),
                                (0.066 * math.cos(a), yb - 0.060, 0.066 * math.sin(a)), 0.0055, seg=12))
    _bool(fr, slots)
    rear = MU.lathe(_nm("alt_rear"), [(0.0, yb - 0.128), (0.050, yb - 0.128), (0.056, yb - 0.120),
                                      (0.056, yb - 0.109), (0.0, yb - 0.109)], _seg(64), collection=_COL,
                    material="plastic_black")
    ear_a = _box("alt_ear0", 0.040, 0.080, yb - 0.105, yb - 0.090, -0.010, 0.010, r=0.003, material="cast_aluminium")
    ear_b = _box("alt_ear1", 0.040, 0.080, yb - 0.075, yb - 0.060, -0.010, 0.010, r=0.003, material="cast_aluminium")
    for ob in (fr, rear, ear_a, ear_b):
        _translate(ob, (ax_, 0.0, az_))
    brk = _box("alt_bracket", ax_ + 0.060, -0.0905, yb - 0.106, yb - 0.056, az_ - 0.012, az_ + 0.006, r=0.003,
               material="paint_black")
    out["alternator"] = _join("alternator", [fr, rear, ear_a, ear_b, brk])
    return out
