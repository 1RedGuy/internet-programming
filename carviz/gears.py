"""Gear, spline, sprocket and dog-tooth geometry (profile math + mesh building).

Conventions (ARCHITECTURE.md section 2 / 6):

* Every toothed part spins about its LOCAL +Y axis.  Profile angle ``psi`` is
  measured in the local XZ plane from +X toward +Z (psi increases with the
  part's scalar angle theta, ``rotation_euler[1] = -theta``).
* Tooth 0 of every external toothed part is centred on psi = 0 (local +X);
  tooth k at psi = k * 2pi/z.  Internal teeth (hub bores, synchro sleeves) sit
  at psi = (k + 1/2) * 2pi/z so they mate with an external part at phase 0.
* Spur/helical/sprocket/spline parts: mid-face plane at local y = 0.
* Helical gears: the transverse profile is twisted along Y with
      psi_offset(y) = s * y * tan(helix) / r_pitch,   s = -1 for hand='right',
                                                      s = +1 for hand='left'
  ('right' is a geometric right-hand helix, like a normal screw thread:
  psi is right-handed about -Y).  At y = 0 the phase is the nominal one, so a
  mesh phase computed at the mid-plane (``mesh_phase`` / ``kin.mesh_phase``)
  holds on every slice when the mating gear has the opposite hand.
* Bevel gears: the HEEL pitch circle lies in the plane y = 0, the pitch-cone
  APEX is at local (0, bevel_apex_y(...), 0) on +Y, the teeth face +Y and the
  back of the gear is toward -Y.  See ``bevel_gear`` and ``bevel_pair_frames``.

Pure-python/numpy profile functions (no bpy needed):
    involute_profile, profile_polygon, pitch_radius, mesh_phase,
    helix_psi_offset, working_centre_distance, min_profile_shift,
    check_mesh_2d, check_helical_pair, check_bevel_pair, bevel_cone,
    bevel_apex_y, bevel_pair_frames, sprocket_profile, spline_profile
Mesh builders (need bpy):
    spur_gear, helical_gear, ring_gear, bevel_gear, sprocket,
    external_splines, internal_splines, dog_ring, sleeve_internal_teeth

All lengths in metres, angles in radians unless a name ends in ``_deg``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

TAU = 2.0 * math.pi
PI = math.pi
DEG = PI / 180.0

# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------


def _inv(a):
    """Involute function inv(a) = tan a - a (works on arrays)."""
    return np.tan(a) - a


def _resample(pts, n):
    """Resample a polyline (k,2) to n points evenly spaced in arc length."""
    pts = np.asarray(pts, dtype=float)
    if n <= 1 or len(pts) < 2:
        return pts[:max(n, 1)]
    seg = np.hypot(*np.diff(pts, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] <= 0:
        return np.repeat(pts[:1], n, axis=0)
    t = np.linspace(0.0, s[-1], n)
    return np.stack([np.interp(t, s, pts[:, 0]), np.interp(t, s, pts[:, 1])], axis=1)


def _polar(xy):
    xy = np.asarray(xy, dtype=float)
    return np.hypot(xy[:, 0], xy[:, 1]), np.arctan2(xy[:, 1], xy[:, 0])


def _cart(r, a):
    return np.stack([r * np.cos(a), r * np.sin(a)], axis=-1)


def _rot2(xy, ang):
    c, s = math.cos(ang), math.sin(ang)
    xy = np.asarray(xy, dtype=float)
    return np.stack([xy[:, 0] * c - xy[:, 1] * s, xy[:, 0] * s + xy[:, 1] * c], axis=1)


def _hand_sign(hand):
    """'right' -> -1, 'left' -> +1 (sign of d psi / d y for a helical gear)."""
    if isinstance(hand, str):
        h = hand.lower()[0]
        if h == "r":
            return -1.0
        if h == "l":
            return 1.0
        raise ValueError(f"hand must be 'right' or 'left', got {hand!r}")
    return -1.0 if hand > 0 else 1.0


DETAIL = {
    # points per half tooth: root arc, fillet, involute, tip round, tip land
    "low": dict(root=1, fillet=3, inv=5, tip=1, land=1, seg=48, helix_step=6.0 * DEG),
    "medium": dict(root=2, fillet=4, inv=7, tip=2, land=1, seg=64, helix_step=4.0 * DEG),
    "high": dict(root=2, fillet=5, inv=10, tip=2, land=2, seg=96, helix_step=2.5 * DEG),
}


def _detail(detail):
    if isinstance(detail, dict):
        return detail
    return DETAIL[detail or "high"]


# ---------------------------------------------------------------------------
# Basic gear relations
# ---------------------------------------------------------------------------


def transverse(m_n, helix, alpha_n):
    """(m_t, alpha_t) of a helical gear from normal module / pressure angle."""
    m_t = m_n / math.cos(helix)
    alpha_t = math.atan(math.tan(alpha_n) / math.cos(helix))
    return m_t, alpha_t


def pitch_radius(z, m_t):
    """Reference (pitch) radius r = m_t * z / 2."""
    return m_t * z / 2.0


def helix_psi_offset(y, r_pitch, helix, hand):
    """psi offset of the transverse profile at axial position y (see module doc)."""
    return _hand_sign(hand) * np.asarray(y) * math.tan(helix) / r_pitch


def mesh_phase(z1, z2, dir_angle, phi1=None):
    """Phase (abs angle) of gear 2 that meshes with gear 1.

    dir_angle: direction (psi) from gear 1's centre to gear 2's centre, in a
    frame shared by both gears (parallel axes).  If phi1 is None gear 1 is
    assumed to have a tooth centred on dir_angle (phi1 = dir_angle); gear 2
    then has a GAP centred toward gear 1:  phi2 = (dir + pi) - pi/z2.
    General:  phi2 = dir + pi - pi/z2 - (phi1 - dir) * z1/z2
    (identical to carviz.kin.mesh_phase(phi1, z1, z2, dir)).
    """
    if phi1 is None:
        phi1 = dir_angle
    return dir_angle + PI - PI / z2 - (phi1 - dir_angle) * z1 / z2


def min_profile_shift(z, alpha_n=20 * DEG, helix=0.0):
    """Smallest profile shift coefficient x that avoids undercut (ha* = 1)."""
    _, at = transverse(1.0, helix, alpha_n)
    zv = z / math.cos(helix)          # transverse tooth count in normal units
    return max(0.0, 1.0 - zv * math.sin(at) ** 2 / 2.0)


def _inv_inverse(v):
    a = 0.5 if v > 0.05 else (3 * v) ** (1 / 3)
    for _ in range(40):
        f = math.tan(a) - a - v
        a -= f / (math.tan(a) ** 2)
    return a


def working_centre_distance(z1, z2, m_n, helix=0.0, alpha_n=20 * DEG, x1=0.0, x2=0.0):
    """Centre distance for profile-shifted gears with zero (nominal) backlash."""
    m_t, at = transverse(m_n, helix, alpha_n)
    if abs(x1 + x2) < 1e-12:
        return m_t * (z1 + z2) / 2.0
    inv_w = 2 * (x1 + x2) * math.tan(alpha_n) / (z1 + z2) + float(_inv(at))
    aw = _inv_inverse(inv_w)
    return m_t * (z1 + z2) / 2.0 * math.cos(at) / math.cos(aw)


# ---------------------------------------------------------------------------
# Involute tooth (rack-generated, with trochoidal root fillet and undercut)
# ---------------------------------------------------------------------------


@dataclass
class ToothInfo:
    z: float
    m_t: float
    m_n: float
    alpha_t: float
    x: float
    r_p: float
    r_b: float
    r_a: float          # tip radius actually used (may be shortened if pointed)
    r_f: float          # root radius
    s_t: float          # transverse tooth thickness at the pitch circle (after thinning)
    undercut: bool
    r_form: float       # lowest radius of the involute (form radius)
    half: np.ndarray    # (k,2) polar (r, angle) half tooth from space centre (-tau/2) to tooth centre (0)


def _half_tooth(z, m_t, alpha_t, m_n=None, x=0.0, ha=None, hf=None, thin=0.0,
                rho=None, tip_round=None, detail="high", min_land=0.25):
    """Half of one tooth, polar points from the space centre (angle -pi/z)
    to the tooth centre (angle 0), ordered by increasing angle.

    Tooth generated by a basic rack (transverse plane): straight flanks at
    alpha_t, tip fillet radius rho, rack addendum hf.  The fillet is the exact
    envelope of the rack's tip-fillet circle (trochoid offset); undercut is
    handled by intersecting that envelope with the involute.
    """
    d = _detail(detail)
    m_n = m_t if m_n is None else m_n
    ha = (1.0 + x) * m_n if ha is None else ha
    hf = (1.25 - x) * m_n if hf is None else hf
    rho = 0.38 * m_n if rho is None else rho
    tip_round = 0.12 * m_n if tip_round is None else tip_round
    tau = TAU / z
    r_p = m_t * z / 2.0
    ca, sa, ta = math.cos(alpha_t), math.sin(alpha_t), math.tan(alpha_t)
    r_b = r_p * ca
    r_f = r_p - hf
    s_t = PI * m_t / 2.0 + 2.0 * x * m_n * ta - thin
    inv_at = float(_inv(alpha_t))

    def psi(r):
        r = np.maximum(np.asarray(r, dtype=float), r_b)
        return s_t / (2 * r_p) + inv_at - _inv(np.arccos(r_b / r))

    # --- tip radius (shorten if the tooth would be pointed)
    r_a = r_p + ha
    if 2 * psi(r_a) * r_a < min_land * m_n:
        lo, hi = max(r_b, r_p), r_a
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if 2 * psi(mid) * mid < min_land * m_n:
                hi = mid
            else:
                lo = mid
        r_a = lo

    # --- rack cutter (rack coords: n = depth below its reference line toward
    # the gear centre, s = along the pitch line)
    h_c = r_p + x * m_n - r_f              # cutter tip depth below reference line
    w0 = PI * m_t / 4.0 + thin / 2.0       # cutter half tooth width at reference line
    rho_max = (w0 - h_c * ta) * ca / (1.0 - sa)
    rho = max(1e-6 * m_n, min(rho, 0.999 * rho_max))
    n_c = h_c - rho
    s_c = max(0.0, w0 - n_c * ta - rho / ca)
    a_c = r_p + x * m_n - n_c              # fillet-centre distance from gear centre at alignment
    if a_c >= r_p:
        raise ValueError("profile shift too large for the rack fillet construction (x < ~0.85)")
    phi0 = -s_c / r_p
    n_t = n_c + rho * sa                   # flank/fillet tangent point depth
    lam_t = (n_t - x * m_n) / sa           # its position on the line of action
    phi_e = (lam_t / ca - w0 + x * m_n * ta) / r_p
    undercut = lam_t > r_p * sa * (1 + 1e-9)

    def envelope(phi):
        cx = np.full_like(phi, a_c)
        cy = s_c + r_p * phi
        dx, dy = cx - r_p, cy
        L = np.hypot(dx, dy)
        ex = cx + rho * dx / L
        ey = cy + rho * dy / L
        c, s = np.cos(phi), np.sin(phi)
        gx = ex * c + ey * s
        gy = -ex * s + ey * c
        return np.stack([gx, gy], axis=1)   # gear frame, space centred on angle 0

    phis = np.linspace(phi0, phi_e, 241)
    fil = envelope(phis)                     # (space frame) the +angle flank's fillet

    def inv_pts(r):
        return _cart(r, tau / 2 - psi(r))    # involute of the tooth at +tau/2, space frame

    if not undercut:
        r_form = math.hypot(r_p - lam_t * sa, lam_t * ca)
    else:
        # first crossing (from the root) of the fillet envelope with the involute
        r_form = None
        rr = np.linspace(r_b, r_a, 400)
        ip = inv_pts(rr)
        fr, fa = _polar(fil)
        ia_f = np.interp(fr, rr, tau / 2 - psi(rr), left=np.nan, right=np.nan)
        # fillet removes more material where its angle exceeds the involute's
        diff = fa - ia_f        # >0: fillet beyond the involute (undercut zone)
        idx = None
        for i in range(1, len(diff)):
            if np.isfinite(diff[i - 1]) and np.isfinite(diff[i]) and diff[i - 1] > 0 >= diff[i]:
                idx = i
                break
        if idx is None:
            # no clean crossing: join where the two curves are closest
            dd = np.min(np.hypot(fil[:, None, 0] - ip[None, :, 0], fil[:, None, 1] - ip[None, :, 1]), axis=1)
            idx = int(np.argmin(dd)) + 1
            r_form = float(fr[idx - 1])
        else:
            t = diff[idx - 1] / (diff[idx - 1] - diff[idx])
            r_form = float(fr[idx - 1] + t * (fr[idx] - fr[idx - 1]))
        fil = np.vstack([fil[:idx], inv_pts(np.array([r_form]))])
        del ip

    # resample the fillet evenly in arc length
    fil_r = _resample(fil, d["fillet"] + 1)

    # --- involute from r_form up to the start of the tip round
    tr = min(tip_round, 0.45 * (r_a - r_form))
    r_tr0 = r_a - tr
    eb0 = math.sqrt(max((r_form / r_b) ** 2 - 1, 0.0))
    eb1 = math.sqrt(max((r_tr0 / r_b) ** 2 - 1, 0.0))
    eps = np.linspace(eb0, eb1, d["inv"] + 1)
    r_inv = r_b * np.sqrt(1 + eps ** 2)
    inv_r = inv_pts(r_inv)

    # --- tip round: quadratic Bezier from flank (r_a - tr) to tip circle via the corner
    corner = inv_pts(np.array([r_a]))[0]
    ang_corner = math.atan2(corner[1], corner[0])
    ang_g = min(ang_corner + tr / r_a, tau / 2 - 1e-9)
    g = np.array([r_a * math.cos(ang_g), r_a * math.sin(ang_g)])
    f = inv_r[-1]
    tt = np.linspace(0, 1, d["tip"] + 2)[1:]
    bez = ((1 - tt) ** 2)[:, None] * f + (2 * (1 - tt) * tt)[:, None] * corner + (tt ** 2)[:, None] * g

    # --- assemble in the space frame (angles from 0 = space centre)
    pts = []
    a_root = s_c / r_p
    n_root = d["root"]
    for i in range(n_root):
        pts.append((r_f, a_root * i / n_root))
    fr, fa = _polar(fil_r)
    pts += list(zip(fr, fa))
    ir, ia = _polar(inv_r[1:])
    pts += list(zip(ir, ia))
    br, ba = _polar(bez)
    # make the tip-round radii never exceed r_a (Bezier stays inside the corner)
    pts += list(zip(np.minimum(br, r_a), ba))
    # tip land up to the tooth centre (space-frame angle tau/2)
    a_last = pts[-1][1]
    n_land = d["land"]
    for i in range(1, n_land + 1):
        pts.append((r_a, a_last + (tau / 2 - a_last) * i / n_land))
    half = np.array(pts, dtype=float)
    half[:, 1] -= tau / 2.0               # tooth frame: tooth centred at 0
    half[0, 1] = -tau / 2.0
    half[-1, 1] = 0.0
    # enforce monotone non-decreasing angle in the tip region (numerical safety)
    return ToothInfo(z=z, m_t=m_t, m_n=m_n, alpha_t=alpha_t, x=x, r_p=r_p, r_b=r_b, r_a=r_a,
                     r_f=r_f, s_t=s_t, undercut=undercut, r_form=r_form, half=half)


def _full_outline_polar(half, z, n_teeth=None, start=0, ang_scale=1.0):
    """Replicate a half tooth (polar, from -tau/2 to 0) into a gear outline.

    Returns (r, ang) arrays of the closed outline (no repeated end point),
    ordered by increasing angle, tooth k centred at k*tau.  ``ang_scale``
    maps profile angles to psi (bevel virtual gear -> real gear).
    """
    r_h, a_h = half[:, 0], half[:, 1]
    # one tooth: half (space centre .. tooth centre) + mirror (tooth centre .. next space centre, excl.)
    r_t = np.concatenate([r_h, r_h[-2:0:-1]])
    a_t = np.concatenate([a_h, -a_h[-2:0:-1]]) * ang_scale
    tau = TAU / z
    teeth = range(start, start + (int(round(z)) if n_teeth is None else n_teeth))
    r = np.concatenate([r_t for _ in teeth])
    a = np.concatenate([a_t + k * tau for k in teeth])
    return r, a


def involute_profile(z, m_t, alpha_t, x_shift=0.0, addendum=None, dedendum=None, backlash=None,
                     tip_round=None, root_fillet=None, points_per_flank=None, m_n=None,
                     detail="high", n_teeth=None, start_tooth=0, info=False):
    """Closed 2D outline [(x, z), ...] of an external involute gear.

    z: tooth count, m_t / alpha_t: transverse module & pressure angle,
    x_shift: profile shift coefficient (radial shift x*m_n),
    addendum/dedendum: default (1+x)*m_n / (1.25-x)*m_n,
    backlash: TOTAL circular backlash of a meshing pair at the pitch circle
      (transverse); this gear is thinned by backlash/2.  Default 0.05*m_n.
    tip_round: tip corner radius (default 0.12*m_n); root_fillet: rack tip
      radius (default 0.38*m_n, limited by geometry).
    points_per_flank: override the number of involute points per flank.
    n_teeth/start_tooth: only emit a run of teeth (partial outline, open).
    info=True also returns the ToothInfo.
    Tooth 0 centred on +X, CCW (increasing psi) order, no repeated end point.
    """
    m_n = m_t if m_n is None else m_n
    backlash = 0.05 * m_n if backlash is None else backlash
    det = dict(_detail(detail))
    if points_per_flank:
        det["inv"] = int(points_per_flank)
    ti = _half_tooth(z, m_t, alpha_t, m_n=m_n, x=x_shift, ha=addendum, hf=dedendum, thin=backlash / 2.0,
                     rho=root_fillet, tip_round=tip_round, detail=det)
    r, a = _full_outline_polar(ti.half, z, n_teeth=n_teeth, start=start_tooth)
    xy = _cart(r, a)
    return (xy, ti) if info else xy


def profile_polygon(z, m_n, helix=0.0, pressure_angle=20 * DEG, x_shift=0.0, **kw):
    """involute_profile() from NORMAL module / pressure angle (helical aware)."""
    m_t, at = transverse(m_n, helix, pressure_angle)
    return involute_profile(z, m_t, at, x_shift=x_shift, m_n=m_n, **kw)


# ---------------------------------------------------------------------------
# Spline / dog-tooth / sprocket profiles
# ---------------------------------------------------------------------------


def spline_profile(n, r_minor, r_major, internal=False, flank_angle=30 * DEG, fill=0.5,
                   clearance=None, pts_flank=3, pts_arc=2, r_pitch=None):
    """Outline of a straight-flank (involute-like) spline, polar half-tooth data.

    External teeth (internal=False) are centred at psi = k*tau and occupy
    r_minor..r_major; flanks are straight lines (in XZ) inclined at
    flank_angle to the tooth centre line, tooth thickness at r_pitch
    (default mid-depth) = fill * pitch.
    Internal teeth (internal=True) are the complement of the external teeth
    of the same (n, r_pitch, flank_angle, fill), shrunk by ``clearance`` on
    every flank, centred at psi = (k+1/2)*tau, and run from r_minor (tips,
    inner) to r_major (roots, outer).

    Returns dict(xy=(N,2) closed outline (CCW in psi), tags=list, d=array)
    where tags[i] in {'gap','corner','tooth'} and d[i] is the distance of a
    tooth point from that tooth's centre plane (0 for gap points), used to
    build roof-shaped (pointed) tooth ends.
    """
    tau = TAU / n
    r_pitch = 0.5 * (r_minor + r_major) if r_pitch is None else r_pitch
    clearance = 0.0 if clearance is None else clearance
    tf = math.tan(flank_angle)
    h_p = math.sin(fill * tau / 2) * r_pitch      # half chordal thickness at r_pitch (ext. tooth)

    def ext_halfwidth(x):
        """Half width (perp. distance to the centre line) of the external tooth at abscissa x."""
        return h_p - (x - r_pitch * math.cos(fill * tau / 2)) * tf

    # For internal teeth the tooth centred at tau/2 sits between external tooth 0
    # (upper flank) and tooth 1 (lower flank).  Its half-width about its own
    # centre line follows from the external flank line offset by clearance.
    def flank_point_ext(r):
        # point on the upper flank of external tooth 0 at radius r: (x, hw(x)) with x^2+hw^2=r^2
        lo, hi = 0.0, r
        for _ in range(50):
            x = 0.5 * (lo + hi)
            if x * x + ext_halfwidth(x) ** 2 > r * r:
                hi = x
            else:
                lo = x
        x = 0.5 * (lo + hi)
        return x, ext_halfwidth(x)

    def ext_half_angle(r):
        x, h = flank_point_ext(r)
        return math.atan2(h, x)

    def int_half_angle(r):
        # angular half width of the internal tooth at radius r: gap of external
        # teeth minus clearance (clearance converted to angle at r)
        return tau / 2 - ext_half_angle(r) - clearance / r

    pts = []
    tags = []
    if not internal:
        r_root, r_tip = r_minor, r_major
        half_ang = ext_half_angle
        centre = 0.0
    else:
        r_root, r_tip = r_major, r_minor
        half_ang = int_half_angle
        centre = tau / 2
    a_root = half_ang(r_root)
    a_tip = half_ang(r_tip)
    if a_tip <= 0.02 * tau:
        raise ValueError("spline tooth becomes pointed: reduce depth or flank angle")
    # one pitch, from centre - tau/2 (gap centre) to centre + tau/2 (exclusive)
    # gap arc (half)
    g0 = centre - tau / 2
    for i in range(pts_arc):
        a = g0 + (centre - a_root - g0) * i / pts_arc
        pts.append((r_root, a)); tags.append("gap")
    pts.append((r_root, centre - a_root)); tags.append("corner")
    rr = np.linspace(r_root, r_tip, pts_flank + 2)[1:-1]
    for r in rr:
        pts.append((r, centre - half_ang(r))); tags.append("tooth")
    pts.append((r_tip, centre - a_tip)); tags.append("tooth")
    pts.append((r_tip, centre)); tags.append("tooth")
    pts.append((r_tip, centre + a_tip)); tags.append("tooth")
    for r in rr[::-1]:
        pts.append((r, centre + half_ang(r))); tags.append("tooth")
    pts.append((r_root, centre + a_root)); tags.append("corner")
    g1 = centre + tau / 2
    for i in range(1, pts_arc):
        a = centre + a_root + (g1 - centre - a_root) * i / pts_arc
        pts.append((r_root, a)); tags.append("gap")
    one = np.array(pts)
    r_all = np.concatenate([one[:, 0]] * n)
    a_all = np.concatenate([one[:, 1] + k * tau for k in range(n)])
    tag_all = tags * n
    # distance to the tooth's own centre plane
    d_one = np.array([abs(r * math.sin(a - centre)) if t != "gap" else 0.0 for (r, a), t in zip(pts, tags)])
    return dict(xy=_cart(r_all, a_all), r=r_all, a=a_all, tags=tag_all, d=np.concatenate([d_one] * n),
                per_tooth=len(pts), centre=centre, tau=tau, r_root=r_root, r_tip=r_tip)


def sprocket_profile(z, pitch=9.525e-3, roller_d=6.35e-3, detail="high"):
    """ANSI-style roller-chain sprocket outline (approximation).

    Rollers seat in circular gaps (radius 0.505*d + 0.0381 mm) centred on the
    pitch circle at psi = (k+1/2)*tau, so tooth 0 is centred on +X.  Tooth
    flanks are arcs centred on the neighbouring seat centre with radius
    (pitch - seat radius) - the envelope of a roller swinging out about its
    neighbour - cut by the tip circle  OD = p*(0.6 + cot(pi/z)).
    Returns (xy outline, info dict).
    """
    nseat = 7 if detail == "high" else 4
    nfl = 5 if detail == "high" else 3
    tau = TAU / z
    r_p = pitch / (2 * math.sin(PI / z))
    R_s = 0.5025 * roller_d + 0.0381e-3
    r_tip = 0.5 * pitch * (0.6 + 1.0 / math.tan(PI / z))
    # seat centres around tooth 0
    Sm = np.array([r_p * math.cos(-tau / 2), r_p * math.sin(-tau / 2)])
    Sp = np.array([r_p * math.cos(tau / 2), r_p * math.sin(tau / 2)])
    u = (Sp - Sm) / np.linalg.norm(Sp - Sm)
    T = Sm + R_s * u                      # seat/flank tangent point (lower flank of tooth 0)
    a_in = math.atan2(-Sm[1], -Sm[0])     # direction from seat centre toward the gear centre
    a_T = math.atan2(u[1], u[0])
    # seat arc from the gap centre (pointing inward) to T (going CCW in angle from a_in?)
    # a_in ~ pi - tau/2, a_T ~ pi/2: sweep downward in angle
    sweep = np.linspace(a_in, a_T if a_T < a_in else a_T + TAU, nseat + 1)
    seat = Sm + R_s * np.stack([np.cos(sweep), np.sin(sweep)], axis=1)
    # flank arc centred on Sp with radius p - R_s, from T outward to the tip circle
    R_f = pitch - R_s
    a0 = math.atan2(T[1] - Sp[1], T[0] - Sp[0])
    # find angle where the arc reaches r_tip (or the tooth centre line z=0)
    def pt(a):
        return Sp + R_f * np.array([math.cos(a), math.sin(a)])
    # the arc goes from T toward smaller z (toward the tooth centre line); search direction
    lo, hi = a0, a0 + (-0.5 * PI)
    # choose direction that increases radius from T
    if np.linalg.norm(pt(a0 - 0.01)) < np.linalg.norm(pt(a0 + 0.01)):
        hi = a0 + 0.5 * PI
    end = hi
    for k in range(200):
        a = a0 + (hi - a0) * (k + 1) / 200
        p = pt(a)
        if np.linalg.norm(p) >= r_tip or p[1] >= 0.0:
            end = a
            break
    # refine
    lo_a, hi_a = a0, end
    for _ in range(50):
        mid = 0.5 * (lo_a + hi_a)
        p = pt(mid)
        if np.linalg.norm(p) >= r_tip or p[1] >= 0.0:
            hi_a = mid
        else:
            lo_a = mid
    flank = np.array([pt(a) for a in np.linspace(a0, hi_a, nfl + 1)])
    tipc = flank[-1]
    a_tipc = math.atan2(tipc[1], tipc[0])
    land = []
    if a_tipc < -1e-6:
        nl = 2 if detail == "high" else 1
        for i in range(1, nl + 1):
            aa = a_tipc * (1 - i / nl)
            land.append((r_tip * math.cos(aa), r_tip * math.sin(aa)))
    half = np.vstack([seat, flank[1:], np.array(land).reshape(-1, 2)])
    # half goes from the gap centre (angle -tau/2 side, inner point) to the tooth centre
    r_h, a_h = _polar(half)
    # make sure the first point (gap bottom) is exactly at -tau/2
    a_h[0] = -tau / 2
    if abs(a_h[-1]) < 1e-9:
        a_h[-1] = 0.0
        half_pol = np.stack([r_h, a_h], axis=1)
    else:
        half_pol = np.vstack([np.stack([r_h, a_h], axis=1), [[r_tip, 0.0]]])
    r, a = _full_outline_polar(half_pol, z)
    return _cart(r, a), dict(r_pitch=r_p, r_tip=r_tip, r_root=r_p - R_s, seat_radius=R_s)


# ---------------------------------------------------------------------------
# Mesh checks (shapely)
# ---------------------------------------------------------------------------


def _shapely():
    import shapely
    from shapely.geometry import Polygon, box
    return shapely, Polygon, box


def check_mesh_2d(profileA, profileB, centre_distance, phaseA=0.0, ratio=None, n_samples=48,
                  zA=None, zB=None, phaseB=None, dir_angle=0.0, window=None, sweep=None):
    """Prove that two meshing 2D gear outlines neither intersect nor separate.

    profileA/profileB: closed outlines (N,2) at phase 0 (tooth 0 on +X).
    Gear A sits at the origin, gear B at centre_distance in direction
    dir_angle.  Gear A is turned to phaseA + delta, gear B to
    phaseB - delta*ratio  (ratio = zA/zB).  phaseB defaults to
    mesh_phase(zA, zB, dir_angle, phaseA)  (needs zA, zB).
    delta sweeps one tooth pitch of A (``sweep`` overrides, radians).

    Returns dict(max_area, min_gap, max_gap, n) - areas in m^2, gaps (shapely
    distance between the outlines near the mesh) in metres.  A correct pair
    has max_area ~ 0 and 0 < min_gap <= max_gap ~ (backlash/2)*cos(alpha).
    """
    shapely, Polygon, box = _shapely()
    if ratio is None:
        ratio = zA / zB
    if phaseB is None:
        phaseB = mesh_phase(zA, zB, dir_angle, phaseA)
    if sweep is None:
        sweep = TAU / zA if zA else TAU / 20
    A = np.asarray(profileA, dtype=float)
    B = np.asarray(profileB, dtype=float)
    rA = float(np.max(np.hypot(A[:, 0], A[:, 1])))
    rB = float(np.max(np.hypot(B[:, 0], B[:, 1])))
    c = np.array([math.cos(dir_angle), math.sin(dir_angle)]) * centre_distance
    # pitch point (approx) and an analysis window around it
    pp = c * (rA / (rA + rB))
    w = window or 0.6 * min(rA, rB, 4 * (rA + rB - centre_distance) + 1e-9) + 1e-9
    win = box(pp[0] - w, pp[1] - w, pp[0] + w, pp[1] + w)
    max_area = 0.0
    gaps = []
    for i in range(n_samples):
        dl = sweep * i / n_samples
        pa = Polygon(_rot2(A, phaseA + dl)).buffer(0)
        pb = Polygon(_rot2(B, phaseB - dl * ratio) + c).buffer(0)
        pa = pa.intersection(win)
        pb = pb.intersection(win)
        inter = pa.intersection(pb).area
        max_area = max(max_area, inter)
        gaps.append(0.0 if inter > 0 else pa.distance(pb))
    return dict(max_area=max_area, min_gap=float(min(gaps)), max_gap=float(max(gaps)), n=n_samples)


def check_helical_pair(zA, zB, m_n, helix, width, pressure_angle=20 * DEG, xA=0.0, xB=0.0,
                       handA="right", n_samples=24, n_slices=5, detail="high", centre_distance=None,
                       phaseA=0.0, dir_angle=0.0, **kw):
    """check_mesh_2d on several transverse slices across the face width of a
    helical (or spur, helix=0) pair with opposite hands.  Returns the worst
    values over all slices plus 'slices'."""
    m_t, at = transverse(m_n, helix, pressure_angle)
    pa = involute_profile(zA, m_t, at, x_shift=xA, m_n=m_n, detail=detail, **kw)
    pb = involute_profile(zB, m_t, at, x_shift=xB, m_n=m_n, detail=detail, **kw)
    a = centre_distance or working_centre_distance(zA, zB, m_n, helix, pressure_angle, xA, xB)
    rpa, rpb = pitch_radius(zA, m_t), pitch_radius(zB, m_t)
    handB = "left" if _hand_sign(handA) < 0 else "right"
    phB0 = mesh_phase(zA, zB, dir_angle, phaseA)
    out = dict(max_area=0.0, min_gap=1e9, max_gap=0.0, slices=n_slices)
    for y in np.linspace(-width / 2, width / 2, n_slices):
        oa = float(helix_psi_offset(y, rpa, helix, handA)) if helix else 0.0
        ob = float(helix_psi_offset(y, rpb, helix, handB)) if helix else 0.0
        r = check_mesh_2d(pa, pb, a, phaseA + oa, zA / zB, n_samples=n_samples,
                          phaseB=phB0 + ob, dir_angle=dir_angle, zA=zA, zB=zB)
        out["max_area"] = max(out["max_area"], r["max_area"])
        out["min_gap"] = min(out["min_gap"], r["min_gap"])
        out["max_gap"] = max(out["max_gap"], r["max_gap"])
    out["centre_distance"] = a
    return out


# ---------------------------------------------------------------------------
# Bevel gears (Tredgold back-cone tooth form)
# ---------------------------------------------------------------------------


def bevel_cone(z, z_mate, shaft_angle=PI / 2):
    """Pitch-cone half angle delta of a bevel gear with z teeth meshing z_mate."""
    return math.atan2(math.sin(shaft_angle), z_mate / z + math.cos(shaft_angle))


def bevel_apex_y(z, z_mate, module_outer, shaft_angle=PI / 2):
    """Local +Y coordinate of the pitch-cone apex (heel pitch circle at y=0)."""
    d = bevel_cone(z, z_mate, shaft_angle)
    return (module_outer * z / 2.0) / math.tan(d)


def bevel_default_shift(z, z_mate, spiral=0.0):
    """Gleason-style long/short addendum: + for the smaller gear, - for the larger."""
    if z == z_mate:
        return 0.0
    small, large = min(z, z_mate), max(z, z_mate)
    k = 0.46 if spiral == 0.0 else 0.54
    x = k * (1.0 - (small / large) ** 2)
    return x if z < z_mate else -x


@dataclass
class BevelGeom:
    z: int
    z_mate: int
    m: float            # outer (heel) module
    delta: float        # pitch cone half angle
    r_o: float          # heel pitch radius
    R_e: float          # outer cone distance
    b: float            # face width (along the cone)
    apex_y: float       # local y of the apex
    z_v: float          # virtual (Tredgold) tooth count
    spiral: float
    hand_sign: float    # +1 right hand (psi grows toward the heel), -1 left
    tooth: ToothInfo    # virtual spur tooth at the heel
    outline_r: np.ndarray   # virtual radius rho of every outline point (heel)
    outline_psi: np.ndarray  # psi of every outline point (heel, before spiral offset)

    @property
    def R_m(self):
        return self.R_e - self.b / 2

    def spiral_psi(self, R):
        if self.spiral == 0.0:
            return np.zeros_like(np.asarray(R, dtype=float))
        return self.hand_sign * math.tan(self.spiral) * np.log(np.asarray(R) / self.R_m) / math.sin(self.delta)

    def heel_points(self, psi_extra=0.0, rho=None, psi=None):
        """3D local points (N,3) of the heel outline (on the back cone)."""
        rho = self.outline_r if rho is None else rho
        psi = (self.outline_psi if psi is None else psi) + psi_extra
        cd, sd = math.cos(self.delta), math.sin(self.delta)
        y = -self.r_o * math.tan(self.delta) + rho * sd
        return np.stack([rho * cd * np.cos(psi), y, rho * cd * np.sin(psi)], axis=1)

    def section(self, R, phase=0.0, rho=None, psi=None):
        """Tooth section at cone distance R (points scaled toward the apex and
        rotated by the spiral offset)."""
        s = R / self.R_e
        P = self.heel_points(phase + float(self.spiral_psi(R)), rho=rho, psi=psi)
        apex = np.array([0.0, self.apex_y, 0.0])
        return apex + s * (P - apex)


def bevel_geom(z, z_mate, module_outer, face_width=None, shaft_angle=PI / 2, spiral=0.0, hand="right",
               pressure_angle=20 * DEG, x_shift=None, backlash=None, detail="high", tip_round=None,
               root_fillet=None):
    delta = bevel_cone(z, z_mate, shaft_angle)
    m = module_outer
    r_o = m * z / 2
    R_e = r_o / math.sin(delta)
    b = min(R_e / 3.0, 10 * m) if face_width is None else face_width
    z_v = z / math.cos(delta)
    x = bevel_default_shift(z, z_mate, spiral) if x_shift is None else x_shift
    backlash = 0.05 * m if backlash is None else backlash
    ti = _half_tooth(z_v, m, pressure_angle, m_n=m, x=x, thin=backlash / 2, rho=root_fillet,
                     tip_round=tip_round, detail=detail)
    r, a = _full_outline_polar(ti.half, z, ang_scale=z_v / z)
    hs = 1.0 if (isinstance(hand, str) and hand.lower()[0] == "r") or (not isinstance(hand, str) and hand > 0) else -1.0
    return BevelGeom(z=z, z_mate=z_mate, m=m, delta=delta, r_o=r_o, R_e=R_e, b=b,
                     apex_y=r_o / math.tan(delta), z_v=z_v, spiral=spiral, hand_sign=hs, tooth=ti,
                     outline_r=r, outline_psi=a)


def bevel_pair_frames(zA, zB, module_outer, shaft_angle=PI / 2):
    """Placement of a meshing bevel pair around a common apex at the origin.

    Gear A: local frame = world axes, origin at (0, -apexA, 0) (axis along
    world Y, teeth facing +Y).  Gear B: axis in the world XY plane at
    shaft_angle from A's axis, on the +X side; local Z = world Z.
    Returns dict(MA, MB) 4x4 numpy matrices (local -> world), the mesh
    direction psi of each gear (dirA = 0, dirB), and 'sense' such that
    psiB_rate = sense * psiA_rate * zA/zB  (sense = -1 with these frames).
    Phase rule (tooth of A centred on the mesh line, gap of B):
        phiB = dirB - pi/zB + sense * (phiA - dirA) * zA/zB
    """
    dA = bevel_cone(zA, zB, shaft_angle)
    aA = bevel_apex_y(zA, zB, module_outer, shaft_angle)
    aB = bevel_apex_y(zB, zA, module_outer, shaft_angle)
    MA = np.eye(4)
    MA[1, 3] = -aA
    dB = np.array([math.sin(shaft_angle), -math.cos(shaft_angle), 0.0])   # apex -> B's back
    yB = -dB
    zB_ = np.array([0.0, 0.0, 1.0])
    xB = np.cross(yB, zB_)
    MB = np.eye(4)
    MB[:3, 0], MB[:3, 1], MB[:3, 2] = xB, yB, zB_
    MB[:3, 3] = aB * dB
    g = np.array([math.sin(dA), -math.cos(dA), 0.0])        # common pitch generatrix (unit)
    q_local = MB[:3, :3].T @ (g - MB[:3, 3])
    dirB = math.atan2(q_local[2], q_local[0])
    # rotation sense from pitch-point velocities (psi is right-handed about -Y_local)
    vA = np.cross(-np.array([0.0, 1.0, 0.0]), g)
    vB = np.cross(-yB, g)
    sense = -1.0 if np.dot(vA, vB) < 0 else 1.0
    return dict(MA=MA, MB=MB, dirA=0.0, dirB=dirB, sense=sense, generatrix=g, deltaA=dA,
                deltaB=bevel_cone(zB, zA, shaft_angle), apexA=aA, apexB=aB)


def bevel_mesh_phase(zA, zB, phiA=0.0, dirA=0.0, dirB=PI, sense=-1.0):
    """Phase of bevel gear B (see bevel_pair_frames) meshing with A at phiA."""
    return dirB - PI / zB + sense * (phiA - dirA) * zA / zB


def check_bevel_pair(zA, zB, module_outer, face_width=None, shaft_angle=PI / 2, spiral=0.0, handA="left",
                     pressure_angle=20 * DEG, xA=None, xB=None, backlash=None, n_samples=24, n_slices=3,
                     detail="high", phaseA=0.0, handB=None, phaseB_offset=0.0):
    """3D interference check of a bevel pair built by bevel_gear().

    Both gears' tooth surfaces are ruled toward the common apex (spiral:
    plus a twist), so each slice at cone distance R is centrally projected
    onto the plane normal to the common pitch generatrix at distance R;
    overlapping projected teeth <=> 3D interpenetration.  Returns the same
    dict as check_mesh_2d (areas in m^2, gaps in m, worst over all slices).
    """
    shapely, Polygon, box = _shapely()
    if handB is None:
        handB = "right" if (isinstance(handA, str) and handA.lower()[0] == "l") else "left"
    gA = bevel_geom(zA, zB, module_outer, face_width, shaft_angle, spiral, handA, pressure_angle, xA, backlash, detail)
    gB = bevel_geom(zB, zA, module_outer, gA.b, shaft_angle, spiral, handB, pressure_angle, xB, backlash, detail)
    F = bevel_pair_frames(zA, zB, module_outer, shaft_angle)
    g = F["generatrix"]
    e1 = np.array([0.0, 0.0, 1.0])
    e2 = np.cross(g, e1)
    phaseB0 = bevel_mesh_phase(zA, zB, phaseA, F["dirA"], F["dirB"], F["sense"]) + phaseB_offset

    def partial(geom, phase, dir_psi, R, M):
        tau = TAU / geom.z
        # teeth near the mesh direction
        k0 = int(round((dir_psi - phase) / tau))
        per = len(geom.outline_r) // geom.z
        idx = np.concatenate([np.arange(per) + ((k0 + k) % geom.z) * per for k in range(-3, 4)])
        rho = geom.outline_r[idx]
        psi = geom.outline_psi[idx] + (np.repeat(np.arange(-3, 4) + k0, per) - np.repeat(((np.arange(-3, 4) + k0) % geom.z), per)) * tau
        # close through the body
        r_in = geom.tooth.r_f - 2.5 * geom.m
        rho = np.concatenate([rho, [r_in, r_in]])
        psi = np.concatenate([psi, [psi[-1], psi[0]]])
        P = geom.section(R, phase, rho=rho, psi=psi)
        W = (M[:3, :3] @ P.T).T + M[:3, 3]
        dist = W @ g
        W = W * (R / dist)[:, None]
        return np.stack([W @ e1, W @ e2], axis=1)

    out = dict(max_area=0.0, min_gap=1e9, max_gap=0.0, n=n_samples, slices=n_slices)
    for R in np.linspace(gA.R_e - gA.b, gA.R_e, n_slices):
        s = R / gA.R_e
        w = 3.0 * gA.m * s
        cpt = np.array([(R * g) @ e1, (R * g) @ e2])
        win = box(cpt[0] - w, cpt[1] - w, cpt[0] + w, cpt[1] + w)
        for i in range(n_samples):
            dl = TAU / zA * i / n_samples
            pa = Polygon(partial(gA, phaseA + dl, F["dirA"], R, F["MA"])).buffer(0).intersection(win)
            pb = Polygon(partial(gB, phaseB0 + F["sense"] * dl * zA / zB, F["dirB"], R, F["MB"])).buffer(0).intersection(win)
            inter = pa.intersection(pb).area
            out["max_area"] = max(out["max_area"], inter)
            gap = 0.0 if inter > 0 else pa.distance(pb)
            out["min_gap"] = min(out["min_gap"], gap)
            out["max_gap"] = max(out["max_gap"], gap)
    out["face_width"] = gA.b
    out["deltaA"], out["deltaB"] = gA.delta, gB.delta
    return out
