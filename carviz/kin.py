"""Shared kinematics: every derived angle / position in the car.

Pure Python + numpy (no bpy) so it can be unit-tested.  The drivetrain state
(`carviz.state`) produces the fundamental scalars; assemblies call the
functions here to turn them into the angle/position of every part.  Angles
follow ARCHITECTURE.md section 2: a part spinning about its local +Y gets
rotation_euler[1] = -abs_angle, and its local profile angle psi (from +X
toward +Z) advances with abs_angle.  "abs" angles below already include the
mesh phase of that part (tooth 0 of the mesh is on local +X at abs = 0).

Functions accept scalars or numpy arrays.
"""
from __future__ import annotations

import math

import numpy as np

from . import spec as S

TAU = 2.0 * math.pi
PI = math.pi


def wrap_pm_pi(a):
    return (np.asarray(a) + PI) % TAU - PI


def smoothstep(e0, e1, x):
    t = np.clip((np.asarray(x, dtype=float) - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


# ===========================================================================
# Generic gear meshing (planar)
# ===========================================================================

def mesh_phase(phi1, z1, z2, delta):
    """Abs phase of gear 2 so it meshes with gear 1.

    gear 1 (z1 teeth) has abs phase phi1; the direction (psi) from gear 1's
    centre to gear 2's centre is `delta`.  Teeth are at psi = phi + 2pi k/z.
    Condition: u1 + u2 = 1/2 (mod 1) with u_i = (dir_i - phi_i) * z_i / 2pi,
    which is invariant while the gears roll (gear 2 turns by -z1/z2 per unit).
    """
    u1 = (delta - phi1) * z1 / TAU
    return delta + PI - (TAU / z2) * (0.5 - u1)


def driven_angle(phi2_0, theta1, z1, z2):
    """Gear 2 abs angle when gear 1 has turned by theta1 from its phase."""
    return phi2_0 - theta1 * z1 / z2


# ===========================================================================
# Engine
# ===========================================================================

def crankpin_psi(theta, cyl):
    """Profile angle of the crankpin of `cyl` (psi=+90deg is straight up = TDC)."""
    return PI / 2 + theta + math.radians(S.CRANKPIN_PHASE_DEG[cyl])


def slider_crank(theta, cyl):
    """Returns (pin_z, pin_x, crankpin_x, crankpin_z, rod_theta).

    Heights relative to the crank axis.  rod_theta is the scalar spin to give
    a con-rod modelled vertically (big end at origin, small end at +Z local),
    i.e. rotation_euler[1] = -rod_theta.
    """
    r, L = S.CRANK_THROW, S.CONROD_LENGTH
    psi = crankpin_psi(theta, cyl)
    cx = r * np.cos(psi)
    cz = r * np.sin(psi)
    pin_z = cz + np.sqrt(L * L - cx * cx)
    rod_theta = np.arcsin(cx / L)          # rotation_euler[1] = -asin(cx/L)
    return pin_z, 0.0 * pin_z, cx, cz, rod_theta


def piston_height(theta, cyl):
    """Gudgeon-pin centre height above the crank axis (m)."""
    return slider_crank(theta, cyl)[0]


def cycle_angle_deg(theta, cyl):
    """Angle in cylinder `cyl`'s own 720-deg cycle (0 = TDC firing)."""
    return (np.degrees(theta) - S.FIRING_TDC_DEG[cyl]) % 720.0


def stroke_of(theta, cyl):
    """(index 0..3 in INTAKE,COMPRESSION,POWER,EXHAUST order, name, progress 0..1)."""
    phi = float(cycle_angle_deg(theta, cyl))
    order = {"intake": 0, "compression": 1, "power": 2, "exhaust": 3}
    for name, a0, a1 in S.STROKES:
        if a0 <= phi < a1:
            return order[name], name, (phi - a0) / (a1 - a0)
    return 2, "power", 0.0


def _lift(phi, open_deg, close_deg, lmax):
    phi = np.asarray(phi, dtype=float)
    span = close_deg - open_deg
    u = (phi - open_deg) / span
    inside = (u > 0) & (u < 1)
    return np.where(inside, lmax * np.sin(PI * np.clip(u, 0, 1)) ** 2, 0.0)


def valve_lift(theta, cyl, kind):
    """Valve lift (m) for kind 'intake'|'exhaust' at crank angle theta."""
    phi = cycle_angle_deg(theta, cyl)
    if kind == "intake":
        return _lift(phi, S.IVO_DEG, S.IVC_DEG, S.VALVE_LIFT_INTAKE)
    return _lift(phi, S.EVO_DEG, S.EVC_DEG, S.VALVE_LIFT_EXHAUST)


def valve_peak_cycle_deg(kind):
    return (S.IVO_DEG + S.IVC_DEG) / 2 if kind == "intake" else (S.EVO_DEG + S.EVC_DEG) / 2


def cam_angle(theta_crank):
    """Camshaft scalar angle: half crank speed, same direction (chain drive)."""
    return theta_crank * S.CAM_SPEED_RATIO


def cam_lobe_psi(cyl, kind, valve_dir_psi=-PI / 2):
    """Local profile angle of the lobe NOSE for (cyl, kind) on its camshaft.

    At peak lift the nose must point from the cam centre toward the valve
    (direction valve_dir_psi; default straight down).  With cam abs angle =
    cam_angle(theta) (+ this local nose angle), the nose direction is
    lobe_psi + cam_angle(theta).
    """
    theta_peak = math.radians(S.FIRING_TDC_DEG[cyl] + valve_peak_cycle_deg(kind))
    return valve_dir_psi - cam_angle(theta_peak)


def cam_profile(kind, base_radius, n=180):
    """Polar lobe profile consistent with valve_lift: list of (angle_from_nose, r).

    angle measured in cam degrees; crank offset from peak = 2 * cam offset.
    (Flat-tappet geometry ignored: radial lift = valve lift.)
    """
    peak = valve_peak_cycle_deg(kind)
    out = []
    for i in range(n):
        a = -PI + TAU * i / n
        phi = peak + math.degrees(-2 * a)
        lift = float(_lift(np.array(phi), *((S.IVO_DEG, S.IVC_DEG, S.VALVE_LIFT_INTAKE) if kind == "intake"
                                           else (S.EVO_DEG, S.EVC_DEG, S.VALVE_LIFT_EXHAUST))))
        out.append((a, base_radius + lift))
    return out


def sprocket_pitch_radius(z, pitch=S.CHAIN_PITCH):
    return pitch / (2.0 * math.sin(PI / z))


def chain_travel(theta_crank):
    """Arc length (m) the timing chain has moved for a crank angle."""
    return theta_crank * sprocket_pitch_radius(S.CRANK_SPROCKET_TEETH)


def spark(theta, cyl, width_deg=8.0):
    """Spark intensity 0..1, a short pulse at SPARK_ADVANCE before TDC firing."""
    phi = cycle_angle_deg(theta, cyl)
    d = (phi - (720.0 - S.SPARK_ADVANCE_DEG) + 360.0) % 720.0 - 360.0
    return np.clip(1.0 - np.abs(d) / width_deg, 0.0, 1.0)


def combustion(theta, cyl):
    """Burning-gas glow 0..1: rises from the spark to ~12 deg ATDC, decays to EVO."""
    phi = np.asarray(cycle_angle_deg(theta, cyl), dtype=float)
    pre = (phi >= 720.0 - S.SPARK_ADVANCE_DEG)
    rise = np.where(pre, smoothstep(720.0 - S.SPARK_ADVANCE_DEG, 720.0, phi) * 0.7, 0.0)
    post = (phi < S.EVO_DEG + 30)
    early = smoothstep(0.0, 12.0, phi) * 0.3 + 0.7
    decay = np.exp(-np.clip(phi - 12.0, 0, None) / 45.0)
    val = np.where(post, np.where(phi < 12.0, early, decay), 0.0)
    return np.where(pre, rise, val)


def gas_mix(theta, cyl):
    """Weights (intake, compressed, burning, exhaust) for the cylinder gas volume.

    The gas colour follows the stroke: fresh charge during intake, denser
    during compression, burning during power, exhaust after EVO.
    """
    phi = np.asarray(cycle_angle_deg(theta, cyl), dtype=float)
    burn = combustion(theta, cyl)
    w_int = np.where((phi >= 360) & (phi < 540), 1.0, 0.0)
    comp_t = np.where((phi >= 540), smoothstep(540, 700, phi), 0.0)
    w_comp = np.where(phi >= 540, comp_t, 0.0)
    w_int = w_int + np.where(phi >= 540, 1.0 - comp_t, 0.0)
    w_exh = np.where((phi >= S.EVO_DEG) & (phi < 360), 1.0, 0.0)
    # burning overrides
    w_burn = burn
    rest = 1.0 - w_burn
    return w_int * rest, w_comp * rest, w_burn, w_exh * rest


# ===========================================================================
# Clutch & hydraulics
# ===========================================================================

def clutch_capacity(pedal):
    """Fraction of full clamp torque capacity for pedal position 0 (up)..1 (floor)."""
    return 1.0 - smoothstep(S.CLUTCH_BITE_LO, S.CLUTCH_BITE_HI, pedal)


def clutch_geometry(pedal, pedal_arm=0.30):
    """All clutch release positions for a pedal position (0 up .. 1 floor).

    Returns dict (metres / radians):
      pedal_angle   - pedal arm rotation (pad travel along an arc of pedal_arm)
      master        - master-cylinder pushrod travel
      slave         - slave-cylinder pushrod travel (volume conservation)
      bearing       - release bearing travel toward the flywheel (-> +Y)
      finger        - diaphragm finger-tip deflection (= bearing travel)
      plate_lift    - pressure plate retraction (away from flywheel, -Y)
      capacity      - torque capacity fraction
    """
    p = np.clip(np.asarray(pedal, dtype=float), 0.0, 1.0)
    pad = p * S.CLUTCH_PEDAL_TRAVEL
    master = pad / S.CLUTCH_PEDAL_RATIO
    slave = master * (S.MASTER_CYL_BORE / S.SLAVE_CYL_BORE) ** 2
    # free play: the first CLUTCH_FREE_PLAY of travel takes up clearances
    eff = np.clip((p - S.CLUTCH_FREE_PLAY) / (1.0 - S.CLUTCH_FREE_PLAY), 0.0, 1.0)
    bearing = eff * S.RELEASE_BEARING_TRAVEL
    # Plate only starts to lift once the clamp load is relieved (bite zone)
    lift = S.PRESSURE_PLATE_LIFT * np.clip((p - S.CLUTCH_BITE_LO) / (1.0 - S.CLUTCH_BITE_LO), 0.0, 1.0)
    return dict(pedal_angle=pad / pedal_arm, master=master, slave=slave, bearing=bearing,
                finger=bearing, plate_lift=lift, capacity=clutch_capacity(p))


# ===========================================================================
# Gearbox (3-shaft).  Geometry in the gearbox XZ plane: main axis at (0,0),
# countershaft directly below at (0, -C).  psi = atan2(z, x).
# ===========================================================================

C = S.GEARBOX_CENTRE_DISTANCE
DIR_MAIN_TO_CS = -PI / 2
DIR_CS_TO_MAIN = PI / 2


def reverse_idler_centre():
    """(x, z) of the reverse idler relative to the main axis (right-hand side)."""
    m = S.REV_MODULE
    a1 = m * (S.Z_REV_CS + S.Z_REV_IDLER) / 2      # countershaft - idler
    a2 = m * (S.Z_REV_IDLER + S.Z_REV_OUT) / 2     # idler - main
    # x^2 + z^2 = a2^2 ; x^2 + (z + C)^2 = a1^2
    z = (a1 * a1 - a2 * a2 - C * C) / (2 * C)
    x = math.sqrt(max(a2 * a2 - z * z, 0.0))
    return (x, z)


def _dir(p_from, p_to):
    return math.atan2(p_to[1] - p_from[1], p_to[0] - p_from[0])


def gearbox_phases():
    """Abs phases (at theta_in = 0) of every toothed gearbox part, plus speed
    factors.  Returns dict name -> (phase, factor, ref) meaning
        abs_angle = phase + factor * theta_ref,   ref in {'in','out'}.
    """
    P = {}
    zi, zc0 = S.Z_INPUT, S.Z_CS_DRIVEN
    k_cs = -zi / zc0                     # countershaft turns per input turn
    # input gear: tooth 0 toward countershaft
    P["input_gear"] = (DIR_MAIN_TO_CS, 1.0, "in")
    P["cs_drive"] = (mesh_phase(DIR_MAIN_TO_CS, zi, zc0, DIR_MAIN_TO_CS), k_cs, "in")
    for g, (zc, zm) in S.GEAR_PAIRS.items():
        cs_phase = DIR_CS_TO_MAIN       # cluster gear: tooth 0 toward main axis
        P[f"cs_{g}"] = (cs_phase, k_cs, "in")
        P[f"gear_{g}"] = (mesh_phase(cs_phase, zc, zm, DIR_CS_TO_MAIN), k_cs * (-zc / zm), "in")
    # reverse train
    ix, iz = reverse_idler_centre()
    cs_c = (0.0, -C)
    d_ci = _dir(cs_c, (ix, iz))
    P["cs_R"] = (d_ci, k_cs, "in")
    idl0 = mesh_phase(d_ci, S.Z_REV_CS, S.Z_REV_IDLER, d_ci)
    k_idl = k_cs * (-S.Z_REV_CS / S.Z_REV_IDLER)
    P["idler"] = (idl0, k_idl, "in")
    d_im = _dir((ix, iz), (0.0, 0.0))
    P["gear_R"] = (mesh_phase(idl0, S.Z_REV_IDLER, S.Z_REV_OUT, d_im), k_idl * (-S.Z_REV_IDLER / S.Z_REV_OUT), "in")
    # output-side parts
    for name in ("output_shaft", "hub_12", "hub_34", "hub_5R", "sleeve_12", "sleeve_34", "sleeve_5R"):
        P[name] = (0.0, 1.0, "out")
    return P


_GB = None


def gb_phase_table():
    global _GB
    if _GB is None:
        _GB = gearbox_phases()
    return _GB


def gb_angle(name, theta_in, theta_out=0.0):
    """Abs angle of a gearbox part given input/output scalar angles."""
    ph, k, ref = gb_phase_table()[name]
    return ph + k * (theta_in if ref == "in" else theta_out)


def gear_speed_factor(g):
    """abs-angle rate of mainshaft gear g per unit input-shaft angle (= 1/ratio)."""
    if g == 4:
        return 1.0
    return gb_phase_table()["input_gear" if g == 4 else f"gear_{g}"][1]


def gear_abs(g, theta_in):
    """Abs angle of the dog-ring carrier for gear g (4 = input gear)."""
    return gb_angle("input_gear" if g == 4 else f"gear_{g}", theta_in)


def dog_misalignment(g, theta_in, theta_out):
    """Signed angle (rad, on the gear) by which gear g's dogs miss alignment
    with the sleeve/hub (0 = aligned).  Wrapped to +-half a dog pitch."""
    pitch = TAU / S.DOG_TEETH
    d = gear_abs(g, theta_in) - theta_out
    return (d + pitch / 2) % pitch - pitch / 2


# ===========================================================================
# Driveline
# ===========================================================================

def hooke(theta_in, beta, phase=0.0):
    """Output angle of a Hooke (Cardan) joint at bend angle beta.

    tan(theta_out) = tan(theta_in) / cos(beta), theta measured from the
    position where the input yoke lies in the plane of the two shafts.
    Continuous (unwrapped) for array input.
    """
    t = np.asarray(theta_in, dtype=float) - phase
    base = np.arctan2(np.sin(t), np.cos(t) * math.cos(beta))
    turns = np.round((t - base) / TAU)
    return base + turns * TAU + phase


def diff_case_angle(theta_left, theta_right):
    return 0.5 * (np.asarray(theta_left) + np.asarray(theta_right))


def spider_spin(theta_left, theta_right):
    """Spider-gear rotation about its pin RELATIVE to the case (magnitude per
    side-gear relative angle; sign to be applied by the assembly's geometry)."""
    rel = np.asarray(theta_left) - diff_case_angle(theta_left, theta_right)
    return rel * S.Z_SIDE_GEAR / S.Z_SPIDER


def wheel_speeds(v, curvature, track=S.TRACK_REAR):
    """Ground speeds of left/right wheels of an axle whose centre moves at v
    along a path of curvature (1/m, + = left turn)."""
    return v * (1.0 - curvature * track / 2), v * (1.0 + curvature * track / 2)


def front_wheel_kinematics(v_rear, curvature, wheelbase=S.WHEELBASE, track=S.TRACK_FRONT):
    """Ackermann: (steer_left, steer_right, v_left, v_right) for the front axle
    when the rear-axle centre moves at v_rear on curvature kappa."""
    k = np.asarray(curvature, dtype=float)
    Lk = wheelbase * k
    dl = np.arctan2(Lk, 1.0 - k * track / 2)
    dr = np.arctan2(Lk, 1.0 + k * track / 2)
    vl = v_rear * np.sqrt(Lk ** 2 + (1.0 - k * track / 2) ** 2)
    vr = v_rear * np.sqrt(Lk ** 2 + (1.0 + k * track / 2) ** 2)
    return dl, dr, vl, vr
