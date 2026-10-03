"""Master drivetrain state.

A scene describes the *driver and vehicle inputs* as keyframed curves in
video time (a `Program`).  `Program.run()` integrates the drivetrain physics
and returns a `Track`: per-frame numpy arrays of every derived quantity.  All
mechanical motion in the film is computed from a Track (via carviz.kin).

Inputs (all in VIDEO time, seconds from scene start):
    slowmo        sim-seconds per video-second (1 = real time, 1/150 = 150x slow)
    speed_kmh     road speed of the rear-axle centre (signed; < 0 reversing)
    curvature     1/m of the rear-axle-centre path (+ = left turn)
    pedal         clutch pedal, 0 = up (engaged) .. 1 = floor (released)
    throttle_rpm  engine speed the driver/idle governor aims for when the
                  engine is not locked to the wheels
    engine_on     1/0
    sleeve[k]     synchro sleeve position s in [-1,1] for k in '12','34','5R'
                  (s=+1 forward (+Y) gear, s=-1 rearward gear; see spec)
    lever_plane   -1 (1-2 rail), 0 (3-4 rail, neutral rest), +1 (5-R rail)
    susp[w]       wheel-centre vertical offset (m) for 'FL','FR','RL','RR'

Physics rules (sim time):
  * wheels roll without slip: dtheta = v_wheel * dt / ROLLING_RADIUS, with
    open-diff / Ackermann wheel speeds from the path curvature.
  * output shaft = FINAL_DRIVE * diff case (= mean of rear wheels).
  * a gear is rotationally engaged once its sleeve passes SYNC_THROUGH; then
    input = output / k_g exactly (k_g = 1/ratio from carviz.kin).
  * while a sleeve is between SYNC_CONTACT and the end of its blocking hold,
    the blocker-ring cone brings the input side to synchronous speed (cosine
    blend) and the gear is then indexed so its dog teeth align with the
    sleeve before they meet.
  * clutch: Coulomb friction, torque capacity = kin.clutch_capacity(pedal)
    * T_CLUTCH_MAX.  While slipping it transmits capacity*sign(slip); it
    locks when the slip speed crosses zero and stays locked while the torque
    needed to keep both sides together is within capacity.  In gear the input
    side is tied to the (much heavier) car; in neutral it is the small
    input-side inertia I_INPUT.
  * engine torque = K_GOVERNOR*(throttle target - speed), clamped to
    [T_ENGINE_MIN, T_ENGINE_MAX]; inertia I_ENGINE.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import kin
from . import spec as S
from . import timeline

# Rotating-inertia / torque model (typical 2.0 L petrol, single-mass flywheel)
I_ENGINE = 0.18        # kg m^2  crank + flywheel + clutch cover/pressure plate
I_INPUT = 0.012        # kg m^2  disc + input shaft + countershaft + gears, referred to input
T_CLUTCH_MAX = 350.0   # N m     clamp torque capacity of the clutch (pedal up)
T_ENGINE_MAX = 190.0   # N m     full-throttle torque
T_ENGINE_MIN = -35.0   # N m     closed-throttle friction + pumping
K_GOVERNOR = 4.0       # N m per rad/s: driver/idle control toward throttle_rpm
TAU_DRAG = 2.0         # s       spin-down time constant of a free input shaft (oil drag)
RPM = 60.0 / (2 * math.pi)


# ===========================================================================
# Keyframed curves
# ===========================================================================

class Curve:
    """Scalar keyframed signal.  key(t, v, mode): `mode` is how the curve
    ARRIVES at this key from the previous one: 'ease' (smoothstep), 'linear',
    'step' (jump at t), 'cubic' (monotone C1 through neighbours)."""

    def __init__(self, default=0.0):
        self.default = float(default)
        self.keys = []      # (t, v, mode)

    def key(self, t, v, mode="ease"):
        self.keys.append((float(t), float(v), mode))
        self.keys.sort(key=lambda k: k[0])
        return self

    def keys_(self, pairs, mode="ease"):
        for p in pairs:
            if len(p) == 3:
                self.key(*p)
            else:
                self.key(p[0], p[1], mode)
        return self

    def hold(self, t0, t1, v):
        self.key(t0, v, "step" if not self.keys else "ease")
        self.key(t1, v, "linear")
        return self

    def __call__(self, t):
        t = np.asarray(t, dtype=float)
        if not self.keys:
            return np.full_like(t, self.default)
        ks = self.keys
        T = np.array([k[0] for k in ks])
        V = np.array([k[1] for k in ks])
        out = np.full_like(t, V[0])
        out = np.where(t >= T[-1], V[-1], out)
        # cubic slopes (Fritsch-Carlson)
        m = np.zeros(len(ks))
        if len(ks) > 1:
            d = np.diff(V) / np.maximum(np.diff(T), 1e-12)
            m[0], m[-1] = d[0], d[-1]
            for i in range(1, len(ks) - 1):
                m[i] = 0.0 if d[i - 1] * d[i] <= 0 else 2.0 / (1.0 / d[i - 1] + 1.0 / d[i])
        for i in range(len(ks) - 1):
            t0, v0, _ = ks[i]
            t1, v1, mode = ks[i + 1]
            sel = (t >= t0) & (t < t1)
            if not np.any(sel):
                continue
            h = max(t1 - t0, 1e-12)
            u = (t[sel] - t0) / h
            if mode == "linear":
                y = v0 + (v1 - v0) * u
            elif mode == "step":
                y = np.full_like(u, v0)
            elif mode == "cubic":
                h00 = 2 * u**3 - 3 * u**2 + 1
                h10 = u**3 - 2 * u**2 + u
                h01 = -2 * u**3 + 3 * u**2
                h11 = u**3 - u**2
                y = h00 * v0 + h10 * h * m[i] + h01 * v1 + h11 * h * m[i + 1]
            else:  # ease
                y = v0 + (v1 - v0) * (u * u * (3 - 2 * u))
            out[sel] = y
        return out


# ===========================================================================
# Program
# ===========================================================================

class Program:
    def __init__(self, scene_id=None, duration=None, fps=S.FPS, substeps=8):
        self.scene = timeline.scene(scene_id) if scene_id else None
        self.scene_id = scene_id
        self.duration = float(duration if duration is not None else self.scene.dur)
        self.fps = fps
        self.n_frames = int(round(self.duration * fps))
        self.substeps = substeps
        self.slowmo = Curve(1.0)
        self.speed_kmh = Curve(0.0)
        self.curvature = Curve(0.0)
        self.pedal = Curve(0.0)
        self.throttle_rpm = Curve(S.IDLE_RPM)
        self.engine_on = Curve(1.0)
        self.sleeve = {k: Curve(0.0) for k in S.SYNCHROS}
        self.lever_plane = Curve(0.0)
        self.susp = {w: Curve(0.0) for w in ("FL", "FR", "RL", "RR")}
        self.cuts = []
        self.theta_e0 = 0.0
        self.theta_in0 = 0.0
        self.wheel0 = 0.0
        self.car_pose0 = (0.0, 0.0, 0.0)   # front-axle ground point x, y; heading (rad, CCW from +Y)
        self.w_in0_rpm = None              # initial free input speed (neutral, clutch out)
        self._engine_align = []            # (t, cycle_deg) requests

    # -- timing helpers ----------------------------------------------------
    def t(self, beat_id, frac=0.0):
        b = self.scene.beat(beat_id)
        return b.start + frac * b.dur

    def beat(self, beat_id):
        return self.scene.beat(beat_id)

    # -- shift helpers -----------------------------------------------------
    def select_plane(self, t0, plane, dur=0.3):
        """Move the lever across the gate (only valid in neutral)."""
        cur = float(self.lever_plane(np.array([t0]))[0])
        self.lever_plane.key(t0, cur, "linear" if self.lever_plane.keys else "step")
        self.lever_plane.key(t0 + dur, plane, "ease")
        return t0 + dur

    def engage(self, gear, t0, travel=0.3, hold=0.6, through=0.3, seat=0.15):
        """Neutral -> gear with a synchro event.  Returns the time fully seated.

        Sleeve reaches the blocking position at t0+travel, is HELD there for
        `hold` (synchronising), then passes through the blocker ring and seats.
        """
        k, side = S.GEAR_SYNCHRO[gear]
        c = self.sleeve[k]
        if not c.keys:
            c.key(0.0, 0.0, "step")
        c.key(t0, 0.0, "linear")
        c.key(t0 + travel, side * S.SYNC_BLOCK, "ease")
        c.key(t0 + travel + hold, side * (S.SYNC_BLOCK + 0.005), "linear")
        c.key(t0 + travel + hold + through, side * S.SYNC_THROUGH, "linear")
        c.key(t0 + travel + hold + through + seat, side * 1.0, "ease")
        return t0 + travel + hold + through + seat

    def disengage(self, gear, t0, dur=0.35):
        """Gear -> neutral (sleeve back to centre).  Returns end time."""
        k, side = S.GEAR_SYNCHRO[gear]
        c = self.sleeve[k]
        if not c.keys:
            c.key(0.0, side * 1.0, "step")
        c.key(t0, side * 1.0, "linear")
        c.key(t0 + dur, 0.0, "ease")
        return t0 + dur

    def start_in_gear(self, gear):
        """Sleeve already fully engaged at t=0 (and the lever in that gate)."""
        if gear in (None, "N", 0):
            return
        k, side = S.GEAR_SYNCHRO[gear]
        self.sleeve[k].key(0.0, side * 1.0, "step")
        plane = {"12": -1, "34": 0, "5R": 1}[k]
        self.lever_plane.key(0.0, plane, "step")

    def cut(self, t):
        """Hard cut: state may jump here (speeds re-settled, dogs re-aligned)."""
        self.cuts.append(float(t))

    def align_engine(self, t, cycle_deg_cyl1):
        """After integration, shift the crank phase so that at time t
        cylinder 1's cycle angle equals cycle_deg_cyl1 (crank angle has no
        constraint with the rest of the drivetrain: the clutch is friction)."""
        self._engine_align.append((float(t), float(cycle_deg_cyl1)))

    # -- integration -------------------------------------------------------
    def run(self):
        return _integrate(self)


# ===========================================================================
# Track
# ===========================================================================

@dataclass
class Track:
    fps: int
    frames: np.ndarray            # Blender frame numbers (1..N)
    t: np.ndarray                 # video time
    data: dict = field(default_factory=dict)
    violations: list = field(default_factory=list)
    scene_id: str = None

    def __getattr__(self, k):
        d = self.__dict__.get("data")
        if d is not None and k in d:
            return d[k]
        raise AttributeError(k)

    def __getitem__(self, k):
        return self.data[k]

    @property
    def n(self):
        return len(self.frames)

    def frame_at(self, t):
        return int(round(t * self.fps)) + 1

    def idx(self, t):
        return int(np.clip(round(t * self.fps), 0, self.n - 1))

    def at(self, key, t):
        return self.data[key][self.idx(t)]

    # convenience derived angles ------------------------------------------
    def gb(self, name):
        """Abs angle array of a gearbox part (see kin.gearbox_phases)."""
        return kin.gb_angle(name, self.data["theta_in"], self.data["theta_out"])

    def validate(self, strict=True, aliasing=None):
        """Raise (strict) or return the list of physics violations.

        aliasing: optional dict name -> (abs_angle_array, feature_pitch_rad,
        visible_mask or None) to check for wagon-wheel strobing (per-frame
        step must stay below 0.35 pitch unless motion blur is used)."""
        v = list(self.violations)
        if aliasing:
            for name, item in aliasing.items():
                ang, pitch = item[0], item[1]
                mask = item[2] if len(item) > 2 and item[2] is not None else np.ones(self.n, bool)
                step = np.abs(np.diff(np.asarray(ang)))
                m = mask[1:] & mask[:-1]
                if np.any(m):
                    worst = float(np.max(step[m]) / pitch)
                    if worst > 0.35:
                        f = int(np.argmax(np.where(m, step, 0))) + 1
                        v.append(f"aliasing: {name} moves {worst:.2f} pitch/frame at frame {f}")
        if strict and v:
            raise AssertionError(f"{self.scene_id}: drivetrain state violations:\n  " + "\n  ".join(v[:40]))
        return v

    def summary(self, every_s=1.0):
        rows = []
        step = max(1, int(round(every_s * self.fps)))
        for i in range(0, self.n, step):
            d = self.data
            rows.append(f"t={self.t[i]:6.2f} gear={d['gear'][i]:>2} pedal={d['pedal'][i]:.2f} "
                        f"cap={d['capacity'][i]:.2f} eng={d['rpm_e'][i]:6.0f} in={d['rpm_in'][i]:6.0f} "
                        f"out={d['rpm_out'][i]:6.0f} v={d['v_kmh'][i]:5.1f} {d['status'][i]}")
        return "\n".join(rows)


def _gear_k(g):
    """abs-angle rate of gear g's dog ring per unit input angle."""
    if g == 4:
        return 1.0
    return kin.gb_phase_table()[f"gear_{g}"][1]


def _integrate(P: Program) -> Track:
    fps, sub = P.fps, P.substeps
    N = P.n_frames
    h = 1.0 / (fps * sub)
    nsteps = (N - 1) * sub
    tg = np.arange(nsteps + 1) * h            # substep boundaries
    tm = tg[:-1] + h / 2                       # midpoints
    if nsteps == 0:
        tm = np.array([0.0])
    slow = np.maximum(P.slowmo(tm), 0.0)
    vk = P.speed_kmh(tm)
    kap = P.curvature(tm)
    ped = np.clip(P.pedal(tm), 0, 1)
    thr = P.throttle_rpm(tm)
    eon = P.engine_on(tm) > 0.5
    sl = {k: np.clip(P.sleeve[k](tm), -1, 1) for k in S.SYNCHROS}
    lpl = P.lever_plane(tm)
    cap = kin.clutch_capacity(ped)
    cut_steps = sorted({int(round(tc / h)) for tc in P.cuts})

    st = dict(th_e=P.theta_e0, th_in=P.theta_in0, th_RL=P.wheel0, th_RR=P.wheel0, th_FL=P.wheel0,
              th_FR=P.wheel0, w_e=0.0, w_in=0.0, locked=False, w_in_prev=0.0)
    rx0, ry0, hd = P.car_pose0
    rx = rx0 + math.sin(hd) * S.WHEELBASE      # rear-axle centre
    ry = ry0 - math.cos(hd) * S.WHEELBASE
    viol = []
    sync_plan = {}
    blocker = {k: 0.0 for k in S.SYNCHROS}

    def engaged_gear(i):
        for k in S.SYNCHROS:
            s = sl[k][i]
            if abs(s) >= S.SYNC_THROUGH:
                return S.SYNCHRO_GEARS[k][1 if s > 0 else -1], k
        return None, None

    def find_after(arr, i0, pred):
        idx = np.nonzero(pred(arr[i0:]))[0]
        return i0 + int(idx[0]) if len(idx) else None

    def out_speed(i):
        vl, vr = kin.wheel_speeds(vk[i] / 3.6, kap[i])
        return S.FINAL_DRIVE * 0.5 * (vl + vr) / S.ROLLING_RADIUS

    def settle(i):
        """Make speeds consistent instantly (scene start and hard cuts)."""
        g, _ = engaged_gear(i)
        w_out = out_speed(i)
        w_thr = S.rpm_to_rad_s(thr[i]) if eon[i] else 0.0
        if g is not None:
            k_g = _gear_k(g)
            st["w_in"] = w_out / k_g
            th_out = S.FINAL_DRIVE * 0.5 * (st["th_RL"] + st["th_RR"])
            st["th_in"] -= kin.dog_misalignment(g, st["th_in"], th_out) / k_g
            if cap[i] > 0.02 and eon[i]:
                st["w_e"], st["locked"] = st["w_in"], True
            else:
                st["w_e"], st["locked"] = w_thr, False
        else:
            st["w_e"] = w_thr
            if cap[i] > 0.02:
                st["w_in"], st["locked"] = st["w_e"], True
            else:
                st["w_in"] = S.rpm_to_rad_s(P.w_in0_rpm) if P.w_in0_rpm is not None else 0.0
                st["locked"] = False
        st["w_in_prev"] = st["w_in"]

    settle(0)

    keys = ("theta_e", "w_e", "theta_in", "w_in", "theta_out", "w_out", "theta_RL", "theta_RR", "theta_FL",
            "theta_FR", "w_RL", "w_RR", "w_FL", "w_FR", "steer_FL", "steer_FR", "car_x", "car_y",
            "car_heading", "v_kmh", "curvature", "pedal", "capacity", "slowmo", "t_sim", "locked",
            "throttle_rpm", "clutch_torque", "engine_torque")
    rec = {k: np.zeros(N) for k in keys}
    rec_bl = {k: np.zeros(N) for k in S.SYNCHROS}
    rec_sync = {k: np.zeros(N) for k in S.SYNCHROS}
    gear_rec = ["N"] * N
    aux = dict(t_sim=0.0, Tc=0.0, Te=0.0)

    def record(fi, i):
        v = vk[i] / 3.6
        vl, vr = kin.wheel_speeds(v, kap[i])
        dl, dr, vfl, vfr = kin.front_wheel_kinematics(v, kap[i])
        hx, hy = -math.sin(hd), math.cos(hd)
        r = rec
        r["theta_e"][fi], r["w_e"][fi] = st["th_e"], st["w_e"]
        r["theta_in"][fi], r["w_in"][fi] = st["th_in"], st["w_in"]
        r["theta_out"][fi] = S.FINAL_DRIVE * 0.5 * (st["th_RL"] + st["th_RR"])
        r["w_out"][fi] = S.FINAL_DRIVE * 0.5 * (vl + vr) / S.ROLLING_RADIUS
        r["theta_RL"][fi], r["theta_RR"][fi] = st["th_RL"], st["th_RR"]
        r["theta_FL"][fi], r["theta_FR"][fi] = st["th_FL"], st["th_FR"]
        r["w_RL"][fi], r["w_RR"][fi] = vl / S.ROLLING_RADIUS, vr / S.ROLLING_RADIUS
        r["w_FL"][fi], r["w_FR"][fi] = vfl / S.ROLLING_RADIUS, vfr / S.ROLLING_RADIUS
        r["steer_FL"][fi], r["steer_FR"][fi] = dl, dr
        r["car_x"][fi] = rx + hx * S.WHEELBASE
        r["car_y"][fi] = ry + hy * S.WHEELBASE
        r["car_heading"][fi] = hd
        r["v_kmh"][fi], r["curvature"][fi] = vk[i], kap[i]
        r["pedal"][fi], r["capacity"][fi] = ped[i], cap[i]
        r["slowmo"][fi], r["t_sim"][fi] = slow[i], aux["t_sim"]
        r["locked"][fi] = 1.0 if st["locked"] else 0.0
        r["throttle_rpm"][fi] = thr[i]
        r["clutch_torque"][fi], r["engine_torque"][fi] = aux["Tc"], aux["Te"]
        g, _ = engaged_gear(i)
        gear_rec[fi] = "N" if g is None else str(g)
        for k in S.SYNCHROS:
            rec_bl[k][fi] = blocker[k]
            rec_sync[k][fi] = 1.0 if (k in sync_plan and not sync_plan[k].get("done")) else 0.0

    record(0, 0)
    prev_g = engaged_gear(0)[0]
    planes = {"12": -1, "34": 0, "5R": 1}
    for i in range(nsteps):
        t0 = tg[i]
        if i in cut_steps and i > 0:
            settle(i)
            sync_plan.clear()
            for k in blocker:
                blocker[k] = 0.0
            prev_g = engaged_gear(i)[0]
        dt = slow[i] * h
        aux["t_sim"] += dt
        v = vk[i] / 3.6
        vl, vr = kin.wheel_speeds(v, kap[i])
        dl, dr, vfl, vfr = kin.front_wheel_kinematics(v, kap[i])
        dRL, dRR = vl * dt / S.ROLLING_RADIUS, vr * dt / S.ROLLING_RADIUS
        st["th_RL"] += dRL
        st["th_RR"] += dRR
        st["th_FL"] += vfl * dt / S.ROLLING_RADIUS
        st["th_FR"] += vfr * dt / S.ROLLING_RADIUS
        hx, hy = -math.sin(hd), math.cos(hd)
        rx += v * dt * hx
        ry += v * dt * hy
        hd += v * kap[i] * dt
        d_out = S.FINAL_DRIVE * 0.5 * (dRL + dRR)
        w_out = S.FINAL_DRIVE * 0.5 * (vl + vr) / S.ROLLING_RADIUS
        th_out = S.FINAL_DRIVE * 0.5 * (st["th_RL"] + st["th_RR"])
        g, _ = engaged_gear(i)

        # ---------------- interlock / lever / load checks (not across a hard cut)
        near_cut = any(-2 <= i - c <= 2 for c in cut_steps)
        moving = [] if near_cut else [k for k in S.SYNCHROS if abs(sl[k][i]) > 0.03]
        if len(moving) > 1:
            viol.append(f"t={t0:.2f}: interlock violated, sleeves {moving} out of neutral")
        for k in moving:
            if abs(lpl[i] - planes[k]) > 0.05:
                viol.append(f"t={t0:.2f}: sleeve {k} out of neutral but lever is in plane {lpl[i]:.2f}")
        if not moving and i + 1 < nsteps and abs(lpl[i + 1] - lpl[i]) > 1e-9:
            pass  # lever crossing the gate in neutral: fine
        elif moving and i + 1 < nsteps and abs(lpl[i + 1] - lpl[i]) > 1e-9:
            viol.append(f"t={t0:.2f}: lever moved across the gate while in gear")
        for k in S.SYNCHROS:
            s0 = abs(sl[k][i])
            s1 = abs(sl[k][min(i + 1, nsteps - 1)])
            if (not near_cut and 0.03 < max(s0, s1) < S.SYNC_ENGAGED and abs(s1 - s0) > 1e-12
                    and cap[i] > 0.05 and eon[i]):
                viol.append(f"t={t0:.2f}: sleeve {k} moving with the clutch engaged (capacity {cap[i]:.2f})")

        # ---------------- synchroniser planning
        for k in S.SYNCHROS:
            s = sl[k][i]
            a = abs(s)
            a_next = abs(sl[k][min(i + 1, nsteps - 1)])
            plan = sync_plan.get(k)
            if plan is not None and not plan.get("done") and (np.sign(s) != plan["side"] or a_next < a - 1e-12):
                sync_plan.pop(k)
                blocker[k] = 0.0
                plan = None
            if plan is not None and plan.get("done") and a < S.SYNC_CONTACT:
                sync_plan.pop(k)
                plan = None
            approaching = a_next > a + 1e-12 or (S.SYNC_BLOCK - 0.01 <= a and a_next >= a)
            if plan is None and S.SYNC_CONTACT <= a < S.SYNC_BLOCK + 0.004 and g is None and approaching:
                side = 1 if s > 0 else -1
                i_rel = find_after(np.abs(sl[k]), i, lambda x: x >= S.SYNC_BLOCK + 0.004)
                i_thr = find_after(np.abs(sl[k]), i, lambda x: x >= S.SYNC_THROUGH)
                if i_rel is None or i_thr is None:
                    i_rel = i_thr = nsteps
                i_rel = max(i_rel, i + 1)
                sync_plan[k] = dict(i0=i, w0=st["w_in"], i_rel=i_rel, i_thr=max(i_thr, i_rel + 1),
                                    gear=S.SYNCHRO_GEARS[k][side], side=side, align=None, applied=0.0)
        active = [k for k in sync_plan if not sync_plan[k].get("done")]

        # ---------------- input shaft (+ synchroniser)
        w_in_old = st["w_in"]
        syncing = False
        if g is not None:
            k_g = _gear_k(g)
            st["w_in"] = w_out / k_g
            d_in = d_out / k_g
            for k in active:
                sync_plan[k]["done"] = True
                blocker[k] = 0.0
            if prev_g is None and i not in cut_steps:
                rel_rpm = (w_in_old * k_g - w_out) * RPM
                if abs(rel_rpm) > 2.0:
                    viol.append(f"t={t0:.2f}: dog teeth of gear {g} meet at {rel_rpm:.0f} rpm difference (clash)")
                mis = kin.dog_misalignment(g, st["th_in"] + d_in, th_out)
                if abs(mis) > 2e-3:
                    viol.append(f"t={t0:.2f}: gear {g} dogs misaligned by {math.degrees(mis):.2f} deg at engagement")
        elif active:
            syncing = True
            k = active[0]
            plan = sync_plan[k]
            k_g = _gear_k(plan["gear"])
            w_target = w_out / k_g
            if i < plan["i_rel"]:
                u = min((i + 1 - plan["i0"]) / max(plan["i_rel"] - plan["i0"], 1), 1.0)
                f = 0.5 - 0.5 * math.cos(math.pi * u)
                w_new = plan["w0"] + (w_target - plan["w0"]) * f
                d_in = 0.5 * (st["w_in"] + w_new) * dt
                st["w_in"] = w_new
                rel = st["w_in"] * k_g - w_out
                ramp = min(1.0, (i + 1 - plan["i0"]) / max(1.0, 0.15 * (plan["i_rel"] - plan["i0"])))
                # cone friction drags the ring the way the gear slips relative to the hub (SYN-03)
                direction = 1.0 if rel > 0 else -1.0
                if abs(rel * RPM) < 0.5:
                    direction = plan.get("dir", direction)
                plan["dir"] = direction
                blocker[k] = direction * S.BLOCKER_INDEX * (2 * math.pi / S.DOG_TEETH) * ramp
            else:
                st["w_in"] = w_target
                d_in = st["w_in"] * dt
                if plan["align"] is None:
                    mis = kin.dog_misalignment(plan["gear"], st["th_in"] + d_in, th_out)
                    plan["align"] = -mis / k_g
                    plan["bl0"] = blocker[k]
                u = min((i + 1 - plan["i_rel"]) / max(plan["i_thr"] - plan["i_rel"], 1), 1.0)
                f = 0.5 - 0.5 * math.cos(math.pi * u)
                tgt = plan["align"] * f
                d_in += tgt - plan["applied"]
                plan["applied"] = tgt
                blocker[k] = plan["bl0"] * (1.0 - f)
            if cap[i] > 0.05 and eon[i]:
                viol.append(f"t={t0:.2f}: synchroniser {k} working against an engaged clutch")
        else:
            d_in = None   # decided with the engine below (neutral)

        # ---------------- engine + clutch (torque model)
        Tc = cap[i] * T_CLUTCH_MAX
        w_thr = S.rpm_to_rad_s(thr[i])
        Te = float(np.clip(K_GOVERNOR * (w_thr - st["w_e"]), T_ENGINE_MIN, T_ENGINE_MAX)) if eon[i] else 0.0
        w_e = st["w_e"]
        if not eon[i]:
            # engine off: it stays still; a clutch at capacity holds the input too
            st["w_e"], d_e = 0.0, 0.0
            if g is None and not syncing:
                if Tc > 1.0:
                    st["w_in"], d_in, st["locked"] = 0.0, 0.0, True
                else:
                    st["w_in"] *= math.exp(-dt / TAU_DRAG)
                    d_in = st["w_in"] * dt
                    st["locked"] = False
            Tcl = 0.0
        elif g is not None:
            alpha_in = (st["w_in"] - w_in_old) / dt if dt > 0 else 0.0
            if st["locked"] and Tc < abs(Te - I_ENGINE * alpha_in):
                st["locked"] = False
            if st["locked"]:
                Tcl = Te - I_ENGINE * alpha_in
                st["w_e"], d_e = st["w_in"], d_in
            else:
                slip = w_e - st["w_in"]
                Tcl = Tc * np.sign(slip)
                w_new = w_e + (Te - Tcl) / I_ENGINE * dt
                if Tc > 0 and slip * (w_new - st["w_in"]) <= 0 and Tc >= abs(Te - I_ENGINE * alpha_in):
                    w_new, st["locked"] = st["w_in"], True
                d_e = 0.5 * (w_e + w_new) * dt
                st["w_e"] = w_new
        elif syncing:
            Tcl = 0.0
            w_new = w_e + Te / I_ENGINE * dt
            d_e = 0.5 * (w_e + w_new) * dt
            st["w_e"] = w_new
            st["locked"] = False
        else:
            # neutral, input side free (small inertia)
            if st["locked"] and Tc > 1.0:
                w_new = w_e + Te / (I_ENGINE + I_INPUT) * dt
                d_e = 0.5 * (w_e + w_new) * dt
                st["w_e"] = st["w_in"] = w_new
                d_in = d_e
                Tcl = I_INPUT * (w_new - w_e) / dt if dt > 0 else 0.0
            else:
                st["locked"] = False
                wi = st["w_in"]
                slip = w_e - wi
                Tcl = Tc * np.sign(slip)
                drag = -I_INPUT * wi / TAU_DRAG
                wi_new = wi + (Tcl + drag) / I_INPUT * dt
                we_new = w_e + (Te - Tcl) / I_ENGINE * dt
                if Tc > 1.0 and slip * (we_new - wi_new) <= 0:
                    wc = (I_ENGINE * we_new + I_INPUT * wi_new) / (I_ENGINE + I_INPUT)
                    we_new = wi_new = wc
                    st["locked"] = True
                d_e = 0.5 * (w_e + we_new) * dt
                d_in = 0.5 * (wi + wi_new) * dt
                st["w_e"], st["w_in"] = we_new, wi_new
        if eon[i] and st["w_e"] * RPM < 450:
            viol.append(f"t={t0:.2f}: engine speed {st['w_e'] * RPM:.0f} rpm - the engine would stall")
        aux["Tc"], aux["Te"] = float(Tcl), Te
        st["th_e"] += d_e
        st["th_in"] += d_in
        prev_g = g
        if (i + 1) % sub == 0:
            record((i + 1) // sub, i)

    tr = Track(fps=fps, frames=np.arange(1, N + 1), t=np.arange(N) / fps, scene_id=P.scene_id)
    d = rec
    for (ta, cyc) in P._engine_align:
        fi = int(np.clip(round(ta * fps), 0, N - 1))
        cur = math.degrees(d["theta_e"][fi]) - S.FIRING_TDC_DEG[1]
        off = (cyc - cur) % 720.0
        d["theta_e"] = d["theta_e"] + math.radians(off)
    d["gear"] = np.array(gear_rec)
    for k in S.SYNCHROS:
        d[f"sleeve_{k}"] = np.clip(P.sleeve[k](tr.t), -1, 1)
        d[f"blocker_{k}"] = rec_bl[k]
        d[f"syncing_{k}"] = rec_sync[k]
    d["lever_plane"] = P.lever_plane(tr.t)
    lever_y = np.zeros(N)
    for k in S.SYNCHROS:
        lever_y += -d[f"sleeve_{k}"]
    d["lever_x"], d["lever_y"] = d["lever_plane"], lever_y
    for w in ("FL", "FR", "RL", "RR"):
        d[f"susp_{w}"] = P.susp[w](tr.t)
    for kk, vv in kin.clutch_geometry(d["pedal"]).items():
        d[f"clutch_{kk}"] = vv
    d["rpm_e"], d["rpm_in"], d["rpm_out"] = d["w_e"] * RPM, d["w_in"] * RPM, d["w_out"] * RPM
    d["theta_case"] = 0.5 * (d["theta_RL"] + d["theta_RR"])
    d["rpm_case"] = 0.5 * (d["w_RL"] + d["w_RR"]) * RPM
    for w in ("RL", "RR", "FL", "FR"):
        d[f"rpm_{w}"] = d[f"w_{w}"] * RPM
    k_cs = -S.Z_INPUT / S.Z_CS_DRIVEN
    d["theta_cs"] = k_cs * d["theta_in"]
    d["rpm_cs"] = d["rpm_in"] * k_cs
    for g in (1, 2, 3, 5, "R"):
        d[f"rpm_gear_{g}"] = d["rpm_in"] * _gear_k(g)
    d["slip_rpm"] = d["rpm_e"] - d["rpm_in"]
    status = []
    for i in range(N):
        if d["w_e"][i] == 0 and d["throttle_rpm"][i] >= 0 and not (P.engine_on(np.array([tr.t[i]]))[0] > 0.5):
            status.append("ENGINE OFF")
        elif d["capacity"][i] < 0.02:
            status.append("DISENGAGED")
        elif d["locked"][i] > 0.5:
            status.append("ENGAGED")
        else:
            status.append("SLIPPING")
    d["status"] = np.array(status)
    tr.data = d
    seen, out = set(), []
    for v in viol:
        key = v.split(":", 1)[1] if ":" in v else v
        if key not in seen:
            seen.add(key)
            out.append(v)
    tr.violations = out
    return tr
