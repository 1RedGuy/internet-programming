"""Numerical proof that the kinematics are consistent.  Run: python3 tools/test_kinematics.py

Checks (no Blender needed):
  * every gearbox mesh keeps teeth interleaved at all angles (u1+u2 = 1/2 mod 1)
  * speed factors equal the spec ratios; rotation directions are right
  * firing order / TDC / valve events / cam nose direction / spark timing
  * clutch hydraulics conserve volume; differential averages wheel speeds
  * Hooke joints in Z-arrangement cancel; Ackermann geometry sane
  * drivetrain state: a 1-2 shift and a take-off integrate without violations
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from carviz import kin, spec as S, state  # noqa: E402

FAILS = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        FAILS.append(msg)


def u(phi, z, direction):
    return ((direction - phi) * z / (2 * math.pi)) % 1.0


def mesh_ok(name1, name2, z1, z2, c1, c2, thetas):
    P = kin.gb_phase_table()
    d12 = math.atan2(c2[1] - c1[1], c2[0] - c1[0])
    worst = 0.0
    for th in thetas:
        a1 = kin.gb_angle(name1, th)
        a2 = kin.gb_angle(name2, th)
        s = (u(a1, z1, d12) + u(a2, z2, d12 + math.pi)) % 1.0
        worst = max(worst, abs(s - 0.5))
    return worst


def main():
    th = np.linspace(-40, 40, 997)
    C = S.GEARBOX_CENTRE_DISTANCE
    M, CS = (0.0, 0.0), (0.0, -C)
    print("gear meshes (max deviation of u1+u2 from 1/2, in teeth):")
    w = mesh_ok("input_gear", "cs_drive", S.Z_INPUT, S.Z_CS_DRIVEN, M, CS, th)
    check(w < 1e-9, f"input 26 / countershaft 35 interleave (dev {w:.1e})")
    for g, (zc, zm) in S.GEAR_PAIRS.items():
        w = mesh_ok(f"cs_{g}", f"gear_{g}", zc, zm, CS, M, th)
        check(w < 1e-9, f"gear {g}: countershaft {zc} / output {zm} interleave (dev {w:.1e})")
    I = kin.reverse_idler_centre()
    w1 = mesh_ok("cs_R", "idler", S.Z_REV_CS, S.Z_REV_IDLER, CS, I, th)
    w2 = mesh_ok("idler", "gear_R", S.Z_REV_IDLER, S.Z_REV_OUT, I, M, th)
    check(w1 < 1e-9 and w2 < 1e-9, f"reverse train 15-22-38 interleave (dev {max(w1, w2):.1e})")
    m = S.REV_MODULE
    check(abs(math.dist(CS, I) - m * (S.Z_REV_CS + S.Z_REV_IDLER) / 2) < 1e-9 and
          abs(math.dist(I, M) - m * (S.Z_REV_IDLER + S.Z_REV_OUT) / 2) < 1e-9,
          f"reverse idler centre distances exact, idler at ({I[0]*1000:.1f}, {I[1]*1000:.1f}) mm")

    print("ratios and directions:")
    P = kin.gb_phase_table()
    check(P["cs_drive"][1] < 0, "countershaft turns opposite to the input shaft")
    for g in (1, 2, 3, 5):
        k = P[f"gear_{g}"][1]
        check(k > 0 and abs(1 / k - S.GEAR_RATIOS[g]) < 1e-12,
              f"gear {g}: output gear turns with input, ratio {1/k:.3f} = spec {S.GEAR_RATIOS[g]:.3f}")
    kR = P["gear_R"][1]
    check(kR < 0 and abs(1 / kR - S.GEAR_RATIOS["R"]) < 1e-12,
          f"reverse: output gear turns backwards, ratio {1/kR:.3f}")
    check(abs(S.GEAR_RATIOS[4] - 1.0) < 1e-12, "4th is direct drive 1:1")
    check(S.GEAR_RATIOS[5] < 1.0, f"5th is an overdrive ({S.GEAR_RATIOS[5]:.3f})")
    for g, (zc, zm) in S.GEAR_PAIRS.items():
        check(zc + zm == S.GEAR_TOOTH_SUM, f"gear {g} tooth sum {zc}+{zm} = {S.GEAR_TOOTH_SUM} (common centre distance)")
    check(S.Z_INPUT + S.Z_CS_DRIVEN == S.GEAR_TOOTH_SUM, "headset tooth sum 26+35 = 61")

    print("engine:")
    for c in (1, 2, 3, 4):
        t = math.radians(S.FIRING_TDC_DEG[c])
        z = kin.piston_height(t, c)
        check(abs(z - (S.CRANK_THROW + S.CONROD_LENGTH)) < 1e-12, f"cyl {c} at TDC when it fires ({math.degrees(t):.0f} deg)")
        zb = kin.piston_height(t + math.pi, c)
        check(abs(zb - (S.CONROD_LENGTH - S.CRANK_THROW)) < 1e-12, f"cyl {c} at BDC half a turn later")
    seq = sorted(S.FIRING_TDC_DEG, key=lambda c: S.FIRING_TDC_DEG[c])
    check(tuple(seq) == S.FIRING_ORDER, f"firing sequence by crank angle {seq} = {S.FIRING_ORDER}")
    gaps = np.diff(sorted(S.FIRING_TDC_DEG.values()) + [720.0])
    check(np.allclose(gaps, 180.0), "one power stroke every 180 deg")
    # crank rotation: crankpin moves from top toward -X (viewer's right seen from the front) = clockwise from front
    p0 = kin.crankpin_psi(0.0, 1)
    p1 = kin.crankpin_psi(0.1, 1)
    check(math.cos(p1) < math.cos(p0) + 1e-12 and math.cos(p1) < 0, "crank turns clockwise seen from the front")
    for c in (1, 2, 3, 4):
        base = math.radians(S.FIRING_TDC_DEG[c])
        lift = lambda deg, kind: float(kin.valve_lift(base + math.radians(deg), c, kind))
        check(lift(450, "intake") > 0.8 * S.VALVE_LIFT_INTAKE and lift(450, "exhaust") == 0.0,
              f"cyl {c}: intake open, exhaust shut mid-intake")
        check(lift(630, "intake") == 0.0 and lift(630, "exhaust") == 0.0, f"cyl {c}: both shut mid-compression")
        check(lift(90, "intake") == 0.0 and lift(90, "exhaust") == 0.0, f"cyl {c}: both shut mid-power")
        check(lift(270, "exhaust") > 0.8 * S.VALVE_LIFT_EXHAUST and lift(270, "intake") == 0.0,
              f"cyl {c}: exhaust open mid-exhaust")
        check(lift(360, "exhaust") > 0 and lift(360, "intake") > 0, f"cyl {c}: valve overlap at TDC exhaust/intake")
        sp = [float(kin.spark(base + math.radians(d), c)) for d in np.arange(-60, 60, 0.5)]
        dpk = np.arange(-60, 60, 0.5)[int(np.argmax(sp))]
        check(abs(dpk + S.SPARK_ADVANCE_DEG) < 0.6, f"cyl {c}: spark peaks {dpk:.1f} deg (before TDC)")
    # cam nose points at the valve at peak lift
    for c in (1, 3):
        for kind in ("intake", "exhaust"):
            lobe = kin.cam_lobe_psi(c, kind)
            tpk = math.radians(S.FIRING_TDC_DEG[c] + kin.valve_peak_cycle_deg(kind))
            nose = (lobe + kin.cam_angle(tpk)) % (2 * math.pi)
            check(abs(((nose + math.pi / 2) + math.pi) % (2 * math.pi) - math.pi) < 1e-9,
                  f"cyl {c} {kind} cam nose points down at its valve at peak lift")
    check(abs(kin.cam_angle(4 * math.pi) - 2 * math.pi) < 1e-12, "camshaft makes one turn per two crank turns")
    check(S.CAM_SPROCKET_TEETH == 2 * S.CRANK_SPROCKET_TEETH, "cam sprocket has twice the crank sprocket teeth")

    print("clutch:")
    g = kin.clutch_geometry(np.array([0.0, 0.5, 1.0]))
    va = math.pi / 4 * S.MASTER_CYL_BORE ** 2 * g["master"]
    vb = math.pi / 4 * S.SLAVE_CYL_BORE ** 2 * g["slave"]
    check(np.allclose(va, vb), "fluid volume leaving master = volume entering slave")
    check(abs(g["bearing"][2] - S.RELEASE_BEARING_TRAVEL) < 1e-12 and abs(g["plate_lift"][2] - S.PRESSURE_PLATE_LIFT) < 1e-12,
          f"full pedal: bearing {S.RELEASE_BEARING_TRAVEL*1000:.1f} mm, plate lift {S.PRESSURE_PLATE_LIFT*1000:.1f} mm")
    check(kin.clutch_capacity(0.0) == 1.0 and kin.clutch_capacity(1.0) == 0.0, "pedal up = full clamp, floor = free")

    print("driveline:")
    a = np.linspace(0, 20, 2001)
    beta = math.radians(2.5)
    mid = kin.hooke(a, beta)
    out = kin.hooke(mid, -beta)  # second joint, Z-arrangement (equal and opposite)
    # inverse relation: tan(out) = tan(mid) * cos(beta)  -> equals input
    out2 = np.arctan2(np.sin(mid), np.cos(mid) / math.cos(beta))
    out2 = out2 + np.round((mid - out2) / (2 * math.pi)) * 2 * math.pi
    check(np.max(np.abs(out2 - a)) < 1e-9, "two Hooke joints in Z-arrangement: pinion follows gearbox exactly")
    fl = np.max(np.abs(np.diff(mid) / np.diff(a) - 1))
    check(fl < 0.002, f"propshaft speed fluctuation at 2.5 deg only {fl*100:.3f} %")
    vl, vr = kin.wheel_speeds(10.0, 1 / 5.0)
    check(vl < vr and abs((vl + vr) / 2 - 10.0) < 1e-12, f"left turn R=5 m: inner {vl:.2f} m/s < outer {vr:.2f}, mean = axle")
    dl, dr, _, _ = kin.front_wheel_kinematics(10.0, 1 / 5.0)
    check(dl > dr > 0, f"Ackermann: inner wheel steers more ({math.degrees(dl):.1f} > {math.degrees(dr):.1f} deg)")
    ss = kin.spider_spin(np.array([1.0]), np.array([1.0]))
    check(abs(ss[0]) < 1e-12, "straight line: spiders do not spin on their pin")

    print("drivetrain state programs:")
    P = state.Program(None, duration=50)
    P.slowmo.key(0, 1 / 150, "step")
    v1 = S.road_speed_kmh(3000, 1)
    P.speed_kmh.key(0, v1, "step")
    P.start_in_gear(1)
    P.throttle_rpm.key(0, 3000, "step"); P.throttle_rpm.key(7.4, 3000, "linear"); P.throttle_rpm.key(7.6, 800, "step")
    P.pedal.key(0, 0, "step"); P.pedal.key(7.2, 0, "linear"); P.pedal.key(10.5, 1, "ease")
    P.pedal.key(38.5, 1, "linear"); P.pedal.key(42, 0, "ease")
    P.disengage(1, 12.6, 3.5)
    P.engage(2, 18.1, travel=2.5, hold=9.0, through=4.0, seat=2.0)
    T = P.run()
    check(not T.violations, f"1-2 shift integrates cleanly {T.violations[:2]}")
    exp = 3000 * S.GEAR_RATIOS[2] / S.GEAR_RATIOS[1]
    check(abs(T.rpm_e[-1] - exp) < 5 and T.locked[-1] > 0.5, f"after the shift engine locked at {T.rpm_e[-1]:.0f} rpm (expected {exp:.0f})")
    i = T.idx(34.0)
    check(abs(T.rpm_gear_2[i] - T.rpm_out[i]) < 0.5, "2nd gear synchronised to the output shaft before the dogs meet")
    i = T.idx(22.0)
    check(T.syncing_12[i] > 0 and abs(T.blocker_12[i]) > 0, "blocker ring indexed (blocking) while speeds differ")

    if FAILS:
        print(f"\n{len(FAILS)} FAILURES")
        sys.exit(1)
    print("\nall kinematic checks passed")


if __name__ == "__main__":
    main()
