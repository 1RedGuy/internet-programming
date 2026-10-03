# Orchestrator sync list (things to fold into FACTS.md / docs at the final pass)

FACTS.md must reflect the as-built model:
- Valve lift law: flat-tappet three-arc cam (kin.valve_lift; base R 18 mm, nose R 5 mm) — same
  IVO/IVC/EVO/EVC and peak lift; not a sin^2 law.
- Timing chain travel: exactly z*p per sprocket turn (kin.chain_travel). Cam centre spacing
  135.52 mm, chain 126 links.
- Ring gear on the LEFT (-X) of the pinion, pinion at the front (FD-04 already says -X; check text).
- Propshaft Hooke joint angle 2.64 deg (joint centres 44 mm inside the flanges), not 2.46.
- Inner (tripod) joint centre at |x| = 0.1771 (bolted to the diff flange at 0.150): joint
  spacing 0.478 m, plunge 3.78 mm, joint angle 7.2 deg at +-60 mm (CVJ-01/02/06).
  CVJ-06 plunge formula: exact L - sqrt(L^2 - dz^2).
- Clutch hydraulics: free play before the master piston; working strokes master 21.5 mm,
  slave 14.9 mm; effective fork ratio 1.655 (arms 69.7:115.4 mm); pedal 6:1;
  pressure plate lift via diaphragm lever 4.24 over its lifting range; disc gap 0.615 mm/face
  at full release (facing cushion 0.65 mm).
- Narration lines changed (s02.compression, s04.ratios, s04.synchro, s03.engaged, s03.slip,
  s07.plunge, s08.summary): update the narration->facts table (section 20) and section 21.
- Slow-motion factors actually used per scene (from the scene modules) -> PRS section.
- PRS-13 s08 row: built — real time (slowmo 1), Cycles motion blur shutter 0.5; road speed held
  through each 2.2–2.6 s clutch-in so the engine locks at exactly 1787 / 2011 rpm (a real coast
  would land ~50 rpm lower). Take-off: slip from 4.79 s, lock at 6.25 s (8.6 km/h, 1073 rpm).
- state.status reads DISENGAGED whenever clutch capacity < 2 %, even if the speeds still differ.
- s02: flywheel motion-blurred during the x85 firing beat (ring gear would strobe otherwise).
- PRS-02/PRS-11: s02 flywheel is now visible all scene; ring gear de-strobed by per-object motion
  blur (only flywheel + ring gear) with a keyed shutter = 1 tooth pitch of smear (1.0 frame at
  x198-230, 1.09 at x85, up to ~1.9 in the ramps). Remove "faded out 17.4-75.0 s".
- PRS-13 per scene (as built):
  * s03: x230 (0-27.6 s) -> x56 (pedal press 0.25 s real; disc coasts 850->762 rpm) -> hard cut at
    46.5 s to x8 (synchro stops disc ~0.1 s; take-off bite->lock 1.19 s real, lock 1208 rpm at
    9.7 km/h, peak accel 2.7 m/s^2; clutch torque matched to the car's acceleration ~95 N m).
    (slip shots being reworked — update if the slow-motion profile changes)
  * s04: x72 idle/neutral/synchro/lock/linkage; driving cuts x130 (1st 12.05 km/h, 4th 42.0,
    5th 51.6 at 1500 rpm); reverse x80 at -7.8 km/h (950 rpm); synchro at x72 = ~26 ms sim.
  * s05: x150 constant; shift 0.27 s real; clutch locks at 1778 rpm (road speed coasts
    24.15 -> 24.03 km/h at 0.13 m/s^2, SFT-07), lock at 47.4 s.
  * s06: x20, frozen (PAUSED) 21.55-31.45 s for the exploded diff, x20 -> x12; 15 km/h held
    through the R = 5 m left turn: inner 111.1, outer 149.8, case 130.5 rpm.
  * s07: x12 (tyre tread needs >= x11.05 at 10 km/h), ramp to real time 35.3-38.6 s; body heave
    +-60 mm at 1.33 Hz; true-scale plunge gauge.
- Presentation liberties to list: s06 time freeze for the explode; s03/s05/s07 world-fixed live
  section planes on rotating parts; ghosted shells; s04 housings removed for the reverse shot.
