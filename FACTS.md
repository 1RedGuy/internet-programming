# FACTS: every mechanical behaviour the film shows or narrates

This is the technical reference for *How a Manual Car Works*. Each fact has an ID, a precise statement and a short
justification. Every number comes from `carviz/spec.py` and was computed in Python (the formula is shown; the
appendix has a script that reproduces the key values). Facts marked **(context)** are not shown on screen. They
explain or constrain something that is shown.

Conventions follow ARCHITECTURE.md section 2:

* **CW-F** means clockwise seen from the front of the car. It is the positive running direction of every
  longitudinal member. CCW-F is the opposite.
* **fwd-roll** is the direction a wheel turns when the car rolls forward (right-handed about -X). It is the positive
  direction of every transverse member.
* *i* is a gearbox ratio (input rpm / output rpm). "Typical" or "illustrative" marks a value that is not in spec; its
  assumptions are stated.

## Key numbers

| Quantity | Formula | Value |
|---|---|---|
| Displacement | 4 · π/4 · 86² · 86 mm³ | 1998 cm³ (499.6 cm³ per cylinder) |
| Rod ratio | L/r = 145/43 | 3.372 (rod swings ±17.25°) |
| Firing interval | 720°/4 | 180° (order 1-3-4-2) |
| Cam speed | 21/42 | 0.5 × crank (425 rpm at 850 rpm idle) |
| Clutch slave stroke | 23.33 mm × (15.87/19.05)² | 16.19 mm (fork 1.80:1 → bearing 9.0 mm → plate lift 1.8 mm) |
| Gear centre distance | (2.25/cos 25°) × 61/2 | 75.72 mm |
| Gear ratios | (35/26) × z_out/z_cs | 3.484 / 2.075 / 1.391 / 1.000 / 0.815 / R -3.410 |
| Final drive | 41/10 | 4.10 |
| Overall ratio | i × 4.10 | 14.29 / 8.51 / 5.70 / 4.10 / 3.34 / R -13.98 |
| Road speed at 3000 rpm | (2π·3000/60)/(i·4.1) × 0.305 | 24.1 / 40.5 / 60.5 / 84.1 / 103.3 km/h, R -24.7 |
| 1→2 shift at 3000 rpm | 3000 × 2.075/3.484 | lands at 1787 rpm (2→3: 2011) |
| Propshaft joint angle | atan(55/1280) | 2.46° per joint (Z arrangement) |
| Turn, R = 5 m | (5 ∓ 0.74)/5 | inner 0.852 ×, outer 1.148 × case speed (inner/outer 0.742) |

---

## 1. Vehicle layout (VEH)

- **VEH-01**: The car is a front-engine, rear-wheel-drive (FR) left-hand-drive saloon with a longitudinal inline-4
  and a 5-speed manual gearbox. Wheelbase is 2.620 m, track is 1.470 m front and 1.480 m rear, and tyres are
  205/55 R16.
  *Why:* spec "Vehicle package" (BMW 3-series / GT86 class). In an FR car the front wheels steer and the rear wheels
  drive. Under acceleration, weight moves rearward onto the driven wheels.
- **VEH-02**: The power path, in order, is: engine → clutch → gearbox → propeller shaft → final drive (pinion + ring
  gear) → open differential → two driveshafts (each with an inner tripod joint and an outer Rzeppa joint) → rear
  wheels. The front wheels are not driven.
  *Why:* this is the FR layout, and it matches the s01 "path" storyboard.
- **VEH-03**: The engine, clutch and gearbox share one level axis on the car's centre line (X = 0, Z = 0.360 m). The
  block spans Y = +0.130 to -0.292 m, so 69% of its length is behind the front-axle line.
  *Why:* 0.292/(0.130+0.292) = 0.692. The undriven front axle lets the engine sit behind it, which moves mass toward
  the middle of the car.
- **VEH-04**: A three-shaft gearbox (input / countershaft / output) suits a longitudinal RWD car for four reasons:
  - Input and output are coaxial, so the drive goes straight through to a propshaft on the centre line.
  - One gear (4th) can be direct, at 1:1 with no gear mesh loaded.
  - The output turns the same way as the engine.
  - Two reductions in series (1.346 × 2.588) reach 3.48:1 in 1st without a tiny pinion or a huge gear.

  *Why:* see GBX-10 and GBX-14. With a 61-tooth sum, a single 3.48:1 mesh would need about 13.6 : 47.4 teeth, close
  to the 13.1-tooth undercut limit (GBX-05).
- **VEH-05**: Most front-wheel-drive cars mount the engine transversely and use a transaxle with no countershaft.
  The input and output shafts are parallel and side by side, and each gear uses one mesh. The output shaft's pinion
  drives the differential's ring gear inside the same case. Classic 5-speeds have 2 shafts; many 6-speeds split the
  gears over two output shafts. **(context)**
  *Why:* a transverse engine leaves no length for an in-line box and a propshaft. A short, wide box with the
  differential beside it fits between the front wheels. Such a box has no direct drive, and its output turning
  opposite to the input does not matter because the final drive is in the same case.
- **VEH-06**: The gearbox output flange (Y -1.120, Z 0.360) is 55 mm higher than the pinion flange (Y -2.400,
  Z 0.305), and the propshaft spans 1.281 m.
  *Why:* spec Y_GEARBOX_REAR, Y_PINION_FLANGE, Z_PINION = WHEEL_CENTER_Z. See PRP-02.
- **VEH-07**: The differential is fixed to the body (independent rear suspension). Only the wheel hubs move (±60 mm
  in s07), so each driveshaft needs a joint at both ends.
  *Why:* spec Y_DIFF/Z_DIFF are fixed, and SUSPENSION_TRAVEL = 0.060.

## 2. Engine (ENG)

- **ENG-01**: The engine is a 2.0 L DOHC 16-valve port-injected petrol inline-4 with compression ratio 10.5:1, idle
  850 rpm and redline 6800 rpm. Its four cylinders sit in a row along the car, with cylinder 1 at the front.
  *Why:* spec engine block.
- **ENG-02**: Swept volume per cylinder is V_s = π/4·B²·S = π/4 × 86² × 86 mm³ = 499.6 cm³, so the total is
  1998 cm³. Bore equals stroke (a "square" engine).
- **ENG-03**: Clearance volume is V_c = V_s/(CR - 1) = 499.6/9.5 = 52.6 cm³. At TDC the piston crown is 0.8 mm
  below the deck (DECK_HEIGHT = 43 + 145 + 31 + 0.8 = 219.8 mm). That gap holds 4.6 cm³, so the chamber plus gasket
  holds 47.9 cm³. **(context)**
- **ENG-04**: Cylinder centres are at Y = 0.060, -0.034, -0.128 and -0.222 m. The pitch is 94 mm, leaving 8 mm of
  metal between the 86 mm bores.
- **ENG-05**: Each piston follows slider-crank motion:
  s(θ) = r + L - [r·cos θ + √(L² - r²·sin² θ)], with r = 43 mm and L = 145 mm (L/r = 3.372, λ = r/L = 0.297).
  The rod swings ±asin(r/L) = ±17.25°. The rod turns the up-and-down motion into rotation: the gas force along the
  rod has a tangential component at the crankpin, and that component makes torque. The torque is zero at TDC and BDC.
- **ENG-06**: Piston motion is not a sine wave:
  - At 90° ATDC the piston has already travelled 49.5 mm (57.6% of the stroke).
  - Mid-stroke comes at 81.5° ATDC.
  - Peak piston speed comes at 74.7° ATDC and is 1.64 × the mean speed.
  - The piston moves faster near TDC and dwells near BDC.

  *Why:* rod angularity, computed from ENG-05. Animations must use the exact formula, not a sine.
- **ENG-07**: Mean piston speed (2·S·n/60) is 2.44 m/s at 850 rpm, 8.6 m/s at 3000 and 19.5 m/s at 6800. Peak speed
  is 4.0, 14.1 and 31.9 m/s. Acceleration at TDC, r·ω²·(1 + λ), is 442, 5502 and 28 270 m/s² (about 2900 g at
  redline). **(context)**
- **ENG-08**: The crank is flat-plane. Crankpins 1 and 4 are at 0°, and 2 and 3 at 180°. Pistons 1 and 4 move
  together, and 2 and 3 move together in the opposite direction, so two pistons are at TDC while the other two are
  at BDC.
  *Why:* CRANKPIN_PHASE_DEG. This is the standard inline-4 layout, and its primary forces balance.
- **ENG-09**: The firing order is 1-3-4-2. Each cylinder reaches TDC on its power stroke at crank angle 0° (1),
  180° (3), 360° (4) and 540° (2), evenly spaced at 720°/4 = 180°.
  *Why:* FIRING_TDC_DEG. Each value equals its crankpin phase mod 360 (checked). Cylinders reach TDC in pairs. At
  each such TDC one cylinder fires, and its partner (1↔4 or 2↔3) is at the overlap TDC between exhaust and intake.
- **ENG-10**: In each cylinder's own angle φ = (θ - θ_fire) mod 720, the strokes are: power 0-180, exhaust 180-360,
  intake 360-540 and compression 540-720. One cycle takes two crankshaft turns.
  *Why:* STROKES.
- **ENG-11**: At any instant the four cylinders are in four different strokes. For 0 ≤ θ < 180: cylinder 1 is on
  power, 3 on compression, 4 on intake and 2 on exhaust. Every 180° each cylinder moves on to its next stroke.
  *Why:* computed from FIRING_TDC_DEG and STROKES at θ = 0, 180, 360 and 540, and checked on a plotted timing
  diagram.
- **ENG-12**: The crankshaft turns CW-F, which is counter-clockwise seen from the flywheel end (SAE standard
  rotation). Everything coupled to it without a gear mesh turns the same way.
- **ENG-13**: Only the power stroke delivers work. The crankshaft drives the other three strokes, using flywheel
  energy and the cylinder that is firing. With four cylinders there is a power stroke every 180°, so two per turn.
  That is 28.3 Hz at idle (35.3 ms apart) and 100 Hz at 3000 rpm.

## 3. Four-stroke cycle, valve timing, ignition, fuel (CYC)

- **CYC-01**: Valve events, in cycle degrees with 0 = TDC firing, are:

  | Event | Cycle angle | Relative to dead centre |
  |---|---|---|
  | Exhaust opens (EVO) | 130 | 50° BBDC |
  | Exhaust closes (EVC) | 370 | 10° ATDC |
  | Intake opens (IVO) | 350 | 10° BTDC |
  | Intake closes (IVC) | 590 | 50° ABDC |

  Each valve is open for 240 crank° (120 cam°).
  *Why:* spec. These are typical road-engine figures.
- **CYC-02**: Valve overlap runs from 350 to 370 (20°) around the TDC between exhaust and intake. Both valves are
  slightly open there, which helps flush the chamber.
- **CYC-03**: The exhaust valves open 50° before BDC, near the end of the power stroke, when the piston is 86.5% of
  the way down. This blowdown releases the remaining pressure so the piston does not have to push against it. Little
  work is lost because the crank is near BDC and has little leverage. The s02 "exhaust" storyboard says the exhaust
  valves are "already opening near the end of the power stroke", which is correct.
- **CYC-04**: The intake valves close 50° after BDC because the moving charge keeps flowing in after BDC. Real
  compression only begins at IVC, after the piston has risen 11.6 mm. The effective compression stroke is 74.4 mm.
- **CYC-05**: Peak lift comes at cycle 470° for the intake (110° ATDC) and 250° for the exhaust (110° BTDC). The
  lobe separation angle is 110 cam°.
- **CYC-06**: Maximum lift is 9.5 mm for the intake valves (33 mm heads) and 9.0 mm for the exhaust valves (28 mm
  heads). Lift/diameter is 0.288 and 0.321, both above 0.25, so at full lift the curtain area π·D·L exceeds the head
  area. The intake valves are larger (2 × 33 mm covers 29.4% of the bore area, against 21.2% for the exhaust)
  because the intake charge is pushed in by less than 1 bar, while the exhaust leaves under cylinder pressure.
- **CYC-07**: Each cylinder has four valves: two intake valves on the intake camshaft and two exhaust valves on the
  exhaust camshaft, 16 in total. The narration's plural "valves" is correct.
- **CYC-08**: With port injection, each injector sprays petrol into the intake port, onto the back of the intake
  valves. The charge entering the cylinder on the intake stroke is therefore air plus fuel, at about 14.7:1 by mass
  (stoichiometric). Illustrative full-load figures: 0.532 g of air (ρ = 1.184 kg/m³, volumetric efficiency 0.9) and
  36 mg of fuel per cylinder per cycle.
  *Why:* spec says "port injected". A direct-injection engine would take in air only.
- **CYC-09**: Intake (φ 360-540): the descending piston lowers cylinder pressure below manifold pressure.
  Atmospheric pressure pushes the mixture in through the open intake valves; this is the "drawing in".
- **CYC-10**: Compression (540-720): with both valves shut (exhaust since 370, intake since 590), the rising piston
  squeezes the mixture into the clearance volume. The geometric ratio is 10.5:1.
- **CYC-11**: The spark comes 15° BTDC (cycle 705°, the end of compression). Burning takes time, so ignition leads
  TDC and peak pressure arrives shortly after TDC. 15° is 2.94 ms at 850 rpm and 0.83 ms at 3000 rpm.
  *Why:* SPARK_ADVANCE_DEG. A real ECU varies the advance with speed and load; 15° is a representative idle or
  light-load value.
- **CYC-12**: Power (0-180): the pressure of the burning gas drives the piston down. This is the only stroke that
  does work.
- **CYC-13**: Exhaust (180-360): the rising piston pushes the burnt gas out past the open exhaust valves.

## 4. Valvetrain drive (VLV)

- **VLV-01**: Two overhead camshafts (DOHC) open the valves. Each lobe pushes its valve open through a follower, and
  the valve spring closes it again. Each camshaft carries 8 lobes, two per cylinder.
- **VLV-02**: The camshafts turn at exactly half crank speed: 21T crank sprocket / 42T cam sprocket = 0.5, so
  425 rpm at 850 rpm idle. This is necessary because each valve opens once per 720° cycle, which is once per cam
  turn.
- **VLV-03**: A chain does not reverse rotation. Both camshafts turn CW-F like the crank, because every sprocket is
  wrapped on the outside of the chain loop.
- **VLV-04**: The chain is 3/8 in (9.525 mm) pitch, and sprocket pitch diameter is PD = p/sin(180°/z). The 21T crank
  sprocket is 63.9 mm. The 42T cam sprocket is 127.5 mm, with a tip diameter of about 132.8 mm. See note O1: this tip
  diameter is larger than the 130 mm cam spacing.
- **VLV-05**: Lobes of successive cylinders in the firing order are 90 cam° apart (180 crank°/2). **(context)**

## 5. Flywheel (FLY)

- **FLY-01**: A heavy single-mass flywheel (300 mm diameter, 30 mm thick) is bolted to the crank's rear flange. It
  stores kinetic energy: it absorbs the surplus of each power pulse and returns it between pulses (two pulses per
  turn), which smooths crank speed. Illustratively, a solid steel disc this size weighs 16.6 kg and has
  I = m·r²/2 = 0.187 kg·m², storing 742 J at 850 rpm.
- **FLY-02**: The flywheel's rear face is one of the clutch's two driving friction faces, and the clutch cover is
  bolted to it. The flywheel, cover, diaphragm spring and pressure plate therefore always turn at engine speed.
- **FLY-03**: The rim carries a 132-tooth starter ring gear. With a typical starter-ring module of about 2.25 mm, the
  PD is 297 mm and the tip diameter 301.5 mm, consistent with the 300 mm flywheel. The starter pinion meshes only
  while cranking. **(context)**
- **FLY-04**: A spigot (pilot) bearing in the crank's rear end supports the nose of the gearbox input shaft.
  **(context; standard practice)**

## 6. Clutch (CLU)

- **CLU-01**: The clutch is a single dry plate. From front to rear: flywheel face → friction disc → pressure plate →
  diaphragm spring (inside a cover bolted to the flywheel) → release bearing. The driving members are the flywheel,
  cover, spring and pressure plate. The driven member is the disc.
- **CLU-02**: The disc has these parts:
  - Facings 228/150 mm in diameter on both sides: n = 2 friction surfaces of 23 157 mm² each.
  - Cushion segments between the facings (clamped thickness 8.4 mm).
  - Six torsional damper springs between the facings and the hub.
  - A 23-tooth splined hub.

  *Why:* spec CLUTCH_*. The damper springs soften engagement shocks and engine torsional vibration.
- **CLU-03**: The disc hub is splined to the input shaft, so the two always turn together. The disc can still slide
  axially, so the clamp load acts equally on both faces and the disc floats free when released.
- **CLU-04**: Engaged (pedal up): the diaphragm spring, a dished Belleville spring, pushes the pressure plate forward
  and clamps the disc against the flywheel. Torque capacity is T_c = n·μ·F·r_m, with n = 2 and
  r_m = (114 + 75)/2 = 94.5 mm (uniform wear). While engine torque is below T_c nothing slips, and the input shaft
  turns at engine speed.
  *Illustrative:* to carry 200 N·m with a 1.3 margin and μ = 0.3, F = 260/(2 × 0.3 × 0.0945) ≈ 4.6 kN.
- **CLU-05**: The release is push-type. The release bearing pushes the diaphragm finger tips **forward** (+Y, toward
  the flywheel). The spring pivots on fulcrum rings in the cover, so its outer rim moves **rearward**, and strap
  springs pull the pressure plate rearward (-Y), unclamping the disc. Finger travel to plate lift is
  9.0 : 1.8 = 5.0 : 1 (DIAPHRAGM_LEVER_RATIO).
- **CLU-06**: Pedal to plate mapping, with p the pedal fraction from the top and linear motion after free play
  (lift = 1.8 mm × (p - 0.08)/0.92):

  | Pedal fraction p | Pedal travel | Bearing travel | Plate lift | Clutch |
  |---|---|---|---|---|
  | 0 to 0.08 | 0 to 11.2 mm | 0 | 0 | free play, nothing moves |
  | 0.22 | 30.8 mm | 1.37 mm | 0.27 mm | still full capacity |
  | 0.50 | 70 mm | 4.11 mm | 0.82 mm | capacity reaches zero |
  | 1.00 | 140 mm | 9.0 mm | 1.8 mm | fully released |

  *Why:* the 0.55 mm of lift over which clamp load fades is the disc's cushion-spring travel. The rest is running
  clearance, about 0.5 mm per face at full pedal. See note O3.
- **CLU-07**: The diaphragm spring's falling force curve keeps the clamp load nearly constant as the facings wear,
  and pedal effort drops once past its peak. **(context; standard practice)**
- **CLU-08**: Released with the gearbox in neutral: nothing drives the disc. The disc, input shaft, countershaft and
  free gears coast down slowly against oil churning and bearing drag, and the engine keeps idling.
- **CLU-09**: Released with 1st selected and the car stationary: the synchro forces 1st gear to the output-shaft
  speed, which is 0 rpm. 1st gear meshes with the countershaft, and the countershaft meshes with the input gear, so
  the countershaft, input shaft and disc stop too. The flywheel and pressure plate keep turning at 850 rpm past the
  released disc, transmitting essentially no torque.
- **CLU-10**: While the clutch slips, the torque it passes is the friction torque n·μ·F·r_m, set by the clamp load
  (that is, by pedal position) whatever the speed difference. The same torque brakes the engine. Heat generated is
  T × Δω; for example, 60 N·m at 1000 rpm of slip is 6.3 kW.
- **CLU-11**: Pulling away, the clutch locks in four steps:
  1. As the pedal rises, clamp load and clutch torque rise.
  2. Once clutch torque × overall ratio exceeds the resistance at the wheels, the car accelerates, and the disc with
     it.
  3. The engine needs extra throttle so it does not slow down.
  4. When disc speed reaches engine speed, slip stops. Static friction then holds the clutch locked, because capacity
     exceeds the transmitted torque.
- **CLU-12**: The release bearing's inner race spins with the diaphragm fingers while touching them. Its outer race,
  held by the fork, does not rotate.

## 7. Hydraulic release (HYD)

- **HYD-01**: The pedal lever ratio is 6:1, so 140 mm at the pad becomes 23.33 mm of master-cylinder pushrod
  stroke.
- **HYD-02**: Brake fluid is incompressible, so the volume the master piston displaces enters the slave cylinder:
  A_m·x_m = A_s·x_s. The bores are 15.87 mm (5/8 in, A_m = 197.8 mm²) and 19.05 mm (3/4 in, A_s = 285.0 mm²). The
  volume is 197.8 × 23.33 = 4615 mm³ (4.6 mL), and the slave stroke is x_s = 23.33 × (15.87/19.05)² = **16.19 mm**.
- **HYD-03**: The slave cylinder pushes the release fork. The fork ratio 16.19/9.0 = 1.80 turns this into 9.0 mm of
  forward bearing travel. Whether the slave end of the fork moves forward or rearward depends on where the fork
  pivots; the bearing always moves forward in a push-type clutch.
- **HYD-04**: Overall motion ratios are pedal : bearing = 140/9.0 = 15.6 and pedal : pressure plate = 140/1.8 = 77.8
  (= 6 × 1.441 × 1.80 × 5.0). Force is multiplied by the same factor, minus friction, because work in equals work
  out.
- **HYD-05**: The 8% free play (11.2 mm at the pad, 1.87 mm at the pushrod) is pushrod clearance plus the master
  piston closing its reservoir port. No pressure builds until it is taken up.
- **HYD-06**: Pressure travels along the line at the speed of sound in the fluid (on the order of 1 km/s), so the
  slave follows the pedal within milliseconds. The pulse animated along the line is a visualisation (PRS-10).

## 8. Gearbox (GBX)

- **GBX-01**: The gearbox has three shafts:
  - **Input shaft** (front): carries the 26T input gear, which has the 4th-gear dog teeth. Its nose runs in the
    crank spigot bearing.
  - **Countershaft**: directly below the input, at Z = 0.360 - 0.0757 = 0.2843 m. All of its gears are fixed to it.
  - **Output shaft**: coaxial behind the input, with its front end piloted inside the input shaft. Its free gears run
    on needle bearings, its synchro hubs are splined to it, and its rear flange drives the propshaft.
- **GBX-02**: The gearbox is constant mesh: every forward pair is always meshed. Shifting never slides gears into
  mesh; it only locks a free output gear to its shaft with a synchro sleeve.
- **GBX-03**: All pairs share one centre distance. The transverse module is m_t = m_n/cos β = 2.25/cos 25° =
  2.4826 mm, and the centre distance is a = m_t·Σz/2 = 2.4826 × 61/2 = **75.72 mm**. Every pair (26+35, 17+44,
  24+37, 30+31, 38+23) sums to 61 teeth, so all pairs share the same two shafts without profile shift.
- **GBX-04**: Pitch diameters (d = m_t·z), countershaft gear / output-side gear:

  | Pair | Countershaft gear | Output-side gear |
  |---|---|---|
  | Headset | 35T, 86.89 mm | input 26T, 64.55 mm |
  | 1st | 17T, 42.20 mm | 44T, 109.23 mm |
  | 2nd | 24T, 59.58 mm | 37T, 91.86 mm |
  | 3rd | 30T, 74.48 mm | 31T, 76.96 mm |
  | 5th | 38T, 94.34 mm | 23T, 57.10 mm |

  Tip diameters are 2·m_n = 4.5 mm larger.
- **GBX-05**: The teeth are helical at β = 25°. With standard addendum, every pair has a transverse contact ratio of
  1.42-1.45, plus an overlap ratio b·sin β/(π·m_n) = 0.60 per 10 mm of face width. Load passes smoothly from tooth to
  tooth, which makes the gears quiet and strong. The cost is axial thrust F_a = F_t·tan 25° = 0.47·F_t, carried by
  the bearings. The 17T pinion is not undercut, because the minimum is z_min = 2·cos β/sin² α_t = 13.1
  (α_t = 21.88°).
- **GBX-06**: Mating external helical gears have opposite hands. The input gear and all output-shaft gears have one
  hand, and all countershaft gears the other. On the countershaft, the driven 35T gear and the driving speed gear
  have the same hand but opposite tangential loads, so their thrusts partly cancel. **(context)**
- **GBX-07**: The countershaft turns **opposite** to the input (CCW-F) at 26/35 = 0.743 of its speed: 631 rpm at
  idle and 2229 rpm at 3000 rpm.
- **GBX-08**: Ratios are i = (35/26) × (z_out/z_cs):

  | Gear | Calculation | Ratio |
  |---|---|---|
  | 1st | 1.3462 × 44/17 | **3.484** |
  | 2nd | 1.3462 × 37/24 | **2.075** |
  | 3rd | 1.3462 × 31/30 | **1.391** |
  | 4th | direct | **1.000** |
  | 5th | 1.3462 × 23/38 | **0.815** |
  | Reverse | -(35/26) × (38/15) | **-3.410** |

  Torque is multiplied by the same factor, less about 2-3% loss per loaded pair of meshes.
- **GBX-09**: Steps between gears are 1.679, 1.492, 1.391 and 1.227, getting closer toward the top. The total spread
  is 3.484/0.815 = 4.28. **(context)**
- **GBX-10**: 4th is direct. The 3-4 sleeve moves forward and locks the output shaft to the input gear's dog teeth,
  so output turns with input and no gear mesh carries torque. The countershaft still spins, unloaded. This is the
  most efficient gear.
- **GBX-11**: 5th is an overdrive (0.815). The large 38T countershaft gear drives the small 23T output gear, so the
  output turns 1/0.815 = 1.227 × faster than the engine.
- **GBX-12**: In neutral (car stationary, clutch engaged, 850 rpm), each free output gear turns at input/i: 1st
  244 rpm, 2nd 410, 3rd 611 and 5th 1043 rpm, all CW-F. The reverse gear turns 249 rpm CCW-F and the idler 431 rpm.
  The output shaft, synchro hubs and sleeves stand still, and the needle bearings let the gears turn on the
  stationary shaft.
- **GBX-13**: Axial order on the output shaft, front to rear: input gear (4th dogs) | 3-4 synchro | 3rd | 2nd |
  1-2 synchro | 1st | reverse | 5-R synchro | 5th.
  *Why:* spec SYNCHROS comments: 2nd is in front of the 1-2 synchro, 4th in front of the 3-4 synchro, and reverse in
  front of the 5-R synchro.
- **GBX-14**: Each external mesh reverses rotation:
  - Forward gears use 2 meshes (input → countershaft → output), so the output turns like the engine (CW-F).
  - 4th uses no mesh.
  - Reverse uses 3 meshes (the extra one is the idler), so the output turns backwards.
- **GBX-15**: Neutral means all three sleeves are central. Every gear can spin, but none is connected to the output
  shaft, so no torque passes.

## 9. Reverse (REV)

- **REV-01**: The reverse train runs countershaft 15T → idler 22T (on its own short fixed shaft) → 38T reverse gear.
  The reverse gear is free on the output shaft and is locked by the 5-R sleeve moving **forward**. These are
  straight-cut spur gears with module 2.5 mm.
- **REV-02**: The ratio is -(35/26) × (22/15) × (38/22) = **-3.410**. The idler's tooth count cancels; it only adds
  one mesh, which reverses the direction.
- **REV-03**: The 15T gear (37.5 mm PD) and the 38T gear (95.0 mm PD) sit on 75.72 mm centres and cannot touch: their
  tip radii total 21.25 + 50.0 = 71.25 mm, leaving a 4.47 mm gap. The idler bridges it.
  - The idler centre is 46.25 mm from the countershaft axis and 75.00 mm from the output axis. That puts it 14.8 mm
    above the countershaft axis and 43.8 mm to one side.
  - The idler (tip radius 30 mm) would hit 1st gear's 44T (tip radius 56.9) and 17T (tip radius 23.4) if it reached
    into their planes, so its face must stay inside the reverse plane.
- **REV-04**: The reverse gears are spur rather than helical. Spur gears produce no axial thrust, and in many
  gearboxes reverse is engaged by sliding them into mesh; the straight teeth cause reverse's typical whine. Contact
  ratios are 1.53 (15/22) and 1.64 (22/38). **(context)**
- **REV-05**: The 15T spur pinion is below the 17-tooth no-undercut limit for a 20° pressure angle
  (2/sin² 20° = 17.1). It needs a positive profile shift x ≥ 0.12 (standard practice), or it accepts slight
  undercut. The ratio is unaffected. See note O2.
- **REV-06**: Reverse is engaged only with the car stationary. The 5-R synchro then brings the free reverse gear to
  the output's 0 rpm. With the car rolling forward, reverse would have to turn the whole gear train backwards.

## 10. Synchronisers (SYN)

- **SYN-01**: Three single-cone synchronisers sit on the output shaft: 1-2, 3-4 and 5-R. Each has:
  - a hub splined to the shaft;
  - a sleeve sliding on the hub's 32 external splines;
  - three spring-loaded struts;
  - on each side, a brass blocker ring whose internal cone faces a steel cone on the gear, next to that gear's ring
    of 32 dog teeth.
- **SYN-02**: The sleeve travels 8.5 mm each way from neutral to engaged. The phases (spec fraction × 8.5 mm) are:

  | Phase | Sleeve position | What happens |
  |---|---|---|
  | Contact | 2.55 mm | struts press the blocker onto the cone |
  | Block | 3.83 mm | sleeve chamfers rest on blocker chamfers; held here while synchronising |
  | Through | 6.12 mm | sleeve has passed the blocker teeth and touches the dog chamfers |
  | Engaged | ≥ 8.08 mm (95%) | counts as in gear |
  | Home | 8.5 mm | fully engaged |
- **SYN-03**: Indexing: when the cones touch, friction drags the blocker ring round with the gear until its lugs hit
  the ends of their hub slots. That is half a tooth pitch, 360°/32/2 = **5.625°**, in the direction the gear slips
  relative to the shaft. The blocker's chamfered teeth now sit tip-to-tip in front of the sleeve teeth.
- **SYN-04**: Blocking: the shift force F presses the sleeve onto the blocker chamfers. This produces a cone friction
  torque T = μ·F·r_c/sin α; a cone half-angle α of about 6-7° multiplies the torque by about 9 (typical). The chamfer
  angle is chosen so that this friction torque exceeds the chamfers' turn-back torque while any slip remains. The
  sleeve therefore cannot pass until the speeds are equal, however hard it is pushed; pushing harder only
  synchronises faster.
- **SYN-05**: Once synchronised, slip is zero and the cone torque vanishes. The chamfers turn the blocker back by up
  to 5.625°, and the sleeve passes through. Its chamfers then nudge the free gear by up to half a dog pitch to line
  up the dogs, and the sleeve slides over the dog teeth. Torque then flows gear → dog teeth → sleeve → hub → shaft;
  the cone carries none.
- **SYN-06**: This prevents clash, because the dog teeth only meet at zero relative speed. Without a synchro, the
  sleeve would strike dogs moving 585 rpm faster than it (1→2 at 3000 rpm, see SFT-01).
- **SYN-07**: The dog teeth transmit torque by direct contact, not friction. A slight back-taper on the dogs, plus
  the rail detent, stops the gear jumping out under load. **(context; standard practice)**
- **SYN-08**: The synchro only has to change the speed of the parts that the released clutch disconnects: the disc,
  input shaft, countershaft and free gears, a small inertia. The output shaft is tied to the whole car, so its speed
  barely changes.

## 11. Selector linkage (SEL)

- **SEL-01**: The shift pattern is an H with reverse at bottom right:

  | | Left plane | Centre plane | Right plane |
  |---|---|---|---|
  | Lever forward | 1st | 3rd | 5th |
  | Lever back | 2nd | 4th | Reverse |

  Neutral is the crossbar, and the lever is spring-centred on the 3-4 plane.
  *Why:* SHIFT_GATE.
- **SEL-02**: There are three shift rails (1-2, 3-4 and 5-R), each with a fork that sits in its sleeve's groove. 5th
  and reverse share one rail and one sleeve.
- **SEL-03**: Moving the lever sideways (select) puts its finger into one rail's slot. Moving it forward or back
  (shift) slides that rail, its fork and its sleeve.
- **SEL-04**: The linkage is direct. The lever pivots on a ball on top of the box (Y = -0.960 m) and its finger is
  below the pivot, so the finger moves opposite to the knob: **lever forward → rail, fork and sleeve rearward
  (-Y)**. This matches the layout: 1st, 3rd and 5th sit behind their synchros (sleeve rearward, lever forward), and
  2nd, 4th and reverse sit in front (sleeve forward, lever back).
- **SEL-05**: The same reversal applies sideways: knob left → finger right. With a direct linkage, the 1-2 rail's
  slot must therefore be at the +X end of the row under the finger, and the 5-R slot at the -X end.
- **SEL-06**: The lever ratio (knob : finger) is about 55 mm/8.5 mm = 6.5 : 1. The 30 mm gate spacing at the knob is
  therefore only about 4.6 mm at the finger. The rails' slotted shift heads sit side by side under the finger even
  though the rails themselves are further apart.
- **SEL-07**: Interlock: when one rail leaves neutral, interlock pins lock the other two in neutral. Two gears can
  never be engaged at once, because two ratios on one output shaft would jam the box.
- **SEL-08**: Spring-loaded detent balls hold each rail at neutral and at engaged. Real boxes with reverse beside 5th
  add a lockout so reverse cannot be selected straight from 5th; the film does not model one. **(context)**
- **SEL-09**: The forks do not rotate. The sleeve spins inside the fork's pads, which only push it axially.

## 12. The 1→2 shift at 3000 rpm (SFT)

- **SFT-01**: Before the shift, in 1st at 3000 rpm, the output shaft turns at 3000/3.484 = **861 rpm** (24.1 km/h).
  Free 2nd gear turns at 3000/2.075 = **1446 rpm**, 585 rpm faster than its shaft.
- **SFT-02**: Step 1, clutch in: no engine torque reaches the gear train. The driver closes the throttle, and the
  engine slows on its own friction and pumping losses, not through the gearbox.
- **SFT-03**: The clutch must be in for two reasons:
  - Dog teeth under load are held by friction and back-taper and will not slide out of 1st.
  - The synchro can only re-speed the small input-side inertia (SYN-08). With the clutch engaged, it would also have
    to drag the engine and flywheel, which are producing torque.
- **SFT-04**: Step 2: the 1-2 sleeve slides forward off 1st gear's dogs to the centre, which is neutral. With the
  lever direct, the lever moves rearward to neutral.
- **SFT-05**: Step 3: as the sleeve moves forward toward 2nd, the blocker cone rubs on 2nd gear's cone and slows 2nd
  gear from 1446 to 861 rpm. 2nd meshes with the countershaft, which meshes with the input gear, so the whole input
  side slows with it:
  - countershaft: 2229 → 1327 rpm;
  - input shaft and clutch disc: 3000 → 1787 rpm;
  - every other free gear too. Free 1st gear ends up at 513 rpm, now slower than its 861 rpm shaft.
- **SFT-06**: The new engine speed is n₂ = n₁ × i₂/i₁ = 3000 × 2.075/3.484 = **1787 rpm** at the same road speed.
  Other upshifts from 3000 rpm land at 2011 rpm (2→3), 2157 rpm (3→4) and 2444 rpm (4→5).
- **SFT-07**: Road speed is effectively constant during the shift. Coasting deceleration is about 0.13 m/s²
  (illustrative: rolling resistance 0.012 g, CdA 0.66 m², 1400 kg). Over 0.29 s that loses 0.14 km/h, so the engine
  would land at 1777 rpm instead of 1787 (-0.6%).
- **SFT-08**: Step 5, clutch out: if the engine is still above 1787 rpm, the clutch slips briefly. Its friction
  torque (CLU-10) pulls the engine down to disc speed, and then the clutch locks. A "taller" gear means the engine
  turns 2.075/3.484 = 0.596 × as fast for the same road speed, and wheel torque drops by the same factor.

## 13. Propeller shaft (PRP)

- **PRP-01**: The propeller shaft is a one-piece tube (65 mm diameter) with a Hooke (Cardan) universal joint at each
  end. It turns at gearbox-output speed: 861 rpm in 1st at 3000 rpm, and engine speed in 4th. It turns CW-F in
  forward gears.
- **PRP-02**: The gearbox output and the pinion are both horizontal and parallel, 55 mm apart in height over 1.280 m.
  Each joint therefore works at β = atan(55/1280) = **2.46°**.
- **PRP-03**: A single Hooke joint at an angle does not pass a constant speed:
  ω_out/ω_in = cos β/(1 - sin² β·sin² θ). The ratio swings between cos β and 1/cos β twice per turn. At 2.46° that is
  ±0.092%, an angle error of ±0.026° (about β²/4).
- **PRP-04**: In the Z arrangement the input and output shafts are parallel, the two joint angles are equal, and both
  yokes on the tube lie in the same plane. The second joint then exactly undoes the first, so the pinion turns at
  exactly gearbox-output speed. A numerical check gave a residual of 1×10⁻¹³°. Only the tube itself fluctuates, by
  ±0.092%.
- **PRP-05**: The fluctuation is invisible at 2.46°; it would take about 30° to reach ±15%. Uniform rotation on
  screen is therefore correct. A small non-zero angle is good practice because it keeps the joints' needle rollers
  moving. **(context)**

## 14. Final drive (FD)

- **FD-01**: A 10T pinion drives a 41T ring gear: 41/10 = **4.10**. Ring speed is pinion speed/4.1 and ring torque
  is 4.1 × pinion torque. At 3000 rpm in 1st, the pinion turns at 861 rpm and the ring at 210 rpm.
- **FD-02**: The bevel gears' axes meet at 90°, which turns the drive from along the car to across it. The ring's
  190 mm PD gives a module of 4.634 mm and a pinion PD of 46.3 mm. The pitch cone angles are atan(10/41) = 13.71° and
  76.29°, summing to 90°.
- **FD-03**: 41 and 10 share no common factor (gcd = 1). This is a hunting-tooth set: every pinion tooth meets every
  ring tooth, which evens out wear. **(context)**
- **FD-04**: With the pinion turning CW-F, the wheels roll forward only if the ring gear lies on the **left (-X)**
  of the pinion axis with its teeth facing +X. At the mesh, which is at the front of the ring, the pinion's left
  flank moves down, and so does the front edge of a forward-rolling ring.
  *Why:* a vector check of v = ω × r at the pitch point (ARCHITECTURE sign conventions). See note O4.
- **FD-05**: The film models a spiral-bevel set with no offset: the pinion axis passes through the ring axis, both at
  Z = 0.305 m. Real RWD axles are hypoid: the pinion sits below the ring centre line. That allows a bigger, stronger
  pinion and a lower propshaft and tunnel, at the cost of more tooth sliding, which needs hypoid oil. The ratio is
  still 41/10.
- **FD-06**: Spiral teeth engage gradually, like helical teeth, so they are quieter and stronger than straight
  bevels. **(context)**

## 15. Open differential (DIF)

- **DIF-01**: The ring gear is bolted to the differential case. A cross-pin in the case carries two 10T spider gears,
  which mesh with two 16T side gears. Each side gear is splined to one output, which is the stub of that
  driveshaft's inner joint.
- **DIF-02**: The case turns at the mean of the two side-gear (wheel) speeds: **ω_case = (ω_L + ω_R)/2**.
  *Why:* the side gears are equal, so relative to the case they must turn by equal and opposite amounts; the spider
  simply rolls between them.
- **DIF-03**: A spider's spin on its pin is ((ω_R - ω_L)/2) × 16/10. Going straight (ω_L = ω_R) it is zero: the
  spiders orbit with the case without turning on the pin, and the case, spiders and side gears turn as one block.
- **DIF-04**: An open differential splits torque equally: T_L = T_R = T_case/2, neglecting friction, because each
  spider is a balanced lever free to turn on its pin. In a turn the outer wheel gets more *power* (P = T·ω), not more
  torque.
- **DIF-05**: As a result, the total drive is limited to twice what the wheel with less grip can take. A wheel on ice
  can spin at twice case speed while the other wheel stands still. **(context; not shown)**
- **DIF-06**: In a turn of radius R = 5.0 m at the rear-axle centre, with a 1.48 m track, the inner wheel follows a
  4.26 m path and the outer wheel 5.74 m (no tyre slip). Outer/inner = **1.347** (inner/outer = 0.742). The inner
  wheel turns at **0.852 ×** case speed and the outer at **1.148 ×**.
- **DIF-07**: Worked example at 15 km/h (measured at the rear-axle centre); all values scale linearly with speed:

  | Quantity | Value |
  |---|---|
  | Case | 130.5 rpm |
  | Inner wheel | 111.1 rpm |
  | Outer wheel | 149.8 rpm |
  | Side gears relative to case | ±19.3 rpm |
  | Spiders on their pin | 30.9 rpm |
  | Engine in 1st / 2nd | 1864 / 1110 rpm |
  | Yaw rate | 47.7°/s |
- **DIF-08**: In the left turn in s06, the left wheel is the inner, slower wheel and the right wheel the outer one. In
  a right turn the spiders spin the other way.
- **DIF-09**: At R = 5.0 m, with Ackermann steering and no slip, the inner front wheel steers 31.6° and the outer
  24.6°. The front-axle centre follows a radius of √(5² + 2.62²) = 5.645 m. **(context for the turn shot)**

## 16. Driveshafts and CV joints (CVJ)

- **CVJ-01**: Each rear driveshaft has a plunging tripod joint at the differential (centre X = ±0.150 m) and a Rzeppa
  ball joint at the hub (X = ±0.655 m). The joint centres are 505 mm apart, and the shaft is level at rest (both
  joints at Z = 0.305 m).
- **CVJ-02**: The differential is body-mounted while the wheel moves ±60 mm, so the shaft angle changes by up to
  atan(60/505) = 6.8°. Joints are needed at both ends. They must be constant-velocity joints: a plain Hooke joint
  would make the wheel speed fluctuate between cos β and 1/cos β (PRP-03), and it cannot plunge.
- **CVJ-03**: In the Rzeppa joint, six balls run in curved grooves between an inner race (on the shaft) and an outer
  race (the bell on the hub stub). A cage holds all the ball centres in the plane that bisects the angle between the
  two shafts.
- **CVJ-04**: Each ball centre is therefore the same distance from both shaft axes, so its contact speeds on the two
  races match at every instant. Wheel speed equals shaft speed exactly, at any angle: this is what "constant
  velocity" means.
- **CVJ-05**: In the tripod joint, three rollers on a three-armed spider fixed to the shaft run in three straight
  axial tracks in the differential-side housing (the tulip). The rollers roll and slide along the tracks, allowing
  both angle and plunge with essentially constant velocity.
- **CVJ-06**: The shaft is rigid, but the distance between the joint centres changes as the wheel moves. For pure
  vertical travel of ±60 mm it grows by √(505² + 60²) - 505 = **3.55 mm**, and the tripod rollers slide that far in
  their tracks. The real figure depends on the arcs of the suspension links.
- **CVJ-07**: Shaft speed equals side-gear speed equals wheel speed. Shafts and wheels turn fwd-roll when the car
  moves forward.

## 17. Wheels and road speed (RD)

- **RD-01**: Road speed is v = ω_wheel × r, with r = 0.305 m, the dynamic rolling radius. It is smaller than the
  0.316 m unloaded radius because the tyre flattens at the contact patch. One wheel turn covers 1.916 m, and the
  wheels turn at 870 rpm at 100 km/h.
- **RD-02**: Engine speed and road speed are related by v = (2π·n_e/60)/(i × 4.10) × 0.305 m:

  | Gear | i | Overall | Output rpm at 3000 | Wheel rpm at 3000 | km/h at 3000 | km/h at 850 idle | km/h at 6800 | rpm at 100 km/h |
  |---|---|---|---|---|---|---|---|---|
  | 1 | 3.484 | 14.29 | 861 | 210.0 | **24.1** | 6.8 | 54.7 | (12 424) |
  | 2 | 2.075 | 8.51 | 1446 | 352.6 | **40.5** | 11.5 | 91.9 | (7400) |
  | 3 | 1.391 | 5.70 | 2157 | 526.0 | **60.5** | 17.1 | 137.1 | 4960 |
  | 4 | 1.000 | 4.10 | 3000 | 731.7 | **84.1** | 23.8 | 190.7 | 3566 |
  | 5 | 0.815 | 3.34 | 3682 | 898.0 | **103.3** | 29.3 | 234.1* | 2905 |
  | R | -3.410 | -13.98 | -880 | -214.6 | **-24.7** | -7.0 | n/a | n/a |

  Values in parentheses are above the redline. *234.1 km/h is only the speed the gearing allows; aerodynamic drag
  limits the real top speed.
- **RD-03**: Tractive force at the tyres per 100 N·m of engine torque is F = T·i·4.10/0.305, ignoring losses:
  4684, 2790, 1870, 1344 and 1095 N in 1st to 5th. That is why low gears are used to pull away. **(context)**
- **RD-04**: Grip at the contact patch is a friction force: the tyre pushes the road backward and the road pushes the
  car forward. Without grip the wheel would spin and the car would not move.

## 18. Rotation directions (ROT)

- **ROT-01**: Direction and speed of every member, in a forward gear with the car moving forward:

| Member | Direction | Speed relative to the crank (gear i) |
|---|---|---|
| Crank, flywheel, clutch cover, pressure plate, diaphragm spring | CW-F | 1 |
| Intake and exhaust camshafts, timing chain loop | CW-F | 0.5 |
| Clutch disc, input shaft, input gear (clutch locked) | CW-F | 1 |
| Countershaft and all its gears | **CCW-F** | 0.743 |
| Free output gears for 1st, 2nd, 3rd and 5th | CW-F | 1/i of that gear |
| Reverse idler | CW-F | 0.506 |
| Reverse output gear | **CCW-F** | 1/3.410 |
| Output shaft, synchro hubs, sleeves, blocker rings | CW-F (CCW-F in reverse) | 1/i |
| Propshaft, pinion | same as output shaft | 1/i |
| Ring gear, differential case | fwd-roll (backwards in reverse) | 1/(4.1·i) |
| Side gears, driveshafts, CV joints, hubs, wheels | fwd-roll | equal to the case going straight; ×0.852 / ×1.148 at R = 5 m |
| Spider gears | orbit with the case; spin on the pin only in a turn | (ω_R - ω_L)/2 × 1.6 |
| Release bearing inner race | spins with the fingers when touching | 1 |
| Pedal, release fork, forks, rails, shift lever | pivot or translate only | - |

## 19. Presentation liberties (PRS)

- **PRS-01**: In slow motion, every angle integrates ω × slowmo(t) × dt. HUD rpm and km/h always show the physical
  values.
- **PRS-02**: s02 shows the 850 rpm idle at about 200 ×, so one crank turn takes 14.1 s on screen and one stroke
  7.06 s. A factor of **198.3 ×** makes each 7.0 s stroke beat show exactly 180° of crank.
- **PRS-03**: s05 shows 3000 rpm at 150 ×, which turns the crank 5° per frame. If 150 × holds through steps 1-5
  (43 s of video), the real shift takes 0.29 s, and the 14 s sync beat lasts 93 ms. That is a quick but plausible
  shift; typical shifts take 0.3-1 s.
- **PRS-04**: To avoid wagon-wheel strobing, a part with N-fold symmetry must turn less than half a pitch per frame:
  slowmo > (rpm/60) × 2N/24. Required slow-motion factors (values are for half a pitch; double them for a smooth
  quarter pitch):

  | Part | Teeth or symmetry | Speed | Minimum slowmo |
  |---|---|---|---|
  | Flywheel ring gear | 132 | 3000 rpm | 550 × |
  | Countershaft 38T gear | 38 | 2229 rpm | 118 × |
  | Input gear | 26 | 3000 rpm | 108 × |
  | Diaphragm fingers | 18 | 3000 rpm | 75 × |
  | 2nd gear | 37 | 1446 rpm | 74 × |
  | 1st output gear | 44 | 861 rpm | 53 × |
  | Ring gear / pinion | 41 / 10 | 210 / 861 rpm | 12 × |
  | Flywheel ring gear | 132 | 850 rpm | 156 × |
  | Timing sprockets | 21 / 42 | 850 / 425 rpm | 25 × |

  At 150 × and 3000 rpm the flywheel ring gear advances 1.83 teeth per frame and aliases, so keep it out of shot or
  blurred in s05.
- **PRS-05**: In real-time shots (the end of s07, and s08) the wheels turn 52.5° per frame at 24 km/h and 88° per
  frame at 40.5 km/h. Spokes strobe exactly as they would on a real 24 fps camera; Cycles motion blur makes this look
  natural.
- **PRS-06**: Cutaways, the x-ray or fading bodywork, exploded views, labels and the warm power-path glow are
  presentation only. They never change any part's motion.
- **PRS-07**: The gas colours (blue intake, orange combustion, grey-brown exhaust) are illustrative. A petrol flame is
  faint and bluish, and the gases are colourless.
- **PRS-08**: The model simplifies in these ways:
  - single-mass flywheel (many modern cars use a dual-mass one);
  - spiral-bevel final drive with no hypoid offset (FD-05);
  - no oil, seals or chain guides unless an assembly adds them.

  Clearances are true scale: about 0.5 mm per clutch face, and a 1.8 mm plate lift. If a scene magnifies them, the
  storyboard should say so.
- **PRS-09**: The tyre is modelled at a 0.3115 m radius while the motion uses r = 0.305 m, so the tread surface moves
  2.1% faster than the ground. This is invisible at normal viewing distance, and the 6.5 mm "squash" hides the
  contact.
- **PRS-10**: The hydraulic pulse along the line, the stroke strip, the firing ticker and the step cards are visual
  aids.

---

## 20. Narration line → fact IDs

Every sentence in `carviz/timeline.py`. The key is scene.beat.sentence number.

| Key | Narration sentence | Fact IDs |
|---|---|---|
| s01.car.1 | This is a front-engine, rear-wheel-drive car with a five-speed manual gearbox. | VEH-01, GBX-08 |
| s01.inside.1 | Let's look inside, at the parts that turn burning fuel into turning wheels. | VEH-02, CYC-12 |
| s01.path.1 | Power starts in the engine. | ENG-13, CYC-12 |
| s01.path.2 | It flows through the clutch and gearbox, along the propeller shaft to the differential at the rear axle, and out through two driveshafts to the wheels. | VEH-02, VEH-04, PRP-01, FD-01, DIF-01, CVJ-01 |
| s01.follow.1 | Let's follow it. | VEH-02 |
| s02.inline4.1 | The engine is an inline four: four cylinders in a row, each with a sliding piston. | ENG-01, ENG-04, ENG-05 |
| s02.crank.1 | Connecting rods link the pistons to the crankshaft, turning up-and-down motion into rotation. | ENG-05, ENG-06, ENG-08, ENG-12 |
| s02.cycle.1 | Each cylinder repeats a four-stroke cycle over two crankshaft turns. | ENG-10, ENG-11 |
| s02.intake.1 | Intake: the piston moves down, drawing air and fuel in through the open intake valves. | CYC-09, CYC-08, CYC-01, CYC-07 |
| s02.compression.1 | Compression: both valves close, and the rising piston squeezes the mixture. | CYC-10, CYC-04, CYC-01, ENG-03 (see N1) |
| s02.power.1 | Power: a spark ignites the mixture, and the hot gas forces the piston down. | CYC-11, CYC-12 |
| s02.exhaust.1 | Exhaust: with the exhaust valves open, the rising piston pushes the burnt gas out. | CYC-13, CYC-03 |
| s02.valvetrain.1 | Camshafts open the valves. | VLV-01, CYC-07 |
| s02.valvetrain.2 | A timing chain drives them from the crankshaft at exactly half its speed, because each valve opens only once every two turns. | VLV-02, VLV-03, VLV-04 |
| s02.firing.1 | Only the power stroke drives the crankshaft, so the cylinders take turns, firing in the order one, three, four, two: a power stroke every half turn. | ENG-09, ENG-13, ENG-11, ENG-08 |
| s02.flywheel.1 | At the back, a heavy flywheel smooths out the pulses. | FLY-01, ENG-13 |
| s02.flywheel.2 | Its face is one half of the clutch. | FLY-02, CLU-01 |
| s03.parts.1 | Between the engine and the gearbox sits the clutch: the flywheel, a friction disc, and a pressure plate with a diaphragm spring. | CLU-01, CLU-02, CLU-04 |
| s03.splines.1 | The disc's hub is splined to the gearbox input shaft, so they always turn together. | CLU-03 |
| s03.engaged.1 | With the pedal up, the spring clamps the disc between the pressure plate and the flywheel, and the gearbox turns with the engine. | CLU-04, GBX-12 (see N5) |
| s03.release.1 | Press the pedal, and fluid flows from the master cylinder to the slave cylinder. | HYD-01, HYD-02, HYD-05, HYD-06 |
| s03.release.2 | It pushes the release fork and bearing against the spring's fingers. | HYD-03, CLU-05, CLU-12 |
| s03.release.3 | The spring flexes, the pressure plate pulls back, and the disc is free. | CLU-05, CLU-06, CLU-08, CLU-09 |
| s03.slip.1 | To pull away, the pedal comes up slowly. | CLU-06, CLU-11 |
| s03.slip.2 | The disc slips against the flywheel, speeding up as it passes on torque, until it matches the engine and locks. | CLU-10, CLU-11 (see N6) |
| s04.shafts.1 | The gearbox has three shafts. | GBX-01, VEH-04 |
| s04.shafts.2 | The input shaft, driven by the clutch, turns the countershaft below. | GBX-07, GBX-01 |
| s04.shafts.3 | The output shaft runs out the back, in line with the input. | GBX-01, VEH-04 |
| s04.neutral.1 | The other countershaft gears each mesh with a gear on the output shaft. | GBX-03, GBX-04, GBX-13 (see N8) |
| s04.neutral.2 | They're always in mesh, but the output gears spin freely on bearings. | GBX-02, GBX-12 |
| s04.neutral.3 | Until one is locked to the shaft, no power gets through: neutral. | GBX-15 |
| s04.synchro.1 | Synchronizers do the locking. | SYN-01 |
| s04.synchro.2 | A hub is splined to the shaft, and a sleeve slides on it. | SYN-01 |
| s04.synchro.3 | Each gear carries a ring of dog teeth and a cone, with a brass blocker ring in between. | SYN-01, SYN-03 (see N7) |
| s04.lock.1 | Slide the sleeve over the dog teeth, and the gear is locked to the shaft. | SYN-05, SYN-07, CLU-09 |
| s04.linkage.1 | Forks on shift rails move the sleeves. | SEL-02, SEL-09 |
| s04.linkage.2 | Moving the lever sideways picks a rail; forward or back slides it. | SEL-01, SEL-03, SEL-04, SEL-05, SEL-07 |
| s04.ratios.1 | In first, a small countershaft gear drives a large output gear, so the engine turns about three and a half times for each turn of the output shaft. | GBX-08, GBX-04 (see N2) |
| s04.ratios.2 | Fourth locks input to output: one to one. | GBX-10 |
| s04.ratios.3 | Fifth is an overdrive. | GBX-11 |
| s04.reverse.1 | Reverse adds an idler gear between the shafts, so the output turns backwards. | REV-01, REV-02, GBX-14 |
| s05.intro.1 | Here's a shift from first to second at three thousand rpm, slowed right down. | SFT-01, PRS-03 |
| s05.clutch_in.1 | One: clutch in. | SFT-02 |
| s05.clutch_in.2 | The engine is disconnected. | SFT-02, SFT-03 |
| s05.neutral.1 | Two: the sleeve slides out of first, into neutral. | SFT-04, SEL-04 |
| s05.sync.1 | Three: the blocker ring's cone presses on second gear. | SFT-05, SYN-03, SYN-04 |
| s05.sync.2 | Friction slows it, along with the countershaft, input shaft and clutch disc, until it matches the output shaft's speed. | SFT-05, SYN-08 |
| s05.engage.1 | Four: speeds matched, the blocker ring lets the sleeve through, onto second gear's dog teeth. | SYN-05, SYN-06 |
| s05.clutch_out.1 | Five: clutch out. | SFT-08 |
| s05.clutch_out.2 | The engine is reconnected at about eighteen hundred rpm: same road speed, taller gear. | SFT-06, SFT-07, SFT-08 |
| s06.prop.1 | The propeller shaft carries the drive back to the rear axle. | PRP-01, PRP-02, PRP-03, PRP-04 |
| s06.ringpinion.1 | There, a small pinion drives a large ring gear: ten teeth against forty-one, a final drive ratio of 4.1 to 1, turning the drive through a right angle. | FD-01, FD-02, FD-03, FD-04 |
| s06.diffparts.1 | The ring gear is bolted to the differential case. | DIF-01 |
| s06.diffparts.2 | Inside, two spider gears mesh with two side gears, one splined to each driveshaft. | DIF-01 |
| s06.straight.1 | Going straight, the spider gears don't spin on their pin. | DIF-03 |
| s06.straight.2 | Everything turns as one, and both wheels match. | DIF-02, DIF-03 |
| s06.turn.1 | In a turn, the outer wheel travels further than the inner one. | DIF-06 |
| s06.turn.2 | The spider gears now spin on their pin, letting one side speed up as the other slows. | DIF-03, DIF-07, DIF-08 |
| s06.turn.3 | The case turns at the average of the two. | DIF-02, DIF-07 |
| s07.why.1 | Each driveshaft has a constant-velocity joint at each end, because the wheel moves up and down while the differential stays put. | CVJ-01, CVJ-02, VEH-07 |
| s07.rzeppa.1 | In the outer joint, six balls run in grooves between inner and outer races. | CVJ-03 |
| s07.rzeppa.2 | A cage holds them in the plane that splits the angle, so the wheel turns at exactly the shaft's speed. | CVJ-04 |
| s07.plunge.1 | The inner joint can also slide, as the shaft's length changes. | CVJ-05, CVJ-06 (see N3) |
| s07.moves.1 | Finally, the wheel turns, the tire grips the road, and the car moves. | RD-01, RD-04 |
| s08.together.1 | Let's put it all together. | VEH-02 |
| s08.first.1 | Clutch up in first, and the engine pulls to three thousand rpm. | CLU-11, RD-02 |
| s08.second.1 | Clutch in, second gear, clutch out: the revs drop, and the car keeps accelerating. | SFT-06, RD-02, RD-03 |
| s08.third.1 | Then third. | SFT-06 |
| s08.summary.1 | Engine, clutch, gearbox, final drive, differential, and driveshafts: one chain of gears and shafts, turning fuel into motion. | VEH-02 (see N4) |

## 21. Possible inaccuracies in the narration

Each proposed rewording was checked against `timeline.check()` rules (140 wpm, LEAD 0.25 s, 0.2 s tail) and fits its
beat. `timeline.py` has not been edited.

**Recommended changes**

1. **N1, s02.compression**: the narration says "Compression: both valves close". In fact the exhaust valves closed
   at 10° ATDC at the *start* of the intake stroke (CYC-01). Only the intake valves close here, 50° after BDC
   (CYC-04).
   Proposed: "Compression: the intake valves close, and the rising piston squeezes the mixture."
   (12 words, 5.6 s of 7.0 s.)
2. **N2, s04.ratios.1**: the narration says "a small countershaft gear drives a large output gear, **so** the engine
   turns about three and a half times". The 17→44 pair alone gives 2.59:1; 3.48:1 needs the 26→35 headset reduction
   (1.35:1) as well (GBX-08).
   Proposed: "In first, the input gear slows the countershaft, and a small countershaft gear drives a large output
   gear: the engine turns about three and a half times per output-shaft turn. Fourth locks input to output: one to
   one. Fifth is an overdrive." (42 words, 18.5 s of 20.0 s.)
3. **N3, s07.plunge.1**: the narration says "as the shaft's length changes", but the shaft is rigid. What changes is
   the distance between the joint centres, by 3.55 mm (CVJ-06).
   Proposed: "The inner joint can also slide, as the distance between the joints changes." (13 words, 6.0 s of 6.5 s.)
4. **N4, s08.summary.1**: the list omits the propeller shaft, which s06 presents as a chain link.
   Proposed: "Engine, clutch, gearbox, propeller shaft, final drive, differential, and driveshafts: one chain of gears
   and shafts, turning fuel into motion." (20 words, 9.0 s of 10.0 s.)

**Optional refinements**

5. **N5, s03.engaged.1**: the narration says "the gearbox turns with the engine". In this shot the gearbox is in
   neutral, so only the input side turns, not the output (GBX-12).
   Proposed: "With the pedal up, the spring clamps the disc between the pressure plate and flywheel, so the input shaft
   turns with the engine." (23 words, 10.3 s of 11.0 s.)
6. **N6, s03.slip.2**: the narration says the disc "slips against the flywheel". It slips against both driving faces,
   the flywheel and the pressure plate (CLU-01).
   Proposed: "...The disc slips between the flywheel and pressure plate, speeding up as it passes on torque, until it
   matches the engine and locks." (Whole beat 31 words, 13.7 s of 15.0 s.)
7. **N7, s04.synchro.3**: "with a brass blocker ring in between" does not say between what.
   Proposed: "Each gear carries dog teeth and a cone, with a brass blocker ring between cone and sleeve." (Whole beat
   34 words, 15.0 s of 16.0 s.)

**Acceptable as written (no change)**

8. **N8, s04.neutral.1**: "The other countershaft gears each mesh with a gear on the output shaft." The reverse
   countershaft gear meshes with the idler, not directly with the output gear (REV-03). Reverse is introduced two
   beats later, and the beat is already at its word limit (37 words, 16.3 s of 16.5 s).
9. **N9, s02.flywheel.2**: "one half of the clutch" is loose but fair, because the flywheel and pressure plate are
   the two clamping members (FLY-02).
10. **N10, s06.diffparts.2**: "one splined to each driveshaft". In this independent-suspension car the side gear is
    splined to the stub of the inner joint (DIF-01). The storyboard shows "driveshaft stubs", so this is consistent.
11. **N11, s05.clutch_out.2**: "about eighteen hundred rpm" matches 1787 rpm (SFT-06).
12. **N12, s05.sync.1**: "the blocker ring's cone presses on second gear" is correct: the blocker's internal cone
    presses on 2nd gear's cone.

**Storyboard (visual notes), not narration**

13. **V1, s04.ratios**: the HUD says "5th (0.82:1)". 0.8148 rounds to **0.81:1**; use 0.81 or 0.815.
14. **V2, s02**: "~200x". Use 198.3 × at 850 rpm so that each 7.0 s stroke beat is exactly 180° (PRS-02).
15. **V3, s05**: at "SLOW x150", any visible flywheel starter ring aliases (PRS-04).
16. **V4, s08**: "rpm drops to ~1790 … (40 km/h at 3000)" and "rpm drops to ~2010" are both correct (1787 rpm,
    40.5 km/h, 2011 rpm).

## 22. Notes for builders and the orchestrator (spec issues found)

- **O1, cam sprockets clash**: `CAM_CENTRE_SPACING` = 130 mm, but a 42T 3/8 in sprocket has a tip diameter of about
  132.8 mm (PD 127.5 mm). Two such sprockets side by side overlap by 2.8 mm, and since they turn the same way their
  teeth would collide. Fixes, in order of preference:
  1. Raise the spacing to at least about 0.137 m.
  2. Use a 20T/40T pair (tip diameter 126.7 mm, 3.3 mm gap), which also needs the storyboard labels changed.
  3. Drive the second cam through a gear pair.
- **O2, reverse pinion undercut**: the 15T spur at module 2.5 is below the 17.1-tooth limit. `gears.py` should apply
  profile shift x of about 0.12-0.2 to it (and -x to the idler, or adjust the idler position), or accept slight
  undercut (REV-05). The idler's face width must stay within the reverse plane (REV-03).
- **O3, clutch free play vs. slave stroke**: with no free play, the spec values form one consistent chain: 140 mm
  pedal → 23.33 mm master → 16.19 mm slave → 9.0 mm bearing → 1.8 mm lift. If 8% free play is modelled as lost
  pushrod stroke, full pedal gives slave 14.9 mm, bearing 8.28 mm and lift 1.66 mm. The difference is invisible.
  `state.py`, the HUD and the labels just need to use one convention. CLU-06 assumes lift is linear in
  (p - 0.08)/0.92 and reaches 1.8 mm at full pedal.
- **O4, ring gear side**: the differential assembly must put the ring gear on the **-X (left)** side of the pinion
  with its teeth facing +X. Otherwise the car drives backwards in forward gears (FD-04).
- **O5, shift linkage**: lever forward moves the rail rearward (SEL-04). Sideways is reversed too, so on a direct
  linkage the 1-2 rail's slot is at +X and the 5-R slot at -X (SEL-05). The finger moves only about 4.6 mm per plane
  (SEL-06).
- **O6, aliasing**: Track.validate() should use the table in PRS-04. The flywheel ring gear (132T) is the strictest
  part.
- **O7, tyre radius**: PRS-09 notes the 2.1% tread-speed mismatch. It can be ignored, or tread rotation can be scaled
  by 0.305/0.3115.

## Appendix: reproduce the key numbers

```python
import math, sys; sys.path.insert(0, ".")
from carviz import spec as S
r = S.GEAR_RATIOS
print(S.DISPLACEMENT_L, S.GEARBOX_CENTRE_DISTANCE * 1e3, S.SLAVE_STROKE * 1e3, S.RELEASE_FORK_RATIO)
print({g: round(v, 3) for g, v in r.items()}, S.FINAL_DRIVE)
print({g: round(S.road_speed_kmh(3000, g), 1) for g in (1, 2, 3, 4, 5)})
print("1->2 lands at", 3000 * r[2] / r[1])
print("prop joint angle", math.degrees(math.atan2(S.Z_CRANK - S.Z_PINION, S.Y_GEARBOX_REAR - S.Y_PINION_FLANGE)))
Ri, Ro = 5 - S.TRACK_REAR / 2, 5 + S.TRACK_REAR / 2
print("inner/outer", Ri / Ro, "inner/mean", Ri / 5, "outer/mean", Ro / 5)
L, rr = S.CONROD_LENGTH, S.CRANK_THROW
print("rod ratio", L / rr, "max rod angle", math.degrees(math.asin(rr / L)))
p = S.CHAIN_PITCH
print("42T tip dia mm", p * (0.6 + 1 / math.tan(math.pi / 42)) * 1e3, "vs cam spacing", S.CAM_CENTRE_SPACING * 1e3)
```
