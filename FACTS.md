# FACTS: every mechanical behaviour the film shows or narrates

This is the technical reference for *How a Manual Car Works*. Each fact has an ID, a precise statement and a short
justification. Every number comes from `carviz/spec.py`, `carviz/kin.py`, `carviz/state.py` or an assembly module
docstring (the as-built geometry) and was computed in Python (the formula is shown; the appendix reproduces the key
values). Facts marked **(context)** are not shown on screen. They explain or constrain something that is shown.

This revision matches the **as-built** 3D model and the **current** narration in `carviz/timeline.py`. Read the
"As-built model notes" first: they list what the model simplifies.

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
| Cam drive | 135.52 mm cam spacing, 126-link 3/8 in chain | chain moves z·p = 200.0 mm per crank turn |
| Valve lift law | flat-tappet three-arc cam, base R 18, nose R 5 mm | spec events and peak lifts (VLV-06) |
| Clutch release (working travel) | 21.47 mm × (15.87/19.05)² | master 21.47 → slave 14.90 mm → fork 115.4/69.7 = 1.655 → bearing 9.0 mm → plate 1.8 mm (diaphragm 4.24:1 while lifting). Free play 1.87 mm before the master piston |
| Gear centre distance | (2.25/cos 25°) × 61/2 | 75.72 mm |
| Gear ratios | (35/26) × z_out/z_cs | 3.484 / 2.075 / 1.391 / 1.000 / 0.815 / R -3.410 |
| Synchro | 0.25 × 360°/32; sleeve stations | index 2.81°; sleeve meets dogs at 5.51 mm; 3.0 mm dog engagement at 8.5 mm |
| Final drive | 41/10 | 4.10 |
| Overall ratio | i × 4.10 | 14.29 / 8.51 / 5.70 / 4.10 / 3.34 / R -13.98 |
| Road speed at 3000 rpm | (2π·3000/60)/(i·4.1) × 0.305 | 24.1 / 40.5 / 60.5 / 84.1 / 103.3 km/h, R -24.7 |
| 1→2 shift at 3000 rpm | 3000 × 2.075/3.484 | lands at 1787 rpm (2→3: 2011) |
| Propshaft joint angle | atan(55/1192) | 2.64° per joint (Z arrangement, joint centres 44 mm inside the flanges) |
| Driveshaft | 0.655 - 0.1771 | joint spacing 0.478 m; at ±60 mm: angle asin(60/478) = 7.2°, plunge 3.78 mm |
| Turn, R = 5 m | (5 ∓ 0.74)/5 | inner 0.852 ×, outer 1.148 × case speed (inner/outer 0.742) |

---

## As-built model notes (AB)

Simplifications a car engineer should know before reviewing the film. None of them changes a narrated statement.

- **AB-01, kinematic car, dynamic engine and clutch.** Road speed is a keyframed input (`Program.speed_kmh`); the
  wheels roll without slip and everything from the output shaft to the wheels follows kinematically. Only the engine
  and the input side are integrated: engine torque = 4 N·m/(rad/s) × (target - speed), clamped to -35 … +190 N·m, on
  I_ENGINE = 0.18 kg·m²; a Coulomb clutch of capacity `clutch_capacity(pedal)` × 350 N·m; in neutral the input side
  is I_INPUT = 0.012 kg·m² with a 2 s oil-drag spin-down. Engine torque never changes the car's speed.
- **AB-02, uniform crank speed.** There is no cyclic speed ripple from the individual power strokes. A real idle
  ripple of 2-4% would move the crank by less than ±1° from uniform rotation (ε/2 rad for a ripple at twice crank
  frequency), so it would be invisible anyway. The flywheel's smoothing role is narrated, not simulated.
- **AB-03, synchroniser as a speed blend.** Between cone contact and the end of the blocking hold, `state.py` blends
  the input-side speed to the target with a cosine ramp; it does not integrate a cone torque. It then indexes the
  gear so the dogs meet exactly aligned and flags any clash or misalignment.
- **AB-04, single-mass flywheel** (300 × 30 mm solid disc with a 132T starter ring), no dual-mass flywheel or
  torsional damper on the crank other than the pulley damper shown.
- **AB-05, flat-tappet three-arc cam** (VLV-06). It meets spec's opening/closing angles and peak lifts exactly, but
  it has no clearance ramps, and the follower acceleration jumps at the arc junctions. Production cams use smooth
  polynomial profiles. The cam outline is drawn 0.05 mm inside the true profile so cam and bucket never intersect.
- **AB-06, chain at mean speed.** The chain advances exactly z·p per sprocket turn (VLV-07); the 1.1% chordal
  (polygon) speed ripple of the 21T sprocket is not modelled. Guides and the hydraulic tensioner are shown static.
- **AB-07, fixed 15° spark advance** at all speeds and loads (CYC-11).
- **AB-08, no hypoid offset.** Spiral-bevel final drive with the pinion axis through the ring axis (FD-05).
- **AB-09, propshaft.** One piece, 65 × 1.8 mm tube, Hooke joint at each end, slip yoke under a boot at the front.
  The slip spline never slides because the gearbox and axle are rigidly placed (no engine rock). A real shaft of
  this length is close to its whirl speed (PRP-06).
- **AB-10, short dog engagement.** At full sleeve travel (8.5 mm) the sleeve overlaps the 4.5 mm dog teeth by
  3.0 mm (ridge past ridge). That is on the short side for a production box. The dogs have no back-taper (SYN-07),
  there are no interlock pins (the state validator enforces the interlock logically, SEL-07) and no reverse lockout.
- **AB-11, light contacts drawn with a gap.** Touching parts are separated by 0.04 mm in the clutch (facings,
  fingers/bearing, fork contacts) and 0.05 mm at the cams, so collision checks stay clean. Example: the disc shows
  0.615 mm per face at full release where the physical figure is (1.8 - 0.65)/2 = 0.575 mm. After the free play the
  pedal pushrod sits 0.04-0.21 mm off the master piston, because the pushrod follows the pedal arc while `kin`
  moves the piston linearly.
- **AB-12, suspension.** The rear wheel moves purely vertically; the rear links are aimed at their joints and
  stretched by up to 2.5% instead of swinging the wheel on arcs, so there is no camber or toe change. The front is a
  MacPherson strut with exact rigid kinematics (KPI 14°, no caster); the steering arms are within 12 mm of ideal
  Ackermann rack travel at R = 5 m, with the difference hidden in the rack housing.
- **AB-13, tripod joint.** The spider centre is kept on the tulip axis; a real tripod's centre orbits about
  0.1 mm at three times shaft speed (CVJ-05).
- **AB-14, tyre radius.** The tyre mesh is 0.3115 m but the motion uses r = 0.305 m, so the tread surface moves
  2.1% faster than the ground (PRS-09).
- **AB-15, cutaways.** Housings are cut at build time by fixed planes, with matte signal-red section faces
  (`section_cut`). Moving parts stay whole, except where the cut lies in a plane their motion preserves (the engine's
  'cyl1' planes y = const), where the cut turns with the part like a motorised training cutaway (gearbox synchro
  quarter sections, the CV joints' own-frame cuts), or where a live boolean holds a fixed section on a spinning part
  (s03's clutch 'section_rotating' plus the flywheel and ring gear, s05's quarter section of the 1-2 synchroniser,
  s07's joint sections; PRS-15).
- **AB-16, slow motion.** Every angle integrates ω × slowmo × dt, and the HUD always shows the physical rpm and km/h
  and the slow-motion factor. The factors actually used are in PRS-13. Keyframed suspension motion is in video
  time. s07's body heave (all four wheel centres ±60 mm relative to the body, tyres on the road) has a **9 s
  video period, 0.11 Hz of video time**; at the 12 × in force that is 0.75 s, **1.33 Hz of real time**, inside the
  1-1.5 Hz body-bounce mode of a car. The tripod plunge (CVJ-06) cycles at twice that: 2.67 Hz real, 0.22 Hz video.
  Positions are unaffected by the time base.

## 1. Vehicle layout (VEH)

- **VEH-01**: The car is a front-engine, rear-wheel-drive (FR) left-hand-drive saloon with a longitudinal inline-4
  and a 5-speed manual gearbox. Wheelbase is 2.620 m, track is 1.470 m front and 1.480 m rear, and tyres are
  205/55 R16.
  *Why:* spec "Vehicle package" (BMW 3-series / GT86 class). In an FR car the front wheels steer and the rear wheels
  drive. Under acceleration, weight moves rearward onto the driven wheels.
- **VEH-02**: The power path, in order, is: engine → clutch → gearbox → propeller shaft → final drive (pinion + ring
  gear) → open differential → two driveshafts (each with an inner tripod joint and an outer Rzeppa joint) → rear
  wheels. The front wheels are not driven.
  *Why:* this is the FR layout; each assembly publishes this order in `meta['power_path']`.
- **VEH-03**: The engine, clutch and gearbox share one level axis on the car's centre line (X = 0, Z = 0.360 m). The
  block spans Y = +0.130 to -0.292 m, so 69% of its length is behind the front-axle line.
  *Why:* 0.292/(0.130+0.292) = 0.692. Because the front axle is not driven, most of the engine can sit behind it,
  which moves mass toward the middle of the car.
- **VEH-04**: A three-shaft gearbox (input / countershaft / output) suits a longitudinal RWD car for four reasons:
  - Input and output are coaxial, so the drive goes straight through to a propshaft on the centre line.
  - One gear (4th) can be direct, at 1:1 with no gear mesh loaded.
  - The output turns the same way as the engine.
  - Two reductions in series (1.346 × 2.588) reach 3.48:1 in 1st without a tiny pinion or a huge gear.

  *Why:* see GBX-10 and GBX-14. With a 61-tooth sum, a single 3.48:1 mesh would need about 13.6 : 47.4 teeth, close
  to the 13.1-tooth undercut limit (GBX-05).
- **VEH-05**: Most front-wheel-drive cars mount the engine transversely and use a transaxle with no countershaft.
  The input and output shafts are parallel and side by side, and each gear uses one mesh. The output shaft's pinion
  drives the differential's ring gear inside the same case. **(context)**
- **VEH-06**: The gearbox output flange (Y -1.120, Z 0.360) is 55 mm higher than the pinion flange (Y -2.400,
  Z 0.305). The flanges are 1.281 m apart; the Hooke-joint centres, 44 mm inside each flange, are 1.193 m apart.
  *Why:* spec Y_GEARBOX_REAR, Y_PINION_FLANGE, Z_PINION = WHEEL_CENTER_Z; axle assembly JOFF = 0.044. See PRP-02.
- **VEH-07**: The differential is fixed to the body (independent rear suspension). Only the wheel hubs move
  relative to it (±60 mm in s07, where the body heaves over wheels that stay on the road, AB-16), so each driveshaft
  needs a joint at both ends.
  *Why:* spec Y_DIFF/Z_DIFF are fixed, and SUSPENSION_TRAVEL = 0.060.

## 2. Engine (ENG)

- **ENG-01**: The engine is a 2.0 L DOHC 16-valve port-injected petrol inline-4 with compression ratio 10.5:1, idle
  850 rpm and redline 6800 rpm. Its four cylinders sit in a row along the car, with cylinder 1 at the front.
- **ENG-02**: Swept volume per cylinder is V_s = π/4·B²·S = π/4 × 86² × 86 mm³ = 499.6 cm³, so the total is
  1998 cm³. Bore equals stroke (a "square" engine).
- **ENG-03**: Clearance volume is V_c = V_s/(CR - 1) = 499.6/9.5 = 52.6 cm³. At TDC the piston crown is 0.8 mm
  below the deck (DECK_HEIGHT = 43 + 145 + 31 + 0.8 = 219.8 mm). As built, a pent-roof chamber plus a 1.9 mm crown
  dish and a 1.2 mm gasket give CR 10.5. **(context)**
- **ENG-04**: Cylinder centres are at Y = 0.060, -0.034, -0.128 and -0.222 m. The pitch is 94 mm, leaving 8 mm of
  metal between the 86 mm bores.
- **ENG-05**: Each piston follows slider-crank motion:
  s(θ) = r + L - [r·cos θ + √(L² - r²·sin² θ)], with r = 43 mm and L = 145 mm (L/r = 3.372, λ = r/L = 0.297).
  The rod swings ±asin(r/L) = ±17.25°. The rod turns the up-and-down motion into rotation: the gas force along the
  rod has a tangential component at the crankpin, and that component makes torque. The torque is zero at TDC and BDC.
  *As built:* `kin.slider_crank` drives every piston and rod.
- **ENG-06**: Piston motion is not a sine wave:
  - At 90° ATDC the piston has already travelled 49.5 mm (57.6% of the stroke).
  - Mid-stroke comes at 81.5° ATDC.
  - Peak piston speed comes at 74.7° ATDC and is 1.64 × the mean speed.
  - The piston moves faster near TDC and dwells near BDC.
- **ENG-07**: Mean piston speed (2·S·n/60) is 2.44 m/s at 850 rpm, 8.6 m/s at 3000 and 19.5 m/s at 6800. Peak speed
  is 4.0, 14.1 and 31.9 m/s. Acceleration at TDC, r·ω²·(1 + λ), is 442, 5502 and 28 270 m/s² (about 2900 g at
  redline). **(context)**
- **ENG-08**: The crank is flat-plane. Crankpins 1 and 4 are at 0°, and 2 and 3 at 180°. Pistons 1 and 4 move
  together, and 2 and 3 move together in the opposite direction, so two pistons are at TDC while the other two are
  at BDC.
- **ENG-09**: The firing order is 1-3-4-2. Each cylinder reaches TDC on its power stroke at crank angle 0° (1),
  180° (3), 360° (4) and 540° (2), evenly spaced at 720°/4 = 180°. At each TDC one cylinder fires and its partner
  (1↔4 or 2↔3) is at the overlap TDC between exhaust and intake.
- **ENG-10**: In each cylinder's own angle φ = (θ - θ_fire) mod 720, the strokes are: power 0-180, exhaust 180-360,
  intake 360-540 and compression 540-720. One cycle takes two crankshaft turns.
- **ENG-11**: At any instant the four cylinders are in four different strokes. For 0 ≤ θ < 180: cylinder 1 is on
  power, 3 on compression, 4 on intake and 2 on exhaust. Every 180° each cylinder moves on to its next stroke.
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

  Each valve is open for 240 crank° (120 cam°). These are the zero-lift points of the as-built cam (VLV-06).
- **CYC-02**: Valve overlap runs from 350 to 370 (20°) around the TDC between exhaust and intake. Both valves are
  slightly open there: at TDC (360°) the intake valves are lifted 0.37 mm and the exhaust valves 0.30 mm
  (`kin.valve_lift`). This helps flush the chamber.
- **CYC-03**: The exhaust valves open 50° before BDC, near the end of the power stroke, when the piston is 86.5% of
  the way down. This blowdown releases the remaining pressure so the piston does not have to push against it. Little
  work is lost because the crank is near BDC and has little leverage. The cam lifts the exhaust valves 0.30 mm by
  140° and 2.56 mm by 160°.
- **CYC-04**: The intake valves close 50° after BDC because the moving charge keeps flowing in after BDC. At BDC
  (540°) they are still 5.43 mm open; they drop below 1 mm at 573.5° and close at 590°. Real compression only
  begins at IVC, after the piston has risen 11.6 mm; the effective compression stroke is 74.4 mm.
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
  36 mg of fuel per cylinder per cycle. A direct-injection engine would take in air only.
- **CYC-09**: Intake (φ 360-540): the descending piston lowers cylinder pressure below manifold pressure.
  Manifold pressure pushes the mixture in through the open intake valves; this is the "drawing in". The throttle sets
  manifold pressure: about 1 bar at full throttle, but only about 0.3-0.4 bar absolute at the 850 rpm idle shown in
  s02.
- **CYC-10**: Compression (540-720): once the intake valves have closed (590; the exhaust valves closed at 370), the
  rising piston squeezes the mixture into the clearance volume. The geometric ratio is 10.5:1.
- **CYC-11**: The spark comes 15° BTDC (cycle 705°, the end of compression). Burning takes time, so ignition leads
  TDC and peak pressure arrives shortly after TDC. 15° is 2.94 ms at 850 rpm and 0.83 ms at 3000 rpm.
  *Why:* SPARK_ADVANCE_DEG. A real ECU varies the advance: about 30-40° BTDC at light-load cruise, knock-limited to
  roughly 10-25° at full load. 15° is a representative idle value, which is where the film shows the spark (s02,
  850 rpm). The film uses it everywhere (AB-07).
- **CYC-12**: Power (0-180): the pressure of the burning gas drives the piston down. This is the only stroke that
  does work.
- **CYC-13**: Exhaust (180-360): the rising piston pushes the burnt gas out past the open exhaust valves.

## 4. Valvetrain drive (VLV)

- **VLV-01**: Two overhead camshafts (DOHC) open the valves. Each lobe pushes its valve open through a flat bucket
  tappet (37 mm intake, 36 mm exhaust), and the valve spring closes it again. Each camshaft carries 8 lobes, two per
  cylinder.
- **VLV-02**: The camshafts turn at exactly half crank speed: 21T crank sprocket / 42T cam sprocket = 0.5, so
  425 rpm at 850 rpm idle. This is necessary because each valve opens once per 720° cycle, which is once per cam
  turn.
- **VLV-03**: A chain does not reverse rotation. Both camshafts turn CW-F like the crank, because every sprocket sits
  inside the chain loop, with the chain wrapped around its outside. The fixed guide (tight side) and the tensioner
  arm (slack side) touch the outside of the loop but do not rotate.
- **VLV-04**: The chain is single-row 3/8 in (9.525 mm, 06B) roller chain. Sprocket pitch diameter is
  PD = p/sin(180°/z): 63.91 mm for the 21T crank sprocket and 127.46 mm for the 42T cam sprockets, whose tip
  diameter is p·(0.6 + cot(180°/z)) = 132.82 mm. As built, the cam centres are **135.52 mm** apart (spec
  CAM_CENTRE_SPACING), so the two cam sprockets clear by 2.70 mm. That is the smallest spacing at which they clear
  and the loop closes on a whole, even number of links, **126** (1200.15 mm), with a 6 mm tensioner push.
- **VLV-05**: Lobes of successive cylinders in the firing order are 90 cam° apart (180 crank°/2). **(context)**
- **VLV-06**: **Cam profile.** A flat tappet can only follow a convex cam, so the lift law comes from cam
  geometry (`kin.cam_geometry`, `kin.cam_support`, `kin.valve_lift`): a classic three-arc cam with base circle radius
  R_b = 18 mm, nose radius r_n = 5 mm and flank arcs of radius r_f = 114.4 mm (intake) / 96.8 mm (exhaust), tangent
  to both. Lift = h(β) - R_b, where h is the cam's support function (distance from cam axis to the bucket face) and
  β = (crank angle from peak)/2. Each lobe opens over ±60 cam° (240 crank°) and peaks at spec's 9.5 / 9.0 mm.
  - Lift is above 1 mm for 207 crank° (intake) and 203.5° (exhaust).
  - The contact point moves up to 17.2 mm (intake) / 16.4 mm (exhaust) off the bucket centre, inside the 18.5 /
    18.0 mm bucket radius. Valve velocity is ω_cam × that offset: at most 6.1 m/s at 6800 rpm.
  - Follower acceleration is ω_cam²·(r_f - R_b) on the flanks (191 m/s² at idle, about 12 200 m/s² at redline for
    the intake) and ω_cam²·(r_n - R_b - L) on the nose (-2850 m/s² at redline). It steps from zero at opening
    (AB-05).

  The engine assembly's lobes are exactly this cam, so the bucket rides on the lobe at every angle.
- **VLV-07**: **Chain travel.** A roller chain advances exactly z·p per sprocket turn (`kin.chain_travel`):
  21 × 9.525 = 200.0 mm per crank turn, i.e. 2.83 m/s at idle and 22.7 m/s at 6800 rpm. Using the pitch-circle arc
  π·PD instead would overstate the travel by 0.37% (0.75 mm per crank turn) and let the rollers creep off the teeth.
  The rollers therefore run on R_eff = z·p/2π. The 126-link loop passes a given crank tooth every 6 crank turns.

## 5. Flywheel (FLY)

- **FLY-01**: A heavy single-mass flywheel (300 mm diameter, 30 mm thick) is bolted to the crank's rear flange. It
  stores kinetic energy: it absorbs the surplus of each power pulse and returns it between pulses (two pulses per
  turn), which smooths crank speed. A solid steel disc this size would weigh 16.6 kg and have I = m·r²/2 =
  0.187 kg·m², an **upper bound**; real single-mass flywheels for 2.0 L engines are about 8-11 kg and
  0.09-0.13 kg·m². With the crank, damper, clutch cover and pressure plate the rotating group comes to about
  0.15-0.20 kg·m², so `state.py`'s I_ENGINE = 0.18 kg·m² for that whole group is plausible. It stores 713 J at
  850 rpm (½·I·ω², ω = 89.0 rad/s). All illustrative. The model's crank speed has no per-pulse ripple (AB-02).
- **FLY-02**: The flywheel's rear face is one of the clutch's two driving friction faces, and the clutch cover is
  bolted to it. The flywheel, cover, diaphragm spring and pressure plate therefore always turn at engine speed.
- **FLY-03**: The rim carries a 132-tooth starter ring gear. With a typical starter-ring module of about 2.25 mm, the
  PD is 297 mm and the tip diameter 301.5 mm, consistent with the 300 mm flywheel. The starter pinion meshes only
  while cranking. **(context)**
- **FLY-04**: A spigot (pilot) bush in the crank's rear end supports the nose of the gearbox input shaft. As built
  it is a brass bush around the 15 mm (r 7.5 mm) pilot, which passes through the flywheel centre. **(context)**

## 6. Clutch (CLU)

- **CLU-01**: The clutch is a single dry plate. From front to rear: flywheel face → friction disc → pressure plate →
  diaphragm spring (inside a cover bolted to the flywheel) → release bearing. The driving members are the flywheel,
  cover, spring and pressure plate. The driven member is the disc.
- **CLU-02**: The disc has these parts:
  - Facings 228/150 mm in diameter on both sides: n = 2 friction surfaces of 23 157 mm² each.
  - Cushion segments between the facings (clamped thickness 8.4 mm; as built 12 wavy segments, 0.65 mm travel).
  - Six torsional damper springs between the facings and the hub.
  - A 23-tooth splined hub (25.4 × 21.5 mm splines, shared with the input shaft).

  The damper springs soften engagement shocks and engine torsional vibration.
- **CLU-03**: The disc hub is splined to the input shaft, so the two always turn together (both are driven by
  `theta_in`, splines in phase). The disc can still slide axially, so the clamp load acts equally on both faces and
  the disc floats free when released (it floats by half the plate lift).
- **CLU-04**: Engaged (pedal up): the diaphragm spring, a dished Belleville spring, pushes the pressure plate forward
  and clamps the disc against the flywheel. Torque capacity is T_c = n·μ·F·r_m, with n = 2 and
  r_m = (114 + 75)/2 = 94.5 mm (uniform wear). While engine torque is below T_c nothing slips.
  *Model:* T_CLUTCH_MAX = 350 N·m, 1.84 × the 190 N·m engine maximum. With μ = 0.3 that needs a clamp load
  F = 350/(2 × 0.3 × 0.0945) ≈ 6.2 kN, inside the typical 4-7 kN range.
- **CLU-05**: The release is push-type. The release bearing pushes the diaphragm finger tips **forward** (+Y, toward
  the flywheel). The spring pivots on two fulcrum wire rings in the cover (r = 90 mm), so its outer rim moves
  **rearward**, and strap springs pull the pressure plate rearward (-Y), unclamping the disc. As built:
  - p = 0.08-0.22: the fingers bend 1.37 mm with the rim still (shape key 'bend'), see CLU-06.
  - p = 0.22-1.0: the spring rotates rigidly about the fulcrum (about 6.9°, shape key 'release'): fingers 7.63 mm,
    rim and plate 1.80 mm, an axial lever ratio of **4.24 : 1**. The bearing contact radius (27.3 mm) is solved so
    this holds; the radial arm ratio (90 - 27.3)/(104.2 - 90) = 4.42 is reduced to 4.24 by the 12° cone and the
    2.3 mm spring thickness.

  Spec's DIAPHRAGM_LEVER_RATIO = 9.0 : 1.8 = 5.0 is an end-to-end travel ratio that includes the 1.37 mm of bending;
  the model does not use it.
- **CLU-06**: Pedal to plate mapping (`kin.clutch_geometry`), p = pedal fraction from the top. Master piston =
  21.47 mm × (p - 0.08)/0.92; slave = master × (15.87/19.05)²; bearing = slave/1.655 = 9.0 mm × (p - 0.08)/0.92;
  plate lift = 1.8 mm × (p - 0.22)/0.78; capacity = 1 - smoothstep(0.22, 0.50, p).

  | Pedal fraction p | Pedal travel | Master / slave | Bearing | Plate lift | Capacity (× 350 N·m) | Clutch |
  |---|---|---|---|---|---|---|
  | 0 to 0.08 | 0 to 11.2 mm | 0 / 0 (pushrod closes its 1.87 mm free play) | 0 | 0 | 100% | free play |
  | 0.22 | 30.8 mm | 3.27 / 2.27 mm | 1.37 mm | 0 | 100% (350 N·m) | capacity starts to fall (going down); full clamp regained (coming up) |
  | 0.352 | 49.3 mm | 6.35 / 4.40 mm | 2.66 mm | 0.30 mm | 54% (190 N·m) | from here down, full engine torque would slip it |
  | 0.36 | 50.4 mm | 6.53 / 4.53 mm | 2.74 mm | 0.32 mm | 50% (175 N·m) | slipping zone |
  | 0.50 | 70 mm | 9.80 / 6.80 mm | 4.11 mm | 0.65 mm | 0% | **bite point** |
  | 1.00 | 140 mm | 21.47 / 14.90 mm | 9.0 mm | 1.8 mm | 0% | full pedal |

  The engagement zone is p = 0.22-0.50 (spec CLUTCH_BITE_LO/HI). Pulling away, the driver feels the bite at the
  **0.50** end, where capacity first rises from zero; it reaches 10% (35 N·m, enough to creep) at p = 0.445.
  *Why:* from p = 0.08 to 0.22 the bearing loads the fingers. In a real clutch, finger bending and the elasticity
  of the cover and release parts take up this travel, and the clamp load starts to fall at once; the model keeps
  plate lift 0 and capacity 100% there because 350 N·m is far above the 190 N·m engine maximum. From p = 0.22 to
  0.50 the plate lifts 0.65 mm, the cushion-spring travel, while clamp load fades to zero (the disc's 'free' shape
  key opens the cushion). The remaining 1.15 mm is running clearance: **0.575 mm per face** at full pedal
  (0.615 mm as drawn, AB-11).
- **CLU-07**: The diaphragm spring's falling force curve keeps the clamp load nearly constant as the facings wear,
  and pedal effort drops once past its peak. **(context; standard practice)**
- **CLU-08**: Released with the gearbox in neutral: nothing drives the disc. The disc, input shaft, countershaft and
  free gears coast down slowly against oil churning and bearing drag (`state.py`: 0.012 kg·m², time constant 2 s),
  and the engine keeps idling.
- **CLU-09**: Released with 1st selected and the car stationary: the synchro forces 1st gear to the output-shaft
  speed, which is 0 rpm. 1st gear meshes with the countershaft, and the countershaft meshes with the input gear, so
  the countershaft, input shaft and disc stop too. The flywheel and pressure plate keep turning at 850 rpm past the
  released disc, transmitting essentially no torque.
- **CLU-10**: While the clutch slips, the torque it passes is the friction torque n·μ·F·r_m, set by the clamp load
  (that is, by pedal position) whatever the speed difference. The same torque brakes the engine. Heat generated is
  T × Δω; for example, 60 N·m at 1000 rpm of slip is 6.3 kW.
  *Model:* Coulomb friction: while slipping, `state.py` transmits capacity × sign(slip).
- **CLU-11**: Pulling away, the clutch locks in four steps:
  1. As the pedal rises, clamp load and clutch torque rise.
  2. Once clutch torque × overall ratio exceeds the resistance at the wheels, the car accelerates, and the disc with
     it.
  3. The engine needs extra throttle so it does not slow down.
  4. When disc speed reaches engine speed, slip stops. Static friction then holds the clutch locked, because capacity
     exceeds the transmitted torque.

  *Model:* the clutch locks when the slip speed crosses zero and stays locked while the torque needed to hold both
  sides together is within capacity; the validator flags an engine below 450 rpm as a stall. Locking therefore
  usually happens with the pedal still partly down (CLU-13).
- **CLU-12**: The hydraulic release is self-adjusting. A light preload spring in the slave keeps the release bearing
  in constant contact with the diaphragm finger tips, pedal up or down (HYD-05). The bearing's rotating race (the one
  touching the fingers) therefore always turns with the cover at engine speed (clutch option race_spin 'always').
  The other race, held by the fork and carrier, does not rotate.
- **CLU-13**: **HUD clutch status** (`state.py`, `Track.status`), shown in s03, s04 ("CLUTCH …"), s05 (which shows
  SYNCHRONIZING while the cone works) and s08:
  - **DISENGAGED** whenever capacity < 2% (p > 0.477), even if engine and disc speeds still differ or the car has
    already started to creep. In the s03 take-off the car starts to roll at 51.42 s but the status reads DISENGAGED
    until 51.92 s (0.06 s real).
  - **ENGAGED** means the clutch is locked (engine speed = disc speed). That can happen with the pedal still partly
    down, because capacity already exceeds the torque needed: s03 locks at p = 0.41 (25% capacity, 89 N·m; the
    clutch passed up to 95 N·m while slipping and holds 70 N·m once locked); s08 locks at p = 0.42 / 0.44 / 0.45
    in 1st / 2nd / 3rd. The pedal then comes up the rest of the way with nothing slipping.
  - **SLIPPING** otherwise (capacity ≥ 2% and the speeds differ).
  - **ENGINE OFF** when the engine is stopped (s01; the status is not shown there).

## 7. Hydraulic release (HYD)

- **HYD-01**: The pedal lever ratio is 6:1: the pedal pushrod is pinned 50 mm from the pivot on the 300 mm arm, so
  140 mm at the pad becomes 23.33 mm of pushrod stroke.
- **HYD-02**: Brake fluid is incompressible, so the volume the master piston displaces enters the slave cylinder:
  A_m·x_m = A_s·x_s. The bores are 15.87 mm (5/8 in, A_m = 197.8 mm²) and 19.05 mm (3/4 in, A_s = 285.0 mm²).
  Over the working travel the master piston moves 21.47 mm and displaces 4246 mm³ (4.2 mL), and the slave moves
  x_s = 21.47 × (15.87/19.05)² = **14.90 mm** (spec MASTER/SLAVE_WORKING_STROKE). Spec's MASTER_STROKE 23.33 mm and
  SLAVE_STROKE 16.19 mm are pushrod-stroke figures that ignore the free play; the model does not use them.
- **HYD-03**: The slave cylinder pushes the release fork, a **first-class lever** that turns about a vertical axis on
  a ball stud inside the bellhousing. As built (spec SLAVE_CYL_POS, RELEASE_FORK_PIVOT): pivot at X = -0.0697 m,
  bearing contact line on the crank axis (arm 69.7 mm), slave pushrod at X = -0.1851 m (arm 115.4 mm). The ratio is
  115.4/69.7 = **1.655** = 14.90/9.0 (spec RELEASE_FORK_RATIO_EFFECTIVE). The slave pushes its end of the fork
  **rearward** (-Y) and the bearing moves **forward** (+Y). Both ends follow arm × sin(fork angle), so the ratio is
  exact at every angle (up to asin(9.0/69.7) = 7.4°). Spec RELEASE_FORK_RATIO = 16.19/9.0 = 1.80 ignores the free
  play and is not used.
- **HYD-04**: Motion ratios, end to end, are pedal : bearing = 140/9.0 = 15.6 and pedal : pressure plate =
  140/1.8 = 77.8. Over the working travel, pedal : bearing is 128.8/9.0 = 6 × 1.441 × 1.655 = **14.3**. Only that
  pedal : bearing ratio is a force ratio: bearing load ≈ 14.3 × pedal force, minus friction. The driver's force
  never reaches the pressure plate. The bearing holds the diaphragm, and the finger load times the diaphragm lever
  ratio (4.24, CLU-05) equals the rim load it holds off the plate; the strap springs lift the plate with only a small
  force. The pedal force therefore follows the diaphragm's non-linear release-load curve (rising, then falling;
  CLU-07). Illustrative: a 6.2 kN clamp load needs about 6.2/4.24 ≈ 1.46 kN at the fingers, about 100 N at the pad
  before friction and any assist spring.
- **HYD-05**: The first 8% of pedal travel (11.2 mm at the pad, 1.87 mm at the pushrod) does not move the bearing;
  this is free play. As built (and as in a real hydraulic release) all of it is **upstream** of the hydraulics: a
  1.87 mm gap between the pedal pushrod and the master piston at rest. Downstream there is no gap: the slave
  pushrod, fork and bearing stay in light contact, and the bearing touches the fingers at rest, which makes the
  release self-adjusting as the facings wear (CLU-12).
- **HYD-06**: Pressure travels along the line at the speed of sound in the fluid (on the order of 1 km/s), so the
  slave follows the pedal within milliseconds. The pulse animated along the line is a visualisation (PRS-10).

## 8. Gearbox (GBX)

- **GBX-01**: The gearbox has three shafts:
  - **Input shaft** (front): carries the 26T input gear, which has the 4th-gear dog teeth and cone. Its pilot runs
    in the crank spigot bush (FLY-04).
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
  1.42-1.45, plus an overlap ratio b·sin β/(π·m_n) = 0.60 per 10 mm of face width (as built 19-24 mm faces). Load
  passes smoothly from tooth to tooth, which makes the gears quiet and strong. The cost is axial thrust
  F_a = F_t·tan 25° = 0.47·F_t, carried by the bearings. The 17T pinion is not undercut: z_min = 2·cos β/sin² α_t =
  13.1 (α_t = 21.88°).
- **GBX-06**: Mating external helical gears have opposite hands. The input gear and all output-shaft gears have one
  hand, and all countershaft gears the other. **(context)**
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

  Torque is multiplied by the same factor, less roughly 1-1.5% per loaded mesh (about 2-3% for the two meshes of an
  indirect gear; almost nothing in direct 4th).
- **GBX-09**: Steps between gears are 1.679, 1.492, 1.391 and 1.227, getting closer toward the top. The total spread
  is 3.484/0.815 = 4.28. **(context)**
- **GBX-10**: 4th is direct. The 3-4 sleeve moves forward and locks the output shaft to the input gear's dog teeth,
  so output turns with input and no gear mesh carries torque. The countershaft still spins, unloaded.
- **GBX-11**: 5th is an overdrive (0.815). The large 38T countershaft gear drives the small 23T output gear, so the
  output turns 1/0.815 = 1.227 × faster than the engine.
- **GBX-12**: In neutral (car stationary, clutch engaged, 850 rpm), each free output gear turns at input/i: 1st
  244 rpm, 2nd 410, 3rd 611 and 5th 1043 rpm, all CW-F. The reverse gear turns 249 rpm CCW-F and the idler 431 rpm.
  The output shaft, synchro hubs and sleeves stand still, and the needle bearings let the gears turn on the
  stationary shaft.
- **GBX-13**: Axial order, front to rear: input gear (4th dogs) | 3-4 synchro | 3rd | 2nd | 1-2 synchro | 1st |
  intermediate web | reverse | 5-R synchro | 5th | rear wall, tail housing (shift mechanism), output flange.
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
- **REV-03**: The 15T and 38T gears sit on 75.72 mm centres and cannot touch: with the as-built profile shifts their
  tip radii are 21.62 + 50.38 = 72.00 mm, leaving a 3.72 mm gap. The idler bridges it.
  - The idler centre is 46.25 mm from the countershaft axis and 75.00 mm from the output axis (zero-sum shifts keep
    the standard centre distances): 14.8 mm above the countershaft axis (60.9 mm below the main axis) and 43.8 mm to
    the +X side (`kin.reverse_idler_centre`).
  - The idler (tip radius 29.6 mm) would hit 1st gear's 44T and 17T if it reached into their planes, so its face
    stays inside the reverse plane.
- **REV-04**: The reverse gears are spur rather than helical: no axial thrust, and the straight teeth cause reverse's
  typical whine. Contact ratios with the profile shifts are 1.52 (15/22) and 1.65 (22/38). **(context)**
- **REV-05**: The 15T spur pinion is below the 17.1-tooth no-undercut limit for a 20° pressure angle
  (2/sin² 20°). As built, the train is profile shifted x = +0.15 / -0.15 / +0.15 (15T / 22T / 38T), which lowers the
  15T's limit to 2(1 - x)/sin² 20° = 14.5 teeth; the idler's limit rises to 19.7, still below 22. Ratios are
  unaffected.
- **REV-06**: Reverse is engaged only with the car stationary. The 5-R synchro then brings the free reverse gear to
  the output's 0 rpm. With the car rolling forward, reverse would have to turn the whole gear train backwards.

## 10. Synchronisers (SYN)

- **SYN-01**: Three single-cone synchronisers sit on the output shaft: 1-2, 3-4 and 5-R. Each has:
  - a hub splined to the shaft;
  - a sleeve sliding on the hub's 32 external splines;
  - three spring-loaded struts;
  - on each side, a brass blocker ring whose internal cone rides on a steel cone on the gear, between the sleeve
    and that gear's cone, next to the gear's ring of 32 dog teeth (4.5 mm long, 120° roof chamfers).
- **SYN-02**: The sleeve travels 8.5 mm each way from neutral to engaged. As-built stations (gearbox assembly,
  spec SYNC_* fractions × 8.5 mm):

  | Sleeve position | What happens |
  |---|---|
  | 0-1.95 mm | struts travel with the sleeve to the blocker lugs |
  | 1.95-2.55 mm (**Contact**) | struts push the blocker 0.6 mm onto the cone |
  | 3.01 mm | sleeve tooth ridge level with the blocker ridge |
  | 3.83-3.87 mm (**Block**) | sleeve roof bears on the blocker roof, 0.85 mm deeper because the blocker is indexed 2.81°; held here while synchronising |
  | 5.11 mm | sleeve clears the blocker teeth |
  | 5.51 mm | sleeve ridge reaches the dog ridges |
  | 6.12 mm (**Through**) | 0.61 mm onto the dog chamfers; `state.py` counts the gear as rotationally locked from here |
  | ≥ 8.08 mm (**Engaged**, 95%) | counts as in gear |
  | 8.5 mm (**Home**) | 3.0 mm of dog engagement, ridge past ridge (AB-10) |
- **SYN-03**: Indexing: when the cones touch, friction drags the blocker ring round with the gear until its lugs hit
  the ends of their hub slots. That is a quarter of a tooth pitch (BLOCKER_INDEX = 0.25), 0.25 × 360°/32 =
  **2.81°**, in the direction the gear slips relative to the shaft (as built: 3 lugs in 3 hub slots with ±3.26°
  free play; the model indexes exactly 2.81°). With teeth about half a pitch wide, this brings the roof chamfers
  flank-to-flank against the sleeve's chamfers, a stable blocking position.
- **SYN-04**: Blocking: the shift force F presses the sleeve onto the blocker chamfers. This produces a cone friction
  torque T = μ·F·r_c/sin α. As built α = 6.5° (mean cone diameter about 52 mm), so 1/sin α = 8.8. The chamfer angle
  is chosen so that this friction torque exceeds the chamfers' turn-back torque while any slip remains. The sleeve
  therefore cannot pass until the speeds are equal; pushing harder only synchronises faster.
- **SYN-05**: Once synchronised, slip is zero and the cone torque vanishes. The chamfers turn the blocker back
  through its 2.81° index, and the sleeve passes through. Its chamfers then nudge the free gear by up to half a dog
  pitch (±5.625°) to line up the dogs, and the sleeve slides over the dog teeth. `state.py` indexes the gear before
  the dogs meet and flags any misalignment. Torque then flows gear → dog teeth → sleeve → hub → shaft; the cone
  carries none.
- **SYN-06**: This prevents clash, because the dog teeth only meet at zero relative speed. Without a synchro, the
  sleeve would strike dogs moving 585 rpm faster than it (1→2 at 3000 rpm, see SFT-01).
- **SYN-07**: The dog teeth transmit torque by direct contact, not friction. On real boxes a slight back-taper on the
  dogs, plus the rail detent, stops the gear jumping out under load; the model shows straight dogs and the detents.
  **(context)**
- **SYN-08**: The synchro only has to change the speed of the parts that the released clutch disconnects: the disc,
  input shaft, countershaft and free gears, I_INPUT = 0.012 kg·m² referred to the input. The output shaft is tied to
  the whole car, so its speed barely changes.

## 11. Selector linkage (SEL)

- **SEL-01**: The shift pattern is an H with reverse at bottom right:

  | | Left plane | Centre plane | Right plane |
  |---|---|---|---|
  | Lever forward | 1st | 3rd | 5th |
  | Lever back | 2nd | 4th | Reverse |

  Neutral is the crossbar, and the lever is spring-centred on the 3-4 plane.
- **SEL-02**: There are three shift rails (1-2, 3-4 and 5-R), each with a fork that sits in its sleeve's groove. 5th
  and reverse share one rail and one sleeve.
- **SEL-03**: Moving the lever sideways (select) puts its finger into one rail's slot. Moving it forward or back
  (shift) slides that rail, its fork and its sleeve.
- **SEL-04**: The linkage is direct. The lever pivots on a ball on top of the box (Y = -0.960 m) and its finger is
  below the pivot, so the finger moves opposite to the knob: **lever forward → rail, fork and sleeve rearward
  (-Y)**. 1st, 3rd and 5th sit behind their synchros, and 2nd, 4th and reverse in front.
- **SEL-05**: The same reversal applies sideways: knob left → finger right. As built the 1-2 rail is at X = +24 mm,
  3-4 at 0 and 5-R at -24 mm.
- **SEL-06**: The lever ratio (knob : finger) is about 55 mm/8.5 mm = 6.5 : 1. The 30 mm gate spacing at the knob is
  therefore only about 4.6 mm at the finger; the slotted shift heads sit 4.67 mm apart under the finger.
- **SEL-07**: Interlock: when one rail leaves neutral, the other two must stay in neutral; two ratios on one output
  shaft would jam the box. The model enforces this in the state validator rather than with interlock pins.
- **SEL-08**: Spring-loaded detent balls hold each rail at neutral and at engaged. Real boxes with reverse beside
  5th add a lockout so reverse cannot be selected straight from 5th; the film does not model one. **(context)**
- **SEL-09**: The forks do not rotate. The sleeve spins inside the fork's brass pads, which only push it axially.

## 12. The 1→2 shift at 3000 rpm (SFT)

- **SFT-01**: Before the shift, in 1st at 3000 rpm, the output shaft turns at 3000/3.484 = **861 rpm** (24.1 km/h).
  Free 2nd gear turns at 3000/2.075 = **1446 rpm**, 585 rpm faster than its shaft.
- **SFT-02**: Step 1, clutch in: no engine torque reaches the gear train. The driver closes the throttle, and the
  engine slows on its own friction and pumping losses: `state.py` uses -35 N·m on 0.18 kg·m², about 1860 rpm/s. In
  s05's very fast 0.27 s shift (PRS-03) the engine drops only about 400 rpm on its own, to about 2600 rpm at
  clutch-out, and the clutch-out slip (SFT-08) pulls it the rest of the way to 1778 rpm. In a normal-paced shift it
  would fall to about 1900-2250 rpm by itself (s08 holds it at 1950 / 2150 rpm with the throttle while the clutch is
  in, PRS-13).
- **SFT-03**: The clutch must be in for two reasons:
  - Dog teeth under load are held by friction and will not slide out of 1st.
  - The synchro can only re-speed the small input-side inertia (SYN-08). With the clutch engaged, it would also have
    to drag the engine and flywheel. `state.py` flags a sleeve moving or a synchro working against an engaged
    clutch.
- **SFT-04**: Step 2: the 1-2 sleeve slides forward off 1st gear's dogs to the centre, which is neutral. With the
  lever direct, the lever moves rearward to neutral.
- **SFT-05**: Step 3: as the sleeve moves forward toward 2nd, the blocker cone rubs on 2nd gear's cone and slows 2nd
  gear to 861 rpm. 2nd meshes with the countershaft, which meshes with the input gear, so the whole input side slows
  with it. `state.py` also applies the 2 s oil-drag spin-down while the input side runs free in neutral, so at cone
  contact the speeds are about 1-2% below the no-drag values. HUDs and tests must read them from the Track:
  - 2nd gear: about 1420-1446 → 861 rpm;
  - countershaft: about 2185-2229 → 1327 rpm;
  - input shaft and clutch disc: about 2940-3000 → 1787 rpm;
  - every other free gear too. Free 1st gear ends up at 513 rpm, now slower than its 861 rpm shaft.

  The end values assume constant road speed. If the program lets the car coast down, they fall slightly (SFT-07).
  As built in s05 (coasting): 2nd gear 1420 → 858 rpm and input shaft 2947 → 1782 rpm between cone contact (19.5 s)
  and the end of the blocking hold (30.85 s), 76 ms of real time.
- **SFT-06**: The new engine speed is n₂ = n₁ × i₂/i₁ = 3000 × 2.075/3.484 = **1787 rpm** at the same road speed.
  Other upshifts from 3000 rpm land at 2011 rpm (2→3), 2157 rpm (3→4) and 2444 rpm (4→5).
- **SFT-07**: Road speed is effectively constant during the shift. Coasting deceleration is about 0.13 m/s²
  (illustrative: rolling resistance 0.012 g, CdA 0.66 m², 1400 kg). As built, s05 coasts at this rate while no
  power flows (8.4-47.3 s of video, 0.26 s real): 24.15 → 24.03 km/h, so the engine locks at **1778 rpm** instead
  of 1787 (-0.5%). s08 instead holds road speed through each 2.2-2.6 s clutch-in so the engine locks at exactly
  1787 / 2011 rpm; a real coast over those times would land about 80 / 55 rpm lower (about 1710 / 1960 rpm).
- **SFT-08**: Step 5, clutch out: if the engine is still above 1787 rpm, the clutch slips briefly. Its friction
  torque (CLU-10) pulls the engine down to disc speed, and then the clutch locks. A "taller" gear means the engine
  turns 2.075/3.484 = 0.596 × as fast for the same road speed, and wheel torque drops by the same factor.

## 13. Propeller shaft (PRP)

- **PRP-01**: The propeller shaft is a one-piece 65 × 1.8 mm tube with a Hooke (Cardan) universal joint at each end
  and a slip yoke (splined sleeve under a rubber boot) at the gearbox end. It turns at gearbox-output speed: 861 rpm
  in 1st at 3000 rpm, and engine speed in 4th. It turns CW-F in forward gears.
- **PRP-02**: The gearbox output and the pinion are both horizontal and parallel, 55 mm apart in height. The joint
  centres sit 44 mm inside each flange face, so they are 1280 - 2 × 44 = 1192 mm apart along the car, and each
  joint works at β = atan(55/1192) = **2.64°**. (An earlier revision used the 1280 mm flange spacing and quoted
  2.46°.)
- **PRP-03**: A single Hooke joint at an angle does not pass a constant speed:
  ω_out/ω_in = cos β/(1 - sin² β·sin² θ). The ratio swings between cos β and 1/cos β twice per turn. At 2.64° that is
  ±0.106%, an angle error of ±0.030° (about β²/4).
- **PRP-04**: In the Z arrangement the input and output shafts are parallel, the two joint angles are equal, and both
  yokes on the tube lie in the same plane. The second joint then exactly undoes the first, so the pinion turns at
  exactly gearbox-output speed. Only the tube itself fluctuates, by ±0.106%.
  *As built:* the tube turns by `kin.hooke(theta_out, β)`; each cross is carried round by its flange yoke and rocked
  by φ = atan(-tan β·sin θ), the exact solution that keeps its other arm in the tube yoke.
- **PRP-05**: The fluctuation is invisible at 2.64°; it would take about 30° to reach ±15%. Uniform rotation of the
  flanges and pinion on screen is therefore correct. A small non-zero angle is good practice because it keeps the
  needle rollers moving. **(context)**
- **PRP-06**: Two points about the one-piece layout. **(context; AB-09)**
  - *Slip joint.* Every real propshaft needs a sliding spline (or a plunging CV joint) to take up engine and gearbox
    rock on their mounts and assembly tolerance. The model shows a slip yoke at the gearbox end; it never needs to
    slide in the film because the gearbox and axle are rigidly placed.
  - *Whirl.* The first bending critical speed of a pinned-pinned tube is ω = (π/L)²·√(EI/ρA), almost independent of
    wall thickness for a thin tube. For the 65 × 1.8 mm steel tube with L = 1.193 m between joint centres it is
    about **7,650 rpm**, which is **215 km/h** in 4th. That is only 13% above the 6,800 rpm the shaft reaches at the
    redline in 4th (190.7 km/h), and 5th's gearing would allow 8,346 rpm. Designers keep a wider margin, which is
    why cars of this class (BMW 3-series, GT86) use a two-piece shaft with a centre bearing, or a large-diameter
    aluminium or CFRP tube. No beat drives anywhere near these speeds.

## 14. Final drive (FD)

- **FD-01**: A 10T pinion drives a 41T ring gear: 41/10 = **4.10**. Ring speed is pinion speed/4.1 and ring torque
  is 4.1 × pinion torque. At 3000 rpm in 1st, the pinion turns at 861 rpm and the ring at 210 rpm.
- **FD-02**: The bevel gears' axes meet at 90°, which turns the drive from along the car to across it. The ring's
  190 mm PD gives a module of 4.634 mm and a pinion PD of 46.3 mm. The pitch cone angles are atan(10/41) = 13.71° and
  76.29°, summing to 90°. As built: spiral angle 35°, pinion left-hand, ring right-hand, 32.6 mm face width.
- **FD-03**: 41 and 10 share no common factor (gcd = 1). This is a hunting-tooth set: every pinion tooth meets every
  ring tooth, which evens out wear. **(context)**
- **FD-04**: With the pinion turning CW-F, the wheels roll forward only if the ring gear lies on the **left (-X)**
  of the pinion axis with its teeth facing +X, and the pinion meshes at the **front** of the ring. At the mesh the
  pinion's -X side moves down, and so does the front edge of a forward-rolling ring.
  *Why:* a vector check of v = ω × r at the pitch point. *As built:* the axle assembly places the ring on -X
  (`meta['final_drive']['ring_side'] = '-X'`); the axle brief's +X would have driven the car backwards.
- **FD-05**: The film models a spiral-bevel set with no offset: the pinion axis passes through the ring axis, both at
  Z = 0.305 m. Real RWD axles are hypoid: the pinion sits below the ring centre line. That allows a bigger, stronger
  pinion and a lower propshaft and tunnel, at the cost of more tooth sliding, which needs hypoid oil (AB-08).
- **FD-06**: Spiral teeth engage gradually, like helical teeth, so they are quieter and stronger than straight
  bevels. **(context)**

## 15. Open differential (DIF)

- **DIF-01**: The ring gear is bolted to the differential case (as built a two-piece case split on the cross-pin
  plane). A cross-pin in the case carries two 10T spider gears, which mesh with two 16T side gears (straight bevels).
  Each side gear is splined to an output stub whose flange, at |X| = 0.150 m, carries that driveshaft's inner-joint
  housing.
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
  4.26 m radius and the outer wheel a 5.74 m radius (no tyre slip). Outer/inner = **1.347** (inner/outer = 0.742).
  Over a 90° turn, for example, that is 6.69 m against 9.02 m. The inner wheel turns at **0.852 ×** case speed and
  the outer at **1.148 ×**.
- **DIF-07**: Worked example; all values scale linearly with speed. s06 (as built) drives at 15 km/h in 1st for the
  whole scene: straight, then held at 15 km/h while the curvature eases into the R = 5 m left turn (41.0-44.6 s), so
  the case stays at exactly 130.5 rpm while the outer wheel speeds up to 149.8 rpm and the inner slows to 111.1 rpm.
  The 10 km/h column is for comparison only:

  | Quantity | 15 km/h, R = 5 m (s06 as built) | 10 km/h, R = 5 m |
  |---|---|---|
  | Case | 130.5 rpm | 87.0 rpm |
  | Inner wheel | 111.1 rpm | 74.1 rpm |
  | Outer wheel | 149.8 rpm | 99.8 rpm |
  | Side gears relative to case | ±19.3 rpm | ±12.9 rpm |
  | Spiders on their pin | 30.9 rpm | 20.6 rpm |
  | Engine in 1st | 1864 rpm | 1242 rpm |
  | Yaw rate | 47.7°/s | 31.8°/s |
- **DIF-08**: In the left turn in s06, the left wheel is the inner, slower wheel and the right wheel the outer one. In
  a right turn the spiders spin the other way.
- **DIF-09**: At R = 5.0 m, with Ackermann steering and no slip, the inner front wheel steers 31.6° and the outer
  24.6°. The front-axle centre follows a radius of √(5² + 2.62²) = 5.645 m. The body's wheelhouses allow about 32° of
  lock. **(context for the turn shot)**

## 16. Driveshafts and CV joints (CVJ)

- **CVJ-01**: Each rear driveshaft has a plunging tripod joint at the differential and a Rzeppa ball joint at the hub.
  As built the tripod housing (tulip) bolts to the differential output flange at |X| = 0.150 m, which puts the tripod
  centre at **|X| = 0.1771 m** (spec X_INNER_JOINT_CENTRE); the Rzeppa centre is at |X| = 0.655 m. The joint centres
  are **478 mm** apart, and the shaft is level at rest (both joints at Z = 0.305 m).
- **CVJ-02**: The differential is body-mounted while the wheel moves ±60 mm, so the shaft angle changes by up to
  asin(60/478) = **7.2°** (equal at both joints, because the hub axis stays parallel to the differential output).
  Joints are needed at both ends. Two in-phase Hooke joints would then form a Z arrangement and cancel exactly
  (PRP-04); some production IRS cars (Jaguar IRS, C2/C3 Corvette) did run Hooke-jointed halfshafts. Constant-velocity
  joints are used for these reasons:
  - Camber and toe change as a real wheel moves, so the two joint angles become unequal and the cancellation is
    lost. For example, 7.2° against 6.2° leaves a ±0.20% speed ripple at twice shaft frequency; a single 7.2° joint
    would give ±0.8%.
  - A Hooke joint at an angle puts a secondary couple (about T·tan β, twice per turn) into the shaft and its
    bearings.
  - The geometry needs plunge (CVJ-06). A Hooke-jointed shaft would need a sliding spline, which binds and shudders
    when it has to slide under drive torque. A tripod plunges on rollers with low friction.
  - CV joints take larger angles in a compact, sealed package.

  The standard answer, used here, is a plunging tripod inside and a fixed Rzeppa outside. Each gives constant
  velocity at any angle on its own.
- **CVJ-03**: In the Rzeppa joint, six balls (17 mm, on a 61 mm pitch circle) run in curved grooves between an inner
  race (on the shaft) and an outer race (the bell on the hub stub). The inner and outer grooves are curved about
  centres offset 3.5 mm on either side of the joint centre (Birfield-type track offset). The offset steers each ball
  into the plane that bisects the angle between the two shafts, and the cage keeps all six ball centres in that one
  plane. *As built:* each ball centre is computed per frame on both groove centre lines and in the bisecting plane,
  and the cage tilts by half the joint angle. The narration's "a cage holds them in the plane that splits the angle"
  is an acceptable simplification (N14).
- **CVJ-04**: Each ball centre is therefore the same distance from both shaft axes, so its contact speeds on the two
  races match at every instant. Wheel speed equals shaft speed exactly, at any angle: this is what "constant
  velocity" means.
- **CVJ-05**: In the tripod joint, three spherical rollers (29 mm) on a three-armed spider fixed to the shaft run in
  three straight axial tracks in the differential-side housing (the tulip). The rollers roll and slide along the
  tracks, allowing both angle and plunge with essentially constant velocity. At 7.2° each roller slides
  ±r·sin α = ±3.4 mm per turn on top of the plunge. A real tripod's centre also orbits about 0.1 mm at three times
  shaft speed; the model ignores this (AB-13).
- **CVJ-06**: **Plunge.** The shaft is rigid: its joint centres stay L = 478 mm apart. The outer centre O moves
  vertically with the hub by s, so the tripod centre must slide outboard along the tulip axis by
  p = L - √(L² - s²) ≈ s²/2L. That is **3.78 mm at ±60 mm** and 0.94 mm at ±30 mm. Equivalently, the distance from
  the tulip (fixed to the differential) to the outer joint grows by √(L² + s²) - L = 3.75 mm.
  - *Direction and frequency.* The shaft is level at ride height, so p is zero there and positive on both bump and
    droop: the rollers move outboard and back, never inboard of their ride-height position, at **twice** the bounce
    frequency. The real figure also depends on the arcs of the suspension links.
  - *Visibility.* 3.78 mm is 0.8% of the joint spacing. s07 shows it at true scale in a sectioned close-up with a
    live "Plunge" readout (and the joint angle, max 7.2°).
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
  car forward. Without grip the wheel would spin and the car would not move. (The model assumes no slip, AB-01.)

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
| Release bearing rotating race (the one touching the fingers) | CW-F, always (constant contact, CLU-12) | 1 |
| Pedal, release fork, forks, rails, shift lever | pivot or translate only | - |

## 19. Presentation liberties (PRS)

- **PRS-01**: In slow motion, every angle integrates ω × slowmo(t) × dt. HUD rpm and km/h always show the physical
  values, and the badge shows the factor.
- **PRS-02**: s02 shows the 850 rpm idle at **230 ×** in inline4, crank, cycle and flywheel (the 132T starter
  ring at 0.34 teeth/frame) and at **198.3 ×** for the four 7.0 s stroke beats, so each shows exactly 180° of crank
  (π/(7.0 s × 89.0 rad/s)). The valvetrain beat runs at 230.3 ×, solved so cylinder 1 reaches its firing TDC 1.5 s
  into the firing beat. The firing beat is in PRS-11. The flywheel and its ring gear stay **visible for the whole
  scene**. The ring is de-strobed by **per-object Cycles motion blur**: only the flywheel and ring-gear objects
  have motion blur (everything else, the camera included, is unblurred), and the scene shutter is keyed per frame
  from the ring's tooth pitches per frame p: shutter = 1/p while p ≥ 0.5, so the smear is exactly **one tooth
  pitch** (a uniform band, nothing to step backwards), blending to a 1-frame shutter below p = 0.4 (the smear joins
  consecutive positions, so the motion reads forward). As built: 1.0 frame at 198.3-230 × (p = 0.34-0.39), 1.09
  frames at 85 × (p = 0.92, smear 1.00 pitch), up to 2.0 frames in the slow-motion ramps into and out of the firing
  beat (61.3-74.2 s), where p passes 0.5. The aliasing check keeps only frames where the ring is in frame and its
  blur does not hide strobing (smear < 0.95 pitch and p > 0.5): none, by construction.
- **PRS-03**: s05 shows 3000 rpm at a constant **150 ×** (the crank turns 5° per frame), with no motion blur and
  no flywheel in shot (gearbox only). From the pedal starting down (7.35 s) to the clutch locking in 2nd (47.4 s),
  the real shift takes **0.27 s**. Within it: pedal down 13 ms (7.35-9.3 s), out of 1st 15 ms (13.55-15.85 s),
  synchronising 76 ms (cone contact 19.5 s to the end of the hold 30.85 s), through the blocker and seated 47 ms
  (36.2 / 37.9 s), and clutch out 46 ms from the pedal starting up (40.55 s; fully up 8 ms later) to the lock. The
  remaining time is the pauses between the steps, paced to the narration. That is a **very fast,
  racing-style shift**. A 76 ms sync must decelerate 0.012 kg·m² by about 1165 rpm (2947 → 1782 rpm at the input):
  a mean 19.4 N·m at the input (30 N·m at the peak of the cosine blend), 40 N·m at 2nd gear's cone, which needs
  about 1.75 kN of sleeve force (μ 0.1, r_c 26 mm, α 6.5°), about 270 N at the knob. A normal shift takes about
  0.5-1 s. Matching that would need about 30-70 × in steps 1-3 and 5, and at those factors most of the gear train
  strobes (PRS-04). The film therefore keeps 150 ×, shows the shift as a fast one, and step 5 shows a clear slip
  from about 2600 down to 1778 rpm (SFT-02, SFT-07, SFT-08). The throttle closes as the pedal goes down (target
  800 rpm, 7.45-7.85 s) and reopens at clutch-out (target 1850 rpm, 40.6-41.4 s).
- **PRS-04**: To avoid wagon-wheel strobing, `Track.validate(aliasing=...)` requires a part with N-fold symmetry to
  turn at most 0.35 of a pitch per frame: **slowmo ≥ rpm × N / 504**. Meshing gears share one tooth-pass frequency,
  so both members of a pair need the same factor. Motion blur only hides strobing once the smear s·p ≥ about 1
  pitch (shutter s frames, p pitches per frame); between 0.35 and about 2 pitches per frame visible toothed parts
  must be avoided. Hidden or fully smeared parts can be masked out of the check. s02 also accepts p ≤ 0.5 under a
  full 1-frame shutter, where the smear joins consecutive positions and the motion reads forward (PRS-02).

  **s05: 1st gear, 3000 rpm (output 861 rpm), at 150 ×**, worst first:

  | Part | Teeth or symmetry | Speed | Minimum slowmo | Pitch per frame at 150 × |
  |---|---|---|---|---|
  | Flywheel ring gear | 132 | 3000 rpm | 786 × | 1.83 |
  | 5th gear dog ring | 32 | 3682 rpm | **234 ×** | 0.55 |
  | Input gear's 4th-gear dog ring | 32 | 3000 rpm | **191 ×** | 0.44 |
  | 5th output gear 23T / countershaft 38T | 23 / 38 | 3682 / 2229 rpm | 168 × | 0.39 |
  | Input gear 26T / countershaft 35T (headset) | 26 / 35 | 3000 / 2229 rpm | 155 × | 0.36 |
  | 3rd gear dog ring | 32 | 2157 rpm | 137 × | 0.32 |
  | 3rd pair 31T / 30T | 31 / 30 | 2157 / 2229 rpm | 133 × | 0.31 |
  | Diaphragm fingers | 18 | 3000 rpm | 107 × | 0.25 |
  | 2nd gear 37T / countershaft 24T | 37 / 24 | 1446 / 2229 rpm | 106 × | 0.25 |
  | 2nd gear dog ring | 32 | 1446 rpm | 92 × | 0.21 |
  | 1st gear 44T / countershaft 17T | 44 / 17 | 861 / 2229 rpm | 75 × | 0.18 |
  | Reverse train 15T / idler 22T / 38T | 15 / 22 / 38 | 2229 / 1519 / 880 rpm | 66 × | 0.15 |
  | 1st gear dog ring; sleeves and hubs (32) | 32 | 861 rpm | 55 × | 0.13 |
  | Final-drive ring gear / pinion | 41 / 10 | 210 / 861 rpm | 17 × | 0.04 |

  **Other scenes:**

  | Scene and condition | Part | Teeth | Speed | Minimum slowmo |
  |---|---|---|---|---|
  | s02, s03: 850 rpm idle | Flywheel ring gear | 132 | 850 rpm | **223 ×** (as built 230 ×: 0.34 pitch/frame; s02 also motion-blurs it, PRS-02) |
  | s02 | Timing sprockets and chain | 21 / 42 | 850 / 425 rpm | 35 × |
  | s03: idle | Disc hub / input splines | 23 | 850 rpm | 39 × (as built 56 × in the release: 0.24) |
  | s03: idle | Diaphragm fingers | 18 | 850 rpm | 30 × (as built 56 ×: 0.19) |
  | s03: idle | Input gear's 4th-gear dogs | 32 | 850 rpm | 54 × (as built 56 ×: 0.34) |
  | s03 release, 56 × | Flywheel ring gear | 132 | 850 rpm | not met (1.39 teeth/frame): out of view or behind the flywheel, otherwise smeared ≥ 1 pitch by a 0.75-frame shutter |
  | s03 take-off, 8 × then 10 × | Cover pockets / windows / straps | 3 | ≤ 1342 / ≤ 1438 rpm | 8.0 × / 8.6 × (as built 8 × / 10 ×) |
  | s03 take-off | Fingers, bolts, rivets, splines, damper springs, 26T gear and its dogs | 6-32 | up to 1438 rpm | not met (up to 3.4 pitch/frame): relaxed under a 0.4-frame shutter (O6) |
  | s04: neutral, idle | 5th gear dog ring | 32 | 1043 rpm | **66 ×** (as built 72 ×) |
  | s04: neutral, idle | Input gear's 4th-gear dogs | 32 | 850 rpm | 54 × |
  | s04: neutral, idle | 5th pair 23T / 38T | 23 / 38 | 1043 / 631 rpm | 48 × |
  | s04 ratios: 1500 rpm in 1st/4th/5th | 5th gear dog ring | 32 | 1841 rpm | 117 × (as built 130 ×) |
  | s04 reverse: 950 rpm, -7.8 km/h | 5th gear dog ring | 32 | 1166 rpm | 74 × (as built 80 ×) |
  | s06: 15 km/h straight | Tyre tread | 64 | 130.5 rpm | **16.6 ×** (as built 20 ×) |
  | s06: 15 km/h, R = 5 m | Outer tyre tread | 64 | 149.8 rpm | **19.0 ×** (as built 20 ×: 0.33 pitch/frame) |
  | s06: 15 km/h | Ring gear 41T / pinion 10T | 41 / 10 | 130.5 / 535 rpm | 10.6 × (0.71 × v in km/h; as built 20 ×, then 12 × in the spider close-up with the tyres out of frame) |
  | s07: 10 km/h | Tyre tread | 64 | 87.0 rpm | 11.0 × (as built 12 ×) |

  As built, s05 (150 ×, no motion blur) shows no flywheel and keeps the headset (155 ×), 5th pair (168 ×), input-gear
  dogs (191 ×) and 5th dog ring (234 ×) out of frame until the synchroniser has slowed the input side to 1782 rpm,
  below all four limits (at 150 × they need input speeds ≤ 2908, 2678, 2362 and 1925 rpm); the validator gets
  per-frame frustum masks for them (no occlusion credit). s08 runs in real time with motion blur on every frame
  (PRS-05).
- **PRS-05**: In real-time shots (the end of s07, and s08) the wheels turn 52.5° per frame at 24 km/h and 88° per
  frame at 40.5 km/h. Spokes strobe exactly as they would on a real 24 fps camera; Cycles motion blur makes this look
  natural. As built: s07 keys the shutter from 0.25 frame in slow motion to 0.5 frame over the ramp to real time
  (35.3-36.5 s); s08 uses 0.5 frame (a 180° shutter) throughout.
- **PRS-06**: Cutaways (with red section faces, AB-15), the x-ray or fading bodywork, exploded views, labels and the
  warm power-path glow are presentation only. They never change any part's motion.
- **PRS-07**: The gas colours (blue intake, orange combustion, grey-brown exhaust) are illustrative. A petrol flame is
  faint and bluish, and the gases are colourless.
- **PRS-08**: Clearances and small motions are true scale: about 0.6 mm per clutch face at full pedal (0.575 mm,
  drawn 0.615 mm), a 1.8 mm plate lift, 3.78 mm of CV plunge at ±60 mm (shown with a live readout in s07). If a
  scene magnifies any of them, the storyboard must say so. The model's simplifications are listed under AB.
- **PRS-09**: The tyre is modelled at a 0.3115 m radius while the motion uses r = 0.305 m, so the tread surface moves
  2.1% faster than the ground. This is invisible at normal viewing distance, and the 6.5 mm "squash" hides the
  contact.
- **PRS-10**: The hydraulic pulse along the line, the stroke strip, the firing ticker, the step cards, the
  "PAUSED" freeze in s06 (PRS-14) and the witness marks in s03 (PRS-19) are visual aids.
- **PRS-11**: s02 "firing" beat (12.5 s), as built at **85 ×**: 180° of crank every 3.0 s, so cylinder 1 fires at
  63.0 s and cylinders 3, 4 and 2 at 66.0, 69.0 and 72.0 s; the beat covers 722°. The flywheel stays in view: its
  132T ring moves 0.92 teeth per frame and is motion-blurred with a 1.09-frame shutter into a uniform band (exactly
  one tooth pitch of smear, PRS-02). The timing sprockets (35 ×) and the flywheel's 6 bolt holes are fine.
- **PRS-12**: s03 runs slow enough for the ring gear while it is in shot, then close to real time for the
  take-off (PRS-13):
  - 0-27.6 s (parts, splines, engaged) at **230 ×**: the 132T ring at 0.34 teeth/frame.
  - 27.6-29.3 s, with the camera on the input shaft (the ring out of view or behind the flywheel; the visibility
    test includes that occlusion), the factor eases to **56 ×** and holds to the cut. Fingers 0.19, hub splines
    0.24, facing rivets 0.25, 26T input gear 0.27 and its dog ring 0.34 pitch/frame; the ring (1.39 teeth/frame) is
    smeared ≥ 1 pitch by a 0.75-frame shutter whenever it is visible. The release is one continuous press,
    30.5-44.6 s = **0.25 s real**; capacity falls below 2% at 34.1 s (CLU-13) and the free disc coasts on oil drag
    from 850 to 762 rpm by the cut (CLU-08).
  - **Hard cut at 46.5 s to 8 ×** (shutter 0.4 frame): 1st gear is selected (46.75-48.6 s) and the synchroniser
    stops the disc and input shaft in about 0.1 s real, car stationary (CLU-09).
  - Take-off: the pedal leaves the floor at 49.0 s and reaches the bite (p = 0.49) at 51.4 s. From there the pedal
    is solved as the inverse of `kin.clutch_capacity` at the torque a prescribed take-off needs (illustrative
    1450 kg effective mass, 160 N rolling resistance, 92% driveline efficiency): up to about 95 N·m at the clutch,
    peak acceleration **2.7 m/s²**. With the throttle target at 1385 rpm the engine rises to about 1340 rpm and
    sags to about 1150 rpm as the clutch takes up. The clutch **locks at 60.62 s at 1168 rpm and 9.4 km/h** (pedal
    0.41, CLU-13), **1.15 s of real time after the bite**. A realistic pull-away slips for about 1-1.5 s.
  - After the lock the factor eases to **10 ×** (60.9-62.0 s), the pedal comes fully up and the engine pulls to
    **1438 rpm, 11.6 km/h** at the end of the scene.
- **PRS-13**: **Slow-motion factors per scene**, all as built (read from the scene modules):

  | Scene | Beats (video time) | Factor | Reason / notes |
  |---|---|---|---|
  | s01 | all | none (engine off, parked) | nothing turns |
  | s02 | inline4, crank, cycle, flywheel | 230 × | 132T ring gear in shot (≥ 223 ×); also motion-blurred (PRS-02) |
  | s02 | intake, compression, power, exhaust | 198.3 × | 180° per 7.0 s beat |
  | s02 | valvetrain | 230.3 × | solved: cyl 1 at firing TDC at 63.0 s |
  | s02 | firing | 85 × | four firings 3.0 s apart; ring motion-blurred (PRS-11) |
  | s03 | parts, splines, engaged (0-27.6 s) | 230 × | ring gear in shot (≥ 223 ×) |
  | s03 | end of engaged, release (ease 27.6-29.3 s, to the cut at 46.5 s) | 56 × | pedal press 0.25 s real; disc coasts 850 → 762 rpm (PRS-12) |
  | s03 | end of release, slip (hard cut at 46.5 s) | 8 ×, easing to 10 × after the lock (60.9-62.0 s) | synchro stops the disc in ~0.1 s; bite → lock 1.15 s real, lock 60.62 s at 1168 rpm, 9.4 km/h; peak 2.7 m/s²; end 1438 rpm, 11.6 km/h; shutter 0.4 frame |
  | s04 | shafts, neutral, synchro, lock, linkage | 72 × | 5th dogs at idle ≥ 66 ×; the lock-beat synchro event (cone contact to end of hold, 1.7 s) is about 24 ms real |
  | s04 | ratios (hard cuts; 1500 rpm; 1st 12.1, 4th 42.1, 5th 51.6 km/h) | 130 × | 5th dogs at 1841 rpm ≥ 117 × |
  | s04 | reverse (950 rpm, -7.8 km/h) | 80 × | 5th dogs at 1166 rpm ≥ 74 × |
  | s05 | all | 150 × constant, no motion blur | shift 0.27 s real; synchronising 76 ms; lock at 47.4 s at 1778 rpm, road speed coasting 24.15 → 24.03 km/h at 0.13 m/s² (PRS-03, SFT-07) |
  | s06 | prop, ringpinion, straight; turn while the tyres are in frame | 20 × at 15 km/h | tread ≥ 16.6 × straight, outer tread ≥ 19.0 × in the turn |
  | s06 | diffparts | 0: eases to a freeze 20.55-21.55 s, frozen to 31.45 s, back to 20 × by 32.45 s | exploded diff holds still, cross-pin vertical (PRS-14) |
  | s06 | turn, spider close-up (ease 48.9-50.0 s) | 12 × | ring/pinion ≥ 10.6 ×, tyres out of frame; 15 km/h held through the R = 5 m left turn: inner 111.1, outer 149.8, case 130.5 rpm (DIF-07) |
  | s07 | why, rzeppa, plunge | 12 × at 10 km/h | tread ≥ 11.05 ×; body heave ±60 mm at 1.33 Hz real (AB-16); plunge gauge at true scale (CVJ-06) |
  | s07 | moves | 12 × → real time (35.3-38.6 s, log-linear) | car pulls away 10 → 20 km/h (38.2-41.5 s); shutter 0.25 → 0.5 frame (PRS-05) |
  | s08 | all | real time (1 ×), Cycles motion blur shutter 0.5 frame | take-off slips from 4.79 s and locks at 6.25 s (8.6 km/h, 1073 rpm); 3000 rpm in 1st at 10.0 s; road speed held through each 2.2-2.6 s clutch-in so the engine locks at exactly 1787 rpm (12.6 s) and 2011 rpm (19.1 s), where a real coast would land about 80 / 55 rpm lower (SFT-07); cruise in 3rd at 49.1 km/h, 2436 rpm |
- **PRS-14**: **Time freeze in s06.** For the exploded differential (diffparts) the slow-motion factor eases to zero
  (20.55-21.55 s), time stands still with a "PAUSED" badge until 31.45 s and resumes at 20 × by 32.45 s. The
  initial wheel phase is solved so the freeze lands with the cross-pin exactly vertical: the pin and spiders explode
  straight up and down, the case halves, side gears and stubs sideways along the axle. No part moves relative to
  another while time is frozen; only the explode carriers and the camera move.
- **PRS-15**: **World-fixed live section planes on rotating parts.** A real cutaway part carries its cut round with
  it; these shots instead hold the section plane fixed (in the world or in the car) while the part turns inside it,
  using live Manifold booleans evaluated per frame, so the cut always faces the camera:
  - s03: the clutch's 'section_rotating' half section, plus the same cut on the flywheel and ring gear; the cutter
    sweeps in from outside the parts to the axis plane (10.9-12.5 s) and back out (49.0-51.0 s), so the take-off
    shows the clutch whole.
  - s05: a static quarter-section box (x < 0, above the axis, between 3rd gear and the web) through the 1-2 hub,
    sleeve, struts, both blocker rings, both cones and dog rings and the fork. Cut faces use shades of section red
    per part (sleeve red, hub darker, blocker rings orange, cones and dog rings maroon). The lower half stays whole,
    so the blocker teeth (output speed) and 2nd gear's dog teeth (gear speed) run side by side.
  - s07: planes fixed in the car on the outer joint's bell, boot and clamps (the cutter rides with the RR corner)
    and on the inner joint's tulip, boot and clamps, removing the rear half; each plane sweeps open and closed.
    Balls, cage, inner race, spider and rollers stay whole.
- **PRS-16**: **Ghosted shells.** Bodywork and housings are faded so the parts inside show; nothing changes their
  motion (PRS-06):
  - s01: the paint fades to an x-ray shell (exterior 0.15, trim and lamps half that, mirrors a quarter) with feature
    lines; glass, cabin trim and floor/tunnel/firewall fade to 0; everything but the engine and flywheel fades out
    at the end (29.0-30.1 s).
  - s03: for the take-off (51.8-52.5 s) the half bellhousing ghosts to 10%, the hydraulics and pedal box to 12% and
    the firewall patch to about 9% (the camera swings behind them).
  - s04: the -X half of the case, web and tail housing fades away in the first beat.
  - s06: the axle housing is cut in half at 6.8 s; the kept half fades out for the exploded view; the case halves
    are ghosted to 0.15 from 32 s; a faint x-ray body (0.08) with feature lines appears only for the wide turn shot.
  - s07: the body is hidden for the close-ups and fades back in for "moves"; the RR coil-over fades out while the
    joints are shown.
  - s08: x-ray body (exterior 0.12, glass 0.05, interior 0.06, underbody 0.05, feature lines 0.55); the housings
    stay opaque and the parts inside them, which can never be seen, are hidden from the render.
- **PRS-17**: **s04 reverse shot.** The idler sits on the +X side, behind the countershaft and reverse gears as seen
  from the cut (-X) side, so the reverse shot looks from +X with the kept half of the gearbox housings (case, web,
  tail housing) removed, not ghosted, from the cut at the start of the reverse beat to the end of the scene.
- **PRS-18**: **s01 camera path through the body.** The camera never crosses a visible surface. It enters the cabin
  at 13.3 s through the front-left door-window opening (the glass is hidden by 10.8 s and the shell is already an
  x-ray), travels rearward inside the cabin (≥ 0.15 m from every visible surface, ≥ 0.14 m at the window frame),
  and leaves forward through the dash and windscreen only after the body has faded out (29.0-30.1 s; hidden from
  29.9 s), ending on the engine for the hand-off to s02.
- **PRS-19**: **s03 witness marks.** Two marker-paint daubs, orange with the presentation glow, fade in at
  51.0-51.8 s for the take-off: one on the back of the clutch cover (engine side) and one on the rear face of the
  input gear's cone (gearbox side). They rotate with their parts, so under the take-off's motion blur the viewer
  can still see one side turning at engine speed and the other starting from rest and catching up. They are a
  visual aid; one feature per turn, so they are checked strictly for strobing.

---

## 20. Narration line → fact IDs

Every sentence of the current `carviz/timeline.py`. The key is scene.beat.sentence number. N-notes are in section 21.

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
| s02.intake.1 | Intake: the piston moves down, drawing air and fuel in through the open intake valves. | CYC-09, CYC-08, CYC-01, CYC-07, VLV-06 |
| s02.compression.1 | Compression: the intake valves close, and the rising piston squeezes the mixture. | CYC-10, CYC-04, CYC-01, VLV-06, ENG-03 (N1 applied; V5) |
| s02.power.1 | Power: a spark ignites the mixture, and the hot gas forces the piston down. | CYC-11, CYC-12 |
| s02.exhaust.1 | Exhaust: with the exhaust valves open, the rising piston pushes the burnt gas out. | CYC-13, CYC-03 |
| s02.valvetrain.1 | Camshafts open the valves. | VLV-01, VLV-06, CYC-07 |
| s02.valvetrain.2 | A timing chain drives them from the crankshaft at exactly half its speed, because each valve opens only once every two turns. | VLV-02, VLV-03, VLV-04, VLV-07 |
| s02.firing.1 | Only the power stroke drives the crankshaft, so the cylinders take turns, firing in the order one, three, four, two: a power stroke every half turn. | ENG-09, ENG-13, ENG-11, ENG-08 (PRS-11) |
| s02.flywheel.1 | At the back, a heavy flywheel smooths out the pulses. | FLY-01, ENG-13 (AB-02) |
| s02.flywheel.2 | Its face is one half of the clutch. | FLY-02, CLU-01 (N9) |
| s03.parts.1 | Between the engine and the gearbox sits the clutch: the flywheel, a friction disc, and a pressure plate with a diaphragm spring. | CLU-01, CLU-02, CLU-04 |
| s03.splines.1 | The disc's hub is splined to the gearbox input shaft, so they always turn together. | CLU-03 |
| s03.engaged.1 | With the pedal up, the spring clamps the disc between the pressure plate and flywheel, so the input shaft turns with the engine. | CLU-04, GBX-12 (N5 applied) |
| s03.release.1 | Press the pedal, and fluid flows from the master cylinder to the slave cylinder. | HYD-01, HYD-02, HYD-05, HYD-06 |
| s03.release.2 | It pushes the release fork and bearing against the spring's fingers. | HYD-03, HYD-05, CLU-05, CLU-12 |
| s03.release.3 | The spring flexes, the pressure plate pulls back, and the disc is free. | CLU-05, CLU-06, CLU-08, CLU-09, CLU-13, PRS-12 (V7) |
| s03.slip.1 | To pull away, the pedal comes up slowly. | CLU-06, CLU-11, PRS-12 |
| s03.slip.2 | The disc slips between the flywheel and pressure plate, speeding up as it passes on torque, until it matches the engine and locks. | CLU-04, CLU-10, CLU-11, CLU-13, PRS-12 (N6 applied) |
| s04.shafts.1 | The gearbox has three shafts. | GBX-01, VEH-04 |
| s04.shafts.2 | The input shaft, driven by the clutch, turns the countershaft below. | GBX-07, GBX-01 |
| s04.shafts.3 | The output shaft runs out the back, in line with the input. | GBX-01, VEH-04 |
| s04.neutral.1 | The countershaft's forward gears each mesh with a gear on the output shaft. | GBX-03, GBX-04, GBX-13 (N8) |
| s04.neutral.2 | They're always in mesh, but the output gears spin freely on bearings. | GBX-02, GBX-12 |
| s04.neutral.3 | Until one is locked to the shaft, no power gets through: neutral. | GBX-15 |
| s04.synchro.1 | Synchronizers do the locking. | SYN-01 |
| s04.synchro.2 | A hub is splined to the shaft, and a sleeve slides on it. | SYN-01 |
| s04.synchro.3 | Each gear carries dog teeth and a cone, with a brass blocker ring between cone and sleeve. | SYN-01, SYN-02, SYN-03 (N7 applied) |
| s04.lock.1 | Slide the sleeve over the dog teeth, and the gear is locked to the shaft. | SYN-02, SYN-05, SYN-07, CLU-09 |
| s04.linkage.1 | Forks on shift rails move the sleeves. | SEL-02, SEL-09 |
| s04.linkage.2 | Moving the lever sideways picks a rail; forward or back slides it. | SEL-01, SEL-03, SEL-04, SEL-05, SEL-07 |
| s04.ratios.1 | In first, the input gear turns the countershaft more slowly, and a small countershaft gear drives a large output gear: the engine turns about three and a half times per output-shaft turn. | GBX-07, GBX-08, GBX-04 (N2 applied; V6) |
| s04.ratios.2 | Fourth locks input to output: one to one. | GBX-10 |
| s04.ratios.3 | Fifth is an overdrive. | GBX-11 |
| s04.reverse.1 | Reverse adds an idler gear between the shafts, so the output turns backwards. | REV-01, REV-02, GBX-14 |
| s05.intro.1 | Here's a shift from first to second at three thousand rpm, slowed right down. | SFT-01, PRS-03, PRS-13 |
| s05.clutch_in.1 | One: clutch in. | SFT-02 |
| s05.clutch_in.2 | The engine is disconnected. | SFT-02, SFT-03 |
| s05.neutral.1 | Two: the sleeve slides out of first, into neutral. | SFT-04, SEL-04 |
| s05.sync.1 | Three: the blocker ring's cone presses on second gear. | SFT-05, SYN-03, SYN-04 (N12) |
| s05.sync.2 | Friction slows it, along with the countershaft, input shaft and clutch disc, until it matches the output shaft's speed. | SFT-05, SYN-08 |
| s05.engage.1 | Four: speeds matched, the blocker ring lets the sleeve through, onto second gear's dog teeth. | SYN-02, SYN-05, SYN-06 |
| s05.clutch_out.1 | Five: clutch out. | SFT-08 |
| s05.clutch_out.2 | The engine is reconnected at about eighteen hundred rpm: same road speed, taller gear. | SFT-06, SFT-07, SFT-08, PRS-03 (N11) |
| s06.prop.1 | The propeller shaft carries the drive back to the rear axle. | PRP-01, PRP-02, PRP-03, PRP-04 |
| s06.ringpinion.1 | There, a small pinion drives a large ring gear: ten teeth against forty-one, a final drive ratio of 4.1 to 1, turning the drive through a right angle. | FD-01, FD-02, FD-03, FD-04 |
| s06.diffparts.1 | The ring gear is bolted to the differential case. | DIF-01 |
| s06.diffparts.2 | Inside, two spider gears mesh with two side gears, one splined to each driveshaft. | DIF-01 (N10) |
| s06.straight.1 | Going straight, the spider gears don't spin on their pin. | DIF-03 |
| s06.straight.2 | Everything turns as one, and both wheels match. | DIF-02, DIF-03 |
| s06.turn.1 | In a turn, the outer wheel travels further than the inner one. | DIF-06 |
| s06.turn.2 | The spider gears now spin on their pin, letting one side speed up as the other slows. | DIF-03, DIF-07, DIF-08 (V10 resolved) |
| s06.turn.3 | The case turns at the average of the two. | DIF-02, DIF-07 |
| s07.why.1 | Each driveshaft has a constant-velocity joint at each end, because the wheel moves up and down while the differential stays put. | CVJ-01, CVJ-02, VEH-07, AB-16 (N13) |
| s07.rzeppa.1 | In the outer joint, six balls run in grooves between inner and outer races. | CVJ-03 |
| s07.rzeppa.2 | A cage holds them in the plane that splits the angle, so the wheel turns at exactly the shaft's speed. | CVJ-03, CVJ-04 (N14) |
| s07.plunge.1 | The inner joint can also slide, as the wheel's distance from the differential changes. | CVJ-05, CVJ-06 (N3 applied; N3a) |
| s07.moves.1 | Finally, the wheel turns, the tire grips the road, and the car moves. | RD-01, RD-04 |
| s08.together.1 | Let's put it all together. | VEH-02 |
| s08.first.1 | Clutch up in first, and the engine pulls to three thousand rpm. | CLU-11, CLU-13, RD-02, PRS-13 |
| s08.second.1 | Clutch in, second gear, clutch out: the revs drop, and the car keeps accelerating. | SFT-06, SFT-07, RD-02, RD-03, PRS-13 (N15) |
| s08.third.1 | Then third. | SFT-06 |
| s08.summary.1 | Engine, clutch, gearbox, propeller shaft, final drive, differential, and driveshafts: one chain of gears and shafts, turning fuel into motion. | VEH-02, PRP-01, FD-01, DIF-01, CVJ-01 (N4 applied) |

## 21. Narration accuracy

**Applied since the first review.** All seven recommended rewordings are now in `timeline.py`, and
`timeline.check()` passes (140 wpm, LEAD 0.25 s, 0.2 s tail).

| Note | Beat | Was | Now | Fact |
|---|---|---|---|---|
| N1 | s02.compression | "both valves close" | "the intake valves close" ("close" is spoken 1.96 s in; IVC is at 1.94 s at 198.3 ×) | CYC-04 |
| N2 | s04.ratios | small gear drives large gear, "so" 3.5 turns | headset reduction named ("the input gear turns the countershaft more slowly") | GBX-08 |
| N3 | s07.plunge | "as the shaft's length changes" | "as the wheel's distance from the differential changes" (N3a) | CVJ-06 |
| N4 | s08.summary | propeller shaft missing | propeller shaft listed | VEH-02 |
| N5 | s03.engaged | "the gearbox turns with the engine" | "so the input shaft turns with the engine" (gearbox in neutral) | GBX-12 |
| N6 | s03.slip | "slips against the flywheel" | "slips between the flywheel and pressure plate" | CLU-04 |
| N7 | s04.synchro | "blocker ring in between" | "a brass blocker ring between cone and sleeve" | SYN-01 |

**Still slightly imprecise (acceptable; optional fixes)**

1. **N3a, s07.plunge.1** — APPLIED: the line now reads "The inner joint can also slide, as the wheel's distance
   from the differential changes." (the joint-centre spacing of a rigid shaft stays 478 mm; the tripod slides 3.78 mm, CVJ-06).
2. **N8, s04.neutral.1** — APPLIED: was "The other countershaft gears each mesh with a gear on the output shaft."
   (the reverse countershaft gear meshes with the idler, REV-03); now "The countershaft's forward gears each mesh with
   a gear on the output shaft."
3. **N9, s02.flywheel.2**: "one half of the clutch" is loose but fair: the flywheel and pressure plate are the two
   clamping members (FLY-02).
4. **N10, s06.diffparts.2**: "one splined to each driveshaft". As built the side gear is splined to an output stub
   that carries the inner-joint housing (DIF-01); the storyboard shows "driveshaft stubs", so this is consistent.
5. **N11, s05.clutch_out.2**: "about eighteen hundred rpm" matches 1787 rpm (SFT-06); as built s05 locks at
   1778 rpm because the car coasts 0.12 km/h during the 0.27 s shift (SFT-07, PRS-03). "Same road speed" holds to
   0.5%.
6. **N12, s05.sync.1**: "the blocker ring's cone presses on second gear" is correct: the blocker's internal cone
   presses on 2nd gear's cone.
7. **N13, s07.why.1**: the "because" explains why the shaft needs a joint at each end, not why those joints must be
   constant-velocity joints (CVJ-02 gives the real reasons). True as stated, and the next beat explains constant
   velocity. Optional (21 words, 9.45 s of 10.5 s): "Each driveshaft needs a flexible joint at each end, because
   the wheel moves up and down while the differential stays put."
8. **N14, s07.rzeppa.2**: "A cage holds them in the plane that splits the angle". The offset tracks steer the balls
   into the bisecting plane; the cage keeps them together in it (CVJ-03). Acceptable for narration; the model shows
   the true offset-track geometry.
9. **N15, s08.second.1**: "the car keeps accelerating". A real car coasts while the clutch is in (about
   -0.13 m/s², SFT-07). As built, s08 holds road speed through each clutch-in (acceleration 0, PRS-13) and the car
   accelerates again after the lock (about 1.1-1.2 m/s² in 1st and 2nd). It never slows on screen, so the line
   is fine as a summary of the beat.

**Storyboard (visual notes), not narration**

- **V1, s04.ratios HUD**: now reads "5th (0.81:1)". Fixed.
- **V2, s02 factors**: the inline4 note still says "~200x"; as built the scene runs 230 × / 198.3 × / 85 × (PRS-02,
  PRS-13) and the badge shows the true factor. Fine.
- **V3, s05 "SLOW x150"**: as built (150 × constant). The headset, 5th pair, input-gear dogs and 5th dog ring stay
  out of frame until the synchroniser has slowed the input side, and no flywheel is in shot (PRS-04).
- **V4, s08**: "rpm drops to ~1790 … (40 km/h at 3000)" and "~2010" are correct (1787 rpm, 40.5 km/h, 2011 rpm).
- **V5, s02 valve timing**: the compression note now reads "Intake valves close ~50 deg after BDC (~1.9 s in)".
  Fixed. With the as-built cam at 198.3 × (1° = 0.0389 s): intake beat, exhaust valves 0.30 mm open at the start and
  closed 0.39 s in; compression beat, intake valves 5.4 mm open at the start, below 1 mm at 1.30 s, closed at 1.94 s,
  spark at 6.42 s; power beat, exhaust valves start to open 5.06 s in; exhaust beat, intake valves start to open at
  6.61 s.
- **V6, s04.ratios glow**: the note now says "headset + selected pair (4th: dogs, sleeve, hub only)". Fixed.
- **V7, s03**: resolved. The release runs at 56 × (the disc visibly slows, 850 → 762 rpm, with the pedal press
  taking 0.25 s real) and the take-off at 8 × (bite → lock 1.15 s real) after a hard cut (PRS-12). The HUD reads
  DISENGAGED → SLIPPING (51.92 s) → ENGAGED (60.62 s) and the car rolls from 51.4 s (CLU-13).
- **V8, s02 firing**: fixed; as built at 85 × all four firings show, and the flywheel stays in view with its ring
  gear motion-blurred into a band (PRS-02, PRS-11).
- **V9, s07 plunge**: fixed; the note says the rollers slide outboard on both bump and droop, and s07 shows the
  plunge at true scale with a readout.
- **V10, s06 turn**: resolved. s06 now holds 15 km/h through the turn (DIF-07), so in absolute terms the outer
  wheel speeds up (130.5 → 149.8 rpm) and the inner slows (130.5 → 111.1 rpm) while the case stays at 130.5 rpm,
  exactly as narrated. The spider close-up runs at 12 × (≥ 10.6 ×).
- **V11, s05 clutch_out**: the storyboard's "engine rpm settles at ~1790" reads 1778 rpm as built (coasting,
  SFT-07). Within the "~"; no change needed.

## 22. Notes for builders and the orchestrator

Resolved in the as-built model:

- **O1, cam sprockets clash**: resolved. CAM_CENTRE_SPACING = 135.52 mm, 2.70 mm tip clearance, 126 links (VLV-04).
- **O2, reverse pinion undercut**: resolved. Profile shift +0.15 / -0.15 / +0.15 (REV-05).
- **O3, clutch free play and fork geometry**: resolved. Free play before the master piston (`kin.clutch_geometry`,
  spec MASTER/SLAVE_WORKING_STROKE), effective fork 1.655 with arms 69.7 : 115.4 mm, slave pushing rearward
  (HYD-03, HYD-05), diaphragm lever 4.24 while lifting (CLU-05). On-screen slave stroke labels must read 14.9 mm.
- **O4, ring gear side**: resolved. Ring on -X, pinion at the front of the ring (FD-04).
- **O5, shift linkage**: resolved. Class-1 lever (knob forward = rail rearward), 1-2 rail at +X, heads 4.67 mm apart
  (SEL-04 to SEL-06).

Still open:

- **O6, aliasing**: every scene's `build()` runs `Track.validate(aliasing=...)` with no violations (PRS-04,
  PRS-13). The 132T flywheel ring is the strictest part (≥ 223 × at idle in any shot that shows it, unless blurred).
  What each scene relaxes, and why:
  - s01: nothing turns (engine off, parked).
  - s02: the ring gear is checked only in frames where it is in frame and its per-object motion blur does not hide
    strobing (none, by construction, PRS-02); this accepts 0.37-0.39 pitch/frame in the 198.3 × stroke beats under
    the 1-frame shutter. Sprockets, chain and flywheel bolt holes are checked in every frame.
  - s03: everything is checked strictly at 230 ×. At 56 × the ring gear is checked only where it is visible (frustum
    and occlusion by the flywheel) and not smeared ≥ 1 pitch by the 0.75-frame shutter. After the cut (8 × / 10 ×,
    0.4-frame shutter) the fingers (18), cover/flywheel bolts (6), crank bolts (8), facing rivets (24), hub splines
    (23), damper springs (6), 26T input gear and its dog ring (32) are relaxed, as the brief allows for
    motion-blurred frames: they move 0.6-3.4 pitch/frame. The cover's 3-fold pockets, windows and straps (≤ 0.35)
    and both witness marks stay checked everywhere. Open: the 6 cover bolts (0.70 pitch/frame) and 8 crank bolts
    (0.93) are smeared only 0.28 / 0.37 pitch, inside PRS-04's 0.35-2 band, so if they are visible in the take-off
    shot they may read as turning backwards; the witness mark on the cover shows the true direction.
  - s04: everything is checked strictly; only the frame pair across each hard cut is skipped, and the needle
    cages only while the synchro is exploded.
  - s05: no motion blur; every gear and dog ring is checked, with the headset, 4th-gear dogs, 5th pair and 5th dog
    ring masked to the frames in which they are inside the camera frustum (they are kept out of frame until the
    synchroniser has slowed the input side).
  - s06: the tyre tread (64) and brake-disc vents (36) are checked only while a rear tyre is inside the frustum;
    everything else always.
  - s07: everything is checked strictly until the ramp to real time (35.25 s), then relaxed under motion blur
    (shutter 0.25 → 0.5 frame).
  - s08: real time with motion blur on every frame (shutter 0.5): all masks are empty, and the per-frame steps are
    only printed for the record (PRS-05).
- **O7, tyre radius**: the 2.1% tread-speed mismatch remains (PRS-09). It can be ignored, or tread rotation scaled
  by 0.305/0.3115.
- **O8, propshaft whirl**: a slip yoke is now shown, but the one-piece shaft would whirl at about 7,650 rpm (about
  215 km/h, PRP-06). No beat drives near that speed; PRP-06 and AB-09 record it.
- **O9, clutch naming**: spec CLUTCH_BITE_LO (0.22) and CLUTCH_BITE_HI (0.50) bound the engagement (slip) zone. In
  driver terms the bite point is the **0.50** end (CLU-06); the overlay's pedal-bar band shows the whole zone, which
  is fine. A label that says "bite" should point at 0.50.
- **O10, unused end-to-end spec ratios**: spec still defines RELEASE_FORK_RATIO (1.80), DIAPHRAGM_LEVER_RATIO
  (5.0), MASTER_STROKE (23.33 mm) and SLAVE_STROKE (16.19 mm). They ignore the free play and nothing in the model
  uses them. HUDs and labels must use 1.655, 4.24, 21.5 mm and 14.9 mm.

## Appendix: reproduce the key numbers

```python
import math, sys; sys.path.insert(0, ".")
import numpy as np
from carviz import spec as S, kin
r = S.GEAR_RATIOS
print({g: round(v, 3) for g, v in r.items()}, S.FINAL_DRIVE, S.GEARBOX_CENTRE_DISTANCE * 1e3)
print({g: round(S.road_speed_kmh(3000, g), 1) for g in (1, 2, 3, 4, 5)}, "1->2 lands at", 3000 * r[2] / r[1])
# clutch (as built: free play before the master piston)
print("master/slave working mm", S.MASTER_WORKING_STROKE * 1e3, S.SLAVE_WORKING_STROKE * 1e3,
      "fork", S.RELEASE_FORK_RATIO_EFFECTIVE, "per-face clearance mm", (1.8 - 0.65) / 2)
print({p: {k: round(float(v) * 1e3, 2) for k, v in kin.clutch_geometry(p).items() if k in ("master", "slave", "bearing", "plate_lift")}
       for p in (0.22, 0.5, 1.0)})
# cam and chain
for kind in ("intake", "exhaust"):
    g = kin.cam_geometry(kind)
    print(kind, "flank R mm", g["rf"] * 1e3,
          "lift at TDC overlap mm", float(kin.valve_lift(math.radians(360), 1, kind)) * 1e3)
p = S.CHAIN_PITCH
print("42T tip mm", p * (0.6 + 1 / math.tan(math.pi / 42)) * 1e3, "cam spacing", S.CAM_CENTRE_SPACING * 1e3,
      "chain mm per crank turn", float(kin.chain_travel(2 * math.pi)) * 1e3)
# propshaft (joint centres 44 mm inside the flanges)
L1 = S.Y_GEARBOX_REAR - S.Y_PINION_FLANGE - 2 * 0.044
beta = math.atan2(S.Z_CRANK - S.Z_PINION, L1); Lj = math.hypot(L1, S.Z_CRANK - S.Z_PINION)
print("prop beta deg", math.degrees(beta), "ripple %", (1 / math.cos(beta) - 1) * 100)
Do, Di = 0.065, 0.065 - 2 * 0.0018
wc = (math.pi / Lj) ** 2 * math.sqrt(210e9 * (Do**2 + Di**2) / 16 / 7850)
print("whirl rpm", wc * 60 / (2 * math.pi), "km/h in 4th", wc / S.FINAL_DRIVE * S.ROLLING_RADIUS * 3.6)
# driveshaft
L = S.X_WHEEL_HUB - S.X_INNER_JOINT_CENTRE; s = S.SUSPENSION_TRAVEL
print("joint spacing", L, "angle", math.degrees(math.asin(s / L)), "plunge mm", (L - math.sqrt(L * L - s * s)) * 1e3)
# synchro and differential
print("blocker index deg", S.BLOCKER_INDEX * 360 / S.DOG_TEETH)
Ri, Ro = 5 - S.TRACK_REAR / 2, 5 + S.TRACK_REAR / 2
print("inner/outer", Ri / Ro, "inner/mean", Ri / 5, "outer/mean", Ro / 5)
print("min slowmo = rpm * N / 504; s02 ring at 850 rpm:", 850 * 132 / 504, "s06 tread at 15 km/h:",
      15 / 3.6 / S.ROLLING_RADIUS * 60 / (2 * math.pi) * 64 / 504)
```
