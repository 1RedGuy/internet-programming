# Storyboard — How a Manual Car Works

Generated from `carviz/timeline.py` (timing + narration + visual intent). Timestamps are global (video) time; scene-local times are in brackets. Every mechanical motion is computed from the drivetrain state (`carviz/state.py`); cameras, labels and HUD are code-driven (`scenes/`).

**Look:** realistic studio cutaway style — PBR cast iron/aluminium, machined steel, brass synchro rings, friction material; soft studio lighting; subtle depth of field; red-painted section faces like real training cutaways; a single warm accent glow for the power path. **Overlay:** minimal part labels with leader lines, gear indicator, rpm, km/h, slow-motion badge; bottom-centre kept free for subtitles.

## 1. Overview — 0:00.0–0:32.0 (32.0 s, 768 frames)

_The whole car, then the power path from engine to wheels._

**0:00.0–0:07.0** [0.0–7.0 s] `s01.car`  
*Narration:* “This is a front-engine, rear-wheel-drive car with a five-speed manual gearbox.”  
*Picture:* Studio. Opaque car, 3/4 front-left view, slow orbit at eye level (50 mm). Title 'How a Manual Car Works' fades in and out. Car parked, engine off: drivetrain static.

**0:07.0–0:14.0** [7.0–14.0 s] `s01.inside`  
*Narration:* “Let's look inside, at the parts that turn burning fuel into turning wheels.”  
*Picture:* Camera pushes in over the front wing. Body fades from opaque to a faint x-ray shell; the camera passes through the bodywork and the whole drivetrain is revealed.

**0:14.0–0:29.0** [14.0–29.0 s] `s01.path`  
*Narration:* “Power starts in the engine. It flows through the clutch and gearbox, along the propeller shaft to the differential at the rear axle, and out through two driveshafts to the wheels.”  
*Picture:* High 3/4 view travelling front to rear along the drivetrain. A warm glow sweeps the power path, each part lighting and getting its label as it is named: Engine, Clutch, Gearbox, Propeller shaft, Differential, Driveshafts, Rear wheels.

**0:29.0–0:32.0** [29.0–32.0 s] `s01.follow`  
*Narration:* “Let's follow it.”  
*Picture:* Camera dives toward the engine; body and chassis fade away; hand-off to the engine scene.

## 2. The engine — 0:32.0–1:54.5 (82.5 s, 1980 frames)

_Inline-4, four-stroke cycle, valvetrain, firing order, flywheel._

**0:32.0–0:40.0** [0.0–8.0 s] `s02.inline4`  
*Narration:* “The engine is an inline four: four cylinders in a row, each with a sliding piston.”  
*Picture:* Engine alone, dark studio. Block and head cut away down the cylinder centreline. Slow orbit. Labels: cylinder numbers 1-4 (1 at the front), Piston. Engine idling 850 rpm shown ~200x slowed.

**0:40.0–0:47.0** [8.0–15.0 s] `s02.crank`  
*Narration:* “Connecting rods link the pistons to the crankshaft, turning up-and-down motion into rotation.”  
*Picture:* Lower angle on the bottom end through the cutaway. Rods and crankshaft highlighted. Labels: Connecting rod, Crankshaft.

**0:47.0–0:53.0** [15.0–21.0 s] `s02.cycle`  
*Narration:* “Each cylinder repeats a four-stroke cycle over two crankshaft turns.”  
*Picture:* Push in to a cross-section of cylinder 1 (valves, ports, spark plug, piston). A 4-box stroke strip appears. Cylinder 1 reaches TDC at the end of this beat.

**0:53.0–1:00.0** [21.0–28.0 s] `s02.intake`  
*Narration:* “Intake: the piston moves down, drawing air and fuel in through the open intake valves.”  
*Picture:* Cylinder 1, crank 0-180 deg of its intake stroke. Intake valves open, cool blue charge fills the cylinder. Stroke strip: INTAKE. Label: Intake valve.

**1:00.0–1:07.0** [28.0–35.0 s] `s02.compression`  
*Narration:* “Compression: the intake valves close, and the rising piston squeezes the mixture.”  
*Picture:* Intake valves close ~50 deg after BDC (~1.9 s in); then the piston rises with both valves shut and the charge becomes denser. Spark plug fires ~15 deg before TDC at the very end. Strip: COMPRESSION. Label: Spark plug.

**1:07.0–1:14.0** [35.0–42.0 s] `s02.power`  
*Narration:* “Power: a spark ignites the mixture, and the hot gas forces the piston down.”  
*Picture:* Combustion glow (orange) fills the chamber and fades as the piston is driven down. Strip: POWER.

**1:14.0–1:21.0** [42.0–49.0 s] `s02.exhaust`  
*Narration:* “Exhaust: with the exhaust valves open, the rising piston pushes the burnt gas out.”  
*Picture:* Exhaust valves (already opening near the end of the power stroke) are open; grey-brown gas leaves through the exhaust port. Strip: EXHAUST. Label: Exhaust valve.

**1:21.0–1:33.5** [49.0–61.5 s] `s02.valvetrain`  
*Narration:* “Camshafts open the valves. A timing chain drives them from the crankshaft at exactly half its speed, because each valve opens only once every two turns.”  
*Picture:* Camera rises and orbits to the front. Cam cover removed, timing cover cut away. Labels: Intake camshaft, Exhaust camshaft, Timing chain, Crank sprocket 21T, Cam sprocket 42T. HUD: crank and cam rpm (850 / 425).

**1:33.5–1:46.0** [61.5–74.0 s] `s02.firing`  
*Narration:* “Only the power stroke drives the crankshaft, so the cylinders take turns, firing in the order one, three, four, two: a power stroke every half turn.”  
*Picture:* Side view of all four cylinders. Each combustion flash happens in turn; overlay ticker 1 > 3 > 4 > 2 highlights the cylinder now firing.

**1:46.0–1:54.5** [74.0–82.5 s] `s02.flywheel`  
*Narration:* “At the back, a heavy flywheel smooths out the pulses. Its face is one half of the clutch.”  
*Picture:* Camera travels to the rear of the engine: flywheel with starter ring gear, ending on its friction face. Label: Flywheel.

## 3. The clutch — 1:54.5–2:58.0 (63.5 s, 1524 frames)

_Parts, splines, engaged, released via hydraulics, slipping take-off._

**1:54.5–2:05.5** [0.0–11.0 s] `s03.parts`  
*Narration:* “Between the engine and the gearbox sits the clutch: the flywheel, a friction disc, and a pressure plate with a diaphragm spring.”  
*Picture:* Exploded view along the crank axis: flywheel, friction disc, pressure plate + diaphragm spring + cover, release bearing, input shaft. Parts slide together and assemble. Labels on each part.

**2:05.5–2:13.0** [11.0–18.5 s] `s03.splines`  
*Narration:* “The disc's hub is splined to the gearbox input shaft, so they always turn together.”  
*Picture:* Close-up of the disc hub on the input-shaft splines. Labels: Splines, Input shaft.

**2:13.0–2:24.0** [18.5–29.5 s] `s03.engaged`  
*Narration:* “With the pedal up, the spring clamps the disc between the pressure plate and flywheel, so the input shaft turns with the engine.”  
*Picture:* Half-section of the assembled clutch, everything turning together (gearbox in neutral, engine idling). HUD: CLUTCH ENGAGED, engine rpm = input-shaft rpm.

**2:24.0–2:43.0** [29.5–48.5 s] `s03.release`  
*Narration:* “Press the pedal, and fluid flows from the master cylinder to the slave cylinder. It pushes the release fork and bearing against the spring's fingers. The spring flexes, the pressure plate pulls back, and the disc is free.”  
*Picture:* Pull back to show pedal, master cylinder, hydraulic line, slave cylinder, release fork. Pedal goes down; a pulse runs along the line; fork and bearing move; fingers deflect; pressure plate lifts. Disc slows (no longer driven). At the end, first gear is selected (HUD) and the disc stops.

**2:43.0–2:58.0** [48.5–63.5 s] `s03.slip`  
*Narration:* “To pull away, the pedal comes up slowly. The disc slips between the flywheel and pressure plate, speeding up as it passes on torque, until it matches the engine and locks.”  
*Picture:* Pedal rises; HUD shows engine rpm and disc rpm converging; status SLIPPING then ENGAGED; car starts to roll in 1st.

## 4. The gearbox — 2:58.0–4:29.0 (91.0 s, 2184 frames)

_Three shafts, constant mesh, synchronisers, linkage, ratios, reverse._

**2:58.0–3:11.0** [0.0–13.0 s] `s04.shafts`  
*Narration:* “The gearbox has three shafts. The input shaft, driven by the clutch, turns the countershaft below. The output shaft runs out the back, in line with the input.”  
*Picture:* Gearbox case becomes a cutaway. Labels: Input shaft, Countershaft, Output shaft. Neutral, clutch engaged, engine idling, car stationary (output shaft still).

**3:11.0–3:27.5** [13.0–29.5 s] `s04.neutral`  
*Narration:* “The countershaft's forward gears each mesh with a gear on the output shaft. They're always in mesh, but the output gears spin freely on bearings. Until one is locked to the shaft, no power gets through: neutral.”  
*Picture:* Slow track along the gear train. All gears turn, but the output shaft and synchro hubs stand still. Label: Free-spinning gears. HUD gear N.

**3:27.5–3:43.5** [29.5–45.5 s] `s04.synchro`  
*Narration:* “Synchronizers do the locking. A hub is splined to the shaft, and a sleeve slides on it. Each gear carries dog teeth and a cone, with a brass blocker ring between cone and sleeve.”  
*Picture:* Close-up on the 1-2 synchroniser; exploded view: hub, sleeve, blocker ring, cone, dog teeth; reassembles. Clutch pressed at the start of this beat (HUD).

**3:43.5–3:51.0** [45.5–53.0 s] `s04.lock`  
*Narration:* “Slide the sleeve over the dog teeth, and the gear is locked to the shaft.”  
*Picture:* Sleeve moves toward 1st: blocker ring stops the gear train (output is stationary), sleeve slides over the dog teeth. HUD gear 1.

**3:51.0–4:01.5** [53.0–63.5 s] `s04.linkage`  
*Narration:* “Forks on shift rails move the sleeves. Moving the lever sideways picks a rail; forward or back slides it.”  
*Picture:* Pull back: gear lever, selector finger, three rails and forks. Lever moves across the gate and back to 1st; matching rail/fork/sleeve moves. H-pattern diagram in HUD.

**4:01.5–4:21.5** [63.5–83.5 s] `s04.ratios`  
*Narration:* “In first, the input gear turns the countershaft more slowly, and a small countershaft gear drives a large output gear: the engine turns about three and a half times per output-shaft turn. Fourth locks input to output: one to one. Fifth is an overdrive.”  
*Picture:* Driving shots (hard cuts between gears): power path glows through the engaged pair. HUD: input rpm, output rpm and ratio for 1st (3.48:1), then 4th (1:1), then 5th (0.81:1). Glow: headset + selected pair (4th: dogs, sleeve, hub only).

**4:21.5–4:29.0** [83.5–91.0 s] `s04.reverse`  
*Narration:* “Reverse adds an idler gear between the shafts, so the output turns backwards.”  
*Picture:* Cut to reverse: idler highlighted, output shaft turning the opposite way. HUD gear R.

## 5. A gear shift, step by step — 4:29.0–5:19.0 (50.0 s, 1200 frames)

_1st to 2nd at 3000 rpm, slowed 150x._

**4:29.0–4:36.0** [0.0–7.0 s] `s05.intro`  
*Narration:* “Here's a shift from first to second at three thousand rpm, slowed right down.”  
*Picture:* Cutaway gearbox close on the 1-2 synchro and 2nd gear; car driving in 1st at 3000 rpm, clutch engaged. HUD: engine rpm, 2nd-gear rpm vs output rpm, pedal bar, H-pattern dot, SLOW x150.

**4:36.0–4:41.5** [7.0–12.5 s] `s05.clutch_in`  
*Narration:* “One: clutch in. The engine is disconnected.”  
*Picture:* Step card 1. Pedal bar goes down; HUD status DISENGAGED; engine rpm starts falling.

**4:41.5–4:47.0** [12.5–18.0 s] `s05.neutral`  
*Narration:* “Two: the sleeve slides out of first, into neutral.”  
*Picture:* Step card 2. Fork moves the 1-2 sleeve off 1st gear's dogs to the centre. Lever dot to N.

**4:47.0–5:01.0** [18.0–32.0 s] `s05.sync`  
*Narration:* “Three: the blocker ring's cone presses on second gear. Friction slows it, along with the countershaft, input shaft and clutch disc, until it matches the output shaft's speed.”  
*Picture:* Step card 3. Sleeve moves toward 2nd, blocker ring touches the cone and indexes to block. HUD 2nd-gear rpm falls to output rpm. Glow on the cone contact.

**5:01.0–5:09.5** [32.0–40.5 s] `s05.engage`  
*Narration:* “Four: speeds matched, the blocker ring lets the sleeve through, onto second gear's dog teeth.”  
*Picture:* Step card 4. Sleeve passes the blocker ring and engages 2nd's dog teeth. HUD gear 2.

**5:09.5–5:19.0** [40.5–50.0 s] `s05.clutch_out`  
*Narration:* “Five: clutch out. The engine is reconnected at about eighteen hundred rpm: same road speed, taller gear.”  
*Picture:* Step card 5. Pedal up, brief slip, engine rpm settles at ~1780. HUD ENGAGED.

## 6. Final drive and differential — 5:19.0–6:18.0 (59.0 s, 1416 frames)

_Propshaft, ring and pinion, open diff straight and in a turn._

**5:19.0–5:25.5** [0.0–6.5 s] `s06.prop`  
*Narration:* “The propeller shaft carries the drive back to the rear axle.”  
*Picture:* Camera travels from the gearbox tail along the spinning propshaft (U-joints) to the rear axle.

**5:25.5–5:39.5** [6.5–20.5 s] `s06.ringpinion`  
*Narration:* “There, a small pinion drives a large ring gear: ten teeth against forty-one, a final drive ratio of 4.1 to 1, turning the drive through a right angle.”  
*Picture:* Axle housing cut away. Pinion and ring gear close-up. Labels: Pinion 10T, Ring gear 41T. HUD: propshaft rpm vs ring rpm.

**5:39.5–5:51.0** [20.5–32.0 s] `s06.diffparts`  
*Narration:* “The ring gear is bolted to the differential case. Inside, two spider gears mesh with two side gears, one splined to each driveshaft.”  
*Picture:* Exploded view of the differential: case, cross-pin, 2 spider gears, 2 side gears, driveshaft stubs; reassembles. Labels.

**5:51.0–6:00.0** [32.0–41.0 s] `s06.straight`  
*Narration:* “Going straight, the spider gears don't spin on their pin. Everything turns as one, and both wheels match.”  
*Picture:* Straight-line driving: case, spiders, side gears turn as one. HUD: left/right wheel rpm equal.

**6:00.0–6:18.0** [41.0–59.0 s] `s06.turn`  
*Narration:* “In a turn, the outer wheel travels further than the inner one. The spider gears now spin on their pin, letting one side speed up as the other slows. The case turns at the average of the two.”  
*Picture:* Car enters a tight left turn (R 5 m). High view over the rear axle (x-ray). Spiders spin on the pin. HUD: inner/outer wheel rpm and case rpm = average.

## 7. Driveshafts and CV joints — 6:18.0–6:59.5 (41.5 s, 996 frames)

_Constant-velocity joints, plunge, wheels, car moves._

**6:18.0–6:28.5** [0.0–10.5 s] `s07.why`  
*Narration:* “Each driveshaft has a constant-velocity joint at each end, because the wheel moves up and down while the differential stays put.”  
*Picture:* Rear-right driveshaft from behind. Suspension moves the wheel up/down; shaft angle changes. Labels: Inner joint, Outer joint, Driveshaft.

**6:28.5–6:45.0** [10.5–27.0 s] `s07.rzeppa`  
*Narration:* “In the outer joint, six balls run in grooves between inner and outer races. A cage holds them in the plane that splits the angle, so the wheel turns at exactly the shaft's speed.”  
*Picture:* Cutaway outer (Rzeppa) joint: balls, cage, races; joint articulates. Labels: Balls, Cage, Inner race, Outer race. HUD: shaft rpm = wheel rpm.

**6:45.0–6:51.5** [27.0–33.5 s] `s07.plunge`  
*Narration:* “The inner joint can also slide, as the wheel's distance from the differential changes.”  
*Picture:* Cutaway inner (tripod) joint; rollers slide outboard in their tracks on both bump and droop. Label: Plunge.

**6:51.5–6:59.5** [33.5–41.5 s] `s07.moves`  
*Narration:* “Finally, the wheel turns, the tire grips the road, and the car moves.”  
*Picture:* Pull out; body fades back in; slow motion ramps to real time and the car drives off.

## 8. Recap — 6:59.5–7:31.5 (32.0 s, 768 frames)

_Drive through 1st, 2nd and 3rd with live readouts._

**6:59.5–7:03.5** [0.0–4.0 s] `s08.together`  
*Narration:* “Let's put it all together.”  
*Picture:* Real time. Semi-transparent car stationary in 1st with clutch pressed, engine idling. Tracking camera alongside. HUD: gear, rpm gauge, km/h, pedal, H-pattern.

**7:03.5–7:09.5** [4.0–10.0 s] `s08.first`  
*Narration:* “Clutch up in first, and the engine pulls to three thousand rpm.”  
*Picture:* Clutch slips then locks; car accelerates; rpm climbs to 3000 (24 km/h). Motion blur.

**7:09.5–7:16.5** [10.0–17.0 s] `s08.second`  
*Narration:* “Clutch in, second gear, clutch out: the revs drop, and the car keeps accelerating.”  
*Picture:* Quick shift to 2nd: rpm drops to ~1790, then climbs again (40 km/h at 3000).

**7:16.5–7:21.5** [17.0–22.0 s] `s08.third`  
*Narration:* “Then third.”  
*Picture:* Shift to 3rd: rpm drops to ~2010 and climbs.

**7:21.5–7:31.5** [22.0–32.0 s] `s08.summary`  
*Narration:* “Engine, clutch, gearbox, propeller shaft, final drive, differential, and driveshafts: one chain of gears and shafts, turning fuel into motion.”  
*Picture:* Camera pulls back and up as the car cruises; power path glows once more; fade to black.

