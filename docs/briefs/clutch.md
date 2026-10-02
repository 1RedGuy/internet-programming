# Brief: clutch assembly — `carviz/assemblies/clutch.py`, `tools/test_clutch.py`, prefix `clu_`

Single dry plate, push-type diaphragm-spring clutch with hydraulic release (spec.py
"Clutch" and "Cabin / pedal box" sections). The flywheel belongs to the ENGINE assembly
(spec.Y_FLYWHEEL_FACE is its friction face); for your test build a simple stand-in flywheel
in the test script only (or import engine if it exists — do not depend on it).

## Parts
* Friction disc (centre at spec.Y_DISC_CENTRE): splined hub (23 internal splines matching
  the input shaft, gears.internal_splines), 6 torsional damper coil springs in windows,
  retainer plates with rivets, cushion segments, two `friction` facings (OD 228, ID 150)
  with rivet holes and radial grooves.
* Pressure plate (`cast_iron`, machined friction face, `steel_machined`) — moves axially by
  `clutch_plate_lift` (away from the flywheel = -Y) and spins with the engine (theta_e).
* Diaphragm spring (`steel_dark`): conical Belleville ring with 18 radial fingers; inner
  finger tips deflect toward the flywheel (+Y) by `clutch_finger` (= bearing travel) while
  the outer rim pivots on the fulcrum rings — implement with a shape key (rest -> fully
  released) whose value is baked from the Track; spins with theta_e.
* Clutch cover (pressed steel, `paint_black` or `steel_dark`) with bolts to the flywheel,
  straps, fulcrum rings; spins with theta_e.
* Release (throw-out) bearing on its guide tube: moves +Y by `clutch_bearing`; its contact
  race spins with theta_e while it touches the fingers (bearing travel > ~0.3 mm), else holds.
* Release fork (`paint_black` / pressed steel) pivoting on a ball stud
  (spec.RELEASE_FORK_PIVOT): inner end pushes the bearing, outer end pushed by the slave
  pushrod; rotation angle consistent with both (lever ratio spec.RELEASE_FORK_RATIO).
* Slave cylinder (external, on the bellhousing at spec.SLAVE_CYL_POS) with pushrod extending
  by `clutch_slave`; bleed nipple; hose union (`copper`).
* Hydraulic line from master to slave (steel pipe + flexible hose `rubber`), split into ~12
  segments listed in `meta['hydraulic_segments']` (ordered master -> slave) so a scene can
  run a glow pulse along it; optional cutaway showing `brake_fluid` inside.
* Master cylinder on the firewall (spec.MASTER_CYL_POS) with reservoir; pushrod moves by
  `clutch_master`.
* Clutch pedal (spec.CLUTCH_PEDAL_PIVOT, PEDAL_ARM, PEDAL_REST_ANGLE; pad `rubber`):
  rotates by `clutch_pedal_angle` about an axis parallel to X; return spring; small pedal-box
  bracket. Geometry of pedal -> pushrod -> master must look connected at all positions.
* Bellhousing (`cast_aluminium`) from spec.Y_BLOCK_REAR to spec.Y_GEARBOX_FRONT around the
  clutch, with the fork window and slave mount. Cutaway variants: `'half'` (remove the -X
  half, red section faces; kept + removed pieces) and `'none'`.
The input shaft belongs to the GEARBOX assembly (do not build it except as a test stand-in).

## Motion (Track fields; see state.py/kin.clutch_geometry)
disc spin = track.theta_in; pressure plate/cover/spring spin = track.theta_e;
`clutch_plate_lift`, `clutch_finger`, `clutch_bearing`, `clutch_slave`, `clutch_master`,
`clutch_pedal_angle` per frame. Invariants: when the pedal is up the disc is clamped
(facings touch flywheel face and pressure plate within 0.1 mm; disc thickness = the gap);
with the pedal down there is a visible gap of ~plate_lift on each side; fingers never pass
through the bearing; fork stays in contact with both bearing and pushrod.

## Exploded view (scene 3 'parts' beat)
`explode` offsets along the crank axis (-Y direction away from the engine) for: disc,
pressure plate+spring+cover (as one group or separate — separate is nicer: plate, spring,
cover), release bearing; spacing ~60-90 mm so all parts read separately. Include the
flywheel stand-in only in tests.
Anchors: `disc`, `facing`, `hub_splines`, `damper_springs`, `pressure_plate`,
`diaphragm_spring`, `fingers`, `cover`, `release_bearing`, `fork`, `slave_cylinder`,
`master_cylinder`, `pedal`, `hydraulic_line`, `bellhousing`.
Note: real pressure-plate lift (1.8 mm) is small; keep it physically correct in the model
(the scene may add an on-screen note or magnified inset; do not exaggerate in geometry).
