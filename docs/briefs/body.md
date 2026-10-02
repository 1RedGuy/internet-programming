# Brief: car body + interior — `carviz/assemblies/body.py`, `tools/test_body.py`, prefix `body_`

A believable modern compact RWD saloon/coupé (think BMW 3-series E90 / Toyota GT86
proportions) built procedurally — this is the first thing the viewer sees (scene 1 opens on
the opaque car in a light studio), so the silhouette must read as a real car, not a toy.
Dimensions from spec: length 4.45 m (front bumper y=+0.87, rear y=-3.58), width 1.76 m, height
1.42 m, wheelbase 2.62, tracks 1.47/1.48, tyre OD 0.632 (wheel arches with ~25 mm clearance
and room for steering lock and ±60 mm wheel travel), ground clearance ~0.13 m.

## Approach
Loft/subdivision modelling from section curves (side profile, plan view, cross-sections)
into a clean quad cage + Subdivision Surface APPLIED at build time (no live subsurf), then
cut wheel arches, door/hood/boot panel lines (thin grooves or seams), window openings.
Separate objects: `body_shell` (`car_paint`), `body_glass` (windscreen, side windows, rear
screen, `glass`), `body_trim` (black window surrounds, grille, mirror bases: `plastic_black`),
`body_lights_front/rear` (lens glass + reflectors; emissive off), `body_bumpers` (painted),
`body_mirrors`, `body_handles`, `body_underbody` (floor pan `paint_black` with the
transmission TUNNEL, firewall, inner wheel arches — must not intersect the drivetrain:
engine block/head (x ±0.25 at the head, top ~0.80 m), bellhousing, gearbox case, propshaft
(65 mm tube on the centreline from y=-1.12 to -2.40, z 0.36 -> 0.305), diff (around
y=-2.62, z=0.305, ~0.35 m wide), exhaust (optional)), `body_interior`: 2 front seats + rear
bench (`plastic_black`/fabric-like dark material), dashboard, steering wheel + column at
spec.STEERING_WHEEL_CENTRE, brake + throttle pedals (spec.BRAKE_PEDAL_X/THROTTLE_PEDAL_X,
same pedal box as the clutch pedal which belongs to the clutch assembly), centre console
with an opening for the gear lever at spec.SHIFT_KNOB_REST (lever belongs to the gearbox).
Hood line must clear the engine (cam cover top ~0.80 m) — hood ~0.90-0.95 m at the engine.

## Presentation
* Scenes fade the body: every body object honours cv_opacity (x-ray look at ~0.12-0.25).
  Provide `meta['groups']`: 'exterior' (shell, bumpers, trim, lights, mirrors, handles,
  glass), 'interior', 'underbody' so scenes fade groups separately.
* Option `opts['xray_edges']` (optional): faint edge/silhouette emphasis so the x-ray shell
  still reads as a car shape when at low opacity — only if cheap in Cycles.
* No moving parts (doors closed). `_driver` may do nothing; the car's world motion is applied
  by the scene to the car root, not here.
Anchors: `hood`, `windscreen`, `roof`, `front_bumper`, `door_left`, `cabin`, `firewall`,
`tunnel`.

## Verification
Render the opaque car in `lighting.setup_studio('light')` from front-3/4, side, rear-3/4 and
top at 640x360 (Cycles <=16 spp) — compare against your knowledge of real car proportions
(greenhouse ~1/3 of height, A-pillar rake ~60 deg from vertical, short front overhang,
wheels filling the arches). Then x-ray renders (cv_opacity 0.18) with simple stand-in boxes
for engine/gearbox/propshaft/diff at their spec positions to show nothing intersects and the
drivetrain reads through the shell. Iterate until the car looks like a real production car.
Stand-in wheels: use simple cylinders of the right size in tests (real wheels come from the
wheels assembly).
