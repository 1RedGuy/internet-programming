# Brief: driveshafts, CV joints, hubs, wheels, brakes, suspension — `carviz/assemblies/wheels.py`, `tools/test_wheels.py`, prefix `whl_`

## Rear driveshafts (both sides; mirrored)
From the diff output at x = ±spec.X_DIFF_OUTPUT (z = spec.Z_DIFF, y = spec.Y_DIFF) to the
wheel hub at x = ±spec.X_WHEEL_HUB (wheel centre z = spec.WHEEL_CENTER_Z + suspension
offset `track.susp_RL/RR`).
* Inner joint: plunging TRIPOD joint — housing ("tulip", 3 roller tracks, splined/flanged to
  the diff stub, `steel_machined`), tripod spider with 3 rollers (needle-bearing rollers) on
  the shaft end. Rollers slide along the tracks as the shaft length/angle changes (plunge).
* Shaft: solid bar spec.HALFSHAFT_D with splined ends, rubber boots (`rubber`, convoluted,
  fade-able separately so scenes can hide them).
* Outer joint: RZEPPA ball joint — outer race "bell" with 6 ball grooves (stub axle into the
  hub), inner race on the shaft, cage with 6 windows, 6 balls (`steel_ground`). The balls must
  sit in the plane that bisects the joint angle at every frame (this is WHY it is constant
  velocity) and stay in their grooves/cage windows. Cutaway variant `'cv_cut'`: half of the
  outer race (and boot) removed (kept + removed pieces) to show balls, cage and inner race.
  Tripod housing also gets a cutaway variant `'tripod_cut'`.
* Kinematics: keep the shaft length (tripod centre to Rzeppa centre) constant; the tripod
  centre slides along the diff output axis (plunge = how much); outer joint angle = angle
  between shaft and wheel axis. Spin: shaft, races, cage, balls, wheel all advance with the
  wheel angle (track.theta_RL / theta_RR — CV joints transmit equal angles). Verify: balls
  in the bisecting plane (numeric), no interpenetration (collide) at 24+ frames while the
  suspension moves ±spec.SUSPENSION_TRAVEL, plunge values printed.

## Wheels, hubs, brakes, suspension (all 4 corners)
* Wheels: 16" alloy (spec.RIM_DIAMETER), 5- or 10-spoke design, `rim_alloy`, lug nuts,
  centre cap; tyres 205/55 R16 (`tire_rubber`, real tread pattern with circumferential
  grooves + sipes; sidewall bulge; modelled radius spec.TYRE_MESH_RADIUS; wheel centre at
  z = spec.WHEEL_CENTER_Z). Rear wheels centred at x = ±spec.TRACK_REAR/2, y = spec.Y_REAR_AXLE;
  front at x = ±spec.TRACK_FRONT/2, y = 0.
* Hubs + wheel bearings, ventilated brake discs (`cast_iron` with machined faces), calipers
  (painted, static relative to the upright), backing plates.
* Rear suspension: simplified multi-link/semi-trailing arms, upright (knuckle), coil spring
  + damper — enough to read as real and to show the wheel moving up and down; the upright and
  arms follow track.susp_RL/RR. Front: MacPherson strut + lower arm, steering knuckle; front
  wheels steer by track.steer_FL / steer_FR (about a near-vertical axis) and spin with
  track.theta_FL / theta_FR.
* Spin: wheels/hubs/discs about the wheel axis with the transverse convention (positive =
  forward rolling). Calipers do not spin.

Anchors: `driveshaft_left/right`, `inner_joint_right`, `outer_joint_right`, `balls_right`,
`cage_right`, `inner_race_right`, `outer_race_right`, `tripod_rollers_right`, `wheel_RR`,
`tire_RR`, `brake_disc_RR`, `wheel_FL`... `meta['power_path']`: shafts, joints, hubs, rear
wheels.
Presentation keys: none required; optional `boot_opacity`.
Test with a program that drives forward at ~10 km/h in slow motion with the RR suspension
oscillating ±60 mm, and a left turn (curvature 1/5) to check steering and wheel speeds.
