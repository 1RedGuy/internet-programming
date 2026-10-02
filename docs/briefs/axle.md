# Brief: propshaft + final drive/differential — `carviz/assemblies/axle.py`, `tools/test_axle.py`, prefix `axl_`

## Propeller shaft
One-piece steel tube (spec.PROPSHAFT_TUBE_D) from the gearbox output flange (0,
spec.Y_GEARBOX_REAR, spec.Z_CRANK) to the pinion flange (0, spec.Y_PINION_FLANGE,
spec.Z_PINION), a Hooke (Cardan) joint at each end (yokes, cross/spider with 4 needle-bearing
caps, `steel_machined`/`steel_dark`), slip yoke at the front, balance weights. The shaft is
inclined (z drop over its length): both joints work at the same angle beta in a
Z-arrangement so the pinion follows the gearbox output exactly. Kinematics:
flanges spin with track.theta_out (pinion side identical), the tube spins with
`kin.hooke(theta_out, beta)`; crosses oriented so each arm stays in its yoke (exact).

## Final drive + open differential (rear axle centre (0, spec.Y_DIFF, spec.Z_DIFF))
* Diff housing / carrier (`cast_iron` or `cast_aluminium`, IRS-style centre section bolted
  to the subframe: cover with fins, mounting ears, filler plug, output-shaft seals), cutaway
  variants `'half'` (remove the upper/rear half so the ring, pinion and spiders are visible
  from above-behind; kept + removed pieces) and `'none'`.
* Pinion (10T, spiral bevel, `steel_machined`) with shaft, two taper-roller bearings and the
  companion flange; pinion axis along Y on the ring-gear centre line (spec: no hypoid offset).
* Ring gear (41T spiral bevel, pitch diameter spec.RING_PITCH_DIAMETER) bolted (bolts!) to
  the differential case on the RIGHT (+X) side of the pinion axis, pinion meshing at the
  FRONT of the ring (this gives forward wheel rotation for normal engine rotation; derived
  in spec docstring: ring turns about -X).
* Differential case (`cast_iron`, windows so the gears are visible), cross-pin (`steel_ground`)
  with retaining bolt, 2 spider (pinion) gears 10T and 2 side gears 16T (straight bevel), thrust
  washers. Side gears are splined to the left/right output stubs/inner joint housings at
  spec.X_DIFF_OUTPUT (the halfshafts themselves belong to another assembly; build only the
  stubs/flanges up to x = +-spec.X_DIFF_OUTPUT).

## Motion
case + ring: track.theta_case (transverse convention, under a transverse pivot); pinion:
track.theta_out + mesh phase (compute the bevel mesh phase so teeth interleave; verify with
collide at 24+ frames and visually); side gears: track.theta_RL / theta_RR (+ phases);
spiders: ride with the case and spin about the cross-pin by ±kin.spider_spin(theta_RL,
theta_RR) — derive the sign from your geometry and PROVE with collide that spider/side-gear
teeth never intersect during a turn (test program with curvature 1/5 m^-1).
Rates to verify by reading back baked keys: pinion = 4.10 x case; case = mean of the two
side gears; straight line -> spiders still relative to the case.
Explode offsets for scene 6 'diffparts': case halves/windows (if split), cross-pin, spiders
(along the pin axis), side gears (along ±X), ring gear (+X). Use explode carriers.
Anchors: `propshaft`, `ujoint_front`, `ujoint_rear`, `pinion`, `ring_gear`, `diff_case`,
`spider_gear`, `side_gear_left`, `side_gear_right`, `cross_pin`, `diff_housing`.
`meta['power_path']`: ordered part names (propshaft -> pinion -> ring -> case -> spiders ->
side gears -> stubs).
Also publish `meta['power_groups'] = {'prop': [...propshaft part names...], 'diff': [...pinion,
ring, case, spiders, side gears, stubs...]}` (used by carviz/assemblies/car.py for the
power-path glow in scenes 1 and 8).
