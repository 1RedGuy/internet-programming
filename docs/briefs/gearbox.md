# Brief: gearbox assembly — `carviz/assemblies/gearbox.py`, `tools/test_gearbox.py`, prefix `gbx_`

5-speed, 3-shaft, constant-mesh, synchronised manual gearbox for a longitudinal RWD car
(spec.py "Gearbox" section): input + output shafts coaxial on the main axis (x=0,
z=Z_CRANK); countershaft directly below at z = Z_COUNTERSHAFT; reverse idler at
`kin.reverse_idler_centre()` (relative to the main axis, in the XZ plane). Forward gears
helical (normal module 2.25, helix 25 deg, opposite hands on mating gears), reverse spur
(module 2.5). Tooth counts: headset 26/35, 1st 17/44, 2nd 24/37, 3rd 30/31, 5th 38/23,
reverse 15 -> 22 idler -> 38. 4th = input shaft locked directly to the output shaft.

## Axial layout (front -> rear; you choose exact Y values, store them in meta['y'])
case front face at spec.Y_GEARBOX_FRONT: input bearing, input gear (26T) with its 4th-gear
dog ring + cone, [3-4 synchro], 3rd (31T), 2nd (37T), [1-2 synchro], 1st (44T), reverse
(38T spur), [5-R synchro], 5th (23T), rear bearing, tail housing to spec.Y_GEARBOX_REAR with
the output flange. Countershaft gears aligned axially under their partners (35T under the
input gear, 30, 24, 17, 15 (+ idler at the reverse plane), 38). Sleeve directions MUST match
spec.SYNCHROS: sleeve moving forward (+Y) engages 2 / 4 / R; rearward engages 1 / 3 / 5.
Face widths ~14-20 mm; synchro hubs ~20 mm wide; sleeve travel spec.SLEEVE_TRAVEL; leave
realistic clearances. Total main case length should end up ~0.40-0.48 m.

## Parts
* Input shaft (clutch splines 23T at the front end reaching the clutch disc at
  spec.Y_DISC_CENTRE, pilot spigot into the crank), input gear + dog ring + cone.
* Countershaft cluster: shaft + 6 gears (separate objects so each gets its own kin phase).
* Output shaft (`steel_machined`) with: free-running gears 3, 2, 1, R, 5 each on a visible
  needle-roller bearing (cage + rollers, at least partly visible in the cut), each with a
  dog-tooth ring (spec.DOG_TEETH) and a synchro cone on the side facing its synchro;
  3 synchro hubs (external splines, slots for 3 struts/keys), 3 sleeves (internal teeth at
  (k+1/2)*2pi/N, chamfered ends, external fork groove), 6 brass blocker rings (`brass`;
  internal cone, external chamfered teeth, lugs into the hub slots), struts/keys + detent
  springs (optional but nice).
* Reverse idler gear (22T spur) on its own short shaft.
* Selector mechanism: 3 shift rails (parallel to Y above/beside the gears), forks (riding in
  the sleeve grooves, `cast_aluminium`/`steel` with brass/plastic pads), rail detents,
  interlock pins (optional), selector finger and the gear lever with ball pivot at
  spec.Y_SHIFT_LEVER on top of the tail housing, boot (`rubber`) and knob (`plastic_black`
  with an engraved-looking H-pattern if cheap) reaching spec.SHIFT_KNOB_REST. Lever motion:
  sideways tilt from track.lever_x (plane -1/0/+1), fore/aft tilt from track.lever_y
  (+1 = knob forward = rail/sleeve REARWARD — class-1 lever about the ball), consistent with
  the rail/fork/sleeve displacement of the active rail.
* Case (`cast_aluminium`): main case with bearing bosses and ribs, top cover, tail housing,
  drain/fill plugs, bolts. Cutaway variants: `'half'` (remove the -X half through the main
  axis plane so all three shafts are visible in profile; kept + removed pieces),
  `'quarter'` (remove the upper -X quadrant), `'none'`. Oil: optional thin oil film look on
  gears is in the material; a static oil level in the sump cut is a nice touch.

## Motion (Track + kin; see kin.gearbox_phases / Track.gb)
Every toothed part: `rig.bake_spin(obj, frames, track.gb(name))` with names `input_gear`,
`cs_drive`, `cs_1`, `cs_2`, `cs_3`, `cs_5`, `cs_R`, `idler`, `gear_1`, `gear_2`, `gear_3`,
`gear_5`, `gear_R`; shafts with their gears (input shaft = input_gear angle; countershaft =
any cs_* minus its phase). Output shaft, hubs, sleeves: track.theta_out. Blocker rings:
theta_out + track.blocker_<k> (the ring on the side being engaged) and a small axial move
onto the cone as the sleeve passes spec.SYNC_CONTACT. Sleeves/forks/rails axial =
track.sleeve_<k> * SLEEVE_TRAVEL. Gear-mesh phases come ONLY from kin (tooth 0 on +X at abs
angle 0). Verify: (a) every pair interleaves at 24+ sampled frames (collide.check_pairs on
pairs and gears.check_mesh_2d), (b) when a sleeve is fully engaged its internal teeth sit
between the gear's dog teeth (no overlap; dog alignment is guaranteed by state.py — check it
visually and with collide at the engaged frames), (c) read-back speeds: rpm of gear_n /
output in a test program equals 1/ratio, countershaft opposite, reverse output backwards.

## For the scenes
`meta['power_path'][g]` for g in 1,2,3,4,5,'R','N': ordered part names carrying torque in
that gear (input shaft -> ... -> output shaft) so scenes can glow them. `explode` offsets
for an exploded 1-2 synchroniser (hub, sleeve, both blocker rings, gear cones/dog rings
pulled apart along Y; ~25-40 mm spacing) — use explode carriers so spin still applies.
Anchors: `input_shaft`, `countershaft`, `output_shaft`, `gear_1`, `gear_2`, `gear_3`,
`gear_5`, `gear_R`, `idler`, `hub_12`, `sleeve_12`, `blocker_2`, `blocker_1`, `dogs_2`,
`dogs_1`, `cone_2`, `fork_12`, `rail_12`, `rail_34`, `rail_5R`, `lever`, `knob`,
`case`, `needle_bearing_2`.
Presentation keys: `explode` (0..1).
