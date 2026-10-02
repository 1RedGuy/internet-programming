# Brief: engine assembly — `carviz/assemblies/engine.py`, `tools/test_engine.py`, prefix `eng_`

2.0 L DOHC 16-valve inline-4, port-injected, longitudinal, upright cylinders, cylinder 1 at
the FRONT (+Y). All numbers in spec.py (bore/stroke 86, rod 145, bore pitch 94, deck height,
valve timing/lift, sprockets 21T/42T, chain pitch 9.525, flywheel 300 mm with 132T ring
gear, crank axis at (0, y, Z_CRANK)). Intake side = -X (left), exhaust side = +X (right).
Crank turns clockwise seen from the front (kin handles this).

## Parts (minimum)
* Static: cylinder block (grey cast iron, `cast_iron`; bores with liners' machined finish,
  water-jacket cavities around the bores in the cut, 5 main bulkheads, skirt, oil-pan rail),
  main bearing caps with bolts, cylinder head (`cast_aluminium`; pent-roof combustion chambers,
  intake and exhaust ports, valve guides, cam journals/bearing caps, spark-plug wells),
  cam cover (`cast_aluminium` or `plastic_black`), timing cover (`cast_aluminium`),
  oil pan/sump, intake manifold with 4 runners + plenum + throttle body (on -X),
  exhaust manifold/header (cast iron or steel tubes, on +X), 4 spark plugs (`ceramic`
  insulator, steel shell, electrode at the chamber roof), chain guides + tensioner,
  engine mounts brackets (optional), head gasket line.
* Moving: crankshaft (`steel_forged`, machined `steel_ground` journals; flat-plane I4
  throws: 1&4 at 0 deg, 2&3 at 180 deg per spec.CRANKPIN_PHASE_DEG; 5 main journals,
  counterweights, front snout with crank sprocket 21T + pulley/damper, rear flange),
  4 connecting rods (I-beam, big-end caps + bolts, small end on the gudgeon pin),
  4 pistons (`machined_aluminium`, ring lands + 3 rings, gudgeon pin, slightly domed/valve
  reliefs), 16 valves (stem `steel_ground`, head; intake heads 33 mm, exhaust 28 mm, two per
  side per cylinder, inclined ~±20 deg from the cylinder axis as in a pent-roof head),
  valve springs (helical coils, `steel_dark`, compress with lift via a cheap method such as
  scale along the spring axis or shape keys), retainers, bucket tappets, 2 camshafts
  (lobes generated from `kin.cam_profile` and phased with `kin.cam_lobe_psi(cyl, kind,
  valve_dir_psi)` where valve_dir_psi is the direction from the cam centre toward its
  valve/tappet), cam sprockets 42T, timing chain (roller chain wrapping crank sprocket and
  both cam sprockets, guides and tensioner; links move along the path by
  `kin.chain_travel(theta)`; sprocket teeth must stay engaged with the rollers — the chain
  speed equals sprocket pitch-radius x angular speed, so phase sprockets so rollers sit in
  tooth gaps; an Array+Curve-modifier chain rig is acceptable if cheap), flywheel
  (`cast_iron` body, `steel_machined` friction face, ring gear 132T via gears.py, bolts).
* Combustion visualisation per cylinder: gas-volume objects filling the space between piston
  crown and chamber roof, one per material `gas_intake`, `gas_compressed`, `gas_burning`,
  `gas_exhaust`, whose `cv_opacity` is baked from `kin.gas_mix(theta, cyl)` (scaled so the gas
  reads as a soft translucent tint, not a solid), height following the piston. Spark: small
  emissive flash at the plug tip with opacity from `kin.spark(theta, cyl)`. Optional faint
  intake/exhaust flow tint in the ports while the valves are open.

## Motion (from the Track, all through kin)
crank/flywheel/sprocket/pulley spin = track.theta_e; pistons = kin.piston_height; rods via
kin.slider_crank (big end location + rod_theta); camshafts spin = kin.cam_angle(theta_e) (+
lobe phases); valves/buckets/retainers translate along each valve axis by kin.valve_lift;
springs compress accordingly; chain travel = kin.chain_travel(theta_e).
Invariants to test: piston crown never touches the head or valves (check valve-to-piston
clearance during overlap at TDC — real engines have valve reliefs; ensure clearance > 0);
valve heads seat flush when closed; cam nose touches the bucket at peak lift; cam turns
once per two crank turns; rod never intersects the cylinder bore/skirt.

## Cutaway variants needed by the storyboard (opts['cutaways'] = list)
* `'long'` — longitudinal section through the cylinder axes, removing the -X half of
  block, head, cam cover, timing cover, sump, intake manifold (scene 2 overview: all four
  pistons, rods and crank visible from the left). Moving parts stay whole.
* `'cyl1'` — transverse section through cylinder 1 (plane perpendicular to Y through the
  cylinder-1 axis or through one valve pair, your choice for the best view), removing
  everything in FRONT of it (timing cover/chain region included), giving the classic
  four-stroke cross-section seen from the front: piston, rod, crank throw, both cams,
  intake + exhaust valves, ports, spark plug.
* `'front'` — timing cover and cam cover removed/cut so the chain, sprockets, cams and
  their phasing are visible from the front-top.
* `'none'`.
Build only the variants requested (default `['none']`). Housing pieces of each variant are
separate objects (kept + removed) listed in `meta['cutaway_pieces'][variant]`.
Anchors: `cyl1..cyl4` (top of each bore), `piston1`, `conrod1`, `crankshaft`, `cam_intake`,
`cam_exhaust`, `intake_valve1`, `exhaust_valve1`, `spark_plug1`, `timing_chain`,
`crank_sprocket`, `cam_sprocket`, `flywheel`, `block`, `head`.
Explode: not required (optional for flywheel).
`meta['power_path']`: list of part names to glow for "the engine" (crank, flywheel, pistons
etc.).
