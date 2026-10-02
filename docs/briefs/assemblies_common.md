# Brief: rules common to every assembly agent

You build one subsystem of the car for the explainer film "How a Manual Car Works"
(realistic, technically correct, rendered in Cycles at 1280x720). Read first:
`ARCHITECTURE.md`, `carviz/spec.py`, `carviz/kin.py`, `carviz/state.py` (Program/Track),
`carviz/rig.py`, `carviz/gears.py`, `carviz/meshutil.py`, `carviz/materials.py`,
`carviz/lighting.py`, `carviz/collide.py`, `carviz/timeline.py` (what the film says and shows
— the `visual` column tells you which views of your parts the scenes need).

## Ownership
* You own exactly the files named in your brief (your assembly module + `tools/test_<name>.py`).
  Never edit other files. If a shared module (spec/kin/gears/meshutil/materials/rig) needs a
  change, do NOT edit it — work around it locally in your module and list the request in
  your final report. (You MAY append new constants for your subsystem to the bottom of your
  own module, not spec.py.)
* Name every object with your prefix (given in the brief) and put all of them in a
  collection named after the assembly (`rig.collection(...)`).

## Interface (ARCHITECTURE.md §4)
`build(opts=None) -> carviz.rig.Assembly` with `root` (Empty at the spec anchor, no rotation),
`parts` (stable names, documented in the module docstring), `anchors` (label points),
`explode` (per-part offsets at factor 1, for exploded views), `meta` (anything useful:
axial positions, part groups such as power-path lists, cutaway piece names), and
`_driver(asm, track, presentation)` which bakes keyframes for every moving part from the
Track for all `track.frames`. `presentation` may carry per-frame arrays (e.g. `explode`
0..1, or scene-specific extras documented in your docstring).
* Positions: car frame, from spec. Rotations: ARCHITECTURE.md §2 — spinning parts spin
  about their LOCAL +Y via `rig.bake_spin(obj, frames, abs_angle)`; transverse parts sit
  under `rig.transverse_pivot`. All angles come from the Track through `carviz.kin`
  (e.g. `track.gb("gear_2")`, `kin.slider_crank`, `kin.valve_lift`, `kin.clutch_geometry`).
  **Never invent motion** that is not derived from the Track.
* Exploded views: the explode offset must compose with baked motion. Pattern: put each
  explodable part (or group) under an "explode carrier" Empty (`rig.explode_parent`) and bake
  the carrier's location = rest + factor*offset; the part's own spin stays on the part.
* Cutaways: build at BUILD time with `meshutil.cut_and_apply`; for each cut housing create
  BOTH the kept piece and the removed piece (the complement), so a scene can fade or slide
  the removed piece away to reveal the cut. Cut faces use material `section_cut`.
  Offer the cutaway variants the storyboard needs (listed in your brief) via `opts`.
* Every mesh gets presentation props (`materials.ensure_props(obj)` or
  `rig.set_presentation(obj, 1, 0)`), real materials from `materials.get(name)`, and smooth
  shading by angle where appropriate. No modifiers left that are expensive to evaluate per
  frame (apply them at build time), except cheap ones you document (e.g. shape keys).

## Realism and correctness bar
* Real proportions (use spec numbers; for anything not in spec, use typical values for a
  2.0 L front-engine RWD saloon and state them in your docstring).
* Recognisable, detailed but efficient geometry: bevelled/chamfered edges (no razor CG
  edges), fillets where castings have them, bolts where real parts have bolts, machined vs
  cast surfaces with the right materials. Budget: keep each assembly under ~1.5 M triangles
  at `detail='high'` (`'low'` a few hundred k) and build time under ~60 s.
* Meshing teeth must not intersect and must stay in mesh (use gears.check_mesh_2d and
  collide.check_pairs at sampled frames). Moving parts must not pass through housings or each
  other at any frame of a full cycle.

## Verification you must do (tools/test_<name>.py)
1. Build the assembly alone; print build time and triangle count.
2. Drive it with a realistic test `state.Program` (a few seconds; slow motion so motion is
   visible) and run numeric checks: collisions at >= 24 sampled frames over a full cycle,
   rotation directions/speeds vs the Track (e.g. read back baked rotation at two frames and
   compare with kin/Track values), any assembly-specific invariants listed in your brief.
3. Render stills: `lighting.setup_studio('dark')` (or as your brief says), Cycles 640x360,
   <= 16 spp, OIDN, from 3-4 informative angles (overall, close-up, cutaway, exploded) plus a
   short Workbench motion strip (e.g. 6 frames) to check motion. Put images in your scratch
   dir. LOOK at every image (Read tool) and fix what looks wrong or fake. Do at least two
   render-inspect-fix passes; keep going until a pass finds nothing significant.
4. Your final message: files, parts list (names), anchors, opts, explode keys, meta keys,
   presentation keys, build time, triangle counts, what you verified (numbers), image paths
   of your best renders, known limitations, and requests for shared-module changes.

Be a good neighbour on the 4 shared CPU cores: one Blender process at a time, small test
renders, no background jobs left running.

## Notes from wave 1 (gears / materials / lighting agents)
* gears.py: bevel gears are built with local +Y pointing AT THE APEX — a gear whose apex
  direction is opposite to its conventional spin axis must have its scalar angle negated
  when baked. Place bevel pairs with `gears.bevel_pair_frames` + `gears.bevel_mesh_phase`.
* Helix hands: input + output-shaft gears `'right'`, countershaft gears `'left'`.
* Splines/dogs: use the same (n, r_in, r_out) for hub external splines, gear dog rings and the
  sleeve internal teeth (tested: n = spec.DOG_TEETH = 32, r_in 30 mm, r_out 33.5 mm).
* meshutil.cut_half / cut_quarter / cut_and_apply keep closed meshes closed; give cutaway
  objects their main material in slot 0 before cutting so only section faces get section_cut.
* Materials: use `cast_iron` (not steel) for cylinder bores. Ghosted closed shells look right at
  cv_opacity ~0.10-0.15 (two surfaces). Workbench ignores cv_opacity/cv_glow.
* Lighting: `lighting.setup_studio('dark', key_azimuth=...)` — aim the key ~45-60 deg off the
  camera azimuth so cut faces catch light.
