# Architecture & conventions (the contract every module follows)

Read this before touching any code. Every scene and assembly must follow these
rules so the car is consistent across the whole video.

## 0. Runtime

* Blender 5.0.1 as a Python module (`pip install bpy==5.0.1`, Python 3.11).
  Run scripts with plain `python3` (no Blender binary).  Set
  `EGL_PLATFORM=surfaceless` for Workbench renders (Mesa llvmpipe).
* Machine: 4 CPU cores, no GPU.  **Be a good neighbour**: other agents render
  at the same time.  For test renders use Workbench or Cycles at <= 640x360,
  <= 16 samples, and never run more than one Blender process at a time.
* Repo root must be on `sys.path`; every entry script does
  `sys.path.insert(0, <repo root>)`.

## 1. Units, axes, origin

* Metres, radians, seconds.  rpm only where a name says `rpm`.
* Car frame = world frame when the car is parked at the origin:
  **+X right (passenger side, LHD car), +Y forward, +Z up**.  Origin is the
  ground point under the centre of the front axle.  Rear axle at Y = -2.62.
* All dimensions, tooth counts and ratios live in `carviz/spec.py`.  Never
  hard-code a number that is in spec; import it.  If you need a new shared
  dimension, add it to spec.py (append only; never change an existing value
  without telling the orchestrator).

## 2. Rotation convention (critical)

* The drivetrain state (`carviz/state.py`) stores **scalar angles** that are
  positive in the normal running direction:
  * longitudinal members (crank, flywheel, clutch, input/output shaft,
    propshaft, pinion): + = clockwise seen from the FRONT of the car =
    right-handed about **-Y**.
  * transverse members (wheels, diff case, ring gear, side gears, driveshafts):
    + = forward rolling = right-handed about **-X**.
* **Every rotating part is modelled with its spin axis along its LOCAL +Y axis**
  and is animated ONLY through `rotation_euler[1] = -theta` (rotation_mode
  'XYZ', other components 0).  Orientation comes from a parent Empty.  With
  that, local profile angle psi (measured in the local XZ plane from +X toward
  +Z) increases with theta.
  * Transverse parts: parent Empty rotated `(0, 0, -pi/2)` maps local +Y to
    world +X, so `rotation_euler[1] = -theta` is a right-handed turn about
    world -X.  (`carviz.rig.transverse_pivot()` makes such an Empty.)
* Gear/tooth phase convention: tooth 0 of every toothed part is centred on
  local +X (psi = 0) at rotation 0.  Absolute mesh phases for every gear in the
  car come from `carviz/kin.py`, never from ad-hoc offsets in assemblies.
* Splines/dog teeth: hub external teeth and gear dog teeth at
  psi = k*2pi/N (tooth 0 on +X); sleeve internal teeth at (k+1/2)*2pi/N.  Dogs
  are aligned for engagement when (gear angle - output shaft angle) = 0 mod
  2pi/N.  `kin.py` guarantees this at engagement.

## 3. Master drivetrain state

`carviz/state.py`:  a scene builds a `Program` (keyframed *inputs* in video
time: road speed, turn curvature, clutch pedal, engine throttle target,
synchro sleeve positions, lever plane, slow-motion factor, suspension,
hard cuts) and calls `program.run()` -> `Track`, which holds per-frame numpy
arrays for every derived quantity (engine/input/output/wheel angles and rpm,
gear engaged, clutch capacity and slip, pressure-plate lift, bearing travel,
sleeve/blocker positions, lever position, car pose ...).  `Track.validate()`
enforces the physics (interlock, no shifting under load, sync before dog
engagement, locked clutch => equal speeds, no aliasing ...).

**Every moving part reads its motion from the Track (directly or through
`kin.py`).  Nothing is hand-animated.**  Presentation-only motion (exploded
view offsets, fades, glow, camera) is allowed to be keyframed by scenes, but
never rotation of a mechanical part.

Slow motion: angles integrate `omega * slowmo(t) * dt_video`.  Displayed rpm
is always the physical value.

## 4. Assemblies (`carviz/assemblies/*.py`)

Each assembly module exposes

```python
def build(opts: dict | None = None) -> Assembly
```

and returns `carviz.rig.Assembly` with:

* `root`   - an Empty at the assembly's anchor in car coordinates (spec
  positions).  Children use local coordinates relative to it; root has no
  rotation.
* `parts`  - `dict[str, bpy.types.Object]`, every named part (stable names,
  documented in the module docstring).
* `anchors`- `dict[str, (object, (x, y, z) local offset)]` label anchor points.
* `explode`- `dict[part_name, (dx, dy, dz)]` exploded-view offset at factor 1.
* `drive(track, presentation=None)` - bakes keyframes for every moving part
  for all frames of the track, using `carviz.rig.bake_*` helpers.
  `presentation` may carry per-frame arrays such as `explode` (0..1).

Options (`opts`) commonly include: `cutaway` ('none' | 'half' | 'quarter' |
part-specific), `detail` ('low' | 'high'), `collection`.

Cutaways are made at BUILD time (boolean applied, so render time is not spent
on booleans); cut faces get the `section_cut` material.

Assemblies never create cameras, lights, worlds or render settings.

## 5. Materials (`carviz/materials.py`)

`materials.get(name)` returns a cached `bpy.types.Material`.  Names:

| name | use |
|---|---|
| `cast_iron` | engine block (sand-cast grey iron), brake discs (non-machined areas) |
| `cast_aluminium` | cylinder head, gearbox case, bellhousing, diff housing, sump |
| `machined_aluminium` | machined faces on aluminium parts, pistons |
| `steel_machined` | gears, shafts, splines, synchro hubs/sleeves, CV races |
| `steel_ground` | bearing races, balls, valve stems, gudgeon pins (bright) |
| `steel_forged` | crankshaft, connecting rods (dark forged with machined journals) |
| `steel_dark` | springs, black-oxide parts, timing chain |
| `brass` | synchro blocker rings |
| `friction` | clutch facings, brake pads |
| `rubber` | boots, hoses, seals, pedal pads |
| `tire_rubber` | tyres |
| `plastic_black` | covers, connectors, knob |
| `paint_black` | pedal box, brackets, fork |
| `car_paint` | body paint |
| `glass` | windows (thin glass, no refraction cost) |
| `rim_alloy` | wheels |
| `chrome` | trim |
| `copper` | electrodes, hydraulic union |
| `ceramic` | spark-plug insulator |
| `section_cut` | faces produced by cutaways |
| `gas_intake`, `gas_compressed`, `gas_burning`, `gas_exhaust` | cylinder gas volumes |
| `brake_fluid` | fluid inside the hydraulic line cutaway |
| `floor`, `backdrop` | studio |

Every material reads two object custom properties through an Attribute node
(type OBJECT): `cv_opacity` (1 = opaque, 0 = invisible) and `cv_glow`
(0..1 warm emissive highlight used for the power path).  Set them with
`carviz.rig.set_presentation(obj, opacity=..., glow=...)` and animate with
`carviz.rig.bake_prop`.  `material.diffuse_color` is set to a sensible
viewport colour so Workbench previews read correctly.

## 6. Gears and geometry helpers

* `carviz/gears.py`: involute spur/helical external gears, internal splines,
  straight/spiral bevel gears, sprockets, dog-tooth rings.  All built with the
  axis along local +Y, tooth 0 on +X, mid-face plane at local y = 0 (bevel:
  apex on the local +Y axis, documented in the function).  Includes
  `profile_polygon(...)` and `check_mesh_2d(...)` (shapely) to prove meshing
  teeth neither intersect nor separate.
* `carviz/meshutil.py`: bmesh helpers (lathe/revolve a 2D profile around Y,
  extrude polygons, tubes along paths, bevelled boxes, boolean cut-and-apply
  with section material, smooth shading by angle).

## 7. Presentation: camera, labels, HUD, overlay

* `carviz/camera.py`: keyframed camera rigs (eye/target/lens/f-stop keys with
  smooth interpolation, orbit/push-in helpers), baked per frame; DOF focus
  follows the target.
* `carviz/labels.py`: label anchors -> per-frame 2D positions written to
  `labels.json`.  HUD widgets and values per frame -> `hud.json`.
* `carviz/overlay.py`: Pillow compositor that draws labels (leader lines),
  HUD widgets, titles onto the raw frames.  Fonts: `assets/fonts/Inter-*.otf`.
  Keep the bottom-centre 18% of the frame clear (subtitles live there).

## 8. Scenes (`scenes/sNN_name.py`)

Each scene module defines `SCENE_ID` and `build(quality) -> SceneBuild`
(see `carviz/scenebase.py`) which creates the studio, assemblies, the state
program, camera and labels.  Timing comes from `carviz.timeline` beats.
Scene modules never edit shared modules' behaviour; if a shared module needs
a change, make it backwards compatible and re-run that module's self-test.

## 9. Rendering

`python3 tools/render.py <scene> --quality preview|draft|final [...]`
renders resumably (existing frames are skipped; frames are written atomically),
then overlays, then encodes with ffmpeg.  Outputs: `out/<scene>/<quality>/`
(raw/, comp/, labels.json, hud.json) and `video/<scene>_<quality>.mp4`.

| quality | engine | resolution | notes |
|---|---|---|---|
| preview | Workbench | 640x360 | motion/framing/pacing; every 2nd frame by default |
| draft | Cycles | 640x360 | 8 spp + OIDN; materials/lighting checks |
| final | Cycles | 1280x720 | see NOTES.md for samples |
