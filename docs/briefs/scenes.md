# Brief: scene agents (scenes/sNN_*.py)

You build ONE scene of the film. Read: ARCHITECTURE.md, carviz/timeline.py (your beats:
durations, narration, visual intent — the narration timing is FIXED; the picture must follow
it), carviz/state.py (Program/Track; helpers engage/disengage/select_plane/start_in_gear/cut/
align_engine), carviz/kin.py, carviz/camera.py, carviz/labels.py (Labels, Hud, fmt_rpm),
carviz/overlay.py (widget types), carviz/scenebase.py, carviz/render.py, carviz/rig.py,
carviz/materials.py, carviz/lighting.py, carviz/assemblies/*.py (read their docstrings for
parts/anchors/opts/explode/meta), carviz/assemblies/car.py (full-car composition), FACTS.md
(every narrated/shown behaviour must agree with it), and your section below.

## Files you own
`scenes/<id>_<name>.py` and anything under `out/<id>/`, `video/<id>_*`. Do not edit shared
modules or assemblies; if one has a bug or lacks a feature, work around it in your scene if
reasonable and report it precisely (file, function, what and why) — the orchestrator fixes
shared code. Do not git commit.

## Scene module contract
```python
SCENE_ID = "s0N"
def build(quality: str) -> scenebase.SceneBuild
```
1. `scenebase.new_scene(SCENE_ID)`; `lighting.setup_studio(...)`; `lighting.setup_color_management`.
2. Build only the assemblies/parts you need (hide/delete what is never visible: it costs
   render time).
3. Build the drivetrain `state.Program(SCENE_ID)` from the physics plan below; `track =
   P.run()`; `track.validate(aliasing={...})` must pass with NO violations (list the visible
   fast parts with their tooth/feature pitch in the aliasing dict; motion blur scenes may relax
   aliasing only for the frames with motion blur on).
4. `asm.drive(track, presentation)` for every assembly; bake presentation (fades via
   `rig.bake_fade`, glow via `rig.bake_prop(obj, 'cv_glow', ...)`, exploded views via each
   assembly's `explode` presentation array).
5. Camera with `camera.CameraPath` — slow, smooth, motivated moves (orbits, push-ins, tracking),
   subtle depth of field (f/2.8-f/8 at these scales; focus on the subject; never so shallow
   that labelled parts are blurred). Frame for 16:9 and keep the bottom 18% free of important
   action (subtitles).
6. Labels (`Labels.add`) — minimal: part names exactly as narrated, appearing when the part is
   named (`timeline beat.word_time('clutch')`), 2-4 on screen max, positioned so they don't
   overlap each other or the subject. HUD (`Hud.add`) with the widgets listed for your scene,
   values from the Track (rpm via `labels.fmt_rpm`). Section title widget for the first ~4 s:
   `section_title(number, title)`.
7. Return `SceneBuild(scene_id, track, camera, labels, hud, motion_blur=..., preview_hide=...)`.

## Iteration loop (required)
For each pass: render a preview (`python3 tools/render.py <id>` = Workbench 640x360, every 2nd
frame, ~0.3 s/frame) and a handful of Cycles draft stills (`--quality draft --frames a,b,c`,
640x360 8 spp) at key moments of every beat. Extract frames (they are PNGs in
out/<id>/<quality>/comp/) and LOOK at them (Read tool), plus make a contact sheet (ffmpeg tile
or PIL) of ~1 frame/second to check pacing against the narration. Check: clipping/intersecting
meshes, wrong rotation directions, speeds vs ratios (read the HUD + the Track), unreadable or
overlapping labels, bad framing, things leaving frame, flicker/popping (fades, visibility),
unrealistic materials or lighting, pacing vs the script (is the named part on screen and
labelled when it is said?). Fix and re-render. At least 2 full passes; keep going until a pass
finds nothing significant. Then produce one Cycles draft of the WHOLE scene at every 4th
frame (`--quality draft --every 4`) and review that too. CPU is shared by up to 4 agents: keep
draft stills few; previews are cheap.

Final message: what you built, state program summary (key times/speeds), the Track validation
result, render timing per draft frame, the list of issues you found and fixed per pass, known
limitations, shared-module requests, and paths of 6-10 representative draft frames.

---------------------------------------------------------------------------------------------

## s01 Overview (32 s) — full car, `light` studio
* State: car parked, engine OFF (`engine_on` 0), neutral, all speeds 0 — drivetrain static.
* 0-7 `car`: opaque car, 3/4 front-left, slow orbit at eye height (~1.2-1.4 m), 50 mm.
  title_card "How a Manual Car Works" / subtitle e.g. "Inside a front-engine, rear-wheel-drive
  car" ~0.6-5.6 s.
* 7-14 `inside`: push in over the front-left wing; body exterior fades 1 -> ~0.15 (x-ray) from
  ~8 to ~12 s, interior/underbody fade further; the camera passes THROUGH the bodywork around
  11-13 s (the shell must be faint by then) and ends high above-left of the engine bay.
* 14-29 `path`: camera travels front -> rear along the drivetrain (high 3/4 from the left).
  Power-path glow + labels exactly at the spoken words: engine, clutch, gearbox, propeller
  shaft, differential, driveshafts, wheels (`beat.word_time`). Glow stays on (dimmer) once lit.
  Use each assembly's `meta['power_path']`.
* 29-32 `follow`: camera dives to a 3/4 front-left view of the engine; body/chassis fade to 0;
  fade to black over the last ~0.4 s (hud `fade` widget) for the hand-off to s02.
* HUD: title card only (no rpm: engine off).

## s02 The engine (82.5 s) — `dark` studio, engine (+ flywheel) only
* State: idling 850 rpm, neutral, clutch engaged, car stationary. Slow motion so that each
  stroke (180 deg crank) takes exactly 7.0 s during the four stroke beats:
  slowmo = pi / (7.0 * rpm_to_rad_s(850)) ~= 1/198. `P.align_engine(t_intake_start, 360)` so
  cylinder 1 is at TDC beginning its intake stroke exactly when the "intake" beat starts.
  For `firing` (12.5 s) raise the slow-motion rate smoothly so all four firings (720 deg) are
  seen in order within the beat (e.g. ~1/70), then ease back for `flywheel`.
* Cutaways: 'long' for inline4/crank/firing, 'cyl1' for cycle/intake/compression/power/exhaust,
  'front' for valvetrain (fade removed pieces; never pop).
* HUD: section_title (2, "The engine"), rpm 850 (label "Engine"), slowmo badge (factor =
  1/slowmo, rounded), stroke_strip for cylinder 1 during cycle..exhaust (active + progress from
  `kin.stroke_of`), firing_ticker during `firing` (active = cylinder currently on its power
  stroke), readouts "Crank 850 rpm / Camshafts 425 rpm" in valvetrain.
* Labels per timeline visual notes. Combustion gas/spark visibility comes from the engine
  assembly's drive (verify the colours read clearly but realistically).

## s03 The clutch (63.5 s) — `dark` studio; engine rear + flywheel, clutch, bellhousing, gearbox input shaft (+ pedal box, hydraulics, firewall hint from body)
* State: `parts`..`release`: idle 850 rpm, neutral, clutch engaged; slowmo ~1/200 (flywheel
  ring gear 132T must not strobe; check aliasing). `release`: pedal pressed (pedal 0->1 over
  ~1.5 s), then FIRST GEAR selected (select_plane -1, engage(1, ...)) — the synchroniser stops
  the disc/input shaft because the car is stationary — HUD gear N -> 1, status DISENGAGED.
* `slip`: pedal rises slowly through the bite point while throttle_rpm rises to ~1400-1600;
  road speed rises from 0 (prescribe a smooth take-off reaching ~8-12 km/h); disc speed rises
  until it equals the engine speed and the clutch LOCKS (Track `locked`, status ENGAGED) —
  around 70-85 % of the beat. Ramp slowmo from ~1/200 up (e.g. ~1/12) during the slip so the
  take-off takes a realistic ~1-1.5 s of sim time; enable motion blur for those frames if parts
  strobe; the engine must not stall (validator).
* HUD: section_title (3, "The clutch"), status badge from track.status, readouts Engine rpm /
  Disc rpm (= input shaft), pedal bar, gear indicator, slowmo badge.
* Exploded view in `parts` (assemble by ~9-10 s). `release`: show the pedal->master->line->
  slave->fork->bearing chain (pull back), glow pulse along `meta['hydraulic_segments']` while the
  pedal moves, then the fingers/plate in close-up. Note the 1.8 mm plate lift is small: frame
  it close, optionally add a label "gap ~2 mm".

## s04 The gearbox (91 s) — `dark` studio; gearbox (+ clutch/bellhousing context faded)
* State: `shafts`, `neutral`: idle 850, neutral, clutch engaged, car stationary (output shaft
  and hubs still while all gears spin). Slowmo ~1/60 (input gear 26T and 32 dog teeth must not
  strobe; verify). `synchro`: pedal pressed at the start. `lock`: engage 1st — the synchro
  stops the input side (car stationary), sleeve passes the blocker ring and slides over the
  dogs. `linkage`: lever demonstrations with the clutch in and car stationary (e.g. 1 -> N ->
  across -> 3 -> N -> across -> 5 ... ending somewhere sensible) — rails/forks/sleeves follow.
* `ratios`: hard cuts (`P.cut`) to driving shots: 1st at ~1500 rpm (~12 km/h), then 4th, then
  5th at the same engine speed (road speeds from spec.road_speed_kmh); clutch engaged; glow
  the gearbox `meta['power_path'][g]`; HUD ratio_card + readouts Input rpm / Output rpm.
  Adjust slowmo per shot to avoid strobing. `reverse`: cut, reverse engaged, reversing ~6-8 km/h
  (engine ~900-1100 rpm), idler glowing, output turning backwards (HUD gear R).
* HUD: section_title (4, "The gearbox"), gear, rpm, status, hpattern (from lever_x/lever_y),
  slowmo badge.

## s05 A gear shift, step by step (50 s) — gearbox close-up on the 1-2 synchro + 2nd gear
* State (validated example in tools/test_kinematics.py): slowmo 1/150 constant; road speed
  24.1 km/h (3000 rpm in 1st) easing down a few tenths during the shift; start_in_gear(1),
  throttle 3000 then closed (~800) at clutch-in; pedal down during `clutch_in`; disengage(1)
  during `neutral`; engage(2) so the blocking hold (synchronising) spans most of `sync` and
  the sleeve passes through/seat during `engage`; pedal up in `clutch_out` so the clutch locks
  with the engine at ~1787 rpm before the end (throttle target ~1800-1900 then).
* HUD: section_title (5, "A gear shift"), step_card 1..5 per beat (texts: "Clutch in",
  "Out of first", "Synchronise", "Engage second", "Clutch out"), readouts Engine / 2nd gear /
  Output shaft rpm (live), pedal bar, hpattern, gear, status (SYNCHRONIZING while
  `track.syncing_12`), slowmo badge "x150".
* Glow the blocker ring/cone of 2nd during the friction phase.

## s06 Final drive and differential (59 s) — gearbox tail, propshaft, rear axle (+ wheels, x-ray body for the turn)
* State: straight-line cruise (e.g. 15 km/h in 2nd), slowmo ~1/20 (check pinion 10T/ring 41T
  aliasing). `turn`: curvature eases to 1/5 m^-1 (left) while speed eases to ~10 km/h; inner
  wheel slower, outer faster; spiders spin on the pin; case = average. Optionally speed up
  slowmo in the turn so the spider rotation is clearly visible (verify aliasing).
* For the turn, show the car moving (camera rides with the car or high view) over the
  textured floor so the curve reads; HUD readouts Left wheel / Right wheel / Diff case rpm.
* HUD: section_title (6, "Final drive & differential"), ratio_card "4.10 : 1" in ringpinion,
  slowmo badge.

## s07 Driveshafts and CV joints (41.5 s) — rear-right corner
* State: straight 10 km/h, slowmo ~1/10; susp_RR oscillates ±0.06 m (smooth, ~4-6 s period in
  video). `moves`: slowmo ramps to 1 and the car pulls away (motion blur on).
* Cutaway outer joint (`'cv_cut'`) and inner tripod (`'tripod_cut'`) per beat.
* HUD: section_title (7, "Driveshafts & CV joints"), readouts Shaft rpm = Wheel rpm (they are
  identical by construction — that is the point), slowmo badge.

## s08 Recap (32 s) — `road` studio, full car, x-ray body (~0.25), real time, motion blur
* State (real time, slowmo 1): `together` (0-4): stationary, 1st selected, clutch pressed,
  idle. `first` (4-10): clutch up with a short slip, accelerate to 3000 rpm in 1st (24.1 km/h)
  by ~9.8 s. `second` (10-17): clutch in, 1st->N->2nd (real-time synchro), clutch out — rpm
  drops to ~1790 — accelerate to ~3000 rpm (40.5 km/h). `third` (17-22): shift to 3rd (across
  the gate), rpm drops to ~2010, accelerate gently. `summary` (22-32): cruise in 3rd; camera
  pulls back/up; power path glows once more; fade to black over the last 1.5 s.
* Camera tracks alongside the moving car (parent to the car root or keyed in world space).
* HUD: gear, rpm gauge, speed km/h, pedal, hpattern, status; "REAL TIME" slowmo badge.
