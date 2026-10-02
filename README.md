# How a Manual Car Works — realistic 3D explainer

A 7½-minute, fully code-driven Blender/Cycles film that takes the viewer inside a
front-engine, rear-wheel-drive car with a five-speed manual gearbox, plus a narration
script and subtitles timed to the picture.

| Deliverable | File |
|---|---|
| Final film (H.264, 1280x720, 24 fps, soft subtitles) | `video/final.mp4` |
| Individual scenes | `video/s01_final.mp4` … `video/s08_final.mp4` |
| Narration script (timestamped, 140 wpm) | `script.md` |
| Subtitles (same timing) | `narration.srt` |
| Every mechanical behaviour shown/narrated, with justification | `FACTS.md` |
| Storyboard | `storyboard.md` |
| Render settings, timings, limitations, future work | `NOTES.md` |
| Code conventions / module contract | `ARCHITECTURE.md` |

## Setup (Ubuntu, CPU only)

```bash
tools/setup.sh            # apt: Mesa EGL + ffmpeg; pip: bpy==5.0.1 (Python 3.11), numpy, pillow, shapely, scipy
python3 tools/test_kinematics.py   # numeric proof that gears mesh, ratios/directions/timing are right
python3 tools/test_gears.py        # gear-tooth interference checks (+ test renders in out/test_gears)
```
Blender is used as a Python module (`import bpy`) — no Blender binary is needed.
Workbench previews need `EGL_PLATFORM=surfaceless` (tools/render.py sets it).

## Re-render any scene

```bash
python3 tools/render.py s04                       # Workbench preview (640x360, every 2nd frame)
python3 tools/render.py s04 --quality draft --every 4    # Cycles 640x360, 8 spp
python3 tools/render.py s04 --quality final       # Cycles 1280x720 final  -> video/s04_final.mp4
python3 tools/render.py s04 --quality final --range 1-300   # part of a scene
python3 tools/render.py s04 --quality draft --frames 1,240,600   # spot stills
python3 tools/contact_sheet.py s04 --quality final --step 2      # review sheet
python3 tools/stitch.py                            # all scenes -> video/final.mp4
python3 tools/make_docs.py                         # script.md, narration.srt, storyboard.md from the timeline
```
Rendering is **resumable**: frames are written atomically to `out/<scene>/<quality>/raw/`
and existing frames are skipped, so re-running a command after a crash continues where it
stopped. Labels/HUD are composited afterwards (`comp/`) from `labels.json`/`hud.json`, which
are recomputed on every run; delete `raw/` after changing geometry or motion.

## How it works

* `carviz/spec.py` — every dimension, tooth count and ratio (2.0 L DOHC inline-4,
  86x86 mm, firing order 1-3-4-2; 5-speed 3-shaft gearbox 3.484/2.075/1.391/1.000/0.815,
  R 3.410, all forward pairs on one 75.72 mm centre distance; final drive 41:10 = 4.10).
* `carviz/timeline.py` — the narration, beat by beat, with durations: the single source of
  timing for the scenes, the script and the subtitles.
* `carviz/state.py` — the master **drivetrain state**. Each scene keys *inputs* (road speed,
  turn curvature, clutch pedal, throttle target, synchro sleeve positions, lever plane,
  slow-motion factor). The integrator derives everything else: wheel/diff/propshaft/output
  speeds, gear engagement, synchroniser speed matching and dog-tooth indexing, Coulomb
  clutch slip and lock-up against the engine, engine speed, car pose. `Track.validate()`
  rejects physically wrong programs (shifting under load, clash, interlock, stall,
  wagon-wheel strobing).
* `carviz/kin.py` — slider-crank, valve lift and cam phasing, chain travel, clutch
  hydraulics, gear mesh phases for every gear, Hooke joints, differential, Ackermann.
* `carviz/assemblies/` — procedural models (engine, clutch, gearbox, axle, wheels, body,
  car) whose every moving part is baked from the Track; `carviz/gears.py` builds involute
  spur/helical/bevel gears that are proven not to interfere (`tools/test_gears.py`).
* `carviz/materials.py`, `carviz/lighting.py` — PBR materials and studio lighting.
* `carviz/camera.py`, `carviz/labels.py`, `carviz/overlay.py` — camera moves, label anchors
  and HUD streams, Pillow compositor.
* `scenes/s01_*.py … s08_*.py` — one module per scene.
* `carviz/render.py`, `tools/render.py` — resumable render → overlay → ffmpeg pipeline.
