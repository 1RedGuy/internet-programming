# How a Manual Car Works — realistic 3D explainer

A 7½-minute, fully code-driven Blender/Cycles film that takes the viewer inside a
front-engine, rear-wheel-drive car with a five-speed manual gearbox, plus a narration
script and subtitles timed to the picture.

| Deliverable | File |
|---|---|
| Final film (H.264, 1920x1080, 24 fps, soft subtitles, ≤ 95 MB) | `video/final.mp4` |
| Full-quality master (CRF 18; too big for GitHub, produced next to it) | `video/final_hq.mp4` |
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
python3 tools/render.py s04 --quality final       # Cycles 1920x1080 final -> video/s04_final.mp4
python3 tools/render.py s04 --quality final --range 1-300   # part of a scene
python3 tools/render.py s04 --quality draft --frames 1,240,600   # spot stills
python3 tools/contact_sheet.py s04 --quality final --step 2      # review sheet
python3 tools/stitch.py                            # all scenes -> video/final.mp4
python3 tools/make_docs.py                         # script.md, narration.srt, storyboard.md from the timeline
```
### All finals (what produced `video/`)

```bash
# render queue: one scene at a time, 2 shard processes x 2 threads, nice 10, overlay + encode at the end
setsid nohup tools/render_daemon.sh > out/render_daemon.log 2>&1 < /dev/null &
echo s04 >> out/final_queue.txt        # schedule a scene; finished ones are listed in out/final_done.txt
echo STOP >> out/final_queue.txt       # let the daemon exit once the queue is empty
# or directly, without the daemon:
python3 tools/render_finals.py s01 s02 --shards 2 --threads 2 --nice 10
python3 tools/stitch.py                # video/final.mp4 (+ soft subtitles from narration.srt)
```

### Rendering the finals on a Mac (Apple Silicon GPU) — how `video/` was produced

```bash
brew install python@3.11 ffmpeg
git clone -b claude/manual-car-explainer https://github.com/1redguy/internet-programming.git
cd internet-programming
python3.11 -m venv .venv && .venv/bin/pip install -r requirements.txt
tools/mac_render_all.sh --push      # all 8 scenes at 1080p on Metal, stitch, commit + push the videos
```
The script checks the environment, keeps the Mac awake (`caffeinate`), times one heavy frame
(`tools/bench.py`: picks the ACCURATE denoiser prefilter unless it is too slow on that machine and
prints an estimate of the total hours), renders each scene as two
shard processes on the GPU (`tools/render_finals.py --device metal`), writes `video/sNN_final.mp4`,
then `video/final.mp4` (2-pass, ≤ 95 MB) and `video/final_hq.mp4` (CRF 18). It is resumable: run
it again after an interruption. Progress: `tail -f out/render_<scene>_final_shard0.log`.
One scene only: `CARVIZ_SCENES="s03" tools/mac_render_all.sh`. Any other GPU:
`python3 tools/render.py s04 --quality final --device optix|cuda|hip|oneapi`. Render each scene
completely on one device (CPU and GPU noise patterns differ slightly).

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
