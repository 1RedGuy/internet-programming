# NOTES — render settings, timing, limitations

_(Living document; sections marked TODO are completed after the final renders.)_

## 1. Environment: what worked

| Step | Result |
|---|---|
| `pip install bpy` | **Worked.** bpy **5.0.1** wheel for CPython 3.11 (374 MB) installed from PyPI; scripts run with plain `python3`. No apt/tarball needed. |
| Cycles (CPU) | Works headless out of the box (OpenImageDenoise available). |
| EEVEE headless | Fails without an OpenGL/EGL driver (`libEGL.so.1` missing). After `apt install libegl1 libegl-mesa0 libgl1-mesa-dri` it **does render** through Mesa **llvmpipe** (software GL) with `EGL_PLATFORM=surfaceless`, but slowly: 38 s for the default cube and **30–60 s per frame** at 640x360 on a simple test scene — slower than Cycles, so it is not used. |
| Workbench | Works through the same Mesa EGL path: **~0.2–0.5 s per frame at 640x360** → used for motion/framing/pacing previews. |
| Previews | Workbench (motion, framing, pacing) + low-sample Cycles "draft" stills/sequences (materials, lighting). |
| Finals | Cycles CPU, adaptive sampling, OpenImageDenoise. |

Machine: 4 vCPU Intel Xeon @ 2.8 GHz (AVX-512), 15 GB RAM, no GPU.

## 2. Benchmarks (proxy scene, before the real models existed)

Proxy = 30 bevelled metal gears + iron block + glossy floor + HDRI + area light + DOF.

| Config | Time / frame |
|---|---|
| 1280x720, 32 spp, camera *inside* a semi-transparent shell (worst case) | 66 s |
| 1280x720, 64 spp, same | 131 s |
| 1920x1080, 32 spp, same | 160 s |
| 1280x720, 16 spp, no shell | 19 s |
| 1280x720, 8 spp, no shell, fewer bounces | 13 s |
| 1280x720, 1 spp, OIDN prefilter ACCURATE (default) | 6.1 s first / 4.9 s next |
| 1280x720, 1 spp, OIDN prefilter FAST, albedo only | 2.0 s |
| 1280x720, 1 spp, no denoise, persistent data | 1.1 s |

Findings: ~0.75 s per sample at 720p for metal-heavy shots; OIDN's ACCURATE prefilter costs
~3 s per 720p frame on this CPU (FAST prefilter ≈ 1 s); persistent data cuts per-frame scene
sync from ~5 s to ~1 s. 1080p costs 2.25× 720p.

Decision: the film is 7:31.5 = **10,836 frames**. At 1080p even 16 spp would take
~100+ hours on 4 cores, so the finals are **1280x720, 24 fps** (720p is acceptable per the
brief) with adaptive sampling + OIDN (albedo+normal, FAST prefilter) and the sample count
chosen from a benchmark of real frames (section 3).

## 3. Final render settings (chosen from benchmarks of real frames)

| Setting | Value | Why |
|---|---|---|
| Engine | Cycles, CPU, 4 threads per machine (2 processes x 2 threads) | |
| Resolution / fps | **1280x720, 24 fps** (10,836 frames, 7:31.5) | 1080p = 2.25x the cost (would be ~80 h) |
| Samples | **8 max**, adaptive (threshold 0.05, min 4) | A/B on real frames: 8 ~= 12 after denoising even on the x-ray worst case; 16 ~= 32 |
| Denoiser | OpenImageDenoise, albedo+normal, prefilter FAST | ACCURATE prefilter cost +3 s/frame for no visible gain |
| Light tree | **off** | with ~6 area lights + HDRI it cost **38 %** of render time (measured: 42.8 -> 26.6 CPU-s at 4 spp) |
| Bounces | max 6, diffuse 2, glossy 3, transmission 4, transparent 12 | lowering them saved < 10 % on these scenes |
| Seed | fixed (not animated) | avoids frame-to-frame noise "boiling" on static areas |
| Motion blur | s08 recap (shutter 0.5), s07 end (keyed), s02 flywheel only (keyed shutter = 1 tooth pitch) | real-time spinning parts; anti-strobing |
| Parallelism | `--shard i/2`: two processes with 2 threads each overlap per-frame serial overhead | ~10 % faster than one 4-thread process |

Measured on the idle machine (one 4-thread process, 1280x720), heaviest scene (s08: whole car,
x-ray shell, motion blur): **4 spp 10.0 s, 8 spp 17.2 s, 12 spp 24.7 s** per frame
(~1.8 s per sample + ~2.8 s fixed: scene sync, denoise, PNG). Engine/gearbox close-ups cost
roughly 60 % of that. CPU-time profiling (load-independent) on an engine frame: fixed ~2.7
CPU-s + 3.6 CPU-s per sample + ~5 CPU-s denoise; shader (noise/bump, presentation group) and
bounce reductions each saved < 10-20 %, the light tree 38 %.

Estimated total for the finals at these settings: **~36 h** of rendering on this 4-core
machine (vs ~50 h at 12 spp and ~65 h at 16 spp). Rendering is resumable per frame
(`tools/render_finals.py`, `tools/render_daemon.sh`), which mattered: the container was
restarted once mid-project and the queue simply continued.

## 4. Time spent — TODO

## 5. Known limitations

**Physics / model** (details and justifications in FACTS.md, section "As-built model notes"):

- The car is driven *kinematically*: road speed is a keyframed input; only the engine, clutch
  and gearbox input side are integrated (Coulomb clutch, engine torque model). Engine torque
  never feeds back into the car's speed (AB-01).
- The synchroniser is a cosine speed blend between cone contact and the end of the blocking
  hold, not an integrated cone-friction torque. It does index the gear so the dogs always meet
  aligned, and the validator checks for clashes (AB-03).
- No cyclic crank-speed ripple, fixed spark advance, single-mass flywheel, three-arc flat-tappet
  cam without clearance ramps, chain at mean speed (AB-02, AB-04 to AB-07).
- Spiral-bevel final drive without hypoid offset (AB-08). Rear suspension moves the wheel
  purely vertically (links stretch up to 2.5 %). No engine rock, so the propshaft slip spline
  never slides (AB-09, AB-12).
- Touching parts are drawn with a 0.04-0.05 mm gap so collision checks stay clean (AB-11).
  The tyre mesh is 2.1 % larger than the rolling radius used for motion (AB-14).
- Dog engagement is on the short side (3 mm overlap), dogs have no back-taper, and there are no
  physical interlock pins: the interlock is enforced logically by the validator (AB-10).

**Presentation:**

- Slow motion is everywhere except s08 and the end of s07: factors from 8x to 230x, chosen so no
  toothed part strobes. The HUD always shows physical rpm/km/h plus the factor, but the *pace* of
  events (e.g. a 0.27 s racing-style shift in s05) is real-time pace scaled down. s06 freezes time
  for the exploded differential (PRS-13).
- Cutaways, ghosted shells, the warm power-path glow and the gas colours are illustrative (PRS-06,
  PRS-07). Some sections on spinning parts are world-fixed live booleans.
- The body is a generic procedural 4-door saloon (with seats, dashboard, steering wheel and
  pedals), not a particular car. Wiring, fuel system, the exhaust beyond the manifold, cooling
  and the brake hydraulics are omitted; brakes are discs and calipers only.
- Scene changes are fades through black; the camera is continuous *within* a scene only.
- Labels/HUD are a 2D overlay composited after rendering (ray-cast occlusion dims labels behind
  parts); they are not depth-sorted with motion blur.
- **No voice-over audio** is included: the narration is delivered as `script.md` and as soft
  subtitles (`narration.srt`, also muxed into `final.mp4`). The picture is timed to 140 wpm, so a
  recorded or TTS read of the script drops straight in.

**Rendering:**

- 720p at 8 samples + OpenImageDenoise: fine detail is slightly soft, and dark glossy areas can
  show faint denoiser blotches. The seed is fixed, so residual noise does not "boil" on static
  areas but can look like a slight pattern sliding over moving parts.
- Motion blur only where real-time motion would otherwise strobe (s08, the end of s07, the s02
  flywheel). Elsewhere slow motion keeps per-frame motion below 0.35 tooth pitch.
- Workbench previews show flat studio shading (no materials' roughness/transparency), so look
  decisions were checked on low-sample Cycles draft frames.

## 6. What I would improve with more time

1. **Voice-over**: record or synthesise the narration and mix it with subtle mechanical sound
   (idle, gear whine, clutch engagement) driven by the same Track (rpm -> pitch).
2. **Render quality**: 1080p at 16-32 spp on a GPU or a small render farm. The renderer is already
   sharded (`--shard i/n`), so distributing frames over several machines needs only shared storage.
   Reuse frames when nothing in view moves (paused holds) to save render time.
3. **Physics depth**: integrate synchroniser cone torque, a lumped driveline with propshaft and
   halfshaft compliance (shunt/shuffle at take-off), tyre slip, and engine-torque feedback to road
   speed; add the crank-speed ripple and a dual-mass flywheel variant.
4. **Model detail**: hypoid offset with a matching pinion; tapered roller bearings shown in
   section; real interlock pins and a reverse lockout in the shift tower; detents on the rails;
   helical gear contact patterns; a proper exhaust and intake system.
5. **Presentation**: depth-aware labels; a continuous camera between scenes instead of fades; a
   picture-in-picture pedal/lever view during the shift; a 3D tachometer in the cabin.
6. **Testing**: automated collision checks over every frame of every scene (today they run on
   sampled frames and in the assembly tests) and a CI job that re-runs `test_kinematics.py`.
