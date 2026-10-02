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

## 3. Final render settings  — TODO (filled after benchmarking real frames)

## 4. Time spent — TODO

## 5. Known limitations — TODO

## 6. What I would improve with more time — TODO
