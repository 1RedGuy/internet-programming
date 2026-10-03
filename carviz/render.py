"""Render pipeline: quality presets, resumable frame rendering, metadata,
overlay and encoding.  Driven by tools/render.py.

Directory layout (all under the repo):
    out/<scene>/<quality>/raw/00001.png     rendered frames (no overlay)
    out/<scene>/<quality>/comp/00001.png    frames with labels/HUD
    out/<scene>/<quality>/labels.json, hud.json, state.json (summary)
    video/<scene>_<quality>.mp4

Every step skips work that already exists, and frames are written to a temp
file then renamed, so a crash never leaves a corrupt frame and re-running the
same command resumes where it stopped.
"""
from __future__ import annotations

import importlib
import glob
import json
import os
import subprocess
import sys
import time

import bpy

from . import spec as S
from . import timeline

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Final-quality Cycles settings (chosen from the benchmark; see NOTES.md)
FINAL_SAMPLES = 8           # A/B on real 720p frames: 8 ~= 12 after OIDN (x-ray worst case), 16 ~= 32; budget ~36 h (NOTES.md)
FINAL_ADAPTIVE_THRESHOLD = 0.05

QUALITY = {
    "preview": dict(engine="BLENDER_WORKBENCH", res=S.RES_PREVIEW),
    "draft": dict(engine="CYCLES", res=S.RES_PREVIEW, samples=8, threshold=0.05, prefilter="FAST"),
    # ACCURATE prefilter: semi-transparent (x-ray/ghosted) shells make the albedo/normal guide passes
    # noisy at 8 spp and FAST keeps that noise as white speckles; ACCURATE removes them (A/B on s08
    # f577 at 1080p) for ~+15 % on the CPU and a small cost with OIDN on the GPU
    "final": dict(engine="CYCLES", res=S.RES_FINAL, samples=FINAL_SAMPLES, threshold=FINAL_ADAPTIVE_THRESHOLD,
                  prefilter="ACCURATE"),
    "hq": dict(engine="CYCLES", res=S.RES_FINAL, samples=64, threshold=0.02, prefilter="ACCURATE"),
}


def scene_module(scene_id):
    sys.path.insert(0, ROOT)
    for f in sorted(glob.glob(os.path.join(ROOT, "scenes", f"{scene_id}_*.py"))):
        name = os.path.splitext(os.path.basename(f))[0]
        return importlib.import_module(f"scenes.{name}")
    raise SystemExit(f"no scene module scenes/{scene_id}_*.py")


def out_dir(scene_id, quality):
    d = os.path.join(ROOT, "out", scene_id, quality)
    for sub in ("raw", "comp"):
        os.makedirs(os.path.join(d, sub), exist_ok=True)
    return d


GPU_KINDS = ("metal", "optix", "cuda", "hip", "oneapi")


def enable_gpu(kind):
    """Select a Cycles GPU backend ('metal' on Apple Silicon, 'optix'/'cuda' on NVIDIA, ...)
    and enable only devices of that type.  Exits with the device list if none is found."""
    kind = kind.upper()
    prefs = bpy.context.preferences.addons["cycles"].preferences
    try:
        prefs.compute_device_type = kind
    except TypeError:
        raise SystemExit(f"[device] {kind} is not available in this Blender build/platform")
    try:
        prefs.refresh_devices()
    except AttributeError:
        prefs.get_devices()
    found = [d for d in prefs.devices if d.type == kind]
    if not found:
        raise SystemExit(f"[device] no {kind} device; found {[(d.name, d.type) for d in prefs.devices]}")
    for d in prefs.devices:
        d.use = d.type == kind
    print(f"[device] Cycles on {kind}: {[d.name for d in found]}", flush=True)


def apply_quality(sc, quality, sb=None, threads=0, device="cpu"):
    q = QUALITY[quality]
    r = sc.render
    r.resolution_x, r.resolution_y = q["res"]
    r.resolution_percentage = 100
    r.image_settings.file_format = "PNG"
    r.image_settings.color_mode = "RGB"
    r.image_settings.color_depth = "8"
    r.image_settings.compression = 15
    r.use_persistent_data = True
    if threads:
        r.threads_mode = "FIXED"
        r.threads = threads
    r.engine = q["engine"]
    if q["engine"] == "BLENDER_WORKBENCH":
        sh = sc.display.shading
        sh.light = "STUDIO"
        sh.color_type = "MATERIAL"
        sh.show_cavity = True
        sh.cavity_type = "BOTH"
        sh.show_shadows = False   # Workbench shadows x AA samples are very slow on llvmpipe
        sh.show_specular_highlight = True
        sc.display.render_aa = "8"
        sc.view_settings.view_transform = "Standard"
        if sb is not None:
            from .rig import remove_fcurves
            for ob in sb.preview_hide:
                remove_fcurves(ob, ("hide_render", "hide_viewport"))   # keyed fades would re-show it
                ob.hide_render = True
        r.use_motion_blur = False
        return
    cy = sc.cycles
    if device and device.lower() != "cpu":
        enable_gpu(device)
        cy.device = "GPU"
        if hasattr(cy, "denoising_use_gpu"):
            cy.denoising_use_gpu = True
    else:
        cy.device = "CPU"
    cy.samples = q["samples"]
    cy.use_adaptive_sampling = True
    cy.adaptive_threshold = q["threshold"]
    cy.adaptive_min_samples = 4
    cy.use_denoising = True
    cy.denoiser = "OPENIMAGEDENOISE"
    cy.denoising_input_passes = "RGB_ALBEDO_NORMAL"
    cy.denoising_prefilter = q.get("prefilter", "FAST")
    cy.max_bounces = 6
    cy.diffuse_bounces = 2
    cy.glossy_bounces = 3
    cy.transmission_bounces = 4
    cy.volume_bounces = 0
    cy.transparent_max_bounces = 12
    cy.caustics_reflective = False
    cy.caustics_refractive = False
    cy.blur_glossy = 1.0
    cy.sample_clamp_indirect = 4.0
    cy.use_light_tree = False   # measured: with ~6 area lights + HDRI the light tree costs ~38% render time
    cy.seed = 7
    cy.use_animated_seed = False
    r.film_transparent = False
    use_mb = bool(sb is not None and sb.motion_blur)
    r.use_motion_blur = use_mb
    if use_mb:
        r.motion_blur_shutter = sb.shutter


def frame_list(scene_id, every=1, frange=None, frames=None):
    n = timeline.scene(scene_id).frames
    if frames:
        return [f for f in frames if 1 <= f <= n]
    a, b = (1, n) if frange is None else frange
    lst = list(range(max(1, a), min(n, b) + 1, max(1, every)))
    return lst


def write_meta(sb, sc, d, frames):
    """labels.json / hud.json for ALL frames of the scene (always recomputed:
    cheap compared with rendering, and stale metadata is a classic bug)."""
    lp, hp = os.path.join(d, "labels.json"), os.path.join(d, "hud.json")
    t0 = time.time()
    sb.labels.write(lp, sc, sb.camera, frames, S.FPS, sc.render.resolution_x, sc.render.resolution_y)
    sb.hud.write(hp, frames)
    tr = sb.track
    summ = dict(scene=sb.scene_id, frames=int(tr.n), violations=list(tr.violations))
    with open(os.path.join(d, "state.json"), "w") as fh:
        json.dump(summ, fh, indent=1)
    print(f"[meta] labels/hud written in {time.time() - t0:.1f}s")


def png_size(path):
    """(width, height) from a PNG header, or None if missing/unreadable."""
    try:
        with open(path, "rb") as fh:
            head = fh.read(24)
    except OSError:
        return None
    if len(head) < 24 or head[:8] != b"\x89PNG\r\n\x1a\n":
        return None
    return int.from_bytes(head[16:20], "big"), int.from_bytes(head[20:24], "big")


def render_frames(sc, d, frames, log_every=10):
    raw = os.path.join(d, "raw")
    res = (sc.render.resolution_x, sc.render.resolution_y)
    # an existing frame counts as done only at the current resolution (e.g. 720p leftovers
    # from an earlier preset are re-rendered, never mixed into a 1080p sequence)
    todo = [f for f in frames if png_size(os.path.join(raw, f"{f:05d}.png")) != res]
    stale = sum(1 for f in frames if png_size(os.path.join(raw, f"{f:05d}.png")) not in (None, res))
    if stale:
        print(f"[render] {stale} existing frames are not {res[0]}x{res[1]}: re-rendering them", flush=True)
    print(f"[render] {len(frames) - len(todo)} of {len(frames)} frames exist, rendering {len(todo)}", flush=True)
    t_start = time.time()
    for k, f in enumerate(todo):
        final = os.path.join(raw, f"{f:05d}.png")
        tmp = os.path.join(raw, f".tmp_{f:05d}.png")
        sc.frame_set(f)
        sc.render.filepath = tmp
        t0 = time.time()
        bpy.ops.render.render(write_still=True)
        os.replace(tmp, final)
        if k % log_every == 0 or k == len(todo) - 1:
            el = time.time() - t_start
            eta = el / (k + 1) * (len(todo) - k - 1)
            print(f"[render] frame {f} ({k + 1}/{len(todo)}) {time.time() - t0:.1f}s  eta {eta / 60:.1f} min",
                  flush=True)
    return len(todo)


def overlay(d, frames, workers=2, overwrite=False):
    """Composite labels/HUD.  Re-does a frame when its comp/ output is missing
    or older than its raw frame or the labels/hud JSON."""
    from . import overlay as ov
    raw, comp = os.path.join(d, "raw"), os.path.join(d, "comp")
    lp, hp = os.path.join(d, "labels.json"), os.path.join(d, "hud.json")
    meta_t = max(os.path.getmtime(lp), os.path.getmtime(hp))
    todo = []
    for f in frames:
        r = os.path.join(raw, f"{f:05d}.png")
        c = os.path.join(comp, f"{f:05d}.png")
        if not os.path.exists(r):
            continue
        if overwrite or not os.path.exists(c) or os.path.getmtime(c) < max(os.path.getmtime(r), meta_t):
            todo.append(f)
    print(f"[overlay] compositing {len(todo)} frames", flush=True)
    if todo:
        ov.compose_sequence(raw, comp, lp, hp, frames=todo, workers=workers, overwrite=True)


MAX_VIDEO_MB = 95.0   # GitHub refuses files > 100 MB: re-encode 2-pass to fit when CRF overshoots


def encode(scene_id, quality, d, frames, every=1, crf=18, suffix="", max_mb=MAX_VIDEO_MB):
    """H.264 MP4 from comp/ frames (contiguous or every-N subset).  Constant quality
    (CRF); if the file exceeds max_mb, a 2-pass encode at the matching bitrate replaces it."""
    vdir = os.path.join(ROOT, "video")
    os.makedirs(vdir, exist_ok=True)
    comp = os.path.join(d, "comp")
    missing = [f for f in frames if not os.path.exists(os.path.join(comp, f"{f:05d}.png"))]
    if missing:
        print(f"[encode] skipped: {len(missing)} composited frames missing (first {missing[0]})")
        return None
    out = os.path.join(vdir, f"{scene_id}_{quality}{suffix}.mp4")
    lst = os.path.join(d, "frames.txt")
    dur = every / S.FPS
    with open(lst, "w") as fh:
        for f in frames:
            fh.write(f"file '{os.path.join(comp, f'{f:05d}.png')}'\nduration {dur:.6f}\n")
        fh.write(f"file '{os.path.join(comp, f'{frames[-1]:05d}.png')}'\n")
    tmp = out + ".tmp.mp4"
    nfr = len(frames) * every
    base = ["ffmpeg", "-loglevel", "error", "-y", "-f", "concat", "-safe", "0", "-i", lst,
            "-vf", f"fps={S.FPS},format=yuv420p", "-frames:v", str(nfr), "-c:v", "libx264", "-preset", "slow"]
    tail = ["-profile:v", "high", "-movflags", "+faststart", tmp]
    subprocess.run(base + ["-crf", str(crf)] + tail, check=True)
    mb = os.path.getsize(tmp) / 1e6
    if max_mb and mb > max_mb:
        kbps = int(max_mb * 8000 / (nfr / S.FPS) * 0.96)
        print(f"[encode] CRF {crf} gave {mb:.0f} MB > {max_mb:.0f} MB: 2-pass at {kbps} kb/s", flush=True)
        log = os.path.join(d, "x264pass")
        subprocess.run(base + ["-b:v", f"{kbps}k", "-pass", "1", "-passlogfile", log, "-an", "-f", "mp4",
                               os.devnull], check=True)
        subprocess.run(base + ["-b:v", f"{kbps}k", "-pass", "2", "-passlogfile", log] + tail, check=True)
    os.replace(tmp, out)
    print(f"[encode] {out}")
    return out


def run(scene_id, quality="preview", every=None, frange=None, frames=None, force_meta=False, no_render=False,
        no_overlay=False, no_encode=False, save_blend=False, threads=0, workers=2, crf=None, suffix="",
        shard=None, no_meta=False, device="cpu"):
    """shard=(i, n): render only every n-th frame of the list starting at i (run n processes
    with fewer threads each to overlap per-frame serial overhead); sharded runs skip
    overlay/encode — finish with a --no-render run."""
    if every is None:
        every = 2 if quality == "preview" else 1
    t0 = time.time()
    mod = scene_module(scene_id)
    sb = mod.build(quality)
    sc = bpy.context.scene
    apply_quality(sc, quality, sb, threads=threads, device=device if not no_render else "cpu")
    d = out_dir(scene_id, quality)
    fl = frame_list(scene_id, every, frange, frames)
    if shard is not None:
        si, sn = shard
        fl = fl[si::sn]
        no_overlay = no_encode = True
    print(f"[build] {scene_id} built in {time.time() - t0:.1f}s; {len(fl)} frames; violations: {sb.track.violations}")
    if save_blend:
        bpy.ops.wm.save_as_mainfile(filepath=os.path.join(d, f"{scene_id}.blend"), compress=True)
    if not no_meta:
        write_meta(sb, sc, d, list(range(1, timeline.scene(scene_id).frames + 1)))
    if not no_render:
        render_frames(sc, d, fl)
    if not no_overlay:
        overlay(d, fl, workers=workers)
    if not no_encode and not frames and frange is None:
        encode(scene_id, quality, d, fl, every=every, crf=crf or (18 if quality in ("final", "hq") else 23),
               suffix=suffix)
    print(f"[done] {scene_id} {quality} in {(time.time() - t0) / 60:.1f} min")
    return sb
