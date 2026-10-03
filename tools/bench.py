#!/usr/bin/env python3
"""Time final-quality frames (isolated: writes only to out/bench/) and compare denoiser prefilters.

    python3 tools/bench.py --device metal                 # s08 f577 (x-ray car, the heaviest kind)
    python3 tools/bench.py s04 1200 --device cpu

Each prefilter is rendered twice; the second time is reported (the first includes GPU kernel
loading).  Prints one machine-readable line at the end:
    BENCH fast=<s> accurate=<s> choose=<ACCURATE|FAST> est_hours=<h>
choose = FAST only if ACCURATE costs more than +50 % and more than 1 s per frame (e.g. when
the denoiser cannot run on the GPU), so an unattended render stays within its time budget.
"""
import argparse
import os
import sys
import time

os.environ.setdefault("EGL_PLATFORM", "surfaceless")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scene", nargs="?", default="s08")
    ap.add_argument("frame", nargs="?", type=int, default=577)
    ap.add_argument("--device", default="cpu")
    a = ap.parse_args()
    import bpy
    from carviz import render, timeline
    t0 = time.time()
    sb = render.scene_module(a.scene).build("final")
    sc = bpy.context.scene
    render.apply_quality(sc, "final", sb, device=a.device)
    print(f"[bench] {a.scene} built in {time.time() - t0:.1f}s; {sc.render.resolution_x}x{sc.render.resolution_y}, "
          f"{sc.cycles.samples} spp, device {a.device}", flush=True)
    out = os.path.join(ROOT, "out", "bench")
    os.makedirs(out, exist_ok=True)
    sc.frame_set(a.frame)
    times = {}
    for pre in ("FAST", "ACCURATE"):
        sc.cycles.denoising_prefilter = pre
        for k in range(2):
            sc.render.filepath = os.path.join(out, f"{a.scene}_{a.frame}_{pre.lower()}_{k}.png")
            t = time.time()
            bpy.ops.render.render(write_still=True)
            times[pre] = time.time() - t
            print(f"[bench] prefilter {pre} run {k + 1}: {times[pre]:.2f}s", flush=True)
    fast, acc = times["FAST"], times["ACCURATE"]
    choose = "FAST" if (acc > 1.5 * fast and acc - fast > 1.0) else "ACCURATE"
    n = sum(s.frames for s in timeline.SCENES)
    # this frame is one of the heaviest kinds; average frames cost ~0.7x; 2 shard processes ~0.85x
    est = (acc if choose == "ACCURATE" else fast) * 0.7 * 0.85 * n / 3600
    print(f"BENCH fast={fast:.2f} accurate={acc:.2f} choose={choose} est_hours={est:.1f}", flush=True)


if __name__ == "__main__":
    main()
