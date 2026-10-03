#!/usr/bin/env python3
"""Render a scene (resumable).

Examples
    python3 tools/render.py s02                      # Workbench preview, every 2nd frame
    python3 tools/render.py s02 --quality draft --every 6
    python3 tools/render.py s02 --quality final      # Cycles 1280x720 (long!)
    python3 tools/render.py s02 --quality draft --frames 1,120,480   # spot stills
    python3 tools/render.py s02 --quality final --range 1-500         # part of a scene
    python3 tools/render.py s02 --quality final --device metal        # GPU (Apple Silicon)

Re-running the same command skips frames that already exist.  Use
--force-meta after changing labels/HUD (cheap) and delete out/<scene>/<quality>/raw
after changing geometry/animation.
"""
import argparse
import os
import sys

os.environ.setdefault("EGL_PLATFORM", "surfaceless")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scene")
    ap.add_argument("--quality", default="preview", choices=["preview", "draft", "final", "hq"])
    ap.add_argument("--every", type=int, default=None)
    ap.add_argument("--range", default=None, help="a-b frame range (1-based, inclusive)")
    ap.add_argument("--frames", default=None, help="comma list of frames")
    ap.add_argument("--force-meta", action="store_true")
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--no-overlay", action="store_true")
    ap.add_argument("--no-encode", action="store_true")
    ap.add_argument("--save-blend", action="store_true")
    ap.add_argument("--threads", type=int, default=0)
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--crf", type=int, default=None)
    ap.add_argument("--suffix", default="")
    ap.add_argument("--shard", default=None, help="i/n: render every n-th frame starting at i (no overlay/encode)")
    ap.add_argument("--no-meta", action="store_true", help="do not recompute labels/hud json")
    ap.add_argument("--device", default="cpu", choices=["cpu", "metal", "optix", "cuda", "hip", "oneapi"],
                    help="Cycles device: cpu, or a GPU backend (metal = Apple Silicon)")
    a = ap.parse_args()
    from carviz import render
    fr = tuple(int(x) for x in a.range.split("-")) if a.range else None
    frames = [int(x) for x in a.frames.split(",")] if a.frames else None
    render.run(a.scene, a.quality, every=a.every, frange=fr, frames=frames, force_meta=a.force_meta,
               no_render=a.no_render, no_overlay=a.no_overlay, no_encode=a.no_encode, save_blend=a.save_blend,
               threads=a.threads, workers=a.workers, crf=a.crf, suffix=a.suffix,
               shard=tuple(int(x) for x in a.shard.split("/")) if a.shard else None, no_meta=a.no_meta,
               device=a.device)


if __name__ == "__main__":
    main()
