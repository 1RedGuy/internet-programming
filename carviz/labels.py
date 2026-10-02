"""Labels (3D anchors -> 2D per frame) and HUD widget streams.

Writes the two JSON files consumed by carviz.overlay:

labels.json  {"fps","width","height","frames":{"<f>":[{id,text,ax,ay,dx,dy,alpha,style,occluded}]}}
hud.json     {"fps","frames":{"<f>":[{type, alpha, ...fields}]}}

Usage in a scene:
    L = Labels()
    L.add("piston", "Piston", anchor=(asm.parts["piston_1"], (0, 0, 0.02)), t_in=1.0, t_out=7.5,
          offset=(0.08, -0.06))
    H = Hud(track)
    H.add("gear", 0, 10, value=lambda i: track.gear[i])
    H.add("rpm", 0, 10, value=lambda i: round(track.rpm_e[i]), label="Engine")
    ...
    L.write(path, scene, camera, frames);  H.write(path)

Anchors: (object, local_offset) | world point (x,y,z) | callable(frame)->world (x,y,z).
Offsets are normalised screen units (fraction of width, height); 'auto' points
the label away from the frame centre.  Labels fade in/out over `fade` seconds.
Occlusion: a ray from the camera to the anchor; if something other than the
anchor object (or its children / `ignore` objects) is hit first, the label is
flagged occluded (overlay draws it dimmer).  Set occlusion=False to skip.
"""
from __future__ import annotations

import json
import math
import os

import numpy as np


def _fade(t, t_in, t_out, fade):
    if fade <= 0:
        return 1.0 if t_in <= t <= t_out else 0.0
    a = min(1.0, max(0.0, (t - t_in) / fade))
    b = min(1.0, max(0.0, (t_out - t) / fade))
    a = a * a * (3 - 2 * a)
    b = b * b * (3 - 2 * b)
    return min(a, b)


class Labels:
    def __init__(self):
        self.items = []

    def add(self, id, text, anchor, t_in, t_out, fade=0.4, offset="auto", style="part",
            occlusion=True, ignore=()):
        self.items.append(dict(id=id, text=text, anchor=anchor, t_in=float(t_in), t_out=float(t_out),
                               fade=float(fade), offset=offset, style=style, occlusion=occlusion,
                               ignore=tuple(ignore)))
        return self

    # ------------------------------------------------------------------
    def _world(self, it, frame):
        from mathutils import Vector
        a = it["anchor"]
        if callable(a):
            return Vector(a(frame))
        if isinstance(a, (tuple, list)) and len(a) == 2 and hasattr(a[0], "matrix_world"):
            ob, off = a
            return ob.matrix_world @ Vector(off)
        return Vector(a)

    def compute(self, scene, camera, frames, fps):
        import bpy
        from bpy_extras.object_utils import world_to_camera_view

        frames = list(frames)
        out = {str(f): [] for f in frames}
        if not self.items:
            return out
        raw = {}  # id -> list of (frame, x, y, z, occluded, alpha)
        for f in frames:
            t = (f - 1) / fps
            active = [it for it in self.items if it["t_in"] - 0.01 <= t <= it["t_out"] + 0.01]
            if not active:
                continue
            scene.frame_set(int(f))
            dg = bpy.context.evaluated_depsgraph_get()
            cam_pos = camera.matrix_world.translation
            for it in active:
                al = _fade(t, it["t_in"], it["t_out"], it["fade"])
                if al <= 0.001:
                    continue
                p = self._world(it, f)
                co = world_to_camera_view(scene, camera, p)
                if co.z <= 0:
                    continue
                occ = False
                if it["occlusion"]:
                    d = p - cam_pos
                    dist = d.length
                    if dist > 1e-6:
                        hit, loc, nrm, idx, hob, mat = scene.ray_cast(dg, cam_pos, d.normalized(), distance=dist - 0.004)
                        if hit:
                            a = it["anchor"]
                            owners = set()
                            if isinstance(a, (tuple, list)) and len(a) == 2 and hasattr(a[0], "matrix_world"):
                                owners.add(a[0].name)
                                owners.update(c.name for c in a[0].children_recursive)
                            owners.update(o.name for o in it["ignore"])
                            occ = hob.name not in owners if hob is not None else True
                raw.setdefault(it["id"], []).append((f, co.x, 1.0 - co.y, co.z, occ, al))
        # resolve offsets (auto = away from frame centre, fixed per label)
        for it in self.items:
            pts = raw.get(it["id"])
            if not pts:
                continue
            if it["offset"] == "auto":
                mx = float(np.mean([p[1] for p in pts]))
                my = float(np.mean([p[2] for p in pts]))
                dx = 0.07 if mx >= 0.5 else -0.07
                dy = -0.06 if my > 0.25 else 0.05
            else:
                dx, dy = it["offset"]
            # temporal smoothing of the occlusion flag (avoid flicker): majority over +-3 frames
            occs = [p[4] for p in pts]
            sm = []
            for j in range(len(occs)):
                w = occs[max(0, j - 3): j + 4]
                sm.append(sum(w) > len(w) / 2)
            for (f, x, y, z, _o, al), o in zip(pts, sm):
                out[str(f)].append(dict(id=it["id"], text=it["text"], ax=round(x, 5), ay=round(y, 5),
                                        dx=dx, dy=dy, alpha=round(al, 4), style=it["style"], occluded=bool(o)))
        return out

    def write(self, path, scene, camera, frames, fps, width, height):
        data = dict(fps=fps, width=width, height=height, frames=self.compute(scene, camera, frames, fps))
        tmp = path + ".tmp"
        with open(tmp, "w") as fh:
            json.dump(data, fh, separators=(",", ":"))
        os.replace(tmp, path)
        return data


class Hud:
    """Per-frame HUD widget stream.  Field values may be constants or
    callables taking the track frame index i (0-based) and returning a value."""

    def __init__(self, track):
        self.track = track
        self.items = []

    def add(self, type, t_in, t_out, fade=0.4, **fields):
        self.items.append(dict(type=type, t_in=float(t_in), t_out=float(t_out), fade=float(fade), fields=fields))
        return self

    def frames_json(self, frames=None):
        tr = self.track
        frames = tr.frames if frames is None else frames
        out = {}
        for f in frames:
            i = int(f) - 1
            t = i / tr.fps
            lst = []
            for it in self.items:
                al = _fade(t, it["t_in"], it["t_out"], it["fade"])
                if al <= 0.001:
                    continue
                w = {"type": it["type"], "alpha": round(al, 4)}
                for k, v in it["fields"].items():
                    val = v(i) if callable(v) else v
                    if isinstance(val, (np.floating,)):
                        val = float(val)
                    elif isinstance(val, (np.integer,)):
                        val = int(val)
                    elif isinstance(val, np.str_):
                        val = str(val)
                    elif isinstance(val, np.ndarray):
                        val = val.tolist()
                    w[k] = val
                lst.append(w)
            out[str(int(f))] = lst
        return out

    def write(self, path, frames=None):
        data = dict(fps=self.track.fps, frames=self.frames_json(frames))
        tmp = path + ".tmp"
        with open(tmp, "w") as fh:
            json.dump(data, fh, separators=(",", ":"))
        os.replace(tmp, path)
        return data


def fmt_rpm(x):
    """Readable rpm: rounded to 10 above 1000 to avoid flicker of the last digit."""
    x = float(x)
    if abs(x) >= 1000:
        return int(round(x / 10.0) * 10)
    return int(round(x))
