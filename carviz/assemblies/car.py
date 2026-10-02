"""The whole car: every assembly under one moving root.

    from carviz.assemblies import car
    C = car.build({"engine": {"cutaways": ["none"]}, "gearbox": {"cutaway": "none"}, ...})
    C.drive(track)                       # drives every assembly + the car's world motion
    C.sub["engine"].parts[...]           # sub-assemblies by name
    C.power_path()                       # ordered [(stage_name, [objects])] engine -> wheels

Sub-assemblies: engine, clutch, gearbox, axle, wheels, body (any that fail to
import are skipped with a warning, so partial builds work during development).
The car root (Empty 'car_root') is keyed from track.car_x / car_y / car_heading
so the whole car drives through the world; parts keep their car-frame motion.
"""
from __future__ import annotations

import importlib
import math
import traceback

import numpy as np

from .. import rig

ORDER = ("engine", "clutch", "gearbox", "axle", "wheels", "body")
STAGES = (  # (stage label, assembly, fallback part-name filter)
    ("Engine", "engine"),
    ("Clutch", "clutch"),
    ("Gearbox", "gearbox"),
    ("Propeller shaft", "axle:prop"),
    ("Differential", "axle:diff"),
    ("Driveshafts", "wheels:shafts"),
    ("Rear wheels", "wheels:rear_wheels"),
)


class Car:
    def __init__(self, root, sub):
        self.root = root
        self.sub = sub
        self.name = "car"

    def drive(self, track, presentation=None, move=True):
        presentation = presentation or {}
        for k, a in self.sub.items():
            a.drive(track, presentation.get(k, {}))
        if move:
            fr = track.frames
            rig.bake_channel(self.root, "location", 0, fr, track.car_x)
            rig.bake_channel(self.root, "location", 1, fr, track.car_y)
            rig.bake_channel(self.root, "rotation_euler", 2, fr, track.car_heading)

    def objects(self):
        out = []
        for a in self.sub.values():
            out += a.objects()
        return out

    def meshes(self):
        return [o for o in self.objects() if o.type == "MESH"]

    def power_path(self):
        """[(label, [objects])] in power-flow order, from each assembly's meta.

        Assemblies publish meta['power_path'] (list of part names, or for the
        gearbox a dict gear -> list) and optionally meta['power_groups']
        (dict group -> list of part names, e.g. axle: {'prop': [...],
        'diff': [...]}, wheels: {'shafts': [...], 'rear_wheels': [...]}).
        """
        out = []
        for label, key in STAGES:
            asm_name, _, group = key.partition(":")
            a = self.sub.get(asm_name)
            if a is None:
                continue
            names = None
            if group:
                names = a.meta.get("power_groups", {}).get(group)
            if names is None:
                pp = a.meta.get("power_path")
                if isinstance(pp, dict):
                    pp = pp.get(4) or pp.get("4") or next(iter(pp.values()))
                names = pp
            objs = []
            for n in names or []:
                ob = a.parts.get(n) if isinstance(n, str) else n
                if ob is not None:
                    objs.append(ob)
                    objs += [c for c in ob.children_recursive if c.type == "MESH"]
            out.append((label, [o for o in dict.fromkeys(objs) if o.type == "MESH"]))
        return out


def build(opts=None, only=None):
    opts = opts or {}
    col = rig.collection("car")
    root = rig.empty("car_root", col=col, size=0.3)
    root.rotation_mode = "XYZ"
    sub = {}
    for name in ORDER:
        if only is not None and name not in only:
            continue
        try:
            mod = importlib.import_module(f"carviz.assemblies.{name}")
        except Exception:
            print(f"[car] skipping {name}: import failed\n{traceback.format_exc(limit=1)}")
            continue
        a = mod.build(opts.get(name))
        a.root.parent = root
        sub[name] = a
    return Car(root, sub)
