"""Rigging helpers: the Assembly container and fast keyframe baking.

All motion in the film is baked as keyframes (one per frame, LINEAR
interpolation) computed from the drivetrain Track.  Baking uses
fcurve.keyframe_points.foreach_set which is ~100x faster than
keyframe_insert.

Conventions (see ARCHITECTURE.md section 2):
  * rotating parts spin about their LOCAL +Y axis;  bake with
    `bake_spin(obj, frames, theta)`  ->  rotation_euler[1] = -theta
  * transverse parts hang under `transverse_pivot()` (Z rotated -90 deg).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import bpy
import numpy as np

# --------------------------------------------------------------------------
# Object / collection helpers
# --------------------------------------------------------------------------


def collection(name, parent=None):
    """Get or create a collection linked under parent (default scene root)."""
    col = bpy.data.collections.get(name)
    if col is None:
        col = bpy.data.collections.new(name)
        (parent or bpy.context.scene.collection).children.link(col)
    return col


def link(obj, col=None):
    col = col or bpy.context.scene.collection
    if obj.name not in col.objects:
        col.objects.link(obj)
    return obj


def empty(name, loc=(0, 0, 0), rot=(0, 0, 0), parent=None, col=None, size=0.05):
    ob = bpy.data.objects.new(name, None)
    ob.empty_display_size = size
    ob.empty_display_type = "PLAIN_AXES"
    ob.location = loc
    ob.rotation_euler = rot
    link(ob, col)
    if parent is not None:
        ob.parent = parent
    return ob


def transverse_pivot(name, loc=(0, 0, 0), parent=None, col=None):
    """Empty whose local +Y points along world +X (for transverse spinning parts)."""
    return empty(name, loc=loc, rot=(0.0, 0.0, -math.pi / 2), parent=parent, col=col)


def parent_keep(child, parent):
    """Parent without moving the child in world space."""
    mw = child.matrix_world.copy()
    child.parent = parent
    child.matrix_parent_inverse = parent.matrix_world.inverted()
    child.matrix_world = mw


def set_presentation(obj, opacity=None, glow=None, recursive=False):
    """Static presentation properties read by every material (cv_opacity / cv_glow)."""
    objs = [obj] + (list(obj.children_recursive) if recursive else [])
    for o in objs:
        if opacity is not None:
            o["cv_opacity"] = float(opacity)
        if glow is not None:
            o["cv_glow"] = float(glow)
        for k in ("cv_opacity", "cv_glow"):
            if k not in o:
                o[k] = 1.0 if k == "cv_opacity" else 0.0


# --------------------------------------------------------------------------
# Fast baking
# --------------------------------------------------------------------------


def _fcurve(obj, data_path, index=-1):
    if obj.animation_data is None:
        obj.animation_data_create()
    act = obj.animation_data.action
    if act is None:
        act = bpy.data.actions.new(name=f"{obj.name}_act")
        obj.animation_data.action = act
    # Blender 4.4+/5.x: layered actions. Use the legacy-compatible helper.
    try:
        fc = act.fcurve_ensure_for_datablock(obj, data_path, index=index)
    except (AttributeError, TypeError):
        fc = act.fcurves.find(data_path, index=max(index, 0))
        if fc is None:
            fc = act.fcurves.new(data_path, index=max(index, 0))
    return fc


def remove_fcurves(obj, data_paths):
    """Delete the obj's fcurves for the given data paths (works with Blender 5 layered actions)."""
    ad = obj.animation_data
    if ad is None or ad.action is None:
        return 0
    act = ad.action
    n = 0
    bags = []
    try:
        for layer in act.layers:
            for strip in layer.strips:
                bags.extend(strip.channelbags)
    except AttributeError:
        bags = []
    if not bags and hasattr(act, "fcurves"):
        bags = [act]
    for bag in bags:
        for fc in list(bag.fcurves):
            if fc.data_path in data_paths:
                bag.fcurves.remove(fc)
                n += 1
    return n


def bake_channel(obj, data_path, index, frames, values, interpolation="LINEAR"):
    """Write one keyframe per frame on obj.<data_path>[index] (index -1 = scalar)."""
    frames = np.asarray(frames, dtype=np.float64)
    values = np.asarray(values, dtype=np.float64)
    assert frames.shape == values.shape, (frames.shape, values.shape)
    fc = _fcurve(obj, data_path, index)
    kp = fc.keyframe_points
    if len(kp):
        kp.clear()
    n = len(frames)
    kp.add(n)
    co = np.empty(2 * n, dtype=np.float32)
    co[0::2] = frames
    co[1::2] = values
    kp.foreach_set("co", co)
    interp_id = {"CONSTANT": 0, "LINEAR": 1, "BEZIER": 2}[interpolation]
    kp.foreach_set("interpolation", np.full(n, interp_id, dtype=np.int32))
    fc.update()
    # Set the property to the first value so un-animated evaluation is sane.
    try:
        if index >= 0:
            getattr(obj, data_path)[index] = float(values[0])
        elif data_path.startswith('["'):
            obj[data_path[2:-2]] = float(values[0])
        else:
            setattr(obj, data_path, float(values[0]))
    except Exception:
        pass
    return fc


def bake_spin(obj, frames, theta):
    """Spin about local +Y by scalar theta (positive = right-handed about -Y)."""
    return bake_channel(obj, "rotation_euler", 1, frames, -np.asarray(theta))


def bake_rot(obj, frames, index, angles):
    """Raw rotation_euler[index] keys (for non-spinning pivots: forks, pedals, rods)."""
    return bake_channel(obj, "rotation_euler", index, frames, angles)


def bake_loc(obj, frames, locs, base=None, axes=(0, 1, 2)):
    """locs: (n,3) array (absolute local location) or dict axis->array."""
    if isinstance(locs, dict):
        for ax, arr in locs.items():
            b = 0.0 if base is None else base[ax]
            bake_channel(obj, "location", ax, frames, np.asarray(arr) + b)
        return
    locs = np.asarray(locs)
    for ax in axes:
        bake_channel(obj, "location", ax, frames, locs[:, ax])


def bake_scale(obj, frames, scales, axes=(0, 1, 2)):
    scales = np.asarray(scales)
    for ax in axes:
        v = scales if scales.ndim == 1 else scales[:, ax]
        bake_channel(obj, "scale", ax, frames, v)


def bake_prop(obj, name, frames, values, interpolation="LINEAR"):
    """Animate a custom property (e.g. cv_opacity / cv_glow)."""
    if name not in obj:
        obj[name] = float(values[0])
    return bake_channel(obj, f'["{name}"]', -1, frames, values, interpolation)


def bake_visibility(obj, frames, opacity, threshold=0.02):
    """hide_render/hide_viewport keyed CONSTANT from an opacity track.

    Fully faded objects are removed from the render (saves Cycles time).
    """
    hide = (np.asarray(opacity) < threshold).astype(np.float64)
    bake_channel(obj, "hide_render", -1, frames, hide, "CONSTANT")
    bake_channel(obj, "hide_viewport", -1, frames, hide, "CONSTANT")


def bake_fade(objs, frames, opacity, glow=None, threshold=0.02):
    """Animate cv_opacity (+ visibility) and optionally cv_glow on objects."""
    for o in objs:
        if o.type not in {"MESH", "CURVE", "FONT", "SURFACE", "META"}:
            continue
        bake_prop(o, "cv_opacity", frames, opacity)
        bake_visibility(o, frames, opacity, threshold)
        if glow is not None:
            bake_prop(o, "cv_glow", frames, glow)


# --------------------------------------------------------------------------
# Assembly container
# --------------------------------------------------------------------------


@dataclass
class Assembly:
    name: str
    root: object
    parts: dict = field(default_factory=dict)
    anchors: dict = field(default_factory=dict)     # name -> (obj, (x,y,z) local)
    explode: dict = field(default_factory=dict)     # part -> (dx,dy,dz) at factor 1
    meta: dict = field(default_factory=dict)        # anything else useful
    _driver: object = None                          # callable(track, presentation)

    def drive(self, track, presentation=None):
        if self._driver is not None:
            self._driver(self, track, presentation or {})
        self.bake_explode(track, presentation or {})

    def bake_explode(self, track, presentation):
        if not self.explode:
            return
        ex = presentation.get("explode")
        frames = track.frames
        # no explode requested: bake the rest pose so keys from an earlier drive() never linger
        ex = np.zeros(len(frames)) if ex is None else np.asarray(ex)
        for pname, off in self.explode.items():
            ob = self.parts.get(pname)
            if ob is None:
                continue
            base = self.meta.setdefault("_rest_loc", {}).setdefault(pname, tuple(ob.location))
            # Explode moves the part's *location*; parts whose location is
            # already baked must be exploded via an explode Empty instead
            # (see Assembly.explode_parent()).
            for ax in range(3):
                if abs(off[ax]) > 0:
                    bake_channel(ob, "location", ax, frames, base[ax] + off[ax] * ex)

    def objects(self):
        out = [self.root]
        out += list(self.root.children_recursive)
        return out

    def meshes(self):
        return [o for o in self.objects() if o.type == "MESH"]

    def anchor_world(self, name):
        from mathutils import Vector
        ob, off = self.anchors[name]
        return ob.matrix_world @ Vector(off)


def explode_parent(name, parent, col=None):
    """An Empty used as an explode carrier: exploded parts are parented to it
    so their own (baked) local motion composes with the explode offset."""
    return empty(name, parent=parent, col=col, size=0.02)
