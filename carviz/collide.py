"""Interference checks between objects at sampled frames (BVH triangle overlap).

    from carviz import collide
    bad = collide.check_pairs([(piston1, head), (gear_a, gear_b)], frames=range(1, 200, 5))
    for f, a, b, n in bad: print(f, a, b, n)

Pairs that should merely *touch* (e.g. a bearing on its shaft) must not be
listed: coincident surfaces register as overlaps.  Meshing gears with
backlash should never overlap.
"""
from __future__ import annotations

import bpy
from mathutils.bvhtree import BVHTree


def _bvh(ob, dg):
    ev = ob.evaluated_get(dg)
    me = ev.to_mesh()
    mw = ev.matrix_world
    me.calc_loop_triangles()
    verts = [mw @ v.co for v in me.vertices]
    # loop triangles (Blender's own triangulation) avoid false overlaps from
    # concave n-gons left by booleans
    tris = [tuple(t.vertices) for t in me.loop_triangles]
    ev.to_mesh_clear()
    return BVHTree.FromPolygons(verts, tris, epsilon=0.0)


def overlap_count(a, b, dg=None):
    dg = dg or bpy.context.evaluated_depsgraph_get()
    return len(_bvh(a, dg).overlap(_bvh(b, dg)))


def check_pairs(pairs, frames, scene=None, verbose=False):
    """Return [(frame, nameA, nameB, n_overlapping_face_pairs)] for every overlap."""
    scene = scene or bpy.context.scene
    out = []
    for f in frames:
        scene.frame_set(int(f))
        dg = bpy.context.evaluated_depsgraph_get()
        cache = {}
        for a, b in pairs:
            ta = cache.get(a.name) or cache.setdefault(a.name, _bvh(a, dg))
            tb = cache.get(b.name) or cache.setdefault(b.name, _bvh(b, dg))
            n = len(ta.overlap(tb))
            if n:
                out.append((int(f), a.name, b.name, n))
                if verbose:
                    print(f"  overlap f{f}: {a.name} x {b.name}: {n} face pairs")
    return out


def min_distance(a, b, samples=400, dg=None):
    """Approximate minimum distance from a's vertices to b's surface (m)."""
    dg = dg or bpy.context.evaluated_depsgraph_get()
    tb = _bvh(b, dg)
    ev = a.evaluated_get(dg)
    me = ev.to_mesh()
    mw = ev.matrix_world
    n = len(me.vertices)
    step = max(1, n // samples)
    best = 1e9
    for i in range(0, n, step):
        hit = tb.find_nearest(mw @ me.vertices[i].co)
        if hit[0] is not None:
            best = min(best, hit[3])
    ev.to_mesh_clear()
    return best
