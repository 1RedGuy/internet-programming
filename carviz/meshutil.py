"""Fast mesh construction helpers (numpy + bmesh, no bpy.ops).

Conventions: round parts are revolved about their LOCAL +Y axis (spin axis,
ARCHITECTURE.md section 2); 2D profiles for revolve are (r, y); polygons for
extrusion are (x, z) in the local XZ plane, extruded along Y.

Public API
----------
MeshBuilder                      numpy vertex/face accumulator -> object (fast foreach_set)
lathe(name, profile_ry, segments, axis='Y', caps, closed, ...)  revolve a (r, y) profile about Y
cylinder(name, radius, y0, y1, segments, inner_radius, chamfer)  (optionally hollow) cylinder
extrude_polygon(name, poly_xz, y0, y1, holes=..., chamfer=...)  prism along Y
tube_along(name, points, radius, segments, bend_radius, ...)    swept tube, rounded bends
rounded_box(name, size, radius, segments, center)              box with rounded edges
cut_and_apply(obj, cutter=None, plane=None, wedge=None, space=..., section_material_name='section_cut')
cut_half(obj, normal, point) / cut_quarter(obj, n1, n2, point)  cutaway wrappers
smooth_by_angle(obj, angle_deg, keep_sharp)   smooth faces + sharp edges by angle (5.0 API)
merge_by_distance(obj, dist), recalc_normals(obj, inside), split_twisted_quads(obj)
get_material(mat), assign_material(obj, material, faces=None) -> slot index
join(name, objects)                           merge mesh objects (world transforms baked)
offset_polygon(xz, dist), triangulate_loops(rings), signed_volume(mesh), mesh_stats(obj)

Every builder links the new object to ``collection`` (default: the scene
collection; pass link=False to skip) and returns it.
"""
from __future__ import annotations

import math

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector
from mathutils import geometry as mgeo

TAU = 2.0 * math.pi

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------


def get_material(mat):
    """Resolve a material: bpy Material, or a name via carviz.materials.get
    (if that module exists and knows it), else a simple placeholder
    Principled material with that name."""
    if mat is None:
        return None
    if isinstance(mat, bpy.types.Material):
        return mat
    name = str(mat)
    try:
        from . import materials as _mats  # noqa: WPS433 (written in parallel)
        getter = getattr(_mats, "get", None)
        if getter is not None:
            m = getter(name)
            if m is not None:
                return m
    except Exception:
        pass
    m = bpy.data.materials.get(name)
    if m is None:
        m = bpy.data.materials.new(name)
        grey = {"section_cut": (0.75, 0.12, 0.08, 1.0)}.get(name, (0.55, 0.55, 0.57, 1.0))
        m.diffuse_color = grey
        try:
            m.use_nodes = True
            bsdf = m.node_tree.nodes.get("Principled BSDF")
            if bsdf is not None:
                bsdf.inputs["Base Color"].default_value = grey
                bsdf.inputs["Metallic"].default_value = 0.0 if name == "section_cut" else 0.9
                bsdf.inputs["Roughness"].default_value = 0.35
        except Exception:
            pass
    return m


def assign_material(obj, material, faces=None):
    """Append material (object/name) to obj's slots if missing and assign it
    to `faces` (iterable of polygon indices, or a bool mask), or to all faces
    if faces is None.  Returns the slot index."""
    mat = get_material(material)
    me = obj.data
    idx = None
    for i, m in enumerate(me.materials):
        if m == mat:
            idx = i
            break
    if idx is None:
        me.materials.append(mat)
        idx = len(me.materials) - 1
    n = len(me.polygons)
    if n == 0:
        return idx
    mi = np.zeros(n, dtype=np.int32)
    me.polygons.foreach_get("material_index", mi)
    if faces is None:
        mi[:] = idx
    else:
        f = np.asarray(faces)
        if f.dtype == bool:
            mi[f] = idx
        elif len(f):
            mi[f.astype(np.int64)] = idx
    me.polygons.foreach_set("material_index", mi)
    me.update()
    return idx


# ---------------------------------------------------------------------------
# Triangulation of planar regions bounded by loops (outer boundaries + holes)
# ---------------------------------------------------------------------------


def triangulate_loops(rings):
    """Triangulate the even-odd region bounded by closed 2D rings.

    rings: list of (k,2) arrays (any orientation, nesting allowed).
    Returns (t,3) int array of indices into np.concatenate(rings).  Uses
    shapely's constrained Delaunay triangulation (keeps every input vertex,
    robust to collinear points); falls back to mathutils tessellate_polygon.
    """
    rings = [np.asarray(r, dtype=np.float64) for r in rings]
    flat = np.concatenate(rings)
    try:
        import shapely
        from shapely.geometry import Polygon
        polys = [Polygon(r) for r in rings]
        if not all(p.is_valid for p in polys):
            raise ValueError("invalid ring")
        reps = [p.representative_point() for p in polys]
        n = len(rings)
        contains = [[j for j in range(n) if j != i and polys[j].contains(reps[i])] for i in range(n)]
        depth = [len(c) for c in contains]
        holes_of = {i: [] for i in range(n) if depth[i] % 2 == 0}
        for i in range(n):
            if depth[i] % 2 == 1:
                parent = min(contains[i], key=lambda j: polys[j].area)
                holes_of.setdefault(parent, []).append(i)
        index = {}
        for k, (x, y) in enumerate(flat):
            index.setdefault((float(x), float(y)), k)
        tris = []
        for o, hs in holes_of.items():
            poly = Polygon(rings[o], [rings[h] for h in hs])
            if not poly.is_valid:
                raise ValueError("invalid polygon")
            tri = shapely.constrained_delaunay_triangles(poly)
            for g in getattr(tri, "geoms", []):
                c = list(g.exterior.coords)[:3]
                tris.append([index[(float(c[0][0]), float(c[0][1]))], index[(float(c[1][0]), float(c[1][1]))],
                             index[(float(c[2][0]), float(c[2][1]))]])
        exp = sum(len(r) for r in rings) + 2 * sum(len(h) for h in holes_of.values()) - 2 * len(holes_of)
        if len(tris) != exp:
            raise ValueError(f"CDT returned {len(tris)} triangles, expected {exp}")
        return np.asarray(tris, dtype=np.int64).reshape(-1, 3)
    except Exception:
        vecs = [[Vector((p[0], p[1], 0.0)) for p in r] for r in rings]
        t = mgeo.tessellate_polygon(vecs)
        return np.asarray(t, dtype=np.int64).reshape(-1, 3)


# ---------------------------------------------------------------------------
# MeshBuilder
# ---------------------------------------------------------------------------


class MeshBuilder:
    """Accumulates vertices and faces (numpy), then creates a bpy mesh fast.

    Faces may be added in any orientation; ``to_object(..., fix_normals=True)``
    makes a closed mesh's normals consistent and outward.
    """

    def __init__(self):
        self._v = []
        self.nv = 0
        self._faces = {}        # size -> list of (k,size) int arrays
        self._fmat = {}         # size -> list of (k,) int arrays
        self._ngons = []        # list of (tuple, mat)
        self.sharp_edges = []   # list of (a, b) vertex pairs to force sharp

    # -- vertices ---------------------------------------------------------
    def verts(self, pts):
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        idx = np.arange(self.nv, self.nv + len(pts))
        self._v.append(pts)
        self.nv += len(pts)
        return idx

    def ring_xz(self, xz, y):
        """Add a ring of points given in the local XZ plane at height y."""
        xz = np.asarray(xz, dtype=np.float64)
        y = np.broadcast_to(np.asarray(y, dtype=np.float64), (len(xz),))
        return self.verts(np.stack([xz[:, 0], y, xz[:, 1]], axis=1))

    def circle(self, r, y, n, phase=0.0):
        a = phase + np.arange(n) * TAU / n
        return self.ring_xz(np.stack([r * np.cos(a), r * np.sin(a)], axis=1), y)

    def coords(self):
        return np.concatenate(self._v, axis=0) if self._v else np.zeros((0, 3))

    # -- faces --------------------------------------------------------------
    def faces(self, arr, mat=0):
        arr = np.asarray(arr, dtype=np.int64)
        if arr.ndim == 1:
            arr = arr[None, :]
        k = arr.shape[1]
        self._faces.setdefault(k, []).append(arr)
        self._fmat.setdefault(k, []).append(np.full(len(arr), mat, dtype=np.int32))

    def face(self, idx, mat=0):
        idx = [int(i) for i in idx]
        if len(idx) in (3, 4):
            self.faces(np.array([idx]), mat)
        else:
            self._ngons.append((tuple(idx), mat))

    def bridge(self, a, b, closed=True, mat=0):
        """Quads between two equal-length index rings a and b."""
        a = np.asarray(a)
        b = np.asarray(b)
        if closed:
            a1, b1 = np.roll(a, -1), np.roll(b, -1)
            q = np.stack([a, a1, b1, b], axis=1)
        else:
            q = np.stack([a[:-1], a[1:], b[1:], b[:-1]], axis=1)
        self.faces(q, mat)

    def fan(self, centre, ring, closed=True, mat=0):
        ring = np.asarray(ring)
        r1 = np.roll(ring, -1) if closed else ring[1:]
        r0 = ring if closed else ring[:-1]
        self.faces(np.stack([np.full(len(r0), centre), r0, r1], axis=1), mat)

    def cap(self, loops, axis=1, mat=0, ngon_if_single=True):
        """Fill planar region bounded by index loops (outer + holes, any
        orientation) projected along `axis` (0=X, 1=Y, 2=Z)."""
        V = self.coords()
        if len(loops) == 1 and ngon_if_single:
            self.face(loops[0], mat)
            return
        ax = [i for i in range(3) if i != axis]
        rings = [V[np.asarray(lp)][:, ax] for lp in loops]
        flat = np.concatenate([np.asarray(lp) for lp in loops])
        tris = triangulate_loops(rings)
        if len(tris) == 0:
            return
        self.faces(flat[tris], mat)

    # -- build ----------------------------------------------------------------
    def _all_faces(self):
        sizes, loops, mats = [], [], []
        for k, lst in self._faces.items():
            arr = np.concatenate(lst, axis=0)
            sizes.append(np.full(len(arr), k, dtype=np.int64))
            loops.append(arr.ravel())
            mats.append(np.concatenate(self._fmat[k]))
        for f, m in self._ngons:
            sizes.append(np.array([len(f)]))
            loops.append(np.array(f, dtype=np.int64))
            mats.append(np.array([m], dtype=np.int32))
        if not sizes:
            return np.zeros(0, np.int64), np.zeros(0, np.int64), np.zeros(0, np.int32)
        return np.concatenate(sizes), np.concatenate(loops), np.concatenate(mats)

    def to_mesh(self, name):
        V = self.coords()
        sizes, loops, mats = self._all_faces()
        me = bpy.data.meshes.new(name)
        me.vertices.add(len(V))
        me.vertices.foreach_set("co", V.astype(np.float32).ravel())
        me.loops.add(len(loops))
        me.loops.foreach_set("vertex_index", loops.astype(np.int32))
        me.polygons.add(len(sizes))
        starts = np.concatenate([[0], np.cumsum(sizes)[:-1]]).astype(np.int32)
        me.polygons.foreach_set("loop_start", starts)
        me.polygons.foreach_set("loop_total", sizes.astype(np.int32))
        if len(mats):
            me.polygons.foreach_set("material_index", mats)
        me.update(calc_edges=True)
        return me

    def to_object(self, name, collection=None, smooth_angle=35.0, fix_normals=True, materials=None,
                  link=True, inward_quads=False):
        me = self.to_mesh(name)
        if fix_normals:
            _fix_normals_mesh(me)
        if inward_quads:
            split_twisted_quads(me)
        obj = bpy.data.objects.new(name, me)
        if materials:
            for m in materials:
                me.materials.append(get_material(m))
        if smooth_angle is not None:
            smooth_by_angle(obj, smooth_angle)
        if self.sharp_edges:
            mark_sharp_pairs(obj, self.sharp_edges)
        if link:
            (collection or bpy.context.scene.collection).objects.link(obj)
        return obj


def _fix_normals_mesh(me):
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    bm.to_mesh(me)
    bm.free()
    # recalc can choose inward for a single closed shell in degenerate cases:
    # make total signed volume positive.
    if signed_volume(me) < 0:
        me.flip_normals()


def split_twisted_quads(obj_or_mesh, tol=None):
    """Triangulate non-planar quads along their INWARD diagonal.

    A twisted quad (e.g. on a helical or spiral-bevel tooth flank) can be
    split along either diagonal; one choice bulges outward by up to half the
    quad's non-planarity.  Splitting along the inward diagonal keeps the
    surface inside the true (ruled) surface, so meshing parts never
    interpenetrate because of tessellation.  Normals must already be
    consistent and outward.  Returns the number of quads split."""
    me = obj_or_mesh.data if hasattr(obj_or_mesh, "data") else obj_or_mesh
    n = len(me.polygons)
    if n == 0:
        return 0
    lt = np.zeros(n, dtype=np.int32)
    ls = np.zeros(n, dtype=np.int32)
    me.polygons.foreach_get("loop_total", lt)
    me.polygons.foreach_get("loop_start", ls)
    lv = np.zeros(len(me.loops), dtype=np.int32)
    me.loops.foreach_get("vertex_index", lv)
    co = np.zeros(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    q = np.nonzero(lt == 4)[0]
    if len(q) == 0:
        return 0
    vi = lv[ls[q][:, None] + np.arange(4)[None, :]]
    v0, v1, v2, v3 = (co[vi[:, k]] for k in range(4))
    nrm = np.cross(v2 - v0, v3 - v1)
    nl = np.linalg.norm(nrm, axis=1)
    nl[nl == 0] = 1.0
    nrm /= nl[:, None]
    d = np.einsum("ij,ij->i", 0.5 * (v0 + v2) - 0.5 * (v1 + v3), nrm)
    if tol is None:
        tol = 1e-7 * float(np.max(np.ptp(co, axis=0)) + 1e-12)
    use_13 = q[d > tol]       # diagonal v0-v2 is outward -> split along v1-v3
    use_02 = q[d < -tol]      # diagonal v1-v3 is outward -> split along v0-v2
    if len(use_13) + len(use_02) == 0:
        return 0
    bm = bmesh.new()
    bm.from_mesh(me)
    bm.faces.ensure_lookup_table()
    f02 = [bm.faces[int(i)] for i in use_02]
    f13 = [bm.faces[int(i)] for i in use_13]
    if f02:
        bmesh.ops.triangulate(bm, faces=f02, quad_method="FIXED")
    if f13:
        bmesh.ops.triangulate(bm, faces=f13, quad_method="ALTERNATE")
    bm.to_mesh(me)
    bm.free()
    me.update()
    return len(use_13) + len(use_02)


def signed_volume(me):
    """Signed volume of a mesh (positive when normals point outward)."""
    n = len(me.loop_triangles)
    if n == 0:
        me.calc_loop_triangles()
        n = len(me.loop_triangles)
    if n == 0:
        return 0.0
    co = np.zeros(len(me.vertices) * 3, dtype=np.float64)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    tri = np.zeros(n * 3, dtype=np.int64)
    me.loop_triangles.foreach_get("vertices", tri)
    tri = tri.reshape(-1, 3)
    a, b, c = co[tri[:, 0]], co[tri[:, 1]], co[tri[:, 2]]
    return float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)


def mark_sharp_pairs(obj, pairs):
    me = obj.data
    ev = np.zeros(len(me.edges) * 2, dtype=np.int64)
    me.edges.foreach_get("vertices", ev)
    ev = ev.reshape(-1, 2)
    key = {tuple(sorted(e)): i for i, e in enumerate(ev.tolist())}
    attr = me.attributes.get("sharp_edge") or me.attributes.new("sharp_edge", "BOOLEAN", "EDGE")
    vals = np.zeros(len(me.edges), dtype=bool)
    attr.data.foreach_get("value", vals)
    for a, b in pairs:
        i = key.get((min(a, b), max(a, b)))
        if i is not None:
            vals[i] = True
    attr.data.foreach_set("value", vals)


# ---------------------------------------------------------------------------
# Shading / cleanup
# ---------------------------------------------------------------------------


def smooth_by_angle(obj, angle_deg=35.0, keep_sharp=False):
    """Smooth-shade all faces and mark edges sharper than angle_deg as sharp
    (Blender 4.1+/5.x replacement for auto-smooth; data-level, no modifier).
    keep_sharp=True keeps edges that were already marked sharp."""
    me = obj.data if hasattr(obj, "data") else obj
    old = None
    if keep_sharp and me.attributes.get("sharp_edge") is not None:
        old = np.zeros(len(me.edges), dtype=bool)
        me.attributes["sharp_edge"].data.foreach_get("value", old)
    me.shade_smooth()
    me.set_sharp_from_angle(angle=math.radians(angle_deg))
    if old is not None:
        attr = me.attributes.get("sharp_edge") or me.attributes.new("sharp_edge", "BOOLEAN", "EDGE")
        new = np.zeros(len(me.edges), dtype=bool)
        attr.data.foreach_get("value", new)
        attr.data.foreach_set("value", new | old)
    return obj


def merge_by_distance(obj, dist=1e-6):
    me = obj.data
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts[:], dist=dist)
    bm.to_mesh(me)
    bm.free()
    me.update()
    return obj


def recalc_normals(obj, inside=False):
    me = obj.data
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    if inside:
        bmesh.ops.reverse_faces(bm, faces=bm.faces[:])
    bm.to_mesh(me)
    bm.free()
    me.update()
    return obj


def mesh_stats(obj):
    """dict(verts, faces, tris, non_manifold_edges, volume) for checks."""
    me = obj.data
    me.calc_loop_triangles()
    bm = bmesh.new()
    bm.from_mesh(me)
    nm = sum(1 for e in bm.edges if not e.is_manifold)
    bm.free()
    return dict(verts=len(me.vertices), faces=len(me.polygons), tris=len(me.loop_triangles),
                non_manifold_edges=nm, volume=signed_volume(me))


def _new_object(me, name, collection, link=True):
    obj = bpy.data.objects.new(name, me)
    if link:
        (collection or bpy.context.scene.collection).objects.link(obj)
    return obj


# ---------------------------------------------------------------------------
# Primitives
# ---------------------------------------------------------------------------


def _revolve_into(mb, profile, segments, closed=False, phase=0.0, axis_eps=1e-9, mat=0):
    """Revolve (r, y) profile into MeshBuilder mb about Y; returns ring index
    arrays (or a 0-d array for a pole vertex).  Faces bridge consecutive rings."""
    prof = np.asarray(profile, dtype=np.float64)
    a = phase + np.arange(segments) * TAU / segments
    ca, sa = np.cos(a), np.sin(a)
    rings = []
    for r, y in prof:
        if r <= axis_eps:
            rings.append(mb.verts([[0.0, y, 0.0]])[0])
        else:
            rings.append(mb.verts(np.stack([r * ca, np.full(segments, y), r * sa], axis=1)))
    pairs = list(zip(range(len(rings) - 1), range(1, len(rings))))
    if closed:
        pairs.append((len(rings) - 1, 0))
    for i, j in pairs:
        ri, rj = rings[i], rings[j]
        si, sj = np.ndim(ri) == 0, np.ndim(rj) == 0
        if si and sj:
            continue
        if si:
            mb.fan(int(ri), rj, mat=mat)
        elif sj:
            mb.fan(int(rj), ri, mat=mat)
        else:
            mb.bridge(ri, rj, mat=mat)
    return rings


def lathe(name, profile_ry, segments=64, axis="Y", caps=True, closed=False, collection=None,
          material=None, smooth_angle=35.0, phase=0.0, link=True):
    """Revolve a 2D (r, y) profile around local Y -> mesh object.

    profile points with r == 0 become a single pole vertex.  If the profile
    is open and an end point has r > 0, ``caps`` closes that end with a flat
    disc (n-gon).  closed=True joins the last point back to the first
    (e.g. a ring section).  Normals are made outward for closed results.
    axis='X'|'Z' re-orients the result (Y is the convention; prefer Y).
    """
    mb = MeshBuilder()
    rings = _revolve_into(mb, profile_ry, segments, closed=closed, phase=phase)
    if caps and not closed:
        for rg in (rings[0], rings[-1]):
            if np.ndim(rg) != 0:
                mb.face(list(rg))
    obj = mb.to_object(name, collection, smooth_angle=smooth_angle, fix_normals=True, link=link)
    if axis != "Y":
        rot = {"X": Matrix.Rotation(-math.pi / 2, 4, "Z"), "Z": Matrix.Rotation(math.pi / 2, 4, "X")}[axis]
        obj.data.transform(rot)
    if material is not None:
        assign_material(obj, material)
    return obj


def cylinder(name, radius, y0, y1, segments=48, inner_radius=0.0, chamfer=0.0, collection=None,
             material=None, smooth_angle=35.0, link=True):
    """Cylinder along Y from y0 to y1 (optional bore and outer-edge chamfer)."""
    c = min(chamfer, 0.45 * (y1 - y0), 0.45 * (radius - inner_radius)) if chamfer else 0.0
    ri = inner_radius if inner_radius > 0 else 0.0
    prof = [(ri, y0)]
    if c:
        prof += [(radius - c, y0), (radius, y0 + c), (radius, y1 - c), (radius - c, y1)]
    else:
        prof += [(radius, y0), (radius, y1)]
    prof += [(ri, y1)]
    return lathe(name, prof, segments, closed=ri > 0, caps=False, collection=collection,
                 material=material, smooth_angle=smooth_angle, link=link)


def offset_polygon(xz, dist):
    """Offset a closed polygon's vertices along their (mitred) normals.
    dist > 0 moves outward for a CCW polygon (in X->Z orientation)."""
    p = np.asarray(xz, dtype=np.float64)
    e = np.roll(p, -1, axis=0) - p
    L = np.hypot(e[:, 0], e[:, 1])
    L[L == 0] = 1e-12
    t = e / L[:, None]
    n = np.stack([t[:, 1], -t[:, 0]], axis=1)       # right normal = outward for CCW
    n_prev = np.roll(n, 1, axis=0)
    nv = n + n_prev
    nl = np.hypot(nv[:, 0], nv[:, 1])
    nl[nl < 1e-12] = 1e-12
    nv = nv / nl[:, None]
    cosh = np.clip(np.einsum("ij,ij->i", nv, n), 0.35, 1.0)
    return p + nv * (dist / cosh)[:, None]


def _signed_area(xz):
    x, z = xz[:, 0], xz[:, 1]
    return 0.5 * float(np.sum(x * np.roll(z, -1) - np.roll(x, -1) * z))


def extrude_polygon(name, poly_xz, y0, y1, holes=None, chamfer=0.0, collection=None, material=None,
                    smooth_angle=35.0, link=True):
    """Prism: closed polygon in the local XZ plane extruded along Y from y0 to
    y1, with optional holes (list of polygons) and a 45-degree chamfer on
    the outer boundary edges (vertex-normal offset; keep it small relative to
    features).  Caps are triangulated with holes."""
    mb = MeshBuilder()
    outer = np.asarray(poly_xz, dtype=np.float64)
    if _signed_area(outer) < 0:
        outer = outer[::-1]
    holes = [np.asarray(h, dtype=np.float64) for h in (holes or [])]
    holes = [h[::-1] if _signed_area(h) > 0 else h for h in holes]   # CW holes
    loops_lo, loops_hi = [], []
    for k, poly in enumerate([outer] + holes):
        c = chamfer if k == 0 else 0.0
        if c:
            inset = offset_polygon(poly, -c)
            r0 = mb.ring_xz(inset, y0)
            r1 = mb.ring_xz(poly, y0 + c)
            r2 = mb.ring_xz(poly, y1 - c)
            r3 = mb.ring_xz(inset, y1)
            mb.bridge(r0, r1)
            mb.bridge(r1, r2)
            mb.bridge(r2, r3)
            loops_lo.append(r0)
            loops_hi.append(r3)
        else:
            r0 = mb.ring_xz(poly, y0)
            r1 = mb.ring_xz(poly, y1)
            mb.bridge(r0, r1)
            loops_lo.append(r0)
            loops_hi.append(r1)
    mb.cap(loops_lo, axis=1)
    mb.cap(loops_hi, axis=1)
    obj = mb.to_object(name, collection, smooth_angle=smooth_angle, link=link)
    if material is not None:
        assign_material(obj, material)
    return obj


def _fillet_path(points, bend_radius, samples):
    P = [np.asarray(p, dtype=np.float64) for p in points]
    if bend_radius is None or bend_radius <= 0 or len(P) < 3:
        return np.array(P)
    out = [P[0]]
    for i in range(1, len(P) - 1):
        a, b, c = P[i - 1], P[i], P[i + 1]
        d0 = b - a
        d1 = c - b
        l0, l1 = np.linalg.norm(d0), np.linalg.norm(d1)
        if l0 < 1e-12 or l1 < 1e-12:
            continue
        d0 /= l0
        d1 /= l1
        cosang = float(np.clip(np.dot(d0, d1), -1, 1))
        theta = math.acos(cosang)           # turning angle
        if theta < 1e-4:
            out.append(b)
            continue
        t = bend_radius * math.tan(theta / 2)
        t = min(t, 0.49 * l0 if i > 1 else 0.98 * l0, 0.49 * l1 if i < len(P) - 2 else 0.98 * l1)
        R = t / math.tan(theta / 2)
        p0 = b - d0 * t
        p1 = b + d1 * t
        nrm = d1 - d0
        nrm /= np.linalg.norm(nrm)
        centre = b + nrm * (R / math.cos(theta / 2))
        v0 = p0 - centre
        v1 = p1 - centre
        for k in range(samples + 1):
            s = k / samples
            # slerp between v0 and v1
            v = (math.sin((1 - s) * theta) * v0 + math.sin(s * theta) * v1) / math.sin(theta)
            out.append(centre + v)
    out.append(P[-1])
    return np.array(out)


def tube_along(name, points, radius, segments=12, bend_radius=None, bend_samples=8, caps=True,
               inner_radius=0.0, collection=None, material=None, smooth_angle=50.0, link=True):
    """Circular tube swept along a 3D polyline with rounded bends.

    bend_radius: centre-line radius of the bends (default 3*radius); each
    corner of the polyline is replaced by a circular arc.  Frames are
    rotation-minimising (no twist).  caps closes the ends (solid rod / pipe
    end); inner_radius > 0 makes a hollow pipe with annular end caps."""
    if bend_radius is None:
        bend_radius = 3.0 * radius
    path = _fillet_path(points, bend_radius, bend_samples)
    # remove duplicates
    keep = [0]
    for i in range(1, len(path)):
        if np.linalg.norm(path[i] - path[keep[-1]]) > 1e-9:
            keep.append(i)
    path = path[keep]
    n = len(path)
    T = np.zeros_like(path)
    T[1:-1] = path[2:] - path[:-2]
    T[0] = path[1] - path[0]
    T[-1] = path[-1] - path[-2]
    T /= np.linalg.norm(T, axis=1)[:, None]
    # initial normal
    ref = np.array([0.0, 0.0, 1.0]) if abs(T[0][2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    N = np.cross(T[0], ref)
    N /= np.linalg.norm(N)
    Ns = [N]
    for i in range(1, n):
        v = np.cross(T[i - 1], T[i])
        s = np.linalg.norm(v)
        if s > 1e-12:
            ang = math.atan2(s, float(np.dot(T[i - 1], T[i])))
            N = np.array(Matrix.Rotation(ang, 3, Vector(v / s)) @ Vector(N))
        N = N - T[i] * np.dot(N, T[i])
        N /= np.linalg.norm(N)
        Ns.append(N)
    Ns = np.array(Ns)
    B = np.cross(T, Ns)
    a = np.arange(segments) * TAU / segments
    mb = MeshBuilder()

    def rings_for(r):
        out = []
        for i in range(n):
            pts = path[i] + r * (np.cos(a)[:, None] * Ns[i] + np.sin(a)[:, None] * B[i])
            out.append(mb.verts(pts))
        return out

    outer = rings_for(radius)
    for i in range(n - 1):
        mb.bridge(outer[i], outer[i + 1])
    if inner_radius > 0:
        inner = rings_for(inner_radius)
        for i in range(n - 1):
            mb.bridge(inner[i], inner[i + 1])
        if caps:
            mb.bridge(outer[0], inner[0])
            mb.bridge(outer[-1], inner[-1])
    elif caps:
        mb.face(list(outer[0]))
        mb.face(list(outer[-1]))
    obj = mb.to_object(name, collection, smooth_angle=smooth_angle, fix_normals=caps, link=link)
    if material is not None:
        assign_material(obj, material)
    return obj


def rounded_box(name, size, radius=0.0, segments=3, center=(0.0, 0.0, 0.0), collection=None,
                material=None, smooth_angle=35.0, link=True):
    """Box of full size (sx, sy, sz) centred at `center` with rounded edges
    (bmesh bevel of radius `radius`, `segments` segments)."""
    sx, sy, sz = size
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bmesh.ops.scale(bm, vec=Vector((sx, sy, sz)), verts=bm.verts[:])
    r = min(radius, 0.49 * min(sx, sy, sz))
    if r > 0:
        bmesh.ops.bevel(bm, geom=bm.edges[:] + bm.verts[:], offset=r, offset_type="OFFSET",
                        segments=max(1, int(segments)), profile=0.5, affect="EDGES", clamp_overlap=True)
    bmesh.ops.translate(bm, vec=Vector(center), verts=bm.verts[:])
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    obj = _new_object(me, name, collection, link)
    if smooth_angle is not None:
        smooth_by_angle(obj, smooth_angle)
    if material is not None:
        assign_material(obj, material)
    return obj


# ---------------------------------------------------------------------------
# Joining
# ---------------------------------------------------------------------------


def _world_matrix(obj):
    """matrix_world computed from basis + parent chain (no depsgraph update)."""
    M = obj.matrix_basis.copy()
    o = obj
    while o.parent is not None:
        M = o.parent.matrix_basis @ o.matrix_parent_inverse @ M
        o = o.parent
    return M


def join(name, objects, collection=None, delete=True, smooth_angle=None, link=True):
    """Merge mesh objects into one new object (world transforms baked into
    the result's local space = world).  Materials are merged by identity.
    Sharp-edge and smooth flags are preserved (via bmesh)."""
    bm = bmesh.new()
    mats = []
    for ob in objects:
        me = ob.data.copy()
        me.transform(_world_matrix(ob))
        remap = []
        for m in me.materials:
            if m not in mats:
                mats.append(m)
            remap.append(mats.index(m))
        if remap:
            mi = np.zeros(len(me.polygons), dtype=np.int32)
            me.polygons.foreach_get("material_index", mi)
            mi = np.asarray(remap, dtype=np.int32)[np.clip(mi, 0, len(remap) - 1)]
            me.polygons.foreach_set("material_index", mi)
        bm.from_mesh(me)
        bpy.data.meshes.remove(me)
    out = bpy.data.meshes.new(name)
    bm.to_mesh(out)
    bm.free()
    for m in mats:
        out.materials.append(m)
    obj = _new_object(out, name, collection, link)
    if smooth_angle is not None:
        smooth_by_angle(obj, smooth_angle)
    if delete:
        for ob in objects:
            me = ob.data
            bpy.data.objects.remove(ob)
            if me.users == 0:
                bpy.data.meshes.remove(me)
    return obj


# ---------------------------------------------------------------------------
# Cutaways
# ---------------------------------------------------------------------------

_AXES = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}


def _plane_spec(p):
    """Accept (point, normal) | '+X' / '-Z' (plane through origin, remove the
    side the sign points to) | {'co':..., 'no':...}."""
    if isinstance(p, str):
        s = -1.0 if p.strip().startswith("-") else 1.0
        ax = p.strip().lstrip("+-").upper()
        return np.zeros(3), s * np.array(_AXES[ax])
    if isinstance(p, dict):
        return np.asarray(p["co"], dtype=np.float64), np.asarray(p["no"], dtype=np.float64)
    co, no = p
    return np.asarray(co, dtype=np.float64), np.asarray(no, dtype=np.float64)


def _plane_basis(no):
    no = no / np.linalg.norm(no)
    ref = np.array([0.0, 0.0, 1.0]) if abs(no[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(ref, no)
    u /= np.linalg.norm(u)
    v = np.cross(no, u)
    return u, v      # (u, v, no) right-handed


def cut_and_apply(obj, cutter=None, plane=None, wedge=None, planes=None, space="WORLD",
                  section_material_name="section_cut", delete_cutter=False, tol=None):
    """Boolean DIFFERENCE applied at build time; cut faces get the section
    material.

    * plane=(co, no) | '+X' ...: removes the half-space on the +no side
      (half-section).
    * wedge=[(co, n1), (co, n2)] (or planes=...): removes the region on the
      + side of BOTH planes (quarter-section = two perpendicular planes;
      two non-parallel planes are supported).
    * cutter=<mesh object>: generic boolean (Manifold solver, Exact fallback);
      faces lying on the cutter surface get the section material.
    Planes are in world space (space='WORLD', object transform taken from the
    parent chain) or in the object's local space (space='LOCAL').

    The plane path is exact and fast: the mesh is bisected (bmesh, vertices
    within `tol` of a plane count as on it), faces in the removed region are
    deleted, and the cut boundary loops lying on each plane (closed along
    the planes' common line for a wedge) are triangulated with the existing
    boundary vertices (constrained Delaunay, holes by even-odd), so a closed
    input stays closed/manifold.  Section faces are flat-shaded and their
    border edges marked sharp.
    Returns the list of new section face indices.
    """
    if cutter is not None:
        return _cut_with_object(obj, cutter, section_material_name, delete_cutter, tol)
    specs = []
    if plane is not None:
        specs = [_plane_spec(plane)]
    else:
        specs = [_plane_spec(p) for p in (wedge or planes or [])]
    if not specs:
        raise ValueError("cut_and_apply needs cutter, plane or wedge")
    M = _world_matrix(obj) if space.upper() == "WORLD" else Matrix.Identity(4)
    Minv = M.inverted()
    me = obj.data
    co_arr = np.zeros(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co_arr)
    co_arr = co_arr.reshape(-1, 3)
    if len(co_arr) == 0:
        return []
    size = float(np.max(np.ptp(co_arr, axis=0))) + 1e-9
    tol = tol if tol is not None else 1e-6 * size
    loc_specs = []
    for k, (co, no) in enumerate(specs):
        co_l = np.array(Minv @ Vector(co))
        no_l = np.array((Minv.to_3x3().inverted().transposed() @ Vector(no)).normalized())
        loc_specs.append((co_l, no_l))

    bm = bmesh.new()
    bm.from_mesh(me)
    # 1. bisect by all planes (no clearing); vertices within tol of a plane
    #    count as on it (no micro-edges), near-coincident cut points are welded
    for co_i, no_i in loc_specs:
        geom = bm.verts[:] + bm.edges[:] + bm.faces[:]
        res = bmesh.ops.bisect_plane(bm, geom=geom, dist=tol, plane_co=Vector(co_i), plane_no=Vector(no_i))
        cut_v = [g for g in res["geom_cut"] if isinstance(g, bmesh.types.BMVert)]
        if cut_v:
            bmesh.ops.remove_doubles(bm, verts=cut_v, dist=tol)
    # 2. delete faces in the removed region (every face is now on one side of every plane)
    kill = []
    for f in bm.faces:
        c = np.array(f.calc_center_median())
        if all(np.dot(c - co, no) > tol for co, no in loc_specs):
            kill.append(f)
    bmesh.ops.delete(bm, geom=kill, context="FACES")
    for v in [v for v in bm.verts if not v.link_edges]:
        bm.verts.remove(v)
    if len(me.materials) == 0:
        me.materials.append(None)          # keep the original faces on slot 0
    sec_mat = assign_material(obj, section_material_name, faces=[])  # ensure slot
    # 3. fill: for each plane, the cut-boundary edges lying on it form chains;
    #    chains ending on the line shared with another plane are closed by
    #    segments along that line; loops are triangulated (even-odd holes)
    #    using the existing boundary vertices only -> manifold result.
    bedges = [e for e in bm.edges if e.is_boundary]
    for e in bedges:
        e.smooth = False

    def dist(v, i):
        co, no = loc_specs[i]
        return float(np.dot(np.array(v.co) - co, no))

    new_faces = []
    for i, (co_i, no_i) in enumerate(loc_specs):
        u, v = _plane_basis(no_i)
        on = [e for e in bedges if all(abs(dist(x, i)) < tol for x in e.verts)]
        if not on:
            continue
        adj = {}
        for e in on:
            a_, b_ = e.verts
            adj.setdefault(a_, []).append(b_)
            adj.setdefault(b_, []).append(a_)
        # chain ends lie on the line shared with another plane: pair them along it
        ends = [x for x, nb in adj.items() if len(nb) == 1]
        if ends:
            others = [j for j in range(len(loc_specs)) if j != i]
            ldir = np.cross(no_i, loc_specs[others[0]][1]) if others else np.zeros(3)
            if np.linalg.norm(ldir) > 1e-9:
                ldir /= np.linalg.norm(ldir)
                ends.sort(key=lambda x: float(np.dot(np.array(x.co), ldir)))
                for k in range(0, len(ends) - 1, 2):
                    a_, b_ = ends[k], ends[k + 1]
                    adj[a_].append(b_)
                    adj[b_].append(a_)
        loops = []
        seen = set()
        for start in adj:
            if start in seen or len(adj[start]) != 2:
                continue
            lp = [start]
            seen.add(start)
            prev, cur = None, start
            closed = False
            while True:
                nb = [w for w in adj[cur] if w is not prev]
                if len(adj[cur]) != 2 or not nb:
                    break
                nxt = nb[0]
                if nxt is start:
                    closed = True
                    break
                if nxt in seen:
                    break
                lp.append(nxt)
                seen.add(nxt)
                prev, cur = cur, nxt
            if closed and len(lp) >= 3:
                loops.append(lp)
        if not loops:
            continue
        flat = [x for lp in loops for x in lp]
        P2 = np.array([[np.dot(np.array(x.co) - co_i, u), np.dot(np.array(x.co) - co_i, v)] for x in flat])
        rings2 = []
        k0 = 0
        for lp in loops:
            rings2.append(P2[k0:k0 + len(lp)])
            k0 += len(lp)
        tris = triangulate_loops(rings2)
        for t in tris:
            a_, b_, c_ = (int(t[0]), int(t[1]), int(t[2]))
            pa, pb, pc = P2[a_], P2[b_], P2[c_]
            cross = (pb[0] - pa[0]) * (pc[1] - pa[1]) - (pb[1] - pa[1]) * (pc[0] - pa[0])
            if abs(cross) < 1e-30:
                continue
            # (u, v, no) right-handed: CCW in (u,v) -> normal +no (outward of the remaining solid)
            order = (a_, b_, c_) if cross > 0 else (a_, c_, b_)
            try:
                f = bm.faces.new([flat[k] for k in order])
            except ValueError:
                continue
            f.material_index = sec_mat
            f.smooth = False
            new_faces.append(f)
    for f in new_faces:
        for e in f.edges:
            if any(lf.material_index != sec_mat for lf in e.link_faces):
                e.smooth = False
    bm.to_mesh(me)
    bm.free()
    me.update()
    mi = np.zeros(len(me.polygons), dtype=np.int32)
    me.polygons.foreach_get("material_index", mi)
    return list(np.nonzero(mi == sec_mat)[0])


def cut_half(obj, normal="+X", point=(0.0, 0.0, 0.0), space="WORLD", **kw):
    """Half-section: remove everything on the +normal side of the plane
    through `point` ('+X'/'-Z'/... or a 3-vector normal)."""
    no = _plane_spec(normal)[1] if isinstance(normal, str) else np.asarray(normal, dtype=float)
    return cut_and_apply(obj, plane=(point, no), space=space, **kw)


def cut_quarter(obj, n1="+X", n2="+Z", point=(0.0, 0.0, 0.0), space="WORLD", **kw):
    """Quarter-section: remove the 90-degree wedge on the + side of both planes."""
    a = _plane_spec(n1)[1] if isinstance(n1, str) else np.asarray(n1, dtype=float)
    b = _plane_spec(n2)[1] if isinstance(n2, str) else np.asarray(n2, dtype=float)
    return cut_and_apply(obj, wedge=[(point, a), (point, b)], space=space, **kw)


def _cut_with_object(obj, cutter, section_material_name, delete_cutter, tol):
    from mathutils.bvhtree import BVHTree
    me = obj.data
    if len(me.materials) == 0:
        me.materials.append(None)
    sec_slot = assign_material(obj, section_material_name, faces=[])
    mod = obj.modifiers.new("cv_cut", "BOOLEAN")
    mod.operation = "DIFFERENCE"
    mod.object = cutter
    scene_cols = []
    if not cutter.users_collection:
        bpy.context.scene.collection.objects.link(cutter)
        scene_cols.append(cutter)
    unlinked_obj = False
    if not obj.users_collection:
        bpy.context.scene.collection.objects.link(obj)
        unlinked_obj = True
    result = None
    for solver in ("MANIFOLD", "EXACT"):
        try:
            mod.solver = solver
        except TypeError:
            continue
        dg = bpy.context.evaluated_depsgraph_get()
        dg.update()
        ev = obj.evaluated_get(dg)
        tmp = bpy.data.meshes.new_from_object(ev)
        if len(tmp.polygons) > 0 or len(me.polygons) == 0:
            result = tmp
            break
        bpy.data.meshes.remove(tmp)
    obj.modifiers.remove(mod)
    if unlinked_obj:
        bpy.context.scene.collection.objects.unlink(obj)
    for c in scene_cols:
        bpy.context.scene.collection.objects.unlink(c)
    if result is None:
        raise RuntimeError("boolean cut failed")
    # section faces = faces lying on the cutter surface
    Mc = obj.matrix_world.inverted() @ cutter.matrix_world
    cme = cutter.data
    cv = [Mc @ v.co for v in cme.vertices]
    bvh = BVHTree.FromPolygons(cv, [tuple(p.vertices) for p in cme.polygons])
    size = max(obj.dimensions) + 1e-9
    tol = tol if tol is not None else 1e-5 * size
    mi = np.zeros(len(result.polygons), dtype=np.int32)
    result.polygons.foreach_get("material_index", mi)
    sec = []
    for p in result.polygons:
        loc, _nrm, _idx, dist = bvh.find_nearest(p.center)
        if loc is not None and dist < tol:
            mi[p.index] = sec_slot
            sec.append(p.index)
    result.polygons.foreach_set("material_index", mi)
    old = obj.data
    result.materials.clear()
    for m in old.materials:
        result.materials.append(m)
    obj.data = result
    if old.users == 0:
        bpy.data.meshes.remove(old)
    if sec:
        flat = np.zeros(len(result.polygons), dtype=bool)
        flat[sec] = True
        result.polygons.foreach_set("use_smooth", ~flat)
    if delete_cutter:
        cme = cutter.data
        bpy.data.objects.remove(cutter)
        if cme.users == 0:
            bpy.data.meshes.remove(cme)
    return sec
