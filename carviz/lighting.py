"""Studio lighting, worlds (HDRI) and colour management for Cycles.

    from carviz import lighting
    lighting.setup_color_management(scene)                  # AgX + look
    st = lighting.setup_studio("dark", center=(0, -0.08, 0.45), size=0.7)
    st = lighting.setup_studio("light")                     # car on a light-grey cyclorama
    st = lighting.setup_studio("road", follow=car_root)     # long road, lights ride with the car
    lighting.camera_dof(cam, 1.8, fstop=4.0)

Presets
* ``'light'`` - seamless light-grey round cyclorama (radius >= 10 m) for the
  exterior car shots.  Defaults frame the whole car (origin at the front-axle
  ground point; body from y=+0.87 to y=-3.58): centre (0, -1.355, 0.71),
  size 4.45.  Big overhead softbox + key/fill/rim area lights + the
  studio_small_09 HDRI (white cyc + octaboxes) for reflections in the paint.
* ``'dark'`` - dark charcoal round cyclorama for mechanical close-ups.  Soft
  key (front-left, high), fill (right, low), strip rim (behind), overhead pool
  light, plus two glossy-only "V-flat" reflector cards so polished steel
  reads as metal; studio_small_03 HDRI (black curtains, umbrella, ceiling
  light; hot spots soft-compressed) for reflections.  The wall darkens toward
  the top (gentle gradient) and the floor gets a pool of light under the
  subject.  Pass key_azimuth ~ camera azimuth +-45..60 deg so the key comes
  from the camera side (cut faces of cutaways should face the key).
* ``'road'`` - ground 1000 m wide x 1400 m long (y = -400..1000 by default):
  two asphalt lanes along +Y centred on
  x=0 (car lane) and x=-3.5 with dashed/solid markings and tarred joints every
  15 m, concrete slab apron outside; everything fades into a horizon haze, so
  there is no visible edge.  The texture is world-space (the ground never
  moves), so motion reads clearly.  Sun + sky-gradient background + the
  (desaturated) loft-hall HDRI for reflections.

Geometry visibility: the cyclorama WALL is camera-only (invisible to diffuse,
glossy and shadow rays), so metals reflect the HDRI studio and the HDRI is
not blocked; the FLOOR is a normal surface (receives shadows, shows in
reflections).  The world shows a flat preset colour (dark/light) or a sky
gradient (road) to camera rays instead of the HDRI.

Moving centre: every preset puts its lights under an Empty ``studio_rig``
(returned as ``d['rig']``) located at ``center``.  Pass ``follow=obj`` (e.g.
the car root Empty, keyed from the Track) and the rig rides along: a Copy
Location (X, Y, with offset) constraint, or a Child Of constraint when
``follow_heading=True`` (lights also turn with the car).  ``center`` is then
given in the followed object's frame (car frame; default = car centre).
The sun and the road surface stay put in the world.  Or animate/parent
``d['rig']`` yourself.  Rotating the rig about Z swings the
whole light set around the subject (use it to keep the key on camera side).

Cost: 4-6 area lights (+ sun for road) + HDRI; light tree friendly.  Lights
are invisible to camera rays but show as softboxes in reflections.  Light
power is set from a target irradiance at the subject (W/m^2, ~3 = normal
exposure under AgX at exposure 0), compensated for Cycles' spread gain, so
the presets scale with `size`.
"""
from __future__ import annotations

import math
import os

import bpy
import bmesh  # noqa: E402 (needs bpy first)
from mathutils import Vector

from . import materials

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HDRI_DIR = os.path.join(ROOT, "assets", "hdri")

CAR_CENTER = (0.0, -1.355, 0.71)
CAR_SIZE = 4.45

# HDRI per preset: file, strength, z-rotation (deg), saturation, camera background
WORLD_PRESETS = {
    "dark": dict(hdri="studio_small_03_2k.hdr", strength=0.6, rotation=200.0, saturation=0.6,
                 background=(0.0105, 0.0105, 0.0115)),
    "light": dict(hdri="studio_small_09_2k.hdr", strength=1.3, rotation=60.0, saturation=0.5,
                  background=(0.33, 0.33, 0.335)),
    "road": dict(hdri="photo_studio_loft_hall_2k.hdr", strength=1.1, rotation=0.0, saturation=0.25,
                 background="sky"),
}
SKY_HORIZON = materials.ROAD_HAZE
SKY_ZENITH = (0.20, 0.32, 0.55)


# ---------------------------------------------------------------------------
# Colour management, DOF
# ---------------------------------------------------------------------------

def setup_color_management(scene=None, look="AgX - Medium High Contrast", exposure=0.0, gamma=1.0):
    """AgX view transform (graceful highlight roll-off, no clipped hues) with a
    contrast look so metals and paint don't look washed out."""
    scene = scene or bpy.context.scene
    scene.display_settings.display_device = "sRGB"
    vs = scene.view_settings
    vs.view_transform = "AgX"
    try:
        vs.look = look
    except TypeError:
        vs.look = "None"
    vs.exposure = exposure
    vs.gamma = gamma
    vs.use_curve_mapping = False
    scene.sequencer_colorspace_settings.name = "sRGB"
    return vs


def camera_dof(camera, target_distance=None, fstop=5.6, target=None, blades=7, rotation_deg=15.0):
    """Enable depth of field on a camera object (or Camera data).

    target_distance: focus distance in metres; target: object to track focus
    (overrides the distance).  Returns the camera data."""
    cam = camera.data if isinstance(camera, bpy.types.Object) else camera
    dof = cam.dof
    dof.use_dof = True
    dof.aperture_fstop = float(fstop)
    dof.aperture_blades = int(blades)
    dof.aperture_rotation = math.radians(rotation_deg)
    if target is not None:
        dof.focus_object = target
    if target_distance is not None:
        dof.focus_distance = float(target_distance)
    return cam


# ---------------------------------------------------------------------------
# World
# ---------------------------------------------------------------------------

def setup_world(preset="dark", strength=None, rotation_deg=None, show_hdri=False, background=None,
                scene=None, name=None):
    """World for a preset.  The HDRI lights the scene and appears in
    reflections; camera rays see `background` (preset colour, 'sky' gradient)
    unless show_hdri=True.  Returns the world."""
    p = dict(WORLD_PRESETS[preset])
    if strength is not None:
        p["strength"] = strength
    if rotation_deg is not None:
        p["rotation"] = rotation_deg
    if background is not None:
        p["background"] = background
    scene = scene or bpy.context.scene
    name = name or f"cv_world_{preset}"
    w = bpy.data.worlds.get(name) or bpy.data.worlds.new(name)
    w.use_nodes = True
    nt = w.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputWorld")
    tc = nt.nodes.new("ShaderNodeTexCoord")
    mp = nt.nodes.new("ShaderNodeMapping")
    mp.inputs["Rotation"].default_value = (0.0, 0.0, math.radians(p["rotation"]))
    nt.links.new(tc.outputs["Generated"], mp.inputs["Vector"])
    env = nt.nodes.new("ShaderNodeTexEnvironment")
    path = os.path.join(HDRI_DIR, p["hdri"])
    img = bpy.data.images.get(p["hdri"])
    if img is None or os.path.abspath(bpy.path.abspath(img.filepath)) != path:
        img = bpy.data.images.load(path, check_existing=True)
    env.image = img
    nt.links.new(mp.outputs["Vector"], env.inputs["Vector"])
    # soft-compress hot spots (c / (1 + c/K)): small lamps in the HDRI would
    # otherwise cast hard shadows; area lights do the key lighting, the HDRI
    # is the reflection/ambient environment.
    k = float(p.get("hot_clamp", 6.0))
    den = nt.nodes.new("ShaderNodeVectorMath")
    den.operation = "MULTIPLY_ADD"
    nt.links.new(env.outputs["Color"], den.inputs[0])
    den.inputs[1].default_value = (1.0 / k, 1.0 / k, 1.0 / k)
    den.inputs[2].default_value = (1.0, 1.0, 1.0)
    comp = nt.nodes.new("ShaderNodeVectorMath")
    comp.operation = "DIVIDE"
    nt.links.new(env.outputs["Color"], comp.inputs[0])
    nt.links.new(den.outputs["Vector"], comp.inputs[1])
    hs = nt.nodes.new("ShaderNodeHueSaturation")
    hs.inputs["Saturation"].default_value = p["saturation"]
    nt.links.new(comp.outputs["Vector"], hs.inputs["Color"])
    bg_light = nt.nodes.new("ShaderNodeBackground")
    bg_light.inputs["Strength"].default_value = p["strength"]
    nt.links.new(hs.outputs["Color"], bg_light.inputs["Color"])
    if show_hdri:
        nt.links.new(bg_light.outputs[0], out.inputs["Surface"])
    else:
        bg_cam = nt.nodes.new("ShaderNodeBackground")
        bg_cam.inputs["Strength"].default_value = 1.0
        if p["background"] == "sky":
            sep = nt.nodes.new("ShaderNodeSeparateXYZ")
            nt.links.new(tc.outputs["Generated"], sep.inputs["Vector"])
            mr = nt.nodes.new("ShaderNodeMapRange")
            mr.interpolation_type = "SMOOTHSTEP"
            mr.inputs["From Min"].default_value = 0.0
            mr.inputs["From Max"].default_value = 0.55
            nt.links.new(sep.outputs["Z"], mr.inputs["Value"])
            mix = nt.nodes.new("ShaderNodeMix")
            mix.data_type = "RGBA"
            nt.links.new(mr.outputs["Result"], mix.inputs[0])
            mix.inputs[6].default_value = (*SKY_HORIZON, 1.0)
            mix.inputs[7].default_value = (*SKY_ZENITH, 1.0)
            nt.links.new(mix.outputs[2], bg_cam.inputs["Color"])
        else:
            bg_cam.inputs["Color"].default_value = (*p["background"], 1.0)
        lp = nt.nodes.new("ShaderNodeLightPath")
        ms = nt.nodes.new("ShaderNodeMixShader")
        nt.links.new(lp.outputs["Is Camera Ray"], ms.inputs[0])
        nt.links.new(bg_light.outputs[0], ms.inputs[1])
        nt.links.new(bg_cam.outputs[0], ms.inputs[2])
        nt.links.new(ms.outputs[0], out.inputs["Surface"])
    try:
        w.cycles.sampling_method = "MANUAL"
        w.cycles.sample_map_resolution = 1024
    except Exception:
        pass
    w["cv_preset"] = preset
    scene.world = w
    return w


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _link(ob, col):
    col.objects.link(ob)
    return ob


def _collection(name, scene):
    col = bpy.data.collections.get(name)
    if col is None:
        col = bpy.data.collections.new(name)
        scene.collection.children.link(col)
    return col


def _lathe_mesh(name, profile, segments=128, center_vertex=False):
    """Revolve (r, z) profile points about Z.  Faces wind so normals point
    toward the axis / up (inside of a cyclorama).  If center_vertex, a
    triangle fan closes the first ring (r=profile[0][0]) to the axis."""
    bm = bmesh.new()
    rings = []
    for r, z in profile:
        ring = []
        for k in range(segments):
            a = 2 * math.pi * k / segments
            ring.append(bm.verts.new((r * math.cos(a), r * math.sin(a), z)))
        rings.append(ring)
    for i in range(len(rings) - 1):
        r0, r1 = rings[i], rings[i + 1]
        for k in range(segments):
            k1 = (k + 1) % segments
            bm.faces.new((r0[k], r0[k1], r1[k1], r1[k]))
    if center_vertex:
        c = bm.verts.new((0.0, 0.0, profile[0][1]))
        r0 = rings[0]
        for k in range(segments):
            bm.faces.new((c, r0[k], r0[(k + 1) % segments]))
    bm.normal_update()
    # orient: floor faces up, wall faces inward
    for f in bm.faces:
        cen = f.calc_center_median()
        inward = Vector((-cen.x, -cen.y, 0.0))
        want = Vector((0, 0, 1)) if abs(f.normal.z) > 0.7 else inward
        if f.normal.dot(want) < 0:
            f.normal_flip()
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    for p in me.polygons:
        p.use_smooth = True
    return me


def _cyclorama(name, center, radius, height, fillet, floor_z, mat_floor, mat_wall, col):
    """Round seamless cyclorama, built in unit-radius space and scaled by
    `radius` (object coordinates are therefore normalised: the material's
    vertical gradient is size independent).  Returns (floor_obj, wall_obj)."""
    f = fillet / radius
    h = height / radius
    floor_prof = [(i / 8 * (1 - f), 0.0) for i in range(1, 9)]
    wall_prof = []
    for i in range(0, 13):                                   # quarter-circle fillet
        t = i / 12 * math.pi / 2
        wall_prof.append((1 - f + f * math.sin(t), f - f * math.cos(t)))
    for z in (f + (h - f) * u for u in (0.25, 0.5, 0.75, 1.0)):
        wall_prof.append((1.0, z))
    me_f = _lathe_mesh(name + "_floor", floor_prof, center_vertex=True)
    me_w = _lathe_mesh(name + "_wall", wall_prof)
    me_f.materials.append(mat_floor)
    me_w.materials.append(mat_wall)
    obs = []
    for me in (me_f, me_w):
        ob = bpy.data.objects.new(me.name, me)
        ob.location = (center[0], center[1], floor_z)
        ob.scale = (radius, radius, radius)
        materials.ensure_props(ob)
        _link(ob, col)
        obs.append(ob)
    floor, wall = obs
    # The wall is only seen by the camera: reflections show the HDRI studio,
    # and it never blocks HDRI light.
    wall.visible_diffuse = False
    wall.visible_glossy = False
    wall.visible_shadow = False
    wall.visible_volume_scatter = False
    floor.visible_shadow = False     # floor never needs to cast shadows
    for ob in (floor, wall):         # Workbench previews ignore visible_shadow
        ob.display.show_shadows = False
    return floor, wall


def _aim(ob, target):
    d = Vector(target) - ob.location
    ob.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()


# Cycles keeps an area light's power constant when `spread` narrows, so the
# on-axis intensity rises.  Far-field gain measured in Cycles 5.0 (64 spp,
# 0.2 m light at 2 m, white Lambert card): spread deg -> gain.
SPREAD_GAIN = ((60.0, 10.0), (90.0, 4.47), (100.0, 3.60), (120.0, 2.47), (150.0, 1.52), (180.0, 1.0))


def spread_gain(spread_deg):
    """On-axis intensity gain of a Cycles area light with this spread (log-interp)."""
    pts = SPREAD_GAIN
    s = min(max(spread_deg, pts[0][0]), pts[-1][0])
    for (a, ga), (b, gb) in zip(pts, pts[1:]):
        if a <= s <= b:
            t = (s - a) / (b - a)
            return math.exp(math.log(ga) * (1 - t) + math.log(gb) * t)
    return 1.0


def _area_light(name, col, rig, center, az_deg, el_deg, dist, size, irradiance, spread_deg=180.0,
                shape="RECTANGLE", aim=None, glossy_only=False):
    """Area light placed by azimuth/elevation around `center` (camera.py
    convention: az 0 = behind (-Y), 90 = right (+X), 180 = front, -90 = left).
    `irradiance` (W/m^2, ~3 = normal exposure) at `center` sets its power:
    P = E * pi * d^2 / spread_gain (Lambertian emitter, far field; verified
    to ~1 % for spread 180 with tools-side measurement)."""
    a, e = math.radians(az_deg), math.radians(el_deg)
    off = Vector((math.sin(a) * math.cos(e), -math.cos(a) * math.cos(e), math.sin(e))) * dist
    ld = bpy.data.lights.new(name, "AREA")
    ld.shape = shape
    ld.size = size[0]
    ld.size_y = size[1]
    ld.energy = irradiance * math.pi * dist * dist / spread_gain(spread_deg)
    ld.spread = math.radians(spread_deg)
    ld.color = (1.0, 1.0, 1.0)
    ob = bpy.data.objects.new(name, ld)
    ob.location = Vector(center) + off
    _aim(ob, aim if aim is not None else center)
    ob.visible_camera = False
    if glossy_only:
        ob.visible_diffuse = False
        ob.visible_transmission = False
        ob.visible_volume_scatter = False
        ld.use_shadow = False
    _link(ob, col)
    # parent to the rig keeping the world placement (rig matrix is evaluated
    # in _rig(); the light's own matrix_world is not yet, so don't read it)
    ob.parent = rig
    ob.matrix_parent_inverse = rig.matrix_world.inverted()
    return ob


def _rig(name, col, center):
    rig = bpy.data.objects.new(name, None)
    rig.empty_display_type = "SPHERE"
    rig.empty_display_size = 0.2
    rig.location = center
    _link(rig, col)
    bpy.context.view_layer.update()
    return rig


def _follow(rig, follow, heading):
    """Make the rig ride with `follow`.  `center` is then in follow's local
    frame (the car frame when follow is the car root).  Location only:
    Copy Location (X, Y) with offset.  With heading: Child Of (identity
    inverse), i.e. the whole light set moves rigidly with the car."""
    if follow is None:
        return
    if heading:
        c = rig.constraints.new("CHILD_OF")
        c.target = follow
        c.use_scale_x = c.use_scale_y = c.use_scale_z = False
        c.set_inverse_pending = False
        return
    c = rig.constraints.new("COPY_LOCATION")
    c.target = follow
    c.use_z = False
    c.use_offset = True


def _road_preview_marks(col, y0, y1, floor_z):
    """Lane markings as geometry for Workbench previews only (Workbench shows
    material colours, not the road shader).  All Cycles ray visibility is off,
    so Cycles skips it; it matches the shader markings (see materials._road)."""
    bm = bmesh.new()

    def quad(xa, xb, ya, yb):
        z = floor_z + 0.003
        vs = [bm.verts.new(p) for p in ((xa, ya, z), (xb, ya, z), (xb, yb, z), (xa, yb, z))]
        bm.faces.new(vs)

    y = math.floor(y0 / 9.0) * 9.0
    while y < y1:                                  # dashed centre line, 3 m on / 6 m off
        quad(-1.81, -1.69, y + 0.02, y + 3.02)
        y += 9.0
    y = y0
    while y < y1:                                  # solid edge lines (50 m pieces)
        for xc in (1.75, -5.25):
            quad(xc - 0.075, xc + 0.075, y, min(y + 50.0, y1))
        y += 50.0
    me = bpy.data.meshes.new("road_preview_marks")
    bm.to_mesh(me)
    bm.free()
    me.materials.append(materials.get("road_marking"))
    ob = bpy.data.objects.new("road_preview_marks", me)
    for a in ("visible_camera", "visible_diffuse", "visible_glossy", "visible_transmission",
              "visible_volume_scatter", "visible_shadow"):
        setattr(ob, a, False)
    ob.display.show_shadows = False
    materials.ensure_props(ob)
    _link(ob, col)
    return ob


# ---------------------------------------------------------------------------
# Studio presets
# ---------------------------------------------------------------------------

def setup_studio(preset="dark", center=None, size=None, floor_z=0.0, follow=None, follow_heading=False,
                 key_azimuth=None, energy=1.0, world=True, world_kw=None, collection="studio",
                 scene=None, road_length=(-400.0, 1000.0)):
    """Build a studio.  Returns dict with 'world', 'rig', 'collection' and
    dark:  'key', 'fill', 'rim', 'top', 'cards' (list), 'floor', 'backdrop'
    light: 'key', 'fill', 'rim', 'top', 'floor', 'backdrop'
    road:  'sun', 'top', 'ground', 'preview_marks' (Workbench-only lane marks)
    ('floor' / 'backdrop' are the cyclorama floor disc and wall objects.)

    center: subject centre (world; in the followed object's frame when
      `follow` is given).  Defaults: light/road = car centre (0, -1.355, 0.71),
      dark = (0, 0, 0.4).
    size: characteristic subject size in m (defaults 4.45 / 0.8); scales the
      cyclorama radius (dark: max(5*size, 2.5 m); light: max(3*size, 10 m)),
      light distances and softbox sizes.
    floor_z: floor height (0 = ground in car frame).
    follow / follow_heading: see module docstring (moving centre).
    key_azimuth: key azimuth in camera.py orbit degrees (default 225 =
      front-left); every other light is placed relative to it.
    energy: global light multiplier.  world_kw: kwargs for setup_world()."""
    scene = scene or bpy.context.scene
    col = _collection(collection, scene)
    out = {"collection": col}
    if preset == "light":
        center = Vector(center or CAR_CENTER)
        s = float(size or CAR_SIZE)
    elif preset == "dark":
        center = Vector(center or (0.0, 0.0, 0.4))
        s = float(size or 0.8)
    elif preset == "road":
        center = Vector(center or CAR_CENTER)
        s = float(size or CAR_SIZE)
    else:
        raise ValueError(f"unknown studio preset {preset!r}")
    if world:
        out["world"] = setup_world(preset, scene=scene, **(world_kw or {}))
    rig = _rig("studio_rig", col, center)
    out["rig"] = rig
    E = energy

    if preset == "dark":
        R = max(5.0 * s, 2.5)
        floor, wall = _cyclorama("cyc", center, R, 1.15 * R, 0.3 * R, floor_z,
                                 materials.get("floor"), materials.get("backdrop"), col)
        out["floor"], out["backdrop"] = floor, wall
        ka = 225.0 if key_azimuth is None else key_azimuth
        d = 3.0 * s
        out["key"] = _area_light("key", col, rig, center, ka, 42.0, d, (1.6 * s, 1.0 * s), 3.0 * E,
                                 spread_deg=120.0)
        out["fill"] = _area_light("fill", col, rig, center, ka - 120.0, 12.0, 3.5 * s, (2.0 * s, 1.4 * s),
                                  0.55 * E)
        out["rim"] = _area_light("rim", col, rig, center, ka + 165.0, 38.0, 2.8 * s, (1.8 * s, 0.35 * s),
                                 4.0 * E, spread_deg=90.0)
        out["top"] = _area_light("top", col, rig, center, 0.0, 90.0, 2.4 * s, (1.5 * s, 1.5 * s), 1.1 * E,
                                 shape="DISK", spread_deg=100.0)
        # glossy-only reflector cards (V-flats): polished steel needs something
        # bright to reflect, or it reads black in a dark studio.  They do not
        # light diffuse surfaces, so the mood of the studio is unchanged.
        out["cards"] = [
            _area_light(f"card_{i}", col, rig, center, ka + da, 12.0, 3.2 * s, (3.0 * s, 1.6 * s), 0.9 * E,
                        glossy_only=True)
            for i, da in enumerate((-75.0, 95.0))]
    elif preset == "light":
        R = max(3.0 * s, 10.0)
        floor, wall = _cyclorama("cyc", center, R, 0.8 * R, 0.25 * R, floor_z,
                                 materials.get("floor_light"), materials.get("backdrop_light"), col)
        out["floor"], out["backdrop"] = floor, wall
        ka = 225.0 if key_azimuth is None else key_azimuth
        out["top"] = _area_light("top", col, rig, center, 0.0, 90.0, 1.0 * s, (1.35 * s, 0.6 * s), 1.6 * E,
                                 spread_deg=110.0)
        out["key"] = _area_light("key", col, rig, center, ka, 28.0, 1.8 * s, (0.9 * s, 0.5 * s), 1.6 * E,
                                 spread_deg=120.0)
        out["fill"] = _area_light("fill", col, rig, center, ka - 125.0, 18.0, 2.0 * s, (0.8 * s, 0.5 * s),
                                  0.5 * E)
        out["rim"] = _area_light("rim", col, rig, center, ka + 160.0, 30.0, 1.6 * s, (0.7 * s, 0.25 * s),
                                 2.0 * E, spread_deg=90.0)
    else:  # road
        y0, y1 = road_length
        me = bpy.data.meshes.new("road")
        bm = bmesh.new()
        xs = (-500.0, -40.0, 40.0, 500.0)
        ny = 14
        ys = [y0 + (y1 - y0) * i / ny for i in range(ny + 1)]
        grid = [[bm.verts.new((x, y, 0.0)) for x in xs] for y in ys]
        for j in range(ny):
            for i in range(len(xs) - 1):
                bm.faces.new((grid[j][i], grid[j][i + 1], grid[j + 1][i + 1], grid[j + 1][i]))
        bm.to_mesh(me)
        bm.free()
        me.materials.append(materials.get("road"))
        ground = bpy.data.objects.new("road", me)
        ground.location = (0.0, 0.0, floor_z)
        ground.visible_shadow = False
        ground.display.show_shadows = False
        materials.ensure_props(ground)
        _link(ground, col)
        out["ground"] = ground
        out["preview_marks"] = _road_preview_marks(col, y0, y1, floor_z)
        sun_d = bpy.data.lights.new("sun", "SUN")
        sun_d.energy = 3.2 * E
        sun_d.angle = math.radians(1.5)
        sun_d.color = (1.0, 0.96, 0.90)
        sun = bpy.data.objects.new("sun", sun_d)
        ka = 225.0 if key_azimuth is None else key_azimuth
        a, e = math.radians(ka), math.radians(42.0)
        sun.location = Vector(center) + Vector((math.sin(a), -math.cos(a), math.tan(e))) * 10.0
        _aim(sun, center)
        _link(sun, col)
        out["sun"] = sun
        # soft sky fill above the car, rides along with the rig
        out["top"] = _area_light("top", col, rig, center, 0.0, 90.0, 1.2 * s, (1.3 * s, 0.6 * s), 0.6 * E,
                                 spread_deg=140.0)
    _follow(rig, follow, follow_heading)
    return out


def clear_studio(collection="studio"):
    """Remove a studio collection and its objects (for re-running look-dev)."""
    col = bpy.data.collections.get(collection)
    if col is None:
        return
    for ob in list(col.objects):
        bpy.data.objects.remove(ob, do_unlink=True)
    bpy.data.collections.remove(col)
