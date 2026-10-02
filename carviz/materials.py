"""Physically based look-dev materials for Cycles (CPU) + Workbench viewport colours.

    from carviz import materials
    mat = materials.get("steel_machined")      # cached bpy.types.Material
    materials.ensure_props(obj)                # cv_opacity=1 / cv_glow=0 if missing
    materials.all_names()                      # every name in ARCHITECTURE.md section 5

Every material ends in the shared node group ``CV_Presentation``:

    final = mix(Transparent BSDF, base_shader + warm glow emission, opacity)

* ``opacity`` = object custom property ``cv_opacity`` read with an Attribute
  node (type OBJECT).  A *missing* object attribute reads as Fac 0 / Alpha 0 in
  Cycles, while a present one reads Alpha 1, so the group uses
  ``opacity = lerp(1, Fac, Alpha)``: objects WITHOUT the property render fully
  opaque, objects with it render at its value (verified by tools/lookdev.py
  --verify).  Partly faded surfaces get slightly denser toward their
  silhouettes (``op + op*(1-op)*facing^2``; exact face-on and at 0/1) so
  x-ray shells keep their outline.  NB a closed shell is two surfaces: at
  cv_opacity a it hides 1-(1-a)^2 of what is behind it (0.25 -> 0.44).
  Still, call ``ensure_props(obj)`` / ``rig.set_presentation`` on
  everything you animate so the property exists before it is keyed.
* ``glow`` = ``cv_glow`` (0..1): emission colour GLOW_COLOR = (1.0, 0.32, 0.02)
  (15 % tinted toward the material's own hue), strength
  ``GLOW_STRENGTH * cv_glow`` = 2 * cv_glow, x0.5 (face-on) .. x1.5 (silhouette)
  so glowing parts keep their shape; the base shader is dimmed by
  ``GLOW_DIM * cv_glow`` (0.7).  A deeper hue / lower strength than the
  nominal (1.0, 0.42, 0.08) x 4 because AgX desaturates bright oranges to
  peach/salmon, and neutral metal reflections added on top read pink.

Shading notes
* Texture coordinates are OBJECT space (metres for unscaled objects), so
  textures stick to moving parts.  Lathe-turned look (anisotropy) uses a radial
  tangent about the object's local +Y axis = the spin axis of every rotating
  part (ARCHITECTURE.md section 2).
* No SSS, no volumes, no refraction.  At most two noise textures per material.
* ``material.cycles.emission_sampling`` is NONE for everything except
  ``gas_burning`` so the glow emission never turns every mesh into a light
  source (that would explode the light tree).
* Glass, gases and brake fluid are "thin" shaders built from Transparent +
  Glossy/Emission, so they cost no refraction bounces and cast light shadows.
* ``material.diffuse_color`` / ``metallic`` / ``roughness`` are set for
  Workbench previews (glass/gas/fluid have viewport alpha < 1).

Extra (non-contract) names: ``floor_light``, ``backdrop_light`` (light-grey
cyclorama used by lighting.setup_studio('light')), ``road`` (asphalt road with
lane markings + concrete apron, world-space, used by setup_studio('road')),
``road_marking`` (white road paint; used by the road's Workbench-only marks).
"""
from __future__ import annotations

import warnings

import bpy

VERSION = 9                       # bump to force node trees to be rebuilt
GROUP_NAME = "CV_Presentation"
GLOW_COLOR = (1.0, 0.32, 0.02)
GLOW_STRENGTH = 2.0
GLOW_DIM = 0.7            # fraction of the base shader replaced at cv_glow = 1

# Table names from ARCHITECTURE.md section 5 (order = table order).
NAMES = (
    "cast_iron", "cast_aluminium", "machined_aluminium", "steel_machined", "steel_ground",
    "steel_forged", "steel_dark", "brass", "friction", "rubber", "tire_rubber", "plastic_black",
    "paint_black", "car_paint", "glass", "rim_alloy", "chrome", "copper", "ceramic",
    "section_cut", "gas_intake", "gas_compressed", "gas_burning", "gas_exhaust", "brake_fluid",
    "floor", "backdrop",
)
EXTRA_NAMES = ("floor_light", "backdrop_light", "road", "road_marking")

_BUILDERS = {}


def all_names(include_extra=False):
    """Material names of the ARCHITECTURE.md table (plus studio extras if asked)."""
    return list(NAMES) + (list(EXTRA_NAMES) if include_extra else [])


# ---------------------------------------------------------------------------
# Object helpers
# ---------------------------------------------------------------------------

def ensure_props(obj, recursive=False, opacity=1.0, glow=0.0):
    """Create cv_opacity / cv_glow on obj (and children if recursive) if missing.

    Existing values are left untouched.  Returns obj."""
    objs = [obj] + (list(obj.children_recursive) if recursive else [])
    for o in objs:
        if "cv_opacity" not in o:
            o["cv_opacity"] = float(opacity)
        if "cv_glow" not in o:
            o["cv_glow"] = float(glow)
    return obj


def ensure_scene_props(scene=None):
    """ensure_props() on every renderable object of the scene."""
    scene = scene or bpy.context.scene
    n = 0
    for o in scene.objects:
        if o.type in {"MESH", "CURVE", "SURFACE", "META", "FONT"}:
            ensure_props(o)
            n += 1
    return n


# ---------------------------------------------------------------------------
# Small node-graph helpers
# ---------------------------------------------------------------------------

def _sock(n, name, outputs=False, kind=None):
    """Socket by name or index; for ShaderNodeMix pick the socket of `kind`
    ('Float' | 'Vector' | 'Color') by identifier."""
    coll = n.outputs if outputs else n.inputs
    if isinstance(name, int):
        return coll[name]
    if kind is not None:
        ident = f"{name}_{kind}"
        for s in coll:
            if s.identifier == ident:
                return s
    s = coll.get(name)
    if s is None:
        raise KeyError(f"{n.bl_idname} has no {'output' if outputs else 'input'} {name!r}")
    return s


def _out(v):
    """Node -> its first enabled output socket; socket -> itself."""
    if isinstance(v, bpy.types.Node):
        for s in v.outputs:
            if s.enabled:
                return s
        return v.outputs[0]
    return v


def _set(nt, sock, v):
    if isinstance(v, (bpy.types.Node, bpy.types.NodeSocket)):
        nt.links.new(_out(v), sock)
        return
    if isinstance(v, (tuple, list)) and len(v) == 3 and sock.type == "RGBA":
        v = (v[0], v[1], v[2], 1.0)
    sock.default_value = v


def _n(nt, typ, ins=None, kind=None, **props):
    n = nt.nodes.new(typ)
    for k, v in props.items():
        setattr(n, k, v)
    for k, v in (ins or {}).items():
        _set(nt, _sock(n, k, kind=kind), v)
    return n


def _math(nt, op, a, b=None, c=None, clamp=False):
    n = _n(nt, "ShaderNodeMath", operation=op, use_clamp=clamp)
    for i, v in enumerate((a, b, c)):
        if v is not None:
            _set(nt, n.inputs[i], v)
    return n.outputs[0]


def _mix_f(nt, fac, a, b, clamp=True):
    n = _n(nt, "ShaderNodeMix", data_type="FLOAT", clamp_result=clamp)
    _set(nt, _sock(n, "Factor", kind="Float"), fac)
    _set(nt, _sock(n, "A", kind="Float"), a)
    _set(nt, _sock(n, "B", kind="Float"), b)
    return _sock(n, "Result", outputs=True, kind="Float")


def _mix_c(nt, fac, a, b, blend="MIX"):
    n = _n(nt, "ShaderNodeMix", data_type="RGBA", blend_type=blend)
    _set(nt, _sock(n, "Factor", kind="Float"), fac)
    _set(nt, _sock(n, "A", kind="Color"), a)
    _set(nt, _sock(n, "B", kind="Color"), b)
    return _sock(n, "Result", outputs=True, kind="Color")


def _maprange(nt, v, a0, a1, b0=0.0, b1=1.0, interp="LINEAR", clamp=True):
    n = _n(nt, "ShaderNodeMapRange", interpolation_type=interp, clamp=clamp)
    _set(nt, n.inputs["Value"], v)
    for k, val in (("From Min", a0), ("From Max", a1), ("To Min", b0), ("To Max", b1)):
        _set(nt, n.inputs[k], val)
    return n.outputs["Result"]


def _obj_coords(nt):
    return _n(nt, "ShaderNodeTexCoord").outputs["Object"]


def _noise(nt, vec, scale, detail=2.0, rough=0.55, dist=0.0, out="Fac"):
    n = _n(nt, "ShaderNodeTexNoise", ins={"Vector": vec, "Scale": scale, "Detail": detail,
                                          "Roughness": rough, "Distortion": dist})
    return n.outputs["Factor" if out == "Fac" else "Color"]


def _bump(nt, height, strength, distance, normal=None):
    n = _n(nt, "ShaderNodeBump", ins={"Height": height, "Strength": strength, "Distance": distance})
    if normal is not None:
        _set(nt, n.inputs["Normal"], normal)
    return n.outputs["Normal"]


def _radial_tangent_y(nt):
    """Circumferential tangent about local +Y (lathe-turned / ground surfaces)."""
    return _n(nt, "ShaderNodeTangent", direction_type="RADIAL", axis="Y").outputs["Tangent"]


_PBSDF_KEYS = {
    "base": "Base Color", "metallic": "Metallic", "rough": "Roughness", "ior": "IOR",
    "normal": "Normal", "spec": "Specular IOR Level", "spec_tint": "Specular Tint",
    "aniso": "Anisotropic", "aniso_rot": "Anisotropic Rotation", "tangent": "Tangent",
    "coat": "Coat Weight", "coat_rough": "Coat Roughness", "coat_ior": "Coat IOR",
    "coat_tint": "Coat Tint", "coat_normal": "Coat Normal", "sheen": "Sheen Weight",
    "sheen_rough": "Sheen Roughness", "sheen_tint": "Sheen Tint",
    "diffuse_rough": "Diffuse Roughness",
}


def _principled(nt, **kw):
    ins = {_PBSDF_KEYS[k]: v for k, v in kw.items()}
    return _n(nt, "ShaderNodeBsdfPrincipled", ins=ins).outputs["BSDF"]


def _hue(c):
    """Colour normalised to max component 1 (used to tint the glow)."""
    m = max(c[:3]) or 1.0
    return tuple(min(1.0, x / m) for x in c[:3])


# ---------------------------------------------------------------------------
# Shared presentation group
# ---------------------------------------------------------------------------

def presentation_group():
    """The shared CV_Presentation node group (created/rebuilt on demand)."""
    ng = bpy.data.node_groups.get(GROUP_NAME)
    if ng is not None and ng.get("cv_version") == VERSION:
        return ng
    if ng is None:
        ng = bpy.data.node_groups.new(GROUP_NAME, "ShaderNodeTree")
    ng.nodes.clear()
    ng.interface.clear()
    ng.interface.new_socket("Shader", in_out="INPUT", socket_type="NodeSocketShader")
    tint = ng.interface.new_socket("Glow Tint", in_out="INPUT", socket_type="NodeSocketColor")
    tint.default_value = (1.0, 1.0, 1.0, 1.0)
    ng.interface.new_socket("Shader", in_out="OUTPUT", socket_type="NodeSocketShader")
    gi = ng.nodes.new("NodeGroupInput")
    go = ng.nodes.new("NodeGroupOutput")
    a_op = _n(ng, "ShaderNodeAttribute", attribute_type="OBJECT", attribute_name="cv_opacity")
    a_gl = _n(ng, "ShaderNodeAttribute", attribute_type="OBJECT", attribute_name="cv_glow")
    # Missing property -> Alpha 0 -> opacity 1 (opaque); present -> its value.
    op = _mix_f(ng, a_op.outputs["Alpha"], 1.0, a_op.outputs["Fac"], clamp=True)
    # ghost edges: op_eff = op + op*(1-op)*facing^2 -- exact at op 0 / 1 and
    # face-on, up to 2op-op^2 at silhouettes, so x-ray shells keep their shape
    facing = _n(ng, "ShaderNodeLayerWeight", ins={"Blend": 0.5}).outputs["Facing"]
    edge = _math(ng, "MULTIPLY", facing, facing)
    t = _math(ng, "MULTIPLY", op, _math(ng, "SUBTRACT", 1.0, op))
    opacity = _math(ng, "MULTIPLY_ADD", t, edge, op, clamp=True)
    # hue tint by multiplication: a white (steel) tint leaves the glow unchanged,
    # brass/copper nudge it toward yellow/red (15 %)
    glow_col = _mix_c(ng, 0.15, GLOW_COLOR, gi.outputs["Glow Tint"], blend="MULTIPLY")
    rim = _math(ng, "MULTIPLY_ADD", facing, 1.0, 0.5)           # 0.5 .. 1.5 toward silhouettes
    glow = _math(ng, "MAXIMUM", a_gl.outputs["Fac"], 0.0)
    # base * (1 - GLOW_DIM*glow) + emission: dimming the lit surface a little
    # keeps the glow a saturated orange instead of a pale peach over bright metal
    fac = _math(ng, "MULTIPLY", glow, GLOW_DIM, clamp=True)
    emit = _n(ng, "ShaderNodeEmission", ins={"Color": glow_col,
                                             "Strength": _math(ng, "MULTIPLY", rim, GLOW_STRENGTH / GLOW_DIM)})
    add = _n(ng, "ShaderNodeMixShader")
    ng.links.new(fac, add.inputs[0])
    ng.links.new(gi.outputs["Shader"], add.inputs[1])
    ng.links.new(emit.outputs[0], add.inputs[2])
    transp = _n(ng, "ShaderNodeBsdfTransparent", ins={"Color": (1.0, 1.0, 1.0)})
    mix = _n(ng, "ShaderNodeMixShader")
    ng.links.new(opacity, mix.inputs[0])
    ng.links.new(transp.outputs[0], mix.inputs[1])
    ng.links.new(add.outputs[0], mix.inputs[2])
    ng.links.new(mix.outputs[0], go.inputs["Shader"])
    ng["cv_version"] = VERSION
    return ng


# ---------------------------------------------------------------------------
# Material registry
# ---------------------------------------------------------------------------

def _reg(name, viewport, metallic=0.0, roughness=0.5, tint=None, emission_sampling="NONE"):
    def deco(fn):
        _BUILDERS[name] = dict(fn=fn, viewport=viewport, metallic=metallic, roughness=roughness,
                               tint=tint or viewport[:3], emission_sampling=emission_sampling)
        return fn
    return deco


def get(name):
    """Cached material by name (created on first use, rebuilt if outdated).

    Raises KeyError for unknown names."""
    spec = _BUILDERS.get(name)
    if spec is None:
        raise KeyError(f"carviz.materials: unknown material {name!r}; known: {all_names(True)}")
    m = bpy.data.materials.get(name)
    if m is not None and m.get("cv_version") == VERSION and m.node_tree is not None:
        return m
    if m is None:
        m = bpy.data.materials.new(name)
    _build(m, name, spec)
    return m


def get_all(include_extra=True):
    return {n: get(n) for n in all_names(include_extra)}


def _build(m, name, spec):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        if m.node_tree is None:
            m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    out.target = "ALL"
    shader = spec["fn"](nt)
    grp = nt.nodes.new("ShaderNodeGroup")
    grp.node_tree = presentation_group()
    nt.links.new(_out(shader), grp.inputs["Shader"])
    grp.inputs["Glow Tint"].default_value = (*_hue(spec["tint"]), 1.0)
    nt.links.new(grp.outputs["Shader"], out.inputs["Surface"])
    # tidy layout (purely cosmetic for anyone opening the .blend)
    for i, nd in enumerate(nt.nodes):
        nd.location = (-220 * (i % 8), -180 * (i // 8))
    out.location = (400, 0)
    grp.location = (200, 0)
    # viewport / Workbench
    m.diffuse_color = spec["viewport"]
    m.metallic = spec["metallic"]
    m.roughness = spec["roughness"]
    try:
        m.cycles.emission_sampling = spec["emission_sampling"]
    except Exception:
        pass
    try:
        m.surface_render_method = "BLENDED" if spec["viewport"][3] < 1.0 else "DITHERED"
    except Exception:
        pass
    m["cv_version"] = VERSION
    m["cv_material"] = name


# ---------------------------------------------------------------------------
# Material definitions.  Colours are scene-linear.  Metal base colours are
# their (slightly darkened, real-world-dirty) F0 reflectances.
# ---------------------------------------------------------------------------

def _cast_surface(nt, dark, light, r0, r1, bump_scale, bump_dist, var_scale=5.0):
    """Sand-cast look: low-frequency colour/roughness mottling + fine grain bump."""
    co = _obj_coords(nt)
    var = _noise(nt, co, var_scale, detail=3.0, rough=0.6)
    var = _maprange(nt, var, 0.3, 0.7, interp="SMOOTHSTEP")
    base = _mix_c(nt, var, dark, light)
    rough = _mix_f(nt, var, r1, r0)
    grain = _noise(nt, co, bump_scale, detail=2.0, rough=0.7)
    normal = _bump(nt, grain, 1.0, bump_dist)
    return base, rough, normal


@_reg("cast_iron", viewport=(0.17, 0.17, 0.18, 1), metallic=0.6, roughness=0.6)
def _cast_iron(nt):
    base, rough, normal = _cast_surface(nt, (0.085, 0.084, 0.083), (0.15, 0.148, 0.145),
                                        0.58, 0.72, 420.0, 0.00035)
    return _principled(nt, base=base, metallic=0.8, rough=rough, normal=normal)


@_reg("cast_aluminium", viewport=(0.50, 0.51, 0.52, 1), metallic=0.8, roughness=0.45)
def _cast_aluminium(nt):
    base, rough, normal = _cast_surface(nt, (0.36, 0.365, 0.37), (0.50, 0.505, 0.51),
                                        0.42, 0.55, 380.0, 0.0002)
    return _principled(nt, base=base, metallic=1.0, rough=rough, normal=normal)


@_reg("machined_aluminium", viewport=(0.78, 0.79, 0.80, 1), metallic=0.9, roughness=0.25)
def _machined_aluminium(nt):
    return _principled(nt, base=(0.86, 0.87, 0.88), metallic=1.0, rough=0.25,
                       aniso=0.55, tangent=_radial_tangent_y(nt))


@_reg("steel_machined", viewport=(0.62, 0.63, 0.64, 1), metallic=0.9, roughness=0.22)
def _steel_machined(nt):
    # turned/ground steel with an oil film (thin clear coat)
    return _principled(nt, base=(0.60, 0.605, 0.615), metallic=1.0, rough=0.22,
                       aniso=0.35, tangent=_radial_tangent_y(nt),
                       coat=0.3, coat_rough=0.06, coat_ior=1.47, coat_tint=(1.0, 0.985, 0.95))


@_reg("steel_ground", viewport=(0.80, 0.80, 0.82, 1), metallic=1.0, roughness=0.08)
def _steel_ground(nt):
    return _principled(nt, base=(0.73, 0.735, 0.745), metallic=1.0, rough=0.08,
                       aniso=0.2, tangent=_radial_tangent_y(nt))


@_reg("steel_forged", viewport=(0.30, 0.30, 0.31, 1), metallic=0.8, roughness=0.4)
def _steel_forged(nt):
    co = _obj_coords(nt)
    var = _maprange(nt, _noise(nt, co, 14.0, detail=3.0, rough=0.6), 0.3, 0.7)
    base = _mix_c(nt, var, (0.20, 0.20, 0.205), (0.32, 0.318, 0.315))
    rough = _mix_f(nt, var, 0.46, 0.36)
    normal = _bump(nt, _noise(nt, co, 260.0, detail=1.0), 0.5, 0.0002)
    return _principled(nt, base=base, metallic=0.95, rough=rough, normal=normal,
                       coat=0.15, coat_rough=0.1, coat_ior=1.47)


@_reg("steel_dark", viewport=(0.06, 0.065, 0.08, 1), metallic=0.9, roughness=0.32)
def _steel_dark(nt):
    # black-oxide / blued spring steel, lightly oiled
    return _principled(nt, base=(0.055, 0.06, 0.08), metallic=1.0, rough=0.32,
                       coat=0.3, coat_rough=0.1, coat_ior=1.47)


@_reg("brass", viewport=(0.78, 0.60, 0.28, 1), metallic=1.0, roughness=0.3)
def _brass(nt):
    return _principled(nt, base=(0.80, 0.60, 0.27), metallic=1.0, rough=0.3,
                       aniso=0.3, tangent=_radial_tangent_y(nt), spec_tint=(1.0, 0.92, 0.72))


@_reg("friction", viewport=(0.085, 0.072, 0.06, 1), metallic=0.0, roughness=0.85)
def _friction(nt):
    co = _obj_coords(nt)
    fleck = _noise(nt, co, 900.0, detail=1.0, rough=0.5)
    fleck = _maprange(nt, fleck, 0.66, 0.70)                 # ~5 % brass/copper wire flecks
    mott = _maprange(nt, _noise(nt, co, 60.0, detail=2.0), 0.35, 0.65)
    fibre = _mix_c(nt, mott, (0.052, 0.044, 0.036), (0.085, 0.072, 0.058))
    base = _mix_c(nt, fleck, fibre, (0.40, 0.28, 0.13))
    rough = _mix_f(nt, fleck, 0.86, 0.45)
    metal = _mix_f(nt, fleck, 0.0, 0.85)
    normal = _bump(nt, mott, 0.35, 0.0002)
    return _principled(nt, base=base, metallic=metal, rough=rough, normal=normal, spec=0.35)


@_reg("rubber", viewport=(0.035, 0.035, 0.037, 1), metallic=0.0, roughness=0.55)
def _rubber(nt):
    return _principled(nt, base=(0.028, 0.028, 0.03), rough=0.55, spec=0.4,
                       sheen=0.15, sheen_rough=0.4)


@_reg("tire_rubber", viewport=(0.03, 0.03, 0.032, 1), metallic=0.0, roughness=0.7)
def _tire_rubber(nt):
    co = _obj_coords(nt)
    var = _maprange(nt, _noise(nt, co, 40.0, detail=2.0), 0.35, 0.65)
    base = _mix_c(nt, var, (0.020, 0.020, 0.021), (0.032, 0.031, 0.031))
    rough = _mix_f(nt, var, 0.62, 0.74)
    return _principled(nt, base=base, rough=rough, spec=0.35, sheen=0.25, sheen_rough=0.45,
                       sheen_tint=(0.6, 0.6, 0.62))


@_reg("plastic_black", viewport=(0.03, 0.03, 0.032, 1), metallic=0.0, roughness=0.55)
def _plastic_black(nt):
    co = _obj_coords(nt)
    normal = _bump(nt, _noise(nt, co, 1200.0, detail=1.0), 0.25, 0.0001)   # fine moulded texture
    return _principled(nt, base=(0.024, 0.024, 0.026), rough=0.55, spec=0.5, normal=normal)


@_reg("paint_black", viewport=(0.025, 0.025, 0.027, 1), metallic=0.0, roughness=0.35)
def _paint_black(nt):
    # satin black powder coat
    return _principled(nt, base=(0.018, 0.018, 0.02), rough=0.35, spec=0.5,
                       coat=0.25, coat_rough=0.3)


CAR_PAINT_FACE = (0.020, 0.060, 0.200)   # dark metallic blue, face-on (linear)
CAR_PAINT_FLOP = (0.004, 0.012, 0.050)   # at grazing angles


@_reg("car_paint", viewport=(0.04, 0.08, 0.22, 1), metallic=0.7, roughness=0.25,
      tint=(0.35, 0.55, 1.0))
def _car_paint(nt):
    lw = _n(nt, "ShaderNodeLayerWeight", ins={"Blend": 0.45})
    base = _mix_c(nt, lw.outputs["Facing"], CAR_PAINT_FACE, CAR_PAINT_FLOP)
    # metallic flakes: tiny random normal tilts (sparkle in highlights)
    vor = _n(nt, "ShaderNodeTexVoronoi", ins={"Vector": _obj_coords(nt), "Scale": 2500.0})
    flake = _n(nt, "ShaderNodeVectorMath", operation="SUBTRACT",
               ins={0: vor.outputs["Color"], 1: (0.5, 0.5, 0.5)})
    geo = _n(nt, "ShaderNodeNewGeometry")
    nrm = _n(nt, "ShaderNodeVectorMath", operation="MULTIPLY_ADD",
             ins={0: flake.outputs["Vector"], 1: (0.12, 0.12, 0.12), 2: geo.outputs["Normal"]})
    nrm = _n(nt, "ShaderNodeVectorMath", operation="NORMALIZE", ins={0: nrm.outputs["Vector"]})
    return _principled(nt, base=base, metallic=0.75, rough=0.36, normal=nrm.outputs["Vector"],
                       coat=1.0, coat_rough=0.025, coat_ior=1.5)


@_reg("glass", viewport=(0.55, 0.62, 0.60, 0.25), metallic=0.0, roughness=0.05,
      tint=(0.8, 1.0, 0.9))
def _glass(nt):
    # Thin automotive glass: no refraction.  Transmission = Transparent BSDF with a
    # slight green tint (iron oxide), reflection = Fresnel-weighted glossy.
    fres = _n(nt, "ShaderNodeFresnel", ins={"IOR": 1.52}).outputs[0]
    transp = _n(nt, "ShaderNodeBsdfTransparent", ins={"Color": (0.80, 0.88, 0.84)})
    glossy = _n(nt, "ShaderNodeBsdfGlossy", ins={"Color": (1.0, 1.0, 1.0), "Roughness": 0.02})
    # shadow rays see almost clear glass
    lp = _n(nt, "ShaderNodeLightPath")
    fac = _math(nt, "MULTIPLY", fres, _math(nt, "SUBTRACT", 1.0, lp.outputs["Is Shadow Ray"]))
    mix = _n(nt, "ShaderNodeMixShader")
    nt.links.new(fac, mix.inputs[0])
    nt.links.new(transp.outputs[0], mix.inputs[1])
    nt.links.new(glossy.outputs[0], mix.inputs[2])
    return mix


@_reg("rim_alloy", viewport=(0.62, 0.63, 0.65, 1), metallic=0.8, roughness=0.3)
def _rim_alloy(nt):
    # silver-painted cast alloy wheel with clear coat
    return _principled(nt, base=(0.62, 0.63, 0.65), metallic=0.85, rough=0.32,
                       coat=1.0, coat_rough=0.04, coat_ior=1.5)


@_reg("chrome", viewport=(0.85, 0.86, 0.88, 1), metallic=1.0, roughness=0.04)
def _chrome(nt):
    return _principled(nt, base=(0.72, 0.73, 0.75), metallic=1.0, rough=0.035)


@_reg("copper", viewport=(0.85, 0.50, 0.36, 1), metallic=1.0, roughness=0.3)
def _copper(nt):
    return _principled(nt, base=(0.93, 0.56, 0.40), metallic=1.0, rough=0.3,
                       spec_tint=(1.0, 0.85, 0.75))


@_reg("ceramic", viewport=(0.85, 0.85, 0.83, 1), metallic=0.0, roughness=0.15)
def _ceramic(nt):
    # glazed alumina insulator
    return _principled(nt, base=(0.80, 0.80, 0.78), rough=0.25, coat=1.0, coat_rough=0.03,
                       coat_ior=1.52)


SECTION_RED = (0.37, 0.028, 0.020)   # orchestrator: toned down ~20% so large cut faces do not dominate


@_reg("section_cut", viewport=(0.75, 0.10, 0.07, 1), metallic=0.0, roughness=0.5,
      tint=(1.0, 0.25, 0.2))
def _section_cut(nt):
    # matte signal-red paint, as on real training cutaways
    return _principled(nt, base=SECTION_RED, rough=0.5, spec=0.45)


def _gas(nt, face_col, edge_col, strength, a_face, a_edge, core_boost=0.0, transp_col=(1.0, 1.0, 1.0)):
    """Cheap 'volume' look on a closed surface: denser/brighter toward the
    middle of the silhouette (more gas along the view ray), thin at the rim."""
    facing = _n(nt, "ShaderNodeLayerWeight", ins={"Blend": 0.5}).outputs["Facing"]
    thick = _math(nt, "SUBTRACT", 1.0, facing, clamp=True)       # 1 centre .. 0 rim
    alpha = _mix_f(nt, thick, a_edge, a_face)
    col = _mix_c(nt, thick, edge_col, face_col)
    s = _mix_f(nt, thick, strength * (1.0 - core_boost), strength, clamp=False)
    emit = _n(nt, "ShaderNodeEmission", ins={"Color": col, "Strength": s})
    transp = _n(nt, "ShaderNodeBsdfTransparent", ins={"Color": transp_col})
    mix = _n(nt, "ShaderNodeMixShader")
    nt.links.new(alpha, mix.inputs[0])
    nt.links.new(transp.outputs[0], mix.inputs[1])
    nt.links.new(emit.outputs[0], mix.inputs[2])
    return mix


@_reg("gas_intake", viewport=(0.45, 0.70, 1.0, 0.25), roughness=1.0, tint=(0.5, 0.75, 1.0))
def _gas_intake(nt):
    return _gas(nt, (0.30, 0.58, 1.0), (0.45, 0.70, 1.0), 0.9, 0.38, 0.14, 0.3, (0.85, 0.93, 1.0))


@_reg("gas_compressed", viewport=(0.20, 0.42, 1.0, 0.45), roughness=1.0, tint=(0.3, 0.55, 1.0))
def _gas_compressed(nt):
    return _gas(nt, (0.07, 0.24, 1.0), (0.16, 0.38, 1.0), 1.4, 0.72, 0.30, 0.3, (0.6, 0.75, 1.0))


@_reg("gas_burning", viewport=(1.0, 0.55, 0.12, 0.7), roughness=1.0, tint=(1.0, 0.6, 0.2),
      emission_sampling="AUTO")
def _gas_burning(nt):
    # AgX desaturates bright oranges toward white, so keep the strength modest
    # and the hue deep: core ~ (232,178,107) sRGB, rim deeper orange.
    return _gas(nt, (1.0, 0.45, 0.0), (1.0, 0.22, 0.0), 1.9, 0.92, 0.55, 0.5, (1.0, 0.55, 0.22))


@_reg("gas_exhaust", viewport=(0.40, 0.33, 0.27, 0.4), roughness=1.0, tint=(0.8, 0.65, 0.5))
def _gas_exhaust(nt):
    # grey-brown smoky gas: lit (diffuse) + a faint residual warm glow
    facing = _n(nt, "ShaderNodeLayerWeight", ins={"Blend": 0.5}).outputs["Facing"]
    thick = _math(nt, "SUBTRACT", 1.0, facing, clamp=True)
    alpha = _mix_f(nt, thick, 0.30, 0.68)
    diff = _n(nt, "ShaderNodeBsdfDiffuse", ins={"Color": (0.34, 0.29, 0.24)})
    emit = _n(nt, "ShaderNodeEmission", ins={"Color": (0.50, 0.38, 0.27), "Strength": 0.35})
    add = _n(nt, "ShaderNodeAddShader")
    nt.links.new(diff.outputs[0], add.inputs[0])
    nt.links.new(emit.outputs[0], add.inputs[1])
    transp = _n(nt, "ShaderNodeBsdfTransparent", ins={"Color": (1.0, 1.0, 1.0)})
    mix = _n(nt, "ShaderNodeMixShader")
    nt.links.new(alpha, mix.inputs[0])
    nt.links.new(transp.outputs[0], mix.inputs[1])
    nt.links.new(add.outputs[0], mix.inputs[2])
    return mix


@_reg("brake_fluid", viewport=(0.85, 0.55, 0.15, 0.5), roughness=0.05, tint=(1.0, 0.65, 0.2))
def _brake_fluid(nt):
    # clear amber DOT-4 fluid: tinted transparency + Fresnel reflection + a
    # little emission so it reads inside a dark cutaway line
    fres = _n(nt, "ShaderNodeFresnel", ins={"IOR": 1.45}).outputs[0]
    transp = _n(nt, "ShaderNodeBsdfTransparent", ins={"Color": (0.95, 0.60, 0.16)})
    emit = _n(nt, "ShaderNodeEmission", ins={"Color": (1.0, 0.55, 0.12), "Strength": 0.35})
    body = _n(nt, "ShaderNodeAddShader")
    nt.links.new(transp.outputs[0], body.inputs[0])
    nt.links.new(emit.outputs[0], body.inputs[1])
    glossy = _n(nt, "ShaderNodeBsdfGlossy", ins={"Color": (1.0, 1.0, 1.0), "Roughness": 0.03})
    mix = _n(nt, "ShaderNodeMixShader")
    nt.links.new(fres, mix.inputs[0])
    nt.links.new(body.outputs[0], mix.inputs[1])
    nt.links.new(glossy.outputs[0], mix.inputs[2])
    return mix


# --- studio surfaces ---------------------------------------------------------

def _studio_surface(nt, col_floor, col_top, floor_rough, wall_rough, spec):
    """Seamless cyclorama surface: one graph for floor and wall.  Gloss on
    up-facing (floor) faces fades into a matte wall; colour gets a gentle
    vertical gradient toward the top of the object (Generated Z = 0..1).
    Low-frequency mottling breaks up the CG-perfect look."""
    geo = _n(nt, "ShaderNodeNewGeometry")
    nz = _n(nt, "ShaderNodeSeparateXYZ", ins={"Vector": geo.outputs["Normal"]}).outputs["Z"]
    floorness = _maprange(nt, nz, 0.75, 0.98, interp="SMOOTHSTEP")
    tc = _n(nt, "ShaderNodeTexCoord")
    # lighting.setup_studio builds the cyclorama at unit radius and scales the
    # object, so object Z is normalised (0 = floor, 1 = one radius up) and the
    # floor (z = 0) matches the bottom of the wall exactly.
    gz = _n(nt, "ShaderNodeSeparateXYZ", ins={"Vector": tc.outputs["Object"]}).outputs["Z"]
    grad = _maprange(nt, gz, 0.0, 1.0, interp="SMOOTHSTEP")
    col = _mix_c(nt, grad, col_floor, col_top)
    mott = _maprange(nt, _noise(nt, tc.outputs["Object"], 3.0, detail=3.0, rough=0.55), 0.3, 0.7)
    col = _mix_c(nt, _math(nt, "MULTIPLY", mott, 0.12), col, _mix_c(nt, 0.5, col, (0.0, 0.0, 0.0)))
    rough = _mix_f(nt, floorness, wall_rough, floor_rough)
    rough = _math(nt, "MULTIPLY_ADD", mott, 0.12, rough)
    sp = _mix_f(nt, floorness, spec * 0.4, spec)
    return _principled(nt, base=col, rough=rough, spec=sp)


@_reg("floor", viewport=(0.035, 0.035, 0.038, 1), roughness=0.4)
def _floor(nt):
    return _studio_surface(nt, (0.030, 0.030, 0.032), (0.011, 0.011, 0.012), 0.38, 0.75, 0.5)


@_reg("backdrop", viewport=(0.03, 0.03, 0.033, 1), roughness=0.75)
def _backdrop(nt):
    return _studio_surface(nt, (0.030, 0.030, 0.032), (0.011, 0.011, 0.012), 0.38, 0.75, 0.5)


@_reg("floor_light", viewport=(0.62, 0.62, 0.63, 1), roughness=0.5)
def _floor_light(nt):
    return _studio_surface(nt, (0.60, 0.60, 0.605), (0.52, 0.52, 0.53), 0.45, 0.8, 0.35)


@_reg("backdrop_light", viewport=(0.62, 0.62, 0.63, 1), roughness=0.8)
def _backdrop_light(nt):
    return _studio_surface(nt, (0.60, 0.60, 0.605), (0.52, 0.52, 0.53), 0.45, 0.8, 0.35)


@_reg("road_marking", viewport=(0.85, 0.85, 0.83, 1), roughness=0.6)
def _road_marking(nt):
    return _principled(nt, base=(0.70, 0.70, 0.68), rough=0.6, spec=0.4)


ROAD_HAZE = (0.62, 0.66, 0.70)     # must match lighting's road sky horizon colour


@_reg("road", viewport=(0.09, 0.09, 0.095, 1), roughness=0.85)
def _road(nt):
    """World-space road (object must sit at the origin, unrotated, unscaled).

    Two 3.5 m asphalt lanes centred on x = 0 and x = -3.5 (a LHD car keeps
    right), dashed 3 m / 6 m centre line at x = -1.75, solid edge lines at
    x = +1.75 and x = -5.25, tarred transverse joints every 15 m, concrete
    slab apron with 5 m joints outside.  Fades to haze with view distance."""
    tc = _n(nt, "ShaderNodeTexCoord")
    co = tc.outputs["Object"]
    sx = _n(nt, "ShaderNodeSeparateXYZ", ins={"Vector": co})
    x, y = sx.outputs["X"], sx.outputs["Y"]

    def band(center, half_w, soft=0.01):
        d = _math(nt, "ABSOLUTE", _math(nt, "SUBTRACT", x, center))
        return _maprange(nt, d, half_w + soft, half_w - soft)

    def periodic(v, period, on_from, on_to, soft=0.02):
        ph = _math(nt, "WRAP", v, period, 0.0)
        a = _maprange(nt, ph, on_from - soft, on_from + soft)
        b = _maprange(nt, ph, on_to + soft, on_to - soft)
        return _math(nt, "MINIMUM", a, b)

    grain = _noise(nt, co, 160.0, detail=2.0, rough=0.7)            # aggregate speckle
    patches = _maprange(nt, _noise(nt, co, 0.08, detail=3.0, rough=0.6), 0.3, 0.7)
    asphalt = _mix_c(nt, grain, (0.045, 0.045, 0.047), (0.11, 0.108, 0.105))
    asphalt = _mix_c(nt, _math(nt, "MULTIPLY", patches, 0.6), asphalt,
                     _mix_c(nt, 0.5, asphalt, (0.0, 0.0, 0.0)))
    # polished wheel tracks are a touch lighter/smoother
    tracks = _math(nt, "MAXIMUM", _math(nt, "MAXIMUM", band(-0.75, 0.3, 0.4), band(0.75, 0.3, 0.4)),
                   _math(nt, "MAXIMUM", band(-4.25, 0.3, 0.4), band(-2.75, 0.3, 0.4)))
    asphalt = _mix_c(nt, _math(nt, "MULTIPLY", tracks, 0.25), asphalt, (0.11, 0.11, 0.11))
    joints = periodic(y, 15.0, 0.01, 0.04, 0.01)
    asphalt = _mix_c(nt, _math(nt, "MULTIPLY", joints, 0.85), asphalt, (0.012, 0.012, 0.013))
    # concrete apron outside the carriageway, 5 m slabs
    slab_j = _math(nt, "MAXIMUM", periodic(x, 5.0, 0.006, 0.026, 0.006), periodic(y, 5.0, 0.006, 0.026, 0.006))
    concrete = _mix_c(nt, grain, (0.20, 0.195, 0.185), (0.30, 0.295, 0.285))
    concrete = _mix_c(nt, _math(nt, "MULTIPLY", patches, 0.5), concrete, (0.16, 0.155, 0.15))
    concrete = _mix_c(nt, _math(nt, "MULTIPLY", slab_j, 0.9), concrete, (0.03, 0.03, 0.03))
    on_road = band(-1.75, 3.9, 0.03)            # x in [-5.65, 2.15]
    col = _mix_c(nt, on_road, concrete, asphalt)
    # markings (thermoplastic white, slightly worn)
    centre = _math(nt, "MINIMUM", band(-1.75, 0.06), periodic(y, 9.0, 0.02, 3.0))
    edges = _math(nt, "MAXIMUM", band(1.75, 0.075), band(-5.25, 0.075))
    paint = _math(nt, "MAXIMUM", centre, edges)
    wear = _maprange(nt, grain, 0.2, 0.5, 0.75, 1.0)
    paint = _math(nt, "MULTIPLY", paint, wear)
    col = _mix_c(nt, paint, col, (0.70, 0.70, 0.68))
    rough = _mix_f(nt, paint, _mix_f(nt, on_road, 0.9, _mix_f(nt, tracks, 0.86, 0.72)), 0.6)
    normal = _bump(nt, grain, 0.35, 0.002)
    bsdf = _principled(nt, base=col, rough=rough, spec=0.4, normal=normal)
    # distance haze toward the horizon colour
    cam = _n(nt, "ShaderNodeCameraData")
    haze = _maprange(nt, cam.outputs["View Distance"], 60.0, 380.0, 0.0, 0.92, interp="SMOOTHSTEP")
    emit = _n(nt, "ShaderNodeEmission", ins={"Color": ROAD_HAZE, "Strength": 1.0})
    mix = _n(nt, "ShaderNodeMixShader")
    nt.links.new(haze, mix.inputs[0])
    nt.links.new(_out(bsdf), mix.inputs[1])
    nt.links.new(emit.outputs[0], mix.inputs[2])
    return mix


if __name__ == "__main__":
    for nm in all_names(True):
        mm = get(nm)
        print(f"{nm:20s} nodes={len(mm.node_tree.nodes):3d} viewport={tuple(round(c, 2) for c in mm.diffuse_color)}")
