"""Scene scaffolding shared by every scenes/sNN_*.py module.

A scene module defines

    SCENE_ID = "s02"
    def build(quality: str) -> SceneBuild

and uses `new_scene()` to reset Blender and set the frame range / fps.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import bpy

from . import spec as S
from . import timeline


@dataclass
class SceneBuild:
    scene_id: str
    track: object                 # carviz.state.Track
    camera: object                # bpy camera object
    labels: object                # carviz.labels.Labels
    hud: object                   # carviz.labels.Hud
    motion_blur: bool = False     # Cycles motion blur in final renders
    shutter: float = 0.5          # fraction of a frame
    preview_hide: tuple = ()      # objects hidden in Workbench previews (e.g. glass)
    extra: dict = field(default_factory=dict)


def new_scene(scene_id):
    """Fresh empty Blender scene with this scene's frame range and fps."""
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.name = scene_id
    sc.render.fps = S.FPS
    sc.render.fps_base = 1.0
    sc.frame_start = 1
    sc.frame_end = timeline.scene(scene_id).frames
    sc.frame_set(1)
    try:  # AgX + look used by every Cycles render (Workbench previews switch to Standard)
        from . import lighting
        lighting.setup_color_management(sc)
    except Exception as e:  # pragma: no cover
        print("[scenebase] colour management not applied:", e)
    return sc


def scene_frames(scene_id):
    return range(1, timeline.scene(scene_id).frames + 1)
