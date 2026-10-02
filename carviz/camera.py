"""Code-driven cameras.

    cam = CameraPath()
    cam.key(0.0, eye=(3, -2, 1.2), target=(0, -1, 0.5), lens=35, fstop=4)
    cam.key(6.0, eye=(2, -1, 1.0), target=(0, -1, 0.45), mode='ease')
    cam.orbit(6.0, 14.0, center=(0, -1, .45), radius=1.6, height=0.9, ang0=-30, ang1=40)
    cam.bake(scene, track.frames, fps)      # creates/updates the camera object

Positions are interpolated component-wise with carviz.state.Curve
('ease' = smooth start/stop, 'cubic' = monotone C1 through keys, 'linear').
Orbit samples the arc densely so the path is a true circle.  The camera
looks at `target` with world +Z up (optional roll).  Depth of field focuses
on the target (or an explicit focus point) every frame.

If `parent` is given (e.g. a moving car root Empty), eye/target are in that
object's local coordinates and the camera rides along with it.
"""
from __future__ import annotations

import math

import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector

from .rig import bake_channel, link
from .state import Curve


class CameraPath:
    def __init__(self, name="Camera", lens=35.0, fstop=5.6, sensor=36.0, clip=(0.02, 400.0)):
        self.name = name
        self.sensor = sensor
        self.clip = clip
        self.ex, self.ey, self.ez = Curve(), Curve(), Curve()
        self.tx, self.ty, self.tz = Curve(), Curve(), Curve()
        self.lens = Curve(lens)
        self.fstop = Curve(fstop)
        self.roll = Curve(0.0)
        self.focus_offset = Curve(0.0)    # metres added to the target distance
        self._has = False

    # ------------------------------------------------------------------
    def key(self, t, eye=None, target=None, lens=None, fstop=None, roll=None, mode="ease"):
        if eye is not None:
            for c, v in zip((self.ex, self.ey, self.ez), eye):
                c.key(t, v, mode)
        if target is not None:
            for c, v in zip((self.tx, self.ty, self.tz), target):
                c.key(t, v, mode)
        if lens is not None:
            self.lens.key(t, lens, mode)
        if fstop is not None:
            self.fstop.key(t, fstop, mode)
        if roll is not None:
            self.roll.key(t, math.radians(roll), mode)
        self._has = True
        return self

    def hold(self, t0, t1):
        """Hold the pose at t0 until t1 (no motion)."""
        e = [float(c(np.array([t0]))[0]) for c in (self.ex, self.ey, self.ez)]
        g = [float(c(np.array([t0]))[0]) for c in (self.tx, self.ty, self.tz)]
        self.key(t1, eye=e, target=g, mode="linear")
        return self

    def orbit(self, t0, t1, center, radius, height, ang0, ang1, lens=None, fstop=None,
              ease=True, steps=None, target=None, radius1=None, height1=None):
        """Circular move around `center`.  Angles in degrees: ang = 0 puts the
        camera on the -Y side of the centre (behind it, looking forward along
        +Y), ang = 90 on the +X (right) side, ang = 180 in front, -90 on the
        left.  Increasing angle = counter-clockwise seen from above.
        radius1/height1 turn the orbit into a spiral push-in/pull-out."""
        steps = steps or max(8, int(abs(ang1 - ang0) / 4))
        target = target or center
        r1 = radius if radius1 is None else radius1
        h1 = height if height1 is None else height1
        for k in range(steps + 1):
            u = k / steps
            w = u * u * (3 - 2 * u) if ease else u
            a = math.radians(ang0 + (ang1 - ang0) * w)
            r = radius + (r1 - radius) * w
            hh = height + (h1 - height) * w
            t = t0 + (t1 - t0) * u
            eye = (center[0] + r * math.sin(a), center[1] - r * math.cos(a), center[2] + hh)
            self.key(t, eye=eye, target=target, mode="cubic" if 0 < k else "ease")
        if lens is not None:
            self.lens.key(t1, lens, "ease")
        if fstop is not None:
            self.fstop.key(t1, fstop, "ease")
        return self

    # ------------------------------------------------------------------
    def sample(self, t):
        t = np.asarray(t, dtype=float)
        eye = np.stack([self.ex(t), self.ey(t), self.ez(t)], 1)
        tgt = np.stack([self.tx(t), self.ty(t), self.tz(t)], 1)
        return eye, tgt, self.lens(t), self.fstop(t), self.roll(t)

    def bake(self, frames, fps, parent=None, dof=True, focus_object=None, obj=None):
        frames = np.asarray(frames)
        t = (frames - 1) / fps
        eye, tgt, lens, fstop, roll = self.sample(t)
        cam = obj or bpy.data.objects.get(self.name)
        if cam is None:
            cd = bpy.data.cameras.new(self.name)
            cam = bpy.data.objects.new(self.name, cd)
            link(cam)
        cam.data.sensor_width = self.sensor
        cam.data.clip_start, cam.data.clip_end = self.clip
        if parent is not None:
            cam.parent = parent
        cam.rotation_mode = "QUATERNION"
        quats = np.zeros((len(t), 4))
        prev = None
        for i in range(len(t)):
            d = Vector(tgt[i]) - Vector(eye[i])
            if d.length < 1e-9:
                d = Vector((0, 1, 0))
            q = d.to_track_quat("-Z", "Y")
            if abs(roll[i]) > 1e-9:
                q = q @ Quaternion((0, 0, 1), roll[i])
            q = np.array(q)
            if prev is not None and np.dot(q, prev) < 0:
                q = -q
            quats[i] = q
            prev = q
        for ax in range(3):
            bake_channel(cam, "location", ax, frames, eye[:, ax])
        for ax in range(4):
            bake_channel(cam, "rotation_quaternion", ax, frames, quats[:, ax])
        bake_channel(cam.data, "lens", -1, frames, lens)
        cam.data.dof.use_dof = dof
        if dof:
            dist = np.linalg.norm(tgt - eye, axis=1) + self.focus_offset(t)
            # dof is a nested struct: key it through the camera data path
            bake_channel(cam.data, "dof.focus_distance", -1, frames, dist)
            bake_channel(cam.data, "dof.aperture_fstop", -1, frames, fstop)
        bpy.context.scene.camera = cam
        return cam
