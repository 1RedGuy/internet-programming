"""Preview pictures of the payload box."""
import pathlib

from PIL import ImageDraw

import mavic4pro_mount as m
import payload_box as pb
import render_preview as rp

d, parts = m.build()
b = pb.B
out = pathlib.Path(__file__).parent / "out"
bx, ld = pb.box(d, b), pb.lid(d, b)
bat, brd = pb.contents(d, b)
zb, zf0, zf1, zt = pb.levels(d, b)
TEAL, ORANGE, DARK, GREY, BLUE, GREEN = (40, 130, 140), (240, 120, 30), (70, 72, 80), (200, 202, 208), (90, 140, 220), (60, 160, 90)
mb = rp.mesh(bx, 0.05)

# 1) underside: how it meets the saddle and what touches the print bed
im, px = rp.render([(*mb, TEAL)], eye_dir=(-0.7, -0.6, -0.9), up=(0, 0, 1), size=(1400, 900))
rp.label(im, px, (0, 13, zb), "dovetail rails", (120, -150))
rp.label(im, px, (-28, 23.5, zb), "rear foot, 2 open slots for the lock screws", (60, 200))
rp.label(im, px, (-10, -36, zb), "side pocket", (-230, -40))
ImageDraw.Draw(im).text((30, 22), "Box seen from below - everything flat here sits on the print bed", fill=(20, 20, 20), font=rp.font(26))
im.save(out / "preview_box_underside.png")

# 2) open, with the planned contents
im, px = rp.render([(*mb, TEAL), (*rp.mesh(bat, 0.2), BLUE), (*rp.mesh(brd, 0.2), GREEN)], eye_dir=(-0.5, -0.8, 1.1), size=(1400, 950))
rp.label(im, px, (10, -13, zf1 + 8), "LiPo 50 x 34 x 8", (150, 190))
rp.label(im, px, (-14, 17, zf1 + 18), "2 stacked boards 25 x 18 x 18", (-420, -150))
rp.label(im, px, (12, 22, zf1), "free floor 23 x 34 mm", (170, -60))
ImageDraw.Draw(im).text((30, 22), "Box without the lid (nose of the drone to the upper right)", fill=(20, 20, 20), font=rp.font(26))
im.save(out / "preview_box_open.png")

# 3) on the drone
body = m.body_proxy(d)
cams, vents = m.body_marks(d)
items = [(*rp.mesh(body, 0.15), GREY), (*rp.mesh(cams, 0.1), (25, 25, 28)), (*rp.mesh(vents, 0.1), (235, 235, 230)),
         (*rp.mesh(parts["yoke"]), DARK), (*rp.mesh(parts["saddle"]), ORANGE), (*mb, TEAL), (*rp.mesh(ld, 0.1), (30, 105, 115)),
         (*rp.mesh(m.screws(d), 0.05), (175, 180, 190))]
im, px = rp.render(items, eye_dir=(-0.75, -0.9, 0.62))
rp.label(im, px, (-5, -42, 30), "payload box with lid", (60, 250))
rp.label(im, px, (-30.6, -23.5, 6.5), "lock screw (one of two)", (-60, 330))
im.save(out / "preview_box_on_drone.png")
print("ok")
