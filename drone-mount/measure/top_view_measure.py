"""Reproduces the fuselage measurements from ref/fcc_top_view.png (FCC external photo 1).

1. finds the inner edges of the L-shaped steel ruler,
2. fits the table-plane homography from centimetre ticks read off the photo,
3. uses the four folded propellers (267 mm diameter when unfolded) to get, for the front
   and the rear of the aircraft, how much smaller things at that height really are than
   their projection on the table (k = 1 - height / camera height),
4. prints the fuselage dimensions.

Needs numpy and Pillow.  Run:  python top_view_measure.py
"""
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import cam
import rulers as R

IMG = pathlib.Path(__file__).parent.parent / "ref" / "fcc_top_view.png"

# centimetre mark -> distance in pixels from the ruler corner along the ruler edge (read from enlarged strips)
BOTTOM = {0: -4.7, 1: 15.7, 2: 35.8, 3: 56.7, 4: 76.7, 5: 96.3, 6: 116.7, 14: 275.8, 15: 294.7, 16: 314.2, 17: 333.3, 18: 352.5,
          19: 371.7, 20: 390.8, 28: 540.8, 29: 558.8, 30: 577.0, 31: 595.8, 32: 613.3, 33: 631.7, 34: 649.7, 35: 667.8, 42: 791.3,
          43: 808.7, 44: 825.8, 45: 843.7, 46: 860.8, 47: 878.3, 48: 895.3, 49: 913.0, 50: 930.0}
LEFT = {0: -2.5, 1: 16.7, 2: 38.3, 3: 58.3, 4: 79.2, 5: 99.7, 6: 119.7, 12: 234.2, 13: 252.5, 14: 271.7, 15: 290.0, 16: 308.0,
        17: 325.8, 18: 343.7, 19: 361.3, 26: 479.2, 27: 495.5, 28: 512.2, 29: 528.0, 30: 543.3, 31: 559.2, 32: 575.0, 33: 590.0,
        34: 605.5, 39: 680.3, 40: 695.0, 41: 709.2, 42: 723.3, 43: 737.5, 44: 751.7, 45: 765.8, 46: 780.0, 47: 794.2, 48: 808.0}
# propellers: the two blade pivots, and one blade from its pivot to its tip (pixels)
PROPS = {"front-left":  (((237, 185), (277, 187)), ((277, 187), (278, 412))),
         "front-right": (((921, 224), (956, 224)), ((956, 224), (940, 443))),
         "rear-left":   (((245, 727), (292, 719)), ((292, 719), (277, 468))),
         "rear-right":  (((913, 724), (953.5, 725)), ((953.5, 725), (924, 489)))}
HUBS = dict(fl=(257.5, 186), fr=(938, 224.5), rl=(267, 723), rr=(933.5, 724.5))
CAMERA = (612.5, 401)          # centre top camera

a = R.load(IMG)
(cb, db), _ = R.bottom_edge(a, (240, 1180), (790, 860))
(cl, dl), _ = R.left_edge(a, (150, 300), (20, 800))
db = db if db[0] > 0 else -db
dl = dl if dl[1] < 0 else -dl
corner = cb + np.linalg.solve(np.array([db, -dl]).T, cl - cb)[0] * db
src = [(10.0 * c, 0.0) for c in BOTTOM] + [(0.0, 10.0 * c) for c in LEFT]
dst = [corner + s * db for s in BOTTOM.values()] + [corner + s * dl for s in LEFT.values()]
H = cam.homography(np.array(src), np.array(dst))
err = np.linalg.norm(cam.apply(H, np.array(src)) - np.array(dst), axis=1)
print(f"table homography from {len(src)} ruler ticks: reprojection error mean {err.mean():.2f} px, max {err.max():.2f} px")
Hi = np.linalg.inv(H)
seg = lambda p, q: float(np.linalg.norm(np.diff(cam.apply(Hi, np.array([p, q], float)), axis=0)))

K = {}
for name, (piv, blade) in PROPS.items():
    d, l = seg(*piv), seg(*blade)
    K[name] = 267.0 / (d + 2 * l)
    print(f"  {name:11s}: k = {K[name]:.3f}   pivot spacing {K[name] * d:5.1f} mm, blade {K[name] * l:6.1f} mm")
kf, kr = (K["front-left"] + K["front-right"]) / 2, (K["rear-left"] + K["rear-right"]) / 2
km = (kf + kr) / 2
print(f"front motor span {kf * seg(HUBS['fl'], HUBS['fr']):.0f} mm, rear {kr * seg(HUBS['rl'], HUBS['rr']):.0f} mm, "
      f"diagonals {km * seg(HUBS['fl'], HUBS['rr']):.0f} / {km * seg(HUBS['fr'], HUBS['rl']):.0f} mm")

print("\nfuselage width (k between 0.89 and 0.91 for the body sides):")
for name, p, q in (("front shoulders", (533, 270), (690, 272)), ("waist, at the centre camera", (542, 400), (674.5, 401)),
                   ("waist, 16 mm behind it", (541, 430), (671.5, 431)), ("waist, 26 mm behind it", (540, 450), (670, 451)),
                   ("rear-arm fairings", (520, 500), (696, 500)), ("exhaust grille", (562, 544), (659, 545))):
    w = seg(p, q)
    print(f"  {name:28s} {0.89 * w:5.1f} .. {0.91 * w:5.1f} mm")
print("\nalong the roof, from the centre top camera (k 0.88 .. 0.90):")
for name, p in (("front cameras", (612.5, 273.5)), ("front edge of the shell", (612.5, 214)), ("rear-arm fairings start", (612.5, 465)),
                ("exhaust housing starts", (612.5, 525)), ("exhaust grille starts", (612.5, 536)), ("rear end", (612.5, 590))):
    L = seg(CAMERA, p)
    print(f"  {name:28s} {0.88 * L:5.1f} .. {0.90 * L:5.1f} mm")
