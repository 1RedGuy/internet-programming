"""Find the inner edges of the L-shaped steel ruler and the cm ticks along them."""
import numpy as np
from PIL import Image

def load(path):
    return np.asarray(Image.open(path).convert("RGB")).astype(float)

def blue_mask(a):
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    return (b > 110) & (b > r * 1.6) & (b > g * 1.35)

def fit_line(pts):
    """Robust total-least-squares line through 2D points. Returns (point, unit direction)."""
    pts = np.asarray(pts, float)
    for _ in range(4):
        c = pts.mean(0); u, s, vt = np.linalg.svd(pts - c); d = vt[0]
        n = np.array([-d[1], d[0]]); res = (pts - c) @ n
        keep = np.abs(res) < max(1.0, 2.0 * np.std(res))
        if keep.all(): break
        pts = pts[keep]
    return c, d

def bilinear(g, x, y):
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    fx, fy = x - x0, y - y0
    x0 = np.clip(x0, 0, g.shape[1] - 2); y0 = np.clip(y0, 0, g.shape[0] - 2)
    return (g[y0, x0] * (1 - fx) * (1 - fy) + g[y0, x0 + 1] * fx * (1 - fy) + g[y0 + 1, x0] * (1 - fx) * fy + g[y0 + 1, x0 + 1] * fx * fy)

def bottom_edge(a, xr, yr):
    """Top edge of the bottom ruler: last blue pixel going down, per column."""
    m = blue_mask(a); pts = []
    for x in range(xr[0], xr[1], 3):
        col = m[yr[0]:yr[1], x]
        idx = np.where(col)[0]
        if len(idx) > 5:
            pts.append((x, yr[0] + idx.max() + 0.5))
    return fit_line(pts), np.array(pts)

def left_edge(a, xr, yr):
    """Right edge of the left ruler: first blue pixel going right, per row."""
    m = blue_mask(a); pts = []
    for y in range(yr[0], yr[1], 3):
        row = m[y, xr[0]:xr[1]]
        idx = np.where(row)[0]
        if len(idx) > 5:
            pts.append((xr[0] + idx.min() - 0.5, y))
    return fit_line(pts), np.array(pts)

def profile(a, c, d, inward, s0, s1, offs, step=0.25):
    """Mean grey level along the edge line, averaged over several inward offsets."""
    g = a.mean(axis=2)
    s = np.arange(s0, s1, step)
    n = np.array(inward, float)
    acc = np.zeros_like(s)
    for o in offs:
        p = c[None, :] + s[:, None] * d[None, :] + o * n[None, :]
        acc += bilinear(g, p[:, 0], p[:, 1])
    return s, acc / len(offs)
