"""Camera model from the L-ruler on the table (plane Z=0, X along bottom ruler, Y along left ruler, mm)."""
import numpy as np

def homography(src, dst):
    """DLT with normalisation. src (N,2) plane mm -> dst (N,2) pixels."""
    def norm(p):
        c = p.mean(0); s = np.sqrt(2) / np.mean(np.linalg.norm(p - c, axis=1))
        T = np.array([[s, 0, -s * c[0]], [0, s, -s * c[1]], [0, 0, 1]]); return T
    Ts, Td = norm(src), norm(dst)
    a = (Ts @ np.c_[src, np.ones(len(src))].T).T; b = (Td @ np.c_[dst, np.ones(len(dst))].T).T
    A = []
    for (x, y, _), (u, v, _) in zip(a, b):
        A.append([-x, -y, -1, 0, 0, 0, u * x, u * y, u]); A.append([0, 0, 0, -x, -y, -1, v * x, v * y, v])
    _, _, vt = np.linalg.svd(np.array(A)); Hn = vt[-1].reshape(3, 3)
    H = np.linalg.inv(Td) @ Hn @ Ts
    return H / H[2, 2]

def apply(H, p):
    q = (H @ np.c_[p, np.ones(len(p))].T).T
    return q[:, :2] / q[:, 2:3]

def decompose(H, size):
    """Assume square pixels and principal point at the image centre. Returns f, R, t, camera centre."""
    cx, cy = size[0] / 2, size[1] / 2
    T = np.array([[1, 0, -cx], [0, 1, -cy], [0, 0, 1.0]])
    Hc = T @ H
    h1, h2, h3 = Hc[:, 0], Hc[:, 1], Hc[:, 2]
    f2a = -(h1[0] * h2[0] + h1[1] * h2[1]) / (h1[2] * h2[2])
    f2b = (h2[0] ** 2 + h2[1] ** 2 - h1[0] ** 2 - h1[1] ** 2) / (h1[2] ** 2 - h2[2] ** 2)
    cands = [v for v in (f2a, f2b) if v > 0]
    f = float(np.sqrt(np.mean(cands)))
    Ki = np.diag([1 / f, 1 / f, 1.0])
    r1, r2, t = Ki @ h1, Ki @ h2, Ki @ h3
    lam = (np.linalg.norm(r1) + np.linalg.norm(r2)) / 2
    r1, r2, t = r1 / lam, r2 / lam, t / lam
    if t[2] < 0: r1, r2, t = -r1, -r2, -t
    R = np.c_[r1, r2, np.cross(r1, r2)]
    u, _, vt = np.linalg.svd(R); R = u @ vt
    C = -R.T @ t
    return dict(f=f, f_a=np.sqrt(f2a) if f2a > 0 else np.nan, f_b=np.sqrt(f2b) if f2b > 0 else np.nan, R=R, t=t, C=C, cx=cx, cy=cy)

def project(cam, P):
    P = np.atleast_2d(P); q = (cam["R"] @ P.T).T + cam["t"]
    return np.c_[cam["f"] * q[:, 0] / q[:, 2] + cam["cx"], cam["f"] * q[:, 1] / q[:, 2] + cam["cy"]]

def ray_to_plane_z(cam, px, z):
    """Back-project pixel(s) to the horizontal plane Z=z (mm above the table)."""
    px = np.atleast_2d(px)
    d = np.c_[(px[:, 0] - cam["cx"]) / cam["f"], (px[:, 1] - cam["cy"]) / cam["f"], np.ones(len(px))]
    dw = (cam["R"].T @ d.T).T; C = cam["C"]
    s = (z - C[2]) / dw[:, 2]
    return C[None, :] + s[:, None] * dw

def ray_to_plane(cam, px, p0, n):
    """Back-project pixel(s) to an arbitrary plane through p0 with normal n."""
    px = np.atleast_2d(px)
    d = np.c_[(px[:, 0] - cam["cx"]) / cam["f"], (px[:, 1] - cam["cy"]) / cam["f"], np.ones(len(px))]
    dw = (cam["R"].T @ d.T).T; C = cam["C"]
    s = ((np.asarray(p0) - C) @ n) / (dw @ n)
    return C[None, :] + s[:, None] * dw

def from_ticks(edges_file, bottom, left, size):
    """bottom/left: dict {cm: s_pixels_along_edge}. Returns H, camera."""
    cb, db, cl, dl, corner = np.load(edges_file)
    src, dst = [], []
    for cm, s in bottom.items(): src.append((10.0 * cm, 0.0)); dst.append(corner + s * db)
    for cm, s in left.items():   src.append((0.0, 10.0 * cm)); dst.append(corner + s * dl)
    src, dst = np.array(src), np.array(dst)
    H = homography(src, dst)
    err = np.linalg.norm(apply(H, src) - dst, axis=1)
    return H, decompose(H, size), err
