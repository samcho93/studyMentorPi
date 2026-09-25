"""Virtual RGB-D camera (MentorPi HP60C / ascamera) rendered from the 2D world extruded to 3D.

Geometry: pinhole camera at the depth_cam joint (x = 0.061 m ahead of base_link, z = 0.121 m
above the floor), looking straight ahead. Intrinsics are the MentorPi camera_info.yaml values
(640x480, fx 473.45, fy 474.22) scaled to the simulated resolution.

Scene heights: outer walls / walls 0.30 m, boxes 0.25 m, cylinders 0.30 m, labelled objects
2r (ball-like), movers 0.30 m, walkers (people) 1.60 m. Floor: light tiles with a 0.5 m grid,
plus the black track line in the "track" world.

render(rt, w, h) -> (rgb uint8 [h,w,3], depth float32 [h,w] metres, 0 = invalid)
"""
import math

import numpy as np

CAM_X, CAM_Z = 0.061376, 0.07 + 0.051154          # base_footprint -> depth_cam
FX640, FY640, CX640, CY640 = 473.4506985179141, 474.2169451085363, 323.5512181265506, 238.6016133237558
MIN_RANGE, MAX_RANGE = 0.15, 4.0                    # HP60C-like working range (model values)

WALL_H, BOX_H, CYL_H, MOVER_H, WALKER_H = 0.30, 0.25, 0.30, 0.30, 1.60
C_WALL = np.array([203, 213, 225]); C_BOX = np.array([139, 94, 52]); C_CYL = np.array([100, 116, 139])
C_MOVER = np.array([123, 79, 209]); C_FLOOR_A = np.array([233, 228, 218]); C_FLOOR_B = np.array([222, 216, 205])
C_LINE = np.array([20, 20, 20])


def intrinsics(w, h):
    sx, sy = w / 640.0, h / 480.0
    return FX640 * sx, FY640 * sy, CX640 * sx, CY640 * sy


def label_color(label):
    l = str(label).lower()
    for keys, c in ((("red", "빨강", "빨간"), (220, 38, 38)), (("blue", "파랑", "파란"), (37, 99, 235)),
                    (("green", "초록"), (22, 163, 74)), (("yellow", "노랑", "노란"), (202, 138, 4)),
                    (("chair", "의자", "table", "책상", "sofa"), (139, 94, 52))):
        if any(k in l for k in keys):
            return np.array(c)
    return np.array([8, 145, 178])


def _track_points(t):
    pts, h, r = [], t["half"], t["r"]
    x = -h
    while x <= h + 1e-9:
        pts.append((x, -r)); x += 0.05
    a = -math.pi / 2
    while a <= math.pi / 2 + 1e-9:
        pts.append((h + r * math.cos(a), r * math.sin(a))); a += math.pi / 40
    x = h
    while x >= -h - 1e-9:
        pts.append((x, r - t["wiggle"] * math.sin(math.pi * (x + h) / (2 * h)) ** 2)); x -= 0.05
    a = math.pi / 2
    while a <= 3 * math.pi / 2 + 1e-9:
        pts.append((-h + r * math.cos(a), r * math.sin(a))); a += math.pi / 40
    return np.array(pts)


def _scene(world):
    """Vertical faces: segments (x1,y1,x2,y2,height,color) and circles (x,y,r,height,color)."""
    segs, scol, sh = [], [], []
    x0, y0, x1, y1 = world.bounds
    for s in ([x0, y0, x1, y0], [x1, y0, x1, y1], [x1, y1, x0, y1], [x0, y1, x0, y0], *world.walls):
        segs.append(s); scol.append(C_WALL); sh.append(WALL_H)
    for cx, cy, w, h in world.boxes:
        a, b, c, d = cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2
        for s in ([a, b, c, b], [c, b, c, d], [c, d, a, d], [a, d, a, b]):
            segs.append(s); scol.append(C_BOX); sh.append(BOX_H)
    circ, ccol, ch = [], [], []
    objs = {(round(o[1], 6), round(o[2], 6)): o for o in getattr(world, "objects", [])}
    for cx, cy, r in world.cylinders:
        o = objs.get((round(cx, 6), round(cy, 6)))
        circ.append([cx, cy, r])
        ccol.append(label_color(o[0]) if o else C_CYL)
        ch.append(2 * r if o else CYL_H)
    for m in world.movers:
        circ.append([m[0], m[1], m[4]])
        ccol.append(C_MOVER)
        ch.append(WALKER_H if len(m) > 5 else MOVER_H)
    return (np.array(segs, float).reshape(-1, 4), np.array(scol, float).reshape(-1, 3), np.array(sh, float),
            np.array(circ, float).reshape(-1, 3), np.array(ccol, float).reshape(-1, 3), np.array(ch, float))


def render(world, pose, w=160, h=120, noise=True, rng=None):
    fx, fy, cx, cy = intrinsics(w, h)
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    ox, oy = x + CAM_X * c, y + CAM_X * s                      # camera centre (world)
    u = np.arange(w) + 0.5
    v = np.arange(h) + 0.5
    dx = (u - cx) / fx                                         # optical x (right) per unit depth
    dy = (v - cy) / fy                                         # optical y (down) per unit depth
    # horizontal ray per column for unit forward depth: forward - dx * left
    wx = c + dx * s
    wy = s - dx * c
    segs, scol, sh, circ, ccol, ch = _scene(world)
    T = []   # candidate depths per column: arrays [W, K]
    H_ = []
    COL = []
    if len(segs):
        ex = (segs[:, 2] - segs[:, 0])[None, :]
        ey = (segs[:, 3] - segs[:, 1])[None, :]
        qx = (segs[:, 0] - ox)[None, :]
        qy = (segs[:, 1] - oy)[None, :]
        den = wx[:, None] * ey - wy[:, None] * ex
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (qx * ey - qy * ex) / den
            uu = (qx * wy[:, None] - qy * wx[:, None]) / den
        t = np.where((np.abs(den) > 1e-12) & (t > 1e-6) & (uu >= 0) & (uu <= 1), t, np.inf)
        T.append(t); H_.append(np.broadcast_to(sh[None, :], t.shape)); COL.append(np.broadcast_to(np.arange(len(segs))[None, :], t.shape))
    if len(circ):
        fxv = ox - circ[:, 0]
        fyv = oy - circ[:, 1]
        a = (wx * wx + wy * wy)[:, None]
        b = (wx[:, None] * fxv[None, :] + wy[:, None] * fyv[None, :])
        cc = (fxv * fxv + fyv * fyv - circ[:, 2] ** 2)[None, :]
        disc = b * b - a * cc
        with np.errstate(invalid="ignore"):
            t = (-b - np.sqrt(disc)) / a
        t = np.where((disc >= 0) & (t > 1e-6), t, np.inf)
        T.append(t); H_.append(np.broadcast_to(ch[None, :], t.shape)); COL.append(np.broadcast_to(len(segs) + np.arange(len(circ))[None, :], t.shape))
    colors = np.vstack([scol, ccol]) if (len(scol) or len(ccol)) else np.zeros((1, 3))
    T = np.concatenate(T, axis=1) if T else np.full((w, 1), np.inf)
    HH = np.concatenate(H_, axis=1) if H_ else np.zeros((w, 1))
    CI = np.concatenate(COL, axis=1) if COL else np.zeros((w, 1), int)
    # per pixel: height of the ray at each candidate depth
    z = CAM_Z - dy[:, None, None] * T[None, :, :]            # [H, W, K]
    ok = (z >= 0) & (z <= HH[None, :, :]) & np.isfinite(T)[None, :, :]
    Tm = np.where(ok, T[None, :, :], np.inf)
    k = np.argmin(Tm, axis=2)                                  # [H, W]
    t_obj = np.take_along_axis(Tm, k[..., None], axis=2)[..., 0]
    # floor
    with np.errstate(divide="ignore"):
        t_floor = np.where(dy[:, None] > 1e-9, CAM_Z / dy[:, None], np.inf) * np.ones((1, w))
    depth = np.minimum(t_obj, t_floor)
    floor = t_floor <= t_obj
    # colours
    rgb = np.empty((h, w, 3))
    idx = CI[np.arange(w)[None, :], k]
    rgb[:] = colors[idx]
    tf_ = np.where(np.isfinite(t_floor), t_floor, 0.0)
    fxw = ox + tf_ * wx[None, :]
    fyw = oy + tf_ * wy[None, :]
    tile = ((np.floor(fxw / 0.5) + np.floor(fyw / 0.5)) % 2 == 0)
    fcol = np.where(tile[..., None], C_FLOOR_A, C_FLOOR_B).astype(float)
    grid = (np.abs(fxw / 0.5 - np.round(fxw / 0.5)) < 0.02) | (np.abs(fyw / 0.5 - np.round(fyw / 0.5)) < 0.02)
    fcol[grid] = fcol[grid] * 0.85
    tr = getattr(world, "track_spec", None)
    if tr:
        pts = getattr(world, "_track_cache", None)
        if pts is None:
            pts = world._track_cache = _track_points(tr)
        a, b = pts, np.roll(pts, -1, axis=0)
        near_floor = floor & (t_floor < 3.0)
        P = np.stack([fxw[near_floor], fyw[near_floor]], -1)[:, None, :]     # [N, 1, 2]
        e = (b - a)[None]
        L2 = (e ** 2).sum(-1)
        tt = np.clip(((P - a[None]) * e).sum(-1) / np.where(L2 > 0, L2, 1), 0, 1)
        d2 = ((a[None] + tt[..., None] * e - P) ** 2).sum(-1).min(-1)
        line = np.zeros_like(floor)
        line[near_floor] = d2 <= (tr["width"] / 2) ** 2
        fcol[line] = C_LINE
    rgb = np.where(floor[..., None], fcol, rgb)
    shade = np.clip(1.0 - 0.06 * np.where(np.isfinite(depth), depth, 8.0), 0.55, 1.0)
    rgb = np.clip(rgb * shade[..., None], 0, 255)
    sky = ~np.isfinite(depth)
    rgb[sky] = (60, 64, 72)
    # sensor model: range limits, depth noise ~ d^2, 1 % dropouts
    d = np.where(np.isfinite(depth), depth, 0.0)
    if noise:
        g = rng if rng is not None else np.random.default_rng(0)
        d = d + g.normal(0, 1, d.shape) * (0.001 + 0.0025 * d * d)
        d[g.random(d.shape) < 0.01] = 0.0
    d[(d < MIN_RANGE) | (d > MAX_RANGE)] = 0.0
    return rgb.astype(np.uint8), d.astype(np.float32)


def backproject(depth, w=None, h=None, step=1):
    """Depth image (m) -> Nx3 points in the camera optical frame (x right, y down, z forward)."""
    h, w = depth.shape
    fx, fy, cx, cy = intrinsics(w, h)
    vv, uu = np.mgrid[0:h:step, 0:w:step]
    z = depth[::step, ::step]
    m = z > 0
    x = (uu[m] + 0.5 - cx) / fx * z[m]
    y = (vv[m] + 0.5 - cy) / fy * z[m]
    return np.stack([x, y, z[m]], axis=1).astype(np.float32)
