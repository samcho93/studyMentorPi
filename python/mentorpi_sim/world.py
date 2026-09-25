"""2D world geometry: segments + circles, numpy ray casting and collision checks."""
import json
import math
import os

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))


def load_worlds():
    with open(os.path.join(_HERE, "worlds.json"), encoding="utf-8") as f:
        data = json.load(f)
    data.pop("_comment", None)
    return data


class World:
    def __init__(self, name="room"):
        worlds = load_worlds()
        if name not in worlds:
            raise ValueError("unknown world '%s' (choose: %s)" % (name, ", ".join(worlds)))
        spec = worlds[name]
        self.name = name
        self.title = spec["title"]
        self.bounds = list(spec["bounds"])
        self.start = tuple(spec["start"])
        self.boxes = [list(b) for b in spec["boxes"]]
        self.cylinders = [list(c) for c in spec["cylinders"]]
        self.walls = [list(w) for w in spec["walls"]]
        self.movers = []            # [x, y, vx, vy, r]
        self._rebuild()

    # ---------------------------------------------------------------- edit
    def add_box(self, x, y, w, h):
        self.boxes.append([x, y, w, h])
        self._rebuild()

    def add_cylinder(self, x, y, r=0.1):
        self.cylinders.append([x, y, r])

    def add_wall(self, x1, y1, x2, y2):
        self.walls.append([x1, y1, x2, y2])
        self._rebuild()

    def add_mover(self, x, y, vx=0.0, vy=0.0, r=0.1):
        self.movers.append([x, y, vx, vy, r])

    def _rebuild(self):
        x0, y0, x1, y1 = self.bounds
        segs = [[x0, y0, x1, y0], [x1, y0, x1, y1], [x1, y1, x0, y1], [x0, y1, x0, y0]]
        segs += self.walls
        for cx, cy, w, h in self.boxes:
            a, b, c, d = cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2
            segs += [[a, b, c, b], [c, b, c, d], [c, d, a, d], [a, d, a, b]]
        self.segs = np.array(segs, dtype=float)

    def step_movers(self, dt):
        x0, y0, x1, y1 = self.bounds
        for m in self.movers:
            m[0] += m[2] * dt
            m[1] += m[3] * dt
            if m[0] - m[4] < x0 or m[0] + m[4] > x1:
                m[2] = -m[2]
            if m[1] - m[4] < y0 or m[1] + m[4] > y1:
                m[3] = -m[3]

    def circles(self):
        return [tuple(c) for c in self.cylinders] + [(m[0], m[1], m[4]) for m in self.movers]

    # ---------------------------------------------------------------- sensing
    def raycast(self, x, y, angles, max_range=12.0):
        """Distances from (x, y) along world-frame angles (numpy array)."""
        dx = np.cos(angles)[:, None]
        dy = np.sin(angles)[:, None]
        best = np.full(len(angles), np.inf)
        s = self.segs
        if len(s):
            ex = (s[:, 2] - s[:, 0])[None, :]
            ey = (s[:, 3] - s[:, 1])[None, :]
            qx = (s[:, 0] - x)[None, :]
            qy = (s[:, 1] - y)[None, :]
            den = dx * ey - dy * ex
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (qx * ey - qy * ex) / den
                u = (qx * dy - qy * dx) / den
            ok = (np.abs(den) > 1e-12) & (t > 1e-6) & (u >= 0) & (u <= 1)
            t = np.where(ok, t, np.inf)
            best = np.minimum(best, t.min(axis=1))
        for cx, cy, r in self.circles():
            fx, fy = x - cx, y - cy
            b = fx * dx[:, 0] + fy * dy[:, 0]
            c = fx * fx + fy * fy - r * r
            disc = b * b - c
            with np.errstate(invalid="ignore"):
                t = -b - np.sqrt(disc)
            t = np.where((disc >= 0) & (t > 1e-6), t, np.inf)
            best = np.minimum(best, t)
        best[best > max_range] = np.inf
        return best

    def collides(self, x, y, radius):
        s = self.segs
        if len(s):
            px, py = s[:, 0], s[:, 1]
            ex, ey = s[:, 2] - px, s[:, 3] - py
            L2 = ex * ex + ey * ey
            t = np.clip(((x - px) * ex + (y - py) * ey) / np.where(L2 > 0, L2, 1), 0, 1)
            d2 = (px + t * ex - x) ** 2 + (py + t * ey - y) ** 2
            if d2.min() < radius * radius:
                return True
        for cx, cy, r in self.circles():
            if (x - cx) ** 2 + (y - cy) ** 2 < (r + radius) ** 2:
                return True
        return False

    def to_dict(self):
        return {"name": self.name, "title": self.title, "bounds": self.bounds,
                "walls": self.walls, "boxes": self.boxes, "cylinders": self.cylinders}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))
