"""Copper geometry for clearance checks: boxes (pads), circles (vias / round pads) and
capsules (tracks), with a uniform-grid spatial hash."""
import math
from collections import defaultdict


class Shape:
    __slots__ = ("kind", "a", "net", "layer", "tag", "bbox")

    def __init__(self, kind, a, net, layer, tag=None):
        self.kind, self.a, self.net, self.layer, self.tag = kind, a, net, layer, tag
        if kind == "box":
            self.bbox = a
        elif kind == "circle":
            x, y, r = a
            self.bbox = (x - r, y - r, x + r, y + r)
        else:  # seg: x1, y1, x2, y2, hw
            x1, y1, x2, y2, hw = a
            self.bbox = (min(x1, x2) - hw, min(y1, y2) - hw, max(x1, x2) + hw, max(y1, y2) + hw)


def pt_seg(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    L = dx * dx + dy * dy
    if L == 0:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def seg_seg(a, b):
    x1, y1, x2, y2 = a
    x3, y3, x4, y4 = b
    if _intersect(a, b):
        return 0.0
    return min(pt_seg(x1, y1, *b), pt_seg(x2, y2, *b), pt_seg(x3, y3, *a), pt_seg(x4, y4, *a))


def _ccw(ax, ay, bx, by, cx, cy):
    return (cy - ay) * (bx - ax) - (by - ay) * (cx - ax)


def _intersect(a, b):
    x1, y1, x2, y2 = a
    x3, y3, x4, y4 = b
    d1 = _ccw(x3, y3, x4, y4, x1, y1)
    d2 = _ccw(x3, y3, x4, y4, x2, y2)
    d3 = _ccw(x1, y1, x2, y2, x3, y3)
    d4 = _ccw(x1, y1, x2, y2, x4, y4)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)) and d1 and d2 and d3 and d4


def pt_box(px, py, b):
    dx = max(b[0] - px, 0, px - b[2])
    dy = max(b[1] - py, 0, py - b[3])
    return math.hypot(dx, dy)


def seg_box(s, b):
    x1, y1, x2, y2 = s
    if (b[0] <= x1 <= b[2] and b[1] <= y1 <= b[3]) or (b[0] <= x2 <= b[2] and b[1] <= y2 <= b[3]):
        return 0.0
    edges = [(b[0], b[1], b[2], b[1]), (b[2], b[1], b[2], b[3]), (b[2], b[3], b[0], b[3]),
             (b[0], b[3], b[0], b[1])]
    return min(seg_seg(s, e) for e in edges)


def box_box(a, b):
    dx = max(a[0] - b[2], 0, b[0] - a[2])
    dy = max(a[1] - b[3], 0, b[1] - a[3])
    return math.hypot(dx, dy)


def distance(s, t):
    """Edge-to-edge distance between two shapes (0 if overlapping)."""
    k1, k2 = s.kind, t.kind
    if k1 > k2:
        s, t = t, s
        k1, k2 = k2, k1
    a, b = s.a, t.a
    if k1 == "box" and k2 == "box":
        return box_box(a, b)
    if k1 == "box" and k2 == "circle":
        return max(0.0, pt_box(b[0], b[1], a) - b[2])
    if k1 == "box" and k2 == "seg":
        return max(0.0, seg_box(b[:4], a) - b[4])
    if k1 == "circle" and k2 == "circle":
        return max(0.0, math.hypot(a[0] - b[0], a[1] - b[1]) - a[2] - b[2])
    if k1 == "circle" and k2 == "seg":
        return max(0.0, pt_seg(a[0], a[1], *b[:4]) - a[2] - b[4])
    if k1 == "seg" and k2 == "seg":
        return max(0.0, seg_seg(a[:4], b[:4]) - a[4] - b[4])
    raise ValueError((k1, k2))


class SpatialIndex:
    def __init__(self, cell=1.0):
        self.cell = cell
        self.grid = defaultdict(list)
        self.items = []

    def _cells(self, bb, grow=0.0):
        c = self.cell
        for i in range(int(math.floor((bb[0] - grow) / c)), int(math.floor((bb[2] + grow) / c)) + 1):
            for j in range(int(math.floor((bb[1] - grow) / c)), int(math.floor((bb[3] + grow) / c)) + 1):
                yield (i, j)

    def add(self, s):
        self.items.append(s)
        for k in self._cells(s.bbox):
            self.grid[k].append(s)

    def near(self, s, grow):
        seen = set()
        for k in self._cells(s.bbox, grow):
            for t in self.grid.get(k, ()):
                if id(t) not in seen:
                    seen.add(id(t))
                    yield t

    def conflicts(self, s, clearance, layers=None):
        """Items on the same layer, other net, closer than clearance."""
        out = []
        for t in self.near(s, clearance + 0.5):
            if t.layer != s.layer and t.layer != "*" and s.layer != "*":
                continue
            if t.net == s.net and s.net:
                continue
            if distance(s, t) < clearance - 1e-6:
                out.append(t)
        return out
