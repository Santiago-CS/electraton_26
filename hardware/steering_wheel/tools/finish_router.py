"""Finishing router: closes the connections an autorouter left open.

Grid A* on F.Cu + B.Cu (0.05 mm cells, 8-connected, vias between the two layers) over
distance fields: every other-net copper shape is rasterised, and a cell is usable for a
track of half-width hw when its distance to foreign copper is >= clearance + hw (+ the
grid error margin). Vias need the same on both layers, clearance to inner-layer tracks
and hole-to-hole spacing. Each open net is split into its connected copper components
(pads, tracks, vias; plane nets are joined through their plane) and the components are
chained together shortest-first. Every new track/via is re-checked with the exact
geometry checker before it is accepted.

The search runs in a window around the two components (grown when it fails), so the
distance fields stay small.

    python tools/finish_router.py build/routed.kicad_pcb build/finish.json [--rr]

--rr adds rip-up and re-route: a connection that cannot be made around the existing
(unlocked, autorouted) wiring is routed around the fixed copper only; the unlocked
wiring in its way is removed and those nets go back on the queue.
"""
import heapq
import json
import math
import os
import sys
import time

import numpy as np
from scipy import ndimage
try:
    from skimage.graph import MCP_Geometric      # compiled Dijkstra (pip install scikit-image)
except ImportError:                                # falls back to the pure-Python A*
    MCP_Geometric = None

sys.path.insert(0, os.path.dirname(__file__))
import layout_steering as LS  # noqa: E402
import fanout  # noqa: E402
from board_rules import NETCLASSES, netclass_of  # noqa: E402
from geom import Shape, SpatialIndex, distance  # noqa: E402
from kpcb_read import tracks_vias  # noqa: E402

RES = 0.05
# 8-layer JLC08161H-7628. The outer pair is tried first because it needs no vias;
# In2/In5 are ordinary signal layers (on 4 layers In2 was the +3V3 plane, hence the
# old "last resort" treatment), and the four planes are never routed on.
LAYERS = ("F.Cu", "B.Cu")
INNER_SIG = ("In2.Cu", "In5.Cu")
LAYERS3 = LAYERS + INNER_SIG                # escalation: outer plus the inner signal layers
PLANE_LAYERS = ("In1.Cu", "In3.Cu", "In4.Cu", "In6.Cu")
ALL_CU = ("F.Cu", "In1.Cu", "In2.Cu", "In3.Cu", "In4.Cu", "In5.Cu", "In6.Cu", "B.Cu")
PLANES = {"GND": ("In1.Cu", "In4.Cu", "In6.Cu"), "+3V3": ("In3.Cu",)}
PAD_CELLS = int(math.ceil(1.0 / RES))      # distance-field context around a search window


def clearance_of(net):
    return NETCLASSES[netclass_of(net)][0]


class Grid:
    def __init__(self, x0, y0, x1, y1):
        self.x0, self.y0 = x0, y0
        self.nx = int(math.ceil((x1 - x0) / RES)) + 1
        self.ny = int(math.ceil((y1 - y0) / RES)) + 1

    def ij(self, x, y):
        return int(round((x - self.x0) / RES)), int(round((y - self.y0) / RES))

    def xy(self, i, j):
        return self.x0 + i * RES, self.y0 + j * RES

    def sub(self, i0, j0, i1, j1):
        s = Grid.__new__(Grid)
        s.x0, s.y0 = self.x0 + i0 * RES, self.y0 + j0 * RES
        s.nx, s.ny = i1 - i0, j1 - j0
        return s


def _inside(grid, shape):
    """(i0, j0, bool array) of cell centres inside the shape, over its bbox"""
    x0, y0, x1, y1 = shape.bbox
    i0, j0 = grid.ij(x0, y0)
    i1, j1 = grid.ij(x1, y1)
    i0, j0 = max(i0 - 1, 0), max(j0 - 1, 0)
    i1, j1 = min(i1 + 1, grid.nx - 1), min(j1 + 1, grid.ny - 1)
    if i1 < i0 or j1 < j0:
        return i0, j0, None
    ii, jj = np.meshgrid(np.arange(i0, i1 + 1), np.arange(j0, j1 + 1), indexing="ij")
    xs, ys = grid.x0 + ii * RES, grid.y0 + jj * RES
    if shape.kind == "box":
        bx = shape.a
        inside = (xs >= bx[0]) & (xs <= bx[2]) & (ys >= bx[1]) & (ys <= bx[3])
    elif shape.kind == "circle":
        cx, cy, r = shape.a
        inside = (xs - cx) ** 2 + (ys - cy) ** 2 <= r * r
    else:
        ax, ay, bx_, by_, hw = shape.a
        dx, dy = bx_ - ax, by_ - ay
        L2 = dx * dx + dy * dy
        t = np.zeros_like(xs) if L2 == 0 else np.clip(((xs - ax) * dx + (ys - ay) * dy) / L2, 0, 1)
        inside = (xs - (ax + t * dx)) ** 2 + (ys - (ay + t * dy)) ** 2 <= hw * hw
    return i0, j0, inside


def raster_shape(grid, arr, shape, value=True):
    i0, j0, inside = _inside(grid, shape)
    if inside is None:
        return
    sub = arr[i0:i0 + inside.shape[0], j0:j0 + inside.shape[1]]
    sub[inside] = value


def copper_items(L, tracks, vias):
    """all copper as (net, layers, shape, kind) records"""
    items = []
    for fp in L.b.fps.values():
        for p in fp.pads:
            cl = p.copper_layers(list(ALL_CU))
            if p.kind == "np_thru_hole":
                x, y = fp.pad_xy(p)
                r = max(p.drill) / 2 if p.drill else p.w / 2
                items.append(dict(net=None, layers=list(ALL_CU),
                                  shape=("circle", (x, y, r)), kind="npth", hole=r, hc=(x, y)))
                continue
            if not cl:
                continue
            s = fanout.Copper.pad_shape(fp, p, "x")
            c = fanout.Copper.pad_conn_shape(fp, p, "x")
            poly = fanout.Copper.pad_poly(fp, p)
            x, y = fp.pad_xy(p)
            items.append(dict(net=fp.pad_net.get(p.num), layers=cl, shape=(s.kind, s.a), kind="pad",
                              conn=(c.kind, c.a), poly=poly,
                              hole=(max(p.drill) / 2 if (p.kind == "thru_hole" and p.drill) else None),
                              hc=(x, y), ref=(fp.ref, p.num)))
    for t in tracks:
        items.append(dict(net=t["net"], layers=[t["layer"]],
                          shape=("seg", (t["x1"], t["y1"], t["x2"], t["y2"], t["w"] / 2)), kind="track"))
    for v in vias:
        items.append(dict(net=v["net"], layers=list(ALL_CU),
                          shape=("circle", (v["x"], v["y"], v["d"] / 2)), kind="via", hole=v["drill"] / 2,
                          hc=(v["x"], v["y"])))
    return items


def raster_poly(g, arr, poly, value):
    """Set the cells whose centre lies inside an exact shapely polygon."""
    import shapely
    x0, y0, x1, y1 = poly.bounds
    i0 = max(int(math.floor((x0 - g.x0) / RES)), 0)
    j0 = max(int(math.floor((y0 - g.y0) / RES)), 0)
    i1 = min(int(math.ceil((x1 - g.x0) / RES)) + 1, g.nx)
    j1 = min(int(math.ceil((y1 - g.y0) / RES)) + 1, g.ny)
    if i1 <= i0 or j1 <= j0:
        return
    xs = g.x0 + np.arange(i0, i1) * RES
    ys = g.y0 + np.arange(j0, j1) * RES
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    inside = shapely.contains_xy(poly, X, Y)
    if not inside.any():
        # a pad smaller than one cell: keep the nearest cell so it is still a target
        cx, cy = poly.centroid.x, poly.centroid.y
        i = min(max(int(round((cx - g.x0) / RES)), 0), g.nx - 1)
        j = min(max(int(round((cy - g.y0) / RES)), 0), g.ny - 1)
        arr[i, j] = value
        return
    sub = arr[i0:i1, j0:j1]
    sub[inside] = value


def conn_of(it):
    """What the router should aim at: a pad's inscribed capsule (guaranteed to be real
    copper), or the item's own shape for tracks and vias."""
    return it.get("conn") or it["shape"]


def exact_geom(it):
    """The item's true outline as a shapely geometry, cached on the item.

    Only used to confirm a candidate touch found with the cheap bounding-box shapes, so a
    pad's rotation and corner rounding decide connectivity rather than its bounding box.
    """
    g = it.get("_sg")
    if g is None:
        from shapely.geometry import LineString, Point, box as _box
        g = it.get("poly")
        if g is None:
            kind, a = it["shape"]
            if kind == "seg":
                x1, y1, x2, y2, r = a
                g = (Point(x1, y1) if (x1, y1) == (x2, y2) else
                     LineString([(x1, y1), (x2, y2)])).buffer(r, quad_segs=8)
            elif kind == "circle":
                g = Point(a[0], a[1]).buffer(a[2], quad_segs=8)
            else:
                g = _box(*a)
        it["_sg"] = g
    return g


def item_bbox(it):
    return Shape(it["shape"][0], it["shape"][1], None, "x").bbox


def components(items):
    """connected copper components per net (plane nets joined through their plane)"""
    parent = list(range(len(items)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
    idx = SpatialIndex(1.0)
    shapes = []
    for k, it in enumerate(items):
        s = Shape(it["shape"][0], it["shape"][1], it["net"], "x", tag=k)
        shapes.append(s)
        if it["net"]:
            idx.add(s)
    for k, it in enumerate(items):
        if not it["net"]:
            continue
        s = shapes[k]
        for t in idx.near(s, 0.01):
            j = t.tag
            if j <= k or items[j]["net"] != it["net"]:
                continue
            if not set(it["layers"]) & set(items[j]["layers"]):
                continue
            if distance(s, t) > 1e-4:
                continue
            if (it.get("poly") is not None or items[j].get("poly") is not None) and \
                    exact_geom(it).distance(exact_geom(items[j])) > 1e-4:
                continue        # the boxes overlap but the real copper does not touch
            union(k, j)
    # plane nets: every item touching the plane layer is on the plane
    first = {}
    for k, it in enumerate(items):
        pls = PLANES.get(it["net"]) or ()
        if any(pl in it["layers"] for pl in pls):
            if it["net"] in first:
                union(k, first[it["net"]])
            else:
                first[it["net"]] = k
    comps = {}
    for k, it in enumerate(items):
        if it["net"]:
            comps.setdefault(it["net"], {}).setdefault(find(k), []).append(k)
    return comps


class Router:
    """grid maze router over class-aware clearance fields, with any-angle pull-tight"""

    def __init__(self, L, tracks, vias, box):
        self.L = L
        self.grid = Grid(*box)
        self.new_tracks, self.new_vias = [], []
        self.items = copper_items(L, list(tracks), list(vias))
        g = self.grid
        nets = sorted({it["net"] for it in self.items if it["net"]} | set(L.nets))
        self.net_id = {n: i + 1 for i, n in enumerate(nets)}
        self.id_cl = {i + 1: clearance_of(n) for i, n in enumerate(nets)}
        self.id_cl[-1] = 0.2                      # NPTH / unconnected copper
        self.label = {l: np.zeros((g.nx, g.ny), np.int32) for l in ALL_CU}
        self.hole = np.zeros((g.nx, g.ny), bool)
        self.smd = np.zeros((g.nx, g.ny), bool)
        for it in self.items:
            self._raster(it)
            if it["kind"] == "pad" and it["hole"] is None:
                raster_shape(g, self.smd, Shape(it["shape"][0], it["shape"][1], None, "x"), True)
        import shapely
        from shapely.geometry import Polygon
        poly = Polygon(L.b.outline).buffer(-0.35)
        X, Y = np.meshgrid(g.x0 + np.arange(g.nx) * RES, g.y0 + np.arange(g.ny) * RES, indexing="ij")
        self.inside = shapely.contains_xy(poly, X, Y)
        self.dist_hole = ndimage.distance_transform_edt(~self.hole) * RES
        self.dist_smd = ndimage.distance_transform_edt(~self.smd) * RES
        # exact checker for pull-tight / acceptance
        L.b.tracks, L.b.vias = list(tracks), list(vias)
        self.cop = fanout.Copper(L.b, L.b.outline, inner=True)
        self.label_hard = None                    # set by RRRouter
        self.keepout = {}                         # layer -> bool array: cells closed to routing
        self.keepout_shapes = getattr(self, "keepout_shapes", [])
        for l, sh in self.keepout_shapes:
            self.add_keepout(l, sh, remember=False)

    def _raster(self, it):
        g = self.grid
        nid = self.net_id.get(it["net"], -1)
        sh = Shape(it["shape"][0], it["shape"][1], None, "x")
        for l in it["layers"]:
            if l in self.label:
                raster_shape(g, self.label[l], sh, nid)
        if it.get("hole"):
            raster_shape(g, self.hole, Shape("circle", (it["hc"][0], it["hc"][1], it["hole"]), None, "x"), True)

    def add(self, tracks, vias):
        new = copper_items(_NoFp(), tracks, vias)
        for it in new:
            self._raster(it)
            self.items.append(it)
        for t in tracks:
            self.cop.add_track(t)
        for v in vias:
            self.cop.add_via(v)
        if vias:
            self.dist_hole = ndimage.distance_transform_edt(~self.hole) * RES
        self.new_tracks += tracks
        self.new_vias += vias

    # ------------------------------------------------------------------ masks
    def _labels(self, layer, sl, mode):
        if mode == "all":
            return self.label[layer][sl]
        if mode == "hard":
            return self.label_hard[layer][sl]
        lf = self.label_hard[layer][sl]           # "soft": unlocked wiring only
        return np.where(lf == 0, self.label[layer][sl], 0)

    def free_mask(self, net, layer, rad, margin, win=None, mode="all"):
        """cells of the window where a disc of radius rad keeps class clearance to all
        foreign copper"""
        g = self.grid
        i0, j0, i1, j1 = win or (0, 0, g.nx, g.ny)
        a0, b0 = max(i0 - PAD_CELLS, 0), max(j0 - PAD_CELLS, 0)
        a1, b1 = min(i1 + PAD_CELLS, g.nx), min(j1 + PAD_CELLS, g.ny)
        sl = (slice(a0, a1), slice(b0, b1))
        lab = self._labels(layer, sl, mode)
        nid = self.net_id.get(net, 0)
        own = clearance_of(net)
        ok = np.ones((a1 - a0, b1 - b0), bool) if mode == "soft" else self.inside[sl].copy()
        present = [int(v) for v in np.unique(lab) if v != 0 and v != nid]
        by_cl = {}
        for v in present:
            by_cl.setdefault(self.id_cl.get(v, 0.2), []).append(v)
        for c, ids in by_cl.items():
            foreign = np.isin(lab, ids)
            d = ndimage.distance_transform_edt(~foreign) * RES
            ok &= d >= max(own, c) + rad + margin
        return ok[i0 - a0:i1 - a0, j0 - b0:j1 - b0]

    def add_keepout(self, layer, shape, remember=True):
        """close the cells of a Shape on one layer to routing"""
        g = self.grid
        if remember:
            self.keepout_shapes.append((layer, shape))
        if layer not in self.keepout:
            self.keepout[layer] = np.zeros((g.nx, g.ny), bool)
        raster_shape(g, self.keepout[layer], shape, True)

    def own_mask(self, comp, layer, grid=None):
        g = grid or self.grid
        m = np.zeros((g.nx, g.ny), bool)
        for k in comp:
            it = self.items[k]
            if layer in it["layers"]:
                if it.get("poly") is not None:
                    raster_poly(g, m, it["poly"], True)
                else:
                    raster_shape(g, m, Shape(it["shape"][0], it["shape"][1], None, "x"), True)
        return m

    def window(self, comp_a, comp_b, m):
        """search window: comp_a plus the nearest item of comp_b, grown by m mm"""
        g = self.grid
        if m is None:
            return (0, 0, g.nx, g.ny)
        ba = [item_bbox(self.items[k]) for k in comp_a]
        x0, y0 = min(b[0] for b in ba), min(b[1] for b in ba)
        x1, y1 = max(b[2] for b in ba), max(b[3] for b in ba)
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        best, bb = None, None
        for k in comp_b:
            b = item_bbox(self.items[k])
            dx = max(b[0] - cx, 0, cx - b[2])
            dy = max(b[1] - cy, 0, cy - b[3])
            d = dx * dx + dy * dy
            if best is None or d < best:
                best, bb = d, b
        x0, y0 = min(x0, bb[0]) - m, min(y0, bb[1]) - m
        x1, y1 = max(x1, bb[2]) + m, max(y1, bb[3]) + m
        i0, j0 = g.ij(x0, y0)
        i1, j1 = g.ij(x1, y1)
        return (max(i0, 0), max(j0, 0), min(i1 + 1, g.nx), min(j1 + 1, g.ny))

    # ------------------------------------------------------------------ search
    def route(self, net, comp_a, comp_b, width, via=(0.4, 0.2), via_cost=48, margin=0.04, limit=3_000_000,
              ban=(), relaxed=False, penalty=30.0, margins=(2.5, 8.0, 25.0), layers=LAYERS, inner_cost=1.5):
        last = None
        for m in margins:
            win = self.window(comp_a, comp_b, m)
            if win == last:
                continue
            last = win
            res = self._route_win(net, comp_a, comp_b, width, via, via_cost, margin, limit, ban, relaxed,
                                  penalty, win, tuple(layers), inner_cost)
            if res is not None:
                return res
        return None

    def _route_win(self, net, comp_a, comp_b, width, via, via_cost, margin, limit, ban, relaxed, penalty, win,
                   lays=LAYERS, inner_cost=1.5):
        i0, j0, i1, j1 = win
        sg = self.grid.sub(i0, j0, i1, j1)
        nx, ny = sg.nx, sg.ny
        mode = "hard" if relaxed else "all"
        hw = width / 2
        ok = {l: self.free_mask(net, l, hw, margin, win, mode) for l in lays}
        r = via[0] / 2
        via_ok = self.free_mask(net, "F.Cu", r, margin, win, mode) & \
            self.free_mask(net, "B.Cu", r, margin, win, mode) & \
            (self.dist_hole[i0:i1, j0:j1] >= 0.27 + via[1] / 2) & (self.dist_smd[i0:i1, j0:j1] >= r + 0.08)
        for _pl in PLANE_LAYERS + INNER_SIG:   # a through via pierces every other layer too
            via_ok &= self.free_mask(net, _pl, r, 0.1, win, mode)
        for l, ko in self.keepout.items():
            if l in ok:
                ok[l] &= ~ko[i0:i1, j0:j1]
            via_ok &= ~ko[i0:i1, j0:j1]
        soft = None
        if relaxed:
            soft = {l: ~self.free_mask(net, l, hw, margin, win, "soft") for l in lays}
        for (bi, bj) in ban:
            i, j = bi - i0, bj - j0
            if not (-4 <= i < nx + 4 and -4 <= j < ny + 4):
                continue
            via_ok[max(i - 3, 0):max(i + 4, 0), max(j - 3, 0):max(j + 4, 0)] = False
            for l in lays:
                ok[l][max(i - 2, 0):max(i + 3, 0), max(j - 2, 0):max(j + 3, 0)] = False
        src = {l: self.own_mask(comp_a, l, sg) for l in lays}
        dst = {l: self.own_mask(comp_b, l, sg) for l in lays}
        dsts = [(i, j) for l in lays for i, j in zip(*np.nonzero(dst[l]))]
        if not dsts or not any(src[l].any() for l in lays):
            return None
        lcost = [1.0 if l in LAYERS else inner_cost for l in lays]
        if MCP_Geometric is not None:
            path = self._search_mcp(lays, ok, via_ok, src, dst, soft, penalty, lcost, via_cost, nx, ny)
            if path is None:
                return None
            return self._to_copper(path, net, width, via, sg, relaxed, lays)
        di = np.array([p[0] for p in dsts])
        dj = np.array([p[1] for p in dsts])
        ci, cj = di.mean(), dj.mean()
        rad = math.sqrt(((di - ci) ** 2 + (dj - cj) ** 2).max())

        def h(i, j):
            return max(0.0, math.hypot(i - ci, j - cj) - rad)
        openq, best, prev = [], {}, {}
        for li, l in enumerate(lays):
            ii, jj = np.nonzero(src[l])
            for i, j in zip(ii, jj):
                key = (int(i), int(j), li)
                best[key] = 0.0
                heapq.heappush(openq, (h(i, j), 0.0, key))
        moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
        found = None
        n = 0
        while openq and n < limit:
            f, gcost, (i, j, li) = heapq.heappop(openq)
            if gcost > best.get((i, j, li), 1e18):
                continue
            n += 1
            l = lays[li]
            if dst[l][i, j] and (i, j, li) in prev:
                found = (i, j, li)
                break
            okl, dstl, srcl = ok[l], dst[l], src[l]
            lc = lcost[li]
            for di_, dj_, c in moves:
                a, b_ = i + di_, j + dj_
                if not (0 <= a < nx and 0 <= b_ < ny):
                    continue
                if not (okl[a, b_] or dstl[a, b_] or srcl[a, b_]):
                    continue
                ng = gcost + c * lc
                if soft is not None and soft[l][a, b_]:
                    ng += penalty
                k2 = (a, b_, li)
                if ng < best.get(k2, 1e18):
                    best[k2] = ng
                    prev[k2] = (i, j, li)
                    heapq.heappush(openq, (ng + h(a, b_), ng, k2))
            if via_ok[i, j] and not srcl[i, j] and not dstl[i, j]:
                for lj in range(len(lays)):
                    if lj == li:
                        continue
                    k2 = (i, j, lj)
                    ng = gcost + via_cost
                    if soft is not None and any(soft[q][i, j] for q in lays):
                        ng += penalty
                    if ng < best.get(k2, 1e18):
                        best[k2] = ng
                        prev[k2] = (i, j, li)
                        heapq.heappush(openq, (ng + h(i, j), ng, k2))
        if not found:
            return None
        path = [found]
        while path[-1] in prev:
            path.append(prev[path[-1]])
        path.reverse()
        start = 0
        for k, (i, j, li) in enumerate(path):
            if src[lays[li]][i, j]:
                start = k
        path = path[start:]
        end = len(path) - 1
        for k, (i, j, li) in enumerate(path):
            if dst[lays[li]][i, j]:
                end = k
                break
        path = path[:end + 1]
        return self._to_copper(path, net, width, via, sg, relaxed, lays)

    def _search_mcp(self, lays, ok, via_ok, src, dst, soft, penalty, lcost, via_cost, nx, ny):
        """minimum-cost path over a layer stack [L0, via, L1, via, L2 ...] (scikit-image MCP,
        compiled Dijkstra); returns [(i, j, layer index), ...] from src to dst"""
        nl = len(lays)
        cost = np.full((2 * nl - 1, nx, ny), np.inf)
        for li, l in enumerate(lays):
            c = np.where(ok[l], lcost[li], np.inf)
            if soft is not None:
                c = np.where(ok[l] & soft[l], lcost[li] + penalty, c)
            c = np.where(src[l] | dst[l], 1.0, c)
            cost[2 * li] = c
        vc = max(via_cost - 1.0, 1.0)
        for k in range(nl - 1):
            v = np.where(via_ok, vc, np.inf)
            if soft is not None:
                anys = np.zeros((nx, ny), bool)
                for l in lays:
                    anys |= soft[l]
                v = np.where(via_ok & anys, vc + penalty, v)
            # no via inside the own copper (it would land in a pad)
            for l in lays:
                v = np.where(src[l] | dst[l], np.inf, v)
            cost[2 * k + 1] = v
        starts = [(2 * li, int(i), int(j)) for li, l in enumerate(lays) for i, j in zip(*np.nonzero(src[l]))]
        ends = [(2 * li, int(i), int(j)) for li, l in enumerate(lays) for i, j in zip(*np.nonzero(dst[l]))]
        if not starts or not ends:
            return None
        offs = [(0, a, b) for a in (-1, 0, 1) for b in (-1, 0, 1) if (a, b) != (0, 0)] + [(1, 0, 0), (-1, 0, 0)]
        m = MCP_Geometric(cost, offsets=offs, fully_connected=False)
        cum, _ = m.find_costs(starts, ends, find_all_ends=False)
        ez = np.array([e[0] for e in ends])
        ei = np.array([e[1] for e in ends])
        ej = np.array([e[2] for e in ends])
        ce = cum[ez, ei, ej]
        k = int(np.argmin(ce))
        if not np.isfinite(ce[k]):
            return None
        raw = m.traceback(ends[k])
        path = [(i, j, z // 2) for z, i, j in raw if z % 2 == 0]
        # trim to the last source cell and the first destination cell
        start = 0
        for q, (i, j, li) in enumerate(path):
            if src[lays[li]][i, j]:
                start = q
        path = path[start:]
        end = len(path) - 1
        for q, (i, j, li) in enumerate(path):
            if dst[lays[li]][i, j]:
                end = q
                break
        return path[:end + 1]

    def _to_copper(self, path, net, width, via, sg, relaxed=False, lays=LAYERS):
        tracks, vias = [], []
        seg = [path[0]]
        for k in range(1, len(path)):
            a, b = path[k - 1], path[k]
            if a[2] != b[2]:
                tracks += self._chain(seg, net, width, sg, relaxed, lays)
                x, y = sg.xy(a[0], a[1])
                if not (vias and abs(vias[-1]["x"] - x) < 1e-6 and abs(vias[-1]["y"] - y) < 1e-6):
                    vias.append(dict(x=x, y=y, d=via[0], drill=via[1], net=net, locked=False))
                seg = [b]
            else:
                seg.append(b)
        tracks += self._chain(seg, net, width, sg, relaxed, lays)
        return tracks, vias

    def _chain(self, seg, net, width, sg, relaxed=False, lays=LAYERS):
        """grid cells -> polyline, then any-angle pull-tight with exact clearance checks"""
        if len(seg) < 2:
            return []
        layer = lays[seg[0][2]]
        pts = [sg.xy(i, j) for i, j, _ in seg]
        # collapse collinear runs first (fewer exact checks)
        simp = [pts[0]]
        for k in range(1, len(pts) - 1):
            ax, ay = simp[-1]
            bx, by = pts[k]
            cx, cy = pts[k + 1]
            if abs((bx - ax) * (cy - by) - (by - ay) * (cx - bx)) > 1e-9:
                simp.append(pts[k])
        simp.append(pts[-1])
        pts = simp
        cop = self.cop_hard if relaxed else self.cop
        out = [pts[0]]
        k = 0
        while k < len(pts) - 1:
            nxt = k + 1
            for m in range(len(pts) - 1, k + 1, -1):
                if cop.seg_ok(pts[k][0], pts[k][1], pts[m][0], pts[m][1], width, net, layer, margin=0.02):
                    nxt = m
                    break
            out.append(pts[nxt])
            k = nxt
        return [dict(layer=layer, x1=a[0], y1=a[1], x2=b[0], y2=b[1], w=width, net=net, locked=False)
                for a, b in zip(out, out[1:])]

    def exact_bad(self, res, net, cop=None):
        cop = cop or self.cop
        t_new, v_new = res
        bad = [(v["x"], v["y"]) for v in v_new if not cop.via_ok(v["x"], v["y"], v["d"], v["drill"], net,
                                                                 margin=0.01)]
        bad += [((t["x1"] + t["x2"]) / 2, (t["y1"] + t["y2"]) / 2) for t in t_new
                if not cop.seg_ok(t["x1"], t["y1"], t["x2"], t["y2"], t["w"], net, t["layer"], margin=0.005)]
        return bad

    def checked_route(self, net, comp_a, comp_b, width, tries=4, **kw):
        """route + exact check; offending spots are banned and the search repeated"""
        ban = []
        for _ in range(tries):
            res = self.route(net, comp_a, comp_b, width, ban=ban, **kw)
            if res is None:
                return None
            bad = self.exact_bad(res, net)
            if not bad:
                return res
            ban += [self.grid.ij(x, y) for x, y in bad]
        return None


class _NoFp:
    class _B:
        fps = {}
    b = _B()


class RRRouter(Router):
    """Router with rip-up and re-route. Unlocked copper inside no_rip_box (the BGA escape
    wiring) is never ripped: it counts as fixed for the relaxed search."""

    def __init__(self, L, tracks, vias, box, no_rip_box=None):
        self.box = box
        self.no_rip_box = no_rip_box
        self.fixed_t = [t for t in tracks if t.get("locked")]
        self.fixed_v = [v for v in vias if v.get("locked")]
        self.soft_t = [t for t in tracks if not t.get("locked")]
        self.soft_v = [v for v in vias if not v.get("locked")]
        super().__init__(L, tracks, vias, box)
        g = self.grid
        hard_t = self.fixed_t + [t for t in self.soft_t if not self.t_rippable(t)]
        hard_v = self.fixed_v + [v for v in self.soft_v if not self.v_rippable(v)]
        L.b.tracks, L.b.vias = list(hard_t), list(hard_v)
        self.cop_hard = fanout.Copper(L.b, L.b.outline, inner=True)
        L.b.tracks, L.b.vias = self.fixed_t + self.soft_t, self.fixed_v + self.soft_v
        self.label_hard = {l: np.zeros((g.nx, g.ny), np.int32) for l in ALL_CU}
        for it in copper_items(L, hard_t, hard_v):
            nid = self.net_id.get(it["net"], -1)
            sh = Shape(it["shape"][0], it["shape"][1], None, "x")
            for l in it["layers"]:
                if l in self.label_hard:
                    raster_shape(g, self.label_hard[l], sh, nid)

    def add(self, tracks, vias):
        super().add(tracks, vias)
        self.soft_t += tracks
        self.soft_v += vias

    def rebuild(self):
        self.__init__(self.L, self.fixed_t + self.soft_t, self.fixed_v + self.soft_v, self.box, self.no_rip_box)

    def rippable(self, x, y):
        b = self.no_rip_box
        return not (b and b[0] <= x <= b[2] and b[1] <= y <= b[3])

    def t_rippable(self, t):
        return self.rippable((t["x1"] + t["x2"]) / 2, (t["y1"] + t["y2"]) / 2)

    def v_rippable(self, v):
        return self.rippable(v["x"], v["y"])

    def relaxed_route(self, net, comp_a, comp_b, width, via=(0.4, 0.2), penalty=40.0, layers=LAYERS):
        """route around fixed copper only (exact-checked against it); returns copper and the
        unlocked items it hits"""
        ban = []
        for attempt in range(5):
            res = self.route(net, comp_a, comp_b, width, via=via, margin=0.04, relaxed=True, penalty=penalty,
                             ban=ban, margins=(4.0, 12.0, 30.0), layers=layers)
            if res is None:
                return None, ([], [])
            bad = self.exact_bad(res, net, self.cop_hard)
            if not bad:
                return res, self.hits(net, *res)
            ban += [self.grid.ij(x, y) for x, y in bad]
        return None, ([], [])

    def hits(self, net, t_new, v_new):
        new_shapes = [Shape("seg", (t["x1"], t["y1"], t["x2"], t["y2"], t["w"] / 2), net, t["layer"]) for t in t_new]
        new_shapes += [Shape("circle", (v["x"], v["y"], v["d"] / 2), net, l) for v in v_new for l in ALL_CU]
        bbs = [s.bbox for s in new_shapes]
        own = clearance_of(net)
        hit_t, hit_v = [], []

        def near(bb, need):
            return any(bb[0] - need <= b[2] and b[0] - need <= bb[2] and bb[1] - need <= b[3] and
                       b[1] - need <= bb[3] for b in bbs)
        for t in self.soft_t:
            if t["net"] == net or not self.t_rippable(t):
                continue
            st = Shape("seg", (t["x1"], t["y1"], t["x2"], t["y2"], t["w"] / 2), t["net"], t["layer"])
            need = max(own, clearance_of(t["net"])) + 0.02
            if not near(st.bbox, need):
                continue
            if any(ns.layer == st.layer and distance(ns, st) < need for ns in new_shapes):
                hit_t.append(t)
        for v in self.soft_v:
            if v["net"] == net or not self.v_rippable(v):
                continue
            need = max(own, clearance_of(v["net"])) + 0.02
            sv = Shape("circle", (v["x"], v["y"], v["d"] / 2), v["net"], "F.Cu")
            if not near(sv.bbox, need):
                continue
            if any(distance(ns, sv) < need for ns in new_shapes):
                hit_v.append(v)
        # a new via also needs hole-to-hole spacing to unlocked vias
        for v in self.soft_v:
            if v["net"] == net or not self.v_rippable(v) or any(v is h for h in hit_v):
                continue
            for nv in v_new:
                if math.hypot(v["x"] - nv["x"], v["y"] - nv["y"]) < (v["drill"] + nv["drill"]) / 2 + 0.27:
                    hit_v.append(v)
                    break
        return hit_t, hit_v

    def rip(self, hit_t, hit_v):
        ids_t = {id(t) for t in hit_t}
        ids_v = {id(v) for v in hit_v}
        self.soft_t = [t for t in self.soft_t if id(t) not in ids_t]
        self.soft_v = [v for v in self.soft_v if id(v) not in ids_v]


def inner_len(tracks):
    return sum(math.hypot(t["x2"] - t["x1"], t["y2"] - t["y1"]) for t in tracks if t["layer"] not in LAYERS)


def width_for(net):
    nc = netclass_of(net)
    return {"Power": 0.3, "CorePower": 0.15, "USB": 0.2, "RF": 0.34}.get(nc, 0.127)


def widths_for(net):
    """preferred width first, then the narrowest acceptable one"""
    w = width_for(net)
    w_min = 0.2 if netclass_of(net) in ("Power", "RF", "USB") else 0.127
    return [w] if w <= w_min else [w, w_min]


# Half-width of the square around the BGA centre where the finisher may not rip copper, so
# it cannot unpick the exactly-solved fan-out/escape. Shrink it (--no-rip=N) when the last
# few connections are balls whose only route out is through the ball-field perimeter.
NO_RIP_HALF = 5.2


def rr_main(src, dst, max_rounds=150, max_passes=4, time_budget=3600, inner=True, max_inner=1e9,
            no_rip_half=None):
    t0 = time.time()
    L = LS.build_from(src)
    tr, vi = tracks_vias(src)
    x0, y0 = LS.OX, LS.OY
    u = L.fp("U201")
    h = NO_RIP_HALF if no_rip_half is None else no_rip_half
    r = RRRouter(L, tr, vi, (x0, y0, x0 + LS.W, y0 + LS.H),
                 no_rip_box=(u.x - h, u.y - h, u.x + h, u.y + h))
    print(f"  no-rip box around U201: +/-{h} mm", flush=True)
    # On 4 layers In2 was the +3V3 plane, so wiring under the ball array would have cut the
    # copper feeding the balls and was kept out. On 8 layers the +3V3 plane is In3 and is
    # never routed, so In2/In5 are free here; the escape copper is an ordinary obstacle.
    ripped, failed = {}, set()
    best_open = None
    best = None                      # (missing connections, soft tracks, soft vias, open nets)
    npass, rnd = 0, -1
    vmax = 5
    while True:
        rnd += 1
        if time.time() - t0 > time_budget:
            print("  time budget reached", flush=True)
            break
        if rnd >= max_rounds:
            # new pass: forget rip counters and failures while the open count keeps falling
            open_now = sum(1 for c in components(r.items).values() if len(c) > 1)
            npass += 1
            if npass >= max_passes or (best_open is not None and open_now >= best_open):
                break
            best_open = open_now
            vmax += 2
            print(f"  --- pass {npass + 1}: {open_now} open nets", flush=True)
            ripped, failed, rnd = {}, set(), 0
        comps = components(r.items)
        missing = sum(len(c) - 1 for c in comps.values())
        if best is None or missing < best[0]:
            best = (missing, list(r.soft_t), list(r.soft_v),
                    sorted(n for n, c in comps.items() if len(c) > 1))
        todo = sorted(((n, list(c.values())) for n, c in comps.items() if len(c) > 1 and n not in failed),
                      key=lambda t: len(t[1]))
        if not todo:
            if failed:
                rnd = max_rounds - 1        # try another pass on the failures
                continue
            break
        net, cs = todo[0]
        cs.sort(key=len)
        sources = list(range(min(len(cs) - 1, 3)))
        stages = [LAYERS] + ([LAYERS3] if inner else [])
        done = False
        why = "no path around fixed copper"
        for lays in stages:
            tag = "" if lays == LAYERS else " (inner)"
            # 1. around all existing copper
            for si in sources:
                a = cs[si]
                b = [k for j, c in enumerate(cs) if j != si for k in c]
                for ww in widths_for(net):
                    res = r.checked_route(net, a, b, ww, layers=lays)
                    if res is not None and inner_len(res[0]) > max_inner:
                        res = None
                    if res is not None and not res[0] and not res[1]:
                        # The rasterised masks of the two components already touch at grid
                        # resolution while their exact outlines do not (a sub-RES gap at a
                        # pad edge). Adding nothing would leave the net open and spin this
                        # loop for ever, so treat it as a failure for this net.
                        res = None
                        why = "sub-grid gap at a pad edge; needs an explicit link"
                    if res is not None:
                        r.add(*res)
                        print(f"  [{rnd}] routed{tag} {net}: {len(res[0])} segments, {len(res[1])} vias",
                              flush=True)
                        done = True
                        break
                if done:
                    break
            if done:
                break
            # 2. around fixed copper, ripping up the unlocked wiring in the way
            if ripped.get(net, 0) >= 5:
                why = "rip limit"
                continue
            for si in sources:
                a = cs[si]
                b = [k for j, c in enumerate(cs) if j != si for k in c]
                res, (hit_t, hit_v) = r.relaxed_route(net, a, b, widths_for(net)[-1], layers=lays)
                if res is None or inner_len(res[0]) > max_inner:
                    continue
                victims = sorted({t["net"] for t in hit_t} | {v["net"] for v in hit_v})
                if len(victims) > vmax or any(ripped.get(vn, 0) >= 6 for vn in victims):
                    why = f"victims ripped too often: {victims}"
                    continue
                r.rip(hit_t, hit_v)
                r.soft_t += res[0]
                r.soft_v += res[1]
                for vn in victims:
                    ripped[vn] = ripped.get(vn, 0) + 1
                ripped[net] = ripped.get(net, 0) + 1
                r.rebuild()
                print(f"  [{rnd}] rip-up{tag} for {net}: removed {len(hit_t)} tracks / {len(hit_v)} vias of "
                      f"{victims}", flush=True)
                done = True
                break
            if done:
                break
        if not done:
            failed.add(net)
            print(f"  [{rnd}] FAILED {net} ({why})", flush=True)
    comps = components(r.items)
    missing = sum(len(c) - 1 for c in comps.values())
    soft_t, soft_v = r.soft_t, r.soft_v
    open_nets = sorted(n for n, c in comps.items() if len(c) > 1)
    if best is not None and best[0] < missing:
        print(f"  restoring the best state: {best[0]} missing connections instead of {missing}", flush=True)
        missing, soft_t, soft_v, open_nets = best
    json.dump(dict(tracks=soft_t, vias=soft_v, failed=open_nets, missing=missing, replace_unlocked=True),
              open(dst, "w"))
    print(f"done in {time.time() - t0:.0f} s: unlocked copper {len(soft_t)} tracks, {len(soft_v)} vias; "
          f"{missing} missing connections, open: {open_nets}", flush=True)


def main(src, dst):
    L = LS.build_from(src)
    tr, vi = tracks_vias(src)
    x0, y0 = LS.OX, LS.OY
    r = Router(L, tr, vi, (x0, y0, x0 + LS.W, y0 + LS.H))
    comps = components(r.items)
    todo = [(n, c) for n, c in comps.items() if len(c) > 1]
    print("open nets:", len(todo), sorted(n for n, _ in todo))
    failed = []
    for net, cs in sorted(todo, key=lambda t: len(t[1])):
        while True:
            comps = components(r.items)
            cs = list(comps.get(net, {}).values())
            if len(cs) < 2:
                break
            cs.sort(key=len)
            a = cs[0]
            b = [k for c in cs[1:] for k in c]
            res = None
            for ww in widths_for(net):
                res = r.checked_route(net, a, b, ww, tries=6)
                if res is not None:
                    break
            if res is None:
                failed.append(net)
                print("  FAILED", net)
                break
            t, v = res
            r.add(t, v)
            print(f"  routed {net}: {len(t)} segments, {len(v)} vias")
    json.dump(dict(tracks=r.new_tracks, vias=r.new_vias, failed=failed), open(dst, "w"))
    print("done:", len(r.new_tracks), "tracks,", len(r.new_vias), "vias, failed:", failed)


if __name__ == "__main__":
    if len(sys.argv) > 3 and sys.argv[3] == "--rr":
        nr = next((float(a.split("=", 1)[1]) for a in sys.argv[4:] if a.startswith("--no-rip=")), None)
        rr_main(sys.argv[1], sys.argv[2], inner="--no-inner" not in sys.argv[4:], no_rip_half=nr)
    else:
        main(sys.argv[1], sys.argv[2])
