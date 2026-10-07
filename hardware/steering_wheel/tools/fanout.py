"""Pre-routing helpers: a copper clearance index, the BGA dogbone fan-out (ball -> via
in an empty grid site, solved as a small CP-SAT assignment), and plane fan-out vias
for every SMD pad on a plane net (GND on In1/In4/In6, +3V3 on In3).

All coordinates are absolute board coordinates in mm (KiCad page coordinates)."""
import math
import os
import re

from shapely.geometry import Point, Polygon

from geom import Shape, SpatialIndex, distance
from pcbgen import rot_pt
from board_rules import NETCLASSES, netclass_of

SIG = ("F.Cu", "B.Cu")                 # outer copper: SMD pads live here
INNER = ("In1.Cu", "In2.Cu", "In3.Cu", "In4.Cu", "In5.Cu", "In6.Cu")
ROWS = "ABCDEFGHJKLMNPR"


def class_clearance(net):
    if not net:
        return 0.1
    return NETCLASSES[netclass_of(net)][0]


class Copper:
    """Copper on the routing layers plus drilled holes, for clearance checks."""

    def __init__(self, board, outline, inner=False):
        self.b = board
        self.inner = inner          # also index vias / through-hole pads on the inner layers
        self.idx = SpatialIndex(0.5)
        self.holes = SpatialIndex(0.5)
        self.edge = Polygon(outline)
        for fp in board.fps.values():
            for p in fp.pads:
                self.add_pad(fp, p)
        for t in board.tracks:
            self.add_track(t)
        for v in board.vias:
            self.add_via(v)

    # ---------------------------------------------------------------- shapes
    @staticmethod
    def pad_shape(fp, p, layer):
        x, y = fp.pad_xy(p)
        net = fp.pad_net.get(p.num)
        if p.shape == "circle":
            return Shape("circle", (x, y, p.w / 2), net, layer, tag=(fp.ref, p.num))
        bx = fp.pad_box(p)
        if p.shape == "oval":
            w, h = bx[2] - bx[0], bx[3] - bx[1]
            if w >= h:
                r = h / 2
                return Shape("seg", (bx[0] + r, y, bx[2] - r, y, r), net, layer, tag=(fp.ref, p.num))
            r = w / 2
            return Shape("seg", (x, bx[1] + r, x, bx[3] - r, r), net, layer, tag=(fp.ref, p.num))
        return Shape("box", bx, net, layer, tag=(fp.ref, p.num))

    @staticmethod
    def pad_conn_shape(fp, p, layer):
        """A shape *inscribed* in the pad copper, for connectivity and routing targets.

        pad_shape() returns the axis-aligned bounding box, which is right for clearance
        (it never under-states the copper) but over-states it at the corners of a rotated
        or rounded pad. Aiming a track at such a corner produces copper that this model
        calls connected and KiCad's DRC calls unconnected. A capsule along the pad's long
        axis is inside the real pad for any rotation and any corner rounding, so reaching
        it means really touching the pad.
        """
        x, y = fp.pad_xy(p)
        net = fp.pad_net.get(p.num)
        if p.shape == "circle":
            return Shape("circle", (x, y, p.w / 2), net, layer, tag=(fp.ref, p.num))
        long_, short = max(p.w, p.h), min(p.w, p.h)
        r = short / 2
        half = (long_ - short) / 2
        a = fp.rot + p.lrot
        dx, dy = rot_pt(half, 0, a) if p.w >= p.h else rot_pt(0, half, a)
        if half <= 1e-9:
            return Shape("circle", (x, y, r), net, layer, tag=(fp.ref, p.num))
        return Shape("seg", (x - dx, y - dy, x + dx, y + dy, r), net, layer, tag=(fp.ref, p.num))

    @staticmethod
    def pad_poly(fp, p):
        """The pad copper as an exact shapely polygon, rotation and corner rounding included.

        pad_shape() (bounding box) and pad_conn_shape() (inscribed capsule) bracket the real
        pad; this is the pad itself, for deciding whether copper actually touches it.
        """
        from shapely.affinity import rotate as _rotate
        from shapely.geometry import Point as _Point, box as _box
        x, y = fp.pad_xy(p)
        if p.shape == "circle":
            return _Point(x, y).buffer(p.w / 2, quad_segs=16)
        if p.shape == "oval":
            r = min(p.w, p.h) / 2
            half = (max(p.w, p.h) - min(p.w, p.h)) / 2
            g = _box(-half, -r, half, r).buffer(r, quad_segs=16) if p.w >= p.h else \
                _box(-r, -half, r, half).buffer(r, quad_segs=16)
        elif p.shape == "roundrect":
            r = min(p.w, p.h) * (p.rratio or 0.0)
            g = _box(-p.w / 2 + r, -p.h / 2 + r, p.w / 2 - r, p.h / 2 - r).buffer(r, quad_segs=8) \
                if r > 1e-9 else _box(-p.w / 2, -p.h / 2, p.w / 2, p.h / 2)
        else:
            g = _box(-p.w / 2, -p.h / 2, p.w / 2, p.h / 2)
        # KiCad's y axis points down, so a positive KiCad angle turns clockwise on screen;
        # that is a negative rotation in shapely's convention. rot_pt() (used for pad
        # positions) already encodes the same sign.
        g = _rotate(g, -(fp.rot + p.lrot), origin=(0, 0), use_radians=False)
        from shapely.affinity import translate as _translate
        return _translate(g, x, y)

    def add_pad(self, fp, p):
        cl = p.copper_layers(list(SIG))
        if p.kind == "np_thru_hole":
            x, y = fp.pad_xy(p)
            r = max(p.drill) / 2 if p.drill else p.w / 2
            self.holes.add(Shape("circle", (x, y, r), None, "*", tag=(fp.ref, p.num)))
            for l in (SIG + INNER if self.inner else SIG):   # hole edge keeps copper away
                self.idx.add(Shape("circle", (x, y, r), None, l, tag=(fp.ref, "NPTH")))
            return
        for l in cl:
            self.idx.add(self.pad_shape(fp, p, l))
        if self.inner and p.kind == "thru_hole":
            for l in INNER:
                self.idx.add(self.pad_shape(fp, p, l))
        if p.kind == "thru_hole" and p.drill:
            x, y = fp.pad_xy(p)
            self.holes.add(Shape("circle", (x, y, max(p.drill) / 2), None, "*", tag=(fp.ref, p.num)))

    def add_track(self, t):
        s = Shape("seg", (t["x1"], t["y1"], t["x2"], t["y2"], t["w"] / 2), t["net"], t["layer"], tag="trk")
        self.idx.add(s)

    def add_via(self, v):
        for l in (SIG + INNER if self.inner else SIG):
            self.idx.add(Shape("circle", (v["x"], v["y"], v["d"] / 2), v["net"], l, tag="via"))
        self.holes.add(Shape("circle", (v["x"], v["y"], v["drill"] / 2), None, "*", tag="via"))

    # ---------------------------------------------------------------- checks
    def clear(self, s, margin=0.02, extra=0.0):
        """True if shape s keeps the class clearance (+margin) to all other-net copper."""
        mine = class_clearance(s.net)
        for t in self.idx.near(s, 0.8):
            if t.layer != s.layer:
                continue
            if s.net and t.net == s.net:
                continue
            need = max(mine, class_clearance(t.net)) + margin + extra
            if distance(s, t) < need:
                return False
        return True

    def hole_ok(self, x, y, r, gap=0.27):
        s = Shape("circle", (x, y, r), None, "*")
        for t in self.holes.near(s, gap + 0.5):
            if distance(s, t) < gap:
                return False
        return True

    def edge_ok(self, x, y, r, gap=0.35):
        p = Point(x, y)
        return self.edge.contains(p) and self.edge.exterior.distance(p) >= r + gap

    def via_ok(self, x, y, d, drill, net, margin=0.02):
        if not self.edge_ok(x, y, d / 2):
            return False
        if not self.hole_ok(x, y, drill / 2):
            return False
        for l in SIG + INNER:                    # inner layers: pre-routed escape tracks
            if not self.clear(Shape("circle", (x, y, d / 2), net, l), margin):
                return False
        return True

    def seg_ok(self, x1, y1, x2, y2, w, net, layer="F.Cu", margin=0.02):
        return self.clear(Shape("seg", (x1, y1, x2, y2, w / 2), net, layer), margin)


# ======================================================================== BGA fan-out
def bga_fanout(board, cop, ref, plane_nets=("GND", "+3V3"), skip_nets=(), via=(0.35, 0.2),
               w_sig=0.127, w_pwr=0.2, log=print, share_nets=("+1V8", "VCORE"), trapped_rows=(14,),
               cache_path=None):
    """Give every ball that cannot escape on the top layer a via in a neighbouring empty
    grid site (orthogonal or diagonal), sharing vias between balls of the same power net.
    Outer-ring balls on plane nets get their via just outside the array when there is room.
    Returns (vias, tracks) added to the board."""
    from ortools.sat.python import cp_model

    fp = board.fps[ref]
    balls = {}
    for p in fp.pads:
        m = re.match(r"([A-Z])(\d+)$", p.num)
        if m:
            balls[(ROWS.index(m.group(1)), int(m.group(2)))] = p
    a1 = fp.pad_xy(balls[(0, 1)])
    b1 = fp.pad_xy(balls[(1, 1)])
    a2 = fp.pad_xy(balls[(0, 2)])
    rv = (b1[0] - a1[0], b1[1] - a1[1])
    cv = (a2[0] - a1[0], a2[1] - a1[1])
    nr, nc = len(ROWS), 15

    def pos(ri, c):
        return (a1[0] + ri * rv[0] + (c - 1) * cv[0], a1[1] + ri * rv[1] + (c - 1) * cv[1])

    def net_of(k):
        return fp.pad_net.get(balls[k].num) if k in balls else None

    def outer(k):
        return k[0] in (0, nr - 1) or k[1] in (1, nc)

    sites = {(ri, c) for ri in range(nr) for c in range(1, nc + 1)} - set(balls)
    need = []
    for k, p in balls.items():
        n = net_of(k)
        if not n or n in skip_nets:
            continue
        # outer-ring core-rail balls escape on the top layer (to their decoupling caps),
        # except in rows whose top-layer escape is closed (RF corridor above row R)
        if not outer(k) or n in plane_nets or (n in share_nets and k[0] in trapped_rows):
            need.append(k)

    # candidate (ball, site) pairs
    cand = {}
    for k in need:
        n = net_of(k)
        opts = []
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == dc == 0:
                    continue
                s = (k[0] + dr, k[1] + dc)
                if s in sites:
                    opts.append((s, 1.0 if dr == 0 or dc == 0 else 1.35))
        if outer(k) and (n in plane_nets or n in share_nets):
            outs = []
            if k[0] == 0:
                outs.append((-1, k[1]))
            if k[0] == nr - 1:
                outs.append((nr, k[1]))
            if k[1] == 1:
                outs.append((k[0], 0))
            if k[1] == nc:
                outs.append((k[0], nc + 1))
            for s in outs:
                trapped = k[0] in trapped_rows
                # via right outside the ball (blocks the first escape ring on B.Cu) or
                # three pitches out, beyond the B.Cu escape region, on a top-layer stub
                cands = [(s, 0.8 if trapped else 2.5)]
                if not trapped:
                    d = (s[0] - k[0], s[1] - k[1])
                    cands.append(((k[0] + 3 * d[0], k[1] + 3 * d[1]), 0.7))
                for s2, cost in cands:
                    x, y = pos(*s2)
                    if cop.via_ok(x, y, via[0], via[1], n) and \
                            cop.seg_ok(*pos(*k), x, y, w_pwr, n):
                        opts.append((s2, cost))
        # signals: lean towards the side the net has to go to
        if n not in plane_nets and n not in share_nets:
            dest = [board.fps[r].pad_xy(board.fps[r].pads_by_num(pn)[0])
                    for r, pn in _net_pins(board, n) if r != ref]
            if dest:
                dx = sum(d[0] for d in dest) / len(dest)
                dy = sum(d[1] for d in dest) / len(dest)
                bx, by = pos(*k)
                d0 = math.hypot(dx - bx, dy - by)
                opts = [(s, c + 0.25 * (math.hypot(dx - pos(*s)[0], dy - pos(*s)[1]) - d0))
                        for s, c in opts]
        if not opts:
            log(f"  !! ball {ROWS[k[0]]}{k[1]} ({n}) has no free neighbour site")
            continue
        cand[k] = opts

    nets_at = {}
    for k, opts in cand.items():
        for s, _ in opts:
            nets_at.setdefault(s, set()).add(net_of(k))

    m = cp_model.CpModel()
    x = {(k, s): m.NewBoolVar(f"x{k}{s}") for k, opts in cand.items() for s, _ in opts}
    y = {(s, n): m.NewBoolVar(f"y{s}{n}") for s, ns in nets_at.items() for n in ns}
    # power balls may instead be tied on the top layer to an orthogonal neighbour of the
    # same net that has its own via (saves vias under the BGA, i.e. B.Cu routing room)
    tie_nets = set(plane_nets) | set(share_nets)
    u = {}
    for k in cand:
        if net_of(k) not in tie_nets:
            continue
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            k2 = (k[0] + dr, k[1] + dc)
            if k2 in cand and net_of(k2) == net_of(k):
                u[(k, k2)] = m.NewBoolVar(f"u{k}{k2}")
    direct = {k: sum(x[(k, s)] for s, _ in opts) for k, opts in cand.items()}
    for k, opts in cand.items():
        m.Add(direct[k] + sum(v for (a, _), v in u.items() if a == k) == 1)
        for s, _ in opts:
            m.AddImplication(x[(k, s)], y[(s, net_of(k))])
    for (a, b_), v in u.items():
        m.Add(direct[b_] >= 1).OnlyEnforceIf(v)
    for s, ns in nets_at.items():
        m.AddAtMostOne(y[(s, n)] for n in ns)
    # two diagonal dogbones of different nets must not cross inside one 2x2 cell
    diag = {}
    for (k, s) in x:
        if abs(k[0] - s[0]) == 1 and abs(k[1] - s[1]) == 1:
            cell = (min(k[0], s[0]), min(k[1], s[1]))
            d = 0 if (k[0] - s[0]) == (k[1] - s[1]) else 1
            diag.setdefault((cell, d), []).append((k, s))
    for (cell, d), lst in diag.items():
        other = diag.get((cell, 1 - d), [])
        if d == 1:
            continue
        for a in lst:
            for b_ in other:
                if net_of(a[0]) != net_of(b_[0]):
                    m.AddBoolOr([x[a].Not(), x[b_].Not()])
    obj = []
    for k, opts in cand.items():
        for s, c in opts:
            obj.append(int(round(c * 100)) * x[(k, s)])
    obj += [60 * v for v in y.values()]
    obj += [45 * v for v in u.values()]
    m.Minimize(sum(obj))
    # deterministic single-thread solve, cached on the exact candidate set so that the
    # (slow) escape solve that depends on it can be cached too
    import hashlib
    import json as _json
    sig = hashlib.sha1(_json.dumps(sorted((list(k), [[list(s_), c] for s_, c in o])
                                          for k, o in cand.items())).encode()).hexdigest()
    sol = None
    if cache_path and os.path.exists(cache_path):
        c = _json.load(open(cache_path))
        if c.get("sig") == sig:
            sol = c
    if sol is None:
        solver = cp_model.CpSolver()
        solver.parameters.max_time_in_seconds = 60
        solver.parameters.num_workers = 1
        solver.parameters.random_seed = 1
        st = solver.Solve(m)
        if st not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            raise RuntimeError("BGA fan-out assignment infeasible")
        log(f"  BGA fan-out: {solver.StatusName(st)}, {len(cand)} balls")
        sol = dict(sig=sig,
                   y=[[list(s_), n] for (s_, n), v in y.items() if solver.Value(v)],
                   x=[[list(k), list(s_)] for (k, s_), v in x.items() if solver.Value(v)],
                   u=[[list(a), list(b_)] for (a, b_), v in u.items() if solver.Value(v)])
        if cache_path:
            _json.dump(sol, open(cache_path, "w"))
    else:
        log(f"  BGA fan-out: cached solution, {len(cand)} balls")

    vias, tracks = [], []
    used = {}
    for s_, n in sol["y"]:
        s_ = tuple(s_)
        xx, yy = pos(*s_)
        vd = board.via(xx, yy, via[0], via[1], n, locked=True)
        cop.add_via(vd)
        used[s_] = n
        vias.append(vd)
    for k, s_ in sol["x"]:
        k, s_ = tuple(k), tuple(s_)
        n = net_of(k)
        w = w_pwr if n in plane_nets or n in ("+1V8", "VCORE") else w_sig
        t = board.track("F.Cu", pos(*k), pos(*s_), w, n, locked=True)
        cop.add_track(t)
        tracks.append(t)
    ties = 0
    for a, b_ in sol["u"]:
        a, b_ = tuple(a), tuple(b_)
        n = net_of(a)
        t = board.track("F.Cu", pos(*a), pos(*b_), w_pwr, n, locked=True)
        cop.add_track(t)
        tracks.append(t)
        ties += 1
    log(f"  BGA fan-out: {len(vias)} vias, {len(tracks) - ties} dogbones, {ties} ball-to-ball ties")
    return vias, tracks, used, pos


def _net_pins(board, net):
    out = []
    for r, fp in board.fps.items():
        for p in fp.pads:
            if fp.pad_net.get(p.num) == net:
                out.append((r, p.num))
    return out


# ======================================================================== plane fan-out
def _ray_exit(bx, x, y, ux, uy):
    """Distance from (x, y) inside box bx along (ux, uy) to the box edge."""
    ts = []
    if ux > 1e-9:
        ts.append((bx[2] - x) / ux)
    elif ux < -1e-9:
        ts.append((bx[0] - x) / ux)
    if uy > 1e-9:
        ts.append((bx[3] - y) / uy)
    elif uy < -1e-9:
        ts.append((bx[1] - y) / uy)
    return min(ts) if ts else 0.0


def plane_fanout(board, cop, nets=("GND", "+3V3"), skip_refs=(), via=(0.4, 0.2), small=(0.35, 0.2),
                 log=print, max_reach=2.2, avoid=(), avoid_segs=()):
    """Connect every SMD pad on a plane net to its plane with a via (exposed pads get an
    array of vias inside the pad). avoid: [(x0, y0, x1, y1, penalty)] areas where vias
    would block escape routing (e.g. the BGA halo)."""
    added, failed = 0, []
    courts = {r: fp.courtyard() for r, fp in board.fps.items()}
    for ref in sorted(board.fps):
        fp = board.fps[ref]
        if ref in skip_refs:
            continue
        cx, cy = fp.x, fp.y
        smd = [p for p in fp.pads if p.kind == "smd" and "F.Cu" in p.layers]
        for p in smd:
            net = fp.pad_net.get(p.num)
            if net not in nets:
                continue
            px, py = fp.pad_xy(p)
            bx = fp.pad_box(p)
            w, h = bx[2] - bx[0], bx[3] - bx[1]
            central = abs(p.lx) < 0.15 and abs(p.ly) < 0.15 and len(smd) >= 5
            if central and min(w, h) >= 0.3:
                # exposed pad: via array inside
                n_here = 0
                for vd, vdr in (via, small):
                    pitch = max(vd + 0.4, 0.9)
                    m = vd / 2 + 0.08
                    nx = max(1, int((w - 2 * m) / pitch) + 1) if w >= vd else 0
                    ny = max(1, int((h - 2 * m) / pitch) + 1) if h >= vd else 0
                    if not nx or not ny:
                        continue
                    for i in range(nx):
                        for j in range(ny):
                            vx = px - (nx - 1) * pitch / 2 + i * pitch
                            vy = py - (ny - 1) * pitch / 2 + j * pitch
                            if cop.via_ok(vx, vy, vd, vdr, net):
                                v = board.via(vx, vy, vd, vdr, net, locked=True)
                                cop.add_via(v)
                                n_here += 1
                    if n_here:
                        break
                if n_here:
                    added += n_here
                    continue
            best = None
            tw = max(0.15, min(0.3, 0.8 * min(w, h)))
            # reuse a same-net via close by
            probe = Shape("circle", (px, py, max_reach), net, "F.Cu")
            for t in cop.idx.near(probe, 0.1):
                if t.tag != "via" or t.net != net or t.layer != "F.Cu":
                    continue
                vx, vy = t.a[0], t.a[1]
                d = math.hypot(vx - px, vy - py)
                if d > max_reach or d < 1e-6:
                    continue
                if cop.seg_ok(px, py, vx, vy, tw, net):
                    sc = d - 0.25
                    if best is None or sc < best[0]:
                        best = (sc, vx, vy, None)
            # outward direction from the footprint centre
            ox, oy = px - cx, py - cy
            on = math.hypot(ox, oy)
            for vd, vdr in (via, small):
                for k in range(24):
                    a = 2 * math.pi * k / 24
                    ux, uy = math.cos(a), math.sin(a)
                    d0 = _ray_exit(bx, px, py, ux, uy) + vd / 2 + 0.06
                    dd = d0
                    while dd <= d0 + max_reach:
                        vx, vy = px + ux * dd, py + uy * dd
                        vx, vy = round(vx / 0.025) * 0.025, round(vy / 0.025) * 0.025
                        if cop.via_ok(vx, vy, vd, vdr, net) and cop.seg_ok(px, py, vx, vy, tw, net):
                            sc = dd
                            if on > 1e-6:
                                sc += 0.35 * (1 - (ux * ox + uy * oy) / on)
                            for r2, c2 in courts.items():
                                if c2[0] < vx < c2[2] and c2[1] < vy < c2[3]:
                                    sc += 0.5 if r2 != ref else 0.25
                            for ax0, ay0, ax1, ay1, pen in avoid:
                                if ax0 < vx < ax1 and ay0 < vy < ay1:
                                    sc += pen
                            for sx0, sy0, sx1, sy1, rr, pen in avoid_segs:
                                from geom import pt_seg
                                if pt_seg(vx, vy, sx0, sy0, sx1, sy1) < rr:
                                    sc += pen
                            if vd < via[0]:
                                sc += 0.4
                            if best is None or sc < best[0]:
                                best = (sc, vx, vy, (vd, vdr))
                            break
                        dd += 0.05
                if best is not None and best[3] is not None and best[3][0] == via[0]:
                    break
            if best is None:
                failed.append((ref, p.num, net))
                continue
            _, vx, vy, vv = best
            if vv is not None:
                v = board.via(vx, vy, vv[0], vv[1], net, locked=True)
                cop.add_via(v)
                added += 1
            t = board.track("F.Cu", (px, py), (vx, vy), tw, net, locked=True)
            cop.add_track(t)
    log(f"  plane fan-out: {added} vias, {len(failed)} pads without a via: {failed}")
    return failed
