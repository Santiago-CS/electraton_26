"""BGA escape routing, solved exactly with CP-SAT.

Under the RW612 the bottom layer only carries the dogbone vias (at the depopulated grid
sites). Traces of 0.127 mm (signals) / 0.15 mm (core rails) with 0.1 mm clearance fit
along any grid line or diagonal of the 0.5 mm ball grid as long as the grid points they
pass through are free, and two diagonals may not cross inside one grid cell. So the
escape is a set of vertex-disjoint paths on the extended grid (8-neighbourhood) from
each via to the edge of the region, 1.5 mm outside the ball array:

* signals are grouped by the side their destination lies on (N/E/S/W); a group is a
  unit-capacity flow, so different signals never share a grid point;
* +1V8 and VCORE are flows that may merge with themselves (a small tree per rail);
* leaving on a side other than the preferred one costs extra;
* B.Cu, In2.Cu and In5.Cu are all ordinary escape layers (8-layer stackup); a path that
  ends on an inner layer comes up through a via at the region edge, which costs a little
  (inner_cost / inner_via_cost) but carries no plane-crossing penalty;
* an escape that cannot be made at all costs a lot (reported, left to the autorouter).
"""
from ortools.sat.python import cp_model

from geom import Shape, distance


def solve(board, cop, pos, via_sites, net_dest, bga_centre, rows=15, cols=15, margin=3,
          share_nets=("+1V8", "VCORE"), plane_nets=("GND", "+3V3"), w_sig=0.127, w_pwr=0.15,
          clearance=0.1, side_penalty=12, inner_layers=("In2.Cu", "In5.Cu"), inner_cost=1.15,
          inner_via_cost=200, exit_via=(0.35, 0.2), time_limit=240, log=print):
    """via_sites: {(ri, c): net} for all vias of the fan-out (incl. outward ones).
    net_dest: {net: (x, y)} centroid of the net's other pads (board coords).
    Returns (tracks, vias, skipped)."""
    cx, cy = bga_centre
    nodes = [(ri, c) for ri in range(-margin, rows + margin) for c in range(1 - margin, cols + margin + 1)]
    nodeset = set(nodes)
    P = {v: pos(*v) for v in nodes}
    layers = ["B.Cu"] + list(inner_layers)

    def side_of(v):
        x, y = P[v]
        dx, dy = x - cx, y - cy
        if abs(dx) >= abs(dy):
            return "E" if dx > 0 else "W"
        return "S" if dy > 0 else "N"

    boundary = {v for v in nodes if v[0] in (-margin, rows + margin - 1) or
                v[1] in (1 - margin, cols + margin)}
    on_grid = {v: n for v, n in via_sites.items() if v in nodeset}
    grid_pts = {(round(P[v][0], 3), round(P[v][1], 3)): v for v in nodes}

    def offgrid(shape, grow, layer):
        out = []
        for t in cop.idx.near(shape, grow + 0.5):
            if t.layer != "B.Cu":
                continue
            is_via = t.kind == "circle" and t.tag == "via"
            if is_via:
                k = (round(t.a[0], 3), round(t.a[1], 3))
                if k in grid_pts and grid_pts[k] in on_grid:
                    continue            # on-grid vias: node occupancy handles them
            elif layer != "B.Cu":
                continue                # inner layer: only through-vias are obstacles
            out.append(t)
        return out

    hw = max(w_sig, w_pwr) / 2
    node_ok, edges = {}, {}
    dirs = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    for l in layers:
        for v in nodes:
            s = Shape("circle", (P[v][0], P[v][1], hw), None, "B.Cu")
            node_ok[l, v] = all(distance(s, t) >= clearance + 0.01 for t in offgrid(s, clearance + hw, l))
        edges[l] = []
        for v in nodes:
            for d in dirs:
                w = (v[0] + d[0], v[1] + d[1])
                if w not in nodeset:
                    continue
                s = Shape("seg", (P[v][0], P[v][1], P[w][0], P[w][1], hw), None, "B.Cu")
                if any(distance(s, t) < clearance + 0.01 for t in offgrid(s, clearance + hw, l)):
                    continue
                edges[l].append((v, w))
    exit_ok = set()
    if inner_layers:
        for v in boundary:
            if v in on_grid:
                continue
            if cop.via_ok(P[v][0], P[v][1], exit_via[0], exit_via[1], None, margin=0.03):
                exit_ok.add(v)
    log(f"  escape grid: {len(nodes)} nodes x {len(layers)} layers, "
        f"{sum(len(e) for e in edges.values())} directed edges, {len(exit_ok)} inner-layer exits")

    # ---- groups
    sig_sources = {}
    for v, n in on_grid.items():
        if n in plane_nets or n in share_nets:
            continue
        d = net_dest.get(n)
        if d is None:
            continue
        dx, dy = d[0] - cx, d[1] - cy
        sd = ("E" if dx > 0 else "W") if abs(dx) >= abs(dy) else ("S" if dy > 0 else "N")
        sig_sources.setdefault(sd, []).append((v, n))
    groups = [("sig", sd, srcs) for sd, srcs in sig_sources.items()]
    for n in share_nets:
        srcs = [(v, n) for v, nn in on_grid.items() if nn == n]
        if srcs:
            groups.append(("pwr", n, srcs))
    src_node = {}
    for gi, (_, _, srcs) in enumerate(groups):
        for v, n in srcs:
            src_node[v] = (gi, n)
    plane_nodes = {v for v, n in on_grid.items() if n in plane_nets}

    m = cp_model.CpModel()
    X, F, U, SK, SUP, SKIP = {}, {}, {}, {}, {}, {}
    for gi, (kind, key, srcs) in enumerate(groups):
        nsrc = len(srcs)
        own_power = key if kind == "pwr" else None
        glayers = layers if kind == "sig" else ["B.Cu"]
        for l in glayers:
            for (v, w) in edges[l]:
                if w in plane_nodes or v in plane_nodes:
                    continue
                if w in src_node and not (own_power and src_node[w] == (gi, own_power)):
                    continue
                if v in src_node and src_node[v][0] != gi:
                    continue
                if not (node_ok[l, v] or v in src_node) or not (node_ok[l, w] or w in src_node):
                    continue
                X[gi, l, v, w] = m.NewBoolVar("")
                if kind == "pwr":
                    F[gi, l, v, w] = m.NewIntVar(0, nsrc, "")
                    m.Add(F[gi, l, v, w] <= nsrc * X[gi, l, v, w])
                    m.Add(F[gi, l, v, w] >= X[gi, l, v, w])
            for v in nodes:
                U[gi, l, v] = m.NewBoolVar("")
            for v in boundary:
                if v in plane_nodes or v in src_node:
                    continue
                if l == "B.Cu" and node_ok[l, v]:
                    SK[gi, l, v] = m.NewIntVar(0, nsrc, "") if kind == "pwr" else m.NewBoolVar("")
                elif l != "B.Cu" and v in exit_ok and node_ok[l, v]:
                    SK[gi, l, v] = m.NewBoolVar("")
            for v, n in srcs:
                SUP[gi, l, v] = m.NewBoolVar("")
        for v, n in srcs:
            SKIP[gi, v] = m.NewBoolVar("")
            m.Add(sum(SUP[gi, l, v] for l in glayers) + SKIP[gi, v] == 1)

    out_e, in_e = {}, {}
    for (gi, l, v, w) in X:
        out_e.setdefault((gi, l, v), []).append((v, w))
        in_e.setdefault((gi, l, w), []).append((v, w))

    for gi, (kind, key, srcs) in enumerate(groups):
        srcset = {v for v, _ in srcs}
        glayers = layers if kind == "sig" else ["B.Cu"]
        for l in glayers:
            def flow(e, gi=gi, l=l, kind=kind):
                return F[gi, l, e[0], e[1]] if kind == "pwr" else X[gi, l, e[0], e[1]]
            for v in nodes:
                ins, outs = in_e.get((gi, l, v), []), out_e.get((gi, l, v), [])
                supply = SUP[gi, l, v] if v in srcset else 0
                m.Add(sum(flow(e) for e in ins) + supply == sum(flow(e) for e in outs) +
                      SK.get((gi, l, v), 0))
                inc = [X[gi, l, e[0], e[1]] for e in ins + outs]
                for xv in inc:
                    m.AddImplication(xv, U[gi, l, v])
                if (gi, l, v) in SK:
                    m.Add(U[gi, l, v] >= SK[gi, l, v]) if kind == "sig" else None
                if kind == "sig":
                    m.Add(sum(X[gi, l, e[0], e[1]] for e in ins) <= 1)
                    if v in srcset:
                        m.Add(sum(X[gi, l, e[0], e[1]] for e in ins) == 0)
                if not inc and (gi, l, v) not in SK and v not in srcset:
                    m.Add(U[gi, l, v] == 0)
    for l in layers:
        for v in nodes:
            us = [U[gi, l, v] for gi in range(len(groups)) if (gi, l, v) in U]
            if us:
                m.Add(sum(us) <= 1)
    # an inner-layer exit via is a through via: it occupies its node on every other layer
    for (gi, l, v), sk in SK.items():
        if l != "B.Cu":
            for l2 in layers:
                if l2 == l:
                    continue
                for gj in range(len(groups)):
                    if (gj, l2, v) in U:
                        m.AddImplication(sk, U[gj, l2, v].Not())
    # diagonals of one cell must not cross (per layer)
    for l in layers:
        for (ri, c) in nodes:
            a, b_, cc, d = (ri, c), (ri + 1, c + 1), (ri, c + 1), (ri + 1, c)
            if b_ not in nodeset:
                continue
            d1 = [X[k] for gi in range(len(groups)) for k in ((gi, l, a, b_), (gi, l, b_, a)) if k in X]
            d2 = [X[k] for gi in range(len(groups)) for k in ((gi, l, cc, d), (gi, l, d, cc)) if k in X]
            if d1 and d2:
                m.Add(sum(d1) + sum(d2) <= 1)

    obj = []
    for (gi, l, v, w), xv in X.items():
        L = 100 if (v[0] == w[0] or v[1] == w[1]) else 141
        obj.append(int(L * (inner_cost if l != "B.Cu" else 1)) * xv)
    for (gi, l, v), sk in SK.items():
        kind, key, _ = groups[gi]
        if kind == "sig" and side_of(v) != key:
            obj.append(side_penalty * 100 * sk)
        if l != "B.Cu":
            obj.append(inner_via_cost * sk)
    obj += [100000 * s for s in SKIP.values()]
    m.Minimize(sum(obj))
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = time_limit
    solver.parameters.num_workers = 8
    st = solver.Solve(m)
    if st not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        raise RuntimeError("escape model infeasible")
    skipped = [(v, src_node[v][1]) for (gi, v), s in SKIP.items() if solver.Value(s)]
    per_inner = {}
    for (gi, l, v), sk in SK.items():
        if l != "B.Cu" and solver.Value(sk):
            per_inner[l] = per_inner.get(l, 0) + 1
    n_inner = sum(per_inner.values())
    detail = ", ".join(f"{n} via {l}" for l, n in sorted(per_inner.items())) or "all on B.Cu"
    log(f"  escape: {solver.StatusName(st)}, {sum(len(g[2]) for g in groups)} vias, "
        f"{detail} ({n_inner} inner), {len(skipped)} left to the autorouter {skipped}")

    # ---- paths -> tracks (merge collinear steps) + exit vias for inner-layer escapes
    tracks, vias = [], []
    for gi, (kind, key, srcs) in enumerate(groups):
        width = w_pwr if kind == "pwr" else w_sig
        glayers = layers if kind == "sig" else ["B.Cu"]
        for l in glayers:
            used = [(v, w) for (g2, l2, v, w), xv in X.items() if g2 == gi and l2 == l and solver.Value(xv)]
            nxt = {}
            for v, w in used:
                nxt.setdefault(v, []).append(w)
            if kind == "sig":
                for v, n in srcs:
                    if not solver.Value(SUP[gi, l, v]):
                        continue
                    path = [v]
                    while path[-1] in nxt:
                        path.append(nxt[path[-1]][0])
                    tracks += _poly(path, P, width, n, l)
                    if l != "B.Cu":
                        e = path[-1]
                        vias.append(dict(x=P[e][0], y=P[e][1], d=exit_via[0], drill=exit_via[1], net=n,
                                         locked=True))
            else:
                for v, w in used:
                    tracks += _poly([v, w], P, width, key, l)
    return tracks, vias, skipped


def _poly(path, P, width, net, layer):
    out = []
    if len(path) < 2:
        return out
    pts = [path[0]]
    for k in range(1, len(path) - 1):
        a, b, c = path[k - 1], path[k], path[k + 1]
        if (b[0] - a[0], b[1] - a[1]) != (c[0] - b[0], c[1] - b[1]):
            pts.append(b)
    pts.append(path[-1])
    for a, b in zip(pts, pts[1:]):
        out.append(dict(layer=layer, x1=P[a][0], y1=P[a][1], x2=P[b][0], y2=P[b][1], w=width,
                        net=net, locked=True))
    return out
