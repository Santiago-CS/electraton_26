"""Board outline and mounting holes from the mechanical DXF (SolidWorks export).

    from dxf_outline import load_outline
    o = load_outline("docs/PCB_Volante_2026.DXF")

DXF y points up, KiCad y points down. Board-relative coordinates put (0, 0) at the
top-left corner of the outline's bounding box:  bx = x - xmin,  by = ymax - y.

Returned object:
  o.segments  Edge.Cuts primitives in board-relative mm:
                ("line", (x1, y1), (x2, y2))
                ("arc", start, mid, end)                 -- DXF arcs, exact
                ("poly", [p0, p1, ...])                  -- DXF splines, flattened to 5 um
  o.polygon   closed outline as a dense point list (board-relative), counter-clockwise
  o.holes     [(x, y, diameter)] board-relative
  o.width, o.height, o.axis_x (x of the mirror axis)
"""
import math

import ezdxf

TOL = 0.005          # spline flattening tolerance (mm)
JOIN = 0.01          # endpoint match tolerance when chaining (mm)


class Outline:
    pass


def _arc_pts(cx, cy, r, a0, a1, step_deg=2.0):
    """DXF arcs run counter-clockwise from a0 to a1 (degrees)."""
    if a1 < a0:
        a1 += 360.0
    n = max(2, int(math.ceil((a1 - a0) / step_deg)) + 1)
    return [(cx + r * math.cos(math.radians(a0 + (a1 - a0) * i / (n - 1))),
             cy + r * math.sin(math.radians(a0 + (a1 - a0) * i / (n - 1)))) for i in range(n)]


def load_outline(path):
    doc = ezdxf.readfile(path)
    raw = []            # (kind, data, dense point list in DXF coords)
    holes = []
    for e in doc.modelspace():
        t = e.dxftype()
        if t == "LINE":
            a, b = (e.dxf.start.x, e.dxf.start.y), (e.dxf.end.x, e.dxf.end.y)
            raw.append(("line", None, [a, b]))
        elif t == "ARC":
            c, r = e.dxf.center, e.dxf.radius
            pts = _arc_pts(c.x, c.y, r, e.dxf.start_angle, e.dxf.end_angle)
            raw.append(("arc", (c.x, c.y, r, e.dxf.start_angle, e.dxf.end_angle), pts))
        elif t == "SPLINE":
            pts = [(p.x, p.y) for p in e.flattening(TOL)]
            raw.append(("poly", None, pts))
        elif t == "CIRCLE":
            holes.append((e.dxf.center.x, e.dxf.center.y, 2 * e.dxf.radius))
    # chain the open primitives into one closed loop
    chain = [raw.pop(0)]
    loop = list(chain[0][2])
    while raw:
        end = loop[-1]
        for i, (k, d, pts) in enumerate(raw):
            if math.dist(pts[0], end) < JOIN:
                break
            if math.dist(pts[-1], end) < JOIN:
                pts = pts[::-1]
                break
        else:
            raise ValueError(f"outline is not closed near {end}")
        raw.pop(i)
        chain.append((k, d, pts))
        loop.extend(pts[1:])
    if math.dist(loop[0], loop[-1]) > JOIN:
        raise ValueError("outline does not close")
    loop.pop()
    xs = [p[0] for p in loop]
    ys = [p[1] for p in loop]
    # arcs can bulge past their sampled points only by < r(1-cos(1 deg)); use exact extremes
    xmin, xmax, ymin, ymax = min(xs), max(xs), min(ys), max(ys)
    for k, d, _ in chain:
        if k == "arc":
            cx, cy, r, a0, a1 = d
            a1 = a1 + 360.0 if a1 < a0 else a1
            for q in (0, 90, 180, 270, 360, 450, 540, 630):
                if a0 <= q <= a1:
                    x, y = cx + r * math.cos(math.radians(q)), cy + r * math.sin(math.radians(q))
                    xmin, xmax, ymin, ymax = min(xmin, x), max(xmax, x), min(ymin, y), max(ymax, y)

    def T(p):
        return (round(p[0] - xmin, 6), round(ymax - p[1], 6))

    o = Outline()
    o.dxf_origin = (xmin, ymax)
    o.width, o.height = xmax - xmin, ymax - ymin
    o.segments = []
    for k, d, pts in chain:
        if k == "line":
            o.segments.append(("line", T(pts[0]), T(pts[-1])))
        elif k == "arc":
            cx, cy, r, a0, a1 = d
            a1 = a1 + 360.0 if a1 < a0 else a1
            am = math.radians((a0 + a1) / 2)
            mid = (cx + r * math.cos(am), cy + r * math.sin(am))
            # keep the traversal direction of the loop
            o.segments.append(("arc", T(pts[0]), T(mid), T(pts[-1])))
        else:
            o.segments.append(("poly", [T(p) for p in pts]))
    poly = [T(p) for p in loop]
    # counter-clockwise in board coords (y down) is clockwise in the DXF; normalise
    area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(poly, poly[1:] + poly[:1]))
    if area < 0:
        poly.reverse()
    o.polygon = poly
    o.holes = [(round(x - xmin, 6), round(ymax - y, 6), dia) for x, y, dia in holes]
    # mirror axis: mean of the hole pairs' x (the DXF is symmetric about it)
    o.axis_x = sum(h[0] for h in o.holes) / len(o.holes) if o.holes else o.width / 2
    return o


if __name__ == "__main__":
    import sys
    o = load_outline(sys.argv[1] if len(sys.argv) > 1 else "docs/PCB_Volante_2026.DXF")
    print(f"outline {o.width:.3f} x {o.height:.3f} mm, {len(o.segments)} segments, "
          f"{len(o.polygon)} polygon points, axis x = {o.axis_x:.4f}")
    for h in o.holes:
        print(f"  hole at ({h[0]:.4f}, {h[1]:.4f}) d = {h[2]:.3f}")
