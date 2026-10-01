"""Place footprint reference designators on the silkscreen so they do not overlap pads,
other silkscreen or the board edge; references that do not fit anywhere near their part
are hidden (they stay on the F.Fab assembly layer via ${REFERENCE})."""
import math

from shapely.geometry import Point, Polygon

from geom import Shape, SpatialIndex, distance
from pcbgen import rot_pt
from sexp import find, find_all


def text_extent(text, size, thick):
    # KiCad stroke font (newstroke) glyph outline, measured with pcbnew's
    # GetEffectiveTextShape(): ~0.93 x size advance per capital/digit
    return len(text) * size * 0.95 + thick, size + thick + 0.05


def silk_shapes(fp):
    """F.SilkS graphics of a footprint as board-coordinate shapes (excluding text)."""
    out = []
    for g in fp.node:
        if not isinstance(g, list) or not g or g[0] not in ("fp_line", "fp_rect", "fp_circle",
                                                              "fp_arc", "fp_poly"):
            continue
        lay = find(g, "layer")
        if not lay or lay[1] != "F.SilkS":
            continue
        st = find(g, "stroke")
        w = float(find(st, "width")[1]) if st and find(st, "width") else 0.12
        hw = w / 2

        def P(node):
            x, y = rot_pt(float(node[1]), float(node[2]), fp.rot)
            return fp.x + x, fp.y + y
        if g[0] == "fp_line":
            a, b = P(find(g, "start")), P(find(g, "end"))
            out.append(Shape("seg", (a[0], a[1], b[0], b[1], hw), None, "silk"))
        elif g[0] == "fp_rect":
            a, b = P(find(g, "start")), P(find(g, "end"))
            xs, ys = sorted((a[0], b[0])), sorted((a[1], b[1]))
            for s in (((xs[0], ys[0]), (xs[1], ys[0])), ((xs[1], ys[0]), (xs[1], ys[1])),
                      ((xs[1], ys[1]), (xs[0], ys[1])), ((xs[0], ys[1]), (xs[0], ys[0]))):
                out.append(Shape("seg", (s[0][0], s[0][1], s[1][0], s[1][1], hw), None, "silk"))
        elif g[0] == "fp_circle":
            c, e = P(find(g, "center")), P(find(g, "end"))
            r = math.hypot(e[0] - c[0], e[1] - c[1])
            out.append(Shape("box", (c[0] - r - hw, c[1] - r - hw, c[0] + r + hw, c[1] + r + hw),
                             None, "silk"))
        elif g[0] == "fp_arc":
            pts = [P(find(g, k)) for k in ("start", "mid", "end")]
            for a, b in zip(pts, pts[1:]):
                out.append(Shape("seg", (a[0], a[1], b[0], b[1], hw), None, "silk"))
        elif g[0] == "fp_poly":
            pts = [P(xy) for xy in find(g, "pts")[1:]]
            for a, b in zip(pts, pts[1:] + pts[:1]):
                out.append(Shape("seg", (a[0], a[1], b[0], b[1], hw), None, "silk"))
    return out


def place_references(board, size_small=0.8, size_big=1.0, thick=0.15, gap=0.12, log=print):
    idx = SpatialIndex(1.0)
    edge = Polygon(board.outline)
    for fp in board.fps.values():
        for p in fp.pads:
            if not any(l.endswith(".Cu") or l == "*.Cu" for l in p.layers):
                continue
            bx = fp.pad_box(p)
            idx.add(Shape("box", (bx[0] - 0.05, bx[1] - 0.05, bx[2] + 0.05, bx[3] + 0.05), None, "silk"))
        for s in silk_shapes(fp):
            idx.add(s)
    for v in board.vias:            # tented vias: allowed under silk, but keep text readable
        pass
    # a reference printed inside another large part's outline reads as that part's name
    bigs = []
    for f in board.fps.values():
        c = f.courtyard()
        if (c[2] - c[0]) * (c[3] - c[1]) > 12:
            bigs.append((f.ref, (c[0] + 0.25, c[1] + 0.25, c[2] - 0.25, c[3] - 0.25)))
    placed, hidden = 0, []
    order = sorted(board.fps.values(), key=lambda f: -(f.courtyard()[2] - f.courtyard()[0]) *
                   (f.courtyard()[3] - f.courtyard()[1]))
    for fp in order:
        if getattr(fp, "board_only", False):
            fp.ref_override = dict(hide=True)
            continue
        cy = fp.courtyard()
        area = (cy[2] - cy[0]) * (cy[3] - cy[1])
        sizes = [size_big, size_small, 0.6] if area > 12 else [size_small, 0.6]
        best = None
        for size in sizes:
          if best is not None:
            break
          tw, th = text_extent(fp.ref, size, thick if size > 0.7 else 0.1)
          cx, cyy = (cy[0] + cy[2]) / 2, (cy[1] + cy[3]) / 2
          best = None
          cands = []
          for off in ((0.05, 0.3, 0.6, 1.0, 1.6, 2.4) if area > 12 else (0.05, 0.3, 0.6, 1.0)):
              for vertical in (False, True):
                  w, h = (th, tw) if vertical else (tw, th)
                  cands += [
                      (cx, cy[1] - off - h / 2, vertical),      # above
                      (cx, cy[3] + off + h / 2, vertical),      # below
                      (cy[0] - off - w / 2, cyy, vertical),     # left
                      (cy[2] + off + w / 2, cyy, vertical),     # right
                      (cy[0] + w / 2, cy[1] - off - h / 2, vertical),
                      (cy[2] - w / 2, cy[1] - off - h / 2, vertical),
                      (cy[0] + w / 2, cy[3] + off + h / 2, vertical),
                      (cy[2] - w / 2, cy[3] + off + h / 2, vertical),
                  ]
          if area > 12:   # big parts: inside the body outline is fine too (e.g. between the
              # pins and the back wall of a connector), nearest to the centre first
              inside = []
              nx, ny = int((cy[2] - cy[0]) / 0.4), int((cy[3] - cy[1]) / 0.4)
              for i in range(nx + 1):
                  for j in range(ny + 1):
                      x, y = cy[0] + i * 0.4, cy[1] + j * 0.4
                      for vertical in (False, True):
                          w, h = (th, tw) if vertical else (tw, th)
                          if cy[0] <= x - w / 2 and x + w / 2 <= cy[2] and cy[1] <= y - h / 2 and y + h / 2 <= cy[3]:
                              inside.append((math.hypot(x - cx, y - cyy), (x, y, vertical)))
              cands = [(cx, cyy, False)] + [c for _, c in sorted(inside)] + cands
          for k, (x, y, vertical) in enumerate(cands):
              w, h = (th, tw) if vertical else (tw, th)
              box = (x - w / 2, y - h / 2, x + w / 2, y + h / 2)
              poly = Polygon([(box[0], box[1]), (box[2], box[1]), (box[2], box[3]), (box[0], box[3])])
              if not edge.contains(poly) or edge.exterior.distance(poly) < 0.3:
                  continue
              if any(r != fp.ref and box[0] < c[2] and box[2] > c[0] and box[1] < c[3] and box[3] > c[1]
                     for r, c in bigs):
                  continue
              s = Shape("box", box, None, "silk")
              if any(distance(s, t) < gap for t in idx.near(s, gap + 0.2)):
                  continue
              best = (x, y, vertical, box, size)
              break
        if best is None:
            fp.ref_override = dict(hide=True)
            hidden.append(fp.ref)
            continue
        x, y, vertical, box, size = best
        idx.add(Shape("box", box, None, "silk"))
        # local position (footprint frame) + absolute text angle
        lx, ly = rot_pt(x - fp.x, y - fp.y, -fp.rot)
        fp.ref_override = dict(x=lx, y=ly, angle=90 if vertical else 0, size=size,
                               thick=thick if size > 0.7 else 0.1)
        placed += 1
    log(f"  silkscreen: {placed} references placed, {len(hidden)} hidden: {' '.join(sorted(hidden))}")
    return hidden
