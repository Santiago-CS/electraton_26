"""GND stitching vias between the outer-layer GND pours and the In1/In4/In6 GND planes, placed on a
grid wherever a via keeps clear of all other copper, part courtyards and the RF lines.

    python tools/stitch.py build/routed.kicad_pcb build/stitch.json
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import layout_steering as LS  # noqa: E402
import fanout  # noqa: E402
from geom import Shape  # noqa: E402
from kpcb_read import tracks_vias  # noqa: E402


def stitch_points(L, tracks, vias, pitch=5.08, via=(0.45, 0.2), margin=0.12):
    b = L.b
    b.tracks, b.vias = tracks, vias
    cop = fanout.Copper(b, b.outline)
    courts = [fp.courtyard() for fp in b.fps.values() if not getattr(fp, "board_only", False)]
    rf = [t for t in tracks if t["net"] in LS.RF_NETS]
    x0, y0 = LS.OX + 2.0, LS.OY + 2.0
    out = []
    nx, ny = int((LS.W - 4) / pitch) + 1, int((LS.H - 4) / pitch) + 1
    for i in range(nx):
        for j in range(ny):
            x, y = x0 + i * pitch, y0 + j * pitch
            if any(c[0] - 0.2 < x < c[2] + 0.2 and c[1] - 0.2 < y < c[3] + 0.2 for c in courts):
                continue
            if any(fanout.distance(Shape("circle", (x, y, via[0] / 2), None, "F.Cu"),
                                   Shape("seg", (t["x1"], t["y1"], t["x2"], t["y2"], t["w"] / 2),
                                         None, "F.Cu")) < 0.6 for t in rf):
                continue
            if not cop.via_ok(x, y, via[0], via[1], "GND", margin=margin):
                continue
            v = dict(x=x, y=y, d=via[0], drill=via[1], net="GND", locked=True)
            cop.add_via(v)
            out.append(v)
    return out


if __name__ == "__main__":
    import finish_router as FR
    src, dst = sys.argv[1], sys.argv[2]
    L = LS.build_from(src)
    tr, vi = tracks_vias(src)
    comps = FR.components(FR.copper_items(L, tr, vi))
    complete = sorted(n for n, c in comps.items() if len(c) == 1)
    open_nets = sorted(n for n, c in comps.items() if len(c) > 1)
    pts = stitch_points(L, tr, vi)
    json.dump(dict(vias=[dict(x=p["x"], y=p["y"], d=p["d"], drill=p["drill"]) for p in pts],
                   complete_nets=complete, open_nets=open_nets), open(dst, "w"))
    print("stitching vias:", len(pts), "| complete nets:", len(complete), "| open nets:", open_nets)
