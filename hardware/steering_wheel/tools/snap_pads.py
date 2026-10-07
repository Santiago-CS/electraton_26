"""Find track endpoints that stop just short of their own pad, and write the corrections.

The finishing router's pull-tight can leave an endpoint a few microns outside a pad while
the trace body still clips its corner. The copper overlaps, so this model and the autorouter
both call the net routed, but KiCad's connectivity wants the endpoint inside the pad and
reports the pad unconnected. Extending the endpoint along its own segment fixes it, and is
only proposed when the longer segment still keeps clearance.

    python tools/snap_pads.py board.kicad_pcb snaps.json [max_reach_mm]
    python tools/kicad/apply_snaps.py board.kicad_pcb snaps.json out.kicad_pcb
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fanout  # noqa: E402
import layout_steering as LS  # noqa: E402
from kpcb_read import tracks_vias  # noqa: E402
from shapely.geometry import Point  # noqa: E402

src, dst = sys.argv[1:3]
REACH = float(sys.argv[3]) if len(sys.argv) > 3 else 0.4
INSIDE = 0.03          # how far inside the pad the endpoint must end up

L = LS.build_from(src)
tr, vi = tracks_vias(src)
L.b.tracks, L.b.vias = tr, vi
cop = fanout.Copper(L.b, L.b.outline, inner=True)

pads = {}
for ref, fp in L.b.fps.items():
    for p in fp.pads:
        net = fp.pad_net.get(p.num)
        lay = p.copper_layers(list(fanout.SIG))
        if not net or not lay:
            continue
        pads.setdefault(net, []).append((ref, p.num, fanout.Copper.pad_poly(fp, p), lay))

snaps = []
for t in tr:
    for (ex, ey), (ox, oy), which in (((t["x1"], t["y1"]), (t["x2"], t["y2"]), "start"),
                                      ((t["x2"], t["y2"]), (t["x1"], t["y1"]), "end")):
        for ref, num, poly, play in pads.get(t["net"], []):
            if t["layer"] not in play:
                continue
            pt = Point(ex, ey)
            if poly.contains(pt) or poly.distance(pt) > REACH:
                continue
            dx, dy = ex - ox, ey - oy
            n = math.hypot(dx, dy)
            if n < 1e-9:
                continue
            dx, dy = dx / n, dy / n
            k = 0.01
            while k <= REACH:
                nx, ny = ex + dx * k, ey + dy * k
                q = Point(nx, ny)
                if poly.contains(q) and poly.exterior.distance(q) >= INSIDE:
                    if cop.seg_ok(ox, oy, nx, ny, t["w"], t["net"], t["layer"], margin=0.0):
                        snaps.append(dict(net=t["net"], layer=t["layer"], which=which,
                                          x1=t["x1"], y1=t["y1"], x2=t["x2"], y2=t["y2"],
                                          nx=round(nx, 4), ny=round(ny, 4), pad=f"{ref}.{num}"))
                    break
                k += 0.01
            break

json.dump(snaps, open(dst, "w"), indent=1)
print(f"snap_pads: {len(snaps)} endpoint(s) to extend -> {dst}")
for s in snaps:
    print(f"   {s['net']} {s['layer']} {s['which']} -> ({s['nx']},{s['ny']}) into {s['pad']}")
