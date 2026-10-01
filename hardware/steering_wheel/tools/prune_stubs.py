"""List redundant stub copper: unlocked tracks/vias that carry no connection.

Per net, the copper is a graph whose terminals are the pads. Any non-pad item that touches
the rest of the net at only one point is a leaf: nothing is reached through it, so removing
it cannot disconnect anything. Pruning leaves repeatedly strips whole abandoned branches
left behind by the autorouter and by rip-up, which DRC reports as track_dangling.

Plane nets are skipped: their copper also connects through the pours, which this model does
not see, so a track ending on a pour would look like a leaf.

    python tools/prune_stubs.py board.kicad_pcb stubs.json
    python tools/kicad/rip_items.py board.kicad_pcb stubs.json out.kicad_pcb
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import finish_router as FR  # noqa: E402
import layout_steering as LS  # noqa: E402
from kpcb_read import tracks_vias  # noqa: E402

src, dst = sys.argv[1:3]
PLANE = set(LS.PLANE_NETS)

L = LS.build_from(src)
tr, vi = tracks_vias(src)
items = FR.copper_items(L, tr, vi)

# locked copper is part of the floorplan and never pruned
locked = {(round(t["x1"], 4), round(t["y1"], 4), round(t["x2"], 4), round(t["y2"], 4), t["layer"])
          for t in tr if t.get("locked")}
locked_v = {(round(v["x"], 4), round(v["y"], 4)) for v in vi if v.get("locked")}

by_net = {}
for k, it in enumerate(items):
    if it["net"] and it["net"] not in PLANE:
        by_net.setdefault(it["net"], []).append(k)

drop_t, drop_v = [], []
for net, ks in by_net.items():
    # adjacency on exact geometry
    adj = {k: set() for k in ks}
    geo = {k: FR.exact_geom(items[k]) for k in ks}
    for a in range(len(ks)):
        for b in range(a + 1, len(ks)):
            ka, kb = ks[a], ks[b]
            if not set(items[ka]["layers"]) & set(items[kb]["layers"]):
                continue
            if geo[ka].distance(geo[kb]) <= 1e-4:
                adj[ka].add(kb)
                adj[kb].add(ka)
    alive = set(ks)
    while True:
        leaves = [k for k in alive
                  if items[k]["kind"] != "pad" and len(adj[k] & alive) <= 1]
        gone = []
        for k in leaves:
            it = items[k]
            if it["kind"] == "track":
                key = (round(it["shape"][1][0], 4), round(it["shape"][1][1], 4),
                       round(it["shape"][1][2], 4), round(it["shape"][1][3], 4), it["layers"][0])
                if key in locked:
                    continue
            else:
                key = (round(it["shape"][1][0], 4), round(it["shape"][1][1], 4))
                if key in locked_v:
                    continue
            gone.append(k)
        if not gone:
            break
        for k in gone:
            alive.discard(k)
            it = items[k]
            a = it["shape"][1]
            if it["kind"] == "track":
                drop_t.append(dict(net=net, layer=it["layers"][0], x1=a[0], y1=a[1],
                                   x2=a[2], y2=a[3], w=a[4] * 2))
            else:
                drop_v.append(dict(net=net, x=a[0], y=a[1], d=a[2] * 2))

json.dump(dict(tracks=drop_t, vias=drop_v), open(dst, "w"), indent=1)
print(f"prune_stubs: {len(drop_t)} tracks, {len(drop_v)} vias of stub copper -> {dst}")
