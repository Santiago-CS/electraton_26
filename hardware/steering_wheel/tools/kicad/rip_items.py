"""Remove the tracks and vias listed by tools/prune_stubs.py, then refill the zones.
    python rip_items.py in.kicad_pcb stubs.json out.kicad_pcb"""
import json
import os
import sys

import pcbnew

src, sj, out = sys.argv[1:4]
data = json.load(open(sj))
want_t = {(d["net"], d["layer"], round(d["x1"], 4), round(d["y1"], 4),
           round(d["x2"], 4), round(d["y2"], 4)) for d in data.get("tracks", [])}
want_v = {(d["net"], round(d["x"], 4), round(d["y"], 4)) for d in data.get("vias", [])}

b = pcbnew.LoadBoard(src)
to = pcbnew.ToMM


def r(v):
    return round(to(v), 4)


nt = nv = 0
for t in list(b.GetTracks()):
    if t.IsLocked():
        continue
    net = t.GetNetname()
    if t.Type() == pcbnew.PCB_VIA_T:
        p = t.GetPosition()
        if (net, r(p.x), r(p.y)) in want_v:
            b.Remove(t)
            t.thisown = 0
            nv += 1
        continue
    s, e = t.GetStart(), t.GetEnd()
    lay = t.GetLayerName()
    if (net, lay, r(s.x), r(s.y), r(e.x), r(e.y)) in want_t or \
       (net, lay, r(e.x), r(e.y), r(s.x), r(s.y)) in want_t:
        b.Remove(t)
        t.thisown = 0
        nt += 1

pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.BuildConnectivity()
print(f"rip_items: removed {nt} tracks, {nv} vias; "
      f"unconnected now {b.GetConnectivity().GetUnconnectedCount(False)}")
b.Save(out)
print("saved", out)
sys.stdout.flush()
sys.stderr.flush()
os._exit(0)
