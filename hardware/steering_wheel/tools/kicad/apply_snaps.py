"""Apply the endpoint corrections from tools/snap_pads.py to a board.
    python apply_snaps.py in.kicad_pcb snaps.json out.kicad_pcb"""
import json
import os
import sys

import pcbnew

src, sj, out = sys.argv[1:4]
snaps = json.load(open(sj))
b = pcbnew.LoadBoard(src)
mm, to = pcbnew.FromMM, pcbnew.ToMM


def eq(a, v):
    return abs(to(a) - v) < 1e-4


n = 0
for t in list(b.GetTracks()):
    if t.Type() != pcbnew.PCB_TRACE_T:
        continue
    s, e = t.GetStart(), t.GetEnd()
    for sn in snaps:
        if t.GetNetname() != sn["net"] or t.GetLayerName() != sn["layer"]:
            continue
        if not (eq(s.x, sn["x1"]) and eq(s.y, sn["y1"]) and eq(e.x, sn["x2"]) and eq(e.y, sn["y2"])):
            continue
        if sn["which"] == "start":
            t.SetStart(pcbnew.VECTOR2I(mm(sn["nx"]), mm(sn["ny"])))
        else:
            t.SetEnd(pcbnew.VECTOR2I(mm(sn["nx"]), mm(sn["ny"])))
        n += 1
        break
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(out)
print("apply_snaps: moved", n, "endpoints ->", out)
sys.stdout.flush()
sys.stderr.flush()
os._exit(0)
