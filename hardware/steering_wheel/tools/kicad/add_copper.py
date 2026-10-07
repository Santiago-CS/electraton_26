"""Add tracks/vias from a JSON file (finish_router.py output) to a board.
    python add_copper.py in.kicad_pcb finish.json out.kicad_pcb"""
import json
import sys
import pcbnew

src, js, out = sys.argv[1:4]
b = pcbnew.LoadBoard(src)
d = json.load(open(js))
if d.get("replace_unlocked"):
    # the file holds the complete set of unlocked wiring (rip-up may have removed some)
    for t in list(b.GetTracks()):
        if not t.IsLocked():
            b.Remove(t)
layer = {"F.Cu": pcbnew.F_Cu, "B.Cu": pcbnew.B_Cu,
         "In1.Cu": pcbnew.In1_Cu, "In2.Cu": pcbnew.In2_Cu, "In3.Cu": pcbnew.In3_Cu,
         "In4.Cu": pcbnew.In4_Cu, "In5.Cu": pcbnew.In5_Cu, "In6.Cu": pcbnew.In6_Cu}
mm = pcbnew.FromMM
for t in d["tracks"]:
    tr = pcbnew.PCB_TRACK(b)
    tr.SetStart(pcbnew.VECTOR2I(mm(t["x1"]), mm(t["y1"])))
    tr.SetEnd(pcbnew.VECTOR2I(mm(t["x2"]), mm(t["y2"])))
    tr.SetWidth(mm(t["w"]))
    tr.SetLayer(layer[t["layer"]])
    tr.SetNet(b.FindNet(t["net"]))
    if t.get("locked"):
        tr.SetLocked(True)
    b.Add(tr)
for v in d["vias"]:
    via = pcbnew.PCB_VIA(b)
    via.SetPosition(pcbnew.VECTOR2I(mm(v["x"]), mm(v["y"])))
    via.SetWidth(mm(v["d"]))
    via.SetDrill(mm(v["drill"]))
    via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    if v.get("locked"):
        via.SetLocked(True)
    via.SetNet(b.FindNet(v["net"]))
    b.Add(via)
b.Save(out)
print("added", len(d["tracks"]), "tracks,", len(d["vias"]), "vias")
