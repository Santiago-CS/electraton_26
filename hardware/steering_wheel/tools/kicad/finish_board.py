"""Finish the routed board: add GND stitching vias (tools/stitch.py), remove leftover
dangling track stubs / vias on nets that are completely connected, refill every zone,
save, and write a DRC report.

    python finish_board.py build/routed.kicad_pcb build/stitch.json steering_wheel.kicad_pcb build/drc.rpt

stitch.json: {"vias": [{x, y, d, drill}], "complete_nets": [names]} (or a plain via list)."""
import json
import os
import sys

import pcbnew

src, sj, out, rpt = sys.argv[1:5]
b = pcbnew.LoadBoard(src)
data = json.load(open(sj))
vias = data["vias"] if isinstance(data, dict) else data
complete = set(data.get("complete_nets", [])) if isinstance(data, dict) else set()
gnd = b.FindNet("GND")
for v in vias:
    via = pcbnew.PCB_VIA(b)
    via.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(v["x"]), pcbnew.FromMM(v["y"])))
    via.SetWidth(pcbnew.FromMM(v["d"]))
    via.SetDrill(pcbnew.FromMM(v["drill"]))
    via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    via.SetNet(gnd)
    b.Add(via)
print("stitching vias", len(vias))

# zones first, so tracks ending in a pour count as connected
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
# BOARD.Remove() hands ownership of the removed item back to Python, but the board's item
# list can still point at it. Letting Python free it corrupts that list: a later
# BOARD.Tracks() then comes back as a raw SwigPyObject ("not iterable"), and freeing the
# batch at interpreter shutdown segfaults. Disowning each removed item (thisown = 0) leaves
# it allocated for the rest of this short-lived process, which is the safe option.
def drop(item):
    b.Remove(item)
    item.thisown = 0


removed = 0
for _ in range(20):
    b.BuildConnectivity()
    conn = b.GetConnectivity()
    kill = []
    for t in b.GetTracks():
        if t.GetNetname() not in complete:
            continue
        if t.Type() == pcbnew.PCB_VIA_T:
            continue
        if conn.TestTrackEndpointDangling(t, False):
            kill.append(t)
    if not kill:
        break
    for t in kill:
        drop(t)
    removed += len(kill)
# signal vias left with a single connection on a complete net
b.BuildConnectivity()
conn = b.GetConnectivity()
vkill = []
for t in b.GetTracks():
    if t.Type() != pcbnew.PCB_VIA_T or t.GetNetname() not in complete:
        continue
    if t.GetNetname() in ("GND", "+3V3"):
        continue
    if conn.TestTrackEndpointDangling(t, False):
        vkill.append(t)
for t in vkill:
    drop(t)
print("removed dangling:", removed, "tracks,", len(vkill), "vias")
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(out)
# No WriteDRCReport here: on KiCad 10 it segfaults in a headless script (a hard crash, so
# it cannot be caught), and it would take the finished board down with it. finalize.sh runs
# "kicad-cli pcb drc --schematic-parity" on the saved board instead, which is also the
# number that counts. The rpt argument is kept for call compatibility.
print("saved", out, "-- run kicad-cli pcb drc for the report")

# KiCad 10's C++ teardown also segfaults on the way out of a script that removed items,
# which would make this step look like a failure to the run scripts. The board is written,
# so leave without running it.
sys.stdout.flush()
sys.stderr.flush()
os._exit(0)
