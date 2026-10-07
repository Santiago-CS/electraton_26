"""Remove dangling track stubs and vias, refilling the zones between passes.

A dangling end is only meaningful against a current zone fill: removing copper changes what
the pours connect, so a loop that fills once and then deletes for twenty rounds cascades
into real connections. This refills before every pass, stops as soon as a pass finds
nothing, and aborts the whole removal if the number of unconnected items ever grows.

    python drop_dangling.py in.kicad_pcb out.kicad_pcb [max_passes]
"""
import os
import sys

import pcbnew

src, out = sys.argv[1:3]
PASSES = int(sys.argv[3]) if len(sys.argv) > 3 else 6
KEEP_NETS = {"GND", "+3V3"}          # plane nets: a "dangling" end may sit on its pour

b = pcbnew.LoadBoard(src)


def unconnected(board):
    board.BuildConnectivity()
    return board.GetConnectivity().GetUnconnectedCount(False)


pcbnew.ZONE_FILLER(b).Fill(b.Zones())
base = unconnected(b)
print("unconnected at start:", base)

removed_t = removed_v = 0
for npass in range(PASSES):
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    b.BuildConnectivity()
    conn = b.GetConnectivity()
    tracks = list(b.GetTracks())            # take the list before touching connectivity again
    kill = []
    for t in tracks:
        if t.GetNetname() in KEEP_NETS:
            continue
        if t.IsLocked():
            continue
        if conn.TestTrackEndpointDangling(t, False):
            kill.append(t)
    if not kill:
        break
    for t in kill:
        is_via = t.Type() == pcbnew.PCB_VIA_T
        b.Remove(t)
        t.thisown = 0                       # see finish_board.py: do not let Python free it
        if is_via:
            removed_v += 1
        else:
            removed_t += 1
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    now = unconnected(b)
    print(f"  pass {npass + 1}: removed {len(kill)} items, unconnected now {now}")
    if now > base:
        print("  ABORT: the removal broke a connection; writing the input unchanged")
        b = pcbnew.LoadBoard(src)
        pcbnew.ZONE_FILLER(b).Fill(b.Zones())
        removed_t = removed_v = 0
        break

pcbnew.ZONE_FILLER(b).Fill(b.Zones())
print(f"removed {removed_t} tracks, {removed_v} vias; unconnected {unconnected(b)} (was {base})")
b.Save(out)
print("saved", out)
sys.stdout.flush()
sys.stderr.flush()
os._exit(0)
