"""Remove dangling stub copper, checking each removal against KiCad's own connectivity.

KiCad decides what counts as a dangling end and what counts as connected, and the two do
not always agree with a geometric model (a via whose copper only lands on one layer, a
track ending mid-span of another). So: take KiCad's list of dangling items, try them one at
a time, and put back any whose removal increases the unconnected count. Nothing is removed
on trust.

Plane nets are left alone -- their copper also connects through the pours, and a stub ending
on its own pour is connected.

    python prune_dangling.py in.kicad_pcb out.kicad_pcb [max_passes]
"""
import os
import sys

import pcbnew

src, out = sys.argv[1:3]
PASSES = int(sys.argv[3]) if len(sys.argv) > 3 else 8
PLANE_NETS = {"GND", "+3V3"}

b = pcbnew.LoadBoard(src)
pcbnew.ZONE_FILLER(b).Fill(b.Zones())


def unconnected():
    b.BuildConnectivity()
    return b.GetConnectivity().GetUnconnectedCount(False)


base = unconnected()
print("unconnected at start:", base)
kept_alive = []
tot_t = tot_v = 0

for npass in range(PASSES):
    b.BuildConnectivity()
    conn = b.GetConnectivity()
    cands = [t for t in list(b.GetTracks())
             if not t.IsLocked() and t.GetNetname() not in PLANE_NETS
             and conn.TestTrackEndpointDangling(t, False)]
    removed_t = removed_v = 0
    for t in cands:
        is_via = t.Type() == pcbnew.PCB_VIA_T
        b.Remove(t)
        if unconnected() > base:
            b.Add(t)                       # it was carrying a connection after all
            b.BuildConnectivity()
        else:
            t.thisown = 0                  # see finish_board.py
            kept_alive.append(t)
            if is_via:
                removed_v += 1
            else:
                removed_t += 1
    tot_t += removed_t
    tot_v += removed_v
    print(f"  pass {npass + 1}: {len(cands)} candidates, removed {removed_t} tracks, "
          f"{removed_v} vias", flush=True)
    if removed_t + removed_v == 0:
        break

pcbnew.ZONE_FILLER(b).Fill(b.Zones())
print(f"removed {tot_t} tracks, {tot_v} vias; unconnected {unconnected()} (was {base})")
b.Save(out)
print("saved", out)
sys.stdout.flush()
sys.stderr.flush()
os._exit(0)
