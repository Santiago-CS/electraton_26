"""Remove all *unlocked* tracks and vias of the named nets, so the finishing router sees
them as open and routes them again from scratch.

Use it when KiCad's DRC disagrees with the finisher's own connectivity/clearance model --
for example on a soft (knowingly violating) via it left behind, or where a rotated
footprint makes fanout.Copper's axis-aligned pad box look like it touches a track.

    python rip_nets.py in.kicad_pcb out.kicad_pcb NET [NET ...]
"""
import os
import sys

import pcbnew

src, out = sys.argv[1:3]
nets = set(sys.argv[3:])
if not nets:
    raise SystemExit("rip_nets: name at least one net")

b = pcbnew.LoadBoard(src)
t_removed = v_removed = kept_locked = 0
for t in list(b.GetTracks()):
    if t.GetNetname() not in nets:
        continue
    if t.IsLocked():
        kept_locked += 1
        continue
    is_via = t.Type() == pcbnew.PCB_VIA_T
    b.Remove(t)
    # Remove() hands ownership back to Python; disown so nothing frees it while the
    # board's item list may still reference it (see finish_board.py).
    t.thisown = 0
    if is_via:
        v_removed += 1
    else:
        t_removed += 1
b.Save(out)
print(f"ripped {t_removed} tracks, {v_removed} vias of {sorted(nets)} "
      f"({kept_locked} locked items kept)")
sys.stdout.flush()
sys.stderr.flush()
os._exit(0)
