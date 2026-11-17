"""KiCad's session import takes the via drill from the net class, so a small padstack used on
a net class with a bigger drill ends up with a tiny annular ring: restore the drill that
belongs to each padstack.

8-layer board (JLC08161H-7628, 1.6 mm): the BGA fan-out padstack is 0.35/0.2 and no longer
0.3/0.15, because a 0.15 mm drill through 1.6 mm is a 10.7:1 aspect ratio, over JLCPCB's
10:1 limit. 0.2 mm gives 8:1.

    python fix_vias.py in.kicad_pcb out.kicad_pcb
"""
import sys
import pcbnew

# (max pad diameter, drill) -- first match wins
PADSTACKS = [(0.5, 0.2),      # 0.35 BGA fan-out and 0.4 signal/CorePower
             (1e9, 0.3)]      # 0.6 power
MIN_ANNULAR = 0.075

b = pcbnew.LoadBoard(sys.argv[1])
n = 0
for t in b.GetTracks():
    if t.Type() != pcbnew.PCB_VIA_T:
        continue
    w = pcbnew.ToMM(t.GetWidth(pcbnew.F_Cu))
    d = pcbnew.ToMM(t.GetDrillValue())
    want = next(dr for lim, dr in PADSTACKS if w < lim)
    if want > w - 2 * MIN_ANNULAR:        # never leave less than the minimum ring
        want = round(w - 2 * MIN_ANNULAR, 3)
    if abs(d - want) > 0.005:
        t.SetDrill(pcbnew.FromMM(want))
        n += 1
b.Save(sys.argv[2])
print("vias fixed", n)
