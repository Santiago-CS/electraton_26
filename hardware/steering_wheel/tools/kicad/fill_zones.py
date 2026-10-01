"""Fill all zones of a board and save it (pcbnew Python, KiCad 9/10).

    python3 tools/kicad/fill_zones.py in.kicad_pcb [out.kicad_pcb]
"""
import sys

import pcbnew

src = sys.argv[1]
dst = sys.argv[2] if len(sys.argv) > 2 else src
b = pcbnew.LoadBoard(src)
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
b.Save(dst)
print("filled", len(list(b.Zones())), "zones ->", dst)
