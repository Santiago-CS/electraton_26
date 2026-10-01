"""Export a DSN from an already (partly) routed board for another Freerouting pass:
the outer-layer GND pours are dropped (Freerouting would treat them as solid copper).
    python export_dsn_routed.py routed.kicad_pcb out.dsn"""
import sys
import pcbnew

b = pcbnew.LoadBoard(sys.argv[1])
for z in list(b.Zones()):
    if z.GetIsRuleArea():
        continue
    if z.GetNetname() == "GND" and (z.IsOnLayer(pcbnew.F_Cu) or z.IsOnLayer(pcbnew.B_Cu)):
        b.Remove(z)
print("dsn export", pcbnew.ExportSpecctraDSN(b, sys.argv[2]))
