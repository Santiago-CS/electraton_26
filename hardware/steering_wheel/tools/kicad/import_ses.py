"""Import a Freerouting session into the full board (with the outer GND pours).
    python import_ses.py build/prerouted.kicad_pcb build/board.ses build/routed.kicad_pcb"""
import sys
import pcbnew

b = pcbnew.LoadBoard(sys.argv[1])
print("ses import", pcbnew.ImportSpecctraSES(b, sys.argv[2]))
b.Save(sys.argv[3])
