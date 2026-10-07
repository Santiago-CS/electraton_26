"""Export a Specctra DSN for Freerouting with KiCad's own exporter.
Run with KiCad's Python (pcbnew module, KiCad 9 or 10):
    python export_dsn.py build/prerouted_dsn.kicad_pcb build/board.dsn"""
import sys
import pcbnew

b = pcbnew.LoadBoard(sys.argv[1])
print("dsn export", pcbnew.ExportSpecctraDSN(b, sys.argv[2]))
