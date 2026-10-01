# Steering wheel controller (RW612) — schematic + placement

Hardware for the Electratón 26 steering wheel: an NXP RW612 (Wi-Fi 6 / BLE) controller
driving a 4" LCD, NeoPixels, two encoders and ten buttons, running from a 1S Li-ion pack
with USB-C charging.

**This folder holds the schematic and the board placement. The board is not routed.**

```
source <your venv>/bin/activate
pip install skidl ortools shapely scipy pymupdf scikit-image ezdxf

python steering_wheel.py                  # netlist + KiCad sheets + bom.csv
python tools/check_sch.py                 # re-reads the sheets, checks every pin
python tools/layout_steering.py place     # placement -> build/placed.kicad_pcb
```

Open `steering_wheel.kicad_pro` in KiCad 10 and press **B** to refill the zones.

> The schematic is **generated** by `steering_wheel.py` (SKiDL). Edit that script and
> re-run it — anything changed directly in Eeschema is lost on the next run. Same for the
> placement: it is code in `tools/layout_steering.py`.

> The sheets here are saved in KiCad 10 format. `tools/kicad_sch_export.py` stamps
> regenerated sheets as KiCad 9 (`version 20250114`, lines 461 and 693), so after re-running
> `steering_wheel.py` KiCad 10 will offer to upgrade them again — harmless, or change those
> two lines to `20260306` / `"10.0"` to stop it.

## State

| | |
|---|---|
| Schematic | rev 0.2 — 5 sheets, 237 parts, 152 nets, **ERC 0 violations** |
| Board | **placed, not routed** — 8 layers, JLCPCB `JLC08161H-7628`, 1.6 mm, 138.9 × 83.8 mm |
| Placement copper | 108 tracks / 17 vias, all locked (power loops, USB breakout, battery trunks, RW612 buck and crystal links) |
| DRC (zones filled, `--schematic-parity`) | **0 errors, 0 parity errors, 384 unconnected** |

The 384 unconnected are the connections still to be routed — exactly what a placement-only
board should report.

## Layer stack

| # | Layer | Use |
|---|---|---|
| 1 | F.Cu | parts (all on top), signals, 50 Ω RF microstrip, GND pour |
| 2 | In1.Cu | **GND plane** — RF and USB reference |
| 3 | In2.Cu | signal |
| 4 | In3.Cu | **+3V3 plane** |
| 5 | In4.Cu | **GND plane** |
| 6 | In5.Cu | signal |
| 7 | In6.Cu | **GND plane** |
| 8 | B.Cu | signals, VSYS_SW trunk, GND pour |

Four routing layers, each against a ground plane. Vias are 0.4/0.2 mm, BGA fan-out
0.35/0.2 mm (8.01:1 aspect ratio — no small-via option needed).

## Routing it

The routing pipeline is not in this folder. A **fully routed** 8-layer version of this same
design (0 DRC errors, 0 parity errors) lives in `~/projects/irt/steering_wheel/pcb_8layer`,
and the 4-layer original in `~/projects/irt/steering_wheel/pcb`. To route from here, copy
`build/env.sh`, `build/run_all.sh`, `build/finalize.sh` and `build/strip_vias.py` from
`pcb_8layer/build/` and fix the paths in `env.sh`.

## Layout

| Path | What |
|---|---|
| `steering_wheel.py` | the design (SKiDL) — **the source of truth** |
| `*.kicad_sch`, `steering_wheel.kicad_pro`, `*-lib-table` | generated KiCad project |
| `steering_wheel.kicad_pcb` | the placed board (copy of `build/placed.kicad_pcb`) |
| `steering_wheel.net`, `bom.csv` | generated netlist and BOM (98 lines, 237 parts, 9 DNP) |
| `lib/` | project symbols and footprints, the stock KiCad symbols used, RW612 ball map |
| `tools/` | schematic export, placement, pre-routing and KiCad-Python helpers |
| `docs/design_notes.md` | **full design documentation** — power tree, pin map, firmware hooks, layout, schematic review, what to check before ordering |
| `docs/PCB_Volante_2026.DXF` | mechanical outline and mounting holes (placement reads this) |
| `docs/pcb_*.png`, `docs/pcb_layers.pdf` | previews of the placed board |
| `build/kicad_netlist.net` | KiCad's own netlist; the board takes no-connect net names from it so DRC parity matches |

## Before ordering

See `docs/design_notes.md` section 7. Two items need a decision:

1. **Q101 (FS8205A) pads 1 and 8** are both the `D12` common-drain pin, which the schematic
   leaves unconnected, so DRC always reports one unconnected item. The usual SOP-8 pinout
   has no drain pin (1,2,3 = S1, 6,7,8 = S2), so the symbol/footprint pin map may be wrong.
2. **The USB pair is ~128 Ω differential, not 90 Ω** on this stackup. ~0.40 mm traces at a
   0.15 mm gap would reach 90 Ω.
