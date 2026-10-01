#!/usr/bin/env bash
# Rebuild the board from the schematic netlist:
#   placement + pre-routing (Python: RF, power, BGA fan-out/escape, core-supply hookup)
#   -> Freerouting -> finishing router with rip-up (Python) -> KiCad (stitching vias, zone fill, DRC)
#
#   PY         python with ortools, shapely, scipy (the pcb .venv)          default: python
#   KICAD_PY   python interpreter that can "import pcbnew" (KiCad 10)       default: python3
#   FR_JAR     freerouting-2.1.0.jar (https://github.com/freerouting/freerouting/releases)
#   FR_MIN     autorouter time budget in minutes                            default: 45
#   FR_THREADS autorouter threads (2 routes this board much better than 1)  default: 2
#
#   KICAD_PY=/usr/bin/python3 FR_JAR=~/tools/freerouting/freerouting-2.1.0.jar tools/route_board.sh
#
# 8-layer board: see also build/run_all.sh, which does the same with the paths of this
# machine baked in (build/env.sh) and keeps per-attempt tags.
set -euo pipefail
cd "$(dirname "$0")/.."
PY=${PY:-python}
KICAD_PY=${KICAD_PY:-python3}
FR_JAR=${FR_JAR:?set FR_JAR to the freerouting jar}
JAVA=${JAVA:-java}
FR_MIN=${FR_MIN:-45}
FR_THREADS=${FR_THREADS:-2}
B=build

$PY tools/layout_steering.py                       # $B/placed, $B/prerouted(_dsn).kicad_pcb
"$KICAD_PY" tools/kicad/export_dsn.py $B/prerouted_dsn.kicad_pcb $B/board.dsn
# new vias from the autorouter: 0.4/0.2 and 0.6/0.3 only (0.35/0.2 is for the BGA fan-out).
# The padstack names carry the layer span, so match on the size, not on "Via[0-3]".
$PY $B/strip_vias.py $B/board.dsn 350:200
# Freerouting stops on its job timeout (it ignores max_passes); the outer timeout is a
# watchdog, because 2.1.0 occasionally fails to finish the job after the timeout
timeout $((FR_MIN * 60 + 360)) "$JAVA" -Xmx4g -jar "$FR_JAR" -de $B/board.dsn -do $B/board.ses \
     --gui.enabled=false --router.job_timeout=$(printf '%02d:%02d:00' $((FR_MIN / 60)) $((FR_MIN % 60))) \
     --router.optimizer.max_passes=0 --router.max_threads=$FR_THREADS --profile.allow_telemetry=false \
     --usage_and_diagnostic_data.disable_analytics=true
"$KICAD_PY" tools/kicad/import_ses.py $B/prerouted.kicad_pcb $B/board.ses $B/routed.kicad_pcb
"$KICAD_PY" tools/kicad/fix_vias.py $B/routed.kicad_pcb $B/routed.kicad_pcb
$PY tools/finish_router.py $B/routed.kicad_pcb $B/finish.json --rr
"$KICAD_PY" tools/kicad/add_copper.py $B/routed.kicad_pcb $B/finish.json $B/finished.kicad_pcb
$PY tools/stitch.py $B/finished.kicad_pcb $B/stitch.json
"$KICAD_PY" tools/kicad/finish_board.py $B/finished.kicad_pcb $B/stitch.json steering_wheel.kicad_pcb $B/drc.rpt
grep -E '^\[' $B/drc.rpt | sed 's/\].*/]/' | sort | uniq -c | sort -rn || true
