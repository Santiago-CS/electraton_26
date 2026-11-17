"""List the open (unconnected) nets of a routed board with their copper islands.

    python tools/open_nets.py build/r5f.kicad_pcb
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import layout_steering as LS  # noqa: E402
import finish_router as FR  # noqa: E402
from kpcb_read import tracks_vias  # noqa: E402


def describe(items, comp):
    pads = [items[k]["ref"] for k in comp if items[k]["kind"] == "pad"]
    xs, ys = [], []
    for k in comp:
        s = items[k]["shape"]
        if s[0] == "seg":
            xs += [s[1][0], s[1][2]]
            ys += [s[1][1], s[1][3]]
        elif s[0] == "circle":
            xs.append(s[1][0])
            ys.append(s[1][1])
        else:
            xs += [s[1][0], s[1][2]]
            ys += [s[1][1], s[1][3]]
    nt = sum(1 for k in comp if items[k]["kind"] == "track")
    nv = sum(1 for k in comp if items[k]["kind"] == "via")
    bb = (min(xs) - LS.OX, min(ys) - LS.OY, max(xs) - LS.OX, max(ys) - LS.OY)
    p = ",".join(f"{r}.{n}" for r, n in pads[:6]) + ("..." if len(pads) > 6 else "")
    return f"pads[{p}] t{nt} v{nv} bbox({bb[0]:.2f},{bb[1]:.2f})-({bb[2]:.2f},{bb[3]:.2f})"


if __name__ == "__main__":
    L = LS.build_from(sys.argv[1])
    tr, vi = tracks_vias(sys.argv[1])
    items = FR.copper_items(L, tr, vi)
    comps = FR.components(items)
    opened = sorted((n, list(c.values())) for n, c in comps.items() if len(c) > 1)
    print(len(opened), "open nets")
    for n, cs in opened:
        print(n)
        for c in sorted(cs, key=len):
            print("   ", describe(items, c))
