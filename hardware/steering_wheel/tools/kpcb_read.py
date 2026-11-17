"""Read tracks / vias / unconnected report from KiCad board + DRC report files."""
import re
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))
from sexp import parse, find, find_all  # noqa: E402


def tracks_vias(path):
    doc = parse(open(path).read())
    nets = {}
    for n in find_all(doc, "net"):
        nets[str(n[1])] = str(n[2])
    tr, vi = [], []
    for s in find_all(doc, "segment"):
        a, b = find(s, "start"), find(s, "end")
        net = find(s, "net")[1]
        tr.append(dict(layer=str(find(s, "layer")[1]), x1=float(a[1]), y1=float(a[2]), x2=float(b[1]),
                       y2=float(b[2]), w=float(find(s, "width")[1]), net=nets.get(str(net), str(net)),
                       locked=bool(find(s, "locked"))))
    for v in find_all(doc, "via"):
        at = find(v, "at")
        net = find(v, "net")[1]
        vi.append(dict(x=float(at[1]), y=float(at[2]), d=float(find(v, "size")[1]),
                       drill=float(find(v, "drill")[1]), net=nets.get(str(net), str(net)),
                       locked=bool(find(v, "locked"))))
    return tr, vi


def drc_items(rpt):
    out = []
    cur = None
    for line in open(rpt):
        if line.startswith("["):
            cur = dict(kind=line[1:line.index("]")], text=line.strip(), items=[])
            out.append(cur)
        elif line.strip().startswith("@(") and cur is not None:
            m = re.match(r"\s*@\(([-\d.]+) mm, ([-\d.]+) mm\): (.*)", line)
            if m:
                cur["items"].append((float(m.group(1)), float(m.group(2)), m.group(3)))
    return out
