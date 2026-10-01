"""Independent check of the generated KiCad schematic.

  python tools/check_sch.py            # connectivity check vs steering_wheel.net
  python tools/check_sch.py --render   # also writes build/<sheet>.png previews

Connectivity is rebuilt from the .kicad_sch geometry only (pin positions from the
embedded lib_symbols, wires, labels, power symbols, no-connect flags) the way Eeschema
does it, then compared pin-by-pin against the SKiDL netlist.
"""
import math
import os
import re
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(__file__))
from sexp import find, find_all, parse  # noqa: E402

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def rot(x, y, r):
    rr = math.radians(r)
    xr = x * math.cos(rr) - y * math.sin(rr)
    yr = x * math.sin(rr) + y * math.cos(rr)
    return xr, -yr


def key(x, y):
    return (round(x * 100), round(y * 100))


class UF:
    def __init__(self):
        self.p = {}

    def f(self, a):
        self.p.setdefault(a, a)
        while self.p[a] != a:
            self.p[a] = self.p[self.p[a]]
            a = self.p[a]
        return a

    def u(self, a, b):
        self.p[self.f(a)] = self.f(b)


def lib_pins(libsym, unit):
    name = libsym[1].split(":", 1)[1]
    out = []
    for sub in find_all(libsym, "symbol"):
        m = re.match(re.escape(name) + r"_(\d+)_(\d+)$", sub[1])
        if not m or int(m.group(1)) not in (0, unit) or int(m.group(2)) not in (0, 1):
            continue
        for p in find_all(sub, "pin"):
            at = find(p, "at")
            out.append((str(find(p, "number")[1]), float(at[1]), float(at[2]), float(at[3]),
                        float(find(p, "length")[1]), p[1]))
    return out


def load_sheet(path):
    return parse(open(path).read())


def parse_netlist(path):
    """KiCad s-expression netlist from skidl -> {(ref, pin): net}."""
    d = parse(open(path).read())
    nets = find(d, "nets")
    out = {}
    for n in find_all(nets, "net"):
        name = str(find(n, "name")[1])
        for node in find_all(n, "node"):
            out[(str(find(node, "ref")[1]), str(find(node, "pin")[1]))] = name
    comps = {}
    for c in find_all(find(d, "components"), "comp"):
        comps[str(find(c, "ref")[1])] = c
    return out, comps


def main(render=False):
    root = load_sheet(os.path.join(HERE, "steering_wheel.kicad_sch"))
    sheets = []
    for sh in find_all(root, "sheet"):
        f = [p for p in find_all(sh, "property") if p[1] == "Sheetfile"][0][2]
        sheets.append(str(f))
    uf = UF()
    point_names = defaultdict(set)      # component point -> {net names}
    pin_points = {}                     # (ref, pin) -> global point id
    problems = []
    all_pts_by_sheet = {}
    for sf in sheets:
        doc = load_sheet(os.path.join(HERE, sf))
        libs = {str(s[1]): s for s in find(doc, "lib_symbols")[1:]}
        pts = defaultdict(list)          # local key -> descriptions
        wires = []
        for it in doc:
            if not isinstance(it, list):
                continue
            if it[0] == "symbol":
                lib_id = str(find(it, "lib_id")[1])
                at = find(it, "at")
                sx, sy, r = float(at[1]), float(at[2]), float(at[3])
                unit = int(find(it, "unit")[1])
                ref = [p for p in find_all(it, "property") if p[1] == "Reference"][0][2]
                val = [p for p in find_all(it, "property") if p[1] == "Value"][0][2]
                ls = libs[lib_id]
                is_power = find(ls, "power") is not None
                for num, px, py, pa, ln, et in lib_pins(ls, unit):
                    dx, dy = rot(px, py, r)
                    k = (sf, key(sx + dx, sy + dy))
                    uf.f(k)
                    if is_power:
                        if lib_id != "power:PWR_FLAG":
                            point_names[k].add(("G", str(val)))
                    else:
                        pin_points[(str(ref), num)] = k
                    pts[key(sx + dx, sy + dy)].append(("pin", str(ref), num))
            elif it[0] == "wire":
                a, b = find(it, "pts")[1:3]
                ka = (sf, key(float(a[1]), float(a[2])))
                kb = (sf, key(float(b[1]), float(b[2])))
                uf.u(ka, kb)
                wires.append(((float(a[1]), float(a[2])), (float(b[1]), float(b[2]))))
                pts[ka[1]].append(("wire",)); pts[kb[1]].append(("wire",))
            elif it[0] in ("label", "global_label"):
                at = find(it, "at")
                k = (sf, key(float(at[1]), float(at[2])))
                uf.f(k)
                scope = "G" if it[0] == "global_label" else sf
                point_names[k].add((scope, str(it[1])))
                pts[k[1]].append(("label", str(it[1])))
            elif it[0] == "no_connect":
                at = find(it, "at")
                pts[key(float(at[1]), float(at[2]))].append(("nc",))
        # T-junction hazards: any pin/label point strictly inside a wire segment
        for (a, b) in wires:
            for kp, descs in pts.items():
                x, y = kp[0] / 100, kp[1] / 100
                if (abs(x - a[0]) < 1e-3 and abs(x - b[0]) < 1e-3 and
                        min(a[1], b[1]) + 1e-3 < y < max(a[1], b[1]) - 1e-3) or \
                   (abs(y - a[1]) < 1e-3 and abs(y - b[1]) < 1e-3 and
                        min(a[0], b[0]) + 1e-3 < x < max(a[0], b[0]) - 1e-3):
                    problems.append(f"{sf}: point {descs} lies inside wire {a}-{b}")
        # dangling wire ends
        for kp, descs in pts.items():
            kinds = [d[0] for d in descs]
            if kinds.count("wire") == 1 and len(kinds) == 1:
                problems.append(f"{sf}: dangling wire end at {kp}")
        all_pts_by_sheet[sf] = pts
    # merge names -> nets
    comp_names = defaultdict(set)
    for k, names in point_names.items():
        comp_names[uf.f(k)] |= names
    # label names join components across the design
    name_uf = UF()
    for c, names in comp_names.items():
        for scope, n in names:
            name_uf.u(("C", c), ("N", scope, n))
    def net_of_point(k):
        c = uf.f(k)
        names = comp_names.get(c, set())
        return names
    expected, comps = parse_netlist(os.path.join(HERE, "steering_wheel.net"))
    ok = 0
    shorted = set()
    for c, names in comp_names.items():
        plain = {n for s, n in names}
        if len(plain) > 1:
            shorted.add(tuple(sorted(plain)))
    for s in shorted:
        problems.append(f"labels shorted together: {s}")
    for (ref, pin), net in expected.items():
        k = pin_points.get((ref, pin))
        if k is None:
            problems.append(f"{ref}.{pin} (net {net}) not found on any sheet")
            continue
        names = {n for s, n in net_of_point(k)}
        if net not in names:
            problems.append(f"{ref}.{pin}: expected {net}, schematic has {sorted(names) or 'nothing'}")
        else:
            ok += 1
    # pins in schematic with a name but not in netlist (should be NC)
    exp_keys = set(expected)
    for (ref, pin), k in pin_points.items():
        if (ref, pin) not in exp_keys and net_of_point(k):
            problems.append(f"{ref}.{pin}: schematic net {net_of_point(k)} but NC in netlist")
    # same-name local labels on different sheets are different nets: ok. global/local clash:
    print(f"checked {ok}/{len(expected)} netlist pin connections, {len(pin_points)} placed pins")
    for p in problems[:80]:
        print("PROBLEM:", p)
    print(f"{len(problems)} problems")
    if render:
        import render_sch
        for sf in sheets + ["steering_wheel.kicad_sch"]:
            render_sch.render(os.path.join(HERE, sf), os.path.join(HERE, "build", sf.replace(".kicad_sch", ".png")))
    return len(problems)


if __name__ == "__main__":
    sys.exit(1 if main("--render" in sys.argv) else 0)
