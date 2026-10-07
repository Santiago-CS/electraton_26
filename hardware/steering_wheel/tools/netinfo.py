"""Collect what the board needs from the generated schematic + SKiDL netlist:
KiCad-style net names (global vs '/<sheet>/<label>'), footprint links (path,
sheetname, sheetfile), values, footprints, DNP flags and extra fields."""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from sexp import find, find_all, parse  # noqa: E402
from kicad_sch_export import POWER_SYMBOLS  # noqa: E402

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load(project="steering_wheel"):
    root = parse(open(os.path.join(HERE, f"{project}.kicad_sch")).read())
    sheets = []
    for sh in find_all(root, "sheet"):
        props = {str(p[1]): str(p[2]) for p in find_all(sh, "property")}
        sheets.append((props["Sheetname"], props["Sheetfile"], str(find(sh, "uuid")[1])))
    globals_, locals_ = set(POWER_SYMBOLS), {}
    comps = {}
    for sname, sfile, suuid in sheets:
        doc = parse(open(os.path.join(HERE, sfile)).read())
        # group boxes: dashed rectangles with a bold title text at their top-left
        rects, titles = [], []
        for it in doc:
            if isinstance(it, list) and it[0] == "rectangle":
                a, b_ = find(it, "start"), find(it, "end")
                rects.append((float(a[1]), float(a[2]), float(b_[1]), float(b_[2])))
            elif isinstance(it, list) and it[0] == "text":
                at = find(it, "at")
                titles.append((float(at[1]), float(at[2]), str(it[1])))
        groups = []
        for r in rects:
            best = None
            for tx, ty, t in titles:
                if r[0] <= tx <= r[2] and r[1] <= ty <= r[1] + 6:
                    best = t
            groups.append((r, best or "?"))

        def group_at(x, y):
            for r, t in groups:
                if r[0] <= x <= r[2] and r[1] <= y <= r[3]:
                    return f"{sname}/{t}"
            return f"{sname}/?"
        for it in doc:
            if not isinstance(it, list):
                continue
            if it[0] == "global_label":
                globals_.add(str(it[1]))
            elif it[0] == "label":
                locals_.setdefault(str(it[1]), set()).add(sname)
            elif it[0] == "symbol":
                props = {str(p[1]): str(p[2]) for p in find_all(it, "property")}
                ref = props["Reference"]
                if ref.startswith("#"):
                    continue
                unit = int(find(it, "unit")[1])
                dnp = find(it, "dnp")
                c = comps.setdefault(ref, dict(ref=ref, units={}, groups=[]))
                at = find(it, "at")
                c["groups"].append(group_at(float(at[1]), float(at[2])))
                c["units"][unit] = (sname, sfile, suuid, str(find(it, "uuid")[1]))
                c["value"] = props.get("Value", "")
                c["footprint"] = props.get("Footprint", "")
                c["dnp"] = bool(dnp and dnp[1] == "yes")
                c["props"] = {k: v for k, v in props.items()
                              if k not in ("Reference", "Value", "Footprint")}
    for c in comps.values():
        u = min(c["units"])
        sname, sfile, suuid, symuuid = c["units"][u]
        c["path"] = f"/{suuid}/{symuuid}"
        c["sheetname"] = f"/{sname}/"
        c["sheetfile"] = sfile
        c["group"] = c["groups"][0]

    def kname(n):
        if n in globals_:
            return n
        if n in locals_ and len(locals_[n]) == 1:
            return f"/{next(iter(locals_[n]))}/{n}"
        return n

    net = parse(open(os.path.join(HERE, f"{project}.net")).read())
    pin_net = {}
    nets = {}
    for n in find_all(find(net, "nets"), "net"):
        name = kname(str(find(n, "name")[1]))
        for node in find_all(n, "node"):
            ref, pin = str(find(node, "ref")[1]), str(find(node, "pin")[1])
            pin_net[(ref, pin)] = name
            nets.setdefault(name, []).append((ref, pin))
    return comps, nets, pin_net


if __name__ == "__main__":
    comps, nets, pin_net = load()
    print(len(comps), "components,", len(nets), "nets")
    for n in sorted(nets)[:15]:
        print(n, nets[n][:4])
    print(comps["U201"]["path"], comps["U201"]["sheetname"], comps["R101"])
