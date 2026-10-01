"""Write a grouped BOM (CSV) from the SKiDL circuit."""
import csv
import os
import re


def _natkey(ref):
    m = re.match(r"([A-Za-z#]+)(\d+)", ref)
    return (m.group(1), int(m.group(2))) if m else (ref, 0)


def write_bom(circuit, path):
    groups = {}
    for p in circuit.parts:
        props = getattr(p, "sw_props", {}) or {}
        dnp = bool(getattr(p, "sw_dnp", False))
        key = (str(p.value), p.footprint or "", props.get("MPN", ""), dnp)
        g = groups.setdefault(key, {"refs": [], "props": props})
        g["refs"].append(p.ref)
    rows = []
    for (value, fp, mpn, dnp), g in groups.items():
        refs = sorted(g["refs"], key=_natkey)
        rows.append({"Qty": len(refs), "References": " ".join(refs), "Value": value,
                     "Footprint": fp.split(":", 1)[-1], "MPN": mpn,
                     "Manufacturer": g["props"].get("Manufacturer", ""),
                     "LCSC": g["props"].get("LCSC", ""), "DNP": "DNP" if dnp else "",
                     "Note": g["props"].get("Note", "") if len(refs) == 1 else ""})
    rows.sort(key=lambda r: (r["DNP"], _natkey(r["References"].split()[0])))
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    return rows
