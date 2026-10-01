"""Copy the stock symbols this project uses from a KiCad 10 install into lib/std/,
converted to the KiCad 9 symbol-library syntax (version 20241209).

  python tools/import_k10_symbols.py /usr/share/kicad/symbols
  python tools/import_k10_symbols.py /usr/share/kicad/symbols --add   # only symbols missing from lib/std

KiCad 9 stock libraries work too: their only difference from KiCad 10 for these symbols
is "~" for an empty Datasheet / pin name, which is normalised to "" here, so the copies
still match KiCad 10's library.

The conversion only drops tokens that carry default values (in_pos_files,
duplicate_pin_numbers_are_jumpers, show_name, do_not_autoplace) and moves the
property-level (hide yes) back into (effects ...), so the embedded copies in the
schematic match KiCad 10's own library (no "doesn't match copy in library" ERC warning)
while staying readable by SKiDL's KiCad 9 parser.
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(__file__))
from sexp import QStr, dumps, find, parse  # noqa: E402
from trim_libs import blocks  # noqa: E402

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

USED = {
    "Device": ["R", "C", "L", "LED", "C_Polarized", "Crystal", "Crystal_GND24", "Polyfuse"],
    "Connector": ["USB_C_Receptacle_USB2.0_16P", "Conn_01x02_Pin", "Conn_01x03_Pin",
                  "Conn_01x04_Pin", "Conn_01x06_Pin", "Conn_01x14_Pin", "Conn_01x14_Socket", "Conn_ARM_JTAG_SWD_10",
                  "Conn_Coaxial", "TestPoint"],
    "Interface_Expansion": ["TCA9555PWR"],
    "power": ["GND", "+3V3", "+5V", "+1V8", "VBUS", "PWR_FLAG"],
    "Transistor_FET": ["Q_PMOS_GSD"],
}
DROP = {"in_pos_files", "duplicate_pin_numbers_are_jumpers", "show_name", "do_not_autoplace"}


def convert(node):
    if not isinstance(node, list):
        return node
    out = []
    hide = False
    for c in node:
        if isinstance(c, list) and c and c[0] in DROP:
            continue
        if node[0] == "property" and isinstance(c, list) and c and c[0] == "hide":
            hide = c[1] == "yes" if len(c) > 1 else True
            continue
        out.append(convert(c))
    if node[0] == "property" and hide:
        eff = find(out, "effects")
        if eff is None:
            eff = ["effects", ["font", ["size", "1.27", "1.27"]]]
            out.append(eff)
        eff.append(["hide", "yes"])
    return out


def normalise(node):
    """KiCad 9 writes "~" for an empty Datasheet and pin name/number; KiCad 10 writes ""."""
    if not isinstance(node, list):
        return node
    out = [normalise(c) for c in node]
    if len(out) > 2 and out[0] == "property" and str(out[1]) == "Datasheet" and str(out[2]) == "~":
        out[2] = QStr("")
    if len(out) > 1 and out[0] in ("name", "number") and str(out[1]) == "~":
        out[1] = QStr("")
    return out


def main(src, add_only=False):
    os.makedirs(os.path.join(HERE, "lib", "std"), exist_ok=True)
    for lib, names in USED.items():
        text = open(os.path.join(src, f"{lib}.kicad_sym")).read()
        blk = blocks(text)
        dst = os.path.join(HERE, "lib", "std", f"{lib}.kicad_sym")
        have = blocks(open(dst).read()) if add_only and os.path.exists(dst) else {}
        keep, todo = [], list(names)
        while todo:
            n = todo.pop()
            if n in keep:
                continue
            keep.append(n)
            m = re.search(r'\(extends "([^"]+)"\)', blk[n])
            if m:
                todo.append(m.group(1))
        doc = ["kicad_symbol_lib", ["version", "20241209"], ["generator", QStr("kicad_symbol_editor")],
               ["generator_version", QStr("9.0")]]
        added = 0
        for n in sorted(set(keep) | set(have)):
            if n in have:                      # --add: keep the existing copy untouched
                doc.append(parse(have[n]))
            else:
                doc.append(normalise(convert(parse(blk[n]))))
                added += 1
        with open(dst, "w") as fh:
            fh.write(dumps(doc) + "\n")
        print(f"{lib}: {len(doc) - 4} symbols ({added} new)")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    main(args[0] if args else "/usr/share/kicad/symbols", add_only="--add" in sys.argv)
