"""Reduce full KiCad stock libraries to the symbols this project uses.

  python tools/trim_libs.py <full_lib_dir>

Writes lib/std/<Lib>.kicad_sym containing only the symbols referenced by the
generated schematic (plus any 'extends' parents), so the project stays small and does
not depend on the KiCad version installed. To use other stock symbols in
steering_wheel.py, re-run this against KiCad's own symbol folder after adding them.
"""
import glob
import os
import re
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def blocks(text):
    """Top-level (symbol ...) blocks of a .kicad_sym file -> {name: text}."""
    out = {}
    for m in re.finditer(r'\n\t\(symbol "([^"]+)"\n', text):
        i = m.start() + 2
        d, j = 0, i
        while True:
            c = text[j]
            if c == '"':
                j += 1
                while text[j] != '"':
                    j += 2 if text[j] == "\\" else 1
            elif c == "(":
                d += 1
            elif c == ")":
                d -= 1
                if d == 0:
                    break
            j += 1
        out[m.group(1)] = text[i:j + 1]
    return out


def main(full_dir):
    used = {}
    for sch in glob.glob(os.path.join(HERE, "*.kicad_sch")):
        for lib, name in re.findall(r'\(lib_id "([^":]+):([^"]+)"\)', open(sch).read()):
            used.setdefault(lib, set()).add(name)
    used.pop("steering_wheel", None)
    os.makedirs(os.path.join(HERE, "lib", "std"), exist_ok=True)
    for lib, names in sorted(used.items()):
        text = open(os.path.join(full_dir, f"{lib}.kicad_sym")).read()
        header = text[:text.index('\n\t(symbol "') + 1]
        blk = blocks(text)
        todo, keep = list(names), []
        while todo:
            n = todo.pop()
            if n in keep:
                continue
            keep.append(n)
            m = re.search(r'\(extends "([^"]+)"\)', blk[n])
            if m:
                todo.append(m.group(1))
        body = "".join("\t" + blk[n].rstrip() + "\n" for n in sorted(keep))
        with open(os.path.join(HERE, "lib", "std", f"{lib}.kicad_sym"), "w") as fh:
            fh.write(header + body + ")\n")
        print(f"{lib}: {len(keep)} symbols")


if __name__ == "__main__":
    main(sys.argv[1])
