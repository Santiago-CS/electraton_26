"""Export a SKiDL circuit to a hierarchical KiCad schematic (label-based layout).

Why not skidl.generate_schematic()? Its auto-router draws every net as a wire, which is
unreadable for a 145-ball BGA. This exporter instead:
  * places each part (or part unit) in a tidy grid, grouped by the `block()` it was
    created in, one KiCad sheet per design block;
  * connects every pin with a short wire stub + net label (global label if the net is
    used on more than one sheet, local label otherwise), or a power symbol for rails;
  * puts a no-connect flag on every pin explicitly connected to NC in the design.

Output is KiCad 9 file format (schematic version 20250114, KiCad 10 opens it natively).
The generated sheets are meant to be regenerated from steering_wheel.py; cosmetic edits
made in Eeschema are lost on the next run.
"""
import datetime
import json
import math
import os
import re
import uuid

from sexp import QStr, dumps, find, find_all, parse

G = 1.27            # KiCad schematic grid (50 mil)
STUB = 2.54         # wire stub from pin to label
CHAR_W = 1.15       # approx. width of one 1.27 mm glyph
NS = uuid.UUID("6c1a2b52-8a3e-4c43-9a8f-9f3b1d1f0a11")

# net name -> (lib nickname, symbol) for rails drawn as power symbols
POWER_SYMBOLS = {
    "GND": ("power", "GND"),
    "+3V3": ("power", "+3V3"),
    "+5V": ("power", "+5V"),
    "+1V8": ("power", "+1V8"),
    "VBUS": ("power", "VBUS"),
    "VCORE": ("steering_wheel", "VCORE"),
    "VBAT": ("steering_wheel", "VBAT"),
    "VSYS": ("steering_wheel", "VSYS"),
    "VSYS_SW": ("steering_wheel", "VSYS_SW"),
    "LCD_VCC": ("steering_wheel", "LCD_VCC"),
}
GROUND_NETS = {"GND"}

PAPERS = [("A4", 297, 210), ("A3", 420, 297), ("A2", 594, 420), ("A1", 841, 594)]


def uid(*parts):
    return str(uuid.uuid5(NS, "/".join(str(p) for p in parts)))


def snap(v, g=G):
    return round(round(v / g) * g, 4)


def fmt(v):
    s = f"{v:.4f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def font(size=1.27, bold=False):
    f = ["font", ["size", fmt(size), fmt(size)]]
    if bold:
        f.append(["thickness", "0.254"])
        f.append(["bold", "yes"])
    return f


# ------------------------------------------------------------------ symbol library access
class SymLib:
    def __init__(self, paths):
        self.paths = paths          # nickname -> file
        self.text = {}
        self.cache = {}

    def _raw(self, nick, name):
        if nick not in self.text:
            self.text[nick] = open(self.paths[nick]).read()
        s = self.text[nick]
        key = f'\n\t(symbol "{name}"\n'
        i = s.find(key)
        if i < 0:
            raise KeyError(f"{nick}:{name} not found")
        d, j = 0, i + 1
        while True:
            c = s[j]
            if c == '"':
                j += 1
                while s[j] != '"':
                    j += 2 if s[j] == "\\" else 1
            elif c == "(":
                d += 1
            elif c == ")":
                d -= 1
                if d == 0:
                    break
            j += 1
        return parse(s[i + 1:j + 1])

    def get(self, nick, name):
        """Flattened symbol definition (extends resolved), top name = nick:name."""
        k = (nick, name)
        if k in self.cache:
            return self.cache[k]
        sym = self._raw(nick, name)
        ext = find(sym, "extends")
        if ext:
            base = self._raw(nick, ext[1])
            props = {c[1]: c for c in find_all(sym, "property")}
            new = [x for x in base if not (isinstance(x, list) and x[0] == "property")]
            new[1] = QStr(name)
            base_props = find_all(base, "property")
            out_props = []
            for bp in base_props:
                out_props.append(props.pop(bp[1], bp))
            out_props.extend(props.values())
            # insert props after flags
            idx = next(i for i, x in enumerate(new) if isinstance(x, list) and x[0] == "symbol")
            new = new[:idx] + out_props + new[idx:]
            for c in new:
                if isinstance(c, list) and c[0] == "symbol":
                    c[1] = QStr(re.sub("^" + re.escape(ext[1]) + "_", name + "_", c[1]))
            sym = new
        full = [x for x in sym]
        full[1] = QStr(f"{nick}:{name}")
        self.cache[k] = full
        return full


def unit_pins(sym, unit):
    """Pins (dict) for one unit (unit 0 = common) of body style 1."""
    name = sym[1].split(":", 1)[1]
    pins = []
    for sub in find_all(sym, "symbol"):
        m = re.match(re.escape(name) + r"_(\d+)_(\d+)$", sub[1])
        if not m:
            continue
        u, style = int(m.group(1)), int(m.group(2))
        if style not in (0, 1) or u not in (0, unit):
            continue
        for p in find_all(sub, "pin"):
            at = find(p, "at")
            hide = find(p, "hide")
            pins.append(dict(num=str(find(p, "number")[1]), name=str(find(p, "name")[1]),
                             etype=p[1], x=float(at[1]), y=float(at[2]), a=int(float(at[3])),
                             length=float(find(p, "length")[1]),
                             hidden=bool(hide and hide[-1] in ("yes",)) or "hide" in p))
    return pins


def unit_count(sym):
    name = sym[1].split(":", 1)[1]
    units = set()
    for sub in find_all(sym, "symbol"):
        m = re.match(re.escape(name) + r"_(\d+)_(\d+)$", sub[1])
        if m:
            units.add(int(m.group(1)))
    units.discard(0)
    return max(units) if units else 1


def body_bbox(sym, unit):
    """Graphics bbox (lib coords, y up) incl. pins, for one unit."""
    name = sym[1].split(":", 1)[1]
    xs, ys = [], []
    for sub in find_all(sym, "symbol"):
        m = re.match(re.escape(name) + r"_(\d+)_(\d+)$", sub[1])
        if not m or int(m.group(1)) not in (0, unit) or int(m.group(2)) not in (0, 1):
            continue
        for g in sub:
            if not isinstance(g, list):
                continue
            if g[0] == "rectangle":
                for k in ("start", "end"):
                    v = find(g, k)
                    xs.append(float(v[1])); ys.append(float(v[2]))
            elif g[0] == "polyline" or g[0] == "bezier":
                for xy in find(g, "pts")[1:]:
                    xs.append(float(xy[1])); ys.append(float(xy[2]))
            elif g[0] == "circle":
                c = find(g, "center"); r = float(find(g, "radius")[1])
                xs += [float(c[1]) - r, float(c[1]) + r]; ys += [float(c[2]) - r, float(c[2]) + r]
            elif g[0] == "arc":
                for k in ("start", "mid", "end"):
                    v = find(g, k)
                    xs.append(float(v[1])); ys.append(float(v[2]))
            elif g[0] == "pin":
                at = find(g, "at")
                x, y, a = float(at[1]), float(at[2]), float(at[3])
                ln = float(find(g, "length")[1])
                xs += [x, x + ln * math.cos(math.radians(a))]
                ys += [y, y + ln * math.sin(math.radians(a))]
    if not xs:
        return (-1.27, -1.27, 1.27, 1.27)
    return (min(xs), min(ys), max(xs), max(ys))


def rot(x, y, r):
    """lib (x, y-up) rotated CCW by r degrees -> screen offset (dx, dy-down)."""
    rr = math.radians(r)
    xr = x * math.cos(rr) - y * math.sin(rr)
    yr = x * math.sin(rr) + y * math.cos(rr)
    return round(xr, 4), round(-yr, 4)


def out_dir(a, r):
    """Outward direction (screen unit vector) of a pin whose lib angle is a."""
    ang = math.radians(a + 180 + r)
    return round(math.cos(ang)), round(-math.sin(ang))


# ------------------------------------------------------------------ exporter
class Placed:
    def __init__(self, part, unit, sym, rotation):
        self.part, self.unit, self.sym, self.rot = part, unit, sym, rotation
        self.x = self.y = 0.0


def skyline(boxes, width):
    """Bottom-left skyline packing, keeps input order. boxes: [(w, h)].
    Returns ([(x, y)], used_width, used_height)."""
    sky = [(0.0, max(width, max(w for w, h in boxes)), 0.0)]   # (x0, x1, height)
    out = []
    for w, h in boxes:
        best = None
        cands = sorted({seg[0] for seg in sky})
        for x in cands:
            if x + w > sky[-1][1] + 1e-6:
                continue
            y = max(seg[2] for seg in sky if seg[1] > x + 1e-6 and seg[0] < x + w - 1e-6)
            if best is None or y < best[1] - 1e-6 or (abs(y - best[1]) < 1e-6 and x < best[0]):
                best = (x, y)
        if best is None:
            best = (0.0, max(seg[2] for seg in sky))
        x, y = best
        out.append((x, y))
        new = []
        for x0, x1, hh in sky:
            if x1 <= x or x0 >= x + w:
                new.append((x0, x1, hh))
            else:
                if x0 < x:
                    new.append((x0, x, hh))
                if x1 > x + w:
                    new.append((x + w, x1, hh))
        new.append((x, x + w, y + h))
        sky = sorted(new)
    used_w = max(x + w for (x, y), (w, h) in zip(out, boxes))
    used_h = max(y + h for (x, y), (w, h) in zip(out, boxes))
    return out, used_w, used_h


def text_len(s, size=1.27):
    return len(s) * CHAR_W * size / 1.27


class FlagPart:
    """Stand-in for a KiCad PWR_FLAG (virtual symbol, not a skidl part)."""

    def __init__(self, ref, net, sheet):
        self.ref, self.name, self.value, self.footprint = ref, "PWR_FLAG", "PWR_FLAG", ""
        self.sw_group, self.sw_sheet = "ERC power flags", sheet
        self.sw_props, self.sw_dnp = {}, False
        self._sw_pin_net = {"1": net}


def export_project(circuit, outdir, project, sheets, title="", flag_nets=()):
    libdir = os.path.join(outdir, "lib")
    paths = {"steering_wheel": os.path.join(libdir, "steering_wheel.kicad_sym")}
    for f in os.listdir(os.path.join(libdir, "std")):
        if f.endswith(".kicad_sym"):
            paths[f[:-10]] = os.path.join(libdir, "std", f)
    lib = SymLib(paths)

    root_uuid = uid(project, "root")
    sheet_uuid = {s: uid(project, "sheet", s) for s, _ in sheets}
    sheet_index = {s: i for i, (s, _) in enumerate(sheets)}

    # ---- pin -> net map
    def net_of(pin):
        n = pin.net
        if n is None:
            return None
        nm = n.name
        if type(n).__name__ == "NCNet" or nm.upper() == "NC" or nm.startswith("__NC"):
            return "__NC__"
        return nm

    placements = {s: [] for s, _ in sheets}
    nick_of = {}
    for p in circuit.parts:
        nick = os.path.splitext(os.path.basename(str(p.lib.filename)))[0] if p.lib else None
        nick_of[p] = nick
        sym = lib.get(nick, p.name)
        nunits = unit_count(sym)
        pin_net = {}
        for pin in p.pins:
            pin_net.setdefault(str(pin.num), net_of(pin))
        p._sw_pin_net = pin_net
        unit_sheets = getattr(p, "sw_unit_sheets", None) or {u: p.sw_sheet for u in range(1, nunits + 1)}
        for u, sh in unit_sheets.items():
            pins = unit_pins(sym, u)
            r = 0
            nums = [q["num"] for q in pins]
            if len(pins) == 2 and p.name in ("R", "C", "L", "C_Polarized"):
                nets = [pin_net.get(n) for n in nums]
                if not all(n in POWER_SYMBOLS for n in nets):
                    r = 90
            placements[sh].append(Placed(p, u, sym, r))

    for i, (net, sh) in enumerate(flag_nets, start=1):
        fp_ = FlagPart(f"#FLG{i:02d}", net, sh)
        nick_of[fp_] = "power"
        placements[sh].append(Placed(fp_, 1, lib.get("power", "PWR_FLAG"), 0))

    # nets -> sheets
    net_sheets = {}
    for sh, pl in placements.items():
        for P in pl:
            for q in unit_pins(P.sym, P.unit):
                n = P.part._sw_pin_net.get(q["num"])
                if n and n != "__NC__":
                    net_sheets.setdefault(n, set()).add(sh)

    used_syms = {}
    pwr_count = [0]
    sheet_files = {}
    today = datetime.date.today().isoformat()

    for sh, stitle in sheets:
        items = []
        su = sheet_uuid[sh]
        path = f"/{root_uuid}/{su}"
        # ------------------------------------------------ geometry of each placement
        cells = []
        for P in placements[sh]:
            pins = unit_pins(P.sym, P.unit)
            bx0, by0, bx1, by1 = body_bbox(P.sym, P.unit)
            corners = [rot(x, y, P.rot) for x, y in ((bx0, by0), (bx0, by1), (bx1, by0), (bx1, by1))]
            cx0 = min(c[0] for c in corners); cx1 = max(c[0] for c in corners)
            cy0 = min(c[1] for c in corners); cy1 = max(c[1] for c in corners)
            body = (cx0, cy0, cx1, cy1)
            ext = [cx0, cy0, cx1, cy1]
            seen = set()
            for q in pins:
                dx, dy = rot(q["x"], q["y"], P.rot)
                if (dx, dy) in seen:
                    continue
                seen.add((dx, dy))
                n = P.part._sw_pin_net.get(q["num"])
                if n is None:
                    continue
                ox, oy = out_dir(q["a"], P.rot)
                if n == "__NC__":
                    reach = 1.27
                elif n in POWER_SYMBOLS:
                    reach = STUB + (3.6 + text_len(n) if ox else 5.5)
                else:
                    reach = STUB + text_len(n) + (3.2 if len(net_sheets.get(n, ())) > 1 else 0.8)
                px, py = dx + ox * reach, dy + oy * reach
                side = 1.9 if n in POWER_SYMBOLS else 1.3
                if ox:  # horizontal reach
                    ext[0] = min(ext[0], px); ext[2] = max(ext[2], px)
                    ext[1] = min(ext[1], dy - side); ext[3] = max(ext[3], dy + side)
                else:
                    ext[1] = min(ext[1], py); ext[3] = max(ext[3], py)
                    w = (text_len(n) / 2 + 1) if n in POWER_SYMBOLS else 1.3
                    ext[0] = min(ext[0], dx - w); ext[2] = max(ext[2], dx + w)
            # reference / value text
            ref = P.part.ref
            val = str(P.part.value)
            small = len(pins) <= 2 and P.part.name in ("R", "C", "L", "C_Polarized")
            if small and P.rot == 0:
                tw = max(text_len(ref), text_len(val)) + 2.5
                ext[2] = max(ext[2], cx1 + tw)
            elif small:
                ext[1] = min(ext[1], cy0 - 3.0)
                ext[3] = max(ext[3], cy1 + 3.0)
            else:
                ext[1] = min(ext[1], cy0 - 5.2)
                ext[2] = max(ext[2], cx0 + max(text_len(ref), text_len(val)) + 1)
            cells.append(dict(P=P, ext=ext, body=body, small=small))

        # ------------------------------------------------ layout
        groups = []
        for c in cells:
            ug = getattr(c["P"].part, "sw_unit_groups", None)
            g = ug.get(c["P"].unit, c["P"].part.sw_group) if ug else c["P"].part.sw_group
            if not groups or groups[-1][0] != g:
                # merge with an earlier group of the same name (e.g. RW612 units)
                prev = [gg for gg in groups if gg[0] == g]
                if prev:
                    prev[0][1].append(c)
                    continue
                groups.append([g, [c]])
            else:
                groups[-1][1].append(c)

        def gkey(gr):
            nat = [c["P"].part.sw_order for c in gr[1]
                   if not getattr(c["P"].part, "sw_unit_groups", None)
                   and hasattr(c["P"].part, "sw_order")]
            return min(nat) if nat else min(getattr(c["P"].part, "sw_order", 1e9) for c in gr[1])
        groups.sort(key=lambda gr: (gr[0] == "ERC power flags", gkey(gr)))

        def layout(page_w, gmax):
            usable = page_w - 30
            gboxes = []
            for gname, gcells in groups:
                big = max((c["ext"][2] - c["ext"][0]) + 2.54 for c in gcells)
                maxw = min(usable, max(gmax, big))
                boxes = [(c["ext"][2] - c["ext"][0] + 2.54, c["ext"][3] - c["ext"][1] + 2.54)
                         for c in gcells]
                spots, gw, gh = skyline(boxes, maxw)
                pos = [(c, x, y) for c, (x, y) in zip(gcells, spots)]
                gboxes.append((gname, pos, gw, gh + 7.0))
            spots, W, H = skyline([(gw + 8, gh + 5) for _, _, gw, gh in gboxes], usable)
            placed = [(g[0], g[1], x, y, g[2], g[3]) for g, (x, y) in zip(gboxes, spots)]
            return placed, H

        best = None
        for pname, pw, ph in PAPERS:
            for gmax in (120, 160, 200, 260):
                placed_, total_h = layout(pw, gmax)
                if total_h <= ph - 40:
                    if best is None or total_h < best[0]:
                        best = (total_h, placed_, (pname, pw, ph))
            if best:
                break
        if best is None:
            placed_, total_h = layout(PAPERS[-1][1], 260)
            best = (total_h, placed_, PAPERS[-1])
        _, placed, paper = best
        ox0, oy0 = 15.24, 20.32

        # ------------------------------------------------ emit symbols, wires, labels
        for gname, pos, GX, GY, gw, gh in placed:
            items.append(["text", QStr(gname), ["exclude_from_sim", "no"],
                          ["at", fmt(snap(ox0 + GX)), fmt(snap(oy0 + GY + 2.54)), "0"],
                          ["effects", font(2.0, bold=True), ["justify", "left", "bottom"]],
                          ["uuid", QStr(uid(sh, "grouptext", gname))]])
            items.append(["rectangle", ["start", fmt(snap(ox0 + GX - 2.54)), fmt(snap(oy0 + GY - 1.27))],
                          ["end", fmt(snap(ox0 + GX + gw)), fmt(snap(oy0 + GY + gh))],
                          ["stroke", ["width", "0.1"], ["type", "dash"], ["color", "132", "132", "132", "1"]],
                          ["fill", ["type", "none"]],
                          ["uuid", QStr(uid(sh, "grouprect", gname))]])
            for c, px, py in pos:
                P = c["P"]
                sx = snap(ox0 + GX + px - c["ext"][0], 2.54)
                sy = snap(oy0 + GY + 7.0 + py - c["ext"][1], 2.54)
                P.x, P.y = sx, sy
                items.append(symbol_instance(P, sx, sy, path, project, nick_of[P.part], c))
                used_syms[P.sym[1]] = P.sym
                emit_pin_nets(P, sx, sy, items, net_sheets, sh, path, project, lib, used_syms,
                              pwr_count)

        sheet_files[sh] = (paper, items, stitle)

    # ---------------------------------------------------- write sub-sheets
    for i, (sh, stitle) in enumerate(sheets):
        paper, items, stitle = sheet_files[sh]
        doc = ["kicad_sch", ["version", "20250114"], ["generator", QStr("eeschema")],
               ["generator_version", QStr("9.0")], ["uuid", QStr(uid(project, "file", sh))],
               ["paper", QStr(paper[0])],
               title_block(stitle, today, title, i + 2),
               ["lib_symbols"] + [used_syms[k] for k in sorted(used_syms) if _sym_used(k, items)]]
        doc += items
        doc.append(["embedded_fonts", "no"])
        with open(os.path.join(outdir, f"{sh}.kicad_sch"), "w") as fh:
            fh.write(dumps(doc) + "\n")

    # ---------------------------------------------------- root sheet
    write_root(outdir, project, sheets, root_uuid, sheet_uuid, title, today)
    write_project_files(outdir, project, sheets, root_uuid, sheet_uuid)
    return placements


def _sym_used(lib_id, items):
    for it in items:
        if isinstance(it, list) and it[0] == "symbol":
            if find(it, "lib_id")[1] == lib_id:
                return True
    return False


def title_block(stitle, date, company_title, page):
    return ["title_block", ["title", QStr(stitle)], ["date", QStr(date)], ["rev", QStr("0.1")],
            ["company", QStr("IRT")],
            ["comment", "1", QStr(company_title)],
            ["comment", "2", QStr("Generated from steering_wheel.py (SKiDL) - edit the script, not this sheet")]]


def prop(name, value, x, y, angle=0, hide=False, justify=None, size=1.27):
    eff = ["effects", font(size)]
    if justify:
        eff.append(["justify"] + justify.split())
    if hide:
        eff.append(["hide", "yes"])
    return ["property", QStr(name), QStr(value), ["at", fmt(x), fmt(y), str(angle)], eff]


def symbol_instance(P, sx, sy, path, project, nick, cell):
    part = P.part
    ref = part.ref
    lib_id = P.sym[1]
    x0, y0, x1, y1 = cell["body"]
    val = str(part.value)
    fp = part.footprint or ""
    props = []
    if cell["small"] and P.rot == 0:
        props.append(prop("Reference", ref, sx + x1 + 1.27, sy - 1.27, justify="left"))
        props.append(prop("Value", val, sx + x1 + 1.27, sy + 1.27, justify="left"))
    elif cell["small"]:
        props.append(prop("Reference", ref, sx, sy + y0 - 1.27))
        props.append(prop("Value", val, sx, sy + y1 + 1.27))
    else:
        props.append(prop("Reference", ref, sx + x0, sy + y0 - 3.4, justify="left"))
        props.append(prop("Value", val, sx + x0, sy + y0 - 1.3, justify="left"))
    props.append(prop("Footprint", fp, sx, sy, hide=True))
    ds = ""
    descr = ""
    for c in find_all(P.sym, "property"):
        if c[1] == "Datasheet":
            ds = str(c[2])
        if c[1] == "Description":
            descr = str(c[2])
    props.append(prop("Datasheet", ds, sx, sy, hide=True))
    props.append(prop("Description", descr, sx, sy, hide=True))
    for k, v in (getattr(part, "sw_props", None) or {}).items():
        props.append(prop(k, v, sx, sy, hide=True))
    dnp = bool(getattr(part, "sw_dnp", False))
    is_virtual = ref.startswith("#")
    inst = ["symbol", ["lib_id", QStr(lib_id)], ["at", fmt(sx), fmt(sy), str(P.rot)],
            ["unit", str(P.unit)], ["exclude_from_sim", "no"],
            ["in_bom", "no" if is_virtual else "yes"], ["on_board", "no" if is_virtual else "yes"],
            ["dnp", "yes" if dnp else "no"], ["uuid", QStr(uid("sym", path, ref, P.unit))]]
    inst += props
    for q in unit_pins(P.sym, P.unit):
        inst.append(["pin", QStr(q["num"]), ["uuid", QStr(uid("pin", path, ref, q["num"]))]])
    inst.append(["instances", ["project", QStr(project),
                               ["path", QStr(path), ["reference", QStr(ref)], ["unit", str(P.unit)]]]])
    return inst


def _wire(items, a, b, tag):
    items.append(["wire", ["pts", ["xy", fmt(a[0]), fmt(a[1])], ["xy", fmt(b[0]), fmt(b[1])]],
                  ["stroke", ["width", "0"], ["type", "default"]], ["uuid", QStr(uid("w", *tag))]])


def _power_symbol(items, lib, used_syms, pwr_count, n, pt, d, path, project, tag):
    nick, sname = POWER_SYMBOLS[n]
    sym = lib.get(nick, sname)
    used_syms[sym[1]] = sym
    pwr_count[0] += 1
    ref = f"#PWR{pwr_count[0]:03d}"
    ground = n in GROUND_NETS
    if ground:   # natural outward direction: down for ground, up for rails
        r = {(0, 1): 0, (1, 0): 90, (0, -1): 180, (-1, 0): 270}[d]
    else:
        r = {(0, -1): 0, (-1, 0): 90, (0, 1): 180, (1, 0): 270}[d]
    ex, ey = pt
    off = 3.3 if ground else 3.0
    vx, vy, just = {(1, 0): (off, 0, "left"), (-1, 0): (-off, 0, "right"),
                    (0, -1): (0, -off, None), (0, 1): (0, off + 0.6, None)}[d]
    items.append(["symbol", ["lib_id", QStr(sym[1])], ["at", fmt(ex), fmt(ey), str(r)],
                  ["unit", "1"], ["exclude_from_sim", "no"], ["in_bom", "yes"],
                  ["on_board", "yes"], ["dnp", "no"],
                  ["uuid", QStr(uid("pwr", *tag))],
                  prop("Reference", ref, ex, ey, hide=True),
                  prop("Value", n, round(ex + vx, 4), round(ey + vy, 4), justify=just),
                  prop("Footprint", "", ex, ey, hide=True),
                  prop("Datasheet", "", ex, ey, hide=True),
                  prop("Description", f'Power symbol creates a global label with name "{n}"',
                       ex, ey, hide=True),
                  ["pin", QStr("1"), ["uuid", QStr(uid("pwrpin", *tag))]],
                  ["instances", ["project", QStr(project),
                                 ["path", QStr(path), ["reference", QStr(ref)], ["unit", "1"]]]]])


def emit_pin_nets(P, sx, sy, items, net_sheets, sh, path, project, lib, used_syms, pwr_count):
    # unique pin positions (stacked pins share one connection point)
    spots = {}
    for q in unit_pins(P.sym, P.unit):
        dx, dy = rot(q["x"], q["y"], P.rot)
        n = P.part._sw_pin_net.get(q["num"])
        k = (dx, dy)
        if k in spots:
            if spots[k]["net"] != n:
                raise ValueError(f"{P.part.ref}: stacked pins at {k} on different nets "
                                 f"{spots[k]['net']} / {n}")
            continue
        spots[k] = dict(net=n, num=q["num"], d=out_dir(q["a"], P.rot),
                        p=(round(sx + dx, 4), round(sy + dy, 4)))
    # power-net pins on the same side that follow each other are joined by one wire
    by_side = {}
    for sp in spots.values():
        by_side.setdefault(sp["d"], []).append(sp)
    for d, lst in by_side.items():
        horiz = d[0] != 0            # pins stacked vertically, stubs horizontal
        lst.sort(key=lambda s: s["p"][1] if horiz else s["p"][0])
        runs, cur = [], []
        for spt in lst:
            if cur and spt["net"] == cur[-1]["net"] and spt["net"] in POWER_SYMBOLS:
                cur.append(spt)
            else:
                if cur:
                    runs.append(cur)
                cur = [spt]
        if cur:
            runs.append(cur)
        for run in runs:
            n = run[0]["net"]
            tagbase = (path, P.part.ref, P.unit, run[0]["num"])
            if n is None:
                continue
            if n == "__NC__":
                for spt in run:
                    items.append(["no_connect", ["at", fmt(spt["p"][0]), fmt(spt["p"][1])],
                                  ["uuid", QStr(uid("nc", path, P.part.ref, spt["num"]))]])
                continue
            ends = []
            for spt in run:
                px, py = spt["p"]
                e = (round(px + d[0] * STUB, 4), round(py + d[1] * STUB, 4))
                _wire(items, spt["p"], e, (path, P.part.ref, spt["num"]))
                ends.append(e)
            for i in range(1, len(ends)):
                _wire(items, ends[i - 1], ends[i], (path, P.part.ref, run[i]["num"], "bus"))
                if i < len(ends) - 1:
                    items.append(["junction", ["at", fmt(ends[i][0]), fmt(ends[i][1])],
                                  ["diameter", "0"], ["color", "0", "0", "0", "0"],
                                  ["uuid", QStr(uid("j", path, P.part.ref, run[i]["num"]))]])
            if n in POWER_SYMBOLS:
                # symbol at the run end matching its natural direction
                if horiz:
                    at = ends[-1] if n in GROUND_NETS else ends[0]
                else:
                    at = ends[0]
                _power_symbol(items, lib, used_syms, pwr_count, n, at, d, path, project, tagbase)
                continue
            ex, ey = ends[0]
            angle, just = {(1, 0): (0, "left"), (-1, 0): (180, "right"), (0, -1): (90, "left"),
                           (0, 1): (270, "right")}[d]
            if len(net_sheets.get(n, ())) > 1:
                items.append(["global_label", QStr(n), ["shape", "bidirectional"],
                              ["at", fmt(ex), fmt(ey), str(angle)], ["fields_autoplaced", "yes"],
                              ["effects", font(), ["justify", just]],
                              ["uuid", QStr(uid("gl", *tagbase))],
                              ["property", QStr("Intersheetrefs"), QStr("${INTERSHEET_REFS}"),
                               ["at", fmt(ex), fmt(ey), "0"],
                               ["effects", font(), ["justify", just], ["hide", "yes"]]]])
            else:
                items.append(["label", QStr(n), ["at", fmt(ex), fmt(ey), str(angle)],
                              ["fields_autoplaced", "yes"],
                              ["effects", font(), ["justify", just, "bottom"]],
                              ["uuid", QStr(uid("lb", *tagbase))]])


def write_root(outdir, project, sheets, root_uuid, sheet_uuid, title, today):
    items = []
    w, h = 60.96, 12.7
    for i, (sh, stitle) in enumerate(sheets):
        col, row = i % 3, i // 3
        x = 30.48 + col * (w + 20.32)
        y = 50.8 + row * (h + 22.86)
        items.append(["sheet", ["at", fmt(x), fmt(y)], ["size", fmt(w), fmt(h)],
                      ["exclude_from_sim", "no"], ["in_bom", "yes"], ["on_board", "yes"],
                      ["dnp", "no"], ["fields_autoplaced", "yes"],
                      ["stroke", ["width", "0.1524"], ["type", "solid"]],
                      ["fill", ["color", "0", "0", "0", "0.0000"]],
                      ["uuid", QStr(sheet_uuid[sh])],
                      ["property", QStr("Sheetname"), QStr(sh),
                       ["at", fmt(x), fmt(round(y - 0.7116, 4)), "0"],
                       ["effects", font(), ["justify", "left", "bottom"]]],
                      ["property", QStr("Sheetfile"), QStr(f"{sh}.kicad_sch"),
                       ["at", fmt(x), fmt(round(y + h + 0.5996, 4)), "0"],
                       ["effects", font(), ["justify", "left", "top"]]],
                      ["instances", ["project", QStr(project),
                                     ["path", QStr(f"/{root_uuid}"), ["page", QStr(str(i + 2))]]]]])
        items.append(["text", QStr(stitle), ["exclude_from_sim", "no"],
                      ["at", fmt(x + 1.27), fmt(y + h / 2 + 0.635), "0"],
                      ["effects", font(1.27), ["justify", "left", "bottom"]],
                      ["uuid", QStr(uid("roottext", sh))]])
    notes_path = os.path.join(outdir, "tools", "root_notes.txt")
    if os.path.exists(notes_path):
        notes = open(notes_path).read().rstrip()
        items.append(["text", QStr(notes), ["exclude_from_sim", "no"],
                      ["at", "30.48", "132.08", "0"],
                      ["effects", font(1.27), ["justify", "left", "top"]],
                      ["uuid", QStr(uid("rootnotes"))]])
    items.append(["text", QStr(title), ["exclude_from_sim", "no"], ["at", "30.48", "30.48", "0"],
                  ["effects", font(3.5, bold=True), ["justify", "left", "bottom"]],
                  ["uuid", QStr(uid("roottitle"))]])
    doc = ["kicad_sch", ["version", "20250114"], ["generator", QStr("eeschema")],
           ["generator_version", QStr("9.0")], ["uuid", QStr(root_uuid)], ["paper", QStr("A3")],
           title_block(title, today, "Top level: block diagram and notes", 1),
           ["lib_symbols"]] + items + [
        ["sheet_instances", ["path", QStr("/"), ["page", QStr("1")]]],
        ["embedded_fonts", "no"]]
    with open(os.path.join(outdir, f"{project}.kicad_sch"), "w") as fh:
        fh.write(dumps(doc) + "\n")


def write_project_files(outdir, project, sheets, root_uuid, sheet_uuid):
    pro_path = os.path.join(outdir, f"{project}.kicad_pro")
    if os.path.exists(pro_path):
        pro = json.load(open(pro_path))
    else:
        pro = json.load(open(os.path.join(outdir, "tools", "kicad_pro_template.json")))
        pro["boards"] = []
        pro["text_variables"] = {}
        nc = pro["net_settings"]["classes"][0]
        nc.update({"clearance": 0.1, "track_width": 0.15, "via_diameter": 0.4, "via_drill": 0.2,
                   "microvia_diameter": 0.25, "microvia_drill": 0.1})
        fields = pro["schematic"]["bom_settings"]["fields_ordered"]
        for extra in ("MPN", "Manufacturer", "LCSC", "Note"):
            fields.append({"group_by": extra == "MPN", "label": extra, "name": extra, "show": True})
    pro["meta"] = {"filename": f"{project}.kicad_pro", "version": pro.get("meta", {}).get("version", 3)}
    pro["sheets"] = [[root_uuid, "Root"]] + [[sheet_uuid[s], s] for s, t in sheets]
    with open(pro_path, "w") as fh:
        json.dump(pro, fh, indent=2)
        fh.write("\n")
    with open(os.path.join(outdir, "sym-lib-table"), "w") as fh:
        fh.write('(sym_lib_table\n  (version 7)\n'
                 '  (lib (name "steering_wheel")(type "KiCad")'
                 '(uri "${KIPRJMOD}/lib/steering_wheel.kicad_sym")(options "")'
                 '(descr "IRT steering wheel project symbols"))\n)\n')
    with open(os.path.join(outdir, "fp-lib-table"), "w") as fh:
        fh.write('(fp_lib_table\n  (version 7)\n'
                 '  (lib (name "steering_wheel")(type "KiCad")'
                 '(uri "${KIPRJMOD}/lib/steering_wheel.pretty")(options "")'
                 '(descr "IRT steering wheel project footprints"))\n)\n')
