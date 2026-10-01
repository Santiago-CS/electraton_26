"""Minimal KiCad board model + writer (KiCad 9 board format, version 20241229).

KiCad 10 opens this format natively (its parser accepts both net codes and net names).
Everything is placed on the top side; rotations are multiples of 90 degrees so every
pad stays an axis-aligned shape (keeps the DSN export and the checker simple).
"""
import math
import os
import re
import uuid

from sexp import QStr, dumps, find, find_all, parse

NS = uuid.UUID("0b6e3a57-4b0f-4a8e-bb61-3d1f5a7c9e21")


def uid(*a):
    return QStr(str(uuid.uuid5(NS, "/".join(map(str, a)))))


def f(v):
    s = f"{v:.4f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def rot_pt(x, y, deg):
    """Rotate local point by footprint angle (KiCad: CCW on screen, y down)."""
    r = math.radians(deg)
    c, s = round(math.cos(r), 12), round(math.sin(r), 12)
    return x * c + y * s, -x * s + y * c


class Pad:
    def __init__(self, node):
        self.node = node
        self.num = str(node[1])
        self.kind = node[2]             # smd / thru_hole / np_thru_hole
        self.shape = node[3]            # rect / roundrect / circle / oval
        at = find(node, "at")
        self.lx, self.ly = float(at[1]), float(at[2])
        self.lrot = float(at[3]) if len(at) > 3 else 0.0
        sz = find(node, "size")
        self.w, self.h = float(sz[1]), float(sz[2])
        d = find(node, "drill")
        self.drill = None
        if d:
            if d[1] == "oval":
                self.drill = (float(d[2]), float(d[3]))
            else:
                self.drill = (float(d[1]), float(d[1]))
        self.layers = [str(x) for x in find(node, "layers")[1:]]
        rr = find(node, "roundrect_rratio")
        self.rratio = float(rr[1]) if rr else 0.0

    def copper_layers(self, board_layers):
        out = []
        for l in self.layers:
            if l == "*.Cu":
                out += board_layers
            elif l.endswith(".Cu"):
                out.append(l)
        return out


class FootprintLib:
    def __init__(self, dirs):
        self.dirs = dirs    # nickname -> .pretty dir
        self.cache = {}

    def load(self, lib_id):
        if lib_id not in self.cache:
            nick, name = lib_id.split(":", 1)
            path = os.path.join(self.dirs[nick], name + ".kicad_mod")
            self.cache[lib_id] = parse(open(path).read())
        return self.cache[lib_id]


class Footprint:
    def __init__(self, ref, lib_id, node, value="", x=0.0, y=0.0, rot=0.0):
        self.ref, self.lib_id, self.node, self.value = ref, lib_id, node, value
        self.x, self.y, self.rot = x, y, rot
        self.pads = [Pad(p) for p in find_all(node, "pad")]
        self.pad_net = {}           # pad num -> net name
        self.path = self.sheetname = self.sheetfile = None
        self.props = {}
        self.dnp = False
        self.locked = False

    # --- geometry -----------------------------------------------------------
    def place(self, x, y, rot=0.0):
        self.x, self.y, self.rot = x, y, rot % 360
        return self

    def pad_xy(self, pad):
        dx, dy = rot_pt(pad.lx, pad.ly, self.rot)
        return self.x + dx, self.y + dy

    def pad_box(self, pad):
        """Axis-aligned (x0, y0, x1, y1) enclosing the pad copper, for any rotation.

        The old form only handled multiples of 90 degrees and swapped w/h for anything
        else, so a pad rotated by, say, 9 degrees came out turned a quarter turn.
        """
        x, y = self.pad_xy(pad)
        a = math.radians(self.rot + pad.lrot)
        ca, sa = abs(math.cos(a)), abs(math.sin(a))
        hx = pad.w / 2 * ca + pad.h / 2 * sa
        hy = pad.w / 2 * sa + pad.h / 2 * ca
        return (x - hx, y - hy, x + hx, y + hy)

    def pads_by_num(self, num):
        return [p for p in self.pads if p.num == str(num)]

    def courtyard(self):
        """Axis-aligned courtyard bbox in board coords (falls back to pads)."""
        pts = []
        for g in self.node:
            if not isinstance(g, list) or not g or g[0] not in ("fp_line", "fp_rect", "fp_circle",
                                                                  "fp_poly", "fp_arc"):
                continue
            lay = find(g, "layer")
            if not lay or lay[1] != "F.CrtYd":
                continue
            if g[0] in ("fp_line", "fp_rect"):
                for k in ("start", "end"):
                    v = find(g, k); pts.append((float(v[1]), float(v[2])))
            elif g[0] == "fp_arc":
                for k in ("start", "mid", "end"):
                    v = find(g, k); pts.append((float(v[1]), float(v[2])))
            elif g[0] == "fp_circle":
                c = find(g, "center"); e = find(g, "end")
                r = math.hypot(float(e[1]) - float(c[1]), float(e[2]) - float(c[2]))
                cx, cy = float(c[1]), float(c[2])
                pts += [(cx - r, cy - r), (cx + r, cy + r)]
            elif g[0] == "fp_poly":
                for xy in find(g, "pts")[1:]:
                    pts.append((float(xy[1]), float(xy[2])))
        if not pts:
            bs = [self.pad_box(p) for p in self.pads]
            return (min(b[0] for b in bs) - 0.25, min(b[1] for b in bs) - 0.25,
                    max(b[2] for b in bs) + 0.25, max(b[3] for b in bs) + 0.25)
        tp = [rot_pt(x, y, self.rot) for x, y in pts]
        return (self.x + min(p[0] for p in tp), self.y + min(p[1] for p in tp),
                self.x + max(p[0] for p in tp), self.y + max(p[1] for p in tp))


class Board:
    COPPER = ["F.Cu", "In1.Cu", "In2.Cu", "In3.Cu", "In4.Cu",
              "In5.Cu", "In6.Cu", "B.Cu"]

    def __init__(self, title=""):
        self.title = title
        self.fps = {}
        self.nets = [""]
        self.tracks = []      # dict(layer, x1, y1, x2, y2, w, net, locked)
        self.vias = []        # dict(x, y, d, drill, net, locked)
        self.zones = []       # dict(net, layer(s), pts, priority, clearance, min_w, keepout...)
        self.graphics = []    # raw sexp nodes (Edge.Cuts etc.)
        self.outline = []     # polygon pts (board edge)

    def net(self, name):
        if name not in self.nets:
            self.nets.append(name)
        return self.nets.index(name)

    def add_fp(self, fp):
        self.fps[fp.ref] = fp
        for n in fp.pad_net.values():
            if n:
                self.net(n)
        return fp

    def track(self, layer, a, b, w, net, locked=False):
        self.net(net)
        t = dict(layer=layer, x1=a[0], y1=a[1], x2=b[0], y2=b[1], w=w, net=net, locked=locked)
        self.tracks.append(t)
        return t

    def via(self, x, y, d, drill, net, locked=False):
        self.net(net)
        v = dict(x=x, y=y, d=d, drill=drill, net=net, locked=locked)
        self.vias.append(v)
        return v

    # --- writer -------------------------------------------------------------
    def _fp_node(self, fp):
        lib = fp.node
        rot = fp.rot
        out = ["footprint", QStr(fp.lib_id), ["layer", QStr("F.Cu")], ["uuid", uid("fp", fp.ref)]]
        at = ["at", f(fp.x), f(fp.y)] + ([f(rot)] if rot else [])
        out.append(at)
        if fp.locked:
            out.insert(2, ["locked", "yes"])
        for k in ("descr", "tags"):
            n = find(lib, k)
            if n:
                out.append(n)

        def with_angle(node):
            node = [c for c in node]
            a = find(node, "at")
            if a:
                i = node.index(a)
                base = float(a[3]) if len(a) > 3 else 0.0
                na = ["at", a[1], a[2], f((base + rot) % 360)]
                if a[-1] == "unlocked":
                    na.append("unlocked")
                node[i] = na
            return node

        props = {str(p[1]): p for p in find_all(lib, "property")}
        ref = props.get("Reference")
        val = props.get("Value")
        for name, value, src, layer, hide in (
                ("Reference", fp.ref, ref, "F.SilkS", False),
                ("Value", fp.value, val, "F.Fab", False),
                ("Footprint", fp.lib_id, None, "F.Fab", True),
                ("Datasheet", fp.props.get("Datasheet", ""), None, "F.Fab", True),
                ("Description", fp.props.get("Description", ""), None, "F.Fab", True)):
            if src is not None:
                node = with_angle(src)
                node[2] = QStr(value)
                node = [c for c in node if not (isinstance(c, list) and c[0] == "uuid")]
                node.insert(4, ["uuid", uid("prop", fp.ref, name)])
                ov = getattr(fp, "ref_override", None) if name == "Reference" else None
                if ov:
                    node = [c for c in node if not (isinstance(c, list) and c[0] == "hide")]
                    if ov.get("hide"):
                        node.insert(4, ["hide", "yes"])
                    else:
                        for i, c in enumerate(node):
                            if isinstance(c, list) and c[0] == "at":
                                node[i] = ["at", f(ov["x"]), f(ov["y"]), f(ov["angle"])]
                            elif isinstance(c, list) and c[0] == "effects":
                                node[i] = ["effects", ["font", ["size", f(ov["size"]), f(ov["size"])],
                                                       ["thickness", f(ov["thick"])]]]
            else:
                node = ["property", QStr(name), QStr(value), ["at", "0", "0", f(rot)],
                        ["layer", QStr(layer)], ["hide", "yes"], ["uuid", uid("prop", fp.ref, name)],
                        ["effects", ["font", ["size", "1.27", "1.27"], ["thickness", "0.15"]]]]
            out.append(node)
        for k, v in fp.props.items():
            if k in ("Datasheet", "Description"):
                continue
            out.append(["property", QStr(k), QStr(v), ["at", "0", "0", f(rot)], ["layer", QStr("F.Fab")],
                        ["hide", "yes"], ["uuid", uid("prop", fp.ref, k)],
                        ["effects", ["font", ["size", "1", "1"], ["thickness", "0.15"]]]])
        if fp.path:
            out.append(["path", QStr(fp.path)])
            out.append(["sheetname", QStr(fp.sheetname)])
            out.append(["sheetfile", QStr(fp.sheetfile)])
        attr = find(lib, "attr")
        a = [x for x in attr] if attr else ["attr"]
        if not getattr(fp, "board_only", False) and "exclude_from_bom" in a:
            a.remove("exclude_from_bom")      # every schematic symbol is in the BOM (parity)
        if fp.dnp and "dnp" not in a:
            a.append("dnp")
        if getattr(fp, "board_only", False):
            a += [t for t in ("board_only", "exclude_from_pos_files", "exclude_from_bom") if t not in a]
        if len(a) > 1:
            out.append(a)
        k = 0
        for c in lib:
            if not isinstance(c, list):
                continue
            # (duplicate_pad_numbers_are_jumpers no) is KiCad 10 only: KiCad 9 cannot parse it
            if c[0] in ("fp_line", "fp_rect", "fp_circle", "fp_arc", "fp_poly", "zone",
                        "embedded_fonts", "model", "group"):
                node = [x for x in c]
                if c[0] in ("fp_line", "fp_rect", "fp_circle", "fp_arc", "fp_poly") and not find(c, "uuid"):
                    node.append(["uuid", uid("g", fp.ref, k)])
                out.append(node)
            elif c[0] == "fp_text":
                node = with_angle(c)
                if not find(node, "uuid"):
                    node.insert(4, ["uuid", uid("t", fp.ref, k)])
                out.append(node)
            elif c[0] == "pad":
                node = with_angle(c)
                node = [x for x in node if not (isinstance(x, list) and x[0] in ("net", "uuid"))]
                netname = fp.pad_net.get(str(c[1])) or getattr(fp, "pad_nc", {}).get(str(c[1]))
                if netname:
                    node.append(["net", str(self.net(netname)), QStr(netname)])
                node.append(["uuid", uid("pad", fp.ref, k)])
                out.append(node)
            k += 1
        return out

    def write(self, path, stackup, extra=()):
        doc = ["kicad_pcb", ["version", "20241229"], ["generator", QStr("pcbnew")],
               ["generator_version", QStr("9.0")],
               ["general", ["thickness", "1.6"], ["legacy_teardrops", "no"]],
               ["paper", QStr("A3")],
               ["title_block", ["title", QStr(self.title)], ["rev", QStr("0.1")],
                ["company", QStr("IRT")]],
               ["layers",
                ["0", QStr("F.Cu"), "signal"], ["4", QStr("In1.Cu"), "power"],
                ["6", QStr("In2.Cu"), "signal"], ["8", QStr("In3.Cu"), "power"],
                ["10", QStr("In4.Cu"), "power"], ["12", QStr("In5.Cu"), "signal"],
                ["14", QStr("In6.Cu"), "power"], ["2", QStr("B.Cu"), "signal"],
                ["9", QStr("F.Adhes"), "user", QStr("F.Adhesive")],
                ["11", QStr("B.Adhes"), "user", QStr("B.Adhesive")],
                ["13", QStr("F.Paste"), "user"], ["15", QStr("B.Paste"), "user"],
                ["5", QStr("F.SilkS"), "user", QStr("F.Silkscreen")],
                ["7", QStr("B.SilkS"), "user", QStr("B.Silkscreen")],
                ["1", QStr("F.Mask"), "user"], ["3", QStr("B.Mask"), "user"],
                ["17", QStr("Dwgs.User"), "user", QStr("User.Drawings")],
                ["19", QStr("Cmts.User"), "user", QStr("User.Comments")],
                ["21", QStr("Eco1.User"), "user", QStr("User.Eco1")],
                ["23", QStr("Eco2.User"), "user", QStr("User.Eco2")],
                ["25", QStr("Edge.Cuts"), "user"], ["27", QStr("Margin"), "user"],
                ["31", QStr("F.CrtYd"), "user", QStr("F.Courtyard")],
                ["29", QStr("B.CrtYd"), "user", QStr("B.Courtyard")],
                ["35", QStr("F.Fab"), "user"], ["33", QStr("B.Fab"), "user"]],
               ["setup", stackup, ["pad_to_mask_clearance", "0"],
                ["allow_soldermask_bridges_in_footprints", "no"], ["tenting", "front", "back"]]]
        for i, n in enumerate(self.nets):
            doc.append(["net", str(i), QStr(n)])
        for ref in sorted(self.fps):
            doc.append(self._fp_node(self.fps[ref]))
        doc.extend(self.graphics)
        for i, t in enumerate(self.tracks):
            node = ["segment", ["start", f(t["x1"]), f(t["y1"])], ["end", f(t["x2"]), f(t["y2"])],
                    ["width", f(t["w"])], ["layer", QStr(t["layer"])], ["net", str(self.net(t["net"]))],
                    ["uuid", uid("seg", i, t["x1"], t["y1"], t["x2"], t["y2"], t["layer"])]]
            if t.get("locked"):
                node.insert(1, ["locked", "yes"])
            doc.append(node)
        for i, v in enumerate(self.vias):
            node = ["via", ["at", f(v["x"]), f(v["y"])], ["size", f(v["d"])], ["drill", f(v["drill"])],
                    ["layers", QStr("F.Cu"), QStr("B.Cu")], ["net", str(self.net(v["net"]))],
                    ["uuid", uid("via", i, v["x"], v["y"])]]
            if v.get("locked"):
                node.insert(1, ["locked", "yes"])
            doc.append(node)
        for i, z in enumerate(self.zones):
            doc.append(zone_node(self, z, i))
        doc.extend(extra)
        doc.append(["embedded_fonts", "no"])
        with open(path, "w") as fh:
            fh.write(dumps(doc) + "\n")


def zone_node(board, z, i):
    layers = z["layers"]
    node = ["zone"]
    if z.get("keepout"):
        node += [["net", "0"], ["net_name", QStr("")]]
    else:
        node += [["net", str(board.net(z["net"]))], ["net_name", QStr(z["net"])]]
    if len(layers) == 1:
        node.append(["layer", QStr(layers[0])])
    else:
        node.append(["layers"] + [QStr(l) for l in layers])
    node += [["uuid", uid("zone", i, z.get("net", ""), *layers)], ["hatch", "edge", "0.5"]]
    if z.get("name"):
        node.append(["name", QStr(z["name"])])
    node.append(["priority", str(z.get("priority", 0))])
    if z.get("keepout"):
        k = z["keepout"]
        node.append(["keepout", ["tracks", k.get("tracks", "not_allowed")],
                     ["vias", k.get("vias", "not_allowed")], ["pads", k.get("pads", "allowed")],
                     ["copperpour", k.get("copperpour", "not_allowed")],
                     ["footprints", k.get("footprints", "allowed")]])
        node.append(["connect_pads", ["clearance", "0"]])
    else:
        cp = ["connect_pads"]
        if z.get("connect") == "solid":
            cp.append("yes")
        cp.append(["clearance", f(z.get("clearance", 0.2))])
        node.append(cp)
    node += [["min_thickness", f(z.get("min_w", 0.15))], ["filled_areas_thickness", "no"],
             ["fill", "yes", ["thermal_gap", f(z.get("thermal_gap", 0.25))],
              ["thermal_bridge_width", f(z.get("thermal_w", 0.3))]] if not z.get("keepout") else
             ["fill", ["thermal_gap", "0.5"], ["thermal_bridge_width", "0.5"]],
             ["polygon", ["pts"] + [["xy", f(x), f(y)] for x, y in z["pts"]]]]
    return node


def stackup_jlc08161h_7628():
    """JLCPCB 8-layer JLC08161H-7628 (1.6 mm, 1 oz outer / 0.5 oz inner).

    Uniform build: 4x 7628 prepreg (0.2104 mm, er 4.4) and 3x 0.2 mm core (er 4.36),
    1.6028 mm total. The F.Cu -> In1.Cu prepreg is identical to the 4-layer
    JLC04161H-7628, so the 50 ohm RF microstrip keeps RF_W = 0.34 mm.
    """
    def pp(n):
        return ["layer", QStr(f"dielectric {n}"), ["type", QStr("prepreg")],
                ["thickness", "0.2104"], ["material", QStr("7628")],
                ["epsilon_r", "4.4"], ["loss_tangent", "0.02"]]

    def core(n):
        return ["layer", QStr(f"dielectric {n}"), ["type", QStr("core")],
                ["thickness", "0.2"], ["material", QStr("FR4")],
                ["epsilon_r", "4.36"], ["loss_tangent", "0.02"]]

    def cu(name, th):
        return ["layer", QStr(name), ["type", QStr("copper")], ["thickness", th]]

    return ["stackup",
            ["layer", QStr("F.SilkS"), ["type", QStr("Top Silk Screen")]],
            ["layer", QStr("F.Paste"), ["type", QStr("Top Solder Paste")]],
            ["layer", QStr("F.Mask"), ["type", QStr("Top Solder Mask")], ["thickness", "0.01"]],
            cu("F.Cu", "0.035"),
            pp(1),
            cu("In1.Cu", "0.0152"),
            core(2),
            cu("In2.Cu", "0.0152"),
            pp(3),
            cu("In3.Cu", "0.0152"),
            core(4),
            cu("In4.Cu", "0.0152"),
            pp(5),
            cu("In5.Cu", "0.0152"),
            core(6),
            cu("In6.Cu", "0.0152"),
            pp(7),
            cu("B.Cu", "0.035"),
            ["layer", QStr("B.Mask"), ["type", QStr("Bottom Solder Mask")], ["thickness", "0.01"]],
            ["layer", QStr("B.Paste"), ["type", QStr("Bottom Solder Paste")]],
            ["layer", QStr("B.SilkS"), ["type", QStr("Bottom Silk Screen")]],
            ["copper_finish", QStr("ENIG")], ["dielectric_constraints", "no"]]
