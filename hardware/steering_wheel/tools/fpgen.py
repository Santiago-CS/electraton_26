"""Generate project footprints (lib/steering_wheel.pretty) for parts KiCad doesn't ship.

  NXP_TFBGA145_8x8mm_P0.5mm        RW612ET (SOT2153-1), ball map from rw612_balls.json
  Skyworks_MLPD-6-1EP_1x1mm        SKY13323-378LF
  TDK_DPX165950DT_1608_6T          DPX165950DT-8085D1 diplexer

!! Land-pattern dimensions below are nominal values (IPC-style) chosen without the
!! vendors' drawings at hand. Check each against the official package drawing /
!! recommended land pattern before sending the board to fab.
"""
import json
import os
import sys
import uuid

sys.path.insert(0, os.path.dirname(__file__))
from sexp import find, QStr, dumps  # noqa: E402

NS = uuid.UUID("3b7f7c1e-2d8e-4a8b-9a55-0c2f7f1b9c33")


def uid(*a):
    return QStr(str(uuid.uuid5(NS, "/".join(map(str, a)))))


def f(v):
    s = f"{v:.4f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def line(layer, a, b, w, tag):
    return ["fp_line", ["start", f(a[0]), f(a[1])], ["end", f(b[0]), f(b[1])],
            ["stroke", ["width", f(w)], ["type", "solid"]], ["layer", QStr(layer)], ["uuid", uid(*tag)]]


def rect(layer, x0, y0, x1, y1, w, tag):
    return ["fp_rect", ["start", f(x0), f(y0)], ["end", f(x1), f(y1)],
            ["stroke", ["width", f(w)], ["type", "solid"]], ["fill", "no"], ["layer", QStr(layer)],
            ["uuid", uid(*tag)]]


def text(kind, value, x, y, layer, tag, hide=False, size=1.0):
    t = ["property", QStr(kind), QStr(value), ["at", f(x), f(y), "0"], ["layer", QStr(layer)]]
    if hide:
        t.append(["hide", "yes"])
    t += [["uuid", uid(*tag)],
          ["effects", ["font", ["size", f(size), f(size)], ["thickness", f(size * 0.15)]]]]
    return t


def footprint(name, descr, tags, body_w, body_h, pads, court_margin=0.25, extra=()):
    hw, hh = body_w / 2, body_h / 2
    # pad extents (land patterns can reach past the body)
    pex = max(abs(float(find(p, "at")[1])) + float(find(p, "size")[1]) / 2 for p in pads)
    pey = max(abs(float(find(p, "at")[2])) + float(find(p, "size")[2]) / 2 for p in pads)
    cx, cy = max(hw, pex) + court_margin, max(hh, pey) + court_margin
    fp = ["footprint", QStr(name), ["version", "20241229"], ["generator", QStr("steering_wheel_fpgen")],
          ["generator_version", QStr("9.0")], ["layer", QStr("F.Cu")],
          ["descr", QStr(descr)], ["tags", QStr(tags)],
          text("Reference", "REF**", 0, -cy - 0.8, "F.SilkS", (name, "ref")),
          text("Value", name, 0, cy + 0.8, "F.Fab", (name, "val")),
          text("Footprint", "", 0, 0, "F.Fab", (name, "fp"), hide=True),
          text("Datasheet", "", 0, 0, "F.Fab", (name, "ds"), hide=True),
          text("Description", descr, 0, 0, "F.Fab", (name, "de"), hide=True),
          ["attr", "smd"]]
    # fab outline with pin-1 chamfer
    ch = min(body_w, body_h) * 0.2
    pts = [(-hw + ch, -hh), (hw, -hh), (hw, hh), (-hw, hh), (-hw, -hh + ch), (-hw + ch, -hh)]
    for i in range(len(pts) - 1):
        fp.append(line("F.Fab", pts[i], pts[i + 1], 0.1, (name, "fab", i)))
    # silk corner marks kept 0.2 mm clear of the pads; pin-1 dot at the corner of pad 1
    k = min(body_w, body_h) * 0.25
    sx, sy = max(hw + 0.11, pex + 0.2), max(hh + 0.11, pey + 0.2)
    p1 = [p for p in pads if str(p[1]) in ("1", "A1")][0]
    qx = -1 if float(find(p1, "at")[1]) <= 0 else 1
    qy = -1 if float(find(p1, "at")[2]) <= 0 else 1
    corners = {(-1, -1): [(-sx, -sy + k, -sx, -sy), (-sx, -sy, -sx + k, -sy)],
               (1, -1): [(sx - k, -sy, sx, -sy), (sx, -sy, sx, -sy + k)],
               (1, 1): [(sx, sy - k, sx, sy), (sx, sy, sx - k, sy)],
               (-1, 1): [(-sx + k, sy, -sx, sy), (-sx, sy, -sx, sy - k)]}
    i = 0
    for q, segs in corners.items():
        if q == (qx, qy):
            continue  # pin-1 corner left open, marked by the dot
        for xa, ya, xb, yb in segs:
            fp.append(line("F.SilkS", (xa, ya), (xb, yb), 0.12, (name, "silk", i)))
            i += 1
    # dot level with pad 1, just outside the outline (a diagonal corner dot can land on
    # a neighbouring part's pad in tight RF layouts)
    dx, dy = qx * (sx + 0.3), float(find(p1, "at")[2])
    fp.append(["fp_circle", ["center", f(dx), f(dy)], ["end", f(dx + 0.1), f(dy)],
               ["stroke", ["width", "0.2"], ["type", "solid"]], ["fill", "yes"],
               ["layer", QStr("F.SilkS")], ["uuid", uid(name, "p1")]])
    fp.append(rect("F.CrtYd", -cx, -cy, cx, cy, 0.05, (name, "crt")))
    fp.extend(extra)
    fp.extend(pads)
    fp.append(["embedded_fonts", "no"])
    return fp


def smd_pad(num, shape, x, y, w, h, tag, mask_margin=None, paste=True, rratio=None):
    layers = ["F.Cu", "F.Mask"] + (["F.Paste"] if paste else [])
    p = ["pad", QStr(num), "smd", shape, ["at", f(x), f(y)], ["size", f(w), f(h)],
         ["layers"] + [QStr(l) for l in layers]]
    if rratio is not None:
        p.append(["roundrect_rratio", f(rratio)])
    if mask_margin is not None:
        p.append(["solder_mask_margin", f(mask_margin)])
    p.append(["uuid", uid(*tag)])
    return p


def bga(balls_json):
    balls = json.load(open(balls_json))["pins"]
    rows = "ABCDEFGHJKLMNPR"
    pads = []
    for b in sorted(balls, key=lambda b: (rows.index(b[0]), int(b[1:]))):
        r, c = rows.index(b[0]), int(b[1:])
        x, y = (c - 8) * 0.5, (r - 7) * 0.5
        pads.append(smd_pad(b, "circle", x, y, 0.25, 0.25, ("bga", b), mask_margin=0.05))
    return footprint("NXP_TFBGA145_8x8mm_P0.5mm",
                     "NXP SOT2153-1 TFBGA145, 8x8x1.07 mm, 0.5 mm pitch, 15x15 grid (RW612ET). "
                     "Pads 0.25 mm NSMD, mask +0.05 mm: VERIFY against NXP SOT2153-1 reflow "
                     "footprint. Fan-out: 0.3/0.15 mm dogbone vias in the depopulated grid sites.",
                     "BGA 0.5mm RW612 SOT2153-1", 8.0, 8.0, pads, court_margin=0.5)


def mlpd6():
    pads = []
    pitch, px, pw, pl = 0.35, 0.45, 0.20, 0.30   # row centre x, pad w (y), pad l (x)
    for i in range(3):
        y = (i - 1) * pitch
        pads.append(smd_pad(str(i + 1), "roundrect", -px, y, pl, pw, ("mlpd", i + 1), rratio=0.25))
        pads.append(smd_pad(str(6 - i), "roundrect", px, y, pl, pw, ("mlpd", 6 - i), rratio=0.25))
    pads.append(smd_pad("7", "rect", 0, 0, 0.30, 0.70, ("mlpd", "ep")))
    return footprint("Skyworks_MLPD-6-1EP_1x1mm",
                     "Skyworks 6-pin MLPD 1x1 mm with exposed pad (SKY13323-378LF). Nominal "
                     "0.35 mm pitch land pattern: VERIFY against the Skyworks package drawing.",
                     "MLPD DFN-6 Skyworks SKY13323", 1.0, 1.0, pads, court_margin=0.3)


def dpx():
    pads = []
    for i, x in enumerate((-0.55, 0.0, 0.55)):
        pads.append(smd_pad(str(i + 1), "rect", x, 0.55, 0.30, 0.60, ("dpx", i + 1)))   # 1..3 bottom
        pads.append(smd_pad(str(6 - i), "rect", x, -0.55, 0.30, 0.60, ("dpx", 6 - i)))  # 6..4 top
    return footprint("TDK_DPX165950DT_1608_6T",
                     "TDK DPX165950DT multilayer diplexer, 1.6x0.8 mm, 6 terminals "
                     "(1 GND, 2 COM, 3 GND, 4 HB, 5 GND, 6 LB). Land pattern from TDK catalog "
                     "values (pad 0.30x0.60, pitch 0.55): VERIFY against TDK drawing.",
                     "diplexer TDK DPX165950DT", 1.6, 0.8, pads, court_margin=0.35)


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = os.path.join(root, "lib", "steering_wheel.pretty")
    os.makedirs(out, exist_ok=True)
    for fp in (bga(os.path.join(root, "lib", "rw612_balls.json")), mlpd6(), dpx()):
        with open(os.path.join(out, fp[1] + ".kicad_mod"), "w") as fh:
            fh.write(dumps(fp) + "\n")
        print("wrote", fp[1])


if __name__ == "__main__":
    main()
