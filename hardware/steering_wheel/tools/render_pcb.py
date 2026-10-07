"""PNG preview of the board model (approximate, for layout review)."""
import math
import os
import sys

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(__file__))

COL = {"F.Cu": (200, 50, 50), "B.Cu": (50, 90, 220),
       "In1.Cu": (60, 160, 60), "In2.Cu": (190, 150, 30), "In3.Cu": (150, 80, 190),
       "In4.Cu": (40, 160, 170), "In5.Cu": (210, 110, 40), "In6.Cu": (120, 120, 120)}
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"


def render(board, out, scale=10, box=None, layers=("F.Cu", "B.Cu"), labels=True, zones=True):
    xs = [p[0] for p in board.outline]; ys = [p[1] for p in board.outline]
    x0, y0, x1, y1 = box or (min(xs) - 2, min(ys) - 2, max(xs) + 2, max(ys) + 2)
    Wp, Hp = int((x1 - x0) * scale), int((y1 - y0) * scale)
    img = Image.new("RGB", (Wp, Hp), (15, 15, 20))
    d = ImageDraw.Draw(img, "RGBA")
    T = lambda x, y: ((x - x0) * scale, (y - y0) * scale)  # noqa: E731
    fnt = ImageFont.truetype(FONT, max(8, int(scale * 0.6))) if os.path.exists(FONT) else None
    d.polygon([T(*p) for p in board.outline], outline=(220, 220, 0), fill=(25, 35, 30))
    if zones:
        for z in board.zones:
            if z.get("keepout"):
                d.polygon([T(*p) for p in z["pts"]], outline=(255, 0, 255, 160))
    for fp in board.fps.values():
        cb = fp.courtyard()
        d.rectangle([*T(cb[0], cb[1]), *T(cb[2], cb[3])], outline=(120, 120, 120, 160))
    for lay in reversed(layers):
        c = COL[lay]
        for t in board.tracks:
            if t["layer"] == lay:
                d.line([T(t["x1"], t["y1"]), T(t["x2"], t["y2"])], fill=c + (230,),
                       width=max(1, int(t["w"] * scale)))
        for fp in board.fps.values():
            for p in fp.pads:
                cl = p.copper_layers(list(COL))
                if lay not in cl:
                    continue
                bx = fp.pad_box(p)
                if p.shape == "circle":
                    d.ellipse([*T(bx[0], bx[1]), *T(bx[2], bx[3])], fill=c + (200,))
                else:
                    d.rectangle([*T(bx[0], bx[1]), *T(bx[2], bx[3])], fill=c + (200,))
    for v in board.vias:
        x, y = T(v["x"], v["y"]); rr = v["d"] / 2 * scale
        d.ellipse([x - rr, y - rr, x + rr, y + rr], fill=(210, 210, 210))
        rd = v["drill"] / 2 * scale
        d.ellipse([x - rd, y - rd, x + rd, y + rd], fill=(20, 20, 20))
    if labels and fnt:
        for fp in board.fps.values():
            cb = fp.courtyard()
            if (cb[2] - cb[0]) * scale > 12:
                d.text(T((cb[0] + cb[2]) / 2 - 0.4 * len(fp.ref) * 0.3, (cb[1] + cb[3]) / 2 - 0.3),
                       fp.ref, fill=(255, 255, 255), font=fnt)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    img.save(out)
    return out
