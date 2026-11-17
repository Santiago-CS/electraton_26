"""Quick PNG preview of a .kicad_sch (approximate; for layout review only)."""
import math
import os
import re
import sys

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(__file__))
from sexp import find, find_all, parse  # noqa: E402

PAPERS = {"A4": (297, 210), "A3": (420, 297), "A2": (594, 420), "A1": (841, 594)}
FONT_PATHS = ["/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
              "/usr/share/fonts/dejavu/DejaVuSans.ttf"]


def _font(px):
    for p in FONT_PATHS:
        if os.path.exists(p):
            return ImageFont.truetype(p, max(6, int(px)))
    return ImageFont.load_default()


def rot(x, y, r):
    rr = math.radians(r)
    return x * math.cos(rr) - y * math.sin(rr), -(x * math.sin(rr) + y * math.cos(rr))


def render(path, out, scale=6.0):
    doc = parse(open(path).read())
    paper = str(find(doc, "paper")[1])
    W, H = PAPERS.get(paper, (420, 297))
    img = Image.new("RGB", (int(W * scale), int(H * scale)), "white")
    d = ImageDraw.Draw(img)
    S = lambda v: v * scale  # noqa: E731
    fnt = _font(1.27 * scale * 1.1)
    libs = {str(s[1]): s for s in find(doc, "lib_symbols")[1:]}
    for it in doc:
        if not isinstance(it, list):
            continue
        if it[0] == "symbol":
            lib_id = str(find(it, "lib_id")[1])
            at = find(it, "at")
            sx, sy, r = float(at[1]), float(at[2]), float(at[3])
            unit = int(find(it, "unit")[1])
            ls = libs[lib_id]
            name = lib_id.split(":", 1)[1]
            col = (150, 0, 0)
            for sub in find_all(ls, "symbol"):
                m = re.match(re.escape(name) + r"_(\d+)_(\d+)$", sub[1])
                if not m or int(m.group(1)) not in (0, unit) or int(m.group(2)) not in (0, 1):
                    continue
                for g in sub:
                    if not isinstance(g, list):
                        continue
                    def T(x, y):
                        dx, dy = rot(float(x), float(y), r)
                        return S(sx + dx), S(sy + dy)
                    if g[0] == "rectangle":
                        a = find(g, "start"); b = find(g, "end")
                        p1 = T(a[1], a[2]); p2 = T(b[1], b[2])
                        d.rectangle([min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]),
                                     max(p1[1], p2[1])], outline=col, fill=(255, 255, 220) if find(g, "fill") and find(find(g, "fill"), "type")[1] == "background" else None)
                    elif g[0] == "polyline":
                        pts = [T(xy[1], xy[2]) for xy in find(g, "pts")[1:]]
                        if len(pts) > 1:
                            d.line(pts, fill=col, width=1)
                    elif g[0] == "circle":
                        c = find(g, "center"); rad = float(find(g, "radius")[1])
                        cx, cy = T(c[1], c[2])
                        d.ellipse([cx - S(rad), cy - S(rad), cx + S(rad), cy + S(rad)], outline=col)
                    elif g[0] == "arc":
                        pts = [T(find(g, k)[1], find(g, k)[2]) for k in ("start", "mid", "end")]
                        d.line(pts, fill=col)
                    elif g[0] == "pin":
                        a = find(g, "at"); ln = float(find(g, "length")[1])
                        x, y, ang = float(a[1]), float(a[2]), float(a[3])
                        x2 = x + ln * math.cos(math.radians(ang)); y2 = y + ln * math.sin(math.radians(ang))
                        d.line([T(x, y), T(x2, y2)], fill=(120, 0, 0), width=1)
                        pname = str(find(g, "name")[1])
                        if pname not in ("~", "") and ln > 0 and name not in ("R", "C", "L", "C_Polarized", "LED"):
                            tx, ty = T(x2, y2)
                            dirx = math.cos(math.radians(ang + r))
                            txt = re.sub(r"~\{(.*?)\}", r"\1", pname)
                            if abs(dirx) > 0.5:
                                off = 3 if dirx > 0 else -3 - d.textlength(txt, font=fnt)
                                d.text((tx + off, ty - S(0.7)), txt, fill=(0, 100, 100), font=fnt)
            for p in find_all(it, "property"):
                hide = any(isinstance(e, list) and e[0] == "hide" for e in find(p, "effects") or [])
                if hide or not str(p[2]):
                    continue
                a = find(p, "at")
                d.text((S(float(a[1])), S(float(a[2])) - S(0.8)), str(p[2]), fill=(0, 0, 150), font=fnt)
        elif it[0] == "wire":
            a, b = find(it, "pts")[1:3]
            d.line([S(float(a[1])), S(float(a[2])), S(float(b[1])), S(float(b[2]))], fill=(0, 130, 0), width=2)
        elif it[0] in ("label", "global_label"):
            at = find(it, "at"); x, y, ang = S(float(at[1])), S(float(at[2])), float(at[3])
            txt = str(it[1])
            w = d.textlength(txt, font=fnt)
            col = (180, 90, 0) if it[0] == "global_label" else (0, 0, 0)
            if ang == 0:
                d.text((x + 2, y - S(1.2)), txt, fill=col, font=fnt)
            elif ang == 180:
                d.text((x - 2 - w, y - S(1.2)), txt, fill=col, font=fnt)
            else:
                tim = Image.new("RGBA", (int(w) + 4, int(S(1.8))), (255, 255, 255, 0))
                ImageDraw.Draw(tim).text((0, 0), txt, fill=col, font=fnt)
                tim = tim.rotate(90, expand=True)
                if ang == 90:
                    img.paste(tim, (int(x - S(1.6)), int(y - tim.size[1] - 2)), tim)
                else:
                    img.paste(tim, (int(x - S(1.6)), int(y + 2)), tim)
        elif it[0] == "junction":
            at = find(it, "at"); x, y = S(float(at[1])), S(float(at[2]))
            d.ellipse([x - 4, y - 4, x + 4, y + 4], fill=(0, 130, 0))
        elif it[0] == "no_connect":
            at = find(it, "at"); x, y = S(float(at[1])), S(float(at[2]))
            k = S(0.6)
            d.line([x - k, y - k, x + k, y + k], fill=(0, 0, 200), width=2)
            d.line([x - k, y + k, x + k, y - k], fill=(0, 0, 200), width=2)
        elif it[0] == "text":
            at = find(it, "at")
            eff = find(it, "effects"); fs = float(find(find(eff, "font"), "size")[1])
            d.multiline_text((S(float(at[1])), S(float(at[2])) - S(fs)), str(it[1]), fill=(40, 40, 40),
                             font=_font(fs * scale * 1.1))
        elif it[0] == "rectangle":
            a = find(it, "start"); b = find(it, "end")
            d.rectangle([S(float(a[1])), S(float(a[2])), S(float(b[1])), S(float(b[2]))], outline=(170, 170, 170))
        elif it[0] == "sheet":
            a = find(it, "at"); sz = find(it, "size")
            x, y = S(float(a[1])), S(float(a[2]))
            d.rectangle([x, y, x + S(float(sz[1])), y + S(float(sz[2]))], outline=(100, 0, 100), width=2)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    img.save(out)
    return out


if __name__ == "__main__":
    render(sys.argv[1], sys.argv[2])
