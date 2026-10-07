"""Board previews from KiCad's own plotter: a multipage PDF with every copper/silk layer and
PNG images of the top and bottom side (and a plain 3D render).

    python tools/previews.py steering_wheel.kicad_pcb docs/     [--kicad-cli "kicad-cli"]

Needs kicad-cli (KiCad 9/10) and PyMuPDF (pip install pymupdf) for the PNG conversion.
"""
import os
import shlex
import subprocess
import sys

import pymupdf as fitz


def run(cli, args):
    cmd = shlex.split(cli) + args
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit(f"{' '.join(cmd)}\n{r.stdout}\n{r.stderr}")


def pdf_to_png(pdf, png, px_per_mm=12, margin_mm=2.0):
    """render page 1 and crop it to the drawn content"""
    from PIL import Image, ImageChops
    doc = fitz.open(pdf)
    zoom = px_per_mm * 25.4 / 72.0
    pix = doc[0].get_pixmap(matrix=fitz.Matrix(zoom, zoom), alpha=False)
    im = Image.frombytes("RGB", (pix.width, pix.height), pix.samples)
    bg = Image.new("RGB", im.size, (255, 255, 255))
    bbox = ImageChops.difference(im, bg).getbbox()
    if bbox:
        m = int(margin_mm * px_per_mm)
        bbox = (max(bbox[0] - m, 0), max(bbox[1] - m, 0), min(bbox[2] + m, im.width), min(bbox[3] + m, im.height))
        im = im.crop(bbox)
    im.save(png)


def main(board, outdir, cli="kicad-cli", host_prefix=None):
    """host_prefix: (inside, outside) path prefixes when kicad-cli runs in a chroot"""
    def p_in(p):
        p = os.path.abspath(p)
        return p.replace(host_prefix[1], host_prefix[0], 1) if host_prefix else p
    os.makedirs(outdir, exist_ok=True)
    tmp = os.path.join(os.path.dirname(os.path.abspath(board)), "build")
    os.makedirs(tmp, exist_ok=True)
    # KiCad 10 wants a file name here; KiCad 9 took a directory and named the file after
    # the board. Pass a file and fall back to the directory form for KiCad 9.
    mp = os.path.join(tmp, "multipage")
    os.makedirs(mp, exist_ok=True)
    layers = ("F.Cu,F.Silkscreen,In1.Cu,In2.Cu,In3.Cu,In4.Cu,In5.Cu,In6.Cu,"
              "B.Cu,B.Silkscreen")
    target = os.path.join(mp, "pcb_layers.pdf")
    args = ["pcb", "export", "pdf", "--mode-multipage", "--include-border-title",
            "-l", layers, "--cl", "Edge.Cuts"]
    try:
        run(cli, args + ["-o", p_in(target), p_in(board)])
    except Exception:
        run(cli, args + ["-o", p_in(mp) + "/", p_in(board)])
        target = os.path.join(mp, os.path.splitext(os.path.basename(board))[0] + ".pdf")
    os.replace(target, os.path.join(outdir, "pcb_layers.pdf"))
    views = {
        "pcb_top": (["F.Cu", "F.Silkscreen", "Edge.Cuts"], False),
        "pcb_bottom": (["B.Cu", "B.Silkscreen", "Edge.Cuts"], True),
        "pcb_inner": (["In2.Cu", "In5.Cu", "Edge.Cuts"], False),
    }
    for name, (layers, mirror) in views.items():
        pdf = os.path.join(tmp, name + ".pdf")
        args = ["pcb", "export", "pdf", "--mode-single", "-l", ",".join(layers), "-o", p_in(pdf), p_in(board)]
        if mirror:
            args.insert(3, "--mirror")
        run(cli, args)
        pdf_to_png(pdf, os.path.join(outdir, name + ".png"))
    run(cli, ["pcb", "render", "-o", p_in(os.path.join(outdir, "pcb_3d.png")), "--side", "top",
              "-w", "1920", "-h", "1100", "--quality", "high", "--zoom", "1.35", p_in(board)])


if __name__ == "__main__":
    a = sys.argv[1:]
    cli = "kicad-cli"
    prefix = None
    if "--kicad-cli" in a:
        i = a.index("--kicad-cli")
        cli = a[i + 1]
        del a[i:i + 2]
    if "--chroot-map" in a:          # --chroot-map /work:/home/claude/sw
        i = a.index("--chroot-map")
        prefix = tuple(a[i + 1].split(":"))
        del a[i:i + 2]
    main(a[0], a[1], cli, prefix)
