"""Compose photo figures for the paper from the team's own media (report
images, presentation videos). Outputs JPEGs in paper/figures/."""
from __future__ import annotations

import os
import sys

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FIG = os.path.join(ROOT, "paper", "figures")
SRC = os.path.join(FIG, "src")
MEDIA = sys.argv[1] if len(sys.argv) > 1 else os.path.join(SRC, "media")


def font(sz):
    for f in ("segoeuib.ttf", "arialbd.ttf", "DejaVuSans-Bold.ttf"):
        try:
            return ImageFont.truetype(f, sz)
        except OSError:
            pass
    return ImageFont.load_default()


def load(path, h, crop=None):
    im = Image.open(path).convert("RGB")
    if crop:
        w0, h0 = im.size
        im = im.crop((int(crop[0] * w0), int(crop[1] * h0), int(crop[2] * w0), int(crop[3] * h0)))
    return im.resize((int(im.width * h / im.height), h), Image.LANCZOS)


def row(panels, h, gap=12, label_size=34, out=None, width=None):
    ims = [load(p, h, c) for p, c in panels]
    W = sum(i.width for i in ims) + gap * (len(ims) - 1)
    canvas = Image.new("RGB", (W, h), "white")
    x = 0
    d = ImageDraw.Draw(canvas)
    f = font(label_size)
    for k, im in enumerate(ims):
        canvas.paste(im, (x, 0))
        lab = f"({chr(97 + k)})"
        tw = d.textlength(lab, font=f)
        d.rounded_rectangle((x + 10, 10, x + 26 + tw, 16 + label_size + 8), radius=6, fill=(255, 255, 255))
        d.text((x + 18, 12), lab, fill=(11, 11, 11), font=f)
        x += im.width + gap
    if width:
        canvas = canvas.resize((width, int(canvas.height * width / canvas.width)), Image.LANCZOS)
    canvas.save(out, quality=90)
    print("saved", out, canvas.size)


if __name__ == "__main__":
    M = MEDIA
    row([(os.path.join(M, "image141.png"), (0.04, 0.06, 0.97, 0.95)),
         (os.path.join(M, "image142.jpeg"), None),
         (os.path.join(SRC, "nav_40.jpg"), (0.22, 0.05, 0.92, 1.0))],
        h=760, out=os.path.join(FIG, "fig_rover.jpg"), width=2400)
    row([(os.path.join(M, "image177.png"), (0.22, 0.0, 0.78, 1.0)),
         (os.path.join(M, "image178.jpeg"), None),
         (os.path.join(SRC, "nav_14.jpg"), None),
         (os.path.join(SRC, "nav_6.jpg"), (0.05, 0.1, 0.9, 0.95))],
        h=700, out=os.path.join(FIG, "fig_real_results.jpg"), width=2400)
    row([(os.path.join(M, "image174.png"), (0.0, 0.0, 1.0, 1.0))],
        h=383, out=os.path.join(FIG, "fig_fea.jpg"), width=1400)
    row([(os.path.join(M, "image184.jpeg"), None)],
        h=700, out=os.path.join(FIG, "fig_labview_gs.jpg"), width=1400)
