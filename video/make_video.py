"""MISSION X - final project demo video (1920x1080, 30 fps, H.264 + AAC).

Frames are composed in Python (PIL/numpy) and streamed straight into ffmpeg,
so no large intermediate files are written. Sources: the team's own footage
and photos (report, presentation, show folder), the paper figures and a
re-run of the SLAM digital twin for an animated map. The soundtrack is
synthesised procedurally (no third-party music).

Usage:  python make_video.py [--ar] [--preview]
        --ar       Arabic edition (Arabic captions, right-to-left layout)
        --preview  960x540, faster
Output: video/MissionX_Rover_Demo.mp4 (or MissionX_Rover_Demo_AR.mp4)
"""
from __future__ import annotations

import math
import os
import subprocess
import sys
import wave

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PROJ = os.path.dirname(ROOT)
DOC = os.path.join(PROJ, "Space Exploration Rover", "Documentation")
FIG = os.path.join(ROOT, "paper", "figures")
OUT_FIG = os.path.join(ROOT, "analysis", "out")
ASSETS = os.path.join(HERE, "assets")
BUILD = os.path.join(HERE, "build")
os.makedirs(BUILD, exist_ok=True)
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "rover_core"))
sys.path.insert(0, os.path.join(ROOT, "analysis"))

PREVIEW = "--preview" in sys.argv
# --ar renders the Arabic edition: Arabic captions, right-to-left layout
RTL = "--ar" in sys.argv
W, H = (960, 540) if PREVIEW else (1920, 1080)


def tr(en, ar):
    """Pick the caption for the current edition."""
    return ar if RTL else en


def is_ar(s):
    return any("؀" <= ch <= "ۿ" for ch in s)
S = W / 1920.0
FPS = 30
FONTS = r"C:\Windows\Fonts"
MARS = (235, 104, 52)
INK = (12, 12, 14)
WHITE = (255, 255, 255)
SOFT = (225, 223, 216)


def font(name, size):
    return ImageFont.truetype(os.path.join(FONTS, name), max(8, int(size * S)),
                              layout_engine=ImageFont.Layout.RAQM)


F_TITLE = lambda s: font("bahnschrift.ttf", s)       # Latin display face (no Arabic glyphs)
F_BOLD = lambda s: font("segoeuib.ttf", s)
F_SEMI = lambda s: font("seguisb.ttf", s)
F_REG = lambda s: font("segoeui.ttf", s)
F_LIGHT = (lambda s: font("segoeui.ttf", s)) if RTL else (lambda s: font("segoeuil.ttf", s))
F_HEAD = F_BOLD if RTL else F_TITLE                   # headings that may carry Arabic


# ======================================================================
# image helpers
# ======================================================================
def load(path):
    return Image.open(path).convert("RGB")


def cover(img, w=W, h=H, cx=0.5, cy=0.5):
    r = max(w / img.width, h / img.height)
    im = img.resize((max(w, int(img.width * r + 0.5)), max(h, int(img.height * r + 0.5))), Image.LANCZOS)
    x = int((im.width - w) * cx)
    y = int((im.height - h) * cy)
    return im.crop((x, y, x + w, y + h))


def contain_on_blur(img, w=W, h=H, scale=1.0, darken=0.45):
    bg = cover(img, w, h).filter(ImageFilter.GaussianBlur(int(40 * S)))
    bg = Image.blend(bg, Image.new("RGB", bg.size, (0, 0, 0)), darken)
    r = min(w / img.width, h / img.height) * scale
    fg = img.resize((int(img.width * r), int(img.height * r)), Image.LANCZOS)
    bg.paste(fg, ((w - fg.width) // 2, (h - fg.height) // 2))
    return bg


class KenBurns:
    """Slow zoom/pan over a still (source pre-scaled for the max zoom)."""

    def __init__(self, img, z0=1.0, z1=1.12, c0=(0.5, 0.5), c1=(0.5, 0.5), mode="cover"):
        base = contain_on_blur(img) if mode == "contain" else cover(img)
        zmax = max(z0, z1)
        self.big = base.resize((int(W * zmax), int(H * zmax)), Image.LANCZOS)
        self.z0, self.z1, self.c0, self.c1, self.zmax = z0, z1, c0, c1, zmax

    def frame(self, u):
        u = 0.5 - 0.5 * math.cos(math.pi * min(max(u, 0.0), 1.0))  # ease
        z = self.z0 + (self.z1 - self.z0) * u
        cx = self.c0[0] + (self.c1[0] - self.c0[0]) * u
        cy = self.c0[1] + (self.c1[1] - self.c0[1]) * u
        # visible window on the pre-scaled source: full image at z=1, W x H at z=zmax
        cw = min(self.big.width, W * self.zmax / z)
        ch = min(self.big.height, H * self.zmax / z)
        x0 = max(0.0, (self.big.width - cw) * cx)
        y0 = max(0.0, (self.big.height - ch) * cy)
        return self.big.resize((W, H), Image.BILINEAR, box=(x0, y0, x0 + cw, y0 + ch))


class VideoClip:
    """Sequential frames from a video segment (scaled/cropped by ffmpeg)."""

    def __init__(self, path, start, dur, mode="cover", crop=None, speed=1.0):
        vf = []
        if crop:  # crop=(x, y, w, h) as fractions of the input
            vf.append(f"crop=iw*{crop[2]}:ih*{crop[3]}:iw*{crop[0]}:ih*{crop[1]}")
        if speed != 1.0:
            vf.append(f"setpts=PTS/{speed}")
        if mode == "cover":
            vf.append(f"scale={W}:{H}:force_original_aspect_ratio=increase,crop={W}:{H}")
            fc = ",".join(vf)
            args = ["-vf", fc + f",fps={FPS}"]
        else:  # contain on blurred background
            pre = ",".join(vf) + "," if vf else ""
            fc = (f"[0:v]{pre}split[a][b];[a]scale={W}:{H}:force_original_aspect_ratio=increase,"
                  f"crop={W}:{H},boxblur=30:4,eq=brightness=-0.22[bg];[b]scale=-2:{H}[fg];"
                  f"[bg][fg]overlay=(W-w)/2:0,fps={FPS}[v]")
            args = ["-filter_complex", fc, "-map", "[v]"]
        self.cmd = ["ffmpeg", "-v", "error", "-ss", str(start), "-t", str(dur / speed if speed != 1.0 else dur),
                    "-i", path] + args + ["-an", "-f", "rawvideo", "-pix_fmt", "rgb24", "-"]
        self.p = None          # decoder is started lazily on the first frame
        self.done = False
        self.last = Image.new("RGB", (W, H), (0, 0, 0))

    def frame(self, u=None):
        if self.done:
            return self.last
        if self.p is None:
            self.p = subprocess.Popen(self.cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                      bufsize=W * H * 3 * 4)
        buf = self.p.stdout.read(W * H * 3)
        if len(buf) == W * H * 3:
            self.last = Image.frombytes("RGB", (W, H), buf)
        else:
            self.close()
        return self.last

    def close(self):
        self.done = True
        if self.p is not None:
            try:
                self.p.stdout.close()
                self.p.kill()
            except Exception:
                pass


# ======================================================================
# overlays (RGBA, full frame)
# ======================================================================
def rgba():
    return Image.new("RGBA", (W, H), (0, 0, 0, 0))


def _kw(txt):
    return {"direction": "rtl", "language": "ar"} if is_ar(txt) else {}


def text_w(draw, txt, f):
    return draw.textlength(txt, font=f, **_kw(txt))


def put(d, x, y, txt, f, fill, mirror=True):
    """Draw text whose LEFT edge is at x in the English layout. In the Arabic
    edition the layout is mirrored: the text's RIGHT edge is at W - x."""
    if RTL and mirror:
        d.text((W - x - text_w(d, txt, f), y), txt, font=f, fill=fill, **_kw(txt))
    else:
        d.text((x, y), txt, font=f, fill=fill, **_kw(txt))


def put_right(d, xr, y, txt, f, fill):
    d.text((xr - text_w(d, txt, f), y), txt, font=f, fill=fill, **_kw(txt))


def mbox(box):
    """Mirror a rectangle horizontally in the Arabic edition."""
    x0, y0, x1, y1 = box
    return (W - x1, y0, W - x0, y1) if RTL else box


def spaced(txt):
    # letter-spacing reads well in Latin capitals but breaks Arabic joining
    return txt if is_ar(txt) else " ".join(txt.upper())


def chip(label, number=None):
    """Section label, top-left (top-right in Arabic): '02  STRUCTURE SUBSYSTEM'."""
    im = rgba()
    d = ImageDraw.Draw(im)
    x, y = int(96 * S), int(80 * S)
    f = F_SEMI(26 if not is_ar(label) else 30)
    if number:
        put(d, x, y, number, F_BOLD(26), MARS + (255,))
        x += int(64 * S)
    d.rectangle(mbox((x, y + int(44 * S), x + int(56 * S), y + int(48 * S))), fill=MARS + (255,))
    put(d, x, y - (int(4 * S) if is_ar(label) else 0), spaced(label), f, WHITE + (235,))
    return im


def lower_third(title, sub=None, arabic=None, y_from_bottom=110):
    if RTL:
        arabic = None  # the English edition's extra Arabic line is the main text here
    im = rgba()
    d = ImageDraw.Draw(im)
    ft, fs = F_BOLD(54 if not is_ar(title) else 50), F_REG(32)
    tw = text_w(d, title, ft)
    sw = text_w(d, sub, fs) if sub else 0
    aw = text_w(d, arabic, F_SEMI(34)) if arabic else 0
    pad = int(36 * S)
    bw = int(max(tw, sw, aw) + 2 * pad + 10 * S)
    bh = int(pad * 1.6 + 70 * S + (50 * S if sub else 0) + (54 * S if arabic else 0))
    x0 = int(96 * S)
    y0 = H - int(y_from_bottom * S) - bh
    d.rounded_rectangle(mbox((x0, y0, x0 + bw, y0 + bh)), radius=int(14 * S), fill=(10, 10, 12, 190))
    d.rectangle(mbox((x0, y0, x0 + int(8 * S), y0 + bh)), fill=MARS + (255,))
    ty = y0 + int(pad * 0.8)
    put(d, x0 + pad, ty, title, ft, WHITE + (255,))
    ty += int(70 * S)
    if sub:
        put(d, x0 + pad, ty, sub, fs, SOFT + (255,))
        ty += int(50 * S)
    if arabic:
        put(d, x0 + pad, ty, arabic, F_SEMI(34), (255, 200, 170, 255))
    return im


def bullets(items, x=96, y=300, size=40, gap=74):
    """List of (text, appear_u) -> function(u) returning overlay."""
    def make(u):
        im = rgba()
        d = ImageDraw.Draw(im)
        for i, (txt, ua) in enumerate(items):
            a = min(1.0, max(0.0, (u - ua) / 0.06))
            if a <= 0:
                continue
            dx = int((1 - a) * 30 * S)
            yy = int((y + i * gap) * S)
            d.rounded_rectangle(mbox((int(x * S) - dx, yy, int(x * S) - dx + int(14 * S),
                                      yy + int(14 * S) + int(size * 0.35 * S))),
                                radius=int(4 * S), fill=MARS + (int(255 * a),))
            put(d, int((x + 34) * S) - dx, yy - int(size * 0.28 * S), txt, F_SEMI(size), WHITE + (int(255 * a),))
        return im
    return make


def scrim_left(strength=215, reach=0.62):
    """Dark gradient behind the text side (left; right in the Arabic edition)."""
    x = np.linspace(0, 1, W)
    a = np.clip(1 - x / reach, 0, 1) ** 1.3 * strength
    if RTL:
        a = a[::-1]
    im = np.zeros((H, W, 4), np.uint8)
    im[..., 3] = np.repeat(a[None, :], H, axis=0).astype(np.uint8)
    return Image.fromarray(im, "RGBA")


def vignette():
    yy, xx = np.mgrid[0:H, 0:W]
    r = np.sqrt(((xx - W / 2) / (W / 2)) ** 2 + ((yy - H / 2) / (H / 2)) ** 2)
    a = np.clip((r - 0.65) / 0.75, 0, 1) ** 1.5 * 170
    im = np.zeros((H, W, 4), np.uint8)
    im[..., 3] = a.astype(np.uint8)
    return Image.fromarray(im, "RGBA")


def bottom_gradient(strength=200):
    a = np.linspace(0, 1, H) ** 2.2 * strength
    im = np.zeros((H, W, 4), np.uint8)
    im[..., 3] = np.repeat(a[:, None], W, axis=1).astype(np.uint8)
    return Image.fromarray(im, "RGBA")


VIGNETTE = None
GRADIENT = None


# ======================================================================
# scene framework
# ======================================================================
class Scene:
    def __init__(self, dur, layers, overlays=(), fade_in=0.45, fade_out=0.45, vign=True, grad=True):
        self.dur, self.layers, self.overlays = dur, layers, overlays
        self.fade_in, self.fade_out, self.vign, self.grad = fade_in, fade_out, vign, grad

    def frames(self):
        n = int(round(self.dur * FPS))
        static = {}
        for k in range(n):
            t = k / FPS
            u = k / max(1, n - 1)
            base = self.layers(t, u).convert("RGBA")
            if self.grad:
                base.alpha_composite(GRADIENT)
            if self.vign:
                base.alpha_composite(VIGNETTE)
            for ov in self.overlays:
                img, a = ov(t, u)
                if img is None or a <= 0:
                    continue
                if a >= 0.999:
                    base.alpha_composite(img)
                else:
                    lay = img.copy()
                    lay.putalpha(lay.getchannel("A").point(lambda v, a=a: int(v * a)))
                    base.alpha_composite(lay)
            f = 1.0
            if t < self.fade_in:
                f = t / self.fade_in
            if t > self.dur - self.fade_out:
                f = min(f, max(0.0, (self.dur - t) / self.fade_out))
            rgb = base.convert("RGB")
            if f < 0.999:
                rgb = Image.eval(rgb, lambda v, f=f: int(v * f))
            yield rgb


def timed(img_or_fn, t0, t1, fade=0.4):
    """Overlay visible between t0..t1 with fades; img_or_fn(u_local) allowed."""
    def ov(t, u):
        if t < t0 or t > t1:
            return None, 0
        a = min(1.0, (t - t0) / fade, (t1 - t) / fade)
        img = img_or_fn((t - t0) / max(1e-6, t1 - t0)) if callable(img_or_fn) else img_or_fn
        return img, a
    return ov


def seq(*parts):
    """Concatenate (duration, source) layer parts inside one scene."""
    def layer(t, u):
        acc = 0.0
        for d, src in parts:
            if t < acc + d:
                return src.frame((t - acc) / d)
            acc += d
        return parts[-1][1].frame(1.0)
    return layer


# ======================================================================
# special scenes
# ======================================================================
def title_card(t, u, bg):
    im = bg.frame(u).convert("RGBA")
    shade = Image.new("RGBA", (W, H), (0, 0, 0, 120))
    im.alpha_composite(shade)
    return im


def title_overlay(u):
    im = rgba()
    d = ImageDraw.Draw(im)
    a = min(1.0, u / 0.25)
    t1 = "MISSION X"
    f1 = F_TITLE(190)
    sp1 = "  ".join(t1)
    w1 = text_w(d, sp1, f1)
    d.text(((W - w1) / 2, H * 0.30), sp1, font=f1, fill=WHITE + (int(255 * a),))
    d.rectangle((W / 2 - 70 * S, H * 0.30 + 225 * S, W / 2 + 70 * S, H * 0.30 + 231 * S), fill=MARS + (int(255 * a),))
    sub = tr("SPACE EXPLORATION ROVER", "مركبة استكشاف الفضاء")
    f2 = F_SEMI(46 if not RTL else 54)
    sp2 = spaced(sub)
    d.text(((W - text_w(d, sp2, f2)) / 2, H * 0.30 + 258 * S), sp2, font=f2, fill=SOFT + (int(255 * a),), **_kw(sp2))
    t3 = tr("Design and implementation of a space rover with autonomous navigation",
            "تصميم وتنفيذ مركبة لاستكشاف الفضاء باستخدام نظام ملاحة ذاتي")
    f3 = F_LIGHT(34 if not RTL else 38)
    d.text(((W - text_w(d, t3, f3)) / 2, H * 0.30 + 345 * S), t3, font=f3, fill=SOFT + (int(230 * a),), **_kw(t3))
    return im


def institution_overlay(u):
    im = rgba()
    d = ImageDraw.Draw(im)
    if RTL:
        lines = [("كلية علوم الملاحة وتكنولوجيا الفضاء", F_BOLD(60), WHITE),
                 ("جامعة بني سويف", F_SEMI(46), SOFT),
                 ("بالتعاون مع وكالة الفضاء المصرية", F_REG(40), SOFT)]
        last = ("Faculty of Navigation Science and Space Technology · Beni-Suef University", F_REG(30))
    else:
        lines = [("Faculty of Navigation Science and Space Technology", F_SEMI(52), WHITE),
                 ("Beni-Suef University", F_REG(40), SOFT),
                 ("in cooperation with the Egyptian Space Agency (EgSA)", F_LIGHT(36), SOFT)]
        last = ("كلية علوم الملاحة وتكنولوجيا الفضاء – جامعة بني سويف", F_SEMI(44))
    y = H * 0.33
    for txt, f, c in lines:
        d.text(((W - text_w(d, txt, f)) / 2, y), txt, font=f, fill=c + (255,), **_kw(txt))
        y += f.size * 1.5
    txt, f = last
    d.text(((W - text_w(d, txt, f)) / 2, y + 20 * S), txt, font=f, fill=(255, 200, 170, 255), **_kw(txt))
    return im


def stat_tiles(u):
    tiles = [("36.5 kg", tr("rover mass · 1.02 × 0.73 × 0.79 m", "كتلة المركبة · الأبعاد 1.02 × 0.73 × 0.79 م")),
             ("26.9°", tr("slope at rated torque (Earth)", "أقصى منحدر بالعزم المقنن (الأرض)")),
             ("0.9 km", tr("HC-12 ground-link range (estimated)", "مدى وصلة HC-12 الأرضية (تقديري)")),
             ("6.7 h", tr("endurance, typical sortie", "زمن التشغيل في مهمة نموذجية")),
             ("0.11 m", tr("SLAM trajectory error (sim)", "خطأ مسار SLAM (محاكاة)")),
             ("0 / 520 000", tr("wrong commands, new protocol", "أوامر خاطئة في البروتوكول الجديد"))]
    im = rgba()
    d = ImageDraw.Draw(im)
    cols, rows = 3, 2
    tw, th = 500 * S, 230 * S
    gx, gy = 40 * S, 40 * S
    x0 = (W - (cols * tw + (cols - 1) * gx)) / 2
    y0 = H * 0.24
    for i, (big, small) in enumerate(tiles):
        a = min(1.0, max(0.0, (u - 0.05 - i * 0.06) / 0.08))
        if a <= 0:
            continue
        r, c = divmod(i, cols)
        x = x0 + c * (tw + gx)
        y = y0 + r * (th + gy) + (1 - a) * 20 * S
        d.rounded_rectangle(mbox((x, y, x + tw, y + th)), radius=int(18 * S), fill=(14, 14, 18, int(205 * a)))
        d.rectangle(mbox((x, y, x + 8 * S, y + th)), fill=MARS + (int(255 * a),))
        fb = F_TITLE(84)
        put(d, x + 36 * S, y + 34 * S, big, fb, WHITE + (int(255 * a),))
        put(d, x + 38 * S, y + 150 * S, small, F_REG(30), SOFT + (int(255 * a),))
    head = spaced(tr("MISSION X IN NUMBERS", "ميشن إكس بالأرقام"))
    fh = F_SEMI(34 if not RTL else 40)
    d.text(((W - text_w(d, head, fh)) / 2, H * 0.12), head, font=fh, fill=WHITE + (235,), **_kw(head))
    return im


def credits_overlay_ar(u):
    """Arabic credits: names as the team wrote them (show/Rover.txt), right-aligned."""
    im = rgba()
    d = ImageDraw.Draw(im)
    team = ["عبدالرحمن باسم علي", "احمد ممدوح ابراهيم", "أحمد محمد احمد غلاب", "آيه عيد فكري",
            "براء محمد جبر", "إيمان عزت السيد", "محمد حاتم محمد", "عمر عبدالحكيم عبدالاه",
            "عمر خالد عمار", "سليم الدريني الدريني"]
    a = min(1.0, u / 0.12)
    xr = W - 96 * S
    y = H * 0.07
    put_right(d, xr, y, "فريق ميشن إكس", F_BOLD(36), MARS + (int(255 * a),))
    y += 66 * S
    for i, n in enumerate(team):
        put_right(d, xr, y + i * 45 * S, n, F_REG(33), WHITE + (int(255 * a),))
    y += 10 * 45 * S + 26 * S
    put_right(d, xr, y, "المشرفون", F_BOLD(34), MARS + (int(255 * a),))
    y += 58 * S
    put_right(d, xr, y, "أ.م.د/ خالد يوسف", F_SEMI(33), WHITE + (int(255 * a),))
    put_right(d, xr, y + 42 * S, "وكيل كلية علوم الملاحة وتكنولوجيا الفضاء – جامعة بني سويف", F_REG(25),
              SOFT + (int(255 * a),))
    put_right(d, xr, y + 86 * S, "م/ يسري سليمان", F_SEMI(33), WHITE + (int(255 * a),))
    put_right(d, xr, y + 128 * S, "مدير إدارة تطوير المحطات الأرضية – وكالة الفضاء المصرية", F_REG(25),
              SOFT + (int(255 * a),))
    y += 190 * S
    b = min(1.0, max(0.0, (u - 0.55) / 0.15))
    put_right(d, xr, y, "شكراً لكم", F_BOLD(72), WHITE + (int(255 * b),))
    return im


def credits_overlay(u):
    if RTL:
        return credits_overlay_ar(u)
    im = rgba()
    d = ImageDraw.Draw(im)
    team = ["Abdelrahman Bassem Ali", "Ahmed Mamdouh Ibrahim", "Ahmed Mohammed Ghallab", "Aya Eid Fekry",
            "Baraa Mohamed Ahmed Gabr", "Eman Ezzat El Sayed", "Mohammed Hatem Mohammed",
            "Omar Abdelhakim Abdellah", "Omar Khalid Ammar", "Selim Eldreeny Eldreeny"]
    a = min(1.0, u / 0.12)
    x = W * 0.58
    y = H * 0.08
    d.text((x, y), "T H E   T E A M   ·   M I S S I O N   X", font=F_SEMI(28), fill=MARS + (int(255 * a),))
    y += 54 * S
    for i, n in enumerate(team):
        d.text((x, y + i * 43 * S), n, font=F_REG(30), fill=WHITE + (int(255 * a),))
    y += 10 * 43 * S + 30 * S
    d.text((x, y), "S U P E R V I S O R S", font=F_SEMI(28), fill=MARS + (int(255 * a),))
    y += 52 * S
    d.text((x, y), "Prof. Dr. Khaled Youssef", font=F_REG(30), fill=WHITE + (int(255 * a),))
    d.text((x, y + 38 * S), "Beni-Suef University", font=F_LIGHT(25), fill=SOFT + (int(255 * a),))
    d.text((x, y + 82 * S), "Eng. Youssri Soliman", font=F_REG(30), fill=WHITE + (int(255 * a),))
    d.text((x, y + 120 * S), "Egyptian Space Agency (EgSA)", font=F_LIGHT(25), fill=SOFT + (int(255 * a),))
    y += 185 * S
    b = min(1.0, max(0.0, (u - 0.55) / 0.15))
    fty = F_TITLE(76)
    d.text((x, y), "Thank you", font=fty, fill=WHITE + (int(255 * b),))
    tw = text_w(d, "Thank you", fty)
    d.text((x + tw + 40 * S, y + 4 * S), "شكراً", font=F_SEMI(66), fill=(255, 200, 170, int(255 * b)),
           direction="rtl", language="ar")
    return im


class SlamAnim:
    """Animated digital-twin mapping run (re-computed with snapshots)."""

    def __init__(self, seed=7):
        import sim_slam as SS
        from rover_core.sim import World, RoverSim
        from rover_core.params import DEFAULT
        from rover_core.kinematics import wrap_angle
        from sim_lib import HectorMatcher, GridMap
        world = World.default_lab()
        wps = [(1.5, 1.5), (4.0, 1.7), (6.2, 1.9), (7.6, 2.45), (8.2, 3.4), (7.4, 4.5), (5.3, 5.0),
               (7.4, 4.5), (8.2, 3.2), (7.6, 2.45), (6.2, 1.9), (3.5, 1.8), (1.6, 2.4), (1.55, 4.6),
               (1.6, 2.6), (1.8, 1.5)]
        segs = SS.plan_route(world, wps, clearance=0.62)
        sim = RoverSim(world, seed=seed, chi_true=DEFAULT.skid_chi * 1.10, start=(1.5, 1.5, 0.0))
        hm = HectorMatcher(world.size, res=0.05, levels=3, rmax=DEFAULT.lidar_range[1])
        odo = np.array(sim.true_pose)
        slam = np.array(sim.true_pose)
        last = sim.wheel_ticks()
        odo_last_scan = odo.copy()
        self.snaps = []
        seg, idx, k = 0, 0, 0
        truth, est = [], []
        while sim.t < 400:
            cur = segs[seg]
            if math.hypot(cur[-1, 0] - sim.x, cur[-1, 1] - sim.y) < 0.25:
                seg += 1
                idx = 0
                if seg >= len(segs):
                    break
                cur = segs[seg]
            v, w, idx = SS.pure_pursuit(sim.true_pose, cur, idx)
            sim.set_cmd(v, w)
            sim.step(0.02)
            ticks = sim.wheel_ticks()
            ds, dth = SS.odo_step(last, ticks, DEFAULT.b_eff)
            last = ticks
            odo = SS.compose(odo, ds, dth)
            k += 1
            if k % 10 == 0:
                r, a0, inc = sim.scan()
                ang = a0 + np.arange(r.size) * inc
                c, s = math.cos(-odo_last_scan[2]), math.sin(-odo_last_scan[2])
                dx, dy = odo[0] - odo_last_scan[0], odo[1] - odo_last_scan[1]
                lx, ly = c * dx - s * dy, s * dx + c * dy
                dt_ = wrap_angle(odo[2] - odo_last_scan[2])
                cs, sn = math.cos(slam[2]), math.sin(slam[2])
                guess = np.array([slam[0] + cs * lx - sn * ly, slam[1] + sn * lx + cs * ly, wrap_angle(slam[2] + dt_)])
                slam = hm.process(guess, r, ang)
                odo_last_scan = odo.copy()
                truth.append(sim.true_pose)
                est.append(slam.copy())
                if len(truth) % 3 == 0:
                    P = hm.maps[0].prob()
                    ok = np.isfinite(r)
                    lxw, lyw, lth = sim.lidar_pose()
                    pts = np.stack([lxw + r[ok] * np.cos(lth + ang[ok]), lyw + r[ok] * np.sin(lth + ang[ok])], 1)
                    self.snaps.append((P.copy(), np.array(truth), np.array(est), pts, sim.true_pose, sim.t))
        self.size = world.size

    def render(self, u, box_w=1100, box_h=770):
        i = min(len(self.snaps) - 1, int(u * (len(self.snaps) - 1)))
        P, T, E, pts, pose, t = self.snaps[i]
        img = np.full(P.shape, 205, np.uint8)
        img[P < 0.35] = 250
        img[P > 0.65] = 25
        im = Image.fromarray(img[::-1, :]).convert("RGB")
        bw, bh = int(box_w * S), int(box_h * S)
        im = im.resize((bw, bh), Image.NEAREST)
        d = ImageDraw.Draw(im)
        sx, sy = bw / self.size[0], bh / self.size[1]
        tp = lambda x, y: (x * sx, bh - y * sy)
        if len(T) > 1:
            d.line([tp(x, y) for x, y, _ in T], fill=(40, 40, 40), width=max(1, int(2 * S)))
            d.line([tp(x, y) for x, y, _ in E], fill=(42, 120, 214), width=max(2, int(5 * S)))
        for x, y in pts[::2]:
            px, py = tp(x, y)
            d.ellipse((px - 2 * S, py - 2 * S, px + 2 * S, py + 2 * S), fill=(235, 104, 52))
        x, y, th = pose
        L, Wd = 1.02, 0.73
        c, s = math.cos(th), math.sin(th)
        poly = [(x + c * a - s * b, y + s * a + c * b) for a, b in ((L / 2, Wd / 2), (L / 2, -Wd / 2), (-L / 2, -Wd / 2), (-L / 2, Wd / 2))]
        d.polygon([tp(*p) for p in poly], outline=(235, 104, 52), fill=(235, 104, 52, 80), width=max(2, int(4 * S)))
        err = float(np.hypot(E[-1, 0] - T[-1, 0], E[-1, 1] - T[-1, 1])) if len(T) else 0.0
        return im, t, err


# ======================================================================
# soundtrack
# ======================================================================
def synth_music(dur, path, sr=44100, seed=3):
    rng = np.random.default_rng(seed)
    n = int(dur * sr)
    t = np.arange(n) / sr
    out = np.zeros(n)
    midi = lambda m: 440.0 * 2 ** ((m - 69) / 12)
    prog = [[45, 52, 57, 60, 64], [41, 48, 53, 57, 60], [48, 55, 60, 64, 67], [43, 50, 55, 59, 62]]  # Am F C G
    bar = 8.0
    for i in range(int(math.ceil(dur / bar))):
        chord = prog[i % 4]
        t0 = i * bar
        s0, s1 = int(t0 * sr), min(n, int((t0 + bar + 2.5) * sr))
        tt = np.arange(s1 - s0) / sr
        env = np.minimum(1.0, tt / 2.2) * np.exp(-np.maximum(0, tt - bar) / 0.9)
        for m in chord:
            f = midi(m)
            for det in (-0.12, 0.0, 0.12):
                ph = rng.uniform(0, 2 * np.pi)
                w = 2 * np.pi * f * (1 + det / 100) * tt + ph
                out[s0:s1] += 0.055 * env * (np.sin(w) + 0.25 * np.sin(2 * w) + 0.1 * np.sin(3 * w))
        root = midi(chord[0] - 12)
        out[s0:s1] += 0.10 * env * np.sin(2 * np.pi * root * tt)
    # soft arpeggio (sine plucks, 100 bpm eighths) after the intro
    step = 60 / 100 / 2
    for j in range(int(dur / step)):
        tj = j * step
        if tj < 8.0 or tj > dur - 6:
            continue
        chord = prog[int(tj // bar) % 4]
        m = chord[[0, 2, 3, 4, 3, 2, 1, 2][j % 8]] + 12
        s0 = int(tj * sr)
        s1 = min(n, s0 + int(1.2 * sr))
        tt = np.arange(s1 - s0) / sr
        out[s0:s1] += 0.045 * np.exp(-tt / 0.35) * np.sin(2 * np.pi * midi(m) * tt) * (0.6 + 0.4 * (j % 2 == 0))
    # reverb: exponentially decaying noise IR
    from scipy.signal import fftconvolve
    ir_t = np.arange(int(2.8 * sr)) / sr
    ir = rng.normal(0, 1, ir_t.size) * np.exp(-ir_t / 0.75)
    ir /= np.sqrt(np.sum(ir ** 2))
    wet = fftconvolve(out, ir)[:n]
    mix = 0.75 * out + 0.45 * wet
    # one-pole low-pass for warmth
    a = math.exp(-2 * math.pi * 5000 / sr)
    from scipy.signal import lfilter
    mix = lfilter([1 - a], [1, -a], mix)
    fade = np.ones(n)
    fi, fo = int(2.5 * sr), int(5.0 * sr)
    fade[:fi] = np.linspace(0, 1, fi)
    fade[-fo:] = np.linspace(1, 0, fo)
    mix *= fade
    mix /= np.max(np.abs(mix)) + 1e-9
    mix *= 0.80
    stereo = np.stack([mix, np.roll(mix, int(0.012 * sr))], axis=1)
    pcm = (stereo * 32767).astype(np.int16)
    with wave.open(path, "wb") as wf:
        wf.setnchannels(2)
        wf.setsampwidth(2)
        wf.setframerate(sr)
        wf.writeframes(pcm.tobytes())


# ======================================================================
# storyboard
# ======================================================================
def build_scenes():
    pv = lambda *p: os.path.join(DOC, *p)
    media = os.path.join(FIG, "src", "media")
    pptx = os.path.join(ASSETS, "pptx")
    scenes = []

    # 1 cold open: real rover driving + title
    v = VideoClip(os.path.join(pptx, "media3.mp4"), 6.0, 9.0)
    shade1 = Image.new("RGBA", (W, H), (0, 0, 0, 135))
    scenes.append(Scene(9.0, lambda t, u: Image.alpha_composite(v.frame(u).convert("RGBA"), shade1),
                        [timed(lambda u: title_overlay(u), 1.0, 9.0, 0.8)], fade_in=1.0))

    # 2 institution
    kb = KenBurns(load(os.path.join(pptx, "s2.jpg")), 1.04, 1.14, (0.5, 0.45), (0.5, 0.40))
    dark = Image.new("RGBA", (W, H), (0, 0, 0, 150))
    scenes.append(Scene(7.0, lambda t, u: Image.alpha_composite(kb.frame(u).convert("RGBA"), dark),
                        [timed(institution_overlay, 0.6, 7.0)]))

    # 3 mission
    kb3 = KenBurns(load(os.path.join(media, "image141.png")), 1.0, 1.10, (0.45, 0.5), (0.55, 0.45))
    items = [(tr("Explore unknown terrain", "استكشاف بيئة مجهولة"), 0.08),
             (tr("Map and navigate autonomously (SLAM)", "رسم الخريطة والملاحة الذاتية (SLAM)"), 0.22),
             (tr("Measure the environment", "قياس ظروف البيئة"), 0.36),
             (tr("Collect soil samples with a 6-DOF arm", "جمع عينات التربة بذراع سداسية الحرية"), 0.50),
             (tr("Report everything to the ground station", "إرسال كل البيانات إلى المحطة الأرضية"), 0.64)]
    scrim = scrim_left()
    scenes.append(Scene(11.0, lambda t, u: Image.alpha_composite(kb3.frame(u).convert("RGBA"), scrim),
                        [timed(chip(tr("The mission", "المهمة"), "01"), 0.3, 11.0),
                         timed(bullets(items, y=330), 0.0, 11.0, 0.01)]))

    # 4 structure: CAD -> photos
    a1 = KenBurns(load(os.path.join(media, "image142.jpeg")), 1.0, 1.12, (0.5, 0.4), (0.5, 0.55))
    a2 = KenBurns(load(os.path.join(media, "image144.jpeg")), 1.08, 1.0, (0.6, 0.5), (0.4, 0.5))
    a3 = KenBurns(load(os.path.join(media, "image145.jpeg")), 1.0, 1.1, (0.5, 0.6), (0.5, 0.4))
    scenes.append(Scene(13.5, seq((4.5, a1), (4.5, a2), (4.5, a3)),
                        [timed(chip(tr("Structure subsystem", "الهيكل الإنشائي"), "02"), 0.3, 13.5),
                         timed(lower_third(tr("Rocker–bogie suspension + differential pivot",
                                              "نظام تعليق روكر-بوجي مع محور تفاضلي"),
                                           tr("Laser-cut AL-1050 body and legs · AL-7075 pivot · acrylic-lined electronics box",
                                              "هيكل وأذرع من ألومنيوم AL-1050 مقطوعة بالليزر · محور من AL-7075 · صندوق إلكترونيات معزول")),
                               0.6, 6.6),
                         timed(lower_third(tr("200 mm doubled wheels · 36.5 kg", "عجلات مزدوجة بقطر 200 مم · الكتلة 36.5 كجم"),
                                           tr("1.02 × 0.73 × 0.79 m · six driven + six passive twin wheels",
                                              "الأبعاد 1.02 × 0.73 × 0.79 م · ست عجلات مدفوعة وست عجلات مساندة")),
                               7.0, 13.4)]))

    # 5 mobility: real footage among chairs
    v5 = VideoClip(pv("final presentation", "navigation.mp4"), 12.0, 9.0)
    scenes.append(Scene(9.0, lambda t, u: v5.frame(u),
                        [timed(chip(tr("Mobility subsystem", "منظومة الحركة"), "03"), 0.3, 9.0),
                         timed(lower_third(tr("Six 12 V gear-motors with encoders", "ستة محركات بتروس 12 فولت مع مشفرات"),
                                           tr("2.97 N·m each · 0.66 m/s · climbs 27° slopes at rated torque",
                                              "عزم 2.97 نيوتن·متر لكل محرك · سرعة 0.66 م/ث · تصعد منحدر 27° بالعزم المقنن"),
                                           "ستة محركات بتروس مع مشفرات"), 0.6, 8.8)]))

    # 6 power
    p1 = KenBurns(load(os.path.join(pptx, "image44.png")), 1.0, 1.12, (0.5, 0.5), (0.45, 0.4))
    p2 = KenBurns(load(os.path.join(pptx, "image45.png")), 1.12, 1.0, (0.5, 0.5), (0.5, 0.5))
    scenes.append(Scene(9.0, seq((4.5, p1), (4.5, p2)),
                        [timed(chip(tr("Power subsystem", "منظومة القدرة"), "04"), 0.3, 9.0),
                         timed(lower_third(tr("50 W solar panel · 540 Wh Li-ion", "لوح شمسي 50 وات · بطاريات ليثيوم 540 وات·ساعة"),
                                           tr("~6.7 h on a typical sortie · about two sunny days to recharge",
                                              "نحو 6.7 ساعة في مهمة نموذجية · إعادة شحن كاملة في يومين مشمسين تقريباً")),
                               0.6, 8.8)]))

    # 7 brain: mapping clip + navigation clip
    v7a = KenBurns(load(os.path.join(pptx, "image42.png")), 1.0, 1.15, (0.45, 0.5), (0.4, 0.45))
    v7b = VideoClip(pv("final presentation", "navigation.mp4"), 1.0, 6.0)
    v7c = VideoClip(pv("final presentation", "navigation.mp4"), 38.0, 7.0)
    scenes.append(Scene(18.0, seq((5.0, v7a), (6.0, v7b), (7.0, v7c)),
                        [timed(chip(tr("Brain subsystem · ROS", "العقل · ROS"), "05"), 0.3, 18.0),
                         timed(lower_third(tr("Hector SLAM with a 360° LiDAR", "رسم الخريطة بخوارزمية Hector SLAM وليدار 360°"),
                                           tr("Raspberry Pi 4 builds the map of the lab while the rover drives",
                                              "حاسوب Raspberry Pi 4 يرسم خريطة المعمل أثناء حركة المركبة")), 0.6, 5.0),
                         timed(lower_third(tr("Autonomous navigation", "ملاحة ذاتية"),
                                           tr("Saved map + AMCL localisation + DWA local planner",
                                              "خريطة محفوظة + تحديد الموقع بـ AMCL + مخطط مسار محلي DWA"),
                                           "ملاحة ذاتية بخريطة SLAM"), 5.3, 17.8)]))

    # 8 payload sensors
    v8a = VideoClip(pv("final presentation", "sensor.mp4"), 8.0, 6.0)
    v8b = VideoClip(pv("final presentation", "sensor.mp4"), 49.0, 5.0)
    scenes.append(Scene(11.0, seq((6.0, v8a), (5.0, v8b)),
                        [timed(chip(tr("Payload subsystem", "الحمولة العلمية"), "06"), 0.3, 11.0),
                         timed(lower_third(tr("Environmental sensing", "استشعار البيئة"),
                                           tr("Gas (MQ-2, MQ-4) · temperature & humidity · pressure · IMU · soil · rain · metal",
                                              "الغازات · الحرارة والرطوبة · الضغط · الاتجاه والتسارع · التربة · المطر · المعادن")),
                               0.6, 6.0),
                         timed(lower_third(tr("Live sensor topics on ROS", "قراءات حية على ROS"),
                                           tr("Every reading reaches the Brain and the ground station",
                                              "كل قراءة تصل إلى الحاسوب الرئيسي والمحطة الأرضية")), 6.2, 10.8)]))

    # 9 robotic arm (crop VEED watermark at the top)
    v9 = VideoClip(os.path.join(pptx, "media4.mp4"), 1.0, 9.0, mode="contain", crop=(0.0, 0.10, 1.0, 0.90))
    scenes.append(Scene(9.0, lambda t, u: v9.frame(u),
                        [timed(chip(tr("Robotic arm", "الذراع الروبوتية"), "07"), 0.3, 9.0),
                         timed(lower_third(tr("6-DOF sampling arm", "ذراع لجمع العينات بست درجات حرية"),
                                           tr("Six MG996R servos on a PCA9685 driver · ~37 cm reach",
                                              "ستة محركات سيرفو MG996R عبر مشغل PCA9685 · مدى نحو 37 سم")), 0.6, 8.8)]))

    # 10 ground station
    g1 = KenBurns(load(os.path.join(media, "image184.jpeg")), 1.0, 1.08, (0.5, 0.5), (0.4, 0.4), mode="contain")
    gs_png = os.path.join(ROOT, "ground_station", "docs", "gs_screenshot.png")
    g2 = KenBurns(load(gs_png), 1.0, 1.06, (0.5, 0.5), (0.55, 0.45), mode="contain")
    scenes.append(Scene(13.0, seq((6.0, g1), (7.0, g2)),
                        [timed(chip(tr("Communication & ground station", "الاتصالات والمحطة الأرضية"), "08"), 0.3, 13.0),
                         timed(lower_third(tr("433 MHz HC-12 link · SSP protocol with CRC-16",
                                              "وصلة HC-12 بتردد 433 ميجاهرتز · بروتوكول SSP مع فحص CRC-16"),
                                           tr("LabVIEW ground station (2023) · Flutter + Firebase mobile app",
                                              "محطة LabVIEW الأرضية (2023) · تطبيق هاتف Flutter مع Firebase")), 0.6, 6.0),
                         timed(lower_third(tr("New web ground station (2026)", "محطة أرضية جديدة على الويب (2026)"),
                                           tr("Runs in Chrome with Web Serial · works offline · built-in virtual rover",
                                              "تعمل في متصفح Chrome عبر Web Serial · بدون إنترنت · مع مركبة افتراضية مدمجة")),
                               6.3, 12.8)]))

    # 11 digital twin: animated SLAM
    anim = SlamAnim()
    bgc = Image.new("RGBA", (W, H), (16, 17, 22, 255))

    def twin_layer(t, u):
        im = bgc.copy()
        mp, ts, err = anim.render(min(1.0, u * 1.05))
        x0, y0 = (int(96 * S) if RTL else int(720 * S)), int(170 * S)
        im.paste(mp, (x0, y0))
        d = ImageDraw.Draw(im)
        x = int(96 * S)
        put(d, x, int(300 * S), tr("Digital twin", "التوأم الرقمي"), F_HEAD(66), WHITE)
        put(d, x, int(390 * S), tr("Simulated lab · LiDAR scan matching", "معمل محاكى · مطابقة مسح الليدار"),
            F_REG(32), SOFT)
        put(d, x, int(500 * S), tr(f"t = {ts:5.1f} s", f"الزمن {ts:5.1f} ث"), F_SEMI(40), WHITE)
        put(d, x, int(560 * S), tr(f"position error {err * 100:4.1f} cm", f"خطأ الموقع {err * 100:4.1f} سم"),
            F_SEMI(40), (120, 175, 245))
        put(d, x, int(660 * S), tr("blue: SLAM estimate", "الأزرق: تقدير SLAM"), F_REG(28), (120, 175, 245))
        put(d, x, int(704 * S), tr("dark: ground truth", "الداكن: المسار الحقيقي"), F_REG(28), SOFT)
        put(d, x, int(748 * S), tr("orange: live LiDAR returns", "البرتقالي: نقاط الليدار الحية"), F_REG(28), MARS)
        return im
    scenes.append(Scene(14.0, twin_layer, [timed(chip(tr("2026 upgrade · verification", "تطوير 2026 · التحقق"), "09"),
                                                 0.3, 14.0)], vign=False, grad=False))

    # 12 results figures
    def fig_card(path, title, sub):
        img = load(path)
        card = Image.new("RGB", (W, H), (246, 245, 241))
        r = min(W * 0.86 / img.width, H * 0.62 / img.height)
        im2 = img.resize((int(img.width * r), int(img.height * r)), Image.LANCZOS)
        card.paste(im2, ((W - im2.width) // 2, int(H * 0.16)))
        d = ImageDraw.Draw(card)
        put(d, int(96 * S), int(H * 0.82), title, F_BOLD(46), INK)
        put(d, int(96 * S), int(H * 0.82 + 62 * S), sub, F_REG(30), (82, 81, 78))
        return KenBurns(card, 1.0, 1.04, (0.5, 0.5), (0.5, 0.5))
    f1 = fig_card(os.path.join(OUT_FIG, "fig_obstacle_detection.png"),
                  tr("Seeing low obstacles: 231 → 0 collisions", "رؤية العوائق المنخفضة: من 231 تصادماً إلى صفر"),
                  tr("LiDAR on a mast misses anything lower than 0.6 m; persistent ultrasonic sensing closes the gap",
                     "الليدار على الصاري لا يرى ما هو أقل من 0.6 م؛ والاستشعار فوق الصوتي المستمر يسد هذه الفجوة"))
    f2 = fig_card(os.path.join(OUT_FIG, "fig_protocol_mc.png"),
                  tr("A safer command link: 0 wrong commands in 520 000 trials",
                     "وصلة أوامر أكثر أماناً: لا أوامر خاطئة في 520,000 تجربة"),
                  tr("The 2023 receiver ignored its CRC; the new receiver rejects every corrupted frame",
                     "المستقبل القديم تجاهل فحص CRC، والجديد يرفض كل إطار تالف"))
    f3 = fig_card(os.path.join(OUT_FIG, "fig_odometry.png"),
                  tr("Odometry drift: 11.4 % → 0.5 % of distance", "انحراف الملاحة الحسابية: من 11.4% إلى 0.5% من المسافة"),
                  tr("Calibrated skid-steer model + gyro heading", "نموذج انزلاق معاير مع اتجاه من الجيروسكوب"))
    scenes.append(Scene(18.0, seq((6.0, f1), (6.0, f2), (6.0, f3)), [], vign=False, grad=False))

    # 13 numbers
    kb13 = KenBurns(load(os.path.join(media, "image143.jpeg")), 1.0, 1.08, (0.5, 0.5), (0.5, 0.45))
    dark13 = Image.new("RGBA", (W, H), (0, 0, 0, 165))
    scenes.append(Scene(10.0, lambda t, u: Image.alpha_composite(kb13.frame(u).convert("RGBA"), dark13),
                        [timed(stat_tiles, 0.2, 10.0, 0.2)], grad=False))

    # 14 deliverables
    kb14 = KenBurns(load(os.path.join(FIG, "src", "nav_22.jpg")), 1.0, 1.1, (0.5, 0.5), (0.45, 0.5))
    items14 = [(tr("Research paper (IEEE format)", "ورقة بحثية بصيغة IEEE"), 0.05),
               (tr("ROS 2 Jazzy software stack + simulator", "منظومة برمجيات ROS 2 Jazzy مع محاكي"), 0.17),
               (tr("New firmware for all three microcontrollers", "برمجيات جديدة للمتحكمات الثلاثة"), 0.29),
               (tr("Interface control document v2", "وثيقة التحكم في الواجهات (الإصدار 2)"), 0.41),
               (tr("Web ground station + Python tools", "محطة أرضية على الويب وأدوات بايثون"), 0.53),
               (tr("Analysis scripts for every result", "سكربتات تحليل لكل نتيجة"), 0.65)]
    scenes.append(Scene(10.0, lambda t, u: Image.alpha_composite(kb14.frame(u).convert("RGBA"), scrim),
                        [timed(chip(tr("What this project delivers", "مخرجات المشروع"), "10"), 0.3, 10.0),
                         timed(bullets(items14, y=300, size=38, gap=70), 0.0, 10.0, 0.01)]))

    # 15 team & credits
    kb15 = KenBurns(load(os.path.join(pptx, "s1.jpg")), 1.0, 1.08, (0.35, 0.5), (0.3, 0.5))

    def team_layer(t, u):
        base = kb15.frame(u).convert("RGBA")
        sh = np.zeros((H, W, 4), np.uint8)
        ramp = np.clip((np.arange(W) - W * 0.38) / (W * 0.2), 0, 1) * 225
        sh[..., 3] = ramp[None, :].astype(np.uint8)
        base.alpha_composite(Image.fromarray(sh, "RGBA"))
        return base
    scenes.append(Scene(14.0, team_layer, [timed(credits_overlay, 0.5, 14.0)], fade_out=1.6, grad=False))
    return scenes


def main():
    global VIGNETTE, GRADIENT
    VIGNETTE = vignette()
    GRADIENT = bottom_gradient(170)
    scenes = build_scenes()
    total = sum(s.dur for s in scenes)
    print(f"{len(scenes)} scenes, {total:.1f} s", flush=True)
    music = os.path.join(BUILD, "music.wav")
    synth_music(total + 0.5, music)
    out = os.path.join(HERE, "MissionX_Rover_Demo" + ("_AR" if RTL else "") + ("_preview" if PREVIEW else "") + ".mp4")
    cmd = ["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS),
           "-i", "-", "-i", music, "-map", "0:v", "-map", "1:a", "-c:v", "libx264", "-preset", "medium",
           "-crf", "20" if not PREVIEW else "26", "-pix_fmt", "yuv420p", "-c:a", "aac", "-b:a", "192k",
           "-movflags", "+faststart", "-shortest", out]
    enc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    nframes = 0
    for i, sc in enumerate(scenes):
        for fr in sc.frames():
            enc.stdin.write(fr.tobytes())
            nframes += 1
        for attr in ("layers",):
            pass
        print(f"scene {i + 1}/{len(scenes)} done ({nframes} frames)", flush=True)
    enc.stdin.close()
    enc.wait()
    print("written", out)


if __name__ == "__main__":
    main()
