# -*- coding: utf-8 -*-
"""按 abot_land.urdf 的相机参数，示意「素材原图」与「车上摄像头看到的样子」的差别。"""
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

WS = Path(".")
TEX = WS / "demo_shoot_ws/demo_shoot_ws/assets/point_targets/rescue_camp_01.png"
OUT = WS / "说明图-相机看到的地面图片.png"

IMG = 800
CX = CY = IMG / 2.0
HFOV = 1.3962634
F = (IMG / 2.0) / math.tan(HFOV / 2.0)
H = 0.1284
CAM_FWD = 0.0807
TILE_L = 0.52971
TILE_W = 0.38454

FONT_CACHE = {}


def font(size):
    if size not in FONT_CACHE:
        f = None
        for p in (r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf",
                  r"C:\Windows\Fonts\simsun.ttc"):
            try:
                f = ImageFont.truetype(p, size)
                break
            except Exception:
                continue
        FONT_CACHE[size] = f or ImageFont.load_default()
    return FONT_CACHE[size]


def frame_with_tile(tex, dist):
    d_center = dist - CAM_FWD
    d1 = max(0.16, d_center - TILE_L / 2.0)
    d2 = d_center + TILE_L / 2.0
    Ws, Hs = tex.size
    A = F * H

    def to_img(p, q):
        d = d1 + p
        return CX + F * (q - TILE_W / 2.0) / d, CY + A / d

    quad = [to_img(0, 0), to_img(TILE_L, 0), to_img(TILE_L, TILE_W), to_img(0, TILE_W)]

    M = np.array([
        [(Ws * H / TILE_W), (Ws / 2.0), -(Ws * H / TILE_W) * CX - (Ws / 2.0) * CY],
        [0.0, -(Hs * d1 / TILE_L), (Hs * d1 / TILE_L) * CY + (Hs * A / TILE_L)],
        [0.0, 1.0, -CY],
    ])
    M = M / M[2, 2]
    coeffs = (M[0, 0], M[0, 1], M[0, 2], M[1, 0], M[1, 1], M[1, 2], M[2, 0], M[2, 1])
    warped = tex.convert("RGB").transform((IMG, IMG), Image.PERSPECTIVE, coeffs, Image.BILINEAR)
    mask = Image.new("L", (IMG, IMG), 0)
    ImageDraw.Draw(mask).polygon([tuple(p) for p in quad], fill=255)
    bg = Image.new("RGB", (IMG, IMG), (26, 28, 32))
    bg.paste(warped, (0, 0), mask)
    xs = [p[0] for p in quad]
    ys = [p[1] for p in quad]
    return bg, quad, (max(xs) - min(xs), max(ys) - min(ys)), (min(xs), min(ys), max(xs), max(ys))


def cap(img, text, size=20, pad=7):
    bar = Image.new("RGB", (img.width, size + 2 * pad), (16, 17, 20))
    ImageDraw.Draw(bar).text((pad, pad - 3), text, font=font(size), fill=(240, 240, 240))
    out = Image.new("RGB", (img.width, img.height + bar.height), (16, 17, 20))
    out.paste(bar, (0, 0))
    out.paste(img, (0, bar.height))
    return out


tex = Image.open(TEX).convert("RGB")
SW, SH = tex.size
SIDE = 560

# ① 素材原图
a = tex.copy()
a.thumbnail((SIDE - 20, SIDE - 20), Image.LANCZOS)
ca = Image.new("RGB", (SIDE, SIDE), (26, 28, 32))
ca.paste(a, ((SIDE - a.width) // 2, (SIDE - a.height) // 2))
panels = {"A": cap(ca, f"① 素材库里的原图 {SW}×{SH}")}
for tag, dist in (("B", 1.0), ("C", 0.7)):
    frame, quad, (w_px, h_px), box = frame_with_tile(tex, dist)
    shown = frame.copy()
    ImageDraw.Draw(shown).polygon([tuple(p) for p in quad], outline=(255, 60, 60), width=3)
    shown.thumbnail((SIDE, SIDE), Image.LANCZOS)
    cs = Image.new("RGB", (SIDE, SIDE), (26, 28, 32))
    cs.paste(shown, ((SIDE - shown.width) // 2, (SIDE - shown.height) // 2))
    num = "②" if tag == "B" else "③"
    panels[tag] = cap(cs, f"{num} 车到该点 {dist:.1f}m：{w_px:.0f}×{h_px:.0f}px")

    x0 = max(0, int(box[0]) - 10)
    y0 = max(0, int(box[1]) - 10)
    x1 = min(IMG, int(box[2]) + 10)
    y1 = min(IMG, int(box[3]) + 10)
    crop = frame.crop((x0, y0, x1, y1))
    k = max(1.0, min((SIDE - 20) / crop.width, (SIDE - 20) / crop.height))
    crop = crop.resize((max(1, int(crop.width * k)), max(1, int(crop.height * k))), Image.NEAREST)
    cz = Image.new("RGB", (SIDE, SIDE), (26, 28, 32))
    cz.paste(crop, ((SIDE - crop.width) // 2, (SIDE - crop.height) // 2))
    panels[tag + "z"] = cap(cz, f"{'④' if tag == 'B' else '⑤'} 红框内放大 ×{k:.1f}（模型真正的输入）")
    if tag == "B":
        area_ratio = (w_px * h_px) / float(SW * SH) * 100.0

# ⑥ 文字要点
txt = Image.new("RGB", (SIDE, SIDE), (26, 28, 32))
d = ImageDraw.Draw(txt)
lines = [
    ("这张图说明什么：", (255, 214, 102)),
    ("", None),
    ("• 素材是正的、清楚的", None),
    (f"• 车上看到的是斜的、扁的、很小的", None),
    (f"• 1.0m 处只占原图面积 {area_ratio:.1f}%", None),
    ("• 再远就完全认不出来", None),
    ("", None),
    ("→ 训练用的图必须自己截，", (140, 220, 255)),
    ("   不能拿素材原图当数据。", (140, 220, 255)),
]
y = 24
for s, col in lines:
    if s:
        d.text((18, y), s, font=font(21), fill=col or (235, 235, 235))
    y += 34
panels["T"] = cap(txt, "⑥ 结论")

W, Hh, gap = SIDE, 40, 12
cols, rows = 3, 2
top = 92
canvas = Image.new("RGB", (W * cols + gap * (cols + 1), top + (W + Hh) * rows + gap * (rows + 1)),
                   (10, 11, 13))
t = ImageDraw.Draw(canvas)
t.text((gap + 4, 14), "同一张地面素材图：左边是原图，右边是车上摄像头实际看到的样子",
       font=font(25), fill=(255, 255, 255))
t.text((gap + 4, 52), f"（按 abot_land.urdf 的相机参数投影推算：高 {H}m、rpy=0 光轴水平、FOV 80°、"
                      f"800×800；未叠加材质 rotate 90）", font=font(17), fill=(170, 175, 185))

order = [("A", 0, 0), ("B", 1, 0), ("C", 2, 0), ("T", 0, 1), ("Bz", 1, 1), ("Cz", 2, 1)]
for tag, col, row in order:
    canvas.paste(panels[tag], (gap + col * (W + gap), top + row * (W + Hh + gap)))

canvas.save(OUT)
print("saved", OUT, canvas.size, "面积占比 %.2f%%" % area_ratio)
