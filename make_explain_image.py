# -*- coding: utf-8 -*-
"""按 abot_land.urdf 的相机参数，示意「素材原图」与「车上摄像头看到的同一张图」的差别。"""
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

WS = Path(".")
TEX = WS / "demo_shoot_ws/demo_shoot_ws/assets/point_targets/rescue_camp_01.png"
OUT = WS / "说明图-相机看到的地面图片.png"

# --- 相机参数（来自 abot_land.urdf / world） ---
IMG = 800                     # 800x800
CX = CY = IMG / 2.0
HFOV = 1.3962634              # 水平 FOV
F = (IMG / 2.0) / math.tan(HFOV / 2.0)   # 焦距(像素) ≈ 476.7
H = 0.1284                    # 相机离地高
CAM_FWD = 0.0807              # 相机相对车中心前移
TILE_L = 0.52971              # 地面图片尺寸(沿行进方向)
TILE_W = 0.38454              # 地面图片尺寸(横向)


def frame_with_tile(tex: Image.Image, dist: float):
    """dist = 车中心到图片中心的距离；返回(整帧图像, 图片四角像素坐标)"""
    d_center = dist - CAM_FWD
    d1 = d_center - TILE_L / 2.0          # 近边
    d2 = d_center + TILE_L / 2.0          # 远边
    if d1 <= 0.16:                        # 相机最近可见地面约 0.155m
        d1 = 0.16

    Ws, Hs = tex.size
    A = F * H

    def to_img(p, q):
        d = d1 + p
        y = q - TILE_W / 2.0
        return CX + F * y / d, CY + A / d

    quad = [to_img(0, 0), to_img(TILE_L, 0), to_img(TILE_L, TILE_W), to_img(0, TILE_W)]

    # 输出像素 -> 素材像素 的投影矩阵（地面平面 → 图像 的逆映射）
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
    return bg, quad


def caption(img, text, size=26, pad=8, color=(255, 255, 255)):
    bar = Image.new("RGB", (img.width, size + 2 * pad), (16, 17, 20))
    d = ImageDraw.Draw(bar)
    d.text((pad, pad - 2), text, font=font(size), fill=color)
    out = Image.new("RGB", (img.width, img.height + bar.height), (16, 17, 20))
    out.paste(bar, (0, 0))
    out.paste(img, (0, bar.height))
    return out


FONT_CACHE = {}


def font(size):
    if size in FONT_CACHE:
        return FONT_CACHE[size]
    f = None
    for p in (r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\msyhbd.ttc",
              r"C:\Windows\Fonts\simhei.ttf", r"C:\Windows\Fonts\simsun.ttc"):
        try:
            f = ImageFont.truetype(p, size)
            break
        except Exception:
            continue
    FONT_CACHE[size] = f or ImageFont.load_default()
    return FONT_CACHE[size]


tex = Image.open(TEX).convert("RGB")
print("素材原图尺寸:", tex.size)

panels = {}

# A：素材原图
side = 360
a = tex.copy()
a.thumbnail((side, side), Image.LANCZOS)
canvas_a = Image.new("RGB", (side, side), (26, 28, 32))
canvas_a.paste(a, ((side - a.width) // 2, (side - a.height) // 2))
panels["A"] = caption(canvas_a, "① 素材库里的原图（assets/point_targets）")

for tag, dist in (("B", 1.0), ("C", 0.7)):
    frame, quad = frame_with_tile(tex, dist)
    xs = [p[0] for p in quad]
    ys = [p[1] for p in quad]
    h_px = max(ys) - min(ys)
    w_px = max(xs) - min(xs)

    shown = frame.copy()
    d = ImageDraw.Draw(shown)
    d.polygon([tuple(p) for p in quad], outline=(255, 60, 60), width=3)
    shown.thumbnail((side, side), Image.LANCZOS)
    canvas = Image.new("RGB", (side, side), (26, 28, 32))
    canvas.paste(shown, ((side - shown.width) // 2, (side - shown.height) // 2))
    panels[tag] = caption(
        canvas,
        f"{'②' if tag == 'B' else '③'} 车上看到的（车到该点 {dist:.1f}m）：红框约 {w_px:.0f}×{h_px:.0f} 像素",
    )

    # 放大特写
    box = (max(0, int(min(xs)) - 12), max(0, int(min(ys)) - 12),
           min(IMG, int(max(xs)) + 12), min(IMG, int(max(ys)) + 12))
    crop = frame.crop(box)
    k = max(1, int(min(side / crop.width, side / crop.height)))
    crop = crop.resize((crop.width * k, crop.height * k), Image.NEAREST)
    canvas = Image.new("RGB", (side, side), (26, 28, 32))
    canvas.paste(crop, ((side - crop.width) // 2, (side - crop.height) // 2))
    panels[tag + "z"] = caption(
        canvas,
        f"{'④' if tag == 'B' else '⑤'} 红框里放大 ×{k}（模型真正吃到的输入）",
    )

W, Hh, gap = 360, 40, 10
grid = Image.new("RGB", (W * 3 + gap * 4, (W + Hh) * 2 + gap * 3 + 70), (10, 11, 13))
title = ImageDraw.Draw(grid)
title.text((gap + 4, 12), "同一张地面素材图：左侧是原图，右侧是车上摄像头实际看到的样子"
                          "（按 abot_land.urdf 相机参数投影推算，未叠加材质 rotate 90）",
           font=font(24), fill=(255, 255, 255))

layout = [("A", 0, 0), ("B", 1, 0), ("C", 2, 0), ("Bz", 1, 1), ("Cz", 2, 1)]
for tag, col, row in layout:
    img = panels[tag]
    x = gap + col * (W + gap)
    y = 70 + row * (W + Hh + gap)
    grid.paste(img, (x, y))

grid.save(OUT)
print("saved", OUT, grid.size)
