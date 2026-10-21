"""靶子参考图分析：裁剪放大 AR 码区域 + 量环形靶尺寸与配色。

用途：为 D-01（环形靶 HSV 阈值）和 D-03/D-04（靶位 AR 识别方案）提供实测依据，
      避免凭猜测设阈值。
"""
import sys
from pathlib import Path

import numpy as np
from PIL import Image

REF = Path("交付包/射击/参考图")
OUT = REF / "_analysis"
OUT.mkdir(parents=True, exist_ok=True)


def crop_zoom(name, box, scale=6):
    p = REF / name
    if not p.is_file():
        print(f"[跳过] {p} 不存在")
        return
    im = Image.open(p).convert("RGB")
    c = im.crop(box)
    w, h = c.size
    z = c.resize((w * scale, h * scale), Image.NEAREST)
    out = OUT / f"{p.stem}_zoom_x{scale}.png"
    z.save(out)
    print(f"[放大] {name} crop={box} -> {out.name}  ({w}x{h} -> {w*scale}x{h*scale})")


def analyse_ring():
    p = REF / "point1_ring.png"
    if not p.is_file():
        print("[跳过] 环形靶图不存在")
        return
    im = np.array(Image.open(p).convert("RGB"))
    h, w = im.shape[:2]
    print(f"\n=== 环形靶分析 {p.name} ({w}x{h}) ===")

    # 在靶附近找"非背景"像素：靶板是浅灰/白底，靶纸有高饱和的红/黄/蓝
    import colorsys

    # 转 HSV（用 numpy 自己算，避免依赖 cv2）
    rgb = im.astype(np.float32) / 255.0
    mx = rgb.max(axis=2)
    mn = rgb.min(axis=2)
    diff = mx - mn
    sat = np.where(mx > 0, diff / np.maximum(mx, 1e-6), 0.0)
    val = mx

    # 高饱和区域 = 靶纸彩色部分
    mask = (sat > 0.35) & (val > 0.25)
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        print("  未找到高饱和像素")
        return
    # 只取画面中上部（靶所在区域，排除地面贴图）
    sel = (ys < int(h * 0.62)) & (ys > int(h * 0.35))
    xs2, ys2 = xs[sel], ys[sel]
    print(f"  高饱和像素数={len(xs2)}  x范围=[{xs2.min()},{xs2.max()}]  y范围=[{ys2.min()},{ys2.max()}]")
    print(f"  靶心估计=({xs2.mean():.0f}, {ys2.mean():.0f})   图像中心=({w//2}, {h//2})")
    print(f"  横向偏差={xs2.mean() - w/2:+.1f} px   纵向偏差={ys2.mean() - h/2:+.1f} px")
    bw = xs2.max() - xs2.min()
    bh = ys2.max() - ys2.min()
    print(f"  彩色区域尺寸={bw}x{bh} px   （靶板 0.27 m）")
    if bw > 0:
        print(f"  → 每像素约 {0.27 / bw * 1000:.1f} mm，10 环半径约 {bw/2/10:.1f} px/环")

    # 采样几个点看颜色分布
    cy, cx = int(ys2.mean()), int(xs2.mean())
    print(f"  中心点 RGB={tuple(im[cy, cx])}")
    # 靶纸各环的代表色：沿水平线向外采样
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        x = int(cx + (xs2.max() - cx) * frac)
        x = min(max(x, 0), w - 1)
        print(f"    距中心 {frac*100:3.0f}% 处 RGB={tuple(im[cy, x])}")


def list_markers(name, thresh=60):
    """在指定图里找"高对比小块"（AR 码是黑白方格，局部方差大）。"""
    p = REF / name
    if not p.is_file():
        print(f"[跳过] {name} 不存在")
        return
    im = np.array(Image.open(p).convert("L")).astype(np.float32)
    h, w = im.shape
    print(f"\n=== {name} 高对比块位置（可能的 AR 码）===")
    # 简单网格扫描：16x16 像素格，找标准差大的格子
    cell = 16
    found = []
    for gy in range(0, h - cell, cell):
        for gx in range(0, w - cell, cell):
            blk = im[gy:gy + cell, gx:gx + cell]
            sd = blk.std()
            if sd > thresh and blk.mean() < 200:  # 排除纯白/噪点
                found.append((gx + cell // 2, gy + cell // 2, sd, blk.mean()))
    found.sort(key=lambda t: -t[2])
    for x, y, sd, m in found[:14]:
        print(f"  ({x:3d},{y:3d})  std={sd:5.1f}  mean={m:5.1f}")


if __name__ == "__main__":
    crop_zoom("point2_wheel_00.png", (350, 300, 570, 410), 6)
    crop_zoom("point2_wheel_03.png", (350, 300, 570, 410), 6)
    crop_zoom("point1_ring.png", (330, 290, 480, 430), 6)
    crop_zoom("point3_moving3_00.png", (250, 280, 620, 420), 4)
    crop_zoom("point4_moving4_00.png", (250, 280, 620, 420), 4)
    analyse_ring()
    list_markers("point2_wheel_00.png")
    list_markers("point3_moving3_00.png")
    list_markers("point4_moving4_00.png")
    print(f"\n放大图已存到 {OUT}")
