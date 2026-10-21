"""从参考图测环形靶各环的实际 RGB/HSV 值，用于设定 D-01 的阈值。

为什么需要：仿真渲染的颜色与"标准靶纸"的名义颜色往往差很多，
凭肉眼或凭标称值设 HSV 阈值会检不出来或误检。
本脚本直接取靶心向外的采样点，打印每个环的实测 HSV 区间。
"""
import colorsys
from pathlib import Path

import numpy as np
from PIL import Image

REF = Path("交付包/射击/参考图")

# 由放大图标定的靶心与半径（原图坐标）
CENTER = (410, 358)
RADIUS = 41.0


def hsv(rgb):
    r, g, b = [v / 255.0 for v in rgb[:3]]
    h, s, v = colorsys.rgb_to_hsv(r, g, b)
    return h * 179.0, s * 255.0, v * 255.0   # 与 OpenCV 的 H∈[0,179] 对齐


def main():
    p = REF / "point1_ring.png"
    im = np.array(Image.open(p).convert("RGB"))
    cx, cy = CENTER
    h, w = im.shape[:2]
    print(f"图 {p.name} {w}x{h}  靶心=({cx},{cy}) 半径={RADIUS}")
    print()
    print(f"{'距中心':>8} {'理论环':>6}   {'RGB':>18}   {'H':>6} {'S':>6} {'V':>6}")
    print("-" * 62)

    # 沿 4 个方向采样取中位数，避开数字标注
    rows = []
    for frac in [0.02, 0.06, 0.12, 0.18, 0.26, 0.34, 0.44, 0.54, 0.64, 0.74, 0.84, 0.94]:
        samples = []
        for ang in (0, 45, 90, 135, 180, 225, 270, 315):
            a = np.deg2rad(ang)
            x = int(cx + RADIUS * frac * np.cos(a))
            y = int(cy + RADIUS * frac * np.sin(a))
            if 0 <= x < w and 0 <= y < h:
                samples.append(im[y, x])
        if not samples:
            continue
        med = np.median(np.array(samples), axis=0).astype(int)
        H, S, V = hsv(med)
        # 环号：半径分数 0..1 对应 10 环(中心) .. 6 环(外缘)
        ring = 10 - int(frac * 5)
        rows.append((frac, ring, tuple(med), H, S, V))
        print(f"{frac*RADIUS:7.1f}px {ring:6d}   {str(tuple(med)):>18}   "
              f"{H:6.1f} {S:6.0f} {V:6.0f}")

    print()
    print("=== 结论 ===")
    if rows:
        c = rows[0]
        print(f"黄心 RGB={c[2]}  H≈{c[3]:.0f} S≈{c[4]:.0f} V≈{c[5]:.0f}")
        print("  → 黄色判据建议: H∈[{:.0f},{:.0f}], S>={:.0f}, V>={:.0f}".format(
            max(0, c[3] - 12), min(179, c[3] + 12), max(60, c[4] * 0.6), max(80, c[5] * 0.6)))
        reds = [r for r in rows if 10 < r[3] < 175 or r[3] > 175 or r[3] < 10]
        print()
        print("每环像素宽度检查：")
        for i in range(len(rows) - 1):
            d = (rows[i + 1][0] - rows[i][0]) * RADIUS
            print(f"  环{rows[i][1]}→环{rows[i+1][1]}: {d:.1f} px")


if __name__ == "__main__":
    main()
