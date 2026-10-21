"""环形靶区域颜色聚类：得出各环的实测 HSV（含黄心），供 D-01 设阈值。

前一版沿半径采样会踩到靶心的"10"数字与环上的数字标注，得到偏暗的假值。
改为：裁出靶区域 -> 全像素聚成若干色簇 -> 按色相归类，报每类的像素数与 HSV 范围。
"""
from pathlib import Path

import numpy as np
from PIL import Image

REF = Path("交付包/射击/参考图")
CENTER = (410, 358)
HALF = 46   # 靶半径约 41，留点余量


def rgb_to_hsv_arr(rgb):
    """rgb: uint8 [...,3] -> h(0-179), s(0-255), v(0-255)"""
    a = rgb.astype(np.float32) / 255.0
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    mx = a.max(axis=-1)
    mn = a.min(axis=-1)
    diff = mx - mn
    v = mx
    s = np.where(mx > 1e-6, diff / np.maximum(mx, 1e-6), 0.0)
    h = np.zeros_like(mx)
    m = diff > 1e-6
    # 标准 RGB->Hue
    rm = m & (mx == r)
    gm = m & (mx == g) & ~rm
    bm = m & (mx == b) & ~rm & ~gm
    h[rm] = (60 * ((g[rm] - b[rm]) / diff[rm])) % 360
    h[gm] = 60 * ((b[gm] - r[gm]) / diff[gm] + 2)
    h[bm] = 60 * ((r[bm] - g[bm]) / diff[bm] + 4)
    return (h / 2.0), s * 255.0, v * 255.0   # H 折到 0~179


def describe(name, mask, hsv):
    h, s, v = hsv
    n = int(mask.sum())
    if n == 0:
        print(f"  {name:<8} 无像素")
        return
    hh, ss, vv = h[mask], s[mask], v[mask]
    print(f"  {name:<8} px={n:6d}  H=[{hh.min():5.1f},{hh.max():5.1f}] "
          f"(中位 {np.median(hh):5.1f})  S=[{ss.min():3.0f},{ss.max():3.0f}] "
          f"(中位 {np.median(ss):3.0f})  V=[{vv.min():3.0f},{vv.max():3.0f}] "
          f"(中位 {np.median(vv):3.0f})")


def main():
    im = np.array(Image.open(REF / "point1_ring.png").convert("RGB"))
    cx, cy = CENTER
    region = im[cy - HALF:cy + HALF, cx - HALF:cx + HALF]
    hsv = rgb_to_hsv_arr(region.reshape(-1, 3))
    h, s, v = hsv
    print(f"靶区域 {region.shape[1]}x{region.shape[0]} = {h.shape[0]} 像素")
    print()

    # 五类：黄心 / 红 / 蓝 / 白 / 黑
    yellow = (h >= 18) & (h <= 42) & (s > 110) & (v > 90)
    red = ((h <= 12) | (h >= 168)) & (s > 120) & (v > 70)
    blue = (h >= 88) & (h <= 118) & (s > 110) & (v > 70)
    white = (s < 45) & (v > 95)
    black = (v < 60) & (s < 90)

    print("按色相归类（用于设定 HSV 阈值）:")
    for nm, mk in (("黄心(10环)", yellow), ("红(6环)", red), ("蓝(8环)", blue),
                   ("白(7环)", white), ("黑(9/外圈)", black)):
        describe(nm, mk, hsv)

    print()
    print("=== 可直接用的判据 ===")
    if yellow.sum() > 30:
        hh = h[yellow]; ss = s[yellow]; vv = v[yellow]
        print(f"黄心: H∈[{np.percentile(hh,10):.0f},{np.percentile(hh,90):.0f}] "
              f"S>={np.percentile(ss,10):.0f} V>={np.percentile(vv,10):.0f}")
        ys, xs = np.nonzero(yellow.reshape(HALF * 2, HALF * 2))
        print(f"黄心像素中心: ({xs.mean():.1f}, {ys.mean():.1f}) 相对裁切左上角；"
              f"换算到原图 = ({cx - HALF + xs.mean():.1f}, {cy - HALF + ys.mean():.1f})")
        print(f"黄心像素数={int(yellow.sum())} -> 等效直径约 {2*np.sqrt(yellow.sum()/np.pi):.1f} px")
    if red.sum() > 30:
        hh = h[red]; ss = s[red]; vv = v[red]
        print(f"红环: H∈[{np.percentile(hh,10):.0f},{np.percentile(hh,90):.0f}] "
              f"S>={np.percentile(ss,10):.0f} V>={np.percentile(vv,10):.0f}")
    if blue.sum() > 30:
        hh = h[blue]; ss = s[blue]; vv = v[blue]
        print(f"蓝环: H∈[{np.percentile(hh,10):.0f},{np.percentile(hh,90):.0f}] "
              f"S>={np.percentile(ss,10):.0f} V>={np.percentile(vv,10):.0f}")


if __name__ == "__main__":
    main()
