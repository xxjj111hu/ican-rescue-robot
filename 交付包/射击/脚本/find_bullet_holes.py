"""找弹孔：把射击前后两张靶子图做差分，定位弹孔位置并换算成环数。

为什么用差分
------------
靶纸上弹孔很小（几像素），在 800x800 整图里肉眼几乎看不出。
射击前后两张图在同一停车位拍摄，做逐像素差分后，弹孔会作为局部亮/暗斑凸显出来。

注意：两次拍摄的机器人位姿略有差异（导航误差几厘米），会造成整体轻微错位。
所以先以靶心为参考做小范围平移配准，再差分，避免把错位误判成弹孔。
"""
from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ring_detect import detect_ring, imread_unicode, imwrite_unicode  # noqa: E402

REF = Path("交付包/射击/参考图")
OUT = REF / "_analysis"
OUT.mkdir(parents=True, exist_ok=True)

BEFORE = REF / "point1_ring.png"
AFTER = REF / "after_shot1.png"


def align_shift(a, b, center, win=25):
    """在以 center 为中心的小窗口内，找让 b 最接近 a 的平移量。"""
    cx, cy = int(center[0]), int(center[1])
    x0, y0 = cx - win, cy - win
    x1, y1 = cx + win, cy + win
    pa = a[y0:y1, x0:x1].astype(np.float32)
    best = (0, 0, 1e18)
    for dy in range(-8, 9):
        for dx in range(-8, 9):
            pb = b[y0 + dy:y1 + dy, x0 + dx:x1 + dx].astype(np.float32)
            if pb.shape != pa.shape:
                continue
            err = float(np.mean((pa - pb) ** 2))
            if err < best[2]:
                best = (dx, dy, err)
    return best[0], best[1], best[2]


def main():
    a = imread_unicode(BEFORE)
    b = imread_unicode(AFTER)
    if a is None or b is None:
        print("读图失败")
        return 1
    print(f"before={a.shape}  after={b.shape}")

    da = detect_ring(a)
    db = detect_ring(b)
    print(f"before 靶心=({da['x']:.1f},{da['y']:.1f}) r={da['radius']:.1f}")
    print(f"after  靶心=({db['x']:.1f},{db['y']:.1f}) r={db['radius']:.1f}")

    if not (da["ok"] and db["ok"]):
        print("有一张没检出靶，无法配准")
        return 1

    dx, dy, err = align_shift(a, b, (db["x"], db["y"]))
    print(f"配准平移量 dx={dx} dy={dy}  (残差 MSE={err:.1f})")

    # 把 b 平移回与 a 对齐，再差分
    M = np.float32([[1, 0, -dx], [0, 1, -dy]])
    b_aligned = cv2.warpAffine(b, M, (b.shape[1], b.shape[0]),
                               flags=cv2.INTER_NEAREST,
                               borderMode=cv2.BORDER_REPLICATE)
    imwrite_unicode(OUT / "after_aligned.png", b_aligned)

    ga = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY).astype(np.int16)
    gb = cv2.cvtColor(b_aligned, cv2.COLOR_BGR2GRAY).astype(np.int16)
    diff = np.abs(ga - gb).astype(np.uint8)
    _, thr = cv2.threshold(diff, 40, 255, cv2.THRESH_BINARY)
    imwrite_unicode(OUT / "diff.png", diff)
    imwrite_unicode(OUT / "diff_thr.png", thr)

    # 只在靶面范围内找变化（半径 1.3 倍以内），排除地面/背景
    cx, cy, r = db["x"], db["y"], db["radius"]
    yy, xx = np.mgrid[0:thr.shape[0], 0:thr.shape[1]]
    inside = ((xx - cx) ** 2 + (yy - cy) ** 2) <= (r * 1.25) ** 2
    mask = (thr > 0) & inside
    n = int(mask.sum())
    print(f"\n靶面范围内变化像素数={n}")

    if n == 0:
        print("❌ 靶面内没有检测到变化 —— 子弹可能没打到这张靶（或弹孔太小被阈值滤掉）")
        # 放宽阈值再试
        _, thr2 = cv2.threshold(diff, 20, 255, cv2.THRESH_BINARY)
        mask2 = (thr2 > 0) & inside
        print(f"放宽阈值(20)后变化像素数={int(mask2.sum())}")
        imwrite_unicode(OUT / "diff_thr20.png", thr2)
        return 0

    ys, xs = np.nonzero(mask)
    # 聚类：简单按连通域
    m8 = (mask.astype(np.uint8)) * 255
    ncomp, labels, stats, centroids = cv2.connectedComponentsWithStats(m8, 8)
    print(f"连通域数={ncomp - 1}")
    print(f"{'#':>3} {'位置(px)':>18} {'面积':>6} {'相对靶心':>14} {'估算环数':>9}")
    print("-" * 60)
    rows = []
    for i in range(1, ncomp):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if area < 3:
            continue
        px, py = centroids[i]
        ddx, ddy = px - cx, py - cy
        per_ring = r / 5.0
        rings = 10.0 - float(np.hypot(ddx, ddy)) / per_ring
        rows.append((area, px, py, ddx, ddy, rings))
    rows.sort(key=lambda t: -t[0])
    for k, (area, px, py, ddx, ddy, rings) in enumerate(rows[:8], 1):
        print(f"{k:>3} ({px:6.1f},{py:6.1f}) {area:>6} "
              f"({ddx:+6.1f},{ddy:+6.1f}) {rings:>8.1f}")

    if rows:
        area, px, py, ddx, ddy, rings = rows[0]
        print()
        print(f"最大变化块 = 最可能的弹孔：相对靶心 ({ddx:+.1f}, {ddy:+.1f}) px")
        print(f"  → 估算环数 {rings:.1f}")
        print(f"  → 横向偏 {ddx:+.1f} px = {ddx / (r/5.0):+.2f} 环宽")
        print(f"  → 纵向偏 {ddy:+.1f} px")
    return 0


if __name__ == "__main__":
    sys.exit(main())
