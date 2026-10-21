"""环形靶检测 v2 调试：打印每个颜色的色块统计，定位 ROI 污染与阈值问题。"""
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ring_detect as rd  # noqa: E402

REF = Path("交付包/射击/参考图")
OUT = REF / "_analysis"
OUT.mkdir(parents=True, exist_ok=True)


def main():
    p = REF / "point1_ring.png"
    bgr = rd.imread_unicode(p)
    h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    print(f"{p.name} {w}x{h}")
    print("靶真值: 中心(410,358) 半径41\n")

    for roi_ratio in (0.60, 0.53, 0.50):
        roi = np.zeros((h, w), np.uint8)
        roi[: int(h * roi_ratio), :] = 255
        print(f"===== ROI y < {roi_ratio:.2f}*H = {int(h*roi_ratio)} px =====")
        for color in ("red", "blue", "yellow"):
            mask = cv2.bitwise_and(rd.color_mask(hsv, color), roi)
            ys, xs = np.nonzero(mask)
            n = xs.size
            if n == 0:
                print(f"  {color:<7} 像素=0")
                continue
            cx, cy = xs.mean(), ys.mean()
            d = np.hypot(xs - cx, ys - cy)
            print(f"  {color:<7} 像素={n:6d} 质心=({cx:6.1f},{cy:6.1f}) "
                  f"r99={np.percentile(d,99):6.1f} rmax={d.max():6.1f} "
                  f"min_px={rd.MIN_PIXELS[color]}")
            rd.imwrite_unicode(OUT / f"v2_mask_{color}_roi{roi_ratio:.2f}.png", mask)
        print()

    # 已知靶所在的窄 ROI 下的统计（作为"理想情况"对照）
    print("===== 仅靶区域 y∈[300,420], x∈[350,470] =====")
    sub = hsv[300:420, 350:470]
    for color in ("red", "blue", "yellow"):
        m = rd.color_mask(sub, color)
        ys, xs = np.nonzero(m)
        print(f"  {color:<7} 像素={xs.size:5d}", end="")
        if xs.size:
            print(f" 质心=({350+xs.mean():6.1f},{300+ys.mean():6.1f})")
        else:
            print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
