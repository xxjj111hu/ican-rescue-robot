"""离线验证环形靶检测：在采集到的参考图上跑 ring_detect，检查检出精度。

这是"D-01 视觉部分"的单元测试 —— 不需要 ROS、不需要仿真，
直接用已保存的 point1_ring.png 验证阈值与算法，避免把错误算法搬到仿真里试。
"""
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ring_detect import (  # noqa: E402
    detect_ring, draw_debug, imread_unicode, imwrite_unicode,
    ring_score_from_offset,
)

REF = Path("交付包/射击/参考图")
OUT = REF / "_analysis"
OUT.mkdir(parents=True, exist_ok=True)

# 由放大图标定的真值（见 measure_ring_colors.py）
TRUTH = {"point1_ring.png": ((410, 358), 41.0)}


def main():
    files = sorted(REF.glob("point*.png"))
    if not files:
        print(f"❌ {REF} 下没有参考图")
        return 1

    print(f"{'文件':<26}{'检出':>6}{'靶心 x,y':>16}{'半径':>8}{'dx':>9}{'conf':>7}{'估环':>7}")
    print("-" * 82)

    ok_cnt = 0
    for p in files:
        bgr = imread_unicode(p)
        if bgr is None:
            print(f"{p.name:<26} 读取失败")
            continue
        det = detect_ring(bgr)
        if det["ok"]:
            ok_cnt += 1
            score = ring_score_from_offset(det["dx"], det["radius"])
            print(f"{p.name:<26}{'✅':>6}{det['x']:>8.1f},{det['y']:<6.1f}"
                  f"{det['radius']:>8.1f}{det['dx']:>+9.1f}{det['conf']:>7.2f}{score:>7.1f}")
            if p.name in TRUTH:
                (tx, ty), tr = TRUTH[p.name]
                err = np.hypot(det["x"] - tx, det["y"] - ty)
                print(f"  真值 ({tx},{ty}) r={tr:.0f} → 靶心误差 {err:.1f} px，"
                      f"半径误差 {det['radius']-tr:+.1f} px")
        else:
            print(f"{p.name:<26}{'❌':>6}   未检出")

        dbg = draw_debug(bgr, det)
        imwrite_unicode(OUT / f"{p.stem}_detect.png", dbg)

    print("-" * 82)
    print(f"检出 {ok_cnt}/{len(files)} 张")
    print(f"调试图写到 {OUT}/*_detect.png")

    # 环数-偏移对照表（帮助设定瞄准容差）
    print()
    print("=== 横向偏移 → 预估环数（1.54 m 处，靶半径约 41 px）===")
    r = 41.0
    for dx in (0, 4, 8, 12, 16, 20, 24, 28, 33, 41):
        print(f"  dx={dx:3d} px  →  约 {ring_score_from_offset(dx, r):4.1f} 环"
              f"   （每 px 约 {(0.27/ (2*r)) * 1000:.2f} mm 横向）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
