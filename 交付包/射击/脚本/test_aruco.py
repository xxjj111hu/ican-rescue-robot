"""探测靶面上的 AR 码：用 OpenCV 各内置字典尝试解码。

目的：判断 D-03/D-04 的"指定靶位"能否直接用 cv2.aruco 解码。
      如果某个字典能解出 id，方案就很简单；如果解不出（码太小 17px / 非标准族），
      就要换思路（模板匹配或轮转时序推算）。

用法：python3 test_aruco.py
"""
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ring_detect import imread_unicode  # noqa: E402

REF = Path("交付包/射击/参考图")

# 候选字典（cv2 5.x 的名称；不存在会跳过）
DICT_NAMES = [
    "DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_4X4_1000",
    "DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250", "DICT_5X5_1000",
    "DICT_6X6_50", "DICT_6X6_250", "DICT_7X7_50",
    "DICT_ARUCO_ORIGINAL",
    "DICT_APRILTAG_16h5", "DICT_APRILTAG_25h9",
    "DICT_APRILTAG_36h10", "DICT_APRILTAG_36h11",
]


def get_dict(name):
    if not hasattr(cv2, "aruco"):
        return None
    aruco = cv2.aruco
    if not hasattr(aruco, name):
        return None
    return aruco.getPredefinedDictionary(getattr(aruco, name))


def try_decode(bgr, scale, dict_name):
    d = get_dict(dict_name)
    if d is None:
        return None
    if scale != 1.0:
        img = cv2.resize(bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    else:
        img = bgr
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    try:
        params = cv2.aruco.DetectorParameters()
    except AttributeError:
        params = cv2.aruco.DetectorParameters_create()
    # 码很小，放宽检测阈值
    for attr, val in (("minMarkerPerimeterRate", 0.01),
                      ("maxMarkerPerimeterRate", 0.6),
                      ("adaptiveThreshWinSizeMin", 3),
                      ("adaptiveThreshWinSizeMax", 41),
                      ("adaptiveThreshWinSizeStep", 4),
                      ("polygonalApproxAccuracyRate", 0.06)):
        if hasattr(params, attr):
            setattr(params, attr, val)
    try:
        detector = cv2.aruco.ArucoDetector(d, params)
        corners, ids, _ = detector.detectMarkers(gray)
    except AttributeError:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, d, parameters=params)
    if ids is None or len(ids) == 0:
        return None
    return [int(i) for i in ids.flatten()], len(corners), img


def main():
    if not hasattr(cv2, "aruco"):
        print("❌ 当前 OpenCV 没有 aruco 模块（需要 opencv-contrib-python）")
        return 1

    files = ["point2_wheel_00.png", "point3_moving3_00.png", "point4_moving4_00.png"]
    scales = [1.0, 4.0, 8.0, 12.0]

    any_hit = False
    for fn in files:
        p = REF / fn
        bgr = imread_unicode(p)
        if bgr is None:
            print(f"{fn}: 读不到")
            continue
        print(f"\n=== {fn} ({bgr.shape[1]}x{bgr.shape[0]}) ===")
        for scale in scales:
            hit_here = []
            for name in DICT_NAMES:
                r = try_decode(bgr, scale, name)
                if r:
                    ids, n, img = r
                    print(f"  x{scale:<4} {name:<22} 检到 {n} 个码  ids={ids}")
                    hit_here.append(name)
                    any_hit = True
            if not hit_here:
                print(f"  x{scale:<4} 所有字典均未检出")

    print()
    if any_hit:
        print("✅ 有字典能解码 —— D-03/D-04 可直接用 cv2.aruco")
    else:
        print("❌ 所有字典都解不出")
        print("   可能原因：码太小（约 17px）、非标准族、或渲染模糊")
        print("   备选方案：① 放大 ROI 后再试  ② 模板匹配  ③ 利用轮转时序推算靶位")
    return 0


if __name__ == "__main__":
    sys.exit(main())
