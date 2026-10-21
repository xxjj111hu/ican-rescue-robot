# ⚠️ 本文件有两份副本，修改时必须同步：
#    sim_ws/src/ican_shoot/ican_shoot/ring_detect.py   （ROS 包内，节点用这份）
#    交付包/射击/脚本/ring_detect.py                    （离线验证脚本用这份）
# 离线自测：python 交付包/射击/脚本/test_ring_detect.py
"""环形靶检测（D-01 视觉部分）—— v2，用色块质心代替轮廓圆拟合。

为什么不用 v1
-------------
v1 用 findContours + minEnclosingCircle + 圆度过滤，实测失败：
红环被白环数字与黑环打断成碎弧，圆形度只有 0.15，被 circ<0.35 过滤掉；
只剩蓝环的碎块，圆心严重偏移（点1 偏差 15 px，半径 12 而非 41）。

v2 思路：**对称环带的质心就是圆心**
  - 红环（6环，最外）：质心 = 靶心；99 分位距离 = 靶外半径
  - 蓝环（8环）、黄心（10环）：同心，用于交叉验证与加权
不做开运算（会打碎细环），只做闭运算填补数字标注造成的缺口。

实测依据（仿真光照偏暗，V 仅 90~130，不能按标准靶纸亮色设阈值）
    黄心 (10环)  H 21~31  S>=112  V>=93   仅约 26 px
    红   (6环)   H <=5    S>=164  V>=107
    蓝   (8环)   H 101~105 S>=141 V>=114
靶尺寸（1.54 m、800×800）：直径约 82 px，每环约 8.2 px
    >=8 环 允许横向误差 <=24.6 px；>=9 环 <=16.4 px；10 环 <=8.2 px
"""
from __future__ import annotations

import math
from pathlib import Path

import cv2
import numpy as np


def imread_unicode(path):
    """读取图片，支持中文路径（cv2.imread 在 Windows 读不了非 ASCII 路径）。"""
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
    except OSError:
        return None
    if data.size == 0:
        return None
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def imwrite_unicode(path, img):
    """保存图片，支持中文路径。"""
    ok, buf = cv2.imencode(Path(str(path)).suffix or ".png", img)
    if not ok:
        return False
    buf.tofile(str(path))
    return True


# 实测 HSV 阈值，留一定容差（仿真光照偏暗）
# 注意 yellow 的 S 必须 >=120：靶板是米色(H≈25~35)，S 只有 60~90，
# 阈值放低会把整块靶板并进黄色掩码，导致质心被拉偏（实测偏 30 px）。
HSV_RANGES = {
    "red": [((0, 110, 45), (9, 255, 255)), ((168, 110, 45), (179, 255, 255))],
    "blue": [((96, 90, 45), (112, 255, 255))],
    "yellow": [((18, 120, 90), (38, 255, 255))],
}
MIN_PIXELS = {"red": 120, "blue": 60, "yellow": 12}
COLOR_WEIGHT = {"yellow": 1.8, "blue": 1.2, "red": 1.0}
OUTER_COLOR = "red"     # 最外环，用来量靶半径
# 同心容差：各色质心相对锚点的最大允许偏差（px）
CONCENTRIC_TOL = 12.0
# 靶半径合理范围（实测 1.54 m 处半径约 41 px；留出靠近/远离的余量）
RADIUS_RANGE = (8.0, 70.0)
# 是否要求检出内环色（蓝/黄）才认为有效。
# 真十环靶必然同时有红环(6环)与蓝环(8环)；实测联调时画面右侧某个纯红物体
# 被误检成靶（r=47、dx=+378px、只有 'red'），而真靶检出的是 ['red','blue']。
REQUIRE_INNER_COLOR = True


def color_mask(hsv: np.ndarray, color: str) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in HSV_RANGES[color]:
        mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    # 只闭运算（填补数字标注的缺口）；开运算会打碎细环，不能用
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))


def blob_stats(mask: np.ndarray, min_pixels: int):
    """返回 (cx, cy, n, r_p99) 或 None。r_p99 用 99 分位距离，抗离群点。"""
    ys, xs = np.nonzero(mask)
    n = xs.size
    if n < min_pixels:
        return None
    cx, cy = float(xs.mean()), float(ys.mean())
    d = np.hypot(xs - cx, ys - cy)
    return cx, cy, int(n), float(np.percentile(d, 99))


def detect_ring(bgr: np.ndarray, roi_y_max_ratio: float = 0.53,
                max_radius: float = RADIUS_RANGE[1]):
    """检测环形靶。

    返回 dict: ok / x / y / dx / dy / radius / conf / parts
    dx>0 表示靶心在画面中心右侧，需要向右（顺时针）转。

    ROI 默认只取上 53%：靶在画面上部，而下方彩色地面贴图会把红/黄掩码
    拉成 r99 达 300~440 px 的巨块，必须排除（实测 0.60H 时全部失效）。
    0.55 这个值不够低 —— 蓝环会被地面污染掉，导致真靶只剩红色检出，
    conf 只有 0.85，且纯红物体也能通过检查。收到 0.53 后三色齐全、conf=1.00。
    """
    h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    roi = np.zeros((h, w), np.uint8)
    roi[: int(h * roi_y_max_ratio), :] = 255

    raw = {}
    for color in ("red", "blue", "yellow"):
        mask = cv2.bitwise_and(color_mask(hsv, color), roi)
        st = blob_stats(mask, MIN_PIXELS[color])
        if st is None:
            continue
        cx, cy, n, r99 = st
        if not (RADIUS_RANGE[0] <= r99 <= max_radius):
            continue
        raw[color] = {"cx": cx, "cy": cy, "n": n, "r": r99}

    if not raw:
        return {"ok": False, "x": None, "y": None, "dx": None, "dy": None,
                "radius": None, "conf": 0.0, "parts": raw}

    # 假阳性过滤：真十环靶必然同时有红环(6环)与蓝环(8环)。
    # 实测联调时画面右侧某个纯红物体被误检成靶（r=47、dx=+378px、只有 'red'），
    # 而真靶检出的是 ['red','blue']。要求至少有一个内环色可显著排除这类误检。
    if REQUIRE_INNER_COLOR and not ({"blue", "yellow"} & set(raw.keys())):
        return {"ok": False, "x": None, "y": None, "dx": None, "dy": None,
                "radius": None, "conf": 0.0, "parts": raw,
                "reject": "只有外环色、缺蓝/黄内环 -> 判为误检"}

    # 以"最外环优先"的顺序选锚点，再只接受与锚点同心的颜色
    anchor = None
    for prefer in (OUTER_COLOR, "blue", "yellow"):
        if prefer in raw:
            anchor = prefer
            break
    a = raw[anchor]
    parts = {anchor: a}
    for color, v in raw.items():
        if color == anchor:
            continue
        if math.hypot(v["cx"] - a["cx"], v["cy"] - a["cy"]) <= CONCENTRIC_TOL:
            parts[color] = v

    pts = [(v["cx"], v["cy"]) for v in parts.values()]
    spread = max((math.hypot(p[0] - q[0], p[1] - q[1])
                  for p in pts for q in pts), default=0.0)

    # 质心加权融合
    ax = ay = aw = 0.0
    for color, v in parts.items():
        wgt = v["n"] * COLOR_WEIGHT.get(color, 1.0)
        ax += v["cx"] * wgt
        ay += v["cy"] * wgt
        aw += wgt
    cx, cy = ax / max(aw, 1e-6), ay / max(aw, 1e-6)

    radius = parts[OUTER_COLOR]["r"] if OUTER_COLOR in parts else max(
        v["r"] for v in parts.values())

    coherence = 1.0 - min(1.0, spread / CONCENTRIC_TOL)
    conf = min(1.0, 0.35 + 0.20 * len(parts) + 0.30 * coherence)

    return {
        "ok": True, "x": cx, "y": cy,
        "dx": cx - w * 0.5, "dy": cy - h * 0.5,
        "radius": float(radius), "conf": float(conf), "parts": parts,
    }


def ring_score_from_offset(dx: float, radius: float) -> float:
    """按横向偏移估算命中环数。靶半径覆盖 6~10 环共 5 个环带，每环 = radius/5。"""
    if not radius:
        return 0.0
    return max(0.0, min(10.0, 10.0 - abs(dx) / (radius / 5.0)))


def draw_debug(bgr: np.ndarray, det: dict) -> np.ndarray:
    out = bgr.copy()
    h, w = out.shape[:2]
    cv2.drawMarker(out, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 22, 2)
    if det.get("ok"):
        cx, cy, r = int(det["x"]), int(det["y"]), int(det["radius"])
        cv2.circle(out, (cx, cy), r, (0, 255, 255), 2)
        cv2.circle(out, (cx, cy), max(1, r // 5), (0, 255, 0), 1)
        cv2.drawMarker(out, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
        cv2.line(out, (w // 2, h // 2), (cx, cy), (0, 255, 0), 1)
        cv2.putText(out, f"dx={det['dx']:+.1f}px r={det['radius']:.0f} conf={det['conf']:.2f}",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        for v in det["parts"].values():
            cv2.drawMarker(out, (int(v["cx"]), int(v["cy"])),
                           (255, 0, 255), cv2.MARKER_DIAMOND, 10, 1)
    else:
        cv2.putText(out, "NO RING TARGET", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
    return out
