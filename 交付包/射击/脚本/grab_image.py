#!/usr/bin/env python3
"""抓取一帧 /camera/image 存成 png（射击调试用）。

用途：让设计瞄准算法的人真正"看到"靶子长什么样，而不是靠猜。
      靶位识别（D-03/D-04）用 AR 还是模板匹配，取决于靶面实际画了什么。

用法：
    # 保存当前相机画面
    python3 grab_image.py --out /tmp/point1.png

    # 连拍 10 帧（看旋转靶转动/移动靶往复）
    python3 grab_image.py --out /tmp/wheel --count 10 --interval 0.5

    # 指定话题与超时
    python3 grab_image.py --topic /camera/image --timeout 10
"""
import argparse
import os
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Grabber(Node):
    def __init__(self, topic):
        super().__init__("grab_image")
        self.bridge = CvBridge()
        self.latest = None
        self.count = 0
        self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)

    def on_image(self, msg):
        try:
            self.latest = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.count += 1
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"图像转换失败: {exc}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, help="输出路径；--count>1 时作为前缀")
    ap.add_argument("--topic", default="/camera/image")
    ap.add_argument("--count", type=int, default=1)
    ap.add_argument("--interval", type=float, default=0.5)
    ap.add_argument("--timeout", type=float, default=10.0)
    args = ap.parse_args()

    rclpy.init()
    node = Grabber(args.topic)

    # 等第一帧
    t0 = time.monotonic()
    while node.latest is None and time.monotonic() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.latest is None:
        print(f"超时：{args.timeout:.0f}s 内没收到 {args.topic} 的图像")
        print("排查：仿真是否启动、相机是否出图、话题名是否正确")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    saved = []
    if args.count <= 1:
        out = args.out
        parent = os.path.dirname(os.path.abspath(out))
        os.makedirs(parent, exist_ok=True)
        cv2.imwrite(out, node.latest)
        saved.append(out)
    else:
        prefix = args.out
        parent = os.path.dirname(os.path.abspath(prefix))
        os.makedirs(parent, exist_ok=True)
        for i in range(args.count):
            rclpy.spin_once(node, timeout_sec=args.interval)
            path = f"{prefix}_{i:02d}.png"
            cv2.imwrite(path, node.latest)
            saved.append(path)

    h, w = node.latest.shape[:2]
    print(f"已保存 {len(saved)} 张，分辨率 {w}x{h}，累计收到 {node.count} 帧")
    for p in saved:
        print(f"  {p}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
