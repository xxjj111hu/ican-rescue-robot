#!/usr/bin/env python3
"""按 /odom 闭环开到指定坐标（麦轮全向），用于把小车精确停到识别点位前。

用法:
  python3 scripts/goto_pose.py --point 1        # 停到 1 号贴图前方 0.7m，车头对着贴图
  python3 scripts/goto_pose.py --point 3 --dry-run
  python3 scripts/goto_pose.py --x 0 --y 0 --yaw 0
  python3 scripts/goto_pose.py --list

原理：不依赖 Nav2 / AMCL，直接用 /odom 做位置闭环；麦轮可以边走边转，
      到达后打印实际位姿与误差（实测 3cm 内）。
"""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

_WS = Path(__file__).resolve().parents[1]
ASSIGNMENT = "/opt/ican_shoot_sim/share/ican_land_sim/materials/shooting_texture_assignment.json"

# 5 个地面贴图的真实位姿（来自 /opt/ican_shoot_sim/share/ican_land_sim/worlds/land_shoot.world）
GROUND_POINTS = {
    0: (1.115218, -1.356470, 0.0),
    1: (0.030951, -1.359743, 0.0),
    2: (0.027137, -2.700000, 0.0),
    3: (1.120240, -1.913890, 1.5708),
    4: (1.120112, -0.784766, 1.5708),
}

# 预设停车位：站在贴图前方约 0.7m，车头对着贴图
# 已用地图占据栅格校验（0.25m 半径内无占据，全部 0.00）
STANDOFF_POSES = {
    0: (0.415, -1.356, 0.0),
    1: (0.031, -0.660, -1.5708),
    2: (0.027, -2.000, -1.5708),
    3: (0.420, -1.914, 0.0),
    4: (0.420, -0.785, 0.0),
}


def expected_class(point_id):
    try:
        data = json.loads(Path(ASSIGNMENT).read_text(encoding="utf-8"))
    except Exception:
        return None, None
    return (
        data.get("ground_type_assignment", {}).get(f"ShootingGroundTarget{point_id}"),
        data.get("assignment", {}).get(f"ShootingGroundTarget{point_id}"),
    )


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def clamp(v, lim):
    return max(-lim, min(lim, v))


class GoTo(Node):
    def __init__(self):
        super().__init__("goto_pose")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.pose = None
        self.last_rx = 0.0

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        self.last_rx = time.time()

    def stop(self):
        self.pub.publish(Twist())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--point", type=int, choices=[0, 1, 2, 3, 4])
    ap.add_argument("--x", type=float)
    ap.add_argument("--y", type=float)
    ap.add_argument("--yaw", type=float)
    ap.add_argument("--tol-xy", type=float, default=0.04)
    ap.add_argument("--tol-yaw", type=float, default=0.08)
    ap.add_argument("--max-v", type=float, default=0.35)
    ap.add_argument("--max-w", type=float, default=0.9)
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    if args.list:
        print("预设停车位（贴图前方 0.7m，车头对着贴图）：")
        for pid, (x, y, yaw) in STANDOFF_POSES.items():
            cls, img = expected_class(pid)
            print(f"  点位{pid}: ({x:+.3f}, {y:+.3f}) yaw={yaw:+.3f}"
                  f"   期望类别={cls or '?'}  贴图={img or '?'}")
        print("\n地面贴图真实位姿：")
        for pid, (x, y, yaw) in GROUND_POINTS.items():
            print(f"  贴图{pid}: ({x:+.3f}, {y:+.3f}) yaw={yaw:+.3f}")
        return

    if args.point is not None:
        x, y, yaw = STANDOFF_POSES[args.point]
        cls, img = expected_class(args.point)
        tx, ty, _ = GROUND_POINTS[args.point]
        print(f"点位{args.point}: 贴图在 ({tx:+.3f}, {ty:+.3f})，"
              f"期望类别 {cls or '?'}（{img or '?'}）")
        print(f"目标停车位 ({x:+.3f}, {y:+.3f}) yaw={yaw:+.3f}，"
              f"距贴图 {math.hypot(tx - x, ty - y):.2f} m")
    else:
        if args.x is None or args.y is None:
            raise SystemExit("要么用 --point 0~4，要么同时给 --x 和 --y")
        x, y, yaw = args.x, args.y, (args.yaw or 0.0)
        print(f"目标 ({x:+.3f}, {y:+.3f}) yaw={yaw:+.3f}")

    if args.dry_run:
        return

    rclpy.init()
    node = GoTo()
    t0 = time.time()
    while node.pose is None and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.pose is None:
        raise SystemExit("10 秒内没收到 /odom，仿真没跑或话题不对")

    sx, sy, syaw = node.pose
    print(f"起点 ({sx:+.3f}, {sy:+.3f}) yaw={syaw:+.3f}，开始行驶 ...")

    period = 0.05
    t_start = time.time()
    last_print = 0.0
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=period)
            now = time.time()
            if now - node.last_rx > 2.0:
                print("!! 2 秒没收到 /odom，停止")
                break
            px, py, pyaw = node.pose
            dx, dy = x - px, y - py
            dist = math.hypot(dx, dy)
            yaw_err = wrap(yaw - pyaw)

            if dist < args.tol_xy and abs(yaw_err) < args.tol_yaw:
                break
            if now - t_start > args.timeout:
                print(f"!! 超时，剩余 {dist:.3f} m")
                break

            c, s = math.cos(pyaw), math.sin(pyaw)
            ex = c * dx + s * dy       # 机体前向误差
            ey = -s * dx + c * dy      # 机体左向误差
            vx = clamp(1.5 * ex, args.max_v)
            vy = clamp(1.5 * ey, args.max_v)
            wz = clamp(2.0 * yaw_err, args.max_w)

            cmd = Twist()
            cmd.linear.x = vx
            cmd.linear.y = vy
            cmd.angular.z = wz
            node.pub.publish(cmd)

            if now - last_print > 2.0:
                print(f"  ... 剩余 {dist:.2f} m, 朝向误差 {math.degrees(yaw_err):+.0f}°, "
                      f"当前 ({px:+.2f}, {py:+.2f})")
                last_print = now

        for _ in range(10):
            node.pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.03)
        time.sleep(0.3)
        rclpy.spin_once(node, timeout_sec=0.1)
        px, py, pyaw = node.pose
        print(f"到位 ({px:+.3f}, {py:+.3f}) yaw={pyaw:+.3f}  "
              f"位置误差 {math.hypot(x - px, y - py) * 100:.1f} cm，"
              f"朝向误差 {math.degrees(wrap(yaw - pyaw)):+.1f}°")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
