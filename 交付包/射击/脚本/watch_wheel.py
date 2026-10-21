"""观察旋转靶是否真的在转，并验证叶片位置公式。

用途
----
target_shooter_node 报「叶片垂直偏移始终 -82mm」，怀疑轮子没转。
本脚本订阅 /gazebo/model_states，每秒打印：
  - 轮子绕 x 轴转角 theta
  - theta 的变化量（判断是否在转）
  - 5 个叶片各自的「垂直偏移」（子弹高度方向），应周期性穿过 0
  - 3 号靶位（leaf_2）与水平位置的角度差

判据
----
- 若 theta 一直不变 -> 轮子静止，时序瞄准无从谈起（要找驱动它的插件/节点）
- 若 theta 在变，但 5 个叶片的垂直偏移都不同时穿过 0 -> 公式有误
- 若 theta 在变且某叶片垂直偏移周期性穿过 0 -> 公式正确，可直接开火

用法：
  source /opt/ros/humble/setup.bash && source ~/ican-repo/sim_ws/install/setup.bash
  python3 ~/ican-repo/交付包/射击/脚本/watch_wheel.py --seconds 12
"""
import argparse
import math
import sys
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

LEAF_R = 0.09
PHI0_DEG = 90.0


def quat_to_roll_x(x, y, z, w):
    n = math.hypot(x, w)
    if n < 1e-9:
        return 0.0
    return 2.0 * math.atan2(x / n, w / n)


class Watcher(Node):
    def __init__(self):
        super().__init__("watch_wheel")
        self.table = {}
        self.count = 0
        self.create_subscription(ModelStates, "/gazebo/model_states",
                                 self.on_states, qos_profile_sensor_data)
        self.create_subscription(ModelStates, "/gazebo/model_states",
                                 self.on_states, 10)

    def on_states(self, msg):
        self.table = dict(zip(msg.name, msg.pose))
        self.count += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--period", type=float, default=1.0)
    args = ap.parse_args()

    rclpy.init()
    node = Watcher()
    t_start = time.monotonic()
    prev_theta = None
    print(f"观察 {args.seconds:.0f} 秒，每 {args.period:.1f} 秒一次\n")
    hdr = f"{'t':>5} {'msgs':>5} {'轮角θ':>9} {'Δθ':>8}"
    for k in range(5):
        hdr += f" {'leaf%d竖直' % k:>10}"
    print(hdr + "   (竖直偏移单位 mm，=0 表示该叶片在水平线上)")
    print("-" * (len(hdr) + 30))

    while time.monotonic() - t_start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.count == 0:
            continue
        now = time.monotonic() - t_start
        pose = node.table.get("target_wheel")
        if pose is None:
            print("  model_states 里没有 target_wheel")
            time.sleep(args.period)
            continue
        q = pose.orientation
        theta = quat_to_roll_x(q.x, q.y, q.z, q.w)
        d_theta = 0.0 if prev_theta is None else math.degrees(theta - prev_theta)
        prev_theta = theta

        line = (f"{now:5.1f} {node.count:5d} {math.degrees(theta) % 360:8.1f}° "
                f"{d_theta:+7.1f}°")
        for k in range(5):
            phi = math.radians(PHI0_DEG - 72.0 * k)
            ly, lz = LEAF_R * math.cos(phi), LEAF_R * math.sin(phi)
            z_off = ly * math.sin(theta) + lz * math.cos(theta)
            line += f" {z_off*1000:+9.0f}"
        print(line)
        time.sleep(args.period)

    print()
    if prev_theta is None:
        print("❌ 没收到任何 model_states 消息（话题名字或 QoS 不对）")
    else:
        print("判读：")
        print("  Δθ 全为 0  -> 轮子静止，需要找驱动它的插件/节点")
        print("  Δθ 非 0 且某列竖直偏移周期性穿过 0 -> 公式正确")
        print(f"  理论转速 0.24166 rad/s = 13.85°/s，若 Δθ/秒 接近此值则转速正常")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
