"""定位精度评估节点（任务10 地图与定位精度校验）。

订阅 /amcl_pose（AMCL 定位）与 /odom（Gazebo planar_move 发布的真值位姿），
在指定时长内持续采集两者差值，结束后计算 RMS 与最大误差，判定是否满足
AMCL 定位误差 ≤ 0.1 m。结果打印并可选写入报告文件。

用法：ros2 run ican_nav_tasks localization_eval_node --ros-args -p duration_sec:=60.0
"""
import math
import os

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class LocalizationEvalNode(Node):
    def __init__(self):
        super().__init__("localization_eval_node")
        self.declare_parameter("duration_sec", 60.0)
        self.declare_parameter(
            "report_file",
            os.path.expanduser("~/.ros/ican_nav_tasks/localization_report.txt"),
        )
        self.dur = float(self.get_parameter("duration_sec").value)
        self.report_file = self.get_parameter("report_file").value

        # 与 sequential_mission_node 一致的默认 QoS（RELIABLE/VOLATILE, depth=10），
        # 已验证在该仿真中可稳定收到 /amcl_pose 与 /odom。
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, 10
        )
        # /odom 由 gazebo_ros_planar_move 插件发布，类型为 nav_msgs/msg/Odometry（真值位姿）
        self.create_subscription(
            Odometry, "/odom", self.on_odom, 10
        )

        self.amcl = None
        self.odom = None
        self.dx = []
        self.dy = []
        self.start = self.get_clock().now()
        self.wrote = False
        self.timer = self.create_timer(1.0, self.tick)

        self.get_logger().info(f"定位评估节点就绪，将采集 {self.dur:.0f} 秒。")

    def on_amcl(self, msg):
        self.amcl = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.accum()

    def on_odom(self, msg):
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.accum()

    def accum(self):
        if self.amcl is not None and self.odom is not None:
            self.dx.append(self.amcl[0] - self.odom[0])
            self.dy.append(self.amcl[1] - self.odom[1])

    def tick(self):
        now = self.get_clock().now()
        if (now - self.start).nanoseconds / 1e9 >= self.dur:
            self.write_report()
            rclpy.shutdown()

    def write_report(self):
        if self.wrote:
            return
        self.wrote = True
        n = len(self.dx)
        if n > 0:
            rms = math.sqrt(
                sum(a * a + b * b for a, b in zip(self.dx, self.dy)) / n
            )
            maxe = max(math.hypot(a, b) for a, b in zip(self.dx, self.dy))
        else:
            rms = maxe = 0.0
        ok = maxe <= 0.1
        text = (
            "==== 定位精度评估（AMCL vs 真值/odom）====\n"
            f"采样数={n}\n"
            f"RMS 误差={rms:.4f} m\n"
            f"最大误差={maxe:.4f} m\n"
            f"判定={'PASS(≤0.1m)' if ok else 'CHECK(>0.1m)'}\n"
        )
        self.get_logger().info(text)
        if self.report_file:
            try:
                with open(self.report_file, "w", encoding="utf-8") as f:
                    f.write(text)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"写报告文件失败: {e}")


def main():
    rclpy.init()
    node = LocalizationEvalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.write_report()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
