"""定位精度评估节点（任务10 地图与定位精度校验）。

订阅 /amcl_pose（AMCL 定位）与 /odom（Gazebo planar_move 发布的真值位姿），
在指定时长内持续采集两者差值，结束后计算 RMS 与最大误差，判定是否满足
AMCL 定位误差 ≤ 0.1 m。结果打印并可选写入报告文件。

QoS 说明（曾因这两处导致本节点静默失效）：
- /odom 由 Gazebo 插件以 SensorDataQoS(BEST_EFFORT) 发布，订阅端必须同样用
  BEST_EFFORT，否则**一条消息都收不到**且不报错。
- /amcl_pose 由 nav2_amcl 以 RELIABLE + TRANSIENT_LOCAL 发布（保留最后一条），
  订阅端若用 VOLATILE 就拿不到保留的历史消息；节点晚启动时会一直等不到位姿。

另外：**采集到 0 个样本时一律判为未通过**。此前实现把 0 样本记为误差 0，
会输出「采样数=0，RMS=0.0000，PASS」，是最危险的假通过。

用法：ros2 run ican_nav_tasks localization_eval_node --ros-args -p duration_sec:=60.0
"""
import math
import os
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)


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

        # /amcl_pose：nav2_amcl 以 RELIABLE + TRANSIENT_LOCAL 发布（保留最后一条）。
        # 订阅端必须同样声明 TRANSIENT_LOCAL，否则节点晚启动时收不到保留消息。
        amcl_qos = QoSProfile(depth=10)
        amcl_qos.reliability = ReliabilityPolicy.RELIABLE
        amcl_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, amcl_qos
        )
        # /odom：gazebo_ros_planar_move 以 SensorDataQoS(BEST_EFFORT) 发布，
        # 订阅端用 RELIABLE 会**完全收不到**（静默失败，不报错）。
        self.create_subscription(
            Odometry, "/odom", self.on_odom, qos_profile_sensor_data
        )

        self.amcl = None
        self.odom = None
        self.dx = []
        self.dy = []
        # 计时用单调墙钟：use_sim_time 下若 /clock 停滞，ROS 定时器不会触发，
        # 本节点就永远不结束、不写报告（与 sequential_mission_node 遇到的是同一类坑）。
        self.start = time.monotonic()
        self.wrote = False
        self._wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.timer = self.create_timer(1.0, self.tick, clock=self._wall_clock)

        self.get_logger().info(f"定位评估节点就绪，将采集 {self.dur:.0f} 秒。")

    def on_amcl(self, msg):
        self.amcl = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        # 只在 AMCL 位姿到达时采样：AMCL 频率低（~10 Hz）、/odom 频率高（~100 Hz），
        # 若在 /odom 回调里也采样，会用「上一次的 AMCL」配「最新的 odom」，
        # 运动中会把车速×时延的偏差算进误差里，虚高 RMS。
        self.accum()

    def on_odom(self, msg):
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def accum(self):
        if self.amcl is not None and self.odom is not None:
            self.dx.append(self.amcl[0] - self.odom[0])
            self.dy.append(self.amcl[1] - self.odom[1])

    def tick(self):
        if time.monotonic() - self.start >= self.dur:
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
        # 关键：没有样本一律不通过。
        # 之前写成 ok = maxe <= 0.1，n=0 时 maxe=0.0 → 输出「采样数=0, RMS=0, PASS」，
        # 是验收工具最危险的假通过（什么都没测却显示达标）。
        ok = (n > 0) and (maxe <= 0.1)
        if n == 0:
            verdict = "FAIL(无采样：未收到 /amcl_pose 或 /odom，请检查 QoS 与节点是否启动)"
        elif ok:
            verdict = "PASS(≤0.1m)"
        else:
            verdict = "CHECK(>0.1m)"
        text = (
            "==== 定位精度评估（AMCL vs 真值/odom）====\n"
            f"采样数={n}\n"
            f"RMS 误差={rms:.4f} m\n"
            f"最大误差={maxe:.4f} m\n"
            f"判定={verdict}\n"
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
    except (KeyboardInterrupt, ExternalShutdownException):
        # tick() 里到点会主动 rclpy.shutdown()，此时 spin 抛 ExternalShutdownException；
        # Ctrl+C 则是 KeyboardInterrupt。两种情况都补写一次报告（write_report 幂等）。
        pass
    finally:
        try:
            node.write_report()
        except Exception:  # noqa: BLE001
            pass
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        # 重复 shutdown 会抛 "rcl_shutdown already called"，这里静默忽略
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
