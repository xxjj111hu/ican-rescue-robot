"""速度监控节点（任务12 限速验证记录）。

订阅最终下发到小车的线速度话题（默认 /cmd_vel），实时记录合成线速度的最大值，
一旦超过 hard limit（默认 0.5 m/s）即报警。运行结束（或被中断）时输出验证结论，
可选写入报告文件。本节点为“只读监控”，不修改任何控制量，可安全长期挂接。
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SpeedMonitorNode(Node):
    def __init__(self):
        super().__init__("speed_monitor_node")
        self.declare_parameter("topic", "/cmd_vel")
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("summary_period_sec", 30.0)
        self.declare_parameter("report_file", "")

        self.topic = self.get_parameter("topic").value
        self.max_lin = float(self.get_parameter("max_linear_speed").value)
        self.summary_period = float(self.get_parameter("summary_period_sec").value)
        self.report_file = self.get_parameter("report_file").value

        self.sub = self.create_subscription(Twist, self.topic, self.on_cmd, 10)
        self.max_seen = 0.0
        self.violations = 0
        self.samples = 0
        self.start = self.get_clock().now()
        self.last_warn = self.get_clock().now()
        self.last_summary = self.get_clock().now()

        self.get_logger().info(
            f"速度监控已启动：话题={self.topic}, 硬上限={self.max_lin} m/s"
        )

    def on_cmd(self, msg):
        lin = (msg.linear.x ** 2 + msg.linear.y ** 2) ** 0.5
        self.samples += 1
        if lin > self.max_seen:
            self.max_seen = lin
        if lin > self.max_lin + 1e-6:
            self.violations += 1
            now = self.get_clock().now()
            if (now - self.last_warn).nanoseconds > 1e9:
                self.get_logger().error(
                    f"速度超限！合成线速度={lin:.4f} > {self.max_lin} m/s"
                )
                self.last_warn = now
        # 周期性小结
        now = self.get_clock().now()
        if (now - self.last_summary).nanoseconds / 1e9 >= self.summary_period:
            self.log_summary(prefix="[周期]")
            self.last_summary = now

    def log_summary(self, prefix=""):
        dur = (self.get_clock().now() - self.start).nanoseconds / 1e9
        ok = self.violations == 0 and self.max_seen <= self.max_lin + 1e-6
        self.get_logger().info(
            f"{prefix} 时长={dur:.1f}s 采样={self.samples} "
            f"最大线速度={self.max_seen:.4f} m/s 超限次数={self.violations} "
            f"结论={'合规' if ok else '不合规'}"
        )
        # 周期性把结论写进报告文件，避免依赖进程关闭才能落盘
        if self.report_file:
            try:
                with open(self.report_file, "w", encoding="utf-8") as f:
                    f.write(
                        f"[速度监控报告] 时长={dur:.1f}s 采样={self.samples} "
                        f"最大合成线速度={self.max_seen:.4f} m/s 硬上限={self.max_lin} "
                        f"超限次数={self.violations} -> "
                        f"{'PASS(全程≤0.5m/s)' if ok else 'FAIL'}\n"
                    )
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"写报告文件失败: {e}")

    def finalize(self):
        dur = (self.get_clock().now() - self.start).nanoseconds / 1e9
        ok = self.violations == 0 and self.max_seen <= self.max_lin + 1e-6
        summary = (
            f"[速度监控最终报告] 时长={dur:.1f}s 采样={self.samples} "
            f"最大合成线速度={self.max_seen:.4f} m/s 硬上限={self.max_lin} "
            f"超限次数={self.violations} -> {'PASS(全程≤0.5m/s)' if ok else 'FAIL'}"
        )
        self.get_logger().info(summary)
        if self.report_file:
            try:
                with open(self.report_file, "w", encoding="utf-8") as f:
                    f.write(summary + "\n")
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"无法写入报告文件: {e}")


def main():
    rclpy.init()
    node = SpeedMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:  # noqa: BLE001
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            node.finalize()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
