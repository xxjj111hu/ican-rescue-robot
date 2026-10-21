"""裁判通信监控节点（E-01 配套，E-04 的现场验证工具）。

作用
----
订阅全部裁判相关话题，实时打印一条「裁判交互时间线」：

  [监控] 12.3s  话术上报   /referee/detections      = 已识别到坍塌废墟，…
  [监控] 18.9s  射击声明   /referee/current_target  = shoot_1
  [监控] 24.1s  射击上报   /shooter/shot            = bullet_08ac4d6b7a
  [监控] 95.0s  完成声明   /referee/finish          = finish:return_start

用途：
  1) 录演示视频时作为「体现自主识别」的旁证（能看到话术原文被上报）
  2) 现场核对接口是否真的发出去了（E-04 在无真实裁判端时可做的最强验证）
  3) 事后把时间线导出成 CSV/TXT，作为验收证据

用法：
  ros2 run ican_referee referee_monitor_node
  ros2 launch ican_referee referee_monitor.launch.py
  # 跑完后按 Ctrl-C，时间线会写进 params 里的 log_file
"""
from __future__ import annotations

import json
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from ican_referee.referee_comm import REFEREE_INTERFACE, referee_qos


class RefereeMonitorNode(Node):
    def __init__(self):
        super().__init__("referee_monitor_node")

        self.declare_parameter("log_file",
                               os.path.expanduser("~/.ros/ican_referee/timeline.txt"))
        self.declare_parameter("echo_payload", True)
        self.log_file = str(self.get_parameter("log_file").value)
        self.echo = bool(self.get_parameter("echo_payload").value)

        self.t0 = time.monotonic()
        self.rows = []

        qos = referee_qos()
        for key in ("detections", "current_target", "finish", "shot"):
            spec = REFEREE_INTERFACE[key]
            self.create_subscription(
                String, spec["topic"],
                lambda msg, k=key: self._on_msg(k, msg), qos)

        self._write_header()
        self.get_logger().info(
            "裁判通信监控已启动："
            + "、".join(REFEREE_INTERFACE[k]["topic"] for k in REFEREE_INTERFACE))

    def _write_header(self):
        try:
            parent = os.path.dirname(self.log_file)
            if parent:
                os.makedirs(parent, exist_ok=True)
            with open(self.log_file, "a", encoding="utf-8") as f:
                f.write(f"\n===== 会话 {time.strftime('%Y-%m-%d %H:%M:%S')} =====\n")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"写时间线文件失败: {exc}")

    def _on_msg(self, key, msg):
        t = time.monotonic() - self.t0
        payload = (msg.data or "").strip()
        topic = REFEREE_INTERFACE[key]["topic"]
        label = {
            "detections": "话术上报",
            "current_target": "射击声明",
            "finish": "完成声明",
            "shot": "射击上报",
        }[key]
        line = f"[监控] {t:8.2f}s  {label}  {topic:26s} = {payload}"
        self.rows.append((t, key, payload))
        if self.echo:
            self.get_logger().info(line)
        try:
            with open(self.log_file, "a", encoding="utf-8") as f:
                f.write(f"{t:.2f}\t{key}\t{topic}\t{payload}\n")
        except Exception:  # noqa: BLE001
            pass

    # ---------------- 汇总 ----------------
    def summary(self):
        """打印本次会话的接口一致性小结。"""
        cnt = {}
        for _, key, _ in self.rows:
            cnt[key] = cnt.get(key, 0) + 1
        self.get_logger().info("===== 裁判接口小结 =====")
        for key, spec in REFEREE_INTERFACE.items():
            n = cnt.get(key, 0)
            mark = "✅" if n > 0 else ("—" if key == "shot" else "⚠️")
            self.get_logger().info(
                f"  {mark} {spec['topic']:26s} 收到 {n} 条（示例载荷：{spec['example'][:30]}…）")

        # 逐条做格式核对
        bad = []
        for t, key, payload in self.rows:
            if key == "current_target" and not payload.startswith("shoot_"):
                bad.append((t, key, payload, "应形如 shoot_N"))
            if key == "finish" and not payload.startswith("finish:"):
                bad.append((t, key, payload, "应形如 finish:xxx"))
            if key == "detections" and payload[:1] in "{[":
                bad.append((t, key, payload, "应为话术原文而非 JSON"))
        if bad:
            self.get_logger().warn(f"发现 {len(bad)} 条格式异常：")
            for t, key, payload, why in bad:
                self.get_logger().warn(f"  {t:.2f}s {key} = {payload[:40]!r} —— {why}")
        else:
            self.get_logger().info("  ✅ 全部载荷格式符合官方表 1 示例")

        # 顺序核对：每个点应是「话术 → 射击声明」的顺序
        order_ok = True
        last_det_t = None
        for t, key, _ in self.rows:
            if key == "detections":
                last_det_t = t
            elif key == "current_target":
                if last_det_t is None or last_det_t > t:
                    order_ok = False
        self.get_logger().info(
            "  ✅ 声明顺序正确（识别话术先于射击声明）" if order_ok
            else "  ⚠️ 声明顺序异常：有射击声明早于识别话术")

        self.get_logger().info(f"时间线已写入 {self.log_file}")


def main():
    rclpy.init()
    node = RefereeMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.summary()
        except Exception:  # noqa: BLE001
            pass
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
