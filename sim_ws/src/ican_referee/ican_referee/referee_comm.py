"""裁判通信模块（E-01）—— 把接口契约（referee_spec）落到 ROS 传输层。

为什么需要独立模块
------------------
原来裁判通信散落在两处：
  - sequential_mission_node：发 /referee/current_target、/referee/finish，
                             订阅 /referee/detections 等话术
  - recognize_node：发 /referee/detections 话术
话题名、QoS、载荷格式各写一遍，改一处容易漏另一处；也没地方集中校验
「我们的接口是否符合官方表 1」。

本模块拆成两层：
  referee_spec.py  —— 纯 Python 的接口契约与载荷校验（可离线自检，不依赖 ROS）
  referee_comm.py  —— ROS 传输层（本文件）：RefereeComm 组件 + QoS

官方依据：判分细则 V1.0 表 1；细则 1.1(3) 要求关键动作前先发布声明，
裁判收到声明后才开始判定 —— 所以声明必须**在动作之前**发出且不能被丢弃
（QoS 用 RELIABLE）。
"""
from __future__ import annotations

import time
from typing import Callable, Optional

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from ican_referee.referee_spec import (  # noqa: F401  （转出便于调用方一处导入）
    REFEREE_INTERFACE,
    RefereePayloadError,
    build_finish_payload,
    build_shoot_payload,
    interface_report,
    validate_detection_payload,
)


def referee_qos(depth: int = 10) -> QoSProfile:
    """裁判话题统一 QoS：RELIABLE + VOLATILE。

    RELIABLE 是必须的 —— 判分细则 1.1(3) 要求声明先到、裁判收到后才判定，
    丢一条声明可能直接导致该点不计分。
    """
    qos = QoSProfile(depth=depth)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.VOLATILE
    return qos


class RefereeComm:
    """把裁判通信封装成一个可复用的组件。

    用法（在任意 Node 的 __init__ 里）：
        self.referee = RefereeComm(self)
        self.referee.declare_shooting(1)        # 发 shoot_1
        self.referee.declare_finish()           # 发 finish:return_start
        self.referee.publish_detection(script)  # 发识别话术
        self.referee.on_script = my_cb          # 收到话术时回调 my_cb(text)

    设计说明：本类**不是** Node，而是挂在调用方 Node 上的组件 ——
    这样每个进程只会有一个某话题的发布者，不会出现「两个节点抢同一话题」的问题
    （这类问题在联调里真实发生过，表现为命令互相覆盖）。
    """

    def __init__(
        self,
        node: Node,
        detections_topic: Optional[str] = None,
        current_target_topic: Optional[str] = None,
        finish_topic: Optional[str] = None,
        shot_topic: Optional[str] = None,
        finish_message: str = "finish:return_start",
        enable_subscribe_detections: bool = True,
        enable_subscribe_shot: bool = False,
        logger_prefix: str = "[裁判]",
    ):
        self.node = node
        self.prefix = logger_prefix
        self.finish_message = build_finish_payload(finish_message)

        def name(key: str, override: Optional[str]) -> str:
            return override or REFEREE_INTERFACE[key]["topic"]

        t_det = name("detections", detections_topic)
        t_tgt = name("current_target", current_target_topic)
        t_fin = name("finish", finish_topic)
        t_shot = name("shot", shot_topic)

        qos = referee_qos()
        self._det_pub = node.create_publisher(String, t_det, qos)
        self._target_pub = node.create_publisher(String, t_tgt, qos)
        self._finish_pub = node.create_publisher(String, t_fin, qos)
        self._topics = {
            "detections": t_det,
            "current_target": t_tgt,
            "finish": t_fin,
            "shot": t_shot,
        }

        # 回调（由调用方赋值）
        self.on_script: Optional[Callable[[str], None]] = None
        self.on_shot: Optional[Callable[[str], None]] = None

        # 统计（供报告与监控节点使用）
        self.count = {"detections": 0, "current_target": 0, "finish": 0,
                      "detections_rx": 0, "shot_rx": 0}
        self.last = {"detections": "", "current_target": "", "finish": "",
                     "detections_rx": "", "shot_rx": ""}
        self.timeline = []          # [(monotonic, kind, payload)]

        if enable_subscribe_detections:
            node.create_subscription(String, t_det, self._handle_detection, qos)
        if enable_subscribe_shot:
            node.create_subscription(String, t_shot, self._handle_shot, qos)

        node.get_logger().info(
            f"{self.prefix} 裁判通信就绪："
            f"detections={t_det} current_target={t_tgt} finish={t_fin} "
            f"shot={t_shot}（QoS RELIABLE）")

    # ---------------- 发送 ----------------
    def _publish(self, pub, kind: str, payload: str):
        msg = String()
        msg.data = payload
        pub.publish(msg)
        self.count[kind] += 1
        self.last[kind] = payload
        self.timeline.append((time.monotonic(), kind, payload))
        self.node.get_logger().info(
            f"{self.prefix} 已发布 {self._topics[kind]} = {payload}")

    def publish_detection(self, script: str):
        """上报识别话术（进入点位前）。载荷是话术原文。"""
        payload = validate_detection_payload(script)
        self._publish(self._det_pub, "detections", payload)

    def declare_shooting(self, point: int):
        """射击前声明当前点位（shoot_N）。按细则 1.1(3) 必须在射击动作之前。"""
        payload = build_shoot_payload(point)
        self._publish(self._target_pub, "current_target", payload)

    def declare_finish(self):
        """返航停车后发布完成声明（只结算一次）。"""
        self._publish(self._finish_pub, "finish", self.finish_message)

    # ---------------- 接收 ----------------
    def _handle_detection(self, msg: String):
        text = (msg.data or "").strip()
        if not text:
            return
        self.count["detections_rx"] += 1
        self.last["detections_rx"] = text
        if self.on_script is not None:
            try:
                self.on_script(text)
            except Exception as exc:  # noqa: BLE001
                self.node.get_logger().warn(f"{self.prefix} 话术回调异常: {exc}")

    def _handle_shot(self, msg: String):
        shot = (msg.data or "").strip()
        self.count["shot_rx"] += 1
        self.last["shot_rx"] = shot
        if self.on_shot is not None:
            try:
                self.on_shot(shot)
            except Exception as exc:  # noqa: BLE001
                self.node.get_logger().warn(f"{self.prefix} 射击回调异常: {exc}")

    # ---------------- 自检 ----------------
    def summary(self) -> str:
        """一行摘要，便于写进任务报告。"""
        return (
            f"裁判通信：话术发送 {self.count['detections']} 条、"
            f"射击声明 {self.count['current_target']} 次、"
            f"完成声明 {self.count['finish']} 次、"
            f"射击上报 {self.count['shot_rx']} 条")
