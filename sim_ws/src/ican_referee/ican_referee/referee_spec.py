"""裁判接口契约（纯 Python，不依赖 ROS）—— 可在任何环境离线自检。

这一层只描述「接口长什么样」并负责载荷构造/校验，不碰 rclpy。
好处：交付时可在 Windows/无 ROS 环境直接跑自检，确认接口没被改坏。

官方依据：判分细则 V1.0 表 1「各赛项话题说明」（陆地障碍物清除任务）。
"""
from __future__ import annotations

import re
from typing import Dict

# ---- 单一事实来源：官方表 1 的接口定义 ----
REFEREE_INTERFACE: Dict[str, dict] = {
    "detections": {
        "topic": "/referee/detections",
        "type": "std_msgs/msg/String",
        "dir": "pub",
        "purpose": "任务点识别话术上报（进入点位前）",
        "example": "已识别到坍塌废墟，前方道路被废墟阻断，开始清除障碍，打通救援通道。",
    },
    "current_target": {
        "topic": "/referee/current_target",
        "type": "std_msgs/msg/String",
        "dir": "pub",
        "purpose": "任务点射击声明（射击前发布，如 shoot_1）",
        "example": "shoot_1",
    },
    "finish": {
        "topic": "/referee/finish",
        "type": "std_msgs/msg/String",
        "dir": "pub",
        "purpose": "返回指挥中心停车后的完成声明（只结算一次）",
        "example": "finish:return_start",
    },
    "shot": {
        "topic": "/shooter/shot",
        "type": "std_msgs/msg/String",
        "dir": "sub",
        "purpose": "射击命中上报（由射击节点自动发布，主程序不发布）",
        "example": "bullet_08ac4d6b7a",
    },
}

# ---- 载荷格式（与官方示例逐字对齐） ----
RE_SHOOT = re.compile(r"^shoot_[1-9]$")
RE_FINISH = re.compile(r"^finish:\w+$")


class RefereePayloadError(ValueError):
    """载荷不符合官方格式时抛出，避免把错的声明发给裁判。"""


def build_shoot_payload(point: int) -> str:
    """构造射击声明。官方示例为 shoot_1。"""
    payload = f"shoot_{int(point)}"
    if not RE_SHOOT.match(payload):
        raise RefereePayloadError(
            f"射击声明格式非法: {payload!r}（应形如 shoot_1，点位 1~9）")
    return payload


def build_finish_payload(message: str = "finish:return_start") -> str:
    """构造完成声明。官方示例为 finish:return_start。"""
    payload = str(message).strip()
    if not RE_FINISH.match(payload):
        raise RefereePayloadError(
            f"完成声明格式非法: {payload!r}（应形如 finish:return_start）")
    return payload


def validate_detection_payload(script: str) -> str:
    """校验识别话术。官方要求发「图片种类与识别话术」的原文，不是 JSON。"""
    payload = str(script).strip()
    if not payload:
        raise RefereePayloadError("识别话术为空")
    if payload[:1] in "{[":
        raise RefereePayloadError(
            f"识别话术不应是 JSON，应为话术原文: {payload[:40]!r}")
    # 官方要求话术含「图片种类」描述，至少要有一定长度
    if len(payload) < 6:
        raise RefereePayloadError(f"识别话术过短，疑似不是完整话术: {payload!r}")
    return payload


def interface_report() -> str:
    """生成接口一致性报告（E-04 可验证部分）。"""
    lines = ["裁判接口定义（依据判分细则 V1.0 表 1）:", ""]
    for key, spec in REFEREE_INTERFACE.items():
        lines.append(f"  {key}")
        lines.append(f"    话题: {spec['topic']}")
        lines.append(f"    类型: {spec['type']}")
        lines.append(f"    方向: {spec['dir']}")
        lines.append(f"    用途: {spec['purpose']}")
        lines.append(f"    官方示例: {spec['example']}")
        lines.append("")
    return "\n".join(lines)
