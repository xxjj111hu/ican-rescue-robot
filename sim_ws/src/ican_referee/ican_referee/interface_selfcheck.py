"""裁判接口自检（E-04 可离线验证部分）—— 不需要启动仿真。

检查三件事：
  1) 载荷构造函数生成的字符串与官方表 1 示例**逐字一致**
  2) 非法载荷会被挡住（避免把错的声明发给裁判）
  3) 模块声明的接口清单与官方表 1 完全对应（话题名、类型、方向）

用法：
  python3 -m ican_referee.interface_selfcheck
  ros2 run ican_referee referee_selfcheck
  或直接： python3 交付包/... 见 README
"""
from __future__ import annotations

import sys

from ican_referee.referee_spec import (
    REFEREE_INTERFACE,
    RefereePayloadError,
    build_finish_payload,
    build_shoot_payload,
    validate_detection_payload,
)

# 官方表 1（判分细则 V1.0）原文对照表 —— 这里逐字写死，作为独立事实来源
OFFICIAL = {
    "current_target": {"topic": "/referee/current_target", "type": "std_msgs/msg/String",
                       "dir": "pub", "example": "shoot_1"},
    "finish": {"topic": "/referee/finish", "type": "std_msgs/msg/String",
               "dir": "pub", "example": "finish:return_start"},
    "detections": {"topic": "/referee/detections", "type": "std_msgs/msg/String",
                   "dir": "pub",
                   "example": "已识别到坍塌废墟，前方道路被废墟阻断，开始清除障碍，打通救援通道。"},
    "shot": {"topic": "/shooter/shot", "type": "std_msgs/msg/String",
             "dir": "sub", "example": "bullet_08ac4d6b7a"},
}


def main() -> int:
    fails = []

    print("=== ① 接口清单与官方表 1 对照 ===")
    for key, off in OFFICIAL.items():
        ours = REFEREE_INTERFACE.get(key)
        if ours is None:
            fails.append(f"缺少接口定义: {key}")
            print(f"  ❌ {key}: 未定义")
            continue
        for field in ("topic", "type", "dir", "example"):
            if ours[field] != off[field]:
                fails.append(f"{key}.{field} 不一致: {ours[field]!r} != {off[field]!r}")
                print(f"  ❌ {key}.{field}: {ours[field]!r} != {off[field]!r}")
        if not fails:
            print(f"  ✅ {key:15s} {ours['topic']:26s} {ours['type']:22s} {ours['dir']}")

    print()
    print("=== ② 载荷构造与官方示例逐字比对 ===")
    cases = [
        ("shoot_1", build_shoot_payload(1), "shoot_1"),
        ("shoot_4", build_shoot_payload(4), "shoot_4"),
        ("finish 默认", build_finish_payload(), "finish:return_start"),
        ("话术原文", validate_detection_payload(
            "已识别到坍塌废墟，前方道路被废墟阻断，开始清除障碍，打通救援通道。"),
         "已识别到坍塌废墟，前方道路被废墟阻断，开始清除障碍，打通救援通道。"),
    ]
    for name, got, want in cases:
        ok = got == want
        if not ok:
            fails.append(f"{name} 载荷不一致: {got!r} != {want!r}")
        print(f"  {'✅' if ok else '❌'} {name:12s} -> {got!r}")

    print()
    print("=== ③ 非法载荷必须被挡住 ===")
    bad_cases = [
        ("点位 0", lambda: build_shoot_payload(0)),
        ("点位 -1", lambda: build_shoot_payload(-1)),
        ("空话术", lambda: validate_detection_payload("   ")),
        ("JSON 话术", lambda: validate_detection_payload('{"class": "x"}')),
        ("完成声明缺前缀", lambda: build_finish_payload("return_start")),
    ]
    for name, fn in bad_cases:
        try:
            v = fn()
            fails.append(f"{name} 未被拦截（返回 {v!r}）")
            print(f"  ❌ {name}: 未拦截 -> {v!r}")
        except RefereePayloadError as exc:
            print(f"  ✅ {name}: 已拦截（{str(exc)[:40]}…）")

    print()
    if fails:
        print(f"❌ 自检失败，{len(fails)} 项问题：")
        for f in fails:
            print(f"   - {f}")
        return 1
    print("✅ 裁判接口自检全部通过（话题名/类型/方向/载荷格式均与官方表 1 一致）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
