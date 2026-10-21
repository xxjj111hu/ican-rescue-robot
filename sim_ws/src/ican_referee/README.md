# ican_referee —— 裁判通信模块（E-01）

> 负责人：胡文静
> 依据：《应急救援机器人挑战赛裁判系统判分细则 V1.0》表 1「各赛项话题说明」

---

## 一、为什么需要这个模块

改造前，裁判通信**散落在两处**，各写一遍话题名、QoS 和载荷：

| 功能 | 原位置 |
|---|---|
| 发 `/referee/current_target` | `sequential_mission_node.publish_referee_target()` |
| 发 `/referee/finish` | `sequential_mission_node.publish_referee_finish()` |
| 订阅 `/referee/detections` | `sequential_mission_node.on_detection()` |
| 发 `/referee/detections` | `recognize_node.det_pub` |

问题：
1. **没有单一事实来源** —— 改一个话题名要改两个包，容易漏
2. **没人校验载荷格式** —— 发错了只能等裁判判错
3. **无法集中自检** —— E-04「接口一致性验证」没有落点

本模块把这三件事收拢：

```
ican_referee/
├── referee_spec.py          纯 Python 接口契约（不依赖 ROS，可离线自检）
│     · REFEREE_INTERFACE     官方表 1 的话题/类型/方向/示例载荷
│     · build_shoot_payload() 构造 shoot_N，并断言格式
│     · build_finish_payload()构造 finish:xxx，并断言格式
│     · validate_detection_payload() 校验话术（必须原文，不能是 JSON）
├── referee_comm.py          ROS 传输层
│     · RefereeComm           挂在任意 Node 上的组件（不是独立 Node）
│     · referee_qos()         统一 RELIABLE + VOLATILE
├── referee_monitor_node.py  独立监控节点（订阅全部裁判话题，打印时间线）
└── interface_selfcheck.py   离线自检（不需要 ROS）
```

---

## 二、接口定义（与官方表 1 逐字一致）

| 键 | 话题 | 类型 | 方向 | 官方示例载荷 |
|---|---|---|---|---|
| `detections` | `/referee/detections` | `std_msgs/msg/String` | 发布 | `已识别到坍塌废墟，前方道路被废墟阻断，开始清除障碍，打通救援通道。` |
| `current_target` | `/referee/current_target` | `std_msgs/msg/String` | 发布 | `shoot_1` |
| `finish` | `/referee/finish` | `std_msgs/msg/String` | 发布 | `finish:return_start` |
| `shot` | `/shooter/shot` | `std_msgs/msg/String` | 订阅 | `bullet_08ac4d6b7a`（由射击节点自动发布，**主程序不发布**） |

**QoS 统一为 RELIABLE + VOLATILE。** 依据细则 1.1(3)：关键动作前须先发布声明，
裁判收到声明后才开始判定 —— 丢一条声明可能直接导致该点不计分。

---

## 三、怎么用

### 3.1 在节点里接入

```python
from ican_referee.referee_comm import RefereeComm

# 在 Node.__init__ 里
self.referee = RefereeComm(self, finish_message="finish:return_start")
self.referee.on_script = self.on_detection_text   # 收到话术的回调（已解包）

# 发识别话术（进入点位前）
self.referee.publish_detection("已识别到坍塌废墟，……")

# 射击前声明点位
self.referee.declare_shooting(1)      # → shoot_1

# 返航停车后完成声明
self.referee.declare_finish()         # → finish:return_start
```

**为什么是组件而不是独立 Node**：每个进程只会有一个某话题的发布者。
联调里真实出现过「两个节点抢同一话题导致命令互相覆盖」，组件形式从结构上避免这个问题。

### 3.2 离线自检（不需要 ROS）

```bash
cd ~/ican-repo/sim_ws/src/ican_referee
PYTHONPATH=. python3 -m ican_referee.interface_selfcheck
```

或编译后：

```bash
ros2 run ican_referee referee_selfcheck
```

输出示例：

```
=== ① 接口清单与官方表 1 对照 ===
  ✅ current_target  /referee/current_target    std_msgs/msg/String    pub
  ✅ finish          /referee/finish            std_msgs/msg/String    pub
  ✅ detections      /referee/detections        std_msgs/msg/String    pub
  ✅ shot            /shooter/shot              std_msgs/msg/String    sub

=== ② 载荷构造与官方示例逐字比对 ===
  ✅ shoot_1      -> 'shoot_1'
  ✅ finish 默认   -> 'finish:return_start'
  ...

=== ③ 非法载荷必须被挡住 ===
  ✅ 点位 0: 已拦截（应形如 shoot_1，点位 1~9）
  ✅ JSON 话术: 已拦截（识别话术不应是 JSON，应为话术原文）
  ...
✅ 裁判接口自检全部通过
```

### 3.3 现场监控（E-04 可做的最强验证）

```bash
ros2 launch ican_referee referee_monitor.launch.py
```

会实时打印：

```
[监控]    12.34s  话术上报  /referee/detections     = 已识别到坍塌废墟，……
[监控]    18.90s  射击声明  /referee/current_target = shoot_1
[监控]    24.11s  射击上报  /shooter/shot           = bullet_08ac4d6b7a
[监控]    95.02s  完成声明  /referee/finish         = finish:return_start
```

Ctrl-C 退出时打印**本场小结**：

- 每个话题收到多少条
- 载荷格式是否符合官方表 1
- **声明顺序是否正确**（识别话术必须先于射击声明，依据细则 1.1(3)）

时间线同时追加写入 `~/.ros/ican_referee/timeline.txt`，可直接作为验收证据。

---

## 四、E-04「接口一致性验证」的结论边界

| 项 | 能否验证 | 结论 |
|---|---|---|
| 话题名与官方表 1 一致 | ✅ 离线自检 | 4/4 一致 |
| 消息类型一致 | ✅ 离线自检 | 全部 `std_msgs/msg/String` |
| 载荷格式与示例逐字一致 | ✅ 离线自检 | `shoot_N` / `finish:return_start` / 话术原文 |
| 非法载荷被拦截 | ✅ 离线自检 | 5 类非法输入全部拦截 |
| 声明顺序（话术先于射击声明） | ✅ 监控节点 | 实际跑一轮即可核对 |
| **裁判端能否正确解析** | ❌ **无法验证** | 练习环境没有真实裁判系统 |

**结论：能离线验证的全部验证了；裁判端解析只能在正式比赛现场确认。**
这是环境边界，不是实现缺陷。

---

## 五、编译与验证

```bash
cd ~/ican-repo/sim_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ican_referee ican_recognize ican_nav_tasks --symlink-install
source install/setup.bash

# 离线自检
ros2 run ican_referee referee_selfcheck

# 监控（另开终端，跑任务前启动）
ros2 launch ican_referee referee_monitor.launch.py
```
