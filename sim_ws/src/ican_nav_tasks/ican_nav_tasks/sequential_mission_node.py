"""顺序导航 + 识别等待 + 射击点声明 + 返航停车 状态机（任务13 / 任务14 / 集成）。

每个任务点走「两段导航」：

    ① 导航到 standoff（识别位：贴图前方约 0.70 m，车头朝贴图）
       → 停稳，等待识别话术（订阅 /referee/detections）
       → 最多等 recognize_wait_sec 秒，收不到也继续（不能卡死全程）
    ② 导航到 point（进入位：贴图中心 = 任务点本身）
       → 发 /referee/current_target = shoot_N（判分要求，射击前必须声明）
       → 若 enable_shooting=true 则调 /shooter/shoot
    重复 4 个点 → 返回指挥中心 → 停车微调 → 稳定刹停
    → 发 /referee/finish = finish:return_start
    → 输出结果报告

为什么必须两段：任务说明书要求「到点前识别发话术 → 进入点位 → 发 shoot_N」；
判分细则 §（5）还规定「未完全进入任务点内」会影响得分。只停在 0.7 m 处不进入
点位，到达分可能拿不到；而停在贴图上又会让相机拍不到地面图、识别必然失败。

同时采集定位误差（AMCL 位姿 vs /odom 真值），作为《地图与定位精度校验》的输入数据。

本节点向 /mission/event 发布事件：reached:<label> / shoot:<n> / returned:base / parked。
"""
import json
import math
import os
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def yaw_of(q):
    s = 2.0 * (q.w * q.z + q.x * q.y)
    c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(s, c)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class SequentialMissionNode(Node):
    def __init__(self):
        super().__init__("sequential_mission_node")
        self.declare_parameter("start_delay_sec", 20.0)
        # 超过 start_delay + 本值仍收不到 AMCL 位姿，就用 /odom 真值顶上（防死锁）
        self.declare_parameter("amcl_fallback_sec", 20.0)
        self.declare_parameter("nav_timeout_sec", 120.0)
        # ---- 停滞检测（对应官网比赛流程「机器人超过 30s 状态未发生变化，裁判可停赛」）----
        # 导航中若剩余距离在规定时间内没有有效改善，就判定卡住并主动自救，
        # 避免一直等到 nav_timeout_sec（120s）——那远超裁判的 30s 容忍。
        self.declare_parameter("stall_check_sec", 15.0)
        self.declare_parameter("stall_improve_m", 0.05)
        self.declare_parameter("recover_sec", 6.0)
        self.declare_parameter("max_stall_recover", 3)
        self.declare_parameter("goal_retry_sec", 2.0)
        # 进入位（任务点本身，贴图中心）：[x, y, yaw] * 4
        self.declare_parameter(
            "waypoints",
            [
                0.031, -1.360, 0.0,
                0.027, -2.700, 0.0,
                1.120, -1.914, math.pi / 2.0,
                1.120, -0.785, math.pi / 2.0,
            ],
        )
        # 识别位（贴图前方约 0.70 m，车头朝贴图）：[x, y, yaw] * 4
        # 来自识别模块 goto_pose.py 的 STANDOFF_POSES（已用地图占据栅格校验 0.25m 内无障碍）
        # 留空则跳过「先到识别位」这一步，退化为原来的单段导航。
        self.declare_parameter(
            "standoff_waypoints",
            [
                0.031, -0.660, -math.pi / 2.0,
                0.027, -2.000, -math.pi / 2.0,
                0.420, -1.914, 0.0,
                0.420, -0.785, 0.0,
            ],
        )
        # 识别等待：停稳后最多等这么久的话术；收不到也继续（保证全程不中断）
        self.declare_parameter("recognize_wait_sec", 8.0)
        self.declare_parameter("detections_topic", "/referee/detections")
        self.declare_parameter("recognize_status_topic", "/ican_recognize/status")
        self.declare_parameter("referee_target_topic", "/referee/current_target")
        self.declare_parameter("referee_finish_topic", "/referee/finish")
        self.declare_parameter("finish_message", "finish:return_start")
        self.declare_parameter("base_xyyaw", [0.0, 0.0, 0.0])
        self.declare_parameter("return_to_base", True)
        self.declare_parameter("xy_tol", 0.15)
        self.declare_parameter("yaw_tol", 0.12)
        self.declare_parameter("park_tol", 0.15)
        self.declare_parameter("settle_sec", 2.0)
        self.declare_parameter("nudge_speed", 0.1)
        self.declare_parameter("nudge_max_sec", 12.0)
        self.declare_parameter("enable_shooting", False)
        self.declare_parameter("fire_service", "/shooter/shoot")
        # 分点狙击服务（D-05）：按任务点顺序调用对应的瞄准射击服务
        self.declare_parameter("shoot_ring_service", "/ican_shoot/fire_ring")
        self.declare_parameter("shoot_wheel_service", "/ican_shoot/fire_wheel")
        self.declare_parameter("shoot_moving3_service", "/ican_shoot/fire_moving_3")
        self.declare_parameter("shoot_moving4_service", "/ican_shoot/fire_moving_4")
        # 单个点的射击预算：射击节点内部瞄准最长 20 s，再加服务与开火开销
        self.declare_parameter("shoot_timeout_sec", 30.0)
        # 进任务点后先用 odom 闭环把车头对准到航点 yaw，再交给射击节点
        self.declare_parameter("align_yaw_deg", 0.6)
        self.declare_parameter("align_yaw_timeout_sec", 10.0)
        self.declare_parameter("event_topic", "/mission/event")
        self.declare_parameter("navigate_action", "/navigate_to_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter(
            "result_file",
            os.path.expanduser("~/.ros/ican_nav_tasks/mission_result.txt"),
        )

        wp = self.get_parameter("waypoints").value
        self.waypoints = [
            (float(wp[i]), float(wp[i + 1]), float(wp[i + 2]))
            for i in range(0, len(wp), 3)
        ]
        so = self.get_parameter("standoff_waypoints").value
        self.standoffs = [
            (float(so[i]), float(so[i + 1]), float(so[i + 2]))
            for i in range(0, len(so), 3)
        ]
        self.recognize_wait = float(self.get_parameter("recognize_wait_sec").value)
        self.amcl_fallback_sec = float(self.get_parameter("amcl_fallback_sec").value)
        self.base = [float(x) for x in self.get_parameter("base_xyyaw").value]
        self.return_to_base = bool(self.get_parameter("return_to_base").value)
        self.xy_tol = float(self.get_parameter("xy_tol").value)
        self.yaw_tol = float(self.get_parameter("yaw_tol").value)
        self.park_tol = float(self.get_parameter("park_tol").value)
        self.settle_sec = float(self.get_parameter("settle_sec").value)
        self.nudge_speed = float(self.get_parameter("nudge_speed").value)
        self.nudge_max_sec = float(self.get_parameter("nudge_max_sec").value)
        self.enable_shooting = bool(self.get_parameter("enable_shooting").value)
        self.shoot_timeout = float(self.get_parameter("shoot_timeout_sec").value)
        self.align_yaw_tol = math.radians(
            float(self.get_parameter("align_yaw_deg").value))
        self.align_yaw_timeout = float(
            self.get_parameter("align_yaw_timeout_sec").value)
        self.result_file = self.get_parameter("result_file").value

        # 接口
        self.nav_client = ActionClient(
            self, NavigateToPose, str(self.get_parameter("navigate_action").value)
        )
        self.fire_client = self.create_client(
            Trigger, str(self.get_parameter("fire_service").value)
        )
        # 分点狙击客户端：①环形靶 ②旋转靶 ③移动靶3 ④移动靶4
        self.shoot_clients = {
            1: self.create_client(
                Trigger, str(self.get_parameter("shoot_ring_service").value)),
            2: self.create_client(
                Trigger, str(self.get_parameter("shoot_wheel_service").value)),
            3: self.create_client(
                Trigger, str(self.get_parameter("shoot_moving3_service").value)),
            4: self.create_client(
                Trigger, str(self.get_parameter("shoot_moving4_service").value)),
        }
        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        event_qos = QoSProfile(depth=10)
        event_qos.reliability = ReliabilityPolicy.RELIABLE
        event_qos.durability = DurabilityPolicy.VOLATILE
        self.event_pub = self.create_publisher(
            String, str(self.get_parameter("event_topic").value), event_qos
        )
        # 裁判系统话题（判分要求，QoS RELIABLE）
        self.target_pub = self.create_publisher(
            String, str(self.get_parameter("referee_target_topic").value), event_qos
        )
        self.finish_pub = self.create_publisher(
            String, str(self.get_parameter("referee_finish_topic").value), event_qos
        )
        # 订阅识别话术与识别状态，用于「停在识别位后等话术」
        self.got_script = False
        self.last_script = None
        self.recog_state = None
        self.create_subscription(
            String,
            str(self.get_parameter("detections_topic").value),
            self.on_detection,
            event_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("recognize_status_topic").value),
            self.on_recog_status,
            event_qos,
        )
        # /amcl_pose 由 nav2_amcl 以 RELIABLE + TRANSIENT_LOCAL 发布（保留最后一条）。
        # 订阅端**必须也用 TRANSIENT_LOCAL**，否则 VOLATILE 订阅者只能收到订阅之后
        # 新发布的消息，拿不到保留的那条 —— 而 AMCL 在机器人静止时不会重新发布位姿，
        # 于是本节点会永远等不到 AMCL，形成"任务等定位、定位等移动"的死锁。
        amcl_qos = QoSProfile(depth=10)
        amcl_qos.reliability = ReliabilityPolicy.RELIABLE
        amcl_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, amcl_qos
        )
        # /odom 由 gazebo_ros_planar_move 插件发布（真值位姿）。
        # Gazebo 插件通常用 SensorDataQoS(BEST_EFFORT) 发布，若订阅端用默认的
        # RELIABLE 会**完全收不到**（且不报错），表现就是 odom 一直是 None。
        # 用 qos_profile_sensor_data(BEST_EFFORT) 可同时兼容 BEST_EFFORT 与 RELIABLE 发布端。
        self.create_subscription(
            Odometry, "/odom", self.on_odom, qos_profile_sensor_data
        )

        # 状态
        self.state = "WAITING"
        # 分点狙击（D-05）运行时状态
        self.shoot_future = None
        self.shoot_deadline = 0.0
        self.shoot_point = 0
        self.shoot_results = []
        self.align_t0 = 0.0
        self.align_target_yaw = 0.0
        self.align_point = 0
        # 状态机内部计时一律用"单调墙钟"，不用仿真时钟。
        # 原因：use_sim_time=true 时 rclpy 的定时器由 /clock 驱动，一旦节点没收到
        # /clock 或仿真时钟停滞，定时器就永不触发，状态机静默停摆（节点活着但
        # 一条日志都不打）。而"等 20 秒""导航超时 120 秒"本质是真实时间语义。
        # 消息时间戳（goal pose）仍按 Nav2 的期望处理，见 send_goal。
        self.boot = time.monotonic()
        self.start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self.nav_timeout_sec = float(self.get_parameter("nav_timeout_sec").value)
        self.stall_check_sec = float(self.get_parameter("stall_check_sec").value)
        self.stall_improve_m = float(self.get_parameter("stall_improve_m").value)
        self.recover_sec = float(self.get_parameter("recover_sec").value)
        self.max_stall_recover = int(self.get_parameter("max_stall_recover").value)
        # 停滞检测状态：记录"见过的最好剩余距离"与"该记录的时间"
        self._stall_best = None
        self._stall_t = 0.0
        self.recover_start = None
        self._recover_count = 0
        self.wp_index = 0
        self.nav_goal_handle = None
        self.nav_result_future = None
        self.nav_start = None
        self.current_target = None  # (x, y, yaw, label)
        self.goal_retries = 0
        self.max_goal_retries = 3
        self.retry_timer = None
        self.amcl = None  # {"x","y","yaw"}
        self.odom = None
        self.nudge_start = None
        self.settle_start = None
        self.recog_deadline = None
        self.loc_dx = []
        self.loc_dy = []
        self.wp_errors = []  # (label, err) AMCL vs 目标坐标
        self.loc_at_arrival = []  # (label, err) 到点瞬间 AMCL vs 真值/odom（静态）
        self.last_progress = None  # 进度日志时间戳

        # 5 Hz state machine tick (was 10 Hz) to reduce CPU on the VM
        # 状态机驱动：优先用墙钟定时的 create_timer（clock=SYSTEM_TIME），
        # 若该 rclpy 版本不支持 clock 参数，则退回独立线程驱动 tick。
        # 目的：**绝不让状态机的推进依赖仿真时钟 /clock**。
        # 注意 rclpy 没有 create_wall_timer（那是 rclcpp 的 API）。
        self._wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.timer = None
        self._timer_thread = None
        try:
            self.timer = self.create_timer(0.2, self.tick, clock=self._wall_clock)
            self._timer_mode = "墙钟定时器"
        except TypeError:
            self._start_tick_thread()

        self.get_logger().info(
            f"顺序导航节点就绪：{len(self.waypoints)} 个任务点，"
            f"返航={'是' if self.return_to_base else '否'}。"
            f"任务点坐标={self.waypoints}"
        )
        self.get_logger().info(
            f"[状态机] 驱动方式={self._timer_mode}；"
            f"识别位={self.standoffs if self.standoffs else '未配置（单段导航）'}"
        )

    def _start_tick_thread(self):
        """兜底：用独立守护线程按固定间隔调用 tick，彻底避开 ROS 时钟。"""
        self._timer_mode = "独立线程（rclpy 不支持 clock 参数）"

        def _loop():
            while rclpy.ok():
                try:
                    self.tick()
                except Exception as exc:  # noqa: BLE001
                    try:
                        self.get_logger().error(f"[状态机] tick 异常: {exc}")
                    except Exception:  # noqa: BLE001
                        pass
                time.sleep(0.2)

        self._timer_thread = threading.Thread(target=_loop, daemon=True)
        self._timer_thread.start()

    # ---------------- 订阅回调 ----------------
    def on_amcl(self, msg):
        p = msg.pose.pose
        self.amcl = {
            "x": p.position.x,
            "y": p.position.y,
            "yaw": yaw_of(p.orientation),
        }
        if self.odom is not None:
            self.loc_dx.append(self.amcl["x"] - self.odom["x"])
            self.loc_dy.append(self.amcl["y"] - self.odom["y"])

    def on_odom(self, msg):
        p = msg.pose.pose
        self.odom = {
            "x": p.position.x,
            "y": p.position.y,
            "yaw": yaw_of(p.orientation),
        }

    # ---------------- 识别回调 ----------------
    def on_detection(self, msg):
        """收到识别话术（说明识别模块已经发布过结果）。"""
        text = msg.data.strip()
        if not text:
            return
        if not self.got_script:
            self.get_logger().info(f"[识别] 收到话术：{text}")
        self.got_script = True
        self.last_script = text

    def on_recog_status(self, msg):
        """识别模块状态：idle/in_window/published/forced/timeout。"""
        try:
            data = json.loads(msg.data)
        except Exception:  # noqa: BLE001
            return
        state = data.get("state")
        self.recog_state = state
        # timeout / forced 都算「本次识别已有结论」，不必再等
        if state in ("published", "forced", "timeout"):
            if not self.got_script:
                self.get_logger().info(f"[识别] 状态={state}（点位 {data.get('point')}）")
            self.got_script = True

    def publish_referee_target(self, index):
        """射击前声明当前点位：shoot_1 … shoot_4（判分硬要求）。"""
        m = String()
        m.data = f"shoot_{index}"
        self.target_pub.publish(m)
        self.publish_event(f"shoot:{index}")
        self.get_logger().info(f"[裁判] 已发布 /referee/current_target = {m.data}")

    def publish_referee_finish(self):
        """返航停车完成后发布完成声明。"""
        m = String()
        m.data = str(self.get_parameter("finish_message").value)
        self.finish_pub.publish(m)
        self.get_logger().info(f"[裁判] 已发布 /referee/finish = {m.data}")

    def publish_event(self, text):
        m = String()
        m.data = text
        self.event_pub.publish(m)
        self.get_logger().info(f"[event] {text}")

    def stop_robot(self):
        try:
            self.cmd_pub.publish(Twist())
        except Exception:  # noqa: BLE001
            pass

    # ---------------- 导航 ----------------
    def send_goal(self, x, y, yaw, label):
        if not self.nav_client.wait_for_server(timeout_sec=0.3):
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        # 不设置 stamp（保持 0）：Nav2 会取"最新可用变换"，避免因时间基准不一致
        # 触发 "Extrapolation Error ... would require extrapolation into the past"。
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qx, qy, qz, qw = quat_from_yaw(float(yaw))
        goal.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        self.current_target = (float(x), float(y), float(yaw), label)
        self.nav_goal_handle = None
        self.nav_result_future = None
        self.nav_start = time.monotonic()
        self.state = "NAVIGATING"
        self.get_logger().info(
            f"发送导航目标 {label}: ({x:.3f}, {y:.3f}, {yaw:.2f})"
        )
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(self.on_goal_response)
        return True

    def on_goal_response(self, future):
        self.nav_goal_handle = future.result()
        if not self.nav_goal_handle.accepted:
            self.goal_retries += 1
            if self.goal_retries < self.max_goal_retries:
                self.get_logger().warn(
                    f"Nav2 拒绝导航目标({self.current_target[3]})，"
                    f"{self.goal_retries}/{self.max_goal_retries} 延时重试。"
                )
                self.state = "NAVIGATING"
                self.nav_start = time.monotonic()
                delay = float(self.get_parameter("goal_retry_sec").value)
                self.retry_timer = self.create_timer(
                    delay, self.resend_current_goal, clock=self._wall_clock
                )
                return
            self.get_logger().error(
                f"Nav2 多次拒绝导航目标({self.current_target[3]})，按到达处理继续。"
            )
            self.on_arrived()
            return
        self.get_logger().info("Nav2 已接受导航目标。")
        self.nav_result_future = self.nav_goal_handle.get_result_async()
        # 新目标开始：重置停滞检测（脱困计数在换点时重置，同一目标内累计）
        self._stall_best = None
        self._stall_t = time.monotonic()

    def resend_current_goal(self):
        if self.retry_timer is not None:
            self.retry_timer.cancel()
            self.retry_timer = None
        if self.current_target is None:
            self.on_arrived()
            return
        x, y, yaw, label = self.current_target
        self.send_goal(x, y, yaw, label)

    # ---------------- 主循环 ----------------
    def tick(self):
        now = time.monotonic()
        elapsed = now - self.boot
        if self.state == "WAITING":
            if elapsed >= self.start_delay_sec:
                if self.amcl is None:
                    # 兜底：AMCL 长时间不发位姿时依次退让：
                    #   ① 有 /odom 就用真值位姿；② 连 /odom 都没有也继续导航
                    #      （Nav2 靠 TF 自己定位，本节点的位姿只用于误差统计与微调）。
                    if elapsed >= self.start_delay_sec + self.amcl_fallback_sec:
                        if not getattr(self, "_fallback_logged", False):
                            self._fallback_logged = True
                            if self.odom is not None:
                                self.get_logger().warn(
                                    f"等待 AMCL 位姿超过 {self.amcl_fallback_sec:.0f}s 仍无数据，"
                                    f"改用 /odom 真值位姿继续（仿真中 /odom 即真实位姿）。"
                                )
                                self.amcl = dict(self.odom)
                            else:
                                self.get_logger().warn(
                                    f"AMCL 与 /odom 均未收到（已等 "
                                    f"{self.start_delay_sec + self.amcl_fallback_sec:.0f}s），"
                                    f"仍继续导航（定位由 Nav2/TF 自行负责）。"
                                )
                    else:
                        # now 是 time.monotonic() 的 float 秒，按 5 秒节流打日志
                        slot = int(now // 5)
                        if slot != getattr(self, "_last_wait_log", -1):
                            self._last_wait_log = slot
                            self.get_logger().info("等待 AMCL 定位就绪后再开始导航。")
                        return
                # 重试节流：Nav2 未就绪时不要每 0.2s 刷一次日志
                if now < getattr(self, "_next_retry_t", 0.0):
                    return
                self._next_retry_t = now + 2.0
                self.begin_next()
            return

        if self.state == "NAVIGATING":
            # 进度日志：每 10s 输出剩余距离（无 GUI 排错用）
            if self.last_progress is None or now - self.last_progress >= 10.0:
                if self.amcl is not None and self.current_target is not None:
                    dx = self.current_target[0] - self.amcl["x"]
                    dy = self.current_target[1] - self.amcl["y"]
                    self.get_logger().info(
                        f"[progress] ->{self.current_target[3]} 剩余="
                        f"{math.hypot(dx, dy):.3f} m amcl="
                        f"({self.amcl['x']:.3f},{self.amcl['y']:.3f})"
                    )
                self.last_progress = now
            # ---- 停滞检测 ----
            # 官网比赛流程：机器人超过 30s 状态未变化，裁判可停赛。
            # 因此不能只靠 nav_timeout_sec（120s），必须在 15s 级就主动自救，
            # 否则会一直干等到被裁判叫停。
            if self.current_target is not None and self.amcl is not None:
                _d = math.hypot(
                    self.current_target[0] - self.amcl["x"],
                    self.current_target[1] - self.amcl["y"],
                )
                if (self._stall_best is None
                        or _d < self._stall_best - self.stall_improve_m):
                    self._stall_best = _d
                    self._stall_t = now
                elif now - self._stall_t >= self.stall_check_sec:
                    self.get_logger().warn(
                        f"检测到导航停滞：{self.stall_check_sec:.0f}s 内剩余距离"
                        f"未改善（当前 {_d:.3f} m，最好 {self._stall_best:.3f} m）"
                        f" @ {self.current_target[3]} → 取消目标并尝试脱困"
                    )
                    self._recover_count += 1
                    self.publish_event(f"stall:{self.current_target[3]}")
                    self.cancel_nav_goal()
                    self.stop_robot()
                    self.recover_start = now
                    self.state = "RECOVER"
                    return
            if (
                self.nav_result_future is not None
                and self.nav_result_future.done()
            ):
                r = self.nav_result_future.result()
                if r.status == GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info(f"到达目标 {self.current_target[3]}")
                else:
                    self.get_logger().warn(
                        f"导航结束 状态码={r.status} @ {self.current_target[3]}"
                    )
                self.on_arrived()
                return
            if (
                self.nav_start is not None
                and now - self.nav_start > self.nav_timeout_sec
            ):
                self.get_logger().warn(
                    f"导航超时 @ {self.current_target[3]}，取消并继续。"
                )
                if self.nav_goal_handle is not None:
                    self.nav_goal_handle.cancel_goal_async()
                self.on_arrived()
                return

        if self.state == "RECOVER":
            self.do_recover(now)
            return

        if self.state == "NUDGE":
            self.do_nudge(now, self.xy_tol, self.after_arrived)
            return

        if self.state == "PARK_NUDGE":
            self.do_nudge(now, self.park_tol, self.enter_settling)
            return

        if self.state == "SETTLING":
            self.stop_robot()
            if now - self.settle_start >= self.settle_sec:
                self.finish_parking()
            return

        if self.state == "WAIT_RECOGNITION":
            # 停在识别位原地等待，别漂移
            self.stop_robot()
            if self.got_script:
                self.get_logger().info(
                    f"[识别] 点位 {self.wp_index + 1} 已获得识别结果，进入点位。"
                )
                self.goto_point()
                return
            if self.recog_deadline is not None and now >= self.recog_deadline:
                self.get_logger().warn(
                    f"[识别] 点位 {self.wp_index + 1} 等待 "
                    f"{self.recognize_wait:.1f}s 仍未收到话术，"
                    f"仍继续进入点位（不中断全程）。"
                )
                self.goto_point()
            return

        if self.state == "ALIGN_YAW":
            self.do_align_yaw(now)
            return

        if self.state == "AIM_SHOOT":
            # 分点狙击：等射击节点的瞄准+开火服务返回。
            # 服务内部最长瞄准 20 s（旋转靶要等叶片转到水平窗口），
            # 所以这里只轮询、不阻塞，超时后强制继续（不中断优先）。
            #
            # ⚠️ 这里**绝对不能**发布 /cmd_vel：射击节点此刻正在发 /cmd_vel
            # 微转车头对准靶位，本节点再发零速会与之打架，车会抖动、永远对不准。
            # 本节点只在进入狙击前调用一次 stop_robot() 把车停稳。
            fut = self.shoot_future
            if fut is not None and fut.done():
                try:
                    res = fut.result()
                    ok = bool(getattr(res, "success", False))
                    msg = str(getattr(res, "message", ""))
                except Exception as exc:  # noqa: BLE001
                    self.finish_aim_shoot("CALL_ERR", str(exc))
                    return
                self.finish_aim_shoot("OK" if ok else "FAIL", msg)
                return
            if now >= self.shoot_deadline:
                self.safe_cancel_future(fut)
                self.finish_aim_shoot(
                    "TIMEOUT", f"超过 {self.shoot_timeout:.0f}s 未返回")
            return

    def begin_next(self):
        # 换点：重置脱困计数，避免上一个点的自救次数拖累后续点位
        self._recover_count = 0
        if self.wp_index < len(self.waypoints):
            self.go_to_standoff()
        else:
            if self.return_to_base:
                x, y, yaw = self.base
                self.goal_retries = 0
                if not self.send_goal(x, y, yaw, "base(return)"):
                    self.get_logger().warn("Nav2 action server 未就绪，回到等待重发。")
                    self.state = "WAITING"
                    return
            else:
                self.finish_mission()

    def go_to_standoff(self):
        """第一段：导航到识别位（贴图前方约 0.70 m）。没有配置识别位则直接进点位。"""
        i = self.wp_index
        if i < len(self.standoffs):
            x, y, yaw = self.standoffs[i]
            # 每次进点位前重置识别等待状态，避免拿到上一个点的旧话术
            self.got_script = False
            self.last_script = None
            self.recog_state = None
            self.goal_retries = 0
            if not self.send_goal(x, y, yaw, f"task_{i + 1}:standoff"):
                self.get_logger().warn("Nav2 action server 未就绪，回到等待重发。")
                self.state = "WAITING"
                return
        else:
            self.goto_point()

    def goto_point(self):
        """第二段：导航到进入位（任务点本身 / 贴图中心）。"""
        i = self.wp_index
        if i < len(self.waypoints):
            x, y, yaw = self.waypoints[i]
            self.goal_retries = 0
            if not self.send_goal(x, y, yaw, f"task_{i + 1}"):
                self.get_logger().warn("Nav2 action server 未就绪，回到等待重发。")
                self.state = "WAITING"
                return
        else:
            self.finish_mission()

    def on_arrived(self):
        label = self.current_target[3]
        if self.amcl is not None:
            tx, ty = self.current_target[0], self.current_target[1]
            err = math.hypot(self.amcl["x"] - tx, self.amcl["y"] - ty)
            self.wp_errors.append((label, err))
            self.get_logger().info(f"{label} 到点误差(AMCL vs 目标)={err:.3f} m")
            # 静态定位误差：到点瞬间 AMCL vs 真值/odom（机器人此时基本静止）
            if self.odom is not None:
                loc_err = math.hypot(
                    self.amcl["x"] - self.odom["x"], self.amcl["y"] - self.odom["y"]
                )
                self.loc_at_arrival.append((label, loc_err))
                self.get_logger().info(
                    f"{label} 静态定位误差(AMCL vs odom)={loc_err:.3f} m"
                )
            if err > self.xy_tol:
                self.get_logger().info(f"{label} 超出容差，进入微调(NUDGE)。")
                self.nudge_start = time.monotonic()
                self.state = "NUDGE"
                return
        self.after_arrived()

    def after_arrived(self):
        label = self.current_target[3]
        self.publish_event(f"reached:{label}")

        # ---- 第一段：到达识别位 → 原地等识别话术 ----
        if label.endswith(":standoff"):
            self.got_script = False
            self.recog_deadline = time.monotonic() + self.recognize_wait
            self.get_logger().info(
                f"[识别] 已到达识别位 {label}，等待话术（最长 "
                f"{self.recognize_wait:.1f}s）后再进入点位。"
            )
            self.state = "WAIT_RECOGNITION"
            return

        # ---- 第二段：进入任务点 → 声明射击点 → 瞄准射击 → 下一点 ----
        if label.startswith("task_"):
            point = self.wp_index + 1
            self.publish_referee_target(point)
            if self.enable_shooting:
                # 先精确对准车头，再调射击服务。
                # Nav2 停车的 yaw 偏差可能较大（实测能让 0.27 m 宽的环形靶整块跑出视野），
                # 而射击节点只负责最后 ±3° 的微调，所以这里先把大头消掉。
                self.enter_align_yaw(point)
                return
            self.wp_index += 1
            self.begin_next()
        elif label.startswith("base"):
            self.publish_event("returned:base")
            self.nudge_start = time.monotonic()
            self.state = "PARK_NUDGE"
        else:
            self.finish_mission()

    def enter_align_yaw(self, point):
        """进入 ALIGN_YAW：用 odom 闭环把车头转到当前航点要求的 yaw。"""
        target_yaw = float(self.current_target[2]) if self.current_target else 0.0
        odom = self.odom
        cur = math.degrees(odom["yaw"]) if odom else float("nan")
        err = float("nan")
        if odom is not None:
            err = math.degrees(
                (target_yaw - odom["yaw"] + math.pi) % (2 * math.pi) - math.pi)
        self.get_logger().info(
            f"[对准] 任务点 {point}：目标 yaw={math.degrees(target_yaw):+.1f}° "
            f"当前={cur:+.1f}° 误差={err:+.1f}°（容差 "
            f"{math.degrees(self.align_yaw_tol):.1f}°）")
        self.align_point = point
        self.align_target_yaw = target_yaw
        self.align_t0 = time.monotonic()
        self.state = "ALIGN_YAW"

    def do_align_yaw(self, now):
        """tick 里的 ALIGN_YAW 处理：转到位或超时就进入射击。"""
        odom = self.odom
        if odom is not None:
            err = (self.align_target_yaw - odom["yaw"] + math.pi) % (
                2 * math.pi) - math.pi
            if abs(err) <= self.align_yaw_tol:
                self.stop_robot()
                self.get_logger().info(
                    f"[对准] 任务点 {self.align_point} 车头已到位"
                    f"（残差 {math.degrees(err):+.2f}°）")
                self.start_aim_shoot(self.align_point)
                return
            if now - self.align_t0 < self.align_yaw_timeout:
                tw = Twist()
                # 角度环用比例控制，限幅 0.25 rad/s（远低于 0.5 m/s 的线速等价量）
                tw.angular.z = clamp(1.2 * err, -0.25, 0.25)
                self.cmd_pub.publish(tw)
                return
            self.get_logger().warn(
                f"[对准] 任务点 {self.align_point} 对准超时"
                f"（残差 {math.degrees(err):+.1f}°），仍继续射击")
        else:
            self.get_logger().warn("[对准] 没有 odom，跳过对准")
        self.stop_robot()
        self.start_aim_shoot(self.align_point)

    def start_aim_shoot(self, point):
        """进入分点狙击：调 /ican_shoot/fire_* 服务，异步等结果。

        为什么异步而不同步调：
        射击节点内部瞄准最长 20 s（旋转靶要等叶片转到水平窗口），阻塞调用会把
        状态机卡死、也无法在中途响应退出。改成 call_async + tick 里轮询，
        超时后强制继续 —— 判分要求「全程不中断」优先于「每点都打中」。
        """
        client = self.shoot_clients.get(point)
        if client is None:
            self.get_logger().warn(f"任务点 {point} 没有对应的射击服务，跳过射击。")
            self.wp_index += 1
            self.begin_next()
            return
        if not client.service_is_ready() and not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                f"任务点 {point} 的射击服务 {client.srv_name} 不可用，跳过射击"
                f"（是否忘了启动 ican_shoot？）")
            self.shoot_results.append((point, "NO_SERVICE"))
            self.wp_index += 1
            self.begin_next()
            return

        self.stop_robot()
        self.shoot_point = point
        self.shoot_deadline = time.monotonic() + self.shoot_timeout
        self.publish_event(f"aiming:point{point}")
        self.get_logger().info(
            f"[狙击] 任务点 {point}：调用 {client.srv_name}（预算 "
            f"{self.shoot_timeout:.0f}s）")
        try:
            self.shoot_future = client.call_async(Trigger.Request())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"[狙击] 调用失败: {exc}")
            self.shoot_results.append((point, f"CALL_ERR:{exc}"))
            self.shoot_future = None
            self.wp_index += 1
            self.begin_next()
            return
        self.state = "AIM_SHOOT"

    def finish_aim_shoot(self, verdict, detail=""):
        """结束本点狙击，进入下一点（无论成败都不中断）。"""
        point = self.shoot_point
        self.shoot_future = None
        self.shoot_results.append((point, verdict))
        self.get_logger().info(f"[狙击] 任务点 {point} 结束：{verdict} {detail}".strip())
        self.publish_event(f"shot:point{point}:{verdict}")
        self.wp_index += 1
        self.begin_next()

    def safe_cancel_future(self, fut):
        """尽力取消一个未完成的 future（rclpy 不支持真正取消，仅做清理与记录）。"""
        if fut is None:
            return
        try:
            if not fut.done():
                fut.cancel()
        except Exception:  # noqa: BLE001
            pass

    def cancel_nav_goal(self):
        """异步取消当前导航目标（取消失败不影响后续流程）。"""
        handle = self.nav_goal_handle
        self.nav_goal_handle = None
        self.nav_result_future = None
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"取消导航目标失败: {exc}")

    def do_recover(self, now):
        """停滞自救：朝目标方向做短时微调（麦轮可横移，对脱困有效），再重发目标。

        超时或次数超限后强制继续，保证全程不中断（判分要求"不中断"优先于"完美到达"）。
        """
        if self.current_target is None:
            self.stop_robot()
            self.get_logger().error("脱困时没有当前目标，状态机无法继续。")
            return
        label = self.current_target[3]
        timed_out = (now - self.recover_start) > self.recover_sec

        if timed_out or self.amcl is None:
            self.stop_robot()
            x, y, yaw, _ = self.current_target
            if self._recover_count >= self.max_stall_recover:
                self.get_logger().error(
                    f"目标 {label} 连续 {self._recover_count} 次脱困失败，"
                    f"按到达处理继续（优先保证全程不中断）。"
                )
                self.on_arrived()
                return
            self.get_logger().warn(
                f"脱困结束（第 {self._recover_count} 次）→ 重新发送导航目标 {label}"
            )
            self._stall_best = None
            self._stall_t = now
            self.send_goal(x, y, yaw, label)
            return

        # 朝目标方向低速微调（与到达微调用同一套麦轮解算）
        ex = self.current_target[0] - self.amcl["x"]
        ey = self.current_target[1] - self.amcl["y"]
        yaw = self.amcl["yaw"]
        rx = ex * math.cos(yaw) + ey * math.sin(yaw)
        ry = -ex * math.sin(yaw) + ey * math.cos(yaw)
        tw = Twist()
        tw.linear.x = clamp(rx * 0.6, -self.nudge_speed, self.nudge_speed)
        tw.linear.y = clamp(ry * 0.6, -self.nudge_speed, self.nudge_speed)
        self.cmd_pub.publish(tw)

    def do_nudge(self, now, tol, on_done):
        if self.amcl is None or now - self.nudge_start > self.nudge_max_sec:
            self.get_logger().warn("微调超时/无定位，继续后续流程。")
            self.stop_robot()
            on_done()
            return
        tx, ty = self.current_target[0], self.current_target[1]
        ex = tx - self.amcl["x"]
        ey = ty - self.amcl["y"]
        dist = math.hypot(ex, ey)
        if dist <= tol:
            self.get_logger().info(f"微调到位（误差 {dist:.3f} m）。")
            self.stop_robot()
            on_done()
            return
        yaw = self.amcl["yaw"]
        rx = ex * math.cos(yaw) + ey * math.sin(yaw)
        ry = -ex * math.sin(yaw) + ey * math.cos(yaw)
        tw = Twist()
        tw.linear.x = clamp(rx * 0.6, -self.nudge_speed, self.nudge_speed)
        tw.linear.y = clamp(ry * 0.6, -self.nudge_speed, self.nudge_speed)
        self.cmd_pub.publish(tw)

    def enter_settling(self):
        self.stop_robot()
        self.settle_start = time.monotonic()
        self.state = "SETTLING"

    def finish_parking(self):
        self.stop_robot()
        err = (
            math.hypot(
                self.amcl["x"] - self.base[0], self.amcl["y"] - self.base[1]
            )
            if self.amcl
            else -1.0
        )
        self.get_logger().info(
            f"停车完成，停车误差={err:.3f} m（容差 {self.park_tol} m）"
        )
        self.publish_event(f"parked:err={err:.3f}")
        self.finish_mission()

    def fire(self):
        if not self.fire_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("未找到射击服务 /shooter/shoot。")
            return
        fut = self.fire_client.call_async(Trigger.Request())
        fut.add_done_callback(
            lambda f: self.get_logger().info(
                f"射击调用完成: success={getattr(f.result(), 'success', 'ERR')}"
            )
        )

    # ---------------- 结束与报告 ----------------
    def finish_mission(self):
        if self.state == "DONE":
            return
        self.state = "DONE"
        self.stop_robot()
        # 完成声明：判分要求「返回停车后经 /referee/finish 发布完成声明」
        self.publish_referee_finish()
        self.write_report()
        self.get_logger().info("==== 顺序导航任务完成 ====")

    def write_report(self):
        n = len(self.loc_dx)
        if n > 0:
            rms = math.sqrt(
                sum(dx * dx + dy * dy for dx, dy in zip(self.loc_dx, self.loc_dy))
                / n
            )
            maxe = max(
                math.hypot(dx, dy) for dx, dy in zip(self.loc_dx, self.loc_dy)
            )
        else:
            rms = maxe = 0.0
        lines = ["==== 导航任务结果 ===="]
        lines.append("任务点到达误差(AMCL vs 目标坐标):")
        for label, err in self.wp_errors:
            lines.append(f"  {label}: {err:.3f} m")
        lines.append(
            f"定位精度(AMCL vs 真值/odom, 全程含运动): RMS={rms:.4f} m, MAX={maxe:.4f} m, 样本={n}"
            f" -> {'PASS(≤0.1m)' if maxe <= 0.1 else 'CHECK(运动滞后)'}"
        )
        if self.loc_at_arrival:
            n2 = len(self.loc_at_arrival)
            rms2 = math.sqrt(sum(e * e for _, e in self.loc_at_arrival) / n2)
            max2 = max(e for _, e in self.loc_at_arrival)
            lines.append(
                f"静态定位误差(各点到点瞬间 AMCL vs 真值): RMS={rms2:.4f} m, "
                f"MAX={max2:.4f} m, 样本={n2}"
                f" -> {'PASS(≤0.1m)' if max2 <= 0.1 else 'CHECK(>0.1m)'}"
            )
        if self.amcl is not None:
            park_err = math.hypot(
                self.amcl["x"] - self.base[0], self.amcl["y"] - self.base[1]
            )
            lines.append(f"停车误差(AMCL vs 指挥中心): {park_err:.3f} m")
        # 分点狙击结果（D-05）：OK=服务返回 success、FAIL=瞄准未成功、
        # TIMEOUT=超预算、NO_SERVICE=射击节点没起
        if self.shoot_results:
            lines.append("分点狙击结果(①环形靶 ②旋转靶 ③移动靶3 ④移动靶4):")
            for p, v in self.shoot_results:
                lines.append(f"  任务点{p}: {v}")
        else:
            lines.append("分点狙击结果: 未启用（enable_shooting=false）")
        text = "\n".join(lines)
        self.get_logger().info(text)
        if self.result_file:
            try:
                # 目标目录（默认 ~/.ros/ican_nav_tasks/）可能不存在，先创建
                parent = os.path.dirname(os.path.expanduser(self.result_file))
                if parent:
                    os.makedirs(parent, exist_ok=True)
                with open(os.path.expanduser(self.result_file), "w", encoding="utf-8") as f:
                    f.write(text + "\n")
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"写结果文件失败: {e}")


def main():
    rclpy.init()
    node = SequentialMissionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: BLE001
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            node.stop_robot()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
