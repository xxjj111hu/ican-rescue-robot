"""顺序导航 + 返航停车 状态机（任务13 / 任务14）。

流程：等待定位初始化 -> 依次导航到 1、2、3、4 号任务点（每个点到达后判定进点有效，
       可选发布识别/射击事件）-> 返回指挥中心(0,0) -> 停车微调 -> 稳定刹停 -> 输出结果报告。

同时采集定位误差（AMCL 位姿 vs /odom 真值），作为《地图与定位精度校验》的输入数据。

注意：本节点只负责“导航框架”。每个任务点的“识别话术 + 射击”由识别模块接入：
      - 到达任务点时本节点向 /mission/event 发布 reached:<label>；
      - 若 enable_shooting=true，则调用 /shooter/shoot 触发射击（占位，识别模块接管瞄准逻辑）。
"""
import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
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
        self.declare_parameter("nav_timeout_sec", 120.0)
        self.declare_parameter("goal_retry_sec", 2.0)
        # 任务点：[x, y, yaw] * 4
        self.declare_parameter(
            "waypoints",
            [
                0.031, -1.360, 0.0,
                0.027, -2.700, 0.0,
                1.120, -1.914, math.pi / 2.0,
                1.120, -0.785, math.pi / 2.0,
            ],
        )
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
        self.declare_parameter("event_topic", "/mission/event")
        self.declare_parameter("navigate_action", "/navigate_to_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter(
            "result_file",
            "/home/ican/Ican_fly/demo_shoot_ws/logs/mission_result.txt",
        )

        wp = self.get_parameter("waypoints").value
        self.waypoints = [
            (float(wp[i]), float(wp[i + 1]), float(wp[i + 2]))
            for i in range(0, len(wp), 3)
        ]
        self.base = [float(x) for x in self.get_parameter("base_xyyaw").value]
        self.return_to_base = bool(self.get_parameter("return_to_base").value)
        self.xy_tol = float(self.get_parameter("xy_tol").value)
        self.yaw_tol = float(self.get_parameter("yaw_tol").value)
        self.park_tol = float(self.get_parameter("park_tol").value)
        self.settle_sec = float(self.get_parameter("settle_sec").value)
        self.nudge_speed = float(self.get_parameter("nudge_speed").value)
        self.nudge_max_sec = float(self.get_parameter("nudge_max_sec").value)
        self.enable_shooting = bool(self.get_parameter("enable_shooting").value)
        self.result_file = self.get_parameter("result_file").value

        # 接口
        self.nav_client = ActionClient(
            self, NavigateToPose, str(self.get_parameter("navigate_action").value)
        )
        self.fire_client = self.create_client(
            Trigger, str(self.get_parameter("fire_service").value)
        )
        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        event_qos = QoSProfile(depth=10)
        event_qos.reliability = ReliabilityPolicy.RELIABLE
        event_qos.durability = DurabilityPolicy.VOLATILE
        self.event_pub = self.create_publisher(
            String, str(self.get_parameter("event_topic").value), event_qos
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, 10
        )
        # /odom 由 gazebo_ros_planar_move 插件发布，类型为 nav_msgs/msg/Odometry（真值位姿）
        self.create_subscription(
            Odometry, "/odom", self.on_odom, 10
        )

        # 状态
        self.state = "WAITING"
        self.boot = self.get_clock().now()
        self.start_delay = Duration(
            seconds=float(self.get_parameter("start_delay_sec").value)
        )
        self.nav_timeout = Duration(
            seconds=float(self.get_parameter("nav_timeout_sec").value)
        )
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
        self.loc_dx = []
        self.loc_dy = []
        self.wp_errors = []  # (label, err) AMCL vs 目标坐标
        self.loc_at_arrival = []  # (label, err) 到点瞬间 AMCL vs 真值/odom（静态）
        self.last_progress = None  # 进度日志时间戳

        # 5 Hz state machine tick (was 10 Hz) to reduce CPU on the VM
        self.timer = self.create_timer(0.2, self.tick)
        self.get_logger().info(
            f"顺序导航节点就绪：{len(self.waypoints)} 个任务点，"
            f"返航={'是' if self.return_to_base else '否'}。"
            f"任务点坐标={self.waypoints}"
        )

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
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qx, qy, qz, qw = quat_from_yaw(float(yaw))
        goal.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        self.current_target = (float(x), float(y), float(yaw), label)
        self.nav_goal_handle = None
        self.nav_result_future = None
        self.nav_start = self.get_clock().now()
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
                self.nav_start = self.get_clock().now()
                delay = float(self.get_parameter("goal_retry_sec").value)
                self.retry_timer = self.create_timer(delay, self.resend_current_goal)
                return
            self.get_logger().error(
                f"Nav2 多次拒绝导航目标({self.current_target[3]})，按到达处理继续。"
            )
            self.on_arrived()
            return
        self.get_logger().info("Nav2 已接受导航目标。")
        self.nav_result_future = self.nav_goal_handle.get_result_async()

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
        now = self.get_clock().now()
        if self.state == "WAITING":
            if now - self.boot >= self.start_delay:
                if self.amcl is None:
                    # 尚未收到 AMCL 定位，继续等待（避免 Nav2 因未定位而拒目标）
                    if int(now.nanoseconds // 5e9) != getattr(self, "_last_wait_log", -1):
                        self._last_wait_log = int(now.nanoseconds // 5e9)
                        self.get_logger().info("等待 AMCL 定位就绪后再开始导航。")
                    return
                self.begin_next()
            return

        if self.state == "NAVIGATING":
            # 进度日志：每 10s 输出剩余距离（无 GUI 排错用）
            if self.last_progress is None or now - self.last_progress >= Duration(
                seconds=10.0
            ):
                if self.amcl is not None and self.current_target is not None:
                    dx = self.current_target[0] - self.amcl["x"]
                    dy = self.current_target[1] - self.amcl["y"]
                    self.get_logger().info(
                        f"[progress] ->{self.current_target[3]} 剩余="
                        f"{math.hypot(dx, dy):.3f} m amcl="
                        f"({self.amcl['x']:.3f},{self.amcl['y']:.3f})"
                    )
                self.last_progress = now
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
                and now - self.nav_start > self.nav_timeout
            ):
                self.get_logger().warn(
                    f"导航超时 @ {self.current_target[3]}，取消并继续。"
                )
                if self.nav_goal_handle is not None:
                    self.nav_goal_handle.cancel_goal_async()
                self.on_arrived()
                return

        if self.state == "NUDGE":
            self.do_nudge(now, self.xy_tol, self.after_arrived)
            return

        if self.state == "PARK_NUDGE":
            self.do_nudge(now, self.park_tol, self.enter_settling)
            return

        if self.state == "SETTLING":
            self.stop_robot()
            if now - self.settle_start >= Duration(seconds=self.settle_sec):
                self.finish_parking()
            return

    def begin_next(self):
        if self.wp_index < len(self.waypoints):
            x, y, yaw = self.waypoints[self.wp_index]
            self.goal_retries = 0
            if not self.send_goal(x, y, yaw, f"task_{self.wp_index + 1}"):
                self.get_logger().warn("Nav2 action server 未就绪，回到等待重发。")
                self.state = "WAITING"
                return
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
                self.nudge_start = self.get_clock().now()
                self.state = "NUDGE"
                return
        self.after_arrived()

    def after_arrived(self):
        label = self.current_target[3]
        self.publish_event(f"reached:{label}")
        if self.enable_shooting:
            self.fire()
        if label.startswith("task_"):
            self.wp_index += 1
            self.begin_next()
        elif label.startswith("base"):
            self.publish_event("returned:base")
            self.nudge_start = self.get_clock().now()
            self.state = "PARK_NUDGE"
        else:
            self.finish_mission()

    def do_nudge(self, now, tol, on_done):
        if self.amcl is None or now - self.nudge_start > Duration(
            seconds=self.nudge_max_sec
        ):
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
        self.settle_start = self.get_clock().now()
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
        text = "\n".join(lines)
        self.get_logger().info(text)
        if self.result_file:
            try:
                with open(self.result_file, "w", encoding="utf-8") as f:
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
