"""旋转靶 / 移动靶瞄准射击节点（D-03 旋转靶 1~5 号、D-04 移动靶 6/7/8 号）。

为什么不用视觉认靶
------------------
靶面上的 AR 码在图里只有约 14x14 px 且带透视畸变，OpenCV 全部 16 个内置字典
x 4 种放大倍数都解不出（唯一 DICT_4X4_1000 id=988 是误检）。
而 Gazebo 的 /gazebo/get_entity_state 可以直接读到靶位姿态 —— 于是改为**读几何 + 算时序**：

旋转靶（target_wheel）的内部结构（来自 land_shoot.world，已核实）
--------------------------------------------------------------------
一个 link，5 个叶片等间隔 72°，半径 0.09 m，叶片尺寸 0.01x0.05x0.05：

    leaf_0  pose(0,  0,        0.09)     material WheelLeaf1Texture  -> 靶位 1
    leaf_1  pose(0,  0.085595, 0.027812) material WheelLeaf2Texture  -> 靶位 2
    leaf_2  pose(0,  0.052901, -0.072812) material WheelLeaf3Texture -> 靶位 3
    leaf_3  pose(0, -0.052901, -0.072812) material WheelLeaf4Texture -> 靶位 4
    leaf_4  pose(0, -0.085595, 0.027812) material WheelLeaf5Texture  -> 靶位 5

即 **leaf_k 就是靶位 k+1**，本地角度 phi_k = 90° - 72°*k（从 +y 轴往 +z 量）。
世界角度 = phi_k + theta，theta 为轮子绕 x 轴的转角（orientation 四元数直接可算）。

关键物理约束（这条不知道就永远打不中）
--------------------------------------
叶片排在半径 0.09 m 的圆周上，**圆心是空的**；而子弹从 0.26 m 高度水平射出、
正好穿过轮子中心。所以**正对轮心射击一发都打不中**，必须让叶片转到
左右水平位置（世界角 ≈ 0° 或 180°，此时其 z 偏移为 0、y 偏移为 ±0.09），
并把车头偏转 atan(0.09 / 距离) ≈ 3.33° 对准它。

移动靶（target_moving_3/4）
---------------------------
单个板 0.02x0.18x0.07，贴图 MovingBoardTexture（3 个码画在贴图上，非独立几何），
所以编号顺序无法从 world 文件读 —— 用参数 moving_code_order 配置（默认左→右 = 6,7,8）。
板沿 x 往复 ±0.15 m；其世界 x 可实时读，据此算每个码的世界 x，再转车头对准。

计时
----
一律墙钟 + time.sleep；ROS 定时器在 /clock 停滞时不会触发（本项目踩过）。
服务为阻塞式（Trigger 是同步的，返回值之后无法修改），main 用多线程执行器。
"""
from __future__ import annotations

import json
import math
import threading
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger

# ---- 旋转靶几何（来自 land_shoot.world，勿随意改）----
WHEEL_LEAF_RADIUS = 0.09
WHEEL_LEAF_COUNT = 5
WHEEL_LEAF_HALF_W = 0.025          # 叶片半宽/半高（0.05 的一半）
WHEEL_PHI0_DEG = 90.0              # leaf_0 的本地角度（+y 方向为 0°，向 +z 为正）

# 移动靶几何
MOVING_BOARD_W = 0.18
MOVING_CODE_OFFSETS = (-0.06, 0.0, 0.06)   # 3 个码相对板中心的 x 偏移


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def quat_to_roll_x(x, y, z, w):
    """返回绕 x 轴的旋转角（rad）。假设旋转轴就是 x。"""
    # 纯绕 x 轴: w = cos(t/2), x = sin(t/2)
    n = math.hypot(x, w)
    if n < 1e-9:
        return 0.0
    return 2.0 * math.atan2(x / n, w / n)


def yaw_from_quat(z, w):
    return 2.0 * math.atan2(z, w)


class _Ent:
    """轻量包装，让调用方继续用 st.pose.position / st.pose.orientation。"""

    __slots__ = ("pose",)

    def __init__(self, pose):
        self.pose = pose


class _Resp:
    """占位响应对象：瞄准线程往里写结果，服务回调再拷到真正的 response。

    为什么要绕这一层：在 ROS 服务回调里直接调另一个服务（read state / shoot）
    会遇到 rclpy 的重入问题（实测 future 永远不完成、静默 20 s 后超时）。
    改在普通线程里跑，服务回调只等结果，问题消失。
    """

    __slots__ = ("success", "message")

    def __init__(self):
        self.success = False
        self.message = ""


class TargetShooterNode(Node):
    def __init__(self):
        super().__init__("target_shooter_node")

        self.declare_parameter("model_states_topic", "/gazebo/model_states")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("fire_service", "/shooter/shoot")
        self.declare_parameter("shot_topic", "/shooter/shot")
        self.declare_parameter("status_topic", "/ican_shoot/status")

        # 本节点自己的服务名
        self.declare_parameter("wheel_service", "/ican_shoot/fire_wheel")
        self.declare_parameter("moving3_service", "/ican_shoot/fire_moving_3")
        self.declare_parameter("moving4_service", "/ican_shoot/fire_moving_4")

        # 靶位编号来源：订阅识别节点的 /ican_recognize/target
        self.declare_parameter("recognize_target_topic", "/ican_recognize/target")
        # 识别没给出时的兜底编号
        self.declare_parameter("default_rotate_target", 3)
        self.declare_parameter("default_moving_target", 7)

        # 移动靶 3 个码的顺序（左→右），用于把 id 映射到板的偏移
        self.declare_parameter("moving_code_order", "678")

        # ---- 时序与容差 ----
        self.declare_parameter("poll_hz", 20.0)
        self.declare_parameter("aim_timeout_sec", 20.0)
        # 叶片世界角与水平位置(0° 或 180°)的允许偏差
        self.declare_parameter("wheel_angle_tol_deg", 6.0)
        # 连续满足多久才开火
        self.declare_parameter("settle_sec", 0.15)
        # 对准叶片/码所需的偏转角容差
        self.declare_parameter("yaw_tol_deg", 0.6)
        self.declare_parameter("max_angular_speed", 0.18)
        self.declare_parameter("yaw_gain", 0.8)
        # 旋转靶水平窗口半角：|r*sin(psi)| <= 叶片半高 0.025 即 arcsin(0.025/0.09)=16.1°，
        # 取 13° 留余量
        self.declare_parameter("wheel_window_deg", 13.0)
        # 旋转靶横向对准容差（米）
        self.declare_parameter("wheel_lat_tol_m", 0.012)
        # 服务被调用时若还没收到 /odom，先等这么久再判失败
        # （联调时出现过「本节点没有 /odom」就立刻失败的情况，属瞬时现象）
        self.declare_parameter("odom_wait_sec", 5.0)

        # 移动靶对准容差（板上的码宽 0.06 m -> 用像素无关的米制容差）
        self.declare_parameter("moving_pos_tol_m", 0.012)

        self.declare_parameter("fire_probe_sec", 2.0)
        self.declare_parameter("dry_run", False)

        self.poll_dt = 1.0 / max(1.0, float(self.get_parameter("poll_hz").value))
        self.aim_timeout = float(self.get_parameter("aim_timeout_sec").value)
        self.wheel_tol = math.radians(
            float(self.get_parameter("wheel_angle_tol_deg").value))
        self.settle_sec = float(self.get_parameter("settle_sec").value)
        self.yaw_tol = math.radians(float(self.get_parameter("yaw_tol_deg").value))
        self.max_w = float(self.get_parameter("max_angular_speed").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)
        self.wheel_window = math.radians(
            float(self.get_parameter("wheel_window_deg").value))
        self.wheel_lat_tol = float(self.get_parameter("wheel_lat_tol_m").value)
        self.odom_wait = float(self.get_parameter("odom_wait_sec").value)
        self.moving_pos_tol = float(self.get_parameter("moving_pos_tol_m").value)
        self.fire_probe = float(self.get_parameter("fire_probe_sec").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        order = str(self.get_parameter("moving_code_order").value)
        self.code_order = [int(c) for c in order if c.isdigit()] or [6, 7, 8]

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        status_qos = QoSProfile(depth=10)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.VOLATILE
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), status_qos)

        # ---- 回调组：这是本节点最关键的一处设计 ----
        # ROS 2 节点默认的回调组是 MutuallyExclusiveCallbackGroup：同一组的回调严格串行。
        # 于是「服务回调阻塞等结果」会把同一节点里所有订阅回调一起卡住，
        # 瞄准线程读到的 model_states / odom 就全是冻结的旧值
        # （实测：20 秒里轮角一直是 94.0°、横向一直是 +35mm，连续不变=389）。
        # 开再多执行器线程也没用 —— 限制来自回调组，不是线程数。
        # 因此把订阅放进独立的 ReentrantCallbackGroup，让它能和服务回调并发执行。
        self._io_group = ReentrantCallbackGroup()
        self._srv_group = MutuallyExclusiveCallbackGroup()

        # /odom 用 SensorDataQoS（Gazebo 以 BEST_EFFORT 发布，用 RELIABLE 收不到）
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self.on_odom, qos_profile_sensor_data, callback_group=self._io_group)
        # 靶位姿态：订阅 model_states（比 get_entity_state 服务可靠，见 on_model_states 注释）
        self.create_subscription(
            ModelStates, str(self.get_parameter("model_states_topic").value),
            self.on_model_states, qos_profile_sensor_data, callback_group=self._io_group)
        self.create_subscription(
            String, str(self.get_parameter("shot_topic").value), self.on_shot, 10,
            callback_group=self._io_group)
        self.create_subscription(
            String, str(self.get_parameter("recognize_target_topic").value),
            self.on_recog_target, 10, callback_group=self._io_group)

        self.fire_client = self.create_client(
            Trigger, str(self.get_parameter("fire_service").value),
            callback_group=self._io_group)

        self.create_service(
            Trigger, str(self.get_parameter("wheel_service").value), self.on_fire_wheel,
            callback_group=self._srv_group)
        self.create_service(
            Trigger, str(self.get_parameter("moving3_service").value),
            lambda req, resp: self.on_fire_moving(req, resp, "target_moving_3"),
            callback_group=self._srv_group)
        self.create_service(
            Trigger, str(self.get_parameter("moving4_service").value),
            lambda req, resp: self.on_fire_moving(req, resp, "target_moving_4"),
            callback_group=self._srv_group)

        self._lock = threading.Lock()
        self.odom = None            # {"x","y","yaw"}
        self.odom_t = 0.0
        self._models = {}           # name -> Pose（来自 /gazebo/model_states）
        self._models_t = 0.0
        self.shot_msgs = 0
        self.last_shot_name = ""
        self.recog_rotate = None
        self.recog_moving = None
        self._busy = False

        self.get_logger().info(
            "旋转/移动靶射击节点就绪：读 Gazebo 靶位姿态做时序瞄准。"
            f"服务 {self.get_parameter('wheel_service').value} / "
            f"{self.get_parameter('moving3_service').value} / "
            f"{self.get_parameter('moving4_service').value}"
            f"{' | dry_run' if self.dry_run else ''}"
        )

    # ---------------- 订阅 ----------------
    def on_odom(self, msg):
        p = msg.pose.pose
        yaw = yaw_from_quat(p.orientation.z, p.orientation.w)
        with self._lock:
            self.odom = {"x": p.position.x, "y": p.position.y, "yaw": yaw}
            self.odom_t = time.monotonic()

    def on_shot(self, msg):
        with self._lock:
            self.shot_msgs += 1
            self.last_shot_name = msg.data
        self.get_logger().info(f"[/shooter/shot] {msg.data}")

    def on_recog_target(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            if d.get("rotate_target") is not None:
                self.recog_rotate = int(d["rotate_target"])
            if d.get("moving_target") is not None:
                self.recog_moving = int(d["moving_target"])
        self.get_logger().info(
            f"收到识别靶位：旋转靶={d.get('rotate_target')} 移动靶={d.get('moving_target')}"
            f"（{d.get('class')}）")

    def _snapshot(self):
        with self._lock:
            return (dict(self.odom) if self.odom else None), self.odom_t, self.shot_msgs

    def _wait_odom(self, timeout=None):
        """等一个新鲜 /odom（年龄 < 1 s），返回 (odom 或 None, 年龄秒)。

        联调时出现过「本节点没有 /odom」就立刻失败的情况（前两次同样代码却能跑通），
        说明是瞬时现象。目标节点不该因为一次没收到就放弃整个点的射击。
        """
        if timeout is None:
            timeout = self.odom_wait
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            odom, odom_t, _ = self._snapshot()
            if odom is not None and (time.monotonic() - odom_t) < 1.0:
                return odom, time.monotonic() - odom_t
            time.sleep(0.05)
        odom, odom_t, _ = self._snapshot()
        age = (time.monotonic() - odom_t) if odom_t else 9e9
        return odom, age

    def _publish_status(self, state, **kw):
        payload = {"state": state, "t": round(time.monotonic(), 3)}
        payload.update(kw)
        m = String()
        m.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(m)

    def _stop(self):
        self.cmd_pub.publish(Twist())

    # ---------------- 靶位姿态：订阅 /gazebo/model_states ----------------
    # 为什么不用 /gazebo/get_entity_state 服务：
    # 实测该服务在程序里调用时请求发得出去、响应永远回不来（日志里连续 17 次
    # 「1.0s 内无响应」），而命令行调同一个服务却正常；/gazebo/get_model_list
    # 更是直接卡死。改用持续发布的话题，既可靠又能连续跟踪轮子转角。
    def on_model_states(self, msg):
        try:
            table = {}
            for nm, pose in zip(msg.name, msg.pose):
                table[nm] = pose
            with self._lock:
                self._models = table
                self._models_t = time.monotonic()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"解析 model_states 失败: {exc}")

    def _have_models(self):
        with self._lock:
            return self._models_t > 0.0

    def _get_entity(self, name, timeout=1.0):
        """从缓存的 model_states 里取位姿；最多等 timeout 秒等首帧。"""
        if self._models_t == 0.0:
            t0 = time.monotonic()
            while self._models_t == 0.0 and (time.monotonic() - t0) < timeout:
                time.sleep(0.01)
        with self._lock:
            pose = self._models.get(name)
            age = time.monotonic() - self._models_t if self._models_t else 1e9
        if pose is None:
            self.get_logger().warn(
                f"[诊断] model_states 里没有 {name}"
                f"（已知模型: {sorted(self._models.keys())[:8]}...），"
                f"该话题年龄 {age:.1f}s")
            return None
        return _Ent(pose)

    # ---------------- 车头转向（odom 闭环） ----------------
    def _rotate_to_yaw(self, target_yaw, timeout=6.0):
        """把车头转到 target_yaw（rad），返回 (ok, 最终误差rad)。"""
        t0 = time.monotonic()
        last_err = None
        stable = 0
        need = max(1, int(round(0.15 / self.poll_dt)))
        while time.monotonic() - t0 < timeout:
            odom, odom_t, _ = self._snapshot()
            if odom is None or (time.monotonic() - odom_t) > 1.0:
                self._stop()
                time.sleep(self.poll_dt)
                continue
            err = (target_yaw - odom["yaw"] + math.pi) % (2 * math.pi) - math.pi
            last_err = err
            if abs(err) <= self.yaw_tol:
                stable += 1
                self._stop()
                if stable >= need:
                    return True, err
            else:
                stable = 0
                tw = Twist()
                tw.angular.z = clamp(self.yaw_gain * err, -self.max_w, self.max_w)
                self.cmd_pub.publish(tw)
            time.sleep(self.poll_dt)
        self._stop()
        return False, (last_err if last_err is not None else 9.9)

    # ---------------- 开火 ----------------
    def _fire_and_report(self):
        """调用射击服务。不长时间阻塞 —— 实测响应 5.9~19.9 s 不稳定。"""
        if self.dry_run:
            return True, "dry_run（未实弹）"
        if not self.fire_client.wait_for_service(timeout_sec=2.0):
            return False, "找不到射击服务 /shooter/shoot"
        _, _, shot0 = self._snapshot()
        try:
            fut = self.fire_client.call_async(Trigger.Request())
        except Exception as exc:  # noqa: BLE001
            return False, f"射击服务调用异常: {exc}"
        t0 = time.monotonic()
        while not fut.done() and (time.monotonic() - t0) < self.fire_probe:
            time.sleep(0.02)
        if fut.done():
            try:
                res = fut.result()
                if res is not None and not res.success:
                    return False, f"射击服务报错: {res.message}"
            except Exception:  # noqa: BLE001
                pass
        # 等 /shooter/shot 佐证，等不到也不算失败
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            _, _, n = self._snapshot()
            if n > shot0:
                with self._lock:
                    return True, f"已射出（{self.last_shot_name}）"
            time.sleep(0.03)
        return True, "已发出发射指令（/shooter/shot 确认延迟，属已知现象）"

    # ---------------- 线程化执行（避开 rclpy 回调重入） ----------------
    def _run_in_thread(self, work):
        """在普通线程里执行 work()，最多等 aim_timeout + 20 s。

        work 应往临时响应对象里写 success/message（用 _Resp）。
        关键：不要在 ROS 回调上下文里直接调服务，否则 future 不会完成。
        """
        done = threading.Event()

        def _worker():
            try:
                work()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"瞄准线程异常: {type(exc).__name__}: {exc}")
            finally:
                self._stop()
                done.set()

        th = threading.Thread(target=_worker, daemon=True, name="aim_worker")
        th.start()
        if not done.wait(timeout=self.aim_timeout + 20.0):
            self.get_logger().warn("瞄准线程超时未结束，强制返回")

    # ---------------- D-03：旋转靶 ----------------
    def on_fire_wheel(self, _req, response):
        if self._busy:
            response.success = False
            response.message = "已有任务在进行"
            return response
        self._busy = True
        try:
            tmp = _Resp()
            self._run_in_thread(lambda: self._fire_wheel(tmp))
            response.success = tmp.success
            response.message = tmp.message
            self.get_logger().info(f"[fire_wheel] {response.message}")
            return response
        finally:
            self._busy = False

    def _fire_wheel(self, response):
        with self._lock:
            target_id = self.recog_rotate or int(
                self.get_parameter("default_rotate_target").value)
        if not (1 <= target_id <= WHEEL_LEAF_COUNT):
            response.success = False
            response.message = f"旋转靶编号非法: {target_id}"
            return response
        k = target_id - 1                       # leaf 索引
        phi = math.radians(WHEEL_PHI0_DEG - 72.0 * k)
        # 叶片在轮子本地坐标系的 (y, z)
        leaf_ly = WHEEL_LEAF_RADIUS * math.cos(phi)
        leaf_lz = WHEEL_LEAF_RADIUS * math.sin(phi)

        self.get_logger().info(
            f"准备打旋转靶 {target_id} 号（leaf_{k}，本地角 {math.degrees(phi):.0f}°，"
            f"本地 y={leaf_ly:+.3f} z={leaf_lz:+.3f}）")

        odom, age = self._wait_odom()
        if odom is None or age >= 1.0:
            response.success = False
            response.message = (
                f"本节点没有可用的 /odom（等待 {self.odom_wait:.1f}s，"
                f"最后年龄 {age:.1f}s）—— 请检查 /odom 是否在发布")
            self.get_logger().warn(response.message)
            return response
        self.get_logger().info(
            f"/odom 就绪（年龄 {age:.2f}s）位置=("
            f"{odom['x']:.3f},{odom['y']:.3f}) yaw={math.degrees(odom['yaw']):.1f}deg")

        t_start = time.monotonic()
        settle = 0
        need_settle = max(1, int(round(self.settle_sec / self.poll_dt)))
        last = None
        best_err = None
        it = 0
        prev_theta = None
        log_every = max(1, int(round(0.5 / self.poll_dt)))
        same_count = 0
        now_side = None          # +1 右(+y)、-1 左(-y)

        while time.monotonic() - t_start < self.aim_timeout:
            st = self._get_entity("target_wheel", timeout=1.0)
            if st is None:
                time.sleep(self.poll_dt)
                continue
            wx = st.pose.position.x
            wy = st.pose.position.y
            q = st.pose.orientation
            theta = quat_to_roll_x(q.x, q.y, q.z, q.w)

            # 叶片的世界角 psi（0 = +y 右侧，pi = -y 左侧）
            psi = (phi + theta) % (2 * math.pi)
            # 叶片竖直偏移 = r*sin(psi)；|sin| <= 半高/r 才落在水平窗口内
            bz_off = WHEEL_LEAF_RADIUS * math.sin(psi)
            if best_err is None or abs(bz_off) < best_err:
                best_err = abs(bz_off)

            # 选车头要瞄准的侧别。
            # ⚠️ 不能用"最近窗口"：叶片每转 180° 最近窗口就换一个，side 会来回翻，
            # 车头目标在 ±3.3° 之间反复跳，横向偏差永远收敛不了
            # （实测停在 +107mm，而竖直偏移其实已经到过 1mm）。
            # 改为：不在窗口时选"下一个将要到达的窗口"（沿旋转方向），稳定约 13 s；
            # 在窗口内时直接用叶片当前所在侧（cos(psi) 的符号），与预转的侧一致。
            def _near(x, c):
                return (x - c + math.pi) % (2 * math.pi) - math.pi

            in_window = abs(bz_off) <= WHEEL_LEAF_HALF_W
            if in_window:
                side = +1 if math.cos(psi) > 0 else -1
                d_ang = _near(psi, 0.0 if side > 0 else math.pi)
            else:
                # 沿旋转方向算到两个窗口还需转多少角，取先到的那个
                if (0.0 - psi) % (2 * math.pi) <= (math.pi - psi) % (2 * math.pi):
                    side, d_ang = +1, _near(psi, 0.0)
                else:
                    side, d_ang = -1, _near(psi, math.pi)

            # 车头目标：直接对准该侧的叶片位置（用 ±r，避免绕圈时来回甩）
            odom, _, _ = self._snapshot()
            if odom is not None:
                by_pred = wy + side * WHEEL_LEAF_RADIUS
                desired_yaw = math.atan2(by_pred - odom["y"], wx - odom["x"])
                yaw_err = ((desired_yaw - odom["yaw"] + math.pi)
                           % (2 * math.pi) - math.pi)
                dist = math.hypot(wx - odom["x"], by_pred - odom["y"])
                lateral_err = dist * math.sin(yaw_err)
                aligned = (abs(lateral_err) <= self.wheel_lat_tol
                           and abs(yaw_err) <= self.yaw_tol)
                # 提前把车头转到位（不因竖直条件不满足就停转 —— 这是上一版的 bug）
                if not aligned:
                    tw = Twist()
                    tw.angular.z = clamp(self.yaw_gain * yaw_err,
                                         -self.max_w, self.max_w)
                    self.cmd_pub.publish(tw)
                else:
                    self._stop()
                now_side = side
            else:
                aligned = False
                lateral_err = float("nan")
                yaw_err = float("nan")

            last = (f"轮角={math.degrees(theta) % 360:.0f}° 叶片竖直={bz_off*1000:+.0f}mm "
                    f"距窗口={math.degrees(d_ang):+.1f}° 侧={'右' if side > 0 else '左'} "
                    f"横向={lateral_err*1000:+.0f}mm 朝向误差={math.degrees(yaw_err):+.1f}°")

            # 过程日志
            if prev_theta is not None and abs(theta - prev_theta) < 1e-9:
                same_count += 1
            else:
                same_count = 0
            prev_theta = theta
            it += 1
            if it % log_every == 0:
                self.get_logger().info(
                    f"[过程] 轮角={math.degrees(theta) % 360:6.1f}° "
                    f"竖直={bz_off*1000:+5.0f}mm 距窗口={math.degrees(d_ang):+5.1f}° "
                    f"窗口={'OK' if in_window else '未到'} "
                    f"横向={lateral_err*1000:+5.0f}mm 对准={'是' if aligned else '否'} "
                    f"连续不变={same_count}")
                if same_count >= int(round(2.0 / self.poll_dt)):
                    self.get_logger().warn(
                        f"轮角已连续 {same_count} 次不变 —— 仿真很可能被暂停")

            if in_window and aligned:
                settle += 1
                self._stop()
                if settle >= need_settle:
                    self.get_logger().info(f"窗口命中：靶位 {target_id} → 开火（{last}）")
                    ok, note = self._fire_and_report()
                    response.success = ok
                    response.message = (
                        f"旋转靶 {target_id} 号已射击：叶片竖直偏移 {bz_off*1000:+.0f}mm、"
                        f"侧别 {'右' if side > 0 else '左'}、横向偏差 "
                        f"{lateral_err*1000:+.0f}mm；{note}")
                    self._publish_status("done" if ok else "error", detail=response.message)
                    return response
            else:
                settle = 0
            time.sleep(self.poll_dt)

        self._stop()
        response.success = False
        response.message = (
            f"旋转靶 {target_id} 号：{self.aim_timeout:.0f}s 内未等到窗口与对准同时成立 "
            f"（最后：{last if last else '未读到轮子状态'}；"
            f"最小竖直偏移 {best_err*1000 if best_err else 0:.0f}mm、"
            f"窗口阈值 {math.degrees(self.wheel_window):.0f}°）")
        self._publish_status("timeout", detail=response.message)
        return response

    # ---------------- D-04：移动靶 ----------------
    def on_fire_moving(self, _req, response, name):
        if self._busy:
            response.success = False
            response.message = "已有任务在进行"
            return response
        self._busy = True
        try:
            tmp = _Resp()
            self._run_in_thread(lambda: self._fire_moving(tmp, name))
            response.success = tmp.success
            response.message = tmp.message
            self.get_logger().info(f"[fire_moving {name}] {response.message}")
            return response
        finally:
            self._busy = False

    def _fire_moving(self, response, name):
        with self._lock:
            target_id = self.recog_moving or int(
                self.get_parameter("default_moving_target").value)
        if target_id not in self.code_order:
            response.success = False
            response.message = (
                f"移动靶编号 {target_id} 不在 moving_code_order={self.code_order} 里")
            return response
        idx = self.code_order.index(target_id)
        code_dx = MOVING_CODE_OFFSETS[min(idx, len(MOVING_CODE_OFFSETS) - 1)]

        self.get_logger().info(
            f"准备打 {name} 的 {target_id} 号码（板内 x 偏移 {code_dx:+.3f} m）")

        odom, age = self._wait_odom()
        if odom is None or age >= 1.0:
            response.success = False
            response.message = (
                f"本节点没有可用的 /odom（等待 {self.odom_wait:.1f}s，"
                f"最后年龄 {age:.1f}s）")
            self.get_logger().warn(response.message)
            return response

        t_start = time.monotonic()
        settle = 0
        need_settle = max(1, int(round(self.settle_sec / self.poll_dt)))
        last = None
        best_lat = None

        while time.monotonic() - t_start < self.aim_timeout:
            st = self._get_entity(name, timeout=1.0)
            odom, odom_t, _ = self._snapshot()
            if st is None or odom is None:
                time.sleep(self.poll_dt)
                continue
            bx = st.pose.position.x
            by = st.pose.position.y
            q = st.pose.orientation
            # 板的 yaw：板宽沿本地 y（0.18），绕 z 转 yaw 后映射到世界
            board_yaw = yaw_from_quat(q.z, q.w)
            # 码在板本地坐标 (0, code_dy)，旋转到世界
            off_x = -math.sin(board_yaw) * code_dx
            off_y = math.cos(board_yaw) * code_dx
            code_x = bx + off_x
            code_y = by + off_y

            dist = math.hypot(code_x - odom["x"], code_y - odom["y"])
            desired_yaw = math.atan2(code_y - odom["y"], code_x - odom["x"])
            yaw_err = (desired_yaw - odom["yaw"] + math.pi) % (2 * math.pi) - math.pi
            lateral_err = dist * math.sin(yaw_err)   # 车头方向的垂距（米）
            last = (f"板=({bx:.3f},{by:.3f}) 码=({code_x:.3f},{code_y:.3f}) "
                    f"距离={dist:.3f}m 横向={lateral_err*1000:+.0f}mm "
                    f"朝向误差={math.degrees(yaw_err):+.1f}°")
            if best_lat is None or abs(lateral_err) < best_lat:
                best_lat = abs(lateral_err)

            if abs(lateral_err) <= self.moving_pos_tol and abs(yaw_err) <= self.yaw_tol:
                settle += 1
                self._stop()
                if settle >= need_settle:
                    self.get_logger().info(f"窗口命中：{name} {target_id} 号 → 开火（{last}）")
                    ok, note = self._fire_and_report()
                    response.success = ok
                    response.message = f"{name} 的 {target_id} 号已射击（{last}）；{note}"
                    self._publish_status("done" if ok else "error", detail=response.message)
                    return response
            else:
                settle = 0
                tw = Twist()
                tw.angular.z = clamp(self.yaw_gain * yaw_err, -self.max_w, self.max_w)
                self.cmd_pub.publish(tw)
            time.sleep(self.poll_dt)

        self._stop()
        response.success = False
        response.message = (
            f"{name} 的 {target_id} 号：{self.aim_timeout:.0f}s 内未对准 "
            f"（最后：{last if last else '未读到板状态'}；"
            f"最小横向偏差 {best_lat*1000 if best_lat else 0:.0f}mm，"
            f"容差 {self.moving_pos_tol*1000:.0f}mm）")
        self._publish_status("timeout", detail=response.message)
        return response


def main():
    rclpy.init()
    node = TargetShooterNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node._stop()
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
