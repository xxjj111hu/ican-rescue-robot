"""环形靶瞄准与射击节点（D-01 瞄准算法 + D-02 环数刷分）。

职责
----
1. 订阅 /camera/image，用 HSV 三色质心法检测十环靶纸，发布
   /ring_target/center_offset（PointStamped，x/y = 像素偏移，z = 置信度），
   与官方示例 ring_aim_node.py 的话题一致，便于对照。
2. 提供 Trigger 服务 /ican_shoot/fire_ring（**阻塞式**，最长约 15 s）：
     ① 闭环转向把靶心拉到画面中心（比例控制，容差按实测环距标定）
     ② 连续稳定若干帧后才停稳开火（避免在摆动中开枪）
     ③ 调 /shooter/shoot（std_srvs/Trigger），等 /shooter/shot 确认射出
     ④ 返回 success + 明细（最终偏移、稳定帧数、估算环数）

为什么服务是阻塞的
------------------
ROS 2 的 Python 服务是同步的：回调返回的 response 会立刻发出去，之后改它无效。
所以瞄准/射击这种要数秒的动作，只能在回调里同步做完再返回。
为此 main() 用 MultiThreadedExecutor：服务线程阻塞时，图像回调在另一线程继续
更新检测结果，服务线程用 time.sleep(0.05) 轮询即可。

参数依据（实测，见 交付包/射击/脚本/test_ring_detect.py）
--------------------------------------------------------
    1.54 m 处靶半径约 41 px，覆盖 6~10 环共 5 个环带 -> 每环约 8.2 px
    >=8 环: |dx| <= 24.6 px    >=9 环: <= 16.4 px    10 环: <= 8.2 px
    默认容差 8 px（约 1 环），并需连续稳定 0.25 s。

铁律（判分细则 §1.3(3)）
------------------------
每个任务点**只认第一次命中**。所以在确认对准之前绝不能开火，
宁可多瞄几秒放弃也不要把该点成绩钉死在一个偏靶上。

计时一律用墙钟 + time.sleep：use_sim_time 下若 /clock 停滞，ROS 定时器不会触发，
节点会静默不动作（本项目踩过这个坑）。
"""
from __future__ import annotations

import json
import threading
import time

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Twist
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ican_shoot.ring_detect import detect_ring, draw_debug, ring_score_from_offset


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class RingShooterNode(Node):
    def __init__(self):
        super().__init__("ring_shooter_node")

        # ---- 话题与服务 ----
        for name, default in (
            ("image_topic", "/camera/image"),
            ("offset_topic", "/ring_target/center_offset"),
            ("debug_topic", "/ring_target/debug_image"),
            ("cmd_vel_topic", "/cmd_vel"),
            ("fire_service", "/shooter/shoot"),
            ("shot_topic", "/shooter/shot"),
            ("status_topic", "/ican_shoot/status"),
            ("service_name", "/ican_shoot/fire_ring"),
        ):
            self.declare_parameter(name, default)

        # ---- 视觉 ----
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("debug_every", 10)
        self.declare_parameter("roi_y_max_ratio", 0.53)
        self.declare_parameter("min_conf", 0.55)

        # ---- 闭环瞄准 ----
        self.declare_parameter("tol_px", 8.0)              # 约 1 环
        self.declare_parameter("settle_sec", 0.25)
        self.declare_parameter("aim_gain", 0.0025)         # rad/s per px
        self.declare_parameter("max_angular_speed", 0.30)
        self.declare_parameter("aim_timeout_sec", 12.0)
        self.declare_parameter("detect_max_age_sec", 0.6)
        # 服务被调用时若还没检到靶，先等这么久再判失败（联调时任务点刚到位常见）
        self.declare_parameter("detect_wait_sec", 5.0)

        # ---- 射击 ----
        self.declare_parameter("dry_run", False)
        self.declare_parameter("shot_wait_sec", 8.0)
        # 实测：/shooter/shoot 调用后约 6 秒才返回并发布 /shooter/shot，
        # 所以服务等待必须 >= 10 s，否则会把成功射击误判成失败。
        self.declare_parameter("fire_service_timeout_sec", 12.0)
        self.declare_parameter("holster_settle_sec", 0.4)

        self.bridge = CvBridge()
        self.min_conf = float(self.get_parameter("min_conf").value)
        self.roi_y_ratio = float(self.get_parameter("roi_y_max_ratio").value)
        self.publish_debug = bool(self.get_parameter("publish_debug_image").value)
        self.debug_every = max(1, int(self.get_parameter("debug_every").value))

        self.tol_px = float(self.get_parameter("tol_px").value)
        self.settle_sec = float(self.get_parameter("settle_sec").value)
        self.aim_gain = float(self.get_parameter("aim_gain").value)
        self.max_w = float(self.get_parameter("max_angular_speed").value)
        self.aim_timeout = float(self.get_parameter("aim_timeout_sec").value)
        self.detect_max_age = float(self.get_parameter("detect_max_age_sec").value)
        self.detect_wait = float(self.get_parameter("detect_wait_sec").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.shot_wait = float(self.get_parameter("shot_wait_sec").value)
        self.fire_service_timeout = float(
            self.get_parameter("fire_service_timeout_sec").value)
        self.holster_settle = float(self.get_parameter("holster_settle_sec").value)

        # ---- 发布/订阅 ----
        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10)
        self.offset_pub = self.create_publisher(
            PointStamped, str(self.get_parameter("offset_topic").value), 10)
        status_qos = QoSProfile(depth=10)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.VOLATILE
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), status_qos)
        self.debug_pub = None
        if self.publish_debug:
            self.debug_pub = self.create_publisher(
                Image, str(self.get_parameter("debug_topic").value), 10)

        # 回调组：默认组是 MutuallyExclusive，服务回调一阻塞就会把图像回调一起卡住，
        # 瞄准线程只能读到冻结的旧检测结果。把订阅放独立 Reentrant 组即可并发。
        # 注意：这里必须用 MutuallyExclusiveCallbackGroup，不要用 Reentrant。
        # 实测 Reentrant 组会让组内的订阅互相饿死：目标节点订阅了 /odom 却
        # 永远收不到回调，环形靶节点的 det 12 秒不更新（dx 恒为 -46.8px），
        # 而服务回调（独立组）一切正常。Reentrant 的 can_execute() 恒为 True，
        # 多线程执行器可能反复挑同一个实体，导致组内其它订阅长期拿不到执行机会。
        # 用独立的 MutuallyExclusive 组同样能达到「不被服务回调串行化」的目的。
        self._io_group = MutuallyExclusiveCallbackGroup()
        self._srv_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value),
            self.on_image, qos_profile_sensor_data, callback_group=self._io_group)
        self.create_subscription(
            String, str(self.get_parameter("shot_topic").value), self.on_shot, 10,
            callback_group=self._io_group)

        self.fire_client = self.create_client(
            Trigger, str(self.get_parameter("fire_service").value),
            callback_group=self._io_group)
        self.create_service(
            Trigger, str(self.get_parameter("service_name").value), self.on_fire_ring,
            callback_group=self._srv_group)

        # ---- 共享状态（服务线程读，图像线程写）----
        self.det = None
        self.det_t = 0.0
        self.det_count = 0
        self.shot_msgs = 0
        self.last_shot_name = ""
        self._busy = False
        self._lock = threading.Lock()

        self.get_logger().info(
            "环形靶射击节点就绪："
            f"{self.get_parameter('image_topic').value} -> "
            f"{self.get_parameter('offset_topic').value} | "
            f"容差 {self.tol_px:.0f}px、稳定 {self.settle_sec:.2f}s、"
            f"最长瞄准 {self.aim_timeout:.0f}s | "
            f"服务 {self.get_parameter('service_name').value} -> "
            f"{self.get_parameter('fire_service').value}"
            f"{' | dry_run（不实弹）' if self.dry_run else ''}"
        )

    # ---------------- 视觉（图像线程） ----------------
    def on_image(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"图像转换失败: {exc}")
            return
        self.det_count += 1
        det = detect_ring(bgr, roi_y_max_ratio=self.roi_y_ratio)
        if det["ok"] and det["conf"] < self.min_conf:
            det["ok"] = False
        with self._lock:
            self.det = det
            self.det_t = time.monotonic()

        # 周期打印检测状态：联调时用它判断「没检到靶」到底是没图像、
        # 置信度不够、还是靶不在视野里
        if self.det_count % 30 == 0:      # 约 1 s 一次（相机约 30 Hz）
            if det["ok"]:
                self.get_logger().info(
                    f"[检测] ok dx={det['dx']:+.1f}px dy={det['dy']:+.1f}px "
                    f"r={det['radius']:.0f} conf={det['conf']:.2f} "
                    f"色={sorted(det['parts'].keys())}")
            else:
                self.get_logger().info(
                    f"[检测] 未检出（色块={sorted(det['parts'].keys())}，"
                    f"min_conf={self.min_conf:.2f}）")

        if det["ok"]:
            out = PointStamped()
            out.header = msg.header
            out.header.frame_id = "camera_screen"
            out.point.x = float(det["dx"])
            out.point.y = float(det["dy"])
            out.point.z = float(det["conf"])
            self.offset_pub.publish(out)

        if self.debug_pub is not None and (self.det_count % self.debug_every == 0):
            try:
                dbg = draw_debug(bgr, det)
                m = self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8")
                m.header = msg.header
                self.debug_pub.publish(m)
            except Exception:  # noqa: BLE001
                pass

    def on_shot(self, msg):
        with self._lock:
            self.shot_msgs += 1
            self.last_shot_name = msg.data
        self.get_logger().info(f"[/shooter/shot] {msg.data}")

    def _snapshot(self):
        with self._lock:
            return self.det, self.det_t, self.shot_msgs

    def _publish_status(self, state, **kw):
        payload = {"state": state, "t": round(time.monotonic(), 3)}
        payload.update(kw)
        m = String()
        m.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(m)

    def _stop(self):
        self.cmd_pub.publish(Twist())

    # ---------------- 服务（服务线程，阻塞） ----------------
    def on_fire_ring(self, _request, response):
        if self._busy:
            response.success = False
            response.message = "已有瞄准/射击任务在进行"
            return response
        self._busy = True
        try:
            return self._run_fire_sequence(response)
        finally:
            self._stop()
            self._busy = False

    def _run_fire_sequence(self, response):
        # 先给检测几秒钟：任务节点刚导航到位时可能刚好没有新的图像/靶还没进视野，
        # 一调就判失败会把「还没看到」误判成「看不到」（实测联调时任务点1 就是这样）。
        det, det_t, _ = self._snapshot()
        if det is None or not det["ok"]:
            self.get_logger().info(
                f"暂未检测到环形靶，等待最多 {self.detect_wait:.1f}s"
                f"（当前 det={'None' if det is None else 'ok=' + str(det['ok'])}）")
            t_wait = time.monotonic()
            while time.monotonic() - t_wait < self.detect_wait:
                det, det_t, _ = self._snapshot()
                if det is not None and det["ok"]:
                    break
                time.sleep(0.05)
        if det is None or not det["ok"]:
            detail = "从未收到相机图像" if det is None else (
                f"有图像但未检出靶（ok=False, conf={det.get('conf', 0):.2f}"
                f" < min_conf={self.min_conf:.2f}）"
                if det.get("conf") is not None else "有图像但未检出靶")
            response.success = False
            response.message = (
                f"未检测到环形靶（等待 {self.detect_wait:.1f}s 后仍无有效检测；{detail}）")
            self.get_logger().warn(response.message)
            return response

        self.get_logger().info("开始闭环瞄准环形靶。")
        t0 = time.monotonic()
        settle_frames = max(1, int(round(self.settle_sec / 0.05)))
        stable = 0
        last_dx = None
        rings = None
        dx_hist = []

        while True:
            now = time.monotonic()
            if now - t0 > self.aim_timeout:
                self._stop()
                response.success = False
                response.message = (
                    f"瞄准超时 {now - t0:.1f}s，最后偏移 "
                    f"{last_dx:+.1f}px" if last_dx is not None
                    else f"瞄准超时 {now - t0:.1f}s（始终未检出靶）")
                self.get_logger().warn(response.message)
                self._publish_status("timeout", detail=response.message)
                return response

            det, det_t, _ = self._snapshot()
            if det is None or not det["ok"] or (now - det_t) > self.detect_max_age:
                self._stop()
                self._publish_status("aim", note="no_detection",
                                     elapsed=round(now - t0, 2))
                time.sleep(0.05)
                continue

            dx = det["dx"]
            last_dx = dx
            rings = ring_score_from_offset(dx, det["radius"])
            dx_hist.append(round(dx, 1))
            if len(dx_hist) > 60:
                dx_hist.pop(0)
            self._publish_status("aim", dx=round(dx, 1), conf=round(det["conf"], 2),
                                 rings=round(rings, 1), elapsed=round(now - t0, 2))

            if abs(dx) <= self.tol_px:
                stable += 1
                self._stop()
                if stable >= settle_frames:
                    break
            else:
                stable = 0
                tw = Twist()
                tw.angular.z = clamp(-self.aim_gain * dx, -self.max_w, self.max_w)
                self.cmd_pub.publish(tw)
            time.sleep(0.05)

        self._stop()
        aim_sec = time.monotonic() - t0
        self.get_logger().info(
            f"瞄准到位：dx={last_dx:+.1f}px（容差 {self.tol_px:.0f}px），"
            f"稳定 {stable} 帧，估算 {rings:.1f} 环，耗时 {aim_sec:.1f}s")

        # 停稳后再开火（避免残余角速度影响弹道）
        time.sleep(self.holster_settle)
        self._stop()

        if self.dry_run:
            response.success = True
            response.message = (f"dry_run：瞄准完成 dx={last_dx:+.1f}px，"
                                f"估算 {rings:.1f} 环，耗时 {aim_sec:.1f}s，未实弹")
            self._publish_status("dry_run", detail=response.message)
            return response

        if not self.fire_client.wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = "找不到射击服务（检查 /shooter/shoot 是否存在）"
            self._publish_status("error", detail=response.message)
            return response

        _, _, shot0 = self._snapshot()
        self.get_logger().info("调用 /shooter/shoot ...")
        self._publish_status("firing", dx=round(last_dx, 1))
        future = self.fire_client.call_async(Trigger.Request())
        t_fire = time.monotonic()

        # 只短暂观察服务是否**立刻**报错，不做长时间阻塞。
        # 实测该服务的响应时间极不稳定：5.9 s / 19.9 s 都出现过（且 12 s 内不返回），
        # 等服务返回会把状态机拖死，而射击是否计分由 Gazebo 物理判定，
        # 主程序并不需要等待这个确认。
        probe_sec = min(2.0, self.fire_service_timeout)
        while not future.done() and (time.monotonic() - t_fire) < probe_sec:
            time.sleep(0.05)

        svc_ok = True
        svc_note = ""
        if future.done():
            try:
                res = future.result()
            except Exception as exc:  # noqa: BLE001
                res = None
                svc_ok = False
                svc_note = f"服务调用异常: {exc}"
            if res is not None:
                svc_ok = bool(res.success)
                svc_note = f"服务返回 success={res.success} message='{res.message}'"
                self.get_logger().info(f"{svc_note}（耗时 {time.monotonic() - t_fire:.1f}s）")
        else:
            svc_note = f"服务 {probe_sec:.0f}s 内未返回（该服务响应时间不稳定，属已知现象）"
            self.get_logger().info(svc_note)

        # 再短暂等一下 /shooter/shot 作为"已射出"的佐证；等不到也不判失败
        ok_shot = False
        bullet = ""
        deadline = time.monotonic() + min(3.0, self.shot_wait)
        while time.monotonic() < deadline:
            _, _, n = self._snapshot()
            if n > shot0:
                ok_shot = True
                break
            time.sleep(0.03)
        if ok_shot:
            with self._lock:
                bullet = self.last_shot_name

        if ok_shot:
            response.success = True
            response.message = (f"已射击（confirmed {bullet}）；dx={last_dx:+.1f}px，"
                                f"估算 {rings:.1f} 环，瞄准耗时 {aim_sec:.1f}s")
        elif svc_ok:
            # 服务没有报错，只是确认消息还没到 —— 视为已发射，不阻塞流程
            response.success = True
            response.message = (f"已发射（未等到 /shooter/shot 确认，属已知延迟）；"
                                f"dx={last_dx:+.1f}px，估算 {rings:.1f} 环"
                                f"{'；' + svc_note if svc_note else ''}")
        else:
            response.success = False
            response.message = (f"射击服务报错：{svc_note}；"
                                f"dx={last_dx:+.1f}px，估算 {rings:.1f} 环")
        self.get_logger().info(response.message)
        self._publish_status("done" if response.success else "error",
                             detail=response.message)
        return response


def main():
    rclpy.init()
    node = RingShooterNode()
    # 服务线程阻塞时，图像回调需在另一线程继续跑 -> 多线程执行器
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
