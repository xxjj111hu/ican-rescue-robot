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
        self.declare_parameter("roi_y_max_ratio", 0.55)
        self.declare_parameter("min_conf", 0.55)

        # ---- 闭环瞄准 ----
        self.declare_parameter("tol_px", 8.0)              # 约 1 环
        self.declare_parameter("settle_sec", 0.25)
        self.declare_parameter("aim_gain", 0.0025)         # rad/s per px
        self.declare_parameter("max_angular_speed", 0.30)
        self.declare_parameter("aim_timeout_sec", 12.0)
        self.declare_parameter("detect_max_age_sec", 0.6)

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

        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value),
            self.on_image, qos_profile_sensor_data)
        self.create_subscription(
            String, str(self.get_parameter("shot_topic").value), self.on_shot, 10)

        self.fire_client = self.create_client(
            Trigger, str(self.get_parameter("fire_service").value))
        self.create_service(
            Trigger, str(self.get_parameter("service_name").value), self.on_fire_ring)

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
        det, det_t, _ = self._snapshot()
        if det is None or not det["ok"]:
            response.success = False
            response.message = "未检测到环形靶（检查相机、光照或 roi_y_max_ratio）"
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
        # 实测该服务约 6 s 才返回（每次调用生成一颗新子弹，无弹药上限）。
        # 原来只等 3 s，导致成功射击被误判为失败。
        while not future.done() and (time.monotonic() - t_fire) < self.fire_service_timeout:
            time.sleep(0.05)
        res = None
        if future.done():
            try:
                res = future.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"射击服务调用失败: {exc}")
            if res is not None:
                self.get_logger().info(
                    f"/shooter/shoot 返回 success={res.success} message='{res.message}' "
                    f"（耗时 {time.monotonic() - t_fire:.1f}s）")
        else:
            self.get_logger().warn(
                f"/shooter/shoot {self.fire_service_timeout:.0f}s 未返回，继续等 /shooter/shot")

        # 等 /shooter/shot 确认发射
        # 实测：该话题**每发都会发布**，payload 是子弹名（如 bullet_08ac4d6b7a），
        # 不是只在命中时发 —— 因此可用作"确实射出了"的确认。
        # 注意竞态：消息可能正好落在最后一轮轮询的 sleep 空档里（实测差 10 ms
        # 被误判为失败），所以超时后再宽限复查一次。
        ok_shot = False
        bullet = ""
        deadline = time.monotonic() + self.shot_wait
        while time.monotonic() < deadline:
            _, _, n = self._snapshot()
            if n > shot0:
                ok_shot = True
                break
            time.sleep(0.03)
        if not ok_shot:
            time.sleep(0.3)          # 宽限复查，消除轮询竞态
            _, _, n = self._snapshot()
            ok_shot = n > shot0
        if ok_shot:
            with self._lock:
                bullet = self.last_shot_name

        if ok_shot:
            response.success = True
            response.message = (f"已射击并确认射出（{bullet}）；dx={last_dx:+.1f}px，"
                                f"估算 {rings:.1f} 环，瞄准耗时 {aim_sec:.1f}s")
        else:
            response.success = False
            response.message = (f"射击服务已调用，但 {self.shot_wait:.1f}s（含 0.3s 宽限）"
                                f"内仍未收到 /shooter/shot（dx={last_dx:+.1f}px）")
        self.get_logger().info(response.message)
        self._publish_status("done" if ok_shot else "error", detail=response.message)
        return response


def main():
    rclpy.init()
    node = RingShooterNode()
    # 服务线程阻塞时，图像回调需在另一线程继续跑 -> 多线程执行器
    executor = MultiThreadedExecutor(num_threads=3)
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
