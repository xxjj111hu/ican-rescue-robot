"""识别节点：订阅 /camera/image -> 推理 -> 发布识别结果与裁判话术。

两种模式：
  mode:=classifier  5 类分类模型（runs_cls/best.pt），按 auto_collect 采集的数据训练，推荐
  mode:=yolo        旧版 YOLO 检测权重（runs/ican_weights/best.pt），保留兼容

话题：
  订阅 /camera/image          sensor_msgs/Image     （QoS: sensor_data）
  订阅 /odom                  nav_msgs/Odometry     （距离门控用，可关）
  发布 /detection/result      std_msgs/String（JSON，逐帧输出，用于验证 >=10Hz）
  发布 /referee/detections    std_msgs/String（话术，QoS: RELIABLE，进点前上报）
  发布 /ican_recognize/status std_msgs/String（JSON，门控/发布/超时状态，给 mission 用）
  发布 /ican_recognize/debug_image sensor_msgs/Image（可选）

距离门控（B-03/B-05 补全）
------------------------------------------------------------------
实测识别只在相机到贴图约 0.8 m 时可靠（0.6m 2/5、0.8m 15/15、0.9m 4/5、1.4m 2/5）。
本节点订阅 /odom，实时计算到最近地面贴图点的距离，**只有落在
[gate_min_dist, gate_max_dist]（默认 0.60~0.90 m）窗口内才接受投票并发布话术**；
离开窗口清空投票历史与已发布锁存，避免在远处（准确率仅 40%）
就发出错误话术。窗口内超过 detect_timeout_sec 仍无法发布时，发
status=timeout 通知 mission 继续下一点，保证全程不中断。
"""
import json
import os
import time
from collections import Counter, deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from ican_referee.referee_comm import RefereeComm
from std_msgs.msg import Int32, String

from ican_recognize.scripts_table import (
    CLASS_ORDER,
    CLASS_SCRIPT,
    target_ids,
    validate_scripts,
    validate_targets,
)

# 5 个地面贴图的真实中心坐标（与 land_shoot.world / goto_pose.py 一致）
GROUND_POINTS = {
    0: (1.115218, -1.356470),
    1: (0.030951, -1.359743),
    2: (0.027137, -2.700000),
    3: (1.120240, -1.913890),
    4: (1.120112, -0.784766),
}

DEFAULT_MODEL_DIR = os.path.expanduser("~/.ros/ican_recognize")


def _parse_class_conf(text):
    """把 "trapped_people:0.60,damaged_vehicle:0.60" 解析成 dict。"""
    out = {}
    for item in str(text).split(","):
        item = item.strip()
        if not item or ":" not in item:
            continue
        name, _, val = item.partition(":")
        try:
            out[name.strip()] = float(val)
        except ValueError:
            continue
    return out


class RecognizeNode(Node):
    def __init__(self):
        super().__init__("ican_recognize_node")
        self.declare_parameter("mode", "yolo")   # classifier | yolo
        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("detections_topic", "/referee/detections")
        self.declare_parameter("result_topic", "/detection/result")
        self.declare_parameter("status_topic", "/ican_recognize/status")
        # 射击靶位话题：发布「识别类别 -> 该打哪个靶」的结构化结果（判分细则表 7）
        self.declare_parameter("target_topic", "/ican_recognize/target")
        self.declare_parameter("publish_result", True)
        self.declare_parameter("publish_status", True)
        self.declare_parameter("debug_image_topic", "/ican_recognize/debug_image")
        # 默认关闭：实测开启时每帧要多花约 127 ms（复制 1.9MB 图像 + 画框 +
        # tobytes + rclpy 序列化发布），把整体频率从 25 Hz 拖到 7 Hz。
        # 需要肉眼检查时用 debug_every 控制降频发布。
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("debug_every", 10)

        # 模型路径：默认放 ~/.ros/ican_recognize/，可用环境变量或参数覆盖
        self.declare_parameter("model_dir", DEFAULT_MODEL_DIR)
        self.declare_parameter(
            "cls_model_path",
            os.environ.get("ICAN_CLS_MODEL", os.path.join(DEFAULT_MODEL_DIR, "best.pt")),
        )
        self.declare_parameter(
            "model_path",
            os.environ.get("ICAN_YOLO_MODEL", os.path.join(DEFAULT_MODEL_DIR, "best.pt")),
        )
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("conf_thres", 0.20)
        # 多帧投票窗口：取最近 N 帧的众数，抑制单帧抖动
        self.declare_parameter("vote_window", 7)
        self.declare_parameter("vote_min", 4)
        # 发布话术的置信度下限：低于这个值宁可晚一点发，也不发错的
        self.declare_parameter("min_conf_publish", 0.45)
        # 易混淆类别可单独调整阈值。默认**不做覆盖**（空串）。
        # 说明：按《判分细则》§1.3，识别失败时该点到达分为 0 且射击也不计分；
        # 而「发错」与「不发」得分完全相同（都是 0）。因此抬高阈值"宁可不发"
        # 没有任何计分收益，只会放弃"猜对拿 10 分"的机会 —— 默认保持不覆盖。
        self.declare_parameter("class_min_conf", "")
        self.declare_parameter("iou_thres", 0.45)
        self.declare_parameter("min_publish_interval", 2.0)
        self.declare_parameter("fps_log_every", 30)

        # 距离门控（窗口依据实测标定，见 交付包/识别/鲁棒性测试报告.md「门控实测」）
        # 实测命中：0.6m 2/5、0.7m 3/5、0.8m 15/15、0.9m 4/5、1.1m 3/5、1.4m 2/5
        self.declare_parameter("gate_enable", True)
        self.declare_parameter("gate_min_dist", 0.60)
        self.declare_parameter("gate_max_dist", 0.90)
        # 任务节点可指定「当前去几号点」，避免门控被邻近点位贴图抢走
        # （实测 standoff 2/3/4 到点位1贴图比到本点位更近，0=按最近贴图自动）
        self.declare_parameter("expected_point_topic",
                               "/ican_recognize/expected_point")
        self.declare_parameter("detect_timeout_sec", 5.0)
        # 超时时强制发布当前最优猜测。
        # 依据：发错与不发都是 0 分，但发对了能拿 10 分并保住该点射击分，
        # 所以宁可赌一次也不要空手放弃。
        self.declare_parameter("force_publish_on_timeout", True)
        self.declare_parameter("status_log_every", 5.0)

        self.bridge = CvBridge()
        self.mode = str(self.get_parameter("mode").value).lower()
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.conf = float(self.get_parameter("conf_thres").value)
        self.iou = float(self.get_parameter("iou_thres").value)
        self.min_interval = float(self.get_parameter("min_publish_interval").value)
        self.fps_log_every = int(self.get_parameter("fps_log_every").value)
        self.publish_result = bool(self.get_parameter("publish_result").value)
        self.publish_status = bool(self.get_parameter("publish_status").value)

        self._class_min_conf = _parse_class_conf(self.get_parameter("class_min_conf").value)

        issues = validate_scripts()
        if issues:
            for msg in issues:
                self.get_logger().error(f"[话术自检] {msg}")
        else:
            self.get_logger().info("[话术自检] 5 条话术均含裁判匹配关键词，通过")

        t_issues = validate_targets()
        if t_issues:
            for msg in t_issues:
                self.get_logger().error(f"[靶位自检] {msg}")
        else:
            self.get_logger().info(
                "[靶位自检] 表 7 映射通过：旋转靶 1~5 = 坍塌/滑坡/被困/受损/营地；"
                "移动靶 6=坍塌·滑坡, 7=被困·受损, 8=营地"
            )

        self.class_names = {i: n for i, n in enumerate(CLASS_ORDER)}
        if self.mode == "classifier":
            self._init_classifier()
        else:
            self._init_yolo()

        image_topic = str(self.get_parameter("image_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        det_topic = str(self.get_parameter("detections_topic").value)
        result_topic = str(self.get_parameter("result_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        target_topic = str(self.get_parameter("target_topic").value)
        debug_topic = str(self.get_parameter("debug_image_topic").value)
        self.publish_debug = bool(self.get_parameter("publish_debug_image").value)
        self._debug_every = max(1, int(self.get_parameter("debug_every").value))

        det_qos = QoSProfile(depth=10)
        det_qos.reliability = ReliabilityPolicy.RELIABLE
        det_qos.durability = DurabilityPolicy.VOLATILE
        # 裁判话术统一由 ican_referee 模块发送（E-01）：
        # 话题名/QoS/格式校验都有单一事实来源
        self.referee = RefereeComm(
            self,
            detections_topic=det_topic,
            enable_subscribe_detections=False,
            enable_subscribe_shot=False,
            logger_prefix="[裁判]",
        )
        self.result_pub = self.create_publisher(String, result_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, det_qos)
        self.target_pub = self.create_publisher(String, target_topic, det_qos)
        self.debug_pub = (
            self.create_publisher(Image, debug_topic, 10) if self.publish_debug else None
        )
        self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)

        # 任务节点指定的期望点位（0 表示不指定、按最近贴图自动）
        self._expected_point = 0
        self.gate_enable = bool(self.get_parameter("gate_enable").value)
        self.gate_min = float(self.get_parameter("gate_min_dist").value)
        self.gate_max = float(self.get_parameter("gate_max_dist").value)
        self.detect_timeout = float(self.get_parameter("detect_timeout_sec").value)
        self.force_publish = bool(self.get_parameter("force_publish_on_timeout").value)
        self.status_log_every = float(self.get_parameter("status_log_every").value)

        self._odom = None
        self._in_window = False
        self._window_point = None
        self._window_enter_t = 0.0
        self._timeout_reported = False
        self._last_status_t = 0.0
        self._published_this_approach = False
        # 本次进窗以来见过的最高置信度检测（不论是否过阈值），用于超时兜底赌一次
        self._best_seen = (None, 0.0)

        # 任务节点指定当前点位，避免门控被邻近贴图抢走（见 on_expected_point）
        self.create_subscription(
            Int32, str(self.get_parameter("expected_point_topic").value),
            self.on_expected_point, 10)
        if self.gate_enable:
            self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        else:
            self.get_logger().warn(
                "[门控] 距离门控已关闭：将在任意距离发布话术，远端误报风险显著上升"
            )

        self._vote_window = int(self.get_parameter("vote_window").value)
        self._vote_min = int(self.get_parameter("vote_min").value)
        self._min_conf_pub = float(self.get_parameter("min_conf_publish").value)
        self._hist = deque(maxlen=max(1, self._vote_window))
        self._last_pub_time = 0.0
        self._last_pub_class = None
        self._frame_count = 0
        self._t0 = time.time()
        # 用于计算「实时 FPS」的滑动时间窗（不含窗口外的空闲期）
        self._frame_times = deque(maxlen=max(2, self.fps_log_every))
        self.get_logger().info(
            f"识别节点就绪（{self.mode}）: {image_topic} -> {det_topic} "
            f"| 门控={self.gate_enable} 窗口=[{self.gate_min:.2f},{self.gate_max:.2f}]m "
            f"| 易混淆类阈值={self._class_min_conf or '无'} "
            f"| 调试图={'每 %d 帧' % self._debug_every if self.publish_debug else '关闭'}"
        )

    # ---------- 模型加载 ----------
    def _check_model_file(self, path):
        if not os.path.isfile(path):
            model_dir = os.path.expanduser(str(self.get_parameter("model_dir").value))
            key = "cls_model_path" if self.mode == "classifier" else "model_path"
            self.get_logger().error(
                f"模型文件不存在: {path}\n"
                f"  请先准备模型，例如：\n"
                f"    mkdir -p {model_dir}\n"
                f"    cp 交付包/识别/模型/best.pt {model_dir}/best.pt\n"
                f"  或用参数指定：--ros-args -p {key}:=/绝对路径/best.pt"
            )
            raise FileNotFoundError(path)

    def _init_classifier(self):
        import torch
        import torch.nn as nn
        from torchvision import models

        path = os.path.expanduser(str(self.get_parameter("cls_model_path").value))
        self._check_model_file(path)
        ck = torch.load(path, map_location="cpu")
        self.torch = torch
        self.classes = ck.get("classes", CLASS_ORDER)
        self.crop = tuple(ck.get("crop", (40, 360, 760, 680)))
        backbone = ck.get("backbone", "resnet18")
        if backbone == "mobilenet_v3_small":
            m = models.mobilenet_v3_small(weights=None)
            m.classifier[3] = nn.Linear(m.classifier[3].in_features, len(self.classes))
        else:
            m = models.resnet18(weights=None)
            m.fc = nn.Linear(m.fc.in_features, len(self.classes))
        m.load_state_dict(ck["state_dict"])
        m.eval()
        self.cls_model = m
        self.mean = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
        self.std = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)
        self.get_logger().info(
            f"分类模型加载完成: {path}  backbone={backbone}  类别={self.classes}  crop={self.crop}"
        )

    def _init_yolo(self):
        from ultralytics import YOLO

        path = os.path.expanduser(str(self.get_parameter("model_path").value))
        self._check_model_file(path)
        self.yolo = YOLO(path)
        raw = getattr(self.yolo, "names", None)
        if isinstance(raw, dict):
            self.class_names = {int(k): v for k, v in raw.items()}
        for i, n in enumerate(CLASS_ORDER):
            self.class_names.setdefault(i, n)
        self.get_logger().info(f"YOLO 模型加载完成: {path}")

    # ---------- 距离门控 ----------
    def on_odom(self, msg):
        p = msg.pose.pose.position
        self._odom = (float(p.x), float(p.y))

    def on_expected_point(self, msg):
        """任务节点告知当前要去的点位；0 表示不指定。"""
        try:
            pid = int(msg.data)
        except Exception:  # noqa: BLE001
            return
        if pid != self._expected_point:
            self._expected_point = pid
            if pid:
                self.get_logger().info(
                    f"[门控] 任务节点指定当前点位 {pid}，"
                    f"门控距离改按该点位贴图计算")
            else:
                self.get_logger().info("[门控] 取消指定点位，恢复按最近贴图判定")

    def _nearest_point(self):
        """返回 (点位编号, 距离)；无 odom 时返回 (None, None)。"""
        if self._odom is None:
            return None, None
        x, y = self._odom
        # 任务节点指定了点位就只算那一个：否则会被"离得更近的邻近贴图"抢走，
        # 导致该点位永远进不了识别窗口（实测点位 3 就是这样丢的）。
        if self._expected_point and self._expected_point in GROUND_POINTS:
            gx, gy = GROUND_POINTS[self._expected_point]
            d = ((x - gx) ** 2 + (y - gy) ** 2) ** 0.5
            return self._expected_point, d
        best_id, best_d = None, float("inf")
        for pid, (gx, gy) in GROUND_POINTS.items():
            d = ((x - gx) ** 2 + (y - gy) ** 2) ** 0.5
            if d < best_d:
                best_id, best_d = pid, d
        return best_id, best_d

    def _publish_status(self, state, **kwargs):
        if not self.publish_status:
            return
        now = time.time()
        # idle / in_window 这类高频状态限流，published/timeout 立即发
        if state in ("idle", "in_window") and (now - self._last_status_t) < self.status_log_every:
            return
        self._last_status_t = now
        payload = {"state": state, "t": round(now, 3)}
        payload.update(kwargs)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def _publish_target(self, cls_name, conf, point, dist, source):
        """发布「识别类别 -> 该打哪个靶」的结构化结果。

        依据《判分细则 V1.0》第 5 章 表 7：
          旋转靶 1~5 号分别对应 坍塌废墟/滑坡塌方/被困人员/受损车辆/救援营地
          移动靶 6 号对应 坍塌废墟、滑坡塌方；7 号对应 被困人员、受损车辆；8 号对应 救援营地

        射击节点订阅本话题即可知道该瞄准哪个靶位。
        注意细则 1.3(3)「首击锁定」：命中第一发后该点位不再计分，
        所以打错靶的代价就是丢掉该点位 —— 靶位号必须来自识别结果，不能猜。
        """
        if cls_name is None:
            return
        ids = target_ids(cls_name)
        if ids.get("rotate") is None and ids.get("moving") is None:
            return
        payload = {
            "class": cls_name,
            "rotate_target": ids.get("rotate"),
            "moving_target": ids.get("moving"),
            "conf": None if conf is None else round(float(conf), 4),
            "point": point,
            "dist": None if dist is None else round(float(dist), 3),
            "source": source,
            "script": CLASS_SCRIPT.get(cls_name),
            "t": round(time.time(), 3),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.target_pub.publish(msg)

    def _entered_window(self, pid, dist):
        self._in_window = True
        self._window_point = pid
        self._window_enter_t = time.time()
        self._timeout_reported = False
        self._published_this_approach = False
        self._best_seen = (None, 0.0)
        self._hist.clear()          # 清掉窗口外的旧投票，避免污染
        self._last_pub_class = None
        self.get_logger().info(f"[门控] 进入识别窗口：点位 {pid}，距离 {dist:.2f} m")
        self._publish_status("in_window", point=pid, dist=round(dist, 3))

    def _left_window(self):
        if self._in_window:
            self.get_logger().info(
                f"[门控] 离开识别窗口（点位 {self._window_point}），"
                f"本次{'已' if self._published_this_approach else '未'}发布话术"
            )
        self._in_window = False
        self._window_point = None
        self._timeout_reported = False
        self._published_this_approach = False
        self._best_seen = (None, 0.0)
        self._hist.clear()
        self._last_pub_class = None

    # ---------- 推理 ----------
    def _predict(self, image):
        """返回 (类别名, 置信度, 画框用 xyxy 或 None)"""
        if self.mode == "classifier":
            x0, y0, x1, y1 = self.crop
            h, w = image.shape[:2]
            roi = image[max(0, y0):min(h, y1), max(0, x0):min(w, x1)]
            if roi.size == 0:
                return None, 0.0, None
            t = cv2.resize(roi, (224, 224), interpolation=cv2.INTER_AREA)
            t = cv2.cvtColor(t, cv2.COLOR_BGR2RGB)
            x = self.torch.from_numpy(np.ascontiguousarray(t.transpose(2, 0, 1))).float() / 255.0
            x = (x - self.mean) / self.std
            with self.torch.no_grad():
                logits = self.cls_model(x.unsqueeze(0))[0]
                prob = self.torch.softmax(logits, 0)
            conf, idx = float(prob.max()), int(prob.argmax())
            name = self.classes[idx] if idx < len(self.classes) else str(idx)
            return name, conf, (x0, y0, x1, y1)

        result = self.yolo.predict(
            image, imgsz=self.imgsz, conf=self.conf, iou=self.iou, verbose=False
        )[0]
        best = None
        for box in result.boxes:
            c = float(box.conf[0])
            if best is None or c > best[0]:
                best = (c, int(box.cls[0]), box.xyxy[0].tolist())
        if best is None:
            return None, 0.0, None
        c, cid, xyxy = best
        return self.class_names.get(cid, str(cid)), c, xyxy

    # ---------- 回调 ----------
    def on_image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"[图像] 转换失败: {exc}")
            return

        # ---- 距离门控状态 ----
        # 只影响【投票 + 话术发布】，不影响推理与 /detection/result。
        # 后者是验证「>=10 Hz」的观测通道，必须始终有数据。
        pid, dist = self._nearest_point()
        if not self.gate_enable:
            in_window = True
        elif pid is None:
            in_window = False
            self._publish_status("waiting_odom")
        else:
            in_window = self.gate_min <= dist <= self.gate_max
            if in_window and not self._in_window:
                self._entered_window(pid, dist)
            elif not in_window and self._in_window:
                self._left_window()
        allow_publish = in_window

        t_start = time.time()
        try:
            cls_name, conf, box = self._predict(image)
        except Exception as exc:
            self.get_logger().error(f"[推理] 失败: {exc}")
            return

        self._frame_count += 1
        now = time.time()
        self._frame_times.append(now)

        if self.publish_result:
            res = String()
            _ids = target_ids(cls_name) if cls_name else {"rotate": None, "moving": None}
            res.data = json.dumps(
                {
                    "class": cls_name or "none",
                    "conf": round(conf, 4),
                    "frame": self._frame_count,
                    "infer_ms": round((now - t_start) * 1000, 1),
                    "dist": None if dist is None else round(dist, 3),
                    "point": pid,
                    "gated": bool(self.gate_enable and self._in_window),
                    # 判分细则表 7：该类别对应的射击靶位
                    "rotate_target": _ids.get("rotate"),
                    "moving_target": _ids.get("moving"),
                },
                ensure_ascii=False,
            )
            self.result_pub.publish(res)

        # 窗口外不投票：只在识别窗口内积累票数
        if not allow_publish:
            if pid is not None:
                self._publish_status("idle", nearest_point=pid, dist=round(dist, 3))
        else:
            # 记录本次进窗以来的最优单帧结果（不论是否过阈值），供超时兜底使用
            if cls_name is not None and conf > self._best_seen[1]:
                self._best_seen = (cls_name, conf)
            if cls_name is not None and conf >= self.conf:
                self._hist.append(cls_name)
        # 多帧投票：最近 N 帧里众数且票数够，才认为"识别稳定"
        vote_cls, vote_n = None, 0
        if self._hist:
            vote_cls, vote_n = Counter(self._hist).most_common(1)[0]

        # 每类独立的置信度下限（易混淆类别可单独抬高）
        need_conf = max(
            self._min_conf_pub,
            self._class_min_conf.get(vote_cls, 0.0) if vote_cls else self._min_conf_pub,
        )

        if (allow_publish and vote_cls is not None and vote_n >= self._vote_min
                and conf >= need_conf):
            script = CLASS_SCRIPT.get(vote_cls)
            changed = vote_cls != self._last_pub_class
            if script and (changed or (now - self._last_pub_time) >= self.min_interval):
                self.referee.publish_detection(script)
                self._last_pub_time = now
                self._last_pub_class = vote_cls
                self._published_this_approach = True
                # 同步发布靶位信息，供射击节点使用（判分细则表 7）
                self._publish_target(vote_cls, conf, pid, dist, "vote")
                dist_txt = "n/a" if dist is None else f"{dist:.2f}m"
                self.get_logger().info(
                    f"[识别] {vote_cls}(投票 {vote_n}/{len(self._hist)}, "
                    f"本帧 {cls_name} {conf:.2f}, 阈值 {need_conf:.2f}, "
                    f"点位 {pid} 距离 {dist_txt}): {script}"
                )
                self._publish_status(
                    "published",
                    cls=vote_cls,
                    conf=round(conf, 3),
                    point=pid,
                    dist=None if dist is None else round(dist, 3),
                )

        # ---- 超时兜底 ----
        # 依据《判分细则》§1.3：识别失败时该点到达分为 0、射击亦不计分；
        # 而「发错」与「不发」得分完全相同（都是 0）。因此超时时不必空手放弃，
        # 应当用当前最优猜测赌一次 —— 发对了能拿 10 分并保住该点射击分。
        if (self.gate_enable and self._in_window
                and not self._published_this_approach
                and not self._timeout_reported
                and (now - self._window_enter_t) > self.detect_timeout):
            self._timeout_reported = True

            # 优先用投票结果，其次用本次进窗见过的最高置信度单帧
            guess_cls, guess_conf, guess_src = None, 0.0, None
            if vote_cls is not None:
                guess_cls, guess_conf, guess_src = vote_cls, conf, "vote"
            elif self._best_seen[0] is not None:
                guess_cls, guess_conf, guess_src = self._best_seen[0], self._best_seen[1], "best_frame"

            if self.force_publish and guess_cls is not None and CLASS_SCRIPT.get(guess_cls):
                self.referee.publish_detection(CLASS_SCRIPT[guess_cls])
                self._last_pub_time = now
                self._last_pub_class = guess_cls
                self._published_this_approach = True
                # 兜底发布也要给出靶位号：这一发是赌的，但靶位号仍是查表得出的
                self._publish_target(guess_cls, guess_conf, pid, dist, f"forced:{guess_src}")
                self.get_logger().warn(
                    f"[识别] 点位 {pid} 超时 {self.detect_timeout:.1f}s 未达正常发布条件，"
                    f"按「发错与不发同为 0 分」策略强制发布最优猜测："
                    f"{guess_cls}（来源={guess_src}, 置信度={guess_conf:.2f}, "
                    f"票数={vote_n}/{self._vote_min}, 距离={dist:.2f}m）"
                )
                self._publish_status(
                    "forced", cls=guess_cls, conf=round(guess_conf, 3),
                    src=guess_src, point=pid, dist=round(dist, 3),
                )
            else:
                self.get_logger().warn(
                    f"[识别] 点位 {pid} 在 {self.detect_timeout:.1f}s 内既未达发布条件、"
                    f"也没有任何检测结果（距离 {dist:.2f}m）→ 判为识别失败，mission 可继续下一点"
                )
                self._publish_status(
                    "timeout", point=pid, dist=round(dist, 3),
                    votes=vote_n, conf=round(conf, 3),
                )

        if self.fps_log_every > 0 and self._frame_count % self.fps_log_every == 0:
            # 报「最近 N 帧的实时频率」而不是「启动至今的累计平均」：
            # 累计平均会被窗口外的空闲期、仿真暂停等拉低，严重低估真实处理速度。
            if len(self._frame_times) >= 2:
                span = self._frame_times[-1] - self._frame_times[0]
                inst = (len(self._frame_times) - 1) / span if span > 1e-6 else 0.0
                self.get_logger().info(
                    f"[FPS] 已处理 {self._frame_count} 帧，"
                    f"最近 {len(self._frame_times)} 帧实时 {inst:.1f} Hz"
                )

        if self.debug_pub is not None and box is not None:
            # 降频发布：调试图只供人眼检查，按 debug_every 抽帧，避免拖慢主循环
            if self._debug_every <= 1 or self._frame_count % self._debug_every == 0:
                try:
                    debug = image.copy()
                    x1, y1, x2, y2 = [int(v) for v in box]
                    cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f"{cls_name} {conf:.2f}"
                    if dist is not None:
                        label += f" d={dist:.2f}m"
                    cv2.putText(debug, label, (x1, max(0, y1 - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    self.debug_pub.publish(self._cv2_to_imgmsg(debug, msg.header))
                except Exception as exc:
                    self.get_logger().warn(f"[debug] 发布失败: {exc}")

    @staticmethod
    def _cv2_to_imgmsg(cv_img, header):
        img = Image()
        img.header = header
        img.height = cv_img.shape[0]
        img.width = cv_img.shape[1]
        img.encoding = "bgr8"
        img.is_bigendian = False
        img.step = cv_img.strides[0]
        img.data = cv_img.tobytes()
        return img


def main():
    rclpy.init()
    node = RecognizeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # 收到 SIGINT/SIGTERM（被 pkill、launch 停止等）：
        # 此时 rclpy context 可能已关闭，清理动作必须容错，
        # 否则会打出 RCLError: rcl_shutdown already called 的误导性堆栈。
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001 - 退出阶段任何异常都不应改变退出码
            pass
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
