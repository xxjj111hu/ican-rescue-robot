"""识别节点：订阅 /camera/image -> 推理 -> 发布识别结果与裁判话术。

两种模式：
  mode:=classifier  5 类分类模型（runs_cls/best.pt），按 auto_collect 采集的数据训练，推荐
  mode:=yolo        旧版 YOLO 检测权重（runs/ican_train_v2/weights/best.pt），保留兼容

话题：
  订阅 /camera/image        sensor_msgs/Image            （QoS: sensor_data）
  发布 /detection/result    std_msgs/String（JSON，逐帧输出，用于验证 >=10Hz）
  发布 /referee/detections  std_msgs/String（话术，QoS: RELIABLE，进点前上报）
  发布 /ican_recognize/debug_image sensor_msgs/Image（可选）
"""
import json
import time
from collections import Counter, deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ican_recognize.scripts_table import CLASS_ORDER, CLASS_SCRIPT, validate_scripts


class RecognizeNode(Node):
    def __init__(self):
        super().__init__("ican_recognize_node")
        self.declare_parameter("mode", "yolo")   # 新分类模型未达标前，默认仍用 YOLO          # classifier | yolo
        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("detections_topic", "/referee/detections")
        self.declare_parameter("result_topic", "/detection/result")
        self.declare_parameter("publish_result", True)
        self.declare_parameter("debug_image_topic", "/ican_recognize/debug_image")
        self.declare_parameter("publish_debug_image", True)
        # 分类模式
        self.declare_parameter(
            "cls_model_path",
            "/home/longying01/demo_shoot_ws/runs_cls/best.pt",
        )
        # YOLO 模式
        self.declare_parameter(
            "model_path",
            "/home/longying01/demo_shoot_ws/runs/detect/runs/ican_v6/weights/best.pt",
        )
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("conf_thres", 0.20)
        # 多帧投票窗口：取最近 N 帧的众数，抑制单帧抖动
        self.declare_parameter("vote_window", 7)
        self.declare_parameter("vote_min", 4)
        # 发布话术的置信度下限：低于这个值宁可晚一点发，也不发错的
        self.declare_parameter("min_conf_publish", 0.45)
        self.declare_parameter("iou_thres", 0.45)
        self.declare_parameter("min_publish_interval", 2.0)
        self.declare_parameter("fps_log_every", 30)

        self.bridge = CvBridge()
        self.mode = str(self.get_parameter("mode").value).lower()
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.conf = float(self.get_parameter("conf_thres").value)
        self.iou = float(self.get_parameter("iou_thres").value)
        self.min_interval = float(self.get_parameter("min_publish_interval").value)
        self.fps_log_every = int(self.get_parameter("fps_log_every").value)
        self.publish_result = bool(self.get_parameter("publish_result").value)

        issues = validate_scripts()
        if issues:
            for msg in issues:
                self.get_logger().error(f"[话术自检] {msg}")
        else:
            self.get_logger().info("[话术自检] 5 条话术均含裁判匹配关键词，通过")

        self.class_names = {i: n for i, n in enumerate(CLASS_ORDER)}
        if self.mode == "classifier":
            self._init_classifier()
        else:
            self._init_yolo()

        image_topic = str(self.get_parameter("image_topic").value)
        det_topic = str(self.get_parameter("detections_topic").value)
        result_topic = str(self.get_parameter("result_topic").value)
        debug_topic = str(self.get_parameter("debug_image_topic").value)
        self.publish_debug = bool(self.get_parameter("publish_debug_image").value)

        det_qos = QoSProfile(depth=10)
        det_qos.reliability = ReliabilityPolicy.RELIABLE
        det_qos.durability = DurabilityPolicy.VOLATILE
        self.det_pub = self.create_publisher(String, det_topic, det_qos)
        self.result_pub = self.create_publisher(String, result_topic, 10)
        self.debug_pub = (
            self.create_publisher(Image, debug_topic, 10) if self.publish_debug else None
        )
        self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)

        self._vote_window = int(self.get_parameter("vote_window").value)
        self._vote_min = int(self.get_parameter("vote_min").value)
        self._min_conf_pub = float(self.get_parameter("min_conf_publish").value)
        self._hist = deque(maxlen=max(1, self._vote_window))
        self._last_pub_time = 0.0
        self._last_pub_class = None
        self._frame_count = 0
        self._t0 = time.time()
        self.get_logger().info(
            f"识别节点就绪（{self.mode}）: {image_topic} -> {det_topic} (+{result_topic})"
        )

    # ---------- 模型加载 ----------
    def _init_classifier(self):
        import torch
        import torch.nn as nn
        from torchvision import models

        path = str(self.get_parameter("cls_model_path").value)
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

        path = str(self.get_parameter("model_path").value)
        self.yolo = YOLO(path)
        raw = getattr(self.yolo, "names", None)
        if isinstance(raw, dict):
            self.class_names = {int(k): v for k, v in raw.items()}
        for i, n in enumerate(CLASS_ORDER):
            self.class_names.setdefault(i, n)
        self.get_logger().info(f"YOLO 模型加载完成: {path}")

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

        t_start = time.time()
        try:
            cls_name, conf, box = self._predict(image)
        except Exception as exc:
            self.get_logger().error(f"[推理] 失败: {exc}")
            return

        self._frame_count += 1
        now = time.time()

        if self.publish_result:
            res = String()
            res.data = json.dumps(
                {
                    "class": cls_name or "none",
                    "conf": round(conf, 4),
                    "frame": self._frame_count,
                    "infer_ms": round((now - t_start) * 1000, 1),
                },
                ensure_ascii=False,
            )
            self.result_pub.publish(res)

        if cls_name is not None and conf >= self.conf:
            self._hist.append(cls_name)
        # 多帧投票：最近 N 帧里众数且票数够，才认为"识别稳定"
        vote_cls, vote_n = None, 0
        if self._hist:
            vote_cls, vote_n = Counter(self._hist).most_common(1)[0]
        if (vote_cls is not None and vote_n >= self._vote_min
                and conf >= self._min_conf_pub):
            script = CLASS_SCRIPT.get(vote_cls)
            changed = vote_cls != self._last_pub_class
            if script and (changed or (now - self._last_pub_time) >= self.min_interval):
                out = String()
                out.data = script
                self.det_pub.publish(out)
                self._last_pub_time = now
                self._last_pub_class = vote_cls
                self.get_logger().info(
                    f"[识别] {vote_cls}(投票 {vote_n}/{len(self._hist)}, 本帧 {cls_name} {conf:.2f}): {script}")

        if self.fps_log_every > 0 and self._frame_count % self.fps_log_every == 0:
            dt = max(time.time() - self._t0, 1e-6)
            self.get_logger().info(
                f"[FPS] 已处理 {self._frame_count} 帧，平均 {self._frame_count / dt:.1f} Hz"
            )

        if self.debug_pub is not None and box is not None:
            try:
                debug = image.copy()
                x1, y1, x2, y2 = [int(v) for v in box]
                cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{cls_name} {conf:.2f}"
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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
