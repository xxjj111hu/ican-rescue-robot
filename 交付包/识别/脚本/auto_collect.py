#!/usr/bin/env python3
"""自动采集识别数据集（贴图随机 × 距离 × 偏航角），标签由 assignment json 自动生成。

为什么这样采：
  贴图每次 randomize 会重新分配，所以模型只能靠外观分类。采集必须覆盖
  「多个随机分配（seed）× 多个距离 × 多个车头角」，否则模型会记住位置而不是外观。

用法:
  python3 scripts/auto_collect.py                       # 4 个 seed，全自动（会重启 Gazebo）
  python3 scripts/auto_collect.py --seeds 201 --distances 0.8 1.1 --yaws -20 0 20
  python3 scripts/auto_collect.py --no-restart          # 不换贴图、不重启，只采当前分配
  python3 scripts/auto_collect.py --out dataset_v2

输出:
  dataset_v2/images/<seed>/<point>_<dist>_<yaw>_<k>.jpg
  dataset_v2/labels.jsonl   每行一个样本（含 seed/point/class/dist/yaw/pose）
"""
import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

WS = Path(__file__).resolve().parents[1]
ASSIGN = Path("/opt/ican_shoot_sim/share/ican_land_sim/materials/shooting_texture_assignment.json")

# 5 个地面贴图的真实位姿（land_shoot.world）
GROUND = {
    0: (1.115218, -1.356470),
    1: (0.030951, -1.359743),
    2: (0.027137, -2.700000),
    3: (1.120240, -1.913890),
    4: (1.120112, -0.784766),
}
# 车 -> 贴图 的接近方向（单位向量）：0/3/4 从西侧向东，1/2 从北侧向南
APPROACH = {
    0: (1.0, 0.0),
    1: (0.0, -1.0),
    2: (0.0, -1.0),
    3: (1.0, 0.0),
    4: (1.0, 0.0),
}


def sh(cmd, timeout=180, check=False):
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, cwd=str(WS))
    if check and r.returncode != 0:
        raise SystemExit(f"命令失败: {' '.join(cmd)}\n{r.stdout}\n{r.stderr}")
    return r


class Camera(Node):
    def __init__(self):
        super().__init__("auto_collect")
        self.frames = []
        self.create_subscription(Image, "/camera/image", self._cb, qos_profile_sensor_data)
        from nav_msgs.msg import Odometry
        self.odom_seen = 0
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)

    def _odom_cb(self, msg):
        self.odom_seen += 1

    def _cb(self, msg):
        self.frames.append(msg)
        if len(self.frames) > 30:
            self.frames = self.frames[-30:]

    def latest_bgr(self):
        if not self.frames:
            return None
        m = self.frames[-1]
        arr = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.width, 3)
        enc = m.encoding.lower()
        if enc == "rgb8":
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif enc == "rgba8":
            arr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        return arr


def wait_odom(node, timeout=60):
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.odom_seen > 0:
            return True
    return False


def wait_camera(node, timeout=90):
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.frames:
            return True
    return False


def restart_sim(gui=False):
    sh(["pkill", "-f", "sim.launch.py"], timeout=30)
    sh(["pkill", "-f", "gzserver"], timeout=30)
    sh(["pkill", "-f", "gzclient"], timeout=30)
    time.sleep(2.0)
    cmd = (
        "source /opt/ros/humble/setup.bash && source /opt/ican_shoot_sim/setup.bash && "
        f"nohup ros2 launch ican_shoot_sim sim.launch.py gui:={'true' if gui else 'false'} "
        "> /tmp/sim_auto_collect.log 2>&1 &"
    )
    subprocess.Popen(["bash", "-lc", "cd %s && %s" % (WS, cmd)],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)


def read_assignment():
    data = json.loads(ASSIGN.read_text(encoding="utf-8"))
    out = {}
    for mat, cls in data.get("ground_type_assignment", {}).items():
        out[int("".join(ch for ch in mat if ch.isdigit()))] = cls
    return out, data


def drive(x, y, yaw):
    last = ""
    for _ in range(2):
        r = sh(["python3", "scripts/goto_pose.py", "--x", f"{x:.4f}", "--y", f"{y:.4f}",
                "--yaw", f"{yaw:.4f}"], timeout=180)
        if "到位" in r.stdout:
            return True, r.stdout.strip().splitlines()[-1]
        last = r.stdout.strip().splitlines()[-1] if r.stdout.strip() else r.stderr[-200:]
        time.sleep(1.0)
    return False, last


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="*", default=[101, 102, 103, 104])
    ap.add_argument("--distances", type=float, nargs="*", default=[0.8, 1.1, 1.4])
    ap.add_argument("--yaws", type=float, nargs="*", default=[-20.0, 0.0, 20.0],
                    help="车头相对贴图方向的偏角（度）")
    ap.add_argument("--frames", type=int, default=6, help="每个位置抓几帧")
    ap.add_argument("--points", type=int, nargs="*", default=[0, 1, 2, 3, 4])
    ap.add_argument("--out", default=str(WS / "dataset_v2"))
    ap.add_argument("--no-restart", action="store_true", help="不换贴图、不重启 Gazebo")
    args = ap.parse_args()

    out = Path(args.out)
    (out / "images").mkdir(parents=True, exist_ok=True)
    label_path = out / "labels.jsonl"

    rclpy.init()
    cam = Camera()

    seeds = [0] if args.no_restart else args.seeds
    total = 0
    t_start = time.time()
    try:
        for si, seed in enumerate(seeds):
            if args.no_restart:
                print(f"[{si+1}/{len(seeds)}] 不换贴图，直接用当前分配")
            else:
                print(f"[{si+1}/{len(seeds)}] 换贴图 seed={seed} ...")
                r = sh(["bash", "scripts/randomize_ground_textures.sh", "--seed", str(seed)], timeout=180)
                if r.returncode != 0:
                    print("  贴图替换失败，日志：\n", r.stdout[-500:], r.stderr[-500:])
                    print("  跳过该 seed")
                    continue
                restart_sim(gui=False)
                print("  Gazebo 重启中 ...")
                cam.frames.clear()
                cam.odom_seen = 0
                if not wait_camera(cam, 120):
                    print("  120s 内没等到 /camera/image，跳过该 seed")
                    continue
                if not wait_odom(cam, 60):
                    print("  60s 内没等到 /odom，跳过该 seed")
                    continue
                time.sleep(3.0)  # 等渲染稳定
            cam.frames.clear()
            rclpy.spin_once(cam, timeout_sec=0.5)

            classes, raw = read_assignment()
            print(f"  本次分配: " + ", ".join(f"{k}={v}" for k, v in sorted(classes.items())))
            seed_dir = out / "images" / f"seed{seed}"
            seed_dir.mkdir(parents=True, exist_ok=True)

            for pid in args.points:
                if pid not in classes:
                    continue
                tx, ty = GROUND[pid]
                ux, uy = APPROACH[pid]
                base_yaw = math.atan2(uy, ux)
                for dist in args.distances:
                    for yaw_deg in args.yaws:
                        yaw = base_yaw + math.radians(yaw_deg)
                        sx, sy = tx - ux * dist, ty - uy * dist
                        ok, msg = drive(sx, sy, yaw)
                        if not ok:
                            print(f"    点位{pid} d={dist} yaw={yaw_deg}: 未到位 {msg[:80]}")
                            continue
                        time.sleep(0.25)
                        cam.frames.clear()
                        n = 0
                        t0 = time.time()
                        while n < args.frames and time.time() - t0 < 6:
                            rclpy.spin_once(cam, timeout_sec=0.1)
                            img = cam.latest_bgr()
                            if img is None:
                                continue
                            name = f"p{pid}_d{dist:.1f}_y{yaw_deg:+.0f}_{n}.jpg"
                            cv2.imwrite(str(seed_dir / name), img)
                            with label_path.open("a", encoding="utf-8") as f:
                                f.write(json.dumps({
                                    "file": f"images/seed{seed}/{name}",
                                    "seed": seed,
                                    "point": pid,
                                    "class": classes[pid],
                                    "dist": dist,
                                    "yaw_deg": yaw_deg,
                                    "pose": [round(sx, 4), round(sy, 4), round(yaw, 4)],
                                }, ensure_ascii=False) + "\n")
                            n += 1
                            total += 1
                            time.sleep(0.12)
                        print(f"    点位{pid} d={dist:.1f} yaw={yaw_deg:+.0f} -> {n} 帧")
            print(f"  累计 {total} 帧，用时 {(time.time()-t_start)/60:.1f} 分钟")
    finally:
        cam.destroy_node()
        rclpy.shutdown()

    print(f"\n完成：共 {total} 帧，写入 {out}")
    print(f"标签文件：{label_path}")
    if not args.no_restart:
        print("正在把 Gazebo 以带界面方式重启，恢复你的窗口 ...")
        restart_sim(gui=True)


if __name__ == "__main__":
    main()
