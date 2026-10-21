#!/usr/bin/env python3
"""按验收口径跑一遍：5 个点位 × 每点 10 次，听 /referee/detections 话术判定。

自动开车到 0.8 m 停车位（不靠手开），每点收集 10 条话术，与 assignment 真值比对。
用法:
  python3 scripts/acceptance_run.py --dist 0.8 --per-point 10
"""
import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS / "src" / "ican_recognize"))
from ican_recognize.scripts_table import CLASS_SCRIPT  # noqa: E402

SCRIPT2CLS = {v: k for k, v in CLASS_SCRIPT.items()}
ZH = {"trapped_people": "被困人员", "rescue_camp": "救援营地", "collapsed_ruins": "坍塌废墟",
      "damaged_vehicle": "受损车辆", "landslide": "滑坡塌方"}
GROUND = {0: (1.115218, -1.356470), 1: (0.030951, -1.359743), 2: (0.027137, -2.700000),
          3: (1.120240, -1.913890), 4: (1.120112, -0.784766)}
APPROACH = {0: (1.0, 0.0), 1: (0.0, -1.0), 2: (0.0, -1.0), 3: (1.0, 0.0), 4: (1.0, 0.0)}
ASSIGN = "/opt/ican_shoot_sim/share/ican_land_sim/materials/shooting_texture_assignment.json"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dist", type=float, default=0.8)
    ap.add_argument("--per-point", type=int, default=10)
    ap.add_argument("--timeout", type=float, default=40.0)
    ap.add_argument("--out", default=str(WS / "acceptance_report.txt"))
    args = ap.parse_args()

    assign = json.loads(Path(ASSIGN).read_text(encoding="utf-8"))["ground_type_assignment"]
    expect = {int(k[-1]): v for k, v in assign.items()}

    rclpy.init()
    node = Node("acceptance_run")
    q = []
    node.create_subscription(String, "/referee/detections", lambda m: q.append(m.data), 10)

    rows, ok, total = [], 0, 0
    for pid in range(5):
        tx, ty = GROUND[pid]
        ux, uy = APPROACH[pid]
        yaw = math.atan2(uy, ux)
        sx, sy = tx - ux * args.dist, ty - uy * args.dist
        r = subprocess.run(["python3", "scripts/goto_pose.py", "--x", f"{sx:.4f}",
                            "--y", f"{sy:.4f}", "--yaw", f"{yaw:.4f}"],
                           capture_output=True, text=True, timeout=180)
        if "到位" not in r.stdout:
            print(f"点位{pid}: 开车失败，跳过")
            continue
        time.sleep(3.0)
        q.clear()
        got = 0
        hit = 0
        t0 = time.time()
        while got < args.per_point and time.time() - t0 < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            while q:
                data = q.pop(0)
                got += 1
                cls = SCRIPT2CLS.get(data)
                if cls == expect[pid]:
                    hit += 1
        ok += hit
        total += got
        rate = hit / got * 100 if got else 0
        rows.append((pid, expect[pid], hit, got, rate))
        print(f"点位{pid} ({ZH[expect[pid]]}): {hit}/{got} = {rate:.0f}%")

    rclpy.shutdown()
    subprocess.run(["python3", "scripts/goto_pose.py", "--x", "0", "--y", "0", "--yaw", "0"],
                   capture_output=True, text=True, timeout=180)

    overall = ok / max(total, 1) * 100
    lines = ["=" * 60, "验收口径实测（5 点位 × 10 次，听 /referee/detections）",
             f"停车距离: {args.dist} m", ""]
    for pid, cls, hit, got, rate in rows:
        lines.append(f"点位{pid} ({ZH[cls]}): {hit}/{got} = {rate:.0f}%")
    lines += ["", f"总体: {ok}/{total} = {overall:.1f}%   (验收线 >= 90%)",
              "结论: " + ("通过" if overall >= 90 else "未通过"), "=" * 60]
    rep = "\n".join(lines)
    print("\n" + rep)
    Path(args.out).write_text(rep, encoding="utf-8")


if __name__ == "__main__":
    main()
