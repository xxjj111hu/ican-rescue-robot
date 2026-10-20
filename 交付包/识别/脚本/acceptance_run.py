#!/usr/bin/env python3
"""按验收口径跑一遍：5 个点位 × 每点 10 次，听 /referee/detections 话术判定。

自动开车到指定距离的停车位（不靠手开），每点收集 N 条话术，与真值比对。

真值来源（按优先级）
--------------------
1) `--assign` 指定的 json（`randomize_ground_textures.py` 生成的 assignment 文件）
2) `/opt/ican_shoot_sim/share/ican_land_sim/materials/shooting_texture_assignment.json`
3) 若都不存在，直接解析 `ican_land.material` 推断每个点位的贴图类别
   —— 这样即使还没跑过随机化，也能用"默认分配"做基线验收。

前提
----
- 仿真 + Nav2 已启动，识别节点在跑，**顺序任务节点不能启动**
  （用 `bash sim_ws/start_all.sh --no-mission`），否则它自己会开车抢控制权。

用法
----
  python3 交付包/识别/脚本/acceptance_run.py --dist 0.8 --per-point 10 \
      --out 交付包/识别/报告/acc_seed201_v9.txt
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent


def find_ws() -> Path | None:
    """向上查找含 src/ican_recognize 的工作空间根目录。"""
    for base in [SCRIPT_DIR, *SCRIPT_DIR.parents]:
        if (base / "src" / "ican_recognize").is_dir():
            return base
    return None


WS = find_ws()
if WS is not None:
    sys.path.insert(0, str(WS / "src" / "ican_recognize"))
else:
    # 退路：按仓库布局直接找包目录（交付包/识别/脚本 -> ../../sim_ws/src）
    cand = SCRIPT_DIR.parent.parent / "sim_ws" / "src" / "ican_recognize"
    if (cand / "ican_recognize").is_dir():
        sys.path.insert(0, str(cand))

try:
    from ican_recognize.scripts_table import CLASS_SCRIPT  # noqa: E402
except ModuleNotFoundError as exc:  # pragma: no cover
    sys.exit(
        f"找不到 ican_recognize.scripts_table（{exc}）。\n"
        f"请先 colcon build 并在仓库根执行，或用 --ws 显式指定工作空间。"
    )

SCRIPT2CLS = {v: k for k, v in CLASS_SCRIPT.items()}
ZH = {"trapped_people": "被困人员", "rescue_camp": "救援营地", "collapsed_ruins": "坍塌废墟",
      "damaged_vehicle": "受损车辆", "landslide": "滑坡塌方"}
GROUND = {0: (1.115218, -1.356470), 1: (0.030951, -1.359743), 2: (0.027137, -2.700000),
          3: (1.120240, -1.913890), 4: (1.120112, -0.784766)}
APPROACH = {0: (1.0, 0.0), 1: (0.0, -1.0), 2: (0.0, -1.0), 3: (1.0, 0.0), 4: (1.0, 0.0)}

DEFAULT_SHARE = Path("/opt/ican_shoot_sim/share/ican_land_sim")
ASSIGN_DEFAULT = DEFAULT_SHARE / "materials" / "shooting_texture_assignment.json"
MATERIAL_DEFAULT = DEFAULT_SHARE / "materials" / "scripts" / "ican_land.material"

# 贴图文件名 -> 类别（去掉结尾的 _数字 与扩展名）
TEXTURE_RE = re.compile(r"^(?P<cls>.+?)_\d+\.(?:png|jpg|jpeg)$", re.IGNORECASE)


def class_from_texture(name: str) -> str | None:
    m = TEXTURE_RE.match(Path(name).name)
    if not m:
        return None
    cls = m.group("cls").lower()
    return cls if cls in ZH else None


def truth_from_material(material_path: Path) -> dict[int, str]:
    """解析 ican_land.material，取出 ShootingGroundTargetN 的贴图类别。"""
    text = material_path.read_text(encoding="utf-8", errors="replace")
    result: dict[int, str] = {}
    for m in re.finditer(
        r"material\s+ShootingGroundTarget(\d+)\s*\{(.*?)\n\}", text, re.DOTALL
    ):
        idx = int(m.group(1))
        tex = re.search(r"texture\s+(\S+)", m.group(2))
        if not tex:
            continue
        cls = class_from_texture(tex.group(1))
        if cls:
            result[idx] = cls
    return result


def load_truth(assign_path: Path | None, material_path: Path) -> tuple[dict[int, str], str]:
    """返回 (点位->类别, 真值来源说明)。"""
    path = assign_path or ASSIGN_DEFAULT
    if path.is_file():
        data = json.loads(path.read_text(encoding="utf-8"))
        assign = data.get("ground_type_assignment", {})
        truth = {int(k[len("ShootingGroundTarget"):]): v
                 for k, v in assign.items() if k.startswith("ShootingGroundTarget")}
        if truth:
            return truth, f"assignment json: {path}"
    if material_path.is_file():
        truth = truth_from_material(material_path)
        if truth:
            return truth, f"material 文件（默认分配）: {material_path}"
    raise SystemExit(
        f"无法确定真值：\n  json 不存在: {path}\n  material 不存在: {material_path}\n"
        f"请先运行随机化脚本生成分配，或用 --assign 指定。"
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dist", type=float, default=0.8)
    ap.add_argument("--per-point", type=int, default=10)
    ap.add_argument("--timeout", type=float, default=40.0, help="单点采集超时（秒）")
    ap.add_argument("--assign", default=None, help="真值 json 路径（可省略）")
    ap.add_argument("--material", default=str(MATERIAL_DEFAULT),
                    help="无 json 时用该 material 文件推断真值")
    ap.add_argument("--goto", default=str(SCRIPT_DIR / "goto_pose.py"),
                    help="goto_pose.py 路径")
    ap.add_argument("--out", default=None,
                    help="结果文件；默认 交付包/识别/报告/acceptance_run.txt")
    ap.add_argument("--seed", default=None, help="仅用于写进报告标题")
    args = ap.parse_args()

    expect, truth_src = load_truth(
        Path(args.assign) if args.assign else None, Path(args.material)
    )
    missing = [p for p in range(5) if p not in expect]
    if missing:
        raise SystemExit(f"真值缺少点位 {missing}；来源 {truth_src}")

    goto = Path(args.goto)
    if not goto.is_file():
        raise SystemExit(f"找不到 goto_pose.py: {goto}")
    if not os.access(goto, os.R_OK):
        raise SystemExit(f"goto_pose.py 不可读: {goto}")

    out_path = Path(args.out) if args.out else (
        SCRIPT_DIR.parent / "报告" / f"acceptance_run{'_seed' + args.seed if args.seed else ''}.txt"
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"真值来源: {truth_src}")
    print(f"真值: " + ", ".join(f"{p}={ZH[c]}" for p, c in sorted(expect.items())))
    print(f"结果将写入: {out_path}")
    print()

    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node("acceptance_run")
    q: list[str] = []
    node.create_subscription(String, "/referee/detections", lambda m: q.append(m.data), 10)

    rows, ok, total = [], 0, 0
    for pid in range(5):
        tx, ty = GROUND[pid]
        ux, uy = APPROACH[pid]
        yaw = math.atan2(uy, ux)
        sx, sy = tx - ux * args.dist, ty - uy * args.dist
        try:
            r = subprocess.run(
                [sys.executable, str(goto), "--x", f"{sx:.4f}",
                 "--y", f"{sy:.4f}", "--yaw", f"{yaw:.4f}"],
                capture_output=True, text=True, timeout=180,
            )
        except subprocess.TimeoutExpired:
            print(f"点位{pid}: 开车超时，跳过")
            continue
        if "到位" not in (r.stdout or ""):
            print(f"点位{pid}: 开车失败，跳过（{r.stdout[-200:]}{r.stderr[-200:]}）")
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
    subprocess.run([sys.executable, str(goto), "--x", "0", "--y", "0", "--yaw", "0"],
                   capture_output=True, text=True, timeout=180)

    overall = ok / max(total, 1) * 100
    title = "验收口径实测（5 点位 × {} 次，听 /referee/detections）".format(args.per_point)
    lines = ["=" * 60, title, f"停车距离: {args.dist} m"]
    if args.seed:
        lines.append(f"随机化 seed: {args.seed}")
    lines.append(f"真值来源: {truth_src}")
    lines.append("")
    for pid, cls, hit, got, rate in rows:
        lines.append(f"点位{pid} ({ZH[cls]}): {hit}/{got} = {rate:.0f}%")
    lines += ["", f"总体: {ok}/{total} = {overall:.1f}%   (验收线 >= 90%)",
              "结论: " + ("通过" if overall >= 90 else "未通过"), "=" * 60]
    rep = "\n".join(lines)
    print("\n" + rep)
    out_path.write_text(rep + "\n", encoding="utf-8")
    return 0 if overall >= 90 else 1


if __name__ == "__main__":
    sys.exit(main())
