#!/usr/bin/env python3
"""识别环境自检：确认「模型版本 / 节点参数 / 相机 / 贴图」四件事一致。

队友那边准确率低，九成是这四项之一和本机不一致。跑一遍把差异打出来。

用法：
  python3 交付包/识别/脚本/check_recog_env.py
  python3 交付包/识别/脚本/check_recog_env.py --expect-md5 <md5>
"""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

CLASSES = ["trapped_people", "rescue_camp", "collapsed_ruins", "damaged_vehicle", "landslide"]
ZH = dict(zip(CLASSES, ["被困人员", "救援营地", "坍塌废墟", "受损车辆", "滑坡塌方"]))
ASSIGN_CANDIDATES = [
    "/opt/ican_shoot_sim/share/ican_land_sim/materials/shooting_texture_assignment.json",
    r"\\wsl.localhost\Ubuntu-22.04\opt\ican_shoot_sim\share\ican_land_sim\materials\shooting_texture_assignment.json",
    r"\\wsl$\Ubuntu-22.04\opt\ican_shoot_sim\share\ican_land_sim\materials\shooting_texture_assignment.json",
]


def md5(path):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def human(n):
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024:
            return f"{n:.0f}{unit}"
        n /= 1024
    return f"{n:.1f}TB"


def sh(cmd):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=20).stdout.strip()
    except Exception as exc:
        return f"<失败: {exc}>"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--expect-md5", default=None, help="期望的模型 md5（从主控机器取）")
    args = ap.parse_args()

    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

    print("=" * 68)
    print(" 识别环境自检")
    print("=" * 68)
    if sys.platform.startswith("win"):
        print("\n[注意] 当前在 Windows 下运行：")
        print("   模型 / 脚本 两项有效；相机、贴图分配、节点进程 **必须在 WSL 里查**。")
        print("   在 WSL 里执行： cd ~/ican-rescue-robot && python3 交付包/识别/脚本/check_recog_env.py")

    # 1. 模型
    print("\n[1] 模型")
    default_model = Path.home() / ".ros/ican_recognize/best.pt"
    here = Path(__file__).resolve()
    pkg_candidates = [
        here.parents[1] / "模型/best.pt",                    # 在 交付包/识别/脚本/ 下运行
        here.parents[1] / "交付包/识别/模型/best.pt",         # 在仓库根的 scripts/ 下运行
        here.parents[2] / "模型/best.pt",
        Path.cwd() / "交付包/识别/模型/best.pt",
    ]
    pkg_model = next((c for c in pkg_candidates if c.is_file()), pkg_candidates[0])
    candidates = [default_model, pkg_model]
    found = []
    for p in candidates:
        if p.is_file():
            found.append(p)
            print(f"  {p}")
            print(f"    大小 {human(p.stat().st_size)}  md5 {md5(str(p))}")
            print(f"    修改时间 {p.stat().st_mtime:.0f}")
    if not found:
        print("  NG 找不到任何模型文件")
    else:
        env_model = os.environ.get("ICAN_YOLO_MODEL")
        print(f"  环境变量 ICAN_YOLO_MODEL = {env_model or '(未设置)'}")
        if default_model.is_file() and pkg_model.is_file():
            same = md5(str(default_model)) == md5(str(pkg_model))
            print(f"  节点默认模型 == 交付包模型 ? {'OK 是' if same else 'NG 否（节点用的不是交付包里那个！）'}")
        if args.expect_md5 and found:
            ok = md5(str(found[0])) == args.expect_md5.lower()
            print(f"  与期望 md5 一致 ? {'OK' if ok else 'NG 不一致'}")

    # 2. 节点参数
    print("\n[2] 节点进程与参数")
    ps = sh("ps -eo cmd | grep '[r]ecognize_node'")
    if ps:
        print("  " + ps.replace("\n", "\n  "))
        for key in ("mode", "imgsz", "model_path", "conf_thres"):
            print(f"    {key} 显式指定 ? {'是' if key + ':=' in ps else '否（用默认值）'}")
    else:
        print("  （节点未运行）")

    # 3. 相机与频率
    print("\n[3] 相机")
    info = sh("timeout 6 ros2 topic echo /camera/camera_info --field width --once 2>/dev/null | head -1")
    hz = sh("timeout 8 ros2 topic hz /camera/image 2>&1 | grep 'average rate' | tail -1")
    print(f"  图像宽度: {info or '(读不到 /camera/camera_info)'}")
    print(f"  相机频率: {hz or '(读不到 /camera/image)'}")

    # 4. 当前贴图分配
    print("\n[4] 当前贴图分配（决定这一轮考什么图）")
    assign_path = next((Path(c) for c in ASSIGN_CANDIDATES if Path(c).is_file()), None)
    try:
        if assign_path is None:
            raise FileNotFoundError("找不到 shooting_texture_assignment.json（Windows 下请在 WSL 里运行）")
        d = json.loads(assign_path.read_text())
        a = d.get("assignment", {})
        for k, v in sorted(a.items()):
            print(f"  点位{k[-1]}: {v}")
    except Exception as exc:
        print(f"  读不到: {exc}")

    # 5. 验收脚本版本
    print("\n[5] 验收脚本")
    acc = Path(__file__).resolve().parent / "acceptance_run.py"
    if acc.is_file():
        txt = acc.read_text(encoding="utf-8", errors="ignore")
        has_points = "--points" in txt
        print(f"  {acc}")
        print(f"  支持 --points（官方 4 点位口径）? {'OK' if has_points else 'NG 旧版！会用 5 点位，数字偏低'}")
    else:
        print("  找不到 acceptance_run.py")

    print("\n" + "=" * 68)
    print(" 判读")
    print("=" * 68)
    print("""  准确率低时，按下面顺序排查：
   1. 上面的「节点默认模型 == 交付包模型」若是 NG → 先换模型：
        cp 交付包/识别/模型/best.pt ~/.ros/ican_recognize/best.pt
   2. 验收脚本不支持 --points → 用仓库里最新版 acceptance_run.py
   3. 相机频率 < 10 Hz → 识别投票攒不满，准确率会掉，关掉调试图/其它程序
   4. 停车距离必须是 0.8 m（1.4 m 时准确率只有 40%）
   5. 记下当前贴图分配，换 seed 后结果会差很多——要按多 seed 平均报""")


if __name__ == "__main__":
    main()
