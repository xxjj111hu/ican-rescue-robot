#!/usr/bin/env python3
"""数据集验收自检（B-01）。

对照《任务说明书》B-01 验收口径检查一个 YOLO 数据集目录：
  1. 5 个类别齐全，顺序与 scripts_table.CLASS_ORDER 一致
  2. **每类不少于 50 张**（本项目实测每类 157~176 个框，应远超）
  3. train / val 都不为空
  4. 每个 label 文件格式合法（5 个字段、数值范围 0~1、class id 在范围内）
  5. 报告图片与标签是否一一对应

用法：
    python3 scripts/check_dataset.py --root dataset_all
    python3 scripts/check_dataset.py --root dataset_all --min-per-class 50

退出码：0 = 全部通过，1 = 有不合格项。
"""
from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path

# 与 ican_recognize/scripts_table.py 的 CLASS_ORDER 保持一致
CLASS_ORDER = ["trapped_people", "rescue_camp", "collapsed_ruins", "damaged_vehicle", "landslide"]

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="YOLO 数据集验收自检（B-01）")
    p.add_argument("--root", type=Path, required=True, help="数据集根目录（含 images/ labels/）")
    p.add_argument("--min-per-class", type=int, default=50, help="每类最少框数（默认 50）")
    return p.parse_args()


def read_names(data_yaml: Path):
    """尽量读 data.yaml 的 names（只用标准库，不强制依赖 pyyaml）。"""
    if not data_yaml.is_file():
        return None, "未找到 data.yaml"
    text = data_yaml.read_text(encoding="utf-8", errors="ignore")
    names = []
    for line in text.splitlines():
        s = line.strip()
        if s.startswith("- "):
            names.append(s[2:].strip().strip("'\""))
    return names, None


def count_split(root: Path, split: str, problems: list):
    """统计一个 split 的框数与合法性，返回 (Counter, 图片数, 标签数)。"""
    img_dir = root / "images" / split
    lbl_dir = root / "labels" / split
    boxes = Counter()

    if not img_dir.is_dir():
        problems.append(f"[{split}] 图片目录不存在: {img_dir}")
        return boxes, 0, 0
    if not lbl_dir.is_dir():
        problems.append(f"[{split}] 标签目录不存在: {lbl_dir}")
        return boxes, 0, 0

    imgs = {p.stem for p in img_dir.iterdir() if p.suffix.lower() in IMG_EXTS}
    lbls = {p.stem for p in lbl_dir.glob("*.txt")}

    for stem in sorted(imgs - lbls):
        problems.append(f"[{split}] 图片无对应标签: {stem}")
    for stem in sorted(lbls - imgs):
        problems.append(f"[{split}] 标签无对应图片: {stem}")

    empty = 0
    for lbl in sorted(lbl_dir.glob("*.txt")):
        lines = [ln for ln in lbl.read_text(encoding="utf-8").splitlines() if ln.strip()]
        if not lines:
            empty += 1
            continue
        for i, ln in enumerate(lines, 1):
            parts = ln.split()
            if len(parts) != 5:
                problems.append(f"[{split}] {lbl.name} 第 {i} 行字段数={len(parts)}，应为 5")
                continue
            try:
                cid = int(parts[0])
                *vals, = map(float, parts[1:])
            except ValueError:
                problems.append(f"[{split}] {lbl.name} 第 {i} 行含非数字")
                continue
            if not (0 <= cid < len(CLASS_ORDER)):
                problems.append(f"[{split}] {lbl.name} 第 {i} 行 class id={cid} 越界")
                continue
            if any(v < 0.0 or v > 1.0 for v in vals):
                problems.append(f"[{split}] {lbl.name} 第 {i} 行坐标不在 0~1（应已归一化）")
                continue
            boxes[CLASS_ORDER[cid]] += 1

    if empty:
        problems.append(f"[{split}] 有 {empty} 个空标签文件（无标注）")
    return boxes, len(imgs), len(lbls)


def main() -> int:
    args = parse_args()
    root: Path = args.root
    problems: list[str] = []

    if not root.is_dir():
        print(f"[失败] 数据集目录不存在: {root}")
        return 1

    names, err = read_names(root / "data.yaml")
    if err:
        problems.append(err)
    else:
        if names != CLASS_ORDER:
            problems.append(
                f"data.yaml 的 names 与 CLASS_ORDER 不一致\n"
                f"    data.yaml : {names}\n"
                f"    CLASS_ORDER: {CLASS_ORDER}"
            )

    total = Counter()
    print(f"数据集根目录: {root.resolve()}\n")
    for split in ("train", "val"):
        boxes, n_img, n_lbl = count_split(root, split, problems)
        total.update(boxes)
        print(f"[{split}] 图片 {n_img} 张 / 标签 {n_lbl} 个 / 框 {sum(boxes.values())} 个")
        for cls in CLASS_ORDER:
            print(f"    {cls:<18} {boxes.get(cls, 0):>4}")
        print()

    print(f"[合计] 框 {sum(total.values())} 个")
    for cls in CLASS_ORDER:
        n = total.get(cls, 0)
        mark = "[OK]" if n >= args.min_per_class else "[失败]"
        print(f"    {cls:<18} {n:>4}  {mark} (下限 {args.min_per_class})")
        if n < args.min_per_class:
            problems.append(f"类别 {cls} 仅 {n} 个框，低于下限 {args.min_per_class}")

    print()
    if problems:
        print(f"[失败] 未通过，共 {len(problems)} 个问题：")
        for p in problems:
            print("  -", p)
        return 1

    print("[OK] 数据集自检全部通过：类别齐全、顺序一致、每类框数达标、标注格式合法。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
