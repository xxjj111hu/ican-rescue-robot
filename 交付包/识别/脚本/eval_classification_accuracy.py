#!/usr/bin/env python3
"""离线分类准确率评估（对应 B-02 的「分类准确率」口径）。

为什么需要它
------------
B-02 的验收口径是"地面图片分类准确率 ≥95%"，而训练产出的是 YOLO 检测模型，
衡量的是 mAP50（0.880）。两者不是同一个指标，答辩时会被追问。

本脚本把检测模型当分类器用：对每张验证集图片取置信度最高的检测类别作为
预测类别，与该图片标注的类别比较，得到**分类准确率**与**混淆矩阵**，
从而给出与任务书口径一致的结论。

用法
----
    cd ~/ican-repo
    python3 交付包/识别/脚本/eval_classification_accuracy.py \
        --data 交付包/识别/数据 \
        --model ~/.ros/ican_recognize/best.pt

    # 只评估某一档距离（文件名里的 d0.8 / d1.1 / d1.4）
    python3 ... --dist 0.8

输出
----
    每类准确率 / 召回率、总体准确率、混淆矩阵（文本），以及
    按距离分档的准确率对照（用于说明"最佳工作距离"）。
"""
from __future__ import annotations

import argparse
import sys
from collections import Counter, defaultdict
from pathlib import Path

CLASSES = ["trapped_people", "rescue_camp", "collapsed_ruins", "damaged_vehicle", "landslide"]
ZH = dict(zip(CLASSES, ["被困人员", "救援营地", "坍塌废墟", "受损车辆", "滑坡塌方"]))
IMG_EXT = {".jpg", ".jpeg", ".png"}


def read_first_class(label_path: Path):
    """读 YOLO 标签里第一个框的类别 id。"""
    if not label_path.is_file():
        return None
    for line in label_path.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if len(parts) >= 5:
            try:
                return int(parts[0])
            except ValueError:
                return None
    return None


def dist_tag(stem: str):
    """从文件名里取距离标记，如 b1_101_p0_d0.8_y+0_001 -> "0.8"。"""
    for token in stem.split("_"):
        if token.startswith("d") and len(token) > 1:
            try:
                float(token[1:])
                return token[1:]
            except ValueError:
                continue
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="交付包/识别/数据")
    ap.add_argument("--model", default="~/.ros/ican_recognize/best.pt")
    ap.add_argument("--split", default="val", choices=["val", "train"])
    ap.add_argument("--dist", default=None, help="只评估指定距离，如 0.8")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.20)
    args = ap.parse_args()

    import os

    from ultralytics import YOLO

    root = Path(args.data)
    model_path = Path(os.path.expanduser(args.model))
    img_dir = root / "images" / args.split
    lbl_dir = root / "labels" / args.split

    if not img_dir.is_dir():
        print(f"[失败] 找不到图片目录: {img_dir}")
        return 1
    if not model_path.is_file():
        print(f"[失败] 找不到模型: {model_path}")
        return 1

    print(f"模型: {model_path}")
    print(f"数据集: {img_dir}  (split={args.split})")
    if args.dist:
        print(f"距离筛选: {args.dist} m")
    print()

    model = YOLO(str(model_path))

    images = sorted(p for p in img_dir.iterdir() if p.suffix.lower() in IMG_EXT)
    if args.dist:
        images = [p for p in images if dist_tag(p.stem) == args.dist]
    if not images:
        print("[失败] 筛选后没有图片")
        return 1

    correct = 0
    total = 0
    per_class_total = Counter()
    per_class_correct = Counter()
    confusion = defaultdict(Counter)   # confusion[true][pred] += 1
    by_dist_total = Counter()
    by_dist_correct = Counter()
    no_det = 0

    for img in images:
        truth_id = read_first_class(lbl_dir / f"{img.stem}.txt")
        if truth_id is None or not (0 <= truth_id < len(CLASSES)):
            continue
        truth = CLASSES[truth_id]

        result = model.predict(str(img), imgsz=args.imgsz, conf=args.conf, verbose=False)[0]
        best_cls, best_conf = None, 0.0
        for box in result.boxes:
            c = float(box.conf[0])
            if c > best_conf:
                best_conf = c
                cid = int(box.cls[0])
                best_cls = CLASSES[cid] if 0 <= cid < len(CLASSES) else str(cid)

        total += 1
        per_class_total[truth] += 1
        tag = dist_tag(img.stem)
        if tag:
            by_dist_total[tag] += 1

        if best_cls is None:
            no_det += 1
            confusion[truth]["(未检出)"] += 1
            continue

        confusion[truth][best_cls] += 1
        if best_cls == truth:
            correct += 1
            per_class_correct[truth] += 1
            if tag:
                by_dist_correct[tag] += 1

    if total == 0:
        print("[失败] 没有可评估的样本")
        return 1

    print("=" * 62)
    print(f"总体分类准确率: {correct}/{total} = {correct / total * 100:.1f}%")
    if no_det:
        print(f"其中未检出任何目标: {no_det} 张（计为错误）")
    print("=" * 62)
    print()
    print(f"{'类别':<12}{'样本':>6}{'正确':>6}{'准确率':>9}")
    print("-" * 62)
    for cls in CLASSES:
        n = per_class_total[cls]
        if n == 0:
            continue
        c = per_class_correct[cls]
        print(f"{ZH[cls]:<10}{n:>8}{c:>7}{c / n * 100:>8.1f}%")
    print("-" * 62)
    print()

    print("混淆矩阵（行=真实，列=预测）")
    pred_cols = CLASSES + ["(未检出)"]
    # 注意：Python 3.10 的 f-string 表达式里不能出现反斜杠，
    # 所以表头左标签先单独赋值，不要写成 {'真实\\预测':<10}
    hdr_left = "真实\\预测"
    header = f"{hdr_left:<10}" + "".join(f"{ZH[c][:4]:>8}" for c in CLASSES) + f"{'未检出':>8}"
    print(header)
    for t in CLASSES:
        if per_class_total[t] == 0:
            continue
        row = f"{ZH[t]:<10}"
        for p_ in pred_cols:
            row += f"{confusion[t].get(p_, 0):>8}"
        print(row)
    print()

    if by_dist_total:
        print("按距离分档准确率")
        print("-" * 62)
        for tag in sorted(by_dist_total, key=lambda x: float(x)):
            n = by_dist_total[tag]
            c = by_dist_correct[tag]
            print(f"  {tag} m: {c}/{n} = {c / n * 100:.1f}%")
        print()

    acc = correct / total * 100
    print("=" * 62)
    if acc >= 95.0:
        print(f"[OK] 分类准确率 {acc:.1f}% >= 95%，满足 B-02 验收口径")
        return 0
    print(f"[未达标] 分类准确率 {acc:.1f}% < 95%（B-02 任务书要求）")
    print("  说明：本指标按「取最高置信度检测类别」计算，与 mAP50 不是同一口径；")
    print("        若与 mAP 差异较大，说明有低置信度误检在拉低分类准确率。")
    return 1


if __name__ == "__main__":
    sys.exit(main())
