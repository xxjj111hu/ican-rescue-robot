#!/usr/bin/env python3
"""分类准确率评估（按 GT 框匹配，消除「选到别的贴图」的干扰）。

与 eval_classification_accuracy.py 的区别：
  - 那个：取全图置信度最高的检测，认定为该图的类别 → 一张图里有多张贴图时会选错目标，
          0.8 m 时目标被裁在画面底部、远处小贴图反而更清晰，于是数字暴跌（61%）。
  - 本脚本：对每个 GT 框，找 IoU 最大的检测框，比类别 → 衡量「模型认不认得出这张贴图」。

用法：
  python3 eval_classification_iou.py --model runs/detect/runs/ican_v10/weights/best.pt \
      --data 交付包/识别/数据 [--iou 0.3] [--dist 0.8]
"""
import argparse
import re
from collections import Counter, defaultdict
from pathlib import Path

CLS = ["trapped_people", "rescue_camp", "collapsed_ruins", "damaged_vehicle", "landslide"]
ZH = dict(zip(CLS, ["被困人员", "救援营地", "坍塌废墟", "受损车辆", "滑坡塌方"]))


def iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / ua if ua > 0 else 0.0


def dist_tag(stem):
    m = re.search(r"_d([\d.]+)_", stem)
    return m.group(1) if m else "?"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--data", default="交付包/识别/数据")
    ap.add_argument("--split", default="val")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.20)
    ap.add_argument("--iou", type=float, default=0.30, help="GT 与检测匹配的 IoU 下限")
    ap.add_argument("--dist", default=None, help="只看某个距离，如 0.8")
    args = ap.parse_args()

    from ultralytics import YOLO
    import cv2

    root = Path(args.data)
    model = YOLO(args.model)
    imgs = sorted((root / "images" / args.split).glob("*.jpg"))
    if args.dist:
        imgs = [p for p in imgs if dist_tag(p.stem) == args.dist]

    per_cls = defaultdict(lambda: [0, 0])
    per_dist = defaultdict(lambda: [0, 0])
    unmatched = 0
    for p in imgs:
        lab = root / "labels" / args.split / (p.stem + ".txt")
        if not lab.exists():
            continue
        img = cv2.imread(str(p))
        h, w = img.shape[:2]
        gts = []
        for ln in lab.read_text().splitlines():
            v = ln.split()
            if len(v) != 5:
                continue
            c = int(v[0]); cx, cy, bw, bh = map(float, v[1:5])
            gts.append((c, (cx - bw / 2) * w, (cy - bh / 2) * h,
                        (cx + bw / 2) * w, (cy + bh / 2) * h))
        r = model.predict(str(p), imgsz=args.imgsz, conf=args.conf, verbose=False)[0]
        dets = [(int(b.cls[0]), b.xyxy[0].tolist(), float(b.conf[0])) for b in r.boxes]
        tag = dist_tag(p.stem)
        for c, *box in gts:
            best, best_iou = None, 0.0
            for dc, dbox, dconf in dets:
                v = iou(box, dbox)
                if v > best_iou:
                    best, best_iou = dc, v
            per_cls[CLS[c]][1] += 1
            per_dist[tag][1] += 1
            if best is not None and best_iou >= args.iou:
                if best == c:
                    per_cls[CLS[c]][0] += 1
                    per_dist[tag][0] += 1
            else:
                unmatched += 1

    tot_c = sum(v[0] for v in per_cls.values())
    tot_n = sum(v[1] for v in per_cls.values())
    print(f"\n模型: {args.model}")
    print(f"数据: {root}/images/{args.split}   共 {len(imgs)} 张")
    print("=" * 60)
    print(f"总体分类准确率：{tot_c}/{tot_n} = {tot_c / max(tot_n, 1) * 100:.1f}%")
    print(f"（其中未找到 IoU≥{args.iou} 对应检测的 GT 框：{unmatched} 个，计为错误）")
    print("=" * 60)
    print(f"\n{'类别':<10}{'样本':>6}{'正确':>6}{'准确率':>8}")
    print("-" * 32)
    for c in CLS:
        k, n = per_cls[c]
        if n:
            print(f"{ZH[c]:<8}{n:>8}{k:>8}{k / n * 100:>9.1f}%")
    print("\n按距离分档")
    print("-" * 32)
    for d in sorted(per_dist, key=lambda x: float(x)):
        k, n = per_dist[d]
        print(f"  {d} m: {k}/{n} = {k / n * 100:.1f}%")


if __name__ == "__main__":
    main()
