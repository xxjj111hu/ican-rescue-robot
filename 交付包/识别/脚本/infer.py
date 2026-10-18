#!/usr/bin/env python3
"""独立推理脚本（不依赖 ROS）。

用途：给一张图（或一个目录）做 5 类地面图片识别，输出类别、置信度、框位置。
可单独用来验证模型，也可以嵌进别的程序。

用法:
  python3 infer.py --source 图片.jpg
  python3 infer.py --source 某目录/ --save 输出目录/
  python3 infer.py --source 图片.jpg --json          # 只输出 JSON，方便程序调用

约定（与比赛一致）：
  类别顺序 trapped_people / rescue_camp / collapsed_ruins / damaged_vehicle / landslide
  推荐工作距离 0.8 m（相机到贴图），此时单帧识别最稳。
"""
import argparse
import json
import sys
from pathlib import Path

import cv2

WS = Path(__file__).resolve().parents[2] if "deliverables" in str(Path(__file__).resolve()) else Path(__file__).resolve().parents[1]
CLASS_ORDER = ["trapped_people", "rescue_camp", "collapsed_ruins", "damaged_vehicle", "landslide"]
CLASS_ZH = {"trapped_people": "被困人员", "rescue_camp": "救援营地", "collapsed_ruins": "坍塌废墟",
            "damaged_vehicle": "受损车辆", "landslide": "滑坡塌方"}
DEFAULT_MODEL = str(WS / "runs/detect/runs/ican_v6/weights/best.pt")


def load_model(path):
    from ultralytics import YOLO
    return YOLO(path)


def infer_one(model, img, imgsz=640, conf=0.2):
    r = model.predict(img, imgsz=imgsz, conf=conf, verbose=False)[0]
    best = None
    for b in r.boxes:
        c = float(b.conf[0])
        if best is None or c > best[0]:
            best = (c, int(b.cls[0]), [int(v) for v in b.xyxy[0].tolist()])
    if best is None:
        return None, 0.0, None
    c, cid, box = best
    return r.names[cid], c, box


def draw(img, name, conf, box):
    out = img.copy()
    if box is not None:
        x1, y1, x2, y2 = box
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 3)
        label = f"{CLASS_ZH.get(name, name)} {conf:.2f}"
        cv2.putText(out, label, (x1, max(20, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    return out


def main():
    ap = argparse.ArgumentParser(description="5 类地面图片识别（独立推理）")
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--source", required=True, help="图片路径或目录")
    ap.add_argument("--imgsz", type=int, default=640, help="推理分辨率（与训练一致 640）")
    ap.add_argument("--conf", type=float, default=0.20, help="置信度阈值")
    ap.add_argument("--save", default=None, help="把画框图保存到这个目录")
    ap.add_argument("--json", action="store_true", help="只输出 JSON")
    args = ap.parse_args()

    src = Path(args.source)
    if not src.exists():
        raise SystemExit(f"找不到: {src}")
    model = load_model(args.model)
    files = [src] if src.is_file() else sorted(
        [p for p in src.iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png", ".bmp")])
    if not files:
        raise SystemExit("目录里没有图片")

    if args.save:
        Path(args.save).mkdir(parents=True, exist_ok=True)

    results = []
    for f in files:
        img = cv2.imread(str(f))
        if img is None:
            continue
        name, conf, box = infer_one(model, img, args.imgsz, args.conf)
        results.append({"file": f.name, "class": name, "class_zh": CLASS_ZH.get(name, name),
                        "conf": round(conf, 4), "box": box})
        if not args.json:
            print(f"{f.name:40s} -> {CLASS_ZH.get(name, '无检测'):6s} conf={conf:.2f} box={box}")
        if args.save and box is not None:
            cv2.imwrite(str(Path(args.save) / f.name), draw(img, name, conf, box))

    if args.json:
        print(json.dumps(results, ensure_ascii=False, indent=2))
    else:
        hit = sum(1 for r in results if r["class"])
        print(f"\n共 {len(results)} 张，检出 {hit} 张")


if __name__ == "__main__":
    main()
