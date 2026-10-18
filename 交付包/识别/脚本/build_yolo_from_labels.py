#!/usr/bin/env python3
"""把 labelImg 修正后的标签整理成 YOLO 训练集（按 seed 划分，避免同分布自测）。

输入：label_ws/images + label_ws/labels（文件名前缀是 seed，如 101_p0_...）
输出：dataset_yolo/
        images/train, images/val, labels/train, labels/val, data.yaml
用法：
  python3 scripts/build_yolo_from_labels.py
  cd ~/demo_shoot_ws && yolo detect train data=dataset_yolo/data.yaml model=yolo11n.pt \
      imgsz=640 epochs=40 batch=8 device=cpu name=ican_v3
"""
import argparse
import json
import random
import shutil
from collections import Counter
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
CLASSES = ["trapped_people", "rescue_camp", "collapsed_ruins", "damaged_vehicle", "landslide"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=str(WS / "label_ws"))
    ap.add_argument("--out", default=str(WS / "dataset_yolo"))
    ap.add_argument("--val-seed", type=int, default=None, help="留作验证的 seed；默认文件名里出现最多的 seed 之外的那个")
    ap.add_argument("--drop-empty", action="store_true", help="跳过无框图片和过小的框")
    ap.add_argument("--min-px", type=float, default=20.0, help="框最小宽度(px)，更小的丢弃")
    args = ap.parse_args()

    src = Path(args.src)
    out = Path(args.out)
    imgs = sorted((src / "images").glob("*.jpg"))
    if not imgs:
        raise SystemExit(f"{src}/images 里没有图片")

    def seed_of(p):
        return int(p.stem.split("_")[0])

    seeds = sorted({seed_of(p) for p in imgs})
    if args.val_seed is None:
        val_seed = seeds[-1]
    else:
        val_seed = args.val_seed

    for sub in ["images/train", "images/val", "labels/train", "labels/val"]:
        (out / sub).mkdir(parents=True, exist_ok=True)

    cnt = Counter()
    skipped = 0
    n_box = 0
    empty = 0
    for p in imgs:
        split = "val" if seed_of(p) == val_seed else "train"
        lab = (src / "labels" / (p.stem + ".txt")).read_text(encoding="utf-8").strip()
        if args.drop_empty:
            kept = []
            for ln in lab.splitlines():
                if not ln.strip():
                    continue
                v = ln.split()
                if len(v) == 5 and float(v[3]) * 800 >= args.min_px and float(v[4]) * 800 >= 6:
                    kept.append(ln)
            lab = "\n".join(kept)
            if not lab:
                skipped += 1
                continue
        if not lab:
            empty += 1
        else:
            n_box += len([l for l in lab.splitlines() if l.strip()])
        shutil.copy2(p, out / "images" / split / p.name)
        (out / "labels" / split / (p.stem + ".txt")).write_text(lab + ("\n" if lab else ""),
                                                                 encoding="utf-8")
        cnt[split] += 1

    yaml = [
        f"path: {out}",
        "train: images/train",
        "val: images/val",
        "nc: 5",
        "names:",
    ] + [f"  {i}: {n}" for i, n in enumerate(CLASSES)]
    (out / "data.yaml").write_text("\n".join(yaml) + "\n", encoding="utf-8")

    print(f"图片: train={cnt['train']} val={cnt['val']}（验证 seed={val_seed}）")
    print(f"框总数={n_box}，无框图片={empty}")
    if args.drop_empty:
        print(f"已跳过无有效框的图片: {skipped}")
    if empty:
        print("!! 有图片没有框，训练时会被当成背景图；如果它们其实有贴图，请补画后再打包")
    print(f"已写出 {out}/data.yaml")
    print("\n训练命令：")
    print(f"  cd {WS} && yolo detect train data={out}/data.yaml model=yolo11n.pt \\")
    print("       imgsz=640 epochs=40 batch=8 device=cpu name=ican_v3")


if __name__ == "__main__":
    main()
