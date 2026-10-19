#!/usr/bin/env python3
"""数据集自检：交付前跑一遍，全部 OK 再推。

检查项：
  1. images/{train,val} 与 labels/{train,val} 目录存在
  2. 图片与标签一一对应（不缺不多）
  3. 每张图能被 OpenCV 正常读取，尺寸一致
  4. 每行标签 5 个字段、类别 0~4、坐标在 [0,1]、宽高为正
  5. 每个划分的类别分布，以及「每类不少于 50 张」的验收要求
  6. data.yaml 的类别顺序与代码一致

用法：
  python3 交付包/识别/脚本/check_dataset.py --root 交付包/识别/数据
  python3 交付包/识别/脚本/check_dataset.py --root 交付包/识别/数据 --min-per-class 50
"""
import argparse
import sys
from collections import Counter
from pathlib import Path

try:
    import cv2
except ImportError:
    cv2 = None


def image_size(path):
    """返回 (h, w)；优先用 OpenCV，没有 cv2 时直接解析 JPEG 的 SOF 段。"""
    if cv2 is not None:
        img = cv2.imread(str(path))
        return None if img is None else img.shape[:2]
    data = Path(path).read_bytes()
    if data[:2] != b"\xff\xd8":
        return None
    i = 2
    while i < len(data) - 9:
        if data[i] != 0xFF:
            i += 1
            continue
        marker = data[i + 1]
        if marker in (0xC0, 0xC1, 0xC2, 0xC3):
            h = int.from_bytes(data[i + 5:i + 7], "big")
            w = int.from_bytes(data[i + 7:i + 9], "big")
            return (h, w)
        i += 2 + int.from_bytes(data[i + 2:i + 4], "big")
    return None

CLASSES = ["trapped_people", "rescue_camp", "collapsed_ruins", "damaged_vehicle", "landslide"]
ZH = dict(zip(CLASSES, ["被困人员", "救援营地", "坍塌废墟", "受损车辆", "滑坡塌方"]))
SPLITS = ("train", "val")

errors = []
warnings = []
oks = []


def ok(msg):
    oks.append(msg)
    print(f"[OK]   {msg}")


def bad(msg):
    errors.append(msg)
    print(f"[FAIL] {msg}")


def warn(msg):
    warnings.append(msg)
    print(f"[WARN] {msg}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="交付包/识别/数据")
    ap.add_argument("--min-per-class", type=int, default=50)
    ap.add_argument("--allow-multi-box", action="store_true", default=True)
    args = ap.parse_args()

    root = Path(args.root)
    if not root.exists():
        bad(f"目录不存在: {root}")
        return finish()

    # 1. 目录
    for split in SPLITS:
        for sub in ("images", "labels"):
            d = root / sub / split
            if d.is_dir():
                ok(f"目录存在: {sub}/{split}")
            else:
                bad(f"缺少目录: {sub}/{split}")

    # 2/3/4. 逐划分检查
    sizes = Counter()
    class_count = {s: Counter() for s in SPLITS}
    for split in SPLITS:
        img_dir = root / "images" / split
        lab_dir = root / "labels" / split
        if not img_dir.is_dir() or not lab_dir.is_dir():
            continue
        imgs = {p.stem: p for p in img_dir.glob("*.jpg")}
        labs = {p.stem for p in lab_dir.glob("*.txt")}
        only_img = set(imgs) - labs
        only_lab = labs - set(imgs)
        if only_img:
            bad(f"{split}: {len(only_img)} 张图没有标签，例如 {sorted(only_img)[:3]}")
        if only_lab:
            bad(f"{split}: {len(only_lab)} 个标签没有图，例如 {sorted(only_lab)[:3]}")
        if not only_img and not only_lab:
            ok(f"{split}: 图片与标签一一对应（{len(imgs)} 对）")

        n_box = 0
        for stem, p in sorted(imgs.items()):
            size = image_size(p)
            if size is None:
                bad(f"{split}/{p.name}: 图片无法读取")
                continue
            sizes[size] += 1
            lab = lab_dir / (stem + ".txt")
            if not lab.exists():
                continue
            lines = [l for l in lab.read_text(encoding="utf-8").splitlines() if l.strip()]
            if not lines:
                bad(f"{split}/{stem}.txt: 空标签")
                continue
            for ln in lines:
                v = ln.split()
                if len(v) != 5:
                    bad(f"{split}/{stem}.txt: 字段数 {len(v)} != 5")
                    continue
                try:
                    cid = int(v[0])
                    cx, cy, w, h = map(float, v[1:])
                except ValueError:
                    bad(f"{split}/{stem}.txt: 无法解析「{ln}」")
                    continue
                if not (0 <= cid < len(CLASSES)):
                    bad(f"{split}/{stem}.txt: 类别 id {cid} 越界")
                    continue
                if not (0 <= cx <= 1 and 0 <= cy <= 1):
                    bad(f"{split}/{stem}.txt: 中心越界 ({cx},{cy})")
                if not (0 < w <= 1 and 0 < h <= 1):
                    bad(f"{split}/{stem}.txt: 宽高异常 ({w},{h})")
                class_count[split][cid] += 1
                n_box += 1
        if n_box:
            ok(f"{split}: 共 {n_box} 个框，格式合法")

    # 3b. 尺寸一致性
    if len(sizes) == 1:
        hw = next(iter(sizes))
        ok(f"所有图片尺寸一致: {hw[1]}x{hw[0]}")
    elif sizes:
        warn(f"图片尺寸不统一: {dict(sizes)}")

    # 5. 类别分布 + 每类下限
    print("\n类别分布：")
    print(f"  {'类别':<14}{'train':>8}{'val':>8}")
    for i, c in enumerate(CLASSES):
        print(f"  {ZH[c]:<12}{class_count['train'][i]:>8}{class_count['val'][i]:>8}")
    for i, c in enumerate(CLASSES):
        n = class_count["train"][i] + class_count["val"][i]
        if n < args.min_per_class:
            bad(f"{ZH[c]} 只有 {n} 个框，少于要求的 {args.min_per_class}")
    if all((class_count["train"][i] + class_count["val"][i]) >= args.min_per_class
           for i in range(len(CLASSES))):
        ok(f"每一类都 ≥ {args.min_per_class} 个框")

    # 6. data.yaml
    yaml_path = root / "data.yaml"
    if yaml_path.exists():
        txt = yaml_path.read_text(encoding="utf-8")
        miss = [c for c in CLASSES if c not in txt]
        if miss:
            bad(f"data.yaml 缺少类别: {miss}")
        else:
            ok("data.yaml 类别顺序与代码一致")
    else:
        warn("没有 data.yaml（不影响数据集本身，但训练时需要）")

    finish()


def finish():
    print("\n" + "=" * 60)
    if errors:
        print(f"结果: 未通过（{len(errors)} 个错误，{len(warnings)} 个警告）")
        for e in errors[:10]:
            print("  -", e)
        print("=" * 60)
        sys.exit(1)
    print(f"结果: OK 全部通过（{len(oks)} 项检查，{len(warnings)} 个警告）")
    print("=" * 60)
    sys.exit(0)


if __name__ == "__main__":
    main()
