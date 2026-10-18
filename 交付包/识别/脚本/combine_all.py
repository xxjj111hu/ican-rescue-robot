"""把三份人工标注合并：老数据(172) + 第一批(150) + 第二批(150)。

老数据: dataset/{images,labels}/{train,val}
第一批: dataset_yolo/{images,labels}/{train,val}
第二批: dataset_yolo_b/{images,labels}/{train,val}（第二批标完后由 build_yolo_from_labels.py 生成）

输出: dataset_all/（验证集 = 老 val + 两批的 val seed）
用法: python3 scripts/combine_all.py
"""
import shutil
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
CLS = ["trapped_people", "rescue_camp", "collapsed_ruins", "damaged_vehicle", "landslide"]


def copy_split(img_dir: Path, lab_dir: Path, dst: Path, split: str, prefix: str):
    n = 0
    if not img_dir.exists():
        return 0
    for img in sorted(img_dir.glob("*.jpg")):
        lab = lab_dir / (img.stem + ".txt")
        if not lab.exists():
            continue
        name = f"{prefix}{img.stem}"
        shutil.copy2(img, dst / "images" / split / f"{name}.jpg")
        shutil.copy2(lab, dst / "labels" / split / f"{name}.txt")
        n += 1
    return n


def main():
    out = WS / "dataset_all"
    if out.exists():
        shutil.rmtree(out)
    for s in ("train", "val"):
        (out / "images" / s).mkdir(parents=True, exist_ok=True)
        (out / "labels" / s).mkdir(parents=True, exist_ok=True)

    parts = [
        ("old", WS / "dataset"),
        ("b1", WS / "dataset_yolo"),
        ("b2", WS / "dataset_yolo_b"),
        ("b3", WS / "dataset_yolo_c"),
    ]
    total = {"train": 0, "val": 0}
    for tag, base in parts:
        for split in ("train", "val"):
            n = copy_split(base / "images" / split, base / "labels" / split, out, split, f"{tag}_")
            total[split] += n
            if n:
                print(f"{tag}/{split}: {n}")

    yaml = [f"path: {out}", "train: images/train", "val: images/val", "nc: 5", "names:"]
    yaml += [f"  {i}: {c}" for i, c in enumerate(CLS)]
    (out / "data.yaml").write_text("\n".join(yaml) + "\n", encoding="utf-8")
    print(f"\n合计 train={total['train']} val={total['val']}")
    print(f"已写出 {out}/data.yaml")


if __name__ == "__main__":
    main()
