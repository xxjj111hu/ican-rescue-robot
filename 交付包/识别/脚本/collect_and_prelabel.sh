#!/usr/bin/env bash
# 补采一批数据并生成预标注工程（一条命令跑完）
#
# 用法：
#   bash scripts/collect_and_prelabel.sh                          # 默认 seed 111/112/113 -> dataset_v2d / label_ws4
#   bash scripts/collect_and_prelabel.sh 204 200 203              # 指定 seed
#   bash scripts/collect_and_prelabel.sh 204 200 --out dataset_v2e --pre label_ws5
#   bash scripts/collect_and_prelabel.sh 204 200 --quick          # 快速版：每格 3 帧
#
# 弱变体对应的 seed（把该变体放在可导航点位 1~4）：
#   rescue_camp_01.png    : 200 202 203 204 205 206 208 210
#   collapsed_ruins_02.jpg: 202 204 208 210 211 213 216 218
#   trapped_people_02.jpg : 200 201 202 203 204 206 207 211
#   landslide_01.jpg      : 200 203 204 205 206 207 209 210
#   → 推荐 204 200 203 这三个，四个弱变体全覆盖
#
# 产出：
#   dataset_v2d/   原始采集（含 labels.jsonl）
#   label_ws4/     预标注工程（图 + 框 + classes.txt，标签就在图片旁边）
cd "$(dirname "$0")/.." || exit 1

SEEDS=()
QUICK=0
OUTDIR=dataset_v2d
PREDIR=label_ws4
DISTS="0.7 0.8 0.9"

while [ $# -gt 0 ]; do
  case "$1" in
    --quick) QUICK=1; shift ;;
    --out)   OUTDIR="$2"; shift 2 ;;
    --pre)   PREDIR="$2"; shift 2 ;;
    --dists) DISTS="$2"; shift 2 ;;
    *)       SEEDS+=("$1"); shift ;;
  esac
done
if [ ${#SEEDS[@]} -eq 0 ]; then SEEDS=(111 112 113); fi

FRAMES=8
[ "$QUICK" = "1" ] && FRAMES=3

source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=============================================="
echo " 补采：seed ${SEEDS[*]}"
echo " 距离 $DISTS m，每格 $FRAMES 帧"
echo " 原始数据 -> $OUTDIR    预标注 -> $PREDIR"
echo "=============================================="

python3 scripts/auto_collect.py --seeds "${SEEDS[@]}" \
  --distances $DISTS --yaws -20 0 20 --frames "$FRAMES" --out "$OUTDIR"

echo
echo "=============================================="
echo " 生成预标注工程 $PREDIR"
echo "=============================================="
rm -rf "$PREDIR"
python3 scripts/make_prelabels.py --src "$OUTDIR" --out "$PREDIR"

cp "$PREDIR"/labels/*.txt "$PREDIR"/images/ 2>/dev/null
cp "$PREDIR"/classes.txt "$PREDIR"/labels/classes.txt 2>/dev/null
cp "$PREDIR"/classes.txt "$PREDIR"/images/classes.txt 2>/dev/null

cat > "$PREDIR"/打开我.sh <<EOF
#!/usr/bin/env bash
cd "\$(dirname "\$0")" || exit 1
echo "图片目录: \$(pwd)/images"
echo "保存目录已预设为: \$(pwd)/images"
labelImg images predefined_classes.txt
EOF
chmod +x "$PREDIR"/打开我.sh

python3 - "$PREDIR" <<'PY'
import pickle, sys
from pathlib import Path
sys.path.insert(0, "/usr/local/lib/python3.10/dist-packages")
predir = Path(sys.argv[1]).resolve()
try:
    from libs.constants import SETTING_SAVE_DIR, SETTING_LABEL_FILE_FORMAT
    from libs.labelFile import LabelFileFormat
    pickle.dump({SETTING_SAVE_DIR: str(predir / "images"),
                 SETTING_LABEL_FILE_FORMAT: LabelFileFormat.YOLO},
                open(str(Path.home() / ".labelImgSettings.pkl"), "wb"),
                pickle.HIGHEST_PROTOCOL)
    print(f"已预设 labelImg 保存目录 -> {predir}/images")
except Exception as e:
    print("预设失败（不影响标注，手动选目录即可）:", e)
PY

N=$(ls "$PREDIR"/images/*.jpg 2>/dev/null | wc -l)
echo
echo "=============================================="
echo " 完成"
echo "  原始数据: $OUTDIR（$(find "$OUTDIR" -name '*.jpg' 2>/dev/null | wc -l) 帧）"
echo "  预标注:   $PREDIR（$N 张）"
echo
echo "  开始标注:  bash $PREDIR/打开我.sh"
echo "  标完后:    python3 scripts/build_yolo_from_labels.py --src $PREDIR --out dataset_yolo_e --drop-empty"
echo "            python3 scripts/combine_all.py"
echo "=============================================="
