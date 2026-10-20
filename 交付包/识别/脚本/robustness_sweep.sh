#!/usr/bin/env bash
# 鲁棒性验收驱动：多 seed 随机贴图 × 5 点位 × N 次（B-05 验收口径）
#
# 为什么需要它
# ------------
# 现有的 eval_compare_seeds.sh 有两处硬伤，换台机器就跑不了：
#   1) cd /home/longying01/demo_shoot_ws   —— 硬编码个人目录
#   2) V9=runs/detect/runs/detect/ican_v9-2/weights/best.pt
#      —— 路径里重复了一层 runs/detect，指向不存在的文件
# 本脚本按仓库实际布局自动定位，并把每轮原始记录写进 交付包/识别/报告/，
# 保证报告里的结论有可追溯的数据支撑。
#
# 用法
# ----
#   # ① 先跑基线（不换贴图、不需要 sudo，约 6 分钟）——验证 v9 在默认分配下的 5×10
#   bash 交付包/识别/脚本/robustness_sweep.sh --skip-randomize --tag v9_default
#
#   # ② 再跑多 seed 随机贴图（需要 sudo 写 /opt，约 7 分钟/seed）
#   sudo -v    # 先缓存凭据
#   bash 交付包/识别/脚本/robustness_sweep.sh --seeds 201 202 203 204 --tag v9
#
#   # 可选参数
#   --model <path>   指定模型（默认用 交付包/识别/模型/best.pt，会拷到 ~/.ros/ican_recognize/）
#   --no-model-copy  不覆盖 ~/.ros 下的模型
#   --dist 0.8       停车距离
#   --per-point 10   每点采集次数
#
# 注意
# ----
# 本脚本**不能**使用 set -u：ROS 2 的 setup.bash 会引用未定义变量导致退出。
set -Eeo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../../.." && pwd)"
WS="$REPO/sim_ws"
REPORT_DIR="$REPO/交付包/识别/报告"
mkdir -p "$REPORT_DIR"

SEEDS=""
TAG="v9"
SKIP_RAND=0
MODEL="$REPO/交付包/识别/模型/best.pt"
COPY_MODEL=1
DIST=0.8
PER_POINT=10
POINTS="1,2,3,4"

while [ $# -gt 0 ]; do
  case "$1" in
    --seeds)          SEEDS="$2"; shift 2 ;;
    --tag)            TAG="$2"; shift 2 ;;
    --skip-randomize) SKIP_RAND=1; shift ;;
    --model)          MODEL="$2"; shift 2 ;;
    --no-model-copy)  COPY_MODEL=0; shift ;;
    --dist)           DIST="$2"; shift 2 ;;
    --per-point)      PER_POINT="$2"; shift 2 ;;
    --points)         POINTS="$2"; shift 2 ;;
    -h|--help)        sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "未知参数: $1" >&2; exit 2 ;;
  esac
done

if [ "$SKIP_RAND" -eq 0 ] && [ -z "$SEEDS" ]; then
  echo "错误：需要 --seeds \"201 202 203 204\"，或加 --skip-randomize 只跑基线" >&2
  exit 2
fi

echo "============================================================"
echo " 鲁棒性验收（B-05 口径：5 点位 × ${PER_POINT} 次，>=90% 通过）"
echo " 仓库: $REPO"
echo " 模型: $MODEL"
echo " 模式: $([ "$SKIP_RAND" -eq 1 ] && echo '基线（不换贴图）' || echo "多 seed: $SEEDS")"
echo " 点位: $POINTS"
echo " 记录目录: $REPORT_DIR"
echo "============================================================"
echo

# ---------- 定位随机化脚本 ----------
RANDOMIZE=""
if [ "$SKIP_RAND" -eq 0 ]; then
  for cand in \
    "$REPO"/demo_shoot_ws/*/scripts/randomize_ground_textures.sh \
    "$REPO"/demo_shoot_ws/scripts/randomize_ground_textures.sh
  do
    if [ -f "$cand" ]; then RANDOMIZE="$cand"; break; fi
  done
  if [ -z "$RANDOMIZE" ]; then
    echo "❌ 找不到 randomize_ground_textures.sh" >&2
    echo "   可先只跑基线：bash $0 --skip-randomize --tag ${TAG}_default" >&2
    exit 1
  fi
  echo "随机化脚本: $RANDOMIZE"
  if ! sudo -n true 2>/dev/null; then
    echo "❌ 需要 sudo 免密缓存（随机化要写 /opt/ican_shoot_sim）" >&2
    echo "   请先执行：sudo -v    （或配置 NOPASSWD）" >&2
    exit 1
  fi
fi

# ---------- 准备模型 ----------
if [ "$COPY_MODEL" -eq 1 ] && [ -f "$MODEL" ]; then
  mkdir -p "$HOME/.ros/ican_recognize"
  cp "$MODEL" "$HOME/.ros/ican_recognize/best.pt"
  echo "已部署模型: $(ls -l "$HOME/.ros/ican_recognize/best.pt" | awk '{print $5" B"}')"
fi

SUMMARY="$REPORT_DIR/robustness_sweep_${TAG}.txt"
: > "$SUMMARY"
{
  echo "鲁棒性验收汇总  模型=$MODEL"
  echo "停车距离=${DIST} m  每点=${PER_POINT} 次  验收线=90%"
  echo "生成时间: $(date '+%Y-%m-%d %H:%M:%S')"
  echo "============================================================"
} >> "$SUMMARY"

run_one() {  # $1 = seed（基线传 default）
  local seed="$1"
  local out="$REPORT_DIR/acc_seed${seed}_${TAG}.txt"

  echo
  echo "------------------------------------------------------------"
  echo " seed=$seed"
  echo "------------------------------------------------------------"

  # 1) 停掉全部
  bash "$WS/stop_all.sh" >/dev/null 2>&1 || true
  sleep 2

  # 2) 换贴图（基线模式跳过）
  if [ "$SKIP_RAND" -eq 0 ]; then
    echo "[$seed] 随机化贴图 ..."
    ( cd "$(dirname "$RANDOMIZE")" && bash "$RANDOMIZE" --seed "$seed" ) \
      > "/tmp/rand_${seed}.log" 2>&1 || {
        echo "❌ 随机化失败，见 /tmp/rand_${seed}.log" >&2
        if grep -q "setup-permissions" "/tmp/rand_${seed}.log" 2>/dev/null; then
          echo "" >&2
          echo "   ⚠️ 首次运行需要先给仿真资源授权（一次即可）：" >&2
          echo "      bash $(dirname "$RANDOMIZE")/randomize_ground_textures.sh --setup-permissions" >&2
          echo "   之后再重跑本脚本。" >&2
        fi
        tail -20 "/tmp/rand_${seed}.log" >&2
        return 1
      }
    echo "[$seed] ✅ 贴图已分配"
  else
    echo "[$seed] 跳过随机化（使用当前 material 默认分配）"
  fi

  # 3) 启动仿真 + Nav2 + 识别，**不启动任务节点**
  echo "[$seed] 启动仿真/Nav2/识别 ..."
  if ! bash "$WS/start_all.sh" --no-mission > "/tmp/start_${seed}.log" 2>&1; then
    echo "❌ 启动失败，见 /tmp/start_${seed}.log" >&2
    tail -30 "/tmp/start_${seed}.log" >&2
    return 1
  fi
  echo "[$seed] ✅ 环境就绪"

  # 4) 跑 5×N 验收
  #    必须先 source ROS + 工作空间，否则 python 里 import rclpy / ican_recognize 会失败
  echo "[$seed] 开始 ${POINTS} 点位 × ${PER_POINT} 次 ..."
  set +e
  (
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    # shellcheck disable=SC1091
    source "$WS/install/setup.bash"
    set -u
    cd "$WS"
    ICAN_WS="$WS" python3 "$SCRIPT_DIR/acceptance_run.py" \
      --dist "$DIST" --per-point "$PER_POINT" --points "$POINTS" --seed "$seed" --out "$out"
  )
  local rc=$?
  set -e

  if [ ! -f "$out" ]; then
    echo "❌ 未生成结果文件: $out" >&2
    return 1
  fi

  # 5) 摘出总体行写进汇总
  local overall
  overall=$(grep -E "^总体:" "$out" | head -1)
  local verdict
  verdict=$(grep -E "^结论:" "$out" | head -1)
  {
    echo
    echo "### seed $seed"
    echo "$overall"
    echo "$verdict"
    grep -E "^点位[0-9]" "$out"
  } >> "$SUMMARY"

  echo "[$seed] 完成：$overall  $verdict"
  return "$rc"
}

FAILED=""
if [ "$SKIP_RAND" -eq 1 ]; then
  run_one "default" || FAILED="$FAILED default"
else
  for seed in $SEEDS; do
    run_one "$seed" || FAILED="$FAILED $seed"
  done
fi

# ---------- 汇总 ----------
echo
echo "============================================================"
echo " 汇总（$SUMMARY）"
echo "============================================================"
cat "$SUMMARY"
echo

PASS=$(grep -c "结论: 通过" "$SUMMARY" 2>/dev/null || true)
PASS=${PASS:-0}
TOTAL=$(grep -c "^### seed" "$SUMMARY" 2>/dev/null || true)
TOTAL=${TOTAL:-0}
echo "达标轮数: ${PASS}/${TOTAL}（验收要求：每轮 >=90%）"
if [ -n "$FAILED" ]; then
  echo "⚠️ 失败的 seed:$FAILED"
fi

bash "$WS/stop_all.sh" >/dev/null 2>&1 || true
echo "已停止全部节点。"
[ "$PASS" = "$TOTAL" ] && [ "$TOTAL" != "0" ] && exit 0 || exit 1
