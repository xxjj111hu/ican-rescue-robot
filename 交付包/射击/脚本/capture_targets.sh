#!/usr/bin/env bash
# 采集 4 个任务点面向各自靶位时的相机画面（射击算法设计的前提）。
#
# 背景
# ----
# 仿真没有提供 ar_track_alvar，/ar_target/* 话题不存在，所以旋转靶（5 个叶片）
# 与移动靶（每板 3 个 AR 码）的"指定靶位"识别必须由我们自己实现。
# 但用 AR 还是模板匹配、阈值多少，取决于靶面实际画了什么 ——
# 所以在写算法之前，先把靶子拍下来看。
#
# 依赖
# ----
# 仿真 + Nav2 已在运行，且**任务节点未启动**（用 start_all.sh --no-recog --no-mission）：
#     bash sim_ws/start_all.sh --no-recog --no-mission
#
# 用法
# ----
#     bash 交付包/射击/脚本/capture_targets.sh
#     bash 交付包/射击/脚本/capture_targets.sh --out /tmp/shoot_shots
#
# 输出
# ----
#   <out>/point1_ring.png      ①环形靶（点位1，yaw 0）
#   <out>/point2_wheel_00..05  ②旋转靶连拍 6 帧（间隔 0.8s，看 5 个叶片怎么转）
#   <out>/point3_moving3_00..05 ③移动靶3 连拍（间隔 1.5s，看往复运动）
#   <out>/point4_moving4_00..05 ④移动靶4 连拍
set -Eeo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../../.." && pwd)"
WS="$REPO/sim_ws"
OUT="/tmp/shoot_shots"

while [ $# -gt 0 ]; do
  case "$1" in
    --out) OUT="$2"; shift 2 ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "未知参数: $1" >&2; exit 2 ;;
  esac
done
mkdir -p "$OUT"

# 注意：不能 set -u —— ROS setup.bash 会引用未定义变量
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

GOTO="$SCRIPT_DIR/../../识别/脚本/goto_pose.py"
GRAB="$SCRIPT_DIR/grab_image.py"
if [ ! -f "$GOTO" ]; then
  echo "❌ 找不到 goto_pose.py: $GOTO" >&2
  exit 1
fi

# 停车位与车头朝向必须"正对各自靶位"（依据 land_shoot.world 的靶位坐标）
#   点位1 (0.031,-1.360) yaw 0     -> 环形靶  (1.575,-1.35)
#   点位2 (0.027,-2.700) yaw 0     -> 旋转靶  (1.575,-2.70)
#   点位3 (1.120,-1.914) yaw -90°  -> 移动靶3 (1.125,-2.97)   ← 注意是 -90°
#   点位4 (1.120,-0.785) yaw +90°  -> 移动靶4 (1.125, 0.27)
declare -a PX=(0.031 0.027 1.120 1.120)
declare -a PY=(-1.360 -2.700 -1.914 -0.785)
declare -a PYAWI=(0.0 0.0 -1.5708 1.5708)
declare -a NAME=(point1_ring point2_wheel point3_moving3 point4_moving4)
declare -a COUNT=(1 6 6 6)
declare -a INTERVAL=(0 0.8 1.5 1.5)

for i in 0 1 2 3; do
  n=$((i + 1))
  echo
  echo "=========== 点位 $n → ${NAME[$i]} ==========="
  echo "开往 (${PX[$i]}, ${PY[$i]}, yaw=${PYAWI[$i]}) ..."
  if ! python3 "$GOTO" --x "${PX[$i]}" --y "${PY[$i]}" --yaw "${PYAWI[$i]}"; then
    echo "⚠️ 点位 $n 导航失败，仍尝试抓图"
  fi
  sleep 2
  if [ "${COUNT[$i]}" -le 1 ]; then
    python3 "$GRAB" --out "$OUT/${NAME[$i]}.png" --timeout 15 || echo "⚠️ 点位 $n 抓图失败"
  else
    python3 "$GRAB" --out "$OUT/${NAME[$i]}" --count "${COUNT[$i]}" \
      --interval "${INTERVAL[$i]}" --timeout 15 || echo "⚠️ 点位 $n 连拍失败"
  fi
done

echo
echo "============================================================"
echo "采集完成，文件在: $OUT"
ls -l "$OUT" 2>/dev/null | sed -n '2,40p'
echo "============================================================"
echo "请把这些图发出来，据此决定 D-03/D-04 的靶位识别方案："
echo "  - 若靶面是标准 ArUco/ALVAR 码   → 用 OpenCV aruco 或自写解码"
echo "  - 若只是图片/花纹              → 用模板匹配（assets/point_targets 里有 5 类原图）"
echo "  - 环形靶（point1）检不出        → 调 HSV 阈值（黄心 10 环，向外 9/8/7/6）"
