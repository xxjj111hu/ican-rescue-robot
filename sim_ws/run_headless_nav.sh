#!/usr/bin/env bash
# 无头仿真 + 导航 + 顺序任务（识别位→进入位→射击声明→返航）一键启动
#
# 用法：
#   bash run_headless_nav.sh                 # 默认无头（不可见相机，省 CPU）
#   bash run_headless_nav.sh --with-camera   # 用 xvfb 让相机出图（识别需要）
#
# 说明：
#   - 无头模式下 Gazebo 拿不到渲染上下文，/camera/image 不会出图。
#     需要测识别时必须加 --with-camera（依赖：sudo apt install xvfb mesa-utils
#     libgl1-mesa-dri libegl1-mesa）。
#   - 本脚本不再写死个人路径：工作空间由脚本自身位置推导。
#     sim_ws/
#       run_headless_nav.sh     ← 本脚本
#       install/setup.bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$SCRIPT_DIR"
LOG="$WS/logs"
mkdir -p "$LOG"

WITH_CAMERA=0
for arg in "$@"; do
  case "$arg" in
    --with-camera) WITH_CAMERA=1 ;;
    -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
    *) echo "未知参数: $arg（可用：--with-camera）" >&2; exit 2 ;;
  esac
done

# 禁用 FastRTPS 共享内存：否则 /initialpose、TF、AMCL 位姿偶发丢包，
# 表现为 Nav2 间歇性"导航失败"。只设环境变量即可，不强制依赖外部 profile 文件。
export RMW_FASTRTPS_DISABLE_SHM=1
PROFILE="$HOME/.ros/fastrtps_no_shm.xml"
if [ -f "$PROFILE" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE"
fi

source /opt/ros/humble/setup.bash
[ -f /usr/share/gazebo/setup.bash ] && source /usr/share/gazebo/setup.bash
[ -f /opt/ican_shoot_sim/setup.bash ] && source /opt/ican_shoot_sim/setup.bash

if [ ! -f "$WS/install/setup.bash" ]; then
  echo "找不到 $WS/install/setup.bash —— 请先在 $WS 下执行：" >&2
  echo "  colcon build --symlink-install" >&2
  exit 1
fi
source "$WS/install/setup.bash"

if [ "$WITH_CAMERA" -eq 1 ]; then
  if ! command -v xvfb-run >/dev/null 2>&1; then
    echo "带相机模式需要 xvfb，请先执行：" >&2
    echo "  sudo apt install -y xvfb mesa-utils libgl1-mesa-dri libegl1-mesa" >&2
    exit 1
  fi
  SIM_CMD=(xvfb-run -a ros2 launch ican_shoot_sim sim.launch.py gui:=false)
  echo "[$(date)] launching sim (headless + xvfb，相机可用)"
else
  SIM_CMD=(ros2 launch ican_shoot_sim sim.launch.py gui:=false)
  echo "[$(date)] launching sim (headless，相机不可用)"
fi

"${SIM_CMD[@]}" > "$LOG/sim.log" 2>&1 &
disown
sleep 9

echo "[$(date)] launching navigation (no rviz)"
ros2 launch ican_example_navigation navigation.launch.py rviz:=false \
  > "$LOG/nav.log" 2>&1 &
disown
sleep 7

echo "[$(date)] launching nav_tasks (sequential + speed monitor)"
ros2 launch ican_nav_tasks nav_tasks.launch.py > "$LOG/nav_tasks.log" 2>&1 &
disown

echo "[$(date)] all launched"
echo "日志：$LOG/{sim,nav,nav_tasks}.log"
echo "识别节点请另行启动：ros2 launch ican_recognize recognize.launch.py"
