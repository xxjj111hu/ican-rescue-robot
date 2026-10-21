#!/usr/bin/env bash
# =============================================================================
# competition_entry.sh —— 比赛测评平台入口脚本（唯一提交给平台的 .sh）
#
# 设计依据：
#   - 官方《ROS 仿真平台操作手册》：上传 src 压缩包 + 本 .sh，服务器自动编译并
#     无头运行。启动脚本必须是 .sh 格式。
#   - 队友 D-05 联调验收报告里跑通的组合：sim + nav + 识别 + shoot_all(实弹)
#     + nav_tasks(enable_shooting) 即能完整得分。
#
# 本脚本与本地 start_all.sh 的区别（专为平台做的改造）：
#   1. 一律无头：仿真 gui:=false（平台无显示器，开 GUI 必卡死）。
#   2. 用 xvfb 包住仿真：无头 Gazebo 拿不到渲染上下文，/camera/image 不出图，
#      识别必然失败；xvfb 提供虚拟显示让相机正常出图。
#      → 若平台云端已原生支持无头相机（不需要 xvfb），用 --no-xvfb 关掉即可。
#   3. 实弹：shoot_all.launch.py dry_run:=false（提供开火服务并真发射）。
#   4. 开射击：nav_tasks enable_shooting:=true（任务节点调用开火服务）。
#   5. 强制 UTF-8 + 关 FastRTPS SHM：修 nav2_params.yaml 中文注释解析失败、
#      以及 TF/AMCL 偶发丢包。
#
# 用法：
#   bash competition_entry.sh            # 默认：无头 + xvfb 相机 + 实弹
#   bash competition_entry.sh --no-xvfb  # 平台若原生支持无头相机时用
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$SCRIPT_DIR"
LOG="$WS/logs"
mkdir -p "$LOG"

USE_XVFB=1
for arg in "$@"; do
  case "$arg" in
    --no-xvfb) USE_XVFB=0 ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "未知参数: $arg（可用：--no-xvfb）" >&2; exit 2 ;;
  esac
done

# ---------------------------------------------------------------------------
# 0. 清理残留（避免上次运行的服务没退干净）
# ---------------------------------------------------------------------------
if [ -f "$WS/stop_all.sh" ]; then
  bash "$WS/stop_all.sh" >/dev/null 2>&1 || true
fi

# ---------------------------------------------------------------------------
# 1. 加载环境（关键环境变量，缺一不可）
# ---------------------------------------------------------------------------
export LC_ALL=C.UTF-8
export LANG=C.UTF-8
# 禁用 FastRTPS 共享内存：否则 /initialpose、TF、AMCL 位姿偶发丢包，
# 表现为 Nav2 间歇性"导航失败"。
export RMW_FASTRTPS_DISABLE_SHM=1

source /opt/ros/humble/setup.bash
[ -f /usr/share/gazebo/setup.bash ] && source /usr/share/gazebo/setup.bash
# 官方仿真包平台已预装，不要打进 src；这里只 source 它的环境。
[ -f /opt/ican_shoot_sim/setup.bash ] && source /opt/ican_shoot_sim/setup.bash

if [ ! -f "$WS/install/setup.bash" ]; then
  echo "找不到 $WS/install/setup.bash —— 服务器应先编译 src 生成 install/。" >&2
  exit 1
fi
source "$WS/install/setup.bash"
echo "[$(date)] 环境已加载"

# ---------------------------------------------------------------------------
# 2. 启动仿真（无头；xvfb 包住以出相机图）
# ---------------------------------------------------------------------------
if [ "$USE_XVFB" -eq 1 ]; then
  if ! command -v xvfb-run >/dev/null 2>&1; then
    echo "⚠️ 需要 xvfb 但找不到 xvfb-run，相机可能不出图（识别会失败）。" >&2
    echo "   若平台原生支持无头相机，请用 --no-xvfb 重跑；否则需安装：" >&2
    echo "   sudo apt install -y xvfb mesa-utils libgl1-mesa-dri libegl1-mesa" >&2
  fi
  SIM_CMD=(xvfb-run -a ros2 launch ican_shoot_sim sim.launch.py gui:=false)
  echo "[$(date)] launching sim (headless + xvfb，相机可用)"
else
  SIM_CMD=(ros2 launch ican_shoot_sim sim.launch.py gui:=false)
  echo "[$(date)] launching sim (headless，依赖平台原生相机)"
fi
"${SIM_CMD[@]}" > "$LOG/sim.log" 2>&1 &
disown

# 等待雷达/里程计（核心，失败则退出）
wait_topic() {
  local topic=$1 limit=$2 desc=$3 waited=0
  while [ "$waited" -lt "$limit" ]; do
    if timeout 4 ros2 topic echo "$topic" --once > /dev/null 2>&1; then
      echo "  ✅ $desc ($topic) 有数据"; return 0
    fi
    sleep 2; waited=$((waited + 2))
  done
  echo "  ❌ $desc ($topic) 等了 ${limit}s 仍无数据"; return 1
}
sleep 12
wait_topic /scan 40 "雷达"  || { echo "仿真异常，查看 $LOG/sim.log"; exit 1; }
wait_topic /odom 20 "里程计" || { echo "仿真异常，查看 $LOG/sim.log"; exit 1; }

# ---------------------------------------------------------------------------
# 3. 启动 Nav2 + AMCL（无 RViz，平台无显示器）
# ---------------------------------------------------------------------------
ros2 launch ican_example_navigation navigation.launch.py rviz:=false \
  > "$LOG/nav.log" 2>&1 &
echo "等待 Nav2 激活（约 20s）..."
sleep 20
nav_ok=0; waited=0
while [ "$waited" -lt 60 ]; do
  if ros2 action list 2>/dev/null | grep -q "navigate_to_pose"; then nav_ok=1; break; fi
  sleep 3; waited=$((waited + 3))
done
if [ "$nav_ok" -eq 1 ]; then echo "  ✅ /navigate_to_pose 可用"; else
  echo "  ❌ Nav2 未就绪，查看 $LOG/nav.log"; exit 1
fi

# ---------------------------------------------------------------------------
# 4. 启动识别节点（订阅 /camera/image —— 必须靠 xvfb 出图）
# ---------------------------------------------------------------------------
ros2 launch ican_recognize recognize.launch.py > "$LOG/recog.log" 2>&1 &
recog_ok=0
for _ in $(seq 1 25); do
  if grep -q "识别节点就绪" "$LOG/recog.log" 2>/dev/null; then recog_ok=1; break; fi
  sleep 2
done
if [ "$recog_ok" -eq 1 ]; then echo "  ✅ 识别节点就绪"; else
  echo "  ❌ 识别节点未就绪，查看 $LOG/recog.log"; exit 1
fi

# ---------------------------------------------------------------------------
# 5. 启动射击节点（实弹：dry_run:=false，提供 /ican_shoot/fire_* 服务）
# ---------------------------------------------------------------------------
ros2 launch ican_shoot shoot_all.launch.py dry_run:=false \
  > "$LOG/shoot.log" 2>&1 &
echo "  ℹ️ 射击节点已启动（实弹 dry_run:=false）"
sleep 3

# ---------------------------------------------------------------------------
# 6. 启动顺序任务（enable_shooting:=true → 调用开火服务 + 识别/返航闭环）
# ---------------------------------------------------------------------------
ros2 launch ican_nav_tasks nav_tasks.launch.py enable_shooting:=true \
  > "$LOG/mission.log" 2>&1 &
sleep 5
if grep -q "顺序导航节点就绪" "$LOG/mission.log" 2>/dev/null; then
  echo "  ✅ 任务节点就绪（机器人开始自主跑分）"
else
  echo "  ❌ 任务节点未就绪，查看 $LOG/mission.log"; exit 1
fi

echo
echo "==================================================="
echo "全部启动完成，机器人自主执行：识别 → 进点 → 射击声明 → 返航"
echo "日志：$LOG/{sim,nav,recog,shoot,mission}.log"
echo "关键行过滤："
echo "  grep -E '识别\\]|裁判\\]|发送导航目标|shoot' $LOG/mission.log"
echo "停止全部：bash $WS/stop_all.sh"
echo "==================================================="
