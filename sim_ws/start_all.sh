#!/usr/bin/env bash
# 一键启动全部：仿真 → Nav2/AMCL → 识别 → 顺序任务
#
# 每步都做健康检查，任一步没通过就停下并提示，避免"带着半瘫的环境往下跑"
# （这正是 /scan 无数据 → AMCL 不发位姿 → 任务节点卡死的经典坑）。
#
# 用法：
#   bash start_all.sh            # GUI 模式（相机可用，识别需要）
#   bash start_all.sh --no-gui   # 无头模式（省资源，但相机可能不出图）
#   bash start_all.sh --no-recog # 不启动识别节点（只验导航链路）
#
# 查看日志：
#   tail -f logs/sim.log logs/nav.log logs/recog.log logs/mission.log
#
# 注意：本脚本**不能**使用 set -u（nounset）。
# ROS 2 的 setup.bash 会引用 $AMENT_TRACE_SETUP_FILES 等可能未定义的变量，
# 开启 nounset 后会直接报 "unbound variable" 并退出。

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG="$WS/logs"
mkdir -p "$LOG"

GUI=true
START_RECOG=1
for arg in "$@"; do
  case "$arg" in
    --no-gui)   GUI=false ;;
    --no-recog) START_RECOG=0 ;;
    -h|--help)  sed -n '2,17p' "$0"; exit 0 ;;
    *) echo "未知参数: $arg" >&2; exit 2 ;;
  esac
done

wait_topic() {  # $1=话题 $2=最长等待秒 $3=说明
  local topic=$1 limit=$2 desc=$3 waited=0
  while [ "$waited" -lt "$limit" ]; do
    if timeout 4 ros2 topic echo "$topic" --once > /dev/null 2>&1; then
      echo "  ✅ $desc ($topic) 有数据"
      return 0
    fi
    sleep 2
    waited=$((waited + 2))
  done
  echo "  ❌ $desc ($topic) 等了 ${limit}s 仍无数据"
  return 1
}

echo "=========== 0. 清理残留 ==========="
bash "$WS/stop_all.sh" || exit 1

echo
echo "=========== 1. 加载环境 ==========="
export RMW_FASTRTPS_DISABLE_SHM=1
source /opt/ros/humble/setup.bash
[ -f /usr/share/gazebo/setup.bash ] && source /usr/share/gazebo/setup.bash
[ -f /opt/ican_shoot_sim/setup.bash ] && source /opt/ican_shoot_sim/setup.bash
if [ ! -f "$WS/install/setup.bash" ]; then
  echo "找不到 $WS/install/setup.bash，请先 colcon build --symlink-install" >&2
  exit 1
fi
source "$WS/install/setup.bash"
echo "环境已加载"

echo
echo "=========== 2. 启动仿真 (gui:=$GUI) ==========="
ros2 launch ican_shoot_sim sim.launch.py gui:=$GUI > "$LOG/sim.log" 2>&1 &
sleep 12
wait_topic /scan 40 "雷达" || { echo "仿真异常，查看 $LOG/sim.log"; exit 1; }
wait_topic /odom 20 "里程计" || { echo "仿真异常，查看 $LOG/sim.log"; exit 1; }

echo
echo "=========== 3. 启动 Nav2 + AMCL ==========="
ros2 launch ican_example_navigation navigation.launch.py rviz:=false \
  > "$LOG/nav.log" 2>&1 &
echo "等待 Nav2 激活（约 20s）..."
sleep 20
# /amcl_pose 不作致命检查：nav2_amcl 在机器人静止时可能不更新、不发布位姿，
# 而任务节点自身有 /odom 与"无位姿也继续"的兜底。这里只提示。
if wait_topic /amcl_pose 30 "AMCL 位姿"; then
  :
else
  echo "  ⚠️ /amcl_pose 暂无数据（静止时 AMCL 可能不发布，任务节点会自动兜底）"
fi

echo "  检查 Nav2 action ..."
if ros2 action list 2>/dev/null | grep -q "navigate_to_pose"; then
  echo "  ✅ /navigate_to_pose 可用"
else
  echo "  ❌ 未见 /navigate_to_pose，任务节点会反复报「Nav2 action server 未就绪」"
  tail -20 "$LOG/nav.log"
  exit 1
fi

echo "  检查 map -> base_footprint 变换 ..."
# 注意：tf2_echo 是长驻命令，永远不自己退出，timeout 杀它必然返回非 0。
# 所以不能用退出码判断，要看它有没有真的打印出变换内容。
tf_out=$(timeout 8 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 || true)
if echo "$tf_out" | grep -q "Translation"; then
  echo "  ✅ TF 链通"
else
  echo "  ⚠️ TF 查询无输出（可能 AMCL 尚未发布 map->odom），继续但留意导航是否报 TF 错误"
fi

if [ "$START_RECOG" -eq 1 ]; then
  echo
  echo "=========== 4. 启动识别节点 ==========="
  ros2 launch ican_recognize recognize.launch.py > "$LOG/recog.log" 2>&1 &
  # YOLO 模型加载 + torch 初始化要几秒，必须轮询而不是固定 sleep
  recog_ok=0
  for _ in $(seq 1 25); do
    if grep -q "识别节点就绪" "$LOG/recog.log" 2>/dev/null; then recog_ok=1; break; fi
    sleep 2
  done
  if [ "$recog_ok" -eq 1 ]; then
    echo "  ✅ 识别节点就绪"
    grep "识别节点就绪" "$LOG/recog.log" | tail -1
  else
    echo "  ❌ 识别节点 50s 内未就绪，查看 $LOG/recog.log"
    tail -20 "$LOG/recog.log"
    exit 1
  fi
fi

echo
echo "=========== 5. 启动顺序任务 ==========="
ros2 launch ican_nav_tasks nav_tasks.launch.py > "$LOG/mission.log" 2>&1 &
sleep 5
if grep -q "顺序导航节点就绪" "$LOG/mission.log"; then
  echo "  ✅ 任务节点就绪（约 20s 后开始导航）"
else
  echo "  ❌ 任务节点未就绪，查看 $LOG/mission.log"
  tail -20 "$LOG/mission.log"
  exit 1
fi

echo
echo "==================================================="
echo "全部启动完成。跟踪日志："
echo "  tail -f $LOG/mission.log"
echo "关键行过滤："
echo "  grep -E '识别\\]|裁判\\]|发送导航目标' $LOG/mission.log"
echo "停止全部："
echo "  bash $WS/stop_all.sh"
echo "==================================================="
