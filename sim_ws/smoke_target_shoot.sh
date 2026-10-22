#!/usr/bin/env bash
# 靶位射击节点冒烟测试（VM 专用，轻量）
# 验证点：
#   1) target_shooter_node 能正常加载（不因重复 declare_parameter 崩溃）
#   2) moving_code_order=786、wheel_leaf_offset=1 生效
#   3) fire_wheel / fire_moving_3 服务能正常响应（dry_run，不实弹）
# 不启动 Nav2/识别，只起仿真 + 射击节点。
set +u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG="$WS/logs"
mkdir -p "$LOG"
SIM_LOG="$LOG/smoke_sim.log"
SHOOT_LOG="$LOG/smoke_shoot.log"

export LC_ALL=C.UTF-8
export LANG=C.UTF-8
export RMW_FASTRTPS_DISABLE_SHM=1
source /opt/ros/humble/setup.bash
[ -f /usr/share/gazebo/setup.bash ] && source /usr/share/gazebo/setup.bash
[ -f /opt/ican_shoot_sim/setup.bash ] && source /opt/ican_shoot_sim/setup.bash
source "$WS/install/setup.bash"

wait_topic() {  # $1=topic $2=limit $3=desc
  local t=$1 limit=$2 desc=$3 waited=0
  while [ "$waited" -lt "$limit" ]; do
    if timeout 4 ros2 topic echo "$t" --once >/dev/null 2>&1; then
      echo "  [OK] $desc ($t)"; return 0
    fi
    sleep 2; waited=$((waited+2))
  done
  echo "  [FAIL] $desc ($t) 超时"; return 1
}

echo "===== 0. 清理残留 ====="
pkill -f target_shooter_node 2>/dev/null
pkill -f gzserver 2>/dev/null
pkill -f target_motion_node 2>/dev/null
pkill -f shooter_node 2>/dev/null
sleep 2

echo "===== 1. 启动仿真（无头） ====="
ros2 launch ican_shoot_sim sim.launch.py gui:=false > "$SIM_LOG" 2>&1 &
SIM_PID=$!
echo "  仿真 pid=$SIM_PID，等待话题..."
sleep 12
RC=0
wait_topic /scan 60 "雷达" || RC=1
wait_topic /odom 30 "里程计" || RC=1
wait_topic /gazebo/model_states 30 "model_states" || RC=1
if [ "$RC" -ne 0 ]; then
  echo "仿真话题不全，看日志 tail:"; tail -30 "$SIM_LOG"; exit 1
fi

echo "===== 2. 启动射击节点（dry_run） ====="
ros2 launch ican_shoot target_shoot.launch.py dry_run:=true > "$SHOOT_LOG" 2>&1 &
SHOOT_PID=$!
sleep 8
# 等服务出现
SRV_OK=0
for _ in $(seq 1 15); do
  if ros2 service list 2>/dev/null | grep -q "/ican_shoot/fire_wheel"; then SRV_OK=1; break; fi
  sleep 2
done
if [ "$SRV_OK" -eq 0 ]; then
  echo "  [FAIL] /ican_shoot/fire_wheel 服务未出现，看日志:"; tail -40 "$SHOOT_LOG"; exit 1
fi
echo "  [OK] 射击节点已加载，/ican_shoot/fire_wheel 服务在线"

echo "===== 3. 校验参数 ====="
echo -n "  moving_code_order = "
ros2 param get /target_shooter_node moving_code_order 2>/dev/null || echo "(读取失败)"
echo -n "  wheel_leaf_offset = "
ros2 param get /target_shooter_node wheel_leaf_offset 2>/dev/null || echo "(读取失败)"

echo "===== 4. 触发 fire_wheel（旋转靶3号，dry_run） ====="
timeout 35 ros2 service call /ican_shoot/fire_wheel std_srvs/srv/Trigger 2>&1 | tail -5

echo "===== 5. 触发 fire_moving_3（移动靶7号，dry_run） ====="
timeout 35 ros2 service call /ican_shoot/fire_moving_3 std_srvs/srv/Trigger 2>&1 | tail -5

echo "===== 6. 射击节点日志尾部 ====="
tail -25 "$SHOOT_LOG"

echo "===== 完成（进程保持运行，便于继续排查；需清理时执行 stop_all.sh 或 kill 本脚本的 pid） ====="
echo "sim_pid=$SIM_PID shoot_pid=$SHOOT_PID"
