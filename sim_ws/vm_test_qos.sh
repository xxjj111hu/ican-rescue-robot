#!/usr/bin/env bash
# 模拟验证：识别 target 消息（TRANSIENT_LOCAL）→ 射击节点采用识别值
set +u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG="$WS/logs"; mkdir -p "$LOG"
export LC_ALL=C.UTF-8 LANG=C.UTF-8 RMW_FASTRTPS_DISABLE_SHM=1
source /opt/ros/humble/setup.bash
[ -f /usr/share/gazebo/setup.bash ] && source /usr/share/gazebo/setup.bash
[ -f /opt/ican_shoot_sim/setup.bash ] && source /opt/ican_shoot_sim/setup.bash
source "$WS/install/setup.bash"

pkill -f target_shooter_node 2>/dev/null; pkill -f gzserver 2>/dev/null
pkill -f target_motion 2>/dev/null; pkill -f 'sim.launch' 2>/dev/null
sleep 2

echo "== 1. 无头仿真 =="
ros2 launch ican_shoot_sim sim.launch.py gui:=false > "$LOG/q_sim.log" 2>&1 &
sleep 12
for i in $(seq 1 20); do
  timeout 4 ros2 topic echo /gazebo/model_states --once >/dev/null 2>&1 && break
  sleep 3
done
echo "  model_states OK"

echo "== 2. 射击节点（dry_run） =="
ros2 launch ican_shoot target_shoot.launch.py dry_run:=true > "$LOG/q_shoot.log" 2>&1 &
for i in $(seq 1 15); do
  ros2 service list 2>/dev/null | grep -q fire_wheel && break
  sleep 2
done
echo "  射击节点在线"

echo "== 3. 模拟识别消息：shoot_2 声明 + point=2 rotate=1 moving=6 =="
ros2 topic pub -1 /referee/current_target std_msgs/msg/String "data: 'shoot_2'" >/dev/null 2>&1
sleep 0.5
ros2 topic pub -1 /ican_recognize/target std_msgs/msg/String \
  "{data: '{\"class\": \"collapsed_ruins\", \"rotate_target\": 1, \"moving_target\": 6, \"point\": 2, \"conf\": 0.9, \"source\": \"sim\", \"t\": 0}' }" >/dev/null 2>&1
sleep 1.5

echo "== 4. 调 fire_wheel：期望日志出现『旋转靶 1 号』（识别值），而非『3 号』（default） =="
timeout 35 ros2 service call /ican_shoot/fire_wheel std_srvs/srv/Trigger 2>&1 | grep -o "message='[^']*'" | head -2
echo ""
echo "== 5. 调 fire_moving_3：期望『6 号码』（识别值 moving=6 → 右码），而非『7 号』 =="
timeout 35 ros2 service call /ican_shoot/fire_moving_3 std_srvs/srv/Trigger 2>&1 | grep -o "message='[^']*'" | head -2

echo "== 6. 射击节点收消息日志 =="
grep -E "收到识别靶位|已存档|射击点位" "$LOG/q_shoot.log" | tail -5
