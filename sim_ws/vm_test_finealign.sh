#!/usr/bin/env bash
# 验证前爬+微调状态机：开 enable_shooting（dry_run 射击），观察 [前爬]/[微调]/[对准] 流转
set +u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG="$WS/logs"; mkdir -p "$LOG"
export LC_ALL=C.UTF-8 LANG=C.UTF-8 RMW_FASTRTPS_DISABLE_SHM=1
source /opt/ros/humble/setup.bash
[ -f /usr/share/gazebo/setup.bash ] && source /usr/share/gazebo/setup.bash
[ -f /opt/ican_shoot_sim/setup.bash ] && source /opt/ican_shoot_sim/setup.bash
source "$WS/install/setup.bash"

bash "$WS/stop_all.sh" >/dev/null 2>&1
sleep 2

echo "== 1. xvfb 无头仿真 =="
nohup xvfb-run -a ros2 launch ican_shoot_sim sim.launch.py gui:=false > "$LOG/f_sim.log" 2>&1 &
sleep 15
for i in $(seq 1 30); do timeout 4 ros2 topic echo /camera/image --once >/dev/null 2>&1 && break; sleep 3; done
echo "  camera OK"

echo "== 2. Nav2 =="
nohup ros2 launch ican_example_navigation navigation.launch.py rviz:=false > "$LOG/f_nav.log" 2>&1 &
for i in $(seq 1 20); do ros2 action list 2>/dev/null | grep -q navigate_to_pose && break; sleep 3; done
echo "  nav2 OK"

echo "== 3. 射击节点（dry_run） =="
nohup ros2 launch ican_shoot shoot_all.launch.py dry_run:=true > "$LOG/f_shoot.log" 2>&1 &
for i in $(seq 1 15); do ros2 service list 2>/dev/null | grep -q fire_wheel && break; sleep 2; done
echo "  shoot OK"

echo "== 4. template 识别 =="
nohup ros2 launch ican_recognize recognize.launch.py mode:=template > "$LOG/f_recog.log" 2>&1 &
sleep 5

echo "== 5. 顺序任务（enable_shooting=true） =="
nohup ros2 launch ican_nav_tasks nav_tasks.launch.py enable_shooting:=true > "$LOG/f_mission.log" 2>&1 &
sleep 10
grep -q "顺序导航节点就绪" "$LOG/f_mission.log" && echo "  mission OK"
echo "== 已启动，观察 [前爬]/[微调]/[对准] 流转 =="
