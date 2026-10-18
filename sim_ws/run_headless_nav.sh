#!/usr/bin/env bash
# 无头仿真 + 导航 + 顺序任务（任务13/14）+ 限速监控（任务12）
# 关相机渲染以节省 CPU，保证 AMCL/Nav2 稳定（无需识别时用此模式）。
export RMW_FASTRTPS_DISABLE_SHM=1
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/Ican_fly/fastrtps_no_shm.xml
source /opt/ros/humble/setup.bash
[ -f /usr/share/gazebo/setup.bash ] && source /usr/share/gazebo/setup.bash
source /opt/ican_shoot_sim/setup.bash
WS=~/Ican_fly/demo_shoot_ws
source "$WS/install/setup.bash"
LOG="$WS/logs"; mkdir -p "$LOG"
echo "[$(date)] launching sim (headless, SHM disabled)"
ros2 launch ican_shoot_sim sim.launch.py gui:=false > "$LOG/sim.log" 2>&1 &
disown
sleep 9
echo "[$(date)] launching navigation (no rviz)"
ros2 launch ican_example_navigation navigation.launch.py rviz:=false > "$LOG/nav.log" 2>&1 &
disown
sleep 7
echo "[$(date)] launching nav_tasks (sequential + speed monitor)"
ros2 launch ican_nav_tasks nav_tasks.launch.py > "$LOG/nav_tasks.log" 2>&1 &
disown
echo "[$(date)] all launched"
