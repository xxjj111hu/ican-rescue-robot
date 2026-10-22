#!/usr/bin/env bash
# xvfb 无头全链路测试：验证 template 兜底识别的实际分类效果
set +u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG="$WS/logs"; mkdir -p "$LOG"
export LC_ALL=C.UTF-8 LANG=C.UTF-8 RMW_FASTRTPS_DISABLE_SHM=1
source /opt/ros/humble/setup.bash
[ -f /usr/share/gazebo/setup.bash ] && source /usr/share/gazebo/setup.bash
[ -f /opt/ican_shoot_sim/setup.bash ] && source /opt/ican_shoot_sim/setup.bash
source "$WS/install/setup.bash"

echo "== 0. 清理 =="
bash "$WS/stop_all.sh" >/dev/null 2>&1
sleep 2

echo "== 1. xvfb 无头仿真（相机出图） =="
nohup xvfb-run -a ros2 launch ican_shoot_sim sim.launch.py gui:=false \
  > "$LOG/t_sim.log" 2>&1 &
sleep 15
for i in $(seq 1 30); do
  if timeout 4 ros2 topic echo /camera/image --once >/dev/null 2>&1; then
    echo "  [OK] /camera/image 有数据"; break
  fi
  sleep 3
done

echo "== 2. Nav2 =="
nohup ros2 launch ican_example_navigation navigation.launch.py rviz:=false \
  > "$LOG/t_nav.log" 2>&1 &
for i in $(seq 1 20); do
  if ros2 action list 2>/dev/null | grep -q navigate_to_pose; then
    echo "  [OK] Nav2 就绪"; break
  fi
  sleep 3
done

echo "== 3. template 识别节点（mode:=template 直启） =="
nohup ros2 launch ican_recognize recognize.launch.py mode:=template \
  > "$LOG/t_recog.log" 2>&1 &
for i in $(seq 1 15); do
  if grep -q "识别节点就绪" "$LOG/t_recog.log" 2>/dev/null; then
    echo "  [OK] 识别节点就绪"; break
  fi
  sleep 2
done

echo "== 4. 顺序任务（不开实弹射击） =="
nohup ros2 launch ican_nav_tasks nav_tasks.launch.py enable_shooting:=false \
  > "$LOG/t_mission.log" 2>&1 &
sleep 8
grep -q "顺序导航节点就绪" "$LOG/t_mission.log" && echo "  [OK] 任务节点就绪"

echo "== 测试环境已起。跟踪: tail -f $LOG/t_recog.log $LOG/t_mission.log =="
echo "== 结束后清理: bash $WS/stop_all.sh =="
