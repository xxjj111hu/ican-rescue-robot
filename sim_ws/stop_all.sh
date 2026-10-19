#!/usr/bin/env bash
# 彻底清理所有 ROS 2 / Gazebo 进程与共享内存残留
#
# 为什么需要它：`pkill -f "ros2 launch"` 不会杀掉 launch 拉起的子节点，
# 反复测试会积累多份 gzserver / robot_state_publisher / 识别节点，
# 表现为：/scan 无数据、/amcl_pose 为空、TF 外推错误、话术重复发送。
#
# 用法：bash stop_all.sh
set -u

echo "[1/3] 发送终止信号 ..."
for pat in \
  "opt/ros/humble/lib" \
  "ican_land" "ican_example" "ican_recognize" "ican_nav_tasks" \
  "gazebo" "gzserver" "gzclient" \
  "ros2 launch" "ros2 run" "ros2-daemon"; do
  pkill -9 -f "$pat" 2>/dev/null
done

sleep 3

echo "[2/3] 清理 FastRTPS 共享内存 ..."
find /dev/shm -name 'fastrtps*' -delete 2>/dev/null

echo "[3/3] 检查残留 ..."
left=$(ps aux | grep -cE "[g]zserver|[g]zclient|[r]os2 launch|[r]ecognize_node|[s]equential_mission")
echo "残留进程数: $left"
if [ "$left" -ne 0 ]; then
  echo "⚠️ 仍有残留，请再执行一次："
  ps aux | grep -E "[g]zserver|[g]zclient|[r]os2 launch|[r]ecognize_node|[s]equential_mission"
  exit 1
fi
echo "✅ 已清空"
