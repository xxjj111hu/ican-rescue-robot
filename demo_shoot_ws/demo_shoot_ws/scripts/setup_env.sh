#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "请使用 source 加载环境："
  echo "  source scripts/setup_env.sh"
  exit 1
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "未找到 /opt/ros/humble/setup.bash，请先运行 bash scripts/00_install_ros2_gazebo.sh" >&2
  return 1
fi

if [[ ! -f /opt/ican_shoot_sim/setup.bash ]]; then
  echo "未找到 /opt/ican_shoot_sim/setup.bash，请先运行 bash scripts/01_install_sim_deb.sh" >&2
  return 1
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# Gazebo's shader and media resources are not loaded by the ROS setup script.
if [[ -f /usr/share/gazebo/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /usr/share/gazebo/setup.bash
fi
# shellcheck disable=SC1091
source /opt/ican_shoot_sim/setup.bash

if ! python3 -c 'import importlib.metadata as metadata; metadata.distribution("ican-land-control")' >/dev/null 2>&1; then
  echo "仿真 deb 缺少 ican_land_control Python 包或元数据，请重新安装修复版 deb。" >&2
  return 1
fi

echo "已加载 ROS 2 Humble 和 iCAN 练习仿真环境。"
