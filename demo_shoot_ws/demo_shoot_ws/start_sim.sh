#!/usr/bin/env bash
# iCAN 仿真环境一键启动脚本（在 Ubuntu / WSL 内运行）
#
# 用法：
#   bash start_sim.sh              # 带 Gazebo 图形界面启动
#   bash start_sim.sh --headless   # 无界面启动（省内存，看不到场地）
#   bash start_sim.sh --help       # 显示帮助
#
# 放在工作空间根目录下，随便从哪个目录调用都可以。

set -Eeuo pipefail

# ---------------------------------------------------------------
# 定位工作空间根目录：本脚本所在目录
# ---------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---------------------------------------------------------------
# 解析参数
# ---------------------------------------------------------------
SIM_ARGS=()
for arg in "$@"; do
  case "$arg" in
    --headless|-H)
      SIM_ARGS+=("gui:=false")
      ;;
    -h|--help)
      cat <<'EOF'
iCAN 仿真环境启动脚本

用法：
  bash start_sim.sh              带 Gazebo 图形界面启动
  bash start_sim.sh --headless   无界面启动（gui:=false）
  bash start_sim.sh --help       显示本帮助

说明：
  脚本会自动加载 ROS 2 Humble、Gazebo Classic 和 iCAN 仿真环境，
  然后启动 ros2 launch ican_shoot_sim sim.launch.py。
  按 Ctrl+C 停止仿真。
EOF
      exit 0
      ;;
    *)
      # 其余参数原样透传给 sim.launch.py
      SIM_ARGS+=("$arg")
      ;;
  esac
done

# ---------------------------------------------------------------
# 检查安装是否完整
# ---------------------------------------------------------------
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[错误] 未找到 ROS 2 Humble：/opt/ros/humble/setup.bash" >&2
  echo "       请先执行：bash scripts/00_install_ros2_gazebo.sh" >&2
  exit 1
fi

if [[ ! -f /opt/ican_shoot_sim/setup.bash ]]; then
  echo "[错误] 未找到 iCAN 仿真环境：/opt/ican_shoot_sim/setup.bash" >&2
  echo "       请先执行：bash scripts/01_install_sim_deb.sh" >&2
  exit 1
fi

# ---------------------------------------------------------------
# 加载环境
# setup_env.sh 要求必须用 source 方式加载，这里已经满足
# ---------------------------------------------------------------
echo "[启动] 正在加载环境 ..."
set +u
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/scripts/setup_env.sh"
set -u

# ---------------------------------------------------------------
# 如果示例工作空间已经编译过，一并加载，方便同时使用示例节点
# ---------------------------------------------------------------
if [[ -f "${SCRIPT_DIR}/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/install/setup.bash"
  set -u
  echo "[启动] 已加载示例工作空间：${SCRIPT_DIR}/install/setup.bash"
fi

# ---------------------------------------------------------------
# 启动仿真
# ---------------------------------------------------------------
if [[ ${#SIM_ARGS[@]} -eq 0 ]]; then
  echo "[启动] 正在启动 Gazebo 仿真（Ctrl+C 停止）..."
else
  echo "[启动] 正在启动 Gazebo 仿真，参数：${SIM_ARGS[*]}（Ctrl+C 停止）..."
fi

exec ros2 launch ican_shoot_sim sim.launch.py "${SIM_ARGS[@]}"
