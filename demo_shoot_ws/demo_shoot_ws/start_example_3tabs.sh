#!/usr/bin/env bash
# ===============================================================
# 分步启动示例任务（WSL / 无 gnome-terminal 环境替代方案）
#
# 用法（必须在 Ubuntu 终端里运行，不要在 Windows 里运行）：
#   bash start_example_3tabs.sh
#
# 效果：自动在你当前的 Windows Terminal 窗口里开 3 个新标签页：
#   标签1  sim    -> Gazebo 仿真
#   标签2  nav2   -> Nav2 导航
#   标签3  mission-> 示例任务程序
#
# 为什么需要这个脚本：
#   原始的 scripts/run_example_mission.sh 依赖 gnome-terminal，
#   而 gnome-terminal 需要 D-Bus 会话总线，WSL 里默认没有，
#   会报 "Could not connect: No such file or directory"。
#
# 依赖：需要在 Windows 端安装 wt.exe（Windows Terminal）。
#       如果没装，请改用「手动三终端」方式，见文件末尾提示。
# ===============================================================

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  :
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}"
SETUP_ENV="${WS_DIR}/scripts/setup_env.sh"

# ------------------------------------------------------------------
# 前置检查
# ------------------------------------------------------------------
if [[ ! -f "${SETUP_ENV}" ]]; then
  echo "[错误] 找不到 ${SETUP_ENV}" >&2
  echo "       请在仿真工作空间根目录下运行本脚本。" >&2
  exit 1
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[错误] 未找到 ROS 2 Humble，请先运行 scripts/00_install_ros2_gazebo.sh" >&2
  exit 1
fi

if [[ ! -f /opt/ican_shoot_sim/setup.bash ]]; then
  echo "[错误] 未找到 iCAN 仿真环境，请先运行 scripts/01_install_sim_deb.sh" >&2
  exit 1
fi

# ------------------------------------------------------------------
# 检查 wt.exe（Windows Terminal）是否可用
# ------------------------------------------------------------------
WT_EXE="/mnt/c/Users/${USER}/AppData/Local/Microsoft/WindowsApps/wt.exe"
if [[ ! -x "${WT_EXE}" ]]; then
  # 退而求其次：在 PATH 里找
  WT_IN_PATH="$(command -v wt.exe 2>/dev/null || true)"
  if [[ -n "${WT_IN_PATH}" ]]; then
    WT_EXE="${WT_IN_PATH}"
  fi
fi

if [[ ! -x "${WT_EXE}" ]]; then
  cat >&2 <<'EOF'
[错误] 找不到 wt.exe（Windows Terminal）。

请改用「手动三终端」方式，分别在新标签页里执行：

  标签1（仿真）：
    cd ~/ican/demo_shoot_ws/demo_shoot_ws
    source scripts/setup_env.sh
    ros2 launch ican_shoot_sim sim.launch.py

  标签2（导航，等标签1出现场地后再执行）：
    cd ~/ican/demo_shoot_ws/demo_shoot_ws
    source scripts/setup_env.sh
    ros2 launch ican_example_navigation navigation.launch.py

  标签3（示例程序，等标签2打印出 Nav2 激活后再执行）：
    cd ~/ican/demo_shoot_ws/demo_shoot_ws
    source scripts/setup_env.sh
    source install/setup.bash
    ros2 launch ican_example_mission one_target_demo.launch.py
EOF
  exit 1
fi

# ------------------------------------------------------------------
# 把工作空间路径转成 WSL 内部可见的形式
# WSL 的 home 在 Windows 侧通过 \\wsl.localhost\<distro>\... 访问，
# 但更简单的做法是让每个标签页自己 cd 到 WSL 路径。
# ------------------------------------------------------------------
DISTRO="${WSL_DISTRO_NAME:-Ubuntu-22.04}"

echo "[启动] 正在通过 Windows Terminal 打开 3 个标签页 ..."
echo "       标签1 sim     : Gazebo 仿真"
echo "       标签2 nav2    : Nav2 导航（延迟 5 秒）"
echo "       标签3 mission : 示例任务（延迟 9 秒）"

# 每个标签页都让 Ubuntu 自己完成 cd + source，避免路径转换问题
CMD_SIM="cd ${WS_DIR} && source scripts/setup_env.sh && ros2 launch ican_shoot_sim sim.launch.py"
CMD_NAV="sleep 5; cd ${WS_DIR} && source scripts/setup_env.sh && ros2 launch ican_example_navigation navigation.launch.py"
CMD_MIS="sleep 9; cd ${WS_DIR} && source scripts/setup_env.sh && source install/setup.bash && ros2 launch ican_example_mission one_target_demo.launch.py"

"${WT_EXE}" -w 0 new-tab --title sim     wsl.exe -d "${DISTRO}" -- bash -lc "${CMD_SIM}" \; \
              new-tab --title nav2    wsl.exe -d "${DISTRO}" -- bash -lc "${CMD_NAV}" \; \
              new-tab --title mission wsl.exe -d "${DISTRO}" -- bash -lc "${CMD_MIS}"

echo "[启动] 已发送。请在 Windows Terminal 里查看新开的 3 个标签页。"
echo "       如果没弹出来，请用手动三终端方式（见 --help）。"
