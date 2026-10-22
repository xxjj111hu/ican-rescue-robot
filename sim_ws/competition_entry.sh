#!/usr/bin/env bash
# =============================================================================
# competition_entry.sh —— 评测平台入口脚本
#
# 硬性约束（由两次实测得出）：
#   - 平台 15 分钟超时，超时即 0 分；
#   - 终端只显示平台裁判进程，本脚本 stdout 平台不展示（但行为影响耗时）；
#   - 因此：所有耗时步骤（编译/装依赖）必须设上限；
#     依赖装不上就降级运行——保证任务流程能在时限内完整结束。
# =============================================================================

T0=$(date +%s)
say() { echo "[$(( $(date +%s) - T0 ))s] $*"; }

# ---------------------------------------------------------------------------
# 0. 定位工作空间（平台目录结构未知，防御式探测）
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

find_ws() {
  local cand
  for cand in "$SCRIPT_DIR" "$SCRIPT_DIR/.." "$SCRIPT_DIR/../.." \
              /workspace /ros2_ws /colcon_ws /catkin_ws "$HOME" /root; do
    cand="$(cd "$cand" 2>/dev/null && pwd)" || continue
    if [ -f "$cand/install/setup.bash" ]; then echo "$cand"; return 0; fi
  done
  local hit
  for base in /root /home /workspace /opt; do
    [ -d "$base" ] || continue
    hit="$(find "$base" -maxdepth 4 -name setup.bash -path '*/install/*' 2>/dev/null | head -1)"
    if [ -n "$hit" ]; then dirname "$(dirname "$hit")"; return 0; fi
  done
  return 1
}

WS=""
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
  WS="$SCRIPT_DIR"
else
  WS="$(find_ws)" || WS=""
fi

# 有 src 没 install → 限时现场编译（最多 300s，编不完就用部分产物）
if [ -z "$WS" ]; then
  SRC_DIR=""
  for cand in "$SCRIPT_DIR/src" "$SCRIPT_DIR/../src" /workspace/src /root/src "$HOME/src"; do
    [ -d "$cand/ican_example_navigation" ] && SRC_DIR="$(cd "$cand/.." && pwd)" && break
  done
  if [ -n "$SRC_DIR" ]; then
    WS="$SRC_DIR"
    say "src found but no install, building (max 300s)..."
    source /opt/ros/humble/setup.bash 2>/dev/null
    (cd "$WS" && timeout 300 colcon build --symlink-install) || say "build ended (partial ok)"
  fi
fi

if [ -z "$WS" ] || [ ! -f "$WS/install/setup.bash" ]; then
  say "FATAL: workspace install not found"; exit 1
fi
say "workspace = $WS"
LOG="$WS/logs"
mkdir -p "$LOG"

# ---------------------------------------------------------------------------
# 1. 基础环境
# ---------------------------------------------------------------------------
export LC_ALL=C.UTF-8
export LANG=C.UTF-8
export RMW_FASTRTPS_DISABLE_SHM=1

source /opt/ros/humble/setup.bash 2>/dev/null || say "warn: ros env missing"
source "$WS/install/setup.bash"
say "env ready"

# ---------------------------------------------------------------------------
# 2. 部署识别模型（不因缺模型而中止全流程）
# ---------------------------------------------------------------------------
MODEL_SRC="$WS/src/ican_recognize/models/best.pt"
MODEL_DST_DIR="$HOME/.ros/ican_recognize"
MODEL_DST="$MODEL_DST_DIR/best.pt"
if [ -f "$MODEL_SRC" ]; then
  mkdir -p "$MODEL_DST_DIR"
  cp -f "$MODEL_SRC" "$MODEL_DST"
  export ICAN_YOLO_MODEL="$MODEL_DST"
  export ICAN_CLS_MODEL="$MODEL_DST"
  say "model deployed"
else
  say "warn: model missing, recognition will degrade"
fi

# ---------------------------------------------------------------------------
# 3. 识别依赖：限时单次尝试（240s），装不上就降级（不装识别只跑导航+任务）
# ---------------------------------------------------------------------------
DEPS_OK=0
if python3 -c "import ultralytics" 2>/dev/null; then
  DEPS_OK=1
  say "recognition deps preinstalled"
else
  say "deps missing, bounded pip attempt (240s)..."
  if timeout 240 pip3 install --index-url https://pypi.tuna.tsinghua.edu.cn/simple \
       "numpy<2" ultralytics > "$LOG/pip.log" 2>&1; then
    python3 -c "import ultralytics" 2>/dev/null && DEPS_OK=1
  fi
  pip3 uninstall -y setuptools >/dev/null 2>&1 || true
fi
if [ "$DEPS_OK" -eq 1 ]; then say "recognition deps ready"; else
  say "DEGRADED: no recognition deps, running nav+mission only"
fi

# ---------------------------------------------------------------------------
# 4. 等待平台核心话题
# ---------------------------------------------------------------------------
wait_topic() {
  local topic=$1 limit=$2 desc=$3 waited=0
  while [ "$waited" -lt "$limit" ]; do
    if timeout 4 ros2 topic echo "$topic" --once > /dev/null 2>&1; then
      say "[OK] $desc ($topic)"; return 0
    fi
    sleep 2; waited=$((waited + 2))
  done
  say "[FAIL] $desc ($topic) timeout"; return 1
}

say "waiting for platform topics..."
wait_topic /scan 60 "laser"    || exit 1
wait_topic /odom 20 "odometry" || exit 1
if [ "$DEPS_OK" -eq 1 ]; then
  wait_topic /camera/image 30 "camera" || say "warn: no camera, recognition degraded"
fi

# ---------------------------------------------------------------------------
# 5. 击发节点（实弹，先起——任务调用它时服务必须已在）
# ---------------------------------------------------------------------------
ros2 launch ican_shoot shoot_all.launch.py dry_run:=false \
  > "$LOG/shoot.log" 2>&1 &
SHOOT_PID=$!
say "shoot node starting (live)"
sleep 3

# ---------------------------------------------------------------------------
# 6. 识别节点（仅依赖就绪时启动）
# ---------------------------------------------------------------------------
if [ "$DEPS_OK" -eq 1 ]; then
  ros2 launch ican_recognize recognize.launch.py > "$LOG/recog.log" 2>&1 &
  RECO_PID=$!
  say "recognize node starting"
  sleep 5
else
  RECO_PID=""
  say "skip recognize node (degraded)"
fi

# ---------------------------------------------------------------------------
# 7. 定位导航（Nav2 + AMCL）
# ---------------------------------------------------------------------------
ros2 launch ican_example_navigation navigation.launch.py rviz:=false \
  > "$LOG/nav.log" 2>&1 &
NAV_PID=$!
say "nav2 starting..."
sleep 20
nav_ok=0; waited=0
while [ "$waited" -lt 60 ]; do
  if ros2 action list 2>/dev/null | grep -q "navigate_to_pose"; then nav_ok=1; break; fi
  sleep 3; waited=$((waited + 3))
done
if [ "$nav_ok" -eq 1 ]; then say "[OK] navigate_to_pose"; else
  say "FAIL Nav2, dump:"; tail -20 "$LOG/nav.log"; exit 1
fi

# ---------------------------------------------------------------------------
# 8. 顺序任务（识别缺失时任务节点会按超时跳过识别继续走点）
# ---------------------------------------------------------------------------
ros2 launch ican_nav_tasks nav_tasks.launch.py enable_shooting:=true \
  > "$LOG/mission.log" 2>&1 &
MISSION_PID=$!
sleep 5
if grep -q "顺序导航节点就绪" "$LOG/mission.log" 2>/dev/null; then
  say "[OK] mission node running"
else
  say "FAIL mission, dump:"; tail -20 "$LOG/mission.log"; exit 1
fi

say "all nodes up, watching until mission ends..."

# ---------------------------------------------------------------------------
# 9. 挂住直到任务进程结束（最长再守 8 分钟，防无限占用）
# ---------------------------------------------------------------------------
deadline=$(( $(date +%s) + 480 ))
while kill -0 "$MISSION_PID" 2>/dev/null && [ "$(date +%s)" -lt "$deadline" ]; do
  sleep 10
done
say "mission ended or watch deadline reached, waiting children..."
wait "$MISSION_PID" 2>/dev/null
[ -n "$RECO_PID" ]   && wait "$RECO_PID"   2>/dev/null
wait "$NAV_PID"      2>/dev/null
wait "$SHOOT_PID"    2>/dev/null
say "script done."
