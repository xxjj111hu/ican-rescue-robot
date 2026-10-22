#!/usr/bin/env bash
# =============================================================================
# competition_entry.sh —— 评测平台入口脚本
#
# 职责：仅启动本队自有节点（定位导航 / 识别 / 击发 / 任务调度）。
# 不启动、不引用、不提及任何平台托管的进程与官方包；
# 环境话题（雷达 / 里程 / 相机）由平台就绪后直接等待即可。
# =============================================================================

# ---------------------------------------------------------------------------
# 0. 定位工作空间（平台目录结构未知，做防御式探测）
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[diag] pwd=$(pwd) script_dir=$SCRIPT_DIR"

find_ws() {
  # 优先脚本目录；再找其上级；再扫常见工作空间位置
  local cand
  for cand in "$SCRIPT_DIR" "$SCRIPT_DIR/.." "$SCRIPT_DIR/../.." \
              /workspace /ros2_ws /colcon_ws /catkin_ws "$HOME" /root; do
    cand="$(cd "$cand" 2>/dev/null && pwd)" || continue
    if [ -f "$cand/install/setup.bash" ]; then echo "$cand"; return 0; fi
  done
  # 全局兜底：限定常见目录深搜 install/setup.bash（避免扫全盘太久）
  for base in /root /home /workspace /opt; do
    [ -d "$base" ] || continue
    local hit
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

# 有 src 但没有 install → 现场编译
if [ -z "$WS" ]; then
  # 找 src 目录（含我们自己的包）
  SRC_DIR=""
  for cand in "$SCRIPT_DIR/src" "$SCRIPT_DIR/../src" /workspace/src /root/src "$HOME/src"; do
    [ -d "$cand/ican_example_navigation" ] && SRC_DIR="$(cd "$cand/.." && pwd)" && break
  done
  if [ -n "$SRC_DIR" ]; then
    WS="$SRC_DIR"
    echo "[diag] src found at $SRC_DIR but no install, building..."
    source /opt/ros/humble/setup.bash
    (cd "$WS" && colcon build --symlink-install) || true
  fi
fi

if [ -z "$WS" ] || [ ! -f "$WS/install/setup.bash" ]; then
  echo "[diag] ls script dir:"; ls -la "$SCRIPT_DIR" 2>&1 | head -20
  echo "[FATAL] workspace install not found anywhere" >&2
  exit 1
fi
echo "[diag] workspace = $WS"
LOG="$WS/logs"
mkdir -p "$LOG"

# ---------------------------------------------------------------------------
# 1. 基础环境
# ---------------------------------------------------------------------------
export LC_ALL=C.UTF-8
export LANG=C.UTF-8
# 关闭共享内存传输，避免位姿/TF 偶发丢包
export RMW_FASTRTPS_DISABLE_SHM=1

source /opt/ros/humble/setup.bash 2>/dev/null || echo "[warn] /opt/ros/humble not found"
source "$WS/install/setup.bash"
echo "[$(date)] env ready"

# ---------------------------------------------------------------------------
# 2. 部署识别模型（节点默认读 ~/.ros/ican_recognize/best.pt）
# ---------------------------------------------------------------------------
MODEL_SRC="$WS/src/ican_recognize/models/best.pt"
MODEL_DST_DIR="$HOME/.ros/ican_recognize"
MODEL_DST="$MODEL_DST_DIR/best.pt"
if [ -f "$MODEL_SRC" ]; then
  mkdir -p "$MODEL_DST_DIR"
  cp -f "$MODEL_SRC" "$MODEL_DST"
  export ICAN_YOLO_MODEL="$MODEL_DST"
  export ICAN_CLS_MODEL="$MODEL_DST"
  echo "[$(date)] model deployed: $MODEL_DST"
else
  echo "[warn] model not found at $MODEL_SRC (recognition will fail)" >&2
  find "$WS" -name "best.pt" 2>/dev/null | head -3
fi

# ---------------------------------------------------------------------------
# 3. 识别依赖检查（缺失则安装）
# ---------------------------------------------------------------------------
if ! python3 -c "import ultralytics" 2>/dev/null; then
  echo "[$(date)] installing recognition deps (cpu-only torch)..."
  PIP_INDEX="https://pypi.tuna.tsinghua.edu.cn/simple"
  TORCH_INDEX="https://download.pytorch.org/whl/cpu"
  installed=0
  for attempt in 1 2 3 4 5; do
    echo "  attempt $attempt ..."
    # 先装 cpu-only torch（体积小、下载快），再装 numpy<2 与 ultralytics
    if timeout 600 pip3 install --index-url "$PIP_INDEX" \
         "numpy<2" torch torchvision --index-url "$TORCH_INDEX" 2>&1 | tail -25; then
      if timeout 600 pip3 install --index-url "$PIP_INDEX" ultralytics 2>&1 | tail -15; then
        installed=1; break
      fi
    fi
    sleep 5
  done
  if [ "$installed" -ne 1 ] || ! python3 -c "import ultralytics" 2>/dev/null; then
    echo "[FATAL] recognition deps install failed" >&2
    exit 1
  fi
  # ultralytics 会把 setuptools 连带升级，需退回系统版本，否则影响后续运行
  pip3 uninstall -y setuptools 2>/dev/null || true
  echo "[$(date)] recognition deps ready"
else
  echo "[$(date)] recognition deps ready"
fi

# ---------------------------------------------------------------------------
# 4. 等待平台就绪的核心话题
# ---------------------------------------------------------------------------
wait_topic() {
  local topic=$1 limit=$2 desc=$3 waited=0
  while [ "$waited" -lt "$limit" ]; do
    if timeout 4 ros2 topic echo "$topic" --once > /dev/null 2>&1; then
      echo "  [OK] $desc ($topic)"; return 0
    fi
    sleep 2; waited=$((waited + 2))
  done
  echo "  [FAIL] $desc ($topic) timeout ${limit}s"; return 1
}

echo "[$(date)] waiting for platform topics..."
wait_topic /scan 40 "laser"        || exit 1
wait_topic /odom 20 "odometry"     || exit 1
wait_topic /camera/image 30 "camera" || exit 1

# ---------------------------------------------------------------------------
# 5. 定位导航（Nav2 + AMCL，无可视化）
# ---------------------------------------------------------------------------
ros2 launch ican_example_navigation navigation.launch.py rviz:=false \
  > "$LOG/nav.log" 2>&1 &
NAV_PID=$!
echo "waiting Nav2 active..."
sleep 20
nav_ok=0; waited=0
while [ "$waited" -lt 60 ]; do
  if ros2 action list 2>/dev/null | grep -q "navigate_to_pose"; then nav_ok=1; break; fi
  sleep 3; waited=$((waited + 3))
done
if [ "$nav_ok" -eq 1 ]; then echo "  [OK] navigate_to_pose"; else
  echo "  [FAIL] Nav2 not ready, dump nav.log:"; tail -30 "$LOG/nav.log"; exit 1
fi

# ---------------------------------------------------------------------------
# 6. 识别节点
# ---------------------------------------------------------------------------
ros2 launch ican_recognize recognize.launch.py > "$LOG/recog.log" 2>&1 &
RECO_PID=$!
recog_ok=0
for _ in $(seq 1 25); do
  if grep -q "识别节点就绪" "$LOG/recog.log" 2>/dev/null; then recog_ok=1; break; fi
  sleep 2
done
if [ "$recog_ok" -eq 1 ]; then echo "  [OK] recognize node"; else
  echo "  [WARN] recognize node not confirmed, dump recog.log:"; tail -30 "$LOG/recog.log"
fi

# ---------------------------------------------------------------------------
# 7. 击发节点（实弹 dry_run:=false）
# ---------------------------------------------------------------------------
ros2 launch ican_shoot shoot_all.launch.py dry_run:=false \
  > "$LOG/shoot.log" 2>&1 &
SHOOT_PID=$!
echo "  [INFO] shoot node started (live)"
sleep 3

# ---------------------------------------------------------------------------
# 8. 顺序任务（enable_shooting:=true）
# ---------------------------------------------------------------------------
ros2 launch ican_nav_tasks nav_tasks.launch.py enable_shooting:=true \
  > "$LOG/mission.log" 2>&1 &
MISSION_PID=$!
sleep 5
if grep -q "顺序导航节点就绪" "$LOG/mission.log" 2>/dev/null; then
  echo "  [OK] mission node"
else
  echo "  [FAIL] mission node not ready, dump mission.log:"; tail -30 "$LOG/mission.log"; exit 1
fi

echo
echo "==================================================="
echo "all nodes up: nav -> recognize -> shoot -> mission"
echo "logs: $LOG/{nav,recog,shoot,mission}.log"
echo "entering watch loop (script stays alive with nodes)"
echo "==================================================="

# ---------------------------------------------------------------------------
# 9. 挂住等待任务完成，并把任务日志持续吐到 stdout（供平台终端采集）
# ---------------------------------------------------------------------------
while kill -0 "$MISSION_PID" 2>/dev/null; do
  tail -n 5 "$LOG/mission.log" 2>/dev/null
  sleep 10
done
echo "[$(date)] mission process ended, waiting others..."
wait "$MISSION_PID" "$NAV_PID" "$RECO_PID" "$SHOOT_PID" 2>/dev/null
echo "[$(date)] all nodes exited, script done."
