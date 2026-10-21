#!/usr/bin/env bash
# =============================================================================
# competition_entry.sh —— 评测平台入口脚本
#
# 职责：仅启动本队自有节点（定位导航 / 识别 / 击发 / 任务调度）。
# 不启动、不引用、不提及任何平台托管的进程与官方包；
# 环境话题（雷达 / 里程 / 相机）由平台就绪后直接等待即可。
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$SCRIPT_DIR"
LOG="$WS/logs"
mkdir -p "$LOG"

# ---------------------------------------------------------------------------
# 1. 基础环境
# ---------------------------------------------------------------------------
export LC_ALL=C.UTF-8
export LANG=C.UTF-8
# 关闭共享内存传输，避免位姿/TF 偶发丢包
export RMW_FASTRTPS_DISABLE_SHM=1

source /opt/ros/humble/setup.bash

if [ ! -f "$WS/install/setup.bash" ]; then
  echo "[FATAL] missing $WS/install/setup.bash" >&2
  exit 1
fi
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
  echo "[FATAL] model not found: $MODEL_SRC" >&2
  exit 1
fi

# ---------------------------------------------------------------------------
# 3. 识别依赖检查（缺失则安装）
# ---------------------------------------------------------------------------
if ! python3 -c "import ultralytics" 2>/dev/null; then
  echo "[$(date)] installing recognition deps..."
  REQ="$WS/src/ican_recognize/requirements.txt"
  if [ -f "$REQ" ]; then
    installed=0
    for attempt in 1 2 3; do
      echo "  attempt $attempt ..."
      if timeout 300 pip3 install -r "$REQ" 2>&1 | tail -20; then
        installed=1; break
      fi
      sleep 5
    done
    if [ "$installed" -ne 1 ] || ! python3 -c "import ultralytics" 2>/dev/null; then
      echo "[FATAL] recognition deps install failed" >&2
      exit 1
    fi
    echo "[$(date)] recognition deps ready"
  else
    echo "[FATAL] missing $REQ" >&2
    exit 1
  fi
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
echo "waiting Nav2 active..."
sleep 20
nav_ok=0; waited=0
while [ "$waited" -lt 60 ]; do
  if ros2 action list 2>/dev/null | grep -q "navigate_to_pose"; then nav_ok=1; break; fi
  sleep 3; waited=$((waited + 3))
done
if [ "$nav_ok" -eq 1 ]; then echo "  [OK] navigate_to_pose"; else
  echo "  [FAIL] Nav2 not ready, see $LOG/nav.log"; exit 1
fi

# ---------------------------------------------------------------------------
# 6. 识别节点
# ---------------------------------------------------------------------------
ros2 launch ican_recognize recognize.launch.py > "$LOG/recog.log" 2>&1 &
recog_ok=0
for _ in $(seq 1 25); do
  if grep -q "识别节点就绪" "$LOG/recog.log" 2>/dev/null; then recog_ok=1; break; fi
  sleep 2
done
if [ "$recog_ok" -eq 1 ]; then echo "  [OK] recognize node"; else
  echo "  [FAIL] recognize node not ready, see $LOG/recog.log"; exit 1
fi

# ---------------------------------------------------------------------------
# 7. 击发节点（实弹 dry_run:=false）
# ---------------------------------------------------------------------------
ros2 launch ican_shoot shoot_all.launch.py dry_run:=false \
  > "$LOG/shoot.log" 2>&1 &
echo "  [INFO] shoot node started (live)"
sleep 3

# ---------------------------------------------------------------------------
# 8. 顺序任务（enable_shooting:=true）
# ---------------------------------------------------------------------------
ros2 launch ican_nav_tasks nav_tasks.launch.py enable_shooting:=true \
  > "$LOG/mission.log" 2>&1 &
sleep 5
if grep -q "顺序导航节点就绪" "$LOG/mission.log" 2>/dev/null; then
  echo "  [OK] mission node"
else
  echo "  [FAIL] mission node not ready, see $LOG/mission.log"; exit 1
fi

echo
echo "==================================================="
echo "all nodes up: nav -> recognize -> shoot -> mission"
echo "logs: $LOG/{nav,recog,shoot,mission}.log"
echo "==================================================="
