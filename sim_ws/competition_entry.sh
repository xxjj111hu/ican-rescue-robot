#!/usr/bin/env bash
# =============================================================================
# competition_entry.sh —— 评测平台入口脚本
#
# 硬性约束（由两次实测得出）：
#   - 平台 15 分钟超时，超时即 0 分；
#   - 终端只显示平台托管进程的输出，本脚本 stdout 平台不展示（但行为影响耗时）；
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
# 2. 部署识别模型与模板（不因缺模型而中止全流程）
#    [20261022] 新增 templates/：贴图源文件，template 模式（纯 OpenCV）
#    兜底识别用——torch/ultralytics 装不上时识别链路也能活。
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
TPL_SRC="$WS/src/ican_recognize/templates"
TPL_DST="$MODEL_DST_DIR/templates"
if [ -d "$TPL_SRC" ]; then
  mkdir -p "$TPL_DST"
  cp -f "$TPL_SRC"/* "$TPL_DST"/ 2>/dev/null
  say "templates deployed ($(ls "$TPL_DST" | wc -l) files)"
else
  say "warn: templates missing"
fi

# ---------------------------------------------------------------------------
# 3. 识别依赖：缺啥装啥，但 pip 总预算必须 ≤ 360s
#    [20261022] 第三轮实测 TIMEOUT 教训：多镜像 420s×2+120s×2 最多 1080s，
#    超过平台 901s 强杀线 → 任务节点根本没起来 → 0 分。
#    平台网络装不动完整 ultralytics（上轮 240s 失败），重试无意义：
#    只试阿里云单镜像，先 --no-deps（小包快速活）再完整一次，到 360s 止。
# ---------------------------------------------------------------------------
DEPS_OK=0
PIP_MIRROR="https://mirrors.aliyun.com/pypi/simple"

HAVE_TORCH=0; HAVE_ULTRA=0
python3 -c "import torch" 2>/dev/null && HAVE_TORCH=1
python3 -c "import ultralytics" 2>/dev/null && HAVE_ULTRA=1
if [ "$HAVE_ULTRA" -eq 1 ]; then
  DEPS_OK=1
  say "recognition deps preinstalled (torch=$HAVE_TORCH)"
else
  say "ultralytics missing (torch=$HAVE_TORCH), bounded pip (total<=360s)..."
  # 尝试 1：--no-deps 只装 ultralytics 本体（几 MB，120s）——torch 已在时即可用
  if timeout 120 pip3 install --no-deps --index-url "$PIP_MIRROR" \
       ultralytics > "$LOG/pip1.log" 2>&1; then
    python3 -c "import ultralytics" 2>/dev/null && DEPS_OK=1
  fi
  # 尝试 2：完整安装（240s，单镜像）——torch 不在或尝试1后仍缺依赖时
  if [ "$DEPS_OK" -eq 0 ]; then
    say "pip full attempt (240s)..."
    if timeout 240 pip3 install --index-url "$PIP_MIRROR" \
         "numpy<2" ultralytics > "$LOG/pip2.log" 2>&1; then
      python3 -c "import ultralytics" 2>/dev/null && DEPS_OK=1
    fi
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
# 6. 识别节点（依赖就绪用 yolo；装不上时用 template 模式——纯 OpenCV 兜底）
#    [20261022] 不再"跳过识别"：识别活着 = 3 枪 30 分，识别死了 = 0 分。
# ---------------------------------------------------------------------------
if [ "$DEPS_OK" -eq 1 ]; then
  ros2 launch ican_recognize recognize.launch.py > "$LOG/recog.log" 2>&1 &
  RECO_PID=$!
  say "recognize node starting (yolo)"
  sleep 5
else
  ros2 launch ican_recognize recognize.launch.py mode:=template > "$LOG/recog.log" 2>&1 &
  RECO_PID=$!
  say "recognize node starting (template fallback, no torch)"
  sleep 5
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
# 9. 挂住直到任务进程结束——但全局死线 860s（平台 901s 强杀，留余量）
# ---------------------------------------------------------------------------
say "elapsed so far: $(( $(date +%s) - T0 ))s"
deadline=$(( T0 + 860 ))
while kill -0 "$MISSION_PID" 2>/dev/null && [ "$(date +%s)" -lt "$deadline" ]; do
  sleep 10
done
say "mission ended or watch deadline reached, waiting children..."
wait "$MISSION_PID" 2>/dev/null
[ -n "$RECO_PID" ]   && wait "$RECO_PID"   2>/dev/null
wait "$NAV_PID"      2>/dev/null
wait "$SHOOT_PID"    2>/dev/null
say "script done."
