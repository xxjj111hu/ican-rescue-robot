#!/usr/bin/env bash
# =============================================================================
# competition_entry.sh —— 比赛测评平台入口脚本（唯一提交给平台的 .sh）
#
# 平台规则（由本次提交 HTTP 400 错误明确）：
#   - 平台自己会启动 Gazebo 官方仿真环境 + 裁判系统；
#   - 用户脚本禁止包含任何启动/操作 Gazebo 或官方仿真/裁判的命令；
#   - 用户脚本只能启动自己的节点（导航、识别、射击、任务）。
#
# 因此本脚本流程：
#   1. 加载环境；
#   2. 部署识别模型；
#   3. 检查识别依赖；
#   4. 等待平台已发布的话题（/scan /odom /camera/image）；
#   5. 依次启动 Nav2 → 识别 → 射击（实弹） → 顺序任务。
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$SCRIPT_DIR"
LOG="$WS/logs"
mkdir -p "$LOG"

# ---------------------------------------------------------------------------
# 1. 加载环境（关键环境变量，缺一不可）
# ---------------------------------------------------------------------------
export LC_ALL=C.UTF-8
export LANG=C.UTF-8
# 禁用 FastRTPS 共享内存：否则 /initialpose、TF、AMCL 位姿偶发丢包。
export RMW_FASTRTPS_DISABLE_SHM=1

source /opt/ros/humble/setup.bash
[ -f /usr/share/gazebo/setup.bash ] && source /usr/share/gazebo/setup.bash
# 官方仿真包平台已预装，只 source 环境，严禁启动其中任何 launch。
[ -f /opt/ican_shoot_sim/setup.bash ] && source /opt/ican_shoot_sim/setup.bash

if [ ! -f "$WS/install/setup.bash" ]; then
  echo "❌ 找不到 $WS/install/setup.bash —— 服务器应先编译 src 生成 install/。" >&2
  exit 1
fi
source "$WS/install/setup.bash"
echo "[$(date)] 环境已加载"

# ---------------------------------------------------------------------------
# 2. 部署识别模型（平台只收 src，节点默认读 ~/.ros/ican_recognize/best.pt）
# ---------------------------------------------------------------------------
MODEL_SRC="$WS/src/ican_recognize/models/best.pt"
MODEL_DST_DIR="$HOME/.ros/ican_recognize"
MODEL_DST="$MODEL_DST_DIR/best.pt"
if [ -f "$MODEL_SRC" ]; then
  mkdir -p "$MODEL_DST_DIR"
  cp -f "$MODEL_SRC" "$MODEL_DST"
  export ICAN_YOLO_MODEL="$MODEL_DST"
  export ICAN_CLS_MODEL="$MODEL_DST"
  echo "[$(date)] 模型已部署到 $MODEL_DST"
else
  echo "❌ 未找到 src 内模型 $MODEL_SRC，识别节点无法加载权重。" >&2
  echo "   请把训练好的 best.pt 放进 src/ican_recognize/models/ 后再打包上传。" >&2
  exit 1
fi

# ---------------------------------------------------------------------------
# 3. 检查并安装识别依赖（ultralytics + torch，平台可能未预装）
# ---------------------------------------------------------------------------
if ! python3 -c "import ultralytics" 2>/dev/null; then
  echo "[$(date)] 未检测到 ultralytics，开始安装识别依赖..."
  REQ="$WS/src/ican_recognize/requirements.txt"
  if [ -f "$REQ" ]; then
    installed=0
    for attempt in 1 2 3; do
      echo "  第 $attempt 次尝试 pip install -r $REQ ..."
      if timeout 300 pip3 install -r "$REQ" 2>&1 | tail -20; then
        installed=1; break
      fi
      sleep 5
    done
    if [ "$installed" -ne 1 ] || ! python3 -c "import ultralytics" 2>/dev/null; then
      echo "❌ 识别依赖安装失败。平台未预装 ultralytics/torch 且网络受限。" >&2
      exit 1
    fi
    echo "[$(date)] 识别依赖安装完成"
  else
    echo "❌ 找不到 $REQ" >&2; exit 1
  fi
else
  echo "[$(date)] 识别依赖已就绪"
fi

# ---------------------------------------------------------------------------
# 4. 等待平台仿真就绪（平台已启动 Gazebo，我们只等它的话题）
# ---------------------------------------------------------------------------
wait_topic() {
  local topic=$1 limit=$2 desc=$3 waited=0
  while [ "$waited" -lt "$limit" ]; do
    if timeout 4 ros2 topic echo "$topic" --once > /dev/null 2>&1; then
      echo "  ✅ $desc ($topic) 有数据"; return 0
    fi
    sleep 2; waited=$((waited + 2))
  done
  echo "  ❌ $desc ($topic) 等了 ${limit}s 仍无数据"; return 1
}

echo "[$(date)] 等待平台仿真环境发布话题..."
wait_topic /scan 40 "雷达"    || { echo "平台仿真未就绪，/scan 无数据"; exit 1; }
wait_topic /odom 20 "里程计"  || { echo "平台仿真未就绪，/odom 无数据"; exit 1; }
wait_topic /camera/image 30 "相机图像" || { echo "平台仿真未就绪，/camera/image 无数据"; exit 1; }

# ---------------------------------------------------------------------------
# 5. 启动 Nav2 + AMCL（无 RViz，平台无显示器）
# ---------------------------------------------------------------------------
ros2 launch ican_example_navigation navigation.launch.py rviz:=false \
  > "$LOG/nav.log" 2>&1 &
echo "等待 Nav2 激活（约 20s）..."
sleep 20
nav_ok=0; waited=0
while [ "$waited" -lt 60 ]; do
  if ros2 action list 2>/dev/null | grep -q "navigate_to_pose"; then nav_ok=1; break; fi
  sleep 3; waited=$((waited + 3))
done
if [ "$nav_ok" -eq 1 ]; then echo "  ✅ /navigate_to_pose 可用"; else
  echo "  ❌ Nav2 未就绪，查看 $LOG/nav.log"; exit 1
fi

# ---------------------------------------------------------------------------
# 6. 启动识别节点（订阅平台已发布的 /camera/image）
# ---------------------------------------------------------------------------
ros2 launch ican_recognize recognize.launch.py > "$LOG/recog.log" 2>&1 &
recog_ok=0
for _ in $(seq 1 25); do
  if grep -q "识别节点就绪" "$LOG/recog.log" 2>/dev/null; then recog_ok=1; break; fi
  sleep 2
done
if [ "$recog_ok" -eq 1 ]; then echo "  ✅ 识别节点就绪"; else
  echo "  ❌ 识别节点未就绪，查看 $LOG/recog.log"; exit 1
fi

# ---------------------------------------------------------------------------
# 7. 启动射击节点（实弹：dry_run:=false）
# ---------------------------------------------------------------------------
ros2 launch ican_shoot shoot_all.launch.py dry_run:=false \
  > "$LOG/shoot.log" 2>&1 &
echo "  ℹ️ 射击节点已启动（实弹 dry_run:=false）"
sleep 3

# ---------------------------------------------------------------------------
# 8. 启动顺序任务（enable_shooting:=true → 调用开火服务 + 识别/返航闭环）
# ---------------------------------------------------------------------------
ros2 launch ican_nav_tasks nav_tasks.launch.py enable_shooting:=true \
  > "$LOG/mission.log" 2>&1 &
sleep 5
if grep -q "顺序导航节点就绪" "$LOG/mission.log" 2>/dev/null; then
  echo "  ✅ 任务节点就绪（机器人开始自主跑分）"
else
  echo "  ❌ 任务节点未就绪，查看 $LOG/mission.log"; exit 1
fi

echo
echo "==================================================="
echo "全部启动完成，机器人自主执行：识别 → 进点 → 射击声明 → 返航"
echo "日志：$LOG/{nav,recog,shoot,mission}.log"
echo "关键行过滤："
echo "  grep -E '识别\\]|裁判\\]|发送导航目标|shoot' $LOG/mission.log"
echo "==================================================="
