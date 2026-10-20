#!/usr/bin/env bash
cd /home/longying01/demo_shoot_ws || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash
source /opt/ican_shoot_sim/setup.bash

V10=runs/detect/runs/ican_v10/weights/best.pt
OUT=v10_accept.txt
: > "$OUT"

restart_sim () {
  for p in $(ps -eo pid,cmd | grep -E 'gzserver|gzclient|sim.launch' | grep -v grep | awk '{print $1}'); do kill "$p" 2>/dev/null; done
  sleep 4
  for p in $(ps -eo pid,cmd | grep -E 'gzserver|gzclient' | grep -v grep | awk '{print $1}'); do kill -9 "$p" 2>/dev/null; done
  sleep 2
  setsid nohup ros2 launch ican_shoot_sim sim.launch.py gui:=false > /tmp/sim_v10_$1.log 2>&1 < /dev/null &
  for i in $(seq 1 50); do
    sleep 3
    if timeout 5 ros2 topic list 2>/dev/null | grep -q '^/odom$'; then
      sleep 5
      HZ=$(timeout 8 ros2 topic hz /camera/image 2>&1 | grep "average rate" | tail -1 | awk '{print $3}')
      echo "  camera ${HZ:-0} Hz" | tee -a "$OUT"
      [ -n "$HZ" ] && return 0
    fi
  done
  return 1
}

for SEED in 201 202 203; do
  {
    echo "############################################################"
    echo "###  seed $SEED  (v10)"
    echo "############################################################"
  } | tee -a "$OUT"

  ./scripts/randomize_ground_textures.sh --seed "$SEED" > /tmp/rand_$SEED.log 2>&1
  python3 - <<'PY' | tee -a "$OUT"
import json
from pathlib import Path
d = json.loads(Path("/opt/ican_shoot_sim/share/ican_land_sim/materials/shooting_texture_assignment.json").read_text())
a = d.get("assignment", {})
print("贴图: " + ", ".join(f"{k[-1]}={v}" for k, v in sorted(a.items())))
PY

  restart_sim "$SEED" || { echo "仿真失败，跳过 $SEED" | tee -a "$OUT"; continue; }

  pkill -f recognize_node 2>/dev/null; sleep 2
  setsid nohup ros2 run ican_recognize recognize_node --ros-args \
    -p min_publish_interval:=0.4 -p publish_debug_image:=false \
    -p model_path:="$V10" > /tmp/node_v10_$SEED.log 2>&1 < /dev/null &
  sleep 14
  timeout 900 python3 scripts/acceptance_run.py --dist 0.8 --per-point 10 --out "v10_seed${SEED}.txt" 2>&1 | tail -9 | tee -a "$OUT"
  pkill -f recognize_node 2>/dev/null; sleep 2
done

echo "===== 汇总 ====="
grep -E "seed|总体" "$OUT"
