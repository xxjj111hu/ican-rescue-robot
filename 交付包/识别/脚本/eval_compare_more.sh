#!/usr/bin/env bash
cd /home/longying01/demo_shoot_ws || exit 1
source /opt/ros/humble/setup.bash
source install/setup.bash
source /opt/ican_shoot_sim/setup.bash

V6=runs/detect/runs/ican_v6/weights/best.pt
V9=runs/detect/runs/detect/runs/ican_v9-2/weights/best.pt
OUT=compare_v6_v9_more.txt
: > "$OUT"

restart_sim () {
  for p in $(ps -eo pid,cmd | grep -E 'gzserver|gzclient|sim.launch' | grep -v grep | awk '{print $1}'); do kill "$p" 2>/dev/null; done
  sleep 4
  for p in $(ps -eo pid,cmd | grep -E 'gzserver|gzclient' | grep -v grep | awk '{print $1}'); do kill -9 "$p" 2>/dev/null; done
  sleep 2
  setsid nohup ros2 launch ican_shoot_sim sim.launch.py gui:=false > /tmp/sim_more_$1.log 2>&1 < /dev/null &
  for i in $(seq 1 50); do
    sleep 3
    if timeout 5 ros2 topic list 2>/dev/null | grep -q '^/odom$'; then
      sleep 5
      HZ=$(timeout 8 ros2 topic hz /camera/image 2>&1 | grep "average rate" | tail -1 | awk '{print $3}')
      echo "  /camera/image = ${HZ:-0} Hz"
      [ -n "$HZ" ] && return 0
    fi
  done
  return 1
}

run_model () {
  pkill -f recognize_node 2>/dev/null
  sleep 2
  setsid nohup ros2 run ican_recognize recognize_node --ros-args \
    -p min_publish_interval:=0.4 -p publish_debug_image:=false \
    -p model_path:="$2" > "/tmp/node_$1.log" 2>&1 < /dev/null &
  sleep 14
  timeout 900 python3 scripts/acceptance_run.py --dist 0.8 --per-point 10 --out "$3" 2>&1 | tail -9
  pkill -f recognize_node 2>/dev/null
  sleep 2
}

for SEED in 203 204; do
  {
    echo "############################################################"
    echo "###  seed $SEED"
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

  if ! restart_sim "$SEED"; then
    echo "仿真启动失败，跳过 seed $SEED" | tee -a "$OUT"; continue
  fi

  echo "--- v9 ---" | tee -a "$OUT"
  run_model "v9_$SEED" "$V9" "cmp2_seed${SEED}_v9.txt" | tee -a "$OUT"
  echo "--- v6 ---" | tee -a "$OUT"
  run_model "v6_$SEED" "$V6" "cmp2_seed${SEED}_v6.txt" | tee -a "$OUT"
done

echo "===== 汇总 ====="
grep -E "seed|总体|Hz" "$OUT"
