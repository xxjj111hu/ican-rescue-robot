cd /home/longying01/demo_shoot_ws || exit 1
R=runs/detect/runs/ican_v9
if [ -f "$R/results.csv" ]; then
  echo "轮数: $(( $(wc -l < "$R/results.csv") - 1 ))/50"
  tail -3 "$R/results.csv" | awk -F, '{printf "  ep %2s  mAP50=%.3f  mAP50-95=%.3f  P=%.2f  R=%.2f\n", $1, $8, $9, $6, $7}'
  # 估算速度
  N=$(( $(wc -l < "$R/results.csv") - 1 ))
  if [ "$N" -gt 0 ]; then
    FIRST=$(ls -l --time-style=+%s "$R/results.csv" | awk '{print $6}')
    echo "  已用时间估算依 results.csv 行数"
  fi
else
  echo "还在第 1 轮"
  tail -c 300 /tmp/yolo_v9.log | tr '\r' '\n' | tail -3
fi
