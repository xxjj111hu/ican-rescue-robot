cd /home/longying01/demo_shoot_ws || exit 1
R=runs/detect/runs/detect/runs/ican_v9-2
echo "=== 实际训练目录: $R ==="
if [ -f "$R/results.csv" ]; then
  echo "轮数: $(( $(wc -l < "$R/results.csv") - 1 ))/35"
  tail -4 "$R/results.csv" | awk -F, '{printf "  ep %2s  mAP50=%.3f  mAP50-95=%.3f  P=%.2f  R=%.2f\n", $1, $8, $9, $6, $7}'
  echo "--- 最好 3 轮 ---"
  tail -n +2 "$R/results.csv" | sort -t, -k8 -gr | head -3 | awk -F, '{printf "  ep %2s  mAP50=%.3f\n", $1, $8}'
fi
ls -la "$R/weights" 2>/dev/null
echo
echo "训练进程: $(ps -eo pid,cmd | grep 'yolo detect train' | grep -v grep | wc -l)"
