#!/usr/bin/env bash
# Record a build with the demo views (run inside the container, demo.launch.py already up).
#   experiments/tools/record_demo.sh <config_dir> <out_dir>
# Captures the whole X screen (Gazebo left, RViz right, tiled by demo_windows.sh) while
# build_executor runs the plan, then switches both views to a close-up of the result.
# Writes <out_dir>/raw_screen.mp4, close-up timing in <out_dir>/marks.txt.
source ~/ws/install/setup.bash   # before set -u (colcon setup scripts use unset variables)
set -euo pipefail
CFG=${1:?config dir}; OUT=${2:?out dir}
mkdir -p "$OUT"
S=~/ws/src/lego_dualarm_bringup/scripts

$S/demo_windows.sh
sleep 2
ffmpeg -loglevel error -y -f x11grab -draw_mouse 0 -framerate 20 -video_size 2560x1600 -i :1 \
    -c:v h264_nvenc -preset p5 -cq 19 -pix_fmt yuv420p "$OUT/raw_screen.mp4" &
FF=$!
T0=$(date +%s.%N)
mark() { awk -v n="$1" -v a="$(date +%s.%N)" -v b="$T0" 'BEGIN{printf "%s %.2f\n", n, a-b}' >> "$OUT/marks.txt"; }
: > "$OUT/marks.txt"
sleep 3
mark build_start
ros2 run lego_dual_arm_task build_executor --ros-args -p plan:=$CFG/plan.json \
    -p inventory:=$CFG/inventory.json -p tag:=demo_video 2>&1 | tee "$OUT/build_executor.log" | grep -E "\] (lego|b[0-9])|logged to|reset" || true
mark build_end
sleep 4
# close-up of the finished structure (same viewpoint on both sides)
python3 $S/demo_view.py 0 0 0.765 0.42 0.55 1.15 --rviz-restart
sleep 10
$S/demo_windows.sh
sleep 1
mark closeup_start
sleep 12
mark end
kill -INT $FF; wait $FF || true
echo "recorded $OUT/raw_screen.mp4"; cat "$OUT/marks.txt"
