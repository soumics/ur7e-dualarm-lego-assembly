#!/usr/bin/env bash
# Tile the demo windows side by side: Gazebo GUI left, RViz right (same size).
# Usage: demo_windows.sh [x0 y0 width height gap]
X0=${1:-70}; Y0=${2:-40}; W=${3:-1240}; H=${4:-1390}; GAP=${5:-6}
G=$(wmctrl -l | awk '/Gazebo Sim/{print $1; exit}')
R=$(wmctrl -l | awk '/ - RViz/{print $1; exit}')
[ -z "$G" ] || [ -z "$R" ] && { echo "windows not found (gazebo=$G rviz=$R)"; exit 1; }
wmctrl -i -r "$G" -e 0,$X0,$Y0,$W,$H
wmctrl -i -r "$R" -e 0,$((X0 + W + GAP)),$Y0,$W,$H
# keep both above other windows (e.g. the editor) so a screen recording only sees the demo
wmctrl -i -r "$G" -b add,above; wmctrl -i -r "$R" -b add,above
wmctrl -i -a "$G"; wmctrl -i -a "$R"
echo "gazebo=$G rviz=$R"
