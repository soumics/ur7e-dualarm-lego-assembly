#!/bin/bash
# Stop every simulation process in this container. Needed because SIGINT to
# `ros2 launch` can leave the `gz sim` server orphaned (seen in Phase 0).
pkill -INT -f "ros2 launch" 2>/dev/null
sleep 3
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "ruby .*gz sim" 2>/dev/null
pkill -9 -f "move_group|rviz2|robot_state_publisher|parameter_bridge|controller_manager/spawner|ros_gz_sim/create" 2>/dev/null
sleep 1
if pgrep -f "gz sim|move_group|rviz2" >/dev/null; then
    echo "some processes still running:"; pgrep -fa "gz sim|move_group|rviz2"
else
    echo "simulation stopped"
fi
