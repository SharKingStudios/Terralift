#!/usr/bin/env bash
set -eo pipefail
SESSION="${TL_DEMO_SESSION:-terralift_demo}"
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "Stopping tmux session '$SESSION'"
  tmux kill-session -t "$SESSION" || true
fi
echo "Stopping Terralift ROS processes for user $USER"
pkill -TERM -u "$USER" -f 'ros2 launch terralift|nav2_|slam_toolbox|rplidar|static_transform_publisher' 2>/dev/null || true
sleep 1
pkill -KILL -u "$USER" -f 'ros2 launch terralift|nav2_|slam_toolbox|rplidar|static_transform_publisher' 2>/dev/null || true
ros2 daemon stop >/dev/null 2>&1 || true
echo "Stopped."
