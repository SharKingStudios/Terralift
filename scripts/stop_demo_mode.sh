#!/usr/bin/env bash
set -eo pipefail

SESSION="${TL_DEMO_SESSION:-terralift_demo}"
CLEANUP_PATTERN='ros2 launch terralift|nav2_|slam_toolbox|rplidar|static_transform_publisher|robot_localization|/terralift_ws/install/terralift/lib/terralift/(drivetrain_node|open_loop_odom|cmd_vel_arbiter|cmd_vel_to_mecanum|demo_mode_node|imu_node|led_node|lift_arm_node|tag_|camera|yuyv)'

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "Stopping tmux session '$SESSION'"
  tmux kill-session -t "$SESSION" || true
fi

echo "Stopping Terralift ROS processes for user $USER"
pkill -TERM -u "$USER" -f "$CLEANUP_PATTERN" 2>/dev/null || true
sleep 1
pkill -KILL -u "$USER" -f "$CLEANUP_PATTERN" 2>/dev/null || true

if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
fi

echo "Stopped."
