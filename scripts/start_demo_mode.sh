#!/usr/bin/env bash
set -eo pipefail

SESSION="${TL_DEMO_SESSION:-terralift_demo}"
LOG_DIR="$HOME/terralift_run_logs"
RUNNER="$LOG_DIR/${SESSION}_runner.sh"
NAV2_PARAMS="dwb_smac2d_course_valid.yaml"
USE_APRILTAGS="false"
RESTART="true"
STATUS_ONLY="false"
SKIP_BUILD="false"

usage() {
  cat <<USAGE
Usage: $0 [--nav2-params FILE] [--use-apriltags true|false] [--no-restart] [--skip-build] [--status]
Defaults: --nav2-params dwb_smac2d_course_valid.yaml --use-apriltags false
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --nav2-params)
      NAV2_PARAMS="${2:?missing nav2 params file}"
      shift 2
      ;;
    --use-apriltags)
      USE_APRILTAGS="${2:?missing true/false}"
      shift 2
      ;;
    --no-restart)
      RESTART="false"
      shift
      ;;
    --skip-build)
      SKIP_BUILD="true"
      shift
      ;;
    --status)
      STATUS_ONLY="true"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

mkdir -p "$LOG_DIR"

if [[ "$STATUS_ONLY" == "true" ]]; then
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "tmux session '$SESSION' is running"
    tmux list-panes -t "$SESSION" -F '#{pane_pid} #{pane_current_command}' 2>/dev/null || true
  else
    echo "tmux session '$SESSION' is not running"
  fi
  pgrep -af "ros2 launch terralift|nav2_|slam_toolbox|rplidar|static_transform_publisher|robot_localization|/terralift_ws/install/terralift/lib/terralift/(drivetrain_node|open_loop_odom|cmd_vel_arbiter|cmd_vel_to_mecanum|demo_mode_node|imu_node|led_node|lift_arm_node|tag_|camera|yuyv)" || true
  exit 0
fi

if [[ "$RESTART" == "true" ]]; then
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Stopping existing tmux session '$SESSION'"
    tmux kill-session -t "$SESSION" || true
  fi

  echo "Stopping existing Terralift ROS processes for user $USER"
  CLEANUP_PATTERN='ros2 launch terralift|nav2_|slam_toolbox|rplidar|static_transform_publisher|robot_localization|/terralift_ws/install/terralift/lib/terralift/(drivetrain_node|open_loop_odom|cmd_vel_arbiter|cmd_vel_to_mecanum|demo_mode_node|imu_node|led_node|lift_arm_node|tag_|camera|yuyv)'
  pkill -TERM -u "$USER" -f "$CLEANUP_PATTERN" 2>/dev/null || true
  sleep 1
  pkill -KILL -u "$USER" -f "$CLEANUP_PATTERN" 2>/dev/null || true
fi

cat > "$RUNNER" <<EOF
#!/usr/bin/env bash
set -eo pipefail
cd "\$HOME/terralift_ws"
export ROS_DOMAIN_ID=17
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://\$HOME/cyclonedds.xml
LIDAR_LOG="$LOG_DIR/${SESSION}_rplidar.log"
LIDAR_PID=""
cleanup() {
  if [[ -n "\${LIDAR_PID:-}" ]] && kill -0 "\$LIDAR_PID" 2>/dev/null; then
    echo "Stopping RPLIDAR preflight process \$LIDAR_PID"
    kill "\$LIDAR_PID" 2>/dev/null || true
    wait "\$LIDAR_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM HUP
source /opt/ros/jazzy/setup.bash
# Catch orphaned ROS nodes before build/start. This keeps hotfix restarts from
# inheriting stale graph participants or serial owners.
CLEANUP_PATTERN='ros2 launch terralift|nav2_|slam_toolbox|rplidar|static_transform_publisher|robot_localization|/terralift_ws/install/terralift/lib/terralift/(drivetrain_node|open_loop_odom|cmd_vel_arbiter|cmd_vel_to_mecanum|demo_mode_node|imu_node|led_node|lift_arm_node|tag_|camera|yuyv)'
pkill -TERM -u "\$USER" -f "\$CLEANUP_PATTERN" 2>/dev/null || true
sleep 0.5
pkill -KILL -u "\$USER" -f "\$CLEANUP_PATTERN" 2>/dev/null || true
if [[ "$SKIP_BUILD" != "true" ]]; then
  echo "Building Terralift workspace at \$(date)"
  colcon build --symlink-install
fi
source "\$HOME/terralift_ws/install/setup.bash"
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

echo "Starting RPLIDAR preflight at \$(date)"
ros2 launch terralift rplidar.launch.py >"\$LIDAR_LOG" 2>&1 &
LIDAR_PID=\$!
LIDAR_READY=false
for attempt in \$(seq 1 25); do
  if ! kill -0 "\$LIDAR_PID" 2>/dev/null; then
    echo "RPLIDAR preflight process exited before /scan appeared. Log follows:"
    cat "\$LIDAR_LOG" || true
    exit 1
  fi
  if timeout 2 ros2 topic echo /scan --once >/dev/null 2>&1; then
    LIDAR_READY=true
    break
  fi
  sleep 1
done

if [[ "\$LIDAR_READY" != "true" ]]; then
  echo "RPLIDAR did not publish /scan within preflight timeout. Log follows:"
  cat "\$LIDAR_LOG" || true
  exit 1
fi

echo "RPLIDAR is publishing /scan. Main stack will reuse this lidar process."
echo "Starting Terralift demo at \$(date)"
echo "ROS_DOMAIN_ID=\$ROS_DOMAIN_ID RMW_IMPLEMENTATION=\$RMW_IMPLEMENTATION CYCLONEDDS_URI=\$CYCLONEDDS_URI"
echo "Command: ros2 launch terralift terralift_demo.launch.py use_apriltags:=$USE_APRILTAGS nav2_params:=$NAV2_PARAMS use_lidar:=false"
ros2 launch terralift terralift_demo.launch.py use_apriltags:=$USE_APRILTAGS nav2_params:=$NAV2_PARAMS use_lidar:=false
EOF
chmod +x "$RUNNER"

echo "Starting tmux session '$SESSION'"
tmux new-session -d -s "$SESSION" "bash '$RUNNER' 2>&1 | tee '$LOG_DIR/${SESSION}.log'"
sleep 0.5
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "Started. Attach with: tmux attach -t $SESSION"
  echo "Log: $LOG_DIR/${SESSION}.log"
else
  echo "Failed to start tmux session '$SESSION'" >&2
  exit 1
fi
