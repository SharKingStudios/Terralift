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
DDS_IFACE="${TERRALIFT_DDS_IFACE:-}"
DDS_ADDR="${TERRALIFT_DDS_ADDR:-}"

usage() {
  cat <<USAGE
Usage: $0 [--nav2-params FILE] [--use-apriltags true|false] [--dds-interface IFACE] [--dds-address ADDR] [--no-restart] [--skip-build] [--status]
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
    --dds-interface)
      DDS_IFACE="${2:?missing interface name}"
      shift 2
      ;;
    --dds-address)
      DDS_ADDR="${2:?missing IPv4 address}"
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

iface_for_addr() {
  local addr="$1"
  ip -o -4 addr show scope global 2>/dev/null | awk -v addr="$addr" '{ split($4, a, "/"); if (a[1] == addr) { print $2 " " a[1]; exit } }'
}

addr_for_iface() {
  local iface="$1"
  ip -o -4 addr show dev "$iface" scope global 2>/dev/null | awk '{ split($4, a, "/"); print $2 " " a[1]; exit }'
}

default_route_iface() {
  ip route show default 2>/dev/null | awk 'NR == 1 { for (i = 1; i <= NF; i++) if ($i == "dev") { print $(i + 1); exit } }'
}

resolve_dds_endpoint() {
  local endpoint server_ip iface

  if [[ -n "$DDS_ADDR" ]]; then
    endpoint="$(iface_for_addr "$DDS_ADDR")"
    if [[ -n "$endpoint" ]]; then
      echo "$endpoint"
      return 0
    fi
  fi

  if [[ -n "$DDS_IFACE" ]]; then
    endpoint="$(addr_for_iface "$DDS_IFACE")"
    if [[ -n "$endpoint" ]]; then
      echo "$endpoint"
      return 0
    fi
  fi

  if [[ -n "${SSH_CONNECTION:-}" ]]; then
    server_ip="$(awk '{ print $3 }' <<<"$SSH_CONNECTION")"
    endpoint="$(iface_for_addr "$server_ip")"
    if [[ -n "$endpoint" ]]; then
      echo "$endpoint"
      return 0
    fi
  fi

  iface="$(default_route_iface)"
  if [[ -n "$iface" ]]; then
    endpoint="$(addr_for_iface "$iface")"
    if [[ -n "$endpoint" ]]; then
      echo "$endpoint"
      return 0
    fi
  fi

  ip -o -4 addr show scope global 2>/dev/null | awk '{ split($4, a, "/"); print $2 " " a[1]; exit }'
}

DDS_ENDPOINT="$(resolve_dds_endpoint || true)"
DDS_RUN_IFACE=""
DDS_RUN_ADDR=""
if [[ "$DDS_ENDPOINT" == *" "* ]]; then
  DDS_RUN_IFACE="${DDS_ENDPOINT%% *}"
  DDS_RUN_ADDR="${DDS_ENDPOINT##* }"
fi
cat > "$RUNNER" <<EOF
#!/usr/bin/env bash
set -eo pipefail
cd "\$HOME/terralift_ws"
export ROS_DOMAIN_ID=17
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
DDS_RUN_IFACE="$DDS_RUN_IFACE"
DDS_RUN_ADDR="$DDS_RUN_ADDR"
DDS_CONFIG="$LOG_DIR/${SESSION}_cyclonedds.xml"
if [[ -n "\$DDS_RUN_IFACE" && -n "\$DDS_RUN_ADDR" ]]; then
  cat > "\$DDS_CONFIG" <<DDS_EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="\$DDS_RUN_IFACE" address="\$DDS_RUN_ADDR" />
      </Interfaces>
      <DontRoute>true</DontRoute>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
DDS_EOF
  export CYCLONEDDS_URI="file://\$DDS_CONFIG"
  echo "CycloneDDS pinned to \$DDS_RUN_IFACE at \$DDS_RUN_ADDR"
elif [[ -f "\$HOME/cyclonedds.xml" ]]; then
  export CYCLONEDDS_URI="file://\$HOME/cyclonedds.xml"
  echo "WARNING: could not auto-select a DDS interface; falling back to \$CYCLONEDDS_URI"
else
  unset CYCLONEDDS_URI || true
  echo "WARNING: could not auto-select a DDS interface; CycloneDDS will use defaults."
fi
LIDAR_LOG="$LOG_DIR/${SESSION}_rplidar.log"
LIDAR_PID=""
SCAN_WAIT_PID=""
cleanup() {
  if [[ -n "\${SCAN_WAIT_PID:-}" ]] && kill -0 "\$SCAN_WAIT_PID" 2>/dev/null; then
    kill "\$SCAN_WAIT_PID" 2>/dev/null || true
    wait "\$SCAN_WAIT_PID" 2>/dev/null || true
  fi
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

LIDAR_READY=false
LIDAR_READY_TIMEOUT="\${TERRALIFT_LIDAR_READY_TIMEOUT:-15}"
LIDAR_INITIAL_SETTLE_SECONDS="\${TERRALIFT_LIDAR_INITIAL_SETTLE_SECONDS:-2}"
for lidar_attempt in \$(seq 1 3); do
  echo "Starting RPLIDAR preflight attempt \$lidar_attempt at \$(date)"
  : >"\$LIDAR_LOG"
  if command -v fuser >/dev/null 2>&1; then
    fuser -k /dev/ttyUSB0 >/dev/null 2>&1 || true
  fi
  if [[ "\$lidar_attempt" == "1" ]]; then
    echo "Allowing \${LIDAR_INITIAL_SETTLE_SECONDS}s for the USB lidar reset"
    sleep "\$LIDAR_INITIAL_SETTLE_SECONDS"
  fi
  sleep 0.5
  ros2 launch terralift rplidar.launch.py >"\$LIDAR_LOG" 2>&1 &
  LIDAR_PID=\$!

  echo "Waiting up to \${LIDAR_READY_TIMEOUT}s for /scan"
  timeout "\$LIDAR_READY_TIMEOUT" ros2 topic echo /scan --once >/dev/null 2>&1 &
  SCAN_WAIT_PID=\$!
  LIDAR_EXITED=false
  while kill -0 "\$SCAN_WAIT_PID" 2>/dev/null; do
    if ! kill -0 "\$LIDAR_PID" 2>/dev/null; then
      echo "RPLIDAR preflight process exited before /scan appeared on attempt \$lidar_attempt. Log follows:"
      cat "\$LIDAR_LOG" || true
      LIDAR_EXITED=true
      kill "\$SCAN_WAIT_PID" 2>/dev/null || true
      wait "\$SCAN_WAIT_PID" 2>/dev/null || true
      break
    fi
    sleep 0.25
  done
  if [[ "\$LIDAR_EXITED" == "false" ]] && wait "\$SCAN_WAIT_PID"; then
    LIDAR_READY=true
    SCAN_WAIT_PID=""
    break
  fi
  SCAN_WAIT_PID=""

  echo "RPLIDAR attempt \$lidar_attempt did not publish /scan. Log follows:"
  cat "\$LIDAR_LOG" || true
  if [[ -n "\${LIDAR_PID:-}" ]] && kill -0 "\$LIDAR_PID" 2>/dev/null; then
    kill "\$LIDAR_PID" 2>/dev/null || true
    wait "\$LIDAR_PID" 2>/dev/null || true
  fi
  LIDAR_PID=""
  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 daemon start >/dev/null 2>&1 || true
  sleep 2
done

if [[ "\$LIDAR_READY" != "true" ]]; then
  echo "RPLIDAR did not publish /scan after 3 preflight attempts. Last log follows:"
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
