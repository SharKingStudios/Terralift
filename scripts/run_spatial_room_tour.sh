#!/usr/bin/env bash
# Start a visible, no-AI Nav2 tour through every warehouse room, then leave
# Gazebo open with the robot stopped in room A for visual verification.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ -f /opt/ros/jazzy/setup.bash ]] || { echo "ROS 2 Jazzy was not found at /opt/ros/jazzy." >&2; exit 1; }

set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$ROOT"
colcon build --symlink-install --packages-select terralift
set +u
source "$ROOT/install/setup.bash"
set -u

CACHE_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/terralift_spatial_experiment"
mkdir -p "$CACHE_DIR"
MAP_YAML="$CACHE_DIR/spatial_warehouse.yaml"
python3 "$ROOT/scripts/generate_spatial_map.py" --output "$CACHE_DIR/spatial_warehouse.pgm" --yaml-output "$MAP_YAML"
export GZ_SIM_RESOURCE_PATH="$ROOT/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

cleanup() {
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

echo "Starting visible Nav2 room tour: A -> B -> C -> D -> E -> A"
ros2 launch terralift spatial_experiment.launch.py gui:=true rviz:=true run_trial:=false map_yaml:="$MAP_YAML" &
LAUNCH_PID=$!

set +e
ros2 run terralift spatial_room_tour --ros-args -p use_sim_time:=true -p dwell_sec:=2.0
TOUR_STATUS=$?
set -e
if [[ "$TOUR_STATUS" -eq 0 ]]; then
  echo "Tour complete: robot is stopped in room A. Gazebo remains open; press Ctrl-C here when finished."
else
  echo "Tour stopped with status $TOUR_STATUS. Gazebo remains open for diagnosis; press Ctrl-C here when finished." >&2
fi
wait "$LAUNCH_PID"
