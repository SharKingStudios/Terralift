#!/usr/bin/env bash
set -eo pipefail

# -----------------------------
# Edit these variables for your experiment
# -----------------------------
WORKSPACE_SETUP="$HOME/terralift_ws/install/setup.bash"
PACKAGE_NAME="terralift"
LAUNCH_FILE="terralift_research.launch.py"

CONFIGS=(
  "rpp_smac2d.yaml"
  "rpp_smachybrid.yaml"
  "dwb_smac2d.yaml"
  "dwb_smachybrid.yaml"
)

REPEATS_PER_CONFIG=1

# Small delay after each run finishes, before prompting you
SLEEP_BETWEEN_RUNS_SEC=1

# If true, wait for you to press Enter before starting the first run
WAIT_FOR_ENTER_BEFORE_FIRST_RUN="true"

# If true, wait for you to press Enter before starting the next run
WAIT_FOR_ENTER_BETWEEN_RUNS="true"

AUTO_CLOSE="true"
RECORD_BAG="true"
RECORD_ALL_TOPICS="false"
BAG_BASE_DIR="$HOME/terralift_bags"
TRIAL_PREFIX="trial"
ENVIRONMENT_ID="lab_a"
TRIAL_NOTES="automated_nav2_matrix"
PARAMETER_SET_ID="baseline_2026_03_19"
NAV2_VERSION="unknown"
ROBOT_MODEL="terralift"
ROBOT_FW_DRIVER_VERSIONS="unknown"
USE_SLAM="true"
USE_APRILTAGS="false"
STARTUP_DELAY_SEC="4.0"
GOAL_TIMEOUT_SEC="90.0"
GOAL_FRAME="map"
GOAL_X="1.0"
GOAL_Y="0.0"
GOAL_YAW="0.0"
CMD_VEL_INPUT_TOPIC="/cmd_vel_nav_safe"

# Directory for per-run ROS launch logs
LOG_DIR="$HOME/terralift_run_logs"

if [[ ! -f "$WORKSPACE_SETUP" ]]; then
  echo "Workspace setup file not found: $WORKSPACE_SETUP" >&2
  exit 1
fi

mkdir -p "$LOG_DIR"
mkdir -p "$BAG_BASE_DIR"

# Source the workspace without nounset, since ROS/colcon setup scripts
# may reference unset trace variables.
# shellcheck disable=SC1090
source "$WORKSPACE_SETUP"

TOTAL_RUNS=$(( ${#CONFIGS[@]} * REPEATS_PER_CONFIG ))
RUN_INDEX=0

if [[ "$WAIT_FOR_ENTER_BEFORE_FIRST_RUN" == "true" ]]; then
  echo "Place the robot at the start position."
  read -r -p "Press Enter to start the experiment..."
  echo
fi

for config in "${CONFIGS[@]}"; do
  for ((trial=1; trial<=REPEATS_PER_CONFIG; trial++)); do
    RUN_INDEX=$((RUN_INDEX + 1))

    RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
    RUN_LOG="$LOG_DIR/${config%.yaml}_trial${trial}_${RUN_STAMP}.log"

    echo "============================================================"
    echo "Run $RUN_INDEX / $TOTAL_RUNS"
    echo "Running config: $config"
    echo "Repeat: $trial / $REPEATS_PER_CONFIG"
    echo "Goal: ($GOAL_X, $GOAL_Y, yaw=$GOAL_YAW) in frame $GOAL_FRAME"
    echo "ROS launch log: $RUN_LOG"
    echo "============================================================"

    ros2 launch "$PACKAGE_NAME" "$LAUNCH_FILE" \
      nav2_params:="$config" \
      auto_close:="$AUTO_CLOSE" \
      record_bag:="$RECORD_BAG" \
      record_all_topics:="$RECORD_ALL_TOPICS" \
      bag_base_dir:="$BAG_BASE_DIR" \
      trial_prefix:="$TRIAL_PREFIX" \
      environment_id:="$ENVIRONMENT_ID" \
      trial_notes:="$TRIAL_NOTES" \
      parameter_set_id:="$PARAMETER_SET_ID" \
      nav2_version:="$NAV2_VERSION" \
      robot_model:="$ROBOT_MODEL" \
      robot_firmware_driver_versions:="$ROBOT_FW_DRIVER_VERSIONS" \
      use_slam:="$USE_SLAM" \
      use_apriltags:="$USE_APRILTAGS" \
      startup_delay_sec:="$STARTUP_DELAY_SEC" \
      goal_timeout_sec:="$GOAL_TIMEOUT_SEC" \
      goal_frame:="$GOAL_FRAME" \
      goal_x:="$GOAL_X" \
      goal_y:="$GOAL_Y" \
      goal_yaw:="$GOAL_YAW" \
      cmd_vel_input_topic:="$CMD_VEL_INPUT_TOPIC" \
      >"$RUN_LOG" 2>&1

    LAUNCH_EXIT_CODE=$?

    echo
    echo "Finished config: $config repeat $trial"
    echo "Launch exit code: $LAUNCH_EXIT_CODE"
    echo "ROS launch log: $RUN_LOG"

    LATEST_METADATA="$(ls -1t "$BAG_BASE_DIR"/*/trial_metadata.json 2>/dev/null | head -n 1 || true)"

    if [[ -n "$LATEST_METADATA" ]]; then
      echo "Trial metadata: $LATEST_METADATA"
      python3 - <<'PY' "$LATEST_METADATA"
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
data = json.loads(path.read_text())

trial_id = data.get("trial_id", "unknown")
result = data.get("final_result", {})
success = result.get("success", None)
reason = result.get("reason", "")
error_code = result.get("error_code", "")
error_msg = result.get("error_msg", "")

status_text = "SUCCESS" if success is True else "FAILURE" if success is False else "UNKNOWN"

print(f"Trial ID: {trial_id}")
print(f"Trial result: {status_text}")
print(f"Reason: {reason}")
print(f"Nav2 error_code: {error_code}")
print(f"Nav2 error_msg: {error_msg}")
PY
    else
      echo "No trial metadata file found."
    fi

    if (( RUN_INDEX < TOTAL_RUNS )); then
      sleep "$SLEEP_BETWEEN_RUNS_SEC"

      if [[ "$WAIT_FOR_ENTER_BETWEEN_RUNS" == "true" ]]; then
        echo
        echo "Move the robot back to the start position."
        read -r -p "Press Enter to start the next run..."
      fi
    fi
  done
done

echo
echo "All experiment runs completed."
echo "Logs saved in: $LOG_DIR"