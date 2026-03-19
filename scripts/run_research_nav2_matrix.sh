#!/usr/bin/env bash
set -euo pipefail

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

if [[ ! -f "$WORKSPACE_SETUP" ]]; then
  echo "Workspace setup file not found: $WORKSPACE_SETUP" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$WORKSPACE_SETUP"

TOTAL_RUNS=$(( ${#CONFIGS[@]} * REPEATS_PER_CONFIG ))
RUN_INDEX=0

for config in "${CONFIGS[@]}"; do
  for ((trial=1; trial<=REPEATS_PER_CONFIG; trial++)); do
    RUN_INDEX=$((RUN_INDEX + 1))

    echo "============================================================"
    echo "Run $RUN_INDEX / $TOTAL_RUNS"
    echo "Running config: $config"
    echo "Repeat: $trial / $REPEATS_PER_CONFIG"
    echo "Goal: ($GOAL_X, $GOAL_Y, yaw=$GOAL_YAW) in frame $GOAL_FRAME"
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
      cmd_vel_input_topic:="$CMD_VEL_INPUT_TOPIC"

    echo
    echo "Finished config: $config repeat $trial"

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