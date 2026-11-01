#!/usr/bin/env bash
# Launch one real Claude-guided (or explicit --mock) Gazebo spatial trial.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENARIO="distance_ranking_flip"
MODEL="${ANTHROPIC_MODEL:-claude-haiku-5-5}"
PROMPT_CONDITION="neutral"
TRIAL_SEED=""
GUI=true
RVIZ=true
MOCK=false
MOCK_STRATEGY="answer_flip"
AUTO_CLOSE=false
RESULTS_DIR="${TERRALIFT_RESULTS_DIR:-}"
# The pilot runner supplies these variables.  Sensible defaults keep a direct
# single-trial invocation readable in either a GUI or headless terminal.
BATCH_INDEX="${SPATIAL_BATCH_INDEX:-1}"
BATCH_TOTAL="${SPATIAL_BATCH_TOTAL:-1}"
BATCH_ETA_SECONDS="${SPATIAL_BATCH_ETA_SECONDS:-0}"

format_duration() {
  local total_seconds="$1"
  local hours=$((total_seconds / 3600))
  local minutes=$(((total_seconds % 3600) / 60))
  local seconds=$((total_seconds % 60))
  if (( hours > 0 )); then
    printf '%dh %02dm' "$hours" "$minutes"
  elif (( minutes > 0 )); then
    printf '%dm %02ds' "$minutes" "$seconds"
  else
    printf '%ds' "$seconds"
  fi
}

usage() {
  cat <<'EOF'
Usage: ./scripts/run_spatial_trial.sh [options]
  --scenario NAME       Scenario JSON stem (default: distance_ranking_flip)
  --model MODEL         Anthropic model (default: $ANTHROPIC_MODEL)
  --prompt-condition C  neutral (default) or verification_guided
  --seed VALUE          Persist a paired-trial seed/label in trial metadata
  --gui                 Start Gazebo GUI (default)
  --headless            Run Gazebo server only (and disable RViz)
  --no-rviz             Do not open RViz alongside the Gazebo GUI
  --mock                Use deterministic test fixture; never calls Anthropic
  --mock-survey         Survey every passage with the test fixture; calibration only
  --close               Close Gazebo automatically after the trial
  --results-dir PATH    Override TERRALIFT_RESULTS_DIR
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario) SCENARIO="$2"; shift 2 ;;
    --model) MODEL="$2"; shift 2 ;;
    --prompt-condition) PROMPT_CONDITION="$2"; shift 2 ;;
    --seed) TRIAL_SEED="$2"; shift 2 ;;
    --gui) GUI=true; RVIZ=true; shift ;;
    --headless) GUI=false; RVIZ=false; shift ;;
    --no-rviz) RVIZ=false; shift ;;
    --mock) MOCK=true; shift ;;
    --mock-survey) MOCK=true; MOCK_STRATEGY="survey_passages"; shift ;;
    --close) AUTO_CLOSE=true; shift ;;
    --results-dir) RESULTS_DIR="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done
[[ "$PROMPT_CONDITION" == "neutral" || "$PROMPT_CONDITION" == "verification_guided" ]] || {
  echo "--prompt-condition must be neutral or verification_guided" >&2; exit 2;
}

# Editors on Windows commonly save `.env` with CRLF line endings. Source a
# newline-normalized stream so the same untracked credential file works from
# both Windows and WSL without rewriting or exposing its contents.
if [[ -f "$ROOT/.env" ]]; then
  set -a
  source <(sed 's/\r$//' "$ROOT/.env")
  set +a
fi
if [[ "$MODEL" == "claude-haiku-5-5" && -n "${ANTHROPIC_MODEL:-}" ]]; then
  MODEL="$ANTHROPIC_MODEL"
fi
[[ -f /opt/ros/jazzy/setup.bash ]] || { echo "ROS 2 Jazzy was not found at /opt/ros/jazzy." >&2; exit 1; }
set +u
source /opt/ros/jazzy/setup.bash
set -u
# Keep this experiment separate from a robot bringup or an old Gazebo/ROS
# run left on the default domain.  Every ROS process launched by this script,
# including the bridge and Nav2, inherits the same domain.  A custom Gazebo
# partition remains opt-in: this WSL/Gazebo combination has reliable sensor
# bridging on its normal transport partition, while a custom partition needs
# host-specific validation before use.
SPATIAL_ROS_DOMAIN_ID="${SPATIAL_ROS_DOMAIN_ID:-42}"
[[ "$SPATIAL_ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] && (( SPATIAL_ROS_DOMAIN_ID <= 232 )) || {
  echo "SPATIAL_ROS_DOMAIN_ID must be an integer from 0 to 232." >&2; exit 2;
}
export ROS_DOMAIN_ID="$SPATIAL_ROS_DOMAIN_ID"
if [[ -n "${SPATIAL_GZ_PARTITION:-}" ]]; then
  export GZ_PARTITION="$SPATIAL_GZ_PARTITION"
else
  unset GZ_PARTITION
fi
echo "Spatial experiment isolation: ROS_DOMAIN_ID=$ROS_DOMAIN_ID, GZ_PARTITION=${GZ_PARTITION:-default}"
echo "============================================================"
echo "Spatial trial ${BATCH_INDEX} / ${BATCH_TOTAL}"
echo "Scenario: ${SCENARIO} | prompt: ${PROMPT_CONDITION} | model: ${MODEL}"
if (( BATCH_TOTAL > 1 )); then
  echo "Estimated batch time remaining: $(format_duration "$BATCH_ETA_SECONDS")"
else
  echo "Estimated batch time remaining: standalone trial (no batch queue)"
fi
echo "Live simplified AI prompts and complete visible AI responses follow below."
echo "Full API records: <trial>/ai/requests_responses.jsonl"
echo "============================================================"
for command in ros2 colcon gz python3; do
  command -v "$command" >/dev/null || { echo "Missing required command: $command" >&2; exit 1; }
done
if [[ "$MOCK" != true && -z "${ANTHROPIC_API_KEY:-}" ]]; then
  echo "ANTHROPIC_API_KEY is required. Copy .env.example to .env or export it. Use --mock only for no-cost validation." >&2
  exit 1
fi
SCENARIO_PATH="$ROOT/config/spatial_scenarios/${SCENARIO}.json"
[[ -f "$SCENARIO_PATH" ]] || { echo "Unknown scenario: $SCENARIO" >&2; exit 1; }

cd "$ROOT"
# Keep Claude dependencies isolated from apt-managed ROS Python packages. The
# first invocation creates this venv automatically, so configuration only
# requires the Anthropic key.
VENV_DIR="${TERRALIFT_VENV_DIR:-$ROOT/.venv-spatial-experiment}"
if [[ ! -x "$VENV_DIR/bin/python" ]]; then
  python3 -m venv "$VENV_DIR"
fi
if ! "$VENV_DIR/bin/python" -c 'import anthropic, networkx, dotenv' >/dev/null 2>&1; then
  "$VENV_DIR/bin/python" -m pip install --disable-pip-version-check -r "$ROOT/requirements-spatial-experiment.txt"
fi
# ros2 launch uses the ROS interpreter; expose just this venv's dependencies
# to child ROS nodes without replacing the apt-managed rclpy installation.
export PYTHONPATH="$VENV_DIR/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
colcon build --symlink-install --packages-select terralift
set +u
source "$ROOT/install/setup.bash"
set -u

CACHE_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/terralift_spatial_experiment"
mkdir -p "$CACHE_DIR"
MAP_YAML="$CACHE_DIR/spatial_warehouse.yaml"
python3 "$ROOT/scripts/generate_spatial_map.py" --output "$CACHE_DIR/spatial_warehouse.pgm" --yaml-output "$MAP_YAML"
export GZ_SIM_RESOURCE_PATH="$ROOT/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export ANTHROPIC_MODEL="$MODEL"
[[ -n "$RESULTS_DIR" ]] && export TERRALIFT_RESULTS_DIR="$RESULTS_DIR"

if [[ "$MOCK" == true ]]; then
  echo "Starting ${SCENARIO} (explicit mock mode)"
else
  echo "Starting ${SCENARIO} (real Anthropic API)"
fi
LAUNCH_ARGS=(
  "gui:=$GUI" "rviz:=$RVIZ" "scenario_path:=$SCENARIO_PATH" "map_yaml:=$MAP_YAML"
  "model:=$MODEL" "prompt_condition:=$PROMPT_CONDITION" "mock:=$MOCK" "mock_strategy:=$MOCK_STRATEGY" "auto_close:=$AUTO_CLOSE"
)
[[ -n "$TRIAL_SEED" ]] && LAUNCH_ARGS+=("trial_seed:=$TRIAL_SEED")
ros2 launch terralift spatial_experiment.launch.py "${LAUNCH_ARGS[@]}"
