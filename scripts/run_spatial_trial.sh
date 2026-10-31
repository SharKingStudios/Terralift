#!/usr/bin/env bash
# Launch one real Claude-guided (or explicit --mock) Gazebo spatial trial.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENARIO="distance_ranking_flip"
MODEL="${ANTHROPIC_MODEL:-claude-haiku-5-5}"
GUI=true
RVIZ=true
MOCK=false
AUTO_CLOSE=false
RESULTS_DIR="${TERRALIFT_RESULTS_DIR:-}"

usage() {
  cat <<'EOF'
Usage: ./scripts/run_spatial_trial.sh [options]
  --scenario NAME       Scenario JSON stem (default: distance_ranking_flip)
  --model MODEL         Anthropic model (default: $ANTHROPIC_MODEL)
  --gui                 Start Gazebo GUI (default)
  --headless            Run Gazebo server only (and disable RViz)
  --no-rviz             Do not open RViz alongside the Gazebo GUI
  --mock                Use deterministic test fixture; never calls Anthropic
  --close               Close Gazebo automatically after the trial
  --results-dir PATH    Override TERRALIFT_RESULTS_DIR
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario) SCENARIO="$2"; shift 2 ;;
    --model) MODEL="$2"; shift 2 ;;
    --gui) GUI=true; RVIZ=true; shift ;;
    --headless) GUI=false; RVIZ=false; shift ;;
    --no-rviz) RVIZ=false; shift ;;
    --mock) MOCK=true; shift ;;
    --close) AUTO_CLOSE=true; shift ;;
    --results-dir) RESULTS_DIR="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

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
ros2 launch terralift spatial_experiment.launch.py \
  gui:="$GUI" rviz:="$RVIZ" scenario_path:="$SCENARIO_PATH" map_yaml:="$MAP_YAML" \
  model:="$MODEL" mock:="$MOCK" auto_close:="$AUTO_CLOSE"
