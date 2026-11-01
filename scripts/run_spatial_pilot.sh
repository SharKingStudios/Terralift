#!/usr/bin/env bash
# Run the pre-batch paired pilot only. It never starts the full matrix.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODEL="${ANTHROPIC_MODEL:-claude-haiku-5-5}"
RESULTS_DIR="${TERRALIFT_RESULTS_DIR:-$HOME/TerraliftExperiments/pilot_runs}"
VISUAL_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./scripts/run_spatial_pilot.sh [options]

Runs six real, paired trials only: neutral and verification_guided on the
same fixed seed for a topology answer-flip scenario, an unchanged control,
and a semantic answer-invariant scenario. Each trial auto-closes after its
saved bag, transcript, memory history, metrics, and audit are finalized.

Options:
  --headless            Disable Gazebo and RViz windows.
  --model MODEL         Claude model (default: $ANTHROPIC_MODEL).
  --results-dir PATH    Pilot results root.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless) VISUAL_ARGS+=(--headless); shift ;;
    --model) MODEL="$2"; shift 2 ;;
    --results-dir) RESULTS_DIR="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

mkdir -p "$RESULTS_DIR"
declare -a PILOT_CASES=(
  "distance_ranking_flip:pilot-topology-flip-v1"
  "no_change_control:pilot-no-change-v1"
  "semantic_answer_invariant_move_b_e_to_b:pilot-semantic-invariant-v1"
)
declare -a CONDITIONS=(neutral verification_guided)

for case_item in "${PILOT_CASES[@]}"; do
  scenario="${case_item%%:*}"
  seed="${case_item#*:}"
  for condition in "${CONDITIONS[@]}"; do
    echo "=== pilot scenario=$scenario seed=$seed condition=$condition ==="
    "$ROOT/scripts/run_spatial_trial.sh" --scenario "$scenario" --seed "$seed" \
      --prompt-condition "$condition" --model "$MODEL" --results-dir "$RESULTS_DIR" --close "${VISUAL_ARGS[@]}"
  done
done

python3 "$ROOT/scripts/summarize_spatial_pilot.py" "$RESULTS_DIR"
echo "Pilot complete. Review every ai/requests_responses.jsonl, navigation/observations.jsonl, and audit.json before running any full matrix."
