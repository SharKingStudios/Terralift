#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN_DIR="${TERRALIFT_ROBOT_BIN:-$HOME/terralift_ws/bin}"

mkdir -p "$BIN_DIR"
install -m 0755 "$REPO_ROOT/scripts/start_demo_mode.sh" "$BIN_DIR/start_demo_mode.sh"
install -m 0755 "$REPO_ROOT/scripts/stop_demo_mode.sh" "$BIN_DIR/stop_demo_mode.sh"

echo "Installed Terralift robot helpers to $BIN_DIR"
