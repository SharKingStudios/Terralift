#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_DIR="${TERRALIFT_LAPTOP_WS:-$HOME/terralift_ws}"
DESKTOP_DIR="${XDG_DESKTOP_DIR:-$HOME/Desktop}"
LAUNCHER_SRC="$REPO_ROOT/scripts/start_terralift_teleop.sh"
DESKTOP_SRC="$REPO_ROOT/desktop"

mkdir -p "$WORKSPACE_DIR" "$DESKTOP_DIR"
install -m 0755 "$LAUNCHER_SRC" "$WORKSPACE_DIR/start_terralift_teleop.sh"

for entry in "Terralift Launcher.desktop" "Terralift Wi-Fi Fix.desktop"; do
  tmp="$(mktemp)"
  sed "s|Exec=/home/logan/terralift_ws/start_terralift_teleop.sh|Exec=$WORKSPACE_DIR/start_terralift_teleop.sh|g" \
    "$DESKTOP_SRC/$entry" > "$tmp"
  install -m 0755 "$tmp" "$DESKTOP_DIR/$entry"
  rm -f "$tmp"

  if command -v gio >/dev/null 2>&1; then
    gio set "$DESKTOP_DIR/$entry" metadata::trusted true >/dev/null 2>&1 || true
  fi

done

echo "Installed Terralift laptop launcher to $WORKSPACE_DIR/start_terralift_teleop.sh"
echo "Installed desktop shortcuts to $DESKTOP_DIR"
