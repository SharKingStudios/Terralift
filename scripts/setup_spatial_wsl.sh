#!/usr/bin/env bash
# Install every OS-level dependency for the Gazebo spatial-belief experiment.
# Intended for a fresh Ubuntu 24.04 (Noble) WSL2 or native Ubuntu machine.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -r /etc/os-release ]]; then
  echo "This bootstrap requires Ubuntu 24.04." >&2
  exit 1
fi
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_CODENAME:-}" != "noble" ]]; then
  echo "Unsupported OS: this experiment is pinned to Ubuntu 24.04 (Noble), not ${PRETTY_NAME:-unknown}." >&2
  exit 1
fi

if [[ "${EUID}" -eq 0 ]]; then
  SUDO=()
else
  command -v sudo >/dev/null || { echo "sudo is required for package installation." >&2; exit 1; }
  SUDO=(sudo)
fi

echo "Installing ROS 2 Jazzy, Gazebo Harmonic, Nav2, and build prerequisites..."
"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y ca-certificates curl gnupg lsb-release software-properties-common
"${SUDO[@]}" install -d -m 0755 /etc/apt/keyrings
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | "${SUDO[@]}" gpg --dearmor --yes -o /etc/apt/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
  | "${SUDO[@]}" tee /etc/apt/sources.list.d/ros2.list >/dev/null
"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y \
  ros-jazzy-desktop \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-interfaces \
  ros-jazzy-rosbag2-storage-mcap \
  build-essential git python3-pip python3-venv python3-colcon-common-extensions

echo
echo "System setup complete. From this Bash shell run:"
echo "  cd $ROOT"
echo "  cp .env.example .env"
echo "  # add ANTHROPIC_API_KEY to .env"
echo "  ./scripts/run_spatial_trial.sh"
echo
echo "For a visible, no-cost smoke test first:"
echo "  ./scripts/run_spatial_trial.sh --mock --close"
