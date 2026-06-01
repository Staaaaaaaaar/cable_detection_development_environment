#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

# Skip simulator-related packages so build does not require Gazebo extensions.
SIM_PACKAGES=(
  vehicle_simulator
  velodyne_gazebo_plugins
  velodyne_simulator
  velodyne_description
)

echo "Building non-simulation packages..."
echo "Skipped packages: ${SIM_PACKAGES[*]}"

colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-skip "${SIM_PACKAGES[@]}" \
  "$@"
