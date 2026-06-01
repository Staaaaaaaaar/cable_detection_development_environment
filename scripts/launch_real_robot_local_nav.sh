#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source_setup() {
  local setup_file="$1"
  # ROS setup scripts may read variables that are not initialized.
  set +u
  # shellcheck source=/dev/null
  source "${setup_file}"
  set -u
}

if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  source_setup "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  source_setup "${ROOT_DIR}/install/setup.bash"
fi

LAUNCH_FILE="${ROOT_DIR}/src/vehicle_simulator/launch/system_real_robot.launch.py"
if [[ ! -f "${LAUNCH_FILE}" ]]; then
  echo "Launch file not found: ${LAUNCH_FILE}" >&2
  exit 1
fi

ros2 launch "${LAUNCH_FILE}" "$@"
