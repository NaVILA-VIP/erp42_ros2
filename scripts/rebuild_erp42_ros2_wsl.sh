#!/usr/bin/env bash
# Incrementally rebuild the two ERP42 packages changed by idle-hold controls.
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
set -u
source_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
workspace="${ERP42_WS:-/root/erp42_ws}"
mkdir -p "$workspace"
cd "$workspace"
colcon build --base-paths "$source_dir" \
  --packages-select erp42_control erp42 --event-handlers console_cohesion+
echo "ERP42 ROS 2 packages rebuilt in: $workspace"
