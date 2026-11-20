#!/usr/bin/env bash
set -euo pipefail

# ROS 2 Humble's setup script reads optional unset variables.
set +u
source /opt/ros/humble/setup.bash
set -u
source_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
workspace="${ERP42_WS:-$HOME/erp42_ws}"

mkdir -p "$workspace"
rosdep install --from-paths \
  "$source_dir/erp42" \
  "$source_dir/erp42_control" \
  "$source_dir/erp42_sdf_files" \
  "$source_dir/odometry_localization" \
  --ignore-src --rosdistro humble -r -y

cd "$workspace"
colcon build --base-paths "$source_dir" --packages-up-to erp42 \
  --event-handlers console_direct+

echo "Built ERP42. Source $workspace/install/setup.bash to run the simulator."
