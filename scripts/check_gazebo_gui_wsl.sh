#!/usr/bin/env bash
# Non-mutating checks for the ERP42 Gazebo GUI environment.
set -euo pipefail

workspace="${ERP42_WS:-/root/erp42_ws}"
echo "Distribution: ${WSL_DISTRO_NAME:-unknown}"
echo "Display: ${DISPLAY:-unset}; Wayland: ${WAYLAND_DISPLAY:-unset}"
[[ -n "${DISPLAY:-}" ]] || { echo 'ERROR: WSLg DISPLAY is missing'; exit 1; }

if command -v glxinfo >/dev/null; then
  glxinfo -B | grep -E 'direct rendering:|OpenGL renderer string:|OpenGL version string:'
else
  echo 'glxinfo is unavailable; install mesa-utils for OpenGL diagnostics.'
fi

set +u
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
set -u
echo "Gazebo: $(gazebo --version | head -n 1)"
echo "ERP42 ROS 2 package: $(ros2 pkg prefix erp42)"
echo "RViz2: $(command -v rviz2)"

if pgrep -x gzserver >/dev/null; then
  echo 'Gazebo server: running'
  echo 'ROS topics:'
  timeout 10s ros2 topic list || echo 'ROS 2 graph query timed out'
else
  echo 'Gazebo server: stopped (run start_gazebo_gui.sh to start it)'
fi
