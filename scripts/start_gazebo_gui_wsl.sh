#!/usr/bin/env bash
# Starts the ERP42 ROS 2 simulation with the Gazebo Classic GUI through WSLg.
set -euo pipefail

workspace="${ERP42_WS:-/root/erp42_ws}"
drive_mode=effort
rviz=false
localization=true
software_gl=false
idle_hold=true

usage() {
  cat <<'HELP'
Usage: bash /root/erp42_ws/start_gazebo_gui.sh [options]
  --skid          Use /cmd_vel skid-steer mode (default: wheel effort/steering)
  --effort        Use wheel effort/steering control (default)
  --rviz          Also start RViz2 with the ERP42 configuration
  --no-ekf        Disable the odometry localization/EKF nodes
  --software-gl   Render with Mesa llvmpipe when WSLg GPU rendering fails
  --no-idle-hold  Disable idle anti-creep torque feedback in effort mode
  --help          Show this help

Launch with WSL2 Ubuntu-22.04; a Windows desktop WSLg session is needed.
HELP
}

for arg in "$@"; do
  case "$arg" in
    --skid) drive_mode=skid ;;
    --effort) drive_mode=effort ;;
    --rviz) rviz=true ;;
    --no-ekf) localization=false ;;
    --software-gl) software_gl=true ;;
    --no-idle-hold) idle_hold=false ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown option: %s\n' "$arg" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ ! -r /opt/ros/humble/setup.bash || ! -r "$workspace/install/setup.bash" ]]; then
  echo 'ROS 2 Humble or the ERP42 colcon install was not found.' >&2
  echo "Expected: $workspace/install/setup.bash" >&2
  exit 1
fi

if [[ -z "${DISPLAY:-}" ]]; then
  echo 'DISPLAY is unset. Start this through WSL2 with WSLg enabled.' >&2
  exit 1
fi

if pgrep -x gzserver >/dev/null; then
  echo 'An existing gzserver is running. Stop that simulation before launching another.' >&2
  exit 1
fi

# ROS setup scripts use optional variables and are not safe with Bash nounset.
set +u
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
set -u

export GAZEBO_MODEL_DATABASE_URI=''
export QT_X11_NO_MITSHM=1
if [[ "$software_gl" == true ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  echo 'Renderer: Mesa software (llvmpipe)'
else
  unset LIBGL_ALWAYS_SOFTWARE
  echo 'Renderer: WSLg OpenGL (GPU when supported)'
fi

echo "Workspace: $workspace"
echo "Gazebo GUI: enabled; drive_mode: $drive_mode; RViz2: $rviz; EKF: $localization; idle_hold: $idle_hold"
echo 'Press Ctrl+C in this terminal to stop the simulation.'
exec ros2 launch erp42 erp42.launch.py \
  gui:=true "rviz:=$rviz" "localization:=$localization" "drive_mode:=$drive_mode" "idle_hold:=$idle_hold"
