#!/usr/bin/env bash
set -euo pipefail

mode="${1:-effort}"
if [[ "$mode" != "effort" && "$mode" != "skid" ]]; then
  echo "Usage: $0 [effort|skid]" >&2
  exit 2
fi

# ROS 2's setup scripts are not compatible with Bash nounset.
set +u
source /opt/ros/humble/setup.bash
source "${ERP42_WS:-$HOME/erp42_ws}/install/setup.bash"
set -u
source_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
log_file="/tmp/erp42_${mode}_gazebo.log"
export LIBGL_ALWAYS_SOFTWARE=1
export GAZEBO_MODEL_DATABASE_URI=""

# A dedicated process group lets us clean up Gazebo and all launch children.
setsid xvfb-run -a ros2 launch erp42 erp42.launch.py \
  gui:=false localization:=true "drive_mode:=$mode" >"$log_file" 2>&1 &
launch_pid=$!

cleanup() {
  # ros2 launch manages gzserver as a child process. Give launch the
  # opportunity to stop all its children before terminating xvfb-run.
  local ros_pid
  ros_pid="$(pgrep -P "$launch_pid" -f 'ros2 launch erp42' | head -n 1 || true)"
  if [[ -n "$ros_pid" ]]; then
    kill -INT "$ros_pid" 2>/dev/null || true
    for _ in {1..40}; do
      kill -0 "$ros_pid" 2>/dev/null || break
      sleep 0.25
    done
  fi
  kill -TERM -- "-$launch_pid" 2>/dev/null || true
  # Gazebo Classic sometimes survives SIGTERM inside WSL even after the
  # ros2 launch process exits. The dedicated setsid group is private to
  # this test, so force-stop only processes still inside that group.
  sleep 1
  kill -KILL -- "-$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT

echo "Gazebo log: $log_file"
python3 "$source_dir/scripts/smoke_humble.py" --drive-mode "$mode" \
  --localization --ready-timeout 100
