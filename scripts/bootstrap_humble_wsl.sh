#!/usr/bin/env bash
set -euo pipefail

# Run as root inside a fresh Ubuntu 22.04 WSL distribution.
if (( EUID != 0 )); then
  echo "Run this script as root (wsl -d Ubuntu-22.04 -u root -- bash scripts/bootstrap_humble_wsl.sh)" >&2
  exit 1
fi

source /etc/os-release
if [[ "$VERSION_ID" != "22.04" ]]; then
  echo "ROS 2 Humble debs require Ubuntu 22.04; found $PRETTY_NAME" >&2
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends ca-certificates curl gnupg locales software-properties-common
add-apt-repository -y universe

# The upstream ros2-apt-source package owns the ROS apt key and source list.
if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'install ok installed'; then
  source_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
  curl -fL --retry 3 -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${source_version}/ros2-apt-source_${source_version}.jammy_all.deb"
  dpkg -i /tmp/ros2-apt-source.deb
fi

apt-get update
apt-get install -y --no-install-recommends \
  build-essential cmake python3-colcon-common-extensions python3-rosdep \
  xvfb xauth \
  ros-humble-ros-base ros-humble-xacro ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control ros-humble-ros2-controllers \
  ros-humble-robot-localization ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui ros-humble-rviz2 ros-humble-rqt-plot

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi
rosdep update --rosdistro humble
echo "ROS 2 Humble and Gazebo Classic dependencies are installed."
