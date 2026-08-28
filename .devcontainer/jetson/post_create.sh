#!/usr/bin/env bash
set -euo pipefail

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

sudo apt-get update
sudo rosdep update
sudo rosdep install -r --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y

colcon build --symlink-install
