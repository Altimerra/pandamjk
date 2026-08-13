#!/bin/bash
set -e

# Source ROS 2 base setup
source "/opt/ros/humble/setup.bash"

# The dev bind mount (docker-compose ./src -> /ros2_ws/src) shadows any
# submodule init done at image build time. Re-init here so a stale host
# checkout (e.g. libfranka -> libfranka-common) doesn't break rebuilds.
git config --global --add safe.directory '*' 2>/dev/null || true
find /ros2_ws/src -maxdepth 2 -name ".gitmodules" -execdir git submodule update --init --recursive \; 2>/dev/null || true

# Source workspace setup if built
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source "/ros2_ws/install/setup.bash"
fi

exec "$@"
