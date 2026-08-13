#!/bin/bash
# Convenience launcher for the PandaMJK Docker stack.
#
# Usage:
#   ./run.sh                          # simulation, no teleop
#   ./run.sh --teleop                 # simulation + joystick Cartesian teleop
#   ./run.sh --visual-teleop          # simulation + RViz interactive-marker teleop
#   ./run.sh --real --robot-ip 172.16.0.2   # real Panda hardware
#   ./run.sh --build                  # rebuild the image first
set -e

USE_SIM=true
ROBOT_IP="192.168.1.100"
BUILD_FLAG=""
TELEOP=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --real) USE_SIM=false; shift ;;
    --robot-ip) ROBOT_IP="$2"; shift 2 ;;
    --build) BUILD_FLAG="--build"; shift ;;
    --teleop) TELEOP="teleop"; shift ;;
    --visual-teleop) TELEOP="visual_teleop"; shift ;;
    -h|--help)
      grep '^#' "$0" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
done

# Allow the container's root user to connect to the host X server.
xhost +local:docker >/dev/null 2>&1 || true

# docker-compose bind-mounts ./src over the image's /ros2_ws/src for live
# dev editing, which shadows any submodule init done at image build time
# (e.g. libfranka -> libfranka-common). Keep host checkouts self-healing.
find "$(dirname "$0")/src" -maxdepth 2 -name ".gitmodules" -execdir git submodule update --init --recursive \; 2>/dev/null || true

docker compose up -d $BUILD_FLAG

CONTAINER="ros2_mujoco_container"
ROS_ENV="source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash"

if [[ -n "$TELEOP" ]]; then
  echo "Starting teleop ($TELEOP) in the background..."
  docker exec -d "$CONTAINER" bash -c "$ROS_ENV && ros2 launch panda_mjk ${TELEOP}.launch.py"
fi

echo "Starting panda_control.launch.py (use_sim:=$USE_SIM robot_ip:=$ROBOT_IP)..."
docker exec -it "$CONTAINER" bash -c \
  "$ROS_ENV && ros2 launch panda_mjk panda_control.launch.py use_sim:=$USE_SIM robot_ip:=$ROBOT_IP"
