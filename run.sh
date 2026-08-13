#!/bin/bash
# Convenience launcher for the PandaMJK Docker stack.
#
# Usage:
#   ./run.sh                          # simulation, no teleop
#   ./run.sh --teleop                 # simulation + joystick Cartesian teleop
#   ./run.sh --visual-teleop          # simulation + RViz interactive-marker teleop
#   ./run.sh --real --robot-ip 172.16.0.2   # real Panda hardware
#   ./run.sh --build                  # rebuild the image first
#
# Also starts the web dashboard (panda_dashboard/) on the host. Ctrl+C
# stops the foreground launch, then the container and the dashboard
# are both shut down together.
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

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Allow the container's root user to connect to the host X server.
xhost +local:docker >/dev/null 2>&1 || true

# docker-compose bind-mounts ./src over the image's /ros2_ws/src for live
# dev editing, which shadows any submodule init done at image build time
# (e.g. libfranka -> libfranka-common). Keep host checkouts self-healing.
find "$SCRIPT_DIR/src" -maxdepth 2 -name ".gitmodules" -execdir git submodule update --init --recursive \; 2>/dev/null || true

docker compose up -d $BUILD_FLAG

CONTAINER="ros2_mujoco_container"
ROS_ENV="source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash"

DASHBOARD_PID=""
cleanup() {
  echo
  echo "Shutting down..."
  if [[ -n "$DASHBOARD_PID" ]] && kill -0 "$DASHBOARD_PID" 2>/dev/null; then
    kill "$DASHBOARD_PID" 2>/dev/null || true
  fi
  docker compose stop || true
}
trap cleanup EXIT

echo "Starting web dashboard..."
uv run "$SCRIPT_DIR/run_dashboard.py" &
DASHBOARD_PID=$!

if [[ -n "$TELEOP" ]]; then
  echo "Starting teleop ($TELEOP) in the background..."
  docker exec -d "$CONTAINER" bash -c "$ROS_ENV && ros2 launch panda_mjk ${TELEOP}.launch.py"
fi

echo "Starting panda_control.launch.py (use_sim:=$USE_SIM robot_ip:=$ROBOT_IP)..."
docker exec -it "$CONTAINER" bash -c \
  "$ROS_ENV && ros2 launch panda_mjk panda_control.launch.py use_sim:=$USE_SIM robot_ip:=$ROBOT_IP"
