#!/bin/bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

ros2 topic echo /forward_position_controller/commands > /tmp/commands.log &
PID1=$!

ros2 topic echo /servo_node/status > /tmp/status.log &
PID2=$!

python3 /ros2_ws/src/test_twist.py --ros-args -p use_sim_time:=true

kill $PID1 $PID2
