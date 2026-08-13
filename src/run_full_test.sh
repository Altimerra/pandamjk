#!/bin/bash
export MUJOCO_GL=glfw
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "Starting panda_control..."
xvfb-run -a ros2 launch panda_mjk panda_control.launch.py use_sim:=true > /tmp/panda_control.log 2>&1 &
PANDA_PID=$!

sleep 8

echo "Starting moveit_servo..."
xvfb-run -a ros2 launch panda_mjk servo.launch.py use_sim_time:=true > /tmp/servo.log 2>&1 &
SERVO_PID=$!

sleep 8

echo "Starting topic recording..."
ros2 topic echo /forward_position_controller/commands > /tmp/commands.log &
ECHO_PID1=$!

ros2 topic echo /servo_node/status > /tmp/status.log &
ECHO_PID2=$!

echo "Running Twist test..."
python3 /ros2_ws/src/test_twist.py --ros-args -p use_sim_time:=true

echo "Cleaning up..."
kill $ECHO_PID1 $ECHO_PID2
kill -INT $SERVO_PID
kill -INT $PANDA_PID
sleep 2

echo "=== TEST RESULTS ==="
echo "Servo Errors (Status codes other than 0):"
cat /tmp/status.log | grep -v 'data: 0' | sort | uniq -c

echo "Command changes during movement:"
cat /tmp/commands.log | grep -A 1 'data:' | grep -v 'data:' | grep -v -- '--' | uniq | wc -l
echo "If the command count is > 1, it means the joint positions were successfully updating!"
