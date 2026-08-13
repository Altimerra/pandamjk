# PandaMJK

**PandaMJK** is a unified ROS 2 Humble package for controlling a Franka Emika Panda arm. It provides a seamless architecture that allows you to switch between a high-fidelity MuJoCo physics simulation and real physical hardware using `ros2_control` and `moveit_servo`.

## Features
- **Unified Interface**: Use the exact same ROS 2 controllers, planners, and teleoperation scripts for both simulated and real hardware.
- **MuJoCo Physics**: Out-of-the-box integration with `mujoco_ros2_control`.
- **Headless & GUI Rendering**: Configured with Docker and OSMesa for fully headless rendering (`xvfb`) or GUI rendering (`glfw`).
- **ROS Bridge Integration**: Contains an integrated `rosbridge_server` exposing the control stack to external WebSocket clients on port 9090.
- **Teleoperation**: Uses MoveIt Servo to translate Cartesian twist commands from a joystick or RViz interactive marker into smooth joint space control.

## Architecture

```text
                  (drag + release marker)            (Cartesian twist)
   RViz Interactive Marker ──► servo_interactive_marker ──► /servo_node/delta_twist_cmds
                                 (P-controller on TF error,      │
                                  emits TwistStamped in link0)   ▼
                                                          MoveIt Servo (servo_node)
                                                          - IK / Jacobian
                                                          - command_in_type: speed_units
                                                          - outputs joint POSITIONS
                                                                 │
                                               std_msgs/Float64MultiArray
                                                                 ▼
                                        /forward_position_controller/commands
                                                                 │
                                           forward_command_controller/ForwardCommandController
                                                      (interface_name: position)
                                                                 ▼
                                     ros2_control  ──►  MuJoCo position actuators  (or real FCI)
```

## Setup & Running

This project uses Docker to provide a strict real-time environment (1 kHz UDP for FCI) and properly isolated dependencies (`ros-humble-mujoco-ros2-control`, `osrf/ros:humble-desktop`, etc.).

### 1. Build & Run the Container
A convenience script is provided to automatically build and launch the Docker container using `docker compose`.

```bash
./run.sh
```

By default, this will spin up the `ros2_mujoco_container`. You can run ROS 2 commands directly inside this container:

```bash
docker exec -it ros2_mujoco_container bash
```

### 2. Launch the Control Stack
Inside the container (or by using `docker exec`), launch the control stack.

**For Simulation (MuJoCo):**
```bash
ros2 launch panda_mjk panda_control.launch.py use_sim:=true
```
*(Note: If running headlessly without a display, you must prefix this with `xvfb-run -a` to provide a virtual framebuffer to the MuJoCo OpenGL context).*

**For Real Hardware:**
```bash
ros2 launch panda_mjk panda_control.launch.py use_sim:=false robot_ip:=<ACTUAL_IP>
```

### 3. Launch Teleoperation (MoveIt Servo)
MoveIt Servo must be launched to provide real-time IK control.

**With Joystick:**
```bash
ros2 launch panda_mjk teleop.launch.py
```

**With RViz Interactive Marker:**
```bash
ros2 launch panda_mjk visual_teleop.launch.py
```

### Headless Access via ROS Bridge
The `panda_control.launch.py` script automatically brings up a `rosbridge_websocket` server on port 9090. Since the Docker container uses host networking (`network_mode: host`), this port is directly available on your host machine.

You can use the provided Python scripts from your host machine to interact with the robot:
- `uv run monitor_ros.py` - Monitors the live joint states over WebSockets.
- `uv run control_ros.py` - A headless script demonstrating absolute position control by reading live TF and sending a command over WebSockets.

### Panda Dashboard (Custom Monitor & Control UI)
Unlike Vizanti (a generic ROS visualizer), `panda_dashboard/` is a small purpose-built page for this arm: a live joint-state table and MoveIt Servo status readout, plus controls to jog individual joints, jump to a home pose, jog the end effector in Cartesian space (`/servo_node/delta_twist_cmds`), and open/close the gripper. It's a static HTML/JS page (vendors `roslib.js`, no build step) that talks directly to rosbridge from the browser, so it needs no ROS 2 install on the host either:

```bash
uv run run_dashboard.py
```

Then open `http://localhost:8080` and click **Connect** (defaults to `localhost:9090`). Override the port with `PANDA_DASHBOARD_PORT=<port>` if 8080 is taken. Joint jog sends absolute positions to `/forward_position_controller/commands`; Cartesian jog streams a `TwistStamped` while a button is held and zeroes it on release, matching the same command path the interactive-marker teleop uses.

## Known Limitations
* **MoveIt Servo Absolute Pose:** MoveIt Servo in ROS 2 Humble has a known issue where it does not reliably drive the arm via absolute target poses (`pose_target_cmds`). To bypass this, the teleoperation pipeline computes a proportional velocity (Twist) towards the target and streams it to `/servo_node/delta_twist_cmds`.
