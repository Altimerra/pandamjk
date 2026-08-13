# PandaMJK ROS 2 Package Implementation Plan

This plan details the creation of the `PandaMJK` ROS 2 package. The package provides a unified architecture for controlling a Franka Emika FR3 robot, allowing seamless switching between a high-fidelity MuJoCo physics simulation and physical hardware using `ros2_control` and `moveit_servo`.

## User Review Required

> [!IMPORTANT]
> Please review this updated, highly detailed plan based on the latest ROS 2 Humble research. Note specifically the use of **Host Networking** for Docker, the transition to the **FR3** robot model (as 'panda' is deprecated in `franka_ros2`), and the use of **MoveIt Servo** for real-time Cartesian teleoperation. Is this acceptable?

## Proposed Architecture & Changes

### 1. Docker Environment (ROS 2 Humble + MuJoCo GUI)

We will create a custom Docker environment based on `osrf/ros:humble-desktop` to support MuJoCo's GLFW rendering, joystick passthrough, and the strict 1 kHz UDP requirements of the Franka Control Interface (FCI).

#### [NEW] [Dockerfile](file:///Users/harindu/Workbench/Work/PandaMJK/Dockerfile)
*   **Base:** `osrf/ros:humble-desktop`
*   **Apt Packages:** `ros-humble-mujoco-ros2-control`, `ros-humble-moveit-servo`, `ros-humble-ros2-controllers`, `libglfw3-dev`, `libgl1-mesa-dri`, joystick utilities.
*   **Source Builds:** `franka_ros2` and `libfranka` must be built from source (as they are not in the standard Humble apt repos).
*   **Env Vars:** `MUJOCO_GL=glfw`, `QT_X11_NO_MITSHM=1` for rendering.

#### [NEW] [docker-compose.yml](file:///Users/harindu/Workbench/Work/PandaMJK/docker-compose.yml)
*   **Networking:** `--net=host` is **mandatory** for the 1ms FCI loop and ROS 2 FastDDS multicast discovery.
*   **Privileges:** Requires `privileged: true`, `cap_add: [SYS_NICE]`, and `ulimits: rtprio: 99` for real-time thread scheduling.
*   **Devices:** Mounts `/dev/input:/dev/input` for the joystick, `/tmp/.X11-unix` for X11 forwarding, and GPU passthrough (NVIDIA or DRI).

---

### 2. Workspace & Package Structure

#### [NEW] [package.xml](file:///Users/harindu/Workbench/Work/PandaMJK/package.xml)
Dependencies: `rclcpp`, `controller_manager`, `ros2_control`, `mujoco_ros2_control`, `franka_hardware`, `franka_description`, `joy`, `moveit_servo`, `geometry_msgs`, `xacro`.

#### [NEW] [CMakeLists.txt](file:///Users/harindu/Workbench/Work/PandaMJK/CMakeLists.txt)
Installs launch files, config files, and the custom teleop node.

---

### 3. Robot Description (URDF / Xacro)

> [!WARNING]
> **Joint Naming:** The legacy `panda` model is deprecated in Humble `franka_ros2`. We must use the `fr3` model. To reconcile with DeepMind's `mujoco_menagerie` (which uses un-prefixed names like `joint1`), we will pass `no_prefix:=true` to the `franka_description` macros so both sim and real hardware expect `joint1` through `joint7`.

#### [NEW] [urdf/fr3.urdf.xacro](file:///Users/harindu/Workbench/Work/PandaMJK/urdf/fr3.urdf.xacro)
*   Includes `franka_description` with `no_prefix:=true`.
*   `<ros2_control>` block dynamically selects `mujoco_ros2_control/MujocoSystemInterface` (sim) or `franka_hardware/FrankaHardwareInterface` (real) based on `use_sim`.

#### [NEW] [urdf/panda.xml](file:///Users/harindu/Workbench/Work/PandaMJK/urdf/panda.xml)
*   The `mujoco_menagerie` Franka Panda MJCF model.

---

### 4. Control & Servoing Configuration

#### [NEW] [config/controllers.yaml](file:///Users/harindu/Workbench/Work/PandaMJK/config/controllers.yaml)
*   `joint_state_broadcaster`
*   `forward_position_controller` (for low-latency streaming from MoveIt Servo)
*   `position_controllers/GripperActionController` (Simulation only; real hardware uses `franka_gripper` node directly).

#### [NEW] [config/mujoco_pids.yaml](file:///Users/harindu/Workbench/Work/PandaMJK/config/mujoco_pids.yaml)
*   PID gains for `mujoco_ros2_control` to map position commands to MuJoCo effort actuators.

#### [NEW] [config/servo_parameters.yaml](file:///Users/harindu/Workbench/Work/PandaMJK/config/servo_parameters.yaml)
*   Configures MoveIt Servo for Jacobian-based real-time IK, singularity avoidance (velocity scaling), and collision checking. Outputs to `/forward_position_controller/commands`.

---

### 5. Custom Cartesian Teleop Node

#### [NEW] [src/cartesian_teleop.cpp](file:///Users/harindu/Workbench/Work/PandaMJK/src/cartesian_teleop.cpp)
Subscribes to `/joy` and publishes `geometry_msgs/TwistStamped` to MoveIt Servo (`/servo_node/delta_twist_cmds`) using the symmetric, "hold-to-reverse" scheme:

*   **Left Hand (Translation - X, Y, Z):**
    *   Left Stick (Up/Down): Translate Forward/Backward (X-axis)
    *   Left Stick (Left/Right): Translate Left/Right (Y-axis)
    *   Left Trigger (LT): Translate UP (Throttle - Analog speed)
    *   Left Bumper (LB) + Left Trigger (LT): Translate DOWN (Hold LB to reverse LT direction)
*   **Right Hand (Orientation - Pitch, Yaw, Roll):**
    *   Right Stick (Up/Down): Pitch (Tilt gripper up/down)
    *   Right Stick (Left/Right): Yaw (Pan gripper left/right)
    *   Right Trigger (RT): Roll Clockwise (Throttle - Analog speed)
    *   Right Bumper (RB) + Right Trigger (RT): Roll Counter-Clockwise (Hold RB to reverse RT direction)
*   **Face Buttons (Gripper & Null-Space):**
    *   A / Cross: Close Gripper (Sends `control_msgs/GripperCommand` action)
    *   Y / Triangle: Open Gripper
    *   B / Circle: Soft Brake (Publishes zero twist)
    *   X / Square + Right Stick (L/R): Elbow Clutch (Optionally maps to `delta_joint_cmds` for joint 7)

---

### 6. Launch Files

#### [NEW] [launch/fr3_control.launch.py](file:///Users/harindu/Workbench/Work/PandaMJK/launch/fr3_control.launch.py)
*   Arguments: `use_sim` (default: `true`), `robot_ip`.
*   Logic: Launches `mujoco_ros2_control/ros2_control_node` OR standard `controller_manager`, spawns controllers, and launches MoveIt Servo.

#### [NEW] [launch/teleop.launch.py](file:///Users/harindu/Workbench/Work/PandaMJK/launch/teleop.launch.py)
*   Launches `joy_node` and our custom `cartesian_teleop` node.

## Verification Plan

### Automated Tests
*   Build the Docker container (which clones and builds `franka_ros2` and `libfranka`).
*   Run `colcon build` to ensure CMake and package configurations are correct.

### Manual Verification
*   **Simulation Mode:** Run the Docker container, launch `fr3_control.launch.py use_sim:=true` and `teleop.launch.py`. Verify planar control mapping and MoveIt Servo's singularity avoidance in MuJoCo.
*   **Real Hardware Mode:** Connect container to Franka network, launch with `use_sim:=false robot_ip:=<ACTUAL_IP>`, verify FCI real-time connection.
