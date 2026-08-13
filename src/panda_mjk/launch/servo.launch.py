import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)


def generate_launch_description():
    # MuJoCo's ros2_control_node runs with use_sim_time:=true (see
    # panda_control.launch.py), so /joint_states carries sim-time stamps.
    # Match that here for consistency with the rest of the sim pipeline;
    # pass use_sim_time:=false for real hardware (no /clock source there).
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use simulation (MuJoCo) clock. Must match panda_control.launch.py's use_sim.",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    # robot_description (recomputed here, same as panda_control.launch.py --
    # MoveIt Servo needs it as a parameter on this node, not via topic).
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("panda_mjk"), "urdf", "panda.urdf.xacro"]),
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # robot_description_semantic (SRDF) -- fer.srdf.xacro is franka_description's
    # current identifier for the real Franka Emika Panda (see urdf/panda.urdf.xacro
    # for why this isn't fr3.srdf.xacro).
    #
    # no_prefix:=false (matching panda.urdf.xacro) so link names here agree with
    # the URDF's "fer_link0".."fer_link8"; MoveIt matches links between URDF and
    # SRDF by exact name. hand:=false: we don't need the "hand" group for
    # Cartesian arm servoing (the real gripper is driven separately via
    # franka_gripper, not through Servo), so just omit it from the SRDF.
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("franka_description"), "robots", "fer", "fer.srdf.xacro"]),
        " hand:=false no_prefix:=false",
    ])
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml("panda_mjk", "config/kinematics.yaml")
    }

    servo_yaml = PathJoinSubstitution([
        FindPackageShare("panda_mjk"), "config", "servo_parameters.yaml",
    ])

    # name="servo_node" so the "~/delta_twist_cmds" etc. topics in
    # servo_parameters.yaml resolve to /servo_node/delta_twist_cmds, matching
    # what cartesian_teleop.cpp and interactive_teleop.py publish to.
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            servo_yaml,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # MoveIt Servo starts paused and produces no output at all on
    # command_out_topic until this is called -- confirmed directly: every
    # manual test that skipped this had zero output despite valid, correctly
    # timestamped twist and joint_state input. `ros2 service call` blocks
    # until the service is available, so no explicit delay/wait is needed
    # here even though it starts alongside servo_node.
    start_servo = ExecuteProcess(
        cmd=["ros2", "service", "call", "/servo_node/start_servo", "std_srvs/srv/Trigger", "{}"],
        output="screen",
    )

    return LaunchDescription([use_sim_time_arg, servo_node, start_servo])
