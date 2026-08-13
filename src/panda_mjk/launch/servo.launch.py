import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)


def generate_launch_description():
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
    # hand:=false -- franka_description's franka_robot.xacro (used by our
    # panda.urdf.xacro) doesn't forward no_prefix to its hand attachment, so
    # the real URDF's hand/finger links come out "fer_"-prefixed even with
    # no_prefix:=true, while fer.srdf.xacro's hand group does honor it and
    # comes out bare ("hand", "leftfinger", ...) -- a mismatch that makes
    # MoveIt fail to load those links. We don't need the "hand" group for
    # Cartesian arm servoing (the real gripper is driven separately via
    # franka_gripper, not through Servo), so just omit it from the SRDF.
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("franka_description"), "robots", "fer", "fer.srdf.xacro"]),
        " hand:=false no_prefix:=true",
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
        ],
        output="screen",
    )

    return LaunchDescription([servo_node])
