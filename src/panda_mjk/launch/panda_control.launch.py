import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Start in MuJoCo simulation (true) or real hardware (false).",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.100",
            description="IP address of the physical Franka robot.",
        ),
        DeclareLaunchArgument(
            "start_rosbridge",
            default_value="true",
            description="Start the rosbridge websocket server (true/false).",
        ),
        DeclareLaunchArgument(
            "controller",
            default_value="forward_position_controller",
            description="Which main controller to spawn (e.g., forward_position_controller, joint_impedance_example_controller).",
        ),
    ]

    use_sim = LaunchConfiguration("use_sim")
    robot_ip = LaunchConfiguration("robot_ip")
    start_rosbridge = LaunchConfiguration("start_rosbridge")
    controller_name = LaunchConfiguration("controller")

    # Generate robot_description via Xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("panda_mjk"), "urdf", "panda.urdf.xacro"]),
        " ",
        "use_sim:=", use_sim,
        " ",
        "robot_ip:=", robot_ip,
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    controllers_config = PathJoinSubstitution([
        FindPackageShare("panda_mjk"),
        "config",
        "controllers.yaml",
    ])

    mujoco_pids = PathJoinSubstitution([
        FindPackageShare("panda_mjk"),
        "config",
        "mujoco_pids.yaml",
    ])

    # Standard Controller Manager for Real Hardware
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_config],
        output="both",
        condition=UnlessCondition(use_sim),
    )

    # Dedicated Controller Manager Node for MuJoCo Simulation
    mujoco_control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_config, mujoco_pids, {"use_sim_time": True}],
        output="both",
        condition=IfCondition(use_sim),
    )

    # Controller Spawners
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    main_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller_name, "--controller-manager", "/controller_manager"],
    )

    # Gripper controller for sim only
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_gripper_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(use_sim),
    )

    # For real hardware, you'd typically launch the franka_gripper node here or via another launch file.

    # Robot State Publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Rosbridge WebSocket Server
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rosbridge_server"),
                "launch",
                "rosbridge_websocket_launch.xml"
            ])
        ),
        condition=IfCondition(start_rosbridge)
    )

    # rosapi provides topic/service/type introspection over the rosbridge
    # websocket. rosbridge_websocket_launch.xml does not bring it up on
    # its own, but web clients need it to list/add topics.
    rosapi_node = Node(
        package="rosapi",
        executable="rosapi_node",
        output="both",
        condition=IfCondition(start_rosbridge),
    )

    return LaunchDescription(
        declared_arguments + [
            rosbridge_launch,
            rosapi_node,
            ros2_control_node,
            mujoco_control_node,
            rsp_node,
            jsb_spawner,
            main_controller_spawner,
            gripper_spawner,
        ]
    )
