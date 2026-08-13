import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Joy node
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{"device_id": 0, "devicename": "/dev/input/js0"}],
        output="screen"
    )

    # Our custom Cartesian teleop node
    cartesian_teleop_node = Node(
        package="panda_mjk",
        executable="cartesian_teleop",
        name="cartesian_teleop",
        output="screen",
    )
    
    # In a full MoveIt Servo deployment, we'd also launch the servo_node here,
    # but that requires a full MoveIt config package which we might assume is generated or external.
    # We will just launch the teleop node assuming Servo is running or a custom IK controller is listening.
    
    servo_yaml = PathJoinSubstitution([
        FindPackageShare("panda_mjk"),
        "config",
        "servo_parameters.yaml"
    ])
    
    # NOTE: To actually run moveit_servo, you need a MoveIt Config package for the Panda (FER).
    # Assuming one exists or we mock it:
    # servo_node = Node(
    #     package="moveit_servo",
    #     executable="servo_node",
    #     parameters=[servo_yaml, moveit_config.to_dict()],
    #     output="screen"
    # )

    return LaunchDescription([
        joy_node,
        cartesian_teleop_node,
        # servo_node
    ])
