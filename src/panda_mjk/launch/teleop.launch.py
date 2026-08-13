from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # MoveIt Servo -- consumes the TwistStamped commands cartesian_teleop
    # publishes on /servo_node/delta_twist_cmds and drives the robot.
    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("panda_mjk"), "launch", "servo.launch.py"])
        )
    )

    return LaunchDescription([
        joy_node,
        cartesian_teleop_node,
        servo_launch,
    ])
