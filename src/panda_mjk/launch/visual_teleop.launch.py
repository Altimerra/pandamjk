from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('panda_mjk')

    # Path to rviz config
    rviz_config = os.path.join(pkg_dir, 'config', 'teleop.rviz')

    # MoveIt Servo -- consumes the TwistStamped commands servo_interactive_marker
    # publishes on /servo_node/delta_twist_cmds and drives the robot.
    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("panda_mjk"), "launch", "servo.launch.py"])
        )
    )

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
        Node(
            package='panda_mjk',
            executable='servo_interactive_marker',
            name='servo_interactive_marker',
            output='screen',
            parameters=[{
                'base_frame': 'fer_link0',
                'ee_frame': 'fer_link8',
                'kp_linear': 5.0,
                'kp_angular': 2.0,
                'use_sim_time': True,
            }]
        ),
        servo_launch,
    ])
