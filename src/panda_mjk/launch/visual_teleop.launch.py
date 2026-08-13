from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('panda_mjk')
    
    # Path to rviz config
    rviz_config = os.path.join(pkg_dir, 'config', 'teleop.rviz')
    
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
            executable='interactive_teleop.py',
            name='interactive_teleop',
            output='screen',
            parameters=[{
                'base_frame': 'link0',
                'ee_frame': 'link8',
                'kp_linear': 5.0,
                'kp_angular': 2.0,
            }]
        )
    ])
