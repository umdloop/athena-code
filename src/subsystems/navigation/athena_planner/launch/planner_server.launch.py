from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = 'your_package_name'  # REPLACE with your actual package name
    planner_yaml = os.path.join(
        get_package_share_directory(pkg),
        'config',
        'planner_server.yaml'
    )

    return LaunchDescription([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]
        )
    ])
