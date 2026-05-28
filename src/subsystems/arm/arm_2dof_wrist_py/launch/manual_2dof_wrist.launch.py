import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('arm_2dof_wrist_py'),
        'config', 'manual_2dof_wrist_node.yaml',
    )

    return LaunchDescription([
        Node(
            package='arm_2dof_wrist_py',
            executable='manual_2dof_wrist_node',
            name='manual_2dof_wrist_node',
            output='screen',
            parameters=[params],
            respawn=True,
            respawn_delay=2.0,
        )
    ])
