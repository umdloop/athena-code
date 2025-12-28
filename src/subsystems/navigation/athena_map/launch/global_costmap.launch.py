#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('athena_map')  # <-- replace with your package name

    global_costmap_yaml = os.path.join(pkg_share, 'config', 'global_costmap.yaml')
    common_costmap_yaml = os.path.join(pkg_share, 'config', 'costmap_common.yaml')

    return LaunchDescription([
        Node(
            package='athena_map',
            executable='costmap_2d_node',
            name='global_costmap',
            output='screen',
            # make sure these files exist under your config/ directory
            parameters=[
                global_costmap_yaml,
            ],
        ),
    ])
