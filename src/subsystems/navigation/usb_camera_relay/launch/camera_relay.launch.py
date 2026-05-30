from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('usb_camera_relay'), 'config', 'camera_relay_params.yaml'
    )

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the camera relay params YAML',
        ),
        Node(
            package='usb_camera_relay',
            executable='camera_relay',
            name='usb_camera_relay',
            output='screen',
            parameters=[params_file],
        ),
    ])
