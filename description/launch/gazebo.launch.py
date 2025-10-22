from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['$(find gazebo_ros)/launch/empty_world.launch'])
        ),
        Node(
            package='tf',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint', '40']
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_model',
            name='spawn_model',
            arguments=['-file', '$(find athena_arm)/urdf/description.urdf.xacro', '-urdf', '-model', 'athena_arm'],
            output='screen'
        ),
        Node(
            package='rostopic',
            executable='rostopic',
            name='fake_joint_calibration',
            arguments=['pub', '/calibrated', 'std_msgs/Bool', 'true']
        )
    ])
