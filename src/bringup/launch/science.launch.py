import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


# Drive owns the default /controller_manager. Science runs in /science to avoid the collision.
_SCIENCE_NS = "science"
_SCIENCE_CM = f"/{_SCIENCE_NS}/controller_manager"


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='standalone',
            choices=['standalone', 'jetson', 'base_station'],
            description=(
                "'jetson': hardware nodes on the rover. "
                "'base_station': operator teleop nodes. "
                "'standalone': all nodes on one machine."
            ),
        ),
        DeclareLaunchArgument('use_sim', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_mock_hardware', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('mock_sensor_commands', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'robot_controller',
            default_value='rear_ackermann_controller',
            choices=['front_ackermann_controller', 'ackermann_steering_controller', 'rear_ackermann_controller'],
        ),
        DeclareLaunchArgument('deactivate_talon', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('can_interface', default_value='can1'),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    use_sim = LaunchConfiguration('use_sim').perform(context)
    use_mock_hardware = LaunchConfiguration('use_mock_hardware').perform(context)
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands').perform(context)
    robot_controller = LaunchConfiguration('robot_controller').perform(context)
    deactivate_talon = LaunchConfiguration('deactivate_talon').perform(context)
    can_interface = LaunchConfiguration('can_interface').perform(context)

    drive_share = get_package_share_directory('drive_bringup')
    science_bringup_share = get_package_share_directory('science_bringup')
    description_share = get_package_share_directory('description')
    athena_gps_share = get_package_share_directory('athena_gps')
    mag_heading_share = get_package_share_directory('mag_heading')
    led_indicator_share = get_package_share_directory('led_indicator')

    # ------------------------------------------------------------------
    # Drive: unmodified subsystem launch, root namespace.
    # ------------------------------------------------------------------
    drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(drive_share, 'launch', 'athena_drive.launch.py')),
        launch_arguments={
            'mode': mode,
            'use_sim': use_sim,
            'use_mock_hardware': use_mock_hardware,
            'mock_sensor_commands': mock_sensor_commands,
            'robot_controller': robot_controller,
            'can_interface': can_interface,
        }.items(),
    )

    # ------------------------------------------------------------------
    # GPS + heading + LED indicator: unmodified, root namespace.
    # ------------------------------------------------------------------
    gps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(athena_gps_share, 'launch', 'gps_launch.py')),
        launch_arguments={'sim': use_sim}.items(),
    )
    heading = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mag_heading_share, 'launch', 'mag_heading.launch.py')),
    )
    led = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(led_indicator_share, 'launch', 'led_indicator.launch.py')),
    )

    # ------------------------------------------------------------------
    # Science: inlined, namespaced under /science so its CM/topics don't clash with drive.
    # The science subsystem files (URDF, controllers YAML, joystick YAML) are reused as-is.
    # ------------------------------------------------------------------
    science_urdf_xacro = os.path.join(description_share, 'urdf', 'science', 'athena_science.urdf.xacro')
    science_controllers_yaml = os.path.join(science_bringup_share, 'config', 'athena_science_controllers.yaml')
    science_joystick_yaml = os.path.join(science_bringup_share, 'config', 'joystick.yaml')

    science_urdf = Command([
        FindExecutable(name='xacro'), ' ',
        science_urdf_xacro,
        ' prefix:=""',
        ' use_mock_hardware:=', use_mock_hardware,
        ' mock_sensor_commands:=', mock_sensor_commands,
        ' deactivate_talon:=', deactivate_talon, ' ',
        ' can_interface:=', can_interface, ' ',
    ])

    active_controllers = [
        'science_controller',
        'laser_gpio_controller',
        'fluoro_led_gpio_controller',
    ]
    inactive_controllers = [
        'joint_group_velocity_controller',
        'joint_group_position_controller',
    ]

    science_cm = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=_SCIENCE_NS,
        output='both',
        parameters=[science_controllers_yaml],
        remappings=[('~/robot_description', 'robot_description')],
    )

    science_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=_SCIENCE_NS,
        output='both',
        parameters=[{'robot_description': science_urdf}],
    )

    def spawner(name, inactive=False):
        args = [name, '-c', _SCIENCE_CM, '--param-file', science_controllers_yaml]
        if inactive:
            args.append('--inactive')
        return Node(
            package='controller_manager',
            executable='spawner',
            namespace=_SCIENCE_NS,
            arguments=args,
        )

    jsb = spawner('joint_state_broadcaster')
    motor_status = spawner('motor_status_controller')
    active_spawners = [spawner(c) for c in active_controllers]
    inactive_spawners = [spawner(c, inactive=True) for c in inactive_controllers]

    # Sequence: CM up → JSB → motor_status → active spawners (chained) → inactive spawners (chained).
    delay_jsb = RegisterEventHandler(OnProcessStart(
        target_action=science_cm,
        on_start=[TimerAction(period=3.0, actions=[jsb])],
    ))

    delay_motor_status = RegisterEventHandler(OnProcessExit(
        target_action=jsb,
        on_exit=[motor_status],
    ))

    chained = []
    prev = motor_status
    for sp in active_spawners + inactive_spawners:
        chained.append(RegisterEventHandler(OnProcessExit(target_action=prev, on_exit=[sp])))
        prev = sp

    science_hardware = GroupAction([
        science_cm,
        science_rsp,
        delay_jsb,
        delay_motor_status,
        *chained,
    ])

    science_joystick = Node(
        package='teleop',
        executable='joystick',
        namespace=_SCIENCE_NS,
        name='joystick',
        output='screen',
        parameters=[science_joystick_yaml],
        remappings=[
            ('controller_input', 'science_manual'),
            ('/controller_input', '/science_manual'),
        ],
    )

    if mode == 'jetson':
        return [drive, science_hardware, gps, heading, led]
    elif mode == 'base_station':
        return [drive, science_joystick]
    else:  # standalone
        return [drive, science_hardware, science_joystick, gps, heading, led]
