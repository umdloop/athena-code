# Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Source of this file are templates in
# [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
#
# Author: Dr. Denis
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Simulation mode: enables mock hardware and virtual CAN (vcan0).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="science_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="athena_science_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="athena_science.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states. "
                        "Automatically true when use_sim is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. "
                        "Automatically true when use_sim is true.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="science_controller",
            choices=["science_controller"],
            description="Robot controller to start.",
        )
    )

    # Initialize Arguments
    use_sim = LaunchConfiguration("use_sim")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    robot_controller = LaunchConfiguration("robot_controller")

    # When use_sim is true, force mock hardware and mock sensor commands on
    use_mock_hardware_effective = PythonExpression([
        "'true' if '", use_sim, "' == 'true' else '", use_mock_hardware, "'"
    ])
    mock_sensor_commands_effective = PythonExpression([
        "'true' if '", use_sim, "' == 'true' else '", mock_sensor_commands, "'"
    ])

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware_effective,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands_effective,
            " ",
            "use_sim:=",
            use_sim,
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "athena_science.rviz"]
    )

    joystick_config_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'config', 'joystick.yaml']
    )

    joystick_publisher = Node(
        package='teleop',
        executable='joystick',
        name='joystick',
        output='screen',
        parameters = [joystick_config_file],
        remappings=[
        ('controller_input', 'science_manual'),
        ('/controller_input', '/science_manual'),
    ],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description, robot_controllers],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    motor_status_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["motor_status_broadcaster", "-c", "/controller_manager"],
    )


    # Active Spawners
    robot_controller_names = ["science_controller"]
    robot_controller_spawners = [] 
    for controller in robot_controller_names:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
            )
        ]

    # GPIO controller spawner for Laser
    gpio_controller_names = ["laser_gpio_controller"]
    gpio_controller_spawners = []
    for controller in gpio_controller_names:
        gpio_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
            )
        ]

    inactive_robot_controller_names = ["joint_group_velocity_controller", "joint_group_position_controller"]
    inactive_robot_controller_spawners = []
    for controller in inactive_robot_controller_names:
        inactive_robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager", "--inactive"],
            )
        ]

    # Delay GPIO controller spawners after joint_state_broadcaster
    delay_gpio_controller_spawners_after_joint_state_broadcaster_spawner = []
    for i, controller in enumerate(gpio_controller_spawners):
        delay_gpio_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=(
                        gpio_controller_spawners[i - 1]
                        if i > 0
                        else joint_state_broadcaster_spawner
                    ),
                    on_exit=[controller],
                )
            )
        ]

    # Handle switching between controllers
    controller_switcher_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=inactive_robot_controller_spawners[-1],
            on_exit=[TimerAction(
                period=3.0,
                actions=[Node(
                    package="bringup",
                    executable="controller_switcher.py",
                    name="controller_switcher",
                    output="screen"
                )]
            )],
        )
    )

    # Delay motor_status_broadcaster (inactive) after joint_state_broadcaster
    delay_motor_status_broadcaster_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[motor_status_broadcaster_spawner],
        )
    )

    # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
    delay_joint_state_broadcaster_spawner_after_ros2_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[joint_state_broadcaster_spawner],
                ),
            ],
        )
    )

    umdloop_can_node = Node(
            package='umdloop_can',
            executable='can_node',
            name='can_node',
            output='log',
            arguments=['--ros-args', '--log-level', 'fatal']
    )

    # -- CAN Setup (driven by use_sim) --
    can_setup_sim = ExecuteProcess(
        cmd=['bash', '-c',
             'sudo modprobe vcan || true; '
             'sudo ip link add dev vcan0 type vcan 2>/dev/null || true; '
             'sudo ip link set up vcan0 || true'],
        condition=IfCondition(use_sim),
        output='screen',
    )
    can_setup_real = ExecuteProcess(
        cmd=['bash', '-c',
             'sudo killall slcand 2>/dev/null; sleep 1; '
             'if ip link show can0 >/dev/null 2>&1; then '
             '  sudo ip link set can0 up && sudo ip link set can0 txqueuelen 1000; '
             'else '
             '  if [ -e /dev/ttyACM0 ]; then sudo slcand -o -c -s8 /dev/ttyACM0 can0; '
             '  elif [ -e /dev/ttyACM1 ]; then sudo slcand -o -c -s8 /dev/ttyACM1 can0; '
             '  else echo "No CANable device found and can0 does not exist"; exit 1; fi && '
             '  sudo ip link set can0 up && sudo ip link set can0 txqueuelen 1000; '
             'fi'],
        condition=UnlessCondition(use_sim),
        output='screen',
    )

    # -- CAN Teardown on Shutdown --
    can_teardown = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[ExecuteProcess(
                cmd=['bash', '-c',
                     'sudo -n ip link set down vcan0 2>/dev/null || true; '
                     'sudo -n ip link set down can0 2>/dev/null || true; '
                     'sudo -n killall slcand 2>/dev/null || true'],
                output='screen',
            )],
        ),
    )

    # Delay loading and activation of robot_controller_names after `joint_state_broadcaster`
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for i, controller in enumerate(robot_controller_spawners):
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=(
                        robot_controller_spawners[i - 1]
                        if i > 0
                        else joint_state_broadcaster_spawner
                    ),
                    on_exit=[controller],
                )
            )
        ]

    # Delay start of inactive_robot_controller_names after other controllers
    delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for i, controller in enumerate(inactive_robot_controller_spawners):
        delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=(
                        inactive_robot_controller_spawners[i - 1]
                        if i > 0
                        else robot_controller_spawners[-1]
                    ),
                    on_exit=[controller],
                )
            )
        ]

    return LaunchDescription(
        declared_arguments
        + [
            can_setup_sim,
            can_setup_real,
            can_teardown,
            control_node,
            robot_state_pub_node,
            rviz_node,
            delay_joint_state_broadcaster_spawner_after_ros2_control_node,
            delay_motor_status_broadcaster_after_joint_state_broadcaster,
            # umdloop_can_node,
            controller_switcher_node,
            joystick_publisher,
        ]
        + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner
        + delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner
        + delay_gpio_controller_spawners_after_joint_state_broadcaster_spawner
    )
