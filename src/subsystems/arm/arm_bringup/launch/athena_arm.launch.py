from launch import LaunchDescription, LaunchContext
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, TimerAction, OpaqueFunction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "mode",
            default_value="standalone",
            choices=["standalone", "jetson", "base_station"],
            description=(
                "Deployment mode. "
                "'standalone' (default) starts all nodes on a single machine. "
                "'jetson' starts only the control/hardware nodes (run on the rover). "
                "'base_station' starts only the teleop node (joystick)."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Simulation mode: enables mock hardware, virtual CAN (vcan0), "
                        "MoveIt, and joint trajectory controller on startup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="arm_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="athena_arm_controllers.yaml",
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
            default_value="athena_arm.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_file",
            default_value="rviz_config.rviz",
            description="Rviz config file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_package",
            default_value="arm_moveit",
            description="MoveIt package containing all the configurations and necessary files for \
            integrating MoveIt.",
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
            default_value="manual_arm_joint_by_joint_controller",
            choices=["manual_arm_joint_by_joint_controller"],
            description="Robot controller to start.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    use_sim = LaunchConfiguration("use_sim")
    use_rviz = LaunchConfiguration("use_rviz")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    rviz_file = LaunchConfiguration("rviz_file")
    moveit_package = LaunchConfiguration("moveit_package")
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
    robot_description_path = PathJoinSubstitution(
        [FindPackageShare("description"), "urdf", "athena_arm.urdf.xacro"]
    )
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
    joint_state_yaml = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", "initial_joint_states.yaml"]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", rviz_file]
    )
    joystick_config_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'config', 'joystick.yaml']
    )

    robot_semantic_path = PathJoinSubstitution(
        [FindPackageShare("arm_moveit"), "srdf", "athena_arm.srdf"]
    )
    robot_kinematics_path = PathJoinSubstitution(
        [FindPackageShare("arm_moveit"), "config", "kinematics.yaml"]
    )
    moveit_controllers_config_path = PathJoinSubstitution(
        [FindPackageShare("arm_moveit"), "config", "moveit_controllers.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_description_path,
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

    moveit_config = (
        MoveItConfigsBuilder("athena_arm", package_name="arm_moveit")
        .robot_description(file_path=robot_description_path.perform(LaunchContext()))
        .robot_description_semantic(file_path=robot_semantic_path.perform(LaunchContext()))
        .robot_description_kinematics(file_path=robot_kinematics_path.perform(LaunchContext()))
        .trajectory_execution(file_path=moveit_controllers_config_path.perform(LaunchContext()))
        .planning_scene_monitor(
            publish_robot_description=False, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )

    joystick_publisher = Node(
        package='teleop',
        executable='joystick',
        name='joystick',
        output='screen',
        parameters=[joystick_config_file],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
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
        condition=IfCondition(use_rviz),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    motor_status_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["motor_status_broadcaster", "-c", "/controller_manager"],
    )

    # Manual joint-by-joint controller: active only when use_sim is false
    robot_controller_spawner_active = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "-c", "/controller_manager"],
        condition=UnlessCondition(use_sim),
    )
    robot_controller_spawner_inactive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "-c", "/controller_manager", "--inactive"],
        condition=IfCondition(use_sim),
    )
    robot_controller_spawners = [robot_controller_spawner_active]

    # JTC: active when use_sim=true, inactive otherwise
    jtc_spawner_active = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        condition=IfCondition(use_sim),
    )
    jtc_spawner_inactive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager", "--inactive"],
        condition=UnlessCondition(use_sim),
    )

    inactive_robot_controller_names = ["manual_arm_cylindrical_controller", "arm_velocity_controller"]
    inactive_robot_controller_spawners = []
    inactive_robot_controller_spawners += [robot_controller_spawner_inactive]
    for controller in inactive_robot_controller_names:
        inactive_robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager", "--inactive"],
            )
        ]
    # Append JTC spawners (only one will run based on use_sim condition)
    inactive_robot_controller_spawners += [jtc_spawner_active, jtc_spawner_inactive]

    controller_switcher_node_active = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jtc_spawner_active,
            on_exit=[TimerAction(
                period=3.0,
                actions=[Node(
                    package="arm_bringup",
                    executable="controller_switcher.py",
                    name="controller_switcher",
                    output="screen"
                )]
            )],
        )
    )
    controller_switcher_node_inactive = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jtc_spawner_inactive,
            on_exit=[TimerAction(
                period=3.0,
                actions=[Node(
                    package="arm_bringup",
                    executable="controller_switcher.py",
                    name="controller_switcher",
                    output="screen"
                )]
            )],
        )
    )

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

    delay_motor_status_broadcaster_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[motor_status_broadcaster_spawner],
        )
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

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

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        condition=IfCondition(use_sim),
    )

    jetson_actions = [
        can_setup_sim,
        can_setup_real,
        can_teardown,
        control_node,
        robot_state_pub_node,
        move_group_node,
        delay_joint_state_broadcaster_spawner_after_ros2_control_node,
        delay_motor_status_broadcaster_after_joint_state_broadcaster,
        delay_rviz_after_joint_state_broadcaster_spawner,
        controller_switcher_node_active,
        controller_switcher_node_inactive,
    ] + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner \
      + delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner

    base_station_actions = [
        joystick_publisher,
    ]

    if mode == "jetson":
        return jetson_actions
    elif mode == "base_station":
        return base_station_actions
    else:
        return jetson_actions + base_station_actions
