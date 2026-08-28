from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_moveit",
            default_value="false",
            description="Start the MoveIt move_group and hello_moveit nodes.",
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
            default_value="",
            description="Optional controller YAML override. The wrist-specific configuration is selected by default.",
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
            default_value="arm/athena_arm.urdf.xacro",
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
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. \
            Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_3dof",
            default_value="false",
            description="Enable the joints required for the 3 Degree of Freedom Wrist",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "deactivate_talon",
            default_value="false",
            description="Deactivate the talon joints in the URDF when using mock hardware to prevent excessive CAN flow.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="CAN interface to use for hardware interfaces.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    use_sim = LaunchConfiguration("use_sim")
    use_moveit = LaunchConfiguration("use_moveit")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    rviz_file = LaunchConfiguration("rviz_file")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    using_mock_hardware = use_mock_hardware.perform(context).lower() in ("true", "1", "yes")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    use_3dof = LaunchConfiguration("use_3dof")
    using_3dof = use_3dof.perform(context).lower() in ("true", "1", "yes")
    deactivate_talon = LaunchConfiguration("deactivate_talon")
    can_interface = LaunchConfiguration("can_interface")
    
    # -- Building Path Files --
    # Get URDF via xacro.
    # This is creating a terminal command that essentially expands all macros in this file
    # and creates the FULL URDF
    robot_description_path = PathJoinSubstitution(
        [FindPackageShare(description_package), "urdf", description_file]
    )
    controllers_file_name = controllers_file.perform(context)
    if not controllers_file_name:
        controllers_file_name = (
            "athena_arm_controllers_3dof.yaml"
            if using_3dof
            else "athena_arm_controllers_2dof.yaml"
        )
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file_name]
    )
    joystick_config_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'config', 'joystick.yaml']
    )

    controller_switcher_config = PathJoinSubstitution([
        FindPackageShare(runtime_config_package),
        "config",
        "controller_switcher_3dof.yaml" if using_3dof else "controller_switcher_2dof.yaml",
    ])

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
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
            "use_3dof:=",
            use_3dof,
            " ",
            "deactivate_talon:=",
            deactivate_talon,
            " ",
            "can_interface:=",
            can_interface,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    motor_status_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["motor_status_controller", "-c", "/controller_manager"],
    )

    robot_controller_names = [
        "manual_arm_joint_by_joint_controller",
        "manual_wrist_joint_by_joint_controller",
        "manual_end_effector_gripper_claw_controller",
    ]
    robot_controller_spawners = []
    for controller in robot_controller_names:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
            )
        ]

    inactive_robot_controller_names = [
        "joint_trajectory_controller",
        "arm_velocity_controller",
    ]
    if not using_3dof:
        inactive_robot_controller_names.insert(0, "manual_arm_cylindrical_controller")
    inactive_robot_controller_spawners = []
    for controller in inactive_robot_controller_names:
        inactive_robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager", "--inactive"],
            )
        ]

    active_robot_controller_names = ["cam_position_controller"]
    if not using_mock_hardware:
        active_robot_controller_names.insert(
            0, "rotary_encoder_state_request_controller"
        )
    for controller in active_robot_controller_names:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
            )
        ]
    controller_switcher_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=inactive_robot_controller_spawners[-1],
            on_exit=[TimerAction(
                period=3.0,
                actions=[Node(
                    package="bringup",
                    executable="controller_switcher.py",
                    name="controller_switcher",
                    parameters=[controller_switcher_config, {"subsystem": "arm"}],
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

    delay_motor_status_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[motor_status_controller_spawner],
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

    standalone_visualization_actions = []
    if mode == "standalone":
        from moveit_configs_utils import MoveItConfigsBuilder

        rviz_config_file = PathJoinSubstitution(
            [FindPackageShare(description_package), "rviz", rviz_file]
        )
        robot_semantic_path = PathJoinSubstitution(
            [FindPackageShare("arm_moveit"), "srdf", "athena_arm.srdf"]
        )
        robot_kinematics_path = PathJoinSubstitution(
            [FindPackageShare("arm_moveit"), "config", "kinematics.yaml"]
        )
        moveit_controllers_config_path = PathJoinSubstitution([
            FindPackageShare("arm_moveit"),
            "config",
            "moveit_controllers_3dof.yaml" if using_3dof else "moveit_controllers_2dof.yaml",
        ])
        moveit_config = (
            MoveItConfigsBuilder("athena_arm", package_name="arm_moveit")
            .robot_description(file_path=robot_description_path.perform(context))
            .robot_description_semantic(file_path=robot_semantic_path.perform(context))
            .robot_description_kinematics(file_path=robot_kinematics_path.perform(context))
            .trajectory_execution(file_path=moveit_controllers_config_path.perform(context))
            .planning_scene_monitor(
                publish_robot_description=True, publish_robot_description_semantic=True
            )
            .planning_pipelines(
                pipelines=["ompl", "pilz_industrial_motion_planner"],
                default_planning_pipeline="ompl",
            )
            .to_moveit_configs()
        )
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
            condition=IfCondition(use_sim),
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        )
        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            condition=IfCondition(use_moveit),
            parameters=[moveit_config.to_dict()],
        )
        standalone_visualization_actions.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[rviz_node, move_group_node],
                )
            )
        )

    jetson_actions = [
        control_node,
        robot_state_pub_node,
        delay_joint_state_broadcaster_spawner_after_ros2_control_node,
        controller_switcher_node,
    ] + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner \
      + delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner

    if not using_mock_hardware:
        jetson_actions.append(
            delay_motor_status_controller_after_joint_state_broadcaster
        )

    base_station_actions = [
        joystick_publisher,
    ]

    if mode == "jetson":
        return jetson_actions
    elif mode == "base_station":
        return base_station_actions
    else:
        return jetson_actions + base_station_actions + standalone_visualization_actions
