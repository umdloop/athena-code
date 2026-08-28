from launch import LaunchDescription, LaunchContext
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="arm_bringup",
            description='Package with the controller\'s configuration in "config" folder.',
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
            description="Description package with robot URDF/xacro files.",
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
            "prefix",
            default_value='""',
            description="Prefix of the joint names.",
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
            description="Enable mock command interfaces for sensors.",
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
            default_value="can1",
            description="CAN interface to use for hardware interfaces.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    # -- Initialize Arguments --
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    use_3dof = LaunchConfiguration("use_3dof")
    using_3dof = use_3dof.perform(context).lower() in ("true", "1", "yes")
    deactivate_talon = LaunchConfiguration("deactivate_talon")
    can_interface = LaunchConfiguration("can_interface")
    
    # -- Building Path Files --
    robot_description_path = PathJoinSubstitution(
        [FindPackageShare(description_package), "urdf", LaunchConfiguration("description_file")]
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

    controller_switcher_config = PathJoinSubstitution([
        FindPackageShare(runtime_config_package),
        "config",
        "controller_switcher_3dof.yaml" if using_3dof else "controller_switcher_2dof.yaml",
    ])

    # -- Additional Configuration Setup --
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
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # -- Node Definitions -- 
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
                    output="screen",
                    parameters=[controller_switcher_config, {"subsystem": "arm"}]
                )]
            )],
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

    return (
        [
            control_node,
            robot_state_pub_node,
            delay_joint_state_broadcaster_spawner_after_ros2_control_node,
            controller_switcher_node,
        ]
        + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner
        + delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner
    )
