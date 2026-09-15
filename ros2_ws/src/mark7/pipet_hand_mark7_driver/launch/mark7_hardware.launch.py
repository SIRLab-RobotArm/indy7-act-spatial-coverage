import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def validate_serial_access(context):
    if LaunchConfiguration("use_mock_hardware").perform(context).lower() == "true":
        return []

    port = LaunchConfiguration("port").perform(context)
    if not os.path.exists(port):
        raise RuntimeError(
            f"Mark7 serial port does not exist: {port}. "
            "Connect the Mark7 dongle or set mark7_port:=<device>."
        )
    if not os.access(port, os.R_OK | os.W_OK):
        raise RuntimeError(
            f"Mark7 serial port permission denied: {port}. "
            "Run `sudo usermod -aG dialout $USER`, then log out and back in. "
            "For the current stale session, run the launch through `sg dialout -c`."
        )
    return []


def generate_launch_description():

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    port = LaunchConfiguration("port")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_use_mock = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="false",
        description="Use mock hardware (no serial port required)",
    )
    declare_port = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyACM0",
        description="Serial port for Mark7 dongle (e.g. /dev/ttyACM0, /dev/ttyUSB0)",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Launch RViz2 for visualization",
    )

    # URDF → robot_description
    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution([
            FindPackageShare("pipet_hand_mark7_description"),
            "urdf",
            "pipet_hand_mark7.xacro",
        ]),
        " use_mock_hardware:=", use_mock_hardware,
        " port:=", port,
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("pipet_hand_mark7_driver"),
        "config",
        "mark7_controllers.yaml",
    ])

    # robot_state_publisher - publishes tf
    # A distinct name avoids colliding with the Indy7; joint_states is
    # remapped to /mark7/joint_states
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="mark7_robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        remappings=[
            ("joint_states", "/mark7/joint_states"),
            ("robot_description", "/mark7/robot_description"),
        ],
    )

    # controller_manager - loads the hardware plugin and manages the
    # controllers, in the /mark7 namespace
    # Resulting topics: /mark7/joint_states,
    # /mark7/forward_position_controller/commands
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="mark7",
        parameters=[robot_description, controllers_yaml],
        output="screen",
    )

    # joint_state_broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/mark7/controller_manager",
                   "--controller-manager-timeout", "30"],
        output="screen",
    )

    # forward_position_controller spawner, started once
    # joint_state_broadcaster is active
    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/mark7/controller_manager",
                   "--controller-manager-timeout", "30"],
        output="screen",
    )

    delay_forward_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[forward_position_controller_spawner],
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("pipet_hand_mark7_description"),
            "config",
            "urdf.rviz",
        ])],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_use_mock,
        declare_port,
        declare_use_rviz,
        OpaqueFunction(function=validate_serial_access),
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_forward_controller,
        rviz_node,
    ])
