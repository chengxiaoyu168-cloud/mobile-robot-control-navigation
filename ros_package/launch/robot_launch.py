import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import (
    get_package_share_directory,
    get_packages_with_prefixes,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)
def generate_launch_description():
    package_dir = get_package_share_directory("webots_ros2_robomaster")
    world = LaunchConfiguration("world")
    mode = LaunchConfiguration("mode")
    use_rviz = LaunchConfiguration("rviz", default=False)
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, "worlds", world]),
        mode=mode,
        ros2_supervisor=True,
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": '<robot name=""><link name=""/></robot>'}],
    )
    footprint_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
    )
    controller_manager_timeout = ["--controller-manager-timeout", "50"]
    diffdrive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["diffdrive_controller"] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster"] + controller_manager_timeout,
    )
    ros_control_spawners = [
        diffdrive_controller_spawner,
        joint_state_broadcaster_spawner,
    ]
    robot_description_path = os.path.join(package_dir, "resource", "robomaster.urdf")
    ros2_control_params = os.path.join(package_dir, "resource", "ros2control.yml")
    mappings_0 = [
        ("/diffdrive_controller/cmd_vel_unstamped", "/agent0/cmd_vel"),
        ("/diffdrive_controller/odom", "/agent0/odom"),
    ]
    mappings_1 = [
        ("/diffdrive_controller/cmd_vel_unstamped", "/agent1/cmd_vel"),
        ("/diffdrive_controller/odom", "/agent1/odom"),
    ]
    mappings_0.append(
        ("/agent0/camera/image_color", "/robomaster_0/camera_0/image_proc")
    )
    mappings_1.append(
        ("/agent1/camera/image_color", "/robomaster_1/camera_0/image_proc")
    )
    robomaster_driver = WebotsController(
        robot_name="agent0",
        parameters=[
            {
                "robot_description": robot_description_path,
                "use_sim_time": use_sim_time,
                "set_robot_state_publisher": True,
            },
            ros2_control_params,
        ],
        remappings=mappings_0,
        respawn=True,
    )
    robomaster_driver_1 = WebotsController(
        robot_name="agent1",
        parameters=[
            {
                "robot_description": robot_description_path,
                "use_sim_time": use_sim_time,
                "set_robot_state_publisher": True,
            },
            ros2_control_params,
        ],
        remappings=mappings_1,
        respawn=True,
    )
    rviz_config = os.path.join(
        get_package_share_directory("webots_ros2_robomaster"),
        "resource",
        "default.rviz",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["--display-config=" + rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=launch.conditions.IfCondition(use_rviz),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value='break_room.wbt',
                description="Choose one of the world files from `/webots_ros2_robomaster/world` directory",
            ),
            DeclareLaunchArgument(
                "mode", default_value="realtime", description="Webots startup mode"
            ),
            webots,
            webots._supervisor,
            robot_state_publisher,
            footprint_publisher,
            robomaster_driver,
            robomaster_driver_1,
            diffdrive_controller_spawner,
            joint_state_broadcaster_spawner,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
