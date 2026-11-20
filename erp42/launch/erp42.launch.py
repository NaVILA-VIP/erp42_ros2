from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_simulation(context):
    share = Path(get_package_share_directory("erp42"))
    gazebo = Path(get_package_share_directory("gazebo_ros"))
    localization = Path(get_package_share_directory("odometry_localization"))
    mode = LaunchConfiguration("drive_mode").perform(context)
    drive = "skid_drive" if mode == "skid" else "wheel_effort"
    localization_enabled = LaunchConfiguration("localization").perform(context).lower() in (
        "true", "1", "yes", "on"
    )

    return [
        IncludeLaunchDescription(AnyLaunchDescriptionSource(str(share / "launch/description.launch.py")),
                                 launch_arguments={"drive_mode": mode,
                                                   "publish_ground_truth_tf": "false" if localization_enabled else "true",
                                                   "use_sim_time": "true"}.items()),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(str(gazebo / "launch/gazebo.launch.py")),
                                 launch_arguments={"gui": LaunchConfiguration("gui"),
                                                   "world": LaunchConfiguration("world")}.items()),
        Node(package="gazebo_ros", executable="spawn_entity.py", output="screen",
             arguments=["-entity", "erp42", "-topic", "robot_description", "-z", "0.32"]),
        Node(package="controller_manager", executable="spawner", output="screen",
             arguments=["joint_state_broadcaster", "steering", drive,
                        "--controller-manager-timeout", "120"]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", str(share / "config/erp42.rviz")],
             parameters=[{"use_sim_time": True}], condition=IfCondition(LaunchConfiguration("rviz"))),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(str(localization / "launch/ekf_localization.launch.xml")),
            condition=IfCondition(LaunchConfiguration("localization"))),
    ]


def generate_launch_description():
    worlds = Path(get_package_share_directory("erp42_sdf_files")) / "worlds"
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Open the Gazebo client"),
        DeclareLaunchArgument("rviz", default_value="false", description="Open RViz2"),
        DeclareLaunchArgument("localization", default_value="false", description="Run noisy odometry and EKF"),
        DeclareLaunchArgument("drive_mode", default_value="effort", choices=["effort", "skid"]),
        DeclareLaunchArgument("world", default_value=str(worlds / "simple_world.world")),
        OpaqueFunction(function=start_simulation),
    ])
