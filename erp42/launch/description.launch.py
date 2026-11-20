from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def state_publisher(context):
    path = Path(get_package_share_directory("erp42")) / "urdf" / "erp42.urdf.xacro"
    mode = LaunchConfiguration("drive_mode").perform(context)
    publish_tf = LaunchConfiguration("publish_ground_truth_tf").perform(context)
    description = xacro.process_file(str(path), mappings={
        "drive_mode": mode, "publish_ground_truth_tf": publish_tf,
    }).toxml()
    return [Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": description,
                     "use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("drive_mode", default_value="effort", choices=["effort", "skid"]),
        DeclareLaunchArgument("publish_ground_truth_tf", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=state_publisher),
    ])
