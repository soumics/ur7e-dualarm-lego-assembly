"""lego_perception. backend:=ground_truth (default; rgbd backend comes later)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("world_name", default_value="lego_world"),
            DeclareLaunchArgument("require_in_view", default_value="true"),
            Node(
                package="lego_perception",
                executable="ground_truth_detector",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "world_name": LaunchConfiguration("world_name"),
                        "require_in_view": LaunchConfiguration("require_in_view"),
                    }
                ],
            ),
        ]
    )
