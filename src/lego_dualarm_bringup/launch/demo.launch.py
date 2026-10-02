"""Workcell + MoveIt with recording-friendly views: Gazebo GUI (3D view only) and RViz
from the same viewpoint, showing the robot, perceived bricks, the planner's remaining
target bricks and the overhead RGB-D point cloud.

    ros2 launch lego_dualarm_bringup demo.launch.py world_file:=$PWD/experiments/configs/house_door/world.sdf

The build itself is started separately (build_executor), exactly as without the demo views.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("lego_dualarm_bringup")
    sim_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([share, "launch", "sim_moveit.launch.py"])),
        launch_arguments={
            "launch_rviz": "false",
            "gazebo_gui": "true",
            "gz_gui_args": ["--gui-config ", PathJoinSubstitution([share, "config", "demo_gui.config"])],
        }.items(),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_demo",
        output="log",
        arguments=["-d", PathJoinSubstitution([share, "config", "demo.rviz"])],
        parameters=[{"use_sim_time": True}],
    )
    markers = Node(
        package="lego_perception",
        executable="detection_markers",
        output="log",
        parameters=[{"use_sim_time": True,
                     "baseplate_studs": ParameterValue(LaunchConfiguration("baseplate_studs"), value_type=int),
                     "baseplate_xyz": [0.0, 0.0, 0.75]}],
    )
    return LaunchDescription([
        DeclareLaunchArgument("baseplate_studs", default_value="8"),
        sim_moveit,
        rviz,
        markers,
    ])
