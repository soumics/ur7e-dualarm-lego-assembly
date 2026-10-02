"""Workcell simulation + MoveIt: Gazebo, controllers, perception, move_group and RViz.

    ros2 launch lego_dualarm_bringup sim_moveit.launch.py
    ros2 launch lego_dualarm_bringup sim_moveit.launch.py gazebo_gui:=false launch_rviz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = {
        "ur_type": LaunchConfiguration("ur_type"),
        "gripper": LaunchConfiguration("gripper"),
        "camera": LaunchConfiguration("camera"),
        "dual_arm": LaunchConfiguration("dual_arm"),
    }
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("lego_dualarm_bringup"), "launch", "sim.launch.py"])
        ),
        launch_arguments={**args, "gazebo_gui": LaunchConfiguration("gazebo_gui")}.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("lego_dualarm_moveit_config"), "launch", "moveit.launch.py"])
        ),
        launch_arguments={**args, "use_sim_time": "true",
                          "launch_rviz": LaunchConfiguration("launch_rviz")}.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type", default_value=EnvironmentVariable("UR_TYPE", default_value="ur7e")
            ),
            DeclareLaunchArgument("gripper", default_value="onrobot_2fg7"),
            DeclareLaunchArgument("camera", default_value="true"),
            DeclareLaunchArgument("dual_arm", default_value="true"),
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            sim,
            moveit,
        ]
    )
