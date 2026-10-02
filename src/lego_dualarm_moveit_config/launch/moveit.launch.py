"""MoveIt move_group (+ optional RViz) for the LEGO workcell.

Expects the robot (sim or mock) to be running already; see
lego_dualarm_bringup/launch/sim_phase1_moveit.launch.py for the combined launch.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_config(ur_type, gripper, camera, dual_arm):
    description_xacro = os.path.join(
        get_package_share_directory("lego_dualarm_description"), "urdf", "lego_workcell.urdf.xacro"
    )
    return (
        MoveItConfigsBuilder("lego_workcell", package_name="lego_dualarm_moveit_config")
        .robot_description(
            file_path=description_xacro,
            mappings={"ur_type": ur_type, "gripper": gripper, "camera": camera,
                      "dual_arm": dual_arm, "sim_gz": "true"},
        )
        .robot_description_semantic(
            file_path="srdf/lego_workcell.srdf.xacro", mappings={"gripper": gripper, "camera": camera, "dual_arm": dual_arm}
        )
        .trajectory_execution(file_path=f"config/moveit_controllers_{gripper}.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )


def launch_setup(context):
    ur_type = LaunchConfiguration("ur_type").perform(context)
    gripper = LaunchConfiguration("gripper").perform(context)
    camera = LaunchConfiguration("camera").perform(context)
    dual_arm = LaunchConfiguration("dual_arm").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"

    moveit_config = build_moveit_config(ur_type, gripper, camera, dual_arm)

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time, "publish_robot_description_semantic": True},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", os.path.join(
            get_package_share_directory("lego_dualarm_moveit_config"), "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )
    return [move_group, rviz]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type", default_value=EnvironmentVariable("UR_TYPE", default_value="ur7e")
            ),
            DeclareLaunchArgument("gripper", default_value="onrobot_2fg7"),
            DeclareLaunchArgument("camera", default_value="true"),
            DeclareLaunchArgument("dual_arm", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
