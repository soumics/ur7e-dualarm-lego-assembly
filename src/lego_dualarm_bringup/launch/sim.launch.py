"""Workcell simulation: table + two UR arms + grippers + overhead camera + bricks in Gazebo Harmonic.

    ros2 launch lego_dualarm_bringup sim.launch.py                    # GUI, ur7e, both arms
    ros2 launch lego_dualarm_bringup sim.launch.py gazebo_gui:=false  # headless server
    ros2 launch lego_dualarm_bringup sim.launch.py ur_type:=ur5e dual_arm:=false
"""

from launch import LaunchDescription
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    gripper = LaunchConfiguration("gripper")
    camera = LaunchConfiguration("camera")
    dual_arm = LaunchConfiguration("dual_arm")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")
    controllers_file = LaunchConfiguration("controllers_file")

    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value=EnvironmentVariable("UR_TYPE", default_value="ur7e"),
            description="Any UR model supported by ur_description.",
        ),
        DeclareLaunchArgument("gripper", default_value="onrobot_2fg7",
                              description="onrobot_2fg7 (lab gripper) | parallel (generic)"),
        DeclareLaunchArgument("camera", default_value="true", description="Overhead RGB-D camera?"),
        DeclareLaunchArgument("dual_arm", default_value="true", description="false: left arm only"),
        DeclareLaunchArgument("gazebo_gui", default_value="true", description="Start the Gazebo GUI?"),
        DeclareLaunchArgument("gz_gui_args", default_value="",
                              description="Extra gz sim args for the GUI, e.g. --gui-config <file>"),
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("lego_dualarm_description"), "worlds", "lego_workcell.sdf"]
            ),
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("lego_dualarm_bringup"), "config",
                 ["workcell_controllers_", LaunchConfiguration("gripper"), ".yaml"]]
            ),
        ),
    ]

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("lego_dualarm_description"), "urdf", "lego_workcell.urdf.xacro"]
            ),
            " ur_type:=", ur_type,
            " gripper:=", gripper,
            " camera:=", camera,
            " dual_arm:=", dual_arm,
            " sim_gz:=true",
            " simulation_controllers:=", controllers_file,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": IfElseSubstitution(
                gazebo_gui,
                if_value=[" -r -v 3 ", LaunchConfiguration("gz_gui_args"), " ", world_file],
                else_value=[" -s -r -v 3 ", world_file],
            ),
            "on_exit_shutdown": "true",
        }.items(),
    )

    spawn_workcell = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-name", "lego_workcell", "-topic", "robot_description", "-allow_renaming", "false"],
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # ROS -> gz: grasp lock commands for the GraspLock world plugin
            "/lego/grasp_lock@std_msgs/msg/String]gz.msgs.StringMsg",
        ],
        output="screen",
    )

    # Brick detections on /lego/detections (ground-truth backend, camera FOV filtered).
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("lego_perception"), "launch", "perception.launch.py"])
        ),
        condition=IfCondition(camera),
    )

    # Overhead RGB-D camera (gz rgbd_camera, topic prefix "camera") -> RealSense-style ROS names.
    # Gazebo's own point cloud is not bridged: its data is x-forward but stamped with the
    # optical frame, so it renders ~1 m off. The cloud is built from depth + intrinsics instead.
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        arguments=[
            "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        remappings=[
            ("/camera/image", "/camera/color/image_raw"),
            ("/camera/depth_image", "/camera/depth/image_raw"),
            ("/camera/camera_info", "/camera/color/camera_info"),
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(camera),
    )

    # Coloured point cloud from depth registered to colour (same simulated sensor), as
    # depth_image_proc does for a real RealSense. Published on /camera/points in the optical frame.
    point_cloud = ComposableNodeContainer(
        name="camera_point_cloud_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::PointCloudXyzrgbNode",
                name="point_cloud_xyzrgb",
                remappings=[
                    ("rgb/image_rect_color", "/camera/color/image_raw"),
                    ("rgb/camera_info", "/camera/color/camera_info"),
                    ("depth_registered/image_rect", "/camera/depth/image_raw"),
                    ("points", "/camera/points"),
                ],
                parameters=[{"use_sim_time": True}],
            )
        ],
        output="screen",
        condition=IfCondition(camera),
    )

    def spawner(name, condition=None):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager",
                       "--controller-manager-timeout", "60"],
            output="screen",
            condition=condition,
        )

    # Gazebo resolves package:// meshes through GZ_SIM_RESOURCE_PATH; add the OnRobot packages' share parents.
    onrobot_paths = [
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(get_package_share_directory(p)))
        for p in ("onrobot_2fg7",)
    ]

    return LaunchDescription(
        declared_arguments
        + onrobot_paths
        + [
            robot_state_publisher,
            gz_sim,
            spawn_workcell,
            gz_bridge,
            camera_bridge,
            point_cloud,
            perception,
            spawner("joint_state_broadcaster"),
            spawner("left_arm_controller"),
            spawner("left_gripper_controller"),
            spawner("right_arm_controller", IfCondition(dual_arm)),
            spawner("right_gripper_controller", IfCondition(dual_arm)),
        ]
    )
