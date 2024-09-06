from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    ExecuteProcess,
    GroupAction,
)
from launch.conditions import IfCondition

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    # Make sure to add parameter to all launch files, packages,
    # and parameters.
    launch_arguments = [
        ("image_topic", "/camera/color/image_raw", "Raw camera image topic."),
        ("camera_info_topic", "/camera/color/camera_info", "Camera info topic."),
        ("params_file", "tags_36h11.yaml", "Params file for apriltag node."),
        ("calibration_type", "eye_on_base", "Type of calibration"),
        ("name", "my_eob_calib", "Name of the calibration"),
        ("robot_base_frame", "camera_mount_front_vertical_link", "Robot base frame"),
        ("robot_effector_frame", "apriltag_bracelet", "Robot effector frame"),
        ("tracking_base_frame", "camera_link", "Tracking base frame"),
        ("tracking_marker_frame", "tag36h11:0", "Tracking marker frame"),
    ]

    declared_arguments = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in launch_arguments
    ]

    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        remappings=[
            ("image_rect", LaunchConfiguration("image_topic")),
            ("camera_info", LaunchConfiguration("camera_info_topic")),
        ],
        parameters=[{
            PathJoinSubstitution([
                FindPackageShare(LaunchConfiguration("apriltag_package")),
                "cfg",
                LaunchConfiguration("params_file"),
            ])}
        ]
    )

    easy_handeye_action = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("easy_handeye2"),
                        "launch",
                        "calibrate.launch.py",
                    ])
                ]),
                launch_arguments={
                    "calibration_type": LaunchConfiguration("calibration_type"),
                    "name": LaunchConfiguration("name"),
                    "robot_base_frame": LaunchConfiguration("robot_base_frame"),
                    "robot_effector_frame": LaunchConfiguration("robot_effector_frame"),
                    "tracking_base_frame": LaunchConfiguration("tracking_base_frame"),
                    "tracking_marker_frame": LaunchConfiguration("tracking_marker_frame"),
                }.items()
            )
        ]
    ) 

    launch = []
    nodes = [apriltag_node]
    actions = [easy_handeye_action]
    
    return LaunchDescription(declared_arguments + launch + nodes + actions)
