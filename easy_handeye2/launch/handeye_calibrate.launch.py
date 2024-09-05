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
        ("robot_package", "geodude_hardware", "Package containing robot transform's launch files."),
        ("robot_file", "geodude_hardware.launch.py", "Robot\'s launch file to setup base to effector frame."),
        ("use_left", "true", "Use left hardware"),
        ("use_right", "true", "Use right hardware"),
        ("use_right_hand", "true", "Use right hand hardware"),
        ("use_left_hand", "true", "Use left hand hardware"),
        ("sim", "mock", "Simulation mode"),
        ("moveit_package", "geodude_moveit", "Robot moveit package."),
        ("moveit_file", "demo.launch.py", "Moveit file."),
        ("tracker_package", "orbbec_camera", "Package containing tracker\'s launch files."),
        ("tracker_file", "femto_bolt.launch.py", "Tracker\'s launch file."),
        ("camera_name", "camera", "Camera name."),
        ("usb_port", "", "Usb port for camera"),
        ("device_num", "1", "Camera device number."),
        ("enable_colored_point_cloud", "true", "Setting for enabling colored pc."),
        ("apriltag_package", "apriltag_ros", "Package containing apriltag\'s node."),
        ("apriltag_file", "apriltag_node", "Apriltag executable."),
        ("image_raw_topic", "/front_camera/color/image_raw", "Raw camera image topic."),
        ("camera_info_topic", "/front_camera/color/camera_info", "Camera info topic."),
        ("params_file", "tags_36h11.yaml", "Params file for apriltag node."),
        ("easy_handeye_package", "easy_handeye2", "Package containing easy_handeye2."),
        ("easy_handeye_file", "calibrate.launch.py", "Calibration file for easy_handeye2."),
        ("calibration_type", "eye_on_base", "Type of calibration"),
        ("name", "my_eob_calib", "Name of the calibration"),
        ("robot_base_frame", "camera_mount_front_vertical_link", "Robot base frame"),
        ("robot_effector_frame", "apriltag_bracelet", "Robot effector frame"),
        ("tracking_base_frame", "front_camera_link", "Tracking base frame"),
        ("tracking_marker_frame", "tag36h11:0", "Tracking marker frame"),
        ("publish_file", "publish.launch.py", "Calibration publisher launch file for easy_handeye2."),
    ]

    declared_arguments = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in launch_arguments
    ]
    
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(LaunchConfiguration("robot_package")),
                "launch",
                LaunchConfiguration("robot_file"),
            ])
        ]),
        launch_arguments={
            "use_left": LaunchConfiguration("use_left"),
            "use_right": LaunchConfiguration("use_right"),
            "use_left_hand": LaunchConfiguration("use_left_hand"),
            "use_right_hand": LaunchConfiguration("use_right_hand"),
            "sim": LaunchConfiguration("sim"),
        }.items()
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(LaunchConfiguration("moveit_package")),
                "launch",
                LaunchConfiguration("moveit_file"),
            ])
        ])
    )


    tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(LaunchConfiguration("tracker_package")),
                "launch",
                LaunchConfiguration("tracker_file"),
            ])
        ]),
        launch_arguments={
            "camera_name": LaunchConfiguration("camera_name"),
            "usb_port": LaunchConfiguration("usb_port"),
            "device_num": LaunchConfiguration("device_num"),
            "enable_colored_point_cloud": LaunchConfiguration("enable_colored_point_cloud"),
        }.items()
    )

    apriltag_node = Node(
        package=LaunchConfiguration("apriltag_package"),
        executable=LaunchConfiguration("apriltag_file"),
        name="apriltag",
        remappings=[
            ("image_rect", LaunchConfiguration("image_raw_topic")),
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
                        FindPackageShare(LaunchConfiguration("easy_handeye_package")),
                        "launch",
                        LaunchConfiguration("easy_handeye_file"),
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

    launch = [moveit_launch, tracker_launch]
    nodes = [apriltag_node]
    actions = [easy_handeye_action]
    
    return LaunchDescription(declared_arguments + launch + nodes + actions)
