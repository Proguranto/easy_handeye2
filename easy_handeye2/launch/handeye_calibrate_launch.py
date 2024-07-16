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
)

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
        ("use_left_hand", "true", "Use left hand hardware"),
        ("sim", "mock", "Simulation mode"),
        ("moveit_package", "geodude_moveit", "Robot moveit package."),
        ("moveit_file", "demo.launch.py", "Moveit file."),
        ("tracker_package", "orbbec_camera", "Package containing tracker\'s launch files."),
        ("tracker_file", "femto_mega.launch.py", "Tracker\'s launch file."),
        ("apriltag_package", "apriltag_ros", "Package containing apriltag\'s node."),
        ("apriltag_file", "apriltag_node", "Apriltag executable."),
        ("image_raw_topic", "/camera/color/image_raw", "Raw camera image topic."),
        ("camera_info_topic", "/camera/color/camera_info", "Camera info topic."),
        ("params_file", "tags_36h11.yaml", "Params file for apriltag node."),
        ("easy_handeye_package", "easy_handeye2", "Package containing easy_handeye2."),
        ("easy_handeye_file", "calibrate.launch.py", "Calibration file for easy_handeye2."),
        ("calibration_type", "eye_on_base", "Type of calibration"),
        ("name", "my_eib_calib", "Name of the calibration"),
        ("robot_base_frame", "right_wam_base", "Robot base frame"),
        ("robot_effector_frame", "right_wam5", "Robot effector frame"),
        ("tracking_base_frame", "camera_link", "Tracking base frame"),
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
        ])
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

    easy_handeye_launch = IncludeLaunchDescription(
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

    publish_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(LaunchConfiguration("easy_handeye_package")),
                "launch",
                LaunchConfiguration("publish_file")
            ])
        ]),
        launch_arguments={
            "name": LaunchConfiguration("name"),
        }.items()
    )

    launch_p1 = [robot_launch, moveit_launch, tracker_launch]

    nodes = [apriltag_node]

    launch_p2 = [easy_handeye_launch]
    
    return LaunchDescription(declared_arguments + launch_p1 + nodes + launch_p2)