from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition
from launch.actions import LogInfo, DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration, PythonExpression


import os


def generate_launch_description():
    ld = LaunchDescription()

    zed2 = LaunchConfiguration("zed2")
    ld.add_action(
        DeclareLaunchArgument("zed2", default_value="True"),
    )
    zed2i = LaunchConfiguration("zed2i")
    ld.add_action(
        DeclareLaunchArgument("zed2i", default_value="True"),
    )
    d435 = LaunchConfiguration("d435")
    ld.add_action(
        DeclareLaunchArgument("d435", default_value="True"),
    )

    zed2_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("zed_wrapper"),
                "launch",
                "include",
                "zed_camera.launch.py",
            )
        ),
        launch_arguments={
            "camera_model": "zed2",
            "serial_number": "28066000",
            "base_frame": "zed2_base_frame",
            "publish_tf": "false",
            "ros_params_override_path": os.path.join(
                get_package_share_directory("baxter_6dof_vr"),
                "config",
                "zed2_calibration.yaml",
            ),
        }.items(),
        condition=IfCondition(PythonExpression(zed2)),
    )

    zed2_apriltag_node = Node(
        package="apriltag_ros2",
        executable="apriltag_ros2_continuous_detector_node",
        name="zed2_apriltag_node",
        parameters=[
            {"size": 0.75},
            {"publish_tag_detections_image": True},
            {"camera_frame_id": "zed2_left_camera_frame"},
            {"camera_base_frame_id": "zed2_base_frame"},
            {
                os.path.join(
                    get_package_share_directory("apriltag_ros2"),
                    "config",
                    "settings.param.yaml",
                )
            },
            {
                os.path.join(
                    get_package_share_directory("apriltag_ros2"),
                    "config",
                    "tags.param.yaml",
                )
            },
        ],
        remappings=[
            ("~/image_rect", "/zed2/zed_node/rgb_raw/image_raw_color"),
            ("~/camera_info", "/zed2/zed_node/rgb_raw/camera_info"),
        ],
        condition=IfCondition(PythonExpression(zed2)),
    )

    zed2i_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("zed_wrapper"), "launch", "zed2i.launch.py"
            )
        ),
        launch_arguments={
            "camera_model": "zed2i",
            "serial_number": "35710949",
            "publish_tf": "false",
            "base_frame": "zed2i_base_frame",
            "ros_params_override_path": os.path.join(
                get_package_share_directory("baxter_6dof_vr"),
                "config",
                "zed2i_calibration.yaml",
            ),
        }.items(),
        condition=IfCondition(PythonExpression(zed2i)),
    )

    zed2i_apriltag_node = Node(
        package="apriltag_ros2",
        executable="apriltag_ros2_continuous_detector_node",
        name="zed2i_apriltag_node",
        parameters=[
            {"size": 0.75},
            {"publish_tag_detections_image": True},
            {"camera_frame_id": "zed2i_left_camera_frame"},
            {"camera_base_frame_id": "zed2i_base_frame"},
            {
                os.path.join(
                    get_package_share_directory("apriltag_ros2"),
                    "config",
                    "settings.param.yaml",
                )
            },
            {
                os.path.join(
                    get_package_share_directory("apriltag_ros2"),
                    "config",
                    "tags.param.yaml",
                )
            },
        ],
        remappings=[
            ("~/image_rect", "/zed2i/zed_node/rgb_raw/image_raw_color"),
            ("~/camera_info", "/zed2i/zed_node/rgb_raw/camera_info"),
        ],
        condition=IfCondition(PythonExpression(zed2i)),
    )

    d435_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        launch_arguments={
            "align_depth.enable": "true",
            "pointcloud.enable": "true",
            "camera_name": "D435",
        }.items(),
        condition=IfCondition(PythonExpression(d435)),
    )

    d435_apriltag_node = Node(
        package="apriltag_ros2",
        executable="apriltag_ros2_continuous_detector_node",
        name="d435_apriltag_node",
        parameters=[
            {"size": 0.75},
            {"publish_tag_detections_image": True},
            {"camera_frame_id": "D435_color_frame"},
            {"camera_base_frame_id": "D435_link"},
            {
                os.path.join(
                    get_package_share_directory("apriltag_ros2"),
                    "config",
                    "settings.param.yaml",
                )
            },
            {
                os.path.join(
                    get_package_share_directory("apriltag_ros2"),
                    "config",
                    "tags.param.yaml",
                )
            },
        ],
        remappings=[
            ("~/image_rect", "/D435/color/image_raw"),
            ("~/camera_info", "/D435/color/camera_info"),
        ],
        condition=IfCondition(PythonExpression(d435)),
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("baxter_6dof_vr"),
                "config",
                "calibrate_cameras.rviz",
            ),
        ],
    )

    ld.add_action(zed2_camera_launch)
    ld.add_action(zed2_apriltag_node)

    ld.add_action(zed2i_camera_launch)
    ld.add_action(zed2i_apriltag_node)

    ld.add_action(d435_camera_launch)
    ld.add_action(d435_apriltag_node)

    ld.add_action(rviz2)

    return ld
