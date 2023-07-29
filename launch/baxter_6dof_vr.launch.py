from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


import os


def generate_launch_description():
    ld = LaunchDescription()

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
                "zed2_runtime.yaml",
            ),
        }.items(),
    )

    zed2_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id",
            "table_0",
            "--child-frame-id",
            "zed2_base_frame",
            "--x",
            "0.5343319982006423",
            "--y",
            "-0.48647716285684056",
            "--z",
            "0.5893833698507722",
            "--qx",
            "0.06487012355055964",
            "--qy",
            "0.8778144915164856",
            "--qz",
            "0.22745164520153582",
            "--qw",
            "0.41653251331595187",
        ],
    )

    zed2i_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("zed_wrapper"),
                "launch",
                "include",
                "zed_camera.launch.py",
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
                "zed2i_runtime.yaml",
            ),
        }.items(),
    )

    zed2i_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id",
            "table_0",
            "--child-frame-id",
            "zed2i_base_frame",
            "--x",
            "0.5423720330055823",
            "--y",
            "0.510149396348734",
            "--z",
            "0.447303998733431",
            "--qx",
            "-0.17993694448649428",
            "--qy",
            "0.8500537405118741",
            "--qz",
            "-0.3705726404775456",
            "--qw",
            "0.3281878309446213",
        ],
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
    )

    d435_filter = Node(
        package="pcl_ros",
        executable="filter_voxel_grid_node",
        name="d435_filter_node",
        namespace="d435_filter",
        parameters=[
            {"leaf_size": 0.002},
        ],
        remappings=[
            ("input", "/D435/depth/color/points"),
        ],
    )

    table_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.6", "0", "0.73", "3.1415", "-1.57", "0", "ground", "table_0"],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("baxter_6dof_vr"), "config", "rviz.rviz"
            ),
        ],
    )

    tcp_node = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        name="tcp_node",
        parameters=[
            {"ROS_IP": "130.209.252.206"},
        ],
    )

    ld.add_action(zed2_camera_launch)
    ld.add_action(zed2_tf)

    ld.add_action(zed2i_camera_launch)
    ld.add_action(zed2i_tf)

    ld.add_action(table_tf)
    ld.add_action(rviz2)
    ld.add_action(tcp_node)

    return ld
