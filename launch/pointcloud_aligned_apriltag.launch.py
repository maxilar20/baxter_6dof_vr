from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


import os


def generate_launch_description():
    ld = LaunchDescription()

    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("zed_wrapper"), "launch", "zed.launch.py"
            )
        ),
        launch_arguments={"publish_tf": "false", "zed_id": "0"}.items(),
    )

    zed_apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_ros_continuous_detector_node",
        name="zed_apriltag_node",
        parameters=[
            {"size": 0.75},
            {"publish_tag_detections_image": True},
            {"frame_id": "zed_left_camera_frame"},
            {
                os.path.join(
                    get_package_share_directory("apriltag_ros"),
                    "config",
                    "settings.param.yaml",
                )
            },
            {
                os.path.join(
                    get_package_share_directory("apriltag_ros"),
                    "config",
                    "tags.param.yaml",
                )
            },
        ],
        remappings=[
            ("~/image_rect", "/zed/zed_node/rgb_raw/image_raw_color"),
            ("~/camera_info", "/zed/zed_node/rgb_raw/camera_info"),
        ],
    )

    zed_filter = Node(
        package="pcl_ros",
        executable="filter_voxel_grid_node",
        name="zed_filter_node",
        namespace="zed_filter",
        parameters=[
            {"leaf_size": 0.003},
        ],
        remappings=[
            ("input", "/zed/zed_node/point_cloud/cloud_registered"),
        ],
    )

    zed2_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("zed_wrapper"), "launch", "zed2.launch.py"
            )
        ),
        launch_arguments={"publish_tf": "false", "zed_id": "0"}.items(),
    )

    zed2_apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_ros_continuous_detector_node",
        name="zed2_apriltag_node",
        parameters=[
            {"size": 0.75},
            {"publish_tag_detections_image": True},
            {"frame_id": "zed2_left_camera_frame"},
            {
                os.path.join(
                    get_package_share_directory("apriltag_ros"),
                    "config",
                    "settings.param.yaml",
                )
            },
            {
                os.path.join(
                    get_package_share_directory("apriltag_ros"),
                    "config",
                    "tags.param.yaml",
                )
            },
        ],
        remappings=[
            ("~/image_rect", "/zed2/zed_node/rgb_raw/image_raw_color"),
            ("~/camera_info", "/zed2/zed_node/rgb_raw/camera_info"),
        ],
    )

    zed2_filter = Node(
        package="pcl_ros",
        executable="filter_voxel_grid_node",
        name="zed2_filter_node",
        namespace="zed2_filter",
        parameters=[
            {"leaf_size": 0.005},
        ],
        remappings=[
            ("input", "/zed2/zed_node/point_cloud/cloud_registered"),
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
        package="apriltag_ros",
        executable="apriltag_ros_continuous_detector_node",
        name="d435_apriltag_node",
        parameters=[
            {"size": 0.75},
            {"publish_tag_detections_image": True},
            {"frame_id": "D435_color_frame"},
            {
                os.path.join(
                    get_package_share_directory("apriltag_ros"),
                    "config",
                    "settings.param.yaml",
                )
            },
            {
                os.path.join(
                    get_package_share_directory("apriltag_ros"),
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

    # ld.add_action(zed_camera_launch)
    # ld.add_action(zed_apriltag_node)
    # ld.add_action(zed_filter)

    ld.add_action(zed2_camera_launch)
    ld.add_action(zed2_apriltag_node)
    ld.add_action(zed2_filter)

    ld.add_action(d435_camera_launch)
    ld.add_action(d435_apriltag_node)
    ld.add_action(d435_filter)

    ld.add_action(rviz2)
    ld.add_action(tcp_node)

    return ld
