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
                get_package_share_directory("zed_wrapper"), "launch", "zed2.launch.py"
            )
        ),
        launch_arguments={
            "zed_id": "0",
            "base_frame": "zed2_base_frame",
        }.items(),
    )

    zed2_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id",
            "table_0",
            "--child-frame-id",
            "map",
            "--x",
            "0.5297981623017833",
            "--y",
            "-0.6792716403394491",
            "--z",
            "0.08371118750384102",
            "--qx",
            "0.45185206192850896",
            "--qy",
            "0.5498215966902548",
            "--qz",
            "0.4442062591791017",
            "--qw",
            "0.5442487714731586",
        ],
    )

    # zed2_apriltag_node = Node(
    #     package="apriltag_ros2",
    #     executable="apriltag_ros2_continuous_detector_node",
    #     name="zed2_apriltag_node",
    #     parameters=[
    #         {"size": 0.75},
    #         {"publish_tag_detections_image": True},
    #         {"camera_frame_id": "zed2_left_camera_frame"},
    #         {"camera_base_frame_id": "map"},
    #         {
    #             os.path.join(
    #                 get_package_share_directory("apriltag_ros2"),
    #                 "config",
    #                 "settings.param.yaml",
    #             )
    #         },
    #         {
    #             os.path.join(
    #                 get_package_share_directory("apriltag_ros2"),
    #                 "config",
    #                 "tags.param.yaml",
    #             )
    #         },
    #     ],
    #     remappings=[
    #         ("~/image_rect", "/zed2/zed_node/rgb_raw/image_raw_color"),
    #         ("~/camera_info", "/zed2/zed_node/rgb_raw/camera_info"),
    #     ],
    # )

    zed2i_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("zed_wrapper"), "launch", "zed2i.launch.py"
            )
        ),
        launch_arguments={
            "publish_tf": "false",
            "zed_id": "1",
            "base_frame": "zed2i_base_frame",
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
            "0.5236425999941225",
            "--y",
            "0.446947179243176",
            "--z",
            "0.34343314197411295",
            "--qx",
            "-0.18097212982700334",
            "--qy",
            "0.8263314256176979",
            "--qz",
            "-0.34753493241548433",
            "--qw",
            "0.4045305106088611",
        ],
    )

    # zed2i_apriltag_node = Node(
    #     package="apriltag_ros2",
    #     executable="apriltag_ros2_continuous_detector_node",
    #     name="zed2i_apriltag_node",
    #     parameters=[
    #         {"size": 0.75},
    #         {"publish_tag_detections_image": True},
    #         {"camera_frame_id": "zed2i_left_camera_frame"},
    #         {"camera_base_frame_id": "zed2i_base_frame"},
    #         {
    #             os.path.join(
    #                 get_package_share_directory("apriltag_ros2"),
    #                 "config",
    #                 "settings.param.yaml",
    #             )
    #         },
    #         {
    #             os.path.join(
    #                 get_package_share_directory("apriltag_ros2"),
    #                 "config",
    #                 "tags.param.yaml",
    #             )
    #         },
    #     ],
    #     remappings=[
    #         ("~/image_rect", "/zed2i/zed_node/rgb_raw/image_raw_color"),
    #         ("~/camera_info", "/zed2i/zed_node/rgb_raw/camera_info"),
    #     ],
    # )

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
