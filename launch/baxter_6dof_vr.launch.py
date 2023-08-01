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

from launch.substitutions import LaunchConfiguration, PythonExpression


import os


def generate_launch_description():
    ld = LaunchDescription()

    zed2 = LaunchConfiguration("zed2")
    ld.add_action(
        DeclareLaunchArgument("zed2", default_value="False"),
    )
    zed2i = LaunchConfiguration("zed2i")
    ld.add_action(
        DeclareLaunchArgument("zed2i", default_value="True"),
    )
    d435 = LaunchConfiguration("d435")
    ld.add_action(
        DeclareLaunchArgument("d435", default_value="False"),
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
                "zed2_runtime.yaml",
            ),
        }.items(),
        condition=IfCondition(PythonExpression(zed2)),
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
        condition=IfCondition(PythonExpression(zed2)),
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
            "base_frame": "zed2i_base_frame",
            "publish_tf": "false",
            "ros_params_override_path": os.path.join(
                get_package_share_directory("baxter_6dof_vr"),
                "config",
                "zed2i_runtime.yaml",
            ),
        }.items(),
        condition=IfCondition(PythonExpression(zed2i)),
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
            "0.5020685134404471",
            "--y",
            "0.11238687042970352",
            "--z",
            "0.739467256649142",
            "--qx",
            "-0.042869371021978",
            "--qy",
            "0.8675346983542295",
            "--qz",
            "-0.06944353617087912",
            "--qw",
            "0.49063566876420617",
        ],
        condition=IfCondition(PythonExpression(zed2i)),
    )

    zed2i_rgbd_sync = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="zed2i_rgbd_sync_node",
        remappings=[
            ("rgb/image", "/zed2i/zed_node/rgb/image_rect_color"),
            ("depth/image", "/zed2i/zed_node/depth/depth_registered"),
            ("rgb/camera_info", "/zed2i/zed_node/rgb/camera_info"),
        ],
        condition=IfCondition(PythonExpression(zed2i)),
    )

    zed2i_pointcloud = Node(
        package="rtabmap_util",
        executable="point_cloud_xyzrgb",
        name="zed2i_pointcloud_node",
        parameters=[
            # {"voxel_size": 0.0},
            # {"decimation": 2},
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
        condition=IfCondition(PythonExpression(d435)),
    )

    table_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.57", "0", "0.73", "3.1415", "-1.57", "0", "ground", "table_0"],
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
            {"ROS_IP": "130.209.243.167"},
        ],
    )

    ld.add_action(zed2_camera_launch)
    ld.add_action(zed2_tf)

    ld.add_action(zed2i_camera_launch)
    ld.add_action(zed2i_tf)
    ld.add_action(zed2i_rgbd_sync)
    ld.add_action(zed2i_pointcloud)

    ld.add_action(d435_camera_launch)
    ld.add_action(d435_filter)

    ld.add_action(table_tf)
    ld.add_action(rviz2)
    ld.add_action(tcp_node)

    return ld
