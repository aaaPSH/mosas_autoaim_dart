import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node


def generate_launch_description():
    camera_params_file = LaunchConfiguration(
        "camera_params_file",
        default="/home/mosas_config/camera_params.yaml"
    )
    detector_params_file = LaunchConfiguration(
        "detector_params_file",
        default="/home/mosas_config/detector_params.yaml"
    )
    tracker_params_file = LaunchConfiguration(
        "tracker_params_file",
        default="/home/mosas_config/tracker_params.yaml"
    )
    serial_driver_params_file = LaunchConfiguration(
        "serial_driver_params_file", 
        default="/home/mosas_config/serial_driver_params.yaml"
    )

    serial_container = ComposableNodeContainer(
        name="serial",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="robot_tracker",
                plugin="RobotTrackerNode",
                name="robot_tracker_node",
                parameters=[tracker_params_file],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="serial_driver",
                plugin="SerialDriverNode",
                name="serial_driver_node",
                parameters=[serial_driver_params_file],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )
    container = ComposableNodeContainer(
        name="autoaim",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="hik_publisher",
                plugin="HikCameraNode",
                name="hik_camera",
                parameters=[camera_params_file],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="detector_dl",
                plugin="DetectorDlNode",
                name="detector_dl_node",
                parameters=[detector_params_file],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    rviz_config_file = "/home/nvidia/mosas_autoaim_dl/src/mosas_bringup/config/mosas_autoaim_dl_config.rviz"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_params_file",
                default_value=camera_params_file
            ),
            DeclareLaunchArgument(
                "detector_params_file",
                default_value=detector_params_file
            ),
            DeclareLaunchArgument(
                "tracker_params_file",
                default_value=tracker_params_file
            ),
            DeclareLaunchArgument(
                "serial_driver_params_file",
                default_value=serial_driver_params_file
            ),
            serial_container,
            container,
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_file]
            )
        ]
    )

