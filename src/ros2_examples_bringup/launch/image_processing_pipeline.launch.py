"""
Image processing pipeline example launch file.

Roberto Masocco <robmasocco@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

June 5, 2023
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Builds a LaunchDescription for the image processing pipeline example."""
    ld = LaunchDescription()

    # Build config files path
    #! In the following, it's either YAML files OR parameter dictionaries
    #! That's a weird API inconsistency...
    camera_driver_config_file = os.path.join(
        get_package_share_directory('ros2_usb_camera'),
        'config',
        'config.yaml'
    )
    camera_config_file = 'file://' + os.path.join(
        get_package_share_directory('ros2_usb_camera'),
        'config',
        'camera.yaml'
    )
    detector_config_file = os.path.join(
        get_package_share_directory('aruco_detector'),
        'config',
        'bottom_detector.yaml'
    )

    container = ComposableNodeContainer(
        name='image_processing_pipeline_container',
        namespace='image_processing_pipeline',
        package='rclcpp_components',
        executable='component_container_mt',
        emulate_tty=True,
        output='both',
        log_cmd=True,
        composable_node_descriptions=[
            ComposableNode(
                package='ros2_usb_camera',
                plugin='USBCameraDriver::CameraDriverNode',
                name='usb_camera_driver',
                namespace='image_processing_pipeline',
                parameters=[
                    {
                        'camera_calibration_file': camera_config_file,
                        'fps': 20
                    }
                ],
                extra_arguments=[{'use_intra_process_comms': True}]),
            ComposableNode(
                package='aruco_detector',
                plugin='ArucoDetector::ArucoDetectorNode',
                name='aruco_detector',
                namespace='image_processing_pipeline',
                parameters=[
                    {
                        'camera_topic': '/image_processing_pipeline/usb_camera_driver/camera/image_color',
                        'target_ids': [15, 69, 666]
                    }
                ],
                extra_arguments=[{'use_intra_process_comms': True}])
        ]
    )

    ld.add_action(container)
    return ld
