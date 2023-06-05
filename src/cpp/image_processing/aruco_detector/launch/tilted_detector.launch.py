"""
Tilted Aruco Detector launch file.

Roberto Masocco <robmasocco@gmail.com>
Lorenzo Bianchi <lnz.bnc@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

August 18, 2022
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Builds a LaunchDescription for the tilted Detector"""
    ld = LaunchDescription()

    # Build config file path
    config_file = os.path.join(
        get_package_share_directory('aruco_detector'),
        'config',
        'tilted_detector.yaml'
    )

    # Create node launch description
    node = Node(
        package='aruco_detector',
        executable='aruco_detector',
        exec_name='tilted_detector',
        name='tilted_detector',
        shell=True,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[
            config_file,
            {'camera_topic': '/tilted_camera_driver/camera/image_rect_color'}
        ]
    )

    ld.add_action(node)

    return ld
