"""
ROS 2 USB Camera Driver app launch file.

Roberto Masocco <robmasocco@gmail.com>
Lorenzo Bianchi <lnz.bnc@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

June 4, 2022
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Builds a LaunchDescription for the ROS 2 USB Camera Driver app"""
    ld = LaunchDescription()

    # Build config file path
    config_file = os.path.join(
        get_package_share_directory('ros2_usb_camera'),
        'config',
        'config.yaml'
    )

    # Create node launch description
    node = Node(
        package='ros2_usb_camera',
        executable='usb_camera_app',
        exec_name='usb_camera_app',
        shell=True,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[config_file]
    )

    ld.add_action(node)

    return ld
