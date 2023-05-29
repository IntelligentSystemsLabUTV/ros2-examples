"""
Example launch file for server and parametric nodes.

Roberto Masocco <robmasocco@gmail.com>

January 7, 2022
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# For parametric node (see its launch file for details)
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generates a launch description for server and parametric nodes."""
    ld = LaunchDescription()

    # Set arguments for service clients
    client_a = DeclareLaunchArgument(
        'client_a',
        description='a value for service client',
        default_value='0')
    ld.add_action(client_a)
    client_b = DeclareLaunchArgument(
        'client_b',
        description='b value for service client',
        default_value='0')
    ld.add_action(client_b)

    # Configure service server node
    server = Node(
        package='simple_service_cpp',
        executable='server',
        name='server',
        shell=True,
        emulate_tty=True,
        output='both',
        log_cmd=True)
    ld.add_action(server)

    # Configure service client node
    client = Node(
        package='simple_service_cpp',
        executable='client',
        name='client',
        shell=True,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        arguments=[
          LaunchConfiguration('client_a'),
          LaunchConfiguration('client_b')])
    ld.add_action(client)

    return ld
