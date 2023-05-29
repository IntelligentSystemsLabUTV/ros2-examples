"""
Example launch file for publisher, subscriber and parametric nodes.

Roberto Masocco <robmasocco@gmail.com>

January 7, 2022
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

# For parametric node (see its launch file for details)
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generates a launch description for server and parametric nodes."""
    ld = LaunchDescription()

    # Configure topic publisher node
    pub = Node(
        package='custom_topic_cpp',
        executable='pub',
        name='pub',
        shell=True,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        remappings=[
            ('/pub/examples/test_topic', '/publisher_node/examples/test_topic')])
    ld.add_action(pub)

    # Configure topic subscriber node
    sub = Node(
        package='topic_pubsub_py',
        executable='sub',
        name='sub',
        shell=True,
        emulate_tty=True,
        output='both',
        log_cmd=True)
    ld.add_action(sub)

    # Include Parametric Publisher launch file
    #! Pay attention to the syntax and path conventions
    parametric_pub_launch_dir = os.path.join(
        get_package_share_directory('parameters_example_cpp'),
        'launch')
    ld.add_action(LogInfo(msg=[
        'Including launch file: ', parametric_pub_launch_dir, '/launch_param_example.launch.py']))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([parametric_pub_launch_dir, '/launch_param_example.launch.py'])))

    return ld
