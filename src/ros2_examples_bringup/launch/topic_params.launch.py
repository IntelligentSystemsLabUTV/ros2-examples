"""
Example launch file for publisher, subscriber and parametric nodes.

Roberto Masocco <robmasocco@gmail.com>

January 7, 2022
"""

from launch import LaunchDescription
from launch_ros.actions import Node

# For parametric node (see its launch file for details)
import os
from ament_index_python.packages import get_package_share_directory

"""Generates a launch description for server and parametric nodes."""
def generate_launch_description():
  ld = LaunchDescription()

  # Configure topic publisher node
  pub = Node(
    package='topic_pubsub',
    executable='pub',
    name='pub',
    shell=True,
    emulate_tty=True,
    output='both',
    log_cmd=True
  )
  ld.add_action(pub)

  # Configure topic subscriber node
  sub = Node(
    package='topic_pubsub',
    executable='sub',
    name='sub',
    shell=True,
    emulate_tty=True,
    output='both',
    log_cmd=True
  )
  ld.add_action(sub)

  # Configure parametric node using YAML config file
  config = os.path.join(
    get_package_share_directory('parameters_example'),
    'config',
    'node_parameters.yaml'
  )
  param_node = Node(
    package='parameters_example',
    executable='parametric_pub',
    shell=True,
    emulate_tty=True,
    output='both',
    log_cmd=True,
    parameters=[config]
  )
  ld.add_action(param_node)

  return ld
