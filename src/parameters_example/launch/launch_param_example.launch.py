"""
Example of a launch file for a node with parameters.

Roberto Masocco <robmasocco@gmail.com>

January 6, 2022
"""

# Necessary to build config file path
#! We follow a standard convention that requires it to be placed in:
#! install/PACKAGE/share/PACKAGE/config/
import os
from ament_index_python.packages import get_package_share_directory

#! These are necessary to build the launch description
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  ld = LaunchDescription()

  # Build config file path
  config = os.path.join(
    get_package_share_directory('parameters_example'),
    'config',
    'node_parameters.yaml'
  )

  # Create node launch description
  node = Node(
    package='parameters_example',
    name='parametric_pub',
    executable='parametric_pub',
    parameters=[config] #! We could specify more than one source here
  )

  # Finalize launch description
  ld.add_action(node)
  return ld
