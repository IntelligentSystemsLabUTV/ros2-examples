"""
Custom context example launch file.

Roberto Masocco <robmasocco@gmail.com>

January 14, 2022
"""

from launch import LaunchDescription
from launch_ros.actions import Node

"""Generates a launch description"""
def generate_launch_description():
  ld = LaunchDescription()
  node = Node(
    package='custom_context',
    executable='custom_context',
    shell=True, #! The following options are necessary to see the output on cout
    emulate_tty=True,
    output='both',
    log_cmd=True
  )
  ld.add_action(node)
  return ld
