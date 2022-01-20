"""
Termination example launch file.

Roberto Masocco <robmasocco@gmail.com>

January 20, 2022
"""

from launch import LaunchDescription
from launch_ros.actions import Node

"""Generates a launch description."""
def generate_launch_description():
  ld = LaunchDescription()
  node = Node(
    package='custom_context',
    executable='termination',
    emulate_tty=True,
    log_cmd=True
  )
  ld.add_action(node)
  return ld
