"""
Example of a launch file for a node with parameters.

Roberto Masocco <robmasocco@gmail.com>

January 6, 2022
"""

#! These are necessary to build the launch description
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    #! Here the configuration is given as a Python dictionary
    config = {
        "number": 3
    }

    # Create node launch description
    node = Node(
        package='parameters_example',
        name='parametric_pub',
        executable='parametric_pub',
        parameters=[config]  # ! We could specify more than one source here
    )

    # Finalize launch description
    ld.add_action(node)
    return ld
