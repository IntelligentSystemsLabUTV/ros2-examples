"""
Launch file to show some remapping features.

Roberto Masocco <robmasocco@gmail.com>

January 9, 2022
"""

from launch import LaunchDescription
from launch_ros.actions import Node

"""Generates a launch description for the given module"""


def generate_launch_description():
    ld = LaunchDescription()

    #! Here we'll remap everything: node, topics, namespaces

    # Create publisher/server launch definition
    pub_srv = Node(
        package='namespaces_examples',
        executable='topics_ns',
        name='test_node',  #! This overrides the node name
        namespace='topics_ns',  #! This overrides the default namespace
        shell=True,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        remappings=[  #! These'll override topic and service names
            ('rosservice://~/dummy_srv', '~/trigger'),
            ('rostopic://~/dummy_topic', '~/bool')
        ]  #! Notice the URL-like matching rule, necessary for topic/service names
           #! to make matching more explicit when such names are complex
    )
    ld.add_action(pub_srv)

    return ld
