"""
Example of a Python module with a ROS 2 topic subscriber.

Roberto Masocco <robmasocco@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

May 22, 2023
"""

import sys

#! Import Python client library for ROS 2
import rclpy
from rclpy.node import Node

#! QoS profile classes and enums
import rclpy.qos

#! Our interface definition, which is a class in Python
from ros2_examples_interfaces.msg import String

#! To define a node, we need to inherit from the Node class


class Sub(rclpy.node.Node):
    """Example ROS 2 node with a subscriber to a custom topic."""

    def __init__(self) -> None:
        """
        Creates a new Sub node.
        """
        #! We need to call the base Node constructor
        super().__init__('subscriber_node')

        #! Create a custom, best-effort QoS profile
        qos_profile = rclpy.qos.QoSProfile(
            depth=1,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE)

        #! Create a subscriber to the custom topic
        #! Note that this is Python: we are defining the subscriber attribute as we
        #! create it
        self._sub = self.create_subscription(
            String,
            '/publisher_node/examples/test_topic',
            self.sub_callback,
            qos_profile)

        self.get_logger().info('Node initialized')

    #! Callback functions are methods here too, and are passed message objects
    def sub_callback(self, msg: String) -> None:
        """
        Subscriber callback function.

        :param msg: String message to parse.
        """
        self.get_logger().info('Received message: %s' % msg.data)


def main():
    #! Initialize the ROS 2 client library and context
    rclpy.init(args=sys.argv)

    #! Initialize ROS 2 node
    node = Sub()

    #! Spin the node in a basic executor
    try:
        rclpy.spin(node)
    except:
        pass
        #! If you press Ctrl-C, the handler will call rclpy.shutdown() for us
    node.destroy_node()
    # rclpy.shutdown()

    print('Subscriber terminated')


#! This is required in Python to execute a script
if __name__ == '__main__':
    main()
