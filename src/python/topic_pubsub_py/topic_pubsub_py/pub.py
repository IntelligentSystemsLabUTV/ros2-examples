"""
Example of a Python module with a ROS 2 topic publisher.

Roberto Masocco <robmasocco@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

May 22, 2023
"""

import sys

#! Import Python client library for ROS 2
import rclpy
from rclpy.node import Node

#! QoS profile class
import rclpy.qos

#! Our interface definition, which is a class in Python
from ros2_examples_interfaces.msg import String

#! To define a node, we need to inherit from the Node class
class Pub(Node):
    """Example ROS 2 node with a subscriber to a custom topic."""

    def __init__(self) -> None:
      """
      Creates a new Sub node.
      """
      #! We need to call the base Node constructor
      super().__init__('publisher_node')

      #! Create a custom, best-effort QoS profile
      qos_profile = rclpy.qos.QoSProfile(
          depth=1,
          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
          reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
          durability=rclpy.qos.DurabilityPolicy.VOLATILE)

      #! Create a publisher to the custom topic
      #! Note that this is Python: we are defining the publiser attribute as we
      #! create it
      self._pub = self.create_publisher(
          String,
          '~/examples/test_topic',
          qos_profile)

      #! Create a timer to publish a message every second
      self._timer = self.create_timer(
          0.5,
          self.timer_callback)

      # This will be used to fill messages
      self._cnt = 0

      self.get_logger().info('Node initialized')

    #! Callback functions are methods here too, and are passed message objects
    def timer_callback(self) -> None:
        """
        Timer callback function.
        """
        #! Create a message object, fill it, and publish it
        msg = String()
        msg.data = 'Hello %d' % self._cnt
        self._pub.publish(msg)
        self._cnt += 1

        self.get_logger().info('Published message: %s' % msg.data)

def main():
    #! Initialize the ROS 2 client library and context
    rclpy.init(args=sys.argv)

    #! Initialize ROS 2 node
    node = Pub()

    #! Spin the node in a basic executor
    try:
        rclpy.spin(node)
    except:
        rclpy.shutdown()
        print("ops")

#! This is required in Python to execute a script
if __name__ == '__main__':
    main()
