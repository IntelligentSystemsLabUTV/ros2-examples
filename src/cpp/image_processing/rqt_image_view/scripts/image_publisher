#!/usr/bin/env python3

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys

from copy import deepcopy

from ament_index_python.resources import get_resource

from cv2 import imread

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import Image


class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.i = 0
        qos_profile = QoSProfile(depth=1)

        self.pub = self.create_publisher(Image, 'images', qos_profile=qos_profile)
        timer_period = 0.10
        self.tmr = self.create_timer(timer_period, self.timer_callback)
        _, package_path = get_resource('packages', 'rqt_image_view')
        example_image_file = os.path.join(
            package_path, 'share', 'rqt_image_view', 'resource', 'lena.png')

        self.img = imread(example_image_file)
        self.msg = Image()
        self.msg.data = [int(b) for b in list(self.img.flatten())]
        self.msg.height = self.img.shape[0]
        self.msg.width = self.img.shape[1]
        self.msg.encoding = 'rgb8'
        self.msg.step = self.img.shape[1] * self.img.shape[2]

    def timer_callback(self):
        self.i += 1
        self.get_logger().info('Publishing Lena: "{0}"'.format(self.i))
        self.msg.data = self.msg.data[2:] + self.msg.data[:2]
        self.pub.publish(self.msg)


def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = ImagePublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
