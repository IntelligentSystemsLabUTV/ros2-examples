/**
 * Node names, topic names, and default namespaces remapping example.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 *
 * January 9, 2022
 */

#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include "../../include/namespaces_examples/topic_ns/pub_srv.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::cout << "Initializing dummy node..." << std::endl;
  auto node = std::make_shared<DummyPubSrv>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}

//! Start this normally with ros2 run and query active topics and services,
//! then go have a look at the launch/topics_ns.launch.py launch file
//! You'll see many remapping rules, and their syntax for launch files
//! If you start it, have a look at the log: at the beginning there'll be
//! the very long command line necessary to achieve the same remappings
//! Such line can be given to ros2 run too, and will be parsed by rclcpp::init
//! as anything that comes after --ros-args
//! Remapping rules can be complex, for more info have a look at:
//! https://design.ros2.org/articles/static_remapping.html
//! http://design.ros2.org/articles/topic_and_service_names.html
